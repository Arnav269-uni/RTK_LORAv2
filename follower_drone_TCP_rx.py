import socket
import time
from pymavlink import mavutil

# ==================== CONFIG ====================
LEADER_TCP_IP = "192.168.1.100"   # Replace with Leader drone's IP address
LEADER_TCP_PORT = 6000

# Flight Controller Connection
FC_CONN_STR = "udp:127.0.0.1:14550" # Adjust to your follower drone FC port
FC_BAUD = 115200

MAVLINK_RTCM_MAX_FRAG_LEN = 180
MAVLINK_RTCM_MAX_FRAGMENTS = 4

# ==================== CRC-24Q Calculation ====================
def crc24q(payload: bytes) -> int:
    """
    Calculates the Qualcomm CRC-24Q used in RTCM3 packets.
    Polynomial: 0x1864CFB
    """
    crc = 0
    for byte in payload:
        crc ^= (byte << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


# ==================== MAVLink RTCM Forwarding ====================
def send_rtcm_to_fc(mav, frame: bytes, seq_id: int):
    """
    Forwards one reassembled RTCM3 frame to the flight controller as GPS_RTCM_DATA.
    Uses the exact same fragmentation logic as the leader drone.
    """
    max_frag = MAVLINK_RTCM_MAX_FRAG_LEN

    if len(frame) > max_frag * MAVLINK_RTCM_MAX_FRAGMENTS:
        return None

    seq_bits = (seq_id & 0x1F) << 3
    sent = []

    # Small frame (no fragmentation needed)
    if len(frame) <= max_frag:
        chunk = frame + b"\x00" * (max_frag - len(frame))
        mav.mav.gps_rtcm_data_send(seq_bits, len(frame), chunk)
        sent.append((seq_bits, len(frame)))
        return sent

    # Large frame (fragmentation required)
    offset = 0
    fragment_id = 0
    while offset < len(frame) and fragment_id < MAVLINK_RTCM_MAX_FRAGMENTS:
        piece = frame[offset: offset + max_frag]
        flags = 1 | ((fragment_id & 0x03) << 1) | seq_bits
        chunk = piece + b"\x00" * (max_frag - len(piece))
        mav.mav.gps_rtcm_data_send(flags, len(piece), chunk)
        sent.append((flags, len(piece)))
        offset += max_frag
        fragment_id += 1

    # End fragmentation marker if perfectly divisible
    if len(frame) % max_frag == 0 and fragment_id < MAVLINK_RTCM_MAX_FRAGMENTS:
        flags = 1 | ((fragment_id & 0x03) << 1) | seq_bits
        mav.mav.gps_rtcm_data_send(flags, 0, b"\x00" * max_frag)
        sent.append((flags, 0))

    return sent


# ==================== Main Follower Loop ====================
def main():
    print(f"[mavlink] Connecting to Follower FC at {FC_CONN_STR} ...")
    if FC_BAUD:
        mav = mavutil.mavlink_connection(FC_CONN_STR, baud=int(FC_BAUD))
    else:
        mav = mavutil.mavlink_connection(FC_CONN_STR, baud=115200)
    mav.wait_heartbeat(timeout=10)
    print(f"[mavlink] Heartbeat received from system {mav.target_system}, component {mav.target_component}.")

    seq_id = 0

    while True:
        print(f"[tcp] Connecting to Leader drone at {LEADER_TCP_IP}:{LEADER_TCP_PORT} ...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            sock.connect((LEADER_TCP_IP, LEADER_TCP_PORT))
            print("[tcp] Connected! Listening for RTCM3 stream...")
            
            # Buffer to accumulate incoming TCP stream bytes
            stream_buffer = bytearray()
            
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    print("[tcp] Connection closed by leader.")
                    break
                
                stream_buffer.extend(chunk)
                
                # --- RTCM3 Packet Extraction Logic ---
                while len(stream_buffer) >= 3:
                    # 1. Search for the Preamble (0xD3)
                    if stream_buffer[0] != 0xD3:
                        # Drop the garbage byte and keep searching
                        stream_buffer.pop(0)
                        continue
                    
                    # 2. Extract Length (mask out 6 reserved bits)
                    payload_len = ((stream_buffer[1] & 0x03) << 8) | stream_buffer[2]
                    packet_total_len = 3 + payload_len + 3 # Header + Payload + CRC
                    
                    # Do we have the full packet in our buffer yet?
                    if len(stream_buffer) < packet_total_len:
                        break # Wait for more TCP chunks to arrive
                    
                    # 3. Extract the full packet and the reported CRC
                    full_packet = bytes(stream_buffer[:packet_total_len])
                    header_and_payload = full_packet[:-3]
                    reported_crc = (full_packet[-3] << 16) | (full_packet[-2] << 8) | full_packet[-1]
                    
                    # 4. Validate CRC-24Q
                    calculated_crc = crc24q(header_and_payload)
                    
                    if calculated_crc == reported_crc:
                        # We have a valid RTCM3 packet!
                        msg_type = (full_packet[3] << 4) | (full_packet[4] >> 4) # Extract msg type for logging
                        
                        result = send_rtcm_to_fc(mav, full_packet, seq_id)
                        seq_id = (seq_id + 1) % 32
                        
                        if result is None:
                            print(f"  [{msg_type}] {len(full_packet)} bytes - DROPPED (too large)")
                        else:
                            print(f"  [{msg_type}] {len(full_packet)} bytes -> sent to FC ({len(result)} fragments)")
                        
                        # Remove the processed packet from our stream buffer
                        del stream_buffer[:packet_total_len]
                        
                    else:
                        # CRC mismatch! The 0xD3 we found was likely random noise inside a payload.
                        # Drop the fake preamble byte so we can search for the real one.
                        print("[warning] CRC mismatch found in stream, realigning...")
                        stream_buffer.pop(0)
                        
        except ConnectionRefusedError:
            print("[tcp] Connection refused. Retrying in 3 seconds...")
            time.sleep(3)
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            print(f"[tcp] Connection dropped ({e}). Reconnecting in 3 seconds...")
            time.sleep(3)
        finally:
            sock.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nFollower script stopped by user.")
