"""
LORA_RTK_uf_rx_air.py
----------------------
Air-unit variant of the unfiltered RX script.

Hardcoded for the air unit:
    LoRa address : 200
    LoRa COM port: COM6

Forwarding logic:
    packet arrives at RX -> forward to FC immediately

Wire format (must match RTK_Lora_TX.py):
    AT+SEND=0,<len>,<TYPE>:<hex>            (single-part message)
    AT+SEND=0,<len>,<TYPE>_1:<hex>          (part 1 of a split message)
    AT+SEND=0,<len>,<TYPE>_2:<hex>          (part 2 of a split message)

Stops on Ctrl+C.
"""

import time
import serial
from pymavlink import mavutil

# ==================== CONFIG ====================
LORA_BAUD = 115200
LORA_BAND = 865000000
LORA_NETWORK_ID = 5
LORA_PARAMETER = (7, 9, 1, 12)

# Hardcoded air-unit values
LORA_ADDRESS = 200
LORA_PORT = "COM6"

MAVLINK_RTCM_MAX_FRAG_LEN = 180
MAVLINK_RTCM_MAX_FRAGMENTS = 4


# ==================== LoRa (RYLR998) driver ====================
class RYLR998:
    def __init__(self, port, baud=LORA_BAUD, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def _send_at(self, cmd, wait=0.2):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode())
        time.sleep(wait)
        resp = self.ser.read(self.ser.in_waiting or 1)
        return resp.decode(errors="ignore")

    def configure(self, address, network_id, band, parameter=LORA_PARAMETER):
        sf, bw, cr, preamble = parameter
        results = {
            "ADDRESS": self._send_at(f"AT+ADDRESS={address}"),
            "NETWORKID": self._send_at(f"AT+NETWORKID={network_id}"),
            "BAND": self._send_at(f"AT+BAND={band}"),
            "PARAMETER": self._send_at(f"AT+PARAMETER={sf},{bw},{cr},{preamble}"),
        }
        for name, resp in results.items():
            if "OK" not in resp:
                print(f"[lora] Warning: AT+{name} did not return OK -> {resp!r}")
        print(f"[lora] Configured: address={address}, network_id={network_id}, "
              f"band={band}, parameter={parameter}")

    def receive(self, timeout=1.0):
        deadline = time.time() + timeout
        prefix = b"+RCV="
        buf = b""

        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            if len(buf) > len(prefix):
                buf = buf[-len(prefix):]
            if buf == prefix:
                deadline = time.time() + 0.5
                break
        else:
            return None
        if buf != prefix:
            return None

        addr_str = self._read_until_comma(deadline)
        length_str = self._read_until_comma(deadline)
        if addr_str is None or length_str is None:
            return None
        try:
            src = int(addr_str)
            length = int(length_str)
        except ValueError:
            return None

        data = self._read_exact(length, deadline)
        if data is None:
            return None

        self._read_exact(1, deadline)
        rssi_str = self._read_until_comma(deadline)
        snr_str = self._read_until(b"\r\n", deadline)

        try:
            rssi = int(rssi_str) if rssi_str is not None else None
        except ValueError:
            rssi = None
        try:
            snr = int(snr_str) if snr_str is not None else None
        except ValueError:
            snr = None

        return src, data, rssi, snr

    def _read_exact(self, n, deadline):
        out = b""
        while len(out) < n and time.time() < deadline:
            chunk = self.ser.read(n - len(out))
            out += chunk
        return out if len(out) == n else None

    def _read_until_comma(self, deadline):
        out = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b",":
                return out.decode(errors="ignore")
            out += b
        return None

    def _read_until(self, marker, deadline):
        out = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            out += b
            if out.endswith(marker):
                return out[: -len(marker)].decode(errors="ignore")
        return out.decode(errors="ignore") if out else None

    def close(self):
        self.ser.close()


# ==================== TX wire-format decoder ====================
class RtcmReassembler:
    def __init__(self):
        self._pending = {}

    def feed(self, ascii_payload: str):
        if ":" not in ascii_payload:
            print(f"  [rx] WARNING: malformed payload (no ':'): {ascii_payload!r}")
            return None, None

        header, hexdata = ascii_payload.split(":", 1)

        if "_" in header:
            type_str, part_str = header.split("_", 1)
            try:
                msg_type = int(type_str)
                part_num = int(part_str)
            except ValueError:
                print(f"  [rx] WARNING: bad split header: {header!r}")
                return None, None

            if part_num == 1:
                self._pending[msg_type] = hexdata
                return None, None
            elif part_num == 2:
                first_hex = self._pending.pop(msg_type, None)
                if first_hex is None:
                    print(f"  [rx] WARNING: got part 2 of {msg_type} with no part 1 - dropping")
                    return None, None
                full_hex = first_hex + hexdata
                try:
                    raw = bytes.fromhex(full_hex)
                except ValueError:
                    print(f"  [rx] WARNING: reassembled {msg_type} has invalid hex - dropping")
                    return None, None
                return msg_type, raw
            else:
                print(f"  [rx] WARNING: unexpected part number in {header!r}")
                return None, None

        try:
            msg_type = int(header)
            raw = bytes.fromhex(hexdata)
        except ValueError:
            print(f"  [rx] WARNING: bad single-part payload: {ascii_payload!r}")
            return None, None
        return msg_type, raw


# ==================== MAVLink / pymavlink RTCM forwarding ====================
def send_rtcm_to_fc(mav, frame: bytes, seq_id: int):
    max_frag = MAVLINK_RTCM_MAX_FRAG_LEN

    if len(frame) > max_frag * MAVLINK_RTCM_MAX_FRAGMENTS:
        return None

    seq_bits = (seq_id & 0x1F) << 3
    sent = []

    if len(frame) <= max_frag:
        chunk = frame + b"\x00" * (max_frag - len(frame))
        mav.mav.gps_rtcm_data_send(seq_bits, len(frame), chunk)
        sent.append((seq_bits, len(frame)))
        return sent

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

    if len(frame) % max_frag == 0 and fragment_id < MAVLINK_RTCM_MAX_FRAGMENTS:
        flags = 1 | ((fragment_id & 0x03) << 1) | seq_bits
        mav.mav.gps_rtcm_data_send(flags, 0, b"\x00" * max_frag)
        sent.append((flags, 0))

    return sent


# ==================== Main ====================
def main():
    print(f"[air] LoRa address: {LORA_ADDRESS}, COM port: {LORA_PORT}")

    fc_conn_str = input("Pixhawk connection string (e.g. COM5 or udp:127.0.0.1:14550): ").strip()
    fc_baud = input("Pixhawk baud rate (blank = default 115200, ignored for udp/tcp): ").strip()

    lora = RYLR998(LORA_PORT, LORA_BAUD)
    lora.configure(address=LORA_ADDRESS, network_id=LORA_NETWORK_ID, band=LORA_BAND)

    print(f"[mavlink] Connecting to {fc_conn_str} ...")
    if fc_baud:
        mav = mavutil.mavlink_connection(fc_conn_str, baud=int(fc_baud))
    else:
        mav = mavutil.mavlink_connection(fc_conn_str, baud=115200)
    mav.wait_heartbeat(timeout=10)
    print(f"[mavlink] Heartbeat received from system {mav.target_system}, "
          f"component {mav.target_component}.")

    reassembler = RtcmReassembler()
    seq_id = 0

    print("Listening for RTCM3 packets over LoRa. Press Ctrl+C to stop.")
    print("NOTE: GPS_RTCM_DATA has no protocol-level ACK from the FC - the")
    print("      confirmation below just means pymavlink handed the packet")
    print("      to the link successfully, not that the FC's GPS accepted it.\n")

    try:
        while True:
            packet = lora.receive(timeout=1.0)
            if packet is None:
                continue
            src, data, rssi, snr = packet

            try:
                ascii_payload = data.decode("ascii")
            except UnicodeDecodeError:
                print(f"From {src} | non-ASCII payload ({len(data)} bytes) - skipping | "
                      f"RSSI={rssi} SNR={snr}")
                continue

            print(f"From {src} | {ascii_payload!r} | RSSI={rssi} SNR={snr}")

            msg_type, raw = reassembler.feed(ascii_payload)
            if msg_type is None:
                continue

            result = send_rtcm_to_fc(mav, raw, seq_id)
            seq_id = (seq_id + 1) % 32

            if result is None:
                print(f"  [{msg_type}] {len(raw)} bytes - DROPPED (exceeds max fragmentable size)")
            elif len(result) == 1:
                flags, length = result[0]
                print(f"  [{msg_type}] {len(raw)} bytes -> sent to FC (1 packet, {length}B) - ack")
            else:
                parts = ", ".join(f"frag{((f>>1)&0x3)}:{l}B" for f, l in result)
                print(f"  [{msg_type}] {len(raw)} bytes -> sent to FC "
                      f"({len(result)} fragments: {parts}) - ack")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        lora.close()


if __name__ == "__main__":
    main()
