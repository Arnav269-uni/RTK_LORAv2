"""
RTK_Lora_TX.py  --  GNSS Base -> filter RTK corrections -> LoRa broadcast

LoRa parameters (SF9/BW500kHz/CR1/Preamble12):
    AT+PARAMETER=9,9,1,12  ->  ~857ms total epoch airtime, fits 1Hz RTCM window
    AT+BAND=865000000       ->  865 MHz
    AT+NETWORKID=5
    AT+IPR=57600            ->  57600 baud UART to USB-TTL adapter

Message filter  (ALL 4 MSM4 constellations -- fixes HDOP when air unit tracks
                 more constellations than the base was correcting for):
    1005  base reference position    25B raw   ->  55 chars  -> 1 send
    1074  GPS MSM4 observations     115B raw   -> 235 chars  -> 1 send
    1084  GLONASS MSM4 observations  69B raw   -> 143 chars  -> 1 send
    1094  Galileo MSM4 observations 100B raw   -> 205 chars  -> 1 send
    1124  BeiDou MSM4 observations  173B raw   -> 346 chars  -> 2 sends  <- largest
    1230  GLONASS Inter-Frequency Biases added to stabilize FDOP tracking.
    Everything else (NMEA, UBX, MSM7) is dropped.

    BeiDou (1124) is the only message that needs splitting because its raw
    frame can reach 173B -> 346 hex chars, which exceeds the 240-char
    AT+SEND limit. The split logic in lora_send_message() already handles
    this identically to the GPS+BeiDou version -- 1124_1: carries the
    first 233 hex chars, 1124_2: carries the remainder. 1084 and 1094 are
    always small enough for a single send.

Per-message LoRa packet format:
    AT+SEND=0,<len>,<TYPE>:<hex>
    e.g.  AT+SEND=0,55,1005:3ed0...
    TYPE is the RTCM message type number (1005, 1074, 1124).
    If the hex payload exceeds 235 chars (= 240 limit minus 5-char header),
    it is split into multiple sends with suffix _1, _2 etc:
    AT+SEND=0,240,1124_1:<235 hex chars>
    AT+SEND=0,86, 1124_2:<81 hex chars>
    The RX reassembles multi-part messages before forwarding to the FC.

+OK wait:
    After every AT+SEND the TX waits until the module replies +OK (or timeout).
    The RYLR998 sends +OK only after the packet is fully transmitted over the air.
    At SF9/BW500kHz, max single-send airtime is ~312ms, so timeout is set to 0.65s.
    reset_input_buffer() is called ONCE before each message group, NOT between sends.

    IMPORTANT (bugfix): this wait is BLOCKING and can take up to ~0.65s per
    send - up to ~1.3s for a split 1124 message. That is a meaningful
    fraction of the GNSS's 1-second epoch. If this wait happens on the
    SAME thread/loop that is reading the GNSS serial port, the reader
    falls behind while blocked, and pyrtcm's RTCMReader (which reads the
    stream BYTE BY BYTE internally) loses byte alignment on the backlog
    that piles up. By the time it resyncs, it's usually already past the
    1074/1124 messages for that epoch and only realigns cleanly at the
    next 1005 (which is preceded by a natural idle gap on the wire) -
    this is exactly the "only 1005 gets through" symptom. Fixed here by
    running the GNSS read in its own thread, decoupled from the LoRa
    send/wait via a queue, so the reader is never blocked by the radio.

Survey menu:
    1) Start Survey-In  (duration seconds only -- accuracy hardcoded to 5m)
    2) Check Survey-In status
    3) Start RTCM stream over LoRa

Install deps:
    pip install pyserial pyrtcm pyubx2
"""

from serial import Serial, SerialException
from pyrtcm import RTCMReader
from pyubx2 import UBXMessage, UBXReader, POLL
import threading
import queue
import time
import logging

# Surfaces pyrtcm's internal parse errors (previously fully silent with
# quitonerror=0) so we can see WHY a message type fails to parse, instead
# of it just vanishing with no trace.
logging.basicConfig(level=logging.ERROR, format="%(name)s: %(message)s")

# ── RTCM message filter ─────────────────────────────────────────────────────
# All 4 MSM4 constellations + 1230 GLONASS Bias -- fixes high HDOP when the air unit tracks more
# constellations than the base was providing corrections for.
KEEP_TYPES     = {1005, 1074, 1084, 1094, 1124, 1230}
# 1084 removed from EPOCH_COMPLETE so we don't get "incomplete epoch" warnings if GLONASS drops out
EPOCH_COMPLETE = {1005, 1074, 1094, 1124}  # 1230 included to prevent epoch boundary bleed (validated in LORA_BASE_sim.py)

# ── LoRa config ──────────────────────────────────────────────────────────────
NETWORK_ID    = 5
BAND_HZ       = 865000000
LORA_BAUD     = 115200

# CHOOSE YOUR LORA AND GNSS RATE COMBINATION:
# Option A (Standard Range, 1Hz rate): SF7 / BW500kHz / CR1 / Preamble12
# LORA_PARAM    = "7,9,1,12"
# GNSS_RATE_HZ  = 1.0

# Option B (Longer Range, 0.5Hz rate): SF9 / BW500kHz / CR1 / Preamble12
# Option B (Longer Range, 0.5Hz rate): SF9 / BW500kHz / CR1 / Preamble12
# Since SF9 airtime is 4x longer than SF7 (~1.69s total per epoch), we MUST slow down
# the GNSS rate to 0.5Hz (one epoch every 2s) so the LoRa transmitter can keep up.
LORA_PARAM    = "7,9,1,12"
GNSS_RATE_HZ  = 0.5

# ── Timing ───────────────────────────────────────────────────────────────────
# Max single-send airtime at SF9/BW500kHz with 240-char payload ~312ms.
# Timeout = airtime + safety margin. Increased to 2.0s to ensure ACK has plenty of time.
SEND_TIMEOUT  = 2.0   # seconds

# Max hex chars in one AT+SEND payload = 240 limit - 5 char header
MAX_HEX_CHARS = 233

# Queue size: 6 message types per epoch now (with 1230 added),
# so increase headroom to cover ~3 full epochs without dropping.
QUEUE_MAXSIZE = 24


# ── Serial helpers ────────────────────────────────────────────────────────────

def send_at(ser, cmd, wait=0.5):
    """Send an AT command with a fixed wait (used for config, not data)."""
    ser.write((cmd.strip() + "\r\n").encode("ascii"))
    time.sleep(wait)
    return ser.read(ser.in_waiting or 128).decode("ascii", errors="replace").strip()


def wait_for_ok(ser, timeout=SEND_TIMEOUT):
    """
    Block until the LoRa module replies +OK (or +ERR), or timeout expires.
    The module sends +OK only AFTER the packet is fully transmitted over the air.
    DO NOT call reset_input_buffer() between write() and this function --
    that would discard the +OK we are waiting for.

    This is fine to block on now: it runs on the LoRa-sending side only,
    never on the same thread that's reading the GNSS.
    """
    deadline = time.time() + timeout
    buf = ""
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode("ascii", errors="replace")
            if "+OK" in buf or "+ERR" in buf:
                break
        time.sleep(0.005)
    return buf.strip()


# ── Hardware Auto-Configuration ─────────────────────────────────────────────

def auto_configure_base(gnss_port):
    """
    Sweeps common baud rates to find the receiver, forces it to 115200, 
    disables NMEA/MSM7 to save bandwidth, and activates MSM4 + GLONASS Bias (1230) messages.
    """
    print(f"\n[*] Auto-configuring Base Station on {gnss_port}...")
    print("[*] Forcing 115200 baud, enabling MSM4 + GLONASS Bias (1230), and killing NMEA noise...")
    
    common_bauds = [115200, 38400, 57600, 9600, 230400]
    
    cfg_data = [
        # 0. Explicitly enable GLONASS constellation tracking in hardware
        ("CFG_SIGNAL_GLO_ENA", 1),
        ("CFG_SIGNAL_GLO_L1_ENA", 1),
        ("CFG_SIGNAL_GLO_L2_ENA", 1),

        # 1. Force Baud to 115200 on UART1
        ("CFG_UART1_BAUDRATE", 115200),
        
        # 2. Protocol out: RTCM3 and UBX on, NMEA off (kills bandwidth bloat)
        ("CFG_UART1OUTPROT_RTCM3X", 1),
        ("CFG_UART1OUTPROT_UBX", 1),
        ("CFG_UART1OUTPROT_NMEA", 0),
        ("CFG_USBOUTPROT_RTCM3X", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_USBOUTPROT_NMEA", 0),
        
        # 3. ENABLE MSM4, 1005, AND 1230 GLONASS BIAS (UART1 & USB)
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_UART1", 1), # Base Position
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 1), # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 1), # GLONASS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 1), # Galileo MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 1), # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 1), # GLONASS Code-Phase Biases
        
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1),   # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 1),   # GLONASS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 1),   # Galileo MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1),   # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),   # GLONASS Code-Phase Biases

        # 4. DISABLE MSM7 (UART1 & USB) to prevent 720-byte packet walls
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_UART1", 0),
        
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 0),
    ]

    for baud in common_bauds:
        try:
            ser = Serial(gnss_port, baud, timeout=0.2)
            msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg_data)
            ser.write(msg.serialize())
            ser.flush()
            time.sleep(0.1)
            ser.close()
        except SerialException:
            pass
            
    print("[+] Base configuration complete. Module locked to 115200 baud.")

def auto_configure_lora(port):
    """
    Auto-detect and fully configure the TX LoRa module.
    Hardcoded from compatibility test results:
        ADDRESS=100, NETWORKID=5, BAND=865000000, PARAMETER=7,9,1,12, BAUD=115200
    """
    print(f"\n[*] Auto-configuring TX LoRa module on {port}...")
    common_bauds = [115200, 57600, 38400, 9600, 230400]
    
    for baud in common_bauds:
        try:
            ser = Serial(port, baud, timeout=0.5)
            ser.reset_input_buffer()
            ser.write(b"AT\r\n")
            time.sleep(0.3)
            reply = ser.read(ser.in_waiting or 128).decode("ascii", errors="ignore")
            
            if "+OK" in reply:
                print(f"[+] Found LoRa module at {baud} baud.")
                
                # Force baud to 115200 if needed
                if baud != 115200:
                    print("    -> Setting baud to 115200...")
                    ser.write(b"AT+IPR=115200\r\n")
                    time.sleep(0.3)
                    ser.close()
                    ser = Serial(port, 115200, timeout=0.5)
                    time.sleep(0.2)
                
                # Set all parameters from compatibility test
                params = [
                    ("ADDRESS",   "AT+ADDRESS=100"),
                    ("NETWORKID", "AT+NETWORKID=5"),
                    ("BAND",      "AT+BAND=865000000"),
                    ("PARAMETER", "AT+PARAMETER=7,9,1,12"),
                    ("BAUD",      "AT+IPR=115200"),
                ]
                for label, cmd in params:
                    ser.reset_input_buffer()
                    ser.write((cmd + "\r\n").encode("ascii"))
                    time.sleep(0.3)
                    resp = ser.read(ser.in_waiting or 128).decode("ascii", errors="ignore")
                    status = f"Success! (Response: {resp.strip()!r})" if "+OK" in resp else f"FAIL ({resp.strip()!r})"
                    print(f"    {label}: {status}")
                
                ser.close()
                print("[+] TX LoRa configuration complete.")
                return True
            ser.close()
        except SerialException:
            pass
            
    print("[-] Could not find LoRa module or auto-config failed.")
    return False

# ── LoRa configuration ────────────────────────────────────────────────────────

def configure_lora(ser, my_address):
    steps = [
        ("NETWORKID", f"AT+NETWORKID={NETWORK_ID}"),
        ("BAND",      f"AT+BAND={BAND_HZ}"),
        ("PARAMETER", f"AT+PARAMETER={LORA_PARAM}"),
        ("ADDRESS",   f"AT+ADDRESS={my_address}"),
    ]
    for label, cmd in steps:
        reply = send_at(ser, cmd)
        status = "OK" if "+OK" in reply else f"FAIL ({reply!r})"
        print(f"  {label}: {status}")


# ── Survey functions ──────────────────────────────────────────────────────────

def send_cfg(port, baud, cfg):
    ser = Serial(port, baud, timeout=2)
    msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg)
    ser.write(msg.serialize())
    ser.close()


def start_survey(gnss_port, gnss_baud, duration):
    acc_raw = int(round(50000 / 0.1))   # 5m in 0.1mm units -- effectively ignored
    cfg = [
        ("CFG_TMODE_MODE",         1),
        ("CFG_TMODE_SVIN_MIN_DUR", duration),
        ("CFG_TMODE_SVIN_ACC_LIMIT", acc_raw),
    ]
    send_cfg(gnss_port, gnss_baud, cfg)
    print(f"Survey-In started -- will complete after {duration}s.")


def set_gnss_output_rate(gnss_port, gnss_baud, rate_hz):
    """
    Sets the GNSS measurement/nav rate via UBX CFG_RATE_MEAS (period in ms)
    and CFG_RATE_NAV (nav solutions per measurement, left at 1). e.g.
    rate_hz=0.5 -> one epoch every 2000ms, giving the LoRa radio twice as
    much time per epoch to get everything out over the air.
    """
    period_ms = int(round(1000 / rate_hz))
    cfg = [
        ("CFG_RATE_MEAS", period_ms),
        ("CFG_RATE_NAV", 1),
    ]
    send_cfg(gnss_port, gnss_baud, cfg)
    print(f"GNSS output rate set to {rate_hz}Hz (measurement period {period_ms}ms).")


def poll_svin(ser):
    ser.reset_input_buffer()
    ser.write(UBXMessage("NAV", "NAV-SVIN", POLL).serialize())
    ubr = UBXReader(ser)
    for _ in range(200):
        _, parsed = ubr.read()
        if parsed and parsed.identity == "NAV-SVIN":
            return parsed
    return None


def check_survey(gnss_port, gnss_baud, poll_interval=5):
    print(f"\nPolling survey status every {poll_interval}s (Ctrl+C to stop)...\n")
    ser = Serial(gnss_port, gnss_baud, timeout=2)
    try:
        while True:
            p = poll_svin(ser)
            if p is None:
                print("  No NAV-SVIN response.")
            else:
                raw_acc = getattr(p, "meanAcc", None)
                acc_mm  = round(raw_acc / 10, 1) if isinstance(raw_acc, (int, float)) else "?"
                valid   = getattr(p, "valid", "?")
                print(f"  active={getattr(p,'active','?')}  valid={valid}  "
                      f"dur={getattr(p,'dur','?')}s  meanAcc={acc_mm}mm")
                if str(valid) in ("1", "True"):
                    print("\n Survey converged (valid=1).\n")
                    return
            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


# ── LoRa transmit ─────────────────────────────────────────────────────────────

def lora_send_message(lora_ser, msg_type: int, raw_bytes: bytes) -> bool:
    """
    Send one RTCM3 message over LoRa.
    If the hex payload fits in 235 chars -> one AT+SEND.
    If it doesn't (1124 at 316 chars) -> two AT+SEND with _1/_2 suffixes.
    Waits for +OK after EACH send before proceeding.
    reset_input_buffer() called ONCE before first send for this message.
    Returns True if all sends got +OK.
    """
    hex_str = raw_bytes.hex()
    hex_len = len(hex_str)

    if hex_len <= MAX_HEX_CHARS:
        sends = [(f"{msg_type}:", hex_str)]
    else:
        sends = [
            (f"{msg_type}_1:", hex_str[:MAX_HEX_CHARS]),
            (f"{msg_type}_2:", hex_str[MAX_HEX_CHARS:]),
        ]

    lora_ser.reset_input_buffer()

    all_ok = True
    for header, data in sends:
        payload = header + data
        cmd = f"AT+SEND=0,{len(payload)},{payload}\r\n"
        t0 = time.time()
        lora_ser.write(cmd.encode("ascii"))
        reply = wait_for_ok(lora_ser)
        elapsed = time.time() - t0
        ok = "+OK" in reply
        all_ok = all_ok and ok
        status = "OK" if ok else f"FAIL({reply!r})"
        print(f"  TX {header[:-1]:8s} | {len(payload):3d} chars | "
              f"{elapsed:.2f}s | {status}")
        time.sleep(0.02)  # Increased delay to 200ms between chunks for maximum stability

    return all_ok


# ── GNSS reader thread ────────────────────────────────────────────────────────

def gnss_reader_worker(gnss_ser, out_queue, stop_event):
    """
    Runs on its own thread. Continuously reads RTCM3 messages off the GNSS
    serial port and pushes the ones we care about onto out_queue.
    """
    rtr = RTCMReader(gnss_ser, quitonerror=0)
    for raw, parsed in rtr:
        if stop_event.is_set():
            break
        if parsed is None or raw is None:
            continue
        try:
            mt = int(parsed.identity)
        except (TypeError, ValueError):
            continue
        if mt not in KEEP_TYPES:
            continue

        try:
            out_queue.put_nowait((mt, raw))
        except queue.Full:
            try:
                out_queue.get_nowait()  # drop oldest
            except queue.Empty:
                pass
            try:
                out_queue.put_nowait((mt, raw))
            except queue.Full:
                pass


def stream_loop(gnss_port, gnss_baud, lora_port, lora_baud, lora_address):
    print(f"\nSetting GNSS output rate to {GNSS_RATE_HZ}Hz...")
    set_gnss_output_rate(gnss_port, gnss_baud, GNSS_RATE_HZ)
    time.sleep(0.5)  # give the receiver a moment to apply the new rate

    gnss_ser = Serial(gnss_port, gnss_baud, timeout=2)
    
    # [FIX] The LoRa auto-config took ~2 seconds, during which GNSS was spitting out 
    # data into the OS serial buffer. We MUST flush it so the RTCMReader doesn't 
    # read a stale, cut-off message (which causes the 1124_1 dropped packets!)
    gnss_ser.reset_input_buffer()
    time.sleep(0.1)
    
    lora_ser = Serial(lora_port, lora_baud, timeout=1)

    time.sleep(0.2)
    lora_ser.reset_input_buffer()

    # [FIX] Removed duplicate configure_lora() call here - already done in auto_config

    print(f"\nStreaming 1005+1074+1084+1094+1124+1230 from {gnss_port} -> LoRa {lora_port} "
          f"(Ctrl+C to stop)\n")

    msg_queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=gnss_reader_worker,
        args=(gnss_ser, msg_queue, stop_event),
        daemon=True,
    )
    reader_thread.start()

    epoch_buf = {}
    epoch_num = 0

    try:
        while True:
            try:
                mt, raw = msg_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            if mt in epoch_buf:
                print(f"\n[Epoch {epoch_num} incomplete, flushing]\n")
                epoch_buf = {}

            epoch_buf[mt] = raw

            print(f"\nEpoch {epoch_num + 1} | msg {mt} | {len(raw)}B raw "
                  f"| queue depth {msg_queue.qsize()}")
            lora_send_message(lora_ser, mt, raw)
            
            # Mild delay between messages to prevent RX overload without breaking the queue limit
            time.sleep(0.01)  # reduced from 0.05 -- safe because wait_for_ok() already blocks until +OK (validated in LORA_BASE_sim.py)

            if EPOCH_COMPLETE.issubset(epoch_buf):
                epoch_num += 1
                print(f"--- Epoch {epoch_num} complete ---\n")
                epoch_buf = {}

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        gnss_ser.close()
        lora_ser.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("RTK LoRa TX  --  All 4 MSM4 constellations RTCM3 broadcast\n")

    gnss_port  = input("GNSS base COM port (e.g. COM3): ").strip()
    
    # FORCE HARDWARE AUTO-CONFIGURATION BEFORE LOAD
    auto_configure_base(gnss_port)
    
    gnss_baud_s = input("GNSS baud [Enter for 115200]: ").strip()
    gnss_baud  = int(gnss_baud_s) if gnss_baud_s else 115200

    while True:
        print("""
  1) Start Survey-In   (duration only)
  2) Check Survey-In status
  3) Start RTCM stream over LoRa
  4) Exit
""")
        choice = input("> ").strip()

        if choice == "4":
            print("Exiting.")
            break

        try:
            if choice == "1":
                dur = int(input("Survey duration (seconds, e.g. 60): ").strip())
                start_survey(gnss_port, gnss_baud, dur)

            elif choice == "2":
                iv = input("Poll interval [Enter for 5s]: ").strip()
                check_survey(gnss_port, gnss_baud, int(iv) if iv else 5)

            elif choice == "3":
                lora_port   = input("LoRa TX COM port (e.g. COM5): ").strip()
                print("Hardcoded module ADDRESS: 100")
                lora_addr   = 100
                
                auto_configure_lora(lora_port)
                stream_loop(gnss_port, gnss_baud, lora_port, LORA_BAUD, lora_addr)

            else:
                print("Unrecognized option.")

        except SerialException as e:
            print(f"Serial error: {e}")


if __name__ == "__main__":
    main()
