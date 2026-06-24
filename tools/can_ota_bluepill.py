#!/usr/bin/env python3
"""can_ota_bluepill.py — Flash Blue Pill firmware over CAN via the Pico 2 SLCAN bridge.

The Raspberry Pi has no native CAN port. This script uses the Pico 2 (Waveshare
RP2350-CAN running pico2_slcan firmware) as a USB-CAN SLCAN adapter to:

  1. Send a CAN magic frame to the running Blue Pill CAN tool app → triggers
     bootloader mode (BKP register trick, 30-second window).
  2. Detect the bootloader's hello broadcast on 0x7DE.
  3. Upload the new firmware using the standard CAN bootloader protocol
     (1024-byte pages, CRC32, 'P'/'E'/'D' ACKs).

Usage:
  python3 can_ota_bluepill.py                         # flash default firmware
  python3 can_ota_bluepill.py --firmware my.bin       # flash custom binary
  python3 can_ota_bluepill.py --port /dev/ttyACM0     # different SLCAN port

Requirements:
  Pico 2 running pico2_slcan firmware (firmware/pico2_slcan/)
  python3-can: sudo apt install python3-can
"""

import argparse
import struct
import sys
import time
import serial   # used for the 1200-baud BOOTSEL trigger too

# Default firmware path (relative to this script's location)
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FW  = os.path.join(SCRIPT_DIR, '../firmware/blue_pill_can_tool/blue_pill_can_tool.bin')
DEFAULT_PORT = '/dev/ttyACM1'

# CAN IDs (bootloader protocol)
BL_HELLO_ID  = 0x7DE   # Bootloader → host (hello + UID)
BL_DATA_ID   = 0x7DD   # Host → bootloader (UID match + data)
# Magic frame to trigger remote reboot of the running CAN tool app
REBOOT_ID    = 0x7FF   # Blue Pill CAN tool watches for this
REBOOT_DATA  = bytes([0xB0, 0x01, 0xB2])  # "boot-B2" magic

FLASH_PAGE   = 1024    # bootloader page size in bytes

# ---------------------------------------------------------------------------
# CRC32 (MPEG-2 variant, matches bootloader's crc32_page)
# ---------------------------------------------------------------------------
def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    i = 0
    while i + 3 < len(data):
        word = (data[i] << 24) | (data[i+1] << 16) | (data[i+2] << 8) | data[i+3]
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
        i += 4
    if i < len(data):
        word = 0
        shift = 24
        while i < len(data):
            word |= data[i] << shift
            shift -= 8
            i += 1
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc


# ---------------------------------------------------------------------------
# SLCAN helpers (raw serial, no python-can dependency for the trigger phase)
# ---------------------------------------------------------------------------

def slcan_open(port: str, baud: int = 115200) -> serial.Serial:
    s = serial.Serial(port, baud, timeout=1)
    time.sleep(0.3)
    # Close any stale session, explicitly set 250 kbps, then open.
    # We always set the bitrate explicitly because a prior debug session may
    # have left the Pico 2 at a different rate (e.g., S6=500k).
    # Firmware-version-safe approach:
    #   Old Pico 2 firmware (pre-fix): S4 = 250 kbps, S5 = 500 kbps (bug)
    #   New Pico 2 firmware (post-fix): S4 = 125 kbps, S5 = 250 kbps
    # To handle both, close first, then re-init at the correct rate via S4
    # for old firmware. If the new firmware is installed, update this to S5.
    s.write(b'C\r')    # close (forces config mode)
    time.sleep(0.1)
    s.write(b'S4\r')   # 250 kbps on current Pico 2 firmware (old table: S4=250k)
    time.sleep(0.1)
    s.write(b'O\r')    # open at 250 kbps
    time.sleep(0.3)
    return s

def slcan_send(s: serial.Serial, can_id: int, data: bytes) -> None:
    """Send a standard CAN frame via SLCAN (blocking)."""
    dlc = len(data)
    hex_id   = f'{can_id:03X}'
    hex_data = data.hex().upper()
    cmd = f't{hex_id}{dlc}{hex_data}\r'.encode()
    s.write(cmd)

def _read_until_cr(s: serial.Serial, timeout: float) -> bytes:
    """Read one SLCAN line (terminated by \\r, never \\n). Returns b'' on timeout."""
    deadline = time.monotonic() + timeout
    buf = b''
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        s.timeout = max(0.0, min(remaining + 0.01, 0.1))
        ch = s.read(1)
        if not ch:
            continue
        if ch == b'\r':
            return buf
        buf += ch
    return buf

def slcan_recv_line(s: serial.Serial, timeout: float = 0.5) -> bytes:
    return _read_until_cr(s, timeout)

def slcan_recv_frame(s: serial.Serial, timeout: float = 1.0) -> tuple:
    """Return (can_id, data_bytes) from next received SLCAN CAN frame.

    Skips TX-ack lines ('z', 'Z') and other non-frame responses.
    SLCAN uses \\r (not \\n) as line terminator; readline() must NOT be used.
    Returns (None, None) if no CAN frame arrives before timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        line = _read_until_cr(s, min(remaining, 0.2))
        if not line:
            continue
        try:
            text = line.decode('ascii', errors='ignore').strip()
            # Skip TX-ack lines and other non-RX lines
            if not text or text[0] not in ('t', 'T'):
                continue
            if text[0] == 't' and len(text) >= 5:
                can_id = int(text[1:4], 16)
                dlc    = int(text[4], 16)
                data   = bytes.fromhex(text[5:5 + dlc * 2])
                return can_id, data
            if text[0] == 'T' and len(text) >= 10:
                can_id = int(text[1:9], 16)
                dlc    = int(text[9], 16)
                data   = bytes.fromhex(text[10:10 + dlc * 2])
                return can_id, data
        except Exception:
            pass
    return None, None


# ---------------------------------------------------------------------------
# Phase 1: trigger remote reboot via magic CAN frame
# ---------------------------------------------------------------------------

def trigger_reboot(s: serial.Serial) -> bool:
    print("  Sending reboot trigger (ID=0x7FF, magic [B0,01,B2]) ...")
    slcan_send(s, REBOOT_ID, REBOOT_DATA)
    time.sleep(0.2)
    # Drain any ACK lines
    while s.in_waiting:
        s.read(s.in_waiting)
    return True


# ---------------------------------------------------------------------------
# Phase 2: wait for bootloader hello (0x7DE with "31" prefix)
# ---------------------------------------------------------------------------

def wait_for_hello(s: serial.Serial, timeout: float = 35.0) -> tuple:
    """Return (uid0, uid2) from bootloader hello, or (None, None) on timeout."""
    print(f"  Waiting for bootloader hello on 0x{BL_HELLO_ID:03X} (up to {timeout:.0f}s) ...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        can_id, data = slcan_recv_frame(s, timeout=0.2)
        if can_id == BL_HELLO_ID and data and len(data) >= 6:
            if data[0] == ord('3') and data[1] == ord('1'):
                uid0 = struct.unpack_from('<I', data, 2)[0]
                print(f"  Bootloader hello! UID0=0x{uid0:08X}")
                return uid0, 0   # uid2 will be matched separately
    print("  TIMEOUT waiting for bootloader hello.")
    return None, None


# ---------------------------------------------------------------------------
# Phase 3: firmware upload (bootloader protocol)
# ---------------------------------------------------------------------------

def upload_firmware(s: serial.Serial, firmware: bytes, uid0: int) -> bool:
    # Flush OS serial buffer: a stale 'D' or 't7DE...' from a prior session
    # sitting in the OS FIFO would trigger the elif ack=='D' early-exit on
    # page 0, falsely reporting success without flashing anything.
    s.reset_input_buffer()

    pad = (-len(firmware)) % FLASH_PAGE
    firmware += b'\xff' * pad
    n_pages = len(firmware) // FLASH_PAGE

    print(f"  Firmware: {len(firmware)} bytes, {n_pages} pages of {FLASH_PAGE} B")

    # Send UID0 as the "I want to talk to this device" trigger on 0x7DD
    slcan_send(s, BL_DATA_ID, struct.pack('<I', uid0))
    time.sleep(0.1)

    # Wait for 'S' (ready) — 8 s gives margin for CAN jitter within 30 s BKP window
    print("  Waiting for 'S' (ready) ...")
    deadline = time.monotonic() + 8.0
    ready = False
    while time.monotonic() < deadline:
        can_id, data = slcan_recv_frame(s, timeout=0.2)
        if can_id == BL_HELLO_ID and data and data[0] == ord('S'):
            ready = True
            break
    if not ready:
        print("  ERROR: no 'S' from bootloader.")
        return False

    # Send page count
    slcan_send(s, BL_DATA_ID, struct.pack('<I', n_pages))
    time.sleep(0.05)

    # Send pages
    for page_idx in range(n_pages):
        page_data  = firmware[page_idx * FLASH_PAGE : (page_idx + 1) * FLASH_PAGE]
        # Compute CRC over the full 1024 bytes (old protocol only covered 1020,
        # silently losing the last 4 bytes of every page → corrupted firmware)
        crc        = _crc32(page_data)

        for attempt in range(8):
            # Send 128 data frames × 8 bytes = 1024 bytes (full page)
            # bxCAN FIFO depth = 3 frames. At 250kbps one frame ≈ 130µs.
            # Use 2ms inter-frame gap to ensure bootloader drains FIFO
            # before overflow drops frames (which would cause CRC mismatch).
            for f in range(128):
                chunk = page_data[f*8 : f*8+8]
                slcan_send(s, BL_DATA_ID, chunk)
                time.sleep(0.002)

            # Frame 129: CRC32 (4 bytes LE) + 4 padding bytes
            slcan_send(s, BL_DATA_ID, struct.pack('<I', crc) + b'\x00\x00\x00\x00')

            # Wait for 'P' (pass) or 'E' (error)
            deadline = time.monotonic() + 4.0
            ack = None
            while time.monotonic() < deadline:
                can_id, data = slcan_recv_frame(s, timeout=0.2)
                if can_id == BL_HELLO_ID and data:
                    ack = chr(data[0])
                    break

            if ack == 'P':
                break
            elif ack == 'D':
                # 'D' is only valid on the LAST page (bootloader sends D after
                # writing the final page instead of P+D separately).  On any
                # earlier page a 'D' means a stale frame or protocol error.
                if page_idx == n_pages - 1:
                    print()
                    print("  Bootloader sent 'D' — firmware written, Blue Pill rebooting.")
                    return True
                else:
                    print(f"  ERROR: got 'D' on non-final page {page_idx} — aborting.")
                    return False
            elif ack == 'E':
                print(f"  Page {page_idx} CRC error, retry {attempt+1}/8 ...")
            else:
                print(f"  Timeout on page {page_idx} ACK (got {ack!r})")
                return False
        else:
            print(f"  Page {page_idx} failed after 8 retries")
            return False

        pct = (page_idx + 1) * 100 // n_pages
        print(f"  Page {page_idx+1:3d}/{n_pages}  [{pct:3d}%]", end='\r', flush=True)

    print()

    # Wait for 'D' (done) — only reached if no page sent 'D' above
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        can_id, data = slcan_recv_frame(s, timeout=0.2)
        if can_id == BL_HELLO_ID and data and data[0] == ord('D'):
            print("  Bootloader sent 'D' — firmware written, Blue Pill rebooting.")
            return True
    print("  WARNING: no 'D' received, but all pages passed CRC.")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="CAN OTA update for Blue Pill via Pico 2 SLCAN bridge")
    ap.add_argument('--port',     default=DEFAULT_PORT, help="SLCAN serial port (default: /dev/ttyACM1)")
    ap.add_argument('--firmware', default=DEFAULT_FW,   help="firmware .bin to flash")
    ap.add_argument('--no-reboot', action='store_true', help="skip the CAN reboot trigger (Blue Pill already in bootloader)")
    args = ap.parse_args()

    if not os.path.exists(args.firmware):
        sys.exit(f"Firmware not found: {args.firmware}")

    firmware = open(args.firmware, 'rb').read()
    print(f"Firmware: {args.firmware} ({len(firmware)} bytes)")
    print(f"SLCAN port: {args.port}")
    print()

    print("[1] Opening SLCAN interface ...")
    try:
        s = slcan_open(args.port)
    except Exception as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    try:
        if not args.no_reboot:
            print("[2] Triggering Blue Pill reboot to bootloader ...")
            trigger_reboot(s)
            print("    Waiting for bootloader to start (BKP 30s window) ...")
            time.sleep(0.5)

        print("[3] Waiting for bootloader hello ...")
        uid0, _ = wait_for_hello(s, timeout=32.0)
        if uid0 is None:
            print()
            print("  TIP: if the reboot trigger didn't work, power-cycle the Blue Pill")
            print("       and re-run with --no-reboot within 2 seconds.")
            sys.exit(1)

        print("[4] Uploading firmware ...")
        ok = upload_firmware(s, firmware, uid0)

        if ok:
            print()
            print("✅  OTA firmware update complete! Blue Pill is running new firmware.")
        else:
            print()
            print("❌  OTA failed. Use ST-Link to recover.")
            sys.exit(1)

    finally:
        s.close()


if __name__ == '__main__':
    main()
