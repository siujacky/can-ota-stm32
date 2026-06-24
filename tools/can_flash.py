#!/usr/bin/env python3
"""
can_flash.py — CAN firmware uploader for bipropellant_can_generic bootloader.

Upload channels (unchanged):
    0x7DE   BL -> host  : hello / identity broadcast
    0x7DD   host -> BL  : firmware data (UID0 match triggers upload)

Control channel (new):
    0x7DC   host -> BL  : control commands
    0x7DB   BL -> host  : control responses

Control frame layout (8 bytes on 0x7DC):
    [0]    = CMD byte
    [1..3] = uid0 low 3 bytes (auth)
    [4..7] = args (4 bytes, LE)

Auth is taken from the hello broadcast (advert[2..4] = uid0 bytes 0-2).

Usage examples:
    # Upload to Slot A (default):
    python3 can_flash.py -d can0 -f build/app.bin -i <uid0_hex>

    # Upload to Slot B:
    python3 can_flash.py -d can0 -f build/app.bin --slot b -i <uid0_hex>

    # Upload to custom address:
    python3 can_flash.py -d can0 -f build/app.bin --addr 0x08020000 -i <uid0_hex>

    # Control commands (no firmware needed):
    python3 can_flash.py -d can0 --status
    python3 can_flash.py -d can0 --set-boot b
    python3 can_flash.py -d can0 --clear-nvram
    python3 can_flash.py -d can0 --boot

Requirements:
    pip install python-can
"""

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("Error: python-can not installed.  Run: pip install python-can")

# ---------------------------------------------------------------------------
# CAN frame IDs
# ---------------------------------------------------------------------------
BL_HELLO_ID    = 0x7DE   # BL -> host: identity broadcast; uid0 in advert[2..5]
BL_DATA_ID     = 0x7DD   # host -> BL: firmware upload data / upload trigger
BL_CTRL_RX_ID  = 0x7DC   # host -> BL: control command
BL_CTRL_TX_ID  = 0x7DB   # BL -> host: control response

FLASH_PAGE_SIZE = 1024

# ---------------------------------------------------------------------------
# Control command bytes (must match main_bl.c BL_CMD_* defines)
# ---------------------------------------------------------------------------
BL_CMD_BOOT          = 0x00
BL_CMD_UPLOAD_SLOT_A = 0x01
BL_CMD_UPLOAD_SLOT_B = 0x02
BL_CMD_UPLOAD_ADDR   = 0x03
BL_CMD_SET_BOOT_A    = 0x04
BL_CMD_SET_BOOT_B    = 0x05
BL_CMD_CLEAR_NVRAM   = 0x06
BL_CMD_STATUS        = 0x07
BL_CMD_ENTER_DFU     = 0x08   # Jump to ROM DFU (VID_0483:PID_DF11)

# Control response status codes
BL_STATUS_OK         = 0x00
BL_STATUS_AUTH_FAIL  = 0x01
BL_STATUS_INVALID    = 0x02
BL_STATUS_FLASH_ERR  = 0x03

_STATUS_NAMES = {
    BL_STATUS_OK:        "OK",
    BL_STATUS_AUTH_FAIL: "AUTH_FAIL",
    BL_STATUS_INVALID:   "INVALID",
    BL_STATUS_FLASH_ERR: "FLASH_ERR",
}
_CMD_NAMES = {
    BL_CMD_BOOT:          "BOOT",
    BL_CMD_UPLOAD_SLOT_A: "UPLOAD_SLOT_A",
    BL_CMD_UPLOAD_SLOT_B: "UPLOAD_SLOT_B",
    BL_CMD_UPLOAD_ADDR:   "UPLOAD_ADDR",
    BL_CMD_SET_BOOT_A:    "SET_BOOT_A",
    BL_CMD_SET_BOOT_B:    "SET_BOOT_B",
    BL_CMD_CLEAR_NVRAM:   "CLEAR_NVRAM",
    BL_CMD_STATUS:        "STATUS",
}


# ---------------------------------------------------------------------------
# STATUS banner (3-frame response on 0x7DB)
# ---------------------------------------------------------------------------

# Bootloader slot start addresses (known constants from flash_bl.h)
SLOT_A_START = 0x08002000
SLOT_B_START = 0x08020000

# Sub-frame tags: top byte of frame[0] — 0x07=frame-0, 0x17=frame-1, 0x27=frame-2
_STATUS_FRAME_TAGS = {0x07, 0x17, 0x27}


def collect_status_frames(bus, timeout=0.5):
    """
    Collect the 3 STATUS sub-frames broadcast on BL_CTRL_TX_ID (0x7DB).
    Frame tags (byte[0]):
      0x07 — frame-0: [STATUS][boot_slot][target_slot][uid0 LE]
      0x17 — frame-1: [STATUS][uid1 LE][uid2 low 2B]
      0x27 — frame-2: [STATUS][uid2 high 2B][ver_major][ver_minor][0xA1][0xB2]
    Returns a dict {0x07: bytes, 0x17: bytes, 0x27: bytes} or fewer if timeout.
    """
    frames = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(frames) < 3:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.arbitration_id == BL_CTRL_TX_ID and len(msg.data) >= 8:
            tag = msg.data[0]
            if tag in _STATUS_FRAME_TAGS:
                frames[tag] = bytes(msg.data)
    return frames


def print_banner(frames):
    """
    Parse the 3 STATUS sub-frames and print the bootloader banner:
      ===================================
        biPropellant CAN Generic BL v1.0
        UID: AABBCCDD-11223344-EEFF5566
        Active slot: A (0x08002000)
        Next boot:   A
      ===================================
    Falls back to partial info if not all frames are present.
    """
    f0 = frames.get(0x07)
    f1 = frames.get(0x17)
    f2 = frames.get(0x27)

    # Extract fields with graceful fallbacks
    boot_slot = f0[2] if f0 else 0
    slot_char = "B" if boot_slot == 1 else "A"
    slot_addr = SLOT_B_START if boot_slot == 1 else SLOT_A_START

    uid0 = struct.unpack_from("<I", f0, 4)[0] if f0 else 0
    uid1 = struct.unpack_from("<I", f1, 2)[0] if f1 else 0
    uid2_lo = struct.unpack_from("<H", f1, 6)[0] if f1 else 0
    uid2_hi = struct.unpack_from("<H", f2, 2)[0] if f2 else 0
    uid2 = (uid2_hi << 16) | uid2_lo

    ver_maj = f2[4] if f2 else 1
    ver_min = f2[5] if f2 else 0

    uid_str = f"{uid0:08X}-{uid1:08X}-{uid2:08X}"

    print("===================================")
    print(f"  biPropellant CAN Generic BL v{ver_maj}.{ver_min}")
    print(f"  UID:         {uid_str}")
    print(f"  Active slot: {slot_char} (0x{slot_addr:08X})")
    print(f"  Next boot:   {slot_char}")
    print("===================================")
    return uid2   # caller may need uid2 for the upload trigger


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def discover_bootloader(bus, uid_filter=None, timeout=3.0):
    """
    Wait for the BL hello broadcast on BL_HELLO_ID (0x7DE).
    The hello frame contains uid0 in advert[2..5] (LE).
    After the hello, the bootloader auto-broadcasts 3 STATUS frames on 0x7DB
    — collect them and print the banner.
    Returns (uid0: int, uid2: int, uid0_auth: bytes[3]) where uid0_auth is
    the 3-byte auth token used in control commands, and uid2 is needed for the
    upload trigger.
    Raises TimeoutError if no hello arrives within timeout seconds.
    """
    deadline = time.monotonic() + timeout
    print(f"Waiting for bootloader on CAN ...")
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if msg.arbitration_id == BL_HELLO_ID and len(msg.data) >= 6:
            uid0 = struct.unpack_from("<I", msg.data, 2)[0]
            if uid_filter is not None and uid0 != uid_filter:
                continue
            uid0_auth = bytes(msg.data[2:5])   # first 3 bytes of uid0
            # Collect the STATUS banner frames that the BL broadcasts right after hello
            frames = collect_status_frames(bus, timeout=0.3)
            uid2 = print_banner(frames)
            return uid0, uid2, uid0_auth
    raise TimeoutError(
        "No bootloader hello received within timeout.\n"
        "  Check: board is in reset window, CAN bus is up, baud rate matches."
    )


# ---------------------------------------------------------------------------
# Control channel
# ---------------------------------------------------------------------------

def _build_ctrl_frame(cmd, uid0_auth, args4=b"\x00\x00\x00\x00"):
    """Build an 8-byte control frame for BL_CTRL_RX_ID."""
    assert len(uid0_auth) == 3, "uid0_auth must be exactly 3 bytes"
    assert len(args4) == 4,     "args4 must be exactly 4 bytes"
    return bytes([cmd]) + uid0_auth + args4


def send_ctrl_cmd(bus, cmd, uid0_auth, args4=b"\x00\x00\x00\x00", timeout=2.0):
    """
    Send a control command on 0x7DC and wait for the matching response on 0x7DB.
    Returns the raw 8-byte response data.
    Raises TimeoutError if no matching response arrives, or RuntimeError on
    non-OK status from the bootloader.
    """
    frame = _build_ctrl_frame(cmd, uid0_auth, args4)
    bus.send(can.Message(
        arbitration_id=BL_CTRL_RX_ID,
        data=frame,
        is_extended_id=False,
    ))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        resp = bus.recv(timeout=0.1)
        if resp is None:
            continue
        if resp.arbitration_id == BL_CTRL_TX_ID and len(resp.data) >= 2:
            if resp.data[0] != cmd:
                continue   # response for a different command
            status = resp.data[1]
            if status != BL_STATUS_OK:
                status_str = _STATUS_NAMES.get(status, f"0x{status:02X}")
                raise RuntimeError(
                    f"Command {_CMD_NAMES.get(cmd, f'0x{cmd:02X}')} failed: {status_str}"
                )
            return bytes(resp.data)
    raise TimeoutError(
        f"No response to {_CMD_NAMES.get(cmd, f'0x{cmd:02X}')} within {timeout:.1f}s"
    )


def ctrl_status(bus, uid0_auth):
    """
    Send a STATUS query (0x07) and collect + display the 3-frame banner response.
    The bootloader responds with frames tagged 0x07/0x17/0x27 on 0x7DB.
    """
    # Send the STATUS command (triggers bl_broadcast_status() in the BL)
    frame = _build_ctrl_frame(BL_CMD_STATUS, uid0_auth)
    bus.send(can.Message(
        arbitration_id=BL_CTRL_RX_ID,
        data=frame,
        is_extended_id=False,
    ))
    # Collect all 3 STATUS frames (they arrive within a few ms)
    frames = collect_status_frames(bus, timeout=1.0)
    if not frames:
        print("  (No STATUS response received)")
        return
    print_banner(frames)


def ctrl_set_boot(bus, uid0_auth, slot_char):
    """Send SET_BOOT_A or SET_BOOT_B. Board jumps immediately after."""
    cmd  = BL_CMD_SET_BOOT_B if slot_char.lower() == "b" else BL_CMD_SET_BOOT_A
    name = "B" if slot_char.lower() == "b" else "A"
    print(f"  Setting next boot slot to {name} (board will reboot) ...")
    send_ctrl_cmd(bus, cmd, uid0_auth)
    print(f"  Done — board is now booting from Slot {name}.")


def ctrl_clear_nvram(bus, uid0_auth):
    """Erase app NVRAM and reboot."""
    print("  Erasing application NVRAM (0x0803F000, 4 KB) ...")
    send_ctrl_cmd(bus, BL_CMD_CLEAR_NVRAM, uid0_auth, timeout=4.0)
    print("  Done — board is booting with cleared NVRAM.")


def ctrl_boot(bus, uid0_auth):
    """Tell the bootloader to jump to the application immediately."""
    print("  Sending BOOT command ...")
    send_ctrl_cmd(bus, BL_CMD_BOOT, uid0_auth)
    print("  Done — board is jumping to application.")


def ctrl_set_upload_slot(bus, uid0_auth, slot_char):
    """Set the upload target slot via the control channel."""
    if slot_char.lower() == "b":
        cmd, name = BL_CMD_UPLOAD_SLOT_B, "Slot B"
    else:
        cmd, name = BL_CMD_UPLOAD_SLOT_A, "Slot A"
    print(f"  Setting upload target to {name} ...")
    send_ctrl_cmd(bus, cmd, uid0_auth)
    print(f"  Upload target set to {name}.")


def ctrl_set_upload_addr(bus, uid0_auth, addr):
    """Set the upload target to a custom address via the control channel."""
    args4 = struct.pack("<I", addr)
    print(f"  Setting upload target to 0x{addr:08X} ...")
    send_ctrl_cmd(bus, BL_CMD_UPLOAD_ADDR, uid0_auth, args4=args4)
    print(f"  Upload target set to 0x{addr:08X}.")


# ---------------------------------------------------------------------------
# CRC32 matching the bootloader's crc32_page()
# Polynomial 0x04C11DB7 (MPEG-2), no bit-reflection, init 0xFFFFFFFF,
# data treated as big-endian 32-bit words (matching the BL's __builtin_bswap32).
# ---------------------------------------------------------------------------

def _crc32_bl(data: bytes) -> int:
    crc = 0xFFFFFFFF
    i = 0
    while i + 3 < len(data):
        word = (data[i] << 24) | (data[i+1] << 16) | (data[i+2] << 8) | data[i+3]
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
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
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


# ---------------------------------------------------------------------------
# Firmware upload (bipropellant_can_generic 0x7DD upload protocol)
# ---------------------------------------------------------------------------

def flash_firmware(bus, data: bytes, uid0: int) -> bool:
    """
    Upload firmware binary using the bipropellant_can_generic CAN protocol.

    Protocol (matching main_bl.c can_upload()):
      1. Host sends uid0 (DESIG_UNIQUE_ID0, 4 bytes LE) on 0x7DD.
         Bootloader compares against DESIG_UNIQUE_ID0 (not UID2 — fixed).
      2. BL replies with byte 'S' on CAN_TX_ID (0x7DE).
      3. Host sends n_pages (4 bytes LE) on 0x7DD.
      4. For each page: 129 frames total on 0x7DD:
           Frames 0..127: 8 bytes each = 1024 bytes (full page).
           Frame 128:     CRC32 LE (4 bytes) + 4 bytes padding.
           CRC computed over all 1024 bytes (not 1020 — fixed).
         BL replies 'P' (page OK) or 'E' (CRC error, retry) or 'D' (last page done).
      5. After all pages: BL sends 'D' and jumps.

    Returns True on success.
    """
    # Pad to full page boundary
    pad_len = (-len(data)) % FLASH_PAGE_SIZE
    data    = data + b"\xff" * pad_len
    n_pages = len(data) // FLASH_PAGE_SIZE
    print(f"Firmware: {len(data)} bytes  ({n_pages} pages of {FLASH_PAGE_SIZE} B)")

    # Step 1: send upload trigger (uid0 — bootloader matches UID0 from hello frame)
    bus.send(can.Message(
        arbitration_id=BL_DATA_ID,
        data=struct.pack("<I", uid0),
        is_extended_id=False,
    ))
    print(f"  Sent upload trigger (UID0=0x{uid0:08X}) on 0x{BL_DATA_ID:03X}")

    # Step 2: wait for 'S'
    print("  Waiting for 'S' (ready) from bootloader ...")
    deadline = time.monotonic() + 2.0
    ready = False
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if msg.arbitration_id == BL_HELLO_ID and len(msg.data) >= 1 and msg.data[0] == ord('S'):
            ready = True
            break
    if not ready:
        print("ERROR: Bootloader did not send 'S' after trigger.")
        return False

    # Step 3: send page count
    bus.send(can.Message(
        arbitration_id=BL_DATA_ID,
        data=struct.pack("<I", n_pages),
        is_extended_id=False,
    ))

    # Step 4: send pages — 128 data frames + 1 CRC frame = 129 total per page
    for page_idx in range(n_pages):
        page_data = data[page_idx * FLASH_PAGE_SIZE:(page_idx + 1) * FLASH_PAGE_SIZE]
        crc       = _crc32_bl(page_data)   # CRC over full 1024 bytes (was 1020 — fixed)

        for attempt in range(8):
            # Frames 0..127: 8 bytes each = 1024 bytes (full page)
            for f in range(128):
                chunk = page_data[f * 8:(f + 1) * 8]
                bus.send(can.Message(
                    arbitration_id=BL_DATA_ID,
                    data=chunk,
                    is_extended_id=False,
                ))
                time.sleep(0.0002)   # 200 µs inter-chunk gap

            # Frame 128: CRC32 LE (4 bytes) + 4 bytes padding
            bus.send(can.Message(
                arbitration_id=BL_DATA_ID,
                data=struct.pack("<I", crc) + b"\x00\x00\x00\x00",
                is_extended_id=False,
            ))

            # Wait for 'P' or 'E' on 0x7DE
            deadline = time.monotonic() + 2.0
            ack_byte = None
            while time.monotonic() < deadline:
                resp = bus.recv(timeout=0.1)
                if resp is None:
                    continue
                if resp.arbitration_id == BL_HELLO_ID and len(resp.data) >= 1:
                    ack_byte = resp.data[0]
                    break

            if ack_byte == ord('P'):
                break   # page accepted; move to next
            elif ack_byte == ord('D'):
                # Bootloader sends 'D' on the last page instead of 'P'.
                # Guard: only valid on the final page.
                if page_idx == n_pages - 1:
                    print()
                    print("  Bootloader sent 'D' — firmware written, board rebooting.")
                    return True
                else:
                    print(f"\nERROR: got 'D' on non-final page {page_idx} — aborting.")
                    return False
            elif ack_byte == ord('E'):
                print(f"  Page {page_idx} CRC error, retry {attempt + 1}/8 ...")
                continue
            else:
                print(f"\nTimeout waiting for page {page_idx} ACK")
                return False
        else:
            print(f"\nPage {page_idx} failed after 8 retries.")
            return False

        pct = (page_idx + 1) * 100 // n_pages
        print(f"  Page {page_idx + 1:3d}/{n_pages}  [{pct:3d}%]", end="\r", flush=True)

    print()  # newline after progress bar
    # All pages sent via 'P' ACKs; wait for final 'D' from bootloader
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        resp = bus.recv(timeout=0.1)
        if resp and resp.arbitration_id == BL_HELLO_ID and len(resp.data) >= 1:
            if resp.data[0] == ord('D'):
                print("  Bootloader sent 'D' — firmware written, board rebooting.")
                return True
    print("  WARNING: no 'D' received, but all pages completed.")
    return True


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="CAN firmware uploader for bipropellant_can_generic bootloader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Upload firmware to Slot A (default):
    %(prog)s -d can0 -f build/app.bin -i <uid0_hex>

  Upload firmware to Slot B:
    %(prog)s -d can0 -f build/app.bin --slot b -i <uid0_hex>

  Upload to custom address:
    %(prog)s -d can0 -f build/app.bin --addr 0x08020000 -i <uid0_hex>

  Query bootloader status:
    %(prog)s -d can0 --status

  Set next boot slot and reboot:
    %(prog)s -d can0 --set-boot b

  Clear application NVRAM and reboot:
    %(prog)s -d can0 --clear-nvram

  Jump to application immediately:
    %(prog)s -d can0 --boot
""",
    )
    parser.add_argument("-d", "--device",    required=True,
                        help="SocketCAN interface (e.g. can0)")
    parser.add_argument("-f", "--firmware",  default=None,
                        help="Firmware .bin file to upload")
    parser.add_argument("-i", "--uid",       default=None,
                        help="DESIG_UNIQUE_ID0 in hex — required for firmware upload trigger (printed in bootloader banner)")

    # Upload target modifiers
    upload_grp = parser.add_mutually_exclusive_group()
    upload_grp.add_argument("--slot",  choices=["a", "b"], default=None,
                            help="Upload target slot (default=a)")
    upload_grp.add_argument("--addr",  default=None,
                            help="Upload to custom flash address (hex, e.g. 0x08020000)")

    # Control-only commands (mutually exclusive with each other and with upload modifiers)
    ctrl_grp = parser.add_mutually_exclusive_group()
    ctrl_grp.add_argument("--set-boot",    choices=["a", "b"], default=None, dest="set_boot",
                          help="Set next boot slot and reboot")
    ctrl_grp.add_argument("--clear-nvram", action="store_true",
                          help="Erase application NVRAM and reboot")
    ctrl_grp.add_argument("--boot",        action="store_true",
                          help="Tell bootloader to jump to application now")
    ctrl_grp.add_argument("--status",      action="store_true",
                          help="Query bootloader status (boot slot, UID0)")
    ctrl_grp.add_argument("--enter-dfu",  action="store_true", dest="enter_dfu",
                          help="Enter STM32 ROM DFU (VID_0483:PID_DF11); fixes Windows USB error")

    args = parser.parse_args()

    # Validate arguments
    control_only = bool(args.set_boot or args.clear_nvram or args.boot or args.status or args.enter_dfu)
    if not args.firmware and not control_only:
        parser.error("Provide -f FIRMWARE and/or a control command (--status, --set-boot, etc.)")

    addr_target = None
    if args.addr:
        try:
            addr_target = int(args.addr, 16)
        except ValueError:
            parser.error(f"Invalid --addr value: {args.addr!r} (expected hex, e.g. 0x08020000)")

    uid0_override = int(args.uid, 16) if args.uid else None

    bus = can.interface.Bus(channel=args.device, bustype="socketcan")
    try:
        # Discover bootloader — prints the banner automatically (startup broadcast)
        uid0_from_bl, uid2_from_bl, uid0_auth = discover_bootloader(bus, timeout=3.0)

        # uid0 for upload trigger — prefer explicit -i override, else use discovered value
        uid0_trigger = uid0_override if uid0_override is not None else uid0_from_bl

        # ---- Control-only commands ----------------------------------------
        if args.status:
            # Banner already printed by discover_bootloader; re-query is optional
            # but gives the freshest data if commands ran before --status
            ctrl_status(bus, uid0_auth)
            if not args.firmware:
                return

        if args.set_boot:
            ctrl_set_boot(bus, uid0_auth, args.set_boot)
            return   # board is rebooting

        if args.clear_nvram:
            ctrl_clear_nvram(bus, uid0_auth)
            return   # board is rebooting

        if args.boot:
            ctrl_boot(bus, uid0_auth)
            return   # board is jumping to app

        if args.enter_dfu:
            print("  Sending ENTER_DFU command (board will reboot as VID_0483:PID_DF11)...")
            send_ctrl_cmd(bus, BL_CMD_ENTER_DFU, uid0_auth, timeout=2.0)
            print("  Done. Connect USB and open STM32CubeProgrammer (Windows) or dfu-util.")
            return

        # ---- Firmware upload -----------------------------------------------
        if not args.firmware:
            return

        with open(args.firmware, "rb") as fh:
            firmware = fh.read()
        print(f"Loaded {args.firmware}: {len(firmware)} bytes")

        # Set upload target via control channel (before triggering upload)
        if addr_target is not None:
            ctrl_set_upload_addr(bus, uid0_auth, addr_target)
        elif args.slot == "b":
            ctrl_set_upload_slot(bus, uid0_auth, "b")
        elif args.slot == "a":
            ctrl_set_upload_slot(bus, uid0_auth, "a")
        # else: default Slot A — no control command needed (bootloader default)

        # Proceed with standard upload protocol (0x7DD)
        ok = flash_firmware(bus, firmware, uid0=uid0_trigger)

        if ok:
            print("Flash complete. Board is booting.")
        else:
            print("Flash FAILED. Use ST-Link to recover.")
            sys.exit(1)

    except TimeoutError as exc:
        print(f"Error: {exc}")
        sys.exit(1)
    except RuntimeError as exc:
        print(f"Error: {exc}")
        sys.exit(1)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
