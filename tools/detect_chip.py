#!/usr/bin/env python3
"""detect_chip.py — Detect ST-Link programmer and connected STM32/GD32 chip.

Tries three methods in order:
  1. st-info --probe   (stlink-tools package — most detailed)
  2. openocd           (if stlink-tools absent)
  3. lsusb             (USB presence only — no chip details)

Also optionally listens on CAN for bootloader STATUS frames (frames 0x07/0x17/
0x27/0x37 on 0x7DB) which carry the chip DevID + flash size that the firmware
reads itself at runtime from DBGMCU_IDCODE and the flash-size register.

Usage:
  python3 detect_chip.py                  # ST-Link probe only
  python3 detect_chip.py -d can0          # ST-Link + CAN STATUS (board must be
                                          # in bootloader mode or send a STATUS query)
  python3 detect_chip.py -d can0 --query  # also send a STATUS query (0x07 on 0x7DC)

Requirements:
  ST-Link probe:  sudo apt install stlink-tools
  CAN STATUS:     pip install python-can
"""

import argparse
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Device ID → part name table (matches src/chip_detect.c)
# ---------------------------------------------------------------------------
DEVICE_IDS = {
    # STM32F1
    0x0412: ("STM32F103x4/x6",   "low-density",    6),
    0x0410: ("STM32F103x8/xB",   "medium-density", 20),
    0x0414: ("STM32F103xC/D/E",  "high-density",   48),
    0x0418: ("STM32F105/F107",   "connectivity",   64),
    0x0430: ("STM32F103xF/G",    "XL-density",     96),
    0x0411: ("STM32F2xx",        "F2",             128),
    0x0413: ("STM32F405/F407",   "F4",             192),
    0x0423: ("STM32F401xB/xC",   "F4",              64),  # Black Pill v1
    0x0433: ("STM32F401xD/xE",   "F4",              96),
    0x0419: ("STM32F42x/F43x",   "F4",             256),
    0x0421: ("STM32F446",        "F4",             128),
    0x0431: ("STM32F411xC/xE",   "F4",             128),  # Black Pill v2
    0x0434: ("STM32F469/F479",   "F4",             256),
    0x0441: ("STM32F412",        "F4",             128),
    0x0449: ("STM32F7xx",        "F7",             256),
    # STM32G4 — G474/G484 have TIM1+TIM8+TIM20, native CAN FD, HRTIM
    0x0468: ("STM32G431/G441",   "G4-Cat2",         32),
    0x0469: ("STM32G474/G484",   "G4-Cat3",        128),  # dual-motor + CAN FD
    0x0479: ("STM32G491/G4A1",   "G4-Cat4",         96),
    # GD32
    0xA641: ("GD32F103 (medium)", "medium",         20),
    0xB641: ("GD32F103 (high)",   "high",           48),
    0xB642: ("GD32F130",          "high",           48),
    # MM32
    0xCC68: ("MM32SPIN0x",        "SPIN",            8),
}

# ST-Link USB vendor/product IDs
STLINK_VID = 0x0483
STLINK_PIDS = {
    0x3744: "ST-Link/V1",
    0x3748: "ST-Link/V2",
    0x374B: "ST-Link/V2-1",
    0x374E: "ST-Link/V3E",
    0x374F: "ST-Link/V3 (no MSD)",
    0x3752: "ST-Link/V3 (2 VCP)",
    0x3753: "ST-Link/V3 (1 VCP)",
}

# CAN IDs (must match bootloader main_bl.c)
BL_HELLO_ID   = 0x7DE
BL_DATA_ID    = 0x7DD
BL_CTRL_RX_ID = 0x7DC
BL_CTRL_TX_ID = 0x7DB

STATUS_FRAME_TAGS = {0x07, 0x17, 0x27, 0x37}


# ---------------------------------------------------------------------------
# ST-Link detection
# ---------------------------------------------------------------------------

def _run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr, r.returncode
    except FileNotFoundError:
        return None, -1
    except subprocess.TimeoutExpired:
        return "", -1


def detect_via_stinfo():
    """Use st-info --probe to get chip details. Returns dict or None."""
    out, rc = _run(["st-info", "--probe"])
    if out is None:
        return None          # st-info not installed
    if rc != 0 or "Found 0" in out:
        return {"error": "No ST-Link found by st-info", "raw": out}

    info = {"raw": out, "source": "st-info"}
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("version:"):
            info["stlink_version"] = line.split(":", 1)[1].strip()
        elif line.startswith("serial:"):
            info["stlink_serial"] = line.split(":", 1)[1].strip()
        elif line.startswith("flash:"):
            parts = line.split()
            info["flash_bytes"] = int(parts[1])
            info["flash_kb"] = info["flash_bytes"] // 1024
            if "pagesize:" in line:
                info["page_size"] = int(parts[3].rstrip(")"))
        elif line.startswith("sram:"):
            info["sram_bytes"] = int(line.split()[1])
            info["sram_kb"] = info["sram_bytes"] // 1024
        elif line.startswith("chipid:"):
            raw_id = line.split(":", 1)[1].strip()
            info["dev_id"] = int(raw_id, 16) if raw_id.startswith("0x") else int(raw_id, 16)
        elif line.startswith("descr:"):
            info["stinfo_descr"] = line.split(":", 1)[1].strip()
        elif line.startswith("dev-type:"):
            info["dev_type"] = line.split(":", 1)[1].strip()

    if "dev_id" in info:
        entry = DEVICE_IDS.get(info["dev_id"])
        if entry:
            info["part"], info["density"], info["sram_table_kb"] = entry
        else:
            info["part"] = f"Unknown (DevID=0x{info['dev_id']:04X})"
            info["density"] = "unknown"
    return info


def detect_via_openocd():
    """Try openocd as a fallback. Returns dict or None."""
    # Quick probe — openocd exits 1 even on success, so we parse stdout
    out, _ = _run(
        ["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg",
         "-c", "init", "-c", "exit"],
        timeout=8,
    )
    if out is None:
        return None     # openocd not installed
    info = {"raw": out, "source": "openocd"}
    for line in out.splitlines():
        if "stm32f1x" in line.lower() or "STM32" in line:
            info["openocd_line"] = line.strip()
            break
    if "Error" in out and "stlink" in out.lower():
        return {"error": "openocd found no ST-Link", "raw": out}
    return info


def detect_via_lsusb():
    """Detect ST-Link by USB VID:PID without chip details. Returns dict or None."""
    out, rc = _run(["lsusb"])
    if out is None:
        return None
    found = []
    for line in out.splitlines():
        for pid, name in STLINK_PIDS.items():
            if f"0483:{pid:04x}" in line.lower() or f"0483:{pid:04X}" in line:
                found.append({"name": name, "lsusb_line": line.strip()})
    if not found:
        return {"error": "No ST-Link found via lsusb"}
    return {"source": "lsusb", "found": found}


def print_stlink_results(info):
    if info is None:
        print("  [ST-Link] No detection tool available (install stlink-tools or openocd)")
        return
    if "error" in info:
        print(f"  [ST-Link] {info['error']}")
        if info.get("raw"):
            for line in info["raw"].splitlines()[:4]:
                if line.strip():
                    print(f"    {line}")
        return

    src = info.get("source", "?")
    if src == "lsusb":
        for d in info.get("found", []):
            print(f"  [ST-Link] {d['name']} detected via USB")
        return

    if src == "st-info":
        print(f"  [ST-Link] {info.get('stlink_version', '?')}  serial: {info.get('stlink_serial','?')}")
        part    = info.get("part", info.get("stinfo_descr", "?"))
        dev_id  = info.get("dev_id", 0)
        flash_k = info.get("flash_kb", "?")
        sram_k  = info.get("sram_kb", "?")
        print(f"  [Chip]    {part}  DevID=0x{dev_id:04X}")
        print(f"            Flash={flash_k} KB  SRAM={sram_k} KB")
        if "density" in info:
            print(f"            Density: {info['density']}")
    elif src == "openocd":
        print(f"  [openocd] {info.get('openocd_line', 'detected')}")


# ---------------------------------------------------------------------------
# CAN STATUS collection
# ---------------------------------------------------------------------------

def _collect_can_status(bus, timeout=2.0):
    """Collect bootloader STATUS frames (0x07/0x17/0x27/0x37) from 0x7DB."""
    frames = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(frames) < 4:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if msg.arbitration_id == BL_CTRL_TX_ID and len(msg.data) >= 8:
            tag = msg.data[0]
            if tag in STATUS_FRAME_TAGS:
                frames[tag] = bytes(msg.data)
        # Also accept hello (0x7DE) — contains uid0
        if msg.arbitration_id == BL_HELLO_ID and len(msg.data) >= 6:
            frames["hello"] = bytes(msg.data)
    return frames


def _send_status_query(bus, uid0_auth):
    """Send BL_STATUS (0x07) query on 0x7DC."""
    import can
    frame = bytes([0x07]) + uid0_auth + b"\x00\x00\x00\x00"
    bus.send(can.Message(arbitration_id=BL_CTRL_RX_ID, data=frame, is_extended_id=False))


def print_can_status(frames, query_sent=False):
    """Print the decoded CAN STATUS banner."""
    if not frames:
        print("  [CAN]     No STATUS frames received" +
              (" (board may not be in bootloader mode)" if not query_sent else ""))
        return

    f0 = frames.get(0x07)
    f1 = frames.get(0x17)
    f2 = frames.get(0x27)
    f3 = frames.get(0x37)
    hello = frames.get("hello")

    uid0 = struct.unpack_from("<I", f0, 4)[0] if f0 else (
           struct.unpack_from("<I", hello, 2)[0] if hello else 0)
    uid1 = struct.unpack_from("<I", f1, 2)[0] if f1 else 0
    uid2_lo = struct.unpack_from("<H", f1, 6)[0] if f1 else 0
    uid2_hi = struct.unpack_from("<H", f2, 2)[0] if f2 else 0
    uid2 = (uid2_hi << 16) | uid2_lo
    boot_slot = f0[2] if f0 else 0
    ver_maj   = f2[4] if f2 else 1
    ver_min   = f2[5] if f2 else 0
    slot_char = "B" if boot_slot == 1 else "A"
    slot_addr = 0x08020000 if boot_slot == 1 else 0x08002000

    print()
    print("  ===================================")
    print(f"    biPropellant CAN Generic BL v{ver_maj}.{ver_min}")
    if f3:
        dev_id   = struct.unpack_from("<H", f3, 2)[0]
        rev_id   = struct.unpack_from("<H", f3, 4)[0]
        flash_kb = struct.unpack_from("<H", f3, 6)[0]
        entry    = DEVICE_IDS.get(dev_id)
        part     = entry[0] if entry else f"Unknown (0x{dev_id:04X})"
        print(f"    Chip:  {part}")
        print(f"           DevID=0x{dev_id:04X}  Rev=0x{rev_id:04X}  Flash={flash_kb} KB")
    print(f"    UID:   {uid0:08X}-{uid1:08X}-{uid2:08X}")
    print(f"    Active slot: {slot_char} (0x{slot_addr:08X})")
    print(f"    Next boot:   {slot_char}")
    print("  ===================================")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Detect ST-Link + connected STM32/GD32 chip",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 detect_chip.py                    # ST-Link probe only
  python3 detect_chip.py -d can0            # + listen for CAN bootloader STATUS
  python3 detect_chip.py -d can0 --query    # + actively send a STATUS query
  sudo apt install stlink-tools             # install st-info for full chip info
""")
    parser.add_argument("-d", "--device", default=None,
                        help="SocketCAN interface for CAN STATUS (e.g. can0); optional")
    parser.add_argument("--query", action="store_true",
                        help="Send a STATUS query to the bootloader (requires -d)")
    args = parser.parse_args()

    print("=== ST-Link / Chip Detection ===\n")

    # --- ST-Link probe ---
    print("[1] Probing ST-Link via st-info ...")
    info = detect_via_stinfo()
    if info is not None:
        print_stlink_results(info)
    else:
        print("    st-info not found — trying openocd ...")
        info = detect_via_openocd()
        if info is not None:
            print_stlink_results(info)
        else:
            print("    openocd not found — falling back to lsusb ...")
            info = detect_via_lsusb()
            print_stlink_results(info)

    # --- CAN STATUS ---
    if args.device:
        print(f"\n[2] Listening for CAN bootloader STATUS on {args.device} ...")
        try:
            import can
        except ImportError:
            print("    python-can not installed.  Run: pip install python-can")
            sys.exit(0)

        try:
            bus = can.interface.Bus(channel=args.device, bustype="socketcan")
        except Exception as exc:
            print(f"    Cannot open {args.device}: {exc}")
            sys.exit(1)

        try:
            # If --query, wait for hello first to get uid0_auth, then query
            uid0_auth = b"\x00\x00\x00"
            if args.query:
                print("    Waiting for BL hello (power-cycle or send SDO reboot) ...")
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    msg = bus.recv(timeout=0.1)
                    if msg and msg.arbitration_id == BL_HELLO_ID and len(msg.data) >= 5:
                        uid0_auth = bytes(msg.data[2:5])
                        print(f"    BL hello received, sending STATUS query ...")
                        _send_status_query(bus, uid0_auth)
                        break

            frames = _collect_can_status(bus, timeout=3.0)
            print_can_status(frames, query_sent=args.query)
        finally:
            bus.shutdown()

    print()


if __name__ == "__main__":
    main()
