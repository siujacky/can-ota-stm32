# can-ota-stm32

**STM32F103 CAN OTA Bootloader** — Firmware update over CAN bus using the STM32's native bxCAN peripheral.

Fully verified on a 3-node CAN bus at 250kbps:

| Node | Hardware | Role |
|------|----------|------|
| Pico 2 | Waveshare RP2350-CAN (XL2515) | USB↔CAN bridge (SLCAN) |
| Blue Pill | STM32F103C6T6 + TJA1050 | bxCAN target — bootloader + app |
| MCP2515 | SPI module @ 3.3V + TJA1050 | Optional 3rd node (SPI on Blue Pill PA4-7) |

**Test results — all paths 100%:**

```
Pico2 TX → bxCAN RX (echo)   100% ✓
bxCAN TX → Pico2 RX           100% ✓
MCP2515 TX → Pico2 RX         100% ✓
Overflow R0OVR auto-clear     CLEAN ✓
OTA firmware update            SUCCESS ✓
```

## Quick Start

```bash
# 1. Stop slcand if running (it holds ttyACM1)
killall slcand; ip link set slcan0 down

# 2. Flash bootloader via SWD (once)
st-flash write bootloader/bootloader.bin 0x08000000

# 3. OTA update — automatic (sends reboot trigger to running app)
python3 tools/can_ota_bluepill.py --firmware app/blue_pill_can_tool.bin

# 4. OTA update — manual (reset Blue Pill yourself within 500ms)
python3 tools/can_ota_bluepill.py --firmware app/blue_pill_can_tool.bin --no-reboot

# 5. Full 3-node verification test (stops/restarts slcand automatically)
python3 tools/can_test_3node.py

# 6. Restore slcand for ROS2
slcand -o -s4 -t hw -S 115200 /dev/ttyACM1 slcan0
ip link set slcan0 up
```

## Hardware

### Blue Pill (STM32F103C6T6)
- **bxCAN on PB8 (CAN_RX) / PB9 (CAN_TX)** with AFIO remap (bits[14:13]=10)
- TJA1050 CAN transceiver powered at **3.3V** (same rail as STM32)
- Optional: MCP2515 on SPI1 (PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI, PB0=INT) at 3.3V VCC

### Pico 2 (Waveshare RP2350-CAN)
- XL2515 (MCP2515-compatible, 16MHz crystal) running `pico2_slcan` firmware
- USB CDC at 115200 baud, SLCAN protocol (`S4`=250kbps)
- Appears as `/dev/ttyACM1` when plugged in

### MCP2515 Module (3rd node, optional)
- **Power MCP2515 at 3.3V VCC** — at 5V the SPI VIH_min=3.5V > STM32 output (3.3V) → silent failure
- Confirmed bitrate: CNF1=0x00, CNF2=0x9E, CNF3=0x03 → 250kbps @ 8MHz crystal

### Wiring
See [docs/WIRING.md](docs/WIRING.md) for full pinout diagram.

## Architecture

```
Host PC (Raspberry Pi)
  │ USB CDC (SLCAN, 115200 baud)
  │
Pico 2 (XL2515, 16MHz crystal)   ──── 250kbps CAN bus ────
  │                                                         │
  │                                              Blue Pill  │
  │                                           STM32F103C6T6 │
  │                                    PB8 ← CAN_RX         │
  │                                    PB9 → CAN_TX         │
  │                                    TJA1050 (3.3V)       │
  │                                              │           │
  └──────────────────────────────────────────────┘           │
                                                             │
                                       Optional: MCP2515 ───┘
                                       (Blue Pill SPI, 3.3V)
```

## OTA Protocol

| Step | Direction | CAN ID | Content |
|------|-----------|--------|---------|
| 1 | Blue Pill → Host | `0x7DE` | Hello: `['3','1', UID0_LE(4B), ver_major, ver_minor]` every 50ms |
| 2 | Host → Blue Pill | `0x7DD` | UID0 (4 bytes LE) — identifies target device |
| 3 | Blue Pill → Host | `0x7DE` | `'S'` (start, ready to receive) |
| 4 | Host → Blue Pill | `0x7DD` | Page count (4 bytes LE) |
| 5 | Host → Blue Pill | `0x7DD` | 128 frames × 8 bytes = **1024 bytes** per page |
| 6 | Host → Blue Pill | `0x7DD` | CRC32 frame (4 bytes LE + 4 padding) |
| 7 | Blue Pill → Host | `0x7DE` | `'P'` (page OK) or `'E'` (CRC error, retry) |
| 8 | Blue Pill → Host | `0x7DE` | `'D'` (all pages done, rebooting) |

**Remote reboot** (app running): send ID=`0x7FF`, data=`[0xB0, 0x01, 0xB2]` → app writes `BKP_DR1=0xB001`, calls `NVIC_SystemReset()` → bootloader sees magic, opens 30s OTA window.

**Normal boot** (BKP=0): bootloader opens 500ms window then jumps to app.

## CAN Mode Reference

| Mode | `bxcan_init(osm)` | `CAN_MCR.NART` | Behaviour | When to use |
|------|--------------------|----------------|-----------|-------------|
| Standard | `osm=0` | 0 (cleared) | Auto-retry on no-ACK | App — reliable delivery |
| One-Shot | `osm=1` | 1 (set) | Single attempt only | Bootloader — host controls retries |

## Building

```bash
# Bootloader (requires arm-none-eabi-gcc ≥ 10)
cd bootloader && make -f Makefile.bl

# App (Blue Pill CAN tool)
cd app && make

# Pico 2 SLCAN bridge (requires Pico SDK 2.x)
cd pico2_slcan && cmake -B build -DPICO_BOARD=pico2 && cmake --build build
# Flash: hold BOOTSEL, plug USB, then:
cp build/pico2_slcan.uf2 /media/$USER/RP2350/
```

## Verified Bitrates (MCP2515, 8MHz crystal)

| SLCAN Cmd | Rate | CNF1 | CNF2 | CNF3 | SP% |
|-----------|------|------|------|------|-----|
| `S0` | 10k | 0x18 | 0x9E | 0x03 | 81% |
| `S1` | 20k | 0x0B | 0x9E | 0x03 | 81% |
| `S2` | 50k | 0x04 | 0x9E | 0x03 | 81% |
| `S3` | 100k | 0x01 | 0xAD | 0x02 | 81% |
| `S4` | 125k | 0x01 | 0x9E | 0x03 | 81% |
| `S5` | **250k** | 0x00 | 0x9E | 0x03 | 81% | ← **confirmed ✓** |
| `S6` | 500k | 0x00 | 0x90 | 0x02 | 75% |
| `S7` | 800k | 0x00 | 0x88 | 0x00 | 75% |
| `S8` | 1M | 0x00 | 0x80 | 0x00 | 75% |

## Critical Gotchas

1. **Kill `slcand` before any direct ttyACMx access** — it holds the port exclusively
2. **MCP2515 at 3.3V VCC** — at 5V the SPI bus is unreliable (VIH_min > STM32 output)
3. **TXREQ = bit 2 = 0x04** in MCP2515 TXBnCTRL — many libs wrongly use 0x08 (that's TXERR)
4. **Clear BKP_DR1 before every automated reset** — or get 30s bootloader window instead of 500ms
5. **2ms inter-frame gap** for OTA page upload — bxCAN FIFO depth is only 3 frames

See [docs/LESSONS_LEARNED.md](docs/LESSONS_LEARNED.md) for full writeup of all 20 bugs found.
