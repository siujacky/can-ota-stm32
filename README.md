# can-ota-stm32

**STM32F103 CAN OTA Bootloader** — Firmware update over CAN bus using the STM32's native bxCAN peripheral.

Proven in a 3-node CAN bus: **Pico 2 (XL2515)** ↔ **MCP2515 module** ↔ **Blue Pill (bxCAN PB8/PB9 + TJA1050)**

## Quick Start

```bash
# Flash bootloader
st-flash write bootloader/bootloader.bin 0x08000000

# OTA update (automatic: sends reboot trigger to app, waits for hello)
python3 tools/can_ota_bluepill.py --firmware app/blue_pill_can_tool.bin

# Or manual (--no-reboot: manually reset Blue Pill within 500ms)
python3 tools/can_ota_bluepill.py --firmware app/blue_pill_can_tool.bin --no-reboot
```

## Hardware

### Blue Pill (STM32F103C6T6)
- **bxCAN on PB8 (CAN_RX) / PB9 (CAN_TX)** — native STM32 CAN peripheral
- TJA1050 CAN transceiver powered from **3.3V** (same as STM32 logic)
- SWD programming via PA13/PA14

### Pico 2 (Waveshare RP2350-CAN)
- XL2515 (MCP2515-compatible) — runs `pico2_slcan` firmware
- Acts as USB↔CAN bridge for the host PC
- USB CDC at 115200 baud, SLCAN protocol

### Wiring
See [docs/WIRING.md](docs/WIRING.md) for full diagram.

## Architecture

```
Host PC
  │ USB CDC (SLCAN)
  │
Pico 2 (XL2515)          250kbps CAN bus
  │                    ┌──────────────────────────┐
  ├────────────────────┤ CANH                     │
  │                    │      MCP2515 module       │
  ├────────────────────┤ CANL  (optional 3rd node) │
  │                    └──────────────────────────┘
  │
Blue Pill (bxCAN PB8/PB9 + TJA1050)
  ├── Bootloader @ 0x08000000  (8KB)
  └── App Slot A  @ 0x08002000 (120KB)
```

## OTA Protocol

| Step | Direction | CAN ID | Content |
|------|-----------|--------|---------|
| 1 | Blue Pill → Host | 0x7DE | Hello: `['3','1', UID0_LE, ver]` every 50ms |
| 2 | Host → Blue Pill | 0x7DD | UID0 trigger (4 bytes LE) |
| 3 | Blue Pill → Host | 0x7DE | `'S'` (ready) |
| 4 | Host → Blue Pill | 0x7DD | Page count (4 bytes LE) |
| 5 | Host → Blue Pill | 0x7DD | 128 frames × 8 bytes = 1024 bytes/page |
| 6 | Host → Blue Pill | 0x7DD | CRC32 frame (4 bytes + 4 padding) |
| 7 | Blue Pill → Host | 0x7DE | `'P'` (page OK) or `'E'` (CRC error) |
| 8 | Blue Pill → Host | 0x7DE | `'D'` (all done, rebooting) |

**Remote reboot**: send ID=0x7FF, data=[0xB0, 0x01, 0xB2] to running app → app writes BKP_DR1, resets to bootloader with 30s OTA window.

## One-Shot vs Standard CAN Mode

| Mode | `bxcan_init(osm=)` | Behaviour | When to use |
|------|--------------------|-----------|-------------|
| **Standard** | `osm=0` | Auto-retry on no-ACK | Normal app operation |
| **One-Shot** | `osm=1` | Single attempt, ABTF on no-ACK | OTA bootloader |

The bootloader uses One-Shot Mode to prevent bus flooding. The app uses Standard Mode for reliable communication.

## Building

```bash
# Bootloader (requires arm-none-eabi-gcc)
cd bootloader && make

# App
cd app && make

# Pico 2 SLCAN (requires Pico SDK)
cd pico2_slcan && cmake -B build -DPICO_BOARD=pico2 && cmake --build build
```

## Key Lessons Learned

See [docs/LESSONS_LEARNED.md](docs/LESSONS_LEARNED.md) for detailed writeup of 31 bugs found, MCP2515 vs bxCAN comparison, and hardware gotchas.
