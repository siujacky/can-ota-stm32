# Wiring Guide

## Blue Pill + TJA1050 CAN Transceiver

```
Blue Pill (STM32F103C6T6)
┌──────────────┐
│          PB8 ├──── TJA1050 RxD  (CAN receive)
│          PB9 ├──── TJA1050 TxD  (CAN transmit)
│         3.3V ├──── TJA1050 VCC  ← MUST be 3.3V (not 5V!)
│          GND ├──── TJA1050 GND
│   NSTB/RS=0  │     TJA1050 NSTB → GND (normal mode, not standby)
└──────────────┘

TJA1050 CAN bus:
├── CANH ─────────── CAN bus H
└── CANL ─────────── CAN bus L
```

**Critical:** Power TJA1050 from **3.3V**, not 5V. At 5V, VIH_min=3.5V > STM32 3.3V output → TxD signal unreliable → no CAN TX.

## Pico 2 (Waveshare RP2350-CAN) — Already integrated

The Waveshare RP2350-CAN board has the XL2515 (MCP2515-compatible) built in at 3.3V:
- CAN_H and CAN_L headers on the board
- USB to Raspberry Pi for SLCAN bridge

## 3-Node CAN Bus Topology

```
┌──────────────────────────────────────────────────────┐
│                     CAN Bus (250kbps)                │
│                                                      │
│  ┌──────────┐   CANH  ──────────────────────────────┤
│  │ Blue Pill │                                       │
│  │ bxCAN    ├─ CANL  ──────────────────────────────┤
│  │ PB8/PB9  │                                       │
│  └──────────┘          ┌──────────┐  ┌──────────┐  │
│                        │  MCP2515 ├──┤ Pico 2   │  │
│                        │  module  │  │ XL2515   │  │
│                        └──────────┘  └──────────┘  │
│                                           │          │
│                                      USB CDC         │
│                                      (SLCAN)         │
│                                      Raspberry Pi    │
└──────────────────────────────────────────────────────┘

120Ω termination: one at each end of the bus.
Short bus (<30cm): single 120Ω is usually sufficient.
```

## AFIO Remap Configuration

The STM32F103 CAN has two possible mappings:
| Mapping | CAN_RX | CAN_TX | AFIO_MAPR[14:13] |
|---------|--------|--------|-----------------|
| Default | PA11 | PA12 | 00 |
| Remap 1 | **PB8** | **PB9** | **10** ← use this |
| Remap 2 | PD0 | PD1 | 11 |

```c
// Enable AFIO and GPIOB clocks, then remap CAN to PB8/PB9:
RCC->APB2ENR |= (1<<0) | (1<<3);  // AFIO + GPIOB
AFIO->MAPR    = (AFIO->MAPR & ~(3U<<13)) | (2U<<13);  // bits[14:13] = 10
```

Note: PB8/PB9 avoids conflict with USB (PA11/PA12 are used by USB on Blue Pill).

## MCP2515 Module at 3.3V

When using an MCP2515 module (e.g., blue Arduino module):
```
STM32F103                MCP2515 module
──────────               ──────────────
PA4 (CS)   ─────────── CS
PA5 (SCK)  ─────────── SCK   ← SPI
PA6 (MISO) ─────────── MISO
PA7 (MOSI) ─────────── MOSI
PB0        ─────────── INT
3.3V       ─────────── VCC   ← CRITICAL: 3.3V not 5V!
GND        ─────────── GND
```

The TJA1050 on the module is spec'd for 5V but works at 3.3V for short cables.
CANH/CANL connect to the CAN bus.
