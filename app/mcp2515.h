/* mcp2515.h — MCP2515 CAN controller driver header.
 * Hardware SPI1, no HAL.
 * Pins: PA4=CS  PA5=SCK  PA6=MISO  PA7=MOSI  PB0=INT
 * Crystal: 8 MHz
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>

/* MCP2515 SPI commands */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BIT_MODIFY  0x05
#define MCP_RTS_TXB0    0x81   /* Request-to-send TXB0 */
#define MCP_RTS_TXB1    0x82
#define MCP_RTS_TXB2    0x84
#define MCP_READ_RX0    0x90   /* Read RX buffer 0 starting at RXB0SIDH */
#define MCP_READ_RX1    0x94
#define MCP_READ_STATUS 0xA0

/* MCP2515 register addresses */
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
/* Corrected addresses from MCP2515 datasheet:
 * TEC=0x1C, REC=0x1D, CNF3=0x28, CNF2=0x29, CNF1=0x2A
 * CANINTE=0x2B, CANINTF=0x2C, EFLG=0x2D
 * TXB0CTRL=0x30, TXB0SIDH=0x31 ... TXB0D7=0x3D
 * RXB0CTRL=0x60, RXB0SIDH=0x61 ... RXB0D7=0x6D
 * RXB1CTRL=0x70, RXB1SIDH=0x71 ... RXB1D7=0x7D
 */
#define MCP_TEC         0x1C
#define MCP_REC         0x1D
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C
#define MCP_EFLG_REG    0x2D

#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0SIDL    0x32
#define MCP_TXB0EID8    0x33
#define MCP_TXB0EID0    0x34
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36

#define MCP_TXB1CTRL    0x40
#define MCP_TXB1SIDH    0x41

#define MCP_TXB2CTRL    0x50
#define MCP_TXB2SIDH    0x51

#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0SIDL    0x62
#define MCP_RXB0EID8    0x63
#define MCP_RXB0EID0    0x64
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66

#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71

/* CANCTRL modes */
#define MCP_MODE_NORMAL      0x00
#define MCP_MODE_SLEEP       0x20
#define MCP_MODE_LOOPBACK    0x40
#define MCP_MODE_LISTEN_ONLY 0x60
#define MCP_MODE_CONFIG      0x80
#define MCP_MODE_MASK        0xE0

/* CANINTF bits */
#define MCP_CANINTF_RX0IF   (1U << 0)
#define MCP_CANINTF_RX1IF   (1U << 1)
#define MCP_CANINTF_TX0IF   (1U << 2)
#define MCP_CANINTF_TX1IF   (1U << 3)
#define MCP_CANINTF_TX2IF   (1U << 4)
#define MCP_CANINTF_ERRIF   (1U << 5)
#define MCP_CANINTF_WAKIF   (1U << 6)
#define MCP_CANINTF_MERRF   (1U << 7)

/* RXBnSIDL bits */
#define MCP_RXSIDL_IDE      (1U << 3)  /* Extended frame indicator */
#define MCP_RXSIDL_SRR      (1U << 4)  /* Standard RTR */

/* TXBnSIDL bits */
#define MCP_TXSIDL_EXIDE    (1U << 3)  /* Extended frame enable */

/* TXBnCTRL bits */
#define MCP_TXCTRL_TXREQ    (1U << 2)  /* TX request  — DS21801J bit 2 */
#define MCP_TXCTRL_ABTF     (1U << 5)  /* Aborted flag — DS21801J bit 5 */
#define MCP_TXCTRL_MLOA     (1U << 4)  /* Lost arbitration — DS21801J bit 4 */
#define MCP_TXCTRL_TXERR    (1U << 3)  /* TX error — DS21801J bit 3 */

/* TXBnDLC bits */
#define MCP_DLC_RTR         (1U << 6)  /* Remote Transmission Request */

/* Bitrate S-command values */
#define MCP_BRATE_125K   4
#define MCP_BRATE_250K   5
#define MCP_BRATE_500K   6   /* default */
#define MCP_BRATE_800K   7
#define MCP_BRATE_1M     8

/* Public API */
void    mcp2515_init(uint8_t brate);          /* brate = MCP_BRATE_xxx */
void    mcp2515_set_bitrate(uint8_t s_cmd);   /* s_cmd = '4'..'8' digit char or 4..8 int */
int     mcp2515_tx(uint32_t id, const uint8_t *data, uint8_t len, int extended);
int     mcp2515_tx_rtr(uint32_t id, uint8_t dlc, int extended);
int     mcp2515_rx_available(void);
int     mcp2515_rx(uint32_t *id_out, uint8_t *data_out, uint8_t *len_out, int *ext_out);
uint8_t mcp2515_read_errors(void);            /* reads+clears CANINTF error bits, returns EFLG */
void    mcp2515_release_pins(void);           /* set all SPI pins to INPUT_FLOAT */
void    mcp2515_restore_pins(void);           /* reconfigure SPI pins as outputs */
int     mcp2515_enter_listen_only(void);      /* C command — listen-only mode */
int     mcp2515_enter_normal(void);           /* O command — normal mode */

#endif /* MCP2515_H */
