/* usart_bl.h — USART1 bootloader interface (PA9=TX, PA10=RX).
 *
 * Pins chosen because they do NOT conflict with MCP2515 software-SPI:
 *   MCP2515 uses PA2/PA3/PB10/PB11.  USART1 uses PA9/PA10.
 * PA9/PA10 are phase-PWM in the application but unused in the bootloader.
 *
 * Baud: 115200 @ 8 MHz HSI.  BRR = (4<<4)|5 = 0x0045.
 *
 * Protocol: compatible with jsphuebner/stm32-CANBootloader uart-updater.py.
 *   Trigger: 0xAA byte → enter programming mode.
 *   Then: same page protocol as CAN (page count, pages, CRC32, P/E/D).
 */

#ifndef USART_BL_H
#define USART_BL_H

#include <stdint.h>

/* Full-duplex init: PA9=TX (AF PP), PA10=RX (floating input). Default. */
void usart_init(void);

/* -----------------------------------------------------------------------
 * Single-Wire Half-Duplex (HDSEL) mode.
 *
 * STM32 USART HDSEL (CR3 bit 3) ties the internal TX and RX paths together:
 *   - TX and RX share a single external wire on PA9.
 *   - PA10 is released (left floating; you can use it for anything else).
 *   - PA9 is configured as AF open-drain so multiple devices can share the
 *     bus without electrical contention (add a 4k7 pull-up to VCC).
 *   - Hardware automatically switches between drive (TX) and listen (RX).
 *   - Every byte you transmit is also echoed back to the RX shift register,
 *     so usart_rx_buf() after usart_tx_byte() returns that same byte.
 *     This loopback lets you verify exactly what was put on the wire.
 *
 * Auto-detect: usart_init_autodetect() tries HDSEL first; if no echo comes
 * back within 2 ms it falls back to full-duplex automatically.
 * ----------------------------------------------------------------------- */

/* Single-wire half-duplex init: PA9 AF open-drain, HDSEL set, PA10 unused. */
void usart_init_hdsel(void);

/* Try HDSEL first; if echo check fails, fall back to full-duplex.
 * Returns 1 if HDSEL mode is active, 0 if full-duplex is active. */
int usart_init_autodetect(void);

/* Transmit buf[len] and read back the HDSEL echo; compare byte-by-byte.
 * Returns 0 if all echoes match (wire is clean), or the count of mismatches.
 * timeout_ms applies per byte (recommend 5 for 115200 baud).
 * Only meaningful in HDSEL mode; in full-duplex the caller's RX data is read. */
int usart_tx_verify(const uint8_t *buf, uint8_t len, uint32_t timeout_ms);

/* Returns 1 if a byte is waiting in RDR, 0 otherwise (non-blocking). */
int usart_rx_ready(void);

/* Block until a byte is available, then return it. */
uint8_t usart_rx_byte(void);

/* Transmit a single byte (blocks until TXE). */
void usart_tx_byte(uint8_t b);

/* Transmit a buffer. */
void usart_tx(const uint8_t *buf, uint8_t len);

/* Receive exactly len bytes; returns 0 on timeout (timeout_ms each byte). */
int usart_rx_buf(uint8_t *buf, uint32_t len, uint32_t timeout_ms);

/* Send a null-terminated string. */
void usart_print(const char *s);

/* Send 8 uppercase hex characters representing a 32-bit value (e.g. "08002000"). */
void usart_print_hex32(uint32_t v);

/* Send decimal representation of a 32-bit value as ASCII (no malloc). */
void usart_print_uint32(uint32_t v);

#endif /* USART_BL_H */
