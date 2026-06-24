/* slcan.h — SLCAN protocol parser and formatter.
 * Implements the SLCAN subset for CAN sniffer + injector.
 */

#ifndef SLCAN_H
#define SLCAN_H

#include <stdint.h>

/* Command result codes */
typedef enum {
    SLCAN_OPEN       = 0,  /* O — open CAN bus */
    SLCAN_CLOSE      = 1,  /* C — close CAN bus */
    SLCAN_SETBAUD    = 2,  /* S<n> — set bitrate */
    SLCAN_TX         = 3,  /* t/T — transmit data frame */
    SLCAN_RTR        = 4,  /* r — transmit RTR frame */
    SLCAN_STATUS     = 5,  /* s or i — print status */
    SLCAN_FLAGS      = 6,  /* F — read error flags */
    SLCAN_HELP       = 7,  /* ? or h */
    SLCAN_MODE_UART  = 8,  /* mode uart <baud> */
    SLCAN_MODE_CAN   = 9,  /* mode can */
    SLCAN_UNKNOWN    = 10, /* invalid / parse error */
} slcan_cmd_t;

/* Parsed TX frame info */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    int      extended;   /* 1 = 29-bit extended, 0 = 11-bit standard */
    int      rtr;        /* 1 = RTR frame */
    uint8_t  baud_s;     /* for SLCAN_SETBAUD: 4-8 */
    uint32_t uart_baud;  /* for SLCAN_MODE_UART: baud rate number */
} slcan_frame_t;

/* Format a received CAN frame into SLCAN output line.
 * out must be at least 30 bytes.
 * Returns number of chars written (not including NUL). */
int slcan_format_rx(uint32_t id, int ext, uint8_t dlc,
                    const uint8_t *data, char *out);

/* Parse a null-terminated command line (without trailing \r\n).
 * Fills frame_out with parsed parameters.
 * Returns one of the slcan_cmd_t values. */
slcan_cmd_t slcan_parse_cmd(const char *cmd, slcan_frame_t *frame_out);

/* Hex helpers */
char    nibble_to_hex(uint8_t n);
int     parse_hex_digit(char c);
int32_t parse_hex_u32(const char *s, int ndigits, uint32_t *out);

#endif /* SLCAN_H */
