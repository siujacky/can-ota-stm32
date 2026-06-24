/* slcan.c — SLCAN protocol parser and formatter.
 * Stateless parsing; no heap allocation; no libc.
 */

#include "slcan.h"

/* -----------------------------------------------------------------------
 * Hex helpers
 * ----------------------------------------------------------------------- */
char nibble_to_hex(uint8_t n)
{
    n &= 0x0F;
    return (n < 10) ? (char)('0' + n) : (char)('A' + n - 10);
}

int parse_hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Parse exactly ndigits hex digits from s into *out.
 * Returns 0 on success, -1 on error. */
int32_t parse_hex_u32(const char *s, int ndigits, uint32_t *out)
{
    uint32_t v = 0;
    for (int i = 0; i < ndigits; i++) {
        int d = parse_hex_digit(s[i]);
        if (d < 0) return -1;
        v = (v << 4) | (uint32_t)d;
    }
    *out = v;
    return 0;
}

/* Parse a decimal number (used for baud rate in 'mode uart <baud>').
 * Stops at non-digit or '\0'. Returns 0 if no digits found. */
static uint32_t parse_decimal(const char *s)
{
    uint32_t v = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (uint32_t)(*s - '0');
        s++;
    }
    return v;
}

/* Simple string compare (NUL-terminated, case-sensitive, up to n chars) */
static int str_eq_n(const char *a, const char *b, int n)
{
    for (int i = 0; i < n; i++) {
        if (a[i] != b[i]) return 0;
        if (a[i] == '\0') return 1;
    }
    return 1;
}

/* -----------------------------------------------------------------------
 * slcan_format_rx — build a SLCAN output line for a received frame.
 *
 * Standard: t<ID3><DLC1><DATA...>\r\n
 * Extended: T<ID8><DLC1><DATA...>\r\n
 *
 * out buffer must be >= 30 bytes.
 * Returns length written (including \r\n, not including NUL).
 * ----------------------------------------------------------------------- */
int slcan_format_rx(uint32_t id, int ext, uint8_t dlc,
                    const uint8_t *data, char *out)
{
    int pos = 0;

    if (ext) {
        out[pos++] = 'T';
        /* 8 hex digits for 29-bit ID (padded to 32-bit) */
        out[pos++] = nibble_to_hex((uint8_t)(id >> 28));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 24));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 20));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 16));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 12));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 8));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 4));
        out[pos++] = nibble_to_hex((uint8_t)(id));
    } else {
        out[pos++] = 't';
        /* 3 hex digits for 11-bit ID */
        out[pos++] = nibble_to_hex((uint8_t)(id >> 8));
        out[pos++] = nibble_to_hex((uint8_t)(id >> 4));
        out[pos++] = nibble_to_hex((uint8_t)(id));
    }

    /* DLC: 1 hex digit */
    if (dlc > 8) dlc = 8;
    out[pos++] = (char)('0' + dlc);

    /* Data bytes: 2 hex digits each */
    for (uint8_t i = 0; i < dlc; i++) {
        out[pos++] = nibble_to_hex(data[i] >> 4);
        out[pos++] = nibble_to_hex(data[i]);
    }

    out[pos++] = '\r';
    out[pos++] = '\n';
    out[pos]   = '\0';

    return pos;
}

/* -----------------------------------------------------------------------
 * slcan_parse_cmd — parse a command line.
 *
 * cmd: null-terminated string without trailing \r\n
 * frame_out: filled with parsed frame info (may be NULL if not needed)
 *
 * Commands (first char case-insensitive):
 *   O          → SLCAN_OPEN
 *   C          → SLCAN_CLOSE
 *   S<n>       → SLCAN_SETBAUD  (n=4..8)
 *   t<id3><dlc1><data>  → SLCAN_TX  standard
 *   T<id8><dlc1><data>  → SLCAN_TX  extended
 *   r<id3><dlc1>        → SLCAN_RTR standard
 *   F          → SLCAN_FLAGS
 *   s or i     → SLCAN_STATUS
 *   h or ?     → SLCAN_HELP
 *   mode uart <baud>    → SLCAN_MODE_UART
 *   mode can            → SLCAN_MODE_CAN
 * ----------------------------------------------------------------------- */
slcan_cmd_t slcan_parse_cmd(const char *cmd, slcan_frame_t *frame_out)
{
    if (!cmd || !cmd[0]) return SLCAN_UNKNOWN;

    /* Initialise frame_out */
    if (frame_out) {
        frame_out->id        = 0;
        frame_out->dlc       = 0;
        frame_out->extended  = 0;
        frame_out->rtr       = 0;
        frame_out->baud_s    = 6;
        frame_out->uart_baud = 115200;
        for (int i = 0; i < 8; i++) frame_out->data[i] = 0;
    }

    /* First char (force lowercase for comparison) */
    char c0 = cmd[0];
    if (c0 >= 'A' && c0 <= 'Z') c0 = (char)(c0 + 32);

    switch (c0) {
    case 'o':
        return SLCAN_OPEN;

    case 'c':
        return SLCAN_CLOSE;

    case 's':
        /* 's' alone = status; 's' followed by digit = set baud? No — 'S' alone = status.
         * 'S' followed by digit = set baud.
         * 's' = status (lowercase s is status per spec extension). */
        if (cmd[1] >= '4' && cmd[1] <= '8') {
            /* S<n> = set bitrate */
            if (frame_out) frame_out->baud_s = (uint8_t)(cmd[1] - '0');
            return SLCAN_SETBAUD;
        }
        return SLCAN_STATUS;

    case 'i':
        return SLCAN_STATUS;

    case 'f':
        return SLCAN_FLAGS;

    case '?':
    case 'h':
        return SLCAN_HELP;

    case 't':
    case 'T': {
        /* t<id3><dlc><data>  or  T<id8><dlc><data> */
        int ext = (c0 == 't') ? 0 : 1;
        int id_digits = ext ? 8 : 3;
        const char *p = cmd + 1;

        if (!frame_out) return SLCAN_TX;

        uint32_t id = 0;
        if (parse_hex_u32(p, id_digits, &id) < 0) return SLCAN_UNKNOWN;
        p += id_digits;

        if (*p < '0' || *p > '8') return SLCAN_UNKNOWN;
        uint8_t dlc = (uint8_t)(*p - '0');
        p++;

        for (uint8_t i = 0; i < dlc; i++) {
            uint32_t byte_val = 0;
            if (parse_hex_u32(p, 2, &byte_val) < 0) return SLCAN_UNKNOWN;
            frame_out->data[i] = (uint8_t)byte_val;
            p += 2;
        }

        frame_out->id       = id;
        frame_out->dlc      = dlc;
        frame_out->extended = ext;
        frame_out->rtr      = 0;
        return SLCAN_TX;
    }

    case 'r': {
        /* r<id3><dlc>  — standard RTR frame */
        const char *p = cmd + 1;
        uint32_t id = 0;
        if (parse_hex_u32(p, 3, &id) < 0) return SLCAN_UNKNOWN;
        p += 3;
        if (*p < '0' || *p > '8') return SLCAN_UNKNOWN;
        uint8_t dlc = (uint8_t)(*p - '0');

        if (frame_out) {
            frame_out->id       = id;
            frame_out->dlc      = dlc;
            frame_out->extended = 0;
            frame_out->rtr      = 1;
        }
        return SLCAN_RTR;
    }

    case 'm': {
        /* "mode uart <baud>" or "mode can" */
        /* Check full command is "mode ..." */
        if (!str_eq_n(cmd, "mode", 4)) return SLCAN_UNKNOWN;
        const char *p = cmd + 4;
        /* Skip spaces */
        while (*p == ' ') p++;

        if (str_eq_n(p, "can", 3)) {
            return SLCAN_MODE_CAN;
        }
        if (str_eq_n(p, "uart", 4)) {
            p += 4;
            while (*p == ' ') p++;
            uint32_t baud = parse_decimal(p);
            if (baud == 0) baud = 115200;
            if (frame_out) frame_out->uart_baud = baud;
            return SLCAN_MODE_UART;
        }
        return SLCAN_UNKNOWN;
    }

    default:
        return SLCAN_UNKNOWN;
    }
}
