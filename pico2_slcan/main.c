// =============================================================================
// pico2_slcan/main.c — SLCAN USB-CAN bridge for Waveshare RP2350-CAN board.
//
// Turns the Pico 2 into a USB serial CAN adapter (CANtact/slcand compatible).
// Host uses:  python3 tools/can_flash.py --interface slcan --channel /dev/ttyACM0
//
// Hardware: Waveshare RP2350-CAN
//   XL2515 (MCP2515-compatible) on SPI1:
//     CS=9  SCK=10  MOSI=11  MISO=12  INT=8   Crystal=16MHz
//   USB: native RP2350 USB via pico_enable_stdio_usb (USB CDC)
//
// SLCAN protocol (CANtact subset):
//   Host -> device:
//     'Sn'          set bitrate  S3=100k  S4=125k  S5=250k  S6=500k
//     'O'           open bus (normal mode)
//     'C'           close bus (silent/listen mode)
//     't<III><D><DD..>'  send standard frame  (3 hex ID, 1 hex DLC, data)
//     'T<IIIIIIII><D><DD..>'  send extended frame
//     'V'           firmware version
//     'N'           serial number
//     'F'           read error/status flags
//   Device -> host:
//     't<III><D><DD..>\r'   received standard frame
//     'T<IIIIIIII><D><DD..>\r'   received extended frame
//     'z\r'   ACK for 't' transmit
//     'Z\r'   ACK for 'T' transmit
//     'V0101\r'   version response
//     'N0001\r'   serial response
//     'Fxx\r'     flags (xx = hex EFLG value)
// =============================================================================

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ---- Pin definitions (matches Waveshare RP2350-CAN confirmed wiring) --------
#define SPI_PORT        spi1
#define PIN_CS          9
#define PIN_SCK         10
#define PIN_MOSI        11
#define PIN_MISO        12
#define PIN_INT         8
#define PIN_LED         25      // standard Pico user LED
#define SPI_BAUD_HZ     10000000u   // 10 MHz

// ---- MCP2515/XL2515 SPI instructions ----------------------------------------
#define INSTR_RESET       0xC0
#define INSTR_READ        0x03
#define INSTR_WRITE       0x02
#define INSTR_BIT_MODIFY  0x05
#define INSTR_READ_STATUS 0xA0

// ---- MCP2515 registers -------------------------------------------------------
#define REG_CANCTRL  0x0F
#define REG_CANSTAT  0x0E
#define REG_CNF3     0x28
#define REG_CNF2     0x29
#define REG_CNF1     0x2A
#define REG_CANINTE  0x2B
#define REG_CANINTF  0x2C
#define REG_EFLG     0x2D
#define REG_TXB0CTRL 0x30
#define REG_TXB1CTRL 0x40
#define REG_TXB2CTRL 0x50
#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

// Buffer field offsets relative to CTRL register
#define OFF_SIDH 1
#define OFF_SIDL 2
#define OFF_EID8 3
#define OFF_EID0 4
#define OFF_DLC  5
#define OFF_D0   6

// Mode bits
#define MODE_NORMAL    0x00
#define MODE_SLEEP     0x20
#define MODE_LOOPBACK  0x40
#define MODE_LISTEN    0x60
#define MODE_CONFIG    0x80
#define MODE_MASK      0xE0

// Interrupt/flag bits
#define CANINTF_RX0    0x01
#define CANINTF_RX1    0x02
#define TXBCTRL_TXREQ  0x04  /* bit 2 per DS21801J */
#define DLC_EXIDE      0x08  // SIDL.EXIDE bit
#define DLC_RTR        0x40
#define SIDL_EXIDE     0x08

// =============================================================================
// Minimal inline MCP2515/XL2515 driver
// =============================================================================

static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

static void reg_write(uint8_t addr, uint8_t val) {
    uint8_t tx[3] = { INSTR_WRITE, addr, val };
    cs_low();
    spi_write_blocking(SPI_PORT, tx, 3);
    cs_high();
}

static uint8_t reg_read(uint8_t addr) {
    uint8_t tx[2] = { INSTR_READ, addr };
    uint8_t rx = 0;
    cs_low();
    spi_write_blocking(SPI_PORT, tx, 2);
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    cs_high();
    return rx;
}

static void reg_modify(uint8_t addr, uint8_t mask, uint8_t val) {
    uint8_t tx[4] = { INSTR_BIT_MODIFY, addr, mask, val };
    cs_low();
    spi_write_blocking(SPI_PORT, tx, 4);
    cs_high();
}

static void burst_write(uint8_t addr, const uint8_t *d, uint8_t n) {
    uint8_t hdr[2] = { INSTR_WRITE, addr };
    cs_low();
    spi_write_blocking(SPI_PORT, hdr, 2);
    spi_write_blocking(SPI_PORT, d, n);
    cs_high();
}

static void burst_read(uint8_t addr, uint8_t *d, uint8_t n) {
    uint8_t hdr[2] = { INSTR_READ, addr };
    cs_low();
    spi_write_blocking(SPI_PORT, hdr, 2);
    spi_read_blocking(SPI_PORT, 0x00, d, n);
    cs_high();
}

static bool set_mode(uint8_t mode) {
    reg_modify(REG_CANCTRL, MODE_MASK, mode);
    for (int i = 0; i < 20; i++) {
        if ((reg_read(REG_CANSTAT) & MODE_MASK) == mode) return true;
        sleep_ms(1);
    }
    return false;
}

// Bit-timing table for 16 MHz XL2515 crystal (from Waveshare RP2350-CAN demo).
// CNF values authoritative for this board.
typedef struct { uint32_t bps; uint8_t cnf1, cnf2, cnf3; } cnf_entry_t;
static const cnf_entry_t CNF_TABLE[] = {
    { 1000000u, 0x00, 0x82, 0x02 },
    {  800000u, 0x00, 0x92, 0x02 },
    {  500000u, 0x00, 0x9E, 0x03 },
    {  250000u, 0x01, 0x1E, 0x03 },   // default — matches ODrive + Blue Pill
    {  125000u, 0x03, 0x9E, 0x03 },
    {  100000u, 0x04, 0xB5, 0x01 },   // S3 standard; 16MHz BRP=4 NTQ=16 75%SP
    {   50000u, 0x09, 0xB5, 0x01 },   // 16MHz BRP=9 NTQ=16
    {   20000u, 0x18, 0xB5, 0x01 },   // 16MHz BRP=24 NTQ=16
    {   10000u, 0x31, 0xB5, 0x01 },   // 16MHz BRP=49 NTQ=16
};
#define CNF_TABLE_LEN (int)(sizeof(CNF_TABLE)/sizeof(CNF_TABLE[0]))

static bool mcp_set_bitrate(uint32_t bps) {
    for (int i = 0; i < CNF_TABLE_LEN; i++) {
        if (CNF_TABLE[i].bps == bps) {
            reg_write(REG_CNF1, CNF_TABLE[i].cnf1);
            reg_write(REG_CNF2, CNF_TABLE[i].cnf2);
            reg_write(REG_CNF3, CNF_TABLE[i].cnf3);
            return true;
        }
    }
    return false;
}

static bool mcp_init(uint32_t bps) {
    // SPI1 init
    spi_init(SPI_PORT, SPI_BAUD_HZ);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);  cs_high();
    gpio_init(PIN_INT); gpio_set_dir(PIN_INT, GPIO_IN);  gpio_pull_up(PIN_INT);

    // Hardware reset
    uint8_t rst = INSTR_RESET;
    cs_low(); spi_write_blocking(SPI_PORT, &rst, 1); cs_high();
    sleep_ms(10);

    if (!set_mode(MODE_CONFIG)) return false;

    // Set bit timing
    if (!mcp_set_bitrate(bps)) return false;

    // Accept all frames on both RX buffers; RXB0 rollover to RXB1
    reg_write(REG_RXB0CTRL, 0x64);  // RXM=11 (accept all), BUKT=1 (rollover)
    reg_write(REG_RXB1CTRL, 0x60);  // RXM=11 (accept all)
    reg_write(REG_CANINTE,  0x00);  // no interrupts — we poll CANINTF
    reg_write(REG_CANINTF,  0x00);  // clear any pending flags

    return true;
}

static bool mcp_open(void) {
    /* Enable One-Shot Mode (OSM, CANCTRL bit 3) before entering normal mode.
     * OSM = each frame is transmitted ONCE only — no automatic retransmissions.
     * Without OSM, a single failed TX causes the MCP2515 to retry indefinitely
     * until TEC reaches 256 (BUS-OFF), which breaks the whole CAN bus.
     * With OSM, a failed TX just sets ABAT/error flags without escalating. */
    reg_modify(REG_CANCTRL, 0x08, 0x08);   /* set OSM bit (bit 3) */
    return set_mode(MODE_NORMAL);
}

static bool mcp_close(void) {
    return set_mode(MODE_LISTEN);
}

// Send a CAN frame. Returns true on success.
// frame->ext != 0 means extended (29-bit) ID.
typedef struct {
    uint32_t id;
    uint8_t  ext;   // 1 = extended 29-bit ID
    uint8_t  rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

static const uint8_t TX_CTRL[3] = { REG_TXB0CTRL, REG_TXB1CTRL, REG_TXB2CTRL };

static bool mcp_send(const can_frame_t *f) {
    for (int wait = 0; wait < 200; wait++) {
        for (int b = 0; b < 3; b++) {
            if (reg_read(TX_CTRL[b]) & TXBCTRL_TXREQ) continue;  // busy
            uint8_t sidh, sidl, eid8, eid0, dlc;
            if (f->ext) {
                // Extended 29-bit ID: SIDH[10:3] SIDL[2:0]EXIDE=1 EID[17:16] EID8[15:8] EID0[7:0]
                sidh = (uint8_t)(f->id >> 21);
                sidl = (uint8_t)(((f->id >> 18) & 0x07) << 5) | SIDL_EXIDE | (uint8_t)((f->id >> 16) & 0x03);
                eid8 = (uint8_t)(f->id >> 8);
                eid0 = (uint8_t)(f->id & 0xFF);
            } else {
                // Standard 11-bit ID
                sidh = (uint8_t)(f->id >> 3);
                sidl = (uint8_t)((f->id & 0x07) << 5);
                eid8 = 0; eid0 = 0;
            }
            dlc = (f->dlc & 0x0F) | (f->rtr ? DLC_RTR : 0);
            reg_write(TX_CTRL[b] + OFF_SIDH, sidh);
            reg_write(TX_CTRL[b] + OFF_SIDL, sidl);
            reg_write(TX_CTRL[b] + OFF_EID8, eid8);
            reg_write(TX_CTRL[b] + OFF_EID0, eid0);
            reg_write(TX_CTRL[b] + OFF_DLC,  dlc);
            if (!f->rtr && f->dlc)
                burst_write(TX_CTRL[b] + OFF_D0, f->data, f->dlc);
            reg_modify(TX_CTRL[b], TXBCTRL_TXREQ, TXBCTRL_TXREQ);
            return true;
        }
        busy_wait_us(50);
    }
    return false;
}

// Poll for received frame. Returns true if a frame was read.
static bool mcp_recv(can_frame_t *out) {
    uint8_t intf = reg_read(REG_CANINTF);
    uint8_t base;
    uint8_t clear_bit;
    if (intf & CANINTF_RX0) {
        base = REG_RXB0CTRL;
        clear_bit = CANINTF_RX0;
    } else if (intf & CANINTF_RX1) {
        base = REG_RXB1CTRL;
        clear_bit = CANINTF_RX1;
    } else {
        return false;
    }

    uint8_t sidh = reg_read(base + OFF_SIDH);
    uint8_t sidl = reg_read(base + OFF_SIDL);
    uint8_t eid8 = reg_read(base + OFF_EID8);
    uint8_t eid0 = reg_read(base + OFF_EID0);
    uint8_t dlc  = reg_read(base + OFF_DLC);

    out->ext = (sidl & SIDL_EXIDE) ? 1 : 0;
    out->rtr = (dlc & DLC_RTR) ? 1 : 0;
    out->dlc = dlc & 0x0F;
    if (out->dlc > 8) out->dlc = 8;

    if (out->ext) {
        out->id = ((uint32_t)(sidh) << 21)
                | ((uint32_t)((sidl >> 5) & 0x07) << 18)
                | ((uint32_t)(sidl & 0x03) << 16)
                | ((uint32_t)eid8 << 8)
                | (uint32_t)eid0;
    } else {
        out->id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    }

    memset(out->data, 0, 8);
    if (!out->rtr && out->dlc)
        burst_read(base + OFF_D0, out->data, out->dlc);

    reg_modify(REG_CANINTF, clear_bit, 0);  // clear RX flag
    return true;
}

// =============================================================================
// Hex helpers
// =============================================================================

static uint8_t hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static uint8_t parse_hex_byte(const char *p) {
    return (hex_val(p[0]) << 4) | hex_val(p[1]);
}

static uint32_t parse_hex_n(const char *p, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 4) | hex_val(p[i]);
    return v;
}

static const char HEX[] = "0123456789ABCDEF";

static void format_hex_byte(char *out, uint8_t b) {
    out[0] = HEX[(b >> 4) & 0xF];
    out[1] = HEX[b & 0xF];
}

// =============================================================================
// USB serial (stdio) I/O
// =============================================================================

/* Write a string to USB CDC. stdio_flush() is called by the CAN RX path
 * explicitly; for command responses the SDK usually flushes at idle. */
static void usb_puts(const char *s) {
    while (*s) putchar_raw((unsigned char)*s++);
}

// =============================================================================
// SLCAN command processor
// =============================================================================

static bool bus_open = false;

// Handle one complete SLCAN command (without the trailing CR).
static void slcan_process(const char *cmd, int len) {
    if (len < 1) return;

    char c = cmd[0];

    if (c == 'V') {
        // Version
        usb_puts("V0101\r");

    } else if (c == 'N') {
        // Serial
        usb_puts("N0001\r");

    } else if (c == 'S' && len >= 2) {
        // Standard SLCAN bitrate table: S0=10k S1=20k S2=50k S3=100k
        //   S4=125k S5=250k S6=500k S7=800k S8=1M
        // Previous table was missing S3=100k, causing S5→500k instead of 250k.
        static const uint32_t bitrates[] = {
            10000u, 20000u, 50000u, 100000u, 125000u, 250000u, 500000u, 800000u, 1000000u
        };
        int idx = cmd[1] - '0';
        if (idx >= 0 && idx <= 8) {
            // Re-init chip in config mode, set bitrate, leave in config mode
            if (!set_mode(MODE_CONFIG)) return;
            if (!mcp_set_bitrate(bitrates[idx])) {
                usb_puts("\x07\r");  // BEL = error: unsupported rate
                return;
            }
            // Don't auto-open; wait for 'O' command
            bus_open = false;
            usb_puts("\r");  // ACK only on success, inside guard
        } else {
            usb_puts("\x07\r");  // BEL = unknown index (S9, Sn, etc.)
        }

    } else if (c == 'O') {
        // Open bus
        if (mcp_open()) {
            bus_open = true;
            gpio_put(PIN_LED, 1);
        }
        usb_puts("\r");  // SLCAN standard: CR only for O

    } else if (c == 'C') {
        // Close bus
        mcp_close();
        bus_open = false;
        gpio_put(PIN_LED, 0);
        usb_puts("\r");

    } else if (c == 'F') {
        // Read error flags
        uint8_t eflg = reg_read(REG_EFLG);
        char resp[8];
        resp[0] = 'F';
        format_hex_byte(&resp[1], eflg);
        resp[3] = '\r';
        resp[4] = '\0';
        usb_puts(resp);

    } else if ((c == 't' || c == 'T') && bus_open) {
        // Transmit CAN frame
        // 't' = standard 11-bit:  t<III><D><DD..>
        // 'T' = extended 29-bit:  T<IIIIIIII><D><DD..>
        int id_chars = (c == 'T') ? 8 : 3;
        int min_len = 1 + id_chars + 1;   // cmd + ID + DLC
        if (len < min_len) return;

        can_frame_t frame;
        frame.ext = (c == 'T') ? 1 : 0;
        frame.rtr = 0;
        frame.id  = parse_hex_n(&cmd[1], id_chars);
        frame.dlc = (uint8_t)hex_val(cmd[1 + id_chars]);
        if (frame.dlc > 8) frame.dlc = 8;

        int data_offset = 1 + id_chars + 1;
        int expected_len = data_offset + frame.dlc * 2;
        if (len < expected_len) return;

        for (int i = 0; i < frame.dlc; i++)
            frame.data[i] = parse_hex_byte(&cmd[data_offset + i*2]);

        if (mcp_send(&frame)) {
            usb_puts(c == 'T' ? "Z\r" : "z\r");
        }
        // On send failure: no response (host will timeout/retry)

    } else if (c == 'r' || c == 'R') {
        // RTR frame (we just ACK without actually transmitting for simplicity)
        // Most CAN flash tools don't use RTR
        usb_puts(c == 'R' ? "Z\r" : "z\r");
    }
    // Unknown commands are silently ignored per SLCAN convention
}

// =============================================================================
// Main loop
// =============================================================================

int main(void) {
    stdio_init_all();   // USB CDC

    // LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    // Init MCP2515 at 250 kbps (ODrive + Blue Pill default)
    if (!mcp_init(250000u)) {
        // Blink fast to indicate init failure
        while (true) {
            gpio_put(PIN_LED, 1); sleep_ms(100);
            gpio_put(PIN_LED, 0); sleep_ms(100);
        }
    }

    // Auto-open at 250 kbps so the bridge works immediately without 'O' command
    if (mcp_open()) {
        bus_open = true;
        gpio_put(PIN_LED, 1);
    }

    // SLCAN command line accumulator
    char cmd_buf[32];
    int  cmd_len = 0;

    // LED blink state for CAN RX indication
    uint32_t led_off_at_ms = 0;

    while (true) {
        // ---- USB RX: accumulate SLCAN command --------------------------------
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            uint8_t b = (uint8_t)ch;
            if (b == '\r' || b == '\n') {
                // End of command
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    slcan_process(cmd_buf, cmd_len);
                    cmd_len = 0;
                }
            } else if (cmd_len < (int)(sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_len++] = (char)b;
            } else {
                // Buffer overflow — discard
                cmd_len = 0;
            }
        }

        // ---- CAN RX: forward received frames to USB --------------------------
        can_frame_t frame;
        if (mcp_recv(&frame)) {
            // Build SLCAN RX string
            // Standard:  t<III><D><DD...>\r
            // Extended:  T<IIIIIIII><D><DD...>\r
            char buf[32];
            int pos = 0;

            if (frame.ext) {
                buf[pos++] = 'T';
                // 8 hex chars for 29-bit extended ID
                for (int i = 7; i >= 0; i--)
                    buf[pos++] = HEX[(frame.id >> (i*4)) & 0xF];
            } else {
                buf[pos++] = 't';
                // 3 hex chars for 11-bit standard ID
                buf[pos++] = HEX[(frame.id >> 8) & 0xF];
                buf[pos++] = HEX[(frame.id >> 4) & 0xF];
                buf[pos++] = HEX[frame.id & 0xF];
            }

            buf[pos++] = HEX[frame.dlc & 0xF];

            for (int i = 0; i < frame.dlc; i++) {
                buf[pos++] = HEX[(frame.data[i] >> 4) & 0xF];
                buf[pos++] = HEX[frame.data[i] & 0xF];
            }
            buf[pos++] = '\r';
            buf[pos]   = '\0';

            usb_puts(buf);
            stdio_flush();  /* Force USB CDC flush — Pico SDK only flushes on \n, not \r */

            // Blink LED briefly on CAN RX
            gpio_put(PIN_LED, 1);
            led_off_at_ms = to_ms_since_boot(get_absolute_time()) + 50;
        }

        // ---- LED timeout (restore to solid-on when bus is open) --------------
        if (led_off_at_ms && to_ms_since_boot(get_absolute_time()) >= led_off_at_ms) {
            led_off_at_ms = 0;
            gpio_put(PIN_LED, bus_open ? 1 : 0);
        }
    }

    return 0;
}
