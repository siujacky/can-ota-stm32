/* main.c — Blue Pill CAN Tool application.
 * STM32F103C6T6 (32KB flash, 10KB SRAM), 8MHz HSI, no interrupts, no HAL.
 *
 * Features:
 *   - CAN sniffer + injector via SLCAN protocol over USART1 (PA9/PA10, 115200)
 *   - Single-wire UART bridge via USART3 HDSEL (PB10) — DEDICATED pin, no sharing
 *   - MCP2515 on HARDWARE SPI1: CS=PA4 SCK=PA5 MISO=PA6 MOSI=PA7 INT=PB0
 *     (all pins on GPIOA — cleaner wiring; PA4-PA7 are 3.3V only, not 5V tolerant)
 *     PB10 is now FULLY INDEPENDENT — used only for the 1-wire UART bridge.
 */

#include <stdint.h>
#include "device_regs.h"
#include "mcp2515.h"
#include "bxcan.h"  /* bxCAN on PB8/PB9 for 3-node test */
#include "usart.h"
#include "slcan.h"

/* -----------------------------------------------------------------------
 * Application state
 * ----------------------------------------------------------------------- */
typedef enum { MODE_CAN, MODE_UART } app_mode_t;

static volatile uint32_t g_tx_count = 0;
static volatile uint32_t g_rx_count = 0;
static app_mode_t g_mode   = MODE_CAN;
static uint8_t    g_can_open = 0;          /* 1 = CAN bus open (normal mode) */
static uint8_t    g_brate    = MCP_BRATE_500K;  /* current bitrate selection */
static uint8_t    g_led_state = 0;         /* current PC13 LED state */

/* -----------------------------------------------------------------------
 * PC13 LED (active-low) helpers
 * GPIOC CRH: pin13 = bits[23:20]
 * ----------------------------------------------------------------------- */
#define LED_PIN_BIT  13U

static void led_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
    /* PC13 = output push-pull 50MHz */
    GPIOC->CRH &= ~(0xFU << 20);
    GPIOC->CRH |=  (GPIO_OUT_PP_50 << 20);
    /* LED off (active-low → drive HIGH) */
    GPIOC->BSRR = (1U << LED_PIN_BIT);
    g_led_state = 0;
}

static void led_on(void)
{
    GPIOC->BRR = (1U << LED_PIN_BIT);   /* drive low = LED on */
    g_led_state = 1;
}

static void led_off(void)
{
    GPIOC->BSRR = (1U << LED_PIN_BIT);  /* drive high = LED off */
    g_led_state = 0;
}

static void led_toggle(void)
{
    if (g_led_state) led_off();
    else             led_on();
}

/* -----------------------------------------------------------------------
 * Small helpers — no libc
 * ----------------------------------------------------------------------- */
static void print_hex8(uint8_t v)
{
    usart1_tx_byte((uint8_t)nibble_to_hex(v >> 4));
    usart1_tx_byte((uint8_t)nibble_to_hex(v));
}

static void print_uint32(uint32_t v)
{
    char buf[10];
    int pos = 0;
    if (v == 0) { usart1_tx_byte('0'); return; }
    while (v > 0) { buf[pos++] = (char)('0' + (v % 10)); v /= 10; }
    for (int i = pos - 1; i >= 0; i--)
        usart1_tx_byte((uint8_t)buf[i]);
}

/* -----------------------------------------------------------------------
 * Command line buffer
 * ----------------------------------------------------------------------- */
#define CMD_BUF_LEN  40
static char  cmd_buf[CMD_BUF_LEN];
static int   cmd_len = 0;

static void cmd_reset(void) { cmd_len = 0; }
static void cmd_add_char(char c) { if (cmd_len < CMD_BUF_LEN - 1) cmd_buf[cmd_len++] = c; }

/* -----------------------------------------------------------------------
 * Print help text
 * ----------------------------------------------------------------------- */
static void print_help(void)
{
    usart1_print("\r\nSLCAN Commands:\r\n");
    usart1_print("  O        Open CAN bus (normal mode)\r\n");
    usart1_print("  C        Close CAN bus (listen-only)\r\n");
    usart1_print("  S<n>     Bitrate: S4=125k S5=250k S6=500k S7=800k S8=1M\r\n");
    usart1_print("  t<id3><dlc><data>  TX standard frame\r\n");
    usart1_print("  T<id8><dlc><data>  TX extended frame\r\n");
    usart1_print("  r<id3><dlc>        TX RTR standard frame\r\n");
    usart1_print("  F        Read+clear error flags (hex byte)\r\n");
    usart1_print("  s or i   Status (mode/bitrate/counts)\r\n");
    usart1_print("  mode uart <baud>   Switch to UART bridge on PB10\r\n");
    usart1_print("  mode can           Return from UART bridge\r\n");
    usart1_print("  h or ?   This help\r\n");
    usart1_print("Responses: z=tx ok, Z=ext tx ok, \\x07=error\r\n");
}

/* -----------------------------------------------------------------------
 * Print status
 * ----------------------------------------------------------------------- */
static const char *brate_str(uint8_t s)
{
    switch (s) {
    case 4: return "125k";
    case 5: return "250k";
    case 6: return "500k";
    case 7: return "800k";
    case 8: return "1M";
    default: return "?";
    }
}

static void print_status(void)
{
    usart1_print("\r\nMode: ");
    usart1_print(g_mode == MODE_CAN ? "CAN" : "UART");
    usart1_print("  Bus: ");
    usart1_print(g_can_open ? "open" : "closed");
    usart1_print("  Brate: S");
    usart1_tx_byte((uint8_t)('0' + g_brate));
    usart1_tx_byte('=');
    usart1_print(brate_str(g_brate));
    usart1_print("  TX:");
    print_uint32(g_tx_count);
    usart1_print(" RX:");
    print_uint32(g_rx_count);
    usart1_print("\r\n");
}

/* -----------------------------------------------------------------------
 * Execute a parsed SLCAN command
 * Returns 1 if a prompt should be reprinted, 0 if not (sniffer-only output)
 * ----------------------------------------------------------------------- */
static int execute_cmd(slcan_cmd_t cmd, slcan_frame_t *fr)
{
    switch (cmd) {
    case SLCAN_OPEN:
        if (mcp2515_enter_normal() == 0) {
            g_can_open = 1;
            usart1_tx_byte('\r');
        } else {
            usart1_tx_byte('\x07');
        }
        return 1;

    case SLCAN_CLOSE:
        mcp2515_enter_listen_only();
        g_can_open = 0;
        usart1_tx_byte('\r');
        return 1;

    case SLCAN_SETBAUD:
        /* Can only change bitrate when bus is closed */
        mcp2515_enter_listen_only();
        g_can_open = 0;
        mcp2515_set_bitrate(fr->baud_s);
        g_brate = fr->baud_s;
        usart1_tx_byte('\r');
        return 1;

    case SLCAN_TX: {
        if (!g_can_open) { usart1_tx_byte('\x07'); return 1; }
        int ret = bxcan_app_tx(fr->id, fr->data, fr->dlc, fr->extended);
        if (ret == 0) {
            g_tx_count++;
            usart1_tx_byte(fr->extended ? 'Z' : 'z');
            usart1_tx_byte('\r');
        } else {
            usart1_tx_byte('\x07');
        }
        return 0;  /* no prompt for tx — keep stream clean */
    }

    case SLCAN_RTR: {
        if (!g_can_open) { usart1_tx_byte('\x07'); return 1; }
        int ret = mcp2515_tx_rtr(fr->id, fr->dlc, fr->extended);
        if (ret == 0) {
            g_tx_count++;
            usart1_tx_byte('z');
            usart1_tx_byte('\r');
        } else {
            usart1_tx_byte('\x07');
        }
        return 0;
    }

    case SLCAN_FLAGS: {
        uint8_t eflg = bxcan_app_read_errors();
        usart1_print("F");
        print_hex8(eflg);
        usart1_print("\r\n");
        return 1;
    }

    case SLCAN_STATUS:
        print_status();
        return 1;

    case SLCAN_HELP:
        print_help();
        return 1;

    case SLCAN_MODE_UART:
        /* Handled in main loop — signal by returning 0; we set g_mode there */
        return 0;  /* caller checks g_mode */

    case SLCAN_MODE_CAN:
        return 0;  /* handled in main loop */

    case SLCAN_UNKNOWN:
    default:
        usart1_tx_byte('\x07');
        return 1;
    }
}

/* -----------------------------------------------------------------------
 * UART bridge mode — loop until "mode can\r" received on USART1
 * ----------------------------------------------------------------------- */

/* Shift-register for detecting "mode can\r" (9 bytes) in USART1 input */
#define EXIT_SEQ       "mode can\r"
#define EXIT_SEQ_LEN   9
static uint8_t exit_buf[EXIT_SEQ_LEN];
static int     exit_pos = 0;

static int check_exit_seq(uint8_t b)
{
    /* Shift buffer and check */
    for (int i = 0; i < EXIT_SEQ_LEN - 1; i++)
        exit_buf[i] = exit_buf[i + 1];
    exit_buf[EXIT_SEQ_LEN - 1] = b;

    for (int i = 0; i < EXIT_SEQ_LEN; i++) {
        if (exit_buf[i] != (uint8_t)EXIT_SEQ[i]) return 0;
    }
    return 1;
}

static void uart_bridge_loop(uint32_t baud)
{
    /* Solid LED = UART bridge mode */
    led_on();

    usart1_print("\r\nUART bridge mode ");
    print_uint32(baud);
    usart1_print(" baud on PB10. Send 'mode can\\r' to exit.\r\n");

    /* Stop CAN, release MCP2515 SPI pins */
    mcp2515_enter_listen_only();
    g_can_open = 0;
    mcp2515_release_pins();

    /* Init USART3 HDSEL on PB10 */
    usart3_hdsel_init(baud);

    /* Clear exit buffer */
    for (int i = 0; i < EXIT_SEQ_LEN; i++) exit_buf[i] = 0;
    exit_pos = 0;

    /* Warning-once flag for echo mismatch */
    int echo_warned = 0;

    while (1) {
        /* PC → device: byte from USART1 RX → USART3 HDSEL TX */
        int ch = usart1_getchar_nb();
        if (ch >= 0) {
            uint8_t b = (uint8_t)ch;

            /* Check for exit sequence */
            if (check_exit_seq(b)) break;

            /* Forward to device */
            usart3_tx_byte(b);

            /* Discard HDSEL echo (TX is looped back to RX in HDSEL mode).
             * We already waited for TC in usart3_tx_byte(), so the echo
             * should arrive very quickly. Use a 5ms timeout. */
            uint8_t echo = 0;
            int got_echo = usart3_rx_byte_timeout(&echo, 5);
            if (!got_echo && !echo_warned) {
                usart1_print("\r\n[warn: no HDSEL echo — check PB10 pull-up]\r\n");
                echo_warned = 1;
            } else if (got_echo && echo != b && !echo_warned) {
                usart1_print("\r\n[warn: HDSEL echo mismatch — bus collision?]\r\n");
                echo_warned = 1;
            }
        }

        /* Device → PC: byte from USART3 HDSEL RX → USART1 TX */
        int dev_ch = usart3_getchar_nb();
        if (dev_ch >= 0) {
            usart1_tx_byte((uint8_t)dev_ch);
        }
    }

    /* Restore CAN mode */
    usart3_deinit();
    mcp2515_restore_pins();
    mcp2515_init(g_brate);

    led_off();
    g_mode = MODE_CAN;

    usart1_print("\r\nReturned to CAN mode.\r\n>");
}

/* -----------------------------------------------------------------------
 * main — startup and polling loop
 * ----------------------------------------------------------------------- */
int main(void)
{
    /* 1. Enable GPIO clocks on APB2 */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN
                 | RCC_APB2ENR_USART1EN;

    /* 2. PC13 LED */
    led_init();

    /* 3. USART1 at 115200 */
    usart1_init(115200);

    /* 4. MCP2515 — open immediately in NORMAL mode at 250 kbps.
     * Immediate open prevents the Pico 2 SLCAN bridge from going BUS-OFF
     * during a listen-only window (no ACK node → too many TX errors → BUS-OFF).
     * The 'C' SLCAN command closes the bus if needed; 'O' reopens it. */
    /* Init bxCAN (PB8/PB9 + TJA1050) — Standard Mode (osm=0: auto-retry). */
    bxcan_app_init(250000, 0);
    g_brate   = MCP_BRATE_250K;
    g_mode    = MODE_CAN;
    g_can_open = 1;

    /* MCP2515 init deferred: initialized on first use via ID=0x600 trigger. */
    static uint8_t g_mcp_initialized = 0;

    /* Drain bxCAN FIFO: bootloader may have left frames in the FIFO (hello
     * broadcasts on 0x7DE). If the FIFO is full (FMP=3, FOVR set) when the
     * app starts, bxcan_app_rx_available() returns true but new frames cannot
     * enter until the overflow is cleared.  Drain all pending frames now so
     * the main loop starts with an empty FIFO and correct FOVR=0 state. */
    {
        uint32_t dummy_id; uint8_t dummy_data[8]; uint8_t dummy_dlc; int dummy_ext;
        while (bxcan_app_rx_available())
            bxcan_app_rx(&dummy_id, dummy_data, &dummy_dlc, &dummy_ext);
    }

    /* 5. Banner */
    usart1_print("\r\nBlue Pill CAN Tool v1\r\n");
    usart1_print("Pins: CAN=PA4-PA7(SPI1)+PB0(INT)  1wire=PB10(USART3)  Serial=PA9/PA10\r\n");
    usart1_print("250kbps, bus OPEN immediately (no listen-only window).\r\n");
    usart1_print("SLCAN: O=open C=close S6=500k t/T=tx r=rtr F=flags s=stat\r\n");
    usart1_print(">");

    cmd_reset();

    /* ---- Main polling loop ---- */
    while (1) {

        /* ----------------------------------------------------------------
         * bxCAN RX — always active (bxCAN is always open after init).
         * g_can_open tracks MCP2515 SLCAN open/close state; it does NOT
         * gate bxCAN which has its own independent peripheral. Checking
         * g_can_open here caused bxCAN RX to stop when a spurious SLCAN
         * 'C' or 'S' command arrived on floating USART1 RX (PA10). Fixed:
         * bxCAN RX is now unconditional; g_can_open only gates MCP2515 TX.
         * ---------------------------------------------------------------- */
        if (g_mode == MODE_CAN) {
            while (bxcan_app_rx_available()) {
                uint32_t id  = 0;
                uint8_t  dlc = 0;
                uint8_t  data[8] = {0};
                int      ext = 0;

                if (bxcan_app_rx(&id, data, &dlc, &ext)) {
                    /* CAN bootloader trigger: 0x7FF [0xB0,0x01,0xB2] */
                    if (id == 0x7FFU && dlc >= 3 &&
                        data[0] == 0xB0U && data[1] == 0x01U && data[2] == 0xB2U) {
                        RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
                        PWR_CR      |= PWR_CR_DBP;
                        BKP_DR1      = 0xB001U;
                        NVIC_SystemReset();
                    }
                    /* CAN bootloader CLEAR trigger: 0x7FF [0xB0,0x00,0xB2]
                     * Clears BKP_DR1 so the next reset uses the 500ms window,
                     * not the 30s extended window.  Send this before any reset
                     * that is NOT intended to start a firmware upload session. */
                    if (id == 0x7FFU && dlc >= 3 &&
                        data[0] == 0xB0U && data[1] == 0x00U && data[2] == 0xB2U) {
                        RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
                        PWR_CR      |= PWR_CR_DBP;
                        BKP_DR1      = 0x0000U;   /* clear — BL will use 500ms window */
                        NVIC_SystemReset();
                    }
                    /* MCP2515 TX trigger: ID=0x600 → lazy-init MCP2515, then TX 0x601.
                     * Lazy init avoids interference with bxCAN during startup. */
                    if (id == 0x600U) {
                        uint8_t mcp_canstat = 0xFF;  /* 0xFF = SPI not responding */
                        if (!g_mcp_initialized) {
                            mcp2515_init(MCP_BRATE_250K);
                            int init_ok = mcp2515_enter_normal();
                            mcp_canstat = mcp2515_read_canstat();
                            g_mcp_initialized = 1;
                            /* Send SPI diagnostic on 0x602: [data[0], init_ok, CANSTAT] */
                            uint8_t diag[3]={(uint8_t)data[0],(uint8_t)(init_ok==0?1:0),mcp_canstat};
                            bxcan_app_tx(0x602U, diag, 3, 0);
                        } else {
                            mcp_canstat = mcp2515_read_canstat();
                        }
                        uint8_t mcp_tx[4] = {data[0], 0xC1, mcp_canstat, (uint8_t)(g_rx_count & 0xFF)};
                        mcp2515_tx(0x601U, mcp_tx, 4, 0);
                    }
                    /* Echo received frame on id+1 so Pico2 can see bxCAN received it.
                     * Skipped for 0x7FF (bootloader trigger/clear — resets before TX
                     * would be useful) and 0x600 (MCP2515 trigger — has its own reply).
                     * This proves bxCAN RX without requiring SWD g_rx_count reads. */
                    if (id != 0x7FFU && id != 0x600U)
                        bxcan_app_tx(id + 1U, data, dlc, ext);

                    char line[32];
                    slcan_format_rx(id, ext, dlc, data, line);
                    usart1_print(line);
                    led_toggle();
                    g_rx_count++;
                }

                /* bxCAN handles errors internally via ESR register */
            }
        }

        /* ----------------------------------------------------------------
         * USART1 command input (non-blocking character accumulation)
         * ---------------------------------------------------------------- */
        if (g_mode == MODE_CAN) {
            int ch = usart1_getchar_nb();
            if (ch >= 0) {
                uint8_t b = (uint8_t)ch;

                /* Echo the character back */
                /* (SLCAN tools generally don't need echo, but useful for
                 * interactive terminal use) */

                if (b == '\r' || b == '\n') {
                    /* End of command */
                    if (cmd_len > 0) {
                        cmd_buf[cmd_len] = '\0';

                        slcan_frame_t frame;
                        slcan_cmd_t result = slcan_parse_cmd(cmd_buf, &frame);

                        if (result == SLCAN_MODE_UART) {
                            /* Switch to UART bridge mode */
                            g_mode = MODE_UART;
                            cmd_reset();
                            uart_bridge_loop(frame.uart_baud);
                            /* Returns here after 'mode can' received */
                            continue;
                        }

                        int show_prompt = execute_cmd(result, &frame);
                        if (show_prompt)
                            usart1_print(">");

                        cmd_reset();
                    }
                } else if (b == 0x08 || b == 0x7F) {
                    /* Backspace */
                    if (cmd_len > 0) cmd_len--;
                } else if (cmd_len < CMD_BUF_LEN - 1) {
                    cmd_buf[cmd_len++] = (char)b;
                }
                /* else: buffer full — ignore character */
            }
        }

    } /* while(1) */

    return 0; /* never reached */
}
