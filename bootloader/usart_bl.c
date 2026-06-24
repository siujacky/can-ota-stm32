/* usart_bl.c — USART1 driver for the bootloader (no HAL, direct registers).
 *
 * STM32F103 USART1: APB2 peripheral, base 0x40013800.
 * Pins: PA9 = TX (AF push-pull, 50 MHz), PA10 = RX (floating input).
 * 8 MHz HSI, 115200 baud: USARTDIV = 4.340 → BRR = (4<<4)|5 = 0x0045.
 * 8N1, no parity, no flow control, no interrupts (polling only).
 */

#include "usart_bl.h"
#include "device_regs.h"

/* ---- USART1 register definitions ---- */
#define USART1_BASE  0x40013800UL
#define USART1_SR   (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_DR   (*(volatile uint32_t *)(USART1_BASE + 0x04))
#define USART1_BRR  (*(volatile uint32_t *)(USART1_BASE + 0x08))
#define USART1_CR1  (*(volatile uint32_t *)(USART1_BASE + 0x0C))
#define USART1_CR3  (*(volatile uint32_t *)(USART1_BASE + 0x14))

#define USART_SR_RXNE  (1U << 5)   /* RX data register not empty */
#define USART_SR_TXE   (1U << 7)   /* TX data register empty     */
#define USART_SR_TC    (1U << 6)   /* Transmission complete       */
#define USART_CR1_UE   (1U << 13)  /* USART enable                */
#define USART_CR1_TE   (1U << 3)   /* Transmitter enable          */
#define USART_CR1_RE   (1U << 2)   /* Receiver enable             */
#define USART_CR3_HDSEL (1U << 3)  /* Half-Duplex Selection       */

/* ---- RCC USART1 clock enable ---- */
#define RCC_APB2ENR_USART1EN  (1U << 14)

/* GPIO AF push-pull output for TX (50 MHz = mode 0b11, cnf 0b10)
 * STM32F1 CRL/CRH encoding per pin: MODE[1:0] | CNF[1:0]
 *   AF_PP: MODE=11 (50MHz), CNF=10 → combined 4-bit = 0b1011 = 0xB
 *   AF_OD: MODE=11 (50MHz), CNF=11 → combined 4-bit = 0b1111 = 0xF
 * AF_OD is required for single-wire (HDSEL) mode on a shared bus;
 * it lets multiple nodes drive the line without output contention. */
#define GPIO_MODE_OUT_50_AFPP  0xBU  /* CNF=10 (AF PP), MODE=11 (50MHz) */
#define GPIO_MODE_OUT_50_AFOD  0xFU  /* CNF=11 (AF OD), MODE=11 (50MHz) */

/* Approximate 1-ms spin at 8 MHz (-O2).  Used inside timeout loops. */
static inline void spin_1ms(void)
{
    volatile uint32_t c = 2667;
    while (c--)
        ;
}

void usart_init(void)
{
    /* Enable GPIOA and USART1 clocks on APB2 */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9 = TX: AF push-pull 50 MHz (CRH bits [7:4] = 0b1011 = 0xB) */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (GPIO_MODE_OUT_50_AFPP << 4);

    /* PA10 = RX: floating input (CRH bits [11:8] = 0b0100 = 0x4) */
    GPIOA->CRH &= ~(0xFU << 8);
    GPIOA->CRH |=  ((GPIO_MODE_IN | (GPIO_CNF_FLOAT << 2)) << 8);

    /* Configure USART1: 115200 @ 8 MHz HSI.
     * USARTDIV = 8 000 000 / (16 × 115 200) = 4.340...
     * Mantissa = 4, Fraction = round(0.340 × 16) = 5
     * BRR = (4 << 4) | 5 = 0x0045
     * CR3 stays 0 (HDSEL clear = full-duplex).
     */
    USART1_BRR = 0x0045U;
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* -----------------------------------------------------------------------
 * Single-wire (HDSEL) init — PA9 AF open-drain, PA10 not used.
 *
 * PA9 must be open-drain when shared with other devices so no two nodes
 * drive the line high simultaneously.  Add a 4k7 pull-up to VCC externally.
 * Point-to-point with just two nodes can use push-pull (AF_PP) instead; just
 * swap GPIO_MODE_OUT_50_AFOD for GPIO_MODE_OUT_50_AFPP in the CRH write.
 * ----------------------------------------------------------------------- */
void usart_init_hdsel(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9 = single-wire pin: AF open-drain 50 MHz.
     * In HDSEL mode the USART hardware automatically tristates PA9 between
     * transmissions so the line can be driven by another node. */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (GPIO_MODE_OUT_50_AFOD << 4);  /* 0xF = AF_OD 50MHz */

    /* PA10 is disconnected internally when HDSEL is set — leave it alone. */

    USART1_BRR = 0x0045U;
    /* Enable USART TX+RX, then set HDSEL (must be set while USART is enabled) */
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART1_CR3 |= USART_CR3_HDSEL;
}

/* -----------------------------------------------------------------------
 * Auto-detect: try HDSEL, probe with 0x55, fall back to full-duplex.
 *
 * In HDSEL mode every transmitted byte is internally echoed back to the
 * receive shift register — so if we send 0x55 and get 0x55 back within
 * 2 ms we know single-wire is active on PA9.  If nothing echoes (because
 * only PA10 is wired, not PA9), we re-init in full-duplex mode.
 * ----------------------------------------------------------------------- */
int usart_init_autodetect(void)
{
    usart_init_hdsel();

    /* Drain any stale RX data before the probe */
    if (USART1_SR & USART_SR_RXNE) { (void)USART1_DR; }

    /* Send probe byte and wait for the echo */
    USART1_DR = 0x55U;
    uint8_t echo;
    if (usart_rx_buf(&echo, 1, 2) && echo == 0x55U) {
        return 1;   /* HDSEL echo confirmed — single-wire mode active */
    }

    /* No echo — fall back to full-duplex */
    USART1_CR1  = 0;                   /* disable USART */
    USART1_CR3 &= ~USART_CR3_HDSEL;   /* clear HDSEL */
    usart_init();
    return 0;
}

/* -----------------------------------------------------------------------
 * TX-and-verify: transmit each byte and read back the HDSEL echo.
 *
 * In HDSEL mode the hardware loops TX back to RX, so usart_rx_byte()
 * immediately after usart_tx_byte() returns the transmitted byte — this
 * is not a physical echo from the peer, it is the loopback within the chip.
 * A mismatch means the line was driven to a different value (collision,
 * short, or stuck wire) — useful for bus arbitration or wire integrity.
 *
 * Returns: 0 if all echoes matched; number of mismatched bytes otherwise.
 * timeout_ms: per-byte timeout (5 ms is comfortable at 115200 baud).
 * ----------------------------------------------------------------------- */
int usart_tx_verify(const uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    int errors = 0;
    for (uint8_t i = 0; i < len; i++) {
        usart_tx_byte(buf[i]);
        uint8_t echo;
        if (usart_rx_buf(&echo, 1, timeout_ms)) {
            if (echo != buf[i])
                errors++;
        } else {
            errors++;   /* timeout — no echo within the window */
        }
    }
    return errors;
}

int usart_rx_ready(void)
{
    return (USART1_SR & USART_SR_RXNE) ? 1 : 0;
}

uint8_t usart_rx_byte(void)
{
    while (!(USART1_SR & USART_SR_RXNE))
        ;
    return (uint8_t)(USART1_DR & 0xFFU);
}

void usart_tx_byte(uint8_t b)
{
    while (!(USART1_SR & USART_SR_TXE))
        ;
    USART1_DR = b;
}

void usart_tx(const uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
        usart_tx_byte(buf[i]);
}

int usart_rx_buf(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    for (uint32_t i = 0; i < len; i++) {
        uint32_t t = timeout_ms;
        while (!(USART1_SR & USART_SR_RXNE)) {
            spin_1ms();
            if (--t == 0)
                return 0;  /* timeout */
        }
        buf[i] = (uint8_t)(USART1_DR & 0xFFU);
    }
    return 1;
}

void usart_print(const char *s)
{
    while (*s)
        usart_tx_byte((uint8_t)*s++);
}

void usart_print_hex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 28; i >= 0; i -= 4)
        usart_tx_byte((uint8_t)hex[(v >> i) & 0xFU]);
}

void usart_print_uint32(uint32_t v)
{
    char buf[10];
    int  pos = 0;
    if (v == 0) {
        usart_tx_byte('0');
        return;
    }
    while (v > 0) {
        buf[pos++] = (char)('0' + (v % 10));
        v /= 10;
    }
    /* buf holds digits in reverse order */
    for (int i = pos - 1; i >= 0; i--)
        usart_tx_byte((uint8_t)buf[i]);
}
