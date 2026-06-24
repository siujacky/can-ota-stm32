/* usart.c — USART1 (PC) + USART3 HDSEL (single-wire bridge) driver.
 * No HAL, no interrupts — pure polling.
 * 8 MHz HSI clock on both APB buses.
 *
 * USART1 (APB2 @ 8MHz): PA9=TX PA10=RX, 115200 8N1 permanent PC interface.
 * USART3 (APB1 @ 8MHz): PB10=single-wire HDSEL TX/RX, baud from 'mode uart' cmd.
 *
 * STM32F1 note: USART1 is on APB2, USART2/3/4/5 on APB1.
 * Both buses run at 8MHz HSI (no PLL), so BRR calculation is identical.
 */

#include "usart.h"
#include "device_regs.h"

/* -----------------------------------------------------------------------
 * 1-ms spin at 8 MHz (-O2 approximately 2667 loop iterations)
 * ----------------------------------------------------------------------- */
static void spin_1ms(void)
{
    volatile uint32_t c = 2667;
    while (c--);
}

/* -----------------------------------------------------------------------
 * USART1 — PC interface, always 115200 8N1
 * PA9 = TX: AF push-pull 50MHz (CRH pin9 field = bits[7:4])
 * PA10 = RX: floating input    (CRH pin10 field = bits[11:8])
 * ----------------------------------------------------------------------- */
void usart1_init(uint32_t baud)
{
    /* GPIOA and USART1 clocks on APB2 (should already be on, but be safe) */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9 = TX: AF push-pull 50MHz — CRH bits[7:4] */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (GPIO_AF_PP_50 << 4);   /* 0xB */

    /* PA10 = RX: input with pull-UP — CRH bits[11:8] = 0x8, then BSRR.
     * UART idles HIGH; a floating pin causes noise that can trigger spurious
     * SLCAN commands (e.g. 'C' close) and reset g_can_open. Pull-up prevents this. */
    GPIOA->CRH &= ~(0xFU << 8);
    GPIOA->CRH |=  (0x8U << 8);              /* 0x8 = input with pull-up/down */
    GPIOA->BSRR = (1U << 10);               /* drive ODR=1 → enables pull-UP */

    /* BRR, then enable USART with TX+RX */
    USART1_BRR = usart_brr(baud);
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void usart1_tx_byte(uint8_t b)
{
    while (!(USART1_SR & USART_SR_TXE));
    USART1_DR = b;
}

void usart1_print(const char *s)
{
    while (*s)
        usart1_tx_byte((uint8_t)*s++);
}

int usart1_rx_ready(void)
{
    return (USART1_SR & USART_SR_RXNE) ? 1 : 0;
}

uint8_t usart1_rx_byte(void)
{
    while (!(USART1_SR & USART_SR_RXNE));
    return (uint8_t)(USART1_DR & 0xFFU);
}

int usart1_getchar_nb(void)
{
    if (USART1_SR & USART_SR_RXNE)
        return (int)(USART1_DR & 0xFFU);
    return -1;
}

/* -----------------------------------------------------------------------
 * USART3 HDSEL — single-wire half-duplex bridge
 * PB10 = single-wire pin: AF open-drain 50MHz
 *   CRH pin10 field = bits[11:8]
 *
 * In HDSEL mode:
 *   - TX and RX share the same pin (PB10)
 *   - Hardware automatically tristates between TX and RX
 *   - Every byte transmitted is internally echoed back to RX shift register
 *   - USART3 must be on APB1 (clock enable RCC_APB1ENR bit18)
 *
 * IMPORTANT: PB10 is also used as SCK in CAN/SPI mode.
 * mcp2515_release_pins() must be called before usart3_hdsel_init().
 * ----------------------------------------------------------------------- */
void usart3_hdsel_init(uint32_t baud)
{
    /* Enable GPIOB (should already be on) and USART3 on APB1 */
    RCC_APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART3EN;

    /* PB10 = single-wire: AF open-drain 50MHz — CRH bits[11:8] */
    GPIOB->CRH &= ~(0xFU << 8);
    GPIOB->CRH |=  (GPIO_AF_OD_50 << 8);   /* 0xF */

    /* Configure USART3: disable first, set BRR, enable with HDSEL */
    USART3_CR1 = 0;  /* disable */
    USART3_CR2 = 0;  /* no LIN, no clock, 1 stop bit */
    USART3_CR3 = 0;  /* clear all, then set HDSEL */

    USART3_BRR = usart_brr(baud);

    /* Enable USART3 TX+RX */
    USART3_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* Set HDSEL — must be done while USART is enabled (per RM0008) */
    USART3_CR3 |= USART_CR3_HDSEL;
}

/* Disable USART3 and release PB10 to floating input */
void usart3_deinit(void)
{
    USART3_CR1 = 0;
    USART3_CR3 = 0;

    /* PB10 → floating input (SPI SCK will be re-configured by mcp2515_restore_pins) */
    GPIOB->CRH &= ~(0xFU << 8);
    GPIOB->CRH |=  (GPIO_INPUT_FLOAT << 8);
}

void usart3_tx_byte(uint8_t b)
{
    while (!(USART3_SR & USART_SR_TXE));
    USART3_DR = b;
    /* Wait for TC to ensure byte fully shifted out before we switch to RX.
     * In HDSEL mode this prevents us from reading our own start bit. */
    while (!(USART3_SR & USART_SR_TC));
}

int usart3_rx_ready(void)
{
    return (USART3_SR & USART_SR_RXNE) ? 1 : 0;
}

uint8_t usart3_rx_byte(void)
{
    while (!(USART3_SR & USART_SR_RXNE));
    return (uint8_t)(USART3_DR & 0xFFU);
}

int usart3_getchar_nb(void)
{
    /* Clear framing/overrun errors before checking — prevents stall */
    uint32_t sr = USART3_SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)USART3_DR;  /* reading DR clears error flags */
        return -1;
    }
    if (sr & USART_SR_RXNE)
        return (int)(USART3_DR & 0xFFU);
    return -1;
}

/* Read one byte from USART3 with millisecond timeout.
 * Returns 1 and sets *out on success; returns 0 on timeout. */
int usart3_rx_byte_timeout(uint8_t *out, uint32_t timeout_ms)
{
    while (timeout_ms) {
        if (USART3_SR & USART_SR_RXNE) {
            *out = (uint8_t)(USART3_DR & 0xFFU);
            return 1;
        }
        /* Check for errors — drain and retry */
        if (USART3_SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            (void)USART3_DR;
        }
        spin_1ms();
        timeout_ms--;
    }
    return 0;
}
