/* usart.h — USART1 (PC interface) and USART3 HDSEL (single-wire bridge).
 * No HAL — direct register access.
 * USART1: PA9=TX PA10=RX, always 115200 baud (PC side).
 * USART3: PB10=HDSEL single-wire, baud configurable (device side).
 */

#ifndef USART_H
#define USART_H

#include <stdint.h>

/* ---- USART1 (PC interface, 115200 8N1) ---- */
void    usart1_init(uint32_t baud);
void    usart1_tx_byte(uint8_t b);
void    usart1_print(const char *s);
int     usart1_rx_ready(void);
uint8_t usart1_rx_byte(void);
int     usart1_getchar_nb(void);   /* non-blocking, returns -1 if no data */

/* ---- USART3 HDSEL (single-wire, configurable baud) ---- */
void    usart3_hdsel_init(uint32_t baud);
void    usart3_deinit(void);
void    usart3_tx_byte(uint8_t b);
int     usart3_rx_ready(void);
uint8_t usart3_rx_byte(void);
int     usart3_getchar_nb(void);   /* non-blocking, returns -1 if no data */
int     usart3_rx_byte_timeout(uint8_t *out, uint32_t timeout_ms); /* 0=timeout, 1=ok */

/* ---- BRR helper: integer divisor for OVER8=0 ----
 * STM32F1 at 8MHz: BRR = (mantissa<<4)|(frac) where USARTDIV = fCLK/(16*baud)
 * Simple approach: use pre-computed value for 115200, formula for others.
 */
#define USART_BRR_115200_8MHZ  0x0045U   /* (4<<4)|5 verified from bootloader */

static inline uint32_t usart_brr(uint32_t baud)
{
    /* USARTDIV = fCLK / (16 * baud) expressed as 12.4 fixed-point.
     * mantissa = fCLK / (16 * baud)
     * fraction = round((fCLK % (16*baud)) * 16 / (16*baud))
     * Simplified: BRR integer approximation = (8000000 + baud/2) / baud
     * This works because 8MHz/(16*baud) = 500000/baud and BRR = mantissa<<4|frac.
     * For most baud rates at 8MHz the fraction is small enough that:
     *   BRR ≈ (8000000/baud/16)<<4 + round(8000000/baud * 16 % 16)
     * Use the fixed 16-bit divisor form: BRR = 8000000/(baud) approximation.
     * Actual: USARTDIV_x16 = 8000000*16/baud / 16 = 8000000/baud ... no.
     *
     * Correct STM32F1 formula (OVER8=0):
     *   BRR[15:4] = mantissa = floor(fCLK / (16*baud))
     *   BRR[3:0]  = fraction = round((fCLK / (16*baud) - mantissa) * 16)
     * Combined: BRR = round(fCLK / baud) — but encoded as 12.4 fixed point
     *   = round(8000000 * 16 / baud / 16) expressed correctly.
     *
     * Simplest correct implementation:
     *   uint32_t div16 = (8000000*16 + baud/2) / baud;  // USARTDIV * 16
     *   mantissa = div16 >> 4;
     *   fraction = div16 & 0xF;
     *   BRR = (mantissa << 4) | fraction;
     * But for 115200: div16 = 128000000/115200 = 1111 = 0x457 → man=0x45 frac=7
     *   Hmm, but bootloader uses 0x0045. Let's check:
     *   8000000/(16*115200) = 4.340; mantissa=4, frac=round(0.340*16)=5 → BRR=0x0045. OK.
     *   div16 = 8000000*16/115200 = 128000000/115200 = 1111.111... → round=1112 = 0x458
     *   man=0x45=69, frac=8... that gives different answer.
     * Use the floor approach: div16 = 8000000*16/baud (integer division)
     *   For 115200: 128000000/115200 = 1111 (floor) → man=69=0x45, frac=7 → 0x0047
     * But bootloader uses 0x0045 (frac=5). That comes from round(0.340*16)=round(5.44)=5.
     * So: float_usartdiv = 8000000.0/(16*baud); frac=round((usartdiv-floor(usartdiv))*16)
     *
     * Integer-only, no float: frac = (8000000 - mantissa*16*baud)*16 / (16*baud)
     *   = (8000000 - man*16*baud) / baud
     * For 115200: man=4; frac = (8000000 - 4*16*115200)/115200 = (8000000-7372800)/115200
     *             = 627200/115200 = 5.44 → floor=5. This matches 0x0045!
     */
    if (baud == 115200) return USART_BRR_115200_8MHZ;
    uint32_t man = 8000000U / (16U * baud);
    uint32_t frac = (8000000U - man * 16U * baud) / baud;
    if (frac > 15) frac = 15;
    return (man << 4) | frac;
}

#endif /* USART_H */
