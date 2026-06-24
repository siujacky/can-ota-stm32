/* device_regs.h — All STM32F103 peripheral registers needed by blue_pill_can_tool.
 * No HAL, no CMSIS device headers — direct register access only.
 * Based on STM32F103 Reference Manual RM0008.
 */

#ifndef DEVICE_REGS_H
#define DEVICE_REGS_H

#include <stdint.h>

/* ---- Cortex-M3 SCB ---- */
#define SCB_BASE    0xE000ED00UL
#define SCB_VTOR    (*(volatile uint32_t *)(SCB_BASE + 0x08))
#define SCB_AIRCR   (*(volatile uint32_t *)(SCB_BASE + 0x0C))
#define NVIC_SystemReset() do { SCB_AIRCR = (0x5FA0000UL|(1UL<<2)); while(1); } while(0)

/* ---- NVIC disable-all helpers ---- */
#define NVIC_ICER0  (*(volatile uint32_t *)0xE000E180UL)
#define NVIC_ICER1  (*(volatile uint32_t *)0xE000E184UL)
#define NVIC_ICER2  (*(volatile uint32_t *)0xE000E188UL)

/* ---- SysTick (disabled at startup) ---- */
#define SYSTICK_BASE   0xE000E010UL
#define SYST_CSR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYST_RVR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYST_CVR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

/* ---- RCC ---- */
#define RCC_BASE        0x40021000UL
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x18))

/* RCC_APB2ENR bits */
#define RCC_APB2ENR_IOPAEN    (1U << 2)   /* GPIOA clock */
#define RCC_APB2ENR_IOPBEN    (1U << 3)   /* GPIOB clock */
#define RCC_APB2ENR_IOPCEN    (1U << 4)   /* GPIOC clock */
#define RCC_APB2ENR_SPI1EN    (1U << 12)  /* SPI1 clock (APB2) */
#define RCC_APB2ENR_USART1EN  (1U << 14)  /* USART1 clock (APB2) */

/* ---- SPI1 (APB2, 8 MHz clock) ----
 * Hardware SPI for MCP2515: PA5=SCK, PA6=MISO, PA7=MOSI (AF_PP / float input).
 * PA4=CS controlled as plain GPIO (manual NSS, SSM=1, SSI=1 in CR1).
 * PA8=INT (GPIO input with pull-up, active-low).
 * At 8 MHz APB2 with BR=001 (fPCLK/4) → SPI clock = 2 MHz (well within MCP2515's 10 MHz max).
 */
#define SPI1_BASE    0x40013000UL
#define SPI1_CR1     (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_SR      (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR      (*(volatile uint32_t *)(SPI1_BASE + 0x0C))

/* SPI1_CR1 bits used */
#define SPI1_CR1_CPHA     (1U << 0)   /* Clock phase */
#define SPI1_CR1_CPOL     (1U << 1)   /* Clock polarity */
#define SPI1_CR1_MSTR     (1U << 2)   /* Master mode */
#define SPI1_CR1_BR_MASK  (7U << 3)   /* Baud rate bits [5:3] */
#define SPI1_CR1_BR_DIV4  (1U << 3)   /* fPCLK/4 */
#define SPI1_CR1_SPE      (1U << 6)   /* SPI enable */
#define SPI1_CR1_LSBFIRST (1U << 7)   /* LSB first (0=MSB first) */
#define SPI1_CR1_SSI      (1U << 8)   /* Internal NSS high (software NSS) */
#define SPI1_CR1_SSM      (1U << 9)   /* Software NSS management */

/* SPI1_SR bits */
#define SPI1_SR_RXNE      (1U << 0)   /* Receive buffer not empty */
#define SPI1_SR_TXE       (1U << 1)   /* Transmit buffer empty */
#define SPI1_SR_BSY       (1U << 7)   /* SPI busy */

/* RCC_APB1ENR bits */
#define RCC_APB1ENR_USART3EN  (1U << 18)  /* USART3 clock (APB1) */
#define RCC_APB1ENR_PWREN     (1U << 28)  /* Power interface clock */
#define RCC_APB1ENR_BKPEN     (1U << 27)  /* Backup interface clock */

/* ---- GPIO ---- */
#define GPIOA_BASE  0x40010800UL
#define GPIOB_BASE  0x40010C00UL
#define GPIOC_BASE  0x40011000UL

typedef struct {
    volatile uint32_t CRL;   /* 0x00 — pins 0-7  */
    volatile uint32_t CRH;   /* 0x04 — pins 8-15 */
    volatile uint32_t IDR;   /* 0x08 */
    volatile uint32_t ODR;   /* 0x0C */
    volatile uint32_t BSRR;  /* 0x10  bits[15:0]=set, bits[31:16]=clear */
    volatile uint32_t BRR;   /* 0x14  bits[15:0]=clear */
    volatile uint32_t LCKR;  /* 0x18 */
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *)GPIOC_BASE)

/*
 * GPIO CRL/CRH 4-bit field encoding (CNF[1:0] : MODE[1:0]):
 *   Input floating:          CNF=01, MODE=00 → 0x4
 *   Output PP  50MHz:        CNF=00, MODE=11 → 0x3
 *   Output OD  50MHz:        CNF=01, MODE=11 → 0x7
 *   AF PP      50MHz:        CNF=10, MODE=11 → 0xB
 *   AF OD      50MHz:        CNF=11, MODE=11 → 0xF
 */
#define GPIO_INPUT_FLOAT    0x4U
#define GPIO_OUT_PP_50      0x3U
#define GPIO_OUT_OD_50      0x7U
#define GPIO_AF_PP_50       0xBU
#define GPIO_AF_OD_50       0xFU

/* ---- USART1 (APB2, 8MHz clock) ---- */
#define USART1_BASE  0x40013800UL
#define USART1_SR    (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_DR    (*(volatile uint32_t *)(USART1_BASE + 0x04))
#define USART1_BRR   (*(volatile uint32_t *)(USART1_BASE + 0x08))
#define USART1_CR1   (*(volatile uint32_t *)(USART1_BASE + 0x0C))
#define USART1_CR2   (*(volatile uint32_t *)(USART1_BASE + 0x10))
#define USART1_CR3   (*(volatile uint32_t *)(USART1_BASE + 0x14))

/* ---- USART3 (APB1, 8MHz clock) ---- */
#define USART3_BASE  0x40004800UL
#define USART3_SR    (*(volatile uint32_t *)(USART3_BASE + 0x00))
#define USART3_DR    (*(volatile uint32_t *)(USART3_BASE + 0x04))
#define USART3_BRR   (*(volatile uint32_t *)(USART3_BASE + 0x08))
#define USART3_CR1   (*(volatile uint32_t *)(USART3_BASE + 0x0C))
#define USART3_CR2   (*(volatile uint32_t *)(USART3_BASE + 0x10))
#define USART3_CR3   (*(volatile uint32_t *)(USART3_BASE + 0x14))

/* USART SR bits */
#define USART_SR_PE    (1U << 0)
#define USART_SR_FE    (1U << 1)
#define USART_SR_NE    (1U << 2)
#define USART_SR_ORE   (1U << 3)
#define USART_SR_RXNE  (1U << 5)
#define USART_SR_TC    (1U << 6)
#define USART_SR_TXE   (1U << 7)

/* USART CR1 bits */
#define USART_CR1_RE   (1U << 2)
#define USART_CR1_TE   (1U << 3)
#define USART_CR1_UE   (1U << 13)

/* USART CR3 bits */
#define USART_CR3_HDSEL  (1U << 3)   /* Half-duplex single-wire selection */

/* ---- DBGMCU + Flash size ---- */
#define DBGMCU_IDCODE   (*(volatile uint32_t *)0xE0042000UL)
#define FLASHSIZE_REG   (*(volatile uint16_t *)0x1FFFF7E0UL)

/* ---- PWR + BKP (not needed at runtime, kept for completeness) ---- */
#define PWR_BASE   0x40007000UL
#define PWR_CR     (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CR_DBP (1U << 8)

#define BKP_BASE   0x40006C00UL
#define BKP_DR1    (*(volatile uint16_t *)(BKP_BASE + 0x04))

#endif /* DEVICE_REGS_H */
