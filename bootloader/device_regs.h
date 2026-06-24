/* device_regs.h — Minimal STM32F103 peripheral register definitions.
 * No HAL, no CMSIS device headers — only what the bootloader needs.
 * Peripheral base addresses from STM32F103 reference manual (RM0008).
 */

#ifndef DEVICE_REGS_H
#define DEVICE_REGS_H

#include <stdint.h>

/* ---- Cortex-M3 SCB (from CMSIS core) ---- */
#define SCB_BASE    0xE000ED00UL
#define SCB_VTOR    (*(volatile uint32_t *)(SCB_BASE + 0x08))

/* ---- RCC ---- */
#define RCC_BASE    0x40021000UL
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x1C))  /* APB1 clocks */
#define RCC_APB2ENR (*(volatile uint32_t *)(RCC_BASE + 0x18))  /* APB2 clocks */
#define RCC_APB2ENR_IOPAEN    (1U << 2)
#define RCC_APB2ENR_IOPBEN    (1U << 3)
#define RCC_APB2ENR_SPI1EN    (1U << 12)  /* SPI1 on APB2 */
#define RCC_APB2ENR_USART1EN  (1U << 14)  /* USART1 on APB2 */

/* ---- SPI1 (APB2) — hardware SPI for MCP2515 ----
 * PA4=CS(GPIO), PA5=SCK(AF_PP), PA6=MISO(float), PA7=MOSI(AF_PP), PB0=INT
 * At 8MHz APB2 with BR=001 → SPI clock = 8/4 = 2 MHz */
#define SPI1_BASE     0x40013000UL
#define SPI1_CR1      (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_SR       (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR       (*(volatile uint32_t *)(SPI1_BASE + 0x0C))
#define SPI1_CR1_SSM      (1U << 9)
#define SPI1_CR1_SSI      (1U << 8)
#define SPI1_CR1_SPE      (1U << 6)
#define SPI1_CR1_BR_DIV4  (1U << 3)  /* BR=001 = fPCLK/4 */
#define SPI1_CR1_MSTR     (1U << 2)
#define SPI1_SR_RXNE      (1U << 0)
#define SPI1_SR_TXE       (1U << 1)

/* ---- GPIO ---- */
#define GPIOA_BASE  0x40010800UL
#define GPIOB_BASE  0x40010C00UL
#define GPIOC_BASE  0x40011000UL

typedef struct {
    volatile uint32_t CRL;   /* offset 0x00 — pins 0-7  */
    volatile uint32_t CRH;   /* offset 0x04 — pins 8-15 */
    volatile uint32_t IDR;   /* offset 0x08 */
    volatile uint32_t ODR;   /* offset 0x0C */
    volatile uint32_t BSRR;  /* offset 0x10 */
    volatile uint32_t BRR;   /* offset 0x14 */
    volatile uint32_t LCKR;  /* offset 0x18 */
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *)GPIOC_BASE)

/* GPIO CRL/CRH mode/config values (2+2 bits per pin) */
#define GPIO_MODE_OUT_50  0x3U   /* Output, max speed 50MHz */
#define GPIO_CNF_PP       0x0U   /* Push-pull output        */
#define GPIO_MODE_IN      0x0U   /* Input mode              */
#define GPIO_CNF_FLOAT    0x1U   /* Floating input          */
#define GPIO_CNF_PULL     0x2U   /* Input with pull-up/down */

/* ---- FLASH ---- */
#define FLASH_BASE      0x40022000UL
#define FLASH_ACR       (*(volatile uint32_t *)(FLASH_BASE + 0x00))
#define FLASH_KEYR      (*(volatile uint32_t *)(FLASH_BASE + 0x04))
#define FLASH_SR        (*(volatile uint32_t *)(FLASH_BASE + 0x0C))
#define FLASH_CR        (*(volatile uint32_t *)(FLASH_BASE + 0x10))
#define FLASH_AR        (*(volatile uint32_t *)(FLASH_BASE + 0x14))

#define FLASH_CR_PG     (1U << 0)
#define FLASH_CR_PER    (1U << 1)
#define FLASH_CR_STRT   (1U << 6)
#define FLASH_CR_LOCK   (1U << 7)
#define FLASH_SR_BSY    (1U << 0)
#define FLASH_SR_EOP    (1U << 5)

/* ---- DESIG (Device signature / unique ID) ---- */
#define DESIG_UNIQUE_ID_BASE  0x1FFFF7E8UL
#define DESIG_UNIQUE_ID0  (*(volatile uint32_t *)(DESIG_UNIQUE_ID_BASE + 0x00))
#define DESIG_UNIQUE_ID1  (*(volatile uint32_t *)(DESIG_UNIQUE_ID_BASE + 0x04))
#define DESIG_UNIQUE_ID2  (*(volatile uint32_t *)(DESIG_UNIQUE_ID_BASE + 0x08))

/* ---- DBGMCU (ARM CoreSight debug — always readable, no debugger needed) ----
 * DBGMCU_IDCODE at 0xE0042000:
 *   bits [11:0]  = Device ID  (e.g. 0x414 = STM32F103xC/D/E high-density)
 *   bits [15:12] = reserved
 *   bits [31:16] = Revision ID
 * FLASHSIZE at 0x1FFFF7E0: 16-bit value = total flash in kilobytes.
 * Both are valid in production code — no debug connection required.
 */
#define DBGMCU_IDCODE      (*(volatile uint32_t *)0xE0042000UL)
#define FLASHSIZE_REG      (*(volatile uint16_t *)0x1FFFF7E0UL)

/* ---- SysTick (needed to disable before ROM DFU jump) ---- */
#define SYSTICK_BASE   0xE000E010UL
#define SYST_CSR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYST_RVR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYST_CVR       (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

/* ---- STM32 ROM bootloader (system memory) addresses ----
 * Each family has its own address (AN2606, Table 2).
 * The bootloader reads BOOT0/BOOT1 state at reset AND honours a direct
 * call from application code if peripherals are cleanly reset first.
 */
#define ROM_BL_ADDR_F1   0x1FFFF000UL   /* STM32F103xx  (Cortex-M3)  → USB DFU VID_0483:PID_DF11 */
#define ROM_BL_ADDR_F4   0x1FFF0000UL   /* STM32F4xx    (Cortex-M4F) → USB DFU VID_0483:PID_DF11 */

/* Magic for BKP_DR2 to trigger ROM DFU on next reset (separate from BKP_DR1's
 * enter-our-bootloader magic). */
#define BKP_MAGIC_ENTER_DFU   0xDFD0U   /* "DFU-Device-0" */

/* ---- NVIC disable-all helper ---- */
#define NVIC_ICER0  (*(volatile uint32_t *)0xE000E180UL)
#define NVIC_ICER1  (*(volatile uint32_t *)0xE000E184UL)
#define NVIC_ICER2  (*(volatile uint32_t *)0xE000E188UL)

/* ---- RCC APB1ENR additional bits (for PWR + BKP clock enable) ---- */
#define RCC_APB1ENR_PWREN  (1UL << 28)   /* Power interface clock */
#define RCC_APB1ENR_BKPEN  (1UL << 27)   /* Backup interface clock */

/* ---- PWR (Power control) — needed to disable BKP write protection ---- */
#define PWR_BASE  0x40007000UL
#define PWR_CR    (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CR_DBP (1UL << 8)            /* Disable Backup domain write Protection */

/* ---- BKP (Backup data registers) — survive NVIC_SystemReset() ----
 * The application writes BKP_DR1 = BKP_MAGIC_ENTER_BL before calling
 * NVIC_SystemReset(). The bootloader reads and clears this at startup
 * to detect "reboot-to-bootloader" and extend the programming window to
 * 30 s instead of the normal 500 ms.
 */
#define BKP_BASE            0x40006C00UL
#define BKP_DR1             (*(volatile uint16_t *)(BKP_BASE + 0x04))  /* enter-our-BL flag */
#define BKP_DR2             (*(volatile uint16_t *)(BKP_BASE + 0x08))  /* enter-ROM-DFU flag */
#define BKP_MAGIC_ENTER_BL  0xB001U   /* "Boot-One" — arbitrary, unlikely to
                                        * appear by accident on power-on     */


/* ---- bxCAN (needed by bxcan_bl.c) ---- */
#define RCC_APB1ENR_CANEN   (1UL << 25)   /* CAN clock enable */
#define RCC_APB2ENR_AFIOEN  (1UL << 0)    /* AFIO clock enable */
#endif /* DEVICE_REGS_H */
