/* flash_bl.c — STM32F103 flash programming (no HAL).
 *
 * PAGE SIZE BUG FIX:
 *   STM32F103RCT6 (high-density, 256KB) has 2KB hardware erase pages, not 1KB.
 *   The upload protocol uses 1KB pages (FLASH_PAGE_SIZE). Two consecutive 1KB
 *   writes land in the same 2KB hardware page; a naive per-write erase would
 *   destroy the first write when starting the second.
 *
 *   Fix: s_last_erased_page tracks the 2KB-aligned address of the last erased
 *   hardware page. flash_write_page() only erases when the 2KB-aligned base of
 *   the destination differs from s_last_erased_page.
 *
 * SAFETY NOTES:
 *  - flash_erase_page() and flash_write_page() refuse to operate below
 *    APP_FLASH_START (0x08002000) — the bootloader cannot erase itself.
 *  - _any() variants have no guard — needed for BL_CONFIG and custom uploads.
 *  - Per-page CRC32 in main_bl.c verifies before committing — corrupt page
 *    sends 'E' and master resends.
 */

#include "flash_bl.h"
#include "device_regs.h"

/* Keys from STM32F103 reference manual section 3.3.2 */
#define FLASH_KEY1  0x45670123UL
#define FLASH_KEY2  0xCDEF89ABUL

/* Tracks last 2KB-aligned hardware page that was erased.
 * Initialised to an invalid address so the first write always erases. */
static uint32_t s_last_erased_page = 0xFFFFFFFFUL;

/* Wait until flash is not busy */
static inline void flash_wait(void)
{
    while (FLASH_SR & FLASH_SR_BSY)
        ;
}

void flash_unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
    /* Reset the cached page so next unlock+write erases fresh */
    s_last_erased_page = 0xFFFFFFFFUL;
}

/* Internal: erase the 2KB hardware page that contains 'addr'. No guard. */
static void flash_erase_hw_page(uint32_t addr)
{
    uint32_t hw_page = addr & ~(FLASH_HW_PAGE_SIZE - 1UL);
    flash_wait();
    FLASH_CR |= FLASH_CR_PER;
    FLASH_AR  = hw_page;
    FLASH_CR |= FLASH_CR_STRT;
    flash_wait();
    FLASH_CR &= ~FLASH_CR_PER;
    s_last_erased_page = hw_page;
}

void flash_erase_page(uint32_t addr)
{
    /* Safety: never erase the bootloader */
    if (addr < APP_FLASH_START) return;
    flash_erase_hw_page(addr);
}

void flash_erase_page_any(uint32_t addr)
{
    flash_erase_hw_page(addr);
}

void flash_erase_region(uint32_t start, uint32_t end)
{
    /* Align start down to 2KB boundary, walk forward erasing each page */
    uint32_t page = start & ~(FLASH_HW_PAGE_SIZE - 1UL);
    while (page <= end) {
        flash_erase_hw_page(page);
        page += FLASH_HW_PAGE_SIZE;
    }
}

void flash_write_halfword(uint32_t addr, uint16_t data)
{
    flash_wait();
    FLASH_CR |= FLASH_CR_PG;
    *((volatile uint16_t *)addr) = data;
    flash_wait();
    FLASH_CR &= ~FLASH_CR_PG;
}

/* Internal: write len bytes from src to dst, optionally guarded. */
static void flash_write_page_impl(uint32_t dst, const uint8_t *src,
                                  uint32_t len, int guarded)
{
    if (guarded && dst < APP_FLASH_START) return;

    /* Snap dst to 2KB hardware page boundary */
    uint32_t hw_page = dst & ~(FLASH_HW_PAGE_SIZE - 1UL);

    /* Only erase the hardware page if we haven't already done so */
    if (hw_page != s_last_erased_page) {
        flash_erase_hw_page(hw_page);
        /* s_last_erased_page updated inside flash_erase_hw_page */
    }

    /* Write exactly len bytes, halfword-by-halfword */
    uint32_t i = 0;
    while (i < len) {
        uint8_t lo = src[i++];
        uint8_t hi = (i < len) ? src[i++] : 0xFFU;
        uint16_t hw = (uint16_t)(lo | ((uint16_t)hi << 8));
        flash_write_halfword(dst, hw);
        dst += 2;
    }
    /* No padding — writing past len would cross into the next protocol page
     * which occupies the same 2KB hardware page. */
}

void flash_write_page(uint32_t dst, const uint8_t *src, uint32_t len)
{
    flash_write_page_impl(dst, src, len, 1);
}

void flash_write_page_any(uint32_t dst, const uint8_t *src, uint32_t len)
{
    flash_write_page_impl(dst, src, len, 0);
}
