/* flash_bl.h — STM32F103 flash programming API for the bootloader.
 * No HAL. Uses direct register access via device_regs.h.
 *
 * FLASH PAGE SIZE NOTES:
 *   STM32F103RCT6 is high-density (256KB). Hardware erase granularity is 2KB.
 *   FLASH_PAGE_SIZE (1024) is the protocol page size — matches the CAN/UART
 *   upload protocol. FLASH_HW_PAGE_SIZE (2048) is the actual hardware erase
 *   unit. flash_write_page() tracks the last erased 2KB page so that two
 *   consecutive 1KB protocol writes to the same 2KB hardware page only erase
 *   once (not twice, which would destroy the first write).
 *
 * DUAL-SLOT A/B LAYOUT (256KB total):
 *   0x08000000..0x08001FFF  8KB   Bootloader (this code)
 *   0x08002000..0x0801FFFF  120KB Slot A — primary image
 *   0x08020000..0x0803DFFF  120KB Slot B — secondary image
 *   0x0803E000..0x0803EFFF  2KB   Bootloader config (bl_config_t)
 *   0x0803F000..0x0803FFFF  4KB   Application NVRAM (FlashContent)
 */

#ifndef FLASH_BL_H
#define FLASH_BL_H

#include <stdint.h>

/* Protocol page size — must match the upload tool (1KB per page). */
#define FLASH_PAGE_SIZE     1024U

/* Hardware erase granularity for STM32F103RC (high-density, 256KB). */
#define FLASH_HW_PAGE_SIZE  2048U

/* Legacy APP_FLASH_START kept as Slot A start for backwards compatibility. */
#define APP_FLASH_START     0x08002000UL

/* Dual-slot A/B layout */
#define SLOT_A_START        0x08002000UL   /* Primary image   (compile at 0x08002000) */
#define SLOT_A_SIZE         (120U * 1024U) /* 120 KB */
#define SLOT_B_START        0x08020000UL   /* Secondary image (compile at 0x08020000) */
#define SLOT_B_SIZE         (120U * 1024U) /* 120 KB */
#define BL_CONFIG_ADDR      0x0803E000UL   /* 2KB: bootloader config (bl_config_t) */
#define APP_NVRAM_ADDR      0x0803F000UL   /* 4KB: application FlashContent */

#define BL_CONFIG_MAGIC     0xB007AB01UL   /* "boot AB 01" */

typedef struct {
    uint32_t magic;
    uint32_t boot_slot;   /* 0 = Slot A, 1 = Slot B */
    uint32_t padding[6];  /* reserved */
} bl_config_t;

/* ---- Basic flash operations -------------------------------------------- */

/* Unlock the flash for programming/erasing */
void flash_unlock(void);

/* Lock the flash (set LOCK bit) */
void flash_lock(void);

/* Erase a single 2KB hardware page containing 'addr'.
 * SAFETY: refuses to erase below APP_FLASH_START (0x08002000). */
void flash_erase_page(uint32_t addr);

/* Erase a single 2KB hardware page — NO APP_FLASH_START guard.
 * Use only for BL_CONFIG writes and custom-address uploads. */
void flash_erase_page_any(uint32_t addr);

/* Erase all 2KB hardware pages covering [start, end] (inclusive).
 * No APP_FLASH_START guard. Used for NVRAM clear and region wipes. */
void flash_erase_region(uint32_t start, uint32_t end);

/* Write a single 16-bit halfword to 'addr' (must be 2-byte aligned).
 * Flash must already be unlocked and the page erased. */
void flash_write_halfword(uint32_t addr, uint16_t data);

/* Erase (if needed) and program up to FLASH_PAGE_SIZE bytes of a protocol
 * page at dst.
 *   dst  — flash destination (page-aligned to FLASH_PAGE_SIZE, >= APP_FLASH_START)
 *   src  — pointer to source data in RAM
 *   len  — number of bytes to write (writes exactly len bytes, no padding)
 * Tracks the last-erased 2KB hardware page to avoid double-erase of two
 * consecutive 1KB writes within the same 2KB hardware page.
 * SAFETY: refuses to write below APP_FLASH_START. */
void flash_write_page(uint32_t dst, const uint8_t *src, uint32_t len);

/* Same as flash_write_page but with NO APP_FLASH_START guard.
 * Use for BL_CONFIG writes and force-overwrite of bootloader region. */
void flash_write_page_any(uint32_t dst, const uint8_t *src, uint32_t len);

#endif /* FLASH_BL_H */
