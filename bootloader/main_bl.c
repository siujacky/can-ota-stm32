/* main_bl.c — STM32F103 dual-transport bootloader (CAN/MCP2515 + USART1).
 *
 * Features:
 *   - Interactive UART menu (selectable upload address, A/B slot, NVRAM clear)
 *   - Dual-slot A/B boot with persistent NVRAM config (BL_CONFIG_ADDR)
 *   - Selectable upload target: Slot A, Slot B, or custom address
 *   - NVRAM clear (APP_NVRAM_ADDR, 4KB)
 *   - Flash page-size bug fixed: FLASH_HW_PAGE_SIZE=2048 tracked separately
 *     from protocol FLASH_PAGE_SIZE=1024
 *
 * Startup sequence:
 *   1. Disable interrupts.
 *   2. Read bl_config → determine boot_slot.
 *   3. Init MCP2515 + USART1.
 *   4. Broadcast CAN advert (0x7DE).
 *   5. Poll both transports for 500ms:
 *        UART 0xAA  → auto upload to SLOT_A_START (backwards compat)
 *        UART other → uart_menu()
 *        CAN match  → can_upload(SLOT_A_START)
 *        neither    → jump_to_slot(boot_slot)
 */

#include <stdint.h>
#include "device_regs.h"
#include "bxcan_bl.h"
#include "usart_bl.h"
#include "flash_bl.h"

#define CAN_TX_ID    0x7DEU
#define CAN_RX_ID    0x7DDU
#define UART_TRIGGER 0xAAU

/* Control channel IDs */
#define BL_CTRL_RX_ID  0x7DCU
#define BL_CTRL_TX_ID  0x7DBU

/* Control command bytes */
#define BL_CMD_BOOT          0x00U
#define BL_CMD_UPLOAD_SLOT_A 0x01U
#define BL_CMD_UPLOAD_SLOT_B 0x02U
#define BL_CMD_UPLOAD_ADDR   0x03U
#define BL_CMD_SET_BOOT_A    0x04U
#define BL_CMD_SET_BOOT_B    0x05U
#define BL_CMD_CLEAR_NVRAM   0x06U
#define BL_CMD_STATUS        0x07U
#define BL_CMD_ENTER_DFU     0x08U   /* Jump to ROM DFU (VID_0483:PID_DF11) */

/* Control response status bytes */
#define BL_STATUS_OK         0x00U
#define BL_STATUS_AUTH_FAIL  0x01U
#define BL_STATUS_INVALID    0x02U
#define BL_STATUS_FLASH_ERR  0x03U

/* ----------------------------------------------------------------------- */
/* Simple delay: 8 MHz HSI, ~1ms per call at -O2                           */
/* ----------------------------------------------------------------------- */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        volatile uint32_t c = 2667;
        while (c--)
            ;
    }
}

/* ----------------------------------------------------------------------- */
/* CRC32 — polynomial 0x04C11DB7 (MPEG-2, no bit-reflection)               */
/* ----------------------------------------------------------------------- */
static uint32_t crc32_update(uint32_t crc, uint32_t word)
{
    crc ^= word;
    for (int i = 0; i < 32; i++)
        crc = (crc & 0x80000000UL) ? (crc << 1) ^ 0x04C11DB7UL : (crc << 1);
    return crc;
}

static uint32_t crc32_page(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i = 0;
    while (i + 3 < len) {
        uint32_t word = ((uint32_t)data[i])
                      | ((uint32_t)data[i+1] << 8)
                      | ((uint32_t)data[i+2] << 16)
                      | ((uint32_t)data[i+3] << 24);
        crc = crc32_update(crc, __builtin_bswap32(word));
        i += 4;
    }
    if (i < len) {
        uint32_t word = 0, shift = 24;
        while (i < len) { word |= ((uint32_t)data[i++] << shift); shift -= 8; }
        crc = crc32_update(crc, word);
    }
    return crc;
}

/* ----------------------------------------------------------------------- */
/* BL config read/write                                                     */
/* ----------------------------------------------------------------------- */
/* Check if BL_CONFIG_ADDR is within this chip's physical flash.
 * The STM32F103C6T6 (Blue Pill) has only 32KB flash (0x08000000..0x08007FFF).
 * BL_CONFIG_ADDR = 0x0803E000 is valid on 256KB chips but causes a bus fault
 * on 32KB chips. We read FLASHSIZE_REG to detect this at runtime.               */
static int config_addr_valid(void)
{
    uint32_t flash_end = 0x08000000UL + ((uint32_t)FLASHSIZE_REG * 1024UL);
    return (BL_CONFIG_ADDR + sizeof(bl_config_t)) <= flash_end;
}

static void bl_config_read(bl_config_t *cfg)
{
    cfg->magic     = BL_CONFIG_MAGIC;
    cfg->boot_slot = 0;
    if (!config_addr_valid()) return;  /* 32KB chip: skip, use defaults */
    *cfg = *(const volatile bl_config_t *)BL_CONFIG_ADDR;
    if (cfg->magic != BL_CONFIG_MAGIC) {
        cfg->magic     = BL_CONFIG_MAGIC;
        cfg->boot_slot = 0;
    }
}

static void bl_config_write(const bl_config_t *cfg)
{
    if (!config_addr_valid()) return;  /* 32KB chip: can't store config */
    flash_unlock();
    flash_erase_page_any(BL_CONFIG_ADDR);
    const uint8_t *p = (const uint8_t *)cfg;
    uint32_t addr = BL_CONFIG_ADDR;
    for (uint32_t i = 0; i < sizeof(bl_config_t); i += 2) {
        uint8_t lo = p[i];
        uint8_t hi = (i + 1 < sizeof(bl_config_t)) ? p[i + 1] : 0xFFU;
        flash_write_halfword(addr, (uint16_t)(lo | ((uint16_t)hi << 8)));
        addr += 2;
    }
    flash_lock();
}

/* ----------------------------------------------------------------------- */
/* Jump to STM32 ROM bootloader (system memory) — enables USB DFU on Win  */
/*                                                                          */
/* When Windows shows "DEVICE_DESCRIPTOR_FAILURE" / VID_0000:PID_0002 the  */
/* STM32 is not responding to USB enumeration.  The internal ROM bootloader  */
/* at 0x1FFFF000 (F103) / 0x1FFF0000 (F4) handles USB and appears as       */
/* VID_0483:PID_DF11 which STM32CubeProgrammer or dfu-util recognises.     */
/*                                                                          */
/* Triggered by: BKP_DR2 = BKP_MAGIC_ENTER_DFU (0xDFD0) set by the        */
/* application BEFORE calling NVIC_SystemReset(), OR by menu option [7].   */
/*                                                                          */
/* Sequence (per AN2606 "jump from application" recommendation):            */
/*   1. Disable SysTick so no tick fires into wrong vector table.           */
/*   2. Disable + clear all NVIC IRQs.                                      */
/*   3. Reset APB clocks (USB transceiver needs a clean state).             */
/*   4. Load SP from ROM address[0]; verify it points into SRAM.            */
/*   5. Load PC from ROM address[1]; jump.                                  */
/* The ROM bootloader then initialises USB and enumerates on Windows as     */
/* VID_0483:PID_DF11 without any additional driver if STM32CubeProgrammer  */
/* is installed, or after Zadig assigns WinUSB for dfu-util.               */
/* ----------------------------------------------------------------------- */
static void enter_rom_dfu(void)
{
    /* Step 1: Disable SysTick */
    SYST_CSR = 0; SYST_RVR = 0; SYST_CVR = 0;

    /* Step 2: Disable all IRQs */
    __asm volatile("cpsid i");
    NVIC_ICER0 = 0xFFFFFFFFUL;
    NVIC_ICER1 = 0xFFFFFFFFUL;
    NVIC_ICER2 = 0xFFFFFFFFUL;
    /* Clear pending */
    *(volatile uint32_t *)0xE000E280UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E284UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E288UL = 0xFFFFFFFFUL;

    /* Step 3: Reset APB1+APB2 peripheral clocks so USB sees a clean state */
    RCC_APB1ENR = 0;
    RCC_APB2ENR = 0;

    /* Step 4+5: Read SP and PC from the ROM bootloader vector table.
     * Select address by DevID: F4 uses 0x1FFF0000, F103 uses 0x1FFFF000. */
    uint16_t dev_id = (uint16_t)(DBGMCU_IDCODE & 0x0FFFU);
    uint32_t rom_base = ROM_BL_ADDR_F1;  /* default: STM32F103 */
    /* STM32F4/F7/G4 family: ROM DFU at 0x1FFF0000 (AN2606 Table 2).
     * F401: 0x0423 (xB/xC) / 0x0433 (xD/xE)
     * F405/407: 0x0413   F42x/43x: 0x0419
     * F411: 0x0431/0x0441   F412: 0x0441   F446: 0x0421   F469/479: 0x0434
     * F7xx: 0x0449/0x0458
     * G4 Cat.2 (G431/G441): 0x0468
     * G4 Cat.3 (G471/G473/G474/G483/G484): 0x0469  ← G474CEU6 is here
     * G4 Cat.4 (G491/G4A1): 0x0479
     * NOTE: keep in sync with chip_detect.c device table. */
    if (dev_id == 0x0423U || dev_id == 0x0433U ||   /* F401xB/C, F401xD/E */
        dev_id == 0x0413U || dev_id == 0x0419U ||   /* F405/407, F42x/43x */
        dev_id == 0x0431U || dev_id == 0x0441U ||   /* F411xC/E, F412 */
        dev_id == 0x0421U || dev_id == 0x0434U ||   /* F446, F469/479 */
        dev_id == 0x0449U || dev_id == 0x0458U ||   /* F7xx */
        dev_id == 0x0468U || dev_id == 0x0469U ||   /* G4 Cat.2 (G431/441), G4 Cat.3 (G474) */
        dev_id == 0x0479U) {                         /* G4 Cat.4 (G491/G4A1) */
        rom_base = ROM_BL_ADDR_F4;       /* STM32F4/F7/G4 all share 0x1FFF0000 */
    }

    uint32_t rom_sp = *(volatile uint32_t *)rom_base;
    uint32_t rom_pc = *(volatile uint32_t *)(rom_base + 4);

    /* Sanity-check: SP must point into SRAM (0x20000000+), PC must be odd
     * (Thumb bit set) and point into system flash space (< 0x20000000). */
    if ((rom_sp & 0xFF000000UL) != 0x20000000UL) return;   /* bad SP, stay in BL */
    if ((rom_pc & 1) == 0 || rom_pc < 0x10000000UL) return;

    SCB_VTOR = rom_base;       /* relocate vector table to ROM BL */
    __asm volatile("msr msp, %0\n" : : "r"(rom_sp));
    __asm volatile("bx  %0\n"     : : "r"(rom_pc));
    __builtin_unreachable();
}

/* ----------------------------------------------------------------------- */
/* Jump to application at arbitrary address                                */
/* ----------------------------------------------------------------------- */
static void jump_to_app_at(uint32_t start)
{
    __asm volatile("cpsid i");
    NVIC_ICER0 = 0xFFFFFFFFUL;
    NVIC_ICER1 = 0xFFFFFFFFUL;
    NVIC_ICER2 = 0xFFFFFFFFUL;

    uint32_t app_sp = *(volatile uint32_t *)start;
    uint32_t app_pc = *(volatile uint32_t *)(start + 4);

    SCB_VTOR = start;
    __asm volatile("msr msp, %0\nbx %1\n" : : "r"(app_sp), "r"(app_pc));
    __builtin_unreachable();
}

/* Global config — filled at startup */
static bl_config_t g_config;

/* Upload target — default Slot A, changeable via control channel */
static uint32_t g_upload_target       = SLOT_A_START;
static uint8_t  g_upload_target_slot  = 0;  /* 0=A, 1=B, 0xFF=custom */

static void jump_to_slot(uint32_t slot)
{
    uint32_t target = (slot == 1) ? SLOT_B_START : SLOT_A_START;
    jump_to_app_at(target);
}

/* ----------------------------------------------------------------------- */
/* Control channel helpers                                                  */
/* ----------------------------------------------------------------------- */

/* BL version sent in STATUS frames */
#define BL_VERSION_MAJOR  1U
#define BL_VERSION_MINOR  0U

/* Build and transmit a control response on BL_CTRL_TX_ID.
 *   cmd    — echoed CMD byte
 *   status — BL_STATUS_* code
 *   info4  — 4-byte extra info (LE), e.g. uid0 for STATUS cmd
 * buf[2] = g_config.boot_slot, buf[3] = g_upload_target_slot (always current).
 */
static void ctrl_respond(uint8_t cmd, uint8_t status, uint32_t info4)
{
    uint8_t buf[8];
    buf[0] = cmd;
    buf[1] = status;
    buf[2] = (uint8_t)g_config.boot_slot;
    buf[3] = g_upload_target_slot;
    buf[4] = (uint8_t)(info4);
    buf[5] = (uint8_t)(info4 >> 8);
    buf[6] = (uint8_t)(info4 >> 16);
    buf[7] = (uint8_t)(info4 >> 24);
    bxcan_tx(BL_CTRL_TX_ID, buf, 8);
}

/* Send 3 STATUS frames on BL_CTRL_TX_ID (0x7DB) carrying the full banner info.
 * The host tool (can_flash.py --status) reads all 3 frames and prints:
 *   biPropellant CAN Generic BL v<maj>.<min>
 *   UID: <uid0>-<uid1>-<uid2>
 *   Active slot: A/B (0x08002000 / 0x08020000)
 *   Next boot:   A/B
 *
 * Frame 0  [0x07]: STATUS_OK | boot_slot | target_slot | uid0 LE
 * Frame 1  [0x17]: uid1 LE (4B) | uid2 low 2B
 * Frame 2  [0x27]: uid2 high 2B | ver_major | ver_minor | slot_a_id | slot_b_id
 *   slot_a_id = 0xA1 (fixed marker for Slot A = 0x08002000)
 *   slot_b_id = 0xB2 (fixed marker for Slot B = 0x08020000)
 * Sub-frame tag is top nibble of byte[0]: 0x07 = frame-0, 0x17 = frame-1, 0x27 = frame-2.
 */
static void bl_broadcast_status(void)
{
    uint32_t uid0 = DESIG_UNIQUE_ID0;
    uint32_t uid1 = DESIG_UNIQUE_ID1;
    uint32_t uid2 = DESIG_UNIQUE_ID2;
    uint8_t  slot = (uint8_t)g_config.boot_slot;
    uint8_t buf[8];

    /* Frame 0: cmd=0x07, status=OK, boot_slot, target_slot, uid0 LE */
    buf[0] = BL_CMD_STATUS;            /* 0x07 = frame-0 tag */
    buf[1] = BL_STATUS_OK;
    buf[2] = slot;
    buf[3] = g_upload_target_slot;
    buf[4] = (uint8_t)(uid0);
    buf[5] = (uint8_t)(uid0 >> 8);
    buf[6] = (uint8_t)(uid0 >> 16);
    buf[7] = (uint8_t)(uid0 >> 24);
    bxcan_tx(BL_CTRL_TX_ID, buf, 8);

    /* Frame 1: uid1 LE + uid2 low 2 bytes */
    buf[0] = 0x17U;                    /* 0x17 = frame-1 tag */
    buf[1] = BL_STATUS_OK;
    buf[2] = (uint8_t)(uid1);
    buf[3] = (uint8_t)(uid1 >> 8);
    buf[4] = (uint8_t)(uid1 >> 16);
    buf[5] = (uint8_t)(uid1 >> 24);
    buf[6] = (uint8_t)(uid2);
    buf[7] = (uint8_t)(uid2 >> 8);
    bxcan_tx(BL_CTRL_TX_ID, buf, 8);

    /* Frame 2: uid2 high 2 bytes + version + slot markers */
    buf[0] = 0x27U;                    /* 0x27 = frame-2 tag */
    buf[1] = BL_STATUS_OK;
    buf[2] = (uint8_t)(uid2 >> 16);
    buf[3] = (uint8_t)(uid2 >> 24);
    buf[4] = BL_VERSION_MAJOR;
    buf[5] = BL_VERSION_MINOR;
    buf[6] = 0xA1U;                    /* Slot A marker (host: 0x08002000) */
    buf[7] = 0xB2U;                    /* Slot B marker (host: 0x08020000) */
    bxcan_tx(BL_CTRL_TX_ID, buf, 8);

    /* Frame 3: chip self-identification from DBGMCU + FLASHSIZE.
     * Tag 0x37; both registers are always readable (no debugger required).
     *   dev_id   = DBGMCU_IDCODE bits [11:0]  (e.g. 0x0414 = F103 high-density)
     *   rev_id   = DBGMCU_IDCODE bits [31:16]  (silicon revision)
     *   flash_kb = FLASHSIZE_REG               (e.g. 256 for STM32F103RC)
     * can_flash.py maps dev_id → part name for the printed banner. */
    {
        uint32_t idcode   = DBGMCU_IDCODE;
        uint16_t dev_id   = (uint16_t)(idcode & 0x0FFFU);
        uint16_t rev_id   = (uint16_t)(idcode >> 16);
        uint16_t flash_kb = FLASHSIZE_REG;
        buf[0] = 0x37U;
        buf[1] = BL_STATUS_OK;
        buf[2] = (uint8_t)(dev_id);
        buf[3] = (uint8_t)(dev_id >> 8);
        buf[4] = (uint8_t)(rev_id);
        buf[5] = (uint8_t)(rev_id >> 8);
        buf[6] = (uint8_t)(flash_kb);
        buf[7] = (uint8_t)(flash_kb >> 8);
        bxcan_tx(BL_CTRL_TX_ID, buf, 8);
    }
}

/* Handle an incoming control frame from 0x7DC.
 * Frame layout: [0]=CMD [1..3]=uid0 low 3 bytes (auth) [4..7]=args
 */
static void handle_ctrl_cmd(const uint8_t *data, uint8_t len)
{
    (void)len;

    /* Auth check: first 3 bytes of DESIG_UNIQUE_ID0 */
    uint32_t uid0 = DESIG_UNIQUE_ID0;
    if (data[1] != (uint8_t)(uid0)      ||
        data[2] != (uint8_t)(uid0 >> 8) ||
        data[3] != (uint8_t)(uid0 >> 16)) {
        ctrl_respond(data[0], BL_STATUS_AUTH_FAIL, 0);
        return;
    }

    switch (data[0]) {
    case BL_CMD_BOOT:
        ctrl_respond(BL_CMD_BOOT, BL_STATUS_OK, 0);
        delay_ms(50);
        jump_to_slot(g_config.boot_slot);
        break;

    case BL_CMD_UPLOAD_SLOT_A:
        g_upload_target      = SLOT_A_START;
        g_upload_target_slot = 0;
        ctrl_respond(BL_CMD_UPLOAD_SLOT_A, BL_STATUS_OK, 0);
        break;

    case BL_CMD_UPLOAD_SLOT_B:
        g_upload_target      = SLOT_B_START;
        g_upload_target_slot = 1;
        ctrl_respond(BL_CMD_UPLOAD_SLOT_B, BL_STATUS_OK, 0);
        break;

    case BL_CMD_UPLOAD_ADDR: {
        uint32_t addr = (uint32_t)data[4]
                      | ((uint32_t)data[5] << 8)
                      | ((uint32_t)data[6] << 16)
                      | ((uint32_t)data[7] << 24);
        g_upload_target      = addr;
        g_upload_target_slot = 0xFFU;
        ctrl_respond(BL_CMD_UPLOAD_ADDR, BL_STATUS_OK, 0);
        break;
    }

    case BL_CMD_SET_BOOT_A:
        g_config.boot_slot = 0;
        bl_config_write(&g_config);
        ctrl_respond(BL_CMD_SET_BOOT_A, BL_STATUS_OK, 0);
        delay_ms(50);
        jump_to_slot(0);
        break;

    case BL_CMD_SET_BOOT_B:
        g_config.boot_slot = 1;
        bl_config_write(&g_config);
        ctrl_respond(BL_CMD_SET_BOOT_B, BL_STATUS_OK, 0);
        delay_ms(50);
        jump_to_slot(1);
        break;

    case BL_CMD_CLEAR_NVRAM:
        flash_unlock();
        flash_erase_region(APP_NVRAM_ADDR, APP_NVRAM_ADDR + 4096U - 1U);
        flash_lock();
        ctrl_respond(BL_CMD_CLEAR_NVRAM, BL_STATUS_OK, 0);
        delay_ms(200);
        jump_to_slot(g_config.boot_slot);
        break;

    case BL_CMD_STATUS:
        bl_broadcast_status();
        break;

    case BL_CMD_ENTER_DFU:
        /* Jump to STM32 ROM DFU bootloader.
         * Device will enumerate as VID_0483:PID_DF11 — recognised by
         * STM32CubeProgrammer and dfu-util after Zadig assigns WinUSB. */
        ctrl_respond(BL_CMD_ENTER_DFU, BL_STATUS_OK, 0);
        delay_ms(50);              /* let the ACK frame transmit */
        enter_rom_dfu();
        /* If we get here the board has no USB / ROM BL is invalid */
        ctrl_respond(BL_CMD_ENTER_DFU, BL_STATUS_FLASH_ERR, 0);
        break;

    default:
        ctrl_respond(data[0], BL_STATUS_INVALID, 0);
        break;
    }
}

/* ----------------------------------------------------------------------- */
/* Transport state                                                          */
/* ----------------------------------------------------------------------- */
typedef enum { TRANSPORT_NONE, TRANSPORT_CAN, TRANSPORT_UART } transport_t;
static transport_t g_transport;

static void send_byte(uint8_t b)
{
    if (g_transport == TRANSPORT_CAN) {
        uint8_t buf[1] = { b };
        bxcan_tx(CAN_TX_ID, buf, 1);
    } else {
        usart_tx_byte(b);
    }
}

static int recv_bytes(uint8_t *buf, uint32_t want, uint32_t timeout_ms)
{
    if (g_transport == TRANSPORT_CAN) {
        uint32_t got = 0;
        uint32_t polls_per_ms = 267U;
        while (got < want) {
            uint16_t id;
            uint8_t  tmp[8], rx_len;
            uint32_t polls = timeout_ms * polls_per_ms;
            int found = 0;
            while (polls--) {
                if (bxcan_rx(&id, tmp, &rx_len) && id == CAN_RX_ID) {
                    found = 1;
                    break;
                }
            }
            if (!found) return 0;
            uint32_t remain2 = want - got;
            uint8_t  take    = (rx_len < remain2) ? rx_len : (uint8_t)remain2;
            for (uint8_t i = 0; i < take; i++)
                buf[got++] = tmp[i];
        }
        return 1;
    } else {
        return usart_rx_buf(buf, want, timeout_ms);
    }
}

/* ----------------------------------------------------------------------- */
/* Page buffer (BSS — zeroed by startup_bl.s)                              */
/* ----------------------------------------------------------------------- */
static uint8_t page_buf[1024];

/* ----------------------------------------------------------------------- */
/* uart_do_upload — UART page upload to base_addr                          */
/* force=1: use flash_write_page_any (allows bootloader region overwrite)  */
/* ----------------------------------------------------------------------- */
static void uart_do_upload(uint32_t base_addr, int force)
{
    /* Send 'S' to signal ready */
    usart_tx_byte('S');

    /* Receive page count (4 bytes LE) */
    uint8_t pc_buf[4];
    if (!usart_rx_buf(pc_buf, 4, 500)) jump_to_slot(g_config.boot_slot);
    uint32_t n_pages = (uint32_t)pc_buf[0]
                     | ((uint32_t)pc_buf[1] << 8)
                     | ((uint32_t)pc_buf[2] << 16)
                     | ((uint32_t)pc_buf[3] << 24);
    if (n_pages == 0 || n_pages > 246) jump_to_slot(g_config.boot_slot);

    flash_unlock();
    uint32_t flash_addr = base_addr;

    for (uint32_t p = 0; p < n_pages; p++) {
        uint8_t retry = 0;
page_retry:;
        if (!usart_rx_buf(page_buf, 1024, 5000)) {
            flash_lock();
            jump_to_slot(g_config.boot_slot);
        }
        uint8_t crc_bytes[4];
        if (!usart_rx_buf(crc_bytes, 4, 2000)) {
            flash_lock();
            jump_to_slot(g_config.boot_slot);
        }
        uint32_t master_crc = (uint32_t)crc_bytes[0]
                            | ((uint32_t)crc_bytes[1] << 8)
                            | ((uint32_t)crc_bytes[2] << 16)
                            | ((uint32_t)crc_bytes[3] << 24);

        uint32_t computed_crc = crc32_page(page_buf, 1024);
        if (computed_crc != master_crc) {
            usart_tx_byte('E');
            if (++retry < 8) goto page_retry;
            flash_lock();
            jump_to_slot(g_config.boot_slot);
        }

        if (force)
            flash_write_page_any(flash_addr, page_buf, 1024);
        else
            flash_write_page(flash_addr, page_buf, 1024);

        flash_addr += FLASH_PAGE_SIZE;
        usart_tx_byte('P');
    }

    flash_lock();
    usart_tx_byte('D');
    delay_ms(100);
    jump_to_slot(g_config.boot_slot);
}

/* ----------------------------------------------------------------------- */
/* can_upload — CAN page upload; uses g_upload_target (set by ctrl cmds)   */
/* ----------------------------------------------------------------------- */
static void can_upload(uint32_t base_addr)
{
    (void)base_addr;   /* ignored — g_upload_target set via ctrl channel */

    send_byte('S');

    uint8_t pc_buf[4];
    if (!recv_bytes(pc_buf, 4, 500)) jump_to_slot(g_config.boot_slot);
    uint32_t n_pages = (uint32_t)pc_buf[0]
                     | ((uint32_t)pc_buf[1] << 8)
                     | ((uint32_t)pc_buf[2] << 16)
                     | ((uint32_t)pc_buf[3] << 24);
    if (n_pages == 0 || n_pages > 246) jump_to_slot(g_config.boot_slot);

    flash_unlock();
    uint32_t flash_addr = g_upload_target;

    for (uint32_t p = 0; p < n_pages; p++) {
        uint8_t retry = 0;
can_page_retry:;
        uint32_t byte_idx = 0;
        uint32_t master_crc = 0;

        /* Receive 128 data frames (8 bytes each = 1024 bytes total) followed by
         * 1 CRC frame (4 bytes CRC + 4 bytes ignored).
         * Old protocol sent only 1020 bytes and embedded CRC in the last frame,
         * silently dropping the last 4 bytes of every 1024-byte flash page. */
        for (uint32_t f = 0; f < 129; f++) {
            uint16_t id;
            uint8_t  tmp[8], rlen;
            uint32_t polls = 2000U * 267U;
            int got = 0;
            while (polls--) {
                if (bxcan_rx(&id, tmp, &rlen) && id == CAN_RX_ID) { got = 1; break; }
            }
            if (!got) { flash_lock(); jump_to_slot(g_config.boot_slot); }

            if (f == 128) {
                /* Frame 128: CRC only (4 bytes LE) */
                master_crc = (uint32_t)tmp[0]
                           | ((uint32_t)tmp[1] << 8)
                           | ((uint32_t)tmp[2] << 16)
                           | ((uint32_t)tmp[3] << 24);
            } else {
                /* Frames 0-127: 8 bytes each = 1024 bytes of page data.
                 * CRITICAL: must NOT cast (1024-byte_idx) to uint8_t before the
                 * comparison — when byte_idx=0, (uint8_t)(1024-0)=(uint8_t)1024=0,
                 * making take=0 and copying nothing.  Keep remain as uint32_t. */
                uint32_t remain = 1024U - byte_idx;
                uint8_t  take   = (rlen < remain) ? rlen : (uint8_t)remain;
                for (uint8_t b = 0; b < take; b++)
                    page_buf[byte_idx++] = tmp[b];
            }
        }

        uint32_t computed_crc = crc32_page(page_buf, 1024);
        if (computed_crc != master_crc) {
            send_byte('E');
            if (++retry < 8) goto can_page_retry;
            flash_lock();
            jump_to_slot(g_config.boot_slot);
        }

        flash_write_page(flash_addr, page_buf, 1024);
        flash_addr += FLASH_PAGE_SIZE;
        send_byte('P');
    }

    flash_lock();
    /* Restore default upload target for any subsequent commands */
    g_upload_target      = SLOT_A_START;
    g_upload_target_slot = 0;
    send_byte('D');
    delay_ms(100);
    jump_to_slot(g_config.boot_slot);
}

/* ----------------------------------------------------------------------- */
/* uart_menu — interactive UART menu                                       */
/* ----------------------------------------------------------------------- */

/* Read one hex nibble from UART; returns 0..15, or 0xFF on non-hex input */
static uint8_t read_hex_nibble(void)
{
    uint8_t c = usart_rx_byte();
    usart_tx_byte(c);   /* echo */
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;
}

static void uart_menu(void)
{
    usart_print("\r\n");
    usart_print("===================================\r\n");
    usart_print("  biPropellant CAN Generic BL v1\r\n");
    /* Chip self-identification from DBGMCU_IDCODE + FLASHSIZE */
    {
        uint32_t idc      = DBGMCU_IDCODE;
        uint16_t dev_id   = (uint16_t)(idc & 0x0FFFU);
        uint16_t rev_id   = (uint16_t)(idc >> 16);
        uint16_t flash_kb = FLASHSIZE_REG;
        usart_print("  Chip DevID: 0x");
        usart_print_hex32((uint32_t)dev_id);   /* prints 8 digits; host strips leading zeros */
        usart_print("  Rev: 0x");
        usart_print_hex32((uint32_t)rev_id);
        usart_print("\r\n");
        usart_print("  Flash: ");
        usart_print_uint32((uint32_t)flash_kb);
        usart_print(" KB\r\n");
    }
    usart_print("  UID: ");
    usart_print_hex32(DESIG_UNIQUE_ID0);
    usart_print("-");
    usart_print_hex32(DESIG_UNIQUE_ID1);
    usart_print("-");
    usart_print_hex32(DESIG_UNIQUE_ID2);
    usart_print("\r\n");
    if (g_config.boot_slot == 1) {
        usart_print("  Active slot: B (0x08020000)\r\n");
        usart_print("  Next boot:   B\r\n");
    } else {
        usart_print("  Active slot: A (0x08002000)\r\n");
        usart_print("  Next boot:   A\r\n");
    }
    usart_print("===================================\r\n");
    usart_print(" [1] Flash -> Slot A  (primary,   0x08002000)\r\n");
    usart_print(" [2] Flash -> Slot B  (secondary, 0x08020000)\r\n");
    usart_print(" [3] Flash -> custom address\r\n");
    usart_print(" [4] Set next boot: Slot A\r\n");
    usart_print(" [5] Set next boot: Slot B\r\n");
    usart_print(" [6] Clear NVRAM (settings at 0x0803F000)\r\n");
    usart_print(" [7] Enter USB DFU (ROM bootloader, VID_0483:PID_DF11)\r\n");
    usart_print("     Use STM32CubeProgrammer or dfu-util on Windows/Linux\r\n");
    usart_print(" [0] Boot application\r\n");
    usart_print("===================================\r\n");
    usart_print("Choice [0-6]: ");

    uint8_t ch = usart_rx_byte();
    usart_print("\r\n");

    switch (ch) {
    case '0':
        jump_to_slot(g_config.boot_slot);
        break;

    case '1':
        usart_print("Ready. Send 0xAA to begin...\r\n");
        while (usart_rx_byte() != UART_TRIGGER)
            ;
        uart_do_upload(SLOT_A_START, 0);
        break;

    case '2':
        usart_print("Ready. Send 0xAA to begin...\r\n");
        while (usart_rx_byte() != UART_TRIGGER)
            ;
        uart_do_upload(SLOT_B_START, 0);
        usart_print("Set Slot B as next boot? (y/n): ");
        {
            uint8_t yn = usart_rx_byte();
            usart_print("\r\n");
            if (yn == 'y' || yn == 'Y') {
                g_config.boot_slot = 1;
                bl_config_write(&g_config);
                usart_print("Next boot: Slot B\r\n");
            }
        }
        jump_to_slot(g_config.boot_slot);
        break;

    case '3': {
        usart_print("Enter 8-digit hex address: ");
        uint32_t addr = 0;
        int valid = 1;
        for (int i = 0; i < 8; i++) {
            uint8_t nib = read_hex_nibble();
            if (nib == 0xFF) { valid = 0; break; }
            addr = (addr << 4) | nib;
        }
        usart_print("\r\n");
        if (!valid || addr == 0) {
            usart_print("Invalid address.\r\n");
            jump_to_slot(g_config.boot_slot);
        }
        int force = 0;
        if (addr < APP_FLASH_START) {
            usart_print("!!! BOOTLOADER REGION - overwrite bootloader? (YES/no): ");
            /* Expect "YES" followed by CR */
            uint8_t c0 = usart_rx_byte();
            uint8_t c1 = usart_rx_byte();
            uint8_t c2 = usart_rx_byte();
            uint8_t c3 = usart_rx_byte();   /* CR or \n */
            (void)c3;
            usart_print("\r\n");
            if (!((c0 == 'Y' || c0 == 'y') &&
                  (c1 == 'E' || c1 == 'e') &&
                  (c2 == 'S' || c2 == 's'))) {
                usart_print("Cancelled.\r\n");
                jump_to_slot(g_config.boot_slot);
            }
            force = 1;
        }
        usart_print("Ready. Send 0xAA to begin...\r\n");
        while (usart_rx_byte() != UART_TRIGGER)
            ;
        uart_do_upload(addr, force);
        break;
    }

    case '4':
        g_config.boot_slot = 0;
        bl_config_write(&g_config);
        usart_print("Next boot: Slot A\r\n");
        jump_to_slot(0);
        break;

    case '5':
        g_config.boot_slot = 1;
        bl_config_write(&g_config);
        usart_print("Next boot: Slot B\r\n");
        jump_to_slot(1);
        break;

    case '6':
        usart_print("Erasing NVRAM at 0x0803F000 (4KB)...\r\n");
        flash_unlock();
        flash_erase_region(APP_NVRAM_ADDR, APP_NVRAM_ADDR + 4096U - 1U);
        flash_lock();
        usart_print("Done. Boot and settings will reset to defaults.\r\nBooting...\r\n");
        delay_ms(500);
        jump_to_slot(g_config.boot_slot);
        break;

    case '7':
        /* Enter STM32 ROM DFU bootloader.
         * The device will enumerate as VID_0483:PID_DF11 on USB.
         * Windows: install STM32CubeProgrammer (recommended) or use Zadig
         *   to assign WinUSB to the device, then use dfu-util.
         * If the board has no USB connector this option does nothing. */
        usart_print("Entering USB DFU mode (VID_0483:PID_DF11)...\r\n");
        usart_print("Connect USB, open STM32CubeProgrammer, select USB DFU.\r\n");
        delay_ms(200);
        enter_rom_dfu();
        /* If enter_rom_dfu() returns (board has no USB or ROM BL is broken): */
        usart_print("ROM DFU unavailable on this board — using CAN/UART only.\r\n");
        jump_to_slot(g_config.boot_slot);
        break;

    default:
        usart_print("Invalid.\r\n");
        jump_to_slot(g_config.boot_slot);
        break;
    }
}

/* ----------------------------------------------------------------------- */
/* main — bootloader entry                                                  */
/* ----------------------------------------------------------------------- */
int main(void)
{
    /* 1. Disable all interrupts immediately */
    __asm volatile("cpsid i");
    NVIC_ICER0 = 0xFFFFFFFFUL;
    NVIC_ICER1 = 0xFFFFFFFFUL;
    NVIC_ICER2 = 0xFFFFFFFFUL;

    /* 1b. Check BKP_DR1 for "reboot-to-bootloader" magic written by the app.
     *
     * BKP registers survive NVIC_SystemReset() (they are in the backup power
     * domain) so the application can signal intent before resetting:
     *
     *   // Application side (any HAL or direct register access):
     *   __HAL_RCC_PWR_CLK_ENABLE();
     *   __HAL_RCC_BKP_CLK_ENABLE();
     *   HAL_PWR_EnableBkUpAccess();
     *   BKP->DR1 = 0xB001U;          // BKP_MAGIC_ENTER_BL
     *   NVIC_SystemReset();
     *
     * Or equivalently via the helper in the app: bootloader_request_reset().
     *
     * If the magic is found, the bootloader extends the transport-wait window
     * from 500 ms to 30 s so the upload tool has a comfortable window to connect
     * even if it takes a few seconds to start. */
    uint32_t poll_ms = 500U;          /* normal: 500 ms */
    {
        /* Enable PWR + BKP clocks and drop write-protection on backup domain */
        RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
        PWR_CR |= PWR_CR_DBP;

        if (BKP_DR1 == BKP_MAGIC_ENTER_BL) {
            BKP_DR1 = 0x0000U;        /* consume the flag */
            poll_ms = 30000U;         /* extended window: 30 s */
        }

        /* BKP_DR2 = 0xDFD0 → jump to STM32 ROM DFU bootloader.
         * Set by the application before reset to enter USB DFU on Windows.
         * This makes the device appear as VID_0483:PID_DF11 which Windows
         * recognises (with STM32CubeProgrammer / Zadig+dfu-util).
         * Must clear BKP first so a broken DFU doesn't loop into ROM BL. */
        if (BKP_DR2 == BKP_MAGIC_ENTER_DFU) {
            BKP_DR2 = 0x0000U;        /* consume — clear before jumping */
            enter_rom_dfu();
            /* If enter_rom_dfu() returns (invalid SP/PC), continue normally */
        }
    }

    /* 2. Read BL config; determine boot_slot */
    bl_config_read(&g_config);

    /* 3. Init both transports */
    bxcan_init(1);  /* OSM: no auto-retry in bootloader */   
    usart_init();

    /* ----------------------------------------------------------------
     * LED init: PC13, active-LOW (Blue Pill onboard LED).
     * Fast-blink during the bootloader window so the user can see the
     * chip is alive and listening.  The LED GPIO clock (IOPCEN) is on
     * APB2; we enable it here since mcp2515_init already enabled APB2.
     * ---------------------------------------------------------------- */
    RCC_APB2ENR |= (1U << 4);   /* IOPCEN */
    GPIOC->CRH  = (GPIOC->CRH & ~(0xFU << 20)) | (0x3U << 20); /* PC13 output PP 50MHz */
    GPIOC->BRR  = (1U << 13);   /* LED ON (active-low) immediately */

    /* ----------------------------------------------------------------
     * 4. Build the CAN hello frame.
     * Frame format on 0x7DE (standard, 8 bytes):
     *   [0] = '3' (0x33)          — stm32-CANBootloader compat marker
     *   [1] = '1' (0x31)          — protocol version 1
     *   [2..5] = UID0 (LE)        — unique chip identifier
     *   [6] = BL_VERSION_MAJOR    — firmware version so network knows which node woke
     *   [7] = BL_VERSION_MINOR
     *
     * With OSM (One-Shot Mode) set in bxcan_init(), each broadcast
     * attempt exits immediately if no ACK instead of retrying forever.
     * We retry manually every HELLO_INTERVAL_MS up to HELLO_MAX_TRIES
     * times; after that we boot the existing app even without a host.
     * ---------------------------------------------------------------- */
#define HELLO_INTERVAL_MS  50U   /* retry interval */
#define HELLO_MAX_TRIES    10U   /* 10 × 50ms = 500ms normal window */

    uint8_t advert[8];
    advert[0] = '3'; advert[1] = '1';
    uint32_t uid0 = DESIG_UNIQUE_ID0;
    advert[2] = (uint8_t)(uid0);
    advert[3] = (uint8_t)(uid0 >> 8);
    advert[4] = (uint8_t)(uid0 >> 16);
    advert[5] = (uint8_t)(uid0 >> 24);
    advert[6] = BL_VERSION_MAJOR;
    advert[7] = BL_VERSION_MINOR;

    /* First broadcast immediately; also send the 3-frame STATUS banner */
    bxcan_tx(CAN_TX_ID, advert, 8);
    bl_broadcast_status();

    /* 5. Poll both transports for poll_ms (500 ms normal / 30 s BKP-triggered).
     * Re-broadcast hello every HELLO_INTERVAL_MS, stop after HELLO_MAX_TRIES
     * with no response — then boot normally so a node with no CAN master
     * doesn't get stuck forever.
     * LED fast-blinks (toggle each hello) so the user can see the bootloader
     * is alive.
     */
    g_transport = TRANSPORT_NONE;
    uint8_t rx_buf[8];
    uint8_t rx_len;
    uint8_t uart_byte = 0;
    uint32_t last_hello_ms  = 0;
    uint32_t hello_count    = 0;   /* how many hellos sent so far */
    uint8_t  led_state      = 0;   /* current LED state (toggle per hello) */

    /* With BKP-extended window (30s): allow up to 600 hellos (30000/50).
     * Without BKP: cap at HELLO_MAX_TRIES so we don't wait indefinitely. */
    uint32_t max_hellos = (poll_ms > 500U) ? (poll_ms / HELLO_INTERVAL_MS) : HELLO_MAX_TRIES;

    for (uint32_t ms = 0; ms < poll_ms && g_transport == TRANSPORT_NONE; ms++) {
        /* Re-broadcast hello every HELLO_INTERVAL_MS */
        if (ms - last_hello_ms >= HELLO_INTERVAL_MS) {
            bxcan_tx(CAN_TX_ID, advert, 8);
            last_hello_ms = ms;
            hello_count++;

            /* Toggle LED to give visual feedback */
            led_state ^= 1U;
            if (led_state) GPIOC->BRR  = (1U << 13);   /* LED on  */
            else            GPIOC->BSRR = (1U << 13);  /* LED off */

            /* After max_hellos with no connection, give up and boot app */
            if (hello_count >= max_hellos && poll_ms <= 500U) {
                break;   /* fall through to jump_to_slot() below */
            }
        }
        /* Check UART first */
        if (usart_rx_ready()) {
            uart_byte = usart_rx_byte();
            if (uart_byte == UART_TRIGGER) {
                g_transport = TRANSPORT_UART;
            } else {
                /* Any other byte → interactive menu */
                g_transport = TRANSPORT_UART;
                uart_menu();  /* does not return */
            }
            break;
        }

        /* Check CAN: matching 0x7DD upload trigger or 0x7DC control command */
        uint16_t can_id;
        if (bxcan_rx(&can_id, rx_buf, &rx_len)) {
            if (can_id == CAN_RX_ID && rx_len >= 4) {
                /* Compare received 4 bytes against UID0 (not UID2 — UID0 is
                 * the value broadcast in the hello frame and returned by the
                 * host.  Using UID2 here was a mismatch: hello contains UID0
                 * so the host never had UID2 to send back. */
                uint32_t master_uid = (uint32_t)rx_buf[0]
                                    | ((uint32_t)rx_buf[1] << 8)
                                    | ((uint32_t)rx_buf[2] << 16)
                                    | ((uint32_t)rx_buf[3] << 24);
                if (master_uid == DESIG_UNIQUE_ID0) {
                    g_transport = TRANSPORT_CAN;
                    break;
                }
            } else if (can_id == BL_CTRL_RX_ID && rx_len >= 5) {
                /* Control command — handle in-place; loop continues polling */
                handle_ctrl_cmd(rx_buf, rx_len);
            }
        }

        /* ~1ms delay */
        volatile uint32_t c = 2667;
        while (c--)
            ;
    }

    if (g_transport == TRANSPORT_NONE) {
        /* No master arrived — boot based on config */
        jump_to_slot(g_config.boot_slot);
    }

    if (g_transport == TRANSPORT_CAN) {
        can_upload(SLOT_A_START);
    } else {
        /* UART 0xAA backwards-compat path: go straight to upload on Slot A */
        uart_do_upload(SLOT_A_START, 0);
    }

    return 0; /* unreachable */
}
