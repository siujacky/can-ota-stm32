/* bxcan.c — STM32F103 bxCAN application driver.
 *
 * Pin assignment (AFIO full remap, bits[14:13]=10):
 *   PB8 = CAN_RX  (AF input, floating)
 *   PB9 = CAN_TX  (AF output, push-pull 50 MHz)
 *
 * Bit timing — 250 kbps at 8 MHz HSI:
 *   BRP=1 (prescaler divides by 2) → t_q = 250 ns
 *   TS1=11 (12 t_q), TS2=2 (3 t_q), SJW=1 → 16 t_q/bit, 81.25% sample point
 *
 * Standard vs One-Shot mode (osm parameter of bxcan_app_init):
 *   osm=0  Standard: NART cleared — hardware retransmits on no-ACK.
 *   osm=1  One-Shot: NART set   — single TX attempt; ABTF flag raised on no-ACK.
 *          Used by the bootloader to prevent bus flooding during OTA.
 *
 * Listen-only mode (bxcan_app_enter_listen):
 *   Sets BTR bit31 (SILM).  CAN_TX is held recessive; no ACK is generated.
 *   This is TRUE bus monitoring.  Do not confuse with bit30 (LBKM = loopback).
 *
 * Filter configuration:
 *   Filter bank 0, 32-bit mask mode, FIFO0, FR1=FR2=0 → accepts all frames.
 */

#include "bxcan.h"
#include "device_regs.h"

/* -------------------------------------------------------------------------
 * Register map
 * ------------------------------------------------------------------------- */
#define CAN_BASE  0x40006400UL

typedef struct {
    volatile uint32_t MCR, MSR, TSR, RF0R, RF1R, IER, ESR, BTR;
    uint32_t _pad0[88];
    struct { volatile uint32_t TIR, TDTR, TDLR, TDHR; } TXB[3];
    struct { volatile uint32_t RIR, RDTR, RDLR, RDHR; } RXF[2];
    uint32_t _pad1[12];
    volatile uint32_t FMR, FM1R, _r2, FS1R, _r3, FFA1R, _r4, FA1R;
    uint32_t _pad2[8];
    struct { volatile uint32_t FR1, FR2; } FILTER[28];
} CAN_t;

#define CAN1  ((CAN_t *)CAN_BASE)

#define AFIO_BASE   0x40010000UL
#define AFIO_MAPR   (*(volatile uint32_t *)(AFIO_BASE + 0x04))

/* -------------------------------------------------------------------------
 * Bit definitions
 * ------------------------------------------------------------------------- */

/* MCR */
#define CAN_MCR_INRQ   (1U << 0)   /* Initialisation Request */
#define CAN_MCR_SLEEP  (1U << 1)   /* Sleep Mode Request */
#define CAN_MCR_NART   (1U << 4)   /* No Auto-ReTransmission (One-Shot) */

/* MSR */
#define CAN_MSR_INAK   (1U << 0)   /* Initialisation Acknowledge */

/* TSR */
#define CAN_TSR_TME0   (1U << 26)  /* Transmit Mailbox 0 Empty */

/* TIR / RIR shared layout */
#define CAN_TIR_TXRQ   (1U << 0)   /* Transmit Mailbox Request */
#define CAN_TIR_RTR    (1U << 1)   /* Remote Transmission Request */
#define CAN_TIR_IDE    (1U << 2)   /* Identifier Extension (1 = extended) */

/* BTR */
#define CAN_BTR_SILM   (1U << 31)  /* Silent Mode (listen-only); TX held recessive */
/* Note: bit30 is LBKM (loopback). SILM and LBKM are distinct; do not conflate. */

/* FMR */
#define CAN_FMR_FINIT  (1U << 0)   /* Filter Init Mode */

/* RF0R */
#define CAN_RF0R_FMP0  (3U << 0)   /* FIFO 0 Message Pending count */
#define CAN_RF0R_RFOM0 (1U << 5)   /* Release FIFO 0 Output Mailbox */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/* Enter initialisation mode and wait for INAK acknowledgement. */
static void _enter_init(void)
{
    CAN1->MCR |= CAN_MCR_INRQ;
    volatile uint32_t t = 100000;
    while (t-- && !(CAN1->MSR & CAN_MSR_INAK)) {}
}

/* Leave initialisation mode and wait until INAK is de-asserted. */
static void _leave_init(void)
{
    CAN1->MCR &= ~CAN_MCR_INRQ;
    volatile uint32_t t = 100000;
    while (t-- && (CAN1->MSR & CAN_MSR_INAK)) {}
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void bxcan_app_init(uint32_t bps, int osm)
{
    (void)bps;  /* Fixed at 250 kbps; extend for multi-rate if needed. */

    /* Enable peripheral clocks: AFIO, GPIOB (APB2); CAN (APB1 bit 25). */
    RCC_APB2ENR |= (1U << 0) | (1U << 3);
    RCC_APB1ENR |= (1U << 25);

    /* AFIO remap: CAN1 → PB8/PB9 (full remap, bits[14:13] = 0b10). */
    AFIO_MAPR = (AFIO_MAPR & ~(3U << 13)) | (2U << 13);

    /* GPIO:
     *   CRH bits[3:0]  → PB8: 0x4 = floating input
     *   CRH bits[7:4]  → PB9: 0xB = AF push-pull, 50 MHz
     */
    GPIOB->CRH = (GPIOB->CRH & ~(0xFFU)) | (0x4U) | (0xBU << 4);

    /* Wake from sleep, then enter init mode to configure BTR. */
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    _enter_init();

    /* One-Shot / Standard mode. */
    if (osm)
        CAN1->MCR |=  CAN_MCR_NART;
    else
        CAN1->MCR &= ~CAN_MCR_NART;

    /* Bit timing: SJW=1, TS2=3 (field=2), TS1=12 (field=11), BRP=2 (field=1).
     * → 250 kbps at 8 MHz. SILM and LBKM cleared (normal mode). */
    CAN1->BTR = (0U << 28) | (0U << 24) | (2U << 20) | (11U << 16) | (1U << 0);

    /* Filter bank 0: 32-bit mask mode, FIFO0, FR1=FR2=0 (pass all). */
    CAN1->FMR  |= CAN_FMR_FINIT;
    CAN1->FM1R  = 0;   /* mask mode for all banks */
    CAN1->FS1R  = 1;   /* 32-bit scale for bank 0 */
    CAN1->FFA1R = 0;   /* bank 0 → FIFO 0 */
    CAN1->FILTER[0].FR1 = 0;
    CAN1->FILTER[0].FR2 = 0;
    CAN1->FA1R  = 1;   /* activate bank 0 */
    CAN1->FMR  &= ~CAN_FMR_FINIT;

    _leave_init();
}

void bxcan_app_enter_normal(void)
{
    /* Re-enter init mode so BTR is writable, then clear SILM before leaving.
     * This correctly exits listen-only mode regardless of prior state. */
    _enter_init();
    CAN1->BTR &= ~CAN_BTR_SILM;
    _leave_init();
}

void bxcan_app_enter_listen(void)
{
    /* SILM (BTR bit31) = Silent Mode: CAN_TX held recessive, no ACK generated.
     * Do not confuse with LBKM (BTR bit30 = loopback, for self-test only). */
    _enter_init();
    CAN1->BTR |= CAN_BTR_SILM;
    _leave_init();
}

int bxcan_app_tx(uint32_t id, const uint8_t *data, uint8_t len, int extended)
{
    if (len > 8) len = 8;

    /* Wait for mailbox 0 to become empty. */
    volatile uint32_t t = 50000;
    while (!(CAN1->TSR & CAN_TSR_TME0) && --t) {}
    if (!t) return -1;

    /* Build TIR: RTR=0 (data frame). */
    uint32_t tir = extended ? ((id << 3) | CAN_TIR_IDE) : (id << 21);

    /* Load mailbox — write TIR last (without TXRQ) to arm the frame. */
    CAN1->TXB[0].TIR  = tir;
    CAN1->TXB[0].TDTR = len;

    uint32_t dL = 0, dH = 0;
    for (uint8_t i = 0; i < 4 && i < len; i++) dL |= ((uint32_t)data[i]     << (i * 8));
    for (uint8_t i = 4; i < 8 && i < len; i++) dH |= ((uint32_t)data[i] << ((i - 4) * 8));
    CAN1->TXB[0].TDLR = dL;
    CAN1->TXB[0].TDHR = dH;

    /* Setting TXRQ in TIR triggers transmission. */
    CAN1->TXB[0].TIR |= CAN_TIR_TXRQ;
    return 0;
}

int bxcan_app_rx_available(void)
{
    return (CAN1->RF0R & CAN_RF0R_FMP0) ? 1 : 0;
}

int bxcan_app_rx(uint32_t *id_out, uint8_t *data_out, uint8_t *len_out, int *ext_out)
{
    if (!(CAN1->RF0R & CAN_RF0R_FMP0)) return 0;

    uint32_t rir = CAN1->RXF[0].RIR;
    int ext = (rir & CAN_TIR_IDE) ? 1 : 0;

    if (ext_out) *ext_out = ext;
    if (id_out)  *id_out  = ext ? ((rir >> 3) & 0x1FFFFFFFU) : ((rir >> 21) & 0x7FFU);

    /* Clamp DLC before exposing length to caller to prevent buffer overruns. */
    uint8_t n = CAN1->RXF[0].RDTR & 0xFU;
    if (n > 8) n = 8;
    if (len_out) *len_out = n;

    if (data_out && n) {
        uint32_t dL = CAN1->RXF[0].RDLR;
        uint32_t dH = CAN1->RXF[0].RDHR;
        for (uint8_t i = 0; i < 4 && i < n; i++) data_out[i]     = (dL >> (i * 8))       & 0xFFU;
        for (uint8_t i = 4; i < 8 && i < n; i++) data_out[i] = (dH >> ((i - 4) * 8)) & 0xFFU;
    }

    /* Release FIFO0 slot so hardware can accept the next frame. */
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    return 1;
}

uint8_t bxcan_app_read_errors(void)
{
    /* ESR[23:16] = TEC (Transmit Error Counter). */
    return (uint8_t)((CAN1->ESR >> 16) & 0xFFU);
}
