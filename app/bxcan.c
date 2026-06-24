/* bxcan.c — STM32F103 bxCAN application driver.
 * PB8=CAN_RX (AF input floating), PB9=CAN_TX (AF PP 50MHz)
 * AFIO remap bits[14:13]=10.
 * 250kbps @ 8MHz HSI: BRP=1(div=2), TS1=11(12Tq), TS2=2(3Tq) → 16Tq, 81.25% SP.
 *
 * Standard vs One-Shot mode:
 *   osm=0 (Standard): MCP2515-equivalent default — auto-retry on no-ACK.
 *   osm=1 (One-Shot): single attempt; ABTF flag set on no-ACK. Used by bootloader.
 */
#include "bxcan.h"
#include "device_regs.h"

#define CAN_BASE  0x40006400UL
typedef struct {
    volatile uint32_t MCR,MSR,TSR,RF0R,RF1R,IER,ESR,BTR;
    uint32_t _r0[88];
    struct { volatile uint32_t TIR,TDTR,TDLR,TDHR; } TXB[3];
    struct { volatile uint32_t RIR,RDTR,RDLR,RDHR; } RXF[2];
    uint32_t _r1[12];
    volatile uint32_t FMR,FM1R,_r2,FS1R,_r3,FFA1R,_r4,FA1R;
    uint32_t _r5[8];
    struct { volatile uint32_t FR1,FR2; } FILTER[28];
} CAN_t;
#define CAN1 ((CAN_t*)CAN_BASE)
#define AFIO_BASE  0x40010000UL
#define AFIO_MAPR  (*(volatile uint32_t*)(AFIO_BASE+0x04))

#define CAN_MCR_INRQ   (1U<<0)
#define CAN_MCR_SLEEP  (1U<<1)
#define CAN_MCR_NART   (1U<<4)   /* No Auto-ReTransmission = One-Shot Mode */
#define CAN_MSR_INAK   (1U<<0)
#define CAN_TSR_TME0   (1U<<26)
#define CAN_TIR_TXRQ   (1U<<0)
#define CAN_TIR_RTR    (1U<<1)
#define CAN_TIR_IDE    (1U<<2)
#define CAN_FMR_FINIT  (1U<<0)
#define CAN_RF0R_FMP0  (3U<<0)
#define CAN_RF0R_RFOM0 (1U<<5)

static void _enter_init(void) {
    CAN1->MCR |= CAN_MCR_INRQ;
    volatile uint32_t t = 100000; while (t-- && !(CAN1->MSR & CAN_MSR_INAK));
}
static void _leave_init(void) {
    CAN1->MCR &= ~CAN_MCR_INRQ;
    volatile uint32_t t = 100000; while (t-- && (CAN1->MSR & CAN_MSR_INAK));
}

void bxcan_app_init(uint32_t bps, int osm) {
    (void)bps;   /* fixed at 250kbps; extend for multi-rate if needed */
    RCC_APB2ENR |= (1U<<0)|(1U<<3);   /* AFIO + GPIOB */
    RCC_APB1ENR |= (1U<<25);           /* CAN */
    AFIO_MAPR = (AFIO_MAPR & ~(3U<<13)) | (2U<<13);

    GPIOB->CRH = (GPIOB->CRH & ~(0xFFU)) | (0x4U)|(0xBU<<4);  /* PB8=float-in, PB9=AF-PP */

    CAN1->MCR &= ~CAN_MCR_SLEEP;
    _enter_init();
    if (osm) CAN1->MCR |= CAN_MCR_NART;   /* One-Shot */
    else     CAN1->MCR &= ~CAN_MCR_NART;  /* Standard: auto-retry */
    /* 250kbps @ 8MHz */
    CAN1->BTR = (0U<<28)|(0U<<24)|(2U<<20)|(11U<<16)|(1U<<0);

    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FM1R=0; CAN1->FS1R=1; CAN1->FFA1R=0;
    CAN1->FILTER[0].FR1=0; CAN1->FILTER[0].FR2=0;
    CAN1->FA1R=1;
    CAN1->FMR &= ~CAN_FMR_FINIT;
    _leave_init();
}

void bxcan_app_enter_normal(void) { _enter_init(); _leave_init(); }

void bxcan_app_enter_listen(void) {
    /* SILM = bit31 of BTR = Silent Mode: CAN_TX stays recessive, no ACK generated.
     * This is TRUE listen-only. bit30 (LBKM) is loop-back — do NOT confuse them. */
    _enter_init();
    CAN1->BTR |= (1U<<31);   /* SILM */
    _leave_init();
}

int bxcan_app_tx(uint32_t id, const uint8_t *data, uint8_t len, int extended) {
    if (len > 8) len = 8;
    volatile uint32_t t = 50000;
    while (!(CAN1->TSR & CAN_TSR_TME0) && --t);
    if (!t) return -1;
    uint32_t tir = extended ? ((id<<3)|CAN_TIR_IDE) : (id<<21);
    CAN1->TXB[0].TIR  = tir;
    CAN1->TXB[0].TDTR = len;
    uint32_t dL=0,dH=0;
    for (uint8_t i=0;i<4&&i<len;i++) dL|=((uint32_t)data[i]<<(i*8));
    for (uint8_t i=4;i<8&&i<len;i++) dH|=((uint32_t)data[i]<<((i-4)*8));
    CAN1->TXB[0].TDLR=dL; CAN1->TXB[0].TDHR=dH;
    CAN1->TXB[0].TIR |= CAN_TIR_TXRQ;
    return 0;
}

int bxcan_app_rx_available(void) {
    return (CAN1->RF0R & CAN_RF0R_FMP0) ? 1 : 0;
}

int bxcan_app_rx(uint32_t *id_out, uint8_t *data_out, uint8_t *len_out, int *ext_out) {
    if (!(CAN1->RF0R & CAN_RF0R_FMP0)) return 0;
    uint32_t rir = CAN1->RXF[0].RIR;
    int ext = (rir & CAN_TIR_IDE) ? 1 : 0;
    if (ext_out) *ext_out = ext;
    if (id_out)  *id_out  = ext ? (rir>>3)&0x1FFFFFFFU : (rir>>21)&0x7FFU;
    uint8_t n = CAN1->RXF[0].RDTR & 0xFU;
    if (n > 8) n = 8;            /* clamp DLC before exposing to caller */
    if (len_out) *len_out = n;
    if (data_out && n) {
        uint32_t dL=CAN1->RXF[0].RDLR, dH=CAN1->RXF[0].RDHR;
        for (uint8_t i=0;i<4&&i<n;i++) data_out[i]=(dL>>(i*8))&0xFF;
        for (uint8_t i=4;i<8&&i<n;i++) data_out[i]=(dH>>((i-4)*8))&0xFF;
    }
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    return 1;
}

uint8_t bxcan_app_read_errors(void) {
    return (uint8_t)((CAN1->ESR >> 16) & 0xFFU);  /* TEC */
}
