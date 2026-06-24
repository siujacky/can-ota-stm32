#ifndef BXCAN_BL_H
#define BXCAN_BL_H
#include <stdint.h>

/* bxCAN on PB8(CAN_RX) / PB9(CAN_TX), AFIO remap bits[14:13]=10.
 * 250kbps @ 8MHz HSI: BRP=1(div=2), TS1=11(12Tq), TS2=2(3Tq) → 16Tq, 81.25% SP
 *
 * osm=1 → One-Shot Mode: single TX attempt, no retransmission on no-ACK.
 *          Good for OTA bootloader (host controls retries at protocol level).
 * osm=0 → Standard Mode: automatic retransmission until ACK or bus-off.
 *          Better for normal CAN node operation. */
void bxcan_init(int osm);     /* osm: 1=no-retry, 0=auto-retry */
int  bxcan_tx(uint16_t id, const uint8_t *data, uint8_t len);
int  bxcan_rx(uint16_t *id_out, uint8_t *data_out, uint8_t *len_out);

#endif
