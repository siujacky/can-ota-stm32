#ifndef BXCAN_H
#define BXCAN_H
#include <stdint.h>

/* bxCAN app driver — PB8(CAN_RX) / PB9(CAN_TX), AFIO remap, 250kbps @ 8MHz.
 *
 * osm=0 → Standard Mode: automatic retransmission (normal CAN node behaviour).
 * osm=1 → One-Shot Mode:  single TX attempt, no retry on no-ACK.
 *
 * Standard Mode is recommended for normal application operation.
 * One-Shot Mode is used by the bootloader to prevent bus flooding during OTA. */
void    bxcan_app_init(uint32_t bps, int osm);
int     bxcan_app_tx(uint32_t id, const uint8_t *data, uint8_t len, int extended);
int     bxcan_app_rx(uint32_t *id_out, uint8_t *data_out, uint8_t *len_out, int *ext_out);
int     bxcan_app_rx_available(void);
void    bxcan_app_enter_normal(void);
void    bxcan_app_enter_listen(void);   /* true listen-only: SILM bit31 */
uint8_t bxcan_app_read_errors(void);   /* returns TEC */

#endif
