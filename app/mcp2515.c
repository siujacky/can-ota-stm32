/* mcp2515.c — MCP2515 hardware SPI1 driver for Blue Pill CAN Tool.
 * Pins (hardware SPI1): PA4=CS PA5=SCK PA6=MISO PA7=MOSI PB0=INT
 * Bitrate table corrected for 8MHz crystal (verified mathematically).
 * NOTE: PA4-PA7 are NOT 5V-tolerant. Power MCP2515 module from 3.3V.
 */

#include "mcp2515.h"
#include "device_regs.h"

#define CS_HIGH()   GPIOA->BSRR = (1U << 4)
#define CS_LOW()    GPIOA->BSRR = (1U << (4 + 16))
#define INT_READ()  ((GPIOB->IDR >> 0) & 1U)   /* PB0, active-low */

/* ~1ms delay at 8MHz HSI */
static void delay_ms(uint32_t ms)
{
    while (ms--) { volatile uint32_t c = 2667; while (c--); }
}

/* -----------------------------------------------------------------------
 * Hardware SPI1 init + GPIO configuration
 * ----------------------------------------------------------------------- */
static void gpio_spi_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* PA4 (CS)   — GPIO output push-pull 50 MHz: CRL bits[19:16] */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 16)) | (GPIO_OUT_PP_50 << 16);
    /* PA5 (SCK)  — SPI1_SCK  AF push-pull 50 MHz: CRL bits[23:20] */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 20)) | (GPIO_AF_PP_50  << 20);
    /* PA6 (MISO) — SPI1_MISO floating input: CRL bits[27:24] */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 24)) | (GPIO_INPUT_FLOAT << 24);
    /* PA7 (MOSI) — SPI1_MOSI AF push-pull 50 MHz: CRL bits[31:28] */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 28)) | (GPIO_AF_PP_50  << 28);
    /* PB0 (INT)  — input pull-up: CRL bits[3:0] = 0x8 */
    GPIOB->CRL = (GPIOB->CRL & ~(0xFU << 0)) | (0x8U << 0);
    GPIOB->BSRR = (1U << 0);

    CS_HIGH();

    /* SPI1: SSM=1, SSI=1, SPE=1, BR=001 (2MHz), MSTR=1, Mode-0 */
    SPI1_CR1 = SPI1_CR1_SSM | SPI1_CR1_SSI | SPI1_CR1_SPE |
               SPI1_CR1_BR_DIV4 | SPI1_CR1_MSTR;
}

void mcp2515_release_pins(void)
{
    SPI1_CR1 &= ~SPI1_CR1_SPE;
    CS_HIGH();
    GPIOA->CRL = (GPIOA->CRL & ~(0xFFFFU << 16))
               | (GPIO_INPUT_FLOAT << 16) | (GPIO_INPUT_FLOAT << 20)
               | (GPIO_INPUT_FLOAT << 24) | (GPIO_INPUT_FLOAT << 28);
    GPIOB->CRL = (GPIOB->CRL & ~(0xFU << 0)) | (GPIO_INPUT_FLOAT << 0);
}

void mcp2515_restore_pins(void) { gpio_spi_init(); }

/* -----------------------------------------------------------------------
 * Hardware SPI1 byte transfer — mode 0, MSB first
 * ----------------------------------------------------------------------- */
static uint8_t spi_xfer(uint8_t byte)
{
    if (SPI1_SR & SPI1_SR_RXNE) { (void)SPI1_DR; }
    while (!(SPI1_SR & SPI1_SR_TXE));
    SPI1_DR = byte;
    while (!(SPI1_SR & SPI1_SR_RXNE));
    return (uint8_t)SPI1_DR;
}

/* -----------------------------------------------------------------------
 * MCP2515 register read/write
 * ----------------------------------------------------------------------- */
static void mcp_write_reg(uint8_t addr, uint8_t val)
{
    CS_LOW();
    spi_xfer(MCP_WRITE);
    spi_xfer(addr);
    spi_xfer(val);
    CS_HIGH();
}

static uint8_t mcp_read_reg(uint8_t addr)
{
    CS_LOW();
    spi_xfer(MCP_READ);
    spi_xfer(addr);
    uint8_t val = spi_xfer(0x00);
    CS_HIGH();
    return val;
}

/* Bit-modify: set/clear specific bits in a register */
static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    CS_LOW();
    spi_xfer(MCP_BIT_MODIFY);
    spi_xfer(addr);
    spi_xfer(mask);
    spi_xfer(data);
    CS_HIGH();
}

/* -----------------------------------------------------------------------
 * Bitrate CNF table for 8 MHz crystal.
 * Values from MCP2515 datasheet Table 5-3 (8 MHz oscillator).
 * S4=125k, S5=250k, S6=500k (default), S7=800k, S8=1Mbps
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} bitrate_cfg_t;

/* CNF values for 8 MHz crystal — verified mathematically:
 * TQ=2*(BRP+1)/Fosc; bit time=TQ×(1+PropSeg+PS1+PS2).
 * All entries use BRP=0 (TQ=250ns) with 16TQ/bit = 4µs = 250kbps base.
 * S4=125k: BRP=1(TQ=500ns), 16TQ → 8µs = 125kbps.
 * S5=250k: BRP=0(TQ=250ns), 16TQ → 4µs = 250kbps (matches Pico 2 SLCAN).
 * S6=500k: BRP=0(TQ=250ns),  8TQ → 2µs = 500kbps.
 */
static const bitrate_cfg_t bitrate_table[5] = {
    { 0x01, 0x9E, 0x03 },  /* [0] S4 = 125 kbps  BRP=1,TQ=500ns, 16TQ */
    { 0x00, 0x9E, 0x03 },  /* [1] S5 = 250 kbps  BRP=0,TQ=250ns, 16TQ ← default */
    { 0x00, 0x90, 0x02 },  /* [2] S6 = 500 kbps  BRP=0,TQ=250ns,  8TQ */
    { 0x00, 0x86, 0x02 },  /* [3] S7 = 800 kbps  (approximate) */
    { 0x00, 0x80, 0x00 },  /* [4] S8 = 1 Mbps    BRP=0,TQ=250ns,  4TQ */
};

/* Map S-command (4-8) to table index */
static int brate_index(uint8_t s_cmd)
{
    if (s_cmd >= 4 && s_cmd <= 8)
        return (int)(s_cmd - 4);
    return 2; /* default: 500k */
}

/* Apply CNF registers in config mode */
static void apply_bitrate(int idx)
{
    mcp_write_reg(MCP_CNF1, bitrate_table[idx].cnf1);
    mcp_write_reg(MCP_CNF2, bitrate_table[idx].cnf2);
    mcp_write_reg(MCP_CNF3, bitrate_table[idx].cnf3);
}

/* -----------------------------------------------------------------------
 * mcp2515_set_bitrate — switch bitrate at runtime.
 * s_cmd can be the integer value 4-8 or the ASCII digit '4'-'8'.
 * The caller must have already called mcp2515_enter_listen_only or similar
 * to ensure bus is not active — we handle the config mode transition here.
 * ----------------------------------------------------------------------- */
void mcp2515_set_bitrate(uint8_t s_cmd)
{
    /* Accept either integer 4-8 or ASCII '4'-'8' */
    uint8_t val = s_cmd;
    if (val >= '4' && val <= '8')
        val = val - '0';

    int idx = brate_index(val);

    /* Enter config mode */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_CONFIG);
    delay_ms(2);

    apply_bitrate(idx);

    /* Return to listen-only (caller decides when to open) */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_LISTEN_ONLY);
    delay_ms(1);
}

/* -----------------------------------------------------------------------
 * mcp2515_init — full initialisation sequence
 * brate = MCP_BRATE_xxx (4=125k 5=250k 6=500k 7=800k 8=1M)
 * After init: bus is in listen-only mode (SLCAN "closed" state).
 * User must call mcp2515_enter_normal() to open the bus.
 * ----------------------------------------------------------------------- */
void mcp2515_init(uint8_t brate)
{
    gpio_spi_init();

    /* Hardware reset via SPI */
    CS_LOW();
    spi_xfer(MCP_RESET);
    CS_HIGH();
    delay_ms(10);

    /* Now in config mode (CANSTAT = 0x80) */
    int idx = brate_index(brate);
    apply_bitrate(idx);

    /* Accept all frames: RXB0CTRL = 0x64
     * RXM[1:0]=11 (accept all, no filter), BUKT=1 (rollover to RXB1 if RXB0 full) */
    mcp_write_reg(MCP_RXB0CTRL, 0x64);

    /* RXB1CTRL: accept all (RXM=11), no rollover needed */
    mcp_write_reg(MCP_RXB1CTRL, 0x60);

    /* Disable all MCP2515 interrupts — we poll CANINTF directly */
    mcp_write_reg(MCP_CANINTE, 0x00);

    /* Clear any pending interrupt flags */
    mcp_write_reg(MCP_CANINTF, 0x00);

    /* Start in listen-only mode (SLCAN bus is "closed" until 'O' command) */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_LISTEN_ONLY);
    delay_ms(1);
}

/* -----------------------------------------------------------------------
 * mcp2515_enter_normal — switch to normal (transmit+receive) mode
 * Returns 0 on success, -1 if mode switch failed.
 * ----------------------------------------------------------------------- */
int mcp2515_enter_normal(void)
{
    /* Two-step: enter normal mode first (0x00 = all bits LOW → reliable at any
     * VCC), then set OSM (bit 3 = 0x08) in a second write.  Without OSM a
     * failed TX retries indefinitely → bus-off.  mcp_bit_modify (0x05) is used
     * for the OSM step because its instruction byte has MSB=0 (reliable). */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_NORMAL);
    delay_ms(2);
    mcp_bit_modify(MCP_CANCTRL, 0x08U, 0x08U);  /* set OSM */
    delay_ms(1);
    uint8_t stat = mcp_read_reg(MCP_CANSTAT);
    return ((stat & MCP_MODE_MASK) == MCP_MODE_NORMAL) ? 0 : -1;
}

/* -----------------------------------------------------------------------
 * mcp2515_enter_listen_only — switch to listen-only mode (no TX)
 * Returns 0 on success.
 * ----------------------------------------------------------------------- */
int mcp2515_enter_listen_only(void)
{
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_LISTEN_ONLY);
    delay_ms(2);
    uint8_t stat = mcp_read_reg(MCP_CANSTAT);
    return ((stat & MCP_MODE_MASK) == MCP_MODE_LISTEN_ONLY) ? 0 : -1;
}

/* -----------------------------------------------------------------------
 * mcp2515_tx — transmit a CAN frame (standard or extended)
 * id: 11-bit for standard, 29-bit for extended
 * Returns 0 on success, -1 on timeout/error.
 * ----------------------------------------------------------------------- */
int mcp2515_tx(uint32_t id, const uint8_t *data, uint8_t len, int extended)
{
    if (len > 8) len = 8;

    /* Wait for TXB0 to be free (TXREQ=0) */
    uint32_t timeout = 10000;
    while ((mcp_read_reg(MCP_TXB0CTRL) & MCP_TXCTRL_TXREQ) && --timeout);
    if (!timeout) return -1;

    if (extended) {
        /* Extended frame: 29-bit ID split as SIDH[10:3] SIDL[2:0|EXIDE|EID17:16] EID8[15:8] EID0[7:0] */
        uint8_t sidh = (uint8_t)((id >> 21) & 0xFF);
        /* SIDL: SID[2:0] in bits[7:5], EXIDE=1 in bit3, EID[17:16] in bits[1:0] */
        uint8_t sidl = (uint8_t)(((id >> 18) & 0x07) << 5) | MCP_TXSIDL_EXIDE | (uint8_t)((id >> 16) & 0x03);
        uint8_t eid8 = (uint8_t)((id >> 8) & 0xFF);
        uint8_t eid0 = (uint8_t)(id & 0xFF);

        mcp_write_reg(MCP_TXB0SIDH, sidh);
        mcp_write_reg(MCP_TXB0SIDL, sidl);
        mcp_write_reg(MCP_TXB0EID8, eid8);
        mcp_write_reg(MCP_TXB0EID0, eid0);
    } else {
        /* Standard 11-bit: SIDH=id[10:3], SIDL=id[2:0]<<5 */
        mcp_write_reg(MCP_TXB0SIDH, (uint8_t)(id >> 3));
        mcp_write_reg(MCP_TXB0SIDL, (uint8_t)((id & 0x7U) << 5));
        mcp_write_reg(MCP_TXB0EID8, 0x00);
        mcp_write_reg(MCP_TXB0EID0, 0x00);
    }

    mcp_write_reg(MCP_TXB0DLC, len & 0x0F);

    for (uint8_t i = 0; i < len; i++)
        mcp_write_reg(MCP_TXB0D0 + i, data[i]);

    /* Request transmit via register write (avoids RTS instruction 0x81 whose
     * MSB=1 may not be recognised at 5V VCC with 3.3V MOSI push-pull). */
    mcp_write_reg(MCP_TXB0CTRL, MCP_TXCTRL_TXREQ);

    /* Wait for TX complete (TXREQ clears) with timeout */
    timeout = 50000;
    while ((mcp_read_reg(MCP_TXB0CTRL) & MCP_TXCTRL_TXREQ) && --timeout);

    /* Check for TX error */
    uint8_t txctrl = mcp_read_reg(MCP_TXB0CTRL);
    if (txctrl & (MCP_TXCTRL_ABTF | MCP_TXCTRL_MLOA | MCP_TXCTRL_TXERR))
        return -1;

    return timeout ? 0 : -1;
}

/* -----------------------------------------------------------------------
 * mcp2515_tx_rtr — transmit a Remote Transmission Request frame
 * ----------------------------------------------------------------------- */
int mcp2515_tx_rtr(uint32_t id, uint8_t dlc, int extended)
{
    if (dlc > 8) dlc = 8;

    uint32_t timeout = 10000;
    while ((mcp_read_reg(MCP_TXB0CTRL) & MCP_TXCTRL_TXREQ) && --timeout);
    if (!timeout) return -1;

    if (extended) {
        uint8_t sidh = (uint8_t)((id >> 21) & 0xFF);
        uint8_t sidl = (uint8_t)(((id >> 18) & 0x07) << 5) | MCP_TXSIDL_EXIDE | (uint8_t)((id >> 16) & 0x03);
        mcp_write_reg(MCP_TXB0SIDH, sidh);
        mcp_write_reg(MCP_TXB0SIDL, sidl);
        mcp_write_reg(MCP_TXB0EID8, (uint8_t)((id >> 8) & 0xFF));
        mcp_write_reg(MCP_TXB0EID0, (uint8_t)(id & 0xFF));
    } else {
        mcp_write_reg(MCP_TXB0SIDH, (uint8_t)(id >> 3));
        mcp_write_reg(MCP_TXB0SIDL, (uint8_t)((id & 0x7U) << 5));
        mcp_write_reg(MCP_TXB0EID8, 0x00);
        mcp_write_reg(MCP_TXB0EID0, 0x00);
    }

    /* DLC with RTR bit set */
    mcp_write_reg(MCP_TXB0DLC, (dlc & 0x0F) | MCP_DLC_RTR);

    mcp_write_reg(MCP_TXB0CTRL, MCP_TXCTRL_TXREQ);  /* set TXREQ via WRITE (MSB=0) */

    timeout = 50000;
    while ((mcp_read_reg(MCP_TXB0CTRL) & MCP_TXCTRL_TXREQ) && --timeout);
    /* Check error flags (ABTF|MLOA|TXERR) — same as mcp2515_tx() */
    uint8_t txctrl = mcp_read_reg(MCP_TXB0CTRL);
    if (txctrl & (MCP_TXCTRL_ABTF | MCP_TXCTRL_MLOA | MCP_TXCTRL_TXERR))
        return -1;
    return timeout ? 0 : -1;
}

/* -----------------------------------------------------------------------
 * mcp2515_rx_available — returns 1 if RXB0 or RXB1 has a frame
 * ----------------------------------------------------------------------- */
int mcp2515_rx_available(void)
{
    uint8_t intf = mcp_read_reg(MCP_CANINTF);
    return (intf & (MCP_CANINTF_RX0IF | MCP_CANINTF_RX1IF)) ? 1 : 0;
}

/* -----------------------------------------------------------------------
 * mcp2515_rx — read one frame from RXB0 (or RXB1 if RXB0 empty)
 * Populates id_out, data_out, len_out, ext_out (1=extended).
 * Returns 1 on success, 0 if no frame.
 * ----------------------------------------------------------------------- */
int mcp2515_rx(uint32_t *id_out, uint8_t *data_out, uint8_t *len_out, int *ext_out)
{
    uint8_t intf = mcp_read_reg(MCP_CANINTF);
    uint8_t base_sidh;
    uint8_t clear_flag;

    if (intf & MCP_CANINTF_RX0IF) {
        base_sidh = MCP_RXB0SIDH;
        clear_flag = MCP_CANINTF_RX0IF;
    } else if (intf & MCP_CANINTF_RX1IF) {
        base_sidh = MCP_RXB1SIDH;
        clear_flag = MCP_CANINTF_RX1IF;
    } else {
        return 0;
    }

    uint8_t sidh = mcp_read_reg(base_sidh);
    uint8_t sidl = mcp_read_reg(base_sidh + 1);
    uint8_t eid8 = mcp_read_reg(base_sidh + 2);
    uint8_t eid0 = mcp_read_reg(base_sidh + 3);
    uint8_t dlc  = mcp_read_reg(base_sidh + 4) & 0x0F;

    int is_ext = (sidl & MCP_RXSIDL_IDE) ? 1 : 0;

    if (id_out) {
        if (is_ext) {
            /* Reassemble 29-bit: SID[10:3]=sidh, SID[2:0]=sidl[7:5],
             * EID[17:16]=sidl[1:0], EID[15:8]=eid8, EID[7:0]=eid0 */
            uint32_t sid = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
            uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
            *id_out = (sid << 18) | eid;
        } else {
            *id_out = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
        }
    }
    if (ext_out) *ext_out = is_ext;
    if (len_out) *len_out = dlc;

    if (data_out && dlc > 0) {
        uint8_t n = dlc;
        if (n > 8) n = 8;
        for (uint8_t i = 0; i < n; i++)
            data_out[i] = mcp_read_reg(base_sidh + 5 + i);
    }

    /* Clear the RX interrupt flag to release the buffer */
    mcp_bit_modify(MCP_CANINTF, clear_flag, 0x00);

    return 1;
}

/* -----------------------------------------------------------------------
 * mcp2515_read_errors — read EFLG register and clear error interrupt flags.
 * Returns EFLG value. Clears CANINTF error bits (ERRIF, MERRF).
 * ----------------------------------------------------------------------- */
uint8_t mcp2515_read_errors(void)
{
    uint8_t eflg = mcp_read_reg(MCP_EFLG_REG);
    /* Clear error interrupt flags */
    mcp_bit_modify(MCP_CANINTF,
                   MCP_CANINTF_ERRIF | MCP_CANINTF_MERRF | MCP_CANINTF_WAKIF,
                   0x00);
    return eflg;
}
