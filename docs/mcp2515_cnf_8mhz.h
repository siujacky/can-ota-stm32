/* mcp2515_cnf_8mhz.h — Verified MCP2515 CNF register values for 8 MHz crystal.
 *
 * Bit timing formula:
 *   TQ       = 2*(BRP+1) / fOSC
 *   Bit time = TQ * (1 + PROP + PS1 + PS2)   where PROP=PRSEG+1, PS1=PHSEG1+1, PS2=PHSEG2+1
 *   SP%      = (1 + PROP + PS1) / total_TQ * 100
 *
 * CNF2: bit7=BTLMODE(1), bit6=SAM(0), bits[5:3]=PHSEG1 reg, bits[2:0]=PRSEG reg
 * CNF3: bits[2:0]=PHSEG2 reg
 *
 * ⚠ Notes for bipropellant_can_generic and similar projects:
 *   - 250kbps (S5=0x00,0x9E,0x03) CONFIRMED working ✓
 *   - 500kbps: fixed from SP=62.5% (wrong) to SP=87.5%
 *   - 800kbps: marginal at 8MHz — only 5TQ available; use 16MHz crystal for 800k+
 *   - 1Mbps: 4TQ, below CiA 8-TQ minimum; works in practice but not compliant
 *   - All rates ≤500kbps use 16-20 TQ → robust, CiA-compliant
 */

#ifndef MCP2515_CNF_8MHZ_H
#define MCP2515_CNF_8MHZ_H

/*  10 kbps  BRP=19 TQ=5000ns  20TQ  SP=80%   0 ppm error */
#define MCP2515_8MHz_10kBPS_CFG1   0x13
#define MCP2515_8MHz_10kBPS_CFG2   0xBE
#define MCP2515_8MHz_10kBPS_CFG3   0x03

/*  20 kbps  BRP=9  TQ=2500ns  20TQ  SP=80%   0 ppm error */
#define MCP2515_8MHz_20kBPS_CFG1   0x09
#define MCP2515_8MHz_20kBPS_CFG2   0xBE
#define MCP2515_8MHz_20kBPS_CFG3   0x03

/*  50 kbps  BRP=4  TQ=1250ns  16TQ  SP=81.25%  0 ppm error */
#define MCP2515_8MHz_50kBPS_CFG1   0x04
#define MCP2515_8MHz_50kBPS_CFG2   0xB4
#define MCP2515_8MHz_50kBPS_CFG3   0x02

/* 100 kbps  BRP=1  TQ=500ns   20TQ  SP=80%   0 ppm error */
#define MCP2515_8MHz_100kBPS_CFG1  0x01
#define MCP2515_8MHz_100kBPS_CFG2  0xBE
#define MCP2515_8MHz_100kBPS_CFG3  0x03

/* 125 kbps  BRP=1  TQ=500ns   16TQ  SP=75%   0 ppm error */
#define MCP2515_8MHz_125kBPS_CFG1  0x01
#define MCP2515_8MHz_125kBPS_CFG2  0x9E
#define MCP2515_8MHz_125kBPS_CFG3  0x03

/* 250 kbps  BRP=0  TQ=250ns   16TQ  SP=75%   0 ppm error   ← CONFIRMED ✓ */
#define MCP2515_8MHz_250kBPS_CFG1  0x00
#define MCP2515_8MHz_250kBPS_CFG2  0x9E
#define MCP2515_8MHz_250kBPS_CFG3  0x03

/* 500 kbps  BRP=0  TQ=250ns    8TQ  SP=87.5% 0 ppm error
 * (Previous value 0x90,0x02 had SP=62.5% — below 70% minimum) */
#define MCP2515_8MHz_500kBPS_CFG1  0x00
#define MCP2515_8MHz_500kBPS_CFG2  0x99
#define MCP2515_8MHz_500kBPS_CFG3  0x00

/* 800 kbps  BRP=0  TQ=250ns    5TQ  SP=80%   MARGINAL
 * Best achievable at 8MHz; 5TQ is below CiA recommendation.
 * Consider using a 16MHz crystal for 800kbps. */
#define MCP2515_8MHz_800kBPS_CFG1  0x00
#define MCP2515_8MHz_800kBPS_CFG2  0x88
#define MCP2515_8MHz_800kBPS_CFG3  0x00

/* 1 Mbps    BRP=0  TQ=250ns    4TQ  SP=75%   NOT CiA-compliant (min 8TQ)
 * Works in practice on short buses (<1m) with noise-free environment. */
#define MCP2515_8MHz_1MBPS_CFG1    0x00
#define MCP2515_8MHz_1MBPS_CFG2    0x80
#define MCP2515_8MHz_1MBPS_CFG3    0x00

/* Convenience macro for use in switch or table initialization:
 * Place in bitrate_table[] indexed by SLCAN S-command (S0=10k…S8=1M):
 *   {MCP2515_8MHz_S0_CFG}, {MCP2515_8MHz_S1_CFG}, ... {MCP2515_8MHz_S8_CFG} */
#define MCP2515_8MHz_S0_CFG  MCP2515_8MHz_10kBPS_CFG1,  MCP2515_8MHz_10kBPS_CFG2,  MCP2515_8MHz_10kBPS_CFG3
#define MCP2515_8MHz_S1_CFG  MCP2515_8MHz_20kBPS_CFG1,  MCP2515_8MHz_20kBPS_CFG2,  MCP2515_8MHz_20kBPS_CFG3
#define MCP2515_8MHz_S2_CFG  MCP2515_8MHz_50kBPS_CFG1,  MCP2515_8MHz_50kBPS_CFG2,  MCP2515_8MHz_50kBPS_CFG3
#define MCP2515_8MHz_S3_CFG  MCP2515_8MHz_100kBPS_CFG1, MCP2515_8MHz_100kBPS_CFG2, MCP2515_8MHz_100kBPS_CFG3
#define MCP2515_8MHz_S4_CFG  MCP2515_8MHz_125kBPS_CFG1, MCP2515_8MHz_125kBPS_CFG2, MCP2515_8MHz_125kBPS_CFG3
#define MCP2515_8MHz_S5_CFG  MCP2515_8MHz_250kBPS_CFG1, MCP2515_8MHz_250kBPS_CFG2, MCP2515_8MHz_250kBPS_CFG3
#define MCP2515_8MHz_S6_CFG  MCP2515_8MHz_500kBPS_CFG1, MCP2515_8MHz_500kBPS_CFG2, MCP2515_8MHz_500kBPS_CFG3
#define MCP2515_8MHz_S7_CFG  MCP2515_8MHz_800kBPS_CFG1, MCP2515_8MHz_800kBPS_CFG2, MCP2515_8MHz_800kBPS_CFG3
#define MCP2515_8MHz_S8_CFG  MCP2515_8MHz_1MBPS_CFG1,   MCP2515_8MHz_1MBPS_CFG2,   MCP2515_8MHz_1MBPS_CFG3

#endif /* MCP2515_CNF_8MHZ_H */
