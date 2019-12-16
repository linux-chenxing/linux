/*
 * 0x00
 * 15 |    14     |    13     | 12 - 9 | 8 - 7  |     6      |  5 - 2  |    1    | 0
 *  ? | uv_vy_sel | rm_err_fs | vc_en  | fun_md | rm_err_sot | lane_en | ecc_off | ?
 *
 * 0x04
 *    12    |   11 - 9    |    8 - 6    |    5 - 3    |    2 - 0
 * debug_en | vc3_hs_mode | vc2_hs_mode | vc1_hs_mode | vc0_hs_mode
 *
 * 0x08
 *    12    |   11 - 9    |    8 - 6    |    5 - 3    |    2 - 0
 * debug_en | vc3_vs_mode | vc2_vs_mode | vc1_hs_mode | vc0_hs_mode
 *
 * 0x0c
 *   14 - 0
 * err_int_mask
 *
 * 0x10
 *   14 - 0
 * err_int_force
 *
 * 0x14
 *   14 - 0
 * err_int_clr
 *
 * 0x18 - "rpt" interrupt masks
 * |   1    |  0
 * | frame  | line
 *
 * 0x1c - "rpt" interrupt force
 * -- same as masks
 *
 * 0x20 - "rpt" interrupt clear
 * -- same as masks
 *
 * 0x24 - phy interrupt masks
 * 0x28 - phy interrupt force
 * 0x2c - phy interrupt clear
 * 0x30 - error interrupt source
 * 0x34 - error interrupt raw source
 * 0x38 - rpt interrupt source
 * 0x3c - rpt interrupt raw source
 * 0x40 - phy interrupt source
 * 0x44 - phy interrupt raw source
 * 0x48 - frame number
 * 0x4c - line number
 * 0x50 - g8spd_wc
 * 0x54 - g8spd_dt
 *
 * 0x58
 * |    0
 * | mac idle
 *
 * 0x5c
 * |       0
 * | 1 frame trigger
 *
 * 0x60 - clkgen
 * 0x64 - clkgen
 * 0x68 - reserved
 * 0x6c - reserved
 * 0x70 - reserved
 *
 * 0x74 - x
 * 0x78 - x
 * 0x7c - x
 *
 * 0x80 -
 *       2      |    1   |   0
 * dont care dt | mac en | reset
 *
 * 0x84 -
 *       5       |       4       |    3    |    2    |    1 - 0
 * report line   | report frame  | eot sel | sot sel | raw l sot sel
 * num condition | num condition |         |         |
 */
