// SPDX-License-Identifier: GPL-2.0+

/*
 * utmi registers
 *
 * 0x0 - pwrctrl
 * reset value	msc313  - 0x7f03
 * 		ssc8336 - 0xff05
 *
 *     15      |    14     |      13     |      12     |       11      |      10        |         9          |         8
 *     pdn     | iref_pdn  | vbusdet_pdn | fl_xcvr_pdn | hs_preamp_pdn |  hs_ted_pdn    |     upll_pdn       |     hs_dm_pdn
 *       7     |     6     |      5      |      4      |        3      |       2        |         1          |         0
 *  r_dm_pden  | r_dp_pden |  r_pumode   |   dm_puen   |     dp_puen   | ref power down | term override mode | pdn override mode
 *
 *
 * 0x4 - config
 * reset value  msc313  - 0x2084
 * 		ssc8336 - 0x9080
 *
 * magic value	ssc8336 - 0x2084 - from dashcam app
 *
 *                3                     |               2                |     1    |    0
 * override value to control NRZI mode  |             clk12_sel          | fsls_sel | sel_ovr
 *                                      |		0 - 12mhz        |          |
 *                                      | 1 - "from digital synthesizer" |          |
 *
 * 0x10 - clkctrl
 * |       15      |     14     | 13 |       12      |      11      |        10        |        9         |    8
 * |               |            |    | clk214_syn_en | force_pll_on | clk_ctl_override |   xtal12_en      | all_pass
 * |      7        |     6      | 5  |       4       |       3      |        2         |        1         |     0
 * | pd_bg_current | clktest_en |    |               |       ?      |        ?         |  utmi_clk120_en  | utmi_clk_en
 *
 *
 * 0x14 - clkinv
 *
 *      13    |
 * dp/dn swap |
 *
 *
 * 0x40 - PLL_TEST[15:0]
 * 0x44 - PLL_TEST[31:16]
 * 0x48 - PLL_TEST[39:32]
 *
 * From SSC8336
 * 0x2088 0x8051
 *
 * 0x58 - utmi eye setting 2c
 * 0x59 - utmi eye setting 2d
 * 0x5c - utmi eye setting 2e
 * 0x5d - utmi eye setting 2f
 *
 * 0x60 -
 *
 * 0x78 - calibration
 *          15 - 4          | 3 |     2      |    1    |    0
 *  calibration detail data |   | power good | ca_end  | ca_start
 *
 * 0x1c
 *
 */

#define REG_PWRCTRL			0x0
#define MSTAR_UTMI_REG_CONFIG		0x4
#define REG_CLKCTRL			0x10
#define MSTAR_UTMI_REG_CLKINV		0x14
#define MSTAR_UTMI_REG_CLKINV_DPDNSWP	BIT(13)
#define PWRCTRL_REG_PDN			15
#define PWRCTRL_UPLL_PDN		9
#define PWRCTRL_VREF_PDN		2
#define MSTAR_UTMI_REG_PLL_TEST0	0x40
#define MSTAR_UTMI_REG_PLL_TEST1	0x44
#define MSTAR_UTMI_REG_PLL_TEST2	0x48
#define REG_EYESETTING1			0x58
#define REG_EYESETTING2			0x5c
#define MSTAR_UTMI_REG_CAL		0x78
#define MSTAR_UTMI_REG_CAL_START	BIT(0)
#define MSTAR_UTMI_REG_CAL_END		BIT(1)
#define MSTAR_UTMI_REG_CAL_DATA_SHIFT	4

static struct reg_field mstar_utmi_pwrctrl_pwrdwn_field = REG_FIELD(REG_PWRCTRL, PWRCTRL_REG_PDN, PWRCTRL_REG_PDN);
