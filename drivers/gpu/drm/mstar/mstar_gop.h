/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Some versions of the GOP require bank switching, some have the register banks
 * linearly mapped.
 *
 * all
 *
 * 0x1fc - bank select
 *
 * bank 0
 *
 * 0x00 - config
 *
 *       7      |          6          |                 5             |      4
 *  5541 enable | test pattern enable | yuv transparent colour enable | field invert
 *       3      |          2          |                 1             |      0
 * display mode |     hsync invert    |            vsync invert       |    reset
 *
 * 0x04 - destination/regdma interval
 *
 *          15 - 12       |        11 - 8       |    2 - 0
 *  regdma interval start | regdma interval end | destination
 *
 * bank 1
 *
 * 0x00 - format
 * 0x04 - frame buffer address
 *
 *
 *
 */

/* bank 0 */

#define MSTAR_GOP_BANK_0		0x0
#define MSTAR_GOP_REG_DST_RI		(MSTAR_GOP_BANK_0 + 0x04)
#define MSTAR_GOP_REG_BLINK		(MSTAR_GOP_BANK_0 + 0x08)
#define MSTAR_GOP_REG_PSRAM_WD		(MSTAR_GOP_BANK_0 + 0x0c) // 32 bit
#define MSTAR_GOP_REG_PSRAM_CONFIG	(MSTAR_GOP_BANK_0 + 0x14)
#define MSTAR_GOP_REG_REGDMA_START_END	(MSTAR_GOP_BANK_0 + 0x18) // 32 bit
#define MSTAR_GOP_REG_INT_MASK		(MSTAR_GOP_BANK_0 + 0x20)
#define MSTAR_GOP_REG_STATUS		(MSTAR_GOP_BANK_0 + 0x24)

/* bank 1 */

#define MSTAR_GOP_BANK_1		0x200
#define MSTAR_GOP_REG_FORMAT		(MSTAR_GOP_BANK_1 + 0x00)
