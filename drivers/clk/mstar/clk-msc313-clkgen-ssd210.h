// SPDX-License-Identifier: GPL-2.0
/*
 */

#ifndef DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD210_H_
#define DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD210_H_

#define SPI_SSD210 SPI_SSD20XD
#define MCU_SSD210 MCU_SSD20XD

static const struct msc313_clkgen_parent_data ssd210_mspi_parents[] = {
	PARENT_DIVIDER(9, 2),
	PARENT_OF("unknown"),
	PARENT_OF("xtal_div2"),
	PARENT_DIVIDER(8, 2),
};
#define MSPI0_SSD210 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MSPI0, "mspi0", ssd210_mspi_parents, 0xcc, 0, 2, 2, -1)
#define MSPI1_SSD210 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MSPI1, "mspi1", ssd210_mspi_parents, 0xcc, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data ssd210_pspi_parents[] = {
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
};

#define SSD210_PSPI0	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_PSPI0, "pspi0", ssd210_pspi_parents, 0xe8, 0, 2, 3, -1)
#define SSD210_PSPI1	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_PSPI1, "pspi1", ssd210_pspi_parents, 0xe8, 8, 10, 3, -1)

#define SSD210_BDMA2	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_BDMA2, "bdma2", bdma_parents, 0x180, 8, 10, 2, -1)
#define SSD210_BDMA3	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_BDMA3, "bdma3", bdma_parents, 0x180, 12, 14, 2, -1)

static const struct msc313_mux_data ssd210_muxes[] = {
	COMMON(SSD210),
	FUART0_SYNTH_IN,
	FUART,
	MIIC0,
	MIIC1,
	EMAC_AHB,
	SDIO,
	BDMA,
	AESDMA,
	JPE,
	GE,
	MOP,
	SATA,
	DEC_PCLK,
	DEC_ACLK,
	DEC_BCLK,
	DEC_CCLK,
	SC_PIXEL,
	DISP_432,
	DISP_216,
	SSD210_PSPI0,
	SSD210_PSPI1,
	SSD210_BDMA2,
	SSD210_BDMA3,
};

static const struct msc313_muxes_data ssd210_data = MSC313_MUXES_DATA(ssd210_muxes);

#endif /* DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD210_H_ */
