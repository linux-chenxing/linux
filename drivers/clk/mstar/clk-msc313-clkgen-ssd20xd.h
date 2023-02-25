// SPDX-License-Identifier: GPL-2.0
/*
 */

#ifndef DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD20XD_H_
#define DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD20XD_H_

static const struct msc313_clkgen_parent_data mcu_ssd20xd_parents[] = {
	PARENT_GATE(9),
	PARENT_GATE(10),
	PARENT_DIVIDER(8, 2),
	PARENT_DIVIDER(9, 2), // wrong?
	PARENT_GATE(6),
	PARENT_GATE(0),
	PARENT_GATE(1),
	PARENT_OF("unknown"),
};
#define MCU_SSD20XD MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MCU, "mcu", mcu_ssd20xd_parents, 0x4, 0, 2, 3, 5)

static const struct msc313_clkgen_parent_data spi_ssd20xd_parents[] = {
	PARENT_GATE(9),
	PARENT_DIVIDER(9, 2),
	PARENT_GATE(14),
	PARENT_DIVIDER(8, 4),
	/* This is MIU, datasheet says "must select this one" :) */
	PARENT_OF("unknown"),
};
#define	SPI_SSD20XD MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_SPI, "spi", spi_ssd20xd_parents, 0xc8, 0, 2, 3, 5)

#define MSPI0_SSD20XD MSPI0_MSC313
#define MSPI1_SSD20XD MSPI1_MSC313

#define MSPI_MOVEDMA	MSC313_MUX_PARENT_DATA(SSD20XD_CLKGEN_MSPI_MOVEDMA, "mspi_movedma", mspi_parents, 0xcc, 12, 14, 2, -1)

/* ssd20xd only? */
static const struct msc313_clkgen_parent_data ge_parents[] = {
	/* miu, 240MHz */
	PARENT_MUX(MSC313_CLKGEN_MIU),
	/* 216MHz */
	PARENT_GATE(9),
	/* 172MHz */
	PARENT_GATE(10),
	/* 144MHz */
	PARENT_DIVIDER(8, 2),
	// upll 320
	PARENT_GATE(1),
	// upll 384
	PARENT_GATE(0),
	// mpll 432
	PARENT_GATE(6),
};
#define GE MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_GE, "ge", ge_parents, 0x144, 0, 2, 3, -1)

/* MIPI TX */
static const struct msc313_clkgen_parent_data mipi_tx_parents[] = {
	PARENT_OF("lpll"),
	PARENT_GATE(2),
	PARENT_DIVIDER(8, 2),
	PARENT_DIVIDER(9, 2),
	PARENT_GATE(9),
	PARENT_GATE(4),
};
#define MIPI_TX_DSI	MSC313_MUX_PARENT_DATA(SSD20XD_CLKGEN_MIPI_TX_DSI, "mipi_tx_dsi", mipi_tx_parents, 0x1bc, 0, 2, 3, -1)

static const struct msc313_mux_data ssd20xd_muxes[] = {
	COMMON(SSD20XD),
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
	MIPI_TX_DSI,
	MSPI_MOVEDMA,
};

static const struct msc313_muxes_data ssd20xd_data = MSC313_MUXES_DATA(ssd20xd_muxes);

#endif /* DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_SSD20XD_H_ */
