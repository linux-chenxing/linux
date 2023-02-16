// SPDX-License-Identifier: GPL-2.0
/*
 */

#ifndef DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_MSC313_H_
#define DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_MSC313_H_

static const struct msc313_clkgen_parent_data mcu_msc313_parents[] = {
	PARENT_GATE(9),
	PARENT_GATE(10),
	PARENT_DIVIDER(8, 2),
	PARENT_DIVIDER(9, 2),
};
#define MCU_MSC313 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MCU, "mcu", mcu_msc313_parents, 0x4, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data riubrdg_parents[] = {
	PARENT_OF("unknown"),
};
#define RIUBRDG MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_RIUBRDG, "riubrdg", riubrdg_parents, 0x4, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data miu_parents[] = {
	PARENT_OF("ddrpll"),
	PARENT_OF("unknown"),
	PARENT_OF("miupll"),
	PARENT_GATE(9),
};
#define MIU MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MIU, "miu", miu_parents, 0x5c, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data ddr_syn_parents[] = {
	PARENT_GATE(6),
	PARENT_GATE(9),
	PARENT_OF("xtal_div2"),
};

#define DDR_SYN MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_DDR_SYN, "ddr_syn", ddr_syn_parents, 0x64, 0, 2, 2, 0)

static const struct msc313_clkgen_parent_data uart_parents[] = {
	PARENT_GATE(10),
	PARENT_DIVIDER(8, 2),
	PARENT_OF("xtal_div2"),
};
#define UART0 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_UART0, "uart0", uart_parents, 0xc4, 0, 2, 2, -1)
#define UART1 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_UART1, "uart1", uart_parents, 0xc4, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data spi_msc313_parents[] = {
	PARENT_GATE(9),
	PARENT_DIVIDER(9, 2),
	PARENT_GATE(14),
	PARENT_DIVIDER(8, 4),
};
#define	SPI_MSC313 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_SPI, "spi", spi_msc313_parents, 0xc8, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data mspi_parents[] = {
	PARENT_DIVIDER(9, 2),
	PARENT_DIVIDER(9, 4),
	PARENT_OF("xtal_div2"),
};
#define MSPI0_MSC313 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MSPI0, "mspi0", mspi_parents, 0xcc, 0, 2, 2, -1)
#define MSPI1_MSC313 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MSPI1, "mspi1", mspi_parents, 0xcc, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data fuart0_synth_in_parents[] = {
	PARENT_GATE(6),
	PARENT_GATE(9),
};

static const struct msc313_clkgen_parent_data fuart_parents[] = {
	PARENT_GATE(10),
	PARENT_DIVIDER(9, 2),
	PARENT_OF("xtal_div2"),
	PARENT_MUX(MSC313_CLKGEN_FUART0_SYNTH_IN),
};

#define FUART0_SYNTH_IN	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_FUART0_SYNTH_IN, "fuart0_synth_in", fuart0_synth_in_parents, 0xd0, 4, 6, 2, -1)
#define FUART		MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_FUART, "fuart", fuart_parents, 0xd0, 0, 2, 2, -1)

/* Same for i3 and i2m */
static const struct msc313_clkgen_parent_data miic_parents[] = {
	PARENT_DIVIDER(8, 4),
	PARENT_DIVIDER(9, 4),
	PARENT_OF("xtal_div2"),
};
#define MIIC0 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MIIC0, "miic0", miic_parents, 0xdc, 0, 2, 2, -1)
#define MIIC1 MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_MIIC1, "miic1", miic_parents, 0xdc, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data emac_ahb_parents[] = {
	PARENT_DIVIDER(8, 2),
	PARENT_GATE(12),
	PARENT_GATE(14),
};
#define EMAC_AHB MSC313_MUX_PARENT_DATA_FLAGS(MSC313_CLKGEN_EMAC_AHB, "emac_ahb", emac_ahb_parents, 0x108, 0, 2, 2, -1, CLK_IS_CRITICAL, 0)

/* same for i3 and i2m */
static const struct msc313_clkgen_parent_data sdio_parents[] = {
	PARENT_DIVIDER(3, 4),
	PARENT_DIVIDER(14, 2),
	PARENT_DIVIDER(2, 4),
	PARENT_DIVIDER(8, 8),
	PARENT_DIVIDER(2, 5),
	PARENT_DIVIDER(2, 8),
	PARENT_OF("xtal_div2"),
	/* undocumented but exists */
	PARENT_OF("xtal_div2_div40"),
};
#define SDIO	MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_SDIO, "sdio", sdio_parents, 0x114, 0, 2, 3, -1)

static const struct msc313_clkgen_parent_data bdma_parents[] = {
	PARENT_MUX(MSC313_CLKGEN_MIU),
	PARENT_OF("xtal_div2_div40"),
};
#define BDMA MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_BDMA, "bdma", bdma_parents, 0x180, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data aesdma_parents[] = {
	PARENT_GATE(14),
	PARENT_GATE(10),
};
#define AESDMA MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_AESDMA, "aesdma", aesdma_parents, 0x184, 0, 2, 2, 4)

/* i/i3 only? */
static const struct msc313_clkgen_parent_data isp_parents[] = {
	PARENT_GATE(12),
	PARENT_GATE(14),
	PARENT_DIVIDER(8, 4),
	PARENT_DIVIDER(9, 4),
};
#define ISP MSC313_MUX_PARENT_DATA(MSC313_CLKGEN_ISP, "isp", isp_parents, 0x184, 8, 10, 2, 12)

static const struct msc313_mux_data msc313_muxes[] = {
	COMMON(MSC313),
	FUART0_SYNTH_IN,
	FUART,
	MIIC0,
	MIIC1,
	EMAC_AHB,
	SDIO,
	BDMA,
	AESDMA,
	ISP,
	JPE,
};

static const struct msc313_muxes_data msc313_data = MSC313_MUXES_DATA(msc313_muxes);

#endif /* DRIVERS_CLK_MSTAR_CLK_MSC313_CLKGEN_MSC313_H_ */
