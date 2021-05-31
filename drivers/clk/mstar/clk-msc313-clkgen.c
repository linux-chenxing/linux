// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer
 */

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <dt-bindings/clock/mstar-msc313-clkgen.h>

#include "clk-msc313-mux.h"

#if 0
			clkgen_mux_miu_rec: clkgen_mux@207060 {
				compatible = "mstar,msc313e-clkgen-mux";
				reg = <0x207060 0x4>;
				#clock-cells = <1>;
				clock-output-names = "miu_rec";
				shifts = <0>;
				mux-shifts = <2>;
				mux-widths = <2>;
				clocks = <&xtal_div2_div8>,
					 <&xtal_div2_div16>,
					 <&xtal_div2_div64>,
					 <&xtal_div2_div128>;
				status = "disabled";
			};

		/* can't write on i3, i6 only? */
		clkgen_mux_pwr_ctl: clkgen_mux@1f207010 {
			// pwr_ctrl
			//
		};

		// this one is interesting, looks like
		// the clock thats used to deglitch
		clkgen_mux_boot: clkgen_mux@1f207020 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207020 0x4>;
			#clock-cells = <1>;
			clock-output-names = "boot";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&xtal_div2>;
			status = "disabled";
		};

		clkgen_mux_miu_boot: clkgen_mux@1f207080 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207080 0x4>;
			#clock-cells = <1>;
			clock-output-names = "miu_boot";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&xtal_div2>,
				 <&clkgen_mux_miu 0>;
			status = "disabled";
		};

		/* This seems important as the ipl turns it on at boot.
		 * Maybe tck == test clock? for JTAG?
		 */
		clkgen_mux_tck: clkgen_mux@1f2070c0 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f2070c0 0x4>;
			#clock-cells = <1>;
			clock-output-names = "tck";
			shifts = <0>;
			clocks = <&xtal_div2>;
			status = "disabled";
		};

		clkgen_mux_fclk1: clkgen_mux@1f207190 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207190 0x4>;
			#clock-cells = <1>;
			clock-output-names = "fclk1";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>;
			status = "disabled";
		};

		clkgen_mux_fclk2: clkgen_mux@1f207194 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207194 0x4>;
			#clock-cells = <1>;
			clock-output-names = "fclk2";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>;
			status = "disabled";
		};



		clkgen_mux_gop: clkgen_mux@1f20719c {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f20719c 0x4>;
			#clock-cells = <1>;
			clock-output-names = "gop";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&clkgen_mux_fclk1 0>; // might be incorrect
			status = "disabled";
		};

		// gating 216_2digpm stops the system
		clkgen_mux_digpm: clkgen_gates@1f2071b4 {
			compatible = "mstar,clkgen-gates";
			reg = <0x1f2071b4 0x4>;
			#clock-cells = <1>;
			clock-output-names = "hemcu_216",
					     "216_2digpm",
					     "172_2digpm",
					     "144_2digpm",
					     "123_2digpm",
					     "86_2digpm";
			shifts = <0>, <1>, <2>, <3>, <4>, <5>;
			clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_216>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_216>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_144>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_123>,
				  <&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>;
			output-flags = <0>, <MSTAR_CLKGEN_OUTPUTFLAG_CRITICAL>,
				       <0>, <0>, <0>, <0>;
			status = "disabled";
		};

		clkgen_mux_odclk: clkgen_mux@1f207198 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207198 0x4>;
			#clock-cells = <1>;
			clock-output-names = "odclk";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <2>;
			clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>,
				 <&clk_mpll_86_div2>,
				 <&clk_mpll_86_div4>,
				 <&lpll>;
			status = "disabled";
		};

 		clkgen_mux_sr_sr_mclk: clkgen_mux@1f207188 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f207188 0x4>;
			#clock-cells = <1>;
			clock-output-names = "sr", "sr_mclk";
			shifts = <0>, <8>;
			mux-shifts = <2>, <10>;
			mux-widths = <2>, <3>;
			mux-ranges = <0 4>, <4 8>;
			clocks =
			/* sr */
			<&pad2isp_sr_pclk>,
			<&csi2_mac>,
			<&clk_utmi_160_div4>,
			<&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>,
			/* sr_mclk */
			<&clk_mpll_216_div8>,
			<&clk_mpll_86_div4>,
			<&xtal_div2>,
			<&clk_mpll_86_div16>,
			<&clk_mpll_288_div8>,
			<&clk_mpll_216_div4>,
			<&clk_mpll_86_div2>,
			<&clk_mpll_123_div2>;
			status = "disabled";
		};

  		clkgen_mux_fcie: clkgen_mux@1f20710c {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f20710c 0x4>;
			#clock-cells = <1>;
			clock-output-names = "fcie";
			shifts = <0>;
			mux-shifts = <2>;
			mux-widths = <4>;
			clocks = <&clk_mpll_288_div4>,
				 <&clk_mpll_123_div2>,
				 <&clk_mpll_216_div4>,
				 <&xtal_div2>,
				 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_86>,
				 <&clk_utmi_192_div4>,
				 <&clk_mpll_86_div2>,
				 <&clk_utmi_160_div4>,
				 <&clk_mpll_288_div8>,
				 <&clk_utmi_160_div5>,
				 <&clk_utmi_160_div8>,
				 <&clk_mpll_86_div16>,
				 <&xtal_div2_div40>;
			status = "disabled";
		};
#endif

#define REG_GATES	0x1c0
#define REG_LOCK	(REG_GATES + 0x0)
#define REG_LOCK_OFF	BIT(1)
#define REG_FORCEON	(REG_GATES + 0x4)
#define REG_FORCEOFF	(REG_GATES + 0x8)
#define REG_ENRD	(REG_GATES + 0xc)

struct msc313_clkgen_parent_data {
	const char *fw_name;
	int mux_idx;
	int gate;
	int divider;
};

#define GATE_NODIVIDER	-1

struct msc313_clkgen_gate_data {
	const struct clk_parent_data clk_parent_data;
	const unsigned int *dividers;
	const unsigned int num_dividers;
};

#define GATE_DATA(_parent, _dividers) \
	{							\
		.clk_parent_data = { .fw_name = _parent },	\
		.dividers = _dividers,				\
		.num_dividers = ARRAY_SIZE(_dividers),		\
	}

#define GATE_DATA_NO_DIVIDERS(_parent) \
	{							\
		.clk_parent_data = { .fw_name = _parent },	\
	}

static const unsigned int gate2_dividers[] = { 4, 5, 8 };

#define GATE2_DIVIDEBY_4 0
#define GATE2_DIVIDEBY_5 1
#define GATE2_DIVIDEBY_8 2

static const unsigned int gate3_dividers[] = { 4 };

#define GATE3_DIVIDEBY_4 0

static const unsigned int gate8_dividers[] = { 2, 4, 8 };

#define GATE8_DIVIDEBY_2 0
#define GATE8_DIVIDEBY_4 1
#define GATE8_DIVIDEBY_8 2

static const unsigned int gate9_dividers[] = { 2, 4, 8 };

#define GATE9_DIVIDEBY_2 0
#define GATE9_DIVIDEBY_4 1
#define GATE9_DIVIDEBY_8 2

static const unsigned int gate11_dividers[] = { 2, 4 };
static const unsigned int gate12_dividers[] = { 2 };

static const unsigned int gate14_dividers[] = { 2, 4, 16 };

#define GATE14_DIVIDEBY_2 0
#define GATE14_DIVIDEBY_4 1
#define GATE14_DIVIDEBY_16 2

static const struct msc313_clkgen_gate_data gates_data[] = {
	/* upll 384 */
	GATE_DATA_NO_DIVIDERS("gate0"),
	/* upll 320 */
	GATE_DATA_NO_DIVIDERS("gate1"),
	/* utmi 160 */
	GATE_DATA("gate2", gate2_dividers),
	/* utmi 192 */
	GATE_DATA("gate3", gate3_dividers),
	GATE_DATA_NO_DIVIDERS("gate4"),
	GATE_DATA_NO_DIVIDERS("gate5"),
	/* mpll 432 */
	GATE_DATA_NO_DIVIDERS("gate6"),
	/* mpll 172 */
	GATE_DATA_NO_DIVIDERS("gate7"),
	/* mpll 288 */
	GATE_DATA("gate8", gate8_dividers),
	/* mpll 216 */
	GATE_DATA("gate9", gate9_dividers),
	GATE_DATA_NO_DIVIDERS("gate10"),
	GATE_DATA("gate11", gate11_dividers),
	/* 123 */
	GATE_DATA("gate12", gate12_dividers),
	/* 124 */
	GATE_DATA_NO_DIVIDERS("gate13"),
	/* 86 */
	GATE_DATA("gate14", gate14_dividers),
	GATE_DATA_NO_DIVIDERS("gate15"),
};

struct msc313_clkgen_gate {
	struct regmap_field *force_on;
	struct regmap_field *enrd;
	struct clk_hw clk_hw;
	struct clk_hw **clk_hw_dividers;
};
#define to_gate(_hw) container_of(_hw, struct msc313_clkgen_gate, clk_hw)

struct msc313_clkgen {
	struct msc313_muxes *muxes;
	struct msc313_clkgen_gate gates[];
};


static int msc313_clkgen_gate_enable(struct clk_hw *hw)
{
	struct msc313_clkgen_gate *gate = to_gate(hw);

	//printk("forcing gate %s on\n", clk_hw_get_name(hw));

	return regmap_field_write(gate->force_on, 1);
}

static void msc313_clkgen_gate_disable(struct clk_hw *hw)
{
	struct msc313_clkgen_gate *gate = to_gate(hw);

	regmap_field_write(gate->force_on, 0);
}

static int msc313_clkgen_gate_is_enabled(struct clk_hw *hw)
{
	struct msc313_clkgen_gate *gate = to_gate(hw);
	unsigned int val;
	int ret;

	ret = regmap_field_read(gate->enrd, &val);
	if (ret)
		return ret;

	return val;
}

static const struct clk_ops msc313_clkgen_gate_ops = {
	.enable = msc313_clkgen_gate_enable,
	.disable = msc313_clkgen_gate_disable,
	.is_enabled = msc313_clkgen_gate_is_enabled,
};

#define PARENT_GATE(_gate)			\
	{					\
		.mux_idx = -1,			\
		.gate = _gate,			\
		.divider = GATE_NODIVIDER,	\
	}

#define PARENT_DIVIDER(_gate, _divisor)				\
	{							\
		.mux_idx = -1,					\
		.gate = _gate,					\
		.divider = GATE ##_gate##_DIVIDEBY_##_divisor,	\
	}

#define PARENT_OF(_fwname)			\
	{					\
		.fw_name = _fwname,		\
	}

#define PARENT_MUX(_mux_idx)			\
	{					\
		.mux_idx = _mux_idx,		\
	}

#if 0
// gating xtali stops interrupts from the
// arch timer
clkgen_mux_xtali: clkgen_mux@1f207000 {
	compatible = "mstar,msc313e-clkgen-mux";
	reg = <0x1f207000 0x4>;
	#clock-cells = <1>;
	clock-output-names = "xtali", "xtali_sc_gp", "live";
	output-flags = <MSTAR_CLKGEN_OUTPUTFLAG_CRITICAL>, <0x0>, <0x0>;
	shifts = <0>, <4>, <8>;
	mux-shifts = <2>, <6>, <10>;
	mux-widths = <2>, <2>, <2>;
	clocks = <&xtal_div2>;
	status = "disabled";
};
#endif

#if 0
clkgen_mux_mcu_riubrdg: clkgen_mux@1f207004 {
	compatible = "mstar,msc313e-clkgen-mux";
	reg = <0x1f207004 0x4>;
	#clock-cells = <1>;
	clock-output-names = "mcu", "riubrdg";
	shifts = <0>, <8>;
	mux-shifts = <2>, <10>;
	mux-widths = <2>, <2>;
	mux-ranges = <0 5>, <5 1>;
	mstar,deglitches = <4>, <MSTAR_CLKGEN_MUX_NULL>;
	output-flags = <0>, <MSTAR_CLKGEN_OUTPUTFLAG_CRITICAL>;
	clocks = /* mcu */
		 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_216>,
		 <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
		 <&clk_mpll_288_div2>,
		 <&clk_mpll_216_div2>,
		 /*deglitch */
		 <&xtal_div2>,
		 /* riubrdg */
		 <&clkgen_mux_mcu_riubrdg 0>;
	assigned-clocks = <&clkgen_mux_mcu_riubrdg 0>;
	assigned-clock-parents = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_216>;
	assigned-clock-rates = <0>;
	status = "disabled";
};
#endif

#if 0
clkgen_mux_bist: clkgen_mux@1f207008 {
	compatible = "mstar,msc313e-clkgen-mux";
	reg = <0x1f207008 0x4>;
	#clock-cells = <1>;
	clock-output-names = "bist";
	shifts = <0>;
	mux-shifts = <2>;
	mux-widths = <2>;
	clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
		 <&clk_mpll_216_div2>,
		 <&clk_mpll_216_div4>,
		 <&xtal_div2>;
	status = "disabled";
};
#endif

#if 0
clkgen_mux_bist_sc_gp: clkgen_mux@1f20700c {
	compatible = "mstar,msc313e-clkgen-mux";
	reg = <0x1f20700c 0x4>;
	#clock-cells = <1>;
	clock-output-names = "bist_sc_gp";
	shifts = <0>;
	mux-shifts = <2>;
	mux-widths = <2>;
	clocks = <&clkgen_pll MSTAR_MPLL_GATE_MPLL_172>,
		 <&clk_mpll_216_div2>,
		 <&clk_mpll_216_div4>,
		 <&xtal_div2>;
	status = "disabled";
};
#endif

static const struct msc313_clkgen_parent_data miu_parents[] = {
	PARENT_GATE(9),
	PARENT_OF("miupll"),
	PARENT_OF("unknown"),
	PARENT_OF("unknown"),
};
#define MIU		MSC313_MUX_PARENT_DATA("miu", miu_parents, 0x5c, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data ddr_syn_parents[] = {
	PARENT_GATE(6),
	PARENT_GATE(9),
	PARENT_OF("xtal_div2"),
};

#define DDR_SYN		MSC313_MUX_PARENT_DATA("ddr_syn", ddr_syn_parents, 0x64, 0, 2, 2, 0)

static const struct msc313_clkgen_parent_data uart_parents[] = {
	PARENT_GATE(10),
	PARENT_DIVIDER(8, 2),
	PARENT_OF("xtal_div2"),
};

#define UART0		MSC313_MUX_PARENT_DATA("uart0", uart_parents, 0xc4, 0, 2, 2, -1)
#define UART1		MSC313_MUX_PARENT_DATA("uart1", uart_parents, 0xc4, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data spi_parents[] = {
	PARENT_GATE(9),
	PARENT_DIVIDER(9, 2),
	PARENT_GATE(14),
	PARENT_DIVIDER(8, 4),
};

#define	SPI		MSC313_MUX_PARENT_DATA("spi", spi_parents, 0xc8, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data mspi_parents[] = {
	PARENT_DIVIDER(9, 2),
	PARENT_DIVIDER(9, 4),
	PARENT_OF("xtal_div2"),
};

#define MSPI0		MSC313_MUX_PARENT_DATA("mspi0", mspi_parents, 0xcc, 0, 2, 2, -1)
#define MSPI1		MSC313_MUX_PARENT_DATA("mspi1", mspi_parents, 0xcc, 8, 10, 2, -1)

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

#define FUART0_SYNTH_IN	MSC313_MUX_PARENT_DATA("fuart0_synth_in", fuart0_synth_in_parents, 0xd0, 4, 6, 2, -1)
#define FUART		MSC313_MUX_PARENT_DATA("fuart", fuart_parents, 0xd0, 0, 2, 2, -1)

/* Same for i3 and i2m */
static const struct msc313_clkgen_parent_data miic_parents[] = {
	PARENT_DIVIDER(8, 4),
	PARENT_DIVIDER(9, 4),
	PARENT_OF("xtal_div2"),
};

#define MIIC0		MSC313_MUX_PARENT_DATA("miic0", miic_parents, 0xdc, 0, 2, 2, -1)
#define MIIC1		MSC313_MUX_PARENT_DATA("miic1", miic_parents, 0xdc, 8, 10, 2, -1)

static const struct msc313_clkgen_parent_data emac_ahb_parents[] = {
	PARENT_DIVIDER(8, 2),
	PARENT_GATE(12),
	PARENT_GATE(14),
};
#define EMAC_AHB	MSC313_MUX_PARENT_DATA_FLAGS("emac_ahb", emac_ahb_parents, 0x108, 0, 2, 2, -1, CLK_IS_CRITICAL, 0)

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
#define SDIO	MSC313_MUX_PARENT_DATA("sdio", sdio_parents, 0x114, 0, 2, 3, -1)

/* ssd20xd only? */
static const struct msc313_clkgen_parent_data mop_parents[] = {
	PARENT_MUX(MSC313_CLKGEN_MIU), // wrong
	PARENT_MUX(MSC313_CLKGEN_MIU), // wrong
	PARENT_GATE(8),
	PARENT_MUX(MSC313_CLKGEN_MIU),
};
#define MOP	MSC313_MUX_PARENT_DATA("mop", mop_parents, 0x150, 0, 2, 2, -1)

/* DEC PCLK */
static const struct msc313_clkgen_parent_data dec_pclk_parents[] = {
	PARENT_GATE(9),
	PARENT_DIVIDER(8, 2),
	PARENT_DIVIDER(9, 2),
};
#define DEC_PCLK	MSC313_MUX_PARENT_DATA("dec_pclk", dec_pclk_parents, 0x154, 0, 2, 2, -1)

/* DEC ACLK */
static const struct msc313_clkgen_parent_data dec_aclk_parents[] = {
	PARENT_GATE(1),
	PARENT_GATE(0),
	PARENT_GATE(8),
	PARENT_GATE(9),
};
#define DEC_ACLK	MSC313_MUX_PARENT_DATA("dec_aclk", dec_aclk_parents, 0x154, 8, 10, 2, -1)

/* DEC BCLK */
static const struct msc313_clkgen_parent_data dec_bclk_parents[] = {
	PARENT_GATE(8),
};
#define DEC_BCLK	MSC313_MUX_PARENT_DATA("dec_bclk", dec_bclk_parents, 0x1f8, 0, 2, 3, -1)

/* DEC CCLK */
static const struct msc313_clkgen_parent_data dec_cclk_parents[] = {
	PARENT_GATE(0),
};
#define DEC_CCLK	MSC313_MUX_PARENT_DATA("dec_cclk", dec_cclk_parents, 0x1f8, 8, 10, 3, -1)

static const struct msc313_clkgen_parent_data bdma_parents[] = {
	PARENT_MUX(MSC313_CLKGEN_MIU),
	PARENT_OF("xtal_div2_div40"),
};
#define BDMA	MSC313_MUX_PARENT_DATA("bdma", bdma_parents, 0x180, 0, 2, 2, 4)

static const struct msc313_clkgen_parent_data aesdma_parents[] = {
	PARENT_GATE(14),
	PARENT_GATE(10),
};
#define AESDMA	MSC313_MUX_PARENT_DATA("aesdma", aesdma_parents, 0x184, 0, 2, 2, 4)

/* i/i3 only? */
static const struct msc313_clkgen_parent_data isp_parents[] = {
	PARENT_GATE(12),
	PARENT_GATE(14),
	PARENT_DIVIDER(8, 4),
	PARENT_DIVIDER(9, 4),
};
#define ISP	MSC313_MUX_PARENT_DATA("isp", isp_parents, 0x184, 8, 10, 2, 12)

static const struct msc313_clkgen_parent_data jpe_parents[] = {
	PARENT_GATE(8),
	PARENT_GATE(9),
	PARENT_DIVIDER(8, 4),
	PARENT_DIVIDER(9, 4),
};
#define JPE	MSC313_MUX_PARENT_DATA("jpe", jpe_parents, 0x1a8, 0, 2, 2, -1)

/* SATA */
static const struct msc313_clkgen_parent_data sata_parents[] = {
	PARENT_GATE(9), // incorrect, should be utmi 240m
	PARENT_GATE(8),
};
#define SATA	MSC313_MUX_PARENT_DATA("sata", sata_parents, 0x1b8, 0, 2, 2, -1)

#define COMMON		\
	MIU,		\
	DDR_SYN,	\
	UART0,		\
	UART1,		\
	SPI,		\
	MSPI0,		\
	MSPI1

static const struct msc313_mux_data msc313_muxes[] = {
	COMMON,
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

static const struct msc313_mux_data ssd20xd_muxes[] = {
	COMMON,
	FUART0_SYNTH_IN,
	FUART,
	MIIC0,
	MIIC1,
	EMAC_AHB,
	SDIO,
	BDMA,
	AESDMA,
	MSC313_MUX_GAP(),
	JPE,
	MOP,
	SATA,
	DEC_PCLK,
	DEC_ACLK,
	DEC_BCLK,
	DEC_CCLK,
};

static const struct msc313_muxes_data ssd20xd_data = MSC313_MUXES_DATA(ssd20xd_muxes);

static const struct of_device_id msc313_clkgen_ids[] = {
	{
		.compatible = "mstar,msc313-clkgen",
		.data = &msc313_data,
	},
	{
		.compatible = "sstar,ssd20xd-clkgen",
		.data = &ssd20xd_data,
	},
	{}
};

static const struct regmap_config msc313_clkgen_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static struct clk_hw *msc313_clkgen_xlate(struct of_phandle_args *clkspec, void *data)
{
	struct msc313_clkgen *clkgen = data;
	unsigned int area = clkspec->args[0];
	unsigned int idx = clkspec->args[1];
	unsigned int divider;

	switch(area){
	case MSC313_CLKGEN_MUXES:
		return &clkgen->muxes->muxes[idx].mux_hw;
	case MSC313_CLKGEN_DEGLITCHES:
		return &clkgen->muxes->muxes[idx].deglitch_hw;
	case MSC313_CLKGEN_GATES:
		return &clkgen->gates->clk_hw;
	case MSC313_CLKGEN_DIVIDERS:
		divider = clkspec->args[2];
		return clkgen->gates->clk_hw_dividers[divider];
	}

	return msc313_mux_xlate(clkspec, clkgen->muxes);
}

static int msc313_clkgen_fill_mux_clk_parent_data(struct clk_parent_data *clk_parent_data,
				void *data, const void* parent_data, const struct msc313_muxes *muxes,
				unsigned mux_idx, unsigned int parent_idx)
{
	struct msc313_clkgen *clkgen = data;
	const struct msc313_clkgen_parent_data *mux_parent_data = parent_data;
	const struct msc313_clkgen_parent_data *mux_data = &mux_parent_data[parent_idx];

	//printk("finding parent fo mux %d:%d\n", mux_idx, parent_idx);

	if (mux_data->fw_name)
		clk_parent_data->fw_name = mux_data->fw_name;
	else if (mux_data->mux_idx != -1){
		/* We can only refer to muxes that have already been created */
		if (mux_data->mux_idx >= mux_idx)
			return -EINVAL;
		printk("linking mux %d, back to %d\n", mux_idx, mux_data->mux_idx);
		clk_parent_data->hw = &muxes->muxes[mux_data->mux_idx].deglitch_hw;
	}
	else if (mux_data->divider == -1)
		clk_parent_data->hw = &clkgen->gates[mux_data->gate].clk_hw;
	else
		clk_parent_data->hw = clkgen->gates[mux_data->gate].clk_hw_dividers[mux_data->divider];

	return 0;
}

static int msc313_clkgen_probe(struct platform_device *pdev)
{
	const struct msc313_muxes_data *match_data;
	struct clk_init_data gate_clk_init = {
		.ops = &msc313_clkgen_gate_ops,
		.num_parents = 1,
	};
	struct device *dev = &pdev->dev;
	struct msc313_clkgen *clkgen;
	int gate_index;
	struct regmap *regmap;
	void __iomem *base;
	int ret;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &msc313_clkgen_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	clkgen = devm_kzalloc(dev, struct_size(clkgen, gates,
			ARRAY_SIZE(gates_data)), GFP_KERNEL);
	if (!clkgen)
		return -ENOMEM;

	/* Clear the force on register so we can actually control the gates */
	regmap_write(regmap, REG_FORCEON, 0x0);
	/* Clear the force off register */
	regmap_write(regmap, REG_FORCEOFF, 0x0);
	/* lock the force off bits */
	regmap_write(regmap, REG_LOCK, REG_LOCK_OFF);

	for (gate_index = 0; gate_index < ARRAY_SIZE(gates_data); gate_index++) {
		const struct msc313_clkgen_gate_data *gate_data = &gates_data[gate_index];
		struct reg_field forceon_field = REG_FIELD(REG_FORCEON,
				gate_index, gate_index);
		struct reg_field enrd_field = REG_FIELD(REG_ENRD,
				gate_index, gate_index);
		struct msc313_clkgen_gate *gate = &clkgen->gates[gate_index];
		char* divider_name;
		int divider_index;

		gate->force_on = devm_regmap_field_alloc(dev, regmap, forceon_field);
		gate->enrd = devm_regmap_field_alloc(dev, regmap, enrd_field);
		gate->clk_hw.init = &gate_clk_init;

		gate_clk_init.parent_data = &gates_data[gate_index].clk_parent_data;
		gate_clk_init.name = devm_kasprintf(dev, GFP_KERNEL, "%s_gate_%d",
				dev_name(dev), gate_index);

		ret = devm_clk_hw_register(dev, &gate->clk_hw);
		if (ret)
			return ret;

		if (!gate_data->num_dividers)
			goto no_dividers;

		gate->clk_hw_dividers = devm_kcalloc(dev, gate_data->num_dividers,
				sizeof(*gate->clk_hw_dividers), GFP_KERNEL);

		for (divider_index = 0; divider_index < gate_data->num_dividers; divider_index++) {
			unsigned int divider = gate_data->dividers[divider_index];
			divider_name = devm_kasprintf(dev, GFP_KERNEL, "%s_div_%u",
					gate_clk_init.name, divider);
			if (!divider_name)
				return -ENOMEM;
			gate->clk_hw_dividers[divider_index] = devm_clk_hw_register_fixed_factor(dev, divider_name,
					gate_clk_init.name, 0, 1, divider);
			if (IS_ERR(gate->clk_hw_dividers[divider_index]))
				return PTR_ERR(gate->clk_hw_dividers[divider_index]);
		}

no_dividers:
		devm_kfree(dev, gate_clk_init.name);
	}

	clkgen->muxes = msc313_mux_register_muxes(dev, regmap, match_data,
			msc313_clkgen_fill_mux_clk_parent_data, clkgen);

        return devm_of_clk_add_hw_provider(dev, msc313_clkgen_xlate, clkgen);
}

static struct platform_driver msc313_clkgen_driver = {
	.driver = {
		.name = "msc313-clkgen",
		.of_match_table = msc313_clkgen_ids,
	},
	.probe = msc313_clkgen_probe,
};
builtin_platform_driver(msc313_clkgen_driver);
