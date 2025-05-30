// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer
 */

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <dt-bindings/clock/mstar-msc313-sc-gp.h>

#if 0
		clkgen_special_imi_nm: clkgen_mux@1f226680 {
			compatible = "mstar,msc313e-clkgen-mux";
			reg = <0x1f226680 0x4>;
			#clock-cells = <1>;
			clock-output-names = "imi", "nlm";
			shifts = <0>, <8>;
			clocks = <&clkgen_pll 10>; /* this is wrong */
			status = "disabled";
		};
#endif

#include "clk-msc313-mux.h"

static const struct clk_parent_data emac_rxtx_parents[] = {
	{ .fw_name = "eth_buf" },
	{ .fw_name = "rmii_buf" },
};

static const struct clk_parent_data emac_rxtx_ref_parents[] = {
	{ .fw_name = "rmii_buf" },
};

#define EMAC_MUXES \
	MSC313_MUX_CLK_PARENT_DATA(MSC313_SC_GP_EMAC_RX, "emac_rx", emac_rxtx_parents, 0x88, 0, 2, 1, -1), \
	MSC313_MUX_CLK_PARENT_DATA(MSC313_SC_GP_EMAC_RX_REF, "emac_rx_ref", emac_rxtx_ref_parents, 0x88, 8, 10, 1, -1), \
	MSC313_MUX_CLK_PARENT_DATA(MSC313_SC_GP_EMAC_TX, "emac_tx", emac_rxtx_parents, 0x8c, 0, 2, 1, -1), \
	MSC313_MUX_CLK_PARENT_DATA(MSC313_SC_GP_EMAC_TX_REF, "emac_tx_ref", emac_rxtx_ref_parents, 0x8c, 8, 10, 1, -1)

#ifdef CONFIG_MACH_INFINITY
static const struct msc313_mux_data msc313_muxes[] = {
	EMAC_MUXES,
};

static const struct msc313_muxes_data msc313_data = MSC313_MUXES_DATA(msc313_muxes);
#endif

static const struct clk_parent_data sdio_parents[] = {
	/*
	 * The comments we have say this is the 12MHz xtal "boot" clock
	 * but my LA measures this as 161MHz and the signal is very
	 * messy.
	 *
	 * This is probably some other mux here and the comments
	 * are wrong.
	 */
	{ .fw_name = "xtal_div2" },
	{ .fw_name = "sdio_clkgen" },
};

#define SSD20XD_SDIO_BOOT_MUX	0x94

static const struct clk_parent_data hdmi_parents[] = {
	{ .fw_name = "xtal_div2" },
	{ .fw_name = "sc_pixel" },
};
#define SSD20XD_HDMI_MUX	0xd4
#define SSD20XD_HDMI MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_HDMI_GATE, "hdmi", hdmi_parents, SSD20XD_HDMI_MUX, 0, 2, 2, -1)

static const struct clk_parent_data mipi_tx_dsi_apb_parents[] = {
	{ .fw_name = "xtal_div2" }, // incorrect, should be mcu
	{ .fw_name = "xtal_div2" }, // incorrect, should be mipi_tx_dsi
};
#define SSD20XD_MIPI_TX_DSI_APB_MUX	0xdc
#define SSD20XD_MIPI_TX_DSI_APB MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_MIPI_TX_DSI_APB, "mipi_tx_dsi_apb", mipi_tx_dsi_apb_parents, SSD20XD_MIPI_TX_DSI_APB_MUX, 0, 2, 2, -1)

static const struct msc313_mux_data ssd20xd_muxes[] = {
	EMAC_MUXES,
	MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_EMAC1_RX, "emac1_rx", emac_rxtx_parents, 0xcc, 0, 2, 1, -1),
	MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_EMAC1_RX_REF, "emac1_rx_ref", emac_rxtx_ref_parents, 0xcc, 8, 10, 1, -1),
	MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_EMAC1_TX, "emac1_tx", emac_rxtx_parents, 0xd0, 0, 2, 1, -1),
	MSC313_MUX_CLK_PARENT_DATA(SSD20XD_SC_GP_EMAC1_TX_REF, "emac1_tx_ref", emac_rxtx_ref_parents, 0xd0, 8, 10, 1, -1),
	/*
	 * I guess this is really a "deglitch", but the logic is inverted
	 * (1 for the normal clock instead of 0) so it's modelled
	 * as a single bit mux.
	 */
	MSC313_MUX_CLK_PARENT_DATA_FLAGS(SSD20XD_SC_GP_SDIO_GATE, "sdio_gate", sdio_parents, SSD20XD_SDIO_BOOT_MUX,
			-1, 3, 1, -1, 0, CLK_SET_RATE_PARENT),
	SSD20XD_HDMI,
	SSD20XD_MIPI_TX_DSI_APB,
};

static const struct msc313_muxes_data ssd20xd_data = MSC313_MUXES_DATA(ssd20xd_muxes);

static const struct msc313_mux_data ssd210_muxes[] = {
	EMAC_MUXES,
	/* same as ssd20xd above */
	MSC313_MUX_CLK_PARENT_DATA_FLAGS(SSD20XD_SC_GP_SDIO_GATE, "sdio_gate", sdio_parents, SSD20XD_SDIO_BOOT_MUX,
			-1, 3, 1, -1, 0, CLK_SET_RATE_PARENT),
};
static const struct msc313_muxes_data ssd210_data = MSC313_MUXES_DATA(ssd210_muxes);

static const struct of_device_id msc313e_sc_gp_ctrl_muxes_of_match[] = {
#ifdef CONFIG_MACH_INFINITY
{
		.compatible = "mstar,msc313-sc-gp-ctrl-muxes",
		.data = &msc313_data,
	},
#endif
	{
		.compatible = "sstar,ssd20xd-sc-gp-ctrl-muxes",
		.data = &ssd20xd_data,
	},
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible = "sstar,ssd210-sc-gp-ctrl-muxes",
		.data = &ssd210_data,
	},
#endif
	{}
};

static int msc313e_clkgen_mux_probe(struct platform_device *pdev)
{
	const struct msc313_muxes_data *muxes_data;
	struct device *dev = &pdev->dev;
	struct msc313_muxes *muxes;
	struct regmap *regmap;

	muxes_data = of_device_get_match_data(dev);
	if (!muxes_data)
		return -EINVAL;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	muxes = msc313_mux_register_muxes(dev, regmap, muxes_data, NULL, NULL);

	return devm_of_clk_add_hw_provider(dev, msc313_mux_xlate, muxes);
}

static struct platform_driver msc313_sc_gp_ctrl_muxes_driver = {
	.driver = {
		.name = "msc313-sc-gp-ctrl-muxes",
		.of_match_table = msc313e_sc_gp_ctrl_muxes_of_match,
	},
	.probe = msc313e_clkgen_mux_probe,
};
builtin_platform_driver(msc313_sc_gp_ctrl_muxes_driver);
