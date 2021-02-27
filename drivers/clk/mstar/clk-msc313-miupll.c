// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/module.h>

/*
 * miupll_freq = 24 * INREGMSK16(iMiupllBankAddr + REG_ID_03, 0x00FF) /
 * 22 * 24
 * ((INREGMSK16(iMiupllBankAddr + REG_ID_03, 0x0700) >> 8) + 2);
 * 2 + 2 = 4
 * 216
*/

#define REG_RATE 0xc

struct mstar_miupll {
	void __iomem *base;
	struct clk_hw clk_hw;
	u32 rate;
};

#define to_miupll(_hw) container_of(_hw, struct mstar_miupll, clk_hw)

static const struct of_device_id mstar_miupll_of_match[] = {
	{
		.compatible = "mstar,miupll",
	},
	{}
};

static int mstar_miupll_is_enabled(struct clk_hw *hw)
{
	struct mstar_miupll *miupll = to_miupll(hw);

	return 0;
}

static unsigned long mstar_miupll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct mstar_miupll *miupll = to_miupll(hw);

	uint16_t temp = readw_relaxed(miupll->base + REG_RATE);
	unsigned long freq = parent_rate;
	freq *= temp & 0xff;
	freq /= ((temp >> 8) & GENMASK(2, 0)) + 2;
	return freq;
}

static const struct clk_ops mstar_miupll_ops = {
	.is_enabled = mstar_miupll_is_enabled,
	.recalc_rate = mstar_miupll_recalc_rate,
};

static const struct clk_parent_data miupll_parent = {
	.index	= 0,
};

static int mstar_miupll_probe(struct platform_device *pdev)
{
	struct clk_init_data clk_init = {};
	struct device *dev = &pdev->dev;
	struct mstar_miupll* miupll;
	int ret;

	miupll = devm_kzalloc(dev, sizeof(*miupll), GFP_KERNEL);
	if(!miupll)
		return -ENOMEM;

	miupll->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(miupll->base))
		return PTR_ERR(miupll->base);

	clk_init.name = dev_name(dev);
	clk_init.ops = &mstar_miupll_ops;
	clk_init.parent_data = &miupll_parent;
	clk_init.num_parents = 1;
	miupll->clk_hw.init = &clk_init;

	ret = devm_clk_hw_register(dev, &miupll->clk_hw);
	if (ret)
		return ret;

	return of_clk_add_hw_provider(pdev->dev.of_node, of_clk_hw_simple_get, &miupll->clk_hw);
}

static struct platform_driver mstar_miupll_driver = {
	.driver = {
		.name = "mstar-miupll",
		.of_match_table = mstar_miupll_of_match,
	},
	.probe = mstar_miupll_probe,
};
builtin_platform_driver(mstar_miupll_driver);
