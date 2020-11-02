// SPDX-License-Identifier: GPL-2.0
/*
 * MStar MSC313 LPLL driver
 *
 * Copyright (C) 2019 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#define REG_CTRL 0x0
#define REG_LOOP 0x4

struct msc313_lpll {
	struct clk_hw clk_hw;
	struct regmap_field *pd;
	struct regmap_field *ictrl;
	struct regmap_field *input_div;
	struct clk_hw_onecell_data *clk_data;
};

#define to_lpll(_hw) container_of(_hw, struct msc313_lpll, clk_hw)

static const struct reg_field ctrl_pd = REG_FIELD(REG_CTRL, 15, 15);
static const struct reg_field ctrl_ictrl = REG_FIELD(REG_CTRL, 0, 2);
static const struct reg_field input_div = REG_FIELD(REG_LOOP, 0, 1);

static int msc313_lpll_enable(struct clk_hw *hw)
{
	struct msc313_lpll *lpll = to_lpll(hw);

	regmap_field_write(lpll->pd, 0);

	return 0;
}

static void msc313_lpll_disable(struct clk_hw *hw)
{
	struct msc313_lpll *lpll = to_lpll(hw);

	regmap_field_write(lpll->pd, 1);
}

static int msc313_lpll_is_enabled(struct clk_hw *hw){
	struct msc313_lpll *lpll = to_lpll(hw);
	unsigned pd;

	regmap_field_read(lpll->pd, &pd);

	return pd == 0 ? 1 : 0;
}

static unsigned long msc313_lpll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct msc313_lpll *lpll = to_lpll(hw);
	unsigned long output_rate = parent_rate;
	unsigned div_first;

	regmap_field_read(lpll->input_div, &div_first);

	return output_rate;
}

static const struct clk_ops msc313_lpll_ops = {
	.enable = msc313_lpll_enable,
	.disable = msc313_lpll_disable,
	.is_enabled = msc313_lpll_is_enabled,
	.recalc_rate = msc313_lpll_recalc_rate,
};

static const struct regmap_config msc313_mpll_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static const struct clk_parent_data lpll_parent = {
	.index	= 0,
};

static int msc313_lpll_probe(struct platform_device *pdev)
{
	struct clk_init_data clk_init = { };
	struct device *dev = &pdev->dev;
	void __iomem *base;
	struct msc313_lpll* lpll;
	struct regmap *regmap;
	int ret;

	lpll = devm_kzalloc(dev, sizeof(*lpll), GFP_KERNEL);
	if(!lpll)
		return -ENOMEM;

	lpll->clk_data = devm_kzalloc(dev, struct_size(lpll->clk_data, hws, 1),
			GFP_KERNEL);
	if (!lpll->clk_data)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &msc313_mpll_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	lpll->ictrl = devm_regmap_field_alloc(dev, regmap, ctrl_ictrl);
	lpll->pd = devm_regmap_field_alloc(dev, regmap, ctrl_pd);
	lpll->input_div = devm_regmap_field_alloc(dev, regmap, input_div);

	clk_init.name = dev_name(dev);
	clk_init.ops = &msc313_lpll_ops;
	clk_init.parent_data = &lpll_parent;
	clk_init.num_parents = 1;

	lpll->clk_hw.init = &clk_init;
	ret = devm_clk_hw_register(dev, &lpll->clk_hw);
	if(ret)
		return ret;

	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get,
			lpll->clk_data);
}

static const struct of_device_id msc313_lpll_of_match[] = {
	{
		.compatible = "mstar,msc313-lpll",
	},
	{}
};

static struct platform_driver msc313_lpll_driver = {
	.driver = {
		.name = "mstar-lpll",
		.of_match_table = msc313_lpll_of_match,
	},
	.probe = msc313_lpll_probe,
};
builtin_platform_driver(msc313_lpll_driver);
