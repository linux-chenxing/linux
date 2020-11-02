// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/regmap.h>

/*
 * 0x0  - ??
 * write 0x00c0 - enable
 * write 0x01b2 - disable
 *
 * 0x1c - ??
 *         1         |        0
 * set when disabled | set when enabled
 */

#define REG_MAGIC	0x0
#define REG_ENABLED	0x1c
#define REG_EN_PWRDN	0x1c


/* based on the comments from the upll powerdown code in the i2m sdk kernel */
static const struct reg_field pd_field = REG_FIELD(REG_MAGIC, 1, 1);
static const struct reg_field enddisc_field = REG_FIELD(REG_MAGIC, 4, 4);
static const struct reg_field enfrun_field = REG_FIELD(REG_MAGIC, 5, 5);
static const struct reg_field enxtal_field = REG_FIELD(REG_MAGIC, 7, 7);
static const struct reg_field clk0_upll_384_en_field = REG_FIELD(REG_EN_PWRDN, 0, 0);
static const struct reg_field upll_en_prdt2 = REG_FIELD(REG_EN_PWRDN, 1, 1);
static const struct reg_field en_clk_upll_192m_field = REG_FIELD(REG_EN_PWRDN, 2, 2);
static const struct reg_field ctrl_pd_clk0_audio_field = REG_FIELD(REG_EN_PWRDN, 3, 3);

static const char* msc313_upll_outputs[] = {
	"384",
	"320",
};
static const u32 msc313_upll_output_rates[] = {
	384000000,
	320000000,
};

struct mstar_pll_output {
	struct mstar_upll *pll;
	u32 rate;
	struct clk_hw clk_hw;
};

struct mstar_upll {
	void __iomem *base;
	struct clk_onecell_data clk_data;
	struct mstar_pll_output *outputs;
	unsigned numoutputs;

	struct regmap_field *pd;
	struct regmap_field *pd_clk0_audio;
};

#define to_pll_output(_hw) container_of(_hw, struct mstar_pll_output, clk_hw)

static int mstar_pll_common_probe(struct platform_device *pdev, struct mstar_upll **pll,
		const struct clk_ops *clk_ops)
{
	struct device *dev = &pdev->dev;
	struct mstar_pll_output* output;
	struct clk_init_data clk_init = { };
	struct clk* clk;
	int numparents, numrates, pllindex;
	struct clk_onecell_data *clk_data;
	const char *parents[1];
	int numoutputs = ARRAY_SIZE(msc313_upll_outputs);

	numparents = of_clk_parent_fill(pdev->dev.of_node, parents, ARRAY_SIZE(parents));

	*pll = devm_kzalloc(&pdev->dev, sizeof(**pll), GFP_KERNEL);
	(*pll)->outputs = devm_kzalloc(&pdev->dev, sizeof((*pll)->outputs) * numoutputs, GFP_KERNEL);
	if (!(*pll)->outputs)
		return -ENOMEM;

	(*pll)->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR((*pll)->base))
		return PTR_ERR((*pll)->base);

	clk_data = &(*pll)->clk_data;
	clk_data->clk_num = numoutputs;
	clk_data->clks = devm_kcalloc(&pdev->dev, numoutputs, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_data->clks)
		return -ENOMEM;

	for (pllindex = 0; pllindex < numoutputs; pllindex++) {
		output = &(*pll)->outputs[pllindex];
		output->pll = *pll;

		output->clk_hw.init = &clk_init;
		clk_init.name = devm_kasprintf(dev, GFP_KERNEL, "%s_%s",
				dev_name(dev), msc313_upll_outputs[pllindex]);
		clk_init.ops = clk_ops;
		clk_init.num_parents = 1;
		clk_init.parent_names = parents;

		output->rate = msc313_upll_output_rates[pllindex];

		clk = clk_register(&pdev->dev, &output->clk_hw);
		if (IS_ERR(clk)) {
			printk("failed to register clk");
			return -ENOMEM;
		}
		clk_data->clks[pllindex] = clk;
	}

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get, clk_data);
}

static int msc313_upll_is_enabled(struct clk_hw *hw)
{
	struct mstar_pll_output *output = to_pll_output(hw);
	return ioread16(output->pll->base + REG_ENABLED) & BIT(0);
}

static unsigned long msc313_upll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct mstar_pll_output *output = to_pll_output(hw);
	return output->rate;
}

static const struct clk_ops msc313_upll_ops = {
	.is_enabled = msc313_upll_is_enabled,
	.recalc_rate = msc313_upll_recalc_rate,
};

static const struct of_device_id msc313_upll_of_match[] = {
	{ .compatible = "mstar,msc313-upll", },
	{}
};

static const struct regmap_config msc313_upll_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int msc313_upll_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *id;
	struct mstar_upll *upll;
	struct regmap *regmap;
	int ret = 0;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(msc313_upll_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	ret = mstar_pll_common_probe(pdev, &upll, &msc313_upll_ops);
	if(ret)
		goto out;

	regmap = devm_regmap_init_mmio(dev, upll->base, &msc313_upll_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	iowrite16(0x00c0, upll->base + REG_MAGIC);
	iowrite8(0x01, upll->base + REG_ENABLED);

	upll->pd = devm_regmap_field_alloc(dev, regmap, pd_field);
	upll->pd_clk0_audio = devm_regmap_field_alloc(dev, regmap, ctrl_pd_clk0_audio_field);

	platform_set_drvdata(pdev, upll);
out:
	return ret;
}

static struct platform_driver msc313_upll_driver = {
	.driver = {
		.name = "msc313-upll",
		.of_match_table = msc313_upll_of_match,
	},
	.probe = msc313_upll_probe,
};
builtin_platform_driver(msc313_upll_driver);
