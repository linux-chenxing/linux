// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020, Daniel Palmer
 */

#include <linux/hw_random.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/clk.h>

#define REG_CTRL	0x0
#define REG_VALUE	0x8
#define REG_STATUS	0xc

static struct reg_field ctrl_enable = REG_FIELD(REG_CTRL, 7, 7);
static struct reg_field status_ready = REG_FIELD(REG_STATUS, 0, 0);

struct msc313_rng {
	struct hwrng hwrng;
	struct regmap *regmap;
	struct clk *clk;
	struct regmap_field *enable;
	struct regmap_field *ready;
};

static int msc313_rng_read(struct hwrng *hwrng, void *data, size_t max, bool wait)
{
	struct msc313_rng *rng = container_of(hwrng, struct msc313_rng, hwrng);
	unsigned int value;
	int i, ret = 0;

	while (ret < max) {
		if (regmap_field_read_poll_timeout(rng->ready, value, value == 1, 0, 1000000)) {
			pr_warn("timeout waiting for rdy\n");
			goto out;
		}
		regmap_read(rng->regmap, REG_VALUE, &value);
		for (i = 0; i < 2 && ret < max; i++) {
			*((u8*)(data) + ret) = value >> (8 * i);
			ret += 1;
		}
	}

out:
	return ret;
}

static const struct regmap_config msc313_rng_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int msc313_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_rng *rng;
	struct resource *res;
	void __iomem *base;
	int ret;

	rng = devm_kzalloc(dev, sizeof(*rng), GFP_KERNEL);
	if (!rng) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto out;
	}

	rng->regmap = devm_regmap_init_mmio(dev, base,
				&msc313_rng_regmap_config);
	if (IS_ERR(rng->regmap)) {
		ret = PTR_ERR(rng->regmap);
		goto out;
	}

	rng->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rng->clk)) {
		return PTR_ERR(rng->clk);
	}

	ret = clk_prepare_enable(rng->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed: %d\n", ret);
	}

	rng->enable = devm_regmap_field_alloc(dev, rng->regmap, ctrl_enable);
	rng->ready = devm_regmap_field_alloc(dev, rng->regmap, status_ready);

	regmap_field_write(rng->enable, 1);

	rng->hwrng.name = dev_driver_string(dev),
	rng->hwrng.read = msc313_rng_read,
	rng->hwrng.quality = 0;

	ret = devm_hwrng_register(dev, &rng->hwrng);
out:
	return ret;
}

static const struct of_device_id msc313_rng_match[] = {
	{
		.compatible = "mstar,msc313-rng",
	},
	{},
};
MODULE_DEVICE_TABLE(of, msc313_rng_match);

static struct platform_driver msc313_rng_driver = {
	.driver = {
		.name = "msc313-rng",
		.of_match_table = msc313_rng_match,
	},
	.probe = msc313_rng_probe,
};

module_platform_driver(msc313_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("MStar MSC313 RNG driver");
