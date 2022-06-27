// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME "ssd210-pinctrl"

struct ssd210_pinctrl {
	struct device *dev;
	struct regmap *regmap;
};

static const struct regmap_config ssd210_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int ssd210_pinctrl_probe(struct platform_device *pdev)
{
	struct ssd210_pinctrl *pinctrl;
	void __iomem *base;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pinctrl->regmap = devm_regmap_init_mmio(pinctrl->dev, base,
			&ssd210_pinctrl_regmap_config);
	if (IS_ERR(pinctrl->regmap))
		return PTR_ERR(pinctrl->regmap);

	// hack for i2c0 mode 0
	regmap_write(pinctrl->regmap, 0x1bc, 0x61);

	return 0;
}

static const struct of_device_id ssd210_pinctrl_of_match[] = {
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible	= "sstar,ssd210-pinctrl",
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, ssd210_pinctrl_of_match);

static struct platform_driver ssd210_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ssd210_pinctrl_of_match,
	},
	.probe = ssd210_pinctrl_probe,
};
module_platform_driver(ssd210_pinctrl_driver);
