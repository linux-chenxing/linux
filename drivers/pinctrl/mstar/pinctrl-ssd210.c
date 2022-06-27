// SPDX-License-Identifier: GPL-2.0
/* Pinctrl driver for the new "PADMUX" pinctrl on pioneer3 parts.
 *
 * Copyright (C) 2022 Daniel Palmer
 */

#include <dt-bindings/pinctrl/mstar.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-mstar.h"
#include "pinctrl-ssd210.h"
#include "pinctrl-ssd210-ssd210.h"

#define DRIVER_NAME "ssd210-pinctrl"

static const struct regmap_config ssd210_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

/* chip data tables .. */

static int ssd210_pinctrl_probe(struct platform_device *pdev)
{
	const struct msc313_pinctrl_info *match_data;
	struct msc313_pinctrl *pinctrl;
	void __iomem *base;
	int ret;

	/*
	 * Find the chip data, allocate the private structure
	 * and stuff the dev and chip data into it.
	 */
	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);
	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	/* Create the regmap */
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pinctrl->regmap = devm_regmap_init_mmio(pinctrl->dev, base,
			&ssd210_pinctrl_regmap_config);
	if (IS_ERR(pinctrl->regmap))
		return PTR_ERR(pinctrl->regmap);

	/* Register the pinctrl */
	pinctrl->desc.name = DRIVER_NAME;
	pinctrl->desc.pctlops = &msc313_pinctrl_ops;
	pinctrl->desc.pmxops = &mstar_pinmux_ops;
	pinctrl->desc.owner = THIS_MODULE;
	pinctrl->desc.pins = pinctrl->info->pins;
	pinctrl->desc.npins = pinctrl->info->npins;

	ret = devm_pinctrl_register_and_init(pinctrl->dev, &pinctrl->desc,
					     pinctrl, &pinctrl->pctl);
	if (ret)
		return ret;

	ret = mstar_pinctrl_parse_functions(pinctrl);
	ret = mstar_pinctrl_parse_groups(pinctrl);

	ret = pinctrl_enable(pinctrl->pctl);
	if (ret)
		dev_err(pinctrl->dev, "failed to enable pinctrl\n");

	// hack for i2c0 mode 0
	regmap_write(pinctrl->regmap, REG_SSD210_I2C0, 0x61);

	return 0;
}

static const struct of_device_id ssd210_pinctrl_of_match[] = {
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible	= "sstar,ssd210-pinctrl",
		.data		= &ssd210_info,
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
