// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>

#include <dt-bindings/pinctrl/mstar.h>

#include "../core.h"
#include "../devicetree.h"
#include "../pinconf.h"
#include "../pinmux.h"

#include "pinctrl-mstar.h"
#include "pinctrl-msc313.h"
#include "pinctrl-msc313-msc313.h"
#include "pinctrl-msc313-ssd20xd.h"
#include "pinctrl-msc313-ssd203d.h"
#include "pinctrl-msc313-ssc8336.h"

#define DRIVER_NAME "pinctrl-msc313"

static const struct regmap_config msc313_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int mstar_set_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long config)
{
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int i;
	unsigned int mask;
	const struct msc313_pinctrl_pinconf *confpin;

	dev_dbg(pinctrl->dev, "setting %d:%u on pin %d\n", (int)config, (unsigned int)arg, pin);
	for (i = 0; i < pinctrl->info->npinconfs; i++) {
		if (pinctrl->info->pinconfs[i].pin == pin) {
			confpin = &pinctrl->info->pinconfs[i];
			switch (param) {
			case PIN_CONFIG_BIAS_PULL_UP:
				if (confpin->pull_en_reg != -1) {
					dev_dbg(pinctrl->dev, "setting pull up %d on pin %d\n", (int) arg, pin);
					mask = 1 << confpin->pull_en_bit;
					regmap_update_bits(pinctrl->regmap, confpin->pull_en_reg, mask, arg ? mask : 0);
				} else
					dev_info(pinctrl->dev, "pullup reg/bit isn't known for pin %d\n", pin);
			default:
				break;
			}
			return 0;
		}
	}
	return 0;
}

/*
 * Check if a pin is one that is always pulled up or down
 * then check if there is an optional pull up or down, then
 * check if that is always up or down, and finally if there
 * is a direction bit check that for the direction.
 */
static bool msc313_pinctrl_ispulled(struct msc313_pinctrl *pinctrl,
		const struct msc313_pinctrl_pinconf *confpin, bool down)
{
	unsigned int val;

	if (confpin->pull_en_reg == ALWAYS_PULLUP)
		return !down;
	else if (confpin->pull_en_reg == ALWAYS_PULLDOWN)
		return down;
	else if (confpin->pull_en_reg != NOREG) {
		regmap_read(pinctrl->regmap, confpin->pull_en_reg, &val);
		if (val & BIT(confpin->pull_en_bit)) {
			if (confpin->pull_dir_reg == ALWAYS_PULLUP)
				return !down;
			else if (confpin->pull_dir_reg == ALWAYS_PULLDOWN)
				return down;
			else if (confpin->pull_en_reg != NOREG) {
				regmap_read(pinctrl->regmap, confpin->pull_dir_reg, &val);
				if (val & BIT(confpin->pull_dir_bit))
					return !down;
				else
					return down;
			} else
				return false;
		} else
			return false;
	} else
		return false;
}

static int msc313_pinctrl_get_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long *config)
{
	int i;
	const struct msc313_pinctrl_pinconf *confpin;
	unsigned int val;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int crntidx;

	/* we only support a limited range of conf options so filter the
	 * ones we do here
	 */

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_DRIVE_STRENGTH:
		break;
	default:
		return -ENOTSUPP;
	}

	/* try to find the configuration register(s) for the pin */
	for (i = 0; i < pinctrl->info->npinconfs; i++) {
		if (pinctrl->info->pinconfs[i].pin == pin) {
			confpin = &pinctrl->info->pinconfs[i];
			switch (param) {
			case PIN_CONFIG_BIAS_PULL_UP:
				return msc313_pinctrl_ispulled(pinctrl, confpin, false) ? 0 : -EINVAL;
			case PIN_CONFIG_BIAS_PULL_DOWN:
				return msc313_pinctrl_ispulled(pinctrl, confpin, true) ? 0 : -EINVAL;
			case PIN_CONFIG_DRIVE_STRENGTH:
				if (confpin->drive_reg != -1) {
					regmap_read(pinctrl->regmap, confpin->drive_reg, &val);
					crntidx = (val >> confpin->drive_lsb) & BIT_MASK(confpin->drive_width);
					*config = pinconf_to_config_packed(param, confpin->drivecurrents[crntidx]);
					return 0;
				}
				return -ENOTSUPP;
			default:
				return -ENOTSUPP;
			}
		}
	}

	return -ENOTSUPP;
}

static int mstar_pin_config_get(struct pinctrl_dev *pctldev,
			       unsigned int pin,
			       unsigned long *config)
{
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return msc313_pinctrl_get_config(pinctrl, pin, config);
}

static int mstar_pin_config_set(struct pinctrl_dev *pctldev,
			       unsigned int pin,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	int i;
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	for (i = 0; i < num_configs; i++)
		mstar_set_config(pinctrl, pin, configs[i]);
	return 0;
}

static int mstar_pin_config_group_get(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *config)
{
	return -ENOTSUPP;
}

static int mstar_pin_config_group_set(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *configs,
				     unsigned int num_configs)
{
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct group_desc *group = pinctrl_generic_get_group(pctldev, selector);
	int i, j, ret;

	for (i = 0; i < group->num_pins; i++) {
		for (j = 0; j < num_configs; j++) {
			ret = mstar_set_config(pinctrl, group->pins[i], configs[j]);
			if (ret)
				return ret;
		}
	}
	return 0;
}

static const struct pinconf_ops mstar_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = mstar_pin_config_get,
	.pin_config_set = mstar_pin_config_set,
	.pin_config_group_get = mstar_pin_config_group_get,
	.pin_config_group_set = mstar_pin_config_group_set,
};

static int msc313_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct msc313_pinctrl *pinctrl;
	const struct msc313_pinctrl_info *match_data;
	void __iomem *base;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pinctrl->regmap = devm_regmap_init_mmio(pinctrl->dev, base,
			&msc313_pinctrl_regmap_config);
	if (IS_ERR(pinctrl->regmap))
		return PTR_ERR(pinctrl->regmap);

	pinctrl->desc.name = DRIVER_NAME;
	pinctrl->desc.pctlops = &msc313_pinctrl_ops;
	pinctrl->desc.pmxops = &mstar_pinmux_ops;
	pinctrl->desc.confops = &mstar_pinconf_ops;
	pinctrl->desc.owner = THIS_MODULE;
	pinctrl->desc.pins = pinctrl->info->pins;
	pinctrl->desc.npins = pinctrl->info->npins;

	ret = devm_pinctrl_register_and_init(pinctrl->dev, &pinctrl->desc,
					     pinctrl, &pinctrl->pctl);

	if (ret) {
		dev_err(pinctrl->dev, "failed to register pinctrl\n");
		return ret;
	}

	ret = mstar_pinctrl_parse_functions(pinctrl);
	ret = mstar_pinctrl_parse_groups(pinctrl);

	ret = pinctrl_enable(pinctrl->pctl);
	if (ret)
		dev_err(pinctrl->dev, "failed to enable pinctrl\n");

	return 0;
}

static const struct of_device_id msc313_pinctrl_of_match[] = {
#ifdef CONFIG_MACH_INFINITY
	{
		.compatible	= "mstar,msc313-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "mstar,msc313e-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "sstar,ssd20xd-pinctrl",
		.data		= &ssd20xd_info,
	},
	{
		.compatible	= "sstar,ssd203d-pinctrl",
		.data		= &ssd203d_info,
	},
#endif
#ifdef CONFIG_MACH_MERCURY
	{
		.compatible	= "mstar,ssc8336-pinctrl",
		.data		= &ssc8336n_info,
	},
	{
		.compatible	= "mstar,ssc8336n-pinctrl",
		.data		= &ssc8336n_info,
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, msc313_pinctrl_of_match);

static struct platform_driver msc313_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_pinctrl_of_match,
	},
	.probe = msc313_pinctrl_probe,
};
module_platform_driver(msc313_pinctrl_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("Pin controller driver for MStar SoCs");
MODULE_LICENSE("GPL v2");
