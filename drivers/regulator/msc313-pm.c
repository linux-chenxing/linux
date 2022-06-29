// SPDX-License-Identifier: GPL-2.0+
/*
 * MSC313 PM regulators driver
 * 2022 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define REG_BC	0xbc

static const struct regulator_ops msc313_pm_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
};

enum {
	MSC313_PM_TS,
};

static const struct regulator_desc msc313_pm_regulator_data[] = {
	{
		.name = "ts",
		.of_match = of_match_ptr("ts"),
		.regulators_node = of_match_ptr("regulators"),
		.id = MSC313_PM_TS,
		.ops = &msc313_pm_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.enable_reg = REG_BC,
		.enable_mask = BIT(2),
		.enable_val = BIT(2),
		.owner = THIS_MODULE,
	},
};

static int msc313_pm_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct regmap *regmap;

	printk("%s:%d\n", __func__, __LINE__);

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	config.dev = dev;
	config.regmap = regmap;
	config.of_node = dev->of_node;

	for (int i = 0; i < ARRAY_SIZE(msc313_pm_regulator_data); i++) {
		const struct regulator_desc *regulator_desc;
		struct regulator_dev *regulator_dev;

		regulator_desc = &msc313_pm_regulator_data[i];
		regulator_dev = devm_regulator_register(dev,
				regulator_desc, &config);
		if (IS_ERR(regulator_dev))
			return PTR_ERR(regulator_dev);
	}

	return 0;
}

static const struct of_device_id __maybe_unused msc313_pm_of_match[] = {
	{ .compatible = "mstar,msc313-pm-regulators" },
	{ },
};
MODULE_DEVICE_TABLE(of, msc313_pm_of_match);

static struct platform_driver msc313_pm_regulator_driver = {
	.driver = {
		.name = "msc313-pm-regulators",
		.of_match_table	= of_match_ptr(msc313_pm_of_match),
	},
	.probe = msc313_pm_probe,
};
module_platform_driver(msc313_pm_regulator_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("Regulator device driver MSC313 PM domain");
MODULE_LICENSE("GPL");
