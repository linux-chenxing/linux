// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Injoinic IP6XXX regulators driver.
 *
 * Copyright (C) 2020 <daniel@thingy.jp>
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/ip6xxx.h>

struct ip6xxx_regulator_data {
	const struct regulator_desc *regulators;
	const int nregulators;
};

static const struct regulator_ops ip6xxx_ops = {
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

#define IP6XXX_REGULATOR(_name,_id,_vset, _vsetmask, _min,_step,_max, _en, _enmask) { \
				.owner = THIS_MODULE, \
				.type = REGULATOR_VOLTAGE, \
				.ramp_delay = 200, \
				.ops = &ip6xxx_ops, \
				.min_uV =  _min, \
				.uV_step = _step, \
				.n_voltages = ((_max - _min) / _step) + 1, \
				.vsel_mask = _vsetmask, \
				.name = _name, \
				.of_match = of_match_ptr(_name), \
				.regulators_node= of_match_ptr("regulators"), \
				.id = _id, \
				.vsel_reg = _vset, \
				.enable_reg = _en, \
				.enable_mask = _enmask, \
				.enable_val = _enmask, \
				.disable_val = 0, \
			}

#define IP6XXX_DCDC_REGULATOR(_name,_id,_vset,_enmask) IP6XXX_REGULATOR(_name,_id,_vset, \
		IP6303_DCDC_VSEL_MASK, \
		IP6303_DCDC_MIN_UV, \
		IP6303_DCDC_STEP_UV, \
		IP6303_DCDC_MAX_UV, \
		IP6303_DC_CTL, \
		_enmask)

#define IP6XXX_LDO_REGULATOR(_name,_id,_vset, _enmask) IP6XXX_REGULATOR(_name,_id,_vset, \
		IP6303_LDO_VSEL_MASK, \
		IP6303_LDO_MIN_UV, \
		IP6303_LDO_STEP_UV, \
		IP6303_LDO_MAX_UV, \
		IP6303_LDO_EN, \
		_enmask)

static const struct regulator_desc ip6303_regulators[] = {
		IP6XXX_DCDC_REGULATOR("dc1", 0, IP6303_DC1_VSET, IP6303_DC1_EN_MASK),
		IP6XXX_DCDC_REGULATOR("dc2", 1, IP6303_DC2_VSET, IP6303_DC2_EN_MASK),
		IP6XXX_DCDC_REGULATOR("dc3", 2, IP6303_DC3_VSET, IP6303_DC3_EN_MASK),
		IP6XXX_REGULATOR("sldo1", 3, IP6303_SLDO1_VSEL, \
				IP6303_SLDO1_VSEL_MASK, IP6303_SLDO1_MIN_UV, \
				IP6303_SLDO1_STEP_UV,IP6303_SLDO1_MAX_UV, 0, 0),
		IP6XXX_REGULATOR("sldo2", 4, IP6303_SLDO2_VSEL, \
				IP6303_SLDO2_VSEL_MASK, IP6303_SLDO2_MIN_UV, \
				IP6303_SLDO2_STEP_UV,IP6303_SLDO2_MAX_UV, IP6303_LDO_EN, IP6303_SLDO2_EN_MASK),
		IP6XXX_LDO_REGULATOR("ldo3", 5, IP6303_LDO3_VSEL, IP6303_LDO2_EN_MASK),
		IP6XXX_LDO_REGULATOR("ldo4", 6, IP6303_LDO4_VSEL, IP6303_LDO2_EN_MASK),
		IP6XXX_LDO_REGULATOR("ldo5", 7, IP6303_LDO5_VSEL, IP6303_LDO2_EN_MASK),
		IP6XXX_LDO_REGULATOR("ldo6", 8, IP6303_LDO6_VSEL, IP6303_LDO2_EN_MASK),
		IP6XXX_LDO_REGULATOR("ldo7", 9, IP6303_LDO7_VSEL, IP6303_LDO2_EN_MASK),
};

static const struct ip6xxx_regulator_data ip6xxx_regulator_data[] =
{
	[IP6303_ID] = {
			.regulators = ip6303_regulators,
			.nregulators = ARRAY_SIZE(ip6303_regulators),
	},
};

static int ip6xxx_regulator_probe(struct platform_device *pdev)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(pdev->dev.parent);
	const struct ip6xxx_regulator_data *regulator_data =
			&ip6xxx_regulator_data[ip6xxx->variant];
	struct regulator_dev *rdev;
	struct regulator_config config = { 0 };
	int i;

	config.dev = pdev->dev.parent,
	config.regmap = ip6xxx->regmap;

	for(i = 0; i < regulator_data->nregulators; i++){
		rdev = devm_regulator_register(&pdev->dev,
					       &regulator_data->regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "Failed to register regulator\n");
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

static struct platform_driver ip6xxx_regulator_driver = {
	.probe	= ip6xxx_regulator_probe,
	.driver	= {
		.name = "ip6xxx-regulator",
	},
};

module_platform_driver(ip6xxx_regulator_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("Regulator Driver for IP6XXX PMIC");
MODULE_ALIAS("platform:ip6xxx-regulator");
