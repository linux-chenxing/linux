// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  MFD core driver for Injoinic IP6XXX series PMICs
 *
 * Copyright (C) 2020 <daniel@thingy.jp>
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ip6xxx.h>

struct ip6xxx_data {
	enum ip6xxx_variants variant;
	const struct mfd_cell *mfd_cells;
	unsigned nmfd_cells;
};

static const struct mfd_cell ip6303_cells[] = {
	{
		.name		= "ip6xxx-regulator",
	},
	{
		.name		= "ip6xxx-charger",
	},
};

static const struct ip6xxx_data ip6303_data = {
		.variant = IP6303_ID,
		.mfd_cells = ip6303_cells,
		.nmfd_cells = ARRAY_SIZE(ip6303_cells),
};

static const struct regmap_config ip6xxx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ip6xxx_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct ip6xxx *ip6xxx;
	const struct ip6xxx_data *match_data;
	int ret;

	match_data = of_device_get_match_data(&i2c->dev);
	if (!match_data)
		return -EINVAL;

	ip6xxx = devm_kzalloc(&i2c->dev, sizeof(*ip6xxx), GFP_KERNEL);

	ip6xxx->regmap = devm_regmap_init_i2c(i2c, &ip6xxx_regmap_config);
	if(IS_ERR(ip6xxx->regmap)){
		return PTR_ERR(ip6xxx->regmap);
	}

	dev_set_drvdata(&i2c->dev, ip6xxx);

	ret = mfd_add_devices(&i2c->dev, -1, match_data->mfd_cells,
			match_data->nmfd_cells, NULL, 0, NULL);

	return ret;
}

static const struct of_device_id ip6xxx_i2c_of_match[] = {
	{
		.compatible = "injoinic,ip6303",
		.data = &ip6303_data,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ip6xxx_i2c_of_match);

static const struct i2c_device_id ip6xxx_i2c_id[] = {
	{ "ip6303", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, i2c_device_id);

static struct i2c_driver ip6xxx_driver = {
	.driver = {
		.name = "ip6xxx",
		.of_match_table	= of_match_ptr(ip6xxx_i2c_of_match),
	},
	.probe = ip6xxx_i2c_probe,
	.id_table = ip6xxx_i2c_id,
};

module_i2c_driver(ip6xxx_driver);

MODULE_DESCRIPTION("PMIC MFD core driver for IP6XXX");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL");
