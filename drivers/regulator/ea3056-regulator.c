// SPDX-License-Identifier: GPL-2.0+
/*
 * Everanalog ea3056 regulator driver
 * 2022 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define REGULATOR_VSET "regulator-vset"
#define EA3056_REG_ENABLE	0x0
#define EA3056_REG_VREF1	0x1
#define EA3056_REG_VREF2	0x2
#define EA3056_REG_VREF3	0x3
#define EA3056_REG_VREFLDO	0x4

#define VREF_DEFAULT	600
#define VREF_MIN	580
#define VREF_MAX	800
#define VREF_STEP	5
#define VREF_CTRL	BIT(6)
#define VREF_SEL_MASK	GENMASK_ULL(5, 0)
#define NUM_VOLTAGES	(((VREF_MAX - VREF_MIN) / VREF_STEP) + 1)

struct ea3056_regulator_config {
	unsigned int vselr1, vselr2;
};

static int ea3056_calculate_voltage(struct ea3056_regulator_config *regconfig, unsigned int vref)
{
	unsigned int tmp;

	if (regconfig->vselr1 == 0 || regconfig->vselr2 == 0)
		return 0;

	/* work out the ratio of the select resistors, result multiplied by 1000, the r1/r2 bit */
	tmp = (regconfig->vselr1 * 1000) / regconfig->vselr2;
	/* the +1 bit */
	tmp += 1000;
	/* multiply vref */
	tmp *= vref;

	/* result is multipled by 1000, the parameters are in mV so the result is now in uV */
	return tmp;
}

static int ea3056_list_voltage (struct regulator_dev *rdev, unsigned selector)
{
	struct ea3056_regulator_config *regconfig = rdev_get_drvdata(rdev);

	//printk("%s:%d %d - %d\n", __func__, __LINE__, selector, ea3056_calculate_voltage(regconfig,
	//		VREF_MIN + (selector * VREF_STEP)));

	return ea3056_calculate_voltage(regconfig,
			VREF_MIN + (selector * VREF_STEP));
}

static int ea3056_get_voltage(struct regulator_dev *rdev)
{
	struct ea3056_regulator_config *regconfig = rdev_get_drvdata(rdev);
	unsigned vref = VREF_DEFAULT;
	unsigned reg;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->vsel_reg, &reg);
	if (ret)
		return ret;

	/* The vref register only matters if it's enabled */
	if (reg & VREF_CTRL)
		vref = VREF_MIN + (VREF_STEP * (reg & rdev->desc->vsel_mask));

	return ea3056_calculate_voltage(regconfig, vref);
}

static int ea5036_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	return regmap_update_bits(rdev->regmap, rdev->desc->vsel_reg,
			rdev->desc->vsel_mask | VREF_CTRL, selector | VREF_CTRL);
}

static const struct regulator_ops ea3056_regulator_ops = {
	.list_voltage = ea3056_list_voltage,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_voltage = ea3056_get_voltage,
	.set_voltage_sel = ea5036_set_voltage_sel,
};

static int ea3056_parse_cb(struct device_node *np,
				  const struct regulator_desc *desc,
				  struct regulator_config *cfg)
{
	int vselcount, ret;
	struct ea3056_regulator_config *regconfig = cfg->driver_data;

	vselcount = of_property_count_u32_elems(np, REGULATOR_VSET);
	if (vselcount >= 0 && vselcount != 2){
		dev_err(cfg->dev, "Need 2 elements in vset\n");
		return -EINVAL;
	}
	if (vselcount < 0)
		return vselcount;

	ret = of_property_read_u32_index(np, REGULATOR_VSET, 0, &regconfig->vselr1);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(np, REGULATOR_VSET, 1, &regconfig->vselr2);
	if (ret)
		return ret;

	dev_info(cfg->dev, "vset r1: %d, r2: %d\n",
			regconfig->vselr1, regconfig->vselr2);

	return 0;
}

enum {
	EA3056_DCDC1,
	EA3056_DCDC2,
	EA3056_DCDC3,
	EA3056_LDO,
};

static const struct regulator_desc ea3056_regulator_data[] = {
	{
		.name = "dcdc1",
		.of_match = of_match_ptr("dcdc1"),
		.regulators_node = of_match_ptr("regulators"),
		.id = EA3056_DCDC1,
		.ops = &ea3056_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = NUM_VOLTAGES,
		.enable_reg = EA3056_REG_ENABLE,
		.enable_mask = BIT(0),
		.enable_val = BIT(0),
		.enable_is_inverted = true,
		.vsel_reg = EA3056_REG_VREF1,
		.vsel_mask = VREF_SEL_MASK,
		.owner = THIS_MODULE,
		.of_parse_cb = ea3056_parse_cb,
		.ramp_delay = 200,
	},
	{
		.name = "dcdc2",
		.of_match = of_match_ptr("dcdc2"),
		.regulators_node = of_match_ptr("regulators"),
		.id = EA3056_DCDC2,
		.ops = &ea3056_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = NUM_VOLTAGES,
		.enable_reg = EA3056_REG_ENABLE,
		.enable_mask = BIT(1),
		.enable_val = BIT(1),
		.enable_is_inverted = true,
		.vsel_reg = EA3056_REG_VREF2,
		.vsel_mask = VREF_SEL_MASK,
		.owner = THIS_MODULE,
		.of_parse_cb = ea3056_parse_cb,
		.ramp_delay = 200,
	},
	{
		.name = "dcdc3",
		.of_match = of_match_ptr("dcdc3"),
		.regulators_node = of_match_ptr("regulators"),
		.id = EA3056_DCDC3,
		.ops = &ea3056_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = NUM_VOLTAGES,
		.enable_reg = EA3056_REG_ENABLE,
		.enable_mask = BIT(2),
		.enable_val = BIT(2),
		.enable_is_inverted = true,
		.vsel_reg = EA3056_REG_VREF3,
		.vsel_mask = VREF_SEL_MASK,
		.owner = THIS_MODULE,
		.of_parse_cb = ea3056_parse_cb,
		.ramp_delay = 200,
	},
	{
		.name = "ldo",
		.of_match = of_match_ptr("ldo"),
		.regulators_node = of_match_ptr("regulators"),
		.id = EA3056_LDO,
		.ops = &ea3056_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = NUM_VOLTAGES,
		.enable_reg = EA3056_REG_ENABLE,
		.enable_mask = BIT(3),
		.enable_val = BIT(3),
		.enable_is_inverted = true,
		.vsel_reg = EA3056_REG_VREFLDO,
		.vsel_mask = VREF_SEL_MASK,
		.owner = THIS_MODULE,
		.of_parse_cb = ea3056_parse_cb,
		.ramp_delay = 200,
	},
};

static const struct regmap_config ea3056_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x4,
	.cache_type = REGCACHE_FLAT,
};

static int ea3056_i2c_probe(struct i2c_client *i2c)
{
	struct regulator_config config = { };
	struct device *dev = &i2c->dev;
	struct regmap *regmap;
	unsigned int dummyread;

	regmap = devm_regmap_init_i2c(i2c, &ea3056_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/*
	 * if reset happens at a weird time the ea3056 doesn't
	 * respond to the first i2c transaction. Do a dummy
	 * read. I think the device sees a stop condition on
	 * the bus and then goes back to normal.
	 */
	regmap_read(regmap, EA3056_REG_ENABLE, &dummyread);

	config.dev = &i2c->dev;
	config.regmap = regmap;
	config.of_node = dev->of_node;

	for (int i = 0; i < ARRAY_SIZE(ea3056_regulator_data); i++) {
		const struct regulator_desc *regulator_desc;
		struct ea3056_regulator_config *regconfig;
		struct regulator_dev *regulator_dev;

		regconfig = devm_kzalloc(dev, sizeof(*regconfig), GFP_KERNEL);
		if (!regconfig)
			return -ENOMEM;

		config.driver_data = regconfig;

		regulator_desc = &ea3056_regulator_data[i];
		regulator_dev = devm_regulator_register(dev,
				regulator_desc, &config);
		if (IS_ERR(regulator_dev))
			return PTR_ERR(regulator_dev);
	}

	return 0;
}

static const struct of_device_id __maybe_unused ea3056_i2c_of_match[] = {
	{ .compatible = "everanalog,ea3056" },
	{ },
};
MODULE_DEVICE_TABLE(of, ea3056_i2c_of_match);

static const struct i2c_device_id ea3056_i2c_id[] = {
	{ "ea3056", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ea3056_i2c_id);

static struct i2c_driver ea3056_regulator_driver = {
	.driver = {
		.name = "ea3056",
		.of_match_table	= of_match_ptr(ea3056_i2c_of_match),
	},
	.probe = ea3056_i2c_probe,
	.id_table = ea3056_i2c_id,
};
module_i2c_driver(ea3056_regulator_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("Regulator device driver for Everanalog ea3056");
MODULE_LICENSE("GPL");
