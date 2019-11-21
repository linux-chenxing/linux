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
#include <linux/mfd/ip6xxx.h>

static enum power_supply_property ip6xxx_charger_properties[] = {
	/*POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,*/
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_STATUS
};

struct ip6xxx_charger_data {
	const struct regulator_desc *regulators;
	const int nregulators;
};

static const struct ip6xxx_charger_data ip6xxx_charger_data[] =
{
	[IP6303_ID] = {
	},
};

static int ip6xxx_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct ip6xxx *ip6xxx = power_supply_get_drvdata(psy);
	unsigned int chg_en, batext_ok, vbat_adc_en, vbat, regval;

	regmap_field_read(ip6xxx->chg_en, &chg_en);
	regmap_field_read(ip6xxx->batext_ok, &batext_ok);
	regmap_field_read(ip6xxx->vbat_adc_en, &vbat_adc_en);

	printk("vbat_adc_en %u, chrg_en %u, batext_ok %u\n",
			vbat_adc_en, chg_en, batext_ok);

	regmap_field_read(ip6xxx->adc_data_vbat, &vbat);
	vbat = ((vbat * 15625) / 1000) + 500;
	printk("battery voltage %umV\n", vbat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		regmap_field_read(ip6xxx->charge_state, &regval);
		switch(regval){
		case IP6XXX_CHARGE_STATE_IDLE:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case IP6XXX_CHARGE_STATE_TK:
		case IP6XXX_CHARGE_STATE_CC:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case IP6XXX_CHARGE_STATE_CHG_END:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		regmap_field_read(ip6xxx->charge_state, &regval);
		switch(regval){
		case IP6XXX_CHARGE_STATE_TK:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case IP6XXX_CHARGE_STATE_CC:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		regmap_field_read(ip6xxx->r_chgis, &regval);
		if(regval >= IP6303_CHG_CUR_STEP_THRESHOLD)
			regval *= IP6303_CHR_CUR_STEP_HIGH;
		else
			regval *= IP6303_CHR_CUR_STEP_LOW;
		val->intval = regval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = IP6303_CHR_CUR_MAX;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ip6xxx_charger_probe(struct platform_device *pdev)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(pdev->dev.parent);
	const struct ip6xxx_charger_data *charger_data =
			&ip6xxx_charger_data[ip6xxx->variant];
	struct power_supply_config psy_cfg = {};
	int ret;

	ip6xxx->charger_desc.properties = ip6xxx_charger_properties;
	ip6xxx->charger_desc.num_properties = ARRAY_SIZE(ip6xxx_charger_properties);
	ip6xxx->charger_desc.get_property = ip6xxx_charger_get_property;
	ip6xxx->charger_desc.name = pdev->dev.parent->of_node->name;
	ip6xxx->charger_desc.type = 0;

	psy_cfg.of_node = pdev->dev.parent->of_node;
	psy_cfg.drv_data = ip6xxx;

	ip6xxx->vbat_adc_en = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_vbat_adc_en);

	ip6xxx->adc_data_vbat = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_adc_data_vbat);

	ip6xxx->r_chgis = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_r_chgis);

	ip6xxx->charge_state = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_charge_state);

	ip6xxx->batext_ok = devm_regmap_field_alloc(pdev->dev.parent,
				ip6xxx->regmap, ip6303_batext_ok);

	ip6xxx->chg_en = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_chg_en);

	ip6xxx->io8_mfp = devm_regmap_field_alloc(pdev->dev.parent,
			ip6xxx->regmap, ip6303_io8_mfp);

	ip6xxx->charger = devm_power_supply_register(pdev->dev.parent,
			&ip6xxx->charger_desc, &psy_cfg);

	regmap_field_write(ip6xxx->r_chgis, 0x8);
	regmap_field_write(ip6xxx->chg_en, 0x1);
	regmap_field_write(ip6xxx->vbat_adc_en, 0x1);
	regmap_field_write(ip6xxx->io8_mfp, 0x2);

	if (IS_ERR(ip6xxx->charger)) {
		ret = PTR_ERR(ip6xxx->charger);
	}

	return ret;
}

static struct platform_driver ip6xxx_charger_driver = {
	.probe	= ip6xxx_charger_probe,
	.driver	= {
		.name = "ip6xxx-charger",
	},
};

module_platform_driver(ip6xxx_charger_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("Charger Driver for IP6XXX PMIC");
MODULE_ALIAS("platform:ip6xxx-charger");
