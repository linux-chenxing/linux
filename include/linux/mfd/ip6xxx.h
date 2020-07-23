/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020, Daniel Palmer <daniel@thingy.jp>
 */

#ifndef __LINUX_MFD_IP6XXX_H
#define __LINUX_MFD_IP6XXX_H

#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/gpio/driver.h>

/* IP6303 */
#define IP6303_DC_CTL		0x20
#define IP6303_DC1_VSET		0x21
#define IP6303_DC2_VSET		0x26
#define IP6303_DC3_VSET		0x2b
#define IP6303_LDO_EN		0x40
#define IP6303_LDO3_VSEL	0x43
#define IP6303_LDO4_VSEL	0x44
#define IP6303_LDO5_VSEL	0x45
#define IP6303_LDO6_VSEL	0x46
#define IP6303_LDO7_VSEL	0x47
#define IP6303_CHG_ANA_CTL0	0x50
#define IP6303_CHG_ANA_CTL1	0x51
#define IP6303_CHG_DIG_CTL0	0x53
#define IP6303_CHG_DIG_CTL1	0x54
#define IP6303_CHG_DIG_CTL2	0x55
#define IP6303_CHG_DIG_CTL3	0x58
#define IP6303_ADC_ANA_CTL0	0x60
#define IP6303_ADC_DATA_VBAT	0x64
#define IP6303_ADC_DATA_IBAT	0x65
#define IP6303_ADC_DATA_ICHG	0x66
#define IP6303_ADC_DATA_GP1	0x67
#define IP6303_ADC_DATA_GP2	0x68
#define IP6303_MFP_CTL1		0x76
#define IP6303_GPIO_OE0		0x78
#define IP6303_GPIO_OE1		0x79
#define IP6303_GPIO_IE0		0x7a
#define IP6303_GPIO_IE1		0x7b
#define IP6303_GPIO_DAT0	0x7c
#define IP6303_GPIO_DAT1	0x7d
#define IP6303_GPIO_PU0		0x7e
#define IP6303_GPIO_PU1		0x7f
#define IP6303_GPIO_PD0		0x80
#define IP6303_GPIO_PD1		0x81

#define IP6303_DCDC_MIN_UV	600000
#define IP6303_DCDC_STEP_UV	12500
#define IP6303_DCDC_MAX_UV	3600000
#define IP6303_DCDC_VSEL_MASK	0xff
#define IP6303_DC1_EN_MASK	BIT(0)
#define IP6303_DC2_EN_MASK	BIT(1)
#define IP6303_DC3_EN_MASK	BIT(2)

#define IP6303_LDO_MIN_UV	700000
#define IP6303_LDO_STEP_UV	25000
#define IP6303_LDO_MAX_UV	3400000
#define IP6303_LDO_VSEL_MASK	0x7f

/* aka SVCC */
#define IP6303_SLDO1_MIN_UV	2600000
#define IP6303_SLDO1_STEP_UV	100000
#define IP6303_SLDO1_MAX_UV	3300000
#define IP6303_SLDO1_VSEL	0x4D
#define IP6303_SLDO1_VSEL_MASK	0x07

#define IP6303_SLDO2_MIN_UV	700000
#define IP6303_SLDO2_STEP_UV	100000
#define IP6303_SLDO2_MAX_UV	3800000
#define IP6303_SLDO2_VSEL	0x4D
#define IP6303_SLDO2_VSEL_MASK	0xF8

#define IP6303_SLDO2_EN_MASK	BIT(1)
#define IP6303_LDO2_EN_MASK	BIT(2)
#define IP6303_LDO3_EN_MASK	BIT(3)
#define IP6303_LDO4_EN_MASK	BIT(4)
#define IP6303_LDO5_EN_MASK	BIT(5)
#define IP6303_LDO6_EN_MASK	BIT(6)
#define IP6303_LDO7_EN_MASK	BIT(7)

static const struct reg_field ip6303_vbat_adc_en = {
	.reg = IP6303_ADC_ANA_CTL0,
	.lsb = 0,
	.msb = 0,
};

static const struct reg_field ip6303_ibat_adc_en = {
	.reg = IP6303_ADC_ANA_CTL0,
	.lsb = 1,
	.msb = 1,
};

static const struct reg_field ip6303_ichg_adc_en = {
	.reg = IP6303_ADC_ANA_CTL0,
	.lsb = 2,
	.msb = 2,
};

static const struct reg_field ip6303_adc_data_vbat = {
	.reg = IP6303_ADC_DATA_VBAT,
	.lsb = 0,
	.msb = 7,
};

static const struct reg_field ip6303_r_chgis = {
	.reg = IP6303_CHG_DIG_CTL0,
	.lsb = 0,
	.msb = 4,
};

static const struct reg_field ip6303_charge_state = {
	.reg = IP6303_CHG_DIG_CTL1,
	.lsb = 5,
	.msb = 7,
};

static const struct reg_field ip6303_batext_ok = {
	.reg = IP6303_CHG_DIG_CTL2,
	.lsb = 3,
	.msb = 3,
};

static const struct reg_field ip6303_chg_en = {
	.reg = IP6303_CHG_DIG_CTL3,
	.lsb = 1,
	.msb = 1,
};

static const struct reg_field ip6303_io8_mfp = {
	.reg = IP6303_MFP_CTL1,
	.lsb = 6,
	.msb = 7,
};

enum ip6xxx_charge_states {
	IP6XXX_CHARGE_STATE_IDLE,
	IP6XXX_CHARGE_STATE_TK,
	IP6XXX_CHARGE_STATE_CC,
	IP6XXX_CHARGE_STATE_UNDEFINED0,
	IP6XXX_CHARGE_STATE_UNDEFINED1,
	IP6XXX_CHARGE_STATE_CHG_END,
	IP6XXX_CHARGE_STATE_CHG_OVER_TIME,
};

#define IP6303_CHG_CUR_STEP_THRESHOLD	0x18
#define IP6303_CHR_CUR_STEP_LOW		25
#define IP6303_CHR_CUR_STEP_HIGH	50
#define IP6303_CHR_CUR_MAX		1000

enum ip6xxx_variants {
	IP6303_ID = 0,
	NR_IP6XXX_VARIANTS,
};

struct ip6xxx {
	enum ip6xxx_variants variant;
	struct regmap *regmap;

#ifdef CONFIG_CHARGER_IP6XXX
	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct regmap_field *vbat_adc_en;
	struct regmap_field *adc_data_vbat;
	struct regmap_field *r_chgis;
	struct regmap_field *charge_state;
	struct regmap_field *batext_ok;
	struct regmap_field *chg_en;
	struct regmap_field *io8_mfp;
#endif

#ifdef CONFIG_GPIO_IP6XXX
	struct gpio_chip gpiochip;
#endif
};

#endif /* __LINUX_MFD_IP6XXX_H */
