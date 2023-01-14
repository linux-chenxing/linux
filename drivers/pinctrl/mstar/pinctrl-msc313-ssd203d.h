// SPDX-License-Identifier: GPL-2.0
/*
 * Chipdata for ssd203d
 */

#ifndef DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_SSD203D_H_
#define DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_SSD203D_H_

#ifdef CONFIG_MACH_INFINITY
/* ssd203d */
static const struct pinctrl_pin_desc ssd203d_pins[] = {
	SSD203D_COMMON_PIN(GPIO85),
	SSD203D_COMMON_PIN(GPIO86),
	SSD203D_COMMON_PIN(HDMITX_SCL),
	SSD203D_COMMON_PIN(HDMITX_SDA),
	SSD203D_COMMON_PIN(HDMITXHPD),
	SSD203D_COMMON_PIN(HDMI2TXCN),
	SSD203D_COMMON_PIN(HDMI2TXCP),
	SSD203D_COMMON_PIN(HDMI2TX0N),
	SSD203D_COMMON_PIN(HDMI2TX0P),
	SSD203D_COMMON_PIN(HDMI2TX1N),
	SSD203D_COMMON_PIN(HDMI2TX1P),
	SSD203D_COMMON_PIN(HDMI2TX2N),
	SSD203D_COMMON_PIN(HDMI2TX2P),
};

/* i2c0 */
static const int ssd203d_i2c0_mode1_pins[] = {
	PIN_SSD203D_HDMITX_SCL,
	PIN_SSD203D_HDMITX_SCL,
};

static const char * const ssd203d_i2c0_groups[] = {
	GROUPNAME_I2C0_MODE1,
};

static const u16 ssd203d_i2c0_values[] = {
	SSD20XD_MODE(I2C0, 1),
};

#define SSD203D_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd203d_##_name##_pins)

static const struct msc313_pinctrl_group ssd203d_pinctrl_groups[] = {
	SSD203D_PINCTRL_GROUP(I2C0_MODE1, i2c0_mode1),
};

#define SSD203D_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSD20XD_##_NAME, \
	MASK_SSD20XD_##_NAME, ssd203d_##_name##_groups, ssd203d_##_name##_values)

static const struct msc313_pinctrl_function ssd203d_pinctrl_functions[] = {
	SSD203D_FUNCTION(I2C0, i2c0),
};

static const struct msc313_pinctrl_pinconf ssd203d_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd203d);
#endif /* infinity */

#endif /* DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_SSD203D_H_ */
