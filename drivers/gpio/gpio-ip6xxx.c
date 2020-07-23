// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/regmap.h>
#include <linux/mfd/ip6xxx.h>

#define DRIVER_NAME "ip6xxx-gpio"

static void ip6xxx_gpio_set_gpiobit(struct ip6xxx *ip6xxx, unsigned high_reg,
		unsigned low_reg, unsigned which, int value)
{
	u16 bit = BIT(which + 1);
	regmap_update_bits(ip6xxx->regmap, high_reg, bit >> 8,
			value ? (bit >> 8) : 0);
	regmap_update_bits(ip6xxx->regmap, low_reg, bit >> 8,
			value ? bit & 0xff : 0);
}

static void ip6xxx_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct ip6xxx *ip6xxx = gpiochip_get_data(chip);

	ip6xxx_gpio_set_gpiobit(ip6xxx, IP6303_GPIO_DAT1,
			IP6303_GPIO_DAT0, offset, value);
}

static int ip6xxx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ip6xxx *ip6xxx = gpiochip_get_data(chip);
	return 0;
}

static int ip6xxx_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ip6xxx *ip6xxx = gpiochip_get_data(chip);

	ip6xxx_gpio_set_gpiobit(ip6xxx, IP6303_GPIO_OE1,
			IP6303_GPIO_OE0, offset, 0);
	ip6xxx_gpio_set_gpiobit(ip6xxx, IP6303_GPIO_IE1,
			IP6303_GPIO_IE0, offset, 1);

	return 0;
}

static int ip6xxx_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	struct ip6xxx *ip6xxx = gpiochip_get_data(chip);

	ip6xxx_gpio_set_gpiobit(ip6xxx, IP6303_GPIO_IE1,
			IP6303_GPIO_IE0, offset, 0);
	ip6xxx_gpio_set_gpiobit(ip6xxx, IP6303_GPIO_OE1,
			IP6303_GPIO_OE0, offset, 1);

	return 0;
}

static const char* names[] = {
		"gpio1",
		"gpio2",
		"gpio3",
		"gpio4",
		"gpio5",
		"gpio6",
		"gpio7",
		"gpio8",
		"gpio9",
		"gpio10",
		"gpio11",
};

static int ip6xxx_gpio_probe(struct platform_device *pdev)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(pdev->dev.parent);
	int ret;

	ip6xxx->gpiochip.label            = DRIVER_NAME;
	ip6xxx->gpiochip.parent           = &pdev->dev;
	ip6xxx->gpiochip.request          = gpiochip_generic_request;
	ip6xxx->gpiochip.free             = gpiochip_generic_free;
	ip6xxx->gpiochip.direction_input  = ip6xxx_gpio_direction_input;
	ip6xxx->gpiochip.get              = ip6xxx_gpio_get;
	ip6xxx->gpiochip.direction_output = ip6xxx_gpio_direction_output;
	ip6xxx->gpiochip.set              = ip6xxx_gpio_set;
	ip6xxx->gpiochip.base             = -1;
	ip6xxx->gpiochip.ngpio            = 11;
	ip6xxx->gpiochip.names            = names;

	ret = gpiochip_add_data(&ip6xxx->gpiochip, ip6xxx);
	if (ret < 0) {
		dev_err(pdev->dev.parent,"failed to register gpio chip");
		return ret;
	}

	return ret;
}

static struct platform_driver ip6xxx_gpio_driver = {
	.probe	= ip6xxx_gpio_probe,
	.driver	= {
		.name = DRIVER_NAME,
	},
};

module_platform_driver(ip6xxx_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("GPIO Driver for IP6XXX PMIC");
MODULE_ALIAS("platform:ip6xxx-gpio");
