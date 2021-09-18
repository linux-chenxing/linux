// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Daniel Palmer<daniel@thingy.jp> */

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <dt-bindings/gpio/ssd210-gpio.h>

#define DRIVER_NAME "gpio-ssd210"

#define SSD210_GPIO_IN  BIT(0)
#define SSD210_GPIO_OUT BIT(1)
#define SSD210_GPIO_OEN BIT(2)

/* sr pins */
#define SSD210_PINNAME_SR_IO0		"sr_io0"
#define SSD210_PINNAME_SR_IO1		"sr_io1"
#define SSD210_PINNAME_SR_IO2		"sr_io2"
#define SSD210_PINNAME_SR_IO3		"sr_io3"
#define SSD210_PINNAME_SR_IO4		"sr_io4"
#define SSD210_PINNAME_SR_IO5		"sr_io5"
#define SSD210_PINNAME_SR_IO6		"sr_io6"
#define SSD210_PINNAME_SR_IO7		"sr_io7"
#define SSD210_PINNAME_SR_IO8		"sr_io8"
#define SSD210_PINNAME_SR_IO9		"sr_io9"
#define SSD210_PINNAME_SR_IO10		"sr_io10"
#define SSD210_PINNAME_SR_IO11		"sr_io11"
#define SSD210_PINNAME_SR_IO12		"sr_io12"
#define SSD210_PINNAME_SR_IO13		"sr_io13"
#define SSD210_PINNAME_SR_IO14		"sr_io14"
#define SSD210_PINNAME_SR_IO15		"sr_io15"
#define SSD210_PINNAME_SR_IO16		"sr_io16"

#define SR_NAMES		\
	SSD210_PINNAME_SR_IO0,	\
	SSD210_PINNAME_SR_IO1,	\
	SSD210_PINNAME_SR_IO2,	\
	SSD210_PINNAME_SR_IO3,	\
	SSD210_PINNAME_SR_IO4,	\
	SSD210_PINNAME_SR_IO5,	\
	SSD210_PINNAME_SR_IO6,	\
	SSD210_PINNAME_SR_IO7,	\
	SSD210_PINNAME_SR_IO8,	\
	SSD210_PINNAME_SR_IO9,	\
	SSD210_PINNAME_SR_IO10,	\
	SSD210_PINNAME_SR_IO11,	\
	SSD210_PINNAME_SR_IO12,	\
	SSD210_PINNAME_SR_IO13,	\
	SSD210_PINNAME_SR_IO14,	\
	SSD210_PINNAME_SR_IO15,	\
	SSD210_PINNAME_SR_IO16

#define OFF_SR_IO0	0x00
#define OFF_SR_IO1	0x04
#define OFF_SR_IO2	0x08
#define OFF_SR_IO3	0x0c
#define OFF_SR_IO4	0x10
#define OFF_SR_IO5	0x14
#define OFF_SR_IO6	0x18
#define OFF_SR_IO7	0x1c
#define OFF_SR_IO8	0x20
#define OFF_SR_IO9	0x24
#define OFF_SR_IO10	0x28
#define OFF_SR_IO11	0x2c
#define OFF_SR_IO12	0x30
#define OFF_SR_IO13	0x34
#define OFF_SR_IO14	0x38
#define OFF_SR_IO15	0x3c
#define OFF_SR_IO16	0x40

#define SR_OFFSETS	\
	OFF_SR_IO0,	\
	OFF_SR_IO1,	\
	OFF_SR_IO2,	\
	OFF_SR_IO3,	\
	OFF_SR_IO4,	\
	OFF_SR_IO5,	\
	OFF_SR_IO6,	\
	OFF_SR_IO7,	\
	OFF_SR_IO8,	\
	OFF_SR_IO9,	\
	OFF_SR_IO10,	\
	OFF_SR_IO11,	\
	OFF_SR_IO12,	\
	OFF_SR_IO13,	\
	OFF_SR_IO14,	\
	OFF_SR_IO15,	\
	OFF_SR_IO16

/* ttl pins */
#define SSD210_PINNAME_TTL0		"ttl0"
#define SSD210_PINNAME_TTL1		"ttl1"
#define SSD210_PINNAME_TTL2		"ttl2"
#define SSD210_PINNAME_TTL3		"ttl3"
#define SSD210_PINNAME_TTL4		"ttl4"
#define SSD210_PINNAME_TTL5		"ttl5"
#define SSD210_PINNAME_TTL6		"ttl6"
#define SSD210_PINNAME_TTL7		"ttl7"
#define SSD210_PINNAME_TTL8		"ttl8"
#define SSD210_PINNAME_TTL9		"ttl9"
#define SSD210_PINNAME_TTL10		"ttl10"
#define SSD210_PINNAME_TTL11		"ttl11"
#define SSD210_PINNAME_TTL12		"ttl12"
#define SSD210_PINNAME_TTL13		"ttl13"
#define SSD210_PINNAME_TTL14		"ttl14"
#define SSD210_PINNAME_TTL15		"ttl15"
#define SSD210_PINNAME_TTL16		"ttl16"
#define SSD210_PINNAME_TTL17		"ttl17"
#define SSD210_PINNAME_TTL18		"ttl18"
#define SSD210_PINNAME_TTL19		"ttl19"
#define SSD210_PINNAME_TTL20		"ttl20"
#define SSD210_PINNAME_TTL21		"ttl21"

#define TTL_NAMES		\
	SSD210_PINNAME_TTL0,	\
	SSD210_PINNAME_TTL1,	\
	SSD210_PINNAME_TTL2,	\
	SSD210_PINNAME_TTL3,	\
	SSD210_PINNAME_TTL4,	\
	SSD210_PINNAME_TTL5,	\
	SSD210_PINNAME_TTL6,	\
	SSD210_PINNAME_TTL7,	\
	SSD210_PINNAME_TTL8,	\
	SSD210_PINNAME_TTL9,	\
	SSD210_PINNAME_TTL10,	\
	SSD210_PINNAME_TTL11,	\
	SSD210_PINNAME_TTL12,	\
	SSD210_PINNAME_TTL13,	\
	SSD210_PINNAME_TTL14,	\
	SSD210_PINNAME_TTL15,	\
	SSD210_PINNAME_TTL16,	\
	SSD210_PINNAME_TTL17,	\
	SSD210_PINNAME_TTL18,	\
	SSD210_PINNAME_TTL19,	\
	SSD210_PINNAME_TTL20,	\
	SSD210_PINNAME_TTL21

#define OFF_TTL0	0x44
#define OFF_TTL1	0x48
#define OFF_TTL2	0x4c
#define OFF_TTL3	0x50
#define OFF_TTL4	0x54
#define OFF_TTL5	0x58
#define OFF_TTL6	0x5c
#define OFF_TTL7	0x60
#define OFF_TTL8	0x64
#define OFF_TTL9	0x68
#define OFF_TTL10	0x6c
#define OFF_TTL11	0x70
#define OFF_TTL12	0x74
#define OFF_TTL13	0x78
#define OFF_TTL14	0x7c
#define OFF_TTL15	0x80
#define OFF_TTL16	0x84
#define OFF_TTL17	0x88
#define OFF_TTL18	0x8c
#define OFF_TTL19	0x90
#define OFF_TTL20	0x94
#define OFF_TTL21	0x98

#define TTL_OFFSETS	\
	OFF_TTL0,	\
	OFF_TTL1,	\
	OFF_TTL2,	\
	OFF_TTL3,	\
	OFF_TTL4,	\
	OFF_TTL5,	\
	OFF_TTL6,	\
	OFF_TTL7,	\
	OFF_TTL8,	\
	OFF_TTL9,	\
	OFF_TTL10,	\
	OFF_TTL11,	\
	OFF_TTL12,	\
	OFF_TTL13,	\
	OFF_TTL14,	\
	OFF_TTL15,	\
	OFF_TTL16,	\
	OFF_TTL17,	\
	OFF_TTL18,	\
	OFF_TTL19,	\
	OFF_TTL20,	\
	OFF_TTL21

/* key pins */
#define SSD210_PINNAME_KEY0		"key0"
#define SSD210_PINNAME_KEY1		"key1"
#define SSD210_PINNAME_KEY2		"key2"
#define SSD210_PINNAME_KEY3		"key3"
#define SSD210_PINNAME_KEY4		"key4"
#define SSD210_PINNAME_KEY5		"key5"
#define SSD210_PINNAME_KEY6		"key6"
#define SSD210_PINNAME_KEY7		"key7"
#define SSD210_PINNAME_KEY8		"key8"
#define SSD210_PINNAME_KEY9		"key9"
#define SSD210_PINNAME_KEY10		"key10"
#define SSD210_PINNAME_KEY11		"key11"
#define SSD210_PINNAME_KEY12		"key12"
#define SSD210_PINNAME_KEY13		"key13"

#define KEY_NAMES		\
	SSD210_PINNAME_KEY0,	\
	SSD210_PINNAME_KEY1,	\
	SSD210_PINNAME_KEY2,	\
	SSD210_PINNAME_KEY3,	\
	SSD210_PINNAME_KEY4,	\
	SSD210_PINNAME_KEY5,	\
	SSD210_PINNAME_KEY6,	\
	SSD210_PINNAME_KEY7,	\
	SSD210_PINNAME_KEY8,	\
	SSD210_PINNAME_KEY9,	\
	SSD210_PINNAME_KEY10,	\
	SSD210_PINNAME_KEY11,	\
	SSD210_PINNAME_KEY12,	\
	SSD210_PINNAME_KEY13

#define OFF_KEY0	0x9c
#define OFF_KEY1	0xa0
#define OFF_KEY2	0xa4
#define OFF_KEY3	0xa8
#define OFF_KEY4	0xac
#define OFF_KEY5	0xb0
#define OFF_KEY6	0xb4
#define OFF_KEY7	0xb8
#define OFF_KEY8	0xbc
#define OFF_KEY9	0xc0
#define OFF_KEY10	0xc4
#define OFF_KEY11	0xc8
#define OFF_KEY12	0xcc
#define OFF_KEY13	0xd0

#define KEY_OFFSETS	\
	OFF_KEY0,	\
	OFF_KEY1,	\
	OFF_KEY2,	\
	OFF_KEY3,	\
	OFF_KEY4,	\
	OFF_KEY5,	\
	OFF_KEY6,	\
	OFF_KEY7,	\
	OFF_KEY8,	\
	OFF_KEY9,	\
	OFF_KEY10,	\
	OFF_KEY11,	\
	OFF_KEY12,	\
	OFF_KEY13

/* sd pins */
#define SSD210_PINNAME_SD_D1		"sd_d1"
#define SSD210_PINNAME_SD_D0		"sd_d0"
#define SSD210_PINNAME_SD_CLK		"sd_clk"
#define SSD210_PINNAME_SD_CMD		"sd_cmd"
#define SSD210_PINNAME_SD_D3		"sd_d3"
#define SSD210_PINNAME_SD_D2		"sd_d2"
#define SSD210_PINNAME_SD_GPIO0		"sd_gpio0"
#define SSD210_PINNAME_SD_GPIO1		"sd_gpio1"

#define SD_NAMES 			\
	SSD210_PINNAME_SD_D1,		\
	SSD210_PINNAME_SD_D0,		\
	SSD210_PINNAME_SD_CLK,		\
	SSD210_PINNAME_SD_CMD,		\
	SSD210_PINNAME_SD_D3,		\
	SSD210_PINNAME_SD_D2,		\
	SSD210_PINNAME_SD_GPIO0,	\
	SSD210_PINNAME_SD_GPIO1

#define OFF_SD_D1	0xd4
#define OFF_SD_D0	0xd8
#define OFF_SD_CLK	0xdc
#define OFF_SD_CMD	0xe0
#define OFF_SD_D3	0xe4
#define OFF_SD_D2	0xe8
#define OFF_SD_GPIO0	0xec
#define OFF_SD_GPIO1	0xf0

#define SD_OFFSETS	\
	OFF_SD_D1,	\
	OFF_SD_D0,	\
	OFF_SD_CLK,	\
	OFF_SD_CMD,	\
	OFF_SD_D3,	\
	OFF_SD_D2,	\
	OFF_SD_GPIO0,	\
	OFF_SD_GPIO1

/* gpio, awesome naming.., pins */
#define SSD210_PINNAME_GPIO0		"gpio0"
#define SSD210_PINNAME_GPIO1		"gpio1"
#define SSD210_PINNAME_GPIO2		"gpio2"
#define SSD210_PINNAME_GPIO3		"gpio3"
#define SSD210_PINNAME_GPIO4		"gpio4"
#define SSD210_PINNAME_GPIO5		"gpio5"
#define SSD210_PINNAME_GPIO6		"gpio6"
#define SSD210_PINNAME_GPIO7		"gpio7"
#define SSD210_PINNAME_GPIO8		"gpio8"
#define SSD210_PINNAME_GPIO9		"gpio9"

#define GPIO_NAMES		\
	SSD210_PINNAME_GPIO0,	\
	SSD210_PINNAME_GPIO1,	\
	SSD210_PINNAME_GPIO2,	\
	SSD210_PINNAME_GPIO3,	\
	SSD210_PINNAME_GPIO4,	\
	SSD210_PINNAME_GPIO5,	\
	SSD210_PINNAME_GPIO6,	\
	SSD210_PINNAME_GPIO7,	\
	SSD210_PINNAME_GPIO8,	\
	SSD210_PINNAME_GPIO9

#define OFF_GPIO0	0xf4
#define OFF_GPIO1	0xf8
#define OFF_GPIO2	0xfc
#define OFF_GPIO3	0x100
#define OFF_GPIO4	0x104
#define OFF_GPIO5	0x108
#define OFF_GPIO6	0x10c
#define OFF_GPIO7	0x110
#define OFF_GPIO8	0x114
#define OFF_GPIO9	0x118

#define GPIO_OFFSETS	\
	OFF_GPIO0,	\
	OFF_GPIO1,	\
	OFF_GPIO2,	\
	OFF_GPIO3,	\
	OFF_GPIO4,	\
	OFF_GPIO5,	\
	OFF_GPIO6,	\
	OFF_GPIO7,	\
	OFF_GPIO8,	\
	OFF_GPIO9

struct ssd210_gpio_data {
	const char * const *names;
	const unsigned int *offsets;
	const unsigned int num;
};

#define SSD210_GPIO_CHIPDATA(_chip) \
static const struct ssd210_gpio_data _chip##_data = { \
	.names = _chip##_names, \
	.offsets = _chip##_offsets, \
	.num = ARRAY_SIZE(_chip##_offsets), \
}

static const char * const ssd210_names[] = {
	SR_NAMES,
	TTL_NAMES,
	KEY_NAMES,
	SD_NAMES,
	GPIO_NAMES,
};

static const unsigned int ssd210_offsets[] = {
	SR_OFFSETS,
	TTL_OFFSETS,
	KEY_OFFSETS,
	SD_OFFSETS,
	GPIO_OFFSETS,
};

SSD210_GPIO_CHIPDATA(ssd210);

struct ssd210_gpio {
	void __iomem *base;
	const struct ssd210_gpio_data *gpio_data;
	u8 *saved;
};

static void ssd210_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ssd210_gpio *gpio = gpiochip_get_data(chip);
	u8 gpioreg = readb_relaxed(gpio->base + gpio->gpio_data->offsets[offset]);

	if (value)
		gpioreg |= SSD210_GPIO_OUT;
	else
		gpioreg &= ~SSD210_GPIO_OUT;

	writeb_relaxed(gpioreg, gpio->base + gpio->gpio_data->offsets[offset]);
}

static int ssd210_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ssd210_gpio *gpio = gpiochip_get_data(chip);

	return readb_relaxed(gpio->base + gpio->gpio_data->offsets[offset]) & SSD210_GPIO_IN;
}

static int ssd210_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ssd210_gpio *gpio = gpiochip_get_data(chip);
	u8 gpioreg = readb_relaxed(gpio->base + gpio->gpio_data->offsets[offset]);

	gpioreg |= SSD210_GPIO_OEN;
	writeb_relaxed(gpioreg, gpio->base + gpio->gpio_data->offsets[offset]);

	return 0;
}

static int ssd210_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ssd210_gpio *gpio = gpiochip_get_data(chip);
	u8 gpioreg = readb_relaxed(gpio->base + gpio->gpio_data->offsets[offset]);

	gpioreg &= ~SSD210_GPIO_OEN;
	if (value)
		gpioreg |= SSD210_GPIO_OUT;
	else
		gpioreg &= ~SSD210_GPIO_OUT;
	writeb_relaxed(gpioreg, gpio->base + gpio->gpio_data->offsets[offset]);

	return 0;
}

/*
 * The interrupt handling happens in the parent interrupt controller,
 * we don't do anything here.
 */
static struct irq_chip ssd210_gpio_irqchip = {
	.name = "GPIO",
	.irq_eoi = irq_chip_eoi_parent,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
};

static int ssd210_gpio_child_to_parent_hwirq(struct gpio_chip *chip,
					     unsigned int child,
					     unsigned int child_type,
					     unsigned int *parent,
					     unsigned int *parent_type)
{
	struct ssd210_gpio *priv = gpiochip_get_data(chip);
	unsigned int offset = priv->gpio_data->offsets[child];

	return -EINVAL;
}

static int ssd210_gpio_probe(struct platform_device *pdev)
{
	const struct ssd210_gpio_data *match_data;
	struct ssd210_gpio *gpio;
	struct gpio_chip *gpiochip;
	struct gpio_irq_chip *gpioirqchip;
	struct irq_domain *parent_domain;
	struct device_node *parent_node;
	struct device *dev = &pdev->dev;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return -ENODEV;

	parent_domain = irq_find_host(parent_node);
	if (!parent_domain)
		return -ENODEV;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->gpio_data = match_data;

	gpio->saved = devm_kcalloc(dev, gpio->gpio_data->num, sizeof(*gpio->saved), GFP_KERNEL);
	if (!gpio->saved)
		return -ENOMEM;

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	platform_set_drvdata(pdev, gpio);

	gpiochip = devm_kzalloc(dev, sizeof(*gpiochip), GFP_KERNEL);
	if (!gpiochip)
		return -ENOMEM;

	gpiochip->label = DRIVER_NAME;
	gpiochip->parent = dev;
	gpiochip->request = gpiochip_generic_request;
	gpiochip->free = gpiochip_generic_free;
	gpiochip->direction_input = ssd210_gpio_direction_input;
	gpiochip->direction_output = ssd210_gpio_direction_output;
	gpiochip->get = ssd210_gpio_get;
	gpiochip->set = ssd210_gpio_set;
	gpiochip->base = -1;
	gpiochip->ngpio = gpio->gpio_data->num;
	gpiochip->names = gpio->gpio_data->names;

	gpioirqchip = &gpiochip->irq;
	gpioirqchip->chip = &ssd210_gpio_irqchip;
	gpioirqchip->fwnode = of_node_to_fwnode(dev->of_node);
	gpioirqchip->parent_domain = parent_domain;
	gpioirqchip->child_to_parent_hwirq = ssd210_gpio_child_to_parent_hwirq;
	gpioirqchip->populate_parent_alloc_arg = gpiochip_populate_parent_fwspec_twocell;
	gpioirqchip->handler = handle_bad_irq;
	gpioirqchip->default_type = IRQ_TYPE_NONE;

	return devm_gpiochip_add_data(dev, gpiochip, gpio);
}

static const struct of_device_id ssd210_gpio_of_match[] = {
	{
		.compatible = "sstar,ssd210-gpio",
		.data = &ssd210_data,
	},
	{ }
};

static struct platform_driver ssd210_gpio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ssd210_gpio_of_match,
	},
	.probe = ssd210_gpio_probe,
};
builtin_platform_driver(ssd210_gpio_driver);
