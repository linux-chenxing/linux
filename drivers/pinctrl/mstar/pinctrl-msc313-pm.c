// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <soc/mstar/pmsleep.h>
#include <dt-bindings/pinctrl/mstar.h>

#include "../core.h"
#include "../devicetree.h"
#include "../pinconf.h"
#include "../pinmux.h"

#include "pinctrl-mstar.h"

#define DRIVER_NAME "pinctrl-msc313-pm"

#define GROUPNAME_PM_UART	"pm_uart"
#define GROUPNAME_PM_SPI	"pm_spi"
#define GROUPNAME_PM_LED_MODE1	"pm_led_mode1"
#define GROUPNAME_PM_IRIN	"pm_irin"
#define GROUPNAME_PM_SD_CDZ	"pm_sd_cdz"

#define FUNCTIONNAME_PM_UART	GROUPNAME_PM_UART
#define FUNCTIONNAME_PM_SPI	GROUPNAME_PM_SPI
#define FUNCTIONNAME_PM_LED	"pm_led"
#define FUNCTIONNAME_PM_IRIN	GROUPNAME_PM_IRIN

static const char * const pm_uart_groups[] = {
	GROUPNAME_PM_UART,
};

static const char * const pm_spi_groups[] = {
	GROUPNAME_PM_SPI,
};

static const char * const pm_irin_groups[] = {
	GROUPNAME_PM_IRIN,
};

/* Functions that all chips have */
#define COMMON_FUNCTIONS \
	COMMON_FIXED_FUNCTION(PM_UART, pm_uart), \
	COMMON_FIXED_FUNCTION(PM_SPI, pm_spi), \
	COMMON_FIXED_FUNCTION(PM_IRIN, pm_irin)

#ifdef CONFIG_MACH_INFINITY
#define MSC313_COMMON_PIN(_pinname) COMMON_PIN(MSC313, _pinname)
/* msc313/msc313e */
/* pinctrl pins */
static struct pinctrl_pin_desc msc313_pins[] = {
	MSC313_COMMON_PIN(PM_UART_RX),
	MSC313_COMMON_PIN(PM_UART_TX),
	MSC313_COMMON_PIN(PM_SPI_CZ),
	MSC313_COMMON_PIN(PM_SPI_DI),
	MSC313_COMMON_PIN(PM_SPI_WPZ),
	MSC313_COMMON_PIN(PM_SPI_DO),
	MSC313_COMMON_PIN(PM_SPI_CK),
	MSC313_COMMON_PIN(PM_IRIN),
	MSC313_COMMON_PIN(PM_SD_CDZ),
	MSC313_COMMON_PIN(PM_GPIO4),
};

/* mux pin groupings */
static const int msc313_pm_uart_pins[] = {
	PIN_MSC313_PM_UART_RX,
	PIN_MSC313_PM_UART_TX,
};

static const int msc313_pm_spi_pins[] = {
	PIN_MSC313_PM_SPI_CZ,
	PIN_MSC313_PM_SPI_DI,
	PIN_MSC313_PM_SPI_WPZ,
	PIN_MSC313_PM_SPI_DO,
	PIN_MSC313_PM_SPI_CK,
};

static const int msc313_pm_irin_pins[] = {
	PIN_MSC313_PM_IRIN,
};

static const struct msc313_pinctrl_group msc313_pinctrl_groups[] = {
	MSC313_PINCTRL_GROUP(PM_UART, pm_uart),
	MSC313_PINCTRL_GROUP(PM_SPI, pm_spi),
	MSC313_PINCTRL_GROUP(PM_IRIN, pm_irin),
};

static const struct msc313_pinctrl_function msc313_pinctrl_functions[] = {
	COMMON_FUNCTIONS,
};

static const struct msc313_pinctrl_pinconf msc313_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(msc313);
/* ssd20xd */
/* pinctrl pins */
static const struct pinctrl_pin_desc ssd20xd_pins[] = {
	SSD20XD_COMMON_PIN(PM_UART_RX),
	SSD20XD_COMMON_PIN(PM_UART_TX),
	SSD20XD_COMMON_PIN(PM_SPI_CZ),
	SSD20XD_COMMON_PIN(PM_SPI_CK),
	SSD20XD_COMMON_PIN(PM_SPI_DI),
	SSD20XD_COMMON_PIN(PM_SPI_DO),
	SSD20XD_COMMON_PIN(PM_SPI_HLD),
	SSD20XD_COMMON_PIN(PM_SPI_WPZ),
	SSD20XD_COMMON_PIN(PM_LED0),
	SSD20XD_COMMON_PIN(PM_LED1),
	SSD20XD_COMMON_PIN(PM_IRIN),
	SSD20XD_COMMON_PIN(PM_SD_CDZ),
};

/* mux pin groupings */
static const int ssd20xd_pm_uart_pins[] = {
	PIN_SSD20XD_PM_UART_RX,
	PIN_SSD20XD_PM_UART_TX,
};

static const int ssd20xd_pm_spi_pins[] = {
	PIN_SSD20XD_PM_SPI_CZ,
	PIN_SSD20XD_PM_SPI_DI,
	PIN_SSD20XD_PM_SPI_WPZ,
	PIN_SSD20XD_PM_SPI_DO,
	PIN_SSD20XD_PM_SPI_CK,
	PIN_SSD20XD_PM_SPI_HLD,
};

static const int ssd20xd_pm_led_mode1_pins[] = {
	PIN_SSD20XD_PM_LED0,
	PIN_SSD20XD_PM_LED1,
};

static const int ssd20xd_pm_irin_pins[] = {
	PIN_SSD20XD_PM_IRIN,
};

static const int ssd20xd_pm_sd_cdz_pins[] = {
	PIN_SSD20XD_PM_SD_CDZ
};

static const struct msc313_pinctrl_group ssd20xd_pinctrl_groups[] = {
	SSD20XD_PINCTRL_GROUP(PM_UART, pm_uart),
	SSD20XD_PINCTRL_GROUP(PM_SPI, pm_spi),
	SSD20XD_PINCTRL_GROUP(PM_LED_MODE1, pm_led_mode1),
	SSD20XD_PINCTRL_GROUP(PM_IRIN, pm_irin),
	SSD20XD_PINCTRL_GROUP(PM_SD_CDZ, pm_sd_cdz),
};

/* chip specific functions */
#define REG_SSD20XD_PM_LED MSTAR_PMSLEEP_PMLED
#define SHIFT_SSD20XD_PM_LED 4
#define WIDTH_SSD20XD_PM_LED 2
#define MASK_SSD20XD_PM_LED MAKEMASK(SSD20XD_PM_LED)

static const char * const ssd20xd_pm_led_groups[] = {
	GROUPNAME_PM_LED_MODE1,
};

static const u16 ssd20xd_pm_led_values[] = {
	SSD20XD_MODE(PM_LED, 1),
};

static const struct msc313_pinctrl_function ssd20xd_pinctrl_functions[] = {
	COMMON_FUNCTIONS,
	SSD20XD_FUNCTION(PM_LED, pm_led),
};

static const struct msc313_pinctrl_pinconf ssd20xd_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd20xd);

static const struct pinctrl_pin_desc ssd203d_pins[] = {
	SSD203D_COMMON_PIN(PM_UART_RX),
	SSD203D_COMMON_PIN(PM_UART_TX),
	SSD203D_COMMON_PIN(PM_SPI_CZ),
	SSD203D_COMMON_PIN(PM_SPI_CK),
	SSD203D_COMMON_PIN(PM_SPI_DI),
	SSD203D_COMMON_PIN(PM_SPI_DO),
	SSD203D_COMMON_PIN(PM_SPI_HLD),
	SSD203D_COMMON_PIN(PM_SPI_WPZ),
	SSD203D_COMMON_PIN(PM_IRIN),
};

static const int ssd203d_pm_uart_pins[] = {
	PIN_SSD203D_PM_UART_RX,
	PIN_SSD203D_PM_UART_TX,
};

static const int ssd203d_pm_spi_pins[] = {
	PIN_SSD203D_PM_SPI_CZ,
	PIN_SSD203D_PM_SPI_DI,
	PIN_SSD203D_PM_SPI_WPZ,
	PIN_SSD203D_PM_SPI_DO,
	PIN_SSD203D_PM_SPI_CK,
	PIN_SSD203D_PM_SPI_HLD,
};

static const int ssd203d_pm_irin_pins[] = {
	PIN_SSD203D_PM_IRIN,
};

static const struct msc313_pinctrl_group ssd203d_pinctrl_groups[] = {
	SSD203D_PINCTRL_GROUP(PM_UART, pm_uart),
	SSD203D_PINCTRL_GROUP(PM_SPI, pm_spi),
	SSD203D_PINCTRL_GROUP(PM_IRIN, pm_irin),
};

static const struct msc313_pinctrl_function ssd203d_pinctrl_functions[] = {
	COMMON_FUNCTIONS
};

static const struct msc313_pinctrl_pinconf ssd203d_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd203d);
#endif /* infinity */

#ifdef CONFIG_MACH_MERCURY
/* ssc8336 */
#define SSC8336N_COMMON_PIN(_pinname) COMMON_PIN(SSC8336N, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssc8336n_pins[] = {
	SSC8336N_COMMON_PIN(PM_UART_TX),
	SSC8336N_COMMON_PIN(PM_UART_RX),
	SSC8336N_COMMON_PIN(PM_SPI_CZ),
	SSC8336N_COMMON_PIN(PM_SPI_DI),
	SSC8336N_COMMON_PIN(PM_SPI_WPZ),
	SSC8336N_COMMON_PIN(PM_SPI_DO),
	SSC8336N_COMMON_PIN(PM_SPI_CK),
	SSC8336N_COMMON_PIN(PM_SPI_HOLD),
	SSC8336N_COMMON_PIN(PM_IRIN),
	SSC8336N_COMMON_PIN(PM_GPIO8),
	SSC8336N_COMMON_PIN(PM_GPIO6),
	SSC8336N_COMMON_PIN(PM_GPIO5),
	SSC8336N_COMMON_PIN(PM_GPIO4),
	SSC8336N_COMMON_PIN(PM_GPIO2),
	SSC8336N_COMMON_PIN(PM_GPIO0),
	SSC8336N_COMMON_PIN(PM_SD_CDZ),
};

/* mux pin groupings */
static const int ssc8336n_pm_uart_pins[] = {
	PIN_SSC8336N_PM_UART_TX,
	PIN_SSC8336N_PM_UART_RX,
};

static const int ssc8336n_pm_spi_pins[]  = {
	PIN_SSC8336N_PM_SPI_CZ,
	PIN_SSC8336N_PM_SPI_CZ,
	PIN_SSC8336N_PM_SPI_DI,
	PIN_SSC8336N_PM_SPI_WPZ,
	PIN_SSC8336N_PM_SPI_DO,
	PIN_SSC8336N_PM_SPI_CK,
	PIN_SSC8336N_PM_SPI_HOLD,
};

static const int ssc8336n_pm_irin_pins[] = {
	PIN_SSC8336N_PM_IRIN,
};

#define SSC8336N_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssc8336n_##_name##_pins)

/* pinctrl groups */
static const struct msc313_pinctrl_group ssc8336n_pinctrl_groups[] = {
	SSC8336N_PINCTRL_GROUP(PM_UART, pm_uart),
	SSC8336N_PINCTRL_GROUP(PM_SPI, pm_spi),
	SSC8336N_PINCTRL_GROUP(PM_IRIN, pm_irin),
};

#define SSC8336N_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSC8336N_##_NAME, \
	MASK_SSC8336N_##_NAME, ssc8336n_##_name##_groups, ssc8336n_##_name##_values)

static const struct msc313_pinctrl_function ssc8336n_pinctrl_functions[] = {
	COMMON_FUNCTIONS,
};

static const struct msc313_pinctrl_pinconf ssc8336n_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssc8336n);
#endif /* mercury5 */

#ifdef CONFIG_MACH_PIONEER3
/* ssd210 */
#define SSD210_COMMON_PIN(_pinname) COMMON_PIN(SSD210, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssd210_pins[] = {
	SSD210_COMMON_PIN(PM_UART_TX),
	SSD210_COMMON_PIN(PM_UART_RX),
	SSD210_COMMON_PIN(PM_SPI_CZ),
	SSD210_COMMON_PIN(PM_SPI_DI),
	SSD210_COMMON_PIN(PM_SPI_WPZ),
	SSD210_COMMON_PIN(PM_SPI_DO),
	SSD210_COMMON_PIN(PM_SPI_CK),
	SSD210_COMMON_PIN(PM_SPI_HOLD),
};

/* mux pin groupings */
static const int ssd210_pm_uart_pins[] = {
	PIN_SSD210_PM_UART_TX,
	PIN_SSD210_PM_UART_RX,
};

static const int ssd210_pm_spi_pins[]  = {
	PIN_SSD210_PM_SPI_CZ,
	PIN_SSD210_PM_SPI_CZ,
	PIN_SSD210_PM_SPI_DI,
	PIN_SSD210_PM_SPI_WPZ,
	PIN_SSD210_PM_SPI_DO,
	PIN_SSD210_PM_SPI_CK,
	PIN_SSD210_PM_SPI_HOLD,
};

#define SSD210_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd210_##_name##_pins)

/* pinctrl groups */
static const struct msc313_pinctrl_group ssd210_pinctrl_groups[] = {
	SSD210_PINCTRL_GROUP(PM_UART, pm_uart),
	SSD210_PINCTRL_GROUP(PM_SPI, pm_spi),
};

#define SSD210_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSC8336N_##_NAME, \
	MASK_SSC8336N_##_NAME, ssc8336n_##_name##_groups, ssc8336n_##_name##_values)

static const struct msc313_pinctrl_function ssd210_pinctrl_functions[] = {
	COMMON_FIXED_FUNCTION(PM_UART, pm_uart), \
	COMMON_FIXED_FUNCTION(PM_SPI, pm_spi), \
};

static const struct msc313_pinctrl_pinconf ssd210_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd210);

/* ssd212 */
#define SSD212_COMMON_PIN(_pinname) COMMON_PIN(SSD212, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssd212_pins[] = {
	SSD212_COMMON_PIN(PM_UART_TX),
	SSD212_COMMON_PIN(PM_UART_RX),
	SSD212_COMMON_PIN(PM_SPI_CZ),
	SSD212_COMMON_PIN(PM_SPI_DI),
	SSD212_COMMON_PIN(PM_SPI_WPZ),
	SSD212_COMMON_PIN(PM_SPI_DO),
	SSD212_COMMON_PIN(PM_SPI_CK),
	SSD212_COMMON_PIN(PM_SPI_HOLD),
	SSD212_COMMON_PIN(PM_SD_CDZ),
};

/* mux pin groupings */
static const int ssd212_pm_uart_pins[] = {
	PIN_SSD212_PM_UART_TX,
	PIN_SSD212_PM_UART_RX,
};

static const int ssd212_pm_spi_pins[]  = {
	PIN_SSD212_PM_SPI_CZ,
	PIN_SSD212_PM_SPI_CZ,
	PIN_SSD212_PM_SPI_DI,
	PIN_SSD212_PM_SPI_WPZ,
	PIN_SSD212_PM_SPI_DO,
	PIN_SSD212_PM_SPI_CK,
	PIN_SSD212_PM_SPI_HOLD,
};

#define SSD212_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd212_##_name##_pins)

/* pinctrl groups */
static const struct msc313_pinctrl_group ssd212_pinctrl_groups[] = {
	SSD212_PINCTRL_GROUP(PM_UART, pm_uart),
	SSD212_PINCTRL_GROUP(PM_SPI, pm_spi),
};

#define SSD212_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSC8336N_##_NAME, \
	MASK_SSC8336N_##_NAME, ssc8336n_##_name##_groups, ssc8336n_##_name##_values)

static const struct msc313_pinctrl_function ssd212_pinctrl_functions[] = {
	COMMON_FIXED_FUNCTION(PM_UART, pm_uart), \
	COMMON_FIXED_FUNCTION(PM_SPI, pm_spi), \
};

static const struct msc313_pinctrl_pinconf ssd212_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd212);
#endif /* pioneer3 */

static int msc313_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_pinctrl *pinctrl;
	const struct msc313_pinctrl_info *match_data;
	int ret;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	pinctrl->regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(pinctrl->regmap))
		return PTR_ERR(pinctrl->regmap);

	pinctrl->desc.name = DRIVER_NAME;
	pinctrl->desc.pctlops = &msc313_pinctrl_ops;
	pinctrl->desc.pmxops = &mstar_pinmux_ops;
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

static const struct of_device_id msc313_pinctrl_pm_of_match[] = {
#ifdef CONFIG_MACH_INFINITY
	{
		.compatible	= "mstar,msc313-pm-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "mstar,msc313e-pm-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "sstar,ssd20xd-pm-pinctrl",
		.data		= &ssd20xd_info,
	},
	{
		.compatible	= "sstar,ssd203d-pm-pinctrl",
		.data		= &ssd203d_info,
	},
#endif
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible     = "sstar,ssd210-pm-pinctrl",
		.data           = &ssd210_info,
	},
	{
		.compatible	= "sstar,ssd212-pm-pinctrl",
		.data		= &ssd212_info,
	},
#endif
#ifdef CONFIG_MACH_MERCURY
	{
		.compatible	= "mstar,ssc8336n-pm-pinctrl",
		.data		= &ssc8336n_info,
	},
	{
		.compatible	= "mstar,ssc8336-pm-pinctrl",
		.data		= &ssc8336n_info,
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, msc313_pinctrl_pm_of_match);

static struct platform_driver msc313_pinctrl_pm_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_pinctrl_pm_of_match,
	},
	.probe = msc313_pinctrl_probe,
};
module_platform_driver(msc313_pinctrl_pm_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("PM Pin controller driver for MStar SoCs");
MODULE_LICENSE("GPL v2");
