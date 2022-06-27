/*
 * pinctrl-ssd210.c
 *
 *  Created on: 27 Jun 2022
 *      Author: daniel
 */

#include <linux/of.h>
#include <linux/of_device.h>

static const struct of_device_id ssd210_pinctrl_of_match[] = {
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible	= "sstar,ssd210-pinctrl",
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, ssd210_pinctrl_of_match);
