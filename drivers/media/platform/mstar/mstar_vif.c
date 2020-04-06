/*
 *  0x0
 *
 *  0x8001
 *  0x8021
 *
 *         15        |        3         |      2       |           1         |   0
 *  channel 0 enable |power down sensor | reset sensor | vif if status reset | ~rst
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>

#define DRIVER_NAME "mstar-vif"

static int mstar_vif_probe(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mstar_vif_of_match[] = {
	{
		.compatible	= "mstar,vif",
	},
	{ }
};

static struct platform_driver mstar_vif_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mstar_vif_of_match,
	},
	.probe = mstar_vif_probe,
};

module_platform_driver(mstar_vif_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("MStar VIF");
MODULE_LICENSE("GPL v2");

