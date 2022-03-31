// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>

/*
 * MStar MSC313 MCM
 *
 * This seems to be a bus arbiter
 *
 *      7 - 4      |
 * slow down ratio |
 */

struct msc313_mcm {
	void __iomem *base;
};

static const struct of_device_id msc313_mcm_of_match[] = {
	{
		.compatible = "mstar,msc313-mcm",
	},
	{}
};
MODULE_DEVICE_TABLE(of, msc313_mcm_of_match);

static int msc313_mcm_probe(struct platform_device *pdev)
{
	struct msc313_mcm* mcm;
	struct resource *mem;

	mcm = devm_kzalloc(&pdev->dev, sizeof(*mcm), GFP_KERNEL);
	if(!mcm)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mcm->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(mcm->base))
		return PTR_ERR(mcm->base);

	return 0;
}

static int msc313_mcm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver msc313_mcm_driver = {
	.driver = {
		.name = "msc313-mcm",
		.of_match_table = msc313_mcm_of_match,
	},
	.probe = msc313_mcm_probe,
	.remove = msc313_mcm_remove,
};
module_platform_driver(msc313_mcm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("MStar MSC313 mcm driver");
