// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

struct msc313_did_priv {
	void __iomem *regs;
};

static int msc313_did_reg_read(void *context, unsigned int reg, void *val, size_t bytes)
{
	struct msc313_did_priv *priv = context;

	return 0;
}

static const struct of_device_id msc313_did_of_table[] = {
	{ .compatible = "mstar,msc313-did", },
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, msc313_did_of_table);

static int msc313_did_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct nvmem_device *nvmem;
	struct msc313_did_priv *priv;

	struct nvmem_config config = {
		.name = "msc313-did",
		.size = 0x80,
		.stride = 2,
		.word_size = 2,
		.reg_read = msc313_did_reg_read,
		.read_only = true,
		.root_only = true,
	};

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	config.dev = dev;
	config.priv = priv;

	nvmem = devm_nvmem_register(dev, &config);

	return PTR_ERR_OR_ZERO(nvmem);
}

static struct platform_driver msc313_did_driver = {
	.probe = msc313_did_probe,
	.driver = {
		.name = "msc313-did",
		.of_match_table = msc313_did_of_table,
	},
};
module_platform_driver(msc313_did_driver);
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("MStar/SigmaStar DID driver");
MODULE_LICENSE("GPL v2");
