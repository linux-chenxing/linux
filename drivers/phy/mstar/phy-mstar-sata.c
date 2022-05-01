// SPDX-License-Identifier: GPL-2.0-or-later
/*
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/platform_device.h>

struct priv {
	struct clk	*clk;
	void __iomem	*base;
};

static int phy_mstar_sata_power_on(struct phy *phy)
{
	struct priv *priv = phy_get_drvdata(phy);

	return 0;
}

static int phy_mstar_sata_power_off(struct phy *phy)
{
	struct priv *priv = phy_get_drvdata(phy);

	return 0;
}

static const struct phy_ops phy_mstar_sata_ops = {
	.power_on	= phy_mstar_sata_power_on,
	.power_off	= phy_mstar_sata_power_off,
	.owner		= THIS_MODULE,
};

static int phy_mstar_sata_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct priv *priv;
	struct phy *phy;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	//priv->clk = devm_clk_get(&pdev->dev, "sata");
	//if (IS_ERR(priv->clk))
	//	return PTR_ERR(priv->clk);

	phy = devm_phy_create(&pdev->dev, NULL, &phy_mstar_sata_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	/* The boot loader may of left it on. Turn it off. */
	phy_mstar_sata_power_off(phy);

	return 0;
}

static const struct of_device_id phy_mstar_sata_of_match[] = {
	{ .compatible = "sstar,ssd203d-sata-phy" },
	{ },
};

static struct platform_driver phy_mstar_sata_driver = {
	.probe	= phy_mstar_sata_probe,
	.driver = {
		.name	= "phy-mstar-sata",
		.of_match_table	= phy_mstar_sata_of_match,
	}
};
builtin_platform_driver(phy_mstar_sata_driver);
