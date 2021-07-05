/*
 *  0x0
 *    6    |   0
 *  pd_ldo | sw_rst
 *
 *  0x4
 *               1               |          0
 *  power down whole dphy analog | power down hs mode
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#define DRIVER_NAME "mstar-mipi_dphy"

struct mstar_dphy {
	struct phy *phy;
};

static int mstar_dphy_init(struct phy *phy)
{
	printk("%s\n", __func__);
	return 0;
}

static int mstar_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	printk("%s\n", __func__);
	return 0;
}

static int mstar_dphy_power_on(struct phy *phy)
{
	printk("%s\n", __func__);
	return 0;
}

static int mstar_dphy_power_off(struct phy *phy)
{
	printk("%s\n", __func__);
	return 0;
}

static int mstar_dphy_exit(struct phy *phy)
{
	printk("%s\n", __func__);
	return 0;
}

static const struct phy_ops mstar_dphy_ops = {
	.configure	= mstar_dphy_configure,
	.power_on	= mstar_dphy_power_on,
	.power_off	= mstar_dphy_power_off,
	.init		= mstar_dphy_init,
	.exit		= mstar_dphy_exit,
};

static int mstar_dphy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct mstar_dphy *dphy;

	printk("dphy probe\n");

	dphy = devm_kzalloc(&pdev->dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	dphy->phy = devm_phy_create(&pdev->dev, NULL, &mstar_dphy_ops);
	if (IS_ERR(dphy->phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(dphy->phy);
	}

	phy_set_drvdata(dphy->phy, dphy);
	phy_provider = devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id mstar_dphy_of_match[] = {
	{
		.compatible	= "sstar,ssd20xd-dphy",
	},
	{ }
};

static struct platform_driver mstar_dphy_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mstar_dphy_of_match,
	},
	.probe = mstar_dphy_probe,
};
module_platform_driver(mstar_dphy_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("MStar MIPI DPHY");
MODULE_LICENSE("GPL v2");
