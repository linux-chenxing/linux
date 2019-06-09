// SPDX-License-Identifier: GPL-2.0-only
#include <linux/ethtool.h>
#include <linux/export.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "msc313.h"

#define MSC313_PHY_ID	0xdeadbeef
#define MSC313E_PHY_ID	0xdeadb33f
#define MSC313_PHY_MASK 0xffffffff

struct msc313_phy_data;

struct msc313_phy_priv {
	struct regmap *pmsleep;
	struct regmap *phyana;
	const struct msc313_phy_data *data;
	struct msc313e_fields msc313e_fields;
	bool poweredup;
};

struct msc313_phy_data {
	void (*powerup)(struct msc313_phy_priv*);
	void (*powerdown)(struct msc313_phy_priv*);
};

static void msc313_powerdown(struct msc313_phy_priv *priv)
{
	printk("Doing phy power down\n");
	regmap_write(priv->phyana, REG_LDO, 0xffff);
};

static void msc313_powerup(struct msc313_phy_priv *priv){
	printk("Doing phy power up\n");
}

static const struct msc313_phy_data msc313_data = {
	.powerup = msc313_powerup,
	.powerdown = msc313_powerdown,
};

#define ANA_WRITEB(_phy, _reg, _val) \
	regmap_update_bits(_phy->phyana, _reg, 0xff, _val)

static void msc313e_powerup(struct msc313_phy_priv *priv){
	if(priv->poweredup)
		return;

	printk("Doing phy power up\n");

	regmap_field_write(priv->msc313e_fields.anarst, 1);
	mdelay(100);
	regmap_field_write(priv->msc313e_fields.anarst, 0);
	mdelay(100);

	/* "Power-on LDO" */
	regmap_write(priv->phyana, REG_LDO, 0x0000);
	/* "Power-on SADC" */
	regmap_field_write(priv->msc313e_fields.sadcpd, 0);
        /* "Power-on ADCPL" */
	regmap_field_write(priv->msc313e_fields.adcplpd, 0);
        /* "Power-on REF" */
	regmap_field_write(priv->msc313e_fields.refpd, 0);
        /* "Power-on TX" */
	regmap_field_write(priv->msc313e_fields.txpd1, 0);
        /* "Power-on TX" */
	regmap_field_write(priv->msc313e_fields.txpd2, 0);
        /* "CLKO_ADC_SEL" */
	regmap_field_write(priv->msc313e_fields.clkoadcsel, 1);
	/* "reg_adc_clk_select" */
	regmap_field_write(priv->msc313e_fields.adcclksel, 1);
	/* "100gat" */
	regmap_field_write(priv->msc313e_fields.hundredgat, 0);
	/* "200gat" */
	regmap_field_write(priv->msc313e_fields.twohundredgat, 0);

	priv->poweredup = true;
}

static void msc313e_powerdown(struct msc313_phy_priv *priv)
{
	if(!priv->poweredup)
		return;

	printk("Doing phy power down\n");
	regmap_field_write(priv->msc313e_fields.anarst, 1);
	/* "Power-on LDO" */
	regmap_write(priv->phyana, REG_LDO, 0x0102);
	/* "Power-on SADC" */
	regmap_field_write(priv->msc313e_fields.sadcpd, ~0);
        /* "Power-on ADCPL" */
	regmap_field_write(priv->msc313e_fields.adcplpd, ~0);
        /* "Power-on REF" */
	regmap_field_write(priv->msc313e_fields.refpd, ~0);
        /* "Power-on TX" */
	regmap_field_write(priv->msc313e_fields.txpd1, ~0);
        /* "Power-on TX" */
	regmap_field_write(priv->msc313e_fields.txpd2, ~0);

	priv->poweredup = false;
};

static const struct msc313_phy_data msc313e_data = {
	.powerup = msc313e_powerup,
	.powerdown = msc313e_powerdown,
};

static int msc313_phy_suspend(struct phy_device *phydev)
{
	struct msc313_phy_priv *priv = phydev->priv;
	priv->data->powerdown(priv);
	return 0;
}

static int msc313_phy_resume(struct phy_device *phydev)
{
	struct msc313_phy_priv *priv = phydev->priv;
	priv->data->powerup(priv);
	return 0;
}

static int msc313_phy_probe(struct phy_device *phydev)
{
	struct device_node *of_node = phydev->mdio.dev.of_node;
	struct msc313_phy_priv *priv;
	int ret = 0;

	printk("phy probe\n");

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(*priv), GFP_KERNEL);
	if(IS_ERR(priv)){
		ret = PTR_ERR(priv);
		goto out;
	}

	priv->pmsleep = syscon_regmap_lookup_by_phandle(of_node, "mstar,pmsleep");
	if(IS_ERR(priv->pmsleep)){
		ret = PTR_ERR(priv->pmsleep);
		goto out;
	}

	priv->phyana = syscon_regmap_lookup_by_phandle(of_node, "mstar,phyana");
	if(IS_ERR(priv->phyana)){
		ret = PTR_ERR(priv->phyana);
		goto out;
	}

	priv->msc313e_fields.anarst = regmap_field_alloc(priv->phyana, anarst);
	priv->msc313e_fields.sadcpd = regmap_field_alloc(priv->phyana, sadcpd);
	priv->msc313e_fields.adcplpd = regmap_field_alloc(priv->phyana, adcplpd);
	priv->msc313e_fields.refpd = regmap_field_alloc(priv->phyana, refpd);
	priv->msc313e_fields.txpd1 = regmap_field_alloc(priv->phyana, txpd1);
	priv->msc313e_fields.txpd2 = regmap_field_alloc(priv->phyana, txpd2);
	priv->msc313e_fields.clkoadcsel = regmap_field_alloc(priv->phyana, clkoadcsel);
	priv->msc313e_fields.adcclksel = regmap_field_alloc(priv->phyana, adcclksel);
	priv->msc313e_fields.hundredgat = regmap_field_alloc(priv->phyana, hundredgat);
	priv->msc313e_fields.twohundredgat = regmap_field_alloc(priv->phyana, twohundredgat);

	priv->data = phydev->drv->driver_data;

	phydev->priv = priv;
out:
	return ret;
}

static struct phy_driver msc313_driver[] = {
	{
		.phy_id         = MSC313_PHY_ID,
		.phy_id_mask    = 0xffffffff,
		.name           = "msc313 phy",
		.probe		= msc313_phy_probe,
		.suspend	= msc313_phy_suspend,
		.resume		= msc313_phy_resume,
		.driver_data	= &msc313_data,
	},
	{
		.phy_id         = MSC313E_PHY_ID,
		.phy_id_mask    = 0xffffffff,
		.name           = "msc313e phy",
		.probe		= msc313_phy_probe,
		.suspend	= msc313_phy_suspend,
		.resume		= msc313_phy_resume,
		.driver_data	= &msc313e_data,
	},
};

module_phy_driver(msc313_driver);

static struct mdio_device_id __maybe_unused msc313_tbl[] = {
	{ MSC313_PHY_ID, MSC313_PHY_MASK },
	{ MSC313E_PHY_ID, MSC313_PHY_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, msc313_tbl);
