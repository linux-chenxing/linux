// SPDX-License-Identifier: GPL-2.0+
/*
 * MStar "glue layer" based on jz4740.c
 *
 * Copyright (C) 2013, Apelete Seketeli <apelete@seketeli.net>
 * Copyright (C) 2019, Daniel Palmer <daniel@0x0f.com>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/usb/mstar_usbc.h>

#include "musb_core.h"

struct mstar_glue {
	struct device           *dev;
	struct platform_device  *musb;
	struct clk		*clk;
	struct regmap *usbc;
};

static irqreturn_t mstar_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	/*
	 * The controller is gadget only, the state of the host mode IRQ bits is
	 * undefined. Mask them to make sure that the musb driver core will
	 * never see them set
	 */
	musb->int_usb &= MUSB_INTR_SUSPEND | MUSB_INTR_RESUME |
	    MUSB_INTR_RESET | MUSB_INTR_SOF;

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static struct musb_fifo_cfg mstar_musb_fifo_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX, .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX, .maxpacket = 64, },
};

static const struct musb_hdrc_config mstar_musb_config = {
	/* Silicon does not implement USB OTG. */
	.multipoint = 0,
	/* Max EPs scanned, driver will decide which EP can be used. */
	.num_eps    = 4,
	/* RAMbits needed to configure EPs from table */
	.ram_bits   = 9,
	.fifo_cfg = mstar_musb_fifo_cfg,
	.fifo_cfg_size = ARRAY_SIZE(mstar_musb_fifo_cfg),
};

static struct musb_hdrc_platform_data mstar_musb_platform_data = {
	.mode   = MUSB_PERIPHERAL,
	.config = &mstar_musb_config,
};

static int mstar_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller->parent;

	if (dev->of_node)
		musb->xceiv = devm_usb_get_phy_by_phandle(dev, "phys", 0);
	else
		musb->xceiv = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);

	if (IS_ERR(musb->xceiv)) {
		dev_err(dev, "No transceiver configured\n");
		return PTR_ERR(musb->xceiv);
	}

	/* Silicon does not implement ConfigData register.
	 * Set dyn_fifo to avoid reading EP config from hardware.
	 */
	musb->dyn_fifo = true;

	musb->isr = mstar_musb_interrupt;

	return 0;
}

/*
 * DMA has not been confirmed to work with CONFIG_USB_INVENTRA_DMA,
 * so let's not set up the dma function pointers yet.
 */
static const struct musb_platform_ops mstar_musb_ops = {
	.quirks		= MUSB_DMA_INVENTRA | MUSB_INDEXED_EP,
	.fifo_mode	= 2,
	.init		= mstar_musb_init,
};

static int mstar_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = &mstar_musb_platform_data;
	struct platform_device		*musb;
	struct mstar_glue		*glue;
	struct clk                      *clk;
	int				ret;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->usbc = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "mstar,usbc");
	if (IS_ERR(glue->usbc))
		return PTR_ERR(glue->usbc);

	dev_info(&pdev->dev, "Enabling OTG registers..\n");
	regmap_update_bits(glue->usbc, MSTAR_USBC_REG_RSTCTRL,
			MSTAR_RSTCTRL_REG_SUSPEND | MSTAR_RSTCTRL_OTG_XIU,
			MSTAR_RSTCTRL_REG_SUSPEND | MSTAR_RSTCTRL_OTG_XIU);

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		return -ENOMEM;
	}

	clk = devm_clk_get(&pdev->dev, "udc");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err_platform_device_put;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err_platform_device_put;
	}

	musb->dev.parent		= &pdev->dev;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->clk			= clk;

	pdata->platform_ops		= &mstar_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
					    pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err_clk_disable;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err_clk_disable;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err_clk_disable;
	}

	return 0;

err_clk_disable:
	clk_disable_unprepare(clk);
err_platform_device_put:
	platform_device_put(musb);
	return ret;
}

static int mstar_remove(struct platform_device *pdev)
{
	struct mstar_glue	*glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	clk_disable_unprepare(glue->clk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mstar_musb_of_match[] = {
	{ .compatible = "mstar,msc313-musb" },
	{},
};
MODULE_DEVICE_TABLE(of, mstar_musb_of_match);
#endif

static struct platform_driver mstar_driver = {
	.probe		= mstar_probe,
	.remove		= mstar_remove,
	.driver		= {
		.name	= "musb-mstar",
		.of_match_table = of_match_ptr(mstar_musb_of_match),
	},
};

MODULE_DESCRIPTION("MStar MUSB Glue Layer");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_LICENSE("GPL v2");
module_platform_driver(mstar_driver);
