// SPDX-License-Identifier: GPL-2.0+
/*
 * MStar "glue layer" based on jz4740.c
 *
 * Copyright (C) 2013, Apelete Seketeli <apelete@seketeli.net>
 * Copyright (C) 2023, Daniel Palmer <daniel@0x0f.com>
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

//#define MSTAR_MUSB_DEBUG

struct mstar_glue {
	struct platform_device	*pdev;
	struct musb		*musb;
	struct clk		*clk;
	struct regmap		*usbc;
};

static irqreturn_t mstar_musb_interrupt(int irq, void *__hci)
{
	irqreturn_t retval = IRQ_NONE, retval_dma = IRQ_NONE;
	struct musb *musb = __hci;
	unsigned long flags;

	if (IS_ENABLED(CONFIG_USB_INVENTRA_DMA) && musb->dma_controller)
		retval_dma = dma_controller_irq(irq, musb->dma_controller);

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

#ifdef MSTAR_MUSB_DEBUG
	printk("musb irq: 0x%x, 0x%x, 0x%x\n",
		musb->int_usb, musb->int_tx, musb->int_rx);
#endif
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

	if (retval == IRQ_HANDLED || retval_dma == IRQ_HANDLED)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static const struct musb_hdrc_config mstar_musb_config = {
	/* Silicon does not implement USB OTG. */
	.multipoint = 0,
	/* Max EPs scanned, driver will decide which EP can be used. */
	.num_eps    = 4,
};

static struct musb_hdrc_platform_data mstar_musb_platform_data = {
	.mode   = MUSB_PERIPHERAL,
	.config = &mstar_musb_config,
};

static int mstar_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller->parent;
	int err;

	musb->phy = devm_of_phy_get_by_index(dev, dev->of_node, 0);
	if (IS_ERR(musb->phy)) {
		err = PTR_ERR(musb->phy);
		if (err != -ENODEV) {
			dev_err(dev, "Unable to get PHY\n");
			return err;
		}

		musb->phy = NULL;
	}

	err = phy_init(musb->phy);
	if (err) {
		dev_err(dev, "Failed to init PHY\n");
		return err;
	}

	err = phy_power_on(musb->phy);
	if (err) {
		dev_err(dev, "Unable to power on PHY\n");
		goto err_phy_shutdown;
	}

	musb->isr = mstar_musb_interrupt;

	return 0;

err_phy_shutdown:
	phy_exit(musb->phy);
	return err;
}

static int mstar_musb_exit(struct musb *musb)
{
	phy_power_off(musb->phy);
	phy_exit(musb->phy);

	return 0;
}

/* io jank workarounds */
#define WORDADDR(_offset) (_offset << 1)
#define BYTEADDR(_offset) (((_offset & ~1) << 1) + (_offset & 1))

static u8 mstar_musb_readb(void __iomem *addr, u32 offset)
{
	u8 ret = readb(addr + BYTEADDR(offset));

#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x(0x%x): 0x%x\n", __func__, __LINE__, addr,
			offset, BYTEADDR(offset), ret);
#endif

	return ret;
}

static void mstar_musb_writeb(void __iomem *addr, u32 offset, u8 data)
{
#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x(0x%x) <= 0x%02x\n", __func__, __LINE__, addr,
			offset, BYTEADDR(offset), data);
#endif

	writeb(data, addr + BYTEADDR(offset));
}

static u8 mstar_musb_clearb(void __iomem *addr, u32 offset)
{
	u8 ret = mstar_musb_readb(addr, offset);

	mstar_musb_writeb(addr, offset, 0);
#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x: 0x%02x\n", __func__, __LINE__, addr, offset, ret);
#endif

	return ret;
}

static u16 mstar_musb_readw(void __iomem *addr, u32 offset)
{
	u16 ret = readw(addr + WORDADDR(offset));

#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x(0x%x): 0x%04x\n", __func__, __LINE__, addr,
			offset, WORDADDR(offset), ret);
#endif

	return ret;
}

static void mstar_musb_writew(void __iomem *addr, u32 offset, u16 data)
{
#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x <= 0x%04x\n", __func__, __LINE__, addr, offset, data);
#endif
	writew(data, addr + WORDADDR(offset));
}

static u16 mstar_musb_clearw(void __iomem *addr, u32 offset)
{
	u16 ret = mstar_musb_readw(addr, offset);

#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x: 0x%04x\n", __func__, __LINE__, addr, offset, ret);
#endif
	mstar_musb_writew(addr, offset, 0);

	return ret;
}

static u32 mstar_musb_readl(void __iomem *addr, u32 offset)
{
	u32 ret;

	ret = readw(addr + WORDADDR(offset));
	ret |= readw(addr + WORDADDR(offset) + 4) << 16;

#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x(0x%x): 0x%08x\n", __func__, __LINE__, addr,
			offset, WORDADDR(offset), ret);
#endif

	return ret;
}

static void mstar_musb_writel(void __iomem *addr, u32 offset, u32 data)
{
#ifdef MSTAR_MUSB_DEBUG
	printk("%s:%d - 0x%p 0x%x <= 0x%08x\n", __func__, __LINE__, addr, offset, data);
#endif
	writew(data & 0xffff, addr + WORDADDR(offset));
	writew((data >> 16) & 0xffff, addr + WORDADDR(offset) + 4);
}

static u32 mstar_musb_ep_offset(u8 epnum, u16 offset)
{
	WARN_ON(offset);

	return 0x200 + (0x20 * epnum);
}

static u32 mstar_musb_fifo_offset(u8 epnum)
{
	return 0x40 + (0x8 * epnum);
}

static void mstar_musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len,
				    const u8 *src)
{
	struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;

	if (unlikely(len == 0))
		return;

	dev_dbg(musb->controller, "%cX ep%d fifo %p count %d buf %p\n",
			'T', hw_ep->epnum, fifo, len, src);

	for (int i = 0; i < len; i++)
		writeb(src[i], fifo);
}

static void mstar_musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;

	if (unlikely(len == 0))
		return;

	dev_dbg(musb->controller, "%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->epnum, fifo, len, dst);

	for (int i = 0; i < len; i++)
		dst[i] = readb(fifo);
}

/*
 * DMA has not been confirmed to work with CONFIG_USB_INVENTRA_DMA,
 * so let's not set up the dma function pointers yet.
 */
static const struct musb_platform_ops mstar_musb_ops = {
	.quirks		= MUSB_DMA_INVENTRA,

	/* setup */
	.init		= mstar_musb_init,
	.exit		= mstar_musb_exit,

	/* io */
	.ep_offset	= mstar_musb_ep_offset,
	.fifo_offset	= mstar_musb_fifo_offset,
	.read_fifo	= mstar_musb_read_fifo,
	.write_fifo	= mstar_musb_write_fifo,
	.readb		= mstar_musb_readb,
	.writeb		= mstar_musb_writeb,
	.clearb		= mstar_musb_clearb,
	.readw		= mstar_musb_readw,
	.writew		= mstar_musb_writew,
	.clearw		= mstar_musb_clearw,
	.readl		= mstar_musb_readl,
	.writel		= mstar_musb_writel,

	/* dma */
#ifdef CONFIG_USB_INVENTRA_DMA
	.dma_init	= musbhs_dma_controller_create_noirq,
	.dma_exit	= musbhs_dma_controller_destroy,
#endif
};

static int mstar_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data *pdata = &mstar_musb_platform_data;
	struct device *dev = &pdev->dev;
	struct platform_device *musb;
	struct mstar_glue *glue;
	struct clk *clk;
	int ret;

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

	clk = devm_clk_get_enabled(&pdev->dev, "udc");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err_platform_device_put;
	}

	musb->dev.parent = &pdev->dev;
	musb->dev.dma_mask = &musb->dev.coherent_dma_mask;
	musb->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	device_set_of_node_from_dev(&musb->dev, dev);

	glue->pdev = musb;
	glue->clk = clk;

	pdata->platform_ops = &mstar_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
					    pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err_platform_device_put;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err_platform_device_put;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err_platform_device_put;
	}

	return 0;

err_platform_device_put:
	platform_device_put(musb);
	return ret;
}

static int mstar_remove(struct platform_device *pdev)
{
	struct mstar_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->pdev);

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
