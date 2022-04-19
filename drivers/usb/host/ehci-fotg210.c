// SPDX-License-Identifier: GPL-2.0
/*
 * Faraday FOTG210 and FUSBH200
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/phy.h>
#include <linux/usb/of.h>

#include <linux/regmap.h>
#include <linux/usb/mstar_usbc.h>
#include <linux/mfd/syscon.h>

#include "ehci.h"

struct fotg210_regs {
	/* USBCMD: offset 0x00 */
	u32	command;
	/* USBSTS: offset 0x04 */
	u32	status;
	/* USBINTR: offset 0x08 */
	u32	intr_enable;
	/* FRINDEX: offset 0x0C */
	u32	frame_index;
	/* CTRLDSSEGMENT: offset 0x10 */
	u32	segment;
	/* PERIODICLISTBASE: offset 0x14 */
	u32	frame_list;
	/* ASYNCLISTADDR: offset 0x18 */
	u32	async_next;

	u32	reserved1;
	/* PORTSC: offset 0x20 */
	u32	port_status;

	u32	reserved2[3];

	/* BMCSR: offset 0x30 */
	u32	bmcsr; /* Bus Monitor Control/Status Register */
#define BMCSR_HOST_SPD_TYP	(3<<9)
#define BMCSR_VBUS_OFF		(1<<4)
#define BMCSR_INT_POLARITY	(1<<3)

	/* BMISR: offset 0x34 */
	u32	bmisr; /* Bus Monitor Interrupt Status Register*/
#define BMISR_OVC		(1<<1)

	/* BMIER: offset 0x38 */
	u32	bmier; /* Bus Monitor Interrupt Enable Register */
#define BMIER_OVC_EN		(1<<1)
#define BMIER_VBUS_ERR_EN	(1<<0)

	u32     reserved3[15];

	/* OTGCSR: offet 0x70 */
	u32     otgcsr;
#define OTGCSR_HOST_SPD_TYP     (3 << 22)
#define OTGCSR_A_BUS_DROP	(1 << 5)
#define OTGCSR_A_BUS_REQ	(1 << 4)

	/* OTGISR: offset 0x74 */
	u32     otgisr;
#define OTGISR_OVC	(1 << 10)

	u32     reserved4[15];

	/* GMIR: offset 0xB4 */
	u32     gmir;
#define GMIR_INT_POLARITY	(1 << 3) /*Active High*/
#define GMIR_MHC_INT		(1 << 2)
#define GMIR_MOTG_INT		(1 << 1)
#define GMIR_MDEV_INT	(1 << 0)
};

#define DRIVER_DESC "FOTG210 Host Controller (EHCI) Driver"

#define hcd_to_fotg210_ehci_priv(h) \
	((struct fotg210_ehci_priv *)hcd_to_ehci(h)->priv)

struct fotg210_ehci_priv {
	struct clk	*pclk;
	struct regmap	*usbc;
	bool mstar;
	bool clocked;
};

#ifdef CONFIG_ARCH_MSTARV7
static inline unsigned int mstar_ehci_readl(const struct ehci_hcd *ehci, __u32 __iomem *regs)
{
	unsigned int value;
	u16 l, h;
	unsigned regsaddr = (unsigned) regs;
	void *base = (void*)(regsaddr & ~0xff);
	unsigned offset = (regsaddr & 0xff) * 2;

	if(regs == &ehci->regs->configured_flag)
		return FLAG_CF;

	l = readw(base + offset);
	h = readw_relaxed(base + (offset + 4));
	value = l | (h << 16);
	//printk("mstar readl - %px, %px %u - 0x%x.. %px %px\n", regs, base, offset, value, base + offset, base + (offset + 4));

	return value;
}

static inline void mstar_ehci_writel(const struct ehci_hcd *ehci, const unsigned int val, __u32 __iomem *regs)
{
	unsigned regsaddr = (unsigned) regs;
	void *base = (void*)(regsaddr & ~0xff);
	unsigned offset = (regsaddr & 0xff) * 2;
	u16 l = val & 0xffff, h = (val >> 16) & 0xffff;

	/* We don't have a config flag register */
	if(regs == &ehci->regs->configured_flag)
		return;

	//ehci_info(ehci, "mstar writel, %px, %px %x - %08x\n", regs, base, offset, val);

	writew_relaxed(l, base + offset);
	writew(h, base + (offset + 4));
}
#endif

static unsigned int fotg210_port_speed(struct ehci_hcd *ehci, unsigned int portsc)
{
	struct fotg210_regs *regs = (void *) ehci->regs;
	u32 portspeed;

	if(ehci->fusbh200)
		portspeed = (ehci_readl(ehci, &regs->bmcsr)
				& BMCSR_HOST_SPD_TYP) >> 9;
	else
		portspeed = (ehci_readl(ehci, &regs->otgcsr)
				& OTGCSR_HOST_SPD_TYP) >> 22;

	switch (portspeed) {
	case 0:
		return 0;
	case 1:
		return USB_PORT_STAT_LOW_SPEED;
	case 2:
	default:
		return USB_PORT_STAT_HIGH_SPEED;
	}
}

static struct hc_driver __read_mostly ehci_fotg210_hc_driver;

static const struct ehci_driver_overrides ehci_fotg210_drv_overrides __initconst = {
	.extra_priv_size = sizeof(struct fotg210_ehci_priv),
};

static void fotg210_enable_occ(struct ehci_hcd *ehci, u32 portsc)
{
	struct fotg210_regs *regs = (void *) ehci->regs;
	u32 isr;

	if (ehci->fusbh200) {
		isr = ehci_readl(ehci, &regs->bmisr);
		ehci_writel(ehci, isr | BMISR_OVC,
				&regs->bmisr);
	}
	else {
		isr = ehci_readl(ehci, &regs->otgisr);
		ehci_writel(ehci, isr | OTGISR_OVC,
				&regs->otgisr);
	}
}

bool fotg210_port_oc(struct ehci_hcd *ehci) {
	struct fotg210_regs *regs = (void *) ehci->regs;
	u32 isr;

	if(ehci->fusbh200){
		isr = ehci_readl(ehci, &regs->bmisr);
		if (isr & BMISR_OVC)
			return true;
	}
	else {
		isr = ehci_readl(ehci, &regs->otgisr);
		if (isr & OTGISR_OVC)
			return true;
	}

	return false;
}

static void fotg210_start_clock(struct fotg210_ehci_priv *fotg210_ehci)
{
	if (fotg210_ehci->clocked)
		return;

	if (fotg210_ehci->pclk)
		clk_prepare_enable(fotg210_ehci->pclk);

	fotg210_ehci->clocked = true;
}

static void fotg210_stop_clock(struct fotg210_ehci_priv *fotg210_ehci)
{
	if (!fotg210_ehci->clocked)
		return;

	if (fotg210_ehci->pclk)
		clk_disable_unprepare(fotg210_ehci->pclk);

	fotg210_ehci->clocked = false;
}

static void fotg210_start_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct fotg210_ehci_priv *fotg210_ehci = hcd_to_fotg210_ehci_priv(hcd);
	struct fotg210_regs *regs = (void *) ehci->regs;
	u32 value;

	fotg210_start_clock(fotg210_ehci);

	/* Do the various fix ups needed to make the mstar version of fusbh200 work */
	if (fotg210_ehci->mstar) {
		regmap_update_bits(fotg210_ehci->usbc, MSTAR_USBC_REG_RSTCTRL,
				MSTAR_RSTCTRL_REG_SUSPEND | MSTAR_RSTCTRL_UHC_XIU,
				MSTAR_RSTCTRL_REG_SUSPEND | MSTAR_RSTCTRL_UHC_XIU);

		/*
		 * Without this plugging in a fs/ls device after unplugging a hs one
		 * doesn't work as the port tries to enumerate at hs.
		 */
		regmap_update_bits(fotg210_ehci->usbc, MSTAR_USBC_CONFIG,
				MSTAR_USBC_CONFIG_RSTSPDDIS,
				MSTAR_USBC_CONFIG_RSTSPDDIS);
	}

	if (ehci->fusbh200) {
		value = ehci_readl(ehci, &regs->bmcsr);
		ehci_writel(ehci, (value & ~BMCSR_VBUS_OFF) | BMCSR_INT_POLARITY,
							&regs->bmcsr);
	}
	else {
		iowrite32(GMIR_MDEV_INT | GMIR_MOTG_INT | GMIR_INT_POLARITY,
				&regs->gmir);

		value = ioread32(&regs->otgcsr);
		value &= ~OTGCSR_A_BUS_DROP;
		value |= OTGCSR_A_BUS_REQ;
		iowrite32(value, &regs->otgcsr);
	}
}

static void fotg210_stop_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct fotg210_ehci_priv *fotg210_ehci = hcd_to_fotg210_ehci_priv(hcd);

	fotg210_stop_clock(fotg210_ehci);
}

static u32 __iomem * fotg210_get_port_status_reg(struct ehci_hcd *ehci, unsigned portnum)
{
	struct fotg210_regs *regs = (void *) ehci->regs;

	WARN_ON(portnum != 0);

	return &regs->port_status;
}

static int ehci_fotg210_drv_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver = &ehci_fotg210_hc_driver;
	struct fotg210_ehci_priv *fotg210_ehci;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct ehci_hcd *ehci;
	struct usb_hcd *hcd;
	int retval;
	int irq;
	int len;

	if (usb_disabled())
		return -ENODEV;

	//retval = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	//if (retval)
	//	goto fail_create_hcd;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		retval = -ENODEV;
		goto fail_create_hcd;
	}

	hcd = usb_create_hcd(driver, dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}
	fotg210_ehci = hcd_to_fotg210_ehci_priv(hcd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hcd->regs)) {
		retval = PTR_ERR(hcd->regs);
		goto fail_request_resource;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->has_tt = 1;

	/* It's OK not to supply this clock */
	fotg210_ehci->pclk = clk_get(dev, "PCLK");
	if (PTR_ERR(fotg210_ehci->pclk) == -EPROBE_DEFER) {
		/*
		 * Percolate deferrals, for anything else,
		 * just live without the clocking.
		 */
		retval = PTR_ERR(fotg210_ehci->pclk);
		goto fail_request_resource;
	}
	else
		fotg210_ehci->pclk = NULL;

	ehci = hcd_to_ehci(hcd);
	/* registers start at offset 0x0 */
	ehci->caps = hcd->regs;
	len = HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	ehci->regs = (void __iomem *)(hcd->regs + len);

	if (dev->of_node && of_device_is_compatible(dev->of_node,
			"faraday,fotg210"))
		ehci->fotg210 = 1;
	if (dev->of_node && of_device_is_compatible(dev->of_node,
			"faraday,fusbh200"))
		ehci->fusbh200 = 1;

#ifdef CONFIG_ARCH_MSTARV7
	if (dev->of_node && of_device_is_compatible(dev->of_node,
			"mstar,msc313-ehci")) {
		fotg210_ehci->mstar = true;
		ehci->ehci_writel = mstar_ehci_writel;
		ehci->ehci_readl = mstar_ehci_readl;
		ehci->fusbh200 = 1;
		fotg210_ehci->usbc = syscon_regmap_lookup_by_phandle(dev->of_node, "mstar,usbc");
		if (IS_ERR(fotg210_ehci->usbc))
			goto fail_request_resource;
	}
#endif

	ehci->get_port_status_reg = fotg210_get_port_status_reg;
	ehci->enable_port_occ = fotg210_enable_occ;
	ehci->port_speed = fotg210_port_speed;

	fotg210_start_ehci(pdev);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;
	device_wakeup_enable(hcd->self.controller);

	return retval;

fail_add_hcd:
	fotg210_stop_ehci(pdev);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n",
		dev_name(&pdev->dev), retval);

	return retval;
}

static int ehci_fotg210_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	fotg210_stop_ehci(pdev);

	return 0;
}

static int __maybe_unused ehci_fotg210_drv_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct fotg210_ehci_priv *fotg210_ehci = hcd_to_fotg210_ehci_priv(hcd);
	int ret;

	ret = ehci_suspend(hcd, false);
	if (ret)
		return ret;

	fotg210_stop_clock(fotg210_ehci);
	return 0;
}

static int __maybe_unused ehci_fotg210_drv_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct fotg210_ehci_priv *fotg210_ehci = hcd_to_fotg210_ehci_priv(hcd);

	fotg210_start_clock(fotg210_ehci);
	ehci_resume(hcd, false);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id fotg210_ehci_dt_ids[] = {
	{ .compatible = "faraday,fotg210" },
	{ .compatible = "faraday,fusbh200" },
	{ .compatible = "mstar,msc313-ehci" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, fotg210_ehci_dt_ids);
#endif

static SIMPLE_DEV_PM_OPS(ehci_fotg210_pm_ops, ehci_fotg210_drv_suspend,
					ehci_fotg210_drv_resume);

static struct platform_driver ehci_fotg210_driver = {
	.probe		= ehci_fotg210_drv_probe,
	.remove		= ehci_fotg210_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "fotg210-ehci",
		.pm	= &ehci_fotg210_pm_ops,
		.of_match_table	= of_match_ptr(fotg210_ehci_dt_ids),
	},
};

static int __init ehci_fotg210_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ehci_init_driver(&ehci_fotg210_hc_driver, &ehci_fotg210_drv_overrides);
	return platform_driver_register(&ehci_fotg210_driver);
}
module_init(ehci_fotg210_init);

static void __exit ehci_fotg210_cleanup(void)
{
	platform_driver_unregister(&ehci_fotg210_driver);
}
module_exit(ehci_fotg210_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:fotg210-ehci");
MODULE_AUTHOR("Daniel Palmer");
MODULE_LICENSE("GPL");
