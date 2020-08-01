// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#include <soc/mstar/pmsleep.h>

#define NUM_IRQ		32
#define REG_STATUS	0x0

struct msc313_sleep_intc {
	struct irq_domain *domain;
	struct regmap *pmsleep;
};

static void msc313_sleep_intc_mask_irq(struct irq_data *data)
{
}

static void msc313_sleep_intc_unmask_irq(struct irq_data *data)
{
}

static void msc313_sleep_intc_irq_eoi(struct irq_data *data)
{
}

static int msc313_sleep_intc_set_type_irq(struct irq_data *data, unsigned int flow_type)
{
	return 0;
}

static struct irq_chip msc313_pm_intc_chip = {
	.name		= "PM-INTC",
	.irq_mask	= msc313_sleep_intc_mask_irq,
	.irq_unmask	= msc313_sleep_intc_unmask_irq,
	.irq_eoi	= msc313_sleep_intc_irq_eoi,
	.irq_set_type	= msc313_sleep_intc_set_type_irq,
};

static int msc313_sleep_intc_domain_map(struct irq_domain *domain,
		unsigned int irq, irq_hw_number_t hw)
{
	struct msc313_sleep_intc *intc = domain->host_data;

	irq_set_chip_and_handler(irq, &msc313_pm_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, intc);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops msc313_pm_intc_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = msc313_sleep_intc_domain_map,
};

static irqreturn_t msc313_sleep_intc_chainedhandler(int irq, void *data)
{
	struct irq_domain *domain = data;
	struct msc313_sleep_intc *intc = domain->host_data;
	u32 status;
	unsigned int hwirq, tmp;

	regmap_read(intc->pmsleep, MSTAR_PMSLEEP_INTSTATUS + 4, &tmp);
	status = tmp << 16;
	regmap_read(intc->pmsleep, MSTAR_PMSLEEP_INTSTATUS, &tmp);
	status |= tmp;

	while (status) {
		hwirq = __ffs(status);
		generic_handle_domain_irq(intc->domain, hwirq);
		status &= ~BIT(hwirq);
	}

	return IRQ_HANDLED;
}

static int msc313_pm_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_sleep_intc *intc;
	int ret, gicint;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	intc->pmsleep = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(intc->pmsleep))
		return PTR_ERR(intc->pmsleep);

	gicint = of_irq_get(dev->of_node, 0);
	if (gicint <= 0)
		return gicint;

	intc->domain = irq_domain_add_linear(dev->of_node, NUM_IRQ,
			&msc313_pm_intc_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	ret = request_irq(gicint, msc313_sleep_intc_chainedhandler, IRQF_SHARED,
					  "pmsleep", intc->domain);
	if (ret) {
		irq_domain_remove(intc->domain);
		return ret;
	}

	return 0;
}

static const struct of_device_id msc313_pm_intc_of_match[] = {
	{ .compatible = "mstar,msc313-pm-intc" },
	{}
};

static struct platform_driver msc313_pm_intc_driver = {
	.driver = {
		.name = "MSC313 PM INTC",
		.of_match_table	= msc313_pm_intc_of_match,
	},
	.probe = msc313_pm_intc_probe,
};
module_platform_driver(msc313_pm_intc_driver);
