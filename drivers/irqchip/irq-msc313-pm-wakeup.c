// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer
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

#define NUM_IRQ		8

static struct reg_field mask_field = REG_FIELD(MSTAR_PMSLEEP_WAKEUPSOURCE, 0, 7);
static struct reg_field type_field = REG_FIELD(MSTAR_PMSLEEP_REG24, 0, 7);
static struct reg_field status_field = REG_FIELD(MSTAR_PMSLEEP_WAKEINT_STATUS, 0, 7);

struct msc313_sleep_intc {
	struct regmap_field *mask;
	struct regmap_field *type;
	struct regmap_field *status;
};

static void msc313_pm_wakeup_intc_mask_irq(struct irq_data *data)
{
	struct msc313_sleep_intc *intc = irq_data_get_irq_chip_data(data);

	regmap_field_update_bits(intc->mask, 1 << data->hwirq, ~0);
}

static void msc313_pm_wakeup_intc_unmask_irq(struct irq_data *data)
{
	struct msc313_sleep_intc *intc = irq_data_get_irq_chip_data(data);

	regmap_field_update_bits(intc->mask, 1 << data->hwirq, 0);
}

static void msc313_pm_wakeup_intc_irq_eoi(struct irq_data *data)
{
}

static int msc313_pm_wakeup_intc_set_type_irq(struct irq_data *data,
		unsigned int flow_type)
{
	return 0;
}

static struct irq_chip msc313_pm_wakeup_intc_chip = {
	.name		= "PM-WAKEUP",
	.irq_mask	= msc313_pm_wakeup_intc_mask_irq,
	.irq_unmask	= msc313_pm_wakeup_intc_unmask_irq,
	.irq_eoi	= msc313_pm_wakeup_intc_irq_eoi,
	.irq_set_type	= msc313_pm_wakeup_intc_set_type_irq,
};

static irqreturn_t msc313_pm_wakeup_intc_chainedhandler(int irq, void *data)
{
	struct irq_domain *domain = data;
	struct msc313_sleep_intc *intc = domain->host_data;
	unsigned int hwirq, status;

	regmap_field_read(intc->status, &status);
	printk("wakeupint %x\n", status);

	while (status) {
		hwirq = __ffs(status);
		generic_handle_domain_irq(domain, hwirq);
		status &= ~BIT(hwirq);
	}

	return IRQ_HANDLED;
}

static int msc313_pm_wakeup_intc_domain_map(struct irq_domain *domain,
		unsigned int irq, irq_hw_number_t hw)
{
	struct msc313_sleep_intc *intc = domain->host_data;

	irq_set_chip_and_handler(irq, &msc313_pm_wakeup_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, intc);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops msc313_pm_wakeup_intc_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = msc313_pm_wakeup_intc_domain_map,
};

static int __init msc313_pm_wakeup_intc_of_init(struct device_node *node,
				   struct device_node *parent)
{
	int ret = 0, irq;
	struct regmap *pmsleep;
	struct msc313_sleep_intc *intc;
	struct irq_domain *domain;

	irq = of_irq_get(node, 0);
	if (irq <= 0)
		return irq;

	pmsleep = syscon_regmap_lookup_by_phandle(node, "mstar,pmsleep");
	if (IS_ERR(pmsleep))
		return PTR_ERR(pmsleep);

	intc = kzalloc(sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	intc->mask = regmap_field_alloc(pmsleep, mask_field);
	intc->type = regmap_field_alloc(pmsleep, type_field);
	intc->status = regmap_field_alloc(pmsleep, status_field);

	/* The masks survive deep sleep so clear them. */
	regmap_field_write(intc->mask, ~0);

	domain = irq_domain_add_linear(node, NUM_IRQ,
			&msc313_pm_wakeup_intc_domain_ops, intc);
	if (!domain) {
		ret = -ENOMEM;
		goto out_free;
	}

	request_irq(irq, msc313_pm_wakeup_intc_chainedhandler, IRQF_SHARED,
				"pmsleep", domain);

	return 0;

out_free:
	kfree(intc);
	return ret;
}

IRQCHIP_DECLARE(mstar_msc313_pm_wakeup_intc, "mstar,msc313-pm-wakeup-intc",
		msc313_pm_wakeup_intc_of_init);
