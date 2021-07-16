// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
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

#define NUM_IRQ		76
#define IRQS_PER_REG	16
#define STRIDE		4

#define REG_MASK	0x0
#define REG_ACK		0x28
#define REG_TYPE	0x40
#define REG_STATUS	0xc0

struct ssd20xd_gpi {
	struct regmap *regmap;
	struct irq_domain *domain;
};

#define REG_OFFSET(_hwirq) ((hwirq >> 4) * STRIDE)
#define BIT_OFFSET(_hwirq) (1 << (hwirq & 0xf))

static void ssd20xd_gpi_mask_irq(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	struct ssd20xd_gpi *gpi = irq_data_get_irq_chip_data(data);
	int offset_reg = REG_OFFSET(hwirq);
	int offset_bit = BIT_OFFSET(hwirq);

	regmap_update_bits(gpi->regmap, REG_MASK + offset_reg, offset_bit, offset_bit);
}

static void ssd20xd_gpi_unmask_irq(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	struct ssd20xd_gpi *gpi = irq_data_get_irq_chip_data(data);
	int offset_reg = REG_OFFSET(hwirq);
	int offset_bit = BIT_OFFSET(hwirq);

	regmap_update_bits(gpi->regmap, REG_MASK + offset_reg, offset_bit, 0);

	/* When unmasking a spurious interrupt is generated so ack it */
	regmap_update_bits_base(gpi->regmap, REG_ACK + offset_reg,
			offset_bit, offset_bit, NULL, false, true);
}

static void ssd20xd_gpi_irq_ack(struct irq_data *data)
{
	struct ssd20xd_gpi *gpi = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	int offset_reg = REG_OFFSET(hwirq);
	int offset_bit = BIT_OFFSET(hwirq);

	regmap_update_bits_base(gpi->regmap, REG_ACK + offset_reg,
			offset_bit, offset_bit, NULL, false, true);
}

static int ssd20xd_gpi_set_type_irq(struct irq_data *data, unsigned int flow_type)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	struct ssd20xd_gpi *gpi = irq_data_get_irq_chip_data(data);
	int offset_reg = REG_OFFSET(hwirq);
	int offset_bit = BIT_OFFSET(hwirq);

	switch (flow_type) {
	case IRQ_TYPE_EDGE_FALLING:
		regmap_update_bits(gpi->regmap, REG_TYPE + offset_reg, offset_bit, offset_bit);
		break;
	case IRQ_TYPE_EDGE_RISING:
		regmap_update_bits(gpi->regmap, REG_TYPE + offset_reg, offset_bit, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip ssd20xd_gpi_chip = {
	.name		= "GPI",
	.irq_mask	= ssd20xd_gpi_mask_irq,
	.irq_unmask	= ssd20xd_gpi_unmask_irq,
	.irq_ack	= ssd20xd_gpi_irq_ack,
	.irq_set_type	= ssd20xd_gpi_set_type_irq,
};

static int ssd20xd_gpi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				    unsigned int nr_irqs, void *arg)
{
	struct ssd20xd_gpi *intc = domain->host_data;
	struct irq_fwspec *fwspec = arg;
	int i;

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, fwspec->param[0] + i,
				&ssd20xd_gpi_chip, intc, handle_edge_irq, NULL, NULL);

	return 0;
}

static void ssd20xd_gpi_domain_free(struct irq_domain *domain, unsigned int virq,
				    unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops ssd20xd_gpi_domain_ops = {
	.alloc = ssd20xd_gpi_domain_alloc,
	.free = ssd20xd_gpi_domain_free,
};

static const struct regmap_config ssd20xd_gpi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
	.use_raw_spinlock = true,
};

static void ssd20xd_gpi_chainedhandler(struct irq_desc *desc)
{
	struct ssd20xd_gpi *intc = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int irqbit, hwirq, status;
	int i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < NUM_IRQ / IRQS_PER_REG; i++) {
		int offset_reg = STRIDE * i;
		int offset_irq = IRQS_PER_REG * i;

		regmap_read(intc->regmap, REG_STATUS + offset_reg, &status);

		while (status) {
			irqbit = __ffs(status);
			hwirq = offset_irq + irqbit;
			generic_handle_domain_irq(intc->domain, hwirq);
			status &= ~BIT(irqbit);
		}
	}

	chained_irq_exit(chip, desc);
}

static int ssd20xd_gpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssd20xd_gpi *intc;
	void __iomem *base;
	int irq;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (!base)
		return -ENODEV;

	intc->regmap = devm_regmap_init_mmio(dev, base, &ssd20xd_gpi_regmap_config);
	if (IS_ERR(intc->regmap))
		return PTR_ERR(intc->regmap);

	intc->domain = irq_domain_add_linear(dev->of_node, NUM_IRQ, &ssd20xd_gpi_domain_ops, intc);
	if (!intc->domain)
		return -ENOMEM;

	irq = of_irq_get(dev->of_node, 0);
	if (irq <= 0) {
		irq_domain_remove(intc->domain);
		return irq;
	}

	irq_set_chained_handler_and_data(irq, ssd20xd_gpi_chainedhandler,
			intc);

	return 0;
}

static const struct of_device_id ssd20xd_of_match[] = {
	{ .compatible = "sstar,ssd20xd-gpi" },
	{}
};

static struct platform_driver ssd20xd_gpi_driver = {
	.driver = {
		.name = "SSD20XD GPI",
		.of_match_table	= ssd20xd_of_match,
	},
	.probe = ssd20xd_gpi_probe,
};

module_platform_driver(ssd20xd_gpi_driver);
