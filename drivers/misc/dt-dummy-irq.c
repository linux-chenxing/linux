// SPDX-License-Identifier: GPL-2.0-only
/*
 * DT Dummy IRQ handler driver.
 *
 * Copyright (C) 2013 Jiri Kosina
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>

#define DRIVER_NAME "dummy-irq"

static irqreturn_t dummy_interrupt(int irq, void *data)
{
	static int count = 0;

	if (count == 0) {
		printk(KERN_INFO "dummy-irq: interrupt occurred on IRQ %d\n",
				irq);
		count++;
	}

	return IRQ_NONE;
}

static int dummy_probe(struct platform_device *pdev)
{
	int count, i, irq, ret;

	count = of_irq_count(pdev->dev.of_node);

	dev_info(&pdev->dev, "dummy-irq: registering %d irqs\n", count);
	for(i = 0; i < count; i++){
		irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		if (!irq){
			dev_err(&pdev->dev, "failed to register irq at %d -> %d", i, irq);
			continue;
		}

		ret = devm_request_irq(&pdev->dev, irq, dummy_interrupt, IRQF_SHARED,
				dev_name(&pdev->dev), &pdev->dev);
		if(ret){
			dev_err(&pdev->dev, "failed to request irq at %d -> %d", i, ret);
			continue;
		}


		dev_info(&pdev->dev, "dummy-irq: registered irq at %d -> %d", i, irq);
	}

	return 0;
}

static int dummy_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dummy_dt_ids[] = {
	{ .compatible = "dummy-irq" },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_fcie_dt_ids);

static struct platform_driver dt_dummy_driver = {
	.probe = dummy_probe,
	.remove = dummy_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = dummy_dt_ids,
	},
};

module_platform_driver(dt_dummy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("Dummy IRQ handler driver");
