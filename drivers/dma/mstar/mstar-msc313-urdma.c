// SPDX-License-Identifier: GPL-2.0
//

#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>

#define DRIVER_NAME "msc313-urdma"
#define CHANNELS 1

static const struct regmap_config msc313_urdma_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

struct msc313_urdma {
	struct dma_device dma_device;
	struct clk *clk;
};

struct msc313_urdma_chan {
	struct dma_chan chan;
	int irq;
	struct regmap *regmap;
	struct list_head queue;
	dma_cookie_t cookie;
};

#define to_chan(ch) container_of(ch, struct msc313_urdma_chan, chan);

struct msc313_urdma_desc {
	struct dma_async_tx_descriptor tx;
	size_t len;
	dma_addr_t dst;
	dma_addr_t src;
	struct list_head queue_node;
};

#define to_desc(desc) container_of(desc, struct msc313_urdma_desc, tx);

static const struct of_device_id msc313_urdma_of_match[] = {
	{ .compatible = "mstar,msc313-urdma", },
	{},
};

static irqreturn_t msc313_urdma_irq(int irq, void *data)
{
	struct msc313_urdma *urdma = data;
	return IRQ_HANDLED;
}

static enum dma_status msc313_urdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate){
	printk("urdma tx status\n");
	return DMA_ERROR;
}

void msc313_urdma_issue_pending(struct dma_chan *chan){
	printk("urdma issue pending\n");
}

static dma_cookie_t msc313_tx_submit(struct dma_async_tx_descriptor *tx){
	struct msc313_urdma_chan *chan = to_chan(tx->chan);
	struct msc313_urdma_desc *desc = to_desc(tx);
	list_add_tail(&desc->queue_node, &chan->queue);
	return chan->cookie++;
}

static struct dma_async_tx_descriptor* msc313_urdma_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags){

	struct msc313_urdma_desc *desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->len = len;
	desc->src = src;
	desc->dst = dst;
	desc->tx.tx_submit = msc313_tx_submit;

	return &desc->tx;
};

static int msc313_urdma_probe(struct platform_device *pdev)
{
	struct msc313_urdma *urdma;
	struct msc313_urdma_chan *chan;
	struct resource *res;
	void __iomem *base;
	int i, ret;

	printk("urdma probe\n");

	urdma = devm_kzalloc(&pdev->dev, sizeof(*urdma), GFP_KERNEL);
	if (!urdma)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	urdma->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(urdma->clk)) {
		return PTR_ERR(urdma->clk);
	}

	urdma->dma_device.dev = &pdev->dev;
	urdma->dma_device.device_tx_status = msc313_urdma_tx_status;
	urdma->dma_device.device_issue_pending = msc313_urdma_issue_pending;
	urdma->dma_device.src_addr_widths = BIT(4);
	urdma->dma_device.dst_addr_widths = BIT(4);
	urdma->dma_device.directions = BIT(DMA_MEM_TO_MEM);
	urdma->dma_device.device_prep_dma_memcpy = msc313_urdma_prep_dma_memcpy;

	INIT_LIST_HEAD(&urdma->dma_device.channels);

	dma_cap_set(DMA_MEMCPY, urdma->dma_device.cap_mask);

	for(i = 0; i < CHANNELS; i++){
		chan = devm_kzalloc(&pdev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		INIT_LIST_HEAD(&chan->queue);

		chan->regmap = devm_regmap_init_mmio(&pdev->dev, base + (0x40 * i),
				&msc313_urdma_regmap_config);
		if(IS_ERR(chan->regmap)){
			return PTR_ERR(chan->regmap);
		}

		chan->irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		if (!chan->irq)
			return -EINVAL;
		ret = devm_request_irq(&pdev->dev, chan->irq, msc313_urdma_irq, IRQF_SHARED,
				dev_name(&pdev->dev), chan);

		chan->chan.device = &urdma->dma_device;

		list_add_tail(&chan->chan.device_node, &urdma->dma_device.channels);
	}

	ret = dma_async_device_register(&urdma->dma_device);
	if(ret)
		goto out;

	ret = clk_prepare_enable(urdma->clk);
	if (ret)
		goto out;

	out:
	return ret;
}

static struct platform_driver msc313_urdma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_urdma_of_match,
	},
	.probe = msc313_urdma_probe,
};
builtin_platform_driver(msc313_urdma_driver);

