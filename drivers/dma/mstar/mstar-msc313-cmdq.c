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

/*
 *
 * MSC313 CMDQ DMA controller
 *
 * The MSC313 has 1 of these. The MSC313e seems to have 3.
 * The vendor SDK seems to mostly use it for moving stuff to and from
 * the camera ip blocks. Apparently this thing can do registers writes,
 * polling for a bit to be set or cleared among other operations.
 *
 * descriptors are apparently 8 bytes like this;
 * | 0 - 1 | 2 - 3 | 4 - 6                              | 7:0-3             | 7:4-7 |
 * | mask  | data  | addr                               | cmd               | dbg   |
 * |       |       | this is an address in io/riu space | 0x0 - nop         |       |
 * |       |       | which seems to be 4-byte addressed | 0x1 - write       |       |
 * |       |       |                                    | 0x3 - poll eq     |       |
 * |       |       |                                    | 0xb - poll not eq |       |
 *
 * 0x004 -
 * 0
 * en
 *
 * 0x008 - dma mode
 * 0x00 - increment mode
 * 0x01 - direct mode
 * 0x04 - ring mode
 *
 * 0x00c - trigger?
 *   1
 * start?
 *
 * 0x010 - start pointer
 * 0x018 - end pointer
 * 0x020 - offset pointer
 * 0x040 - miu sel
 * 0x044 - ??
 * 0x080 - ??
 * 0x088 - wait trig
 * 0x090 -
 * 0x0a0 - timeout
 * 0x0a4 - ""
 * 0x0c4 - reset
 *  0
 * ~rst
 *
 * 0x100 - something to do with errors
 * 0x10c - ""
 * 0x110 - something to do with irq
 * 0x11c - irq mask
 * 0x120 - irq clear
 * 0x128 - timer
 * 0x12c - ratio
 *
 * descriptors are 64 bits in length
 *
 * maybe 63 - 60?
 * 63 - 56 | 55 - 32 | 31 - 16 | 15 - 0
 *   cmd   |  addr   |  data   |  mask
 *         |
 *
 * 0x10
 * 0x20
 * 0x30
 * 0xb0
 */

#define DRIVER_NAME "msc313-cmdq"
#define CHANNELS 1

#define REG_RESET 0x0c4

static struct reg_field rst_nrst_field = REG_FIELD(REG_RESET, 0, 0);

static const struct regmap_config msc313_cmdq_regmap_config = {
		.name = DRIVER_NAME,
		.reg_bits = 16,
		.val_bits = 16,
		.reg_stride = 4
};

struct msc313_cmdq {
	struct dma_device dma_device;
	struct clk *clk;
};

struct msc313_cmdq_chan {
	struct dma_chan chan;
	int irq;
	struct regmap *regmap;
	struct list_head queue;
	dma_cookie_t cookie;

	struct regmap_field *nrst;
};

#define to_chan(ch) container_of(ch, struct msc313_cmdq_chan, chan);

struct msc313_cmdq_desc {
	struct dma_async_tx_descriptor tx;
	size_t len;
	dma_addr_t dst;
	dma_addr_t src;
	struct list_head queue_node;
};

#define to_desc(desc) container_of(desc, struct msc313_cmdq_desc, tx);

static const struct of_device_id msc313_cmdq_of_match[] = {
	{ .compatible = "mstar,msc313-cmdq", },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_cmdq_of_match);

static irqreturn_t msc313_cmdq_irq(int irq, void *data)
{
	struct msc313_cmdq *cmdq = data;
	return IRQ_HANDLED;
}

static enum dma_status msc313_cmdq_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate){
	printk("cmdq tx status\n");
	return DMA_ERROR;
}

void msc313_cmdq_issue_pending(struct dma_chan *chan){
	printk("cmdq issue pending\n");
}

static dma_cookie_t msc313_tx_submit(struct dma_async_tx_descriptor *tx){
	struct msc313_cmdq_chan *chan = to_chan(tx->chan);
	struct msc313_cmdq_desc *desc = to_desc(tx);
	list_add_tail(&desc->queue_node, &chan->queue);
	return chan->cookie++;
}

static struct dma_async_tx_descriptor* msc313_cmdq_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags){

	struct msc313_cmdq_desc *desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->len = len;
	desc->src = src;
	desc->dst = dst;
	desc->tx.tx_submit = msc313_tx_submit;

	return &desc->tx;
};

static int msc313_cmdq_probe(struct platform_device *pdev)
{
	struct msc313_cmdq *cmdq;
	struct msc313_cmdq_chan *chan;
	struct resource *res;
	void __iomem *base;
	int i, ret;

	printk("cmdq probe\n");

	cmdq = devm_kzalloc(&pdev->dev, sizeof(*cmdq), GFP_KERNEL);
	if (!cmdq)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	cmdq->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(cmdq->clk)) {
		return PTR_ERR(cmdq->clk);
	}

	cmdq->dma_device.dev = &pdev->dev;
	cmdq->dma_device.device_tx_status = msc313_cmdq_tx_status;
	cmdq->dma_device.device_issue_pending = msc313_cmdq_issue_pending;
	cmdq->dma_device.src_addr_widths = BIT(4);
	cmdq->dma_device.dst_addr_widths = BIT(4);
	cmdq->dma_device.directions = BIT(DMA_MEM_TO_MEM);
	cmdq->dma_device.device_prep_dma_memcpy = msc313_cmdq_prep_dma_memcpy;

	INIT_LIST_HEAD(&cmdq->dma_device.channels);

	dma_cap_set(DMA_MEMCPY, cmdq->dma_device.cap_mask);

	for(i = 0; i < CHANNELS; i++){
		chan = devm_kzalloc(&pdev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		INIT_LIST_HEAD(&chan->queue);

		chan->regmap = devm_regmap_init_mmio(&pdev->dev, base + (0x40 * i),
				&msc313_cmdq_regmap_config);
		if(IS_ERR(chan->regmap)){
			return PTR_ERR(chan->regmap);
		}

		chan->irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		if (!chan->irq)
			return -EINVAL;
		ret = devm_request_irq(&pdev->dev, chan->irq, msc313_cmdq_irq, IRQF_SHARED,
				dev_name(&pdev->dev), chan);

		chan->chan.device = &cmdq->dma_device;

		chan->nrst = devm_regmap_field_alloc(&pdev->dev, chan->regmap, rst_nrst_field);

		list_add_tail(&chan->chan.device_node, &cmdq->dma_device.channels);
	}

	ret = dma_async_device_register(&cmdq->dma_device);
	if(ret)
		goto out;

	ret = clk_prepare_enable(cmdq->clk);
	if (ret)
		goto out;

	out:
	return ret;
}

static int msc313_cmdq_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver msc313_cmdq_driver = {
	.probe = msc313_cmdq_probe,
	.remove = msc313_cmdq_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_cmdq_of_match,
	},
};

module_platform_driver(msc313_cmdq_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("MStar MSC313 CMDQ driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
