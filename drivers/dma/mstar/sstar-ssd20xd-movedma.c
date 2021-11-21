// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Daniel Palmer<daniel@thingy.jp>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>
#include <linux/of_dma.h>

#include "../dmaengine.h"

#define DRIVER_NAME "ssd20xd-movedma"
#define CHANNELS 1

#define REG_EN			0x0
#define REG_OFFSETEN		0x4
#define REG_ENSTATUS		0x8
#define REG_SRC_START_ADDR_L	0xc
#define REG_SRC_START_ADDR_H	0x10
#define REG_DST_START_ADDR_L	0x14
#define REG_DST_START_ADDR_H	0x18
#define REG_BYTE_CNT_L		0x1c
#define REG_BYTE_CNT_H		0x20
#define REG_SWRST		0x90
/* only bit zero? */
#define REG_IRQMASK		0x98
#define REG_IRQCLR		0xa0
#define REG_IRQSTATUS		0xa8
#define REG_MIUCFG		0xc0
#define REG_DMADIR		0x140
#define REG_DMAMODE		0x144
#define REG_DEVSEL		0x148

static const struct reg_field en_field			= REG_FIELD(REG_EN, 0, 0);
static const struct reg_field offseten_field		= REG_FIELD(REG_OFFSETEN, 0, 0);
/* txfr config */
static const struct reg_field srcstartaddrl_field	= REG_FIELD(REG_SRC_START_ADDR_L, 0, 15);
static const struct reg_field srcstartaddrh_field	= REG_FIELD(REG_SRC_START_ADDR_H, 0, 15);
static const struct reg_field deststartaddrl_field	= REG_FIELD(REG_DST_START_ADDR_L, 0, 15);
static const struct reg_field deststartaddrh_field	= REG_FIELD(REG_DST_START_ADDR_H, 0, 15);
static const struct reg_field bytecntl_field		= REG_FIELD(REG_BYTE_CNT_L, 0, 15);
static const struct reg_field bytecnth_field		= REG_FIELD(REG_BYTE_CNT_H, 0, 11);
/* */
static const struct reg_field swrst_field		= REG_FIELD(REG_SWRST, 0 , 0);
/* irq */
static const struct reg_field irqmask_field		= REG_FIELD(REG_IRQMASK, 0, 0);
static const struct reg_field irqclear_field		= REG_FIELD(REG_IRQCLR, 0, 0);
/* miu */
static const struct reg_field miuselen_field		= REG_FIELD(REG_MIUCFG, 0, 0);
static const struct reg_field miusrcsel_field		= REG_FIELD(REG_MIUCFG, 1, 1);
static const struct reg_field miudstsel_field		= REG_FIELD(REG_MIUCFG, 2, 2);
/* dma settings */
static const struct reg_field dmadir_field		= REG_FIELD(REG_DMADIR, 0, 0);
static const struct reg_field dmamode_field		= REG_FIELD(REG_DMAMODE, 0, 0);
#define DMAMODE_DEVICE	1
static const struct reg_field devsel_field		= REG_FIELD(REG_DEVSEL, 0, 0);

static const struct regmap_config ssd20xd_movedma_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

struct ssd20xd_movedma_desc {
	struct dma_async_tx_descriptor tx;
	size_t len;
	dma_addr_t addr;
	struct list_head queue_node;
};

struct ssd20xd_movedma {
	struct dma_device dma_device;
	struct clk *clk;
	int irq;

	spinlock_t lock;
	struct tasklet_struct tasklet;

	struct regmap *regmap;
	struct regmap_field *en;

	/* tfxr settings */
	struct regmap_field *offseten;
	struct regmap_field *srcstartaddrl, *srcstartaddrh;
	struct regmap_field *dststartaddrl, *dststartaddrh;
	struct regmap_field *bytecntl, *bytecnth;

	struct regmap_field *swrst;

	/* dma settings */
	struct regmap_field *dmadir;
	struct regmap_field *dmamode;
	struct regmap_field *devsel;

	/* miu */
	struct regmap_field *miuselen;
	struct regmap_field *miusrcsel;
	struct regmap_field *miudstsel;

	/* irqs */
	struct regmap_field *irqmask;
	struct regmap_field *irqclr;

	struct ssd20xd_movedma_desc *inflight;
	struct list_head queue;
	struct list_head completed;
};

struct ssd20xd_movedma_chan {
	struct ssd20xd_movedma *movedma;
	struct dma_chan chan;
};

#define to_chan(ch) container_of(ch, struct ssd20xd_movedma_chan, chan)
#define to_desc(desc) container_of(desc, struct ssd20xd_movedma_desc, tx)

static void ssd20xd_movedma_tasklet(unsigned long data) {
	struct ssd20xd_movedma *movedma = (struct ssd20xd_movedma*) data;
	struct ssd20xd_movedma_desc *desc;
	struct list_head *cur, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&movedma->lock, flags);
	list_for_each_safe(cur, tmp, &movedma->completed) {
		desc = list_entry(cur, struct ssd20xd_movedma_desc, queue_node);
		list_del(&desc->queue_node);
		dma_cookie_complete(&desc->tx);
		dma_descriptor_unmap(&desc->tx);
		dmaengine_desc_get_callback_invoke(&desc->tx, NULL);
		dma_run_dependencies(&desc->tx);
		this_cpu_ptr(desc->tx.chan->local)->bytes_transferred += desc->len;
		kfree(desc);
	}
	spin_unlock_irqrestore(&movedma->lock, flags);
}

static irqreturn_t ssd20xd_movedma_irq(int irq, void *data)
{
	struct ssd20xd_movedma *movedma = data;
	unsigned long flags;
	unsigned int value;

	regmap_read(movedma->regmap, REG_IRQSTATUS, &value);
	regmap_field_force_write(movedma->irqclr, 1);

	spin_lock_irqsave(&movedma->lock, flags);
	if (movedma->inflight) {
		list_move_tail(&movedma->inflight->queue_node, &movedma->completed);
		movedma->inflight = NULL;
	}
	spin_unlock_irqrestore(&movedma->lock, flags);

	tasklet_schedule(&movedma->tasklet);

	return IRQ_HANDLED;
}

static enum dma_status ssd20xd_movedma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return DMA_ERROR;
}

static void ssd20xd_movedma_do_single(struct ssd20xd_movedma_chan *chan, struct ssd20xd_movedma_desc *desc)
{
	struct ssd20xd_movedma *movedma = chan->movedma;
	bool read = false;
	unsigned long flags;

	//printk("%s:%d\n", __func__, __LINE__);
	//printk("movedma doing 0x%08x(0x%x) %s\n", desc->addr, desc->len, read ? "read" : "write");

	spin_lock_irqsave(&movedma->lock, flags);
	movedma->inflight = desc;
	spin_unlock_irqrestore(&movedma->lock, flags);

	regmap_field_force_write(movedma->swrst, 1);
	mdelay(1);
	regmap_field_write(movedma->offseten, 0);
	regmap_field_write(movedma->miuselen, 1);
	regmap_field_write(movedma->miusrcsel, 0);
	regmap_field_write(movedma->miudstsel, 0);
	regmap_field_write(movedma->dmamode, DMAMODE_DEVICE);
	regmap_field_write(movedma->devsel, 0);
	regmap_field_write(movedma->irqmask, 0);

	regmap_field_write(movedma->dmadir, read ? 1 : 0);
	regmap_field_write(movedma->srcstartaddrl, desc->addr);
	regmap_field_write(movedma->srcstartaddrh, desc->addr >> 16);
	regmap_field_write(movedma->bytecntl, desc->len);
	regmap_field_write(movedma->bytecnth, desc->len >> 16);

	regmap_field_force_write(movedma->en, 1);
}

static void ssd20xd_movedma_issue_pending(struct dma_chan *chan)
{
	struct ssd20xd_movedma_desc *desc;
	struct ssd20xd_movedma_chan *ch = to_chan(chan);
	struct ssd20xd_movedma *movedma = ch->movedma;
	unsigned long flags;

	spin_lock_irqsave(&movedma->lock, flags);
	desc = list_first_entry_or_null(&movedma->queue,
			struct ssd20xd_movedma_desc, queue_node);
	spin_unlock_irqrestore(&movedma->lock, flags);

	if (desc)
		ssd20xd_movedma_do_single(ch, desc);
}

static dma_cookie_t ssd20xd_movedma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct ssd20xd_movedma_chan *chan = to_chan(tx->chan);
	struct ssd20xd_movedma_desc *desc = to_desc(tx);
	struct ssd20xd_movedma *movedma = chan->movedma;
	dma_cookie_t cookie;
	unsigned long flags;

	spin_lock_irqsave(&movedma->lock, flags);
	list_add_tail(&desc->queue_node, &movedma->queue);
	spin_unlock_irqrestore(&movedma->lock, flags);

	cookie = dma_cookie_assign(tx);

	return cookie;
}

static struct dma_async_tx_descriptor* ssd20xd_movedma_prep_slave_sg(struct dma_chan *chan,
		struct scatterlist *sgl, unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct ssd20xd_movedma_desc *desc;
	u32 dmaaddr, dmalen;

	if (sg_len != 1) {
		printk("only one sg for now\n");
		return NULL;
	}

	if (direction != DMA_DEV_TO_MEM && direction != DMA_MEM_TO_DEV)
		return NULL;

	dmaaddr = sg_dma_address(sgl);
	dmalen = sg_dma_len(sgl);

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->len = dmalen;
	desc->addr = dmaaddr;
	desc->tx.tx_submit = ssd20xd_movedma_tx_submit;

	return &desc->tx;
}

static int ssd20xd_movedma_probe(struct platform_device *pdev)
{
	struct ssd20xd_movedma_chan *chan;
	struct ssd20xd_movedma *movedma;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	int i, ret;

	printk("movedma probe\n");

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	movedma = devm_kzalloc(&pdev->dev, sizeof(*movedma), GFP_KERNEL);
	if (!movedma)
		return -ENOMEM;

	spin_lock_init(&movedma->lock);
	INIT_LIST_HEAD(&movedma->queue);
	INIT_LIST_HEAD(&movedma->completed);
	tasklet_init(&movedma->tasklet, ssd20xd_movedma_tasklet, (unsigned long) movedma);

	movedma->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!movedma->irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, movedma->irq, ssd20xd_movedma_irq, IRQF_SHARED,
			dev_name(&pdev->dev), movedma);
	if (ret)
		return ret;

	movedma->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(movedma->clk))
		return PTR_ERR(movedma->clk);

	clk_prepare_enable(movedma->clk);

	movedma->dma_device.dev = &pdev->dev;
	movedma->dma_device.device_tx_status = ssd20xd_movedma_tx_status;
	movedma->dma_device.device_issue_pending = ssd20xd_movedma_issue_pending;
	movedma->dma_device.src_addr_widths = BIT(4);
	movedma->dma_device.dst_addr_widths = BIT(4);
	movedma->dma_device.directions = BIT(DMA_DEV_TO_MEM) |
					 BIT(DMA_MEM_TO_DEV);
	movedma->dma_device.device_prep_slave_sg = ssd20xd_movedma_prep_slave_sg;

	INIT_LIST_HEAD(&movedma->dma_device.channels);

	for(i = 0; i < CHANNELS; i++){
		chan = devm_kzalloc(&pdev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		chan->movedma = movedma;
		chan->chan.device = &movedma->dma_device;

		list_add_tail(&chan->chan.device_node, &movedma->dma_device.channels);
	}

	movedma->regmap = devm_regmap_init_mmio(&pdev->dev, base, &ssd20xd_movedma_regmap_config);
	if (IS_ERR(movedma->regmap))
		return PTR_ERR(movedma->regmap);

	movedma->en = devm_regmap_field_alloc(dev, movedma->regmap, en_field);

	movedma->offseten = devm_regmap_field_alloc(dev, movedma->regmap, offseten_field);
	movedma->dmadir = devm_regmap_field_alloc(dev, movedma->regmap, dmadir_field);
	movedma->dmamode = devm_regmap_field_alloc(dev, movedma->regmap, dmamode_field);
	movedma->devsel = devm_regmap_field_alloc(dev, movedma->regmap, devsel_field);

	movedma->srcstartaddrl = devm_regmap_field_alloc(dev, movedma->regmap, srcstartaddrl_field);
	movedma->srcstartaddrh = devm_regmap_field_alloc(dev, movedma->regmap, srcstartaddrh_field);
	movedma->dststartaddrl = devm_regmap_field_alloc(dev, movedma->regmap, deststartaddrl_field);
	movedma->dststartaddrh = devm_regmap_field_alloc(dev, movedma->regmap, deststartaddrh_field);
	movedma->bytecntl = devm_regmap_field_alloc(dev, movedma->regmap, bytecntl_field);
	movedma->bytecnth = devm_regmap_field_alloc(dev, movedma->regmap, bytecnth_field);

	movedma->miuselen = devm_regmap_field_alloc(dev, movedma->regmap, miuselen_field);
	movedma->miusrcsel = devm_regmap_field_alloc(dev, movedma->regmap, miusrcsel_field);
	movedma->miudstsel = devm_regmap_field_alloc(dev, movedma->regmap, miudstsel_field);

	movedma->swrst = devm_regmap_field_alloc(dev, movedma->regmap, swrst_field);

	movedma->irqmask = devm_regmap_field_alloc(dev, movedma->regmap, irqmask_field);
	movedma->irqclr = devm_regmap_field_alloc(dev, movedma->regmap, irqclear_field);

	ret = dma_async_device_register(&movedma->dma_device);
	if(ret)
		return ret;

	return of_dma_controller_register(pdev->dev.of_node,
			of_dma_xlate_by_chan_id, &movedma->dma_device);
}

static const struct of_device_id ssd20xd_movedma_of_match[] = {
	{ .compatible = "sstar,ssd20xd-movedma", },
	{},
};

static struct platform_driver ssd20xd_movedma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ssd20xd_movedma_of_match,
	},
	.probe = ssd20xd_movedma_probe,
};
builtin_platform_driver(ssd20xd_movedma_driver);
