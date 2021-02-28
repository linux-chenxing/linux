// SPDX-License-Identifier: GPL-2.0

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>

#include "../dmaengine.h"

#define DRIVER_NAME		"msc313-urdma"
#define CHANNELS		2
#define AUTOSUSPEND_DELAY	100

#define REG_CTRL 0x0
static const struct reg_field ctrl_sw_rst = REG_FIELD(REG_CTRL, 0, 0);
static const struct reg_field ctrl_urdma_mode = REG_FIELD(REG_CTRL, 1, 1);
static const struct reg_field ctrl_tx_urdma_en = REG_FIELD(REG_CTRL, 2, 2);
static const struct reg_field ctrl_rx_urdma_en = REG_FIELD(REG_CTRL, 3, 3);
static const struct reg_field ctrl_tx_endian = REG_FIELD(REG_CTRL, 4, 4);
static const struct reg_field ctrl_rx_endian = REG_FIELD(REG_CTRL, 5, 5);
static const struct reg_field ctrl_tx_sw_rst = REG_FIELD(REG_CTRL, 6, 6);
static const struct reg_field ctrl_rx_sw_rst = REG_FIELD(REG_CTRL, 7, 7);
static const struct reg_field ctrl_rx_op_mode = REG_FIELD(REG_CTRL, 11, 11);
static const struct reg_field ctrl_tx_busy = REG_FIELD(REG_CTRL, 12, 12);
static const struct reg_field ctrl_rx_busy = REG_FIELD(REG_CTRL, 13, 13);

#define REG_INTR_THRESHOLD 0x4

#define REG_TX_BUF_BASE_H 0x8
static const struct reg_field tx_buf_h_field	= REG_FIELD(REG_TX_BUF_BASE_H, 0, 11);
#define REG_TX_BUF_BASE_L 0xc
static const struct reg_field tx_buf_l_field	= REG_FIELD(REG_TX_BUF_BASE_L, 0, 15);
#define REG_TX_BUF_SIZE 0x10
static const struct reg_field tx_buf_sz_field	= REG_FIELD(REG_TX_BUF_SIZE, 0, 15);
#define REG_TX_BUF_RPTR 0x14
static const struct reg_field tx_buf_rptr_field	= REG_FIELD(REG_TX_BUF_RPTR, 0, 15);
#define REG_TX_BUF_WPTR 0x18
static const struct reg_field tx_buf_wptr_field	= REG_FIELD(REG_TX_BUF_WPTR, 0, 15);
#define REG_TX_TIMEOUT 0x1c

#define REG_RX_BUF_BASE_H 0x20
static const struct reg_field rx_buf_h_field = REG_FIELD(REG_RX_BUF_BASE_H, 0, 11);
#define REG_RX_BUF_BASE_L 0x24
static const struct reg_field rx_buf_l_field = REG_FIELD(REG_RX_BUF_BASE_L, 0, 15);
#define REG_RX_BUF_SIZE 0x28
static const struct reg_field rx_buf_sz_field = REG_FIELD(REG_RX_BUF_SIZE, 0, 15);
#define REG_RX_BUF_WPTR 0x2c
#define REG_RX_TIMEOUT 0x30

#define REG_STATUS 0x34
static const struct reg_field status_rx_intr_clr_field = REG_FIELD(REG_STATUS, 0, 0);
static const struct reg_field status_rx_intr_en1_field = REG_FIELD(REG_STATUS, 1, 1);
static const struct reg_field status_rx_intr_en2_field = REG_FIELD(REG_STATUS, 2, 2);
static const struct reg_field status_rx_intr1_field = REG_FIELD(REG_STATUS, 4, 4);
static const struct reg_field status_rx_intr2_field = REG_FIELD(REG_STATUS, 5, 5);
static const struct reg_field status_rx_mcu_intr_field = REG_FIELD(REG_STATUS, 7, 7);
static const struct reg_field status_tx_intr_clr_field = REG_FIELD(REG_STATUS, 8, 8);
static const struct reg_field status_tx_intr_en_field = REG_FIELD(REG_STATUS, 9, 9);
static const struct reg_field status_tx_mcu_intr_field = REG_FIELD(REG_STATUS, 15, 15);

static const struct regmap_config msc313_urdma_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

struct msc313_urdma_desc {
	struct dma_async_tx_descriptor tx;
	dma_addr_t buf;
	size_t len;
	struct list_head queue_node;
};

#define to_desc(desc) container_of(desc, struct msc313_urdma_desc, tx);

struct msc313_urdma;

struct msc313_urdma_chan {
	struct msc313_urdma *urdma;

	spinlock_t lock;
	struct dma_chan chan;
	dma_cookie_t cookie;
	bool rxchan;

	/* queue management */
	struct tasklet_struct tasklet;
	struct list_head queue;
	struct list_head complete;
	struct msc313_urdma_desc *inflight;

	struct regmap_field *en;
	struct regmap_field *endian;
	struct regmap_field *sw_rst;
	struct regmap_field *busy;

	/* interrupts */
	struct regmap_field *int_clr;
	struct regmap_field *int_en1;
	struct regmap_field *int_en2;
	struct regmap_field *int1;
	struct regmap_field *int2;
	struct regmap_field *mcu_int;

	/* buffer */
	struct regmap_field *buf_h, *buf_l, *buf_sz;
	struct regmap_field *rptr, *wptr;

	size_t kern_buff_size;
	dma_addr_t kern_buff_addr;
};

struct msc313_urdma {
	/* lock for urdma_mode en/dis */
	spinlock_t lock;

	struct dma_device dma_device;
	struct clk *clk;

	/* global regfields */
	struct regmap_field *sw_rst;
	struct regmap_field *urdma_mode;

	struct msc313_urdma_chan chans[CHANNELS];
};

#define to_chan(ch) container_of(ch, struct msc313_urdma_chan, chan);

static const struct of_device_id msc313_urdma_of_match[] = {
	{ .compatible = "mstar,msc313-urdma", },
	{},
};

static irqreturn_t msc313_urdma_irq(int irq, void *data)
{
	struct msc313_urdma *urdma = data;

	printk("%s:%d\n", __func__, __LINE__);

	for (int i = 0; i < CHANNELS; i++) {
		unsigned int mcu = 0, int1 = 0, int2 = 0;
		struct msc313_urdma_chan *chan = &urdma->chans[i];
		struct msc313_urdma_desc *inflight;
		unsigned long flags;
		bool schedule = false;

		spin_lock_irqsave(&chan->lock, flags);

		if (chan->rxchan) {
			regmap_field_read(chan->int1, &int1);
			regmap_field_read(chan->int2, &int2);
		}
		regmap_field_read(chan->mcu_int, &mcu);

		printk("%s:%d - %d int1: %d, int2: %d, mcu: %d\n", __func__, __LINE__, i, int1, int2, mcu);
		{
			unsigned int rptr = ~0, wptr = ~0;
			if (chan->rptr)
				regmap_field_read(chan->rptr, &rptr);
			if (chan->wptr)
				regmap_field_read(chan->wptr, &wptr);
			printk("%s:%d - rptr %d, wptr %d\n", __func__, __LINE__, rptr, wptr);
		}
		if (!mcu)
			goto unlock;

		regmap_field_force_write(chan->int_clr, 1);
		regmap_field_write(chan->en, 0);

		/* move the descriptor to the done list */
		inflight = chan->inflight;
		chan->inflight = NULL;
		if (inflight) {
			list_move_tail(&inflight->queue_node, &chan->complete);
			schedule = true;
			regmap_field_write(urdma->urdma_mode, 0);
		}
unlock:
		spin_unlock_irqrestore(&chan->lock, flags);

		spin_lock_irqsave(&urdma->lock, flags);
		regmap_field_write(urdma->urdma_mode, 0);
		spin_unlock_irqrestore(&urdma->lock, flags);

		if (schedule)
			tasklet_schedule(&chan->tasklet);
	}

	return IRQ_HANDLED;
}

static enum dma_status msc313_urdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	printk("%s:%d\n", __func__, __LINE__);

	return DMA_ERROR;
}

static void msc313_urdma_do_single(struct msc313_urdma_chan *chan, struct msc313_urdma_desc *desc)
{
	struct msc313_urdma *urdma = chan->urdma;
	unsigned int rptr, wptr;
	unsigned long flags;
	unsigned int wtf = desc->buf % chan->kern_buff_size;

	printk("%s:%d\n", __func__, __LINE__);

	pm_runtime_get_sync(chan->chan.device->dev);

	/* do it! */
	chan->inflight = desc;

	/* globally enable udma */
	spin_lock_irqsave(&urdma->lock, flags);
	regmap_field_write(chan->urdma->urdma_mode, 1);
	spin_unlock_irqrestore(&urdma->lock, flags);

	regmap_field_write(chan->en, 1);
	regmap_field_read(chan->rptr, &rptr);
	regmap_field_read(chan->wptr, &wptr);
	wptr = (wptr + desc->len) % chan->kern_buff_size;
	printk("%s:%d - wtf %d, rptr %d, wptr %d\n", __func__, __LINE__, wtf, rptr, wptr);

	regmap_field_write(chan->wptr, wptr);
}

static void msc313_urdma_issue_pending(struct dma_chan *chan)
{
	struct msc313_urdma_desc *desc;
	struct msc313_urdma_chan *ch = to_chan(chan);
	unsigned long flags;

	printk("%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&ch->lock, flags);
	desc = list_first_entry_or_null(&ch->queue, struct msc313_urdma_desc, queue_node);
	if (desc)
		msc313_urdma_do_single(ch, desc);
	spin_unlock_irqrestore(&ch->lock, flags);
}

static int msc313_urdma_chan_config(struct dma_chan *chan,
				    struct dma_slave_config *config)
{
	struct msc313_urdma_chan *ch = to_chan(chan);

	switch(config->direction) {
	case DMA_DEV_TO_MEM:
		ch->kern_buff_addr = config->src_addr;
		break;
	case DMA_MEM_TO_DEV:
		ch->kern_buff_addr = config->dst_addr;
		break;
	default:
		return -EINVAL;
	}

	ch->kern_buff_size = PAGE_SIZE;

	return 0;
}

static int msc313_urdma_chan_pause(struct dma_chan *chan)
{
	printk("%s:%d\n", __func__, __LINE__);

	return -EINVAL;
}

static int msc313_urdma_chan_resume(struct dma_chan *chan)
{
	printk("%s:%d\n", __func__, __LINE__);

	return -EINVAL;
}

static int msc313_urdma_terminate_all(struct dma_chan *chan)
{
	printk("%s:%d\n", __func__, __LINE__);

	return -EINVAL;
}

static dma_cookie_t msc313_urdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct msc313_urdma_desc *desc = to_desc(tx);
	struct msc313_urdma_chan *chan = to_chan(tx->chan);
	unsigned long flags;

	printk("%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&chan->lock, flags);
	list_add_tail(&desc->queue_node, &chan->queue);
	spin_unlock_irqrestore(&chan->lock, flags);

	printk("%s:%d %pX\n", __func__, __LINE__, desc);

	return dma_cookie_assign(tx);
}

static struct dma_async_tx_descriptor* msc313_urdma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct msc313_urdma_chan *ch = to_chan(chan);
	struct msc313_urdma_desc *desc;
	struct device *dev = chan->device->dev;
	u32 dmaaddr, dmalen;

	printk("%s:%d\n", __func__, __LINE__);

	if (ch->rxchan && direction != DMA_DEV_TO_MEM) {
		dev_err(dev, "Wrong DMA direction for RX channel\n");
		return -EINVAL;
	}
	else if (direction != DMA_MEM_TO_DEV) {
		dev_err(dev, "Wrong DMA direction for TX channel\n");
		return -EINVAL;
	}

	if (sg_len != 1) {
		dev_info(dev, "only one sg for now\n");
		return NULL;
	}

	dmaaddr = sg_dma_address(sgl);
	dmalen = sg_dma_len(sgl);

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->buf = dmaaddr;
	desc->len = dmalen;

	dma_async_tx_descriptor_init(&desc->tx, chan);

	desc->tx.tx_submit = msc313_urdma_tx_submit;

	return &desc->tx;
}

static void msc313_urdma_tasklet(unsigned long data)
{
	struct msc313_urdma_chan *chan = (struct msc313_urdma_chan*) data;
	struct device *dev = chan->chan.device->dev;
	struct list_head *cur, *tmp;
	unsigned long flags;

	printk("%s:%d %d\n", __func__, __LINE__, chan->rxchan);

	while (true) {
		struct msc313_urdma_desc *desc;

		spin_lock_irqsave(&chan->lock, flags);
		desc = list_first_entry_or_null(&chan->complete, struct msc313_urdma_desc, queue_node);
		if (desc)
			list_del(&desc->queue_node);
		spin_unlock_irqrestore(&chan->lock, flags);

		if (!desc)
			break;

		dma_cookie_complete(&desc->tx);
		dma_descriptor_unmap(&desc->tx);
		printk("%s:%d\n", __func__, __LINE__);
		//&desc->result
		dmaengine_desc_get_callback_invoke(&desc->tx, NULL);
		printk("%s:%d\n", __func__, __LINE__);
		dma_run_dependencies(&desc->tx);
		printk("%s:%d\n", __func__, __LINE__);
		this_cpu_ptr(chan->chan.local)->bytes_transferred += desc->len;
		kfree(desc);
		printk("%s:%d\n", __func__, __LINE__);
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
	}
}

static int msc313_urdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_urdma *urdma;
	struct regmap *regmap;
	void __iomem *base;
	int i, irq, ret;

	printk("urdma probe\n");

	urdma = devm_kzalloc(dev, sizeof(*urdma), GFP_KERNEL);
	if (!urdma)
		return -ENOMEM;

	spin_lock_init(&urdma->lock);
	platform_set_drvdata(pdev, urdma);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base,
			&msc313_urdma_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	urdma->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(urdma->clk))
		return PTR_ERR(urdma->clk);

	urdma->dma_device.dev = dev;
	urdma->dma_device.device_tx_status = msc313_urdma_tx_status;
	urdma->dma_device.device_issue_pending = msc313_urdma_issue_pending;
	urdma->dma_device.device_prep_slave_sg = msc313_urdma_prep_slave_sg;
	urdma->dma_device.device_terminate_all = msc313_urdma_terminate_all;
	urdma->dma_device.device_config = msc313_urdma_chan_config;
	urdma->dma_device.device_pause = msc313_urdma_chan_pause;
	urdma->dma_device.device_resume = msc313_urdma_chan_resume;
	urdma->dma_device.src_addr_widths = BIT(4);
	urdma->dma_device.dst_addr_widths = BIT(4);
	urdma->dma_device.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	urdma->dma_device.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;

	dma_cap_set(DMA_SLAVE, urdma->dma_device.cap_mask);

	INIT_LIST_HEAD(&urdma->dma_device.channels);

	irq = irq_of_parse_and_map(dev->of_node, i);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(dev, irq, msc313_urdma_irq, IRQF_SHARED, dev_name(dev), urdma);
	if (ret)
		return ret;

	/* global regfields */
	urdma->sw_rst = devm_regmap_field_alloc(dev, regmap, ctrl_sw_rst);
	urdma->urdma_mode = devm_regmap_field_alloc(dev, regmap, ctrl_urdma_mode);

	for (i = 0; i < CHANNELS; i++) {
		struct msc313_urdma_chan *chan = &urdma->chans[i];

		chan->urdma = urdma;
		spin_lock_init(&chan->lock);
		INIT_LIST_HEAD(&chan->queue);
		INIT_LIST_HEAD(&chan->complete);
		chan->chan.device = &urdma->dma_device;
		tasklet_init(&chan->tasklet, msc313_urdma_tasklet, (unsigned long) chan);

		if (i == 0) {
			chan->en = devm_regmap_field_alloc(dev, regmap, ctrl_tx_urdma_en);
			chan->endian = devm_regmap_field_alloc(dev, regmap, ctrl_tx_endian);
			chan->sw_rst = devm_regmap_field_alloc(dev, regmap, ctrl_tx_sw_rst);
			chan->busy = devm_regmap_field_alloc(dev, regmap, ctrl_tx_busy);

			chan->int_clr = devm_regmap_field_alloc(dev, regmap, status_tx_intr_clr_field);
			chan->int_en1 = devm_regmap_field_alloc(dev, regmap, status_tx_intr_en_field);
			chan->mcu_int = devm_regmap_field_alloc(dev, regmap, status_tx_mcu_intr_field);

			chan->buf_h = devm_regmap_field_alloc(dev, regmap, tx_buf_h_field);
			chan->buf_l = devm_regmap_field_alloc(dev, regmap, tx_buf_l_field);
			chan->buf_sz = devm_regmap_field_alloc(dev, regmap, tx_buf_sz_field);
			chan->rptr = devm_regmap_field_alloc(dev, regmap, tx_buf_rptr_field);
			chan->wptr = devm_regmap_field_alloc(dev, regmap, tx_buf_wptr_field);
		}
		else {
			chan->en = devm_regmap_field_alloc(dev, regmap, ctrl_rx_urdma_en);
			chan->endian = devm_regmap_field_alloc(dev, regmap, ctrl_rx_endian);
			chan->sw_rst = devm_regmap_field_alloc(dev, regmap, ctrl_rx_sw_rst);
			chan->busy = devm_regmap_field_alloc(dev, regmap, ctrl_rx_busy);

			chan->int_clr = devm_regmap_field_alloc(dev, regmap, status_rx_intr_clr_field);
			chan->int_en1 = devm_regmap_field_alloc(dev, regmap, status_rx_intr_en1_field);
			chan->int_en2 = devm_regmap_field_alloc(dev, regmap, status_rx_intr_en2_field);
			chan->int1 = devm_regmap_field_alloc(dev, regmap, status_rx_intr1_field);
			chan->int2 = devm_regmap_field_alloc(dev, regmap, status_rx_intr2_field);
			chan->mcu_int = devm_regmap_field_alloc(dev, regmap, status_rx_mcu_intr_field);

			chan->buf_h = devm_regmap_field_alloc(dev, regmap, rx_buf_h_field);
			chan->buf_l = devm_regmap_field_alloc(dev, regmap, rx_buf_l_field);
			chan->buf_sz = devm_regmap_field_alloc(dev, regmap, rx_buf_sz_field);

			chan->rxchan = true;
		}

		list_add_tail(&chan->chan.device_node, &urdma->dma_device.channels);
	}

	ret = dma_async_device_register(&urdma->dma_device);
	if (ret)
		return ret;

	/* runtime pm */
	pm_runtime_irq_safe(dev);
	pm_runtime_set_autosuspend_delay(dev, AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return of_dma_controller_register(pdev->dev.of_node,
			of_dma_xlate_by_chan_id, &urdma->dma_device);
}

static int msc313_urdma_suspend(struct device *dev)
{
	struct msc313_urdma *urdma = dev_get_drvdata(dev);

	printk("%s:%d\n", __func__, __LINE__);
	regmap_field_write(urdma->sw_rst, 1);

	return 0;
}

static int msc313_urdma_resume(struct device *dev)
{
	struct msc313_urdma *urdma = dev_get_drvdata(dev);

	printk("%s:%d\n", __func__, __LINE__);

	/* global rst */
	regmap_field_write(urdma->sw_rst, 1);
	mdelay(10);
	regmap_field_write(urdma->sw_rst, 0);

	for (int i = 0; i < CHANNELS; i++) {
		struct msc313_urdma_chan *chan = &urdma->chans[i];

		/* reset channel */
		regmap_field_write(chan->sw_rst, 1);
		mdelay(10);
		regmap_field_write(chan->sw_rst, 0);

		regmap_field_write(chan->int_en1, 1);

		regmap_field_write(chan->buf_h, chan->kern_buff_addr >> 16);
		regmap_field_write(chan->buf_l, chan->kern_buff_addr);
		regmap_field_write(chan->buf_sz, chan->kern_buff_size / 8);
	}

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(msc313_urdma_pm_ops,
			    msc313_urdma_suspend,
			    msc313_urdma_resume,
			    NULL);

static struct platform_driver msc313_urdma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_urdma_of_match,
		.pm = &msc313_urdma_pm_ops,
	},
	.probe = msc313_urdma_probe,
};
builtin_platform_driver(msc313_urdma_driver);
