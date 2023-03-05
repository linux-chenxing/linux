// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "../dmaengine.h"

#define DRIVER_NAME "msc313-bdma"
#define BDMA_AUTOSUSPEND_DELAY	100

#define REG_CTRL	0x0
#define REG_STATUS	0x4
#define REG_CONFIG	0x8
#define REG_MISC	0xc
#define REG_SRC_ADDR_L	0x10
#define REG_SRC_ADDR_H	0x14
#define REG_DST_ADDR_L	0x18
#define REG_DST_ADDR_H	0x1c
#define REG_SIZE_L	0x20
#define REG_SIZE_H	0x24

#define WIDTH_1			0x0
#define WIDTH_8			0x3
#define WIDTH_16		0x4
#define SLAVE_ID_MIU		0
#define SLAVE_WIDTH_MIU		WIDTH_16

#define TIMEOUTMS	5000

struct msc313_bdma_info {
	unsigned int channels;
	unsigned int channel_size;
	/* Some versions of this have a single IRQ for all channels. */
	bool single_irq;
};

struct msc313_bdma;

struct msc313_bdma_chan {
	struct msc313_bdma *bdma;

	struct dma_chan chan;

	spinlock_t lock;
	struct tasklet_struct tasklet;

	struct list_head queue;
	struct list_head completed;

	u8 slave_id;
	enum dma_slave_buswidth slave_width;
	u32 slave_offset;

	/* status */
	struct regmap_field *queued;
	struct regmap_field *busy;
	struct regmap_field *done;
	struct regmap_field *result0;

	/* interrupt handling */
	struct regmap_field *irq;
	struct regmap_field *int_en;

	/* transfer controls */
	struct regmap_field *trigger;
	struct regmap_field *stop;

	/* transfer settings */
	struct regmap_field *src_addr_l, *src_addr_h;
	struct regmap_field *dst_addr_l, *dst_addr_h;
	struct regmap_field *size_l, *size_h;

	struct regmap_field *src;
	struct regmap_field *src_width;
	struct regmap_field *dst;
	struct regmap_field *dst_width;
	struct regmap_field *miu_sel_ch0;

	struct msc313_bdma_desc* inflight;

	/* for working around dma getting stuck */
	struct timer_list watchdog;
};

struct msc313_bdma {
	const struct msc313_bdma_info *info;

	spinlock_t lock;
	int runningchannels;

	struct dma_device dma_device;
	struct clk *clk;

	struct msc313_bdma_chan chans[];
};

#define to_chan(ch) container_of(ch, struct msc313_bdma_chan, chan)

struct msc313_bdma_desc {
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result result;

	size_t len;
	dma_addr_t dst;
	dma_addr_t src;
	u8 src_id;
	u8 dst_id;
	u8 src_width;
	u8 dst_width;
	struct list_head queue_node;
};

#define to_desc(desc) container_of(desc, struct msc313_bdma_desc, tx);

static void msc313_bdma_write_low_high_pair(struct regmap_field *low,
		struct regmap_field *high, u32 value)
{
	regmap_field_write(low, value);
	regmap_field_write(high, value >> 16);
}

static unsigned int msc313_bdma_read_low_high_pair(struct regmap_field *low,
		struct regmap_field *high)
{
	unsigned int tmp, result;

	regmap_field_read(high, &tmp);
	result = tmp << 16;
	regmap_field_read(low, &tmp);
	result |= tmp;

	return result;
}

static void msc313_bdma_do_single(struct msc313_bdma_chan *chan,
		struct msc313_bdma_desc *desc)
{
	struct msc313_bdma *bdma = chan->bdma;

	BUG_ON(chan->inflight);

	chan->inflight = desc;

	pm_runtime_get_sync(chan->bdma->dma_device.dev);

	regmap_field_write(chan->src, desc->src_id);
	regmap_field_write(chan->src_width, desc->src_width);
	regmap_field_write(chan->dst, desc->dst_id);
	regmap_field_write(chan->dst_width, desc->dst_width);

	msc313_bdma_write_low_high_pair(chan->src_addr_l, chan->src_addr_h, desc->src);
	msc313_bdma_write_low_high_pair(chan->dst_addr_l, chan->dst_addr_h, desc->dst);
	msc313_bdma_write_low_high_pair(chan->size_l, chan->size_h, desc->len);

	spin_lock(&bdma->lock);
	bdma->runningchannels++;
	spin_unlock(&bdma->lock);

	/*
	 * setup the watchdog just in case we get stuck.
	 * This doesn't happen if things are setup correctly,..
	 */
	mod_timer(&chan->watchdog, jiffies + msecs_to_jiffies(TIMEOUTMS));

	regmap_field_force_write(chan->trigger, 1);
}

static void msc313_bdma_tasklet(unsigned long data)
{
	struct msc313_bdma_chan *chan = (struct msc313_bdma_chan*) data;
	struct msc313_bdma_desc *next;
	struct list_head *cur, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	/* Start next descriptor if there is one */
	next = list_first_entry_or_null(&chan->queue, struct msc313_bdma_desc, queue_node);
	if (next)
		msc313_bdma_do_single(chan, next);

	list_for_each_safe(cur, tmp, &chan->completed) {
		struct msc313_bdma_desc *desc =
				list_entry(cur, struct msc313_bdma_desc, queue_node);
		list_del(&desc->queue_node);

		dma_cookie_complete(&desc->tx);
		dma_descriptor_unmap(&desc->tx);

		dmaengine_desc_get_callback_invoke(&desc->tx, &desc->result);

		dma_run_dependencies(&desc->tx);

		this_cpu_ptr(chan->chan.local)->bytes_transferred += desc->len;
		kfree(desc);

		pm_runtime_mark_last_busy(chan->bdma->dma_device.dev);
		pm_runtime_put_autosuspend(chan->bdma->dma_device.dev);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static enum dma_status msc313_bdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);
	return ret;
}

static void msc313_bdma_issue_pending(struct dma_chan *chan)
{
	struct msc313_bdma_desc *desc;
	struct msc313_bdma_chan *ch = to_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&ch->lock, flags);
	if (ch->inflight) {
		dev_warn(chan->device->dev, "issue pending while something inflight\n");
		goto unlock;
	}

	desc = list_first_entry_or_null(&ch->queue, struct msc313_bdma_desc, queue_node);
	if(desc)
		msc313_bdma_do_single(ch, desc);

unlock:
	spin_unlock_irqrestore(&ch->lock, flags);
}

static dma_cookie_t msc313_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct msc313_bdma_desc *desc;
	struct msc313_bdma_chan *chan;
	unsigned long flags;

	chan = to_chan(tx->chan);
	desc = to_desc(tx);

	spin_lock_irqsave(&chan->lock, flags);
	list_add_tail(&desc->queue_node, &chan->queue);
	spin_unlock_irqrestore(&chan->lock, flags);

	return dma_cookie_assign(tx);
}

static struct dma_async_tx_descriptor* msc313_bdma_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct msc313_bdma_desc *desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);

	desc->len = len;
	desc->src_id = SLAVE_ID_MIU;
	desc->src = src - 0x20000000;
	desc->src_width = SLAVE_WIDTH_MIU;
	desc->dst_id = SLAVE_ID_MIU;
	desc->dst = dst - 0x20000000;
	desc->dst_width = SLAVE_WIDTH_MIU;

	//printk("memcpy %08x %08x\n", desc->dst, desc->src);

	desc->tx.tx_submit = msc313_tx_submit;

	return &desc->tx;
};

static int msc313_towidth(struct msc313_bdma_chan *ch)
{
	switch (ch->slave_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return WIDTH_1;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		return WIDTH_8;
	case DMA_SLAVE_BUSWIDTH_16_BYTES:
		return WIDTH_16;
	default:
		dev_err(ch->chan.device->dev, "unsupported width: %d", ch->slave_width);
		return -EINVAL;
	}
}

static struct dma_async_tx_descriptor* msc313_bdma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct msc313_bdma_chan *ch = to_chan(chan);
	struct msc313_bdma_desc *desc;
	u32 dmaaddr, dmalen;
	int width;

	if (sg_len != 1) {
		dev_info(chan->device->dev, "only one sg for now\n");
		return NULL;
	}

	dmaaddr = sg_dma_address(sgl);
	dmalen = sg_dma_len(sgl);

	if (dmalen % ch->slave_width) {
		dev_info(chan->device->dev, "len (0x%x) must be a multiple of the slave width (0x%x)\n",
				dmalen, ch->slave_width);
	}

	width = msc313_towidth(ch);
	if(width < 0)
		return NULL;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);

	desc->len = dmalen;

	switch(direction){
		case DMA_DEV_TO_MEM:
			desc->dst_id = SLAVE_ID_MIU;
			desc->dst = dmaaddr;
			desc->dst_width = SLAVE_WIDTH_MIU;
			desc->src_id = ch->slave_id;
			desc->src = ch->slave_offset;
			desc->src_width = width;
			break;
		case DMA_MEM_TO_DEV:
			desc->src_id = SLAVE_ID_MIU;
			desc->src = dmaaddr;
			desc->src_width = SLAVE_WIDTH_MIU;
			desc->dst_id = ch->slave_id;
			desc->dst = ch->slave_offset;
			desc->dst_width = width;
			break;
		default:
			goto free_desc;
	}

	desc->tx.tx_submit = msc313_tx_submit;

	//if(ch->slave_id == 0x7)
	//	printk("sg %d:%08x -> %d:%08x\n", (int) desc->src_id, desc->src, (int) desc->dst_id, desc->dst);

	return &desc->tx;

free_desc:
	kfree(desc);
	return NULL;
}

static int msc313_bdma_config(struct dma_chan *chan,
		struct dma_slave_config *config)
{
	struct msc313_bdma_chan *ch = to_chan(chan);

	switch(config->direction){
	case DMA_MEM_TO_DEV:
		ch->slave_offset = config->dst_addr;
		break;
	case DMA_DEV_TO_MEM:
		ch->slave_offset = config->src_addr;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
	.fast_io = true,
};

static inline void msc313_bdma_moveinflighttocomplete(struct msc313_bdma_chan *chan,
		bool success, size_t residue)
{
	struct msc313_bdma_desc *inflight = chan->inflight;
	struct msc313_bdma *bdma = chan->bdma;
	unsigned long flags;

	BUG_ON(!inflight);
	chan->inflight = NULL;

	/* update status */
	inflight->result.result = success ? DMA_TRANS_NOERROR : DMA_TRANS_ABORTED;
	inflight->result.residue = residue;

	/* Move inflight descriptor to complete list */
	list_move_tail(&inflight->queue_node, &chan->completed);

	spin_lock_irqsave(&bdma->lock, flags);
	bdma->runningchannels--;
	BUG_ON(bdma->runningchannels < 0);
	spin_unlock_irqrestore(&bdma->lock, flags);
}

static irqreturn_t msc313_bdma_irq(int irq, void *data)
{
	struct msc313_bdma_chan *chan = data;
	unsigned int irqflag, done, result0;
	unsigned long flags;

	regmap_field_read(chan->irq, &irqflag);
	if (!irqflag)
		return IRQ_NONE;

	regmap_field_read(chan->done, &done);
	regmap_field_read(chan->result0, &result0);

	/* clear flags */
	regmap_field_write(chan->busy, 1);
	regmap_field_write(chan->irq, 1);
	regmap_field_write(chan->done, 1);
	regmap_field_write(chan->result0, 1);

	spin_lock_irqsave(&chan->lock, flags);

	/* call off the watchdog */
	del_timer(&chan->watchdog);
	msc313_bdma_moveinflighttocomplete(chan, true, 0);

	spin_unlock_irqrestore(&chan->lock, flags);

	/* Kick tasklet to finish up completed descriptors */
	tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

static irqreturn_t msc313_bdma_irq_single(int irq, void *data)
{
	struct msc313_bdma *bdma = data;
	int i, ret = IRQ_NONE;

	for (i = 0; i < bdma->info->channels; i++) {
		struct msc313_bdma_chan *chan = &bdma->chans[i];

		if (msc313_bdma_irq(irq, chan) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	return ret;
}

static struct dma_chan *msc313_bdma_of_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	struct dma_device *dev = ofdma->of_dma_data;
	struct dma_chan *chan, *candidate = NULL;
	struct msc313_bdma_chan *bdmachan;

	if (!dev || dma_spec->args_count != 3)
		return NULL;

	list_for_each_entry(chan, &dev->channels, device_node)
		if (chan->chan_id == dma_spec->args[0]) {
			candidate = chan;
			break;
		}

	if (!candidate)
		return NULL;

	bdmachan = to_chan(chan);
	bdmachan->slave_id = dma_spec->args[1];
	bdmachan->slave_width = dma_spec->args[2];

	return dma_get_slave_channel(candidate);
}

static void msc313_bdma_watchdog(struct timer_list *t)
{
	struct msc313_bdma_chan *chan = from_timer(chan, t, watchdog);
	struct msc313_bdma_desc *desc = chan->inflight;
	unsigned int queued, busy, irq, done;
	unsigned int srcid, dstid, residue, src, dst;
	struct device *dev = chan->chan.device->dev;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	regmap_field_read(chan->queued, &queued);
	regmap_field_read(chan->busy, &busy);
	regmap_field_read(chan->irq, &irq);
	regmap_field_read(chan->done, &done);
	regmap_field_read(chan->src, &srcid);
	regmap_field_read(chan->dst, &dstid);
	src = msc313_bdma_read_low_high_pair(chan->src_addr_l, chan->src_addr_h);
	dst = msc313_bdma_read_low_high_pair(chan->dst_addr_l, chan->dst_addr_h);
	residue = msc313_bdma_read_low_high_pair(chan->size_l, chan->size_h);

	/* stop the channel, unused in the vendor code :/ */
	regmap_field_force_write(chan->stop, 1);
	regmap_field_force_write(chan->stop, 0);

	dev_warn(dev, "timeout waiting for completion irq, queued: %d, busy %d, irq: %d, done: %d\n"
			"src 0x%x (id 0x%x), dst 0x%x (id 0x%x), residue 0x%x (from 0x%x)\n",
			queued, busy, irq, done,
			src, srcid, dst, dstid,
			residue, desc->len);

	msc313_bdma_moveinflighttocomplete(chan, false, residue);

	spin_unlock_irqrestore(&chan->lock, flags);

	tasklet_schedule(&chan->tasklet);
}

static int msc313_bdma_suspend(struct device *dev)
{
	struct msc313_bdma *bdma = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < bdma->info->channels; i++)
		regmap_field_write(bdma->chans[i].int_en, 0);

	clk_disable_unprepare(bdma->clk);

	return 0;
}

static int msc313_bdma_resume(struct device *dev)
{
	struct msc313_bdma *bdma = dev_get_drvdata(dev);
	int i, ret;

	ret = clk_prepare_enable(bdma->clk);
	if (ret)
		return ret;

	for (i = 0; i < bdma->info->channels; i++)
		regmap_field_write(bdma->chans[i].int_en, 1);

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(msc313_bdma_pm_ops,
			    msc313_bdma_suspend,
			    msc313_bdma_resume,
			    NULL);

static int msc313_bdma_probe(struct platform_device *pdev)
{
	const struct msc313_bdma_info *match_data;
	struct device *dev = &pdev->dev;
	struct msc313_bdma *bdma;
	struct regmap *regmap;
	void __iomem *base;
	unsigned int irq;
	int i, ret;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	bdma = devm_kzalloc(dev, struct_size(bdma, chans, match_data->channels), GFP_KERNEL);
	if (!bdma)
		return -ENOMEM;

	bdma->info = match_data;
	spin_lock_init(&bdma->lock);

	platform_set_drvdata(pdev, bdma);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	if(IS_ERR(regmap))
		return PTR_ERR(regmap);

	bdma->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(bdma->clk))
		return PTR_ERR(bdma->clk);

	bdma->dma_device.dev = dev;
	bdma->dma_device.device_tx_status = msc313_bdma_tx_status;
	bdma->dma_device.device_issue_pending = msc313_bdma_issue_pending;
	bdma->dma_device.src_addr_widths = BIT(4);
	bdma->dma_device.dst_addr_widths = BIT(4);
	bdma->dma_device.directions = BIT(DMA_MEM_TO_MEM) |
				      BIT(DMA_DEV_TO_MEM) |
				      BIT(DMA_MEM_TO_DEV);
	bdma->dma_device.device_prep_dma_memcpy = msc313_bdma_prep_dma_memcpy;
	bdma->dma_device.device_prep_slave_sg = msc313_bdma_prep_slave_sg;
	bdma->dma_device.device_config = msc313_bdma_config;
	bdma->dma_device.copy_align = 2;

	INIT_LIST_HEAD(&bdma->dma_device.channels);

	dma_cap_set(DMA_MEMCPY, bdma->dma_device.cap_mask);

	for (i = 0; i < match_data->channels; i++) {
		struct msc313_bdma_chan *chan = &bdma->chans[i];
		const unsigned int offset = match_data->channel_size * i;
		const struct reg_field ctrl_trigger = REG_FIELD(REG_CTRL + offset, 0, 0);
		const struct reg_field ctrl_stop = REG_FIELD(REG_CTRL + offset, 4, 4);
		const struct reg_field status_queued = REG_FIELD(REG_STATUS + offset, 0, 0);
		const struct reg_field status_busy = REG_FIELD(REG_STATUS + offset, 1, 1);
		const struct reg_field status_int = REG_FIELD(REG_STATUS + offset, 2, 2);
		const struct reg_field status_done = REG_FIELD(REG_STATUS + offset, 3, 3);
		const struct reg_field status_result0 = REG_FIELD(REG_STATUS + offset, 4, 4);
		const struct reg_field config_src = REG_FIELD(REG_CONFIG + offset, 0, 3);
		const struct reg_field config_src_width = REG_FIELD(REG_CONFIG + offset, 4, 6);
		const struct reg_field misc_int_en = REG_FIELD(REG_MISC + offset, 1, 1);
		const struct reg_field miu_sel_ch0 = REG_FIELD(REG_MISC + offset, 12, 13);
		const struct reg_field config_dst = REG_FIELD(REG_CONFIG + offset, 8, 11);
		const struct reg_field config_dst_width = REG_FIELD(REG_CONFIG + offset, 12, 14);

		const struct reg_field src_addr_l = REG_FIELD(REG_SRC_ADDR_L + offset, 0, 15);
		const struct reg_field src_addr_h = REG_FIELD(REG_SRC_ADDR_H + offset, 0, 15);
		const struct reg_field dst_addr_l = REG_FIELD(REG_DST_ADDR_L + offset, 0, 15);
		const struct reg_field dst_addr_h = REG_FIELD(REG_DST_ADDR_H + offset, 0, 15);
		const struct reg_field size_l = REG_FIELD(REG_SIZE_L + offset, 0, 15);
		const struct reg_field size_h = REG_FIELD(REG_SIZE_H + offset, 0, 15);

		chan->bdma = bdma;
		spin_lock_init(&chan->lock);
		INIT_LIST_HEAD(&chan->queue);
		INIT_LIST_HEAD(&chan->completed);

		chan->irq = devm_regmap_field_alloc(dev, regmap, status_int);
		if(IS_ERR(chan->irq))
			return PTR_ERR(chan->irq);

		chan->queued = devm_regmap_field_alloc(dev, regmap, status_queued);
		if(IS_ERR(chan->queued))
			return PTR_ERR(chan->queued);

		chan->busy = devm_regmap_field_alloc(dev, regmap, status_busy);
		if(IS_ERR(chan->busy))
			return PTR_ERR(chan->busy);

		chan->done = devm_regmap_field_alloc(dev, regmap, status_done);
		if(IS_ERR(chan->done))
			return PTR_ERR(chan->done);

		chan->result0 = devm_regmap_field_alloc(dev, regmap, status_result0);
		if(IS_ERR(chan->result0))
			return PTR_ERR(chan->result0);

		chan->trigger = devm_regmap_field_alloc(dev, regmap, ctrl_trigger);
		if(IS_ERR(chan->trigger))
			return PTR_ERR(chan->trigger);

		chan->stop = devm_regmap_field_alloc(dev, regmap, ctrl_stop);
		if(IS_ERR(chan->stop))
			return PTR_ERR(chan->stop);

		chan->int_en = devm_regmap_field_alloc(dev, regmap, misc_int_en);
		if(IS_ERR(chan->int_en))
			return PTR_ERR(chan->int_en);

		chan->src = devm_regmap_field_alloc(dev, regmap, config_src);
		if(IS_ERR(chan->src))
			return PTR_ERR(chan->src);

		chan->src_width = devm_regmap_field_alloc(dev, regmap, config_src_width);
		if(IS_ERR(chan->src_width))
			return PTR_ERR(chan->src_width);

		chan->dst = devm_regmap_field_alloc(dev, regmap, config_dst);
		if(IS_ERR(chan->dst))
			return PTR_ERR(chan->dst);

		chan->dst_width = devm_regmap_field_alloc(dev, regmap, config_dst_width);
		if(IS_ERR(chan->dst_width))
			return PTR_ERR(chan->dst_width);

		chan->src_addr_l = devm_regmap_field_alloc(dev, regmap, src_addr_l);
		if(IS_ERR(chan->src_addr_l))
			return PTR_ERR(chan->src_addr_l);

		chan->src_addr_h = devm_regmap_field_alloc(dev, regmap, src_addr_h);
		if(IS_ERR(chan->src_addr_h))
			return PTR_ERR(chan->src_addr_h);

		chan->dst_addr_l = devm_regmap_field_alloc(dev, regmap, dst_addr_l);
		if(IS_ERR(chan->dst_addr_l))
			return PTR_ERR(chan->dst_addr_l);

		chan->dst_addr_h = devm_regmap_field_alloc(dev, regmap, dst_addr_h);
		if(IS_ERR(chan->dst_addr_h))
			return PTR_ERR(chan->dst_addr_h);

		chan->size_l = devm_regmap_field_alloc(dev, regmap, size_l);
		if(IS_ERR(chan->size_l))
			return PTR_ERR(chan->size_l);

		chan->size_h = devm_regmap_field_alloc(dev, regmap, size_h);
		if(IS_ERR(chan->size_h))
			return PTR_ERR(chan->size_h);

		chan->miu_sel_ch0 = devm_regmap_field_alloc(dev, regmap, miu_sel_ch0);
		if(IS_ERR(chan->miu_sel_ch0))
			return PTR_ERR(chan->miu_sel_ch0);
		/*
		 * For now force MIU0, this fixes SPI flash access as the
		 * boot rom leaves this with IMI selected.
		 */
		regmap_field_write(chan->miu_sel_ch0, 0);

		if(!match_data->single_irq) {
			irq = irq_of_parse_and_map(pdev->dev.of_node, i);
			if (!irq)
				return -EINVAL;
			ret = devm_request_irq(&pdev->dev, irq, msc313_bdma_irq, IRQF_SHARED,
					dev_name(&pdev->dev), chan);
			if(ret)
				return ret;
		}

		chan->chan.device = &bdma->dma_device;

		tasklet_init(&chan->tasklet, msc313_bdma_tasklet, (unsigned long) chan);

		dma_cookie_init(&chan->chan);

		timer_setup(&chan->watchdog, msc313_bdma_watchdog, 0);

		list_add_tail(&chan->chan.device_node, &bdma->dma_device.channels);
	}

	/*
	 * For the single IRQ versions of this get the single irq
	 * and use the special handler to check all of the channels.
	 */
	if(match_data->single_irq) {
		irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
		if (!irq)
			return -EINVAL;
		ret = devm_request_irq(&pdev->dev, irq, msc313_bdma_irq_single, IRQF_SHARED,
			dev_name(&pdev->dev), bdma);
		if(ret)
			return ret;
	}

	ret = dmaenginem_async_device_register(&bdma->dma_device);
	if(ret)
		return ret;

#ifdef CONFIG_PM
	pm_runtime_irq_safe(dev);
	pm_runtime_set_autosuspend_delay(dev, BDMA_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
#else
	ret = msc313_bdma_resume(dev);
	if (ret)
		return ret;
#endif

	return of_dma_controller_register(pdev->dev.of_node,
			msc313_bdma_of_xlate, &bdma->dma_device);
}

static int msc313_bdma_remove(struct platform_device *pdev)
{
	struct msc313_bdma *bdma = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret, i;

	of_dma_controller_free(dev->of_node);

	for (i = 0; i < bdma->info->channels; i++){
		struct msc313_bdma_chan *chan = &bdma->chans[i];
		tasklet_kill(&chan[i].tasklet);
	}

#ifndef CONFIG_PM
	ret = msc313_bdma_suspend(dev);
	if (ret)
		return ret;
#endif

	return 0;
}

static const struct msc313_bdma_info msc313_info = {
	.channels = 2,
	.channel_size = 0x40,
	.single_irq = false,
};

static const struct msc313_bdma_info ssd20xd_info = {
	.channels = 4,
	.channel_size = 0x40,
	.single_irq = false,
};

#ifdef CONFIG_MACH_PIONEER3
static const struct msc313_bdma_info ssd210_info = {
	.channels = 4,
	.channel_size = 0x80,
	.single_irq = true,
};
#endif

static const struct of_device_id msc313_bdma_of_match[] = {
	{
		.compatible = "mstar,msc313-bdma",
		.data = &msc313_info,
	},
	{
		.compatible = "sstar,ssd20xd-bdma",
		.data = &ssd20xd_info,
	},
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible = "sstar,ssd210-bdma",
		.data = &ssd210_info,
	},
#endif
	{},
};
MODULE_DEVICE_TABLE(of, msc313_bdma_of_match);

static struct platform_driver msc313_bdma_driver = {
	.probe = msc313_bdma_probe,
	.remove = msc313_bdma_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_bdma_of_match,
		.pm = &msc313_bdma_pm_ops,
	},
};
module_platform_driver(msc313_bdma_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("MStar MSC313 BDMA driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
