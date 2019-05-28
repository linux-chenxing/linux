// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/pwm.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>

/*
 * MSC313 i2c controller
 *
 * 0x00 : ctrl
 *   2   |   1   |   0
 * enint | endma |  rst
 *
 * 0x04 : start/stop
 *   8  |   0
 * stop | start
 *
 * 0x08 : write data
 *  8	| 7 - 0
 * ack  | data
 *
 * 0x0c : read data
 *   9  |  8   | 7 - 0
 *  ack | trig | data
 *
 * ack and trig have to be written together to trigger a read with the correct
 * ack/nack at the end
 *
 * 0x10 : int ctl
 *
 * 0x14 : status/int status
 *   13   |   12   |   11   |   10   |    9    |    8     |  4 - 0
 * sclerr | clkstr | txdone | rxdone | stopdet | startdet | hw state
 *                                                        | 0 - idle
 *                                                        | 1 or 2 - start
 *                                                        | 3,4,5 or 6 - write
 *                                                        | 7,8,9,10 - read
 *                                                        | 11 - interrupt
 *                                                        | 12 - wait
 *                                                        | 13,14,15 - stop
 *
 * The below registers are apparently for the number of clocks per state
 * 0x20: stp_cnt (stop clock count?)
 * 0x24: ckh_cnt (clock high count?)
 * 0x28: ckl_cnt (clock low count?)
 * 0x2c: sda_cnt
 * 0x30: stt_cnt
 * 0x34: lth_cnt
 * 0x38: tmt_cnt
 *
 * 0x3c: scli delay?
 *
 * todo: each controller has it's own DMA controller
 */

#define DRIVER_NAME "msc313e-i2c"

#define REG_CTRL		0x00
#define REG_STARTSTOP		0x04
#define REG_WDATA		0x08
#define REG_RDATA		0x0c
#define REG_INT_CTRL 		0x10
#define REG_INT_STAT		0x14
#define REG_STOP_CNT		0x20
#define REG_CKH_CNT		0x24
#define REG_CKL_CNT		0x28
#define REG_SDA_CNT		0x2c
#define REG_START_CNT		0x30
#define REG_DATA_LATCH_CNT	0x34
#define REG_DMA_CFG		0x80
#define REG_DMA_ADDRL		0x84
#define REG_DMA_ADDRH		0x88
#define REG_DMA_CTL		0x8c
#define REG_DMA_TXR		0x90
#define REG_DMA_CMDDAT0_1	0x94
#define REG_DMA_CMDDAT2_3	0x98
#define REG_DMA_CMDDAT4_5	0x9c
#define REG_DMA_CMDDAT6_7	0xa0
#define REG_DMA_CMDLEN		0xa4
#define REG_DMA_DATALEN		0xa8
#define REG_DMA_TCL		0xb0
#define REG_DMA_TCH		0xb4
#define REG_DMA_SLAVECFG	0xb8
#define REG_DMA_TRIGGER		0xbc
#define REG_DMA_STATE		0xc4

static const struct reg_field ctrl_rst_field = REG_FIELD(REG_CTRL, 0, 0);
static const struct reg_field ctrl_endma_field = REG_FIELD(REG_CTRL, 1, 1);
static const struct reg_field ctrl_enint_field = REG_FIELD(REG_CTRL, 2, 2);
/* This causes a flag to be set for clock stretching each time the hardware is triggered.. */
static const struct reg_field ctrl_enclkstr_field = REG_FIELD(REG_CTRL, 3, 3);
static const struct reg_field startstop_start_field = REG_FIELD(REG_STARTSTOP, 0, 0);
static const struct reg_field startstop_stop_field = REG_FIELD(REG_STARTSTOP, 8, 8);

static const struct reg_field wdata_data_field = REG_FIELD(REG_WDATA, 0, 7);
static const struct reg_field wdata_nack_field = REG_FIELD(REG_WDATA, 8, 8);

static const struct reg_field rdata_data_field = REG_FIELD(REG_RDATA, 0, 7);
static const struct reg_field rdata_readack_field = REG_FIELD(REG_RDATA, 9, 9);
static const struct reg_field rdata_readtrig_field = REG_FIELD(REG_RDATA, 8, 8);

static const struct reg_field status_state_field = REG_FIELD(REG_INT_STAT, 0, 4);
static const struct reg_field status_int_field = REG_FIELD(REG_INT_STAT, 8, 13);
#define INT_STAT_IDLE		0
/*
 * The two det flags seem to be for
 * when we are a slave but start_det
 * is set when we send our start condition.
 * Stop is not set though. But if that's
 * the case how do we go into slave
 * mode?
 */
#define INT_STAT_START_DET	BIT(0)
#define INT_STAT_STOP_DET	BIT(1)
#define INT_STAT_RX_DONE	BIT(2)
#define INT_STAT_TX_DONE	BIT(3)
#define INT_STAT_CLKSTR		BIT(4)
#define INT_STAT_SCL_ERR	BIT(5)
#define INT_STAT_TIMEOUT	BIT(6)

/* timing */
static const struct reg_field stopcnt_field = REG_FIELD(REG_STOP_CNT, 0, 15);
static const struct reg_field ckhcnt_field = REG_FIELD(REG_CKH_CNT, 0, 15);
static const struct reg_field cklcnt_field = REG_FIELD(REG_CKL_CNT, 0, 15);
static const struct reg_field sdacnt_field = REG_FIELD(REG_SDA_CNT, 0, 15);
static const struct reg_field startcnt_field = REG_FIELD(REG_START_CNT, 0, 15);
static const struct reg_field datalatchcnt_field = REG_FIELD(REG_DATA_LATCH_CNT, 0, 15);

/* dma */
static const struct reg_field dma_reset_field = REG_FIELD(REG_DMA_CFG, 1, 1);
static const struct reg_field dma_inten_field = REG_FIELD(REG_DMA_CFG, 2, 2);
static const struct reg_field dma_miurst_field = REG_FIELD(REG_DMA_CFG, 3, 3);
static const struct reg_field dma_txrdone_field = REG_FIELD(REG_DMA_TXR, 0, 0);
static const struct reg_field dma_addrl_field = REG_FIELD(REG_DMA_ADDRL, 0, 15);
static const struct reg_field dma_addrh_field = REG_FIELD(REG_DMA_ADDRH, 0, 15);

static const struct reg_field dma_read_field = REG_FIELD(REG_DMA_CTL, 6, 6);
static const struct reg_field dma_stopdis_field = REG_FIELD(REG_DMA_CTL, 5, 5);
static const struct reg_field dma_command_data_field[] = {
	REG_FIELD(REG_DMA_CMDDAT0_1, 0, 15),
	REG_FIELD(REG_DMA_CMDDAT2_3, 0, 15),
	REG_FIELD(REG_DMA_CMDDAT4_5, 0, 15),
	REG_FIELD(REG_DMA_CMDDAT6_7, 0, 15),
};
static const struct reg_field dma_commandlen_field = REG_FIELD(REG_DMA_CMDLEN, 0, 3);
static const struct reg_field dma_datalen_field = REG_FIELD(REG_DMA_DATALEN, 0, 15);
static const struct reg_field dma_tcl_field = REG_FIELD(REG_DMA_TCL, 0, 15);
static const struct reg_field dma_tch_field = REG_FIELD(REG_DMA_TCH, 0, 15);
static const struct reg_field dma_slaveaddr_field = REG_FIELD(REG_DMA_SLAVECFG, 0, 9);
static const struct reg_field dma_10biten_field = REG_FIELD(REG_DMA_SLAVECFG, 10, 10);
static const struct reg_field dma_trig_field = REG_FIELD(REG_DMA_TRIGGER, 0, 0);
static const struct reg_field dma_retrig_field = REG_FIELD(REG_DMA_TRIGGER, 8, 8);
static const struct reg_field dma_state_field = REG_FIELD(REG_DMA_STATE, 0, 7);
static const struct reg_field dma_miulastdonez_field = REG_FIELD(REG_DMA_STATE, 8, 8);

struct msc313e_i2c {
	struct device *dev;
	struct i2c_adapter i2c;
	struct clk_hw sclk;
	struct clk* clk;
	struct regmap *regmap;

	struct i2c_timings timings;

	/* config */
	struct regmap_field *rst;
	struct regmap_field *endma;
	struct regmap_field *enint;
	struct regmap_field *start;

	/* rx data fields */
	struct regmap_field *rdata;
	struct regmap_field *read_trig;
	struct regmap_field *read_ack;

	/* tx data fields */
	struct regmap_field *wdata;
	struct regmap_field *write_nack;

	struct regmap_field *stop;
	struct regmap_field *state;

	/* timing */
	struct regmap_field *stop_count;
	struct regmap_field *clkhcount;
	struct regmap_field *clklcount;
	struct regmap_field *sda_count;
	struct regmap_field *start_count;
	struct regmap_field *data_latch_count;

	/* dma */
	struct regmap_field *dma_reset;
	struct regmap_field *dma_inten;
	struct regmap_field *dma_miurst;
	struct regmap_field *dma_addrl;
	struct regmap_field *dma_addrh;
	struct regmap_field *dma_read;
	struct regmap_field *dma_stopdis;
	/* ack a dma complete irq */
	struct regmap_field *dma_txr_done;
	//struct regmap_field *dma_command_data;
	struct regmap_field *dma_command_len;
	struct regmap_field *dma_data_len;
	struct regmap_field *dma_tcl;
	struct regmap_field *dma_tch;
	struct regmap_field *dma_slave_addr;
	struct regmap_field *dma_10bit_en;
	struct regmap_field *dma_trigger;
	struct regmap_field *dma_retrigger;
	struct regmap_field *dma_state;

	/* irq stuff */
	wait_queue_head_t wait;
	struct regmap_field *intstat;

	spinlock_t lock;
	unsigned int int_status;
	bool waiting;
	bool done;
};

static const struct regmap_config msc313e_i2c_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

static irqreturn_t msc313_i2c_irq(int irq, void *data)
{
	struct msc313e_i2c *bus = data;
	unsigned int_status;
	unsigned long flags;

	regmap_field_read(bus->intstat, &int_status);
	regmap_write(bus->regmap, REG_INT_CTRL, 1);
	regmap_field_force_write(bus->dma_txr_done, 1);

	spin_lock_irqsave(&bus->lock, flags);
	if (!bus->waiting)
		printk("irq came when not waiting for one!\n");

	bus->int_status = int_status;
	bus->waiting = false;
	bus->done = true;
	wake_up(&bus->wait);
	spin_unlock_irqrestore(&bus->lock, flags);

	return IRQ_HANDLED;
}

static void msc313_i2c_printhwstate(struct msc313e_i2c *i2c)
{
	unsigned int pio_state, dma_state, dma_tcl = 0xaa, dma_tch = 0x55, dma_txr_done;

	regmap_field_read(i2c->state, &pio_state);
	regmap_field_read(i2c->dma_state, &dma_state);
	regmap_field_read(i2c->dma_tcl, &dma_tcl);
	regmap_field_read(i2c->dma_tch, &dma_tch);
	dev_info(&i2c->i2c.dev, "pio state %d, dma state %d, dma tc %08x",
			(int) pio_state, (int) dma_state, (int)(dma_tcl | (dma_tch << 16)));

	regmap_field_read(i2c->dma_txr_done, &dma_txr_done);
	dev_info(&i2c->i2c.dev, "dma done %d\n", dma_txr_done);
}

static int msc313e_i2c_waitforidle(struct msc313e_i2c *i2c, unsigned wantedint)
{
	wait_event_timeout(i2c->wait, i2c->done, HZ * 2);

	if (!i2c->done) {
		unsigned int_status;

		regmap_field_read(i2c->intstat, &int_status);
		dev_err(&i2c->i2c.dev, "timeout waiting for hardware to become idle\n");
		msc313_i2c_printhwstate(i2c);

		regmap_field_read(i2c->intstat, &int_status);
		dev_err(&i2c->i2c.dev, "int stat: %x", int_status);
		/* do a reset */
		regmap_field_force_write(i2c->rst, 1);
		mdelay(10);
		regmap_field_force_write(i2c->rst, 0);
		mdelay(10);

		return -ETIMEDOUT;
	}

	if (wantedint && !(i2c->int_status & wantedint)) {
		dev_err(&i2c->i2c.dev, "unexpected interrupt status, wanted %x, got %x\n",
				       wantedint, i2c->int_status);
		return -ENXIO;
	}

	udelay(10);

	return 0;
}

static int msc313_i2c_xfer_dma(struct msc313e_i2c *i2c, struct i2c_msg *msg, u8 *dma_buf, bool last)
{
	bool read = msg->flags & I2C_M_RD;
	unsigned long flags;
	enum dma_data_direction dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t dma_addr;
	int ret = 0;

	//printk("dma i2c addr: %02x, read: %d, len: %d, %px\n", msg->addr, read, msg->len, dma_buf);

	dma_addr = dma_map_single(i2c->dev, dma_buf, msg->len, dir);
	if (dma_mapping_error(i2c->dev, dma_addr)) {
		return -ENOMEM;
	};

	/* controller setup */
	regmap_field_write(i2c->endma, 1);

	/* slave address setup */
	regmap_field_write(i2c->dma_read, read ? 1 : 0);
	regmap_field_write(i2c->dma_stopdis, last ? 0 : 1);
	regmap_field_write(i2c->dma_slave_addr, msg->addr);
	regmap_field_write(i2c->dma_10bit_en, (msg->flags & I2C_M_TEN) ? 1 : 0);

	/* transfer setup */
	regmap_field_write(i2c->dma_addrl, dma_addr);
	regmap_field_write(i2c->dma_addrh, dma_addr >> 16);
	regmap_field_write(i2c->dma_command_len, 0);
	regmap_field_write(i2c->dma_data_len, msg->len);

	/* trigger and wait */
	spin_lock_irqsave(&i2c->lock, flags);
	i2c->waiting = true;
	i2c->done = false;
	regmap_field_force_write(i2c->dma_txr_done, 1);
	regmap_field_force_write(i2c->dma_trigger, 1);
	spin_unlock_irqrestore(&i2c->lock, flags);

	if (msc313e_i2c_waitforidle(i2c, 0)) { //fix int!
		dev_err(&i2c->i2c.dev, "dma transfer timedout\n");
		ret = -ETIMEDOUT;
	}

	dma_unmap_single(i2c->dev, dma_addr, msg->len, dir);

	i2c_put_dma_safe_msg_buf(dma_buf, msg, true);

	return ret;
}

static int msc313_i2c_rxbyte(struct msc313e_i2c *i2c, bool last)
{
	unsigned long flags;
	unsigned int data;
	int ret;

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->waiting = true;
	i2c->done = false;
	regmap_field_write(i2c->read_ack, (last ? 1 : 0));
	regmap_field_force_write(i2c->read_trig, 1);
	spin_unlock_irqrestore(&i2c->lock, flags);

	ret = msc313e_i2c_waitforidle(i2c, INT_STAT_RX_DONE);
	if (ret) {
		dev_err(&i2c->i2c.dev, "Failed to rx byte\n");
		return ret;
	}
	regmap_field_read(i2c->rdata, &data);


	return data;
}

static int msc313_i2c_txbyte(struct msc313e_i2c *i2c, u8 byte)
{
	unsigned long flags;
	unsigned int nack;
	int ret;

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->waiting = true;
	i2c->done = false;
	regmap_field_force_write(i2c->wdata, byte);
	spin_unlock_irqrestore(&i2c->lock, flags);

	ret = msc313e_i2c_waitforidle(i2c, INT_STAT_TX_DONE);
	if (ret) {
		dev_err(&i2c->i2c.dev, "Failed to tx byte\n");
		return ret;
	}

	regmap_field_read(i2c->write_nack, &nack);
	return nack ? -ENXIO : 0;
}


static void msc313_i2c_reset(struct msc313e_i2c *i2c) {
	regmap_field_force_write(i2c->rst, 1);
	regmap_field_force_write(i2c->rst, 0);
}

static int msc313_i2c_xfer_pio(struct msc313e_i2c *i2c, struct i2c_msg *msg, bool first, bool last)
{
	bool read = msg->flags & I2C_M_RD;
	unsigned long flags;
	int i, ret = 0;

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->waiting = true;
	i2c->done = false;

	/*
	 * Seems weird but to send a repeated start we need to do
	 * a reset. Otherwise scl stays low and a proper start
	 * condition is not generated.
	 */
	if (!first)
		msc313_i2c_reset(i2c);

	regmap_field_force_write(i2c->start, 1);
	spin_unlock_irqrestore(&i2c->lock, flags);
	/*
	 * We get the  flag here on
	 * the first start and the nothing on repeated
	 * starts.
	 */
	ret = msc313e_i2c_waitforidle(i2c, INT_STAT_START_DET);
	if (ret) {
		dev_err(&i2c->i2c.dev, "Failed to trigger start\n");
		goto error;
	}

	ret = msc313_i2c_txbyte(i2c, (msg->addr << 1) | (read ? 1 : 0));
	if (ret) {
		/* Making this an error makes i2cdetect very noisy */
		dev_dbg(&i2c->i2c.dev, "Failed to send address\n");
		goto error;
	}
	for (i = 0; i < msg->len; i++) {
		if (read){
			int b = msc313_i2c_rxbyte(i2c, i + 1 == msg->len);
			if(b < 0) {
				ret = b;
				goto error;
			}

			*(msg->buf + i) = b;
		}
		else {
			ret = msc313_i2c_txbyte(i2c, *(msg->buf + i));
			if (ret)
				goto error;
		}
	}

error:
	if (last || ret) {
		spin_lock_irqsave(&i2c->lock, flags);
		i2c->waiting = true;
		i2c->done = false;
		regmap_field_force_write(i2c->stop, 1);
		spin_unlock_irqrestore(&i2c->lock, flags);
		if (msc313e_i2c_waitforidle(i2c, INT_STAT_IDLE))
			dev_err(&i2c->i2c.dev, "Failed to trigger stop\n");
	}
	return ret;
}

//#define DISABLE_DMA

static int msc313_i2c_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[],
			   int num)
{
	struct msc313e_i2c *bus = i2c_get_adapdata(i2c_adap);
	int i, txed = 0, ret = 0;

	ret = pm_runtime_get_sync(bus->dev);
	if (ret < 0) {
		dev_err(bus->dev, "runtime resume failed %d\n", ret);
		pm_runtime_put_noidle(bus->dev);
		return ret;
	}

	regmap_field_write(bus->endma, 0);

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool first = i == 0;
		bool last = i + 1 == num;
#ifndef DISABLE_DMA
		u8 *dma_buf = i2c_get_dma_safe_msg_buf(msg, 8);

		if(dma_buf)
			ret = msc313_i2c_xfer_dma(bus, msg, dma_buf, last);
		else
#endif
			ret = msc313_i2c_xfer_pio(bus, msg, first, last);

		if(ret)
			goto abort;

		txed++;

		udelay(20);
	}

	ret = txed;

abort:
	pm_runtime_put(bus->dev);

	return ret;
}

static u32 msc313_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm msc313_i2c_algo = {
	.master_xfer = msc313_i2c_xfer,
	.functionality = msc313_i2c_func,
};

static unsigned long msc313e_i2c_sclk_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct msc313e_i2c *i2c = container_of(hw, struct msc313e_i2c, sclk);
	unsigned count_high, count_low;

	regmap_field_read(i2c->clkhcount, &count_high);
	regmap_field_read(i2c->clklcount, &count_low);

	return parent_rate / (count_high + count_low);
}

static int msc313_sclk_determine_rate(struct clk_hw *hw,
		struct clk_rate_request *req)
{
	struct msc313e_i2c *i2c = container_of(hw, struct msc313e_i2c, sclk);

	return 0;
}

static int msc313_sclk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct msc313e_i2c *i2c = container_of(hw, struct msc313e_i2c, sclk);
	unsigned divider = parent_rate / rate;

	//printk("i2c set rate %d\n", rate);

	regmap_field_write(i2c->clkhcount, divider/2);
	regmap_field_write(i2c->clklcount, divider/2);

	return 0;
}

static const struct clk_ops sclk_ops = {
	.recalc_rate = msc313e_i2c_sclk_recalc_rate,
	.determine_rate = msc313_sclk_determine_rate,
	.set_rate = msc313_sclk_set_rate,
};

static const struct clk_parent_data sclk_parent = {
	.index	= 0,
};

static int msc313e_i2c_probe(struct platform_device *pdev)
{
	struct clk_init_data sclk_init = {
		.ops = &sclk_ops,
		.parent_data = &sclk_parent,
		.num_parents = 1,
	};
	struct device *dev = &pdev->dev;
	struct msc313e_i2c *msc313ei2c;
	struct i2c_adapter *adap;
	void* __iomem base;
	int irq, ret;

	msc313ei2c = devm_kzalloc(&pdev->dev, sizeof(*msc313ei2c), GFP_KERNEL);
	if (!msc313ei2c)
		return -ENOMEM;

	spin_lock_init(&msc313ei2c->lock);

	msc313ei2c->dev = dev;
	init_waitqueue_head(&msc313ei2c->wait);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	msc313ei2c->regmap = devm_regmap_init_mmio(&pdev->dev, base,
			&msc313e_i2c_regmap_config);
	if(IS_ERR(msc313ei2c->regmap))
		return PTR_ERR(msc313ei2c->regmap);

	i2c_parse_fw_timings(&pdev->dev, &msc313ei2c->timings, true);

	/* config */
	msc313ei2c->rst = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, ctrl_rst_field);
	msc313ei2c->endma = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, ctrl_endma_field);
	msc313ei2c->enint = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, ctrl_enint_field);
	msc313ei2c->start = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, startstop_start_field);
	msc313ei2c->stop = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, startstop_stop_field);

	msc313ei2c->rdata = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, rdata_data_field);
	msc313ei2c->read_trig = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, rdata_readtrig_field);
	msc313ei2c->read_ack = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, rdata_readack_field);

	msc313ei2c->write_nack = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, wdata_nack_field);
	msc313ei2c->wdata = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, wdata_data_field);

	msc313ei2c->state = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, status_state_field);
	msc313ei2c->intstat = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, status_int_field);

	/* timing */
	msc313ei2c->stop_count = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, stopcnt_field);
	msc313ei2c->clkhcount = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, ckhcnt_field);
	msc313ei2c->clklcount = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, cklcnt_field);
	msc313ei2c->sda_count = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, sdacnt_field);
	msc313ei2c->start_count = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, startcnt_field);
	msc313ei2c->data_latch_count = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, datalatchcnt_field);

	/* dma */
	msc313ei2c->dma_reset =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_reset_field);
	msc313ei2c->dma_inten =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_inten_field);
	msc313ei2c->dma_miurst =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_miurst_field);

	msc313ei2c->dma_addrl =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_addrl_field);
	msc313ei2c->dma_addrh =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_addrh_field);
	msc313ei2c->dma_read =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_read_field);
	msc313ei2c->dma_stopdis =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_stopdis_field);
	msc313ei2c->dma_txr_done =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_txrdone_field);
	//devm_regmap_field_bulk_alloc(dev, msc313ei2c->regmap,
	//		&msc313ei2c->dma_command_data, dma_command_data_field, ARRAY_SIZE(dma_command_data_field));
	msc313ei2c->dma_command_len = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_commandlen_field);
	msc313ei2c->dma_data_len = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_datalen_field);
	msc313ei2c->dma_tcl = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_tcl_field);
	msc313ei2c->dma_tch = devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_tch_field);
	msc313ei2c->dma_slave_addr =  devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_slaveaddr_field);
	msc313ei2c->dma_10bit_en =  devm_regmap_field_alloc(&pdev->dev, msc313ei2c->regmap, dma_10biten_field);
	msc313ei2c->dma_trigger =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_trig_field);
	msc313ei2c->dma_retrigger =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_retrig_field);
	msc313ei2c->dma_state =  devm_regmap_field_alloc(dev, msc313ei2c->regmap, dma_state_field);

	regmap_field_write(msc313ei2c->enint, 1);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, irq, msc313_i2c_irq, IRQF_SHARED,
			dev_name(&pdev->dev), msc313ei2c);

	sclk_init.name = devm_kasprintf(dev, GFP_KERNEL, "%s_sclk", dev_name(dev));
	msc313ei2c->sclk.init = &sclk_init;
	ret = devm_clk_hw_register(dev, &msc313ei2c->sclk);
	if (ret)
		return ret;

	msc313ei2c->clk = devm_clk_hw_get_clk(dev, &msc313ei2c->sclk, "what?!");

	platform_set_drvdata(pdev, msc313ei2c);

	pm_runtime_enable(dev);

	adap = &msc313ei2c->i2c;
	i2c_set_adapdata(adap, msc313ei2c);
	snprintf(adap->name, sizeof(adap->name), dev_name(&pdev->dev));
	adap->owner = THIS_MODULE;
	adap->timeout = 2 * HZ;
	adap->retries = 0;
	adap->algo = &msc313_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	return i2c_add_adapter(adap);
}

static int msc313e_i2c_remove(struct platform_device *pdev)
{
	struct msc313e_i2c *bus = platform_get_drvdata(pdev);

	i2c_del_adapter(&bus->i2c);
	pm_runtime_force_suspend(bus->dev);

	return 0;
}

static const struct of_device_id msc313e_i2c_dt_ids[] = {
	{ .compatible = "mstar,msc313e-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, msc313e_i2c_dt_ids);

static int __maybe_unused msc313_i2c_runtime_suspend(struct device *dev)
{
	struct msc313e_i2c *i2c = dev_get_drvdata(dev);

	regmap_field_force_write(i2c->dma_miurst, 1);
	regmap_field_force_write(i2c->dma_reset, 1);
	regmap_field_force_write(i2c->rst, 1);
	mdelay(10);

	clk_disable_unprepare(i2c->clk);

	return 0;
}

static int __maybe_unused msc313_i2c_runtime_resume(struct device *dev)
{
	struct msc313e_i2c *i2c = dev_get_drvdata(dev);
	long clk_rate;

	clk_rate = clk_round_rate(i2c->clk, i2c->timings.bus_freq_hz);

	//printk("i2c rounded rated %d\n", (int) clk_rate);

	// Timing from vendor driver, probably based on 12mhz clock
	regmap_field_write(i2c->start_count, 38);
	regmap_field_write(i2c->stop_count, 38);
	regmap_field_write(i2c->sda_count, 5);
	regmap_field_write(i2c->data_latch_count, 5);

	clk_set_rate(i2c->clk, i2c->timings.bus_freq_hz);
	clk_prepare_enable(i2c->clk);

	regmap_field_force_write(i2c->rst, 0);
	mdelay(10);
	regmap_field_force_write(i2c->dma_reset, 0);
	regmap_field_force_write(i2c->dma_miurst, 0);
	mdelay(10);

	regmap_field_write(i2c->dma_inten, 1);

	return 0;
}

static const struct dev_pm_ops msc313_i2c_pm = {
	SET_RUNTIME_PM_OPS(msc313_i2c_runtime_suspend, msc313_i2c_runtime_resume, NULL)
};

static struct platform_driver msc313e_i2c_driver = {
	.probe = msc313e_i2c_probe,
	.remove = msc313e_i2c_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313e_i2c_dt_ids,
		.pm = &msc313_i2c_pm,
	},
};
module_platform_driver(msc313e_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mstar MSC313E i2c driver");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
