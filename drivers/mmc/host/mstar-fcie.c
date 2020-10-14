/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/regulator/consumer.h>

#include "mstar-fcie.h"

#define DRIVER_NAME "msc313-fcie"

/*
 * MSC313 FCIE - flash card interface engine?
 * There seem to be multiple generations of this controller called
 * FCIEv<n> and then a reduced version called "sdio". The full version
 * seems to also be able to work with raw nand.. honestly the vendor
 * driver for this thing is a mess so it's hard to work out.
 *
 * The MSC313(e) version of this controller is apparently v5. This
 * driver will be for that version
 *
 * 0x00 - interrupt/event status
 *      7        |     6       |      5      |       4      |     3    |    2    |    1    |     0
 * card 2 change | card change | r2n rdy int | busy end int | sdio int | err sts | cmd end | data end
 *
 * 0x04 - interrupt mask
 * - same order as above
 *
 * 0x08 - pri
 *
 * 0x0c - dma address, low word
 * 0x10 - dma address, high word
 * 0x14 - dma length, low word
 * 0x18 - dma length, high word
 *
 * 0x1c - function control
 *     2     |   1    |  0
 * sdio mode | sd en  | emmc
 *
 * 0x20 - job blk cnt
 * 0x24 - blk size
 * 0x28 - cmd rsp size
 *    15 - 8 | 7 - 0
 *  cmd size | rsp size
 *
 * 0x2c - SD mode
 *
 *      2      |     1       |    0
 * bus width 8 | bus width 4 | clk en
 *
 * 0x30 - sd ctl, seems to be for triggering transfers
 *      9     |      8      |    7    |     6     |    5    |    4      |    3    |   2     |   1    |    0
 * err_det_on | busy_det_on | chk_cmd | job_start | adma_en | job dir   | dtrx_en | cmd_en  | rsp_en | rspr2_en
 *            |             |         |           |         | 0 - read  |         |         |        |
 *            |             |         |           |         | 1 - write |         |         |        |
 *
 * rsp_en and rspr2_en control the type of response, everything but no response enables rsp_en, 136 bit responses
 * enable rspr2_en
 *
 * 0x34 - sd sts
 *
 *  15   |   14  |   13  |   12  |   11  |   10  |   9   |   8
 * dat 7 | dat 6 | dat 5 | dat 4 | dat 3 | dat 2 | dat 1 | dat 0
 *
 *    7     |      6     |      5      |      4      |      3     |       2      |       1      |      0
 *    ?     |  card_busy | dat_rd_tout | cmdrsp_cerr |  cmd_norsp |  dat_wr_tout |  dat_wr_cerr | dat_rd_cerr
 *
 * 0x3c - ddr mod - default value is 0x6400
 *
 *     15      |     14     |     13     |       12       |      11       |     10      | 9 |    8
 * pad in mask | fall latch | pad in sel | ddr macro32 en | pad in sel ip | pad clk sel | ? | ddr en
 *       7      | 6 | 5 | 4 |        3       |        2       |       1        |      0
 * ddr macro en | ? | ? | ? | pre full sel 1 | pre full sel 0 | pad in rdy sel | pad in bypass
 *
 * 0x44 - sdio mod
 *
 * 0x54 - test mod
 *
 * 0x80 - cmd/rsp fifo
 *  ..
 * 0xa0?
 *
 * These are listed but not used AFAIK
 * 0xc0 - cifd event
 * 0xc4 - cifd int
 * 0xe4 - cifd "debug"?
 *
 * 0xfc - fcie rst
 *      4     |      3     |      2     |      1     |   0
 * ecc status | mcu status | mie status | miu status | sw rst
 */

//#define SUPERDEBUG

#define FCIE_CMD_TIMEOUT_NS (NSEC_PER_SEC/10)

struct msc313_fcie {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk;
	bool use_polling;

	/* io control */
	struct regmap_field *clk_en;
	struct regmap_field *bus_width;

	/* transfer control */
	struct regmap_field *blk_sz;
	struct regmap_field *blk_cnt;
	struct regmap_field *rspr2_en;
	struct regmap_field *rsp_en;
	struct regmap_field *cmd_en;
	struct regmap_field *dtrf_en;
	struct regmap_field *jobdir;
	struct regmap_field *adma_en;
	struct regmap_field *busydet_en;
	struct regmap_field *errdet_en;
	struct regmap_field *cmd_sz;
	struct regmap_field *rsp_sz;
	struct regmap_field *job_start;

	/* status */
	struct regmap_field *status;
	struct regmap_field *card_busy;
	struct regmap_field *d0;

	/* reset */
	struct regmap_field *nrst;
	struct regmap_field *rst_status;

	/* misc */
	struct regmap_field *func_ctrl;

	wait_queue_head_t wait;
	bool error;
	bool cmd_done;
	bool busy_done;
	bool data_done;

	struct msc313_sdio_adma_desc descs[64];
};

static const struct of_device_id msc313_fcie_dt_ids[] = {
	{ .compatible = "mstar,msc313-sdio" },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_fcie_dt_ids);

static const struct regmap_config msc313_fcie_regmap_config = {
	.name = "msc313-fcie",
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static void msc313_fcie_parse_int_flags(struct msc313_fcie *fcie, unsigned int flags)
{
	if (flags & INT_CMD_END) {
		if(fcie->cmd_done)
			dev_err(fcie->dev, "spurious cmd end interrupt\n");
		fcie->cmd_done = true;
		flags &= ~INT_CMD_END;
	}
	if (flags & INT_DATA_END) {
		if(fcie->data_done)
			dev_err(fcie->dev, "spurious data end interrupt\n");
		fcie->data_done = true;
		flags &= ~INT_DATA_END;
	}
	if (flags & INT_BUSY_END) {
		if(fcie->busy_done)
			dev_err(fcie->dev, "spurious busy end interrupt\n");
		fcie->busy_done = true;
		flags &= ~INT_BUSY_END;
	}
	if (flags & INT_ERR) {
		if(fcie->error)
			dev_err(fcie->dev, "spurious error interrupt\n");
		//printk("err int!\n");
		fcie->error = true;
		flags &= ~INT_ERR;
	}

	if(flags)
		dev_warn(fcie->dev, "left over interrupt flags: %02x\n", flags);
}

static unsigned int msc313_fcie_parse_and_clear_int_flags(struct msc313_fcie *fcie)
{
	unsigned int flags;

	regmap_read(fcie->regmap, REG_INT, &flags);
	regmap_write(fcie->regmap, REG_INT, ~0);

#ifdef SUPERDEBUG
	printk("int: %x\n", flags);
#endif

	msc313_fcie_parse_int_flags(fcie, flags);

	return flags;
}

static irqreturn_t msc313_fcie_irq(int irq, void *data)
{
	struct msc313_fcie *fcie = data;

	msc313_fcie_parse_and_clear_int_flags(fcie);

	if (fcie->cmd_done ||
	    fcie->data_done ||
	    fcie->busy_done ||
	    fcie->error)
		wake_up(&fcie->wait);

	return IRQ_HANDLED;
}

static void mstar_fcie_writecmd(struct msc313_fcie *fcie, u8 cmd, u32 arg)
{
	int i;
	u8 fifo[6] = {0};
	u8 *bytes = fifo;
	u16 *words = (u16*) fifo;

	*bytes++ = cmd | 0x40 ;

	arg = cpu_to_be32(arg);
	memcpy(bytes, &arg, sizeof(arg));

	for (i = 0; i < ARRAY_SIZE(fifo) / 2; i++) {
		regmap_write(fcie->regmap, REG_FIFO + (i * 4), words[i]);
	}
}

static int mstar_fcie_readrsp(struct msc313_fcie *fcie, u8 cmd, u32* rsp, int len, bool hasopcode)
{
	u8 *buf = (u8*) rsp;
	unsigned int value;
	int i, j;

	int words = (len % 2 ? len + 1 : len) / 2;

	for (i = 0; i < words; i++) {
		regmap_read(fcie->regmap, REG_FIFO + (i * 4), &value);
#ifdef SUPERDEBUG
		printk("fifo <- %04x\n", value);
#endif
		for (j = 0; j < 2 && ((i * 2) + j) < len; j++) {
			if (i == 0 && j == 0) {
				/*
				 * if the first byte is the opcode
				 * check that it matches the expected opcode
				 * as the fifo content could be stale.
				 * This was added because sometimes the
				 * error interrupt was not firing.
				 */
				if(hasopcode && (value & 0xff) != cmd)
					return EILSEQ;

				/* always strip the first byte. */
				continue;
			}
			*buf++ = (value >> (8 * j));
		}
	}

	rsp[0] = be32_to_cpu(rsp[0]);
	rsp[1] = be32_to_cpu(rsp[1]);
	rsp[2] = be32_to_cpu(rsp[2]);
	rsp[3] = be32_to_cpu(rsp[3]);

	return 0;
}

static bool mstar_fcie_parse_and_check_flags(struct msc313_fcie *fcie, unsigned int flags,
						bool cmd, bool data, bool busy)
{
	bool ret = true;

	msc313_fcie_parse_int_flags(fcie, flags);
	if (cmd)
		ret &= fcie->cmd_done;
	if (data)
		ret &= fcie->data_done;
	if (busy)
		ret &= fcie->busy_done;

	return ret;
}

static int mstar_fcie_start_transfer_and_wait(struct msc313_fcie *fcie,
		bool cmd, bool data, bool busy, unsigned int timeout_ns,
		unsigned int* status)
{
	unsigned long timeout_jiffies = nsecs_to_jiffies(timeout_ns);
	unsigned int job_start, intflags, ctrl, blkcnt,
			blksz, cmdrspsz, poll_timeout;

	/* clear the flags and start the transfer */
	regmap_field_force_write(fcie->status, ~0);
	fcie->error = false;
	fcie->cmd_done = false;
	fcie->data_done = false;
	fcie->busy_done = false;

	/* enable interrupts */
	regmap_write(fcie->regmap, REG_INTMASK, data ? INT_DATA_END : 0 |
						cmd ? INT_CMD_END : 0   |
						busy ? INT_BUSY_END : 0 |
						INT_ERR);

	regmap_field_read(fcie->job_start, &job_start);
	if (job_start)
		dev_warn(fcie->dev, "job start was 1 before triggering!\n");

	//printk("%s:%d, t:%d\n", __func__, __LINE__, timeout_ns);

	regmap_field_force_write(fcie->job_start, 1);

	if (fcie->use_polling) {
		/*
		 * we have to wait some time before polling the flags otherwise
		 * the controller starts corrupting memory, probably because the
		 * flags we have are old
		 */
		mdelay(100);

		poll_timeout = regmap_read_poll_timeout(fcie->regmap, REG_INT, intflags,
				mstar_fcie_parse_and_check_flags(fcie,intflags,cmd,data,busy), HZ/10, HZ * 10);
		regmap_write(fcie->regmap, REG_INT, ~0);
		if (poll_timeout) {
			dev_warn(fcie->dev, "timeout while polling\n");
			return 1;
		}
	}
	else {
		/* wait until the interrupt fires and sets cmd_done */
		if (cmd && !fcie->cmd_done) {
			if(wait_event_timeout(fcie->wait, fcie->cmd_done || fcie->error,
					timeout_jiffies) == 0)
				goto irq_timeout;
		}

		/* wait until the interrupt fires and sets data_done */
		if (data && !fcie->data_done) {
			if(wait_event_timeout(fcie->wait, fcie->data_done || fcie->error,
					timeout_jiffies) == 0)
				goto irq_timeout;
		}

		/* wait until the interrupt fires and sets busy_done */
		if (busy && !fcie->busy_done) {
			if(wait_event_timeout(fcie->wait, fcie->busy_done || fcie->error,
					timeout_jiffies) == 0)
				goto irq_timeout;
		}

	}

	regmap_field_read(fcie->status, status);

	/* If an error occurred sometimes it's useful to dump out some registers */
	if (fcie->error) {
		regmap_read(fcie->regmap, REG_SD_CTL, &ctrl);
		regmap_read(fcie->regmap, REG_BLOCK_COUNT, &blkcnt);
		regmap_read(fcie->regmap, REG_BLOCK_SIZE, &blksz);
		regmap_read(fcie->regmap, REG_CMDRSP_SIZE, &cmdrspsz);

		dev_info(fcie->dev, "err during job; status: %04x, ctrl: %04x, "
				"blksz: %04x, blkcnt: %04x, cmdrspsz: %04x\n",
				*status, ctrl, blksz, blkcnt, cmdrspsz);
		dev_info(fcie->dev, "err during job; cmd %d (%d), data %d (%d), busy %d (%d)\n",
				fcie->cmd_done, cmd, fcie->data_done, data, fcie->busy_done, busy);
	}

	/* disable interrupts */
	regmap_write(fcie->regmap, REG_INTMASK, 0);

	/*
	 * When the card is ejected we get an error interrupt but no status
	 * bits. So if we have an error but no status bits report that there
	 * was a timeout. If there are status bits we need to check if it's
	 * a false CRC error etc. Only timeouts are handled here.
	 */
	if (fcie->error && *status == 0)
		return 1;

	return 0;

irq_timeout:
	/* disable interrupts */
	regmap_write(fcie->regmap, REG_INTMASK, 0);
//printk("%s:%d\n", __func__, __LINE__);
	intflags = msc313_fcie_parse_and_clear_int_flags(fcie);
	regmap_field_read(fcie->status, status);
	regmap_read(fcie->regmap, REG_SD_CTL, &ctrl);
	regmap_read(fcie->regmap, REG_BLOCK_COUNT, &blkcnt);
	regmap_read(fcie->regmap, REG_BLOCK_SIZE, &blksz);
	regmap_read(fcie->regmap, REG_CMDRSP_SIZE, &cmdrspsz);

	dev_warn(fcie->dev, "timeout waiting for interrupt, timeout: %d, int: %04x, "
			"status: %04x, ctrl: %04x, blksz: %04x, blkcnt: %04x, cmdrspsz: %04x\n",
		timeout_ns, intflags, *status, ctrl, blksz, blkcnt, cmdrspsz);
	if ((cmd && !fcie->cmd_done) || (data && !fcie->data_done) || (busy && !fcie->busy_done)) {
		dev_err(fcie->dev, "timedout and no status flags were set");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mstar_fcie_request_setupcmd(struct msc313_fcie *fcie, struct mmc_command *cmd)
{
	int rspsz = 0;

	/* clear any existing flags */
	regmap_write(fcie->regmap, REG_SD_CTL, 0);

	/* load the command into the fifo */
	mstar_fcie_writecmd(fcie, cmd->opcode, cmd->arg);

	/* configure the response length */
	// not needed?
	regmap_field_write(fcie->rsp_en, 0);
	// not needed?
	regmap_field_write(fcie->rspr2_en, 0);
	if (cmd->flags & MMC_RSP_PRESENT) {
		regmap_field_write(fcie->rsp_en, 1);
		if(cmd->flags & MMC_RSP_136){
			regmap_field_write(fcie->rspr2_en, 1);
			rspsz = 16;
		}
		else
			rspsz = 5;
	}

	regmap_field_write(fcie->busydet_en, cmd->flags & MMC_RSP_BUSY ? 1 : 0);
	regmap_field_write(fcie->errdet_en, cmd->flags & MMC_RSP_CRC ? 1 : 0);
	regmap_field_write(fcie->cmd_en, 1);
	/* this is always the case right? */
	regmap_field_write(fcie->cmd_sz, 0x5);
	regmap_field_write(fcie->rsp_sz, rspsz);

	return rspsz;
}

/*
 * Check for errors and if everything is good and there is a response
 * read it out. Sets cmd->err.
 */
static int mstar_fcie_request_capturecmdresult(struct msc313_fcie *fcie,
		struct mmc_command *cmd, unsigned int status, int rspsz)
{
	/*
	 * There is a no response status flag but I don't think I've
	 * ever seen this get set. If the card is removed we get an
	 * error interrupt and zero in the status.
	 */
	if (status & SD_STS_NORSP) {
		dev_err(fcie->dev, "no response from card, removed?\n");
		cmd->error = -EIO;
		return cmd->error;
	}

	if (status & SD_STS_CMDRSPCRCERR) {
		/*
		 * The vendor driver suggests that the CRC flag
		 * is broken for R3 and R4 responses. But I think
		 * this is for anything without a CRC.
		 */
		if(cmd->flags & MMC_RSP_CRC){
			cmd->error = -EILSEQ;
			return cmd->error;
		}
		status &= ~SD_STS_CMDRSPCRCERR;
	}

	/* card busy isn't really an error .. */
	if (status & SD_STS_CARDBUSY) {
		return 0;
	}

	/*
	 * I guess eventually we'll find situations where the other
	 * bits are set. warn about unhandled bits here so someone's
	 * dmesg copy/paste is useful.
	 */
	if (status)
		dev_warn(fcie->dev, "unhandled status bits: %x\n", status);

	if (rspsz > 0) {
		cmd->error = mstar_fcie_readrsp(fcie, cmd->opcode,
				cmd->resp, rspsz, cmd->flags & MMC_RSP_OPCODE);
		if(cmd->error)
			return cmd->error;
	}

	return 0;
}

/* Prepare a command, transmit it and set cmd->error if there was an issue */
static int mstar_fcie_request_prepcmd_and_tx(struct msc313_fcie *fcie, struct mmc_command *cmd)
{
	unsigned int timeout = cmd->busy_timeout ? cmd->busy_timeout * 1000000 : FCIE_CMD_TIMEOUT_NS;
	int rspsz = mstar_fcie_request_setupcmd(fcie, cmd);
	unsigned int status;

	cmd->error = mstar_fcie_start_transfer_and_wait(fcie, true, false,
			cmd->flags & MMC_RSP_BUSY, timeout, &status);
	if (cmd->error)
		return cmd->error;

	mstar_fcie_request_capturecmdresult(fcie, cmd, status, rspsz);

	return cmd->error;
}

static void msc313_fcie_build_adma(struct msc313_fcie *fcie, struct scatterlist *data_sg,
		int count, int blksz, u32 *tfxlen)
{
	struct scatterlist *sg;
	int i;
	for_each_sg(data_sg, sg, count, i) {
		struct msc313_sdio_adma_desc *desc = &fcie->descs[i];
		desc->dmaaddr = sg_dma_address(sg);
		desc->dmalen = sg_dma_len(sg);
		desc->ctrl = FIELD_PREP(ADMA_DESC_CTRL_END, i + 1 == count) |
			     FIELD_PREP(ADMA_DESC_MIU_SEL, 0) |
			     FIELD_PREP(ADMA_DESC_JOB_CNT, desc->dmalen / blksz);
		dev_dbg(fcie->dev, "desc %d:%d: ctrl: 0x%08x, dmaaddr: 0x%08x, dmalen: 0x%08x",
				i, count, desc->ctrl, desc->dmaaddr, desc->dmalen);
		*tfxlen += desc->dmalen;
	}
}
static void mstar_fcie_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msc313_fcie *fcie = mmc_priv(mmc);
	int rspsz, i, count, dir_data, blks, ret;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_data *data = mrq->data;
	bool dataread, sbcdone, useadma, busydet;
	unsigned int status, cardbusy;
	u32 dmaaddr, dmalen, tfrlen;

	/* If there is just a command, send it and return */
	if (data == NULL) {
		if(mstar_fcie_request_prepcmd_and_tx(fcie, cmd)) {
			dev_err(fcie->dev, "failed to send command; cmd: %d arg: 0x%08x\n",
					cmd->opcode, cmd->arg);
			goto tfr_err;
		}
		mmc_request_done(mmc, mrq);
		return;
	}

	/* There is data, but read or write is not set.. */
	if (!(data->flags & (MMC_DATA_READ | MMC_DATA_WRITE))) {
		dev_err(fcie->dev, "don't know what to do with this data, flags 0x%08x\n", data->flags);
		goto tfr_err;
	}

	/* If we have a set-block-count command send it now */
	if (sbc) {
		if(mstar_fcie_request_prepcmd_and_tx(fcie, sbc)) {
			dev_err(fcie->dev, "failed to sbc; cmd: 0x%02x arg: 0x%08x\n",
					sbc->opcode, sbc->arg);
			goto tfr_err;
		}
		sbcdone = true;
	}

	/*
	 * If there is data to read the cmd goes with the first block
	 * of data coming in if there is no data or we're writing
	 * run the command on it's own.
	 *
	 * It's possible we don't actually need to do this for writes
	 * but I haven't got it to work any other way.
	 */
	dataread = data->flags & MMC_DATA_READ;
	if (!dataread) {
		if(mstar_fcie_request_prepcmd_and_tx(fcie, cmd)) {
			dev_err(fcie->dev, "failed to send command; cmd: 0x%02x arg: 0x%08x\n",
					cmd->opcode, cmd->arg);
			goto tfr_err;
		}
	}

	/* If we're doing a read setup the command for the first block */
	if (dataread)
		rspsz = mstar_fcie_request_setupcmd(fcie, mrq->cmd);
	else
		regmap_write(fcie->regmap, REG_SD_CTL, 0);

	dir_data = dataread ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	count = dma_map_sg(fcie->dev, data->sg, data->sg_len, dir_data);
	if (count == 0)
		goto drv_err;

	/* if there is more than one sg use ADMA */
	useadma = count > 1;
	if (useadma) {
		//printk("using ADMA\n");
		msc313_fcie_build_adma(fcie, data->sg, count, data->blksz, &tfrlen);
		dmaaddr = dma_map_single(fcie->dev, &fcie->descs, sizeof(fcie->descs), DMA_TO_DEVICE);
		ret = dma_mapping_error(fcie->dev, dmaaddr);
		if (ret) {
			printk("dma map fail\n");
			data->error = ret;
			goto tfr_err;
		}
		dmalen = 0x10;
		blks = 1;
		regmap_field_write(fcie->adma_en, 1);
	}
	/* otherwise use direct dma */
	else {
		//printk("Using single dma\n");
		dmaaddr = sg_dma_address(data->sg);
		dmalen = sg_dma_len(data->sg);
		blks = dmalen / data->blksz;
		tfrlen = dmalen;
	}

	/* Setup this transfer */
	regmap_field_write(fcie->jobdir, dataread ? 0 : 1);
	regmap_field_write(fcie->dtrf_en, 1);
	regmap_field_write(fcie->blk_sz, data->blksz);
	regmap_write(fcie->regmap, REG_DMA_ADDR_H, dmaaddr >> 16);
	regmap_write(fcie->regmap, REG_DMA_ADDR_L, dmaaddr & 0xffff);
	regmap_write(fcie->regmap, REG_DMA_LEN_H, dmalen >> 16);
	regmap_write(fcie->regmap, REG_DMA_LEN_L, dmalen & 0xffff);
	regmap_field_write(fcie->blk_cnt, blks);

#ifdef SUPERDEBUG
	printk("%d - 0x%08x %s 0x%08x\n", cmd->opcode, dmaaddr,
			dataread ? "<-" : "->", dmalen);
#endif

	busydet = dataread && (cmd->flags & MMC_RSP_BUSY);
	ret = mstar_fcie_start_transfer_and_wait(fcie, dataread,
			true, busydet, data->timeout_ns, &status);
	if (ret) {
		data->error = ret;
		dev_err(fcie->dev, "data %s error; cmd: 0x%02x arg: 0x%08x, blk_sz: %d, blk_cnt %d .. %d:%d\n",
				dataread ? "read" : "write", cmd->opcode, cmd->arg, data->blksz, blks, i, count);
		goto tfr_err;
	}
	/*
	 * the first block will have also triggered sending the cmd
	 * if this was a read so capture the rsp etc for that here
	 * and clear the cmd flags for the next block
	 */
	if (dataread) {
		ret = mstar_fcie_request_capturecmdresult(fcie, mrq->cmd, status, rspsz);
		if (ret && ret != -EBUSY)
			goto tfr_err;
	}

	{
		unsigned int cardbusy;
		regmap_field_read_poll_timeout(fcie->d0, cardbusy, cardbusy, 0, 1000);
	}

	/* check for errors */
	if (status & SD_STS_DATRDCERR) {
		dev_err(fcie->dev, "data read CRC error\n");
		data->error = -EILSEQ;
	}

	if (status & SD_STS_DATWRCERR) {
		dev_err(fcie->dev, "data write CRC error\n");
		data->error = -EILSEQ;

	}

	data->bytes_xfered += tfrlen;

	/*
	 * If sbc wasn't sent then send the stop command here.
	 * The card doesn't respond to this if sbc was sent.
	 *
	 * We probably also need to do this if there was an error during the transfer.
	 */
	if (!sbcdone && data->stop) {
		struct mmc_command *stop = data->stop;
		int ret = mstar_fcie_request_prepcmd_and_tx(fcie, stop);

		if (ret)
			dev_err(fcie->dev, "data stop command timeout; cmd: 0x%02x arg: 0x%08x, flags: 0x%08x\n",
				stop->opcode, stop->arg, stop->flags);
	}

	if (useadma)
		dma_unmap_single(fcie->dev, dmaaddr, sizeof(fcie->descs), DMA_TO_DEVICE);
	dma_unmap_sg(fcie->dev, data->sg, data->sg_len, dir_data);

done:
	mmc_request_done(mmc, mrq);
	return;

drv_err:
	mrq->cmd->error = -EINVAL;
tfr_err:
	if (mrq->stop)
		mstar_fcie_request_prepcmd_and_tx(fcie, mrq->stop);
	mmc_request_done(mmc, mrq);
}

static void mstar_fcie_card_power(struct mmc_host *mmc,
				 struct mmc_ios *ios)
{
	int ret;

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		dev_dbg(mmc_dev(mmc), "Powering card up\n");

		if (!IS_ERR(mmc->supply.vmmc)) {
			ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
			if (ret)
				return;
		}

		if (!IS_ERR(mmc->supply.vqmmc)) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"failed to enable vqmmc\n");
				return;
			}
		}
		break;

	case MMC_POWER_OFF:
		dev_dbg(mmc_dev(mmc), "Powering card off\n");

		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		if (!IS_ERR(mmc->supply.vqmmc))
			regulator_disable(mmc->supply.vqmmc);
		break;

	default:
		dev_dbg(mmc_dev(mmc), "Ignoring unknown card power state\n");
		break;
	}
}

static void mstar_fcie_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msc313_fcie *fcie = mmc_priv(mmc);
	unsigned int bw;

	mstar_fcie_card_power(mmc, ios);

	/* setup the bus width */
	switch(ios->bus_width){
		case MMC_BUS_WIDTH_1:
			bw = 0;
			break;
		case MMC_BUS_WIDTH_4:
			bw = 1;
			break;
		case MMC_BUS_WIDTH_8:
			bw = 2;
			break;
		default:
			return;
	}
	regmap_field_write(fcie->bus_width, bw);

	/* setup the clock */
	regmap_field_write(fcie->clk_en, 0);

	if (ios->clock) {
		long roundedclk = clk_round_rate(fcie->clk, ios->clock);
		if(roundedclk < 0)
			dev_dbg(fcie->dev, "error rounding clock to %u: %ld, leaving clock alone\n", ios->clock, roundedclk);
		else {
			clk_set_rate(fcie->clk, roundedclk);
			dev_dbg(fcie->dev, "requested clock rate %u became %ld\n", ios->clock, roundedclk);
		}
		regmap_field_write(fcie->clk_en, 1);
	}
}

static void mstar_fcie_hw_reset(struct mmc_host *host)
{
	struct msc313_fcie *fcie = mmc_priv(host);
	unsigned int value;

	// not sure if this is really needed but the vendor driver says
	// "clear for safe".
	regmap_write(fcie->regmap, REG_SD_CTL, 0);

	regmap_field_force_write(fcie->nrst, 0);
	// there are 4 documented rst status bits but the vendor driver only checks
	// the first three
	regmap_field_read_poll_timeout(fcie->rst_status, value, value == 0x7,
			10000, 100000);
	regmap_field_force_write(fcie->nrst, 1);
	regmap_field_read_poll_timeout(fcie->rst_status, value, value == 0, 10000,
			100000);
}

static struct mmc_host_ops mstar_fcie_ops = {
	.request	= mstar_fcie_request,
	.set_ios	= mstar_fcie_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.get_ro		= mmc_gpio_get_ro,
	.card_hw_reset	= mstar_fcie_hw_reset,
};

static int msc313_fcie_probe(struct platform_device *pdev)
{
	struct msc313_fcie *fcie;
	struct mmc_host *mmc;
	__iomem void *base;
	int irq, ret = 0;

	mmc = mmc_alloc_host(sizeof(*fcie), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, mmc);

	ret = mmc_regulator_get_supply(mmc);
	if(ret)
		return ret;

	mmc->ops = &mstar_fcie_ops;

	fcie = mmc_priv(mmc);
	init_waitqueue_head(&fcie->wait);

	ret = mmc_of_parse(mmc);
	if(ret)
		return ret;

	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_33_34;

	fcie->dev = &pdev->dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	fcie->regmap = devm_regmap_init_mmio(&pdev->dev, base, &msc313_fcie_regmap_config);
	if(IS_ERR(fcie->regmap))
		return PTR_ERR(fcie->regmap);

	fcie->clk_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, sd_mode_clken_field);
	fcie->bus_width = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, sd_mode_buswidth_field);

	fcie->blk_cnt = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, blockcount_field);
	fcie->blk_sz = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, blocksize_field);

	fcie->rspr2_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_rspr2en_field);
	fcie->rsp_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_rspen_field);
	fcie->adma_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_admaen_field);
	fcie->dtrf_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_dtrfen_field);
	fcie->jobdir = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_jobdir_field);
	fcie->cmd_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_cmden_field);
	fcie->busydet_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_busydeten_field);
	fcie->errdet_en = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_errdeten_field);

	fcie->cmd_sz = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, cmd_rsp_size_cmdsz_field);
	fcie->rsp_sz = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, cmd_rsp_size_rspsz_field);
	fcie->job_start = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, st_ctl_jobstart_field);

	fcie->status = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, sd_sts_status_field);
	fcie->card_busy = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, sd_sts_cardbusy_field);
	fcie->d0 = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, sd_sts_d0_field);

	fcie->nrst = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, rst_nrst_field);
	fcie->rst_status = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, rst_status_field);

	fcie->func_ctrl = devm_regmap_field_alloc(&pdev->dev, fcie->regmap, func_ctrl_field);
	regmap_field_write(fcie->func_ctrl, FUNC_CTRL_SDIO);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq){
		dev_warn(fcie->dev, "no interrupt provided, will use polling");
		fcie->use_polling = true;
	}
	else {
		ret = devm_request_irq(&pdev->dev, irq, msc313_fcie_irq, IRQF_SHARED,
			dev_name(&pdev->dev), fcie);
		if (ret)
			return ret;
	}

	fcie->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(fcie->clk)) {
			return PTR_ERR(fcie->clk);
	}
	
	ret = clk_prepare_enable(fcie->clk);
	if (ret)
		return ret;

	mmc->caps |= MMC_CAP_CMD23;

	mmc->f_min = clk_round_rate(fcie->clk, 400000);
	if (mmc->f_min < 0)
		return ((int) mmc->f_min);

	mmc->f_max = clk_round_rate(fcie->clk, ~0);
	if (mmc->f_max < 0)
		return ((int) mmc->f_max);

	mmc->max_blk_count = 128;
	mmc->max_blk_size = 512;
	mmc->max_segs = ARRAY_SIZE(fcie->descs);
	mmc->max_req_size = mmc->max_blk_count * mmc->max_blk_size;

	ret = mmc_add_host(mmc);

	return ret;
}

static int msc313_fcie_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);

	mmc_remove_host(mmc);
	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver msc313_fcie_driver = {
	.probe = msc313_fcie_probe,
	.remove = msc313_fcie_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = msc313_fcie_dt_ids,
	},
};
module_platform_driver(msc313_fcie_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mstar MSC313 FCIE driver");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
