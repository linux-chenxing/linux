// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/clk-provider.h>

#include <dt-bindings/dma/msc313-bdma.h>

#define DRIVER_NAME			"msc313-isp"

#define REG_PASSWORD			0x0
#define VAL_PASSWORD_UNLOCK		0xAAAA
#define VAL_PASSWORD_LOCK		0x5555
#define REG_SPI_WDATA			0x10
#define REG_SPI_RDATA			0x14
#define REG_SPI_CLKDIV			0x18
#define REG_SPI_CLKDIV_WIDTH		11
#define REG_SPI_CECLR			0x20
#define REG_SPI_RDREQ			0x30
#define REG_SPI_RD_DATARDY		0x54
#define BIT_SPI_RD_DATARDY_READY	BIT(0)
#define REG_SPI_WR_DATARDY		0x58
#define BIT_SPI_WR_DATARDY_READY	BIT(0)
#define REG_TRIGGER_MODE		0xa8
#define REG_RST				0xfc
#define VAL_TRIGGER_MODE_ENABLE		0x3333
#define VAL_TRIGGER_MODE_DISABLE	0x2222

//unused
#define REG_SPI_COMMAND			(0x1 * REG_OFFSET)
#define REG_SPI_ADDR_L			(0x2 * REG_OFFSET)
#define REG_SPI_ADDR_H			(0x3 * REG_OFFSET)

//#define DISABLE_BDMA

static const struct regmap_config msc313_isp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static struct reg_field rst_nrst_field = REG_FIELD(REG_RST, 2, 2);
static struct reg_field readreq_req_field = REG_FIELD(REG_SPI_RDREQ, 0, 0);
static struct reg_field rdata_field = REG_FIELD(REG_SPI_RDATA, 0, 7);
static struct reg_field rddatardy_ready_field = REG_FIELD(REG_SPI_RD_DATARDY, 0, 0);
static struct reg_field wdata_field = REG_FIELD(REG_SPI_WDATA, 0, 7);
static struct reg_field wrdatardy_ready_field = REG_FIELD(REG_SPI_WR_DATARDY, 0, 0);
static struct reg_field ceclr_clear_field = REG_FIELD(REG_SPI_CECLR, 0, 0);

#define REG_QSPI_CFG			0x1c0
#define REG_QSPI_READMODE		0x1c8
#define REG_QSPI_FUNCSEL		0x1f4

static struct reg_field addrcontdis_field = REG_FIELD(REG_QSPI_CFG, 9, 9);
static struct reg_field addr2en_field = REG_FIELD(REG_QSPI_FUNCSEL, 11, 11);

struct msc313_isp {
	spinlock_t lock;

	struct device *dev;
	struct spi_master *master;
	/* clock for the pm interface */
	struct clk *pm_spi_clk;
	/* clocks for the isp interface */
	struct clk *cpu_clk;
	struct clk *spi_div_clk;
	/* clocks for the qspi interface */
	struct clk *spi_clk;
	struct regmap *regmap;
	struct regmap *qspi;
	void __iomem *base;

	/* The memory mapped area and how big it is */
	void __iomem *memorymapped;
	resource_size_t	mapped_size;

	struct dma_chan *dmachan;

	struct regmap_field *nrst;
	struct regmap_field *rdreq;
	struct regmap_field *rdata;
	struct regmap_field *rddatardy;
	struct regmap_field *wdata;
	struct regmap_field *wrdatardy;
	struct regmap_field *ceclr;

	struct regmap_field *addr2;
	struct regmap_field *addrcontdis;

	wait_queue_head_t dma_wait;
	bool dma_done;
	bool dma_success;
};

struct msc313_qspi_readmode {
	const u16 opcode;
	const u8 readmode;
};

static const struct msc313_qspi_readmode opcode_mapping[] = {
	{ 0x03, 0x0 },
	{ 0x0b,	0x1 },
	{ 0x3b, 0x2 },
	{ 0xbb, 0x3 },
	{ 0x6b, 0xa },
	{ 0xeb, 0xb },
	{ 0x4eb, 0xd },
};

#define MSC313_ISP_DEBUG

static void msc313_isp_enable(struct msc313_isp *isp)
{
	//regmap_field_force_write(isp->nrst, 0);
	//regmap_field_force_write(isp->nrst, 1);
	writew_relaxed(VAL_PASSWORD_UNLOCK, isp->base + REG_PASSWORD);
	writew_relaxed(VAL_TRIGGER_MODE_ENABLE, isp->base + REG_TRIGGER_MODE);
}

static void msc313_isp_disable(struct msc313_isp *isp)
{
	writew_relaxed(VAL_TRIGGER_MODE_DISABLE, isp->base + REG_TRIGGER_MODE);
	writew_relaxed(VAL_PASSWORD_LOCK, isp->base + REG_PASSWORD);
	//regmap_field_force_write(isp->nrst, 0);
}

static void msc313_isp_spi_writebyte(struct msc313_isp *isp, u8 value)
{
	unsigned int rdyval;

	regmap_field_force_write(isp->wdata, value);
	if(regmap_field_read_poll_timeout(isp->wrdatardy, rdyval,
			rdyval & BIT_SPI_WR_DATARDY_READY, 0, 1000000)){
		dev_err(&isp->master->dev, "write timeout");
	}
}

static void msc313_isp_spi_readbyte(struct msc313_isp *isp, u8 *dest)
{
	unsigned int rdyval;
	unsigned int regval;

	regmap_field_force_write(isp->rdreq, 1);
	if(regmap_field_read_poll_timeout(isp->rddatardy, rdyval,
				rdyval & BIT_SPI_RD_DATARDY_READY, 0, 1000000)){
		dev_err(&isp->master->dev, "read timeout");
	}
	regmap_field_read(isp->rdata, &regval);
	*dest = (u8) regval;
}

static void msc313_isp_spi_clearcs(struct msc313_isp *isp)
{
	regmap_field_force_write(isp->ceclr, 1);
}

static int msc313_isp_setup(struct spi_device *spi)
{
	struct msc313_isp *isp = spi_controller_get_devdata(spi->controller);

	//clk_set_rate(isp->spi_div_clk, spi->max_speed_hz);
	//clk_set_rate(isp->spi_clk, spi->max_speed_hz);

	return 0;
}

static int msc313_isp_transfer_one(struct spi_controller *ctlr, struct spi_device *spi,
			    struct spi_transfer *transfer)
{
	struct msc313_isp *isp = spi_controller_get_devdata(ctlr);
	const u8 *tx_buf = transfer->tx_buf;
	u8 *rx_buf = transfer->rx_buf;
	unsigned b = 0;

	/*
	 * this only really works for SPI NOR <cs low><write something><read something><cs high>
	 * transactions
	 */

	for(b = 0; b < transfer->len; b++) {
		if (tx_buf) {
			msc313_isp_spi_writebyte(isp, *tx_buf++);
			/* we don't do full duplex, there should be no rx buffer */
			if(rx_buf){
			}
		}
		else if(rx_buf){
			msc313_isp_spi_readbyte(isp, rx_buf++);
		}
	}

	return 0;
}

static void msc313_isp_set_cs(struct spi_device *spi, bool enable)
{
	/* cs is asserted by the controller, we can only deassert it */
	struct msc313_isp *isp = spi_master_get_devdata(spi->master);
	if(!enable)
		msc313_isp_spi_clearcs(isp);
}

static const struct msc313_qspi_readmode* msc313_isp_spi_mem_op_to_readmode(const struct spi_mem_op *op)
{
	int i;

	/*
	 * The opcodes that the controller can execute are limited.
	 * Check that the requested opcode is one we can use.
	 */
	for (i = 0; i < ARRAY_SIZE(opcode_mapping); i++){
		if (opcode_mapping[i].opcode == op->cmd.opcode)
			return &opcode_mapping[i];
	}

	return NULL;
}

static bool msc313_isp_spi_mem_supports_op(struct spi_mem *mem,
			    const struct spi_mem_op *op)
{
/* Maybe we should filter commands qspi can't handle here? */
#if 0
	if (!msc313_isp_spi_mem_op_to_readmode(op)){
		return false;
	}
#endif

	return spi_mem_default_supports_op(mem, op);
}

static int msc313_isp_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct msc313_isp *isp = spi_controller_get_devdata(mem->spi->controller);
	int i, ret = 0;

	if(op->cmd.opcode)
		msc313_isp_spi_writebyte(isp, op->cmd.opcode);
	if(op->addr.nbytes != 0)
		for(i = op->addr.nbytes; i > 0; i--)
			msc313_isp_spi_writebyte(isp, (op->addr.val >> (8 * (i - 1))) & 0xff);
	if(op->dummy.nbytes != 0)
		for(i = 0; i < op->dummy.nbytes; i++)
			msc313_isp_spi_writebyte(isp, 0xff);
	switch(op->data.dir){
		case SPI_MEM_DATA_IN:
			for(i = 0; i < op->data.nbytes; i++)
				msc313_isp_spi_readbyte(isp, ((u8*)(op->data.buf.in + i)));
			break;
		case SPI_MEM_DATA_OUT:
			for(i = 0; i < op->data.nbytes; i++)
				msc313_isp_spi_writebyte(isp, *((u8*)(op->data.buf.out + i)));
			break;
		case SPI_MEM_NO_DATA:
			break;
	}

	msc313_isp_spi_clearcs(isp);
	return ret;
}

static int msc313_isp_spi_mem_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	const struct msc313_qspi_readmode *readmode = NULL;
	struct spi_mem_op *tmpl = &desc->info.op_tmpl;

	/*
	 * The QSPI controller only supports reads, the FSP seems
	 * to be for writing.
	 */
	if (tmpl->data.dir != SPI_MEM_DATA_IN)
		return -ENOTSUPP;

	/*
	 * We can only do 2 or 3 address bytes
	 */
	if ((tmpl->addr.nbytes != 2) &&
	    (tmpl->addr.nbytes != 3))
		return -ENOTSUPP;


	readmode = msc313_isp_spi_mem_op_to_readmode(&desc->info.op_tmpl);
	if (!readmode){
		pr_info("Opcode %x isn't supported by QSPI\n",
				(unsigned) desc->info.op_tmpl.cmd.opcode);
		return -ENOTSUPP;
	}

	desc->priv = readmode;

	pr_info("Opcode %x \n",
			(unsigned) readmode->opcode);

	desc->nodirmap = 0;

	return 0;
}

static void msc313_isp_spi_mem_dirmap_destroy(struct spi_mem_dirmap_desc *desc)
{
	// nothing for now
}

static void msc313_isp_dma_callback(void *dma_async_param,
				const struct dmaengine_result *result){
	struct msc313_isp *isp = dma_async_param;

	isp->dma_done = true;
	isp->dma_success = result->result == DMA_TRANS_NOERROR;
	if(!isp->dma_success)
		dev_err(&isp->master->dev, "dma failed: %d\n", result->result);
	wake_up(&isp->dma_wait);
}

static ssize_t msc313_isp_spi_mem_dirmap_read(struct spi_mem_dirmap_desc *desc,
		u64 offs, size_t len, void *buf)
{
	struct msc313_isp *isp = spi_controller_get_devdata(desc->mem->spi->controller);
	const struct msc313_qspi_readmode *readmode = desc->priv;
	struct spi_mem_op *tmpl = &desc->info.op_tmpl;
	struct dma_async_tx_descriptor *dmadesc;
	struct dma_slave_config config;
	dma_addr_t dmaaddr = 0;

	msc313_isp_disable(isp);

	/* Make sure we generate the right number of address bytes */
	if (tmpl->addr.nbytes == 2)
		regmap_field_write(isp->addr2, 1);
	else if (tmpl->addr.nbytes == 3)
		regmap_field_write(isp->addr2, 0);
	else
		return -EINVAL;

	regmap_write(isp->qspi, REG_QSPI_READMODE, readmode->readmode);

	/*
	 * QSPI apparently tracks the last address that was accessed,
	 * holds CS low and just outputs more clocks if the next access is
	 * sequential. This saves sending the command and address out again..
	 *
	 * For NAND we will be issuing commands to load the next page in between
	 * accesses using one of the other SPI controllers in this block so CS
	 * is toggled between accesses from QSPI.
	 *
	 * If we read the data of a page in one access and then the OOB
	 * in another access we end up with the commands to load the page, the command
	 * to read the data, then commands to load the page again but then the QSPI
	 * controller thinks reading the OOB is sequential because it read up to the page
	 * size for the data and now it's reading the OOB size from the address after the
	 * data. This results in loading the data correctly, loading the OOB into the cache
	 * correctly but then trying to read the OOB from the cache without sending
	 * a command.
	 *
	 * Fortunately there seems to be a bit to disable the "address continue"
	 * feature and this resolved the issue. This bit isn't documented anywhere
	 * except a comment in a random header. The vendor linux driver doesn't seem
	 * to use this as it loads the complete data + OOB page from NAND so the
	 * address being accessed after loading the page into cache is always 0.
	 *
	 * Using this feature seems to cause a new read command to be issued for
	 * every 128 bytes in a transfer when reading with quad output (6b).
	 * So 60 extra bytes to read a standard 2048 byte NAND page.
	 *
	 */
	regmap_field_write(isp->addrcontdis, 1);

	if(isp->dmachan){
		dmaaddr = dma_map_single(isp->dev, buf, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(isp->dev, dmaaddr)){
			dmaaddr = 0;
			goto memcpy;
		}

		isp->dma_done = false;
		config.direction = DMA_DEV_TO_MEM;
		config.slave_id = MSC313_BDMA_SLAVE_QSPI;
		config.src_addr = offs;
		config.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
		dmaengine_slave_config(isp->dmachan, &config);
		dmadesc = dmaengine_prep_slave_single(isp->dmachan, dmaaddr, len, DMA_DEV_TO_MEM, 0);
		if(dmadesc){
			dmadesc->callback_result = msc313_isp_dma_callback;
			dmadesc->callback_param = isp;
			dmaengine_submit(dmadesc);
			dma_async_issue_pending(isp->dmachan);
			/*
			 * If the settings are wrong DMA doesn't complete so we
			 * use a timeout so we can inform the user that we are borked.
			 * If this happens the CPU interface is also broken and
			 * the CPU will lock up.
			 */
			if (!wait_event_timeout(isp->dma_wait, isp->dma_done, HZ * 10)) {
				dev_err(&isp->master->dev, "timeout waiting for dma, lock up in coming\n");
				len = -EIO;
				goto dma_exit;
			}
			rmb();
			if(isp->dma_success)
				goto dma_exit;
			else
				dev_warn(&isp->master->dev, "dma failed, falling back to cpu read\n");
		}
	}

	/*
	 * Despite the documentation saying not to do this on the
	 * cpu we don't have a lot of choice if dma isn't working.
	 */
memcpy:
	memcpy(buf, isp->memorymapped + offs, len);
dma_exit:
	if(dmaaddr)
		dma_unmap_single(isp->dev, dmaaddr, len, DMA_FROM_DEVICE);
	msc313_isp_enable(isp);

	return len;
}

static struct spi_controller_mem_ops msc313_isp_mem_ops = {
	.supports_op = msc313_isp_spi_mem_supports_op,
	.exec_op = msc313_isp_exec_op,
	.dirmap_create = msc313_isp_spi_mem_dirmap_create,
	.dirmap_destroy = msc313_isp_spi_mem_dirmap_destroy,
	.dirmap_read = msc313_isp_spi_mem_dirmap_read
};

static const struct clk_div_table div_table[] = {
	{0x1, 2},
	{0x4, 4},
	{0x40, 8},
	{0x80, 16},
	{0x100, 32},
	{0x200, 64},
	{0x400, 128},
	{ 0 },
};

static int msc313_isp_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct msc313_isp *isp;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	struct resource *res;
	u32 max_freq;
	int ret;

	master = spi_alloc_master(dev, sizeof(struct msc313_isp));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	isp = spi_master_get_devdata(master);

	isp->dev = &pdev->dev;
	isp->master = master;

	isp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(isp->base))
		return PTR_ERR(isp->base);

	isp->regmap = devm_regmap_init_mmio(&pdev->dev, isp->base,
			&msc313_isp_regmap_config);
	isp->nrst = devm_regmap_field_alloc(&pdev->dev, isp->regmap, rst_nrst_field);
	isp->rdreq = devm_regmap_field_alloc(&pdev->dev, isp->regmap, readreq_req_field);
	isp->rdata = devm_regmap_field_alloc(&pdev->dev, isp->regmap, rdata_field);
	isp->rddatardy = devm_regmap_field_alloc(&pdev->dev, isp->regmap, rddatardy_ready_field);
	isp->wdata = devm_regmap_field_alloc(&pdev->dev, isp->regmap, wdata_field);
	isp->wrdatardy = devm_regmap_field_alloc(&pdev->dev, isp->regmap, wrdatardy_ready_field);
	isp->ceclr = devm_regmap_field_alloc(&pdev->dev, isp->regmap, ceclr_clear_field);

	base = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(base))
		return PTR_ERR(base);

	isp->qspi = devm_regmap_init_mmio(&pdev->dev, base,
			&msc313_isp_regmap_config);
	isp->addrcontdis = devm_regmap_field_alloc(&pdev->dev, isp->qspi, addrcontdis_field);
	isp->addr2 = devm_regmap_field_alloc(&pdev->dev, isp->qspi, addr2en_field);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	isp->memorymapped = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(isp->memorymapped))
			return PTR_ERR(isp->memorymapped);
	isp->mapped_size = resource_size(res);

	isp->dmachan = dma_request_chan(&pdev->dev, "qspi");
	if(IS_ERR(isp->dmachan)){
		dev_warn(&pdev->dev, "failed to request dma channel: %ld, will use cpu!\n", PTR_ERR(isp->dmachan));
		isp->dmachan = NULL;
	}

	isp->pm_spi_clk = devm_clk_get(&pdev->dev, "pm_spi");
	if (IS_ERR(isp->pm_spi_clk)) {
		return PTR_ERR(isp->pm_spi_clk);
	}

	isp->cpu_clk = devm_clk_get(&pdev->dev, "cpuclk");
	if (IS_ERR(isp->cpu_clk)) {
		return PTR_ERR(isp->cpu_clk);
	}

	isp->spi_clk = devm_clk_get(&pdev->dev, "spi");
	if (IS_ERR(isp->spi_clk)) {
		return PTR_ERR(isp->spi_clk);
	}

	/*
	 * Write a very safe default value into the clock divider,
	 * the value we get coming out of reset doesn't work with most
	 * SPI NAND chips and might not be in our table.
	 */
	writew_relaxed(0x400, isp->base + REG_SPI_CLKDIV);
	isp->spi_div_clk = clk_register_divider_table(dev, "spi_clk", "cpuclksrc", 0,
			isp->base + REG_SPI_CLKDIV, 0, REG_SPI_CLKDIV_WIDTH,
			0, div_table, &isp->lock);
	if(IS_ERR(isp->spi_div_clk))
		return PTR_ERR(isp->spi_div_clk);

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->num_chipselect = 1;
	master->max_speed_hz = clk_round_rate(isp->spi_div_clk, ~0);
	master->min_speed_hz = clk_round_rate(isp->spi_div_clk, 0);
	master->flags = SPI_CONTROLLER_HALF_DUPLEX;
	master->setup = msc313_isp_setup;
	master->transfer_one = msc313_isp_transfer_one;
	master->set_cs = msc313_isp_set_cs;
	/* Dual and Quad IO reads only work via QSPI */
#ifndef DISABLE_BDMA
	master->mode_bits = SPI_CPHA | SPI_CPOL |
			    SPI_RX_DUAL | SPI_RX_QUAD;
	master->mem_ops = &msc313_isp_mem_ops;
#endif
	init_waitqueue_head(&isp->dma_wait);

	ret = clk_prepare_enable(isp->pm_spi_clk);
	ret = clk_prepare_enable(isp->spi_div_clk);

	if (of_property_read_u32(pdev->dev.of_node, "spi-max-frequency", &max_freq) == 0){
		clk_set_rate(isp->pm_spi_clk, max_freq);
		clk_set_rate(isp->spi_div_clk, max_freq);
	}

	msc313_isp_enable(isp);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "spi master registration failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static int msc313_isp_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msc313_isp *isp = spi_master_get_devdata(master);

	if(isp->dmachan)
		dma_release_channel(isp->dmachan);

	return 0;
}

static const struct of_device_id msc313_isp_match[] = {
	{ .compatible = "mstar,msc313-isp", },
	{}
};
MODULE_DEVICE_TABLE(of, msc313_isp_match);

static int __maybe_unused msc313_isp_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct msc313_isp *isp = spi_master_get_devdata(master);

	/*
	 * the boot rom wants everything to be at reset state otherwise it
	 * will lock up..
	 */
	regmap_field_force_write(isp->nrst, 0);
	mdelay(1);
	regmap_field_force_write(isp->nrst, 1);
	mdelay(1);

	/*
	 * reset doesn't clear this, if we don't clear it the boot rom
	 * can't read the IPL
	 */
	writew_relaxed(VAL_PASSWORD_LOCK, isp->base + REG_PASSWORD);
	return 0;
}

static int __maybe_unused msc313_isp_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(msc313_isp_pm_ops, msc313_isp_suspend,
			 msc313_isp_resume);

static struct platform_driver msc313_isp_driver = {
	.probe	= msc313_isp_probe,
	.remove	= msc313_isp_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = msc313_isp_match,
		.pm = &msc313_isp_pm_ops
	},
};
module_platform_driver(msc313_isp_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("MStar MSC313 ISP driver");
MODULE_LICENSE("GPL v2");
