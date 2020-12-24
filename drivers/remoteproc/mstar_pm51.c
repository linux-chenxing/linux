// SPDX-License-Identifier: GPL-2.0-only
/*
 * MStar PM51
 *
 * Copyright (C) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/syscon.h>
#include <linux/ihex.h>
#include <soc/mstar/pmsleep.h>

#include <dt-bindings/dma/msc313-bdma.h>

#include "remoteproc_internal.h"

/*
 * Start/end addr 1 is bits 23 to 16.
 * Start/end addr 0 is bits 15 to 0.
 * Apparently bits 7 to zero are "don't care"
 */
#define MCU_SRAMA_START_ADDR_1		0x0
#define MCU_SRAMA_END_ADDR_1		0x4
#define MCU_SRAMA_START_ADDR_0		0x8
#define MCU_SRAMA_END_ADDR_0		0xc
#define MCU_MEMMAP			0x30
#define MCU_P0_OV_REG			0x80
#define MCU_P1_OV_REG			0x84
#define MCU_P2_OV_REG			0x88
#define MCU_P3_OV_REG			0x8c
#define MCU_P0_CNTRL			0x90
#define MCU_P0_OE			0x94
#define MCU_P0_IN			0x98
#define MCU_P1_CNTRL			0x9c
#define MCU_P1_OE			0xa0
#define MCU_P1_IN			0xa4
#define MCU_P2_CNTRL			0xa8
#define MCU_P2_OE			0xac
#define MCU_P2_IN			0xb0
#define MCU_P3_CNTRL			0xb4
#define MCU_P3_OE			0xb8
#define MCU_P3_IN			0xbc
#define MCU_PC				0x1f8

static const struct reg_field mcu_memmap_sram_en = REG_FIELD(MCU_MEMMAP, 0, 0);
static const struct reg_field mcu_memmap_spi_en = REG_FIELD(MCU_MEMMAP, 1, 1);
static const struct reg_field mcu_memmap_dram_en = REG_FIELD(MCU_MEMMAP, 2, 2);
static const struct reg_field mcu_memmap_icache_rstz = REG_FIELD(MCU_MEMMAP, 3, 3);

static const struct reg_field pmsleep_8051_rst = REG_FIELD(MSTARV7_PMSLEEP_RSTCNTRL,
		MSTARV7_PMSLEEP_RSTCNTRL_CPUX_SW_RSTZ_8051, MSTARV7_PMSLEEP_RSTCNTRL_CPUX_SW_RSTZ_8051);

struct mstar_pm51_rproc {
	struct rproc *rproc;
	struct platform_device *pdev;
	struct clk *clk;
	struct regmap *mcu;
	struct regmap *pmsleep;
	struct regmap_field *rst;
	struct regmap_field *sram_en;
	struct regmap_field *spi_en;
	struct regmap_field *dram_en;
	struct regmap_field *icache_rstz;
	bool dma_done, dma_success;
	wait_queue_head_t dma_wait;
};

static void mstar_pm51_set_offset_sram(struct mstar_pm51_rproc *pm51, u32 start, u32 end)
{
	regmap_write(pm51->mcu, MCU_SRAMA_START_ADDR_0, start);
	regmap_write(pm51->mcu, MCU_SRAMA_START_ADDR_1, (start >> 16));
	regmap_write(pm51->mcu, MCU_SRAMA_END_ADDR_0, end);
	regmap_write(pm51->mcu, MCU_SRAMA_END_ADDR_1, (end >> 16));
}

static void mstar_pm51_read_pc_data(struct mstar_pm51_rproc *pm51, u32 *pc, u8 *data)
{
	unsigned tmp;

	regmap_read(pm51->mcu, MCU_PC + 4, &tmp);
	*pc = (tmp & 0xff) << 16;
	*data = tmp >> 8;
	regmap_read(pm51->mcu, MCU_PC, &tmp);
	*pc |= tmp;
}

static void mstar_pm51_dma_callback(void *dma_async_param,
				const struct dmaengine_result *result)
{
	struct mstar_pm51_rproc *pm51 = dma_async_param;

	pm51->dma_done = true;
	pm51->dma_success = result->result == DMA_TRANS_NOERROR;
	if(!pm51->dma_success)
		dev_err(&pm51->pdev->dev, "dma failed: %d\n", result->result);
	wake_up(&pm51->dma_wait);
}

static int mstar_pm51_rproc_start(struct rproc *rproc)
{
	struct mstar_pm51_rproc *pm51 = rproc->priv;
	int i;
	u32 pc;
	u8 data;

	regmap_field_write(pm51->rst, 0);
	mdelay(50);
	regmap_field_write(pm51->rst, 1);

	for(i = 0; i < 64; i++){
		mstar_pm51_read_pc_data(pm51, &pc, &data);
		printk("pm51: %06x %02x\n",(unsigned) pc, (unsigned) data);
	}

	return 0;
}

static int mstar_pm51_rproc_stop(struct rproc *rproc)
{
	struct mstar_pm51_rproc *pm51 = rproc->priv;

	regmap_field_write(pm51->rst, 0);

	return 0;
}

static int mstar_pm51_load(struct rproc *rproc, const struct firmware *fw)
{
	struct mstar_pm51_rproc *pm51 = rproc->priv;
	struct dma_chan *bdma;
	struct dma_async_tx_descriptor *dmadesc;
	struct dma_slave_config config;
	dma_addr_t dmaaddr = 0;
	void* firmware_buf;
	int ret;
	struct device *dev = &pm51->pdev->dev;
	const struct ihex_binrec *rec;
	size_t len = 0;

	regmap_field_write(pm51->sram_en, 1);
	regmap_field_write(pm51->spi_en, 0);
	regmap_field_write(pm51->dram_en, 0);
	regmap_field_write(pm51->icache_rstz, 0);

	ret = ihex_validate_fw(fw);
	if (ret)
		return ret;

	/*
	 * The firmware image isn't a complete memory image
	 * and we probably don't want to faff around DMA'ing
	 * each little piece of it. So allocated 64K and build
	 * a complete image there and DMA that in one go.
	 */

	firmware_buf = kzalloc(SZ_64K, GFP_KERNEL);
	if (!firmware_buf)
		return -ENOMEM;

	for (rec = (void *)fw->data; rec; rec = ihex_next_binrec(rec)) {
		memcpy(firmware_buf + be32_to_cpu(rec->addr), rec->data, ihex_binrec_size(rec));
		len += ihex_binrec_size(rec);
	}

	bdma = dma_request_chan(dev, "bdma0");

	if(IS_ERR(bdma)){
		dev_warn(&rproc->dev, "failed to request bdma channel, can't upload firmware\n");
		return -ENODEV;
	}

	dmaaddr = dma_map_single(dev, firmware_buf, SZ_64K, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dmaaddr))
		goto out;

	pm51->dma_done = false;
	config.direction = DMA_MEM_TO_DEV;
	config.slave_id = MSC313_BDMA_SLAVE_PM51;
	config.dst_addr = 0;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dmaengine_slave_config(bdma, &config);
	dmadesc = dmaengine_prep_slave_single(bdma, dmaaddr, len, DMA_MEM_TO_DEV, 0);
	if(dmadesc){
		dmadesc->callback_result = mstar_pm51_dma_callback;
		dmadesc->callback_param = pm51;
		dmaengine_submit(dmadesc);
		dma_async_issue_pending(bdma);
		wait_event(pm51->dma_wait, pm51->dma_done);
	}

	mstar_pm51_set_offset_sram(pm51, 0, 0x5FFF);

out:
	dma_release_channel(bdma);
	kfree(firmware_buf);

	return 0;
}

static const struct rproc_ops mstar_pm51_rproc_ops = {
	.start		= mstar_pm51_rproc_start,
	.stop		= mstar_pm51_rproc_stop,
	.load		= mstar_pm51_load,
};

static const struct of_device_id mstar_pm51_rproc_of_match[] = {
	{ .compatible = "mstar,msc313-pm51", },
	{},
};
MODULE_DEVICE_TABLE(of, mstar_pm51_rproc_of_match);

static const struct regmap_config mstar_pm51_regmap_config = {
		.reg_bits = 16,
		.val_bits = 16,
		.reg_stride = 4
};

static irqreturn_t mstar_pm51_irq(int irq, void *data)
{
	struct mstar_pm51_rproc *pm51 = data;

	return IRQ_HANDLED;
}

static int mstar_pm51_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mstar_pm51_rproc *pm51;
	const char *fw_name = "pm51.bin";
	struct rproc *rproc;
	void __iomem *base;
	int irq, ret;

	fw_name = kstrdup("pm51.bin", GFP_KERNEL);
	if(!fw_name)
		return -ENOMEM;

	rproc = devm_rproc_alloc(dev, "pm51", &mstar_pm51_rproc_ops,
			    fw_name, sizeof(*pm51));
	if (!rproc)
		return -ENOMEM;

	rproc->auto_boot = false;

	pm51 = rproc->priv;
	pm51->rproc = rproc;
	pm51->pdev = pdev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pm51->mcu = devm_regmap_init_mmio(dev, base,
                        &mstar_pm51_regmap_config);
	if(IS_ERR(pm51->mcu))
		return PTR_ERR(pm51->mcu);

	pm51->pmsleep = syscon_regmap_lookup_by_phandle(dev->of_node, "mstar,pmsleep");
	if (IS_ERR(pm51->pmsleep))
		return PTR_ERR(pm51->pmsleep);

	pm51->rst = devm_regmap_field_alloc(dev, pm51->pmsleep, pmsleep_8051_rst);
	pm51->sram_en = devm_regmap_field_alloc(dev, pm51->mcu, mcu_memmap_sram_en);
	pm51->spi_en = devm_regmap_field_alloc(dev, pm51->mcu, mcu_memmap_spi_en);
	pm51->dram_en = devm_regmap_field_alloc(dev, pm51->mcu, mcu_memmap_dram_en);
	pm51->icache_rstz = devm_regmap_field_alloc(dev, pm51->mcu, mcu_memmap_icache_rstz);

	pm51->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(pm51->clk))
			return PTR_ERR(pm51->clk);

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(dev, irq, mstar_pm51_irq, IRQF_SHARED,
		dev_name(dev), rproc);
	if (ret)
		return ret;

	init_waitqueue_head(&pm51->dma_wait);

	dev_set_drvdata(dev, rproc);

	return devm_rproc_add(dev, rproc);
}

static struct platform_driver mstar_pm51_rproc_driver = {
	.probe = mstar_pm51_rproc_probe,
	.driver = {
		.name = "mstar_pm51",
		.of_match_table = mstar_pm51_rproc_of_match,
	},
};

module_platform_driver(mstar_pm51_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MStar PM51 driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
