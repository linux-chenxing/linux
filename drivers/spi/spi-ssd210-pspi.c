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
#include <linux/clk-provider.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME "spi_ssd210_pspi"

#define REG_DIV	0x4

struct ssd210_pspi_spi {
	struct device *dev;
	struct spi_master *master;

	/* for divider clock */
	spinlock_t lock;
	struct clk *clk;
	struct clk_hw *divider;
};

static const struct regmap_config ssd210_pspi_spi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
	.fast_io = true,
};

static int ssd210_pspi_spi_setup(struct spi_device *spi)
{
	struct ssd210_pspi_spi *mspi = spi_master_get_devdata(spi->master);

	return 0;
}

static void ssd210_pspi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ssd210_pspi_spi *mspi = spi_master_get_devdata(spi->master);
}

static int ssd210_pspi_spi_transfer_one(struct spi_controller *ctlr,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	struct ssd210_pspi_spi *mspi = spi_controller_get_devdata(ctlr);

	return 0;
}

static irqreturn_t ssd210_pspi_spi_irq(int irq, void *data)
{
	struct ssd210_pspi_spi *mspi = data;

	return IRQ_HANDLED;
}

static const struct clk_div_table div_table[] = {
	{0, 2},
	{1, 4},
	{2, 8},
	{3, 16},
	{4, 32},
	{5, 64},
	{6, 128},
	{7, 256},
	{ 0 },
};

static int ssd210_pspi_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	int ret, irq, numparents;
	struct ssd210_pspi_spi *spi;
	const char *parents[1];
	void __iomem *base;
	char *sclk_name;
	//struct clk *sclk;

	numparents = of_clk_parent_fill(pdev->dev.of_node, parents, ARRAY_SIZE(parents));
	if (numparents != 1)
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ssd210_pspi_spi));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	spi = spi_master_get_devdata(master);
	spi->dev = &pdev->dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	spi->clk = devm_clk_get_enabled(&pdev->dev, NULL);
		if (IS_ERR(spi->clk))
			return PTR_ERR(spi->clk);

	spin_lock_init(&spi->lock);
	//sclk_name = devm_kasprintf(dev, GFP_KERNEL, "%s_sclk", dev_name(dev));
	//spi->divider = devm_clk_hw_register_divider_table(dev, sclk_name, parents[0], 0,
	//		base + REG_DIV, 0, 7,
	//		0, div_table, &spi->lock);
	//if (IS_ERR(spi->divider))
	//	return PTR_ERR(spi->divider);

	//sclk = spi->divider->clk;

	//ret = clk_prepare_enable(sclk);
	//if (ret)
	//	dev_err(&pdev->dev, "clk enable failed: %d\n", ret);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, ssd210_pspi_spi_irq, IRQF_SHARED,
			dev_name(&pdev->dev), spi);

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->num_chipselect = 1;
	master->mode_bits = SPI_CPHA | SPI_CPOL;
	master->max_speed_hz = 1000000;
	master->min_speed_hz = 1000000;
	master->setup = ssd210_pspi_spi_setup;
	master->set_cs = ssd210_pspi_spi_set_cs;
	master->transfer_one = ssd210_pspi_spi_transfer_one;
	master->bits_per_word_mask = SPI_BPW_MASK(8);

	return devm_spi_register_master(&pdev->dev, master);
}

static const struct of_device_id ssd210_pspi_spi_of_match[] = {
	{
		.compatible = "sstar,ssd210-pspi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ssd210_pspi_spi_of_match);

static struct platform_driver ssd210_pspi_spi_driver = {
	.probe = ssd210_pspi_spi_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ssd210_pspi_spi_of_match,
	},
};
module_platform_driver(ssd210_pspi_spi_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("MStar MSC313 SPI driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
