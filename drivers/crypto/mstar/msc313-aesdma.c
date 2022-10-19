/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HW AES driver for MStar/SigmaStar MSC313 and later SoCs
 *
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <crypto/aes.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#define DRIVER_NAME "msc313-aesdma"

#define REG_CTRL0	0x0
#define REG_CTRL1	0x4
#define REG_SRC		0x8
#define REG_XIU_LEN	0x10
#define REG_DST_START	0x18
#define REG_DST_END	0x20
/* Might be useless, current hardware doesn't have irq wired anywhere */
#define REG_INT		0x38
/* Key */
#define REG_KEY		0x40
#define REG_KEYSRC	0xa4
#define REG_STATUS	0xbc

static struct list_head dev_list = LIST_HEAD_INIT(dev_list);
static spinlock_t dev_list_lock = __SPIN_LOCK_UNLOCKED(dev_list_lock);

/*
 * key config?
 * Only newer chips
 */
#define REG_KEYCONFIG	0x9c

static const struct regmap_config msc313_aesdma_regmap_config = {
	.name = DRIVER_NAME,
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

struct msc313_aesdma {
	struct list_head dev_list;
	struct device *dev;
	struct clk *clk;
	int irq;
	struct regmap *regmap;
	struct regmap_field *start;
	struct regmap_field *reset;
	struct regmap_field *aes;
	struct regmap_field *decrypt;
	struct regmap_field *mode;
	struct regmap_field *fout;
	struct regmap_field *done;
	/* key control */
	struct regmap_field *keysrc;
	struct regmap_field *keylen;
	struct regmap_field *keybank;
};

static const struct reg_field ctrl_fstart = REG_FIELD(REG_CTRL0, 0, 0);
static const struct reg_field ctrl_reset = REG_FIELD(REG_CTRL0, 7, 7);
static const struct reg_field ctrl_fout = REG_FIELD(REG_CTRL0, 8, 8);
static const struct reg_field ctrl_aes = REG_FIELD(REG_CTRL1, 8, 8);
static const struct reg_field ctrl1_field_decrypt = REG_FIELD(REG_CTRL1, 9, 9);
static const struct reg_field ctrl1_field_mode = REG_FIELD(REG_CTRL1, 12, 13);
#define AES_MODE_ECB 0
#define AES_MODE_CTR 1
#define AES_MODE_CBC 2
static const struct reg_field keysrc_field_src = REG_FIELD(REG_KEYSRC, 5, 6);
#define KEYSRC_USER 0
#define KEYSRC_EFUSE 1
#define KEYSRC_HW 2
static const struct reg_field status_field_done = REG_FIELD(REG_STATUS, 0, 0);
static const struct reg_field keyconfig_field_len = REG_FIELD(REG_KEYCONFIG, 12, 13);
#define KEYLEN_128 0
#define KEYLEN_192 1
#define KEYLEN_256 2
static const struct reg_field keyconfig_field_bank = REG_FIELD(REG_KEYCONFIG, 11, 11);

struct msc313_aesdma_ctx {
	struct msc313_aesdma *aesdma;
};

static const struct of_device_id msc313_aesdma_of_match[] = {
	{ .compatible = "mstar,msc313-aesdma", },
	{ .compatible = "sstar,ssd20xd-aesdma", },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_aesdma_of_match);

static struct msc313_aesdma *msc313_aesdma_find_dev(struct msc313_aesdma_ctx *tfmctx)
{
	struct msc313_aesdma *aesdma = NULL;
	struct msc313_aesdma *tmp;

	spin_lock_bh(&dev_list_lock);
	if (!tfmctx->aesdma) {
		list_for_each_entry(tmp, &dev_list, dev_list) {
			aesdma = tmp;
			break;
		}
		tfmctx->aesdma = aesdma;
	} else {
		aesdma = tfmctx->aesdma;
	}
	spin_unlock_bh(&dev_list_lock);

	return aesdma;
}


static void msc313_aesdma_writekey(struct msc313_aesdma *aesdma, const void *key, size_t len)
{
	int i;
	unsigned int reg;

	/* First 8 words go in the first bank, simple */
	regmap_field_write(aesdma->keybank, 0);
	for(i = 0, reg = REG_KEY; i < 8; i++, reg += 4){
		u16 word = ((u16*)key)[7 - i];
		word = word >> 8 | word << 8;
		regmap_write(aesdma->regmap, reg, word);
	}

	if (len == 16)
		return;

	/*
	 * Second 4 or 8 words are a bit messy..
	 * If we have 4 words left they go in the bottom
	 * of the key register.
	 */
	reg = REG_KEY;
	if(len == 24)
		reg += (4 * 4);

	regmap_field_write(aesdma->keybank, 1);
	for(i = 0; (i + 8) < len/2; i++, reg += 4) {
		u16 word = ((u16*)key)[(len/2 - i) - 1];
		word = word >> 8 | word << 8;
		regmap_write(aesdma->regmap, reg, word);
	}
}

static irqreturn_t msc313_aesdma_irq(int irq, void *data)
{
	struct msc313_aesdma *aesdma = data;

	printk("aesdma int\n");

	return IRQ_HANDLED;
}

static int msc313_aesdma_do_one(struct msc313_aesdma *aesdma, u8 *out,
		const u8 *in, unsigned int len)
{
	dma_addr_t dmaaddr_src, dmaaddr_dst;
	unsigned int end, status, ready;
	int ret;

	dmaaddr_src = dma_map_single(aesdma->dev, in, len, DMA_TO_DEVICE);
	ret = dma_mapping_error(aesdma->dev, dmaaddr_src);
	if(ret)
		return ret;

	dmaaddr_dst = dma_map_single(aesdma->dev, out, len, DMA_FROM_DEVICE);
	ret = dma_mapping_error(aesdma->dev, dmaaddr_dst);
	if(ret)
		goto unmap_src;

	regmap_write(aesdma->regmap, REG_SRC, dmaaddr_src);
	regmap_write(aesdma->regmap, REG_SRC + 4, dmaaddr_src >> 16);

	regmap_write(aesdma->regmap, REG_XIU_LEN, len);
	regmap_write(aesdma->regmap, REG_XIU_LEN + 4, len >> 16);

	regmap_write(aesdma->regmap, REG_DST_START, dmaaddr_dst);
	regmap_write(aesdma->regmap, REG_DST_START + 4, dmaaddr_dst >> 16);

	end = dmaaddr_dst + len - 1;
	regmap_write(aesdma->regmap, REG_DST_END, end);
	regmap_write(aesdma->regmap, REG_DST_END + 4, end >> 16);

	regmap_field_write(aesdma->start, 1);

	if (regmap_field_read_poll_timeout(aesdma->done, ready, ready == 1, 1, 500000)) {
		dev_err(aesdma->dev, "timeout waiting for aes to finish\n");
		ret = -ETIMEDOUT;
	}

	regmap_field_write(aesdma->start, 0);

	dma_unmap_single(aesdma->dev, dmaaddr_dst, len, DMA_FROM_DEVICE);
unmap_src:
	dma_unmap_single(aesdma->dev, dmaaddr_src, len, DMA_TO_DEVICE);

	return 0;
}

static int msc313_aesdma_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	struct msc313_aesdma_ctx *ctx = crypto_tfm_ctx(tfm);
	struct msc313_aesdma *aesdma = msc313_aesdma_find_dev(ctx);

	regmap_field_write(aesdma->keysrc, KEYSRC_USER);
	switch(key_len) {
	case 16:
		regmap_field_write(aesdma->keylen, KEYLEN_128);
		break;
	case 24:
		regmap_field_write(aesdma->keylen, KEYLEN_192);
		break;
	case 32:
		regmap_field_write(aesdma->keylen, KEYLEN_256);
		break;
	default:
		return -EINVAL;
	}

	msc313_aesdma_writekey(aesdma, in_key, key_len);

	return 0;
}

static void msc313_aesdma_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct msc313_aesdma_ctx *ctx = crypto_tfm_ctx(tfm);
	struct msc313_aesdma *aesdma = msc313_aesdma_find_dev(ctx);
	int ret;

	regmap_field_write(aesdma->decrypt, 0);
	regmap_field_write(aesdma->aes, 1);
	regmap_field_write(aesdma->fout, 1);
	regmap_field_write(aesdma->mode, AES_MODE_ECB);

	ret = msc313_aesdma_do_one(aesdma, out, in, 16);
	if(ret)
		dev_err(aesdma->dev, "encrypt failed\n");
}

static void msc313_aesdma_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct msc313_aesdma_ctx *ctx = crypto_tfm_ctx(tfm);
	struct msc313_aesdma *aesdma = msc313_aesdma_find_dev(ctx);
	int ret;

	regmap_field_write(aesdma->decrypt, 1);
	regmap_field_write(aesdma->aes, 1);
	regmap_field_write(aesdma->fout, 1);
	regmap_field_write(aesdma->mode, AES_MODE_ECB);

	ret = msc313_aesdma_do_one(aesdma, out, in, 16);
	if(ret)
		dev_err(aesdma->dev, "decrypt failed\n");
}

static struct crypto_alg aes_alg = {
	.cra_name        = "aes",
	.cra_driver_name = "msc313-aesdma",
	.cra_priority    = 300,
	.cra_flags       = CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize   = AES_BLOCK_SIZE,
	.cra_ctxsize     = sizeof(struct msc313_aesdma_ctx),
	.cra_alignmask   = 16 - 1,
	.cra_module      = THIS_MODULE,
	.cra_u           = {
		.cipher = {
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey	   	= 	msc313_aesdma_setkey,
			.cia_encrypt	 	=	msc313_aesdma_encrypt,
			.cia_decrypt	  	=	msc313_aesdma_decrypt,
		}
	}
};

static int msc313_aesdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_aesdma *aesdma;
	void __iomem *base;
	int ret, irq;

	aesdma = devm_kzalloc(dev, sizeof(*aesdma), GFP_KERNEL);
	if (!aesdma)
		return -ENOMEM;

	aesdma->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	aesdma->regmap = devm_regmap_init_mmio(dev, base,
                        &msc313_aesdma_regmap_config);
	if(IS_ERR(aesdma->regmap)){
		dev_err(dev, "failed to register regmap");
		return PTR_ERR(aesdma->regmap);
	}

	aesdma->start = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl_fstart);
	aesdma->reset = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl_reset);
	aesdma->fout = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl_fout);
	aesdma->aes = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl_aes);
	aesdma->decrypt = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl1_field_decrypt);
	aesdma->mode = devm_regmap_field_alloc(dev, aesdma->regmap, ctrl1_field_mode);
	aesdma->done = devm_regmap_field_alloc(dev, aesdma->regmap, status_field_done);

	/* key fields */
	aesdma->keysrc = devm_regmap_field_alloc(dev, aesdma->regmap, keysrc_field_src);
	aesdma->keylen = devm_regmap_field_alloc(dev, aesdma->regmap, keyconfig_field_len);
	aesdma->keybank = devm_regmap_field_alloc(dev, aesdma->regmap, keyconfig_field_bank);

	aesdma->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(aesdma->clk)) {
		return PTR_ERR(aesdma->clk);
	}

	/* Optional IRQ */
	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq) {
		ret = devm_request_irq(dev, irq, msc313_aesdma_irq, IRQF_SHARED,
		dev_name(dev), aesdma);
		regmap_update_bits(aesdma->regmap, REG_INT, 1 << 7, 1 << 7);
	}

	regmap_field_write(aesdma->reset, 1);
	clk_prepare_enable(aesdma->clk);
	mdelay(10);
	regmap_field_write(aesdma->reset, 0);

	spin_lock(&dev_list_lock);
	list_add_tail(&aesdma->dev_list, &dev_list);
	spin_unlock(&dev_list_lock);

	if ((ret = crypto_register_alg(&aes_alg)) != 0)
		goto err;

	return 0;
err:
	crypto_unregister_alg(&aes_alg);
	return ret;
}

static int msc313_aesdma_remove(struct platform_device *pdev)
{
	crypto_unregister_alg(&aes_alg);
	return 0;
}

static struct platform_driver msc313_aesdma_driver = {
	.probe = msc313_aesdma_probe,
	.remove = msc313_aesdma_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_aesdma_of_match,
	},
};
module_platform_driver(msc313_aesdma_driver);

MODULE_DESCRIPTION("MStar MSC313 AESDMA driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
