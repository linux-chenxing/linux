/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HW SHA driver for MStar/SigmaStar MSC313 and later SoCs
 *
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <crypto/sha2.h>
#include <crypto/internal/hash.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#define DRIVER_NAME	"msc313-sha"
#define REG_CTRL	0x0
#define REG_SRC		0x8
#define REG_LEN		0x10
#define REG_MIUSEL	0x18
#define REG_STATUS	0x1c
#define REG_VALUE	0x20

struct msc313_sha_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

static struct msc313_sha_drv msc313_sha = {
	.dev_list = LIST_HEAD_INIT(msc313_sha.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(msc313_sha.lock),
};

struct msc313_sha_desc_ctx {
	int x;
};

static const struct regmap_config msc313_sha_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

static struct reg_field ctrl_fire = REG_FIELD(REG_CTRL, 0, 0);
static struct reg_field ctrl_clr = REG_FIELD(REG_CTRL, 6, 6);
static struct reg_field ctrl_rst = REG_FIELD(REG_CTRL, 7, 7);
static struct reg_field ctrl_sha256 = REG_FIELD(REG_CTRL, 9, 9);
static struct reg_field ctrl_disablesg = REG_FIELD(REG_CTRL, 11, 11);
static struct reg_field ctrl_inithash = REG_FIELD(REG_CTRL, 13, 13);
/* Controls whether padding is automatically added or not */
static struct reg_field ctrl_manual = REG_FIELD(REG_CTRL, 14, 14);
static struct reg_field status_ready = REG_FIELD(REG_STATUS, 0, 0);

struct msc313_sha {
	struct device *dev;
	struct list_head sha_list;
	struct clk *clk;
	struct regmap *regmap;

	struct regmap_field *fire;
	struct regmap_field *clear;
	struct regmap_field *reset;
	struct regmap_field *sha256;
	struct regmap_field *disablesg;
	struct regmap_field *inithash;
	struct regmap_field *manual;
	struct regmap_field *ready;
};

struct msc313_sha_ctx
{
	struct msc313_sha *sha;
	unsigned int done;
	struct sha256_state sha256_state;
};

static void msc313_sha_reset(struct msc313_sha *sha)
{
	regmap_field_write(sha->reset, 1);
	clk_prepare_enable(sha->clk);
	mdelay(10);
	regmap_field_write(sha->reset, 0);
}

static struct msc313_sha *msc311_sha_find_dev(struct msc313_sha_ctx *tfmctx)
{
	struct msc313_sha *sha = NULL;
	struct msc313_sha *tmp;

	spin_lock_bh(&msc313_sha.lock);
	if (!tfmctx->sha) {
		list_for_each_entry(tmp, &msc313_sha.dev_list, sha_list) {
			sha = tmp;
			break;
		}
		tfmctx->sha = sha;
	} else {
		sha = tfmctx->sha;
	}
	spin_unlock_bh(&msc313_sha.lock);

	return sha;
}

static void msc313_sha_state_out(struct msc313_sha *sha, u8 *in)
{
	unsigned word;
	int i;

	in += SHA256_DIGEST_SIZE;

	/* Write out and fix up the byte order at the same time */
	for (i = 0; i < SHA256_DIGEST_SIZE/2; i++) {
		in--;
		word = *in;
		in--;
		word |= *in << 8;
		regmap_write(sha->regmap, REG_VALUE + (i * 4), word);
	}

	//dev_info(sha->dev, "wrote: %32phN\n", in);
}

static void msc313_sha_state_in(struct msc313_sha *sha, u8 *out)
{
	unsigned word;
	int i;

	out += SHA256_DIGEST_SIZE;

	/* Read out the result and fix up the byte order at the same time */
	for (i = 0; i < SHA256_DIGEST_SIZE/2; i++) {
		regmap_read(sha->regmap, REG_VALUE + (i * 4), &word);
		*--out = word & 0xff;
		*--out = (word >> 8) & 0xff;
	}

	//dev_info(sha->dev, "read: %32phN\n", out);
}

static int msc313_sha_sha256_init(struct shash_desc *desc)
{
	struct msc313_sha_desc_ctx *ctx = shash_desc_ctx(desc);
	struct msc313_sha_ctx *sctx = crypto_shash_ctx(desc->tfm);
	struct msc313_sha *sha = msc311_sha_find_dev(sctx);

	memset(&sctx->sha256_state, 0, sizeof(sctx->sha256_state));
	sctx->done = 0;

	return 0;
}

static int msc313_sha_do_one(struct msc313_sha_ctx *ctx,
			     const void *addr,
			     unsigned int len,
			     u8 *init)
{
	int ret = 0;
	unsigned ready;
	dma_addr_t dmaaddr;
	struct msc313_sha *sha = ctx->sha;

	regmap_field_write(sha->sha256, 1);

	/*
	 * If this is a continuation reload the
	 * state and set the flag.
	 */
	if (ctx->done) {
		msc313_sha_state_out(sha, init);
		regmap_field_write(sha->inithash, 1);
	}
	else
		regmap_field_write(sha->inithash, 0);

	dmaaddr = dma_map_single(sha->dev, addr, len, DMA_TO_DEVICE);
	ret = dma_mapping_error(sha->dev, dmaaddr);
	if(ret)
		return ret;

	regmap_write(sha->regmap, REG_SRC, dmaaddr);
	regmap_write(sha->regmap, REG_SRC + 4, dmaaddr >> 16);
	regmap_write(sha->regmap, REG_LEN, len);
	regmap_write(sha->regmap, REG_LEN + 4, len >> 16);

	regmap_field_write(sha->fire, 1);

	/*
	 * The vendor code for this thing says it takes ~1us.
	 * But they seem to only ever do 64 byte blocks at a time.
	 */
	if (regmap_field_read_poll_timeout(sha->ready, ready, ready == 1, 1, 100)) {
		dev_err(sha->dev, "timeout waiting for update to finish\n");
		ret = -ETIMEDOUT;
	}

	/* Result needs to be read now as it will get cleared */
	msc313_sha_state_in(sha, init);

	regmap_field_write(sha->fire, 0);

	/*
	 * This seems weird but it looks like it's needed to
	 * get the hardware to actually process another block.
	 */
	regmap_field_write(sha->clear, 1);
	regmap_field_write(sha->clear, 0);

	dma_unmap_single(sha->dev, dmaaddr, len, DMA_TO_DEVICE);

	ctx->done += len;

	return ret;
}

static int msc313_sha_sha256_update(struct shash_desc *desc, const u8 *data,
		unsigned int len)
{
	struct msc313_sha_desc_ctx *ctx = shash_desc_ctx(desc);
	struct msc313_sha_ctx *sctx = crypto_shash_ctx(desc->tfm);
	struct sha256_state *sha256_state = &sctx->sha256_state;
	const unsigned int bufsz = sizeof(sha256_state->buf);
	u8 *init = (u8*) sha256_state->state;
	u8 *buf = sha256_state->buf;
	int ret;

	/*
	 * There is nothing buffered and the len is a full block or more and
	 * it is aligned correctly. Feed the full blocks directly in.
	 */
direct:
	if(sha256_state->count == 0 &&
	   (len >= bufsz) &&
	   IS_ALIGNED((unsigned long)data, 64) &&
	   !is_vmalloc_addr(data)) {
		unsigned int tail, head;

		tail = len % bufsz;
		head = len - tail;
		ret = msc313_sha_do_one(sctx, data, head, init);
		if (ret)
			return ret;

		/* Stash the remainder */
		if (tail)
			memcpy(buf, data + head, tail);

		sha256_state->count += len;
	}
	/*
	 * There is some buffered data, either existing or from the block
	 * above. Create full blocks in the buffer and feed that in until
	 * either the buffer is gone or it's ok to directly feed.
	 */
	else {
		while (len) {
			unsigned int space = bufsz - (sha256_state->count % bufsz);
			unsigned int copysz = min(len, space);

			memcpy(sha256_state->buf + (sha256_state->count % bufsz), data, copysz);
			sha256_state->count += copysz;
			/* There is a full block, process it */
			if ((sha256_state->count % bufsz) == 0) {
				ret = msc313_sha_do_one(sctx, buf, bufsz, init);
				if (ret)
					return ret;
			}

			data += copysz;
			len -= copysz;

			/*
			 * There is now a full block available, try to go back
			 * to direct feeding.
			 */
			if(len >= bufsz)
				goto direct;
		}
	}

	return 0;
}

static int msc313_sha_sha256_final(struct shash_desc *desc, u8 *out)
{
	struct msc313_sha_desc_ctx *ctx = shash_desc_ctx(desc);
	struct msc313_sha_ctx *sctx = crypto_shash_ctx(desc->tfm);
	struct sha256_state *sha256_state = &sctx->sha256_state;
	const unsigned int bufsz = sizeof(sha256_state->buf);
	unsigned int head = sha256_state->count % SHA256_BLOCK_SIZE;
	__be64 bits = cpu_to_be64(sha256_state->count * 8);
	u8 *init = (u8*) sha256_state->state;
	u8 *buf = sha256_state->buf;
	unsigned int space;
	int ret;

	/* How many bits are left in the current block ? */
	space = ((sha256_state->count == 0 ? bufsz : bufsz - head) * 8);
	/* Add the 1 bit */
	buf[head++] = 0x80;
	/* Not enough space */
	if (space < ((sizeof(bits) * 8) + 1)) {
		/* Fill the rest of the block with zeros and send it */
		memset(buf + head, 0, bufsz - head);
		ret = msc313_sha_do_one(sctx, buf, bufsz, init);
		if (ret)
			return ret;
		/* Clear the start of the block */
		memset(buf, 0, bufsz - sizeof(bits));
		/* Add the length */
		memcpy(buf + (bufsz - sizeof(bits)), &bits, sizeof(bits));
		ret = msc313_sha_do_one(sctx, buf, bufsz, init);
		if (ret)
			return ret;
	}
	/* pad fits into a single block */
	else {
		/* Add 0s up to the size */
		memset(buf + head, 0, bufsz - head);
		/* Add the length */
		memcpy(buf + (bufsz - sizeof(bits)), &bits, sizeof(bits));
		ret = msc313_sha_do_one(sctx, buf, bufsz, init);
		if (ret)
			return ret;
	}

	memcpy(out, sha256_state->state, sizeof(sha256_state->state));
	//clk_disable_unprepare(sha->clk);

	return 0;
}

static int msc313_sha_sha256_export(struct shash_desc *desc, void *out)
{
	struct msc313_sha_ctx *sctx = crypto_shash_ctx(desc->tfm);

	memcpy(out, sctx, sizeof(*sctx));

	return 0;
}

static int msc313_sha_sha256_import(struct shash_desc *desc, const void *in)
{
	struct msc313_sha_ctx *sctx = crypto_shash_ctx(desc->tfm);

	memcpy(sctx, in, sizeof(*sctx));

	return 0;
}

static struct shash_alg msc313_algos[] = {
	{
		.digestsize = SHA256_DIGEST_SIZE,
		.init       = msc313_sha_sha256_init,
		.update     = msc313_sha_sha256_update,
		.final      = msc313_sha_sha256_final,
		.descsize   = sizeof(struct msc313_sha_desc_ctx),
		.statesize  = sizeof(struct msc313_sha_ctx),
		.export     = msc313_sha_sha256_export,
		.import     = msc313_sha_sha256_import,
		.base       = {
			.cra_name        = "sha256",
			.cra_driver_name = "msc313-sha-sha256",
			.cra_priority    = 300,
			.cra_blocksize   = SHA256_BLOCK_SIZE,
			.cra_module      = THIS_MODULE,
			.cra_ctxsize     = sizeof(struct msc313_sha_ctx),
		},
	},
};

static int msc313_sha_probe(struct platform_device *pdev)
{
	struct msc313_sha *sha;
	void __iomem *base;
	int ret;

	sha = devm_kzalloc(&pdev->dev, sizeof(*sha), GFP_KERNEL);
	if (!sha){
		ret = -ENOMEM;
		goto out;
	}

	sha->dev = &pdev->dev;
	sha->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sha->clk))
		return PTR_ERR(sha->clk);

	platform_set_drvdata(pdev, sha);

	INIT_LIST_HEAD(&sha->sha_list);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	sha->regmap = devm_regmap_init_mmio(&pdev->dev, base,
                        &msc313_sha_regmap_config);
	if(IS_ERR(sha->regmap))
		return PTR_ERR(sha->regmap);

	sha->fire = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_fire);
	sha->clear = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_clr);
	sha->reset = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_rst);
	sha->sha256 = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_sha256);
	sha->disablesg = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_disablesg);
	sha->inithash = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_inithash);
	sha->manual = devm_regmap_field_alloc(&pdev->dev, sha->regmap, ctrl_manual);
	sha->ready = devm_regmap_field_alloc(&pdev->dev, sha->regmap, status_ready);

	msc313_sha_reset(sha);

	regmap_field_write(sha->disablesg, 1);
	regmap_field_write(sha->manual, 1);

	spin_lock(&msc313_sha.lock);
	list_add_tail(&sha->sha_list, &msc313_sha.dev_list);
	spin_unlock(&msc313_sha.lock);

	ret = crypto_register_shashes(msc313_algos, ARRAY_SIZE(msc313_algos));

out:
	return ret;
}

static int msc313_sha_remove(struct platform_device *pdev)
{
	crypto_unregister_shashes(msc313_algos, ARRAY_SIZE(msc313_algos));

	return 0;
}

static const struct of_device_id msc313_sha_of_match[] = {
	{ .compatible = "mstar,msc313-sha", },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_sha_of_match);

static struct platform_driver msc313_sha_driver = {
	.probe = msc313_sha_probe,
	.remove = msc313_sha_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_sha_of_match,
	},
};
module_platform_driver(msc313_sha_driver);

MODULE_DESCRIPTION("MStar MSC313 SHA driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
