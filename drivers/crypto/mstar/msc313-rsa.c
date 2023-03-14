/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HW RSA driver for MStar/SigmaStar MSC313 and later SoCs
 *
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <crypto/akcipher.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/scatterlist.h>

#define DRIVER_NAME "msc313-rsa"

#define SRAM_BYTES	256

#define REG_CTRL	0x0
#define REG_IND32	0x4
#define REG_ADDR	0x8
#define REG_FILE_IN	0xc
#define REG_FILE_OUT	0x14
#define REG_START	0x1c
#define REG_KEYCONFIG	0x20
#define REG_STATUS	0x24

#define ADDR_E		0x00
#define ADDR_N		0x40
#define ADDR_A		0x80
/*
 * This address seems to be for show only.
 * A read from the SRAM seems to only use the bottom 6 bits
 */
#define ADDR_Z		0xc0

static struct list_head dev_list = LIST_HEAD_INIT(dev_list);
static spinlock_t dev_list_lock = __SPIN_LOCK_UNLOCKED(dev_list_lock);

static const struct regmap_config msc313_rsa_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
	.fast_io = true,
};

struct msc313_rsa {
	struct device *dev;
	struct clk *clk;
	struct list_head dev_list;
	int irq;
	struct regmap *regmap;
	bool quirk_dummyread;

	spinlock_t lock;

	struct regmap_field *reset;

	/* sram control */
	struct regmap_field *ind32start;
	struct regmap_field *write;
	struct regmap_field *autoinc;
	struct regmap_field *autostart;
	struct regmap_field *addr;
	struct regmap_field *datainh, *datainl;
	struct regmap_field *dataouth, *dataoutl;

	struct regmap_field *start;

	struct regmap_field *busy;
	struct regmap_field *done;

	/* key config */
	struct regmap_field *hwkey;
	struct regmap_field *pubkey;
	struct regmap_field *keylen;
};

struct msc313_rsa_ctx {
	struct rsa_key raw_key;
	const u8* n;
	unsigned nlen;
	bool keyispub;
	u8 swapbuf[SRAM_BYTES];

	bool fallback;
	struct crypto_akcipher *soft_tfm;
};

static const struct reg_field ctrl_ind32start = REG_FIELD(REG_CTRL, 0, 0);
static const struct reg_field ind32_write = REG_FIELD(REG_IND32, 1, 1);
static const struct reg_field ind32_autoinc = REG_FIELD(REG_IND32, 2, 2);
static const struct reg_field ind32_autostart = REG_FIELD(REG_IND32, 3, 3);
static const struct reg_field addr = REG_FIELD(REG_ADDR, 0, 7);
static const struct reg_field datainh = REG_FIELD(REG_FILE_IN + 4, 0, 15);
static const struct reg_field datainl = REG_FIELD(REG_FILE_IN, 0, 15);
static const struct reg_field dataouth = REG_FIELD(REG_FILE_OUT + 4, 0, 15);
static const struct reg_field dataoutl = REG_FIELD(REG_FILE_OUT, 0, 15);

static const struct reg_field start_field_start = REG_FIELD(REG_START, 0, 0);

static const struct reg_field keyconfig_reset = REG_FIELD(REG_KEYCONFIG, 0, 0);
static const struct reg_field keyconfig_field_hw = REG_FIELD(REG_KEYCONFIG, 1, 1);
static const struct reg_field keyconfig_field_public = REG_FIELD(REG_KEYCONFIG, 2, 2);
static const struct reg_field keyconfig_field_length = REG_FIELD(REG_KEYCONFIG, 8, 13);

static const struct reg_field status_field_busy = REG_FIELD(REG_STATUS, 0, 0);
static const struct reg_field status_field_done = REG_FIELD(REG_STATUS, 1, 1);

static struct msc313_rsa *msc313_rsa_find_dev(void)
{
	struct msc313_rsa *tmp = NULL;
	struct msc313_rsa *rsa = NULL;

	spin_lock_bh(&dev_list_lock);
		list_for_each_entry(tmp, &dev_list, dev_list) {
			rsa = tmp;
			break;
		}
	spin_unlock_bh(&dev_list_lock);

	return rsa;
}

/*
 * This seems to reset the state machine and clear the
 * SRAM.
 * It needs to be called after triggering an operation
 * and reading the result.
 */
static int msc313_rsa_reset(struct msc313_rsa *rsa)
{
	int ret;
	unsigned int busy;

	/*
	 * only the SRAM for the currently configured key size
	 * is cleared by reset. Reset the key size to make sure
	 * all of the SRAM is cleared.
	 */
	regmap_field_write(rsa->keylen, ~0);

	ret = regmap_field_force_write(rsa->reset, 1);
	ret = regmap_field_force_write(rsa->reset, 0);

	/* cryptomgr_test is calling this in an atomic context?? */
	if (regmap_field_read_poll_timeout_atomic(rsa->busy, busy, busy, 10, 1000000))
		return -ETIMEDOUT;

	return ret;
}

static irqreturn_t msc313_rsa_irq(int irq, void *data)
{
	struct msc313_rsa *rsa = data;
	return IRQ_HANDLED;
}

static int msc313_rsa_write_memory(struct msc313_rsa *rsa,
		struct msc313_rsa_ctx *ctx, u16 addr,
		const u8 *buffer, unsigned int len)
{
	int ret;

	/* Buffer passed in must be padded */
	if ((len % 4) != 0)
		return -EINVAL;

	//dev_info(rsa->dev, "Writing %d bytes to SRAM at 0x%x\n", len, (unsigned) addr);

	regmap_field_write(rsa->write, 1);
	regmap_field_write(rsa->autoinc, 1);
	regmap_field_write(rsa->autostart, 1);
	regmap_field_write(rsa->addr, addr);
	regmap_field_force_write(rsa->ind32start, 1);

	/*
	 * The data needs to be loaded into SRAM backwards.
	 * It's easier for my poor brain to swap the buffer here
	 * and load it forwards.
	 */
	for (int i = 0; i < len; i++)
		ctx->swapbuf[i] = buffer[(len - 1) - i];
	buffer = ctx->swapbuf;

	//for (int i = 0; i < len; i += 16)
	//	dev_info(rsa->dev, "data write %d:\t%16phN\n", i, buffer + i);
	for (int i = 0; i < len; i += 4) {
		unsigned int tmp;
		u16 l, h;

		l = buffer[i];
		l |= (buffer[i + 1] << 8);
		h = buffer[i + 2];
		h |= (buffer[i + 3] << 8);

		//regmap_read(rsa->regmap, REG_ADDR, &tmp);
		//printk("write addr: %02x, value: %04x,%04x,\n", addr + (i/4), l, h);

		ret = regmap_field_write(rsa->datainl, l);
		regmap_field_read(rsa->datainh, &tmp);
		if (ret)
			break;
		ret = regmap_field_write(rsa->datainh, h);
		regmap_field_read(rsa->datainh, &tmp);
		if (ret)
			break;
	}

	regmap_field_force_write(rsa->ind32start, 0);

	return ret;
}

static int msc313_rsa_read_memory(struct msc313_rsa *rsa, struct msc313_rsa_ctx *ctx,
		u16 addr, u8 *buffer, unsigned int len)
{
	if (len % 4 != 0)
		return -EINVAL;

	regmap_field_write(rsa->addr, addr);

	regmap_field_write(rsa->write, 0);
	regmap_field_write(rsa->autoinc, 1);
	regmap_field_write(rsa->autostart, 1);

	for (int i = 0; i < len; i += 4) {
		unsigned int l, h;

		/*
		 * boot rom is doing this.. if this isn't
		 * done then we read back the first 4 bytes
		 * over and over..
		 */
		regmap_field_write(rsa->addr, addr + (i >> 2));
		regmap_field_force_write(rsa->ind32start, 1);

		regmap_field_read(rsa->dataoutl, &l);
		regmap_field_read(rsa->dataouth, &h);

		buffer[i] = l;
		buffer[i + 1] = l >> 8;
		buffer[i + 2] = h;
		buffer[i + 3] = h >> 8;
	}

	//for (int i = 0; i < len; i += 16)
	//	dev_info(rsa->dev, "data read %d:\t%16phN\n", i, buffer + i);

	/* Data will be read out in reverse, flip it around */
	for (int i = 0; i < len; i++)
		ctx->swapbuf[i] = buffer[(len - 1) - i];
	memcpy(buffer, ctx->swapbuf, len);

	regmap_field_force_write(rsa->ind32start, 0);


	return 0;
}

static int msc313_rsa_do_one(struct msc313_rsa *rsa, struct msc313_rsa_ctx *ctx, u8 *in, u8 *out, unsigned int len)
{
	unsigned int busy, done;
	unsigned hwkey, pubkey, keylen;
	int ret = 0;

	regmap_field_read(rsa->hwkey, &hwkey);
	regmap_field_read(rsa->keylen, &keylen);
	regmap_field_read(rsa->pubkey, &pubkey);

	//dev_info(rsa->dev, "hwkey %d, pubkey: %d, keylen: %dbits (%x)\n", hwkey, pubkey, ((keylen + 1) * 4) * 8, keylen);

	ret = msc313_rsa_write_memory(rsa, ctx, ADDR_A, in, len);
	if (ret)
		return ret;

	ret = regmap_field_read(rsa->busy, &busy);
	if (ret)
		return ret;

	ret = regmap_field_read(rsa->done, &done);
	if (ret)
		return ret;

	if (busy || done) {
		dev_err(rsa->dev, "busy and/or done flag set before starting\n");
		return -EBUSY;
	}

	regmap_field_force_write(rsa->start, 1);

	/* cryptomgr_test is calling this in an atomic context?? */
	if(regmap_field_read_poll_timeout_atomic(rsa->done, done, done == 1, 10, 1000000)) {
		regmap_field_read(rsa->busy, &busy);
		regmap_field_read(rsa->done, &done);

		dev_err(rsa->dev, "timeout waiting for rsa to finish, busy: %d, done %d\n", busy, done);
		ret = -ETIMEDOUT;
	}

	if (!ret)
		msc313_rsa_read_memory(rsa, ctx, ADDR_Z, out, len);

	regmap_field_write(rsa->start, 0);

	return ret;
}

#if 0
static void msc313_rsa_test(struct msc313_rsa *rsa)
{
	u8 test_in[256] = { }, test_in2[256], test_as[256];
	u8 test_out[256] = { };

	for (int i = 0; i < sizeof(test_in); i++){
		test_in[i] = ~i;
		test_in2[i] = i;
		test_as[i] = 0xaa;
	}

	dev_info(rsa->dev, "testing SRAM works\n");
	msc313_rsa_write_memory(rsa, ADDR_Z, test_in, sizeof(test_in), false);
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	if (memcmp(test_in, test_out, sizeof(test_in)) != 0) {
		dev_err(rsa->dev, "SRAM write test failed\n");
	}

	msc313_rsa_write_memory(rsa, ADDR_Z + 0x8, test_as, 64, false);
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	/* put the known contents back */
	msc313_rsa_write_memory(rsa, ADDR_Z, test_in, sizeof(test_in), false);

	dev_info(rsa->dev, "Checking exponent memory and output memory are unique\n");
	msc313_rsa_write_memory(rsa, ADDR_E, test_in2, sizeof(test_in2), false);
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	if (memcmp(test_in, test_out, sizeof(test_in)) != 0) {
		dev_err(rsa->dev, "exponent overwrote output\n");
	}

	dev_info(rsa->dev, "Checking modulus memory and output memory are unique\n");
	msc313_rsa_write_memory(rsa, ADDR_N, test_in2, sizeof(test_in2), false);
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	if (memcmp(test_in, test_out, sizeof(test_in)) != 0) {
		dev_err(rsa->dev, "modulus overwrote output\n");
	}

	dev_info(rsa->dev, "Checking input memory and output memory are unique\n");
	msc313_rsa_write_memory(rsa, ADDR_A, test_in2, sizeof(test_in2), false);
	msc313_rsa_read_memory(rsa, ADDR_A, test_out, sizeof(test_out));
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	if (memcmp(test_in, test_out, sizeof(test_in)) != 0) {
		dev_err(rsa->dev, "input overwrote output\n");
	}

	dev_info(rsa->dev, "Checking reset clears output memory\n");
	msc313_rsa_reset(rsa);
	msc313_rsa_read_memory(rsa, ADDR_Z, test_out, sizeof(test_out));

	for (int i = 0; i < sizeof(test_out); i++){
		if(test_out[i] != 0) {
			dev_err(rsa->dev, "SRAM reset test failed\n");
			break;
		}
	}
}
#endif

static int msc313_rsa_load_key(struct msc313_rsa *rsa, struct msc313_rsa_ctx *ctx, bool decrypt)
{
	const struct rsa_key *key = &ctx->raw_key;
	unsigned int esz, keylen = ctx->nlen;
	u8 *ee = NULL;
	const u8 *e;
	int ret;

	//dev_info(rsa->dev, "Loading key into SRAM, n len %d, e len %d, d len %d", key->n_sz, key->e_sz, key->d_sz);

	regmap_field_write(rsa->hwkey, 0);
	/* doesn't seem to make any difference? */
	regmap_field_write(rsa->pubkey, ctx->keyispub ? 1 : 0);
	regmap_field_write(rsa->keylen, (ctx->nlen / 4) - 1);

	/* Load N */
	ret = msc313_rsa_write_memory(rsa, ctx, ADDR_N, ctx->n, ctx->nlen);
	if (ret)
		return ret;

	/*
	 * Load either D or E into the E space depending
	 * on if encryption is happening or not.
	 */
	if (decrypt) {
		e = key->d;
		esz = key->d_sz;
	}
	else {
		e = key->e;
		esz = key->e_sz;
	}

	/*
	 * for an exponent that is not the same length as
	 * the modulus pad with zeros
	 */
	if (esz != keylen) {
		ee = kzalloc(ctx->nlen, GFP_KERNEL);
		if (!ee)
			return -ENOMEM;
		//memcpy(e, key->e, key->e_sz);
		memcpy(&ee[ctx->nlen - esz], e, esz);
	}

	ret = msc313_rsa_write_memory(rsa, ctx, ADDR_E, ee ? ee : e, keylen);

	kfree(ee);

	if (ret)
		return ret;

	return 0;
}

static int msc313_rsa_endecrypt(struct akcipher_request *req, bool decrypt)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct msc313_rsa *rsa = msc313_rsa_find_dev();
	u8 *in, *out;
	int ret;
	unsigned int keylen = ctx->nlen;

	in = kzalloc(keylen, GFP_KERNEL);
	if (!in)
		return -ENOMEM;

	out = kzalloc(keylen, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	spin_lock(&rsa->lock);

	msc313_rsa_reset(rsa);
	ret = msc313_rsa_load_key(rsa, ctx, decrypt);
	if (ret)
		goto unlock;

	/* If the input data is smaller than the keylen pad it with zeros. */
	sg_copy_to_buffer(req->src, sg_nents(req->src), in + (keylen - req->src_len), keylen);
	ret = msc313_rsa_do_one(rsa, ctx, in, out, keylen);

unlock:
	spin_unlock(&rsa->lock);

	if (!ret)
		sg_copy_from_buffer(req->dst, sg_nents(req->dst), out, keylen);

	kfree(in);
	kfree(out);

	return ret;
}

static int msc313_rsa_encrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	if (ctx->fallback) {
		int ret;

		akcipher_request_set_tfm(req, ctx->soft_tfm);
		ret = crypto_akcipher_encrypt(req);
		akcipher_request_set_tfm(req, tfm);
		return ret;
	}

	return msc313_rsa_endecrypt(req, false);
}

static int msc313_rsa_decrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	if (ctx->fallback) {
		int ret;

		akcipher_request_set_tfm(req, ctx->soft_tfm);
		ret = crypto_akcipher_decrypt(req);
		akcipher_request_set_tfm(req, tfm);
		return ret;
	}

	return msc313_rsa_endecrypt(req, true);
}

static void msc313_rsa_keyfixup(struct msc313_rsa_ctx* ctx)
{
	const struct rsa_key* key = &ctx->raw_key;
	unsigned leadingzeros;
	unsigned keylen;

	for (leadingzeros = 0; leadingzeros < key->n_sz; leadingzeros++)
		if (key->n[leadingzeros])
			break;

	keylen = key->n_sz - leadingzeros;

	switch(keylen) {
	/* right now only 2048 bit keys work */
	case 256:
		ctx->fallback = false;
	break;
	default:
		ctx->fallback = true;
	}

	ctx->n = key->n + leadingzeros;
	ctx->nlen = keylen;
}

static int msc313_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
		   unsigned int keylen)
{
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	//printk("%s:%d\n", __func__, __LINE__);

	ret = rsa_parse_pub_key(&ctx->raw_key, key, keylen);
	if (ret)
		return ret;

	msc313_rsa_keyfixup(ctx);
	if (ctx->fallback)
		crypto_akcipher_set_pub_key(ctx->soft_tfm, key, keylen);

	ctx->keyispub = true;

	return 0;
}
static int msc313_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
		    unsigned int keylen)
{
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	//printk("%s:%d\n", __func__, __LINE__);

	ret = rsa_parse_priv_key(&ctx->raw_key, key, keylen);
	if (ret)
		return ret;

	msc313_rsa_keyfixup(ctx);
	if (ctx->fallback)
		crypto_akcipher_set_priv_key(ctx->soft_tfm, key, keylen);

	ctx->keyispub = false;

	return 0;
}

static unsigned int msc313_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	return ctx->nlen;
}

static int msc313_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	ctx->soft_tfm = crypto_alloc_akcipher("rsa-generic", 0, 0);
	if (IS_ERR(ctx->soft_tfm))
		return PTR_ERR(ctx->soft_tfm);

	return 0;
}

static void msc313_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	struct msc313_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	crypto_free_akcipher(ctx->soft_tfm);
}

static struct akcipher_alg rsa_alg = {
	.encrypt = msc313_rsa_encrypt,
	.decrypt = msc313_rsa_decrypt,
	.set_pub_key = msc313_rsa_set_pub_key,
	.set_priv_key = msc313_rsa_set_priv_key,
	.max_size = msc313_rsa_max_size,
	.init = msc313_rsa_init_tfm,
	.exit = msc313_rsa_exit_tfm,
	.base = {
		.cra_name = "rsa",
		.cra_driver_name = "rsa-msc313",
		.cra_priority = 3000,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct msc313_rsa_ctx),
	},
};

static int msc313_rsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_rsa *rsa;
	void __iomem *base;
	int ret, irq;

	rsa = devm_kzalloc(&pdev->dev, sizeof(*rsa), GFP_KERNEL);
	if (!rsa)
		return -ENOMEM;

	spin_lock_init(&rsa->lock);

	// only for i2m?
	rsa->quirk_dummyread = true;

	rsa->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rsa->regmap = devm_regmap_init_mmio(&pdev->dev, base,
                        &msc313_rsa_regmap_config);
	if (IS_ERR(rsa->regmap))
		return PTR_ERR(rsa->regmap);

	rsa->ind32start = devm_regmap_field_alloc(dev, rsa->regmap, ctrl_ind32start);

	/* ind32 */
	rsa->write = devm_regmap_field_alloc(dev, rsa->regmap, ind32_write);
	rsa->autoinc = devm_regmap_field_alloc(dev, rsa->regmap, ind32_autoinc);
	rsa->autostart = devm_regmap_field_alloc(dev, rsa->regmap, ind32_autostart);
	rsa->addr = devm_regmap_field_alloc(dev, rsa->regmap, addr);
	rsa->start = devm_regmap_field_alloc(dev, rsa->regmap, start_field_start);

	rsa->datainl = devm_regmap_field_alloc(dev, rsa->regmap, datainl);
	rsa->datainh = devm_regmap_field_alloc(dev, rsa->regmap, datainh);

	rsa->dataoutl = devm_regmap_field_alloc(dev, rsa->regmap, dataoutl);
	rsa->dataouth = devm_regmap_field_alloc(dev, rsa->regmap, dataouth);

	/* status */
	rsa->busy = devm_regmap_field_alloc(dev, rsa->regmap, status_field_busy);
	rsa->done = devm_regmap_field_alloc(dev, rsa->regmap, status_field_done);

	/* keyconfig */
	rsa->reset = devm_regmap_field_alloc(dev, rsa->regmap, keyconfig_reset);
	rsa->hwkey = devm_regmap_field_alloc(dev, rsa->regmap, keyconfig_field_hw);
	rsa->pubkey = devm_regmap_field_alloc(dev, rsa->regmap, keyconfig_field_public);
	rsa->keylen = devm_regmap_field_alloc(dev, rsa->regmap, keyconfig_field_length);

	rsa->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rsa->clk))
		return PTR_ERR(rsa->clk);

	ret = clk_prepare_enable(rsa->clk);
	if (ret)
		return ret;

	/*irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
		if (!irq)
			return -EINVAL;
	ret = devm_request_irq(&pdev->dev, irq, msc313_rsa_irq, IRQF_SHARED,
			dev_name(&pdev->dev), rsa);*/

	msc313_rsa_reset(rsa);

	//msc313_rsa_test(rsa);

	spin_lock(&dev_list_lock);
	list_add_tail(&rsa->dev_list, &dev_list);
	spin_unlock(&dev_list_lock);

	crypto_register_akcipher(&rsa_alg);

	return 0;
}

static int msc313_rsa_remove(struct platform_device *pdev)
{
	crypto_unregister_akcipher(&rsa_alg);
	return 0;
}

static const struct of_device_id msc313_rsa_of_match[] = {
	{ .compatible = "mstar,msc313-rsa", },
	{ .compatible = "sstar,ssd20xd-rsa", },
	{},
};
MODULE_DEVICE_TABLE(of, msc313_rsa_of_match);

static struct platform_driver msc313_rsa_driver = {
	.probe = msc313_rsa_probe,
	.remove = msc313_rsa_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_rsa_of_match,
	},
};
module_platform_driver(msc313_rsa_driver);

MODULE_DESCRIPTION("MStar MSC313 RSA driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
