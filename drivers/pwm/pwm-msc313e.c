// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 * Copyright (C) 2023 Romain Perier <romain.perier@gmail.com>
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/pwm.h>

#define CHANNEL_OFFSET		0x80
#define REG_DUTYL(chan)		(0x8  + CHANNEL_OFFSET * (chan))
#define REG_DUTYH(chan)		(0x12 + CHANNEL_OFFSET * (chan))
#define REG_PERIODL(chan)	(0x10 + CHANNEL_OFFSET * (chan))
#define REG_PERIODH(chan)	(0x14 + CHANNEL_OFFSET * (chan))
#define REG_DIV(chan)		(0x18 + CHANNEL_OFFSET * (chan))
#define REG_CTRL(chan)		(0x1c + CHANNEL_OFFSET * (chan))
#define REG_SWRST		0x1fc

#define POLARITY_EN_BIT	BIT(4)

struct msc313e_pwm {
	void __iomem *base;
	struct pwm_chip pwmchip;
	struct clk *clk;
};

struct msc313e_pwm_info {
	unsigned int channels;
};

#define to_msc313e_pwm(ptr) container_of(ptr, struct msc313e_pwm, pwmchip)

static const struct msc313e_pwm_info msc313e_data = {
	.channels = 8,
};

static const struct msc313e_pwm_info ssd20xd_data = {
	.channels = 4,
};

static void msc313e_pwm_writecounter(struct msc313e_pwm *priv, unsigned long low,
				     unsigned long high, u32 value)
{
	/* The bus that connects the CPU to the peripheral registers splits 32 bit registers into
	 * two 16bit registers placed 4 bytes apart. It's the hardware design they used. The counter
	 * we are about to write has this contrainst.
	 */
	writew(value & 0xffff, priv->base + low);
	writeb((value >> 16) & GENMASK(2, 0), priv->base + high);
}

static void msc313e_pwm_readcounter(struct msc313e_pwm *priv, unsigned long low,
				    unsigned long high, u32 *value)
{
	unsigned int val = 0;

	val = readw(priv->base + low);
	*value = val;
	val = readb(priv->base + high) & GENMASK(2, 0);
	*value = (val << 16) | *value;
}

static int msc313e_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	struct msc313e_pwm *priv = to_msc313e_pwm(chip);
	unsigned long long nspertick = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, clk_get_rate(priv->clk));
	unsigned long long div = 1;

	/* Fit the period into the period register by prescaling the clk */
	while (DIV_ROUND_DOWN_ULL(period_ns, nspertick) > 0x3ffff) {
		div++;
		if (div > (0xffff + 1)) {
			/* Force clk div to the maximum allowed value */
			div = 0xffff;
			break;
		}
		nspertick = DIV_ROUND_DOWN_ULL(nspertick, div);
	}

	writeb(div - 1, priv->base + REG_DIV(pwm->hwpwm));
	msc313e_pwm_writecounter(priv, REG_DUTYL(pwm->hwpwm), REG_DUTYH(pwm->hwpwm),
				 DIV_ROUND_DOWN_ULL(duty_ns, nspertick));
	msc313e_pwm_writecounter(priv, REG_PERIODL(pwm->hwpwm), REG_PERIODH(pwm->hwpwm),
				 DIV_ROUND_DOWN_ULL(period_ns, nspertick));
	return 0;
};

static int msc313e_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				    enum pwm_polarity polarity)
{
	struct msc313e_pwm *priv = to_msc313e_pwm(chip);
	u8 reg;

	reg = readb(priv->base + REG_CTRL(pwm->hwpwm));
	if (polarity == PWM_POLARITY_INVERSED)
		reg |= POLARITY_EN_BIT;
	else
		reg &= ~POLARITY_EN_BIT;
	writeb(reg, priv->base + REG_CTRL(pwm->hwpwm));

	return 0;
}

static int msc313e_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct msc313e_pwm *priv = to_msc313e_pwm(chip);
	int ret;
	u8 reg;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;
	reg = readb(priv->base + REG_SWRST);
	reg &= ~BIT(pwm->hwpwm);
	writeb(reg, priv->base + REG_SWRST);
	return 0;
}

static void msc313e_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct msc313e_pwm *priv = to_msc313e_pwm(chip);
	u8 reg;

	reg = readb(priv->base + REG_SWRST);
	reg |= BIT(pwm->hwpwm);
	writeb(reg, priv->base + REG_SWRST);

	clk_disable_unprepare(priv->clk);
}

static int msc313e_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			 const struct pwm_state *state)
{
	int ret;

	if (state->enabled) {
		if (!pwm->state.enabled) {
			ret = msc313e_pwm_enable(chip, pwm);
			if (ret)
				return ret;
		}
		msc313e_pwm_set_polarity(chip, pwm, state->polarity);
		msc313e_pwm_config(chip, pwm, state->duty_cycle, state->period);
	} else if (pwm->state.enabled) {
		msc313e_pwm_disable(chip, pwm);
	}
	return 0;
}

static int msc313e_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			     struct pwm_state *state)
{
	struct msc313e_pwm *priv = to_msc313e_pwm(chip);
	unsigned long long nspertick = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, clk_get_rate(priv->clk));
	unsigned int val = 0;

	val = readl(priv->base + REG_CTRL(pwm->hwpwm)) & POLARITY_EN_BIT;
	state->polarity = val ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	val = readb(priv->base + REG_SWRST) & BIT(pwm->hwpwm);
	state->enabled = val == 0 ? true : false;

	msc313e_pwm_readcounter(priv, REG_DUTYL(pwm->hwpwm), REG_DUTYH(pwm->hwpwm), &val);
	state->duty_cycle = val * nspertick;

	msc313e_pwm_readcounter(priv, REG_PERIODL(pwm->hwpwm), REG_PERIODH(pwm->hwpwm), &val);
	state->period = val * nspertick;

	return 0;
}

static const struct pwm_ops msc313e_pwm_ops = {
	.apply = msc313e_apply,
	.get_state = msc313e_get_state,
};

static int msc313e_pwm_probe(struct platform_device *pdev)
{
	const struct msc313e_pwm_info *match_data;
	struct device *dev = &pdev->dev;
	struct msc313e_pwm *pwm;
	int ret;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	pwm = devm_kzalloc(dev, sizeof(struct msc313e_pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pwm->base))
		return PTR_ERR(pwm->base);

	pwm->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pwm->clk))
		return dev_err_probe(dev, PTR_ERR(pwm->clk), "Cannot get clk\n");

	ret = clk_prepare_enable(pwm->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot prepare enable clk\n");

	pwm->pwmchip.dev = dev;
	pwm->pwmchip.ops = &msc313e_pwm_ops;
	pwm->pwmchip.npwm = match_data->channels;

	platform_set_drvdata(pdev, pwm);

	return devm_pwmchip_add(dev, &pwm->pwmchip);
}

static const struct of_device_id msc313e_pwm_dt_ids[] = {
	{ .compatible = "mstar,msc313e-pwm", .data = &msc313e_data },
	{ .compatible = "mstar,ssd20xd-pwm", .data = &ssd20xd_data },
	{},
};
MODULE_DEVICE_TABLE(of, msc313e_pwm_dt_ids);

static struct platform_driver msc313e_pwm_driver = {
	.probe = msc313e_pwm_probe,
	.driver = {
		.name = "msc313e-pwm",
		.of_match_table = msc313e_pwm_dt_ids,
	},
};
module_platform_driver(msc313e_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mstar MSC313e PWM driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_AUTHOR("Romain Perier <romain.perier@gmail.com>");
