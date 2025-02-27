// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 * Copyright (C) 2022 Romain Perier <romain.perier@gmail.com>
 */

#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define DRIVER_NAME "msc313e-pwm"

#define CHANNEL_OFFSET	0x80
#define REG_DUTY	0x8
#define REG_PERIOD	0x10
#define REG_DIV		0x18
#define REG_CTRL	0x1c
#define REG_SWRST	0x1fc

struct msc313e_pwm_channel {
	struct regmap_field *clkdiv;
	struct regmap_field *polarity;
	struct regmap_field *dutyl;
	struct regmap_field *dutyh;
	struct regmap_field *periodl;
	struct regmap_field *periodh;
	struct regmap_field *swrst;
};

struct msc313e_pwm {
	struct regmap *regmap;
	struct pwm_chip pwmchip;
	struct clk *clk;
	struct msc313e_pwm_channel channels[];
};

struct msc313e_pwm_info {
	unsigned int channels;
};

#define to_msc313e_pwm(ptr) container_of(ptr, struct msc313e_pwm, pwmchip)

static const struct regmap_config msc313e_pwm_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static const struct msc313e_pwm_info msc313e_data = {
	.channels = 8,
};

static const struct msc313e_pwm_info ssd20xd_data = {
	.channels = 4,
};

static void msc313e_pwm_writecounter(struct regmap_field *low, struct regmap_field *high, u32 value)
{
	/* Each counter is divided into two 32 registers, one for the high part, another the low
	 * part. However we only need to update 16 bits in each of these
	 */
	regmap_field_write(low, value & 0xffff);
	regmap_field_write(high, value >> 16);
}

static void msc313e_pwm_readcounter(struct regmap_field *low, struct regmap_field *high, u32 *value)
{
	unsigned int val = 0;

	regmap_field_read(low, &val);
	*value = val;
	regmap_field_read(high, &val);
	*value = (val << 16) | *value;
}

static int msc313e_pwm_config(struct pwm_chip *chip, struct pwm_device *device,
			      int duty_ns, int period_ns)
{
	struct msc313e_pwm *pwm = to_msc313e_pwm(chip);
	unsigned long long nspertick = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, clk_get_rate(pwm->clk));
	struct msc313e_pwm_channel *channel = &pwm->channels[device->hwpwm];
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

	regmap_field_write(channel->clkdiv, div - 1);
	msc313e_pwm_writecounter(channel->dutyl, channel->dutyh,
				 DIV_ROUND_DOWN_ULL(duty_ns, nspertick));
	msc313e_pwm_writecounter(channel->periodl, channel->periodh,
				 DIV_ROUND_DOWN_ULL(period_ns, nspertick));
	return 0;
};

static int msc313e_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *device,
				    enum pwm_polarity polarity)
{
	struct msc313e_pwm *pwm = to_msc313e_pwm(chip);
	struct msc313e_pwm_channel *channel = &pwm->channels[device->hwpwm];
	unsigned int pol = 0;

	if (polarity == PWM_POLARITY_INVERSED)
		pol = 1;
	regmap_field_update_bits(channel->polarity, 1, pol);

	return 0;
}

static int msc313e_pwm_enable(struct pwm_chip *chip, struct pwm_device *device)
{
	struct msc313e_pwm *pwm = to_msc313e_pwm(chip);
	struct msc313e_pwm_channel *channel = &pwm->channels[device->hwpwm];
	int ret;

	ret = clk_prepare_enable(pwm->clk);
	if (ret)
		return ret;
	return regmap_field_write(channel->swrst, 0);
}

static int msc313e_pwm_disable(struct pwm_chip *chip, struct pwm_device *device)
{
	struct msc313e_pwm *pwm = to_msc313e_pwm(chip);
	struct msc313e_pwm_channel *channel = &pwm->channels[device->hwpwm];
	int ret;

	ret = regmap_field_write(channel->swrst, 1);
	clk_disable_unprepare(pwm->clk);
	return ret;
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
		ret = msc313e_pwm_disable(chip, pwm);
	}
	return 0;
}

static int msc313e_get_state(struct pwm_chip *chip, struct pwm_device *device,
			      struct pwm_state *state)
{
	struct msc313e_pwm *pwm = to_msc313e_pwm(chip);
	struct msc313e_pwm_channel *channel = &pwm->channels[device->hwpwm];
	unsigned long long nspertick = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, clk_get_rate(pwm->clk));
	unsigned int val = 0;

	regmap_field_read(channel->polarity, &val);
	state->polarity = val ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	regmap_field_read(channel->swrst, &val);
	state->enabled = val == 0 ? true : false;

	msc313e_pwm_readcounter(channel->dutyl, channel->dutyh, &val);
	state->duty_cycle = val * nspertick;

	msc313e_pwm_readcounter(channel->periodl, channel->periodh, &val);
	state->period = val * nspertick;

	return 0;
}

static const struct pwm_ops msc313e_pwm_ops = {
	.apply = msc313e_apply,
	.get_state = msc313e_get_state,
	.owner = THIS_MODULE
};

static int msc313e_pwm_probe(struct platform_device *pdev)
{
	const struct msc313e_pwm_info *match_data;
	struct device *dev = &pdev->dev;
	struct msc313e_pwm *pwm;
	__iomem void *base;
	int i;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pwm = devm_kzalloc(dev, struct_size(pwm, channels, match_data->channels), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pwm->clk))
		return dev_err_probe(dev, PTR_ERR(pwm->clk), "Cannot get clk\n");

	pwm->regmap = devm_regmap_init_mmio(dev, base, &msc313e_pwm_regmap_config);
	if (IS_ERR(pwm->regmap))
		return dev_err_probe(dev, PTR_ERR(pwm->regmap), "Cannot get regmap\n");

	for (i = 0; i < match_data->channels; i++) {
		unsigned int offset = CHANNEL_OFFSET * i;
		struct reg_field div_clkdiv_field = REG_FIELD(offset + REG_DIV, 0, 7);
		struct reg_field ctrl_polarity_field = REG_FIELD(offset + REG_CTRL, 4, 4);
		struct reg_field dutyl_field = REG_FIELD(offset + REG_DUTY, 0, 15);
		struct reg_field dutyh_field = REG_FIELD(offset + REG_DUTY + 4, 0, 2);
		struct reg_field periodl_field = REG_FIELD(offset + REG_PERIOD, 0, 15);
		struct reg_field periodh_field = REG_FIELD(offset + REG_PERIOD + 4, 0, 2);
		struct reg_field swrst_field = REG_FIELD(REG_SWRST, i, i);

		pwm->channels[i].clkdiv = devm_regmap_field_alloc(dev, pwm->regmap,
								  div_clkdiv_field);
		pwm->channels[i].polarity = devm_regmap_field_alloc(dev, pwm->regmap,
								    ctrl_polarity_field);
		pwm->channels[i].dutyl = devm_regmap_field_alloc(dev, pwm->regmap, dutyl_field);
		pwm->channels[i].dutyh = devm_regmap_field_alloc(dev, pwm->regmap, dutyh_field);
		pwm->channels[i].periodl = devm_regmap_field_alloc(dev, pwm->regmap, periodl_field);
		pwm->channels[i].periodh = devm_regmap_field_alloc(dev, pwm->regmap, periodh_field);
		pwm->channels[i].swrst = devm_regmap_field_alloc(dev, pwm->regmap, swrst_field);

		/* Channels are enabled on boot, disable it until the pwm subsystem re-enable it
		 * explicitly
		 */
		regmap_field_write(pwm->channels[i].swrst, 1);
	}

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
		.name = DRIVER_NAME,
		.of_match_table = msc313e_pwm_dt_ids,
	},
};
module_platform_driver(msc313e_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mstar MSC313e PWM driver");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
