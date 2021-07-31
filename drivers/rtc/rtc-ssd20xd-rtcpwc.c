// SPDX-License-Identifier: GPL-2.0-only
/*
 * (C) 2021 Daniel Palmer
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/regmap.h>
#include <linux/pm.h>

#define REG_CTRL	0x0
#define REG_CTRL1	0x4
#define REG_WRDATA_L	0x10
#define REG_WRDATA_H	0x14
#define REG_ISOACK	0x20
#define REG_RDCNT_L	0x30
#define REG_RDCNT_H	0x34
#define REG_PWRCTRL	0x3c

static const struct reg_field base_wr_field = REG_FIELD(REG_CTRL, 1, 1);
static const struct reg_field base_rd_field = REG_FIELD(REG_CTRL, 2, 2);
static const struct reg_field cnt_rst_field = REG_FIELD(REG_CTRL, 3, 3);
static const struct reg_field alarm_wr_field = REG_FIELD(REG_CTRL, 4, 4);
static const struct reg_field sw0_wr_field = REG_FIELD(REG_CTRL, 5, 5);
static const struct reg_field sw1_wr_field = REG_FIELD(REG_CTRL, 6, 6);
static const struct reg_field sw0_rd_field = REG_FIELD(REG_CTRL, 7, 7);
static const struct reg_field sw1_rd_field = REG_FIELD(REG_CTRL, 8, 8);
static const struct reg_field cnt_rd_field = REG_FIELD(REG_CTRL1, 0, 0);
static const struct reg_field alarm_rd_field = REG_FIELD(REG_CTRL1, 1, 1);
static const struct reg_field alarm_en_field = REG_FIELD(REG_CTRL1, 2, 2);
static const struct reg_field int_clr_field = REG_FIELD(REG_CTRL1, 3, 3);
static const struct reg_field iso_ctrl_field = REG_FIELD(0xc, 0, 2);
static const struct reg_field wrdata_l_field = REG_FIELD(REG_WRDATA_L, 0, 15);
static const struct reg_field wrdata_h_field = REG_FIELD(REG_WRDATA_H, 0, 15);
static const struct reg_field alarm_is_en_field = REG_FIELD(REG_ISOACK, 0, 0);
static const struct reg_field iso_ctrl_ack_field = REG_FIELD(REG_ISOACK, 3, 3);
static const struct reg_field rddata_l_field = REG_FIELD(0x24, 0, 15);
static const struct reg_field rddata_h_field = REG_FIELD(0x28, 0, 15);
static const struct reg_field cnt_updating_field = REG_FIELD(0x2c, 0, 0);
static const struct reg_field rddata_cnt_l_field = REG_FIELD(REG_RDCNT_L, 0, 15);
static const struct reg_field rddata_cnt_h_field = REG_FIELD(REG_RDCNT_H, 0, 15);
static const struct reg_field cnt_rd_trig_field = REG_FIELD(0x38, 0, 0);
static const struct reg_field pwr_en_field = REG_FIELD(REG_PWRCTRL, 0, 0);
static const struct reg_field alarm_on_en_field = REG_FIELD(REG_PWRCTRL, 1, 1);
static const struct reg_field emgcy_off_en_field = REG_FIELD(REG_PWRCTRL, 2, 2);
static const struct reg_field rst_field = REG_FIELD(0x40, 8, 8);
static const struct reg_field iso_en_field = REG_FIELD(0x54, 0, 0);

#define MAGIC 0xbabe

struct ssd20xd_rtcpwc {
	struct rtc_device *rtc_dev;
	struct regmap_field *base_wr, *base_rd, *cnt_rst,
			    *sw0_wr, *sw1_wr, *sw0_rd, *sw1_rd, *cnt_rd;
	struct regmap_field *alarm_rd, *alarm_wr, *alarm_en, *alarm_is_en;
	struct regmap_field *iso_ctrl, *iso_ctrl_ack, *iso_en;
	struct regmap_field *wrdata_l, *wrdata_h;
	struct regmap_field *rddata_l, *rddata_h;
	struct regmap_field *cnt_updating, *rdcnt_l, *rdcnt_h, *rdcnt_trig;
	struct regmap_field *rst;

	struct regmap_field *pwr_en, *alarm_on_en, *emgcy_off_en;

	struct regmap_field *int_clr;
};

static int ssd20xd_rtcpwc_isoctrl(struct ssd20xd_rtcpwc *rtcpwc)
{
	static const unsigned int sequence[] = {
		0x1, 0x3, 0x7, 0x5, 0x1, 0x0,
	};
	unsigned int val;
	struct device *dev = &rtcpwc->rtc_dev->dev;
	int i, ret;

	/*
	 * The vendor code has this as part of the sequence
	 * but after testing this doesn't trigger a change in the
	 * ack bit. Instead the vendor code has a loop that breaks
	 * when ack is zero but the ack bit is zero until 0x1 is written
	 * so it's pointless. I think the intention here is to reset the
	 * register to zero and it isn't actually part of the
	 * sequence.
	 */
	regmap_field_force_write(rtcpwc->iso_ctrl, 0);

	for(i = 0; i < ARRAY_SIZE(sequence); i++) {
		unsigned int ack;
		/*
		 * The ack bit is seems to be toggled by whatever is actually
		 * handling the operation so we need to know what the original
		 * value was to check if it got inverted.
		 */
		regmap_field_read(rtcpwc->iso_ctrl_ack, &ack);

		regmap_field_force_write(rtcpwc->iso_ctrl, sequence[i]);
		/*
		 * No idea what the correct values are for this.
		 * Vendor code sleeps for 100us per loop and loops up
		 * to 20 times.
		 */
		ret = regmap_field_read_poll_timeout(rtcpwc->iso_ctrl_ack, val, val != ack, 100, 20 * 100);
		if(ret) {
			dev_err(dev, "Timeout waiting for ack byte %i (%x) of sequence\n", i, sequence[i]);
			return ret;
		}
	}

	/*
	 * Same deal as above. No idea of the right values here.
	 * Vendor code loops 22 times with 100us delay.
	 */
	ret = regmap_field_read_poll_timeout(rtcpwc->iso_en, val, val, 100, 22 * 100);
	if(ret)
		dev_err(dev, "Timeout waiting for iso en\n");

	/* Again, no idea if this is needed */
	mdelay(20);

	return 0;
}


static int ssd20xd_rtcpwc_read_reg(struct ssd20xd_rtcpwc *priv, struct regmap_field *rdbit, unsigned int *base)
{
	unsigned l, h;

	regmap_field_write(rdbit, 1);
	ssd20xd_rtcpwc_isoctrl(priv);
	regmap_field_write(rdbit, 0);

	regmap_field_read(priv->rddata_l, &l);
	regmap_field_read(priv->rddata_h, &h);

	*base = (h << 16) | l;

	return 0;
}

static int ssd20xd_rtcpwc_write_reg(struct ssd20xd_rtcpwc* priv, struct regmap_field *wrbit, u32 base)
{
	regmap_field_write(wrbit, 1);
	regmap_field_write(priv->wrdata_l, base);
	regmap_field_write(priv->wrdata_h, base >> 16);
	ssd20xd_rtcpwc_isoctrl(priv);
	regmap_field_write(wrbit, 0);

	return 0;
}

static int ssd20xd_rtcpwc_read_counter(struct ssd20xd_rtcpwc *priv, unsigned int *counter)
{
	unsigned int updating, l, h;
	int ret;

	regmap_field_write(priv->cnt_rd, 1);
	ssd20xd_rtcpwc_isoctrl(priv);
	regmap_field_write(priv->cnt_rd, 0);

	regmap_field_write(priv->rdcnt_trig, 1);

	/*
	 * If the rtc isn't running for some reason the updating bit never clears
	 * and a deadlock happens. So only let this spin for 1s at most.
	 */
	ret = regmap_field_read_poll_timeout(priv->cnt_updating, updating, !updating, 0, 1000000);
	if(ret)
		return ret;

	regmap_field_write(priv->rdcnt_trig, 0);

	regmap_field_read(priv->rdcnt_l, &l);
	regmap_field_read(priv->rdcnt_h, &h);

	*counter = (h << 16) | l;

	//printk("counter --- %x\n", *counter);

	return 0;
}

static int ssd20xd_rtcpwc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ssd20xd_rtcpwc *priv = dev_get_drvdata(dev);
	unsigned int sw0, base, counter;
	u32 seconds;

	ssd20xd_rtcpwc_read_reg(priv, priv->sw0_rd, &sw0);
	if (sw0 != MAGIC)
		return -EINVAL;

	ssd20xd_rtcpwc_read_reg(priv, priv->base_rd, &base);
	ssd20xd_rtcpwc_read_counter(priv, &counter);

	seconds = base + counter;

	rtc_time64_to_tm(seconds, tm);

	return 0;
}

static int ssd20xd_rtcpwc_reset_counter(struct ssd20xd_rtcpwc* priv)
{
	regmap_field_write(priv->cnt_rst, 1);
	ssd20xd_rtcpwc_isoctrl(priv);
	regmap_field_write(priv->cnt_rst, 0);
	ssd20xd_rtcpwc_isoctrl(priv);

	return 0;
}

static int ssd20xd_rtcpwc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct ssd20xd_rtcpwc *priv = dev_get_drvdata(dev);
	unsigned long seconds = rtc_tm_to_time64(tm);

	ssd20xd_rtcpwc_write_reg(priv, priv->base_wr, seconds);
	ssd20xd_rtcpwc_reset_counter(priv);
	ssd20xd_rtcpwc_write_reg(priv, priv->sw0_wr, MAGIC);

	return 0;
}

static int ssd20xd_rtcpwc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct ssd20xd_rtcpwc *priv = dev_get_drvdata(dev);
	unsigned int base, alarm, alarm_is_en;
	u32 seconds;

	ssd20xd_rtcpwc_read_reg(priv, priv->base_rd, &base);
	ssd20xd_rtcpwc_read_reg(priv, priv->alarm_rd, &alarm);

	seconds = base + alarm;

	rtc_time64_to_tm(seconds, &wkalrm->time);

	regmap_field_read(priv->alarm_is_en, &alarm_is_en);
	wkalrm->enabled = alarm_is_en;

	printk("read alarm %d\n", wkalrm->enabled);

	return 0;
}

static void ssd20xd_rtcpwc_clrirq(struct ssd20xd_rtcpwc *priv)
{
	regmap_field_write(priv->int_clr, 1);
	ssd20xd_rtcpwc_isoctrl(priv);
	regmap_field_write(priv->int_clr, 0);
}

static int ssd20xd_rtcpwc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct ssd20xd_rtcpwc *priv = dev_get_drvdata(dev);
	unsigned long seconds = rtc_tm_to_time64(&wkalrm->time);
	unsigned int base;

	ssd20xd_rtcpwc_read_reg(priv, priv->base_rd, &base);
	ssd20xd_rtcpwc_write_reg(priv, priv->alarm_wr, seconds - base);

	printk("set alarm %d\n", wkalrm->enabled);

	ssd20xd_rtcpwc_clrirq(priv);

	regmap_field_write(priv->alarm_en, wkalrm->enabled);
	ssd20xd_rtcpwc_isoctrl(priv);

	return 0;
}

static const struct rtc_class_ops ssd20xd_rtcpwc_ops = {
	.read_time = ssd20xd_rtcpwc_read_time,
	.set_time = ssd20xd_rtcpwc_set_time,
	.read_alarm = ssd20xd_rtcpwc_read_alarm,
	.set_alarm = ssd20xd_rtcpwc_set_alarm,
};

static struct ssd20xd_rtcpwc *poweroff_rtc = NULL;

static void ssd20xd_rtcpwc_poweroff(void)
{
	pr_info("Powering off via rtcpwc\n");

	regmap_field_write(poweroff_rtc->pwr_en, 0);
	ssd20xd_rtcpwc_isoctrl(poweroff_rtc);

	while(1) {
		pr_info(".\n");
	}
}

static const struct regmap_config ssd20xd_rtcpwc_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static irqreturn_t ssd20xd_rtcpwc_alarmirq(int irq, void *id)
{
	struct ssd20xd_rtcpwc *priv = (struct ssd20xd_rtcpwc*)id;

	ssd20xd_rtcpwc_clrirq(priv);

	return IRQ_HANDLED;
}

static int ssd20xd_rtcpwc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssd20xd_rtcpwc *priv;
	struct regmap *regmap;
	void __iomem *base;
	int ret, i, irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct ssd20xd_rtcpwc), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rtc_dev = devm_rtc_allocate_device(dev);
	if (IS_ERR(priv->rtc_dev))
		return PTR_ERR(priv->rtc_dev);

	priv->rtc_dev->ops = &ssd20xd_rtcpwc_ops;
	priv->rtc_dev->range_max = U32_MAX;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &ssd20xd_rtcpwc_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	struct {
			struct reg_field f;
			struct regmap_field **ff;
		} fields[] = {
		{ base_wr_field, &priv->base_wr },
		{ base_rd_field, &priv->base_rd },
		{ alarm_wr_field, &priv->alarm_wr },
		{ alarm_rd_field, &priv->alarm_rd },
		{ alarm_en_field, &priv->alarm_en },
		{ alarm_is_en_field, &priv->alarm_is_en },
		{ pwr_en_field, &priv->pwr_en },
		{ alarm_on_en_field, &priv->alarm_on_en },
		{ emgcy_off_en_field, &priv->emgcy_off_en },
		{ int_clr_field, &priv->int_clr },
	};

	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		struct regmap_field **ff = fields[i].ff;
		*ff = devm_regmap_field_alloc(dev, regmap, fields[i].f);
		if(IS_ERR(*ff))
			return PTR_ERR(*ff);
	}

	priv->cnt_rst = devm_regmap_field_alloc(dev, regmap, cnt_rst_field);
	if(IS_ERR(priv->cnt_rst))
		return PTR_ERR(priv->cnt_rst);

	priv->sw0_wr = devm_regmap_field_alloc(dev, regmap, sw0_wr_field);
	if(IS_ERR(priv->sw0_wr))
		return PTR_ERR(priv->sw0_wr);

	priv->sw1_wr = devm_regmap_field_alloc(dev, regmap, sw1_wr_field);
	if(IS_ERR(priv->sw1_wr))
		return PTR_ERR(priv->sw1_wr);

	priv->sw0_rd = devm_regmap_field_alloc(dev, regmap, sw0_rd_field);
	if(IS_ERR(priv->sw0_rd))
		return PTR_ERR(priv->sw0_rd);

	priv->sw1_rd = devm_regmap_field_alloc(dev, regmap, sw1_rd_field);
	if(IS_ERR(priv->sw1_rd))
		return PTR_ERR(priv->sw1_rd);

	priv->cnt_rd = devm_regmap_field_alloc(dev, regmap, cnt_rd_field);
	if(IS_ERR(priv->cnt_rd))
		return PTR_ERR(priv->cnt_rd);

	priv->iso_ctrl = devm_regmap_field_alloc(dev, regmap, iso_ctrl_field);
	if(IS_ERR(priv->iso_ctrl))
		return PTR_ERR(priv->iso_ctrl);

	priv->iso_ctrl_ack = devm_regmap_field_alloc(dev, regmap, iso_ctrl_ack_field);
	if(IS_ERR(priv->iso_ctrl_ack))
		return PTR_ERR(priv->iso_ctrl_ack);

	priv->iso_en = devm_regmap_field_alloc(dev, regmap, iso_en_field);
	if(IS_ERR(priv->iso_en))
		return PTR_ERR(priv->iso_en);

	priv->wrdata_l = devm_regmap_field_alloc(dev, regmap, wrdata_l_field);
	if(IS_ERR(priv->wrdata_l))
		return PTR_ERR(priv->wrdata_l);

	priv->wrdata_h = devm_regmap_field_alloc(dev, regmap, wrdata_h_field);
	if(IS_ERR(priv->wrdata_h))
		return PTR_ERR(priv->wrdata_h);

	priv->rddata_l = devm_regmap_field_alloc(dev, regmap, rddata_l_field);
	if(IS_ERR(priv->rddata_l))
		return PTR_ERR(priv->rddata_l);

	priv->rddata_h = devm_regmap_field_alloc(dev, regmap, rddata_h_field);
	if(IS_ERR(priv->rddata_h))
		return PTR_ERR(priv->rddata_h);

	priv->cnt_updating = devm_regmap_field_alloc(dev, regmap, cnt_updating_field);
	if(IS_ERR(priv->cnt_updating))
		return PTR_ERR(priv->cnt_updating);

	priv->rdcnt_l = devm_regmap_field_alloc(dev, regmap, rddata_cnt_l_field);
	if(IS_ERR(priv->rdcnt_l))
		return PTR_ERR(priv->rdcnt_l);

	priv->rdcnt_h = devm_regmap_field_alloc(dev, regmap, rddata_cnt_h_field);
	if(IS_ERR(priv->rdcnt_h))
		return PTR_ERR(priv->rdcnt_h);

	priv->rdcnt_trig = devm_regmap_field_alloc(dev, regmap, cnt_rd_trig_field);
	if(IS_ERR(priv->rdcnt_trig))
		return PTR_ERR(priv->rdcnt_trig);

	priv->rst = devm_regmap_field_alloc(dev, regmap, rst_field);

	platform_set_drvdata(pdev, priv);

	device_set_wakeup_capable(dev, true);

	ssd20xd_rtcpwc_clrirq(priv);

	irq = platform_get_irq(pdev, 0);
	if (irq == 0)
		return -ENODEV;
	ret = devm_request_irq(&pdev->dev, irq, ssd20xd_rtcpwc_alarmirq,
			       IRQF_SHARED, dev_name(&pdev->dev), priv);
	if (ret)
		return ret;

	ret = devm_rtc_register_device(priv->rtc_dev);
	if (ret)
		return ret;

	if (pm_power_off) {
		dev_err(&pdev->dev, "pm_power_off already claimed for %ps",
			pm_power_off);
		//return -EBUSY;
	}
	else {
		poweroff_rtc = priv;
		pm_power_off = ssd20xd_rtcpwc_poweroff;
	}

	return 0;
}

static const struct of_device_id ssd20xd_rtcpwc_of_match_table[] = {
	{ .compatible = "sstar,ssd20xd-rtcpwc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ms_rtc_of_match_table);

static struct platform_driver ssd20xd_rtcpwc_driver = {
	.probe = ssd20xd_rtcpwc_probe,
	.driver = {
		.name = "ssd20xd-rtcpwc",
		.of_match_table = ssd20xd_rtcpwc_of_match_table,
	},
};
module_platform_driver(ssd20xd_rtcpwc_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("MStar RTC PWC Driver");
MODULE_LICENSE("GPL v2");
