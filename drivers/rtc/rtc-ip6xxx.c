// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Injoinic IP6XXX regulators driver.
 *
 * Copyright (C) 2020 <daniel@thingy.jp>
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mfd/ip6xxx.h>

#define DRIVER_NAME	"ip6xxx-rtc"

static int ip6xxx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(dev);
	unsigned reg;

	regmap_read(ip6xxx->regmap, IP6303_RTC_SEC, &reg);
	tm->tm_sec = reg;
	regmap_read(ip6xxx->regmap, IP6303_RTC_MIN, &reg);
	tm->tm_min = reg;
	regmap_read(ip6xxx->regmap, IP6303_RTC_HOUR, &reg);
	tm->tm_hour = reg;
	regmap_read(ip6xxx->regmap, IP6303_RTC_DATE, &reg);
	tm->tm_mday = reg;
	regmap_field_read(ip6xxx->rtc_wday, &reg);
	tm->tm_wday = reg;
	regmap_field_read(ip6xxx->rtc_mon, &reg);
	tm->tm_mon = reg;
	regmap_field_read(ip6xxx->rtc_year, &reg);
	tm->tm_year = reg + 100;

	return rtc_valid_tm(tm);
}

static int ip6xxx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(dev);

	regmap_write(ip6xxx->regmap, IP6303_RTC_SEC, tm->tm_sec);
	regmap_write(ip6xxx->regmap, IP6303_RTC_MIN, tm->tm_min);
	regmap_write(ip6xxx->regmap, IP6303_RTC_HOUR, tm->tm_hour);
	regmap_write(ip6xxx->regmap, IP6303_RTC_DATE, tm->tm_mday);
	regmap_field_write(ip6xxx->rtc_wday, tm->tm_wday);
	regmap_field_write(ip6xxx->rtc_mon, tm->tm_mon);
	regmap_field_write(ip6xxx->rtc_year, tm->tm_year - 100);

	return 0;
}

static const struct rtc_class_ops ip6xxx_rtc_ops = {
	.read_time = ip6xxx_rtc_read_time,
	.set_time = ip6xxx_rtc_set_time,
};

static int ip6xxx_rtc_probe(struct platform_device *pdev)
{
	struct ip6xxx *ip6xxx = dev_get_drvdata(pdev->dev.parent);

	ip6xxx->rtc_wday = devm_regmap_field_alloc(pdev->dev.parent,
		ip6xxx->regmap, ip6303_rtc_wday);

	ip6xxx->rtc_mon = devm_regmap_field_alloc(pdev->dev.parent,
		ip6xxx->regmap, ip6303_rtc_mon);

	ip6xxx->rtc_year = devm_regmap_field_alloc(pdev->dev.parent,
		ip6xxx->regmap, ip6303_rtc_year);

	ip6xxx->rtc_dev = devm_rtc_device_register(pdev->dev.parent,
			dev_name(pdev->dev.parent), &ip6xxx_rtc_ops,
			THIS_MODULE);

	if (IS_ERR(ip6xxx->rtc_dev)) {
		return PTR_ERR(ip6xxx->rtc_dev);
	}

	return 0;
}

static struct platform_driver ip6xxx_rtc_driver = {
	.probe	= ip6xxx_rtc_probe,
	.driver	= {
		.name = "ip6xxx-rtc",
	},
};

module_platform_driver(ip6xxx_rtc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("RTC Driver for IP6XXX PMIC");
MODULE_ALIAS("platform:ip6xxx-rtc");
