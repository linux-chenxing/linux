// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/iio/iio.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "../../pinctrl/core.h"
#include "../../pinctrl/pinconf.h"
#include "../../pinctrl/pinmux.h"

#define DRIVER_NAME "msc313e-sar"

#define SAR_MAX_PINS		4

#define REG_CTRL		0x0
#define REG_SAMPLE_PERIOD	0x4
#define REG_GPIO_CTRL		0x44
#define REG_GPIO_DATA		0x48
#define REG_INT_MASK		0x50
#define REG_INT_CLR		0x54
#define REG_INT_FORCE		0x58
#define REG_INT_STATUS		0x5c
#define REG_VREF_SEL		0x64
#define REG_CH1_UPB		0x80
#define REG_CH1_LOB		0xc0

#define REG_PMSLEEP_PD		0xbc
#define REG_PMSLEEP_TS_PD	BIT(2)

static struct reg_field ctrl_load_field = REG_FIELD(REG_CTRL, 14, 14);
static struct reg_field ctrl_freerun_field = REG_FIELD(REG_CTRL, 9, 9);
static struct reg_field ctrl_analogpd_field = REG_FIELD(REG_CTRL, 8, 8);
static struct reg_field ctrl_start_field = REG_FIELD(REG_CTRL, 7, 7);
static struct reg_field ctrl_digitalpd_field = REG_FIELD(REG_CTRL, 6, 6);
static struct reg_field ctrl_mode_field = REG_FIELD(REG_CTRL, 5, 5);
static struct reg_field ctrl_singlech_field = REG_FIELD(REG_CTRL, 4, 4);
static struct reg_field ctrl_channel_field = REG_FIELD(REG_CTRL, 0, 2);

static struct reg_field gpio_ctrl_en_field = REG_FIELD(REG_GPIO_CTRL, 0, 3);
static struct reg_field gpio_ctrl_oen_field = REG_FIELD(REG_GPIO_CTRL, 8, 11);
static struct reg_field gpio_data_value_field = REG_FIELD(REG_GPIO_DATA, 0, 3);
static struct reg_field gpio_data_in_field = REG_FIELD(REG_GPIO_DATA, 8, 11);
static struct reg_field vref_ts_field = REG_FIELD(REG_VREF_SEL, 6, 6);

/* common */
#define PINNAME_SAR_GPIO3	"sar_gpio3"
#define PINNAME_SAR_GPIO2	"sar_gpio2"
#define PINNAME_SAR_GPIO1	"sar_gpio1"
#define PINNAME_SAR_GPIO0	"sar_gpio0"

#define FUNCTION_SAR_3		"sar3"
#define FUNCTION_SAR_2		"sar2"
#define FUNCTION_SAR_1		"sar1"
#define FUNCTION_SAR_0		"sar0"

struct sar_pinctrl_function {
	const char* name;
	const char* group;
};

#define SAR_PINCTRL_FUNCTION(n) { .name = FUNCTION_SAR_##n, .group = PINNAME_SAR_GPIO##n }

static const struct sar_pinctrl_function sar_pinctrl_functions[] = {
	SAR_PINCTRL_FUNCTION(3),
	SAR_PINCTRL_FUNCTION(2),
	SAR_PINCTRL_FUNCTION(1),
	SAR_PINCTRL_FUNCTION(0),
};

struct sar_pinctrl_group {
	const char* name;
	const int pin;
};

#define SAR_PINCTRL_GROUP(n, p) { .name = n, .pin = p }

struct mstar_sar_info {
	const struct pinctrl_pin_desc *pins;
	unsigned npins;
	const struct sar_pinctrl_group *groups;
	unsigned ngroups;
	const unsigned *rangepins;
	unsigned nrangepins;
};
/* */

#ifdef CONFIG_MACH_INFINITY
/* msc313 and msc313e */
#define PIN_MSC313_SAR_GPIO3	9
#define PIN_MSC313_SAR_GPIO2	10
#define PIN_MSC313_SAR_GPIO1	11
#define PIN_MSC313_SAR_GPIO0	12

#define MSC313_PIN(n) \
	PINCTRL_PIN(PIN_MSC313_SAR_GPIO##n, PINNAME_SAR_GPIO##n)

static struct pinctrl_pin_desc msc313_sar_pins[] = {
	MSC313_PIN(3),
	MSC313_PIN(2),
	MSC313_PIN(1),
	MSC313_PIN(0),
};

#define MSC313_SAR_PINCTRL_GROUP(n) \
	SAR_PINCTRL_GROUP(PINNAME_SAR_GPIO##n, PIN_MSC313_SAR_GPIO##n)

static const struct sar_pinctrl_group msc313_sar_pinctrl_groups[] = {
	MSC313_SAR_PINCTRL_GROUP(3),
	MSC313_SAR_PINCTRL_GROUP(2),
	MSC313_SAR_PINCTRL_GROUP(1),
	MSC313_SAR_PINCTRL_GROUP(0),
};

/* pin and gpio order are reversed */
static const unsigned msc313e_rangepins[] = {
	PIN_MSC313_SAR_GPIO0,
	PIN_MSC313_SAR_GPIO1,
	PIN_MSC313_SAR_GPIO2,
	PIN_MSC313_SAR_GPIO3,
};

static const struct mstar_sar_info msc313_info = {
	.pins = msc313_sar_pins,
	.npins = ARRAY_SIZE(msc313_sar_pins),
	.groups = msc313_sar_pinctrl_groups,
	.ngroups = ARRAY_SIZE(msc313_sar_pinctrl_groups),
	.rangepins = msc313e_rangepins,
	.nrangepins = ARRAY_SIZE(msc313e_rangepins),
};

/* SSD201 and SS202 */

#define PIN_SSD20X_SAR_GPIO0 22
#define PIN_SSD20X_SAR_GPIO1 21
#define PIN_SSD20X_SAR_GPIO2 20

#define SSD20X_PIN(n) \
	PINCTRL_PIN(PIN_SSD20X_SAR_GPIO##n, PINNAME_SAR_GPIO##n)

static struct pinctrl_pin_desc ssd20x_sar_pins[] = {
	SSD20X_PIN(2),
	SSD20X_PIN(1),
	SSD20X_PIN(0),
};

#define SSD20X_SAR_PINCTRL_GROUP(n) \
	SAR_PINCTRL_GROUP(PINNAME_SAR_GPIO##n, PIN_SSD20X_SAR_GPIO##n)

static const struct sar_pinctrl_group ssd20x_sar_pinctrl_groups[] = {
	SSD20X_SAR_PINCTRL_GROUP(2),
	SSD20X_SAR_PINCTRL_GROUP(1),
	SSD20X_SAR_PINCTRL_GROUP(0),
};

/* pin and gpio order are reversed */
static const unsigned ssd20x_rangepins[] = {
		PIN_SSD20X_SAR_GPIO0,
		PIN_SSD20X_SAR_GPIO1,
		PIN_SSD20X_SAR_GPIO2,
};

static const struct mstar_sar_info ssd20x_info = {
	.pins = ssd20x_sar_pins,
	.npins = ARRAY_SIZE(ssd20x_sar_pins),
	.groups = ssd20x_sar_pinctrl_groups,
	.ngroups = ARRAY_SIZE(ssd20x_sar_pinctrl_groups),
	.rangepins = ssd20x_rangepins,
	.nrangepins = ARRAY_SIZE(ssd20x_rangepins),
};
#endif /* infinity */

#ifdef CONFIG_MACH_PIONEER3
/* ssd210 */
#define PIN_SSD210_SAR_GPIO0	62
#define PIN_SSD210_SAR_GPIO1	61
#define PIN_SSD210_SAR_GPIO2	60

#define SSD210_PIN(n) PINCTRL_PIN(PIN_SSD210_SAR_GPIO##n, PINNAME_SAR_GPIO##n)

static struct pinctrl_pin_desc ssd210_sar_pins[] = {
	SSD210_PIN(0),
	SSD210_PIN(1),
	SSD210_PIN(2),
};

#define SSD210_SAR_PINCTRL_GROUP(n) SAR_PINCTRL_GROUP(PINNAME_SAR_GPIO##n, PIN_SSD210_SAR_GPIO##n)

static const struct sar_pinctrl_group ssd210_sar_pinctrl_groups[] = {
	SSD210_SAR_PINCTRL_GROUP(0),
	SSD210_SAR_PINCTRL_GROUP(1),
	SSD210_SAR_PINCTRL_GROUP(2),
};

static const unsigned ssd210_rangepins[] = {
	PIN_SSD210_SAR_GPIO0,
	PIN_SSD210_SAR_GPIO1,
	PIN_SSD210_SAR_GPIO2,
};

static const struct mstar_sar_info ssd210_info = {
	.pins = ssd210_sar_pins,
	.npins = ARRAY_SIZE(ssd210_sar_pins),
	.groups = ssd210_sar_pinctrl_groups,
	.ngroups = ARRAY_SIZE(ssd210_sar_pinctrl_groups),
	.rangepins = ssd210_rangepins,
	.nrangepins = ARRAY_SIZE(ssd210_rangepins),
};

/* ssd212 */
#define PIN_SSD212_SAR_GPIO0	125
#define PIN_SSD212_SAR_GPIO1	124

#define SSD212_PIN(n) PINCTRL_PIN(PIN_SSD212_SAR_GPIO##n, PINNAME_SAR_GPIO##n)

static struct pinctrl_pin_desc ssd212_sar_pins[] = {
	SSD212_PIN(0),
	SSD212_PIN(1),
};

#define SSD212_SAR_PINCTRL_GROUP(n) SAR_PINCTRL_GROUP(PINNAME_SAR_GPIO##n, PIN_SSD212_SAR_GPIO##n)

static const struct sar_pinctrl_group ssd212_sar_pinctrl_groups[] = {
	SSD212_SAR_PINCTRL_GROUP(0),
	SSD212_SAR_PINCTRL_GROUP(1),
};

static const unsigned ssd212_rangepins[] = {
	PIN_SSD212_SAR_GPIO0,
	PIN_SSD212_SAR_GPIO1,
};

static const struct mstar_sar_info ssd212_info = {
	.pins = ssd212_sar_pins,
	.npins = ARRAY_SIZE(ssd212_sar_pins),
	.groups = ssd212_sar_pinctrl_groups,
	.ngroups = ARRAY_SIZE(ssd212_sar_pinctrl_groups),
	.rangepins = ssd212_rangepins,
	.nrangepins = ARRAY_SIZE(ssd212_rangepins),
};

#endif /* pioneer3 */

#ifdef CONFIG_MACH_MERCURY
/* ssc8336 */
#define PIN_SSC8336_SAR_GPIO0	24
#define PIN_SSC8336_SAR_GPIO1	25
#define PIN_SSC8336_SAR_GPIO3	26

#define SSC8336_PIN(n) PINCTRL_PIN(PIN_SSC8336_SAR_GPIO##n, PINNAME_SAR_GPIO##n)

static struct pinctrl_pin_desc ssc8336_sar_pins[] = {
	SSC8336_PIN(0),
	SSC8336_PIN(1),
	SSC8336_PIN(3),
};

#define SSC8336_SAR_PINCTRL_GROUP(n) SAR_PINCTRL_GROUP(PINNAME_SAR_GPIO##n, PIN_SSC8336_SAR_GPIO##n)

static const struct sar_pinctrl_group ssc8336_sar_pinctrl_groups[] = {
	SSC8336_SAR_PINCTRL_GROUP(0),
	SSC8336_SAR_PINCTRL_GROUP(1),
	SSC8336_SAR_PINCTRL_GROUP(3),
};

static const unsigned ssc8336_rangepins[] = {
	PIN_SSC8336_SAR_GPIO0,
	PIN_SSC8336_SAR_GPIO1,
	PIN_SSC8336_SAR_GPIO3,
};

static const struct mstar_sar_info ssc8336_info = {
	.pins = ssc8336_sar_pins,
	.npins = ARRAY_SIZE(ssc8336_sar_pins),
	.groups = ssc8336_sar_pinctrl_groups,
	.ngroups = ARRAY_SIZE(ssc8336_sar_pinctrl_groups),
	.rangepins = ssc8336_rangepins,
	.nrangepins = ARRAY_SIZE(ssc8336_rangepins),
};
#endif /* mercury */

static const struct regmap_config msc313_sar_regmap_config = {
	.name = "msc313-sar",
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

struct msc313e_sar {
	struct mstar_sar_info *info;
	struct regmap* regmap;
	struct clk *clk;
	struct gpio_chip gpiochip;
	struct pinctrl_desc pinctrl_desc;
	struct pinctrl_dev *pinctrl_dev;
	struct pinctrl_gpio_range gpio_range;

	struct regmap_field *field_load;
	struct regmap_field *field_freerun;
	struct regmap_field *field_analogpd;
	struct regmap_field *field_start;
	struct regmap_field *field_digitalpd;
	struct regmap_field *field_mode;
	struct regmap_field *field_singlech;
	struct regmap_field *field_channel;
	struct regmap_field *field_gpio_en;
	struct regmap_field *field_gpio_oen;
	struct regmap_field *field_gpio_value;
	struct regmap_field *field_gpio_in;

	struct regmap_field *field_vref_ts;

	int gpio_irqs[SAR_MAX_PINS];

	int wakeirq_sar, wakeirq_gpio;

	struct regmap *pmsleep;
};

static int msc313e_sar_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct msc313e_sar *sar = iio_priv(indio_dev);

	switch(mask){
	case IIO_CHAN_INFO_RAW:
		// single channel mode
		regmap_field_force_write(sar->field_singlech, 1);
		regmap_field_force_write(sar->field_channel, chan->channel);

		// one shot mode
		regmap_field_force_write(sar->field_mode, 0);
		regmap_field_force_write(sar->field_start, 0);
		regmap_field_force_write(sar->field_start, 1);

		//regmap_field_force_write(sar->field_mode, 1);
		//regmap_field_force_write(sar->field_freerun, 1);

		// todo fix this, is this load thing needed
		// surely there is a done bit somewhere
		mdelay(1);

		regmap_field_force_write(sar->field_load, 1);

		regmap_read(sar->regmap, chan->address, val);
		if(chan->type == IIO_TEMP){
			//formula right out of vendor code
			*val = (1220 * (400 - *val) + 25000);
		}
		*val2 = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 3;
		*val2 = 0;
		return IIO_VAL_INT;
	}

	return 0;
};

static const struct iio_info msc313e_sar_iio_info = {
	.read_raw = msc313e_sar_read_raw,
};

#define MSC313E_SAR_CHAN_REG(ch) (0x100 + (ch * 4))
#define MSC313E_SAR_CHAN(index, _type) \
	{ .type = _type, \
	  .indexed = 1, \
	  .channel = index, \
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	  .address = MSC313E_SAR_CHAN_REG(index), \
	  .datasheet_name = "sar#index"\
	}

#define MSC313E_SAR_VOLTAGE_CHAN(index)  MSC313E_SAR_CHAN(index, IIO_VOLTAGE)

static const struct iio_chan_spec msc313e_sar_channels[] = {
	MSC313E_SAR_VOLTAGE_CHAN(0),
	MSC313E_SAR_VOLTAGE_CHAN(1),
	MSC313E_SAR_VOLTAGE_CHAN(2),
	MSC313E_SAR_VOLTAGE_CHAN(3),
	MSC313E_SAR_CHAN(6, IIO_TEMP),
};

static irqreturn_t msc313e_sar_irq(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct msc313e_sar *sar = iio_priv(indio_dev);
	unsigned int status;

	regmap_read(sar->regmap, REG_INT_STATUS, &status);

	regmap_write(sar->regmap, REG_INT_FORCE, 0);
	regmap_update_bits(sar->regmap, REG_INT_CLR, ~0, ~0);

	printk("sar: int: %x\n", status);

	return IRQ_HANDLED;
}

static int msc313e_sar_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	regmap_field_update_bits(sar->field_gpio_en, 1 << offset, 0);
	return gpiochip_generic_request(chip, offset);
}

static void msc313e_sar_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	regmap_field_update_bits(sar->field_gpio_en, 1 << offset, 1 << offset);
	gpiochip_generic_free(chip, offset);
}

static void msc313e_sar_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	regmap_field_update_bits(sar->field_gpio_value, 1 << offset, value << offset);
}

static int msc313e_sar_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);
	unsigned int val;
	regmap_field_read(sar->field_gpio_in, &val);
	return val >> offset;
}

static int msc313e_sar_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	regmap_field_update_bits(sar->field_gpio_oen, 1 << offset, 1 << offset);

	return 0;
}

static int msc313e_sar_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	regmap_field_update_bits(sar->field_gpio_oen, 1 << offset, 0);
	msc313e_sar_gpio_set(chip, offset, value);

	return 0;
}

static int msc313e_sar_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct msc313e_sar *sar = gpiochip_get_data(chip);

	return sar->gpio_irqs[offset];
}

static const char *gpionames[] = {
	"sar_gpio0",
	"sar_gpio1",
	"sar_gpio2",
	"sar_gpio3",
};

static int msc313e_sar_probe_gpio(struct platform_device *pdev, struct msc313e_sar *sar)
{
	int i, ret;

	sar->gpiochip.label            = DRIVER_NAME;
	sar->gpiochip.owner            = THIS_MODULE,
	sar->gpiochip.parent           = &pdev->dev;
	sar->gpiochip.request          = msc313e_sar_gpio_request;
	sar->gpiochip.free             = msc313e_sar_gpio_free;
	sar->gpiochip.direction_input  = msc313e_sar_gpio_direction_input;
	sar->gpiochip.get              = msc313e_sar_gpio_get;
	sar->gpiochip.direction_output = msc313e_sar_gpio_direction_output;
	sar->gpiochip.set              = msc313e_sar_gpio_set;
	sar->gpiochip.to_irq           = msc313e_sar_gpio_to_irq;
	sar->gpiochip.base             = -1;
	sar->gpiochip.ngpio            = 4;
	sar->gpiochip.names            = gpionames;

	for(i = 0; i < sar->gpiochip.ngpio; i++){
			sar->gpio_irqs[i] = of_irq_get_byname(pdev->dev.of_node, gpionames[i]);
	}

	ret = gpiochip_add_data(&sar->gpiochip, sar);
	if (ret < 0) {
		dev_err(&pdev->dev,"failed to register gpio chip\n");
		goto out;
	}

	out:
	return ret;
}

static int sar_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np,
			       struct pinctrl_map **map,
			       unsigned int *num_maps)
{
	return pinconf_generic_dt_node_to_map(pctldev, np,
								map, num_maps,
								PIN_MAP_TYPE_INVALID);
}

static void sar_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops sar_pinctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= sar_dt_node_to_map,
	.dt_free_map		= sar_dt_free_map,
};

static int sar_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			   unsigned int group)
{
	printk("sar set mux %u %u\n", func, group);
	return 0;
}

static const struct pinmux_ops sar_pinmux_ops = {
	.get_functions_count	= pinmux_generic_get_function_count,
	.get_function_name	= pinmux_generic_get_function_name,
	.get_function_groups	= pinmux_generic_get_function_groups,
	.set_mux		= sar_set_mux,
	.strict			= true,
};

static int msc313e_sar_probe_pinctrl(struct platform_device *pdev,
		struct msc313e_sar *sar) {
	int i, ret;

	sar->gpio_range.name = "sar";
	sar->gpio_range.id = 0;
	sar->gpio_range.base = sar->gpiochip.base;
	sar->gpio_range.pins = sar->info->rangepins;
	sar->gpio_range.npins = sar->info->nrangepins;
	sar->gpio_range.gc = &sar->gpiochip;

	sar->pinctrl_desc.name = DRIVER_NAME;
	sar->pinctrl_desc.pctlops = &sar_pinctrl_ops;
	sar->pinctrl_desc.pmxops = &sar_pinmux_ops;
	sar->pinctrl_desc.owner = THIS_MODULE;
	sar->pinctrl_desc.pins = sar->info->pins;
	sar->pinctrl_desc.npins = sar->info->npins;

	ret = devm_pinctrl_register_and_init(&pdev->dev, &sar->pinctrl_desc, sar,
			&sar->pinctrl_dev);

	if (ret) {
		dev_err(&pdev->dev, "failed to register pinctrl\n");
		return ret;
	}

	for (i = 0; i < sar->info->ngroups; i++) {
		const struct sar_pinctrl_group *grp = &sar->info->groups[i];
		ret = pinctrl_generic_add_group(sar->pinctrl_dev, grp->name, &grp->pin,
				1, NULL);
	}

	for (i = 0; i < ARRAY_SIZE(sar_pinctrl_functions); i++) {
		const struct sar_pinctrl_function *func = &sar_pinctrl_functions[i];
		ret = pinmux_generic_add_function(sar->pinctrl_dev, func->name,
				&func->group, 1, func);

	}

	pinctrl_add_gpio_range(sar->pinctrl_dev, &sar->gpio_range);

	ret = pinctrl_enable(sar->pinctrl_dev);
	if (ret)
		dev_err(&pdev->dev, "failed to enable pinctrl\n");

	return ret;
}

static int msc313e_sar_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mstar_sar_info *match_data;
	struct iio_dev *indio_dev;
	struct msc313e_sar *sar;
	__iomem void *base;
	int irq;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*sar));
	if(!indio_dev)
		return -ENOMEM;

	sar = iio_priv(indio_dev);

	sar->info = match_data;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	sar->regmap = devm_regmap_init_mmio(&pdev->dev, base,
				&msc313_sar_regmap_config);
	if (IS_ERR(sar->regmap)) {
		dev_err(&pdev->dev, "failed to register regmap");
		return PTR_ERR(sar->regmap);
	}

	sar->pmsleep = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "mstar,pmsleep");
	if (IS_ERR(sar->pmsleep))
		return PTR_ERR(sar->pmsleep);

	sar->field_load = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_load_field);
	sar->field_freerun = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_freerun_field);
	sar->field_analogpd = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_analogpd_field);
	sar->field_start = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_start_field);
	sar->field_digitalpd = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_digitalpd_field);
	sar->field_mode = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_mode_field);
	sar->field_singlech = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_singlech_field);
	sar->field_channel = devm_regmap_field_alloc(&pdev->dev, sar->regmap, ctrl_channel_field);

	sar->field_gpio_en = devm_regmap_field_alloc(&pdev->dev, sar->regmap, gpio_ctrl_en_field);
	sar->field_gpio_oen = devm_regmap_field_alloc(&pdev->dev, sar->regmap, gpio_ctrl_oen_field);
	sar->field_gpio_value = devm_regmap_field_alloc(&pdev->dev, sar->regmap, gpio_data_value_field);
	sar->field_gpio_in = devm_regmap_field_alloc(&pdev->dev, sar->regmap, gpio_data_in_field);
	sar->field_vref_ts = devm_regmap_field_alloc(&pdev->dev, sar->regmap, vref_ts_field);

	sar->clk = devm_clk_get(&pdev->dev, "sar_clk");
	if (IS_ERR(sar->clk)) {
		dev_err(&pdev->dev, "failed to get clk\n");
		return PTR_ERR(sar->clk);
	}

	irq = of_irq_get_byname(pdev->dev.of_node, "sar");
	if (!irq)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, msc313e_sar_irq, IRQF_SHARED,
			dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;

	irq = of_irq_get_byname(pdev->dev.of_node, "wakeup");
	if (!irq)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, msc313e_sar_irq, IRQF_SHARED,
			dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;

	sar->wakeirq_gpio = of_irq_get_byname(pdev->dev.of_node, "wakeup_gpio");
	if (!sar->wakeirq_gpio)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, sar->wakeirq_gpio, msc313e_sar_irq, IRQF_SHARED,
			dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;

	indio_dev->name = platform_get_device_id(pdev)->name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &msc313e_sar_iio_info;
	indio_dev->num_channels = ARRAY_SIZE(msc313e_sar_channels);
	indio_dev->channels = msc313e_sar_channels;

	platform_set_drvdata(pdev, indio_dev);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	ret = msc313e_sar_probe_gpio(pdev, sar);
	if (ret)
		return ret;

	ret = msc313e_sar_probe_pinctrl(pdev, sar);
	if (ret)
		return ret;

	clk_prepare_enable(sar->clk);

	regmap_write(sar->regmap, REG_SAMPLE_PERIOD, 0x0f);

	regmap_write(sar->regmap, REG_INT_CLR, ~0);
	regmap_write(sar->regmap, REG_INT_MASK, 0);

	/* turn the adc on */
	regmap_field_write(sar->field_analogpd, 0);
	regmap_field_write(sar->field_digitalpd, 0);

	/* set the temp sensor to 2.0v */
	regmap_field_write(sar->field_vref_ts, 0);

	device_init_wakeup(&pdev->dev, 1);

out:
	return ret;
}

static int msc313e_sar_remove(struct platform_device *pdev)
{
	return 0;
}

static int __maybe_unused msc313e_sar_suspend(struct device *dev)
{
	struct msc313e_sar *sar = iio_priv(platform_get_drvdata(to_platform_device(dev)));
	/* turn the tempsensor off */
	regmap_update_bits(sar->pmsleep, REG_PMSLEEP_PD, REG_PMSLEEP_TS_PD, 0);

	if (sar->wakeirq_gpio >= 0) {
		if (device_may_wakeup(dev))
			enable_irq_wake(sar->wakeirq_gpio);
		else
			disable_irq_wake(sar->wakeirq_gpio);
	}

	return 0;
}

static int __maybe_unused msc313e_sar_resume(struct device *dev)
{
	struct msc313e_sar *sar = iio_priv(platform_get_drvdata(to_platform_device(dev)));

	/* turn the tempsensor on */
	regmap_update_bits(sar->pmsleep, REG_PMSLEEP_PD, REG_PMSLEEP_TS_PD, REG_PMSLEEP_TS_PD);

	if (sar->wakeirq_gpio >= 0 && device_may_wakeup(dev))
		disable_irq_wake(sar->wakeirq_gpio);

	return 0;
}

static SIMPLE_DEV_PM_OPS(msc313e_sar_pm_ops, msc313e_sar_suspend,
			 msc313e_sar_resume);


static const struct of_device_id msc313e_sar_dt_ids[] = {
#ifdef CONFIG_MACH_INFINITY
	{
		.compatible = "mstar,msc313e-sar",
		.data = &msc313_info,
	},
	{
		.compatible = "sstar,ssd20x-sar",
		.data = &ssd20x_info,
	},
#endif
#ifdef CONFIG_MACH_PIONEER3
	{
		.compatible = "sstar,ssd210-sar",
		.data = &ssd210_info,
	},
	{
		.compatible = "sstar,ssd212-sar",
		.data = &ssd212_info,
	},
#endif
#ifdef CONFIG_MACH_MERCURY
	{
		.compatible = "mstar,ssc8336-sar",
		.data = &ssc8336_info,
	},
#endif
	{},
};
MODULE_DEVICE_TABLE(of, msc313e_sar_dt_ids);

static struct platform_driver msc313e_sar_driver = {
	.probe = msc313e_sar_probe,
	.remove = msc313e_sar_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = msc313e_sar_dt_ids,
		   .pm = &msc313e_sar_pm_ops,
	},
};

module_platform_driver(msc313e_sar_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mstar MSC313e SAR driver");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
