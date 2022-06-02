/*
 *
 * There are channels at
 * 0x1f001da0
 * 0x1f001dac
 * 0x1f001de0
 * 0x1f001dec
 *
 * Each channel has 3 16 bit registers
 *
 * 0x0 - ctrl/div
 *       12       |    8     |    7 - 0
 * double buffer  | polarity | clock divider
 *
 * polarity = 1 gives a low to high transition.
 *
 * 0x4 - duty cycle
 * 0x8 - period
 *
 *
 * 0xb0 - writes FFFF
 * 0xb4 - writes FFFF
 * 0xbc - 0x3, writes 0x3
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/pwm.h>

#define DRIVER_NAME "msc313-pwm"

#define CHANSZ		0xc
#define REG_DIV		0x0
#define REG_DUTY	0x4
#define REG_PERIOD	0x8

#define REGOFF(ch, reg) ((ch * CHANSZ) + reg)

struct msc313_pwm {
	struct clk *clk;
	struct regmap *regmap;
	struct pwm_chip pwmchip;
	struct regmap_field* clkdiv;
};

static struct reg_field div_clkdiv_field = REG_FIELD(REG_DIV, 0, 7);

#define to_msc313_pwm(ptr) container_of(ptr, struct msc313_pwm, pwmchip)

static const struct regmap_config msc313_pwm_regmap_config = {
		.name = "msc313-pwm",
		.reg_bits = 16,
		.val_bits = 16,
		.reg_stride = 4,
};

static const struct of_device_id msc313_pwm_dt_ids[] = {
	{ .compatible = "mstar,msc313-pwm" },
	{},
};

MODULE_DEVICE_TABLE(of, msc313_pwm_dt_ids);

static int msc313_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		      int duty_ns, int period_ns)
{
	printk("pwm config\n");
	return 0;
};

static int msc313_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *device,
			    enum pwm_polarity polarity)
{
	struct msc313_pwm *pwm = to_msc313_pwm(chip);
	u8 reg = REGOFF(device->hwpwm, REG_DIV);
	u16 mask = BIT(8);
	u16 val;
	printk("pwm polarity\n");

	switch(polarity){
		case PWM_POLARITY_NORMAL:
			val = 0;
			break;
		case PWM_POLARITY_INVERSED:
			val = mask;
			break;
		default:
			return -EINVAL;
	}

	regmap_update_bits(pwm->regmap, reg,
			mask, val);

	return 0;
}

static int msc313_pwm_enable(struct pwm_chip *chip, struct pwm_device *device)
{
	struct msc313_pwm *pwm = to_msc313_pwm(chip);
	printk("pwm enable\n");
	return clk_prepare_enable(pwm->clk);
}

static void msc313_pwm_disable(struct pwm_chip *chip, struct pwm_device *device){
	struct msc313_pwm *pwm = to_msc313_pwm(chip);
	printk("pwm disable\n");
	clk_disable(pwm->clk);
}

static int msc313_apply(struct pwm_chip *chip, struct pwm_device *pwm,
		     const struct pwm_state *state)
{
	printk("pwm apply\n");
	return 0;
}

static void msc313_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			  struct pwm_state *state){
	printk("pwm get state\n");
}

static const struct pwm_ops msc313_pwm_ops = {
	.config = msc313_pwm_config,
	.set_polarity = msc313_pwm_set_polarity,
	.enable = msc313_pwm_enable,
	.disable = msc313_pwm_disable,
	.apply = msc313_apply,
	.get_state = msc313_get_state,
	.owner = THIS_MODULE
};

static int msc313_pwm_probe(struct platform_device *pdev)
{
	int ret;
	struct msc313_pwm *pwm;
	struct resource *res;
	__iomem void* base;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pwm->regmap = devm_regmap_init_mmio(&pdev->dev, base,
			&msc313_pwm_regmap_config);
	if(IS_ERR(pwm->regmap))
		return PTR_ERR(pwm->regmap);

	pwm->clkdiv = devm_regmap_field_alloc(&pdev->dev, pwm->regmap, div_clkdiv_field);

	regmap_field_write(pwm->clkdiv, 0xff);
	regmap_write(pwm->regmap, REG_DUTY, 0xf);
	regmap_write(pwm->regmap, REG_PERIOD, 0xffff);

	pwm->clk = of_clk_get(pdev->dev.of_node, 0);

	pwm->pwmchip.dev = &pdev->dev;
	pwm->pwmchip.ops = &msc313_pwm_ops;
	pwm->pwmchip.base = -1;
	pwm->pwmchip.npwm = 4;
	pwm->pwmchip.of_xlate = of_pwm_xlate_with_flags;
	pwm->pwmchip.of_pwm_n_cells = 3;

	ret = pwmchip_add(&pwm->pwmchip);
	if(ret)
		dev_err(&pdev->dev, "failed to register pwm chip");

	return ret;
}

static int msc313_pwm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver msc313_pwm_driver = {
	.probe = msc313_pwm_probe,
	.remove = msc313_pwm_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = msc313_pwm_dt_ids,
	},
};

module_platform_driver(msc313_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mstar MSC313 PWM driver");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
