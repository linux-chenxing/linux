// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>

#include <dt-bindings/pinctrl/mstar.h>

#include "core.h"
#include "devicetree.h"
#include "pinconf.h"
#include "pinmux.h"

#include "pinctrl-mstar.h"

#define DRIVER_NAME "pinctrl-mstar"

struct msc313_pinctrl {
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl;
	void __iomem *mux;
	struct regmap *regmap;
	const struct msc313_pinctrl_info *info;
};

/*
 * Common groups and register values that are used
 * for all chips so far.
 *
 * This maps functions to the groups that can handle
 * a function and the register bits that need to be
 * set to enable that function.
 */
static const char * const i2c0_groups[]	   = { GROUPNAME_I2C0 };
static const u16	  i2c0_values[]	   = { BIT(0) };
static const char * const i2c1_groups[]	   = { GROUPNAME_I2C1 };
static const u16	  i2c1_values[]	   = { BIT(4)};
static const char * const pm_uart_groups[] = { GROUPNAME_PM_UART };
static const char * const fuart_groups[]   = { GROUPNAME_FUART, GROUPNAME_FUART_RX_TX_RTS };
static const u16	  fuart_values[]   = { BIT(0), BIT(0) };
static const char * const uart0_groups[]   = { GROUPNAME_FUART_RX_TX };
static const char * const uart1_groups[]   = { GROUPNAME_FUART_CTS_RTS, GROUPNAME_FUART_CTS };
static const u16	  uart1_values[]   = { BIT(9), BIT(9) };
static const char * const pm_spi_groups[]  = { GROUPNAME_PM_SPI };
static const char * const usb_groups[]     = { GROUPNAME_USB };
static const char * const usb1_groups[]    = { GROUPNAME_USB1 };
static const char * const pwm0_groups[]    = { GROUPNAME_FUART_RX };
static const u16	  pwm0_values[]    = { BIT(1) | BIT(0) };
static const char * const pwm1_groups[]    = { GROUPNAME_FUART_TX };
static const u16	  pwm1_values[]    = { BIT(3) | BIT(2) };
static const char * const pwm2_groups[]    = { GROUPNAME_FUART_CTS };
static const u16	  pwm2_values[]    = { BIT(5) };
static const char * const pwm3_groups[]    = { GROUPNAME_FUART_RTS };
static const u16	  pwm3_values[]    = { BIT(7) };
static const char * const pwm4_groups[]    = { GROUPNAME_SPI0_CZ };
static const u16	  pwm4_values[]    = { BIT(9) };
static const char * const pwm5_groups[]    = { GROUPNAME_SPI0_CK };
static const u16	  pwm5_values[]    = { BIT(11) };
static const char * const pwm6_groups[]    = { GROUPNAME_SPI0_DI };
static const u16	  pwm6_values[]    = { BIT(13) };
static const char * const pwm7_groups[]    = { GROUPNAME_SPI0_DO };
static const u16	  pwm7_values[]    = { BIT(15) };
static const char * const eth_groups[]     = { GROUPNAME_ETH };
static const u16	  eth_values[]     = { BIT(2) };
static const char * const jtag_groups[]    = { GROUPNAME_FUART };
static const char * const spi0_groups[]    = { GROUPNAME_SPI0, GROUPNAME_FUART };
static const u16	  spi0_values[]    = { BIT(0), BIT(1) | BIT(0) };
static const char * const spi1_groups[]    = { GROUPNAME_SD_D0_D1_D2_D3 };
static const u16	  spi1_values[]    = { BIT(5) | BIT(4) };
static const char * const sdio_groups[]    = { GROUPNAME_SD };
static const u16	  sdio_values[]    = { BIT(8) };

/* for pins that have the same on the different chips */
#define COMMON_PIN(_model, _pinname) \
	PINCTRL_PIN(PIN_##_model##_##_pinname, PINNAME_##_pinname)

struct msc313_pinctrl_function {
	const char *name;
	int reg;
	u16 mask;
	const char * const *groups;
	const u16 *values;
	int numgroups;
};

#define MSTAR_PINCTRL_FUNCTION(n, r, m, g, v) \
	{ \
		.name = n,\
		.reg = r,\
		.mask = m,\
		.groups = g,\
		.values = v,\
		.numgroups = ARRAY_SIZE(g)\
	}

struct msc313_pinctrl_group {
	const char *name;
	const int *pins;
	const int numpins;
};

#define MSTAR_PINCTRL_GROUP(n, p) \
	{\
		.name = n,\
		.pins = p,\
		.numpins = ARRAY_SIZE(p)\
	}

/*
 * Not all pins have "pinconf" so we only
 * carry this extra info for those that do.
 *
 * For some pins all of these bits in the same
 * register, for others they are split across
 * registers. Not all pins have all of the
 * registers.
 */
struct msc313_pinctrl_pinconf {
	const int pin;
	const int pull_en_reg;
	const int pull_en_bit;
	const int pull_dir_reg;
	const int pull_dir_bit;
	const int input_reg;
	const int input_bit;
	const int drive_reg;
	const int drive_lsb;
	const int drive_width;
	const unsigned int *drivecurrents;
	const int ndrivecurrents;
};

/* There isn't a register for the function for this pin */
#define NOREG		-1
/*
 * If used for pull_en_reg this means there is an always
 * on pull up, if used for pull_dir_reg there is an optional
 * pull up.
 */
#define ALWAYS_PULLUP	-2
/* See above but for pull down. */
#define ALWAYS_PULLDOWN	-3

#define MSTAR_PINCTRL_PIN(_pin, _pull_en_reg, _pull_en_bit, \
		_pull_dir_reg, _pull_dir_bit, \
		_drive_reg, _drive_lsb, _drive_width, _drivecurrents) \
	{ \
		.pin = _pin, \
		.pull_en_reg = _pull_en_reg, \
		.pull_en_bit = _pull_en_bit, \
		.pull_dir_reg = _pull_dir_reg, \
		.pull_dir_bit = _pull_dir_bit, \
		.drive_reg = _drive_reg, \
		.drive_lsb = _drive_lsb, \
		.drive_width = _drive_width, \
		.drivecurrents = _drivecurrents, \
		.ndrivecurrents = ARRAY_SIZE(_drivecurrents) \
	}

/*
 * Per-chip info that describes all of the pins,
 * the pin groups, the mappable functions and
 * pins that support pinconf.
 */
struct msc313_pinctrl_info {
	const struct pinctrl_pin_desc *pins;
	const int npins;
	const struct msc313_pinctrl_group *groups;
	const int ngroups;
	const struct msc313_pinctrl_function *functions;
	const int nfunctions;
	const struct msc313_pinctrl_pinconf *pinconfs;
	const int npinconfs;
};

#define MSTAR_PINCTRL_INFO(_chip) static const struct msc313_pinctrl_info _chip##_info = { \
	.pins = _chip##_pins, \
	.npins = ARRAY_SIZE(_chip##_pins), \
	.groups = _chip##_pinctrl_groups, \
	.ngroups = ARRAY_SIZE(_chip##_pinctrl_groups), \
	.functions = _chip##_pinctrl_functions, \
	.nfunctions = ARRAY_SIZE(_chip##_pinctrl_functions), \
	.pinconfs = _chip##_configurable_pins, \
	.npinconfs = ARRAY_SIZE(_chip##_configurable_pins),\
}

#define COMMON_FIXED_FUNCTION(_NAME, _name) MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, -1, 0, _name##_groups, NULL)
#define COMMON_FUNCTION(_NAME, _name) MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, _name##_values)
#define COMMON_FUNCTION_NULLVALUES(_NAME, _name) MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, NULL)

#define COMMON_FUNCTIONS COMMON_FIXED_FUNCTION(PM_UART, pm_uart), \
			 COMMON_FIXED_FUNCTION(PM_SPI, pm_spi), \
			 COMMON_FIXED_FUNCTION(USB, usb), \
			 COMMON_FUNCTION(FUART, fuart), \
			 COMMON_FUNCTION_NULLVALUES(UART0, uart0), \
			 COMMON_FUNCTION(UART1, uart1), \
			 COMMON_FUNCTION(PWM0, pwm0), \
			 COMMON_FUNCTION(PWM1, pwm1), \
			 COMMON_FUNCTION(PWM2, pwm2), \
			 COMMON_FUNCTION(PWM3, pwm3), \
			 COMMON_FUNCTION(PWM4, pwm4), \
			 COMMON_FUNCTION(PWM5, pwm5), \
			 COMMON_FUNCTION(PWM6, pwm6), \
			 COMMON_FUNCTION(PWM7, pwm7), \
			 COMMON_FUNCTION(SDIO, sdio), \
			 COMMON_FUNCTION(I2C0, i2c0), \
			 COMMON_FUNCTION(I2C1, i2c1), \
			 COMMON_FUNCTION(SPI0, spi0), \
			 COMMON_FUNCTION(SPI1, spi1), \
			 COMMON_FUNCTION_NULLVALUES(JTAG, jtag), \
			 COMMON_FUNCTION(ETH, eth)

static const unsigned sd_drivestrengths[] = {4, 8};

#define SD_PIN(_PIN, _PULLUPBIT, _DRIVEBIT) MSTAR_PINCTRL_PIN(_PIN, REG_SDIO_PULLDRIVE, \
		_PULLUPBIT, ALWAYS_PULLUP, -1, REG_SDIO_PULLDRIVE, _DRIVEBIT, 1, sd_drivestrengths)

/* clk has a fixed pull down */
#define SD_PINS(_chipname) SD_PIN(PIN_##_chipname##_SD_CMD, 8, 0), \
			   SD_PIN(PIN_##_chipname##_SD_D0, 9, 1), \
			   SD_PIN(PIN_##_chipname##_SD_D1, 10, 2), \
			   SD_PIN(PIN_##_chipname##_SD_D2, 11, 3), \
			   SD_PIN(PIN_##_chipname##_SD_D3, 12, 4), \
			   MSTAR_PINCTRL_PIN(PIN_##_chipname##_SD_CLK, -1, -1, \
				ALWAYS_PULLDOWN, -1, REG_SDIO_PULLDRIVE, 5, 1, sd_drivestrengths)

static const unsigned int spi0_drivestrengths[] = {4, 8, 12, 16};

#define SPI0_PIN(_pin, _offset) MSTAR_PINCTRL_PIN(_pin, -1, -1, -1, -1,\
		REG_SPI_DRIVE, _offset, 2, spi0_drivestrengths)

#define SPI0_PINS(_chipname) SPI0_PIN(PIN_##_chipname##_SPI0_CZ, 0), \
			     SPI0_PIN(PIN_##_chipname##_SPI0_CK, 2), \
			     SPI0_PIN(PIN_##_chipname##_SPI0_DI, 4), \
			     SPI0_PIN(PIN_##_chipname##_SPI0_DO, 6)

static const unsigned int i2c_drivestrengths[] = {4, 8};

#define I2C1_PIN(_pin, _offset) MSTAR_PINCTRL_PIN(_pin, REG_I2C1_PULL_EN, _offset, \
		REG_I2C1_PULL_DIR, _offset, REG_I2C1_DRIVE, _offset, 1, i2c_drivestrengths)

#define I2C1_PINS(_chipname) I2C1_PIN(PIN_##_chipname##_I2C1_SCL, 0), \
			     I2C1_PIN(PIN_##_chipname##_I2C1_SDA, 1)

static const unsigned int sr_drivestrengths[] = {4, 8};
#define SR_PIN_0(_pin, _offset) MSTAR_PINCTRL_PIN(_pin, REG_SR_PULL_EN0, _offset, \
		REG_SR_PULL_DIR0, _offset, REG_SR_DRIVE0, _offset, 1, sr_drivestrengths)
#define SR_PIN_1(_pin, _offset) MSTAR_PINCTRL_PIN(_pin, REG_SR_PULL_EN1, _offset, \
		REG_SR_PULL_DIR1, _offset, REG_SR_DRIVE1, _offset, 1, sr_drivestrengths)

#define SR_PINS(_chipname) SR_PIN_0(PIN_##_chipname##_SR_IO2, 2), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO3, 3), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO4, 4), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO5, 5), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO6, 6), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO7, 7), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO8, 8), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO9, 9), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO10, 10), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO11, 11), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO12, 12), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO13, 13), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO14, 14), \
			   SR_PIN_0(PIN_##_chipname##_SR_IO15, 15), \
			   SR_PIN_1(PIN_##_chipname##_SR_IO16, 0), \
			   SR_PIN_1(PIN_##_chipname##_SR_IO17, 1)

#if CONFIG_MACH_INFINITY
#define MSC313_COMMON_PIN(_pinname) COMMON_PIN(MSC313, _pinname)
/* msc313/msc313e */

/* pinctrl pins */
static struct pinctrl_pin_desc msc313_pins[] = {
	MSC313_COMMON_PIN(PM_SD_CDZ),
	MSC313_COMMON_PIN(PM_IRIN),
	MSC313_COMMON_PIN(PM_UART_RX),
	MSC313_COMMON_PIN(PM_UART_TX),
	MSC313_COMMON_PIN(PM_GPIO4),
	MSC313_COMMON_PIN(PM_SPI_CZ),
	MSC313_COMMON_PIN(PM_SPI_DI),
	MSC313_COMMON_PIN(PM_SPI_WPZ),
	MSC313_COMMON_PIN(PM_SPI_DO),
	MSC313_COMMON_PIN(PM_SPI_CK),
	MSC313_COMMON_PIN(ETH_RN),
	MSC313_COMMON_PIN(ETH_RP),
	MSC313_COMMON_PIN(ETH_TN),
	MSC313_COMMON_PIN(ETH_TP),
	MSC313_COMMON_PIN(FUART_RX),
	MSC313_COMMON_PIN(FUART_TX),
	MSC313_COMMON_PIN(FUART_CTS),
	MSC313_COMMON_PIN(FUART_RTS),
	MSC313_COMMON_PIN(I2C1_SCL),
	MSC313_COMMON_PIN(I2C1_SDA),
	PINCTRL_PIN(PIN_MSC313_SR_IO2,	"sr_io2"),
	PINCTRL_PIN(PIN_MSC313_SR_IO3,	"sr_io3"),
	PINCTRL_PIN(PIN_MSC313_SR_IO4,	"sr_io4"),
	PINCTRL_PIN(PIN_MSC313_SR_IO5,	"sr_io5"),
	PINCTRL_PIN(PIN_MSC313_SR_IO6,	"sr_io6"),
	PINCTRL_PIN(PIN_MSC313_SR_IO7,	"sr_io7"),
	PINCTRL_PIN(PIN_MSC313_SR_IO8,	"sr_io8"),
	PINCTRL_PIN(PIN_MSC313_SR_IO9,	"sr_io9"),
	PINCTRL_PIN(PIN_MSC313_SR_IO10,	"sr_io10"),
	PINCTRL_PIN(PIN_MSC313_SR_IO11,	"sr_io11"),
	PINCTRL_PIN(PIN_MSC313_SR_IO12,	"sr_io12"),
	PINCTRL_PIN(PIN_MSC313_SR_IO13,	"sr_io13"),
	PINCTRL_PIN(PIN_MSC313_SR_IO14,	"sr_io14"),
	PINCTRL_PIN(PIN_MSC313_SR_IO15,	"sr_io15"),
	PINCTRL_PIN(PIN_MSC313_SR_IO16,	"sr_io16"),
	PINCTRL_PIN(PIN_MSC313_SR_IO17,	"sr_io17"),
	MSC313_COMMON_PIN(SPI0_CZ),
	MSC313_COMMON_PIN(SPI0_CK),
	MSC313_COMMON_PIN(SPI0_DI),
	MSC313_COMMON_PIN(SPI0_DO),
	MSC313_COMMON_PIN(SD_CLK),
	MSC313_COMMON_PIN(SD_CMD),
	MSC313_COMMON_PIN(SD_D0),
	MSC313_COMMON_PIN(SD_D1),
	MSC313_COMMON_PIN(SD_D2),
	MSC313_COMMON_PIN(SD_D3),
	MSC313_COMMON_PIN(USB_DM),
	MSC313_COMMON_PIN(USB_DP),
};

/* mux pin groupings */
static const int msc313_pm_uart_pins[]		= { PIN_MSC313_PM_UART_RX, PIN_MSC313_PM_UART_TX };
static const int msc313_pm_spi_pins[]		= { PIN_MSC313_PM_SPI_CZ, PIN_MSC313_PM_SPI_DI,
						    PIN_MSC313_PM_SPI_WPZ, PIN_MSC313_PM_SPI_DO,
						    PIN_MSC313_PM_SPI_CK };
static const int msc313_eth_pins[]		= { PIN_MSC313_ETH_RN, PIN_MSC313_ETH_RP,
						    PIN_MSC313_ETH_TN, PIN_MSC313_ETH_TP };
static const int msc313_fuart_pins[]		= { PIN_MSC313_FUART_RX, PIN_MSC313_FUART_TX,
						    PIN_MSC313_FUART_CTS, PIN_MSC313_FUART_RTS };
static const int msc313_fuart_rx_pins[]		= { PIN_MSC313_FUART_RX };
static const int msc313_fuart_tx_pins[]		= { PIN_MSC313_FUART_TX };
static const int msc313_fuart_cts_pins[]	= { PIN_MSC313_FUART_CTS };
static const int msc313_fuart_rts_pins[]	= { PIN_MSC313_FUART_RTS };
static const int msc313_fuart_rx_tx_rts_pins[]	= { PIN_MSC313_FUART_RX,
						    PIN_MSC313_FUART_TX,
						    PIN_MSC313_FUART_RTS };
static const int msc313_fuart_cts_rts_pins[]	= { PIN_MSC313_FUART_CTS, PIN_MSC313_FUART_RTS };
static const int msc313_i2c1_pins[]		= { PIN_MSC313_I2C1_SCL, PIN_MSC313_I2C1_SDA };
static const int msc313_spi0_pins[]		= { PIN_MSC313_SPI0_CZ,
						    PIN_MSC313_SPI0_CK,
						    PIN_MSC313_SPI0_DI,
						    PIN_MSC313_SPI0_DO };
static const int msc313_spi0_cz_pins[]		= { PIN_MSC313_SPI0_CZ };
static const int msc313_spi0_ck_pins[]		= { PIN_MSC313_SPI0_CK };
static const int msc313_spi0_di_pins[]		= { PIN_MSC313_SPI0_DI };
static const int msc313_spi0_do_pins[]		= { PIN_MSC313_SPI0_DO };
static const int msc313_sd_d0_d1_d2_d3_pins[]	= { PIN_MSC313_SD_D0, PIN_MSC313_SD_D1,
						    PIN_MSC313_SD_D2,PIN_MSC313_SD_D3 };
static const int msc313_sd_pins[]		= { PIN_MSC313_SD_CLK, PIN_MSC313_SD_CMD,
						    PIN_MSC313_SD_D0, PIN_MSC313_SD_D1,
						    PIN_MSC313_SD_D2, PIN_MSC313_SD_D3 };
static const int msc313_usb_pins[]		= { PIN_MSC313_USB_DM, PIN_MSC313_USB_DP };

#define MSC313_PINCTRL_GROUP(_NAME, _name) MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, msc313_##_name##_pins)

static const struct msc313_pinctrl_group msc313_pinctrl_groups[] = {
	MSC313_PINCTRL_GROUP(PM_UART, pm_uart),
	MSC313_PINCTRL_GROUP(PM_SPI, pm_spi),
	MSC313_PINCTRL_GROUP(USB, usb),
	MSC313_PINCTRL_GROUP(ETH, eth),
	MSC313_PINCTRL_GROUP(FUART, fuart),
	MSC313_PINCTRL_GROUP(FUART_RX, fuart_rx),
	MSC313_PINCTRL_GROUP(FUART_TX, fuart_tx),
	MSC313_PINCTRL_GROUP(FUART_CTS, fuart_cts),
	MSC313_PINCTRL_GROUP(FUART_RTS, fuart_rts),
	MSC313_PINCTRL_GROUP(FUART_RX_TX_RTS, fuart_rx_tx_rts),
	MSC313_PINCTRL_GROUP(FUART_CTS_RTS, fuart_cts_rts),
	MSC313_PINCTRL_GROUP(I2C1, i2c1),
	MSC313_PINCTRL_GROUP(SPI0, spi0),
	MSC313_PINCTRL_GROUP(SPI0_CZ, spi0_cz),
	MSC313_PINCTRL_GROUP(SPI0_CK, spi0_ck),
	MSC313_PINCTRL_GROUP(SPI0_DI, spi0_di),
	MSC313_PINCTRL_GROUP(SPI0_DO, spi0_do),
	MSC313_PINCTRL_GROUP(SD_D0_D1_D2_D3, sd_d0_d1_d2_d3),
	MSC313_PINCTRL_GROUP(SD, sd),
};

static const struct msc313_pinctrl_function msc313_pinctrl_functions[] = {
	COMMON_FUNCTIONS
};

static const struct msc313_pinctrl_pinconf msc313_configurable_pins[] = {
	SD_PINS(MSC313),
	SPI0_PINS(MSC313),
	I2C1_PINS(MSC313),
	SR_PINS(MSC313),
};

MSTAR_PINCTRL_INFO(msc313);
#endif /* infinity */

static int mstar_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np,
			       struct pinctrl_map **map,
			       unsigned int *num_maps)
{
	return pinconf_generic_dt_node_to_map(pctldev, np,
						map, num_maps,
						PIN_MAP_TYPE_INVALID);
}

static void mstar_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops msc313_pinctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= mstar_dt_node_to_map,
	.dt_free_map		= mstar_dt_free_map,
};

static int mstar_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			   unsigned int group)
{
	struct msc313_pinctrl *pinctrl = pctldev->driver_data;
	const char *grpname = pinctrl_generic_get_group_name(pctldev, group);
	struct function_desc *funcdesc = pinmux_generic_get_function(pctldev, func);
	struct msc313_pinctrl_function *function = funcdesc->data;
	int i, ret = 0;

	if(function != NULL){
		if(function->reg >= 0 && function->values != NULL){
			for(i = 0; i < function->numgroups; i++){
				if(strcmp(function->groups[i], grpname) == 0){
					dev_dbg(pinctrl->dev, "updating mux reg %x\n", (unsigned) function->reg);
					ret = regmap_update_bits(pinctrl->regmap, function->reg,
							function->mask, function->values[i]);
					if(ret)
						dev_dbg(pinctrl->dev, "failed to update register\n");
					break;
				}
			}
		}
		else {
			dev_dbg(pinctrl->dev, "reg or values not found\n");
		}
	}
	else {
		dev_info(pinctrl->dev, "missing function data\n");
	}

	return ret;
}

static const struct pinmux_ops mstar_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name   = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux             = mstar_set_mux,
	.strict              = true,
};

static int msc313_pinctrl_parse_groups(struct msc313_pinctrl *pinctrl){
	int i, ret;

	for(i = 0; i < pinctrl->info->ngroups; i++){
		const struct msc313_pinctrl_group *grp = &pinctrl->info->groups[i];
		ret = pinctrl_generic_add_group(pinctrl->pctl, grp->name,
				(int*) grp->pins, grp->numpins, NULL);
	}
	return ret;
}

static int msc313_pinctrl_parse_functions(struct msc313_pinctrl *pinctrl){
	int i, ret;

	for(i = 0; i < pinctrl->info->nfunctions; i++){
		const struct msc313_pinctrl_function *func =  &pinctrl->info->functions[i];

		// clear any existing value for the function
		if(func->reg >= 0){
			regmap_update_bits(pinctrl->regmap, func->reg,
					func->mask, 0);
		}

		ret = pinmux_generic_add_function(pinctrl->pctl, func->name,
				(const char**) func->groups, func->numgroups, (void*) func);
		if(ret < 0){
			dev_err(pinctrl->dev, "failed to add function: %d", ret);
			goto out;
		}
	}
	out:
	return ret;
}

static const struct regmap_config msc313_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int mstar_set_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long config){
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int i;
	unsigned int mask;
	const struct msc313_pinctrl_pinconf *confpin;
	dev_dbg(pinctrl->dev, "setting %d:%u on pin %d\n", (int)config,(unsigned)arg, pin);
	for(i = 0; i < pinctrl->info->npinconfs; i++){
		if(pinctrl->info->pinconfs[i].pin == pin){
			confpin = &pinctrl->info->pinconfs[i];
			switch(param){
			case PIN_CONFIG_BIAS_PULL_UP:
				if(confpin->pull_en_reg != -1){
					dev_dbg(pinctrl->dev, "setting pull up %d on pin %d\n", (int) arg, pin);
					mask = 1 << confpin->pull_en_bit;
					regmap_update_bits(pinctrl->regmap, confpin->pull_en_reg, mask, arg ? mask : 0);
				}
				else
					dev_info(pinctrl->dev, "pullup reg/bit isn't known for pin %d\n", pin);
			default:
				break;
			}
			return 0;
		}
	}
	return 0;
}

/*
 * Check if a pin is one that is always pulled up or down
 * then check if there is an optional pull up or down, then
 * check if that is always up or down, and finally if there
 * is a direction bit check that for the direction.
 */
static bool msc313_pinctrl_ispulled(struct msc313_pinctrl *pinctrl,
		const struct msc313_pinctrl_pinconf *confpin, bool down)
{
	unsigned val;

	if(confpin->pull_en_reg == ALWAYS_PULLUP)
		return !down;
	else if(confpin->pull_en_reg == ALWAYS_PULLDOWN)
		return down;
	else if(confpin->pull_en_reg != NOREG){
		regmap_read(pinctrl->regmap, confpin->pull_en_reg, &val);
		if(val & BIT(confpin->pull_en_bit)){
			if (confpin->pull_dir_reg == ALWAYS_PULLUP)
				return !down;
			else if (confpin->pull_dir_reg == ALWAYS_PULLDOWN)
				return down;
			else if (confpin->pull_en_reg != NOREG){
				regmap_read(pinctrl->regmap, confpin->pull_dir_reg, &val);
				if (val & BIT(confpin->pull_dir_bit))
					return !down;
				else
					return down;
			}
			else
				return false;
		}
		else
			return false;
	}
	else
		return false;
}

static int msc313_pinctrl_get_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long *config){
	int i;
	const struct msc313_pinctrl_pinconf *confpin;
	unsigned val;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned crntidx;

	/* we only support a limited range of conf options so filter the
	 * ones we do here
	 */

	switch(param){
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
		case PIN_CONFIG_DRIVE_STRENGTH:
			break;
		default:
			return -ENOTSUPP;
	}

	/* try to find the configuration register(s) for the pin */
	for(i = 0; i < pinctrl->info->npinconfs; i++){
		if(pinctrl->info->pinconfs[i].pin == pin){
			confpin = &pinctrl->info->pinconfs[i];
			switch(param){
			case PIN_CONFIG_BIAS_PULL_UP:
				return msc313_pinctrl_ispulled(pinctrl, confpin, false) ? 0 : -EINVAL;
			case PIN_CONFIG_BIAS_PULL_DOWN:
				return msc313_pinctrl_ispulled(pinctrl, confpin, true) ? 0 : -EINVAL;
			case PIN_CONFIG_DRIVE_STRENGTH:
				if(confpin->drive_reg != -1){
					regmap_read(pinctrl->regmap, confpin->drive_reg, &val);
					crntidx = (val >> confpin->drive_lsb) & BIT_MASK(confpin->drive_width);
					*config = pinconf_to_config_packed(param, confpin->drivecurrents[crntidx]);
					return 0;
				}
				return -ENOTSUPP;
			default:
				return -ENOTSUPP;
			}
		}
	}

	return -ENOTSUPP;
}

int mstar_pin_config_get(struct pinctrl_dev *pctldev,
			       unsigned pin,
			       unsigned long *config){
	struct msc313_pinctrl *pinctrl = pctldev->driver_data;
	return msc313_pinctrl_get_config(pinctrl, pin, config);
}

int mstar_pin_config_set(struct pinctrl_dev *pctldev,
			       unsigned pin,
			       unsigned long *configs,
			       unsigned num_configs){
	int i;
	struct msc313_pinctrl *pinctrl = pctldev->driver_data;
	for(i = 0; i < num_configs; i++){
		mstar_set_config(pinctrl, pin, configs[i]);
	}
	return 0;
}

int mstar_pin_config_group_get(struct pinctrl_dev *pctldev,
				     unsigned selector,
				     unsigned long *config){
	return -ENOTSUPP;
}

int mstar_pin_config_group_set(struct pinctrl_dev *pctldev,
				     unsigned selector,
				     unsigned long *configs,
				     unsigned num_configs){
	struct msc313_pinctrl *pinctrl = pctldev->driver_data;
	struct group_desc *group = pinctrl_generic_get_group(pctldev, selector);
	int i, j, ret;
	for(i = 0; i < group->num_pins; i++){
		for(j = 0; j < num_configs; j++){
			ret = mstar_set_config(pinctrl, group->pins[i], configs[j]);
			if(ret)
				return ret;
		}
	}
	return 0;
}

static const struct pinconf_ops mstar_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = mstar_pin_config_get,
	.pin_config_set = mstar_pin_config_set,
	.pin_config_group_get = mstar_pin_config_group_get,
	.pin_config_group_set = mstar_pin_config_group_set,
};


static int msc313_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct msc313_pinctrl *pinctrl;
	const struct msc313_pinctrl_info *match_data;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	pinctrl->mux = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pinctrl->mux))
		return PTR_ERR(pinctrl->mux);

	pinctrl->regmap = devm_regmap_init_mmio(pinctrl->dev, pinctrl->mux,
			&msc313_pinctrl_regmap_config);
	if (IS_ERR(pinctrl->regmap)) {
		dev_err(pinctrl->dev, "failed to register regmap");
		return PTR_ERR(pinctrl->regmap);
	}

	pinctrl->desc.name = DRIVER_NAME;
	pinctrl->desc.pctlops = &msc313_pinctrl_ops;
	pinctrl->desc.pmxops = &mstar_pinmux_ops;
	pinctrl->desc.confops = &mstar_pinconf_ops;
	pinctrl->desc.owner = THIS_MODULE;
	pinctrl->desc.pins = pinctrl->info->pins;
	pinctrl->desc.npins = pinctrl->info->npins;

	ret = devm_pinctrl_register_and_init(pinctrl->dev, &pinctrl->desc,
					     pinctrl, &pinctrl->pctl);

	if (ret) {
		dev_err(pinctrl->dev, "failed to register pinctrl\n");
		return ret;
	}

	ret = msc313_pinctrl_parse_functions(pinctrl);
	ret = msc313_pinctrl_parse_groups(pinctrl);

	ret = pinctrl_enable(pinctrl->pctl);
	if (ret)
		dev_err(pinctrl->dev, "failed to enable pinctrl\n");

	return 0;
}

static const struct of_device_id msc313_pinctrl_of_match[] = {
#if CONFIG_MACH_INFINITY
	{
		.compatible	= "mstar,msc313-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "mstar,msc313e-pinctrl",
		.data		= &msc313_info,
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, msc313_pinctrl_of_match);

static struct platform_driver msc313_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_pinctrl_of_match,
	},
	.probe = msc313_pinctrl_probe,
};
module_platform_driver(msc313_pinctrl_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("Pin controller driver for MStar SoCs");
MODULE_LICENSE("GPL v2");
