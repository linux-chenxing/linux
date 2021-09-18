/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Daniel Palmer <daniel@thingy.jp>
 */

#define MAKEMASK(_what) \
	GENMASK((SHIFT_##_what + WIDTH_##_what) - 1, SHIFT_##_what)

/* Common offsets, masks, etc.. */

/* Common Register offsets */
#define REG_UARTS		0xc
#define REG_PWMS		0x1c
#define REG_SDIO_NAND		0x20
#define REG_I2CS		0x24
#define REG_SPIS		0x30
#define REG_ETH_JTAG		0x3c
#define REG_SENSOR_CONFIG	0x54
#define REG_TX_MIPI_UART2	0x58

#define REG_I2C1_PULL_EN	0x94
#define REG_I2C1_PULL_DIR	0x98
#define REG_I2C1_DRIVE		0x9c
#define REG_SPI_DRIVE		0xa8
#define REG_SDIO_PULLDRIVE	0xc8
#define REG_SR_INPUTENABLE0	0xe0
#define REG_SR_INPUTENABLE1	0xe4
#define REG_SR_PULL_EN0		0xe8
#define REG_SR_PULL_EN1		0xec
#define REG_SR_PULL_DIR0	0xf0
#define REG_SR_PULL_DIR1	0xf4
#define REG_SR_DRIVE0		0xf8
#define REG_SR_DRIVE1		0xfc

/* Common group select registers and masks */
#define REG_FUART	REG_UARTS
#define MASK_FUART	(BIT(1) | BIT(0))
#define REG_UART0	REG_UARTS
#define MASK_UART0	(BIT(5) | BIT(4))
#define REG_UART1	REG_UARTS
#define MASK_UART1	(BIT(9) | BIT(8))

#define REG_PWM0	REG_PWMS
#define MASK_PWM0	(BIT(1) | BIT(0))
#define REG_PWM1	REG_PWMS
#define MASK_PWM1	(BIT(3) | BIT(2))
#define REG_PWM2	REG_PWMS
#define MASK_PWM2	(BIT(5) | BIT(4))
#define REG_PWM3	REG_PWMS
#define MASK_PWM3	(BIT(7) | BIT(6))
#define REG_PWM4	REG_PWMS
#define MASK_PWM4	(BIT(9) | BIT(8))
#define REG_PWM5	REG_PWMS
#define MASK_PWM5	(BIT(11) | BIT(10))
#define REG_PWM6	REG_PWMS
#define MASK_PWM6	(BIT(13) | BIT(11))
#define REG_PWM7	REG_PWMS
#define MASK_PWM7	(BIT(15) | BIT(14))

#define REG_SDIO	REG_SDIO_NAND
#define MASK_SDIO	BIT(8)

#define REG_I2C0	REG_I2CS
#define MASK_I2C0	(BIT(1) | BIT(0))
#define REG_I2C1	REG_I2CS
#define MASK_I2C1	(BIT(5) | BIT(4))

#define REG_SPI0	REG_SPIS
#define MASK_SPI0	(BIT(1) | BIT(0))
#define REG_SPI1	REG_SPIS
#define MASK_SPI1	(BIT(5) | BIT(4))

#define REG_JTAG	REG_ETH_JTAG
#define MASK_JTAG	(BIT(1) | BIT(0))

#define REG_ETH		REG_ETH_JTAG
#define MASK_ETH	BIT(2)

#define REG_SR0_MIPI	REG_SENSOR_CONFIG
#define MASK_SR0_MIPI	(BIT(9) | BIT(8))

#define REG_SR1_BT656	REG_SENSOR_CONFIG
#define MASK_SR1_BT656	BIT(12)

#define REG_SR1_MIPI	REG_SENSOR_CONFIG
#define MASK_SR1_MIPI	(BIT(15) | BIT(14) | BIT(13))

#define REG_TX_MIPI	REG_TX_MIPI_UART2
#define MASK_TX_MIPI	(BIT(1) | BIT(0))

/* ssd201/202d specifics */
/*
 * for ssd20xd the uart registers are at the same place but
 * there are more muxing options
 */
#define REG_SSD20XD_FUART	REG_FUART
#define SHIFT_SSD20XD_FUART	0
#define WIDTH_SSD20XD_FUART	3
#define MASK_SSD20XD_FUART	(BIT(2) | BIT(1) | BIT(0))
#define REG_SSD20XD_UART0	REG_UART0
#define SHIFT_SSD20XD_UART0	4
#define WIDTH_SSD20XD_UART0	3
#define MASK_SSD20XD_UART0	(BIT(6) | BIT(5) | BIT(4))
#define REG_SSD20XD_UART1	REG_UART1
#define SHIFT_SSD20XD_UART1	8
#define WIDTH_SSD20XD_UART1	3
#define MASK_SSD20XD_UART1	(BIT(10) | BIT(9) | BIT(8))
#define REG_SSD20XD_UART2	REG_UARTS
#define SHIFT_SSD20XD_UART2	12
#define WIDTH_SSD20XD_UART2	3
#define MASK_SSD20XD_UART2	(BIT(14) | BIT(13) | BIT(12))
/*
 * for ssd20xd the i2c registers are at the same place but
 * there are more muxing options
 */
#define REG_SSD20XD_I2C0	REG_I2C0
#define SHIFT_SSD20XD_I2C0	0
#define WIDTH_SSD20XD_I2C0	3
#define MASK_SSD20XD_I2C0	MAKEMASK(SSD20XD_I2C0)

#define REG_SSD20XD_I2C1	REG_I2C1
#define SHIFT_SSD20XD_I2C1	4
#define WIDTH_SSD20XD_I2C1	3
#define MASK_SSD20XD_I2C1	MAKEMASK(SSD20XD_I2C1)

/*
 * For ssd20xd SPI has more muxing options too.
 */
#define REG_SSD20XD_SPI0	REG_SPI0
#define SHIFT_SSD20XD_SPI0	0
#define WIDTH_SSD20XD_SPI0	3
#define MASK_SSD20XD_SPI0	MAKEMASK(SSD20XD_SPI0)

/*
 * PWM has more muxing options too.
 */
#define REG_SSD20XD_PWM0	REG_PWMS
#define SHIFT_SSD20XD_PWM0	0
#define WIDTH_SSD20XD_PWM0	3
#define MASK_SSD20XD_PWM0	MAKEMASK(SSD20XD_PWM0)
#define REG_SSD20XD_PWM1	REG_PWMS
#define SHIFT_SSD20XD_PWM1	3
#define WIDTH_SSD20XD_PWM1	3
#define MASK_SSD20XD_PWM1	MAKEMASK(SSD20XD_PWM1)
#define REG_SSD20XD_PWM2	REG_PWMS
#define SHIFT_SSD20XD_PWM2	6
#define WIDTH_SSD20XD_PWM2	3
#define MASK_SSD20XD_PWM2	MAKEMASK(SSD20XD_PWM2)
#define REG_SSD20XD_PWM3	REG_PWMS
#define SHIFT_SSD20XD_PWM3	9
#define WIDTH_SSD20XD_PWM3	3
#define MASK_SSD20XD_PWM3	MAKEMASK(SSD20XD_PWM3)

/*
 * TTL and TX_MIPI seems to be totally specific to the
 * SSD20xD.
 */
#define REG_SSD20XD_TTL		0x34
#define SHIFT_SSD20XD_TTL	8
#define WIDTH_SSD20XD_TTL	4
#define MASK_SSD20XD_TTL	MAKEMASK(SSD20XD_TTL)

#define REG_SSD20XD_TX_MIPI	REG_SSD20XD_TTL
#define SHIFT_SSD20XD_TX_MIPI	12
#define WIDTH_SSD20XD_TX_MIPI	2
#define MASK_SSD20XD_TX_MIPI	MAKEMASK(SSD20XD_TX_MIPI)

#define REG_SSD20XD_ETH		0x38
#define MASK_SSD20XD_ETH0	BIT(0)

#define REG_SSD20XD_ETH1	0x38
#define SHIFT_SSD20XD_ETH1	8
#define WIDTH_SSD20XD_ETH1	4
#define MASK_SSD20XD_ETH1	MAKEMASK(SSD20XD_ETH1)

/* common pin group names */
#define GROUPNAME_SDIO_MODE1		"sdio_mode1"
#define GROUPNAME_USB			"usb"
#define GROUPNAME_USB1			"usb1"
#define GROUPNAME_I2C0_MODE1		"i2c0_mode1"
#define GROUPNAME_I2C0_MODE2		"i2c0_mode2"
#define GROUPNAME_I2C0_MODE3		"i2c0_mode3"
#define GROUPNAME_I2C0_MODE4		"i2c0_mode4"
#define GROUPNAME_I2C1_MODE1		"i2c1_mode1"
#define GROUPNAME_I2C1_MODE3		"i2c1_mode3"
#define GROUPNAME_I2C1_MODE4		"i2c1_mode4"
#define GROUPNAME_I2C1_MODE5		"i2c1_mode5"
#define GROUPNAME_FUART_RX_TX		"fuart_rx_tx"
#define GROUPNAME_FUART_MODE1		"fuart_mode1"
#define GROUPNAME_FUART_MODE1_NOCTS	"fuart_mode1_notcts"
#define GROUPNAME_FUART_MODE2		"fuart_mode2"
#define GROUPNAME_FUART_MODE3		"fuart_mode3"
#define GROUPNAME_FUART_MODE4		"fuart_mode4"
#define GROUPNAME_FUART_MODE5		"fuart_mode5"
#define GROUPNAME_FUART_MODE6		"fuart_mode6"
#define GROUPNAME_FUART_MODE7		"fuart_mode7"
#define GROUPNAME_UART0			"uart0"
#define GROUPNAME_UART1_MODE1		"uart1_mode1"
#define GROUPNAME_UART1_MODE2		"uart1_mode2"
#define GROUPNAME_UART1_MODE2_RXONLY	"uart1_mode2_rxonly"
#define GROUPNAME_UART1_MODE3		"uart1_mode3"
#define GROUPNAME_UART1_MODE4		"uart1_mode4"
#define GROUPNAME_ETH_MODE1		"eth_mode1"
#define GROUPNAME_ETH1_MODE1		"eth1_mode1"
#define GROUPNAME_ETH1_MODE2		"eth1_mode2"
#define GROUPNAME_ETH1_MODE3		"eth1_mode3"
#define GROUPNAME_ETH1_MODE4		"eth1_mode4"
#define GROUPNAME_ETH1_MODE5		"eth1_mode5"
#define GROUPNAME_PWM0_MODE1		"pwm0_mode1"
#define GROUPNAME_PWM0_MODE2		"pwm0_mode2"
#define GROUPNAME_PWM0_MODE3		"pwm0_mode3"
#define GROUPNAME_PWM0_MODE4		"pwm0_mode4"
#define GROUPNAME_PWM0_MODE5		"pwm0_mode5"
#define GROUPNAME_PWM1_MODE1		"pwm1_mode1"
#define GROUPNAME_PWM1_MODE2		"pwm1_mode2"
#define GROUPNAME_PWM1_MODE3		"pwm1_mode3"
#define GROUPNAME_PWM1_MODE4		"pwm1_mode4"
#define GROUPNAME_PWM1_MODE5		"pwm1_mode5"
#define GROUPNAME_PWM2_MODE1		"pwm2_mode1"
#define GROUPNAME_PWM2_MODE2		"pwm2_mode2"
#define GROUPNAME_PWM2_MODE3		"pwm2_mode3"
#define GROUPNAME_PWM2_MODE4		"pwm2_mode4"
#define GROUPNAME_PWM2_MODE5		"pwm2_mode5"
#define GROUPNAME_PWM2_MODE6		"pwm2_mode6"
#define GROUPNAME_PWM3_MODE1		"pwm3_mode1"
#define GROUPNAME_PWM3_MODE2		"pwm3_mode2"
#define GROUPNAME_PWM3_MODE3		"pwm3_mode3"
#define GROUPNAME_PWM3_MODE4		"pwm3_mode4"
#define GROUPNAME_PWM3_MODE5		"pwm3_mode5"
#define GROUPNAME_PWM4_MODE2		"pwm4_mode2"
#define GROUPNAME_PWM5_MODE2		"pwm5_mode2"
#define GROUPNAME_PWM6_MODE2		"pwm6_mode2"
#define GROUPNAME_PWM7_MODE2		"pwm7_mode2"
#define GROUPNAME_SPI0_MODE1		"spi0_mode1"
#define GROUPNAME_SPI0_MODE2		"spi0_mode2"
#define GROUPNAME_SPI0_MODE3		"spi0_mode3"
#define GROUPNAME_SPI0_MODE4		"spi0_mode4"
#define GROUPNAME_SPI0_MODE5		"spi0_mode5"
#define GROUPNAME_SPI0_MODE6		"spi0_mode6"
#define GROUPNAME_SPI1_MODE3		"spi1_mode3"
#define GROUPNAME_JTAG_MODE1		"jtag_mode1"
#define GROUPNAME_TX_MIPI_MODE1		"tx_mipi_mode1"
#define GROUPNAME_TX_MIPI_MODE2		"tx_mipi_mode2"

#ifdef CONFIG_MACH_MERCURY
#define GROUPNAME_SR0_MIPI_MODE1	"sr0_mipi_mode1"
#define GROUPNAME_SR0_MIPI_MODE2	"sr0_mipi_mode2"
#define GROUPNAME_SR1_BT656		"sr1_bt656"
#define GROUPNAME_SR1_MIPI_MODE4	"sr1_mipi_mode4"
#endif

#define GROUPNAME_TTL_MODE1		"ttl_mode1"

/* common group function names */
#define FUNCTIONNAME_USB	GROUPNAME_USB
#define FUNCTIONNAME_USB1	GROUPNAME_USB1
#define FUNCTIONNAME_FUART	"fuart"
#define FUNCTIONNAME_UART0	GROUPNAME_UART0
#define FUNCTIONNAME_UART1	"uart1"
#define FUNCTIONNAME_UART2	"uart2"
#define FUNCTIONNAME_ETH	"eth"
#define FUNCTIONNAME_ETH1	"eth1"
#define FUNCTIONNAME_JTAG	"jtag"
#define FUNCTIONNAME_PWM0	"pwm0"
#define FUNCTIONNAME_PWM1	"pwm1"
#define FUNCTIONNAME_PWM2	"pwm2"
#define FUNCTIONNAME_PWM3	"pwm3"
#define FUNCTIONNAME_PWM4	"pwm4"
#define FUNCTIONNAME_PWM5	"pwm5"
#define FUNCTIONNAME_PWM6	"pwm6"
#define FUNCTIONNAME_PWM7	"pwm7"
#define FUNCTIONNAME_SDIO	"sdio"
#define FUNCTIONNAME_I2C0	"i2c0"
#define FUNCTIONNAME_I2C1	"i2c1"
#define FUNCTIONNAME_SPI0	"spi0"
#define FUNCTIONNAME_SPI1	"spi1"

#define FUNCTIONNAME_SR0_MIPI	"sr0_mipi"
#define FUNCTIONNAME_SR1_BT656	GROUPNAME_SR1_BT656
#define FUNCTIONNAME_SR1_MIPI	"sr1_mipi"

#define FUNCTIONNAME_TX_MIPI	"tx_mipi"
#define FUNCTIONNAME_TTL	"ttl"

/* shared structures */
struct msc313_pinctrl {
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl;
	struct regmap *regmap;
	const struct msc313_pinctrl_info *info;
};

struct msc313_pinctrl_function {
	const char *name;
	int reg;
	u16 mask;
	const char * const *groups;
	const u16 *values;
	int numgroups;
};

struct msc313_pinctrl_group {
	const char *name;
	const int *pins;
	const int numpins;
};

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

/* shared struct helpers */
#define MSTAR_PINCTRL_FUNCTION(n, r, m, g, v) \
	{ \
		.name = n,\
		.reg = r,\
		.mask = m,\
		.groups = g,\
		.values = v,\
		.numgroups = ARRAY_SIZE(g)\
	}

#define MSTAR_PINCTRL_GROUP(n, p) \
	{\
		.name = n,\
		.pins = p,\
		.numpins = ARRAY_SIZE(p)\
	}

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

/* shared functions */
int mstar_pinctrl_parse_groups(struct msc313_pinctrl *pinctrl);
int mstar_pinctrl_parse_functions(struct msc313_pinctrl *pinctrl);

extern const struct pinctrl_ops msc313_pinctrl_ops;
extern const struct pinmux_ops mstar_pinmux_ops;

/* Helpers for pins that have the same on the different chips */
#define COMMON_PIN(_model, _pinname) \
	PINCTRL_PIN(PIN_##_model##_##_pinname, PINNAME_##_pinname)

#define COMMON_FIXED_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, -1, 0, _name##_groups, NULL)
#define COMMON_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, _name##_values)
#define COMMON_FUNCTION_NULLVALUES(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, NULL)

/* Helpers for msc313/msc313e pins and groups */
#define MSC313_COMMON_PIN(_pinname) COMMON_PIN(MSC313, _pinname)
#define MSC313_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, msc313_##_name##_pins)

/* for ssd20xd pins */
#define SSD20XD_COMMON_PIN(_pinname) COMMON_PIN(SSD20XD, _pinname)

#define SSD20XD_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd20xd_##_name##_pins)

#define SSD20XD_MODE(_func, _modenum) (_modenum << SHIFT_SSD20XD_##_func)

#define SSD20XD_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSD20XD_##_NAME, \
	MASK_SSD20XD_##_NAME, ssd20xd_##_name##_groups, ssd20xd_##_name##_values)
