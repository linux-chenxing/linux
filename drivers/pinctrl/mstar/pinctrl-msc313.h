// SPDX-License-Identifier: GPL-2.0
/*
 * Common bits for msc313 style pinctrls
 */

#ifndef DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_H_
#define DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_H_

#define COMMON_FUNCTIONS \
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

/*
 * Common groups and register values that are used
 * for all chips so far.
 *
 * This maps functions to the groups that can handle
 * a function and the register bits that need to be
 * set to enable that function.
 */
static const char * const i2c0_groups[] = {
	GROUPNAME_I2C0_MODE1,
};
static const u16 i2c0_values[] = {
	BIT(0),
};

static const char * const i2c1_groups[] = {
	GROUPNAME_I2C1_MODE1,
};
static const u16 i2c1_values[] = {
	BIT(4)
};

static const char * const fuart_groups[] = {
	GROUPNAME_FUART_MODE1,
	GROUPNAME_FUART_MODE1_NOCTS,
};
static const u16 fuart_values[] = {
	BIT(0),
	BIT(0),
};

static const char * const uart0_groups[] = {
	GROUPNAME_FUART_RX_TX,
};

static const char * const uart1_groups[] = {
	GROUPNAME_UART1_MODE2,
	GROUPNAME_UART1_MODE2_RXONLY,
};
static const u16 uart1_values[] = {
	BIT(9),
	BIT(9),
};

static const char * const pwm0_groups[] = {
	GROUPNAME_PWM0_MODE3,
};
static const u16 pwm0_values[] = {
	BIT(1) | BIT(0),
};

static const char * const pwm1_groups[] = {
	GROUPNAME_PWM1_MODE3,
};
static const u16 pwm1_values[] = {
	BIT(3) | BIT(2),
};

static const char * const pwm2_groups[] = {
	GROUPNAME_PWM2_MODE2,
};
static const u16 pwm2_values[] = {
	BIT(5),
};

static const char * const pwm3_groups[] = {
	GROUPNAME_PWM3_MODE2,
};
static const u16 pwm3_values[] = {
	BIT(7),
};

static const char * const pwm4_groups[] = {
	GROUPNAME_PWM4_MODE2,
};
static const u16 pwm4_values[] = {
	BIT(9),
};

static const char * const pwm5_groups[] = {
	GROUPNAME_PWM5_MODE2,
};
static const u16 pwm5_values[] = {
	BIT(11),
};

static const char * const pwm6_groups[] = {
	GROUPNAME_PWM6_MODE2,
};
static const u16 pwm6_values[] = {
	BIT(13),
};

static const char * const pwm7_groups[] = {
	GROUPNAME_PWM7_MODE2,
};
static const u16 pwm7_values[] = {
	BIT(15),
};

static const char * const jtag_groups[] = {
	GROUPNAME_JTAG_MODE1,
};

static const char * const spi0_groups[] = {
	GROUPNAME_SPI0_MODE1,
	GROUPNAME_SPI0_MODE1,
};
static const u16 spi0_values[] = {
	BIT(0),
	BIT(1) | BIT(0),
};
static const char * const spi1_groups[] = {
	GROUPNAME_SPI1_MODE3,
};
static const u16 spi1_values[] = {
	BIT(5) | BIT(4),
};


/* eth */
static const char * const eth_groups[] = {
	GROUPNAME_ETH_MODE1,
};
static const u16 eth_values[] = {
	BIT(2),
};

/* usb */
static const char * const usb_groups[] = {
	GROUPNAME_USB,
};
static const char * const usb1_groups[] = {
	GROUPNAME_USB1,
};

/* sdio */
static const char * const sdio_groups[] = {
	GROUPNAME_SDIO_MODE1,
};
static const u16 sdio_values[] = {
	BIT(8),
};

static const unsigned int sd_drivestrengths[] = {4, 8};

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

#endif /* DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_H_ */
