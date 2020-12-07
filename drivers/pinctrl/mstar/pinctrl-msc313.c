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

#include "../core.h"
#include "../devicetree.h"
#include "../pinconf.h"
#include "../pinmux.h"

#include "pinctrl-mstar.h"

#define DRIVER_NAME "pinctrl-msc313"

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

static const char * const usb_groups[] = {
	GROUPNAME_USB,
};
static const char * const usb1_groups[] = {
	GROUPNAME_USB1,
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

static const char * const eth_groups[] = {
	GROUPNAME_ETH_MODE1,
};
static const u16 eth_values[] = {
	BIT(2),
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
static const char * const sdio_groups[] = {
	GROUPNAME_SDIO_MODE1,
};
static const u16 sdio_values[] = {
	BIT(8),
};

#ifdef CONFIG_MACH_MERCURY
static const char * const sr0_mipi_groups[]  = { GROUPNAME_SR0_MIPI_MODE1, GROUPNAME_SR0_MIPI_MODE2};
static const u16	  sr0_mipi_values[]  = { BIT(8), BIT(9) };
static const char * const sr1_bt656_groups[] = { GROUPNAME_SR1_BT656 };
static const u16	  sr1_bt656_values[] = { BIT(12) };
static const char * const sr1_mipi_groups[]  = { GROUPNAME_SR1_MIPI_MODE4 };
static const u16	  sr1_mipi_values[]  = { BIT(15) };

static const char * const tx_mipi_groups[] = { GROUPNAME_TX_MIPI_MODE1, GROUPNAME_TX_MIPI_MODE2 };
static const u16	  tx_mipi_values[] = { BIT(0), BIT(1) };
#endif

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

#ifdef CONFIG_MACH_INFINITY
/* msc313/msc313e */
/* pinctrl pins */
static struct pinctrl_pin_desc msc313_pins[] = {
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
/* eth */
static const int msc313_eth_mode1_pins[] = {
	PIN_MSC313_ETH_RN,
	PIN_MSC313_ETH_RP,
	PIN_MSC313_ETH_TN,
	PIN_MSC313_ETH_TP,
};
/* fuart */
static const int msc313_fuart_mode1_pins[] = {
	PIN_MSC313_FUART_RX,
	PIN_MSC313_FUART_TX,
	PIN_MSC313_FUART_CTS,
	PIN_MSC313_FUART_RTS,
};
static const int msc313_fuart_mode1_nocts_pins[] = {
	PIN_MSC313_FUART_RX,
	PIN_MSC313_FUART_TX,
	PIN_MSC313_FUART_RTS,
};
/* uart1 */
static const int msc313_uart1_mode2_pins[] = {
	PIN_MSC313_FUART_CTS,
	PIN_MSC313_FUART_RTS,
};
/* i2c1 */
static const int msc313_i2c1_mode1_pins[] = {
	PIN_MSC313_I2C1_SCL,
	PIN_MSC313_I2C1_SDA,
};
/* spi0 */
static const int msc313_spi0_mode1_pins[] = {
	PIN_MSC313_SPI0_CZ,
	PIN_MSC313_SPI0_CK,
	PIN_MSC313_SPI0_DI,
	PIN_MSC313_SPI0_DO,
};
static const int msc313_spi0_mode3_pins[] = {
	PIN_MSC313_FUART_RX,
	PIN_MSC313_FUART_TX,
	PIN_MSC313_FUART_CTS,
	PIN_MSC313_FUART_RTS,
};
/* pwm0 */
static const int msc313_pwm0_mode3_pins[] = {
	PIN_MSC313_FUART_RX,
};

/* pwm1 */
static const int msc313_pwm1_mode3_pins[] = {
	PIN_MSC313_FUART_TX,
};

/* pwm2 */
static const int msc313_pwm2_mode2_pins[] = {
	PIN_MSC313_FUART_CTS,
};
/* pwm3 */
static const int msc313_pwm3_mode2_pins[] = {
	PIN_MSC313_FUART_RTS,
};
/* pwm4 */
static const int msc313_pwm4_mode2_pins[] = {
	PIN_MSC313_SPI0_CZ,
};
/* pwm5 */
static const int msc313_pwm5_mode2_pins[] = {
	PIN_MSC313_SPI0_CK,
};
/* pwm 6 */
static const int msc313_pwm6_mode2_pins[] = {
	PIN_MSC313_SPI0_DI,
};
/* pwm 7*/
static const int msc313_pwm7_mode2_pins[] = {
	PIN_MSC313_SPI0_DO,
};
/* spi1 */
static const int msc313_spi1_mode3_pins[] = {
	PIN_MSC313_SD_D0,
	PIN_MSC313_SD_D1,
	PIN_MSC313_SD_D2,
	PIN_MSC313_SD_D3,
};
/* sdio */
static const int msc313_sdio_mode1_pins[] = {
	PIN_MSC313_SD_CLK,
	PIN_MSC313_SD_CMD,
	PIN_MSC313_SD_D0,
	PIN_MSC313_SD_D1,
	PIN_MSC313_SD_D2,
	PIN_MSC313_SD_D3,
};
/* usb */
static const int msc313_usb_pins[] = {
	PIN_MSC313_USB_DM,
	PIN_MSC313_USB_DP,
};

static const struct msc313_pinctrl_group msc313_pinctrl_groups[] = {
	MSC313_PINCTRL_GROUP(USB, usb),
	MSC313_PINCTRL_GROUP(ETH_MODE1, eth_mode1),
	MSC313_PINCTRL_GROUP(FUART_MODE1, fuart_mode1),
	MSC313_PINCTRL_GROUP(FUART_MODE1_NOCTS, fuart_mode1_nocts),
	MSC313_PINCTRL_GROUP(UART1_MODE2, uart1_mode2),
	MSC313_PINCTRL_GROUP(I2C1_MODE1, i2c1_mode1),
	MSC313_PINCTRL_GROUP(PWM0_MODE3, pwm0_mode3),
	MSC313_PINCTRL_GROUP(PWM1_MODE3, pwm1_mode3),
	MSC313_PINCTRL_GROUP(PWM2_MODE2, pwm2_mode2),
	MSC313_PINCTRL_GROUP(PWM3_MODE2, pwm3_mode2),
	MSC313_PINCTRL_GROUP(PWM4_MODE2, pwm4_mode2),
	MSC313_PINCTRL_GROUP(PWM5_MODE2, pwm5_mode2),
	MSC313_PINCTRL_GROUP(PWM6_MODE2, pwm6_mode2),
	MSC313_PINCTRL_GROUP(PWM7_MODE2, pwm7_mode2),
	MSC313_PINCTRL_GROUP(SPI0_MODE1, spi0_mode1),
	MSC313_PINCTRL_GROUP(SPI0_MODE3, spi0_mode3),
	MSC313_PINCTRL_GROUP(SPI1_MODE3, spi1_mode3),
	MSC313_PINCTRL_GROUP(SDIO_MODE1, sdio_mode1),
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
/* ssd20xd */
#define SSD20XD_COMMON_PIN(_pinname) COMMON_PIN(SSD20XD, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssd20xd_pins[] = {
	SSD20XD_COMMON_PIN(GPIO12),
	SSD20XD_COMMON_PIN(GPIO13),
	SSD20XD_COMMON_PIN(GPIO14),
	SSD20XD_COMMON_PIN(GPIO85),
	SSD20XD_COMMON_PIN(GPIO86),
	SSD20XD_COMMON_PIN(GPIO90),
	SSD20XD_COMMON_PIN(GPIO47),
	SSD20XD_COMMON_PIN(GPIO48),
	SSD20XD_COMMON_PIN(UART1_RX),
	SSD20XD_COMMON_PIN(UART1_TX),
	SSD20XD_COMMON_PIN(FUART_RX),
	SSD20XD_COMMON_PIN(FUART_TX),
	SSD20XD_COMMON_PIN(FUART_CTS),
	SSD20XD_COMMON_PIN(FUART_RTS),
	SSD20XD_COMMON_PIN(TTL0),
	SSD20XD_COMMON_PIN(TTL1),
	SSD20XD_COMMON_PIN(TTL2),
	SSD20XD_COMMON_PIN(TTL3),
	SSD20XD_COMMON_PIN(TTL4),
	SSD20XD_COMMON_PIN(TTL5),
	SSD20XD_COMMON_PIN(USB_DP),
	SSD20XD_COMMON_PIN(USB_DM),
	SSD20XD_COMMON_PIN(TTL6),
	SSD20XD_COMMON_PIN(TTL7),
	SSD20XD_COMMON_PIN(TTL8),
	SSD20XD_COMMON_PIN(TTL9),
	SSD20XD_COMMON_PIN(TTL10),
	SSD20XD_COMMON_PIN(TTL11),
	SSD20XD_COMMON_PIN(TTL12),
	SSD20XD_COMMON_PIN(TTL13),
	SSD20XD_COMMON_PIN(TTL14),
	SSD20XD_COMMON_PIN(TTL15),
	SSD20XD_COMMON_PIN(TTL16),
	SSD20XD_COMMON_PIN(TTL17),
	SSD20XD_COMMON_PIN(TTL18),
	SSD20XD_COMMON_PIN(TTL19),
	SSD20XD_COMMON_PIN(TTL20),
	SSD20XD_COMMON_PIN(TTL21),
	SSD20XD_COMMON_PIN(TTL22),
	SSD20XD_COMMON_PIN(TTL23),
	SSD20XD_COMMON_PIN(TTL24),
	SSD20XD_COMMON_PIN(TTL25),
	SSD20XD_COMMON_PIN(TTL26),
	SSD20XD_COMMON_PIN(TTL27),
	SSD20XD_COMMON_PIN(SD_CLK),
	SSD20XD_COMMON_PIN(SD_CMD),
	SSD20XD_COMMON_PIN(SD_D0),
	SSD20XD_COMMON_PIN(SD_D1),
	SSD20XD_COMMON_PIN(SD_D2),
	SSD20XD_COMMON_PIN(SD_D3),
	SSD20XD_COMMON_PIN(GPIO0),
	SSD20XD_COMMON_PIN(GPIO1),
	SSD20XD_COMMON_PIN(GPIO2),
	SSD20XD_COMMON_PIN(GPIO3),
	SSD20XD_COMMON_PIN(ETH_RN),
	SSD20XD_COMMON_PIN(ETH_RP),
	SSD20XD_COMMON_PIN(ETH_TN),
	SSD20XD_COMMON_PIN(ETH_TP),
	SSD20XD_COMMON_PIN(USB1_DP),
	SSD20XD_COMMON_PIN(USB1_DM),
	SSD20XD_COMMON_PIN(GPIO4),
	SSD20XD_COMMON_PIN(GPIO5),
	SSD20XD_COMMON_PIN(GPIO6),
	SSD20XD_COMMON_PIN(GPIO7),
	SSD20XD_COMMON_PIN(UART2_RX),
	SSD20XD_COMMON_PIN(UART2_TX),
	SSD20XD_COMMON_PIN(GPIO10),
	SSD20XD_COMMON_PIN(GPIO11),
};

/* mux pin groupings */
static const int ssd20xd_fuart_mode1_pins[] = {
	PIN_SSD20XD_FUART_RX,
	PIN_SSD20XD_FUART_TX,
	PIN_SSD20XD_FUART_CTS,
	PIN_SSD20XD_FUART_RTS
};
static const int ssd20xd_fuart_mode2_pins[] = {
	PIN_SSD20XD_FUART_RX,
	PIN_SSD20XD_FUART_TX,
};
static const int ssd20xd_fuart_mode3_pins[] = {
	PIN_SSD20XD_TTL0,
	PIN_SSD20XD_TTL1,
};
static const int ssd20xd_fuart_mode4_pins[] = {
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
};
static const int ssd20xd_fuart_mode5_pins[] = {
	PIN_SSD20XD_GPIO0,
	PIN_SSD20XD_GPIO1,
	PIN_SSD20XD_GPIO2,
	PIN_SSD20XD_GPIO3,
};
static const int ssd20xd_fuart_mode6_pins[] = {
	PIN_SSD20XD_GPIO4,
	PIN_SSD20XD_GPIO5,
	PIN_SSD20XD_TTL16,
};
static const int ssd20xd_fuart_mode7_pins[] = {
	PIN_SSD20XD_SD_D1,
	PIN_SSD20XD_SD_D0,
};
static const int ssd20xd_uart1_mode1_pins[] = {
	PIN_SSD20XD_UART1_RX,
	PIN_SSD20XD_UART1_TX,
};
static const int ssd20xd_uart1_mode2_pins[] = {
	PIN_SSD20XD_TTL15,
	PIN_SSD20XD_TTL15,
};
static const int ssd20xd_uart1_mode3_pins[] = {
	PIN_SSD20XD_GPIO13,
	PIN_SSD20XD_GPIO14,
};
static const int ssd20xd_uart1_mode4_pins[] = {
	PIN_SSD20XD_GPIO10,
	PIN_SSD20XD_GPIO11,
};
static const int ssd20xd_usb_pins[] = {
	PIN_SSD20XD_USB_DP,
	PIN_SSD20XD_USB_DM,
};
static const int ssd20xd_usb1_pins[] = {
	PIN_SSD20XD_USB1_DP,
	PIN_SSD20XD_USB1_DM,
};
/* sdio */
static const int ssd20xd_sdio_mode1_pins[] = {
	PIN_SSD20XD_SD_D1,
	PIN_SSD20XD_SD_D0,
	PIN_SSD20XD_SD_CLK,
	PIN_SSD20XD_SD_CMD,
	PIN_SSD20XD_SD_D3,
	PIN_SSD20XD_SD_D2,
};
/* eth */
static const int ssd20xd_eth_mode1_pins[] = {
	PIN_SSD20XD_ETH_RN,
	PIN_SSD20XD_ETH_RP,
	PIN_SSD20XD_ETH_TN,
	PIN_SSD20XD_ETH_TP,
};
/* eth1 */
static const int ssd20xd_eth1_mode1_pins[] = {
	PIN_SSD20XD_TTL16,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
	PIN_SSD20XD_TTL24,
};
static const int ssd20xd_eth1_mode2_pins[] = {
	PIN_SSD20XD_TTL16,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
	PIN_SSD20XD_TTL24,
};
static const int ssd20xd_eth1_mode3_pins[] = {
	PIN_SSD20XD_TTL16,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
	PIN_SSD20XD_TTL24,
};
static const int ssd20xd_eth1_mode4_pins[] = {
	PIN_SSD20XD_GPIO0,
	PIN_SSD20XD_GPIO1,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
};
static const int ssd20xd_eth1_mode5_pins[] = {
	PIN_SSD20XD_GPIO0,
	PIN_SSD20XD_GPIO1,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
	PIN_SSD20XD_TTL24,
	PIN_SSD20XD_TTL25,
	PIN_SSD20XD_TTL26,
	PIN_SSD20XD_TTL27,
};
/* ttl */
static const int ssd20xd_ttl_mode1_pins[] = {
	PIN_SSD20XD_TTL0,
	PIN_SSD20XD_TTL1,
	PIN_SSD20XD_TTL2,
	PIN_SSD20XD_TTL3,
	PIN_SSD20XD_TTL4,
	PIN_SSD20XD_TTL5,
	PIN_SSD20XD_TTL6,
	PIN_SSD20XD_TTL7,
	PIN_SSD20XD_TTL8,
	PIN_SSD20XD_TTL9,
	PIN_SSD20XD_TTL10,
	PIN_SSD20XD_TTL11,
	PIN_SSD20XD_TTL12,
	PIN_SSD20XD_TTL13,
	PIN_SSD20XD_TTL14,
	PIN_SSD20XD_TTL15,
	PIN_SSD20XD_TTL16,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
	PIN_SSD20XD_TTL20,
	PIN_SSD20XD_TTL21,
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
	PIN_SSD20XD_TTL24,
	PIN_SSD20XD_TTL25,
	PIN_SSD20XD_TTL26,
	PIN_SSD20XD_TTL27,
};
/* tx mipi */
static const int ssd20xd_tx_mipi_mode1_pins[] = {
	PIN_SSD20XD_TTL6,
	PIN_SSD20XD_TTL7,
	PIN_SSD20XD_TTL8,
	PIN_SSD20XD_TTL9,
	PIN_SSD20XD_TTL10,
	PIN_SSD20XD_TTL11,
	PIN_SSD20XD_TTL12,
	PIN_SSD20XD_TTL13,
	PIN_SSD20XD_TTL14,
	PIN_SSD20XD_TTL15,
};
static const int ssd20xd_tx_mipi_mode2_pins[] = {
	PIN_SSD20XD_TTL6,
	PIN_SSD20XD_TTL7,
	PIN_SSD20XD_TTL8,
	PIN_SSD20XD_TTL9,
	PIN_SSD20XD_TTL10,
	PIN_SSD20XD_TTL11,
};
/* i2c0 */
static const int ssd20xd_i2c0_mode2_pins[] = {
	PIN_SSD20XD_TTL1,
	PIN_SSD20XD_TTL2,
};
static const int ssd20xd_i2c0_mode3_pins[] = {
	PIN_SSD20XD_TTL14,
	PIN_SSD20XD_TTL15,
};
static const int ssd20xd_i2c0_mode4_pins[] = {
	PIN_SSD20XD_GPIO6,
	PIN_SSD20XD_GPIO7,
};
/* i2c1 */
static const int ssd20xd_i2c1_mode1_pins[] = {
	PIN_SSD20XD_GPIO2,
	PIN_SSD20XD_GPIO3,
};
static const int ssd20xd_i2c1_mode3_pins[] = {
	PIN_SSD20XD_FUART_CTS,
	PIN_SSD20XD_FUART_RTS,
};
static const int ssd20xd_i2c1_mode4_pins[] = {
	PIN_SSD20XD_TTL22,
	PIN_SSD20XD_TTL23,
};
static const int ssd20xd_i2c1_mode5_pins[] = {
	PIN_SSD20XD_SD_CLK,
	PIN_SSD20XD_SD_CMD,
};
/* spi0 */
static const int ssd20xd_spi0_mode1_pins[] = {
	PIN_SSD20XD_SD_CLK,
	PIN_SSD20XD_SD_CMD,
	PIN_SSD20XD_SD_D0,
	PIN_SSD20XD_SD_D1,
};
static const int ssd20xd_spi0_mode2_pins[] = {
	PIN_SSD20XD_TTL16,
	PIN_SSD20XD_TTL17,
	PIN_SSD20XD_TTL18,
	PIN_SSD20XD_TTL19,
};
static const int ssd20xd_spi0_mode3_pins[] = {
	PIN_SSD20XD_GPIO4,
	PIN_SSD20XD_GPIO5,
	PIN_SSD20XD_GPIO6,
	PIN_SSD20XD_GPIO7,
};
static const int ssd20xd_spi0_mode4_pins[] = {
	PIN_SSD20XD_FUART_RX,
	PIN_SSD20XD_FUART_TX,
	PIN_SSD20XD_FUART_CTS,
	PIN_SSD20XD_FUART_RTS,
};
static const int ssd20xd_spi0_mode6_pins[] = {
	PIN_SSD20XD_GPIO0,
	PIN_SSD20XD_GPIO1,
	PIN_SSD20XD_GPIO2,
	PIN_SSD20XD_GPIO3,
};
/* pwm0 */
static const int ssd20xd_pwm0_mode1_pins[] = {
	PIN_SSD20XD_GPIO0,
};
static const int ssd20xd_pwm0_mode2_pins[] = {
	PIN_SSD20XD_TTL23,
};
static const int ssd20xd_pwm0_mode3_pins[] = {
	PIN_SSD20XD_GPIO4,
};
static const int ssd20xd_pwm0_mode4_pins[] = {
	PIN_SSD20XD_GPIO14,
};
static const int ssd20xd_pwm0_mode5_pins[] = {
	PIN_SSD20XD_TTL0,
};
/* pwm1 */
static const int ssd20xd_pwm1_mode1_pins[] = {
	PIN_SSD20XD_GPIO1,
};
static const int ssd20xd_pwm1_mode2_pins[] = {
	PIN_SSD20XD_TTL12,
};
static const int ssd20xd_pwm1_mode3_pins[] = {
	PIN_SSD20XD_TTL22,
};
static const int ssd20xd_pwm1_mode4_pins[] = {
	PIN_SSD20XD_GPIO5,
};
/* pwm2 */
static const int ssd20xd_pwm2_mode1_pins[] = {
	PIN_SSD20XD_GPIO2,
};
static const int ssd20xd_pwm2_mode2_pins[] = {
	PIN_SSD20XD_GPIO11,
};
static const int ssd20xd_pwm2_mode4_pins[] = {
	PIN_SSD20XD_TTL21,
};
static const int ssd20xd_pwm2_mode5_pins[] = {
	PIN_SSD20XD_FUART_TX,
};
static const int ssd20xd_pwm2_mode6_pins[] = {
	PIN_SSD20XD_SD_D1,
};
/* pwm3 */
static const int ssd20xd_pwm3_mode1_pins[] = {
	PIN_SSD20XD_GPIO3,
};
static const int ssd20xd_pwm3_mode2_pins[] = {
	PIN_SSD20XD_GPIO7,
};
static const int ssd20xd_pwm3_mode3_pins[] = {
	PIN_SSD20XD_GPIO13,
};
static const int ssd20xd_pwm3_mode4_pins[] = {
	PIN_SSD20XD_TTL20,
};
static const int ssd20xd_pwm3_mode5_pins[] = {
	PIN_SSD20XD_FUART_RX,
};
#define SSD20XD_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd20xd_##_name##_pins)

static const struct msc313_pinctrl_group ssd20xd_pinctrl_groups[] = {
	SSD20XD_PINCTRL_GROUP(FUART_MODE1, fuart_mode1),
	SSD20XD_PINCTRL_GROUP(FUART_MODE2, fuart_mode2),
	SSD20XD_PINCTRL_GROUP(FUART_MODE3, fuart_mode3),
	SSD20XD_PINCTRL_GROUP(FUART_MODE4, fuart_mode4),
	SSD20XD_PINCTRL_GROUP(FUART_MODE5, fuart_mode5),
	SSD20XD_PINCTRL_GROUP(FUART_MODE6, fuart_mode6),
	SSD20XD_PINCTRL_GROUP(FUART_MODE7, fuart_mode7),
	SSD20XD_PINCTRL_GROUP(UART1_MODE1, uart1_mode1),
	SSD20XD_PINCTRL_GROUP(UART1_MODE2, uart1_mode2),
	SSD20XD_PINCTRL_GROUP(UART1_MODE3, uart1_mode3),
	SSD20XD_PINCTRL_GROUP(UART1_MODE4, uart1_mode4),
	SSD20XD_PINCTRL_GROUP(USB, usb),
	SSD20XD_PINCTRL_GROUP(USB1, usb1),
	SSD20XD_PINCTRL_GROUP(SDIO_MODE1, sdio_mode1),
	SSD20XD_PINCTRL_GROUP(ETH_MODE1, eth_mode1),
	SSD20XD_PINCTRL_GROUP(ETH1_MODE1, eth1_mode1),
	SSD20XD_PINCTRL_GROUP(ETH1_MODE2, eth1_mode2),
	SSD20XD_PINCTRL_GROUP(ETH1_MODE3, eth1_mode3),
	SSD20XD_PINCTRL_GROUP(ETH1_MODE4, eth1_mode4),
	SSD20XD_PINCTRL_GROUP(ETH1_MODE5, eth1_mode5),
	SSD20XD_PINCTRL_GROUP(TTL_MODE1, ttl_mode1),
	SSD20XD_PINCTRL_GROUP(TX_MIPI_MODE1, tx_mipi_mode1),
	SSD20XD_PINCTRL_GROUP(TX_MIPI_MODE2, tx_mipi_mode2),
	SSD20XD_PINCTRL_GROUP(I2C0_MODE2, i2c0_mode2),
	SSD20XD_PINCTRL_GROUP(I2C0_MODE3, i2c0_mode3),
	SSD20XD_PINCTRL_GROUP(I2C0_MODE4, i2c0_mode4),
	SSD20XD_PINCTRL_GROUP(I2C1_MODE1, i2c1_mode1),
	SSD20XD_PINCTRL_GROUP(I2C1_MODE3, i2c1_mode3),
	SSD20XD_PINCTRL_GROUP(I2C1_MODE4, i2c1_mode4),
	SSD20XD_PINCTRL_GROUP(I2C1_MODE5, i2c1_mode5),
	SSD20XD_PINCTRL_GROUP(SPI0_MODE1, spi0_mode1),
	SSD20XD_PINCTRL_GROUP(SPI0_MODE2, spi0_mode2),
	SSD20XD_PINCTRL_GROUP(SPI0_MODE3, spi0_mode3),
	SSD20XD_PINCTRL_GROUP(SPI0_MODE4, spi0_mode4),
	SSD20XD_PINCTRL_GROUP(SPI0_MODE6, spi0_mode6),
	SSD20XD_PINCTRL_GROUP(PWM0_MODE1, pwm0_mode1),
	SSD20XD_PINCTRL_GROUP(PWM0_MODE2, pwm0_mode2),
	SSD20XD_PINCTRL_GROUP(PWM0_MODE3, pwm0_mode3),
	SSD20XD_PINCTRL_GROUP(PWM0_MODE4, pwm0_mode4),
	SSD20XD_PINCTRL_GROUP(PWM0_MODE5, pwm0_mode5),
	SSD20XD_PINCTRL_GROUP(PWM1_MODE1, pwm1_mode1),
	SSD20XD_PINCTRL_GROUP(PWM1_MODE2, pwm1_mode2),
	SSD20XD_PINCTRL_GROUP(PWM1_MODE3, pwm1_mode3),
	SSD20XD_PINCTRL_GROUP(PWM1_MODE4, pwm1_mode4),
	SSD20XD_PINCTRL_GROUP(PWM2_MODE1, pwm2_mode1),
	SSD20XD_PINCTRL_GROUP(PWM2_MODE2, pwm2_mode2),
	SSD20XD_PINCTRL_GROUP(PWM2_MODE4, pwm2_mode4),
	SSD20XD_PINCTRL_GROUP(PWM2_MODE5, pwm2_mode5),
	SSD20XD_PINCTRL_GROUP(PWM2_MODE6, pwm2_mode6),
	SSD20XD_PINCTRL_GROUP(PWM3_MODE1, pwm3_mode1),
	SSD20XD_PINCTRL_GROUP(PWM3_MODE2, pwm3_mode2),
	SSD20XD_PINCTRL_GROUP(PWM3_MODE3, pwm3_mode3),
	SSD20XD_PINCTRL_GROUP(PWM3_MODE4, pwm3_mode4),
	SSD20XD_PINCTRL_GROUP(PWM3_MODE5, pwm3_mode5),
};

/* chip specific functions */
/* eth1 */
static const char * const ssd20xd_eth1_groups[] = {
	GROUPNAME_ETH1_MODE1,
	GROUPNAME_ETH1_MODE2,
	GROUPNAME_ETH1_MODE3,
	GROUPNAME_ETH1_MODE4,
	GROUPNAME_ETH1_MODE5,
};
static const u16 ssd20xd_eth1_values[] = {
	SSD20XD_MODE(ETH1, 1),
	SSD20XD_MODE(ETH1, 2),
	SSD20XD_MODE(ETH1, 3),
	SSD20XD_MODE(ETH1, 4),
	SSD20XD_MODE(ETH1, 5),
};
/* fuart */
static const char * const ssd20xd_fuart_groups[] = {
	GROUPNAME_FUART_MODE1,
	GROUPNAME_FUART_MODE2,
	GROUPNAME_FUART_MODE3,
	GROUPNAME_FUART_MODE4,
	GROUPNAME_FUART_MODE5,
	GROUPNAME_FUART_MODE6,
	GROUPNAME_FUART_MODE7,
};
static const u16 ssd20xd_fuart_values[] = {
	SSD20XD_MODE(FUART, 1),
	SSD20XD_MODE(FUART, 2),
	SSD20XD_MODE(FUART, 3),
	SSD20XD_MODE(FUART, 4),
	SSD20XD_MODE(FUART, 5),
	SSD20XD_MODE(FUART, 6),
	SSD20XD_MODE(FUART, 7),
};
static const char * const ssd20xd_uart0_groups[] = {
};
static const u16 ssd20xd_uart0_values[] = {
};
static const char * const ssd20xd_uart1_groups[] = {
	GROUPNAME_UART1_MODE1
	GROUPNAME_UART1_MODE2
	GROUPNAME_UART1_MODE3
	GROUPNAME_UART1_MODE4
};
static const u16 ssd20xd_uart1_values[] = {
	SSD20XD_MODE(UART1, 1),
	SSD20XD_MODE(UART1, 2),
	SSD20XD_MODE(UART1, 3),
	SSD20XD_MODE(UART1, 4),
};
static const char * const ssd20xd_uart2_groups[] = {
};
static const u16 ssd20xd_uart2_values[] = {
};
static const char * const ssd20xd_ttl_groups[] = {
	GROUPNAME_TTL_MODE1,
};
static const u16 ssd20xd_ttl_values[] = {
	SSD20XD_MODE(TTL, 1),
};
static const char * const ssd20xd_tx_mipi_groups[] = {
	GROUPNAME_TX_MIPI_MODE1,
	GROUPNAME_TX_MIPI_MODE2,
};
static const u16 ssd20xd_tx_mipi_values[] = {
	SSD20XD_MODE(TX_MIPI, 1),
	SSD20XD_MODE(TX_MIPI, 2),
};
static const char * const ssd20xd_i2c0_groups[] = {
	GROUPNAME_I2C0_MODE2,
	GROUPNAME_I2C0_MODE3,
	GROUPNAME_I2C0_MODE4,
};
static const u16 ssd20xd_i2c0_values[] = {
	SSD20XD_MODE(I2C0, 2),
	SSD20XD_MODE(I2C0, 3),
	SSD20XD_MODE(I2C0, 4),
};
static const char * const ssd20xd_i2c1_groups[] = {
	GROUPNAME_I2C1_MODE1,
	GROUPNAME_I2C1_MODE3,
	GROUPNAME_I2C1_MODE4,
	GROUPNAME_I2C1_MODE5,
};
static const u16 ssd20xd_i2c1_values[] = {
	SSD20XD_MODE(I2C1, 1),
	SSD20XD_MODE(I2C1, 3),
	SSD20XD_MODE(I2C1, 4),
	SSD20XD_MODE(I2C1, 5),
};
static const char * const ssd20xd_spi0_groups[] = {
	GROUPNAME_SPI0_MODE1,
	GROUPNAME_SPI0_MODE2,
	GROUPNAME_SPI0_MODE3,
	GROUPNAME_SPI0_MODE4,
	GROUPNAME_SPI0_MODE6,
};
static const u16 ssd20xd_spi0_values[] = {
	SSD20XD_MODE(SPI0, 1),
	SSD20XD_MODE(SPI0, 2),
	SSD20XD_MODE(SPI0, 3),
	SSD20XD_MODE(SPI0, 4),
	SSD20XD_MODE(SPI0, 6),
};

/* pwm0 */
static const char * const ssd20xd_pwm0_groups[] = {
	GROUPNAME_PWM0_MODE1,
	GROUPNAME_PWM0_MODE2,
	GROUPNAME_PWM0_MODE3,
	GROUPNAME_PWM0_MODE4,
	GROUPNAME_PWM0_MODE5,
};

static const u16 ssd20xd_pwm0_values[] = {
	SSD20XD_MODE(PWM0, 1),
	SSD20XD_MODE(PWM0, 2),
	SSD20XD_MODE(PWM0, 3),
	SSD20XD_MODE(PWM0, 4),
	SSD20XD_MODE(PWM0, 5),
};

/* pwm1 */
static const char * const ssd20xd_pwm1_groups[] = {
	GROUPNAME_PWM1_MODE1,
	GROUPNAME_PWM1_MODE2,
	GROUPNAME_PWM1_MODE3,
	GROUPNAME_PWM1_MODE4,
};

static const u16 ssd20xd_pwm1_values[] = {
	SSD20XD_MODE(PWM1, 1),
	SSD20XD_MODE(PWM1, 2),
	SSD20XD_MODE(PWM1, 3),
	SSD20XD_MODE(PWM1, 4),
};

/* pwm2 */
static const char * const ssd20xd_pwm2_groups[] = {
	GROUPNAME_PWM2_MODE1,
	GROUPNAME_PWM2_MODE2,
	GROUPNAME_PWM2_MODE4,
	GROUPNAME_PWM2_MODE5,
	GROUPNAME_PWM2_MODE6,
};

static const u16 ssd20xd_pwm2_values[] = {
	SSD20XD_MODE(PWM2, 1),
	SSD20XD_MODE(PWM2, 2),
	SSD20XD_MODE(PWM2, 4),
	SSD20XD_MODE(PWM2, 5),
	SSD20XD_MODE(PWM2, 6),
};

/* pwm3 */
static const char * const ssd20xd_pwm3_groups[] = {
	GROUPNAME_PWM3_MODE1,
	GROUPNAME_PWM3_MODE2,
	GROUPNAME_PWM3_MODE3,
	GROUPNAME_PWM3_MODE4,
	GROUPNAME_PWM3_MODE5,
};

static const u16 ssd20xd_pwm3_values[] = {
	SSD20XD_MODE(PWM3, 1),
	SSD20XD_MODE(PWM3, 2),
	SSD20XD_MODE(PWM3, 3),
	SSD20XD_MODE(PWM3, 4),
	SSD20XD_MODE(PWM3, 5),
};

#define SSD20XD_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSD20XD_##_NAME, \
	MASK_SSD20XD_##_NAME, ssd20xd_##_name##_groups, ssd20xd_##_name##_values)

static const struct msc313_pinctrl_function ssd20xd_pinctrl_functions[] = {
	SSD20XD_FUNCTION(FUART, fuart),
	SSD20XD_FUNCTION(UART0, uart0),
	SSD20XD_FUNCTION(UART1, uart1),
	SSD20XD_FUNCTION(UART2, uart2),
	COMMON_FIXED_FUNCTION(USB, usb),
	SSD20XD_FUNCTION(I2C0, i2c0),
	SSD20XD_FUNCTION(I2C1, i2c1),
	COMMON_FUNCTION(SDIO, sdio),
	COMMON_FUNCTION(ETH, eth),
	SSD20XD_FUNCTION(ETH1, eth1),
	SSD20XD_FUNCTION(TTL, ttl),
	SSD20XD_FUNCTION(TX_MIPI, tx_mipi),
	SSD20XD_FUNCTION(SPI0, spi0),
	SSD20XD_FUNCTION(PWM0, pwm0),
	SSD20XD_FUNCTION(PWM1, pwm1),
	SSD20XD_FUNCTION(PWM2, pwm2),
	SSD20XD_FUNCTION(PWM3, pwm3),
};

static const struct msc313_pinctrl_pinconf ssd20xd_configurable_pins[] = {
	SD_PINS(SSD20XD),
};

MSTAR_PINCTRL_INFO(ssd20xd);
#endif /* infinity */

#ifdef CONFIG_MACH_MERCURY
/* ssc8336 */
#define SSC8336N_COMMON_PIN(_pinname) COMMON_PIN(SSC8336N, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssc8336n_pins[] = {
	SSC8336N_COMMON_PIN(USB_DM1),
	SSC8336N_COMMON_PIN(USB_DP1),
	SSC8336N_COMMON_PIN(USB_DM),
	SSC8336N_COMMON_PIN(USB_DP),
	SSC8336N_COMMON_PIN(USB_CID),
	SSC8336N_COMMON_PIN(FUART_RX),
	SSC8336N_COMMON_PIN(FUART_TX),
	SSC8336N_COMMON_PIN(FUART_CTS),
	SSC8336N_COMMON_PIN(FUART_RTS),
	SSC8336N_COMMON_PIN(SPI0_DO),
	SSC8336N_COMMON_PIN(SPI0_DI),
	SSC8336N_COMMON_PIN(SPI0_CK),
	SSC8336N_COMMON_PIN(SPI0_CZ),
	SSC8336N_COMMON_PIN(SPI0_CZ1),
	SSC8336N_COMMON_PIN(I2C0_SCL),
	SSC8336N_COMMON_PIN(I2C0_SDA),
	SSC8336N_COMMON_PIN(SD_D1),
	SSC8336N_COMMON_PIN(SD_D0),
	SSC8336N_COMMON_PIN(SD_CLK),
	SSC8336N_COMMON_PIN(SD_CMD),
	SSC8336N_COMMON_PIN(SD_D3),
	SSC8336N_COMMON_PIN(SD_D2),
	SSC8336N_COMMON_PIN(SR0_D2),
	SSC8336N_COMMON_PIN(SR0_D3),
	SSC8336N_COMMON_PIN(SR0_D4),
	SSC8336N_COMMON_PIN(SR0_D5),
	SSC8336N_COMMON_PIN(SR0_D6),
	SSC8336N_COMMON_PIN(SR0_D7),
	SSC8336N_COMMON_PIN(SR0_D8),
	SSC8336N_COMMON_PIN(SR0_D9),
	SSC8336N_COMMON_PIN(SR0_D10),
	SSC8336N_COMMON_PIN(SR0_D11),
	SSC8336N_COMMON_PIN(SR0_GPIO0),
	SSC8336N_COMMON_PIN(SR0_GPIO1),
	SSC8336N_COMMON_PIN(SR0_GPIO2),
	SSC8336N_COMMON_PIN(SR0_GPIO3),
	SSC8336N_COMMON_PIN(SR0_GPIO4),
	SSC8336N_COMMON_PIN(SR0_GPIO5),
	SSC8336N_COMMON_PIN(SR0_GPIO6),
	SSC8336N_COMMON_PIN(SR1_GPIO0),
	SSC8336N_COMMON_PIN(SR1_GPIO1),
	SSC8336N_COMMON_PIN(SR1_GPIO2),
	SSC8336N_COMMON_PIN(SR1_GPIO3),
	SSC8336N_COMMON_PIN(SR1_GPIO4),
	SSC8336N_COMMON_PIN(SR1_D0P),
	SSC8336N_COMMON_PIN(SR1_D0N),
	SSC8336N_COMMON_PIN(SR1_CKP),
	SSC8336N_COMMON_PIN(SR1_CKN),
	SSC8336N_COMMON_PIN(SR1_D1P),
	SSC8336N_COMMON_PIN(SR1_D1N),
	SSC8336N_COMMON_PIN(LCD_HSYNC),
	SSC8336N_COMMON_PIN(LCD_VSYNC),
	SSC8336N_COMMON_PIN(LCD_PCLK),
	SSC8336N_COMMON_PIN(LCD_DE),
	SSC8336N_COMMON_PIN(LCD_0),
	SSC8336N_COMMON_PIN(LCD_1),
	SSC8336N_COMMON_PIN(LCD_2),
	SSC8336N_COMMON_PIN(LCD_3),
	SSC8336N_COMMON_PIN(LCD_4),
	SSC8336N_COMMON_PIN(LCD_5),
	SSC8336N_COMMON_PIN(LCD_6),
	SSC8336N_COMMON_PIN(LCD_7),
	SSC8336N_COMMON_PIN(LCD_8),
	SSC8336N_COMMON_PIN(LCD_9),
	SSC8336N_COMMON_PIN(LCD_10),
	SSC8336N_COMMON_PIN(LCD_11),
	SSC8336N_COMMON_PIN(LCD_12),
	SSC8336N_COMMON_PIN(LCD_13),
	SSC8336N_COMMON_PIN(LCD_14),
	SSC8336N_COMMON_PIN(LCD_15),
};

/* mux pin groupings */
/* i2c0 */
static const int ssc8336n_i2c0_mode1_pins[] = {
	PIN_SSC8336N_I2C0_SCL,
	PIN_SSC8336N_I2C0_SDA,
};
/* i2c1 */
static const int ssc8336n_i2c1_mode1_pins[] = {
	PIN_SSC8336N_SR0_GPIO0,
	PIN_SSC8336N_SR0_GPIO1,
};
static const int ssc8336n_usb_pins[] = {
	PIN_SSC8336N_USB_DM,
	PIN_SSC8336N_USB_DP,
};
static const int ssc8336n_usb1_pins[] = {
	PIN_SSC8336N_USB_DM1,
	PIN_SSC8336N_USB_DP1,
};

/* sdio */
static const int ssc8336n_sdio_mode1_pins[] = {
	PIN_SSC8336N_SD_CLK,
	PIN_SSC8336N_SD_CMD,
	PIN_SSC8336N_SD_D0,
	PIN_SSC8336N_SD_D1,
	PIN_SSC8336N_SD_D2,
	PIN_SSC8336N_SD_D3,
};
static const int ssc8336n_fuart_mode1_pins[] = {
	PIN_SSC8336N_FUART_RX,
	PIN_SSC8336N_FUART_TX,
	PIN_SSC8336N_FUART_CTS,
	PIN_SSC8336N_FUART_RTS,
};
static const int ssc8336n_lcd_d0_to_d9_pins[]  = {
	PIN_SSC8336N_LCD_0,
	PIN_SSC8336N_LCD_1,
	PIN_SSC8336N_LCD_2,
	PIN_SSC8336N_LCD_3,
	PIN_SSC8336N_LCD_4,
	PIN_SSC8336N_LCD_5,
	PIN_SSC8336N_LCD_6,
	PIN_SSC8336N_LCD_7,
	PIN_SSC8336N_LCD_8,
	PIN_SSC8336N_LCD_9,
};

static const int ssc8336n_sr0_d2_to_d11_pins[] = {
	PIN_SSC8336N_SR0_D2,
	PIN_SSC8336N_SR0_D3,
	PIN_SSC8336N_SR0_D4,
	PIN_SSC8336N_SR0_D5,
	PIN_SSC8336N_SR0_D6,
	PIN_SSC8336N_SR0_D7,
	PIN_SSC8336N_SR0_D8,
	PIN_SSC8336N_SR0_D9,
	PIN_SSC8336N_SR0_D10,
	PIN_SSC8336N_SR0_D11,
};

#define SR0_MIPI_COMMON	PIN_SSC8336N_SR0_GPIO2, \
			PIN_SSC8336N_SR0_GPIO3, \
			PIN_SSC8336N_SR0_GPIO4, \
			PIN_SSC8336N_SR0_D2, \
			PIN_SSC8336N_SR0_D3, \
			PIN_SSC8336N_SR0_D4, \
			PIN_SSC8336N_SR0_D5, \
			PIN_SSC8336N_SR0_D6, \
			PIN_SSC8336N_SR0_D7

static const int ssc8336n_sr0_mipi_mode1_pins[] = {
	SR0_MIPI_COMMON,
};

static const int ssc8336n_sr0_mipi_mode2_pins[] = {
	SR0_MIPI_COMMON,
	PIN_SSC8336N_SR0_D8,
	PIN_SSC8336N_SR0_D9,
	PIN_SSC8336N_SR0_D10,
	PIN_SSC8336N_SR0_D11,
};

static const int ssc8336n_sr1_bt656_pins[] = {
	PIN_SSC8336N_SR1_GPIO0,
	PIN_SSC8336N_SR1_GPIO1,
	PIN_SSC8336N_SR1_GPIO2,
	PIN_SSC8336N_SR1_GPIO3,
	PIN_SSC8336N_SR1_GPIO4,
};
/* incomplete */
static const int ssc8336n_sr1_mipi_mode4_pins[] = {
	PIN_SSC8336N_SR1_D0P,
	PIN_SSC8336N_SR1_D0N,
	PIN_SSC8336N_SR1_CKP,
	PIN_SSC8336N_SR1_CKN,
	PIN_SSC8336N_SR1_D1P,
	PIN_SSC8336N_SR1_D1N,
};

#define TX_MIPI_COMMON_PINS	PIN_SSC8336N_LCD_0, \
				PIN_SSC8336N_LCD_1, \
				PIN_SSC8336N_LCD_2, \
				PIN_SSC8336N_LCD_3, \
				PIN_SSC8336N_LCD_4, \
				PIN_SSC8336N_LCD_5

static const int ssc8336n_tx_mipi_mode1_pins[] = {
	TX_MIPI_COMMON_PINS
};

static const int ssc8336n_tx_mipi_mode2_pins[] = {
	TX_MIPI_COMMON_PINS,
	PIN_SSC8336N_LCD_6,
	PIN_SSC8336N_LCD_7,
	PIN_SSC8336N_LCD_8,
	PIN_SSC8336N_LCD_9,
};

#define SSC8336N_PINCTRL_GROUP(_NAME, _name) MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssc8336n_##_name##_pins)

/* pinctrl groups */
#define GROUPNAME_LCD_DO_TO_D9	"lcd_d0_to_d9"
#define GROUPNAME_SR0_D2_TO_D11	"sr0_d2_to_d11"

static const struct msc313_pinctrl_group ssc8336n_pinctrl_groups[] = {
	SSC8336N_PINCTRL_GROUP(I2C0_MODE1, i2c0_mode1),
	SSC8336N_PINCTRL_GROUP(I2C1_MODE1, i2c1_mode1),
	SSC8336N_PINCTRL_GROUP(USB, usb),
	SSC8336N_PINCTRL_GROUP(USB1, usb1),
	SSC8336N_PINCTRL_GROUP(SDIO_MODE1, sdio_mode1),
	SSC8336N_PINCTRL_GROUP(FUART_MODE1, fuart_mode1),
	SSC8336N_PINCTRL_GROUP(LCD_DO_TO_D9, lcd_d0_to_d9),
	SSC8336N_PINCTRL_GROUP(SR0_D2_TO_D11, sr0_d2_to_d11),
	SSC8336N_PINCTRL_GROUP(SR0_MIPI_MODE1, sr0_mipi_mode1),
	SSC8336N_PINCTRL_GROUP(SR0_MIPI_MODE2, sr0_mipi_mode2),
	SSC8336N_PINCTRL_GROUP(SR1_BT656, sr1_bt656),
	SSC8336N_PINCTRL_GROUP(SR1_MIPI_MODE4, sr1_mipi_mode4),
	SSC8336N_PINCTRL_GROUP(TX_MIPI_MODE1, tx_mipi_mode1),
	SSC8336N_PINCTRL_GROUP(TX_MIPI_MODE2, tx_mipi_mode2),
};

#define SSC8336N_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSC8336N_##_NAME, \
	MASK_SSC8336N_##_NAME, ssc8336n_##_name##_groups, ssc8336n_##_name##_values)

static const struct msc313_pinctrl_function ssc8336n_pinctrl_functions[] = {
	COMMON_FUNCTIONS,
	COMMON_FUNCTION(SR0_MIPI, sr0_mipi),
	COMMON_FUNCTION(SR1_BT656, sr1_bt656),
	COMMON_FUNCTION(SR1_MIPI, sr1_mipi),
	COMMON_FUNCTION(TX_MIPI, tx_mipi),
	COMMON_FIXED_FUNCTION(USB1, usb1)
};

static const struct msc313_pinctrl_pinconf ssc8336n_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssc8336n);
#endif /* mercury5 */

static const struct regmap_config msc313_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int mstar_set_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long config)
{
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int i;
	unsigned int mask;
	const struct msc313_pinctrl_pinconf *confpin;

	dev_dbg(pinctrl->dev, "setting %d:%u on pin %d\n", (int)config, (unsigned int)arg, pin);
	for (i = 0; i < pinctrl->info->npinconfs; i++) {
		if (pinctrl->info->pinconfs[i].pin == pin) {
			confpin = &pinctrl->info->pinconfs[i];
			switch (param) {
			case PIN_CONFIG_BIAS_PULL_UP:
				if (confpin->pull_en_reg != -1) {
					dev_dbg(pinctrl->dev, "setting pull up %d on pin %d\n", (int) arg, pin);
					mask = 1 << confpin->pull_en_bit;
					regmap_update_bits(pinctrl->regmap, confpin->pull_en_reg, mask, arg ? mask : 0);
				} else
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
	unsigned int val;

	if (confpin->pull_en_reg == ALWAYS_PULLUP)
		return !down;
	else if (confpin->pull_en_reg == ALWAYS_PULLDOWN)
		return down;
	else if (confpin->pull_en_reg != NOREG) {
		regmap_read(pinctrl->regmap, confpin->pull_en_reg, &val);
		if (val & BIT(confpin->pull_en_bit)) {
			if (confpin->pull_dir_reg == ALWAYS_PULLUP)
				return !down;
			else if (confpin->pull_dir_reg == ALWAYS_PULLDOWN)
				return down;
			else if (confpin->pull_en_reg != NOREG) {
				regmap_read(pinctrl->regmap, confpin->pull_dir_reg, &val);
				if (val & BIT(confpin->pull_dir_bit))
					return !down;
				else
					return down;
			} else
				return false;
		} else
			return false;
	} else
		return false;
}

static int msc313_pinctrl_get_config(struct msc313_pinctrl *pinctrl, int pin, unsigned long *config)
{
	int i;
	const struct msc313_pinctrl_pinconf *confpin;
	unsigned int val;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int crntidx;

	/* we only support a limited range of conf options so filter the
	 * ones we do here
	 */

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_DRIVE_STRENGTH:
		break;
	default:
		return -ENOTSUPP;
	}

	/* try to find the configuration register(s) for the pin */
	for (i = 0; i < pinctrl->info->npinconfs; i++) {
		if (pinctrl->info->pinconfs[i].pin == pin) {
			confpin = &pinctrl->info->pinconfs[i];
			switch (param) {
			case PIN_CONFIG_BIAS_PULL_UP:
				return msc313_pinctrl_ispulled(pinctrl, confpin, false) ? 0 : -EINVAL;
			case PIN_CONFIG_BIAS_PULL_DOWN:
				return msc313_pinctrl_ispulled(pinctrl, confpin, true) ? 0 : -EINVAL;
			case PIN_CONFIG_DRIVE_STRENGTH:
				if (confpin->drive_reg != -1) {
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

static int mstar_pin_config_get(struct pinctrl_dev *pctldev,
			       unsigned int pin,
			       unsigned long *config)
{
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return msc313_pinctrl_get_config(pinctrl, pin, config);
}

static int mstar_pin_config_set(struct pinctrl_dev *pctldev,
			       unsigned int pin,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	int i;
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	for (i = 0; i < num_configs; i++)
		mstar_set_config(pinctrl, pin, configs[i]);
	return 0;
}

static int mstar_pin_config_group_get(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *config)
{
	return -ENOTSUPP;
}

static int mstar_pin_config_group_set(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *configs,
				     unsigned int num_configs)
{
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct group_desc *group = pinctrl_generic_get_group(pctldev, selector);
	int i, j, ret;

	for (i = 0; i < group->num_pins; i++) {
		for (j = 0; j < num_configs; j++) {
			ret = mstar_set_config(pinctrl, group->pins[i], configs[j]);
			if (ret)
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
	void __iomem *base;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pinctrl->regmap = devm_regmap_init_mmio(pinctrl->dev, base,
			&msc313_pinctrl_regmap_config);
	if (IS_ERR(pinctrl->regmap))
		return PTR_ERR(pinctrl->regmap);

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

	ret = mstar_pinctrl_parse_functions(pinctrl);
	ret = mstar_pinctrl_parse_groups(pinctrl);

	ret = pinctrl_enable(pinctrl->pctl);
	if (ret)
		dev_err(pinctrl->dev, "failed to enable pinctrl\n");

	return 0;
}

static const struct of_device_id msc313_pinctrl_of_match[] = {
#ifdef CONFIG_MACH_INFINITY
	{
		.compatible	= "mstar,msc313-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "mstar,msc313e-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "sstar,ssd20xd-pinctrl",
		.data		= &ssd20xd_info,
	},
#endif
#ifdef CONFIG_MACH_MERCURY
	{
		.compatible	= "mstar,ssc8336-pinctrl",
		.data		= &ssc8336n_info,
	},
	{
		.compatible	= "mstar,ssc8336n-pinctrl",
		.data		= &ssc8336n_info,
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
