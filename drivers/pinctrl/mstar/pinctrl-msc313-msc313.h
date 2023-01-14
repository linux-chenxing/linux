// SPDX-License-Identifier: GPL-2.0
/*
 * Chip data for MSC313 and MSC313e
 */

#ifndef DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_MSC313_H_
#define DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_MSC313_H_

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
#endif

#endif /* DRIVERS_PINCTRL_MSTAR_PINCTRL_MSC313_MSC313_H_ */
