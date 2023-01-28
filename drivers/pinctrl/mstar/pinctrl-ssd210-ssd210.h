// SPDX-License-Identifier: GPL-2.0
/* Chip data for SSD210 chip */

#ifndef DRIVERS_PINCTRL_MSTAR_PINCTRL_SSD210_SSD210_H_
#define DRIVERS_PINCTRL_MSTAR_PINCTRL_SSD210_SSD210_H_

/* ssd210 */
/* pinctrl pins */
static const struct pinctrl_pin_desc ssd210_pins[] = {
	SSD210_COMMON_PIN(SR_GPIO0),
	SSD210_COMMON_PIN(SR_GPIO1),
	SSD210_COMMON_PIN(SR_GPIO2),
	SSD210_COMMON_PIN(SR_GPIO3),
	SSD210_COMMON_PIN(SR_GPIO4),
	SSD210_COMMON_PIN(SR_GPIO5),
	SSD210_COMMON_PIN(SR_GPIO6),
	SSD210_COMMON_PIN(SR_GPIO7),
	SSD210_COMMON_PIN(SR_GPIO8),
	SSD210_COMMON_PIN(SR_GPIO9),
	SSD210_COMMON_PIN(SR_GPIO10),
	SSD210_COMMON_PIN(SR_GPIO11),
	SSD210_COMMON_PIN(TTL0),
	SSD210_COMMON_PIN(TTL1),
	SSD210_COMMON_PIN(TTL2),
	SSD210_COMMON_PIN(TTL3),
	SSD210_COMMON_PIN(TTL4),
	SSD210_COMMON_PIN(TTL5),
	SSD210_COMMON_PIN(TTL6),
	SSD210_COMMON_PIN(TTL7),
	SSD210_COMMON_PIN(TTL8),
	SSD210_COMMON_PIN(TTL11),
	SSD210_COMMON_PIN(TTL12),
	SSD210_COMMON_PIN(TTL13),
	SSD210_COMMON_PIN(TTL14),
	SSD210_COMMON_PIN(TTL15),
	SSD210_COMMON_PIN(TTL16),
	SSD210_COMMON_PIN(TTL17),
	SSD210_COMMON_PIN(TTL18),
	SSD210_COMMON_PIN(TTL19),
	SSD210_COMMON_PIN(TTL20),
	SSD210_COMMON_PIN(TTL21),
};

/* ej - jtag */
static const int ssd210_jtag_mode2_pins[] = {
	PIN_SSD210_SR_GPIO0,
	PIN_SSD210_SR_GPIO1,
	PIN_SSD210_SR_GPIO2,
	PIN_SSD210_SR_GPIO3,
};

static const char * const ssd210_jtag_groups[] = {
	GROUPNAME_JTAG_MODE2,
};

static const u16 ssd210_jtag_values[] = {
	SSD210_MODE(JTAG, 2),
};

/* pwm0 */
static const int ssd210_pwm0_mode1_pins[] = {
	PIN_SSD210_SR_GPIO0,
};

static const int ssd210_pwm0_mode2_pins[] = {
	PIN_SSD210_SR_GPIO4,
};

static const int ssd210_pwm0_mode4_pins[] = {
	PIN_SSD210_TTL4,
};

static const int ssd210_pwm0_mode5_pins[] = {
	PIN_SSD210_TTL14,
};

static const int ssd210_pwm0_mode6_pins[] = {
	PIN_SSD210_TTL18,
};

static const char * const ssd210_pwm0_groups[] = {
	GROUPNAME_PWM0_MODE1,
	GROUPNAME_PWM0_MODE2,
	GROUPNAME_PWM0_MODE4,
	GROUPNAME_PWM0_MODE5,
	GROUPNAME_PWM0_MODE6,
};

static const u16 ssd210_pwm0_values[] = {
	SSD210_MODE(PWM0, 1),
	SSD210_MODE(PWM0, 2),
	SSD210_MODE(PWM0, 4),
	SSD210_MODE(PWM0, 5),
	SSD210_MODE(PWM0, 6),
};

/* pwm1 */
static const int ssd210_pwm1_mode1_pins[] = {
	PIN_SSD210_SR_GPIO1,
};

static const int ssd210_pwm1_mode2_pins[] = {
	PIN_SSD210_SR_GPIO5,
};

static const int ssd210_pwm1_mode3_pins[] = {
	PIN_SSD210_SR_GPIO9,
};

static const int ssd210_pwm1_mode4_pins[] = {
	PIN_SSD210_TTL5,
};

static const int ssd210_pwm1_mode5_pins[] = {
	PIN_SSD210_TTL15,
};

static const int ssd210_pwm1_mode6_pins[] = {
	PIN_SSD210_TTL19,
};

static const char * const ssd210_pwm1_groups[] = {
	GROUPNAME_PWM1_MODE1,
	GROUPNAME_PWM1_MODE2,
	GROUPNAME_PWM1_MODE3,
	GROUPNAME_PWM1_MODE4,
	GROUPNAME_PWM1_MODE5,
	GROUPNAME_PWM1_MODE6,
};

static const u16 ssd210_pwm1_values[] = {
	SSD210_MODE(PWM1, 1),
	SSD210_MODE(PWM1, 2),
	SSD210_MODE(PWM1, 3),
	SSD210_MODE(PWM1, 4),
	SSD210_MODE(PWM1, 5),
	SSD210_MODE(PWM1, 6),
};

/* pwm2 */
static const int ssd210_pwm2_mode1_pins[] = {
	PIN_SSD210_SR_GPIO2,
};

static const int ssd210_pwm2_mode2_pins[] = {
	PIN_SSD210_SR_GPIO6,
};

static const int ssd210_pwm2_mode3_pins[] = {
	PIN_SSD210_SR_GPIO10,
};

static const int ssd210_pwm2_mode4_pins[] = {
	PIN_SSD210_TTL6,
};

static const int ssd210_pwm2_mode5_pins[] = {
	PIN_SSD210_TTL16,
};

static const int ssd210_pwm2_mode6_pins[] = {
	PIN_SSD210_TTL20,
};

static const char * const ssd210_pwm2_groups[] = {
	GROUPNAME_PWM2_MODE1,
	GROUPNAME_PWM2_MODE2,
	GROUPNAME_PWM2_MODE3,
	GROUPNAME_PWM2_MODE4,
	GROUPNAME_PWM2_MODE5,
	GROUPNAME_PWM2_MODE6,
};

static const u16 ssd210_pwm2_values[] = {
	SSD210_MODE(PWM2, 1),
	SSD210_MODE(PWM2, 2),
	SSD210_MODE(PWM2, 3),
	SSD210_MODE(PWM2, 4),
	SSD210_MODE(PWM2, 5),
	SSD210_MODE(PWM2, 6),
};

/* pwm3 */
static const int ssd210_pwm3_mode1_pins[] = {
	PIN_SSD210_SR_GPIO3,
};

static const int ssd210_pwm3_mode2_pins[] = {
	PIN_SSD210_SR_GPIO7,
};

static const int ssd210_pwm3_mode3_pins[] = {
	PIN_SSD210_SR_GPIO11,
};

static const int ssd210_pwm3_mode4_pins[] = {
	PIN_SSD210_TTL7,
};

static const int ssd210_pwm3_mode5_pins[] = {
	PIN_SSD210_TTL17,
};

static const int ssd210_pwm3_mode6_pins[] = {
	PIN_SSD210_TTL21,
};

static const char * const ssd210_pwm3_groups[] = {
	GROUPNAME_PWM3_MODE1,
	GROUPNAME_PWM3_MODE2,
	GROUPNAME_PWM3_MODE3,
	GROUPNAME_PWM3_MODE4,
	GROUPNAME_PWM3_MODE5,
	GROUPNAME_PWM3_MODE6,
};

static const u16 ssd210_pwm3_values[] = {
	SSD210_MODE(PWM3, 1),
	SSD210_MODE(PWM3, 2),
	SSD210_MODE(PWM3, 3),
	SSD210_MODE(PWM3, 4),
	SSD210_MODE(PWM3, 5),
	SSD210_MODE(PWM3, 6),
};

/* sdio */
static const int ssd210_sdio_mode2_pins[] = {
	PIN_SSD210_TTL1,
	PIN_SSD210_TTL2,
	PIN_SSD210_TTL3,
	PIN_SSD210_TTL4,
	PIN_SSD210_TTL5,
	PIN_SSD210_TTL6,
};

static const char * const ssd210_sdio_groups[] = {
	GROUPNAME_SDIO_MODE2,
};

static const u16 ssd210_sdio_values[] = {
	SSD210_MODE(SDIO, 2),
};

/* uart 1 */
static const int ssd210_uart1_mode5_pins[] = {
	PIN_SSD210_TTL8,
	PIN_SSD210_TTL11,
};

static const char * const ssd210_uart1_groups[] = {
	GROUPNAME_UART1_MODE5,
};

static const u16 ssd210_uart1_values[] = {
	SSD210_MODE(UART1, 5),
};

/* uart 2 */
static const int ssd210_uart2_mode3_pins[] = {
	PIN_SSD210_TTL12,
	PIN_SSD210_TTL13,
};

static const char * const ssd210_uart2_groups[] = {
	GROUPNAME_UART2_MODE3,
};

static const u16 ssd210_uart2_values[] = {
	SSD210_MODE(UART2, 3),
};

/* eth */
static const int ssd210_eth_mode7_pins[] = {
	PIN_SSD210_TTL12,
	PIN_SSD210_TTL13,
	PIN_SSD210_TTL14,
	PIN_SSD210_TTL15,
	PIN_SSD210_TTL16,
	PIN_SSD210_TTL17,
	PIN_SSD210_TTL18,
	PIN_SSD210_TTL19,
	PIN_SSD210_TTL20,
};

static const int ssd210_eth_mode8_pins[] = {
	PIN_SSD210_TTL12,
	PIN_SSD210_TTL13,
	PIN_SSD210_TTL14,
	PIN_SSD210_TTL15,
	PIN_SSD210_TTL16,
	PIN_SSD210_TTL17,
	PIN_SSD210_TTL18,
	PIN_SSD210_TTL19,
	PIN_SSD210_TTL20,
};

static const char * const ssd210_eth_groups[] = {
	GROUPNAME_ETH_MODE7,
	GROUPNAME_ETH_MODE8,
};

static const u16 ssd210_eth_values[] = {
	SSD210_MODE(ETH, 7),
	SSD210_MODE(ETH, 8),
};

/* i2c0 */
static const int ssd210_i2c0_mode1_pins[] = {
	PIN_SSD210_TTL0,
	PIN_SSD210_TTL1,
};

static const int ssd210_i2c0_mode3_pins[] = {
	PIN_SSD210_SR_GPIO0,
	PIN_SSD210_SR_GPIO1,
};

static const int ssd210_i2c0_mode4_pins[] = {
	PIN_SSD210_SR_GPIO4,
	PIN_SSD210_SR_GPIO5,
};

static const int ssd210_i2c0_mode5_pins[] = {
	PIN_SSD210_SR_GPIO6,
	PIN_SSD210_SR_GPIO7,
};

static const int ssd210_i2c0_mode8_pins[] = {
	PIN_SSD210_TTL8,
	PIN_SSD210_TTL11,
};

static const int ssd210_i2c0_mode9_pins[] = {
	PIN_SSD210_TTL18,
	PIN_SSD210_TTL19,
};

static const char * const ssd210_i2c0_groups[] = {
	GROUPNAME_I2C0_MODE1,
	GROUPNAME_I2C0_MODE3,
	GROUPNAME_I2C0_MODE4,
	GROUPNAME_I2C0_MODE5,
	GROUPNAME_I2C0_MODE8,
	GROUPNAME_I2C0_MODE9,
};

static const u16 ssd210_i2c0_values[] = {
	SSD210_MODE(I2C0, 1),
	SSD210_MODE(I2C0, 3),
	SSD210_MODE(I2C0, 4),
	SSD210_MODE(I2C0, 5),
	SSD210_MODE(I2C0, 8),
	SSD210_MODE(I2C0, 9),
};

/* i2c1 */
static const int ssd210_i2c1_mode2_pins[] = {
	PIN_SSD210_SR_GPIO0,
	PIN_SSD210_SR_GPIO1,
};

static const int ssd210_i2c1_mode3_pins[] = {
	PIN_SSD210_SR_GPIO4,
	PIN_SSD210_SR_GPIO5,
};

static const int ssd210_i2c1_mode4_pins[] = {
	PIN_SSD210_SR_GPIO6,
	PIN_SSD210_SR_GPIO7,
};

static const int ssd210_i2c1_mode7_pins[] = {
	PIN_SSD210_TTL20,
	PIN_SSD210_TTL21,
};

static const char * const ssd210_i2c1_groups[] = {
	GROUPNAME_I2C1_MODE2,
	GROUPNAME_I2C1_MODE3,
	GROUPNAME_I2C1_MODE4,
	GROUPNAME_I2C1_MODE7,
};

static const u16 ssd210_i2c1_values[] = {
	SSD210_MODE(I2C1, 2),
	SSD210_MODE(I2C1, 3),
	SSD210_MODE(I2C1, 4),
	SSD210_MODE(I2C1, 7),
};

/* spi0 */
static const int ssd210_spi0_mode2_pins[] = {
	PIN_SSD210_SR_GPIO0,
	PIN_SSD210_SR_GPIO1,
	PIN_SSD210_SR_GPIO2,
	PIN_SSD210_SR_GPIO3,
};

static const int ssd210_spi0_mode3_pins[] = {
	PIN_SSD210_TTL18,
	PIN_SSD210_TTL19,
	PIN_SSD210_TTL20,
	PIN_SSD210_TTL21,
};

static const int ssd210_spi0_mode5_pins[] = {
	PIN_SSD210_TTL4,
	PIN_SSD210_TTL5,
	PIN_SSD210_TTL6,
	PIN_SSD210_TTL7,
};

static const char * const ssd210_spi0_groups[] = {
	GROUPNAME_SPI0_MODE2,
	GROUPNAME_SPI0_MODE3,
	GROUPNAME_SPI0_MODE5,
};

static const u16 ssd210_spi0_values[] = {
	SSD210_MODE(SPI0, 2),
	SSD210_MODE(SPI0, 3),
	SSD210_MODE(SPI0, 5),
};

static const struct msc313_pinctrl_group ssd210_pinctrl_groups[] = {
	/* ej */
	SSD210_PINCTRL_GROUP(JTAG_MODE2, jtag_mode2),
	/* pwm0 */
	SSD210_PINCTRL_GROUP(PWM0_MODE1, pwm0_mode1),
	SSD210_PINCTRL_GROUP(PWM0_MODE2, pwm0_mode2),
	SSD210_PINCTRL_GROUP(PWM0_MODE4, pwm0_mode4),
	SSD210_PINCTRL_GROUP(PWM0_MODE5, pwm0_mode5),
	SSD210_PINCTRL_GROUP(PWM0_MODE6, pwm0_mode6),
	/* pwm1 */
	SSD210_PINCTRL_GROUP(PWM1_MODE1, pwm1_mode1),
	SSD210_PINCTRL_GROUP(PWM1_MODE2, pwm1_mode2),
	SSD210_PINCTRL_GROUP(PWM1_MODE3, pwm1_mode3),
	SSD210_PINCTRL_GROUP(PWM1_MODE4, pwm1_mode4),
	SSD210_PINCTRL_GROUP(PWM1_MODE5, pwm1_mode5),
	SSD210_PINCTRL_GROUP(PWM1_MODE6, pwm1_mode6),
	/* pwm2 */
	SSD210_PINCTRL_GROUP(PWM2_MODE1, pwm2_mode1),
	SSD210_PINCTRL_GROUP(PWM2_MODE2, pwm2_mode2),
	SSD210_PINCTRL_GROUP(PWM2_MODE3, pwm2_mode3),
	SSD210_PINCTRL_GROUP(PWM2_MODE4, pwm2_mode4),
	SSD210_PINCTRL_GROUP(PWM2_MODE5, pwm2_mode5),
	SSD210_PINCTRL_GROUP(PWM2_MODE6, pwm2_mode6),
	/* pwm3 */
	SSD210_PINCTRL_GROUP(PWM3_MODE1, pwm3_mode1),
	SSD210_PINCTRL_GROUP(PWM3_MODE2, pwm3_mode2),
	SSD210_PINCTRL_GROUP(PWM3_MODE3, pwm3_mode3),
	SSD210_PINCTRL_GROUP(PWM3_MODE4, pwm3_mode4),
	SSD210_PINCTRL_GROUP(PWM3_MODE5, pwm3_mode5),
	SSD210_PINCTRL_GROUP(PWM3_MODE6, pwm3_mode6),
	/* sdio */
	SSD210_PINCTRL_GROUP(SDIO_MODE2, sdio_mode2),
	/* spi0 */
	SSD210_PINCTRL_GROUP(SPI0_MODE2, spi0_mode2),
	SSD210_PINCTRL_GROUP(SPI0_MODE3, spi0_mode3),
	SSD210_PINCTRL_GROUP(SPI0_MODE5, spi0_mode5),
	/* uart1 */
	SSD210_PINCTRL_GROUP(UART1_MODE5, uart1_mode5),
	/* uart2 */
	SSD210_PINCTRL_GROUP(UART2_MODE3, uart2_mode3),
	/* eth */
	SSD210_PINCTRL_GROUP(ETH_MODE7, eth_mode7),
	SSD210_PINCTRL_GROUP(ETH_MODE8, eth_mode8),
	/* i2c0 */
	SSD210_PINCTRL_GROUP(I2C0_MODE1, i2c0_mode1),
	SSD210_PINCTRL_GROUP(I2C0_MODE3, i2c0_mode3),
	SSD210_PINCTRL_GROUP(I2C0_MODE4, i2c0_mode4),
	SSD210_PINCTRL_GROUP(I2C0_MODE5, i2c0_mode5),
	SSD210_PINCTRL_GROUP(I2C0_MODE8, i2c0_mode8),
	SSD210_PINCTRL_GROUP(I2C0_MODE9, i2c0_mode9),
	/* i2c1 */
	SSD210_PINCTRL_GROUP(I2C1_MODE2, i2c1_mode2),
	SSD210_PINCTRL_GROUP(I2C1_MODE3, i2c1_mode3),
	SSD210_PINCTRL_GROUP(I2C1_MODE4, i2c1_mode4),
	SSD210_PINCTRL_GROUP(I2C1_MODE7, i2c1_mode7),
};

static const struct msc313_pinctrl_function ssd210_pinctrl_functions[] = {
	SSD210_FUNCTION(JTAG, jtag),
	SSD210_FUNCTION(PWM0, pwm0),
	SSD210_FUNCTION(PWM1, pwm1),
	SSD210_FUNCTION(PWM2, pwm2),
	SSD210_FUNCTION(PWM3, pwm3),
	SSD210_FUNCTION(SDIO, sdio),
	SSD210_FUNCTION(SPI0, spi0),
	SSD210_FUNCTION(UART1, uart1),
	SSD210_FUNCTION(UART2, uart2),
	SSD210_FUNCTION(ETH, eth),
	SSD210_FUNCTION(I2C0, i2c0),
	SSD210_FUNCTION(I2C1, i2c1),
};

static const struct msc313_pinctrl_pinconf ssd210_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd210);

#endif /* DRIVERS_PINCTRL_MSTAR_PINCTRL_SSD210_SSD210_H_ */
