/*
 * Copyright (c) 2020 STMIcroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for B_L4S5I_IOT01A board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	{STM32_PIN_PB8, STM32L4X_PINMUX_FUNC_PB8_I2C1_SCL},
	{STM32_PIN_PB9, STM32L4X_PINMUX_FUNC_PB9_I2C1_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C
	/* I2C2 is used for NFC, STSAFE, ToF & MEMS sensors */
	{STM32_PIN_PB10, STM32L4X_PINMUX_FUNC_PB10_I2C2_SCL},
	{STM32_PIN_PB11, STM32L4X_PINMUX_FUNC_PB11_I2C2_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && CONFIG_SPI
	{STM32_PIN_PA5, STM32L4X_PINMUX_FUNC_PA5_SPI1_SCK},
	{STM32_PIN_PA6, STM32L4X_PINMUX_FUNC_PA6_SPI1_MISO},
	{STM32_PIN_PA7, STM32L4X_PINMUX_FUNC_PA7_SPI1_MOSI},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi3), okay) && CONFIG_SPI
	/* SPI3 is used for BT/WIFI, Sub GHZ communication */
	{STM32_PIN_PC10, STM32L4X_PINMUX_FUNC_PC10_SPI3_SCK},
	{STM32_PIN_PC11, STM32L4X_PINMUX_FUNC_PC11_SPI3_MISO | \
		STM32_OSPEEDR_VERY_HIGH_SPEED},
	{STM32_PIN_PC12, STM32L4X_PINMUX_FUNC_PC12_SPI3_MOSI},
#endif
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA10, STM32L4X_PINMUX_FUNC_PA10_OTG_FS_ID},
	{STM32_PIN_PA11, STM32L4X_PINMUX_FUNC_PA11_OTG_FS_DM},
	{STM32_PIN_PA12, STM32L4X_PINMUX_FUNC_PA12_OTG_FS_DP},
#endif /* CONFIG_USB_DC_STM32 */
};

static int pinmux_stm32_init(const struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
		CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
