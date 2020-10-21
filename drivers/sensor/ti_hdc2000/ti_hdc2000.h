/*
 * Copyright (c) 2020 Statropy Software LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TI_HDC2000_TI_HDC2000_H_
#define ZEPHYR_DRIVERS_SENSOR_TI_HDC2000_TI_HDC2000_H_

#include <kernel.h>

#define TI_HDC2000_REG_TEMP			0x00
#define TI_HDC2000_REG_HUMIDITY		0x02
#define TI_HDC2000_REG_DRDY			0x04
#define TI_HDC2000_REG_INT_ENABLE	0x07
#define TI_HDC2000_REG_INT_CONFIG	0x0E
#define TI_HDC2000_REG_MEAS_CONFIG	0x0F
#define TI_HDC2000_REG_MANUFID		0xFC
#define TI_HDC2000_REG_DEVICEID		0xFE

#define TI_HDC2000_MANUFID			0x5449
#define TI_HDC2000_DEVID			0x07D0
#define TI_HDC2000_TRIGGER			0x01
#define TI_HDC2000_DATA_READY_MASK	0x80
#define TI_HDC2000_DATA_READY		0x80
#define TI_HDC2000_ENABLE_DRDY		0x80
#define TI_HDC2000_ENABLE_INT_DRDY	0x04
#define TI_HDC2000_INT_POLARITY		0x02


/* For 14bit conversion RH needs 6.5ms and Temp 6.35ms */
#define TI_HDC2000_CONVERSION_TIME     2

struct ti_hdc2000_data {
	const struct device *i2c;
	uint16_t t_sample;
	uint16_t rh_sample;

#if DT_INST_NODE_HAS_PROP(0, drdy_gpios)
	const struct device *gpio;
	struct gpio_callback gpio_cb;
	struct k_sem data_sem;
#endif  /* DT_INST_NODE_HAS_PROP(0, drdy_gpios) */
};

#endif
