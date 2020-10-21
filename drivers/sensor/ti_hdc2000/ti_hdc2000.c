/*
 * Copyright (c) 2020 Statropy Software LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_hdc2000

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "ti_hdc2000.h"

LOG_MODULE_REGISTER(TI_HDC2000, CONFIG_SENSOR_LOG_LEVEL);

static int read16(const struct ti_hdc2000_data *drv_data, uint8_t addr,
		  uint16_t *data)
{
	uint8_t buf[2];
	int ret = i2c_burst_read(drv_data->i2c, DT_INST_REG_ADDR(0), addr,
				 (uint8_t *)buf, 2);

	if (ret == 0) {
		*data = ((buf[1] << 8) | buf[0]);
	}
	return ret;
}

static int write8(const struct ti_hdc2000_data *drv_data, uint8_t addr,
				  uint8_t data)
{
	uint8_t buf[2] = {addr, data};

	return i2c_write(drv_data->i2c, buf, 2, DT_INST_REG_ADDR(0));
}

static int read8(const struct ti_hdc2000_data *drv_data, uint8_t addr,
		 uint8_t *data)
{
	return i2c_burst_read(drv_data->i2c, DT_INST_REG_ADDR(0), addr,
			      data, 1);
}

#if DT_INST_NODE_HAS_PROP(0, drdy_gpios)
static void ti_hdc2000_gpio_callback(const struct device *dev,
				struct gpio_callback *cb, uint32_t pins)
{
	struct ti_hdc2000_data *drv_data =
		CONTAINER_OF(cb, struct ti_hdc2000_data, gpio_cb);

	ARG_UNUSED(pins);

	LOG_DBG("Int");

	gpio_pin_interrupt_configure(drv_data->gpio,
				     DT_INST_GPIO_PIN(0, drdy_gpios),
				     GPIO_INT_DISABLE);
	k_sem_give(&drv_data->data_sem);
}
#endif

static int ti_hdc2000_sample_fetch(const struct device *dev,
				   enum sensor_channel chan)
{
	struct ti_hdc2000_data *drv_data = dev->data;
	uint8_t data_ready;
	int max_tries = 10;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_HUMIDITY ||
			chan == SENSOR_CHAN_AMBIENT_TEMP);

#if DT_INST_NODE_HAS_PROP(0, drdy_gpios)
	gpio_pin_interrupt_configure(drv_data->gpio,
				     DT_INST_GPIO_PIN(0, drdy_gpios),
				     GPIO_INT_EDGE_TO_ACTIVE);
#endif

	if (write8(drv_data, TI_HDC2000_REG_MEAS_CONFIG,
		   TI_HDC2000_TRIGGER) < 0) {
		LOG_ERR("Failed to trigger measurement");
		return -EIO;
	}

	do {
#if DT_INST_NODE_HAS_PROP(0, drdy_gpios)
		int sema = k_sem_take(&drv_data->data_sem,
				      K_MSEC(TI_HDC2000_CONVERSION_TIME*10));
		if (sema < 0) {
			LOG_ERR("Interrupt timeout %d", sema);
		}
#else
		/* wait for the conversion to finish */
		k_msleep(TI_HDC2000_CONVERSION_TIME);
#endif
		if (read8(drv_data, TI_HDC2000_REG_DRDY, &data_ready) < 0) {
			LOG_ERR("Failed to fetch measurement");
			return -EIO;
		}
	} while (--max_tries &&
		 ((data_ready & TI_HDC2000_DATA_READY_MASK) !=
		 TI_HDC2000_DATA_READY));

	if ((data_ready & TI_HDC2000_DATA_READY_MASK) !=
	    TI_HDC2000_DATA_READY) {
		LOG_ERR("Failed to get measurement");
		return -EIO;
	}

	if (read16(drv_data, TI_HDC2000_REG_TEMP, &drv_data->t_sample) < 0) {
		LOG_ERR("Failed to read temperature");
		return -EIO;
	}
	if (read16(drv_data, TI_HDC2000_REG_TEMP, &drv_data->rh_sample) < 0) {
		LOG_ERR("Failed to read humidity");
		return -EIO;
	}

	return 0;
}


static int ti_hdc2000_channel_get(const struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct ti_hdc2000_data *drv_data = dev->data;
	uint64_t tmp;

	/*
	 * See datasheet "Temperature Register" and "Humidity
	 * Register" sections for more details on processing
	 * sample data.
	 */
	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		/* val = -40 + 165 * sample / 2^16 */
		tmp = (uint64_t)drv_data->t_sample * 165U;
		val->val1 = (int32_t)(tmp >> 16) - 40;
		val->val2 = ((tmp & 0xFFFF) * 1000000U) >> 16;
	} else if (chan == SENSOR_CHAN_HUMIDITY) {
		/* val = 100 * sample / 2^16 */
		tmp = (uint64_t)drv_data->rh_sample * 100U;
		val->val1 = tmp >> 16;
		/* x * 1000000 / 65536 == x * 15625 / 1024 */
		val->val2 = ((tmp & 0xFFFF) * 15625U) >> 10;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api ti_hdc2000_driver_api = {
	.sample_fetch = ti_hdc2000_sample_fetch,
	.channel_get = ti_hdc2000_channel_get,
};

static int ti_hdc2000_init(const struct device *dev)
{
	struct ti_hdc2000_data *drv_data = dev->data;
	uint16_t value;

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));

	if (drv_data->i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	if ((read16(drv_data, TI_HDC2000_REG_MANUFID, &value) < 0) ||
		    (value != TI_HDC2000_MANUFID)) {
		LOG_ERR("Failed to get correct manufacturer ID");
		return -EINVAL;
	}
	if ((read16(drv_data, TI_HDC2000_REG_DEVICEID, &value) < 0) ||
		    (value != TI_HDC2000_DEVID)) {
		LOG_ERR("Unsupported device ID");
		return -EINVAL;
	}

	if (write8(drv_data, TI_HDC2000_REG_INT_ENABLE,
		   TI_HDC2000_ENABLE_DRDY) < 0) {
		LOG_ERR("Could not enable Data Ready");
		return -EINVAL;
	}

#if DT_INST_NODE_HAS_PROP(0, drdy_gpios)
	LOG_INF("setting up interrupt");
	k_sem_init(&drv_data->data_sem, 0, UINT_MAX);

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(DT_INST_GPIO_LABEL(0, drdy_gpios));
	if (drv_data->gpio == NULL) {
		LOG_DBG("Failed to get pointer to %s device",
			 DT_INST_GPIO_LABEL(0, drdy_gpios));
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, DT_INST_GPIO_PIN(0, drdy_gpios),
			   GPIO_INPUT | DT_INST_GPIO_FLAGS(0, drdy_gpios));

	gpio_init_callback(&drv_data->gpio_cb,
			   ti_hdc2000_gpio_callback,
			   BIT(DT_INST_GPIO_PIN(0, drdy_gpios)));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_DBG("Failed to set GPIO callback");
		return -EIO;
	}

	gpio_pin_interrupt_configure(drv_data->gpio,
					DT_INST_GPIO_PIN(0, drdy_gpios),
					GPIO_INT_DISABLE);

	uint8_t int_mask = TI_HDC2000_ENABLE_INT_DRDY;

	if ((DT_INST_GPIO_FLAGS(0, drdy_gpios) & GPIO_ACTIVE_LOW) ==
		GPIO_ACTIVE_HIGH) {
		int_mask |= TI_HDC2000_INT_POLARITY;
	}

	if (write8(drv_data, TI_HDC2000_REG_INT_CONFIG, int_mask) < 0) {
		LOG_ERR("Could not enable Data Ready Interrupt");
		return -EINVAL;
	}

	/* TODO: Remove hack to change polarity */
	uint8_t buf[2] = {0x25, 0x02};

	if (i2c_write(drv_data->i2c, buf, 2, 0x18) < 0) {
		LOG_ERR("Could not change LIS2DE12 interrupt polarity");
	}
#else
	if (write8(drv_data, TI_HDC2000_REG_INT_CONFIG, 0) < 0) {
		LOG_ERR("Could not reset Data Ready Interrupt");
		return -EINVAL;
	}
#endif

	LOG_INF("Initialized device successfully");

	return 0;
}

static struct ti_hdc2000_data ti_hdc2000_data;

DEVICE_AND_API_INIT(ti_hdc2000, DT_INST_LABEL(0), ti_hdc2000_init,
		    &ti_hdc2000_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &ti_hdc2000_driver_api);
