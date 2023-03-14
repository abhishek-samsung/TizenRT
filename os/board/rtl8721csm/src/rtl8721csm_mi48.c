/****************************************************************************
 *
 * Copyright 2023 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/irq.h>
#include <tinyara/sensors/mi48.h>
#include <debug.h>
#include <errno.h>
#include "gpio_api.h"
#include "gpio_irq_api.h"

#ifdef CONFIG_MI48

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DEVNAME_FORMAT	"/dev/img%d"

#define GPIO_RESET		PB_2	// will set it to the right pin after wards
#define GPIO_DATA_READY		PA_19

#define GPIO_UNSET		0
#define GPIO_SET		1

/* i2c config */
#define MI48_I2C_PORT        	0
#define MI48_I2C_FREQ        	100000
#define MI48_I2C_ADDRLEN     	7

#define MI48_I2C_ADDR_H         0x41 /* need to set correct address */
#define MI48_I2C_ADDR_L         0x40

/* spi config */
#define MI48_SPI_PORT		0
#define MI48_SPI_FREQ		1000000
#define MI48_SPI_BPW		16
#define MI48_SPI_CS		0
#define MI48_SPI_MODE		SPIDEV_MODE1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtl8721csm_gpio_info_s {
	gpio_t reset;
	gpio_irq_t data_ready;
};

struct rtl8721csm_mi48_dev_s {
	/* Lower level mi48 dev */
	struct mi48_lower_s lower;
	
	/* board specific gpio information */
	struct rtl8721csm_gpio_info_s gpio_info;

	/* gpio interrupt handler */
	mi48_handler_t handler;
};

static int rtl8721csm_mi48_gpio_reset(struct mi48_lower_s *lower);
static int rtl8721csm_mi48_gpio_irq_attach(struct mi48_lower_s *lower, mi48_handler_t handler, FAR char *arg);
static int rtl8721csm_mi48_gpio_irq_enable(struct mi48_lower_s *lower, int enable);

static struct rtl8721csm_mi48_dev_s g_rtl8721csm_dev = {
	.lower = {
		.reset = rtl8721csm_mi48_gpio_reset,
		.attach = rtl8721csm_mi48_gpio_irq_attach,
		.enable = rtl8721csm_mi48_gpio_irq_enable,
		.i2c_config = {
			.frequency = MI48_I2C_FREQ,
			.address = MI48_I2C_ADDR_L,
			.addrlen = MI48_I2C_ADDRLEN,
		},
		.spi_config = {
			.bpw = MI48_SPI_BPW,
			.freq = MI48_SPI_FREQ,
			.cs = MI48_SPI_CS,
			.mode = MI48_SPI_MODE,
		},
	},
	.gpio_info = {
		.reset.pin = GPIO_RESET,
	},
	.handler = NULL

};

/****************************************************************************
 * Private Functions Prototype
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rtl8721csm_mi48_gpio_reset(struct mi48_lower_s *lower)
{
	gpio_write(&(g_rtl8721csm_dev.gpio_info.reset), GPIO_UNSET);
	HAL_Delay(100);
	gpio_write(&(g_rtl8721csm_dev.gpio_info.reset), GPIO_SET);
	HAL_Delay(1000);
	return OK;
}

static void rtl8721csm_gpio_irq_handler(uint32_t id, gpio_irq_event event)
{
	if (g_rtl8721csm_dev.handler) {
		g_rtl8721csm_dev.handler((void *)id);
	}
}

static int rtl8721csm_mi48_gpio_irq_attach(struct mi48_lower_s *lower, mi48_handler_t handler, FAR char *arg)
{
	g_rtl8721csm_dev.handler = handler;
	return OK;
}

static int rtl8721csm_mi48_gpio_irq_enable(struct mi48_lower_s *lower, int enable)
{
	if (enable) {
		gpio_irq_enable(&(g_rtl8721csm_dev.gpio_info.data_ready));
	} else {
		gpio_irq_disable(&(g_rtl8721csm_dev.gpio_info.data_ready));
	}
	return OK;
}

int rtl8721csm_mi48_initialize(void)
{
	int result = 0;
	FAR struct i2c_dev_s *i2c;
	FAR struct spi_dev_s *spi;
	char devpath[32];

	gpio_irq_init(&(g_rtl8721csm_dev.gpio_info.data_ready), GPIO_DATA_READY, rtl8721csm_gpio_irq_handler, 1);
	gpio_irq_set(&(g_rtl8721csm_dev.gpio_info.data_ready), IRQ_RISE, 1); // Rising edge trigger	

	i2c = up_i2cinitialize(MI48_I2C_PORT);
	if (!i2c) {
		result = -ENODEV;
		goto done;
	}

        spi = up_spiinitialize(1);
	if (!i2c) {
		result = -ENODEV;
		goto done;
	}

	/* register mi48 driver */
	snprintf(devpath, sizeof(devpath), DEVNAME_FORMAT, 0);
	if (mi48_register(devpath, &g_rtl8721csm_dev.lower, i2c, spi) != 0) {
		goto done;
	}

	/* result is success */
	result = 0;

done:
	return result;
}

#endif							/* CONFIG_MI48 */
