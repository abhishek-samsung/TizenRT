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

#include <tinyara/i2c.h>
#include <tinyara/spi/spi.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/
#define HAL_Delay os_delay

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*mi48_handler_t)(void* arg);

struct mi48_lower_s {
	/* I2C related configs */
	struct i2c_config_s i2c_config;
	
	/* SPI related configs */
	struct spi_io_config spi_config;

	/* callback function: reset mi48 */
	int (*reset)(struct mi48_lower_s *lower);

	/* callback function: attach the mi48 interrupt handler to the GPIO interrupt */
	int (*attach)(struct mi48_lower_s *lower, mi48_handler_t handler, FAR char *arg);

	/* callback function: enable or disable gpio pin interrupt */
	int (*enable)(struct mi48_lower_s *lower, int enable);
};

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: mi48_register
 *
 * Description:
 *  This function will register mi48 thermal imaging sensor driver as /dev/imgN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   devname  - The full path to the driver to register. E.g., "/dev/img0"
 *   config      - configuration for the mi48 driver.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/
int mi48_register(FAR const char *devname, FAR struct mi48_lower_s *lower, FAR struct i2c_dev_s *i2c, FAR struct spi_dev_s *spi);
