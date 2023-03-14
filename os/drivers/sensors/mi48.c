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
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <math.h>
#include <string.h>

#include <tinyara/sensors/mi48.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_SIGNAL_LOW				0
#define GPIO_SIGNAL_HIGH			1

#ifdef CONFIG_MI48_ENABLE_FRAME_HEADER
#define FRAME_HEADER_SIZE			80 /* 80 16bit words */
#else
#define FRAME_HEADER_SIZE			0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mi48_dev_s {
	FAR struct mi48_lower_s *lower;			/* mi48 driver config */
	FAR struct i2c_dev_s *i2c;			/* I2C driver to use */
	FAR struct spi_dev_s *spi;			/* SPI driver to use */
	int crefs;					/* reference count on the driver instance */
	sem_t exclsem;					/* exclusive access to the device */
	sem_t datasem;					/* exclusive access while reading sensor data */
	uint16_t frame_data[4960 + FRAME_HEADER_SIZE];	/* Latest frame data */
};

/****************************************************************************
 * Private Functions Prototype
 ****************************************************************************/

void mi48Reset(void);
void mi48EnbleTemporalFilter(void);
void mi48SetFrameRateDivisor(uint8_t framerateDivisor);
void mi48StartContinuousCapture(void);

static int mi48_open(FAR struct file *filep);
static int mi48_close(FAR struct file *filep);
static ssize_t mi48_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int mi48_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* mi48 driver instance */
static FAR struct mi48_dev_s g_mi48_priv;

/* This the vtable that supports the character driver interface */
static const struct file_operations g_mi48_fops = {
	mi48_open,				/* open */
	mi48_close,				/* close */
	mi48_read,				/* read */
	NULL,					/* write ?? probably no direct dependency*/ 
	NULL,					/* seek */
	mi48_ioctl,				/* ioctl */
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static uint8_t mi48_readreg_1byte(FAR struct mi48_dev_s *priv, uint8_t regaddr)
{
        uint8_t reg[4];
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *mi48_i2c_config = &(priv->lower->i2c_config);
        uint8_t reg_w[4];
        reg_w[0] = regaddr;
        int ret = i2c_write(dev, mi48_i2c_config, reg_w, 1);
	if (ret != 1) {
                lldbg("Error, cannot read reg %x\n", regaddr);
                PANIC();
                return ERROR;
        }
	ret =  i2c_read(dev, mi48_i2c_config, reg, 1);
	if (ret != 1) {
                lldbg("Error, cannot read reg %x\n", regaddr);
		PANIC();
                return ERROR;
        }
        return reg[0];
}

static int mi48_writereg_1byte(FAR struct mi48_dev_s *priv, uint8_t regaddr, uint8_t regval)
{
        int ret;
        uint8_t reg[2];
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *mi48_i2c_config = &(priv->lower->i2c_config);

        reg[0] = regaddr;
        reg[1] = regval;

        ret = i2c_write(dev, mi48_i2c_config, (uint8_t *)reg, 2);
        if (ret != 2) {
                lldbg("Error, cannot write reg %x\n", regaddr);
		PANIC();
		return ERROR;
        }
	return ret;
}

void mi48EnbleTemporalFilter()
{
	mi48_writereg_1byte(&g_mi48_priv, 0xd0, 0x0b);
	HAL_Delay(1000);
}

void mi48SetFrameRateDivisor(uint8_t framerateDivisor)
{
	mi48_writereg_1byte(&g_mi48_priv, 0xb4, framerateDivisor);
}

void mi48StartContinuousCapture()
{
	mi48_writereg_1byte(&g_mi48_priv, 0xb1, 0x03);
}

/****************************************************************************
 * Name: mi48_takesem
 *
 * Description:
 *    Take the lock, waiting as necessary
 *
 ****************************************************************************/
static inline int mi48_takesem(FAR sem_t *sem)
{
	/* Take a count from the semaphore, possibly waiting */
	if (sem_wait(sem) < 0) {
		/* EINTR is the only error that we expect */
		int errcode = get_errno();
		DEBUGASSERT(errcode == EINTR);
		return errcode;
	}

	return 0;
}

/****************************************************************************
 * Name: mi48_givesem
 *
 * Description:
 *    release the lock
 *
 ****************************************************************************/
static inline void mi48_givesem(sem_t *sem)
{
	sem_post(sem);
}

/****************************************************************************
 * Name: mi48_interrupt_handler
 *
 * Description:
 *    handle mi48's digital pin interrupt
 *
 ****************************************************************************/
static void mi48_interrupt_handler(void *arg)
{
	/* this is rising edge interrupt handler on mi48's data ready pin */
	FAR struct mi48_dev_s *priv = (FAR struct mi48_dev_s *)&g_mi48_priv;

	if (priv == NULL) {
		return;
	}

	int ret;

	ret = mi48_takesem(&priv->datasem);
	if (ret < 0) {
		lldbg("Error: mi48_takesem() failed: %d\n", ret);
		return;
	}

	/* this interrupt indicates that a complete frame is available to read
	 * So, we need to fetch the frame using SPI */

	SPI_RECVBLOCK(priv->spi, priv->frame_data, 4960 + FRAME_HEADER_SIZE);	
	
	mi48_givesem(&priv->datasem);
	return;
}

/****************************************************************************
 * Name: mi48_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/
static int mi48_open(FAR struct file *filep)
{
	int ret;

	mi48StartContinuousCapture();

	FAR struct inode *inode = filep->f_inode;
	FAR struct mi48_dev_s *priv = inode->i_private;

	ret = mi48_takesem(&priv->exclsem);
	if (ret < 0) {
		/*
		 * A signal received while waiting for the last close
		 * operation.
		 */
		lldbg("ERROR: mi48_takesem() failed: %d\n", ret);
		return ret;
	}

	/* mi48 driver allows only 1 instance */
	if (priv->crefs > 0) {
		lldbg("ERROR: mi48 driver is already opened.\n");
		goto errout_with_sem;
	}

	/* set reference count */
	priv->crefs = 1;

	/* enable the gpio pin interrupt */
	
errout_with_sem:
	mi48_givesem(&priv->exclsem);
	return ret;
}

/****************************************************************************
 * Name: mi48_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/
static int mi48_close(FAR struct file *filep)
{
	int ret;
	FAR struct inode *inode = filep->f_inode;
	FAR struct mi48_dev_s *priv = inode->i_private;

	/* Get exclusive access to the driver structure */
	ret = mi48_takesem(&priv->exclsem);
	if (ret < 0) {
		lldbg("ERROR: mi48_takesem() failed: %d\n", ret);
		return ret;
	}

	if (priv->crefs == 0) {
		lldbg("ERROR: mi48 driver is already closed.\n");
		goto errout_with_sem;
	}

	priv->crefs = 0;

	/* disable the gpio pin interrupt */
	ret = priv->lower->enable(priv->lower, 0);
	if (ret < 0) {
		lldbg("ERROR: failed to disable the interrupt handler. (ret=%d)\n", ret);
		goto errout_with_sem;
	}

	ret = 0;

errout_with_sem:
	mi48_givesem(&priv->exclsem);
	return ret;
}

/****************************************************************************
 * Name: mi48_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/
static ssize_t mi48_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
	int ret;
	FAR struct inode *inode = filep->f_inode;
	FAR struct mi48_dev_s *priv = inode->i_private;

	ret = mi48_takesem(&priv->datasem);
	if (ret < 0) {
		lldbg("ERROR: mi48_takesem() failed: %d\n", ret);
		return ret;
	}

	/* disable gpio pin interrupt */
	priv->lower->enable(priv->lower, 0);
	
	/* copy the frame contents to the buffer */
	//abhishek .. to do may be print the contents here...	
	
	/* enable gpio pin interrupt */
	priv->lower->enable(priv->lower, 1);

	mi48_givesem(&priv->datasem);

	return sizeof(float);
}

/****************************************************************************
 * Name: mi48_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/
static int mi48_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
        FAR struct inode *inode = filep->f_inode;
        FAR struct mi48_dev_s *priv = inode->i_private;

	switch (cmd) {
	case MI48IOC_HW_RESET: {
		priv->lower->reset(priv->lower);
	}
	break;
	case MI48IOC_ENABLE_TEMPORAL_FILTER: {
        	mi48EnbleTemporalFilter();
	}
        break;
	case MI48IOC_SET_FR_DIV: {
        	mi48SetFrameRateDivisor(2);
	}
        break;
	case MI48IOC_CONT_CAPTURE: {
        	mi48StartContinuousCapture();
	}
        break;
	}
	return OK;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: mi48_register
 *
 * Description:
 *  This function will register mi48 dust sensor driver as /dev/dustN where N
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
int mi48_register(FAR const char *devname, FAR struct mi48_lower_s *lower, FAR struct i2c_dev_s *i2c, FAR struct spi_dev_s *spi)
{
	int ret;

	struct mi48_dev_s *priv = &g_mi48_priv;

	/* validate mi48 config */
	if (i2c == NULL || spi == NULL || lower == NULL || lower->reset == NULL || lower->attach == NULL || lower->enable == NULL) {
		ret = EINVAL;
		lldbg("ERROR: invalid mi48 config\n");
		return ret;
	}
	
	priv->i2c = i2c;
	priv->spi = spi;
	/* set mi48 config */
	priv->lower = lower;

	/* reference count is set to 0 */
	priv->crefs = 0;

	/* set the SPI config */
	SPI_SETMODE(priv->spi, lower->spi_config.mode);
	SPI_SETFREQUENCY(priv->spi, lower->spi_config.freq);
	SPI_SETBITS(priv->spi, lower->spi_config.bpw);

	/* init semaphore */
	sem_init(&priv->exclsem, 0, 1);
	sem_init(&priv->datasem, 0, 1);

	/* attach the interrupt handler */
	ret = priv->lower->attach(priv->lower, mi48_interrupt_handler, (FAR void *)priv);
	if (ret < 0) {
		lldbg("ERROR: failed to attach the interrupt handler. (ret=%d)\n", ret);
		sem_destroy(&priv->exclsem);
		sem_destroy(&priv->datasem);
		return ret;
	}

	/* register the character device driver */
	ret = register_driver(devname, &g_mi48_fops, 0666, priv);
	if (ret < 0) {
		lldbg("ERROR: failed to register driver %s. (ret=%d)\n", devname, ret);
		sem_destroy(&priv->exclsem);
		sem_destroy(&priv->datasem);
		return ret;
	}

	lldbg("Registered %s\n", devname);

	return 0;
}
