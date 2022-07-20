/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
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
 * arch/arm/src/stm32h745/stm32h745_flash.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/fs/mtd.h>

#include <tinyara/init.h>
#include <tinyara/kmalloc.h>

#include <stm32h7xx_hal.h>


/****************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#define PAGE_SHIFT (8)
#define FLASH_FS_START      CONFIG_AMEBAD_FLASH_BASE
#define AMEBAD_NSECTORS     (CONFIG_AMEBAD_FLASH_CAPACITY / CONFIG_AMEBAD_FLASH_BLOCK_SIZE)
#define AMEBAD_START_SECOTR (FLASH_FS_START / CONFIG_AMEBAD_FLASH_BLOCK_SIZE)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct amebad_dev_s.
 */
struct stm32h745_dev_s 
{
    struct mtd_dev_s mtd;       /* MTD interface */
    int nsectors;               /* number of erase sectors */
};


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* MTD driver methods */
static int stm32h745_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t stm32h745_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf);
static ssize_t stm32h745_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf);
static ssize_t stm32h745_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR uint8_t *buffer);
static int stm32h745_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

#if defined(CONFIG_MTD_BYTE_WRITE)
static ssize_t stm32h745_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR const uint8_t *buffer);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32h745_erase
 ************************************************************************************/
static int stm32h745_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{

}

/************************************************************************************
 * Name: stm32h745_bread
 ************************************************************************************/
static ssize_t stm32h745_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf)
{

}

/************************************************************************************
 * Name: stm32h745_bwrite
 ************************************************************************************/
static ssize_t stm32h745_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf)
{

}

/************************************************************************************
 * Name: stm32h745_read
 ************************************************************************************/
static ssize_t stm32h745_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR uint8_t *buffer)
{

}


/************************************************************************************
 * Name: stm32h745_ioctl
 ************************************************************************************/
static int stm32h745_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{

}

#if defined(CONFIG_MTD_BYTE_WRITE)
/************************************************************************************
 * Name: stm32h745_write
 ************************************************************************************/
static ssize_t stm32h745_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR const uint8_t *buffer)
{

}
#endif



/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_flashinitialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/
FAR struct mtd_dev_s *up_flashinitialize(void)
{
    return NULL;
}













