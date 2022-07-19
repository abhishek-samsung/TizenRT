/****************************************************************************
 * arch/arm/src/stm32h745/stm32h745_sharedmem.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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
#include <tinyara/mm/heap_regioninfo.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "nvic.h"

#include <stm32h7xx_hal.h>
#include <system_stm32h745.h>



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if defined(CONFIG_SHARED_MEMORY)

#if defined(CONFIG_SHARED_MEMORY_AXI_RAM)
    #define STM32_SHARED_MEMORY_ADDRESS   (0x24000000)
    #define STM32_SHARED_MEMORY_LENGTH    (64*1024)

#elif defined(CONFIG_SHARED_MEMORY_SRAM1)
    #define STM32_SHARED_MEMORY_ADDRESS   (0x30000000)
    #define STM32_SHARED_MEMORY_LENGTH    (64*1024)

#elif defined(CONFIG_SHARED_MEMORY_SRAM2)
    #define STM32_SHARED_MEMORY_ADDRESS   (0x30020000)
    #define STM32_SHARED_MEMORY_LENGTH    (64*1024)

#elif defined(CONFIG_SHARED_MEMORY_SRAM3)
    #define STM32_SHARED_MEMORY_ADDRESS   (0x30040000)
    #define STM32_SHARED_MEMORY_LENGTH    (64*1024)

#elif defined(CONFIG_SHARED_MEMORY_SRAM4)
    #define STM32_SHARED_MEMORY_ADDRESS   (0x38000000)
    #define STM32_SHARED_MEMORY_LENGTH    (64*1024)

#endif

#define HSEM_ID_0 (0U)
#define HSEM_ID_1 (1U)
#define HSEM_ID_2 (2U)
#define HSEM_ID_3 (3U)
#define HSEM_ID_4 (4U)
#define HSEM_ID_5 (5U)
#define HSEM_ID_6 (6U)
#define HSEM_ID_7 (7U)
#define HSEM_ID_8 (8U)
#define HSEM_ID_9 (9U)
#define HSEM_ID_10 (10U)
#define HSEM_ID_11 (11U)
#define HSEM_ID_12 (12U)
#define HSEM_ID_13 (13U)
#define HSEM_ID_14 (14U)
#define HSEM_ID_15 (15U)
#define HSEM_ID_16 (16U)
#define HSEM_ID_17 (17U)
#define HSEM_ID_18 (18U)
#define HSEM_ID_19 (19U)
#define HSEM_ID_20 (20U)
#define HSEM_ID_21 (21U)
#define HSEM_ID_22 (22U)
#define HSEM_ID_23 (23U)
#define HSEM_ID_24 (24U)
#define HSEM_ID_25 (25U)
#define HSEM_ID_26 (26U)
#define HSEM_ID_27 (27U)
#define HSEM_ID_28 (28U)
#define HSEM_ID_29 (29U)
#define HSEM_ID_30 (30U)
#define HSEM_ID_31 (31U)
#define HSEM_ID_MAX HSEM_ID_31


static uint8_t *shared_mem;
static uint32_t hsem_statusreg = 0x00000000;

/****************************************************************************
 * Private Functions - stm32h745_up_interrupt
 ****************************************************************************/
static int  up_interrupt(int irq, void *context, FAR void *arg)
{
    /* Get the list of masked freed semaphores*/
    hsem_statusreg = HSEM_COMMON->MISR;

    /*Disable Interrupts*/
    //HSEM_COMMON->IER &= ~((uint32_t)hsem_statusreg);

    /*Clear Flags*/
    HSEM_COMMON->ICR = ((uint32_t)hsem_statusreg);

    lldbg("HSEM interrupt 0x%08x\n", hsem_statusreg);

    return OK;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h745_shared_memory_init
 *
 * Description:
 *   Initialize shared memory
 *
 ****************************************************************************/

int stm32h745_shared_memory_init(void)
{
    int ret;
    int i;

    shared_mem = (uint8_t *)(STM32_SHARED_MEMORY_ADDRESS);

    while(hsem_statusreg != 0){};

    for(i=0; i<STM32_SHARED_MEMORY_LENGTH; i++)
    {
        shared_mem[i] = 0x00;
    }

    /* Enable Interrupts */
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
    ret = irq_attach(STM32H745_IRQ_HSEM1, up_interrupt, NULL);
    if (ret == OK)
    {
        up_enable_irq(STM32H745_IRQ_HSEM1);
    }
    return ret;
}

/****************************************************************************
 * Name: stm32h745_shared_memory_write
 *
 * Description:
 *   Initialize shared memory
 *
 ****************************************************************************/
int stm32h745_shared_memory_write(uint32_t hsem_id, uint32_t index, uint8_t *src_addr, uint32_t size)
{
    if(hsem_id >= HSEM_ID_MAX)
    {
        return ERROR;
    }

    while(HAL_HSEM_FastTake(hsem_id) != HAL_OK){};

    //do write

    HAL_HSEM_Release(hsem_id, 0);

    return OK;
}

/****************************************************************************
 * Name: stm32h745_shared_memory_read
 *
 * Description:
 *   Initialize shared memory
 *
 ****************************************************************************/
int stm32h745_shared_memory_read(uint32_t hsem_id, uint32_t index, uint8_t *dest_addr, uint32_t size)
{
    if(hsem_id >= HSEM_ID_MAX)
    {
        return ERROR;
    }

    while(HAL_HSEM_FastTake(hsem_id) != HAL_OK){};

    HAL_HSEM_FastTake(hsem_id);

    //do read

    HAL_HSEM_Release(hsem_id, 0);

    return OK;
}

#endif




