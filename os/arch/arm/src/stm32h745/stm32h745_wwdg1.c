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
 * arch/arm/src/stm32h745/stm32h745_wwdg.c
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
#include <tinyara/mm/heap_regioninfo.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/init.h>
#include <tinyara/kmalloc.h>
#include <tinyara/watchdog.h>
#include <tinyara/irq.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "nvic.h"

#include <stm32h7xx_hal.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_wwdg.h>
#include <system_stm32h745.h>


#if defined(CONFIG_WATCHDOG) && defined(CONFIG_STM32_WWDG1)
#define CONFIG_STM32_WWDG_SETWINDOW (127)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct stm32h745_lowerhalf_s
{
    FAR const struct watchdog_ops_s *ops;   /* Lower half operations */
    xcpt_t handler;             /* Current EWI interrupt handler */
    uint32_t timeout;          /* The actual timeout value */
    uint32_t fwwdg;             /* WWDG clock frequency */
    bool started;               /* The timer has been started */
    uint8_t reload;             /* The 7-bit reload field reset value */
    uint8_t window;             /* The 7-bit window (W) field value */
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int stm32h745_wwdg1_start(FAR struct watchdog_lowerhalf_s *lower);
static int stm32h745_wwdg1_stop(FAR struct watchdog_lowerhalf_s *lower);
static int stm32h745_wwdg1_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int stm32h745_wwdg1_getstatus(FAR struct watchdog_lowerhalf_s *lower, FAR struct watchdog_status_s *status);
static int stm32h745_wwdg1_settimeout(FAR struct watchdog_lowerhalf_s *lower, uint32_t timeout);
static xcpt_t stm32h745_wwdg1_capture(FAR struct watchdog_lowerhalf_s *lower, xcpt_t handler);
static int stm32h745_wwdg1_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
const struct watchdog_ops_s g_wdgops = 
{
    .start = stm32h745_wwdg1_start,
    .stop = stm32h745_wwdg1_stop,
    .keepalive = stm32h745_wwdg1_keepalive,
    .getstatus = stm32h745_wwdg1_getstatus,
    .settimeout = stm32h745_wwdg1_settimeout,
    .capture = stm32h745_wwdg1_capture,
    .ioctl = stm32h745_wwdg1_ioctl,
};

/* "Lower half" driver state */

static struct stm32h745_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int  up_interrupt(int irq, void *context, FAR void *arg)
{
    FAR struct stm32h745_lowerhalf_s *priv = (FAR struct stm32h745_lowerhalf_s *)arg;

    stm32h745_wwdg1_keepalive((FAR struct watchdog_lowerhalf_s *)priv);

    lldbg("Go reset!!\n");
    
    up_disable_irq(STM32H745_IRQ_SYSTICK);
    up_disable_irq(STM32H745_IRQ_TIM17);
    up_disable_irq(STM32H745_IRQ_HSEM1);

    __WFI();
    return OK;
}

/****************************************************************************
 * Name: stm32h745_setwindow
 *
 * Description:
 *   Set the CFR window value. The window value is compared to the down-
 *   counter when the counter is updated.  The WWDG counter should be updated
 *   only when the counter is below this window value (and greater than 64)
 *   otherwise a reset will be generated
 *
 ****************************************************************************/
static void stm32h745_wwdg1_setwindow(FAR struct stm32h745_lowerhalf_s *priv, uint8_t window)
{
    lldbg("+++!!\n");

    priv->reload = window;
    priv->window = window;

    LL_WWDG_SetCounter(WWDG1, window);
    LL_WWDG_SetPrescaler(WWDG1, LL_WWDG_PRESCALER_128);
    LL_WWDG_SetWindow(WWDG1, window);
}


/****************************************************************************
 * Name: stm32h745_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_start(FAR struct watchdog_lowerhalf_s *lower)
{
    lldbg("+++!!\n");
    LL_WWDG_ClearFlag_EWKUP(WWDG1);
    LL_WWDG_Enable(WWDG1);
    LL_WWDG_EnableIT_EWKUP(WWDG1);

    up_enable_irq(STM32H745_IRQ_WWDG);
    stm32h745_wwdg1_keepalive(lower);
    return OK;
}


/****************************************************************************
 * Name: stm32h745_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_stop(FAR struct watchdog_lowerhalf_s *lower)
{
    lldbg("+++!!\n");
    return ERROR;
}


/****************************************************************************
 * Name: stm32h745_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the WWDG_CR register at regular
 *   intervals during normal operation to prevent an MCU reset. This operation
 *   must occur only when the counter value is lower than the window register
 *   value. The value to be stored in the WWDG_CR register must be between
 *   0xff and 0xC0:
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
    FAR struct stm32h745_lowerhalf_s *priv = (FAR struct stm32h745_lowerhalf_s *)lower;

    lldbg("+++!!\n");

    LL_WWDG_SetCounter(WWDG1, priv->reload);
    return OK;
}



/****************************************************************************
 * Name: stm32h745_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_getstatus(FAR struct watchdog_lowerhalf_s *lower, FAR struct watchdog_status_s *status)
{
    FAR struct stm32h745_lowerhalf_s *priv = (FAR struct stm32h745_lowerhalf_s *)lower;
    uint32_t elapsed;
    uint16_t reload;

    DEBUGASSERT(priv);

    lldbg("+++!!\n");

    /* Return the status bit */
    status->flags = WDFLAGS_RESET;
    if (priv->started) {
        status->flags |= WDFLAGS_ACTIVE;
    }

    if (priv->handler) {
        status->flags |= WDFLAGS_CAPTURE;
    }

    /* Return the actual timeout is milliseconds */

    status->timeout = priv->timeout;

    /* Get the time remaining until the watchdog expires (in milliseconds) */

    reload = LL_WWDG_GetCounter(WWDG1);
    elapsed = priv->reload - reload;
    status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

    return OK;
}


/****************************************************************************
 * Name: stm32h745_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_settimeout(FAR struct watchdog_lowerhalf_s *lower, uint32_t timeout)
{
    FAR struct stm32h745_lowerhalf_s *priv = (FAR struct stm32h745_lowerhalf_s *)lower;

    lldbg("+++!!\n");

    priv->timeout = timeout;

    return OK;
}



/****************************************************************************
 * Name: stm32h745_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t stm32h745_wwdg1_capture(FAR struct watchdog_lowerhalf_s *lower, xcpt_t handler)
{
    FAR struct stm32h745_lowerhalf_s *priv = (FAR struct stm32h745_lowerhalf_s *)lower;
    irqstate_t flags;
    xcpt_t oldhandler;
    uint16_t regval;

    DEBUGASSERT(priv);
    /* Get the old handler return value */

    lldbg("+++!!\n");

    flags = irqsave();
    oldhandler = priv->handler;

    /* Save the new handler */

    priv->handler = handler;

    /* Are we attaching or detaching the handler? */

    regval = READ_REG(WWDG1->CFR);
    if (handler) {
        /* Attaching... Enable the EWI interrupt */

        regval |= WWDG_CFR_EWI;

        WRITE_REG(WWDG1->CFR, regval);
        up_enable_irq(STM32H745_IRQ_WWDG);
    } else {
        /* Detaching... Disable the EWI interrupt */

        regval &= ~WWDG_CFR_EWI;

        WRITE_REG(WWDG1->CFR, regval);
        up_disable_irq(STM32H745_IRQ_WWDG);
    }

    irqrestore(flags);
    return oldhandler;
}

/****************************************************************************
 * Name: stm32h745_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32h745_wwdg1_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd, unsigned long arg)
{
    int ret=ERROR;

    lldbg("+++!!\n");

    return ret;
}


/****************************************************************************
 * Name: stm32h745_wwdginitialize
 *
 * Description:
 *   Initialize the WWDG watchdog time.  The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void stm32h745_wwdginitialize(FAR const char *devpath)
{
    FAR struct stm32h745_lowerhalf_s *priv = &g_wdgdev;

    /* NOTE we assume that clocking to the IWDG has already been provided by
     * the RCC initialization logic.
     */
    lldbg("+++!!\n");

    /* Initialize the driver state structure.  Here we assume: (1) the state
     * structure lies in .bss and was zeroed at reset time.  (2) This function
     * is only called once so it is never necessary to re-zero the structure.
     */

    priv->ops = &g_wdgops;

    /* Attach our EWI interrupt handler (But don't enable it yet) */

    (void)irq_attach(STM32H745_IRQ_WWDG, up_interrupt, priv);

    /* Select an arbitrary initial timeout value.  But don't start the watchdog
     * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
     * device option bits, the watchdog is automatically enabled at power-on.
     */

    LL_RCC_WWDG1_EnableSystemReset(); //configured by M4 side*/
    LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_WWDG1);

    stm32h745_wwdg1_setwindow(priv, CONFIG_STM32_WWDG_SETWINDOW);
    stm32h745_wwdg1_settimeout((FAR struct watchdog_lowerhalf_s *)priv, 100);

    /* Register the watchdog driver as /dev/watchdog0 */
    (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif












