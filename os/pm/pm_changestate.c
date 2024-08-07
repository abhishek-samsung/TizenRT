/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 * pm/pm_changestate.c
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
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
#include <queue.h>
#include <assert.h>
#include <time.h>
#include <debug.h>
#include <tinyara/pm/pm.h>
#include <tinyara/irq.h>

#include "pm.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_prepall
 *
 * Description:
 *   Prepare every driver for the state change.
 *
 * Input Parameters:
 *   domain - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static int pm_prepall(enum pm_state_e newstate)
{
	FAR dq_entry_t *entry;
	int ret = OK;

	if (newstate <= g_pmglobals.state) {
		/* Visit each registered callback structure in normal order. */

		for (entry = dq_peek(&g_pmglobals.registry); entry && ret == OK; entry = dq_next(entry)) {
			/* Is the prepare callback supported? */

			FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
			if (cb->prepare) {
				/* Yes.. prepare the driver */
				ret = cb->prepare(cb, newstate);
			}
		}
	} else {
		/* Visit each registered callback structure in reverse order. */

		for (entry = dq_tail(&g_pmglobals.registry); entry && ret == OK; entry = dq_prev(entry)) {
			/* Is the prepare callback supported? */

			FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
			if (cb->prepare) {
				/* Yes.. prepare the driver */
				ret = cb->prepare(cb, newstate);
			}
		}
	}

	return ret;
}

/****************************************************************************
 * Name: pm_changeall
 *
 * Description:
 *   domain - Identifies the domain of the new PM state
 *   Inform all drivers of the state change.
 *
 * Input Parameters:
 *   domain - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pm_changeall(enum pm_state_e newstate)
{
	FAR dq_entry_t *entry;
	if (newstate <= g_pmglobals.state) {
		/* Visit each registered callback structure in normal order. */

		for (entry = dq_peek(&g_pmglobals.registry); entry; entry = dq_next(entry)) {
			/* Is the notification callback supported? */

			FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
			if (cb->notify) {
				/* Yes.. notify the driver */
				cb->notify(cb, newstate);
			}
		}
	} else {
		/* Visit each registered callback structure in reverse order. */

		for (entry = dq_tail(&g_pmglobals.registry); entry; entry = dq_prev(entry)) {
			/* Is the notification callback supported? */

			FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
			if (cb->notify) {
				/* Yes.. notify the driver */
				cb->notify(cb, newstate);
			}
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_changestate
 *
 * Description:
 *   This function is used by platform-specific power management logic.  It
 *   will announce the power management power management state change to all
 *   drivers that have registered for power management event callbacks.
 *
 * Input Parameters:
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   It is assumed that interrupts are disabled when this function is
 *   called.  This function is probably called from the IDLE loop... the
 *   lowest priority task in the system.  Changing driver power management
 *   states may result in renewed system activity and, as a result, can
 *   suspend the IDLE thread before it completes the entire state change
 *   unless interrupts are disabled throughout the state change.
 *
 ****************************************************************************/

int pm_changestate(enum pm_state_e newstate)
{
	irqstate_t flags;
	int ret = OK;

	/* Disable interrupts throught this operation... changing driver states
	 * could cause additional driver activity that might interfere with the
	 * state change.  When the state change is complete, interrupts will be
	 * re-enabled.
	 */

	flags = enter_critical_section();

	/* First, prepare the drivers for the state change.  In this phase,
	 * drivers may refuse the state change.
	 */
	if ((newstate != PM_RESTORE) && (newstate != g_pmglobals.state)) {
		ret = pm_prepall(newstate);
		if (ret != OK) {
			goto EXIT;
		}
		/* All drivers have agreed to the state change (or, one or more have
		 * disagreed and the state has been reverted).  Set the new state.
		 */
		pm_changeall(newstate);
#ifdef CONFIG_PM_METRICS
		pm_metrics_update_changestate();
#endif
		g_pmglobals.state = newstate;
	}
EXIT:
	/* Restore the interrupt state */
	leave_critical_section(flags);
	return ret;
}

#endif /* CONFIG_PM */
