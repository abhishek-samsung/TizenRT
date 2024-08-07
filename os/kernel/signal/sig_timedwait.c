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
 * kernel/signal/sig_timedwait.c
 *
 *   Copyright (C) 2007-2009, 2012-2014 Gregory Nutt. All rights reserved.
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
#include <tinyara/compiler.h>

#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/wdog.h>
#include <tinyara/cancelpt.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a special value of si_signo that means that it was the timeout
 * that awakened the wait... not the receipt of a signal.
 */

#define SIG_WAIT_TIMEOUT 0xff

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sig_timeout
 *
 * Description:
 *  A timeout elapsed while waiting for signals to be queued.
 *
 ****************************************************************************/

static void sig_timeout(int argc, uint32_t itcb)
{
	/* On many small machines, pointers are encoded and cannot be simply cast
	 * from uint32_t to struct tcb_s*.  The following union works around this
	 * (see wdogparm_t).  This odd logic could be conditioned on
	 * CONFIG_CAN_CAST_POINTERS, but it is not too bad in any case.
	 */

	union {
		FAR struct tcb_s *wtcb;
		uint32_t itcb;
	} u;

	u.itcb = itcb;
	ASSERT(u.wtcb);

#ifdef CONFIG_SMP
	irqstate_t flags;

	/* We must be in a critical section in order to call up_unblock_task()
	 * below.  If we are running on a single CPU architecture, then we know
	 * interrupts a disabled an there is no need to explicitly call
	 * enter_critical_section().  However, in the SMP case,
	 * enter_critical_section() does much more than just disable interrupts on
	 * the local CPU; it also manages spinlocks to assure the stability of the
	 * TCB that we are manipulating.
	 */

	flags = enter_critical_section();
#endif

	/* There may be a race condition -- make sure the task is
	 * still waiting for a signal
	 */

	if (u.wtcb->task_state == TSTATE_WAIT_SIG) {
		u.wtcb->sigunbinfo.si_signo = SIG_WAIT_TIMEOUT;
		u.wtcb->sigunbinfo.si_code = SI_TIMER;
		u.wtcb->sigunbinfo.si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
		u.wtcb->sigunbinfo.si_pid = 0;	/* Not applicable */
		u.wtcb->sigunbinfo.si_status = OK;
#endif
		up_unblock_task(u.wtcb);
	}

#ifdef CONFIG_SMP
	leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigtimedwait
 *
 * Description:
 *   This function selects the pending signal set specified by the argument
 *   set.  If multiple signals are pending in set, it will remove and return
 *   the lowest numbered one.  If no signals in set are pending at the time
 *   of the call, the calling process will be suspended until one of the
 *   signals in set becomes pending, OR until the process is interrupted by
 *   an unblocked signal, OR until the time interval specified by timeout
 *   (if any), has expired. If timeout is NULL, then the timeout interval
 *   is forever.
 *
 *   If the info argument is non-NULL, the selected signal number is stored
 *   in the si_signo member and the cause of the signal is store din the
 *   si_code member.  The content of si_value is only meaningful if the
 *   signal was generated by sigqueue().
 *
 *   The following values for si_code are defined in signal.h:
 *     SI_USER    - Signal sent from kill, raise, or abort
 *     SI_QUEUE   - Signal sent from sigqueue
 *     SI_TIMER   - Signal is result of timer expiration
 *     SI_ASYNCIO - Signal is the result of asynch IO completion
 *     SI_MESGQ   - Signal generated by arrival of a message on an
 *                  empty message queue.
 *
 * Parameters:
 *   set - The pending signal set.
 *   info - The returned value
 *   timeout - The amount of time to wait
 *
 * Return Value:
 *   Signal number that cause the wait to be terminated, otherwise -1 (ERROR)
 *   is returned with errno set to either:
 *
 *   EAGAIN - No signal specified by set was generated within the specified
 *            timeout period.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigtimedwait(FAR const sigset_t *set, FAR struct siginfo *info, FAR const struct timespec *timeout)
{
	FAR struct tcb_s *rtcb = this_task();
	sigset_t intersection;
	FAR sigpendq_t *sigpend;
	irqstate_t saved_state;
	int32_t waitticks;
	int ret = ERROR;

	DEBUGASSERT(rtcb->waitdog == NULL);

	/* sigtimedwait() is a cancellation point */
	(void)enter_cancellation_point();

	/* Several operations must be performed below:  We must determine if any
	 * signal is pending and, if not, wait for the signal.  Since signals can
	 * be posted from the interrupt level, there is a race condition that
	 * can only be eliminated by disabling interrupts!
	 */

	saved_state = enter_critical_section();

	/* Check if there is a pending signal corresponding to one of the
	 * signals in the pending signal set argument.
	 */

	intersection = *set & sig_pendingset(rtcb);
	if (intersection != NULL_SIGNAL_SET) {
		/* One or more of the signals in intersections is sufficient to cause
		 * us to not wait.  Pick the lowest numbered signal and mark it not
		 * pending.
		 */

		sigpend = sig_removependingsignal(rtcb, sig_lowest(&intersection));
		ASSERT(sigpend);

		/* Return the signal info to the caller if so requested */

		if (info) {
			memcpy(info, &sigpend->info, sizeof(struct siginfo));
		}

		/* The return value is the number of the signal that awakened us */

		ret = sigpend->info.si_signo;

		/* Then dispose of the pending signal structure properly */

		sig_releasependingsignal(sigpend);
		leave_critical_section(saved_state);
	}

	/* We will have to wait for a signal to be posted to this task. */

	else {
		/* Save the set of pending signals to wait for */

		rtcb->sigwaitmask = *set;

		/* Check if we should wait for the timeout */

		if (timeout) {
			/* Convert the timespec to system clock ticks, making sure that
			 * the resulting delay is greater than or equal to the requested
			 * time in nanoseconds.
			 */

#ifdef CONFIG_HAVE_LONG_LONG
			uint64_t waitticks64 = ((uint64_t)timeout->tv_sec * NSEC_PER_SEC + (uint64_t)timeout->tv_nsec + NSEC_PER_TICK - 1) / NSEC_PER_TICK;
			DEBUGASSERT(waitticks64 <= UINT32_MAX);
			waitticks = (uint32_t)waitticks64;
#else
			uint32_t waitmsec;

			DEBUGASSERT(timeout->tv_sec < UINT32_MAX / MSEC_PER_SEC);
			waitmsec = timeout->tv_sec * MSEC_PER_SEC + (timeout->tv_nsec + NSEC_PER_MSEC - 1) / NSEC_PER_MSEC;
			waitticks = MSEC2TICK(waitmsec);
#endif

			/* Create a watchdog */

			rtcb->waitdog = wd_create();
			DEBUGASSERT(rtcb->waitdog);

			if (rtcb->waitdog) {
				/* This little bit of nonsense is necessary for some
				 * processors where sizeof(pointer) < sizeof(uint32_t).
				 * see wdog.h.
				 */

				wdparm_t wdparm;
				wdparm.pvarg = (FAR void *)rtcb;

				/* Start the watchdog */

				wd_start(rtcb->waitdog, waitticks, (wdentry_t)sig_timeout, 1, wdparm.dwarg);

				/* Now wait for either the signal or the watchdog */

				up_block_task(rtcb, TSTATE_WAIT_SIG);

				/* We no longer need the watchdog */

				wd_delete(rtcb->waitdog);
				rtcb->waitdog = NULL;
			}

			/* REVISIT: And do what if there are no watchdog timers?  The wait
			 * will fail and we will return something bogus.
			 */
		}

		/* No timeout, just wait */

		else {
			/* And wait until one of the unblocked signals is posted */

			up_block_task(rtcb, TSTATE_WAIT_SIG);
		}

		/* We are running again, clear the sigwaitmask */

		rtcb->sigwaitmask = NULL_SIGNAL_SET;

		/* When we awaken, the cause will be in the TCB.  Get the signal number
		 * or timeout) that awakened us.
		 */

		if (GOOD_SIGNO(rtcb->sigunbinfo.si_signo)) {
			/* We were awakened by a signal... but is it one of the signals that
			 * we were waiting for?
			 */

			if (sigismember(set, rtcb->sigunbinfo.si_signo)) {
				/* Yes.. the return value is the number of the signal that
				 * awakened us.
				 */

				ret = rtcb->sigunbinfo.si_signo;
			} else {
				/* No... then set EINTR and report an error */

				set_errno(EINTR);
				ret = ERROR;
			}
		} else {
			/* Otherwise, we must have been awakened by the timeout.  Set EGAIN
			 * and return an error.
			 */

			DEBUGASSERT(rtcb->sigunbinfo.si_signo == SIG_WAIT_TIMEOUT);
			set_errno(EAGAIN);
			ret = ERROR;
		}

		/* Return the signal info to the caller if so requested */

		if (info) {
			memcpy(info, &rtcb->sigunbinfo, sizeof(struct siginfo));
		}

		leave_critical_section(saved_state);
	}

	leave_cancellation_point();
	return ret;
}
