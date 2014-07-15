/*
 * Real-Time Driver Model for Xenomai, driver library
 *
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 * Copyright (C) 2008 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/mman.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <linux/highmem.h>
#include <linux/err.h>

#include <rtdm/driver.h>
#include "internal.h"
#include <trace/events/cobalt-rtdm.h>

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_clock Clock Services
 * @{
 */

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */

/**
 * @brief Get system time
 *
 * @return The system time in nanoseconds is returned
 *
 * @note The resolution of this service depends on the system timer. In
 * particular, if the system timer is running in periodic mode, the return
 * value will be limited to multiples of the timer tick period.
 *
 * @note The system timer may have to be started to obtain valid results.
 * Whether this happens automatically (as on Xenomai) or is controlled by the
 * application depends on the RTDM host environment.
 *
 * @coretags{unrestricted}
 */
nanosecs_abs_t rtdm_clock_read(void);

/**
 * @brief Get monotonic time
 *
 * @return The monotonic time in nanoseconds is returned
 *
 * @note The resolution of this service depends on the system timer. In
 * particular, if the system timer is running in periodic mode, the return
 * value will be limited to multiples of the timer tick period.
 *
 * @note The system timer may have to be started to obtain valid results.
 * Whether this happens automatically (as on Xenomai) or is controlled by the
 * application depends on the RTDM host environment.
 *
 * @coretags{unrestricted}
 */
nanosecs_abs_t rtdm_clock_read_monotonic(void);
#endif /* DOXYGEN_CPP */
/** @} */

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_task Task Services
 * @{
 */

/**
 * @brief Initialise and start a real-time task
 *
 * After initialising a task, the task handle remains valid and can be
 * passed to RTDM services until either rtdm_task_destroy() or
 * rtdm_task_join() was invoked.
 *
 * @param[in,out] task Task handle
 * @param[in] name Optional task name
 * @param[in] task_proc Procedure to be executed by the task
 * @param[in] arg Custom argument passed to @c task_proc() on entry
 * @param[in] priority Priority of the task, see also
 * @ref rtdmtaskprio "Task Priority Range"
 * @param[in] period Period in nanoseconds of a cyclic task, 0 for non-cyclic
 * mode. Waiting for the first and subsequent periodic events is
 * done using rtdm_task_wait_period().
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{secondary-only, might-switch}
 */
int rtdm_task_init(rtdm_task_t *task, const char *name,
		   rtdm_task_proc_t task_proc, void *arg,
		   int priority, nanosecs_rel_t period)
{
	union xnsched_policy_param param;
	struct xnthread_start_attr sattr;
	struct xnthread_init_attr iattr;
	int err;

	iattr.name = name;
	iattr.flags = 0;
	iattr.personality = &rtdm_personality;
	iattr.affinity = CPU_MASK_ALL;
	param.rt.prio = priority;

	err = xnthread_init(task, &iattr, &xnsched_class_rt, &param);
	if (err)
		return err;

	/* We need an anonymous registry entry to obtain a handle for fast
	   mutex locking. */
	err = xnthread_register(task, "");
	if (err)
		goto cleanup_out;

	if (period > 0) {
		err = xnthread_set_periodic(task, XN_INFINITE,
					    XN_RELATIVE, period);
		if (err)
			goto cleanup_out;
	}

	sattr.mode = 0;
	sattr.entry = task_proc;
	sattr.cookie = arg;
	err = xnthread_start(task, &sattr);
	if (err)
		goto cleanup_out;

	return 0;

      cleanup_out:
	xnthread_cancel(task);
	return err;
}

EXPORT_SYMBOL_GPL(rtdm_task_init);

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */
/**
 * @brief Destroy a real-time task
 *
 * This call sends a termination request to @a task, then waits for it
 * to exit. All RTDM task should check for pending termination
 * requests by calling rtdm_task_should_stop() from their work loop.
 *
 * If @a task is current, rtdm_task_destroy() terminates the current
 * context, and does not return to the caller.
 *
 * @param[in,out] task Task handle as returned by rtdm_task_init()
 *
 * @note Passing the same task handle to RTDM services after the completion of
 * this function is not allowed.
 *
 * @coretags{secondary-only, might-switch}
 */
void rtdm_task_destroy(rtdm_task_t *task);

/**
 * @brief Check for pending termination request
 *
 * Check whether a termination request was received by the current
 * RTDM task. Termination requests are sent by calling
 * rtdm_task_destroy().
 *
 * @return Non-zero indicates that a termination request is pending,
 * in which case the caller should wrap up and exit.
 *
 * @coretags{rtdm-task, might-switch}
 */
int rtdm_task_should_stop(void);

/**
 * @brief Adjust real-time task priority
 *
 * @param[in,out] task Task handle as returned by rtdm_task_init()
 * @param[in] priority New priority of the task, see also
 * @ref rtdmtaskprio "Task Priority Range"
 *
 * @coretags{task-unrestricted, might-switch}
 */
void rtdm_task_set_priority(rtdm_task_t *task, int priority);

/**
 * @brief Adjust real-time task period
 *
 * @param[in,out] task Task handle as returned by rtdm_task_init()
 * @param[in] period New period in nanoseconds of a cyclic task, 0 for
 * non-cyclic mode
 *
 * @coretags{task-unrestricted}
 */
int rtdm_task_set_period(rtdm_task_t *task, nanosecs_rel_t period);

/**
 * @brief Wait on next real-time task period
 *
 * @return 0 on success, otherwise:
 *
 * - -EINVAL is returned if calling task is not in periodic mode.
 *
 * - -ETIMEDOUT is returned if a timer overrun occurred, which indicates
 * that a previous release point has been missed by the calling task.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_task_wait_period(void);

/**
 * @brief Activate a blocked real-time task
 *
 * @return Non-zero is returned if the task was actually unblocked from a
 * pending wait state, 0 otherwise.
 *
 * @coretags{unrestricted, might-switch}
 */
int rtdm_task_unblock(rtdm_task_t *task);

/**
 * @brief Get current real-time task
 *
 * @return Pointer to task handle
 *
 * @coretags{mode-unrestricted}
 */
rtdm_task_t *rtdm_task_current(void);

/**
 * @brief Sleep a specified amount of time
 *
 * @param[in] delay Delay in nanoseconds, see @ref RTDM_TIMEOUT_xxx for
 * special values.
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_task_sleep(nanosecs_rel_t delay);

/**
 * @brief Sleep until a specified absolute time
 *
 * @deprecated Use rtdm_task_sleep_abs instead!
 *
 * @param[in] wakeup_time Absolute timeout in nanoseconds
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_task_sleep_until(nanosecs_abs_t wakeup_time);

/**
 * @brief Sleep until a specified absolute time
 *
 * @param[in] wakeup_time Absolute timeout in nanoseconds
 * @param[in] mode Selects the timer mode, see RTDM_TIMERMODE_xxx for details
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * - -EINVAL is returned if an invalid parameter was passed.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_task_sleep_abs(nanosecs_abs_t wakeup_time, enum rtdm_timer_mode mode);

/**
 * @brief Busy wait a specified amount of time
 *
 * @param[in] condition Expresion - TRUE will complete the waiting
 * @param[in] busy_time The condition will be evaluated in a loop for this
 * number of nanoseconds
 * @param[in] sleep_time Before testing the condition again, the task will be
 * put to sleep for this number of nanoseconds
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_task_busy_wait(bool condition, nanosecs_rel_t busy_time,
	                nanosecs_rel_t sleep_time);

#endif /* DOXYGEN_CPP */

int __rtdm_task_sleep(xnticks_t timeout, xntmode_t mode)
{
	struct xnthread *thread;

	if (!XENO_ASSERT(RTDM, !xnsched_unblockable_p()))
		return -EPERM;

	thread = xnshadow_current();
	xnthread_suspend(thread, XNDELAY, timeout, mode, NULL);

	return xnthread_test_info(thread, XNBREAK) ? -EINTR : 0;
}

EXPORT_SYMBOL_GPL(__rtdm_task_sleep);

/**
 * @brief Wait on a real-time task to terminate
 *
 * @param[in,out] task Task handle as returned by rtdm_task_init()
 *
 * @note Passing the same task handle to RTDM services after the
 * completion of this function is not allowed.
 *
 * @note This service does not trigger the termination of the targeted
 * task.  The user has to take of this, otherwise rtdm_task_join()
 * will never return.
 *
 * @coretags{mode-unrestricted}
 */
void rtdm_task_join(rtdm_task_t *task)
{
	trace_cobalt_driver_task_join(task);

	xnthread_join(task, true);
}

EXPORT_SYMBOL_GPL(rtdm_task_join);

/**
 * @brief Busy-wait a specified amount of time
 *
 * This service does not schedule out the caller, but rather spins in
 * a tight loop, burning CPU cycles until the timeout elapses.
 *
 * @param[in] delay Delay in nanoseconds. Note that a zero delay does @b not
 * have the meaning of @c RTDM_TIMEOUT_INFINITE here.
 *
 * @note The caller must not be migratable to different CPUs while executing
 * this service. Otherwise, the actual delay will be undefined.
 *
 * @coretags{unrestricted}
 */
void rtdm_task_busy_sleep(nanosecs_rel_t delay)
{
	xnticks_t wakeup;

	wakeup = xnclock_read_raw(&nkclock) +
		xnclock_ns_to_ticks(&nkclock, delay);

	while ((xnsticks_t)(xnclock_read_raw(&nkclock) - wakeup) < 0)
		cpu_relax();
}

EXPORT_SYMBOL_GPL(rtdm_task_busy_sleep);
/** @} */

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_timer Timer Services
 * @{
 */

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */
/**
 * @brief Initialise a timer
 *
 * @param[in,out] timer Timer handle
 * @param[in] handler Handler to be called on timer expiry
 * @param[in] name Optional timer name
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 */
int rtdm_timer_init(rtdm_timer_t *timer, rtdm_timer_handler_t handler,
		    const char *name);
#endif /* DOXYGEN_CPP */

/**
 * @brief Destroy a timer
 *
 * @param[in,out] timer Timer handle as returned by rtdm_timer_init()
 *
 * @coretags{task-unrestricted}
 */
void rtdm_timer_destroy(rtdm_timer_t *timer)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	xntimer_destroy(timer);
	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_timer_destroy);

/**
 * @brief Start a timer
 *
 * @param[in,out] timer Timer handle as returned by rtdm_timer_init()
 * @param[in] expiry Firing time of the timer, @c mode defines if relative or
 * absolute
 * @param[in] interval Relative reload value, > 0 if the timer shall work in
 * periodic mode with the specific interval, 0 for one-shot timers
 * @param[in] mode Defines the operation mode, see @ref RTDM_TIMERMODE_xxx for
 * possible values
 *
 * @return 0 on success, otherwise:
 *
 * - -ETIMEDOUT is returned if @c expiry describes an absolute date in
 * the past. In such an event, the timer is nevertheless armed for the
 * next shot in the timeline if @a interval is non-zero.
 *
 * @coretags{unrestricted}
 */
int rtdm_timer_start(rtdm_timer_t *timer, nanosecs_abs_t expiry,
		     nanosecs_rel_t interval, enum rtdm_timer_mode mode)
{
	spl_t s;
	int err;

	xnlock_get_irqsave(&nklock, s);
	err = xntimer_start(timer, expiry, interval, (xntmode_t)mode);
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

EXPORT_SYMBOL_GPL(rtdm_timer_start);

/**
 * @brief Stop a timer
 *
 * @param[in,out] timer Timer handle as returned by rtdm_timer_init()
 *
 * @coretags{unrestricted}
 */
void rtdm_timer_stop(rtdm_timer_t *timer)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	xntimer_stop(timer);
	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_timer_stop);

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */
/**
 * @brief Start a timer from inside a timer handler
 *
 * @param[in,out] timer Timer handle as returned by rtdm_timer_init()
 * @param[in] expiry Firing time of the timer, @c mode defines if relative or
 * absolute
 * @param[in] interval Relative reload value, > 0 if the timer shall work in
 * periodic mode with the specific interval, 0 for one-shot timers
 * @param[in] mode Defines the operation mode, see @ref RTDM_TIMERMODE_xxx for
 * possible values
 *
 * @return 0 on success, otherwise:
 *
 * - -ETIMEDOUT is returned if @c expiry describes an absolute date in the
 * past.
 *
 * @coretags{coreirq-only}
 */
int rtdm_timer_start_in_handler(rtdm_timer_t *timer, nanosecs_abs_t expiry,
				nanosecs_rel_t interval,
				enum rtdm_timer_mode mode);

/**
 * @brief Stop a timer from inside a timer handler
 *
 * @param[in,out] timer Timer handle as returned by rtdm_timer_init()
 *
 * @coretags{coreirq-only}
 */
void rtdm_timer_stop_in_handler(rtdm_timer_t *timer);
#endif /* DOXYGEN_CPP */
/** @} */

/* --- IPC cleanup helper --- */

#define RTDM_SYNCH_DELETED          XNSYNCH_SPARE0

void __rtdm_synch_flush(struct xnsynch *synch, unsigned long reason)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (reason == XNRMID)
		xnsynch_set_status(synch, RTDM_SYNCH_DELETED);

	if (likely(xnsynch_flush(synch, reason) == XNSYNCH_RESCHED))
		xnsched_run();

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(__rtdm_synch_flush);

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_sync Synchronisation Services
 * @{
 */

/*!
 * @name Timeout Sequence Management
 * @{
 */

/**
 * @brief Initialise a timeout sequence
 *
 * This service initialises a timeout sequence handle according to the given
 * timeout value. Timeout sequences allow to maintain a continuous @a timeout
 * across multiple calls of blocking synchronisation services. A typical
 * application scenario is given below.
 *
 * @param[in,out] timeout_seq Timeout sequence handle
 * @param[in] timeout Relative timeout in nanoseconds, see
 * @ref RTDM_TIMEOUT_xxx for special values
 *
 * Application Scenario:
 * @code
int device_service_routine(...)
{
	rtdm_toseq_t timeout_seq;
	...

	rtdm_toseq_init(&timeout_seq, timeout);
	...
	while (received < requested) {
		ret = rtdm_event_timedwait(&data_available, timeout, &timeout_seq);
		if (ret < 0) // including -ETIMEDOUT
			break;

		// receive some data
		...
	}
	...
}
 * @endcode
 * Using a timeout sequence in such a scenario avoids that the user-provided
 * relative @c timeout is restarted on every call to rtdm_event_timedwait(),
 * potentially causing an overall delay that is larger than specified by
 * @c timeout. Moreover, all functions supporting timeout sequences also
 * interpret special timeout values (infinite and non-blocking),
 * disburdening the driver developer from handling them separately.
 *
 * @coretags{task-unrestricted}
 */
void rtdm_toseq_init(rtdm_toseq_t *timeout_seq, nanosecs_rel_t timeout)
{
	XENO_ASSERT(RTDM, !xnsched_unblockable_p()); /* only warn here */

	*timeout_seq = xnclock_read_monotonic(&nkclock) + timeout;
}

EXPORT_SYMBOL_GPL(rtdm_toseq_init);

/** @} */

/**
 * @ingroup rtdm_sync
 * @defgroup rtdm_sync_event Event Services
 * @{
 */

/**
 * @brief Initialise an event
 *
 * @param[in,out] event Event handle
 * @param[in] pending Non-zero if event shall be initialised as set, 0 otherwise
 *
 * @coretags{task-unrestricted}
 */
void rtdm_event_init(rtdm_event_t *event, unsigned long pending)
{
	spl_t s;

	trace_cobalt_driver_event_init(event, pending);

	/* Make atomic for re-initialisation support */
	xnlock_get_irqsave(&nklock, s);

	xnsynch_init(&event->synch_base, XNSYNCH_PRIO, NULL);
	if (pending)
		xnsynch_set_status(&event->synch_base, RTDM_EVENT_PENDING);
	xnselect_init(&event->select_block);

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_event_init);

/**
 * @brief Destroy an event
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 *
 * @coretags{task-unrestricted, might-switch}
 */
void rtdm_event_destroy(rtdm_event_t *event)
{
	trace_cobalt_driver_event_destroy(event);
	__rtdm_synch_flush(&event->synch_base, XNRMID);
	xnselect_destroy(&event->select_block);
}
EXPORT_SYMBOL_GPL(rtdm_event_destroy);

/**
 * @brief Signal an event occurrence to currently listening waiters
 *
 * This function wakes up all current waiters of the given event, but it does
 * not change the event state. Subsequently callers of rtdm_event_wait() or
 * rtdm_event_timedwait() will therefore be blocked first.
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 *
 * @coretags{unrestricted, might-switch}
 */
void rtdm_event_pulse(rtdm_event_t *event)
{
	trace_cobalt_driver_event_pulse(event);
	__rtdm_synch_flush(&event->synch_base, 0);
}
EXPORT_SYMBOL_GPL(rtdm_event_pulse);

/**
 * @brief Signal an event occurrence
 *
 * This function sets the given event and wakes up all current waiters. If no
 * waiter is presently registered, the next call to rtdm_event_wait() or
 * rtdm_event_timedwait() will return immediately.
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 *
 * @coretags{unrestricted, might-switch}
 */
void rtdm_event_signal(rtdm_event_t *event)
{
	int resched = 0;
	spl_t s;

	trace_cobalt_driver_event_signal(event);

	xnlock_get_irqsave(&nklock, s);

	xnsynch_set_status(&event->synch_base, RTDM_EVENT_PENDING);
	if (xnsynch_flush(&event->synch_base, 0))
		resched = 1;
	if (xnselect_signal(&event->select_block, 1))
		resched = 1;
	if (resched)
		xnsched_run();

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_event_signal);

/**
 * @brief Wait on event occurrence
 *
 * This is the light-weight version of rtdm_event_timedwait(), implying an
 * infinite timeout.
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EIDRM is returned if @a event has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_event_wait(rtdm_event_t *event)
{
	return rtdm_event_timedwait(event, 0, NULL);
}

EXPORT_SYMBOL_GPL(rtdm_event_wait);

/**
 * @brief Wait on event occurrence with timeout
 *
 * This function waits or tests for the occurence of the given event, taking
 * the provided timeout into account. On successful return, the event is
 * reset.
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 * @param[in] timeout Relative timeout in nanoseconds, see
 * @ref RTDM_TIMEOUT_xxx for special values
 * @param[in,out] timeout_seq Handle of a timeout sequence as returned by
 * rtdm_toseq_init() or NULL
 *
 * @return 0 on success, otherwise:
 *
 * - -ETIMEDOUT is returned if the if the request has not been satisfied
 * within the specified amount of time.
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EIDRM is returned if @a event has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * - -EWOULDBLOCK is returned if a negative @a timeout (i.e., non-blocking
 * operation) has been specified.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_event_timedwait(rtdm_event_t *event, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq)
{
	struct xnthread *thread;
	spl_t s;
	int err = 0;

	if (!XENO_ASSERT(RTDM, !xnsched_unblockable_p()))
		return -EPERM;

	trace_cobalt_driver_event_wait(event, xnshadow_current());

	xnlock_get_irqsave(&nklock, s);

	if (unlikely(event->synch_base.status & RTDM_SYNCH_DELETED))
		err = -EIDRM;
	else if (likely(event->synch_base.status & RTDM_EVENT_PENDING)) {
		xnsynch_clear_status(&event->synch_base, RTDM_EVENT_PENDING);
		xnselect_signal(&event->select_block, 0);
	} else {
		/* non-blocking mode */
		if (timeout < 0) {
			err = -EWOULDBLOCK;
			goto unlock_out;
		}

		thread = xnshadow_current();

		if (timeout_seq && (timeout > 0)) {
			/* timeout sequence */
			xnsynch_sleep_on(&event->synch_base, *timeout_seq,
					 XN_ABSOLUTE);
		} else {
			/* infinite or relative timeout */
			xnsynch_sleep_on(&event->synch_base, timeout, XN_RELATIVE);
		}

		if (likely
		    (!xnthread_test_info(thread, XNTIMEO | XNRMID | XNBREAK))) {
			xnsynch_clear_status(&event->synch_base,
					    RTDM_EVENT_PENDING);
			xnselect_signal(&event->select_block, 0);
		} else if (xnthread_test_info(thread, XNTIMEO))
			err = -ETIMEDOUT;
		else if (xnthread_test_info(thread, XNRMID))
			err = -EIDRM;
		else /* XNBREAK */
			err = -EINTR;
	}

unlock_out:
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

EXPORT_SYMBOL_GPL(rtdm_event_timedwait);

/**
 * @brief Clear event state
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 *
 * @coretags{unrestricted}
 */
void rtdm_event_clear(rtdm_event_t *event)
{
	spl_t s;

	trace_cobalt_driver_event_clear(event);

	xnlock_get_irqsave(&nklock, s);

	xnsynch_clear_status(&event->synch_base, RTDM_EVENT_PENDING);
	xnselect_signal(&event->select_block, 0);

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_event_clear);

/**
 * @brief Bind a selector to an event
 *
 * This functions binds the given selector to an event so that the former is
 * notified when the event state changes. Typically the select binding handler
 * will invoke this service.
 *
 * @param[in,out] event Event handle as returned by rtdm_event_init()
 * @param[in,out] selector Selector as passed to the select binding handler
 * @param[in] type Type of the bound event as passed to the select binding handler
 * @param[in] fd_index File descriptor index as passed to the select binding
 * handler
 *
 * @return 0 on success, otherwise:
 *
 * - -ENOMEM is returned if there is insufficient memory to establish the
 * dynamic binding.
 *
 * - -EINVAL is returned if @a type or @a fd_index are invalid.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_event_select_bind(rtdm_event_t *event, rtdm_selector_t *selector,
			   enum rtdm_selecttype type, unsigned int fd_index)
{
	struct xnselect_binding *binding;
	int err;
	spl_t s;

	binding = xnmalloc(sizeof(*binding));
	if (!binding)
		return -ENOMEM;

	xnlock_get_irqsave(&nklock, s);
	err = xnselect_bind(&event->select_block,
			    binding, selector, type, fd_index,
			    event->synch_base.status & (RTDM_SYNCH_DELETED |
						       RTDM_EVENT_PENDING));
	xnlock_put_irqrestore(&nklock, s);

	if (err)
		xnfree(binding);

	return err;
}
EXPORT_SYMBOL_GPL(rtdm_event_select_bind);

/** @} */

/**
 * @ingroup rtdm_sync
 * @defgroup rtdm_sync_sem Semaphore Services
 * @{
 */

/**
 * @brief Initialise a semaphore
 *
 * @param[in,out] sem Semaphore handle
 * @param[in] value Initial value of the semaphore
 *
 * @coretags{task-unrestricted}
 */
void rtdm_sem_init(rtdm_sem_t *sem, unsigned long value)
{
	spl_t s;

	trace_cobalt_driver_sem_init(sem, value);

	/* Make atomic for re-initialisation support */
	xnlock_get_irqsave(&nklock, s);

	sem->value = value;
	xnsynch_init(&sem->synch_base, XNSYNCH_PRIO, NULL);
	xnselect_init(&sem->select_block);

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_sem_init);

/**
 * @brief Destroy a semaphore
 *
 * @param[in,out] sem Semaphore handle as returned by rtdm_sem_init()
 *
 * @coretags{task-unrestricted, might-switch}
 */
void rtdm_sem_destroy(rtdm_sem_t *sem)
{
	trace_cobalt_driver_sem_destroy(sem);
	__rtdm_synch_flush(&sem->synch_base, XNRMID);
	xnselect_destroy(&sem->select_block);
}
EXPORT_SYMBOL_GPL(rtdm_sem_destroy);

/**
 * @brief Decrement a semaphore
 *
 * This is the light-weight version of rtdm_sem_timeddown(), implying an
 * infinite timeout.
 *
 * @param[in,out] sem Semaphore handle as returned by rtdm_sem_init()
 *
 * @return 0 on success, otherwise:
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EIDRM is returned if @a sem has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_sem_down(rtdm_sem_t *sem)
{
	return rtdm_sem_timeddown(sem, 0, NULL);
}

EXPORT_SYMBOL_GPL(rtdm_sem_down);

/**
 * @brief Decrement a semaphore with timeout
 *
 * This function tries to decrement the given semphore's value if it is
 * positive on entry. If not, the caller is blocked unless non-blocking
 * operation was selected.
 *
 * @param[in,out] sem Semaphore handle as returned by rtdm_sem_init()
 * @param[in] timeout Relative timeout in nanoseconds, see
 * @ref RTDM_TIMEOUT_xxx for special values
 * @param[in,out] timeout_seq Handle of a timeout sequence as returned by
 * rtdm_toseq_init() or NULL
 *
 * @return 0 on success, otherwise:
 *
 * - -ETIMEDOUT is returned if the if the request has not been satisfied
 * within the specified amount of time.
 *
 * - -EWOULDBLOCK is returned if @a timeout is negative and the semaphore
 * value is currently not positive.
 *
 * - -EINTR is returned if calling task has been unblock by a signal or
 * explicitly via rtdm_task_unblock().
 *
 * - -EIDRM is returned if @a sem has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_sem_timeddown(rtdm_sem_t *sem, nanosecs_rel_t timeout,
		       rtdm_toseq_t *timeout_seq)
{
	struct xnthread *thread;
	spl_t s;
	int err = 0;

	if (!XENO_ASSERT(RTDM, !xnsched_unblockable_p()))
		return -EPERM;

	trace_cobalt_driver_sem_wait(sem, xnshadow_current());

	xnlock_get_irqsave(&nklock, s);

	if (unlikely(sem->synch_base.status & RTDM_SYNCH_DELETED))
		err = -EIDRM;
	else if (sem->value > 0) {
		if(!--sem->value)
			xnselect_signal(&sem->select_block, 0);
	} else if (timeout < 0) /* non-blocking mode */
		err = -EWOULDBLOCK;
	else {
		thread = xnshadow_current();

		if (timeout_seq && (timeout > 0)) {
			/* timeout sequence */
			xnsynch_sleep_on(&sem->synch_base, *timeout_seq,
					 XN_ABSOLUTE);
		} else
			/* infinite or relative timeout */
			xnsynch_sleep_on(&sem->synch_base, timeout, XN_RELATIVE);

		if (xnthread_test_info(thread, XNTIMEO | XNRMID | XNBREAK)) {
			if (xnthread_test_info(thread, XNTIMEO))
				err = -ETIMEDOUT;
			else if (xnthread_test_info(thread, XNRMID))
				err = -EIDRM;
			else /* XNBREAK */
				err = -EINTR;
		}
	}

	xnlock_put_irqrestore(&nklock, s);

	return err;
}

EXPORT_SYMBOL_GPL(rtdm_sem_timeddown);

/**
 * @brief Increment a semaphore
 *
 * This function increments the given semphore's value, waking up a potential
 * waiter which was blocked upon rtdm_sem_down().
 *
 * @param[in,out] sem Semaphore handle as returned by rtdm_sem_init()
 *
 * @coretags{unrestricted, might-switch}
 */
void rtdm_sem_up(rtdm_sem_t *sem)
{
	spl_t s;

	trace_cobalt_driver_sem_up(sem);

	xnlock_get_irqsave(&nklock, s);

	if (xnsynch_wakeup_one_sleeper(&sem->synch_base))
		xnsched_run();
	else
		if (sem->value++ == 0
		    && xnselect_signal(&sem->select_block, 1))
			xnsched_run();

	xnlock_put_irqrestore(&nklock, s);
}

EXPORT_SYMBOL_GPL(rtdm_sem_up);

/**
 * @brief Bind a selector to a semaphore
 *
 * This functions binds the given selector to the semaphore so that the former
 * is notified when the semaphore state changes. Typically the select binding
 * handler will invoke this service.
 *
 * @param[in,out] sem Semaphore handle as returned by rtdm_sem_init()
 * @param[in,out] selector Selector as passed to the select binding handler
 * @param[in] type Type of the bound event as passed to the select binding handler
 * @param[in] fd_index File descriptor index as passed to the select binding
 * handler
 *
 * @return 0 on success, otherwise:
 *
 * - -ENOMEM is returned if there is insufficient memory to establish the
 * dynamic binding.
 *
 * - -EINVAL is returned if @a type or @a fd_index are invalid.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_sem_select_bind(rtdm_sem_t *sem, rtdm_selector_t *selector,
			 enum rtdm_selecttype type, unsigned int fd_index)
{
	struct xnselect_binding *binding;
	int err;
	spl_t s;

	binding = xnmalloc(sizeof(*binding));
	if (!binding)
		return -ENOMEM;

	xnlock_get_irqsave(&nklock, s);
	err = xnselect_bind(&sem->select_block, binding, selector,
			    type, fd_index,
			    (sem->value > 0) ||
			    sem->synch_base.status & RTDM_SYNCH_DELETED);
	xnlock_put_irqrestore(&nklock, s);

	if (err)
		xnfree(binding);

	return err;
}
EXPORT_SYMBOL_GPL(rtdm_sem_select_bind);

/** @} */

/**
 * @ingroup rtdm_sync
 * @defgroup rtdm_sync_mutex Mutex services
 * @{
 */

/**
 * @brief Initialise a mutex
 *
 * This function initalises a basic mutex with priority inversion protection.
 * "Basic", as it does not allow a mutex owner to recursively lock the same
 * mutex again.
 *
 * @param[in,out] mutex Mutex handle
 *
 * @coretags{task-unrestricted}
 */
void rtdm_mutex_init(rtdm_mutex_t *mutex)
{
	spl_t s;

	/* Make atomic for re-initialisation support */
	xnlock_get_irqsave(&nklock, s);
	xnsynch_init(&mutex->synch_base, XNSYNCH_PIP, &mutex->fastlock);
	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(rtdm_mutex_init);

/**
 * @brief Destroy a mutex
 *
 * @param[in,out] mutex Mutex handle as returned by rtdm_mutex_init()
 *
 * @coretags{task-unrestricted, might-switch}
 */
void rtdm_mutex_destroy(rtdm_mutex_t *mutex)
{
	trace_cobalt_driver_mutex_destroy(mutex);

	__rtdm_synch_flush(&mutex->synch_base, XNRMID);
}
EXPORT_SYMBOL_GPL(rtdm_mutex_destroy);

/**
 * @brief Release a mutex
 *
 * This function releases the given mutex, waking up a potential waiter which
 * was blocked upon rtdm_mutex_lock() or rtdm_mutex_timedlock().
 *
 * @param[in,out] mutex Mutex handle as returned by rtdm_mutex_init()
 *
 * @coretags{primary-only, might-switch}
 */
void rtdm_mutex_unlock(rtdm_mutex_t *mutex)
{
	if (!XENO_ASSERT(RTDM, !xnsched_interrupt_p()))
		return;

	trace_cobalt_driver_mutex_release(mutex);

	if (unlikely(xnsynch_release(&mutex->synch_base,
				     xnsched_current_thread()) != NULL))
		xnsched_run();
}
EXPORT_SYMBOL_GPL(rtdm_mutex_unlock);

/**
 * @brief Request a mutex
 *
 * This is the light-weight version of rtdm_mutex_timedlock(), implying an
 * infinite timeout.
 *
 * @param[in,out] mutex Mutex handle as returned by rtdm_mutex_init()
 *
 * @return 0 on success, otherwise:
 *
 * - -EIDRM is returned if @a mutex has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_mutex_lock(rtdm_mutex_t *mutex)
{
	return rtdm_mutex_timedlock(mutex, 0, NULL);
}

EXPORT_SYMBOL_GPL(rtdm_mutex_lock);

/**
 * @brief Request a mutex with timeout
 *
 * This function tries to acquire the given mutex. If it is not available, the
 * caller is blocked unless non-blocking operation was selected.
 *
 * @param[in,out] mutex Mutex handle as returned by rtdm_mutex_init()
 * @param[in] timeout Relative timeout in nanoseconds, see
 * @ref RTDM_TIMEOUT_xxx for special values
 * @param[in,out] timeout_seq Handle of a timeout sequence as returned by
 * rtdm_toseq_init() or NULL
 *
 * @return 0 on success, otherwise:
 *
 * - -ETIMEDOUT is returned if the if the request has not been satisfied
 * within the specified amount of time.
 *
 * - -EWOULDBLOCK is returned if @a timeout is negative and the semaphore
 * value is currently not positive.
 *
 * - -EIDRM is returned if @a mutex has been destroyed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{primary-only, might-switch}
 */
int rtdm_mutex_timedlock(rtdm_mutex_t *mutex, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq)
{
	struct xnthread *curr_thread;
	int err = 0;
	spl_t s;

	if (!XENO_ASSERT(RTDM, !xnsched_unblockable_p()))
		return -EPERM;

	curr_thread = xnshadow_current();
	trace_cobalt_driver_mutex_wait(mutex, curr_thread);

	xnlock_get_irqsave(&nklock, s);

	if (unlikely(mutex->synch_base.status & RTDM_SYNCH_DELETED))
		err = -EIDRM;
	else if (!xnthread_try_grab(curr_thread, &mutex->synch_base)) {
		/* Redefinition to clarify XENO_ASSERT output */
		#define mutex_owner xnsynch_owner(&mutex->synch_base)
		if (!XENO_ASSERT(RTDM, mutex_owner != curr_thread)) {
			err = -EDEADLK;
			goto unlock_out;
		}

		/* non-blocking mode */
		if (timeout < 0) {
			err = -EWOULDBLOCK;
			goto unlock_out;
		}

restart:
		if (timeout_seq && (timeout > 0)) {
			/* timeout sequence */
			xnsynch_acquire(&mutex->synch_base, *timeout_seq,
					XN_ABSOLUTE);
		} else
			/* infinite or relative timeout */
			xnsynch_acquire(&mutex->synch_base, timeout, XN_RELATIVE);

		if (unlikely(xnthread_test_info(curr_thread,
						XNTIMEO | XNRMID | XNBREAK))) {
			if (xnthread_test_info(curr_thread, XNTIMEO))
				err = -ETIMEDOUT;
			else if (xnthread_test_info(curr_thread, XNRMID))
				err = -EIDRM;
			else /*  XNBREAK */
				goto restart;
		}
	}

unlock_out:
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

EXPORT_SYMBOL_GPL(rtdm_mutex_timedlock);
/** @} */

/** @} Synchronisation services */

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_irq Interrupt Management Services
 * @{
 */

/**
 * @brief Register an interrupt handler
 *
 * This function registers the provided handler with an IRQ line and enables
 * the line.
 *
 * @param[in,out] irq_handle IRQ handle
 * @param[in] irq_no Line number of the addressed IRQ
 * @param[in] handler Interrupt handler
 * @param[in] flags Registration flags, see @ref RTDM_IRQTYPE_xxx for details
 * @param[in] device_name Device name to show up in real-time IRQ lists
 * @param[in] arg Pointer to be passed to the interrupt handler on invocation
 *
 * @return 0 on success, otherwise:
 *
 * - -EINVAL is returned if an invalid parameter was passed.
 *
 * - -EBUSY is returned if the specified IRQ line is already in use.
 *
 * @coretags{secondary-only}
 */
int rtdm_irq_request(rtdm_irq_t *irq_handle, unsigned int irq_no,
		     rtdm_irq_handler_t handler, unsigned long flags,
		     const char *device_name, void *arg)
{
	int err;

	if (!XENO_ASSERT(RTDM, xnsched_root_p()))
		return -EPERM;

	xnintr_init(irq_handle, device_name, irq_no, handler, NULL, flags);

	err = xnintr_attach(irq_handle, arg);
	if (err)
		return err;

	xnintr_enable(irq_handle);

	return 0;
}

EXPORT_SYMBOL_GPL(rtdm_irq_request);

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */
/**
 * @brief Release an interrupt handler
 *
 * @param[in,out] irq_handle IRQ handle as returned by rtdm_irq_request()
 *
 * @return 0 on success, otherwise negative error code
 *
 * @note The caller is responsible for shutting down the IRQ source at device
 * level before invoking this service. In turn, rtdm_irq_free ensures that any
 * pending event on the given IRQ line is fully processed on return from this
 * service.
 *
 * @coretags{secondary-only}
 */
int rtdm_irq_free(rtdm_irq_t *irq_handle);

/**
 * @brief Enable interrupt line
 *
 * @param[in,out] irq_handle IRQ handle as returned by rtdm_irq_request()
 *
 * @return 0 on success, otherwise negative error code
 *
 * @note This service is for exceptional use only. Drivers should always prefer
 * interrupt masking at device level (via corresponding control registers etc.)
 * over masking at line level. Keep in mind that the latter is incompatible
 * with IRQ line sharing and can also be more costly as interrupt controller
 * access requires broader synchronization. Also, certain IRQ types may not
 * allow the invocation over RT and interrupt contexts. The caller is
 * responsible for excluding such conflicts.
 *
 * @coretags{secondary-only}
 */
int rtdm_irq_enable(rtdm_irq_t *irq_handle);

/**
 * @brief Disable interrupt line
 *
 * @param[in,out] irq_handle IRQ handle as returned by rtdm_irq_request()
 *
 * @return 0 on success, otherwise negative error code
 *
 * @note This service is for exceptional use only. Drivers should always prefer
 * interrupt masking at device level (via corresponding control registers etc.)
 * over masking at line level. Keep in mind that the latter is incompatible
 * with IRQ line sharing and can also be more costly as interrupt controller
 * access requires broader synchronization. Also, certain IRQ types may not
 * allow the invocation over RT and interrupt contexts. The caller is
 * responsible for excluding such conflicts.
 *
 * @coretags{secondary-only}
 */
int rtdm_irq_disable(rtdm_irq_t *irq_handle);
#endif /* DOXYGEN_CPP */

/** @} Interrupt Management Services */

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_nrtsignal Non-Real-Time Signalling Services
 *
 * These services provide a mechanism to request the execution of a specified
 * handler in non-real-time context. The triggering can safely be performed in
 * real-time context without suffering from unknown delays. The handler
 * execution will be deferred until the next time the real-time subsystem
 * releases the CPU to the non-real-time part.
 * @{
 */

/**
 * @brief Register a non-real-time signal handler
 *
 * @param[in,out] nrt_sig Signal handle
 * @param[in] handler Non-real-time signal handler
 * @param[in] arg Custom argument passed to @c handler() on each invocation
 *
 * @return 0 on success, otherwise:
 *
 * - -EAGAIN is returned if no free signal slot is available.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_nrtsig_init(rtdm_nrtsig_t *nrt_sig, rtdm_nrtsig_handler_t handler,
		     void *arg);

/**
 * @brief Release a non-realtime signal handler
 *
 * @param[in,out] nrt_sig Signal handle
 *
 * @coretags{task-unrestricted}
 */
void rtdm_nrtsig_destroy(rtdm_nrtsig_t *nrt_sig);

/**
 * Trigger non-real-time signal
 *
 * @param[in,out] nrt_sig Signal handle
 *
 * @coretags{unrestricted}
 */
void rtdm_nrtsig_pend(rtdm_nrtsig_t *nrt_sig);
/** @} Non-Real-Time Signalling Services */

#endif /* DOXYGEN_CPP */

/**
 * @ingroup rtdm_driver_interface
 * @defgroup rtdm_util Utility Services
 * @{
 */

struct rtdm_mmap_data {
	void *src_vaddr;
	phys_addr_t src_paddr;
	struct vm_operations_struct *vm_ops;
	void *vm_private_data;
};

static int rtdm_mmap_buffer(struct file *filp, struct vm_area_struct *vma)
{
	struct rtdm_mmap_data *mmap_data = filp->private_data;
	unsigned long vaddr, maddr, size;
	phys_addr_t paddr;
	int ret;

	vma->vm_ops = mmap_data->vm_ops;
	vma->vm_private_data = mmap_data->vm_private_data;

	vaddr = (unsigned long)mmap_data->src_vaddr;
	paddr = mmap_data->src_paddr;
	if (paddr == 0)	/* kmalloc memory? */
		paddr = __pa(vaddr);

	maddr = vma->vm_start;
	size = vma->vm_end - vma->vm_start;

#ifdef CONFIG_MMU
	/* Catch vmalloc memory (vaddr is 0 for I/O mapping) */
	if ((vaddr >= VMALLOC_START) && (vaddr < VMALLOC_END)) {
		unsigned long mapped_size = 0;

		if (!XENO_ASSERT(RTDM, vaddr == PAGE_ALIGN(vaddr)))
			return -EINVAL;
		if (!XENO_ASSERT(RTDM, (size % PAGE_SIZE) == 0))
			return -EINVAL;

		while (mapped_size < size) {
			if (xnheap_remap_vm_page(vma, maddr, vaddr))
				return -EAGAIN;

			maddr += PAGE_SIZE;
			vaddr += PAGE_SIZE;
			mapped_size += PAGE_SIZE;
		}
		if (xnarch_machdesc.prefault)
			xnarch_machdesc.prefault(vma);
		ret = 0;
	} else
#else
	vma->vm_pgoff = paddr >> PAGE_SHIFT;
#endif /* CONFIG_MMU */
	if (mmap_data->src_paddr)
		ret = xnheap_remap_io_page_range(filp, vma, maddr, paddr,
						 size, PAGE_SHARED);
	else
		ret = xnheap_remap_kmem_page_range(vma, maddr, paddr,
						   size, PAGE_SHARED);
	if (xnarch_machdesc.prefault && ret == 0)
		xnarch_machdesc.prefault(vma);

	return ret;
}

#ifndef CONFIG_MMU
static unsigned long rtdm_unmapped_area(struct file *file,
					unsigned long addr,
					unsigned long len,
					unsigned long pgoff,
					unsigned long flags)
{
	struct rtdm_mmap_data *mmap_data = file->private_data;
	unsigned long pa;

	pa = mmap_data->src_paddr;
	if (pa == 0)
		pa = __pa(mmap_data->src_vaddr);

	return pa;
}
#else
#define rtdm_unmapped_area  NULL
#endif

static struct file_operations rtdm_mmap_fops = {
	.mmap = rtdm_mmap_buffer,
	.get_unmapped_area = rtdm_unmapped_area
};

static int rtdm_do_mmap(struct rtdm_fd *fd,
			struct rtdm_mmap_data *mmap_data,
			size_t len, int prot, void **pptr)
{
	const struct file_operations *old_fops;
	unsigned long u_addr;
	void *old_priv_data;
	struct file *filp;

	if (!XENO_ASSERT(RTDM, xnsched_root_p()))
		return -EPERM;

	filp = filp_open(XNHEAP_DEV_NAME, O_RDWR, 0);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	old_fops = filp->f_op;
	filp->f_op = &rtdm_mmap_fops;

	old_priv_data = filp->private_data;
	filp->private_data = mmap_data;

	u_addr = vm_mmap(filp, (unsigned long)*pptr, len, prot,
 			 MAP_SHARED, 0);

	filp->f_op = (typeof(filp->f_op))old_fops;
	filp->private_data = old_priv_data;

	filp_close(filp, current->files);

	if (IS_ERR_VALUE(u_addr))
		return (int)u_addr;

	*pptr = (void *)u_addr;

	return 0;
}

/**
 * Map a kernel memory range into the address space of the user.
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] src_addr Kernel virtual address to be mapped
 * @param[in] len Length of the memory range
 * @param[in] prot Protection flags for the user's memory range, typically
 * either PROT_READ or PROT_READ|PROT_WRITE
 * @param[in,out] pptr Address of a pointer containing the desired user
 * address or NULL on entry and the finally assigned address on return
 * @param[in] vm_ops vm_operations to be executed on the vma_area of the
 * user memory range or NULL
 * @param[in] vm_private_data Private data to be stored in the vma_area,
 * primarily useful for vm_operation handlers
 *
 * @return 0 on success, otherwise (most common values):
 *
 * - -EINVAL is returned if an invalid start address, size, or destination
 * address was passed.
 *
 * - -ENOMEM is returned if there is insufficient free memory or the limit of
 * memory mapping for the user process was reached.
 *
 * - -EAGAIN is returned if too much memory has been already locked by the
 * user process.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @note This service only works on memory regions allocated via kmalloc() or
 * vmalloc(). To map physical I/O memory to user-space use
 * rtdm_iomap_to_user() instead.
 *
 * @note RTDM supports two models for unmapping the user memory range again.
 * One is explicit unmapping via rtdm_munmap(), either performed when the
 * user requests it via an IOCTL etc. or when the related device is closed.
 * The other is automatic unmapping, triggered by the user invoking standard
 * munmap() or by the termination of the related process. To track release of
 * the mapping and therefore relinquishment of the referenced physical memory,
 * the caller of rtdm_mmap_to_user() can pass a vm_operations_struct on
 * invocation, defining a close handler for the vm_area. See Linux
 * documentaion (e.g. Linux Device Drivers book) on virtual memory management
 * for details.
 *
 * @coretags{secondary-only}
 */
int rtdm_mmap_to_user(struct rtdm_fd *fd,
		      void *src_addr, size_t len,
		      int prot, void **pptr,
		      struct vm_operations_struct *vm_ops,
		      void *vm_private_data)
{
	struct rtdm_mmap_data mmap_data = {
		.src_vaddr = src_addr,
		.src_paddr = 0,
		.vm_ops = vm_ops,
		.vm_private_data = vm_private_data
	};

	return rtdm_do_mmap(fd, &mmap_data, len, prot, pptr);
}

EXPORT_SYMBOL_GPL(rtdm_mmap_to_user);

/**
 * Map an I/O memory range into the address space of the user.
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] src_addr physical I/O address to be mapped
 * @param[in] len Length of the memory range
 * @param[in] prot Protection flags for the user's memory range, typically
 * either PROT_READ or PROT_READ|PROT_WRITE
 * @param[in,out] pptr Address of a pointer containing the desired user
 * address or NULL on entry and the finally assigned address on return
 * @param[in] vm_ops vm_operations to be executed on the vma_area of the
 * user memory range or NULL
 * @param[in] vm_private_data Private data to be stored in the vma_area,
 * primarily useful for vm_operation handlers
 *
 * @return 0 on success, otherwise (most common values):
 *
 * - -EINVAL is returned if an invalid start address, size, or destination
 * address was passed.
 *
 * - -ENOMEM is returned if there is insufficient free memory or the limit of
 * memory mapping for the user process was reached.
 *
 * - -EAGAIN is returned if too much memory has been already locked by the
 * user process.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @note RTDM supports two models for unmapping the user memory range again.
 * One is explicit unmapping via rtdm_munmap(), either performed when the
 * user requests it via an IOCTL etc. or when the related device is closed.
 * The other is automatic unmapping, triggered by the user invoking standard
 * munmap() or by the termination of the related process. To track release of
 * the mapping and therefore relinquishment of the referenced physical memory,
 * the caller of rtdm_iomap_to_user() can pass a vm_operations_struct on
 * invocation, defining a close handler for the vm_area. See Linux
 * documentaion (e.g. Linux Device Drivers book) on virtual memory management
 * for details.
 *
 * @coretags{secondary-only}
 */
int rtdm_iomap_to_user(struct rtdm_fd *fd,
		       phys_addr_t src_addr, size_t len,
		       int prot, void **pptr,
		       struct vm_operations_struct *vm_ops,
		       void *vm_private_data)
{
	struct rtdm_mmap_data mmap_data = {
		.src_vaddr = NULL,
		.src_paddr = src_addr,
		.vm_ops = vm_ops,
		.vm_private_data = vm_private_data
	};

	return rtdm_do_mmap(fd, &mmap_data, len, prot, pptr);
}

EXPORT_SYMBOL_GPL(rtdm_iomap_to_user);

/**
 * Unmap a user memory range.
 *
 * @param[in] fd RTDM file descriptor as passed to 
 * rtdm_mmap_to_user() when requesting to map the memory range
 * @param[in] ptr User address or the memory range
 * @param[in] len Length of the memory range
 *
 * @return 0 on success, otherwise:
 *
 * - -EINVAL is returned if an invalid address or size was passed.
 *
 * - -EPERM @e may be returned if an illegal invocation environment is
 * detected.
 *
 * @coretags{secondary-only}
 */
int rtdm_munmap(struct rtdm_fd *fd, void *ptr, size_t len)
{
	int err;

	if (!XENO_ASSERT(RTDM, xnsched_root_p()))
		return -EPERM;

	down_write(&current->mm->mmap_sem);
	err = do_munmap(current->mm, (unsigned long)ptr, len);
	up_write(&current->mm->mmap_sem);

	return err;
}
EXPORT_SYMBOL_GPL(rtdm_munmap);

/**
 * @brief Enforces a rate limit
 *
 * This function enforces a rate limit: not more than @a rs->burst callbacks
 * in every @a rs->interval.
 *
 * @param[in,out] rs rtdm_ratelimit_state data
 * @param[in] func name of calling function
 *
 * @return 0 means callback will be suppressed and 1 means go ahead and do it
 *
 * @coretags{unrestricted}
 */
int rtdm_ratelimit(struct rtdm_ratelimit_state *rs, const char *func)
{
	rtdm_lockctx_t lock_ctx;
	int ret;

	if (!rs->interval)
		return 1;

	rtdm_lock_get_irqsave(&rs->lock, lock_ctx);

	if (!rs->begin)
		rs->begin = rtdm_clock_read();
	if (rtdm_clock_read() >= rs->begin + rs->interval) {
		if (rs->missed)
			printk(KERN_WARNING "%s: %d callbacks suppressed\n",
			       func, rs->missed);
		rs->begin   = 0;
		rs->printed = 0;
		rs->missed  = 0;
	}
	if (rs->burst && rs->burst > rs->printed) {
		rs->printed++;
		ret = 1;
	} else {
		rs->missed++;
		ret = 0;
	}
	rtdm_lock_put_irqrestore(&rs->lock, lock_ctx);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_ratelimit);

#ifdef DOXYGEN_CPP /* Only used for doxygen doc generation */

/**
 * Real-time safe rate-limited message printing on kernel console
 *
 * @param[in] format Format string (conforming standard @c printf())
 * @param ... Arguments referred by @a format
 *
 * @return On success, this service returns the number of characters printed.
 * Otherwise, a negative error code is returned.
 *
 * @coretags{unrestricted}
 */
void rtdm_printk_ratelimited(const char *format, ...);

/**
 * Real-time safe message printing on kernel console
 *
 * @param[in] format Format string (conforming standard @c printf())
 * @param ... Arguments referred by @a format
 *
 * @return On success, this service returns the number of characters printed.
 * Otherwise, a negative error code is returned.
 *
 * @coretags{unrestricted}
 */
void rtdm_printk(const char *format, ...);

/**
 * Allocate memory block
 *
 * @param[in] size Requested size of the memory block
 *
 * @return The pointer to the allocated block is returned on success, NULL
 * otherwise.
 *
 * @coretags{unrestricted}
 */
void *rtdm_malloc(size_t size);

/**
 * Release real-time memory block
 *
 * @param[in] ptr Pointer to memory block as returned by rtdm_malloc()
 *
 * @coretags{unrestricted}
 */
void rtdm_free(void *ptr);

/**
 * Check if read access to user-space memory block is safe
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] ptr Address of the user-provided memory block
 * @param[in] size Size of the memory block
 *
 * @return Non-zero is return when it is safe to read from the specified
 * memory block, 0 otherwise.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_read_user_ok(struct rtdm_fd *fd, const void __user *ptr,
		      size_t size);

/**
 * Check if read/write access to user-space memory block is safe
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] ptr Address of the user-provided memory block
 * @param[in] size Size of the memory block
 *
 * @return Non-zero is return when it is safe to read from or write to the
 * specified memory block, 0 otherwise.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_rw_user_ok(struct rtdm_fd *fd, const void __user *ptr,
		    size_t size);

/**
 * Copy user-space memory block to specified buffer
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] dst Destination buffer address
 * @param[in] src Address of the user-space memory block
 * @param[in] size Size of the memory block
 *
 * @return 0 on success, otherwise:
 *
 * - -EFAULT is returned if an invalid memory area was accessed.
 *
 * @note Before invoking this service, verify via rtdm_read_user_ok() that the
 * provided user-space address can securely be accessed.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_copy_from_user(struct rtdm_fd *fd, void *dst,
			const void __user *src, size_t size);

/**
 * Check if read access to user-space memory block and copy it to specified
 * buffer
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] dst Destination buffer address
 * @param[in] src Address of the user-space memory block
 * @param[in] size Size of the memory block
 *
 * @return 0 on success, otherwise:
 *
 * - -EFAULT is returned if an invalid memory area was accessed.
 *
 * @note This service is a combination of rtdm_read_user_ok and
 * rtdm_copy_from_user.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_safe_copy_from_user(struct rtdm_fd *fd, void *dst,
			     const void __user *src, size_t size);

/**
 * Copy specified buffer to user-space memory block
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] dst Address of the user-space memory block
 * @param[in] src Source buffer address
 * @param[in] size Size of the memory block
 *
 * @return 0 on success, otherwise:
 *
 * - -EFAULT is returned if an invalid memory area was accessed.
 *
 * @note Before invoking this service, verify via rtdm_rw_user_ok() that the
 * provided user-space address can securely be accessed.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_copy_to_user(struct rtdm_fd *fd, void __user *dst,
		      const void *src, size_t size);

/**
 * Check if read/write access to user-space memory block is safe and copy
 * specified buffer to it
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] dst Address of the user-space memory block
 * @param[in] src Source buffer address
 * @param[in] size Size of the memory block
 *
 * @return 0 on success, otherwise:
 *
 * - -EFAULT is returned if an invalid memory area was accessed.
 *
 * @note This service is a combination of rtdm_rw_user_ok and
 * rtdm_copy_to_user.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_safe_copy_to_user(struct rtdm_fd *fd, void __user *dst,
			   const void *src, size_t size);

/**
 * Copy user-space string to specified buffer
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 * @param[in] dst Destination buffer address
 * @param[in] src Address of the user-space string
 * @param[in] count Maximum number of bytes to copy, including the trailing
 * '0'
 *
 * @return Length of the string on success (not including the trailing '0'),
 * otherwise:
 *
 * - -EFAULT is returned if an invalid memory area was accessed.
 *
 * @note This services already includes a check of the source address,
 * calling rtdm_read_user_ok() for @a src explicitly is not required.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_strncpy_from_user(struct rtdm_fd *fd, char *dst,
			   const char __user *src, size_t count);

/**
 * Test if running in a real-time task
 *
 * @return Non-zero is returned if the caller resides in real-time context, 0
 * otherwise.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_in_rt_context(void);

/**
 * Test if the caller is capable of running in real-time context
 *
 * @param[in] fd RTDM file descriptor as passed to the invoked
 * device operation handler
 *
 * @return Non-zero is returned if the caller is able to execute in real-time
 * context (independent of its current execution mode), 0 otherwise.
 *
 * @note This function can be used by drivers that provide different
 * implementations for the same service depending on the execution mode of
 * the caller. If a caller requests such a service in non-real-time context
 * but is capable of running in real-time as well, it might be appropriate
 * for the driver to reject the request via -ENOSYS so that RTDM can switch
 * the caller and restart the request in real-time context.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_rt_capable(struct rtdm_fd *fd);

#endif /* DOXYGEN_CPP */

/** @} Utility Services */
