/**
 *   @ingroup hal
 *   @file
 *
 *   Adeos-based Real-Time Abstraction Layer for ARM.
 *
 *   ARM port
 *     Copyright (C) 2005 Stelian Pop
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 *
 *   ARM-specific HAL services.
 */
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/ipipe_tickdev.h>
#include <asm/system.h>
#include <asm/hardirq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/xenomai/hal.h>
#include <asm/cacheflush.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif /* CONFIG_PROC_FS */
#include <stdarg.h>

static volatile int sync_op;

rthal_u32frac_t rthal_tsc_to_timer;
EXPORT_SYMBOL_GPL(rthal_tsc_to_timer);

#define RTHAL_CALIBRATE_LOOPS 10

enum rthal_ktimer_mode rthal_ktimer_saved_mode;

#define RTHAL_SET_ONESHOT_XENOMAI	1
#define RTHAL_SET_ONESHOT_LINUX		2
#define RTHAL_SET_PERIODIC		3

static inline void steal_timer(int stolen)
{
	/*
	 * Some platform-specific I-pipe bits may want to know whether
	 * non-vanilla kernel code is currently fiddling with the
	 * timer chip; setting this flag on tells them so.
	 */
	__ipipe_mach_timerstolen = stolen;
}

static inline void force_oneshot_hw_mode(void)
{
	/*
	 * Program next tick ahead at a sensible date. We expect
	 * __ipipe_mach_set_dec() to switch off any auto-reload mode
	 * if that makes sense for the hardware.
	 */
	__ipipe_mach_set_dec(__ipipe_mach_ticks_per_jiffy);
}

static inline void restore_normal_hw_mode(void)
{
	steal_timer(0);
	/*
	 * Ask the I-pipe to reset the normal timer operating mode at
	 * hardware level, which should match the current logical mode
	 * for the active clockevent.
	 */
	__ipipe_mach_release_timer();
}

unsigned long rthal_timer_calibrate(void)
{
	unsigned long long start, end, sum = 0, sum_sq = 0;
	volatile unsigned const_delay = 0xffffffff;
	unsigned long result, flags, tsc_lat;
	unsigned int delay = const_delay;
	long long diff;
	int i, j;

	flags = ipipe_critical_enter(NULL);

	/*
	 * Hw interrupts off, other CPUs quiesced, no migration
	 * possible. We can now fiddle with the timer chip (per-cpu
	 * local or global, rthal_timer_program_shot() will handle
	 * this transparently via the I-pipe).
	 */
	steal_timer(1);
	force_oneshot_hw_mode();

	ipipe_read_tsc(start);
	barrier();
	ipipe_read_tsc(end);
	tsc_lat = end - start;
	barrier();

	for (i = 0; i < RTHAL_CALIBRATE_LOOPS; i++) {
		flush_cache_all();
		for (j = 0; j < RTHAL_CALIBRATE_LOOPS; j++) {
			ipipe_read_tsc(start);
			barrier();
			rthal_timer_program_shot(
				rthal_nodiv_imuldiv_ceil(delay, rthal_tsc_to_timer));
			barrier();
			ipipe_read_tsc(end);
			diff = end - start - tsc_lat;
			if (diff > 0) {
				sum += diff;
				sum_sq += diff * diff;
			}
		}
	}

	restore_normal_hw_mode();

	ipipe_critical_exit(flags);

	/* Use average + standard deviation as timer programming latency. */
	do_div(sum, RTHAL_CALIBRATE_LOOPS * RTHAL_CALIBRATE_LOOPS);
	do_div(sum_sq, RTHAL_CALIBRATE_LOOPS * RTHAL_CALIBRATE_LOOPS);
	result = sum + int_sqrt(sum_sq - sum * sum) + 1;

	return result;
}

#ifdef CONFIG_SMP
static void critical_sync(void)
{
	switch (sync_op) {
	case RTHAL_SET_ONESHOT_XENOMAI:
		force_oneshot_hw_mode();
		steal_timer(1);
		break;

	case RTHAL_SET_ONESHOT_LINUX:
		force_oneshot_hw_mode();
		steal_timer(0);
		/* We need to keep the timing cycle alive for the kernel. */
		ipipe_raise_irq(RTHAL_TIMER_IRQ);
		break;

	case RTHAL_SET_PERIODIC:
		restore_normal_hw_mode();
		break;
	}
}
#else /* CONFIG_SMP */
#define rthal_critical_sync NULL
#endif /* !CONFIG_SMP */

static void rthal_timer_set_oneshot(int rt_mode)
{
	unsigned long flags;

	flags = ipipe_critical_enter(critical_sync);

	if (rt_mode) {
		sync_op = RTHAL_SET_ONESHOT_XENOMAI;
		force_oneshot_hw_mode();
		steal_timer(1);
	} else {
		sync_op = RTHAL_SET_ONESHOT_LINUX;
		force_oneshot_hw_mode();
		steal_timer(0);
		/* We need to keep the timing cycle alive for the kernel. */
		ipipe_raise_irq(RTHAL_TIMER_IRQ);
	}
	ipipe_critical_exit(flags);
}

static void rthal_timer_set_periodic(void)
{
	unsigned long flags;

	flags = ipipe_critical_enter(critical_sync);
	sync_op = RTHAL_SET_PERIODIC;
	restore_normal_hw_mode();
	ipipe_critical_exit(flags);
}

static int cpu_timers_requested;

int rthal_timer_request(
	void (*tick_handler)(void),
	void (*mode_emul)(enum clock_event_mode mode,
			  struct clock_event_device *cdev),
	int (*tick_emul)(unsigned long delay,
			 struct clock_event_device *cdev),
	int cpu)
{
	unsigned long dummy, *tmfreq = &dummy;
	int tickval, ret;

	if (rthal_timerfreq_arg == 0)
		tmfreq = &rthal_archdata.timer_freq;

	ret = ipipe_request_tickdev(RTHAL_TIMER_DEVICE, mode_emul, tick_emul, cpu,
				    tmfreq);
	switch (ret) {
	case CLOCK_EVT_MODE_PERIODIC:
		/* oneshot tick emulation callback won't be used, ask
		 * the caller to start an internal timer for emulating
		 * a periodic tick. */
		tickval = 1000000000UL / HZ;
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* oneshot tick emulation */
		tickval = 1;
		break;

	case CLOCK_EVT_MODE_UNUSED:
		/* we don't need to emulate the tick at all. */
		tickval = 0;
		break;

	case CLOCK_EVT_MODE_SHUTDOWN:
		return -ENODEV;

	default:
		return ret;
	}

	rthal_ktimer_saved_mode = ret;

	/*
	 * The rest of the initialization should only be performed
	 * once by a single CPU.
	 */
	if (cpu_timers_requested++ > 0)
		goto out;

	ret = ipipe_request_irq(&rthal_archdata.domain,
				RTHAL_TIMER_IRQ,
				(ipipe_irq_handler_t)tick_handler,
				NULL, NULL);
	if (ret)
		return ret;

#ifdef CONFIG_SMP
	ret = ipipe_request_irq(&rthal_archdata.domain,
				RTHAL_TIMER_IPI,
				(ipipe_irq_handler_t)tick_handler,
				NULL, NULL);
	if (ret)
		return ret;
#endif /* CONFIG_SMP */

	rthal_timer_set_oneshot(1);
out:
	return tickval;
}

void rthal_timer_release(int cpu)
{
	ipipe_release_tickdev(cpu);

	if (--cpu_timers_requested > 0)
		return;

	ipipe_free_irq(&rthal_archdata.domain, RTHAL_TIMER_IRQ);
#ifdef CONFIG_SMP
	ipipe_free_irq(&rthal_archdata.domain, RTHAL_TIMER_IPI);
#endif /* CONFIG_SMP */

	if (rthal_ktimer_saved_mode == KTIMER_MODE_PERIODIC)
		rthal_timer_set_periodic();
	else if (rthal_ktimer_saved_mode == KTIMER_MODE_ONESHOT)
		rthal_timer_set_oneshot(0);
}

void rthal_timer_notify_switch(enum clock_event_mode mode,
			       struct clock_event_device *cdev)
{
	if (ipipe_processor_id() > 0)
		/*
		 * We assume all CPUs switch the same way, so we only
		 * track mode switches from the boot CPU.
		 */
		return;

	rthal_ktimer_saved_mode = mode;
}
EXPORT_SYMBOL_GPL(rthal_timer_notify_switch);

void __rthal_arm_fault_range(struct vm_area_struct *vma)
{
	unsigned long addr;

	if ((vma->vm_flags & VM_MAYREAD)) {
		unsigned flags;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
		flags = (vma->vm_flags & VM_MAYWRITE) ? FAULT_FLAG_WRITE : 0;
#else /* linux <= 2.6.22 */
		flags = vma->vm_flags & VM_MAYWRITE
#endif /* linux <= 2.6.22 */

		for (addr = vma->vm_start;
		     addr != vma->vm_end; addr += PAGE_SIZE)
			handle_mm_fault(vma->vm_mm, vma, addr, flags);
	}
}

int rthal_arch_init(void)
{
	if (rthal_timerfreq_arg == 0)
		rthal_timerfreq_arg = rthal_get_timerfreq();

	if (rthal_clockfreq_arg == 0)
		rthal_clockfreq_arg = rthal_get_clockfreq();

	xnarch_init_u32frac(&rthal_tsc_to_timer,
			    rthal_timerfreq_arg, rthal_clockfreq_arg);

	return 0;
}

void rthal_arch_cleanup(void)
{
	/* Nothing to cleanup so far. */
	printk(KERN_INFO "Xenomai: hal/arm stopped.\n");
}

EXPORT_SYMBOL_GPL(rthal_arch_init);
EXPORT_SYMBOL_GPL(rthal_arch_cleanup);
EXPORT_SYMBOL_GPL(rthal_thread_switch);
EXPORT_SYMBOL_GPL(rthal_thread_trampoline);
EXPORT_SYMBOL_GPL(__rthal_arm_fault_range);
#if defined(CONFIG_VFP) && defined(CONFIG_XENO_HW_FPU)
EXPORT_SYMBOL_GPL(last_VFP_context);
EXPORT_SYMBOL_GPL(rthal_vfp_save);
EXPORT_SYMBOL_GPL(rthal_vfp_load);
#endif /* CONFIG_VFP && CONFIG_XENO_HW_FPU */
