/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <cobalt/kernel/arith.h>
#include <rtdm/driver.h>
#include <rtdm/autotune.h>

struct autotune_state {
	rtdm_timer_t timer;
	xnticks_t ideal;
	xnticks_t step;
	xnsticks_t min_lat;
	xnsticks_t max_lat;
	xnsticks_t sum_lat;
	unsigned long cur_samples;
	unsigned long max_samples;
	rtdm_event_t done;
};

static void timer_handler(rtdm_timer_t *timer)
{
	struct autotune_state *state;
	xnsticks_t delta;
	xnticks_t now;

	state = container_of(timer, struct autotune_state, timer);
	now = xnclock_read_raw(&nkclock);

	delta = (xnsticks_t)(now - state->ideal);
	if (delta < state->min_lat)
		state->min_lat = delta;
	if (delta > state->max_lat)
		state->max_lat = delta;

	state->sum_lat += delta;
	state->ideal += state->step;

	if (++state->cur_samples >= state->max_samples) {
		rtdm_timer_stop(timer);
		rtdm_event_signal(&state->done);
	}
}

static int autotune_open(struct rtdm_fd *fd, int oflags)
{
	struct autotune_state *state;
	int ret;

	state = rtdm_fd_to_private(fd);

	ret = rtdm_timer_init(&state->timer, timer_handler, "autotune");
	if (ret)
		return ret;

	rtdm_event_init(&state->done, 0);

	return 0;
}

#define ONE_SECOND 1000000000UL

static int tune_timer_gravity(struct autotune_state *state, int period)
{
	xnsticks_t min_ns, max_ns, avg_ns, grav_ns, prev_min;
	xnticks_t gravity, prev_gravity;
	int ret, step, stable = 0;
	long adjust;

	printk(XENO_INFO "auto-tuning core clock gravity\n");

	state->step = xnclock_ns_to_ticks(&nkclock, period);
	state->max_samples = ONE_SECOND / (period ?: 1);
	gravity = nkclock.gravity;
	nkclock.gravity = prev_gravity = 0;
	prev_min = 0;

	for (step = 1; step <= 30; step++) {
		state->ideal = xnclock_read_raw(&nkclock) + state->step;
		state->min_lat = -1LL >> 1;
		state->max_lat = 0;
		state->sum_lat = 0;
		state->cur_samples = 0;

		ret = rtdm_timer_start(&state->timer,
				       xnclock_ticks_to_ns(&nkclock, state->ideal),
				       period, RTDM_TIMERMODE_ABSOLUTE);
		if (ret)
			goto fail;

		ret = rtdm_event_wait(&state->done);
		if (ret)
			goto fail;

		min_ns = xnclock_ticks_to_ns(&nkclock, state->min_lat);
		max_ns = xnclock_ticks_to_ns(&nkclock, state->max_lat);
		avg_ns = xnarch_llimd(xnclock_ticks_to_ns(&nkclock, state->sum_lat), 1,
				      (state->cur_samples ?: 1));
		grav_ns = xnclock_ticks_to_ns(&nkclock, nkclock.gravity);

		if (step > 1) {
			if (state->min_lat < 0 && prev_min > 0) {
				nkclock.gravity = prev_gravity;
				break;
			}
			if (state->min_lat > 0 && prev_min < 0)
				break;
		}

		adjust = (long)state->min_lat / 2;

#ifdef CONFIG_XENO_OPT_DEBUG_NUCLEUS
		printk(XENO_INFO "autotune: min=%Ld | max=%Ld | avg=%Ld\n",
		       min_ns, max_ns, avg_ns);
#endif

		if (adjust)
			stable = 0;
		else if (++stable == 3)
			break;

		prev_gravity = nkclock.gravity;
		prev_min = state->min_lat;

		if (adjust == 0 && state->min_lat < 0)
			adjust = (long)state->min_lat;

		if (nkclock.gravity >= -adjust)
			nkclock.gravity += adjust;
	}

	if (step > 30) {
		printk(XENO_INFO "could not auto-tune after 30s\n");
		return -EINVAL;
	}

	printk(XENO_INFO "auto-tuning completed: gravity=%Lu  [min=%Ld | max=%Ld | avg=%Ld]\n",
	       xnclock_ticks_to_ns(&nkclock, nkclock.gravity),
	       min_ns, max_ns, avg_ns);

	return 0;
fail:
	nkclock.gravity = gravity;

	return ret;
}

static int autotune_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct autotune_state *state;
	int ret, period;

	if (request != AUTOTUNE_RTIOC_LATENCY)
		return -EINVAL;

	ret = rtdm_safe_copy_from_user(fd, &period, arg, sizeof(period));
	if (ret)
		return ret;

	state = rtdm_fd_to_private(fd);

	return tune_timer_gravity(state, period);
}

static void autotune_close(struct rtdm_fd *fd)
{
	struct autotune_state *state;

	state = rtdm_fd_to_private(fd);
	rtdm_timer_destroy(&state->timer);
	rtdm_event_destroy(&state->done);
}

static struct rtdm_device device = {
	.struct_version		=	RTDM_DEVICE_STRUCT_VER,
	.device_flags		=	RTDM_NAMED_DEVICE|RTDM_EXCLUSIVE,
	.context_size		=	sizeof(struct autotune_state),
	.open			=	autotune_open,
	.ops = {
		.ioctl_rt	=	autotune_ioctl,
		.close		=	autotune_close,
	},
	.device_class		=	RTDM_CLASS_AUTOTUNE,
	.device_sub_class	=	RTDM_SUBCLASS_AUTOTUNE,
	.device_name		=	"autotune",
	.driver_name		=	"autotune",
	.driver_version		=	RTDM_DRIVER_VER(1, 0, 0),
	.peripheral_name	=	"Auto-tuning services",
	.proc_name		=	device.device_name,
	.provider_name		=	"Philippe Gerum <rpm@xenomai.org>",
};

static int __init autotune_init(void)
{
	int ret;

	if (!realtime_core_enabled())
		return 0;

	ret = rtdm_dev_register(&device);
	if (ret)
		return ret;

	return 0;
}

device_initcall(autotune_init);

MODULE_LICENSE("GPL");
