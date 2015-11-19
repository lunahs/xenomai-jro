/*
 * Copyright (C) 2015 Jorge Ramirez <jro@xenomai.org>.
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

#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <rtdm/driver.h>
#include <rtdm/gpiopwm.h>

MODULE_AUTHOR("Jorge Ramirez <jro@xenomai.org>");
MODULE_DESCRIPTION("PWM driver");
MODULE_VERSION("0.0.1");
MODULE_LICENSE("GPL");

#define MAX_DUTY_CYCLE		100
#define MAX_SAMPLES		(MAX_DUTY_CYCLE + 1)

struct gpiopwm_debug_info {
	unsigned int expected[MAX_SAMPLES];
	unsigned int max_on[MAX_SAMPLES];
	unsigned int min_on[MAX_SAMPLES];
	unsigned int delta[MAX_SAMPLES];
	nanosecs_abs_t on;
};

struct gpiopwm_base_signal {
	unsigned long period;
};

struct gpiopwm_duty_signal {
	unsigned int range_min;
	unsigned int range_max;
	unsigned long period;
	unsigned int cycle;
};

struct gpiopwm_control {
	struct gpiopwm_duty_signal duty;
	unsigned int configured;
	unsigned int update;
};

struct gpiopwm_priv {
	struct gpiopwm_base_signal base;
	struct gpiopwm_duty_signal duty;
	struct gpiopwm_control ctrl;

	rtdm_timer_t base_timer;
	rtdm_timer_t duty_timer;

	int gpio;
	struct gpiopwm_debug_info *dbg;
};

static inline int div100(long long dividend)
{
	const long long divisor = 0x28f5c29;
	return ((divisor * dividend) >> 32) & 0xffffffff;
}

static inline unsigned long duty_period(struct gpiopwm_duty_signal *p)
{
	unsigned long period;

	period = p->range_min + div100((p->range_max - p->range_min) * p->cycle);

	/* period is set in nsec */
	period = period * 1000;

	return period;
}

static inline void gpiopwm_dbg_print_stats(struct gpiopwm_priv *ctx,
				struct rtdm_dev_context *dev_ctx, int delta)
{
	unsigned int min, max, prev_max, next_min, expected;
	unsigned int overlap, duty_cycle;
	char *dev_name;
	int gpio;

	duty_cycle = ctx->duty.cycle;
	expected = ctx->dbg->expected[duty_cycle];

	min = ctx->dbg->min_on[duty_cycle];
	next_min = duty_cycle < MAX_DUTY_CYCLE ? ctx->dbg->min_on[duty_cycle + 1] : UINT_MAX;

	max = ctx->dbg->max_on[duty_cycle];
	prev_max = duty_cycle > 0 ? ctx->dbg->max_on[duty_cycle - 1] : 0;

	dev_name = dev_ctx->device->name;
	gpio = ctx->gpio;

	rtdm_printk("%s: gpio,duty: %03d,%03d, (min:%05d,max:%05d) "
			"=> delta %05d us, expected %05d\n",
			dev_name, gpio, duty_cycle, min, max, delta, expected);

	if (min < prev_max) {
		overlap = prev_max - min;
		rtdm_printk("\t\toverlap min warn %05d us: \n"
			"\t\tmin[%02d]=%05d < max[%02d]=%05d, expected min=%05d\n",
			overlap,
			ctx->duty.cycle,
			prev_max,
			ctx->duty.cycle - 1,
			min,
			expected);
	}

	if (max > next_min) {
		overlap = max - next_min;
		rtdm_printk("\t\toverlap max warn: %05d us\n"
			"\t\tmax[%02d]=%05d > min[%02d]=%05d, expected max=%05d\n",
			overlap,
			ctx->duty.cycle,
			max,
			ctx->duty.cycle + 1,
			next_min,
			expected);
	}
}

static inline void gpiopwm_dbg_update(struct gpiopwm_priv *ctx)
{
	struct rtdm_fd *fd = rtdm_private_to_fd(ctx);
	struct rtdm_dev_context *dev_ctx = rtdm_fd_to_context(fd);
	unsigned int *max, *min, *delta;
	uint64_t on;

	delta = &ctx->dbg->delta[ctx->duty.cycle];
	max = &ctx->dbg->max_on[ctx->duty.cycle];
	min = &ctx->dbg->min_on[ctx->duty.cycle];

	on = (rtdm_clock_read() - ctx->dbg->on) / 1000;

	if (on > *max)
		*max = on;
	if (on < *min)
		*min = on;
	if (*max - *min > *delta) {
		*delta = *max - *min;
		gpiopwm_dbg_print_stats(ctx, dev_ctx, *delta);
	}
}

static void gpiopwm_handle_base_timer(rtdm_timer_t *timer)
{
	struct gpiopwm_priv *ctx = container_of(timer, struct gpiopwm_priv,
						base_timer);
	gpio_set_value(ctx->gpio, 1);

	if (ctx->dbg)
		ctx->dbg->on = rtdm_clock_read();

	if (ctx->ctrl.update) {
		ctx->duty.period = ctx->ctrl.duty.period;
		ctx->duty.cycle = ctx->ctrl.duty.cycle;
		ctx->ctrl.update = 0;
	}

	/* one shot timer to avoid carrying over errors */
	rtdm_timer_start(&ctx->duty_timer, ctx->duty.period, 0, RTDM_TIMERMODE_RELATIVE);
}

static void gpiopwm_handle_duty_timer(rtdm_timer_t *timer)
{
	struct gpiopwm_priv *ctx = container_of(timer, struct gpiopwm_priv,
						duty_timer);

	gpio_set_value(ctx->gpio, 0);

	if (ctx->dbg)
		gpiopwm_dbg_update(ctx);
}

static inline int gpiopwm_config(struct rtdm_fd *fd, struct gpiopwm *conf)
{
	struct rtdm_dev_context *dev_ctx = rtdm_fd_to_context(fd);
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);
	int ret, i;

	if (ctx->ctrl.configured)
		return -EINVAL;

	if (conf->duty_cycle > MAX_DUTY_CYCLE)
		return -EINVAL;

	ret = gpio_request(conf->gpio, dev_ctx->device->name);
	if (ret < 0) {
		ctx->gpio = -1;
		return ret;
	}

	ret = gpio_direction_output(conf->gpio, 0);
	if (ret < 0)
		return ret;

	gpio_set_value(conf->gpio, 0);

	if (conf->debug)
		ctx->dbg = kzalloc(sizeof(*ctx->dbg), GFP_KERNEL);

	ctx->duty.range_min = ctx->ctrl.duty.range_min = conf->range_min;
	ctx->duty.range_max = ctx->ctrl.duty.range_max = conf->range_max;

	for (i = 0; ctx->dbg && i < MAX_SAMPLES; i++) {

		ctx->dbg->min_on[i] = UINT_MAX;
		ctx->dbg->max_on[i] = 0;
		ctx->dbg->delta[i] = 0;

		ctx->duty.cycle = i;
		ctx->dbg->expected[i] = duty_period(&ctx->duty) / 1000;
	}

	ctx->duty.cycle = conf->duty_cycle;
	ctx->base.period = conf->period;
	ctx->gpio = conf->gpio;
	ctx->duty.period = duty_period(&ctx->duty);

	rtdm_timer_init(&ctx->base_timer, gpiopwm_handle_base_timer, "base_timer");
	rtdm_timer_init(&ctx->duty_timer, gpiopwm_handle_duty_timer, "duty_timer");

	ctx->ctrl.configured = 1;

	return 0;
}

static inline int gpiopwm_change_duty_cycle(struct gpiopwm_priv *ctx, unsigned int cycle)
{
	if (cycle > MAX_DUTY_CYCLE)
		return -EINVAL;

	/* prepare the new data on the calling thread */
	ctx->ctrl.duty.cycle = cycle;
	ctx->ctrl.duty.period = duty_period(&ctx->ctrl.duty);

	/* update data on the next base signal timeout */
	ctx->ctrl.update = 1;

	return 0;
}

static inline int gpiopwm_stop(struct rtdm_fd *fd)
{
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);

	if (!ctx->ctrl.configured)
		return -EINVAL;

	gpio_set_value(ctx->gpio, 0);

	rtdm_timer_stop(&ctx->base_timer);
	rtdm_timer_stop(&ctx->duty_timer);

	return 0;
}

static inline int gpiopwm_start(struct rtdm_fd *fd)
{
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);

	if (!ctx->ctrl.configured)
		return -EINVAL;

	/* update duty cycle on next timeout */
	ctx->ctrl.update = 1;

	/* start the base signal tick */
	rtdm_timer_start(&ctx->base_timer, ctx->base.period, ctx->base.period,
			 RTDM_TIMERMODE_RELATIVE);

	return 0;
}

static int gpiopwm_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void __user *arg)
{
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);
	int err = 0;

	switch (request) {
	case GPIOPWM_RTIOC_SET_CONFIG:
		return -ENOSYS;
	case GPIOPWM_RTIOC_CHANGE_DUTY_CYCLE:
		return gpiopwm_change_duty_cycle(ctx, (unsigned long) arg);
		break;
	case GPIOPWM_RTIOC_START:
		return gpiopwm_start(fd);
	case GPIOPWM_RTIOC_STOP:
		return gpiopwm_stop(fd);
	default:
		err = -EINVAL;
	}

	return err;
}

static int gpiopwm_ioctl_nrt(struct rtdm_fd *fd, unsigned int request, void __user *arg)
{
	struct gpiopwm conf;
	int err = 0;

	switch (request) {
	case GPIOPWM_RTIOC_SET_CONFIG:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(conf)))
			return -EFAULT;

		rtdm_copy_from_user(fd, &conf, arg, sizeof(conf));
		err = gpiopwm_config(fd, &conf);
		break;
	case GPIOPWM_RTIOC_GET_CONFIG:
	default:
		err = -EINVAL;
	}

	return err;
}

static int gpiopwm_open(struct rtdm_fd *fd, int oflags)
{
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);

	ctx->ctrl.configured = 0;
	ctx->dbg = NULL;
	ctx->gpio = -1;

	return 0;
}

static void gpiopwm_close(struct rtdm_fd *fd)
{
	struct gpiopwm_priv *ctx = rtdm_fd_to_private(fd);

	if (ctx->gpio >= 0)
		gpio_free(ctx->gpio);

	if (!ctx->ctrl.configured)
		return;

	if (ctx->dbg)
		kfree(ctx->dbg);

	rtdm_timer_destroy(&ctx->base_timer);
	rtdm_timer_destroy(&ctx->duty_timer);
}

static struct rtdm_driver gpiopwm_driver = {
	.profile_info           = RTDM_PROFILE_INFO(gpiopwm,
						    RTDM_CLASS_PWM,
						    RTDM_SUBCLASS_GENERIC,
						    RTPWM_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 8,
	.context_size		= sizeof(struct gpiopwm_priv),
	.ops = {
		.open		= gpiopwm_open,
		.close		= gpiopwm_close,
		.ioctl_rt	= gpiopwm_ioctl_rt,
		.ioctl_nrt	= gpiopwm_ioctl_nrt,
	},
};

static struct rtdm_device device[8] = {
	[0 ... 7] = {
		.driver = &gpiopwm_driver,
		.label = "gpiopwm%d",
	}
};

static int __init __gpiopwm_init(void)
{
	int i, ret;

	if (!realtime_core_enabled())
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(device); i++) {
		ret = rtdm_dev_register(device + i);
		if (ret)
			goto fail;
	}

	return 0;
fail:
	while (i-- > 0)
		rtdm_dev_unregister(device + i);

	return ret;
}

static void __exit __gpiopwm_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(device); i++)
		rtdm_dev_unregister(device + i);
}

module_init(__gpiopwm_init);
module_exit(__gpiopwm_exit);
