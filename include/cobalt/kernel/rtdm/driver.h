/**
 * @file
 * Real-Time Driver Model for Xenomai, driver API header
 *
 * @note Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 * @note Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 * @note Copyright (C) 2008 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
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
 *
 * @ingroup driverapi
 */
#ifndef _COBALT_RTDM_DRIVER_H
#define _COBALT_RTDM_DRIVER_H

#include <asm/atomic.h>
#include <linux/list.h>

#include <xenomai/version.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/select.h>
#include <cobalt/kernel/vfile.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/apc.h>
#include <cobalt/kernel/shadow.h>
#include <cobalt/kernel/init.h>
#include <cobalt/kernel/ancillaries.h>
#include <cobalt/kernel/tree.h>
#include <rtdm/fd.h>
#include <rtdm/rtdm.h>

/* debug support */
#include <cobalt/kernel/assert.h>
#include <trace/events/cobalt-rtdm.h>
#ifdef CONFIG_PCI
#include <asm-generic/xenomai/pci_ids.h>
#endif /* CONFIG_PCI */

struct rtdm_dev_context;
typedef struct xnselector rtdm_selector_t;
enum rtdm_selecttype;

/*!
 * @addtogroup rtdm_device_register
 * @{
 */

/*!
 * @anchor dev_flags @name Device Flags
 * Static flags describing a RTDM device
 * @{
 */
/** If set, only a single instance of the device can be requested by an
 *  application. */
#define RTDM_EXCLUSIVE			0x0001

/** If set, the device is addressed via a clear-text name. */
#define RTDM_NAMED_DEVICE		0x0010

/** If set, the device is addressed via a combination of protocol ID and
 *  socket type. */
#define RTDM_PROTOCOL_DEVICE		0x0020

/** Mask selecting the device type. */
#define RTDM_DEVICE_TYPE_MASK		0x00F0
/** @} Device Flags */

/*!
 * @anchor drv_versioning @name Driver Versioning
 * Current revisions of RTDM structures, encoding of driver versions. See
 * @ref rtdm_api_versioning "API Versioning" for the interface revision.
 * @{
 */
/** Version of struct rtdm_device */
#define RTDM_DEVICE_STRUCT_VER		6

/** Version of struct rtdm_dev_context */
#define RTDM_CONTEXT_STRUCT_VER		4

/** Flag indicating a secure variant of RTDM (not supported here) */
#define RTDM_SECURE_DEVICE		0x80000000

/** Version code constructor for driver revisions */
#define RTDM_DRIVER_VER(major, minor, patch) \
	(((major & 0xFF) << 16) | ((minor & 0xFF) << 8) | (patch & 0xFF))

/** Get major version number from driver revision code */
#define RTDM_DRIVER_MAJOR_VER(ver)	(((ver) >> 16) & 0xFF)

/** Get minor version number from driver revision code */
#define RTDM_DRIVER_MINOR_VER(ver)	(((ver) >> 8) & 0xFF)

/** Get patch version number from driver revision code */
#define RTDM_DRIVER_PATCH_VER(ver)	((ver) & 0xFF)
/** @} Driver Versioning */

/** @} rtdm_device_register */

/*!
 * @addtogroup rtdm_sync
 * @{
 */

/*!
 * @anchor RTDM_SELECTTYPE_xxx   @name RTDM_SELECTTYPE_xxx
 * Event types select can bind to
 * @{
 */
enum rtdm_selecttype {
	/** Select input data availability events */
	RTDM_SELECTTYPE_READ = XNSELECT_READ,

	/** Select ouput buffer availability events */
	RTDM_SELECTTYPE_WRITE = XNSELECT_WRITE,

	/** Select exceptional events */
	RTDM_SELECTTYPE_EXCEPT = XNSELECT_EXCEPT
};
/** @} RTDM_SELECTTYPE_xxx */

/** @} rtdm_sync */

/*!
 * @name Operation Handler Prototypes
 * @{
 */

/**
 * Named device open handler
 *
 * @param[in] fd File descriptor structure associated with opened device instance
 * @param[in] oflag Open flags as passed by the user
 *
 * @return 0 on success. On failure return either -ENOSYS, to request that
 * this handler be called again from the opposite realtime/non-realtime
 * context, or another negative error code.
 *
 * @see @c open() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399 */
typedef int (*rtdm_open_handler_t)(struct rtdm_fd *fd, int oflag);

/**
 * Socket creation handler for protocol devices
 *
 * @param[in] fd File descriptor structure associated with opened device instance
 * @param[in] protocol Protocol number as passed by the user
 *
 * @return 0 on success. On failure return either -ENOSYS, to request that
 * this handler be called again from the opposite realtime/non-realtime
 * context, or another negative error code.
 *
 * @see @c socket() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399 */
typedef int (*rtdm_socket_handler_t)(struct rtdm_fd *fd, int protocol);

/** @} Operation Handler Prototypes */

struct rtdm_process;

struct rtdm_devctx_reserved {
	struct rtdm_process *owner;
	void (*close)(struct rtdm_fd *fd);
};

/**
 * @brief Device context
 *
 * A device context structure is associated with every open device instance.
 * RTDM takes care of its creation and destruction and passes it to the
 * operation handlers when being invoked.
 *
 * Drivers can attach arbitrary data immediately after the official structure.
 * The size of this data is provided via rtdm_device.context_size during
 * device registration.
 */
struct rtdm_dev_context {
	struct rtdm_fd fd;

	/** Set of active device operation handlers */
	/** Reference to owning device */
	struct rtdm_device *device;

	/** Data stored by RTDM inside a device context (internal use only) */
	struct rtdm_devctx_reserved reserved;

	/** Begin of driver defined context data structure */
	char dev_private[0];
};

static inline struct rtdm_dev_context *rtdm_fd_to_context(struct rtdm_fd *fd)
{
	return container_of(fd, struct rtdm_dev_context, fd);
}

/**
 * Locate the driver private area associated to a device context structure
 *
 * @param[in] fd File descriptor structure associated with opened
 * device instance
 *
 * @return The address of the private driver area associated to @a
 * file descriptor.
 */
static inline void *rtdm_fd_to_private(struct rtdm_fd *fd)
{
	return (void *)rtdm_fd_to_context(fd)->dev_private;
}

/**
 * Locate a device file descriptor structure from its driver private area
 *
 * @param[in] dev_private Address of a private context area
 *
 * @return The address of the file descriptor structure defining @a
 * dev_private.
 */
static inline struct rtdm_fd *rtdm_private_to_fd(void *dev_private)
{
	struct rtdm_dev_context *ctx;
	ctx = container_of(dev_private, struct rtdm_dev_context, dev_private);
	return &ctx->fd;
}

/**
 * Tell whether the passed file descriptor belongs to an application.
 *
 * @param[in] fd File descriptor
 *
 * @return true if passed file descriptor belongs to an application,
 * false otherwise.
 */
static inline bool rtdm_fd_is_user(struct rtdm_fd *fd)
{
	return rtdm_fd_owner(fd) != &__xnsys_global_ppd;
}

/**
 * Locate a device structure from a file descriptor.
 *
 * @param[in] fd File descriptor
 *
 * @return The address of the device structure to which this file
 * descriptor is attached.
 */
static inline struct rtdm_device *rtdm_fd_device(struct rtdm_fd *fd)
{
	return rtdm_fd_to_context(fd)->device;
}

struct rtdm_dev_reserved {
	unsigned magic;
	union {
		struct {
			struct list_head entry;
			xnhandle_t handle;
		};
		struct xnid id;
	};
	atomic_t refcount;
	struct rtdm_dev_context *exclusive_context;
	void (*close)(struct rtdm_fd *);
};

/**
 * @brief RTDM device
 *
 * This structure specifies a RTDM device. As some fields, especially the
 * reserved area, will be modified by RTDM during runtime, the structure must
 * not reside in write-protected memory.
 */
struct rtdm_device {
	/** Data stored by RTDM inside a registered device (internal use only) */
	struct rtdm_dev_reserved reserved;

	/** Revision number of this structure, see
	 *  @ref drv_versioning "Driver Versioning" defines */
	int struct_version;

	/** Device flags, see @ref dev_flags "Device Flags" for details */
	int device_flags;
	/** Size of driver defined appendix to struct rtdm_dev_context */
	size_t context_size;

	/** Named device identification (orthogonal to Linux device name space) */
	char device_name[RTDM_MAX_DEVNAME_LEN + 1];

	/** Protocol device identification: protocol family (PF_xxx) */
	int protocol_family;
	/** Protocol device identification: socket type (SOCK_xxx) */
	int socket_type;

	/** Named device instance creation for real-time contexts. */
	rtdm_open_handler_t open;

	/** Protocol socket creation for real-time contexts. */
	rtdm_socket_handler_t socket;

	/** Default operations on newly opened device instance */
	struct rtdm_fd_ops ops;

	/** Device class ID, see @ref RTDM_CLASS_xxx */
	int device_class;
	/** Device sub-class, see RTDM_SUBCLASS_xxx definition in the
	 *  @ref rtdm_profiles "Device Profiles" */
	int device_sub_class;
	/** Supported device profile version */
	int profile_version;
	/** Informational driver name (reported via /proc) */
	const char *driver_name;
	/** Driver version, see @ref drv_versioning "Driver Versioning" defines */
	int driver_version;
	/** Informational peripheral name the device is attached to
	 *  (reported via /proc) */
	const char *peripheral_name;
	/** Informational driver provider name (reported via /proc) */
	const char *provider_name;

	/** Name of /proc entry for the device, must not be NULL */
	const char *proc_name;
#ifdef CONFIG_XENO_OPT_VFILE
	/** Set to device's vfile data after registration, do not modify */
	struct xnvfile_directory vfroot;
	struct xnvfile_regular info_vfile;
#endif

	/** Driver definable device ID */
	int device_id;
	/** Driver definable device data */
	void *device_data;
};
/** @} devregister */

/* --- device registration --- */

int rtdm_dev_register(struct rtdm_device *device);
int rtdm_dev_unregister(struct rtdm_device *device, unsigned int poll_delay);

/* --- inter-driver API --- */

#define rtdm_open		rt_dev_open
#define rtdm_socket		rt_dev_socket
#define rtdm_close		rt_dev_close
#define rtdm_ioctl		rt_dev_ioctl
#define rtdm_read		rt_dev_read
#define rtdm_write		rt_dev_write
#define rtdm_recvmsg		rt_dev_recvmsg
#define rtdm_recv		rt_dev_recv
#define rtdm_recvfrom		rt_dev_recvfrom
#define rtdm_sendmsg		rt_dev_sendmsg
#define rtdm_send		rt_dev_send
#define rtdm_sendto		rt_dev_sendto
#define rtdm_bind		rt_dev_bind
#define rtdm_listen		rt_dev_listen
#define rtdm_accept		rt_dev_accept
#define rtdm_getsockopt		rt_dev_getsockopt
#define rtdm_setsockopt		rt_dev_setsockopt
#define rtdm_getsockname	rt_dev_getsockname
#define rtdm_getpeername	rt_dev_getpeername
#define rtdm_shutdown		rt_dev_shutdown

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */

/* --- clock services --- */
static inline nanosecs_abs_t rtdm_clock_read(void)
{
	return xnclock_read_realtime(&nkclock);
}

static inline nanosecs_abs_t rtdm_clock_read_monotonic(void)
{
	return xnclock_read_monotonic(&nkclock);
}
#endif /* !DOXYGEN_CPP */

/* --- timeout sequences */

typedef nanosecs_abs_t rtdm_toseq_t;

void rtdm_toseq_init(rtdm_toseq_t *timeout_seq, nanosecs_rel_t timeout);

/*!
 * @addtogroup rtdm_sync
 * @{
 */

/*!
 * @defgroup rtdm_sync_biglock Big dual kernel lock
 * @{
 */

/**
 * @brief Enter atomic section (dual kernel only)
 *
 * This call opens a fully atomic section, serializing execution with
 * respect to all interrupt handlers (including for real-time IRQs)
 * and Xenomai threads running on all CPUs.
 *
 * @param context name of local variable to store the context in. This
 * variable updated by the real-time core will hold the information
 * required to leave the atomic section properly.
 *
 * @note Atomic sections may be nested.
 *
 * @note Since the strongest lock is acquired by this service, it can
 * be used to synchronize real-time and non-real-time contexts.
 *
 * @warning This service is not portable to the Mercury core, and
 * should be restricted to Cobalt-specific use cases.
 */
#define cobalt_atomic_enter(context)			\
	do {						\
		xnlock_get_irqsave(&nklock, (context));	\
		__xnsched_lock();			\
	} while (0)

/**
 * @brief Leave atomic section (dual kernel only)
 *
 * This call closes an atomic section previously opened by a call to
 * cobalt_atomic_enter(), restoring the preemption and interrupt state
 * which prevailed prior to entering the exited section.
 *
 * @param context name of local variable which stored the context.
 *
 * @warning This service is not portable to the Mercury core, and
 * should be restricted to Cobalt-specific use cases.
 */
#define cobalt_atomic_leave(context)				\
	do {							\
		__xnsched_unlock();				\
		xnlock_put_irqrestore(&nklock, (context));	\
	} while (0)

/**
 * @brief Execute code block atomically (DEPRECATED)
 *
 * Generally, it is illegal to suspend the current task by calling
 * rtdm_task_sleep(), rtdm_event_wait(), etc. while holding a spinlock. In
 * contrast, this macro allows to combine several operations including
 * a potentially rescheduling call to an atomic code block with respect to
 * other RTDM_EXECUTE_ATOMICALLY() blocks. The macro is a light-weight
 * alternative for protecting code blocks via mutexes, and it can even be used
 * to synchronise real-time and non-real-time contexts.
 *
 * @param code_block Commands to be executed atomically
 *
 * @note It is not allowed to leave the code block explicitly by using
 * @c break, @c return, @c goto, etc. This would leave the global lock held
 * during the code block execution in an inconsistent state. Moreover, do not
 * embed complex operations into the code bock. Consider that they will be
 * executed under preemption lock with interrupts switched-off. Also note that
 * invocation of rescheduling calls may break the atomicity until the task
 * gains the CPU again.
 *
 * @coretags{unrestricted}
 *
 * @deprecated This construct will be phased out in Xenomai
 * 3.0. Please use rtdm_waitqueue services instead.
 */
#ifdef DOXYGEN_CPP /* Beautify doxygen output */
#define RTDM_EXECUTE_ATOMICALLY(code_block)	\
{						\
	<ENTER_ATOMIC_SECTION>			\
	code_block;				\
	<LEAVE_ATOMIC_SECTION>			\
}
#else /* This is how it really works */
static inline __attribute__((deprecated)) void
rtdm_execute_atomically(void) { }

#define RTDM_EXECUTE_ATOMICALLY(code_block)		\
{							\
	spl_t __rtdm_s;					\
							\
	rtdm_execute_atomically();			\
	xnlock_get_irqsave(&nklock, __rtdm_s);		\
	__xnsched_lock();				\
	code_block;					\
	__xnsched_unlock();				\
	xnlock_put_irqrestore(&nklock, __rtdm_s);	\
}
#endif

/** @} Big dual kernel lock */

/**
 * @defgroup rtdm_sync_spinlock Spinlock with preemption deactivation
 * @{
 */

/**
 * Static lock initialisation
 */
#define RTDM_LOCK_UNLOCKED(__name)	IPIPE_SPIN_LOCK_UNLOCKED

#define DEFINE_RTDM_LOCK(__name)		\
	rtdm_lock_t __name = RTDM_LOCK_UNLOCKED(__name)

/** Lock variable */
typedef ipipe_spinlock_t rtdm_lock_t;

/** Variable to save the context while holding a lock */
typedef unsigned long rtdm_lockctx_t;

/**
 * Dynamic lock initialisation
 *
 * @param lock Address of lock variable
 *
 * @coretags{task-unrestricted}
 */
static inline void rtdm_lock_init(rtdm_lock_t *lock)
{
	spin_lock_init(lock);
}

/**
 * Acquire lock from non-preemptible contexts
 *
 * @param lock Address of lock variable
 *
 * @coretags{unrestricted}
 */
static inline void rtdm_lock_get(rtdm_lock_t *lock)
{
	XENO_BUGON(RTDM, !spltest());
	spin_lock(lock);
	__xnsched_lock();
}

/**
 * Release lock without preemption restoration
 *
 * @param lock Address of lock variable
 *
 * @coretags{unrestricted, might-switch}
 */
static inline void rtdm_lock_put(rtdm_lock_t *lock)
{
	spin_unlock(lock);
	__xnsched_unlock();
}

/**
 * @fn void rtdm_lock_get_irqsave(rtdm_lock_t *lock, rtdm_lockctx_t context)
 * Acquire lock and disable preemption, by stalling the head domain.
 *
 * @param lock Address of lock variable
 * @param context name of local variable to store the context in
 *
 * @coretags{unrestricted}
 */
static inline rtdm_lockctx_t __rtdm_lock_get_irqsave(rtdm_lock_t *lock)
{
	rtdm_lockctx_t context;

	context = ipipe_test_and_stall_head();
	spin_lock(lock);
	__xnsched_lock();

	return context;
}
#define rtdm_lock_get_irqsave(__lock, __context)	\
	((__context) = __rtdm_lock_get_irqsave(__lock))

/**
 * Release lock and restore preemption state
 *
 * @param lock Address of lock variable
 * @param context name of local variable which stored the context
 *
 * @coretags{unrestricted}
 */
static inline
void rtdm_lock_put_irqrestore(rtdm_lock_t *lock, rtdm_lockctx_t context)
{
	spin_unlock(lock);
	__xnsched_unlock();
	ipipe_restore_head(context);
}

/**
 * Disable preemption locally
 *
 * @param context name of local variable to store the context in
 *
 * @coretags{unrestricted}
 */
#define rtdm_lock_irqsave(context)	\
	splhigh(context)

/**
 * Restore preemption state
 *
 * @param context name of local variable which stored the context
 *
 * @coretags{unrestricted}
 */
#define rtdm_lock_irqrestore(context)	\
	splexit(context)

/** @} Spinlock with Preemption Deactivation */

#ifndef DOXYGEN_CPP

struct rtdm_waitqueue {
	struct xnsynch wait;
};
typedef struct rtdm_waitqueue rtdm_waitqueue_t;

#define RTDM_WAITQUEUE_INITIALIZER(__name) {		 \
	    .wait = XNSYNCH_WAITQUEUE_INITIALIZER((__name).wait), \
	}

#define DEFINE_RTDM_WAITQUEUE(__name)				\
	struct rtdm_waitqueue __name = RTDM_WAITQUEUE_INITIALIZER(__name)

#define DEFINE_RTDM_WAITQUEUE_ONSTACK(__name)	\
	DEFINE_RTDM_WAITQUEUE(__name)

static inline void rtdm_waitqueue_init(struct rtdm_waitqueue *wq)
{
	*wq = (struct rtdm_waitqueue)RTDM_WAITQUEUE_INITIALIZER(*wq);
}

static inline void rtdm_waitqueue_destroy(struct rtdm_waitqueue *wq)
{
	xnsynch_destroy(&wq->wait);
}

static inline int __rtdm_timedwait(struct rtdm_waitqueue *wq,
				   nanosecs_rel_t timeout, rtdm_toseq_t *toseq)
{
	if (toseq && timeout > 0)
		return xnsynch_sleep_on(&wq->wait, *toseq, XN_ABSOLUTE);

	return xnsynch_sleep_on(&wq->wait, timeout, XN_RELATIVE);
}

#define rtdm_timedwait_condition_locked(__wq, __cond, __timeout, __toseq) \
	({								\
		int __ret = 0;						\
		while (__ret == 0 && !(__cond))				\
			__ret = __rtdm_timedwait(__wq, __timeout, __toseq); \
		__ret;							\
	})

#define rtdm_wait_condition_locked(__wq, __cond)			\
	({								\
		int __ret = 0;						\
		while (__ret == 0 && !(__cond))				\
			__ret = xnsynch_sleep_on(&(__wq)->wait,		\
						 XN_INFINITE, XN_RELATIVE); \
		__ret;							\
	})

#define rtdm_timedwait_condition(__wq, __cond, __timeout, __toseq)	\
	({								\
		spl_t __s;						\
		int __ret;						\
		xnlock_get_irqsave(&nklock, __s);			\
		__ret = rtdm_timedwait_condition_locked(__wq, __cond,	\
					      __timeout, __toseq);	\
		xnlock_put_irqrestore(&nklock, __s);			\
		__ret;							\
	})

#define rtdm_timedwait(__wq, __timeout, __toseq)			\
	__rtdm_timedwait(__wq, __timeout, __toseq)

#define rtdm_timedwait_locked(__wq, __timeout, __toseq)			\
	rtdm_timedwait(__wq, __timeout, __toseq)

#define rtdm_wait_condition(__wq, __cond)				\
	({								\
		spl_t __s;						\
		int __ret;						\
		xnlock_get_irqsave(&nklock, __s);			\
		__ret = rtdm_wait_condition_locked(__wq, __cond);	\
		xnlock_put_irqrestore(&nklock, __s);			\
		__ret;							\
	})

#define rtdm_wait(__wq)							\
	xnsynch_sleep_on(&(__wq)->wait,	XN_INFINITE, XN_RELATIVE)

#define rtdm_wait_locked(__wq)  rtdm_wait(__wq)

#define rtdm_waitqueue_lock(__wq, __context)  cobalt_atomic_enter(__context)

#define rtdm_waitqueue_unlock(__wq, __context)  cobalt_atomic_leave(__context)

#define rtdm_waitqueue_signal(__wq)					\
	({								\
		struct xnthread *__waiter;				\
		__waiter = xnsynch_wakeup_one_sleeper(&(__wq)->wait);	\
		xnsched_run();						\
		__waiter != NULL;					\
	})

#define __rtdm_waitqueue_flush(__wq, __reason)				\
	({								\
		int __ret;						\
		__ret = xnsynch_flush(&(__wq)->wait, __reason);		\
		xnsched_run();						\
		__ret == XNSYNCH_RESCHED;				\
	})

#define rtdm_waitqueue_broadcast(__wq)	\
	__rtdm_waitqueue_flush(__wq, 0)

#define rtdm_waitqueue_flush(__wq)	\
	__rtdm_waitqueue_flush(__wq, XNBREAK)

#define rtdm_waitqueue_wakeup(__wq, __waiter)				\
	do {								\
		xnsynch_wakeup_this_sleeper(&(__wq)->wait, __waiter);	\
		xnsched_run();						\
	} while (0)

#define rtdm_for_each_waiter(__pos, __wq)		\
	xnsynch_for_each_sleeper(__pos, &(__wq)->wait)

#define rtdm_for_each_waiter_safe(__pos, __tmp, __wq)	\
	xnsynch_for_each_sleeper_safe(__pos, __tmp, &(__wq)->wait)

#endif /* !DOXYGEN_CPP */

/** @} rtdm_sync */

/* --- Interrupt management services --- */
/*!
 * @addtogroup rtdm_irq
 * @{
 */

typedef struct xnintr rtdm_irq_t;

/*!
 * @anchor RTDM_IRQTYPE_xxx   @name RTDM_IRQTYPE_xxx
 * Interrupt registrations flags
 * @{
 */
/** Enable IRQ-sharing with other real-time drivers */
#define RTDM_IRQTYPE_SHARED		XN_ISR_SHARED
/** Mark IRQ as edge-triggered, relevant for correct handling of shared
 *  edge-triggered IRQs */
#define RTDM_IRQTYPE_EDGE		XN_ISR_EDGE
/** @} RTDM_IRQTYPE_xxx */

/**
 * Interrupt handler
 *
 * @param[in] irq_handle IRQ handle as returned by rtdm_irq_request()
 *
 * @return 0 or a combination of @ref RTDM_IRQ_xxx flags
 */
typedef int (*rtdm_irq_handler_t)(rtdm_irq_t *irq_handle);

/*!
 * @anchor RTDM_IRQ_xxx   @name RTDM_IRQ_xxx
 * Return flags of interrupt handlers
 * @{
 */
/** Unhandled interrupt */
#define RTDM_IRQ_NONE			XN_ISR_NONE
/** Denote handled interrupt */
#define RTDM_IRQ_HANDLED		XN_ISR_HANDLED
/** @} RTDM_IRQ_xxx */

/**
 * Retrieve IRQ handler argument
 *
 * @param irq_handle IRQ handle
 * @param type Type of the pointer to return
 *
 * @return The argument pointer registered on rtdm_irq_request() is returned,
 * type-casted to the specified @a type.
 *
 * @coretags{unrestricted}
 */
#define rtdm_irq_get_arg(irq_handle, type)	((type *)irq_handle->cookie)
/** @} rtdm_irq */

int rtdm_irq_request(rtdm_irq_t *irq_handle, unsigned int irq_no,
		     rtdm_irq_handler_t handler, unsigned long flags,
		     const char *device_name, void *arg);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline int rtdm_irq_free(rtdm_irq_t *irq_handle)
{
	if (!XENO_ASSERT(RTDM, xnsched_root_p()))
		return -EPERM;
	xnintr_detach(irq_handle);
	return 0;
}

static inline int rtdm_irq_enable(rtdm_irq_t *irq_handle)
{
	xnintr_enable(irq_handle);
	return 0;
}

static inline int rtdm_irq_disable(rtdm_irq_t *irq_handle)
{
	xnintr_disable(irq_handle);
	return 0;
}
#endif /* !DOXYGEN_CPP */

/* --- non-real-time signalling services --- */

/*!
 * @addtogroup rtdm_nrtsignal
 * @{
 */

typedef unsigned rtdm_nrtsig_t;

/**
 * Non-real-time signal handler
 *
 * @param[in] nrt_sig Signal handle as returned by rtdm_nrtsig_init()
 * @param[in] arg Argument as passed to rtdm_nrtsig_init()
 *
 * @note The signal handler will run in soft-IRQ context of the non-real-time
 * subsystem. Note the implications of this context, e.g. no invocation of
 * blocking operations.
 */
typedef void (*rtdm_nrtsig_handler_t)(rtdm_nrtsig_t nrt_sig, void *arg);
/** @} rtdm_nrtsignal */

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline int rtdm_nrtsig_init(rtdm_nrtsig_t *nrt_sig,
				   rtdm_nrtsig_handler_t handler, void *arg)
{
	*nrt_sig = ipipe_alloc_virq();
	if (*nrt_sig == 0)
		return -EAGAIN;

	ipipe_request_irq(ipipe_root_domain, *nrt_sig, handler, arg, NULL);

	return 0;
}

static inline void rtdm_nrtsig_destroy(rtdm_nrtsig_t *nrt_sig)
{
	ipipe_free_irq(ipipe_root_domain, *nrt_sig);
	ipipe_free_virq(*nrt_sig);
}

static inline void rtdm_nrtsig_pend(rtdm_nrtsig_t *nrt_sig)
{
	ipipe_raise_irq(*nrt_sig);
}
#endif /* !DOXYGEN_CPP */

/* --- timer services --- */

/*!
 * @addtogroup rtdm_timer
 * @{
 */

typedef struct xntimer rtdm_timer_t;

/**
 * Timer handler
 *
 * @param[in] timer Timer handle as returned by rtdm_timer_init()
 */
typedef void (*rtdm_timer_handler_t)(rtdm_timer_t *timer);

/*!
 * @anchor RTDM_TIMERMODE_xxx   @name RTDM_TIMERMODE_xxx
 * Timer operation modes
 * @{
 */
enum rtdm_timer_mode {
	/** Monotonic timer with relative timeout */
	RTDM_TIMERMODE_RELATIVE = XN_RELATIVE,

	/** Monotonic timer with absolute timeout */
	RTDM_TIMERMODE_ABSOLUTE = XN_ABSOLUTE,

	/** Adjustable timer with absolute timeout */
	RTDM_TIMERMODE_REALTIME = XN_REALTIME
};
/** @} RTDM_TIMERMODE_xxx */

/** @} rtdm_timer */

#ifndef DOXYGEN_CPP /* Avoid broken doxygen output */
#define rtdm_timer_init(timer, handler, name)		\
({							\
	xntimer_init((timer), &nkclock, handler, NULL);	\
	xntimer_set_name((timer), (name));		\
	0;						\
})
#endif /* !DOXYGEN_CPP */

void rtdm_timer_destroy(rtdm_timer_t *timer);

int rtdm_timer_start(rtdm_timer_t *timer, nanosecs_abs_t expiry,
		     nanosecs_rel_t interval, enum rtdm_timer_mode mode);

void rtdm_timer_stop(rtdm_timer_t *timer);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline int rtdm_timer_start_in_handler(rtdm_timer_t *timer,
					      nanosecs_abs_t expiry,
					      nanosecs_rel_t interval,
					      enum rtdm_timer_mode mode)
{
	return xntimer_start(timer, expiry, interval, (xntmode_t)mode);
}

static inline void rtdm_timer_stop_in_handler(rtdm_timer_t *timer)
{
	xntimer_stop(timer);
}
#endif /* !DOXYGEN_CPP */

/* --- task services --- */
/*!
 * @addtogroup rtdm_task
 * @{
 */

typedef struct xnthread rtdm_task_t;

/**
 * Real-time task procedure
 *
 * @param[in,out] arg argument as passed to rtdm_task_init()
 */
typedef void (*rtdm_task_proc_t)(void *arg);

/**
 * @anchor rtdmtaskprio @name Task Priority Range
 * Maximum and minimum task priorities
 * @{ */
#define RTDM_TASK_LOWEST_PRIORITY	0
#define RTDM_TASK_HIGHEST_PRIORITY	99
/** @} Task Priority Range */

/**
 * @anchor rtdmchangetaskprio @name Task Priority Modification
 * Raise or lower task priorities by one level
 * @{ */
#define RTDM_TASK_RAISE_PRIORITY	(+1)
#define RTDM_TASK_LOWER_PRIORITY	(-1)
/** @} Task Priority Modification */

/** @} rtdm_task */

int rtdm_task_init(rtdm_task_t *task, const char *name,
		   rtdm_task_proc_t task_proc, void *arg,
		   int priority, nanosecs_rel_t period);
int __rtdm_task_sleep(xnticks_t timeout, xntmode_t mode);
void rtdm_task_busy_sleep(nanosecs_rel_t delay);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline void rtdm_task_destroy(rtdm_task_t *task)
{
	xnthread_cancel(task);
	xnthread_join(task, true);
}

static inline int rtdm_task_should_stop(void)
{
	return xnthread_test_info(xnshadow_current(), XNCANCELD);
}

void rtdm_task_join(rtdm_task_t *task);

static inline void __deprecated rtdm_task_join_nrt(rtdm_task_t *task,
						   unsigned int poll_delay)
{
	rtdm_task_join(task);
}

static inline void rtdm_task_set_priority(rtdm_task_t *task, int priority)
{
	union xnsched_policy_param param = { .rt = { .prio = priority } };
	xnthread_set_schedparam(task, &xnsched_class_rt, &param);
	xnsched_run();
}

static inline int rtdm_task_set_period(rtdm_task_t *task,
				       nanosecs_rel_t period)
{
	if (period < 0)
		period = 0;

	return xnthread_set_periodic(task, XN_INFINITE, XN_RELATIVE, period);
}

static inline int rtdm_task_unblock(rtdm_task_t *task)
{
	int res = xnthread_unblock(task);

	xnsched_run();
	return res;
}

static inline rtdm_task_t *rtdm_task_current(void)
{
	return xnshadow_current();
}

static inline int rtdm_task_wait_period(void)
{
	if (!XENO_ASSERT(RTDM, !xnsched_unblockable_p()))
		return -EPERM;
	return xnthread_wait_period(NULL);
}

static inline int rtdm_task_sleep(nanosecs_rel_t delay)
{
	return __rtdm_task_sleep(delay, XN_RELATIVE);
}

static inline int
rtdm_task_sleep_abs(nanosecs_abs_t wakeup_date, enum rtdm_timer_mode mode)
{
	/* For the sake of a consistent API usage... */
	if (mode != RTDM_TIMERMODE_ABSOLUTE && mode != RTDM_TIMERMODE_REALTIME)
		return -EINVAL;
	return __rtdm_task_sleep(wakeup_date, (xntmode_t)mode);
}

/* rtdm_task_sleep_abs shall be used instead */
static inline int __deprecated rtdm_task_sleep_until(nanosecs_abs_t wakeup_time)
{
	return __rtdm_task_sleep(wakeup_time, XN_REALTIME);
}

#define rtdm_task_busy_wait(__condition, __busy_time, __sleep_time)		\
	({									\
		__label__ done;							\
		nanosecs_abs_t __end;						\
		int __ret = 0;							\
		for (;;) {							\
			__end = rtdm_clock_read_monotonic() + __busy_time;	\
			for (;;) {      					\
				if (__condition)				\
					goto done;				\
				if (rtdm_clock_read_monotonic() >= __end) 	\
					break;					\
			}							\
			__ret = rtdm_task_sleep(__sleep_time);			\
			if (__ret)						\
				break;						\
		}								\
	done:									\
		__ret;								\
	})

#endif /* !DOXYGEN_CPP */

/* --- event services --- */

typedef struct rtdm_event {
	struct xnsynch synch_base;
	DECLARE_XNSELECT(select_block);
} rtdm_event_t;

#define RTDM_EVENT_PENDING		XNSYNCH_SPARE1

void rtdm_event_init(rtdm_event_t *event, unsigned long pending);
int rtdm_event_select_bind(rtdm_event_t *event, rtdm_selector_t *selector,
			   enum rtdm_selecttype type, unsigned fd_index);
int rtdm_event_wait(rtdm_event_t *event);
int rtdm_event_timedwait(rtdm_event_t *event, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq);
void rtdm_event_signal(rtdm_event_t *event);

void rtdm_event_clear(rtdm_event_t *event);

void rtdm_event_pulse(rtdm_event_t *event);

void rtdm_event_destroy(rtdm_event_t *event);

/* --- semaphore services --- */

typedef struct rtdm_sem {
	unsigned long value;
	struct xnsynch synch_base;
	DECLARE_XNSELECT(select_block);
} rtdm_sem_t;

void rtdm_sem_init(rtdm_sem_t *sem, unsigned long value);
int rtdm_sem_select_bind(rtdm_sem_t *sem, rtdm_selector_t *selector,
			 enum rtdm_selecttype type, unsigned fd_index);
int rtdm_sem_down(rtdm_sem_t *sem);
int rtdm_sem_timeddown(rtdm_sem_t *sem, nanosecs_rel_t timeout,
		       rtdm_toseq_t *timeout_seq);
void rtdm_sem_up(rtdm_sem_t *sem);

void rtdm_sem_destroy(rtdm_sem_t *sem);

/* --- mutex services --- */

typedef struct rtdm_mutex {
	struct xnsynch synch_base;
	atomic_long_t fastlock;
} rtdm_mutex_t;

void rtdm_mutex_init(rtdm_mutex_t *mutex);
int rtdm_mutex_lock(rtdm_mutex_t *mutex);
int rtdm_mutex_timedlock(rtdm_mutex_t *mutex, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq);
void rtdm_mutex_unlock(rtdm_mutex_t *mutex);
void rtdm_mutex_destroy(rtdm_mutex_t *mutex);

/* --- utility functions --- */

#define rtdm_printk(format, ...)	printk(format, ##__VA_ARGS__)

struct rtdm_ratelimit_state {
	rtdm_lock_t	lock;		/* protect the state */
	nanosecs_abs_t  interval;
	int		burst;
	int		printed;
	int		missed;
	nanosecs_abs_t	begin;
};

int rtdm_ratelimit(struct rtdm_ratelimit_state *rs, const char *func);

#define DEFINE_RTDM_RATELIMIT_STATE(name, interval_init, burst_init)	\
	struct rtdm_ratelimit_state name = {				\
		.lock		= RTDM_LOCK_UNLOCKED((name).lock),	\
		.interval	= interval_init,			\
		.burst		= burst_init,				\
	}

/* We use the Linux defaults */
#define DEF_RTDM_RATELIMIT_INTERVAL	5000000000LL
#define DEF_RTDM_RATELIMIT_BURST	10

#define rtdm_printk_ratelimited(fmt, ...)  ({				\
	static DEFINE_RTDM_RATELIMIT_STATE(_rs,				\
					   DEF_RTDM_RATELIMIT_INTERVAL,	\
					   DEF_RTDM_RATELIMIT_BURST);	\
									\
	if (rtdm_ratelimit(&_rs, __func__))				\
		printk(fmt, ##__VA_ARGS__);				\
})

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline void *rtdm_malloc(size_t size)
{
	return xnmalloc(size);
}

static inline void rtdm_free(void *ptr)
{
	xnfree(ptr);
}

int rtdm_mmap_to_user(struct rtdm_fd *fd,
		      void *src_addr, size_t len,
		      int prot, void **pptr,
		      struct vm_operations_struct *vm_ops,
		      void *vm_private_data);
int rtdm_iomap_to_user(struct rtdm_fd *fd,
		       phys_addr_t src_addr, size_t len,
		       int prot, void **pptr,
		       struct vm_operations_struct *vm_ops,
		       void *vm_private_data);
int rtdm_munmap(struct rtdm_fd *fd, void *ptr, size_t len);

static inline int rtdm_read_user_ok(struct rtdm_fd *fd,
				    const void __user *ptr, size_t size)
{
	return access_rok(ptr, size);
}

static inline int rtdm_rw_user_ok(struct rtdm_fd *fd,
				  const void __user *ptr, size_t size)
{
	return access_wok(ptr, size);
}

static inline int rtdm_copy_from_user(struct rtdm_fd *fd,
				      void *dst, const void __user *src,
				      size_t size)
{
	return __xn_copy_from_user(dst, src, size) ? -EFAULT : 0;
}

static inline int rtdm_safe_copy_from_user(struct rtdm_fd *fd,
					   void *dst, const void __user *src,
					   size_t size)
{
	return (!access_rok(src, size) ||
		__xn_copy_from_user(dst, src, size)) ? -EFAULT : 0;
}

static inline int rtdm_copy_to_user(struct rtdm_fd *fd,
				    void __user *dst, const void *src,
				    size_t size)
{
	return __xn_copy_to_user(dst, src, size) ? -EFAULT : 0;
}

static inline int rtdm_safe_copy_to_user(struct rtdm_fd *fd,
					 void __user *dst, const void *src,
					 size_t size)
{
	return (!access_wok(dst, size) ||
		__xn_copy_to_user(dst, src, size)) ? -EFAULT : 0;
}

static inline int rtdm_strncpy_from_user(struct rtdm_fd *fd,
					 char *dst,
					 const char __user *src, size_t count)
{
	if (unlikely(!access_rok(src, 1)))
		return -EFAULT;
	return __xn_strncpy_from_user(dst, src, count);
}

static inline int rtdm_rt_capable(struct rtdm_fd *fd)
{
	if (!XENO_ASSERT(RTDM, !xnsched_interrupt_p()))
		return 0;

	if (!rtdm_fd_is_user(fd))
		return !xnsched_root_p();

	return xnshadow_thread(current) != NULL;
}

static inline int rtdm_in_rt_context(void)
{
	return (ipipe_current_domain != ipipe_root_domain);
}

#endif /* !DOXYGEN_CPP */

#endif /* _COBALT_RTDM_DRIVER_H */
