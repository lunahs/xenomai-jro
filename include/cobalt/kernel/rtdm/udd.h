/**
 * @file
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or
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
#ifndef _COBALT_RTDM_UDD_H
#define _COBALT_RTDM_UDD_H

#include <linux/list.h>
#include <rtdm/driver.h>
#include <rtdm/uapi/udd.h>

/**
 * @ingroup rtdm_profiles
 * @defgroup rtdm_udd User-space driver core
 *
 * This profile includes all mini-drivers sitting on top of the
 * User-space Device Driver framework (UDD). The generic UDD core
 * driver enables interrupt control and I/O memory access interfaces
 * to user-space device drivers, as defined by the mini-drivers when
 * registering.
 *
 * A mini-driver supplements the UDD core with ancillary functions for
 * dealing with @ref udd_memory_region "memory mappings" and @ref
 * udd_irq_handler "interrupt control" for a particular I/O
 * card/device.
 *
 * UDD-compliant mini-drivers only have to provide the basic support
 * for dealing with the interrupt sources present in the device, so
 * that most part of the device requests can be handled from a Xenomai
 * application running in user-space.
 *
 * This profile is reminiscent of the UIO framework available with the
 * Linux kernel, adapted to the dual kernel Cobalt environment.
 *
 * @{
 */

/**
 * @anchor udd_irq_special
 * Special IRQ values for udd_device.irq
 *
 * @{
 */
/**
 * No IRQ managed. Passing this code implicitly disables all
 * interrupt-related services, including control (disable/enable) and
 * notification.
 */
#define UDD_IRQ_NONE     0
/**
 * IRQ directly managed from the mini-driver on top of the UDD
 * core. The mini-driver is in charge of notifying the Cobalt threads
 * waiting for IRQ events by calling the udd_notify_event() service.
 */
#define UDD_IRQ_CUSTOM   (-1)
/** @} */

/**
 * @anchor udd_memory_types  @name Memory types for mapping
 * Types of memory for mapping
 *
 * The UDD core implements a default ->mmap() handler which first
 * attempts to hand over the request to the corresponding handler
 * defined by the mini-driver. If not present, the UDD core
 * establishes the mapping automatically, depending on the memory
 * type defined for the region.
 *
 * @{
 */
/**
 * No memory region. Use this type code to disable an entry in the
 * array of memory mappings, i.e. udd_device.mem_regions[].
 */
#define UDD_MEM_NONE     0
/**
 * Physical I/O memory region. By default, the UDD core maps such
 * memory to a virtual user range by calling the rtdm_mmap_iomem()
 * service.
 */
#define UDD_MEM_PHYS     1
/**
 * Kernel logical memory region (e.g. kmalloc()). By default, the UDD
 * core maps such memory to a virtual user range by calling the
 * rtdm_mmap_kem() service. */
#define UDD_MEM_LOGICAL  2
/**
 * Virtual memory region with no direct physical mapping
 * (e.g. vmalloc()). By default, the UDD core maps such memory to a
 * virtual user range by calling the rtdm_mmap_vmem() service.
 */
#define UDD_MEM_VIRTUAL  3
/** @} */

#define UDD_NR_MAPS  5

/**
 * @anchor udd_memory_region
 * UDD memory region descriptor.
 *
 * This descriptor defines the characteristics of a memory region
 * declared to the UDD core by the mini-driver. All valid regions
 * should be declared in the udd_device.mem_regions[] array,
 * invalid/unassigned ones should bear the UDD_MEM_NONE type.
 *
 * The UDD core exposes each region via the mmap(2) interface to the
 * application. To this end, a companion mapper device is created
 * automatically when registering the mini-driver.
 *
 * The mapper device creates special files in the RTDM namespace to
 * reach the individual regions, which the application can open, for
 * mapping the corresponding region to their address space via the
 * mmap(2) system call.
 *
 * For instance, declaring a region of physical memory at index #2 of
 * the memory region array as follows:
 *
 * @code
 * static struct udd_device udd;
 *
 * static int foocard_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
 * {
 *      udd.device_name = "foocard";
 *      ...
 *      udd.mem_regions[2].name = "ADC";
 *      udd.mem_regions[2].addr = pci_resource_start(dev, 1);
 *      udd.mem_regions[2].len = pci_resource_len(dev, 1);
 *      udd.mem_regions[2].type = UDD_MEM_PHYS;
 *      ...
 *      return udd_register_device(&udd);
 * }
 * @endcode
 *
 * will make such region accessible via the mapper device using the
 * following sequence of code, via the default ->mmap() handler from
 * the UDD core:
 *
 * @code
 * int fd, fdm;
 * void *p;
 *
 * fd = open("/dev/foocard", O_RDWR);
 * fdm = open("/dev/foocard,mapper@2", O_RDWR);
 * p = mmap(NULL, 4096, PROT_READ|PROT_WRITE, 0, fdm, 0);
 * @endcode
 *
 * @note No mapper device is created unless a valid region has been
 * declared in the udd_device.mem_regions[] array.
 */
struct udd_memregion {
	/** Name of the region (informational but required) */
	const char *name;
	/**
	 * Start address of the region. This may be a physical or
	 * virtual address, depending on the @ref udd_memory_types
	 * "memory type".
	 */
	unsigned long addr;
	/**
	 * Length (in bytes) of the region. This value must be
	 * PAGE_SIZE aligned.
	 */
	size_t len;
	/**
	 * Type of the region. See the discussion about @ref
	 * udd_memory_types "UDD memory types" for possible values.
	 */
	int type;
};

/**
 * @anchor udd_device
 * UDD device descriptor.
 *
 * This descriptor defines the characteristics of a UDD-based
 * mini-driver when registering via a call to udd_register_device().
 */
struct udd_device {
	/**
	 * Name of the device managed by the mini-driver, appears
	 * automatically in the /dev namespace upon creation.
	 */
	const char *device_name;
	/**
	 * Additional device flags (e.g. RTDM_EXCLUSIVE)
	 * RTDM_NAMED_DEVICE may be omitted).
	 */
	int device_flags;
	/**
	 * Textual description of the device managed by the
	 * mini-driver.
	 */
	const char *device_description;
	/**
	 * Subclass code of the device managed by the mini-driver (see
	 * RTDM_SUBCLASS_xxx definition in the @ref rtdm_profiles
	 * "Device Profiles"). The main class code is forced to
	 * RTDM_CLASS_UDD.
	 */
	int device_subclass;
	/** @ref drv_versioning "Driver version. */
	int driver_version;
	/** Driver author/provider (exposed via /proc/xenomai/rtdm) */
	const char *driver_author;
	struct {
		/**
		 * Ancillary open() handler, optional. See
		 * rtdm_open_handler().
		 */
		int (*open)(struct rtdm_fd *fd, int oflags);
		/**
		 * Ancillary close() handler, optional. See
		 * rtdm_close_handler().
		 */
		void (*close)(struct rtdm_fd *fd);
		/**
		 * Ancillary ioctl() handler, optional. See
		 * rtdm_ioctl_handler().
		 */
		int (*ioctl)(struct rtdm_fd *fd,
			     unsigned int request, void *arg);
		/**
		 * Ancillary mmap() handler for the mapper device,
		 * optional. See rtdm_mmap_handler(). The mapper
		 * device operates on a valid region defined in the @a
		 * mem_regions[] array. A pointer to the region 
		 * can be obtained by a call to udd_get_region().
		 *
		 * If this handler is NULL, the UDD core establishes
		 * the mapping automatically, depending on the memory
		 * type defined for the region.
		 */
		int (*mmap)(struct rtdm_fd *fd,
			    struct vm_area_struct *vma);
		/**
		 * @anchor udd_irq_handler
		 *
		 * Ancillary handler for receiving interrupts. This
		 * handler must be provided if the mini-driver hands
		 * over IRQ handling to the UDD core, by setting the
		 * @a irq field to a valid value, different from
		 * UDD_IRQ_CUSTOM and UDD_IRQ_NONE.
		 *
		 * The ->interrupt() handler shall return one of the
		 * following status codes:
		 *
		 * - RTDM_IRQ_HANDLED, if the mini-driver successfully
		 * handled the IRQ. This flag can be combined with
		 * RTDM_IRQ_DISABLE to prevent the Cobalt kernel from
		 * re-enabling the interrupt line upon return,
		 * otherwise it is re-enabled automatically.
		 *
		 * - RTDM_IRQ_NONE, if the interrupt does not match
		 * any IRQ the mini-driver can handle.
		 *
		 * Once the ->interrupt() handler has returned, the
		 * UDD core notifies user-space Cobalt threads waiting
		 * for IRQ events (if any).
		 */
		int (*interrupt)(struct udd_device *udd);
	} ops;
	/**
	 * IRQ number. If valid, the UDD core manages the
	 * corresponding interrupt line, installing a base handler.
	 * Otherwise, a special value can be passed for declaring
	 * @ref udd_irq_special "unmanaged IRQs".
	 */
	int irq;
	/**
	 * Array of memory regions defined by the device. The array
	 * can be sparse, with some entries bearing the UDD_MEM_NONE
	 * type interleaved with valid ones.  See the discussion about
	 * @ref udd_memory_region "UDD memory regions".
	 */
	struct udd_memregion mem_regions[UDD_NR_MAPS];
	/** Reserved to the UDD core. */
	struct udd_reserved {
		rtdm_irq_t irqh;
		atomic_t event;
		struct udd_signotify signfy;
		struct rtdm_event pulse;
		struct rtdm_device_class class;
		struct rtdm_device device;
		struct rtdm_device_class mapper_class;
		struct rtdm_device mapper;
		char *mapper_name;
		int nr_maps;
	} __reserved;
};

int udd_register_device(struct udd_device *udd);

int udd_unregister_device(struct udd_device *udd,
			  unsigned int poll_delay);

struct udd_device *udd_get_device(struct rtdm_fd *fd);

void udd_notify_event(struct udd_device *udd);

void udd_post_irq_enable(int irq);

void udd_post_irq_disable(int irq);

/** @} */

#endif /* !_COBALT_RTDM_UDD_H */
