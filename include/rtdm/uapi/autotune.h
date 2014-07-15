/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _RTDM_UAPI_AUTOTUNE_H
#define _RTDM_UAPI_AUTOTUNE_H

#define RTDM_CLASS_AUTOTUNE		RTDM_CLASS_TESTING
#define RTDM_SUBCLASS_AUTOTUNE		0

#define AUTOTUNE_RTIOC_LATENCY		_IOW(RTDM_CLASS_AUTOTUNE, 0, int)

#endif /* !_RTDM_UAPI_AUTOTUNE_H */
