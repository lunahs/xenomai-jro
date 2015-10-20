/*
 * Copyright (C) 2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_ARM64_ASM_UAPI_FPTEST_H
#define _COBALT_ARM64_ASM_UAPI_FPTEST_H

#define __COBALT_HAVE_FPU  0x1

static inline void fp_regs_set(int features, unsigned int val)
{

	unsigned long long e[32];
	unsigned int i;

	if (features & __COBALT_HAVE_FPU) {

		for (i = 0; i < 32; i++)
			e[i] = val;

		__asm__ __volatile__("ldp     d0, d1, [%0, #8 * 0] \n\
							ldp   d2, d3, [%0, #8 * 2] \n\
							ldp     d4, d5, [%0, #8 * 4]\n\
							ldp     d6, d7, [%0, #8 * 6]\n\
							ldp     d8, d9, [%0, #8 * 8]\n\
							ldp     d10, d11, [%0, #8 * 10]\n\
							ldp     d12, d13, [%0, #8 * 12]\n\
							ldp     d14, d15, [%0, #8 * 14]\n\
							ldp     d16, d17, [%0, #8 * 16]\n\
							ldp     d18, d19, [%0, #8 * 18]\n\
							ldp     d20, d21, [%0, #8 * 20]\n\
							ldp     d22, d23, [%0, #8 * 22]\n\
							ldp     d24, d25, [%0, #8 * 24]\n\
							ldp     d26, d27, [%0, #8 * 26]\n\
							ldp     d28, d29, [%0, #8 * 28]\n\
							ldp     d30, d31, [%0, #8 * 30]"

			: /* No outputs. */
			: "r"(&e[0])
			: "memory"
		);
	}

}

static inline unsigned int fp_regs_check(int features, unsigned int val,
					 int (*report)(const char *fmt, ...))
{
	unsigned int result = val;

	unsigned int i;
	unsigned long long e[32];

	if (features & __COBALT_HAVE_FPU) {

		__asm__ __volatile__("stp     d0, d1, [%0, #8 * 0] \n\
							stp   d2, d3, [%0, #8 * 2] \n\
							stp     d4, d5, [%0, #8 * 4]\n\
							stp     d6, d7, [%0, #8 * 6]\n\
							stp		d8, d9, [%0, #8 * 8]\n\
							stp     d10, d11, [%0, #8 * 10]\n\
							stp     d12, d13, [%0, #8 * 12]\n\
							stp     d14, d15, [%0, #8 * 14]\n\
							stp     d16, d17, [%0, #8 * 16]\n\
							stp     d18, d19, [%0, #8 * 18]\n\
							stp     d20, d21, [%0, #8 * 20]\n\
							stp     d22, d23, [%0, #8 * 22]\n\
							stp     d24, d25, [%0, #8 * 24]\n\
							stp     d26, d27, [%0, #8 * 26]\n\
							stp     d28, d29, [%0, #8 * 28]\n\
							stp     d30, d31, [%0, #8 * 30]"

			:  /* No outputs. */
			: "r"(&e[0])
			: "memory"
		);



		for (i = 0; i < 32; i++)
			if (e[i] != val) {
				report("d%d: %llu != %u\n", i, e[i], val);
				result = e[i];
			}
	}

	return result;
}

#endif /* !_COBALT_ARM64_ASM_UAPI_FPTEST_H */
