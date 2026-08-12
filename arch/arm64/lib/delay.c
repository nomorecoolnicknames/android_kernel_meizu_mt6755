/*
 * Delay loops based on the OpenRISC implementation.
 *
 * Copyright (C) 2012 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timex.h>

void __delay(unsigned long cycles)
{
	cycles_t start = get_cycles();
#ifdef CONFIG_MACH_MT6755
	/*
	 * Guard against a frozen CNTVCT_EL0: LK hands over with cpuxgpt stopped,
	 * and the counter only runs once forge_enable_cpuxgpt() (SMC 0x82000201)
	 * has run. A udelay() before that, or if the SMC does not take, would
	 * never exit. On a live counter the real condition exits first; on a
	 * frozen one the cap gives a longer (never shorter) delay.
	 */
	unsigned long guard = (cycles << 8) + 0x10000UL;

	while ((get_cycles() - start) < cycles) {
		cpu_relax();
		if (!guard--)
			break;
	}
#else
	while ((get_cycles() - start) < cycles)
		cpu_relax();
#endif
}
EXPORT_SYMBOL(__delay);

inline void __const_udelay(unsigned long xloops)
{
	unsigned long loops;

	loops = xloops * loops_per_jiffy * HZ;
	__delay(loops >> 32);
}
EXPORT_SYMBOL(__const_udelay);

void __udelay(unsigned long usecs)
{
	__const_udelay(usecs * 0x10C7UL); /* 2**32 / 1000000 (rounded up) */
}
EXPORT_SYMBOL(__udelay);

void __ndelay(unsigned long nsecs)
{
	__const_udelay(nsecs * 0x5UL); /* 2**32 / 1000000000 (rounded up) */
}
EXPORT_SYMBOL(__ndelay);
