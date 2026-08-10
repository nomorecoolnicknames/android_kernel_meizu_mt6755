/* SPDX-License-Identifier: GPL-2.0 */
/*
 * forge_conn_trace.h - consys module-init / power-on breadcrumbs.
 *
 * Writes:
 *   - the rolling marker slot (forge_m681_mark_aux: stage byte + step), and
 *   - a pr_err mirror. The 0x5f000000 ram console survives a WDT reset into
 *     recovery, so the dmesg mirror is the primary record.
 * Stage bytes (the bootguard owns 0xDB-0xDF):
 *   0xD7 = wmt_detect / DO_MODULE_INIT path
 *   0xD8 = consys power-on ladder (func-on / reg_ctrl)
 * Process context only (ioctl / worker / sysfs writer), never from an ISR.
 */
#ifndef _FORGE_CONN_TRACE_H
#define _FORGE_CONN_TRACE_H

#include <linux/printk.h>

#define FORGE_CONN_TRACE_DETECT	0xD7	/* module-init / detect path */
#define FORGE_CONN_TRACE_PWRON	0xD8	/* power-on ladder path */

/* init/forge_m681_marker.c; no-op before the marker page is mapped */
extern void forge_m681_mark_aux(u8 stage, u32 aux);

static inline void forge_conn_trace(unsigned char stage, unsigned int step)
{
	forge_m681_mark_aux(stage, step);
	pr_err("[FORGE_CONN_TRACE] %02X:%06X\n", stage, step & 0xFFFFFFu);
}

#endif /* _FORGE_CONN_TRACE_H */
