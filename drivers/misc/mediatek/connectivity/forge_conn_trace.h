/* SPDX-License-Identifier: GPL-2.0 */
/*
 * forge_conn_trace.h - connectivity (consys) power-on tracer.
 *
 * printk to the console can be deferred under console_sem, and a panic
 * corrupts the log ring, so these breadcrumbs are plain writel()s into the
 * marker page at 0x44800000 (init/forge_m681_marker.c), which survives a
 * WDT warm reset, followed by a printk mirror.
 *
 * Layout (marker page offsets; 0x50/0x54 belong to forge_m681_fault_snap()
 * and 0x58-0x60 are written on every IRQ by forge_m681_irq_trace()):
 *
 *   diag 0x70 = 0xSSnnnnnn (SS = stage byte, nnnnnn = step code)
 *   diag 0x74 = ~(diag 0x70)                (validity guard)
 *   diag 0x94 = second copy of 0x70         (bit-rot superset decode x2)
 *   diag 0x98 = ~(diag 0x94)
 *   plus the standard rolling marker slots (stage 0xD7/0xD8, aux = step)
 *   plus a pr_err mirror -> dmesg ring -> pstore console.
 *
 * After a hard power cycle the DRAM page does not survive, so every step is
 * also mirrored to pr_err; for watched runs stream it off the device:
 *   adb shell "dmesg -w" | grep --line-buffered FORGE_CONN
 *
 * Read after a reset, from recovery:
 *   dd if=/dev/mem bs=4 skip=$((0x44800070/4)) count=2 2>/dev/null | xxd
 *   dd if=/dev/mem bs=4 skip=$((0x44800094/4)) count=2 2>/dev/null | xxd
 *   dd if=/dev/mem bs=4 skip=$((0x44800104/4)) count=3 2>/dev/null | xxd
 * Validity: the second word of each pair must be the bitwise inverse of the
 * first (clear-only bit-rot may garble both after a panic or power cycle -
 * superset-match each pair, then intersect the two candidate sets). The
 * last tracer word brackets the failure: it happened between that step and
 * the next one.
 *
 * Stage bytes:
 *   0xD7 = wmt_detect / DO_MODULE_INIT path
 *   0xD8 = consys power-on (func-on / reg_ctrl) path
 *
 * Process context only (ioctl / worker / sysfs writer): the pr_err mirror
 * makes this unsafe in interrupt context.
 */
#ifndef _FORGE_CONN_TRACE_H
#define _FORGE_CONN_TRACE_H

#include <linux/printk.h>

#define FORGE_CONN_TRACE_DETECT	0xD7	/* module-init / detect path */
#define FORGE_CONN_TRACE_PWRON	0xD8	/* power-on ladder path */

/* init/forge_m681_marker.c; both exported, no-ops before the marker page is
 * mapped and under qemu-virt (forge_virt guard nulls the bases). */
extern void forge_m681_diag(unsigned int off, unsigned int val);
extern void forge_m681_mark_aux(unsigned char stage, unsigned int aux);

static inline void forge_conn_trace(unsigned char stage, unsigned int step)
{
	unsigned int code = ((unsigned int)stage << 24) | (step & 0xFFFFFFu);

	/* DRAM copies FIRST (pure writel, immune to printk deferral), the
	 * printk mirror LAST - if pr_err wedges or defers, the breadcrumb is
	 * already committed to both DRAM pairs. */
	forge_m681_diag(0x70u, code);
	forge_m681_diag(0x74u, ~code);
	forge_m681_diag(0x94u, code);
	forge_m681_diag(0x98u, ~code);
	forge_m681_mark_aux(stage, step);
	pr_err("[FORGE_CONN_TRACE] %02X:%06X\n", stage, step & 0xFFFFFFu);
}

#endif /* _FORGE_CONN_TRACE_H */
