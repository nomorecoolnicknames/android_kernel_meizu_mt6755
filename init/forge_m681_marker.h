/* SPDX-License-Identifier: GPL-2.0 */
/*
 * forge_m681_marker.h - persistent DRAM stage markers for M681/L681 (MT6755).
 *
 * Breadcrumb words in reserved DRAM that survives a WDT warm reset, so the
 * stage an early boot stopped at can be read back after the reset. The
 * regions, the slot layout and the readback commands are described in
 * init/forge_m681_marker.c.
 */

#ifndef _FORGE_M681_MARKER_H
#define _FORGE_M681_MARKER_H

#include <linux/types.h>

/*
 * Stage numbers shared by start_kernel() (init/main.c) and setup_arch()
 * (arch/arm64/kernel/setup.c).  Kept unique across both so the recovered
 * stage byte identifies the last reached point without ambiguity.
 *
 *   01..18  -> start_kernel() body (init/main.c)
 *   1E..1F  -> forge_m681_marker.c bookkeeping
 *   32..46  -> setup_arch() body (arch/arm64/kernel/setup.c)
 *   47..69  -> early time_init / clocksource / CCF split
 *   B0..B1  -> MTK power-management init bypass
 *   CA      -> single-core bring-up flag
 *   F0..F1  -> MTK cpuidle governor bypass
 */
#define FORGE_STAGE_START_KERNEL_ENTRY		0x01
#define FORGE_STAGE_POST_SETUP_ARCH		0x07
#define FORGE_STAGE_POST_MM_INIT		0x0C
#define FORGE_STAGE_POST_INIT_IRQ		0x0F
#define FORGE_STAGE_PRE_TIME_INIT		0x10
#define FORGE_STAGE_POST_TIME_INIT		0x11
#define FORGE_STAGE_POST_CONSOLE_INIT		0x12
#define FORGE_STAGE_ABOUT_TO_REST_INIT		0x18

#define FORGE_STAGE_MARKER_EARLY_INIT		0x1E	/* 30 */
#define FORGE_STAGE_MARKER_LATE_INIT		0x1F	/* 31 */

#define FORGE_STAGE_ARCH_SETUP_ENTRY		0x32	/* 50 A01 */
#define FORGE_STAGE_ARCH_CMDLINE_PARSED		0x33	/* 51 A02 */
#define FORGE_STAGE_ARCH_POST_MACHINE_FDT	0x34	/* 52 A03 */
#define FORGE_STAGE_ARCH_POST_EARLY_IOREMAP	0x35	/* 53 A04 */
#define FORGE_STAGE_ARCH_POST_PAGING_INIT	0x39	/* 57 A08 */
#define FORGE_STAGE_ARCH_POST_PSCI_INIT		0x3B	/* 59 A10 */
#define FORGE_STAGE_ARCH_SETUP_EXIT		0x3D	/* 61 A12 */
#define FORGE_STAGE_RAM_CONSOLE_INIT		0x46	/* 70 */

/* time_init() entry / of_clk_init() split. */
#define FORGE_STAGE_TIME_INIT_ENTRY		0x51	/* time_init() entered   */
#define FORGE_STAGE_TIME_POST_OF_CLK_INIT	0x52	/* of_clk_init returned  */
#define FORGE_STAGE_TIME_PRE_CLOCKSOURCE_OF_INIT	0x53
#define FORGE_STAGE_TIME_POST_CLOCKSOURCE_OF_INIT	0x54
#define FORGE_STAGE_TIME_INIT_EXIT		0x57

/* Fixed-clock-only of_clk_init; skip the full MTK CCF early. */
#define FORGE_STAGE_TIME_PRE_FIXED_CLK_INIT	0x67
#define FORGE_STAGE_TIME_POST_FIXED_CLK_INIT	0x68
#define FORGE_STAGE_TIME_SKIP_FULL_OF_CLK_INIT	0x69

/* APXGPT / mtk_timer clocksource bypass (use the ARM arch timer). */
#define FORGE_STAGE_MTK_TIMER_INIT_ENTRY	0x58
#define FORGE_STAGE_MTK_TIMER_BYPASS		0x59

/* MTK power/SPM/freqhopping arch-initcall bypass. */
#define FORGE_STAGE_MTK_PM_INIT_ENTRY		0xB0
#define FORGE_STAGE_MTK_PM_INIT_BYPASS		0xB1

/* Single-core bring-up flag (setup_max_cpus=1). */
#define FORGE_STAGE_KERNEL_FORCE_SINGLE_CPU	0xCA

/* Fine marks across kernel_init_freeable(). */
#define FORGE_STAGE_POST_SMP_PREPARE		0xCB	/* after smp_prepare_cpus */
#define FORGE_STAGE_POST_PRE_SMP_INITCALLS	0xCC	/* after do_pre_smp_initcalls+lockup */
#define FORGE_STAGE_POST_SMP_INIT		0xCD	/* after smp_init */
#define FORGE_STAGE_POST_SCHED_INIT_SMP		0xCE	/* after sched_init_smp */
#define FORGE_STAGE_POST_BASIC_SETUP		0xCF	/* after do_basic_setup (all initcalls) */
#define FORGE_STAGE_PRE_INIT_EXEC		0xD0	/* about to exec /init (userspace) */

/* Bypass of the MTK cpuidle governor / deep-idle framework. */
#define FORGE_STAGE_CPUIDLE_GOV_ENTRY		0xF0
#define FORGE_STAGE_CPUIDLE_GOV_BYPASS		0xF1

/* Initcall-level phase markers (0xE8 enter / 0xE9 done).
 * aux = level number 0-7 (early/core/postcore/arch/subsys/fs/device/late).
 * Per-level completion counts in diag region 0xB0 + level * 4. */
#define FORGE_STAGE_INITCALL_LEVEL_ENTER	0xE8
#define FORGE_STAGE_INITCALL_LEVEL_DONE		0xE9

/*
 * Pointer to the ioremap()ed marker region; NULL until
 * forge_m681_marker_early_init() has run.  Exposed so other early code in this
 * tree may share the mapping if it ever needs to without re-ioremapping.
 */
extern void __iomem *forge_spm_base;
extern void __iomem *forge_spm_base2;	/* second copy, preloader region at 0x44800000 */

/* QEMU virt guard: 1 when the root DT compatible is "linux,dummy-virt"
 * (qemu -M virt). When set, all MTK hardware access is skipped: the raw
 * SMC in forge_enable_cpuxgpt() (no EL3 on virt, undefined), the DRAM
 * marker writes and ioremaps (0x444f0000/0x44800000 are plain guest RAM
 * on virt) and the wdt/gpt/mcusys/spm ioremaps (0x1000xxxx is inside the
 * virt PCIe MMIO window). MT6755 DTs never match this compatible. */
extern int forge_virt;
/* Called from setup_arch() right after setup_machine_fdt() (the earliest point
 * where the flat DT is readable).  Sets forge_virt and NULLs the marker bases
 * so every later forge_m681_mark*() is a no-op under -M virt. */
void forge_m681_virt_detect(void);

void forge_m681_mark(u8 stage);
void forge_m681_mark_aux(u8 stage, u32 aux);
void forge_m681_bump(unsigned int slot);	/* eMMC checkpoint counters at 0xE4.. */
void forge_m681_marker_early_init(void);
void forge_m681_marker_late_init(void);
void forge_m681_wdt_kick(void);	/* pet the MTK WDT once per initcall */
void forge_m681_wdt_disarm(void); /* disable the HW WDT */
/* "Current initcall fn" tracker (diag 0x64..0x78) that the inner
 * platform-probe marks (0xC0/0xCD) do not overwrite, so a wedged initcall
 * is named even when it probes platform drivers. */
void forge_m681_set_initcall(u32 fn);
void forge_m681_set_initcall_done(u32 fn);
u32 forge_m681_get_initcall_seq(void);
/* Name every of_platform node reaching device creation (diag 0x80..0xac). */
void forge_m681_mark_ofnode(const char *name);

/* Initcall-level phase markers. do_initcall_level() calls
 * mark_level_enter() before the level loop and mark_level_done() after.
 * Stage 0xE8 = level entered (aux = level 0-7), 0xE9 = level complete.
 * Per-level counts are kept in diag 0xB0 + level * 4. Level names:
 * 0=early, 1=core, 2=postcore, 3=arch, 4=subsys, 5=fs, 6=device, 7=late.
 * Complements the per-initcall fn tracker (0x64/0x6c), which names the
 * exact initcall. */
void forge_m681_mark_level_enter(int level);
void forge_m681_mark_level_done(int level);
/* Direct SWRST_KEY write to toprgu + 0x14, independent of the panic
 * notifier chain. No-op if neither toprgu_base nor forge_wdt_base is
 * mapped. */
void forge_m681_wdt_swrst(void);

#endif /* _FORGE_M681_MARKER_H */
