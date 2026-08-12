// SPDX-License-Identifier: GPL-2.0
/*
 * forge_m681_marker.c - persistent DRAM stage markers for MT6755.
 *
 * On MT6755 the kernel can die in early boot before the MTK ram_console
 * driver registers a console. Anything emitted with pr_emerg() in
 * start_kernel() before that lives only in the early printk ring buffer,
 * which is wiped by the WDT reset, so a few breadcrumb bytes are kept in
 * DRAM that survives a WDT reset.
 *
 * Multiple guarded slots are written so a reader can cross-check
 * signature/raw/inverted guards and ignore any stale value. Each marker
 * also emits pr_emerg("[FORGE_M681] stage 0xNN"), so if the kernel
 * reaches printk + ram_console the stage is in the normal log that MTK
 * kedump flushes to the expdb partition.
 *
 * forge_m681_marker_early_init() must run after early_ioremap_init() in
 * setup_arch. Before that forge_m681_mark() is a silent no-op. After
 * mm_init() the marker switches to a permanent ioremap() so the
 * late_initcall "early ioremap leak" check stays quiet.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/moduleparam.h>	/* forge_poweroff_veto knob */
#include <linux/notifier.h>	/* atomic_notifier_chain_register + panic_notifier_list */
#include <linux/notifier.h>	/* (kept; kernel.h pulls it but be explicit for the reader) */
#include <asm/early_ioremap.h>
#include <linux/of_fdt.h>	/* of_flat_dt_is_compatible for the forge_virt guard */
#include <linux/console.h>	/* console -> DRAM log */
#include <linux/string.h>
#include <linux/hrtimer.h>	/* irq-context WDT kicker */
#include <linux/ktime.h>	/* ns_to_ktime / NSEC_PER_SEC */

#include <linux/memblock.h>	/* head log: reserve + linear-map check */
#include <linux/workqueue.h>	/* boot deadline */
#include <linux/blkdev.h>	/* boot deadline: BCB write to "para" */
#include <linux/genhd.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/reboot.h>
#include <linux/delay.h>	/* bootguard: msleep */
#include <linux/math64.h>	/* BCB: div_u64 / do_div */
#include <asm/cacheflush.h>	/* head log: __flush_dcache_area */

#include "forge_m681_marker.h"

/*
 * DRAM console: routes every kernel printk (including userspace init's
 * /dev/kmsg writes) into a ring buffer in reserved DRAM that survives a
 * WDT warm reset and the following recovery boot (the normal ram_console
 * is overwritten by recovery). The ring lives inside the marker page
 * (forge_spm_base2 @0x44800000, mapped via the marker's ioremap; a fresh
 * ioremap of 0x44801000 is rejected as RAM). The marker slots end at
 * 0x200, so 0x200..0xFFF (~3.5 KB) is free. Read at phys 0x44800200.
 * Layout: u32 head, u32 'FLOG', then the text ring.
 */
#define FORGE_LOG_OFF	0x200U				/* within the marker page */
#define FORGE_LOG_END	0x1000U				/* one page: a larger ioremap of the reserved RAM region aliases the linear map with mismatched attributes */
#define FORGE_LOG_HDR	8U				/* head + magic */
#define FORGE_LOG_RING	(FORGE_LOG_END - FORGE_LOG_OFF - FORGE_LOG_HDR)
#define FORGE_LOG_MAGIC	0x464C4F47U			/* 'FLOG' */
static u32 forge_log_head;

static void forge_console_write(struct console *con, const char *s,
				unsigned int count)
{
	void __iomem *b = forge_spm_base2;
	unsigned int i;

	if (!b)
		return;
	for (i = 0; i < count; i++) {
		writeb(s[i], b + FORGE_LOG_OFF + FORGE_LOG_HDR +
		       (forge_log_head % FORGE_LOG_RING));
		forge_log_head++;
	}
	writel(forge_log_head, b + FORGE_LOG_OFF + 0);
	writel(FORGE_LOG_MAGIC, b + FORGE_LOG_OFF + 4);
}

static struct console forge_console = {
	.name	= "forgelog",
	.write	= forge_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

/* Primary readout at 0x444f0000 (the minirdump reserved node, size
 * 0x10000). 0x46100000 lies inside lk-reserved (0x46000000+0x400000), so
 * the warm WDT->recovery path re-runs LK and overwrites it.
 * minirdump@0x444f0000 is reserved by both this kernel and recovery and is
 * written only by the minirdump driver on an actual crash dump, so a mark
 * written here survives the LK re-run and recovery. Read from recovery:
 *   dd if=/dev/mem bs=512 skip=2238336 (=0x444f0000/512). */
#define FORGE_MARKER_PHYS_BASE	0x444f0000UL
#define FORGE_MARKER_REGION_SIZE	0x1000UL	/* one page for the header */

/* Secondary readout at 0x44800000 (the preloader reserved node, size
 * 0x100000). The pstore region (0x44410000) is overwritten by the pstore
 * driver during of_platform_populate; the preloader region is written only
 * by the preloader and reserved by both kernels, so it survives the probe
 * phase and the recovery boot. Read from recovery:
 *   dd if=/dev/mem bs=512 skip=2244608 (=0x44800000/512). */
#define FORGE_MARKER2_PHYS_BASE	0x44800000UL
#define FORGE_MARKER2_MAP_SIZE	0x8000UL	/* late ioremap maps 32KB for the console (within the 1MB preloader-reserved region) */

/*
 * MTK toprgu watchdog (0x10007000) kick channel.
 *
 * The WDT is armed at postcore_initcall (mtk_wdt_probe, level 2) with a
 * 30s timeout and kicked once, but the kicker kthread only starts at
 * late_initcall (level 7). Every level 4/5/6 initcall runs with no kick,
 * so the HW dog can fire in the middle of a progressing boot.
 *
 * forge_m681_wdt_kick() is called once per initcall from do_one_initcall()
 * to reload the counter, letting a progressing boot reach /init while a
 * wedged initcall (never returns -> never kicks) still trips the dog after
 * ~30s, so the device warm-resets and the marker localises it.
 */
#define FORGE_WDT_PHYS_BASE	0x10007000UL
#define FORGE_WDT_REGION_SIZE	0x100UL
#define FORGE_WDT_RESTART_OFF	0x08	/* MTK_WDT_RESTART */
#define FORGE_WDT_RESTART_KEY	0x1971U	/* MTK_WDT_RESTART_KEY */
#define FORGE_WDT_MODE_OFF	0x00	/* MTK_WDT_MODE */
#define FORGE_WDT_LENGTH_OFF	0x04	/* MTK_WDT_LENGTH */
#define FORGE_WDT_MODE_KEY	0x22000000U
#define FORGE_WDT_MODE_ENABLE	0x00000001U
#define FORGE_WDT_MODE_EXTEN	0x00000004U
#define FORGE_WDT_MODE_IRQ	0x00000008U
#define FORGE_WDT_MODE_DUAL_MODE 0x00000040U
#define FORGE_WDT_MODE_AUTO_RESTART 0x00000010U
#define FORGE_WDT_LENGTH_KEY	0x00000008U
#define FORGE_WDT_TIMEOUT_SEC	30U

/* manual SWRST backstop, see forge_m681_wdt_kick. */
#define FORGE_WDT_SWRST_OFF	0x14U
#define FORGE_WDT_SWRST_KEY	0x1209U

/* RGU reset forensics and request-line quiesce.
 * WDT_STA latches which source caused the last warm reset; it is the only
 * record that tells a counter-timeout reset (a hang, kickers starved) from
 * a request-line reset (thermal-direct / SPM / EINT / debug, all silent).
 * WDT_REQ_MODE says which request lines may reset the SoC now, including
 * whatever the preloader/LK left enabled at handoff. Bit values match
 * drivers/watchdog/mediatek/wdt/mt6755/mt_wdt.h. */
#define FORGE_WDT_STA_OFF	0x0CU
#define FORGE_WDT_NONRST2_OFF	0x24U
#define FORGE_WDT_REQ_MODE_OFF	0x30U
#define FORGE_WDT_REQ_MODE_KEY	0x33000000U
/* Every RGU reset-request line that is not the petted counter dog itself:
 * SPM_THERMAL(b0) SPM_SCPSYS(b1) EINT(b2) SYSRST(b3) THERMAL_DIRECT(b18)
 * DEBUG(b19). None of these requesters is calibrated here (tscpu is
 * uncalibrated, the SPM firmware is not tuned for this board, the EINT
 * debug key is unused), so a set bit here only causes silent reboots. */
#define FORGE_WDT_REQ_LINES	(0x00000001U | 0x00000002U | 0x00000004U | \
				 0x00000008U | 0x00040000U | 0x00080000U)
/* There are ~600 module initcalls; the threshold is well above that so
 * normal initcall flow never trips it, but a pathological loop that keeps
 * calling do_one_initcall (recursive initcall loop, broken module_init
 * chain, kthread that re-enters do_one_initcall) cannot keep the WDT
 * petted forever. */
#define FORGE_WDT_FORCE_SWRESET_THRESHOLD 4000U

/* Offsets inside the first 512 bytes (never zeroed by ram_console memset_io,
 * which starts at off_linux >= 512). */
#define FORGE_MARKER_LEGACY_OFF	0x100
#define FORGE_MARKER_SHADOW_OFF	0x180
#define FORGE_SLOT_ROLLING_OFF	0x00	/* 0xF6810000 | stage */
#define FORGE_SLOT_STAGE_OFF	0x04	/* raw stage */
#define FORGE_SLOT_SIG_OFF	0x08	/* magic signature 'FRGE' */
#define FORGE_SLOT_AUX_OFF	0x0C	/* auxiliary u32 payload */
#define FORGE_SLOT_SIG_INV_OFF	0x10	/* inverted signature guard */
#define FORGE_SLOT_AUX_INV_OFF	0x14	/* inverted auxiliary guard */
#define FORGE_SLOT_STAGE_INV_OFF	0x18	/* inverted raw stage guard */
#define FORGE_SLOT_ROLLING_INV_OFF	0x1C	/* inverted rolling guard */

#define FORGE_MARKER_MAGIC	0xF6810000U	/* 'F6810000' | stage */
#define FORGE_SIGNATURE		0x46524745U	/* ASCII 'FRGE' */

/*
 * forge_spm_base is exported so the marker engine is reachable from the inline
 * call sites in init/main.c and arch/arm64/kernel/setup.c.  It is updated twice
 * during boot: first by the early_ioremap path, then by the permanent ioremap
 * path.  Until the first update it is NULL and forge_m681_mark() is a no-op.
 */
void __iomem *forge_spm_base;
EXPORT_SYMBOL(forge_spm_base);

/* Secondary marker base; readable from recovery. */
void __iomem *forge_spm_base2;
EXPORT_SYMBOL(forge_spm_base2);

/*
 * QEMU virt guard (forge_virt).
 *
 * Under `-M virt` every MTK physical address the marker code touches is
 * wrong: 0x444f0000/0x44800000 are unreserved guest RAM (writing there
 * corrupts pages the allocator will hand out), 0x10006000/0x10007000/
 * 0x10008000/0x10200000 sit inside virt's PCIe MMIO window, and the raw
 * `smc #0` in forge_enable_cpuxgpt() is undefined at EL1 because default
 * virt has no EL3.
 *
 * forge_m681_virt_detect() is called from setup_arch() immediately after
 * setup_machine_fdt(), the earliest point where the flat DT is readable.
 * If the root compatible is "linux,dummy-virt", set forge_virt=1 and NULL
 * both marker bases (the two early fixmap slots are abandoned exactly like
 * early_ioremap_reset() abandons them on the device; the only writes that
 * happened before detection, stages 0x1E and 0x35, landed in RAM that is
 * still outside memblock at that instant). Every later forge_m681_mark*()
 * no-ops on the NULL bases and forge_m681_marker_late_init() returns before
 * any MTK ioremap or the cpuxgpt SMC. MT6755 DTs never carry the
 * "linux,dummy-virt" compatible.
 */
int forge_virt;
EXPORT_SYMBOL(forge_virt);

/* Userspace power-off veto, consumed by sys_reboot (kernel/reboot.c).
 * 1 = reboot(2) POWER_OFF/HALT are logged and refused with -EPERM;
 * RESTART/RESTART2 are never vetoed. 0 = stock behaviour. Kernel-internal
 * charger shutdown sites have their own gate
 * (charging_hw_sm5414.forge_chg_poweroff). Runtime:
 * /sys/module/forge_m681_marker/parameters/forge_poweroff_veto. */
int forge_poweroff_veto = 1;
module_param(forge_poweroff_veto, int, 0644);

void __init forge_m681_virt_detect(void)
{
	if (!initial_boot_params)
		return;
	if (!of_flat_dt_is_compatible(of_get_flat_dt_root(),
				      "linux,dummy-virt"))
		return;

	forge_virt = 1;
	forge_spm_base = NULL;
	forge_spm_base2 = NULL;
	pr_info("[FORGE_M681] forge_virt: qemu -M virt detected -> MTK marker/WDT/GPT/cpuxgpt/SPM hw init DISABLED\n");
}

/* ioremap of the MTK toprgu watchdog block (see forge_m681_wdt_kick). */
static void __iomem *forge_wdt_base;

/* RGU state latched at boot, before forge_m681_wdt_arm writes MODE (a
 * MODE write may clear STA). Re-printed by the late quiesce so the values
 * survive expdb ring rollover. */
static u32 forge_rgu_boot_sta;
static u32 forge_rgu_boot_reqmode;

/* GPT2 free-run counter (phys 0x10008000, counter at +0x28) for the
 * "does 0x10008028 count?" probe in forge_m681_wdt_kick (runs every
 * initcall, so before any frozen-timer hang). */
static void __iomem *forge_gpt_base;

/* topckgen + apmixedsys mappings for the GPU clock-state snapshot.
 * Mapped in forge_m681_marker_late_init (start_kernel era) so even the
 * kbase-probe-time power-on is observable; the keywatch thread keeps a
 * lazy-map fallback. */
static void __iomem *forge_topck_base;
static void __iomem *forge_apmix_base;

/* GPU clock snapshot: whether MMPLL_CON0.EN is set while kbase_jd_submit
 * runs (the 5s keywatch cannot see the sub-second first-job window).
 * Callable from any context (three always-on readls + pr_emerg), invoked
 * from the first-N marker blocks at submit/MFG-enable time. */
void forge_m681_gpuclk_snap(const char *tag)
{
	u32 c1 = 0xDEAD, p0 = 0xDEAD, p1 = 0xDEAD;

	if (forge_topck_base)
		c1 = readl(forge_topck_base + 0x50);
	if (forge_apmix_base) {
		p0 = readl(forge_apmix_base + 0x240);
		p1 = readl(forge_apmix_base + 0x244);
	}
	pr_emerg("[FORGE_M681] #124 gpuclk@%s: CLK_CFG_1=0x%08x (mfg_sel=%u) MMPLL_CON0=0x%08x (EN=%u) CON1=0x%08x\n",
		 tag, c1, (c1 >> 24) & 0x3, p0, p0 & 0x1, p1);
}
EXPORT_SYMBOL(forge_m681_gpuclk_snap);

/*
 * The driver's own DT-mapped (of_iomap) toprgu base, set by mtk_wdt_probe()
 * at postcore_initcall (level 2). The private start_kernel ioremap() of
 * 0x10007000 (forge_wdt_base) may not be usable that early for SoC
 * register space, so prefer toprgu_base once it is live.
 */
extern void __iomem *toprgu_base;

/* per-initcall kick counter, mirrored into the marker region at offset
 * 0x40 (count) and 0x44 (which bases were non-NULL), so a post-reset
 * readback shows whether the kick ran and how far the boot progressed. */
static u32 forge_wdt_kick_count;

/* Diagnostic offsets in the marker page (free; real slots start at 0x100). */
#define FORGE_DIAG_KICKCNT_OFF	0x40
#define FORGE_DIAG_KICKFLAGS_OFF	0x44
/* initcall-level phase tracking diag offsets. */
#define FORGE_DIAG_LEVEL_ENTER_BASE	0xB0
#define FORGE_DIAG_LEVEL_DONE_BASE	0xC0
#define FORGE_DIAG_LASTGOOD_SEQ_OFF	0xD0
#define FORGE_DIAG_LASTGOOD_SEQ_INV_OFF	0xD4
#define FORGE_DIAG_BOOTPHASE_OFF	0xD8
#define FORGE_DIAG_WDTK_HEARTBEAT_OFF	0xFC	/* kicker heartbeat; 0xDC would collide with LEVEL_DONE_BASE(0xC0)+7*4 */
#define FORGE_DIAG_WDTK_INIT_OFF	0xE0	/* kicker initcall ran (A11E armed) */
#define FORGE_DIAG_EMMC_CTR_BASE	0xE4	/* eMMC checkpoint counters slot[0..5]=C7,C8,C9,CA,C3,C5 (0xE4..0xF8); free zone 0xE4..0xFC, before marker slots @0x100 */

static const unsigned long forge_m681_slot_offsets[] = {
	FORGE_MARKER_LEGACY_OFF,
	0x120, 0x140, 0x160, FORGE_MARKER_SHADOW_OFF,
	0x1a0, 0x1c0, 0x1e0,
};

static void forge_m681_write_slot(void __iomem *base, unsigned long off,
				  u8 stage, u32 aux)
{
	u32 raw = (u32)stage;
	u32 rolling = FORGE_MARKER_MAGIC | raw;

	/*
	 * Guard words come before the final rolling marker.  A reader trusts a
	 * slot only when signature, rolling, raw stage, and inverted guards agree.
	 */
	writel(FORGE_SIGNATURE, base + off + FORGE_SLOT_SIG_OFF);
	writel(aux, base + off + FORGE_SLOT_AUX_OFF);
	writel(~FORGE_SIGNATURE, base + off + FORGE_SLOT_SIG_INV_OFF);
	writel(~aux, base + off + FORGE_SLOT_AUX_INV_OFF);
	writel(raw, base + off + FORGE_SLOT_STAGE_OFF);
	writel(~raw, base + off + FORGE_SLOT_STAGE_INV_OFF);
	writel(rolling, base + off + FORGE_SLOT_ROLLING_OFF);
	writel(~rolling, base + off + FORGE_SLOT_ROLLING_INV_OFF);
}

void forge_m681_mark_aux(u8 stage, u32 aux)
{
	unsigned int i;

	/* Write the marker slots first and do not pr_emerg in the hot path: if
	 * printk ever blocks (console_sem, logbuf, an undrained console), a
	 * pr_emerg before the writels would block the boot thread and hide the
	 * furthest stage reached. Plain writel cannot block. */
	if (forge_spm_base)
		for (i = 0; i < ARRAY_SIZE(forge_m681_slot_offsets); i++)
			forge_m681_write_slot(forge_spm_base,
					      forge_m681_slot_offsets[i], stage, aux);

	if (forge_spm_base2)
		for (i = 0; i < ARRAY_SIZE(forge_m681_slot_offsets); i++)
			forge_m681_write_slot(forge_spm_base2,
					      forge_m681_slot_offsets[i], stage, aux);
}
EXPORT_SYMBOL(forge_m681_mark_aux);

void forge_m681_mark(u8 stage)
{
	forge_m681_mark_aux(stage, 0);
}
EXPORT_SYMBOL(forge_m681_mark);

/* Increment a checkpoint counter in the diag zone (0xE4..0xF8). Unlike
 * the 8-deep rolling stage slots (which only keep the last marks), these
 * show how many times a checkpoint ran, e.g. "stuck at C7 (count 1)" vs a
 * "C7->CA retry loop". Read-modify-write; the magnitude is the signal. */
void forge_m681_bump(unsigned int slot)
{
	u32 off = FORGE_DIAG_EMMC_CTR_BASE + (slot & 0x7u) * 4u;

	if (forge_spm_base)
		writel(readl(forge_spm_base + off) + 1u, forge_spm_base + off);
	if (forge_spm_base2)
		writel(readl(forge_spm_base2 + off) + 1u, forge_spm_base2 + off);
}
EXPORT_SYMBOL(forge_m681_bump);

/* Write a raw u32 to a diag word (diag zone, offset masked to the page).
 * Unlike the wrapping console, this keeps a value captured deep in an
 * async kworker (e.g. the VEMC pwrap readback). */
void forge_m681_diag(unsigned int off, u32 val)
{
	off &= 0x3FCu;
	if (forge_spm_base2)
		writel(val, forge_spm_base2 + off);
	if (forge_spm_base)
		writel(val, forge_spm_base + off);
}
EXPORT_SYMBOL(forge_m681_diag);

/* Record a fatal-fault snapshot from die(): PC -> slots as stage 0xFE
 * (aux = pc low32); LR and ESR -> fixed diag offsets 0x48/0x4c of the
 * marker page, so all three survive the reset. */
void forge_m681_mark_fault(u32 pc, u32 lr, u32 esr)
{
	forge_m681_mark_aux(0xFE, pc);
	if (forge_spm_base) {
		writel(lr,  forge_spm_base  + 0x48);
		writel(esr, forge_spm_base  + 0x4c);
	}
	if (forge_spm_base2) {
		writel(lr,  forge_spm_base2 + 0x48);
		writel(esr, forge_spm_base2 + 0x4c);
	}
}
EXPORT_SYMBOL(forge_m681_mark_fault);

/*
 * Fault/panic net. die (0xFE) only catches faults that route through it;
 * bad_mode->panic and direct panic() bypass it. These hooks record the
 * first fatal event from any sink (do_mem_abort unhandled=0xFD,
 * bad_mode=0xFC, panic=0xFB); if nothing latches, the reset was an IRQ-off
 * hang.
 *
 * Raw slot writes only, no pr_emerg: this may run in a wedged/lock-held
 * context where printk would deadlock or re-fault. A one-shot latch keeps
 * the first event instead of a downstream panic overwriting it.
 *
 * diag layout for the latched snapshot:
 *   slot @0x100/@0x180 stage = sink id, aux = PC low32
 *   0x48 = PC low32   0x4c = ESR   0x50 = fault addr/aux2   0x54 = sink id
 */
static int forge_fault_latched;

void forge_m681_fault_snap(u8 stage, u32 pc, u32 esr, u32 addr)
{
	if (forge_fault_latched)
		return;
	forge_fault_latched = 1;

	if (forge_spm_base) {
		forge_m681_write_slot(forge_spm_base, FORGE_MARKER_LEGACY_OFF, stage, pc);
		forge_m681_write_slot(forge_spm_base, FORGE_MARKER_SHADOW_OFF, stage, pc);
		writel(pc,    forge_spm_base + 0x48);
		writel(esr,   forge_spm_base + 0x4c);
		writel(addr,  forge_spm_base + 0x50);
		writel(stage, forge_spm_base + 0x54);
	}
	if (forge_spm_base2) {
		forge_m681_write_slot(forge_spm_base2, FORGE_MARKER_LEGACY_OFF, stage, pc);
		forge_m681_write_slot(forge_spm_base2, FORGE_MARKER_SHADOW_OFF, stage, pc);
		writel(pc,    forge_spm_base2 + 0x48);
		writel(esr,   forge_spm_base2 + 0x4c);
		writel(addr,  forge_spm_base2 + 0x50);
		writel(stage, forge_spm_base2 + 0x54);
	}
}
EXPORT_SYMBOL(forge_m681_fault_snap);

/*
 * Per-IRQ trace from the EL1 IRQ entry (gic_handle_irq): records a
 * free-running count, the last hwirq and the interrupted pc into diag
 * offsets 0x58/0x5c/0x60. A huge post-reset count with the interrupted pc
 * in the hang window indicates an IRQ storm (a peripheral asserting an
 * interrupt no loaded driver clears) and names the irq.
 * No printk; writel only; no-op until the marker region is mapped.
 */
static u32 forge_irq_count;

void forge_m681_irq_trace(u32 irqnr, u32 pc)
{
	forge_irq_count++;
	if (forge_spm_base2) {
		writel(forge_irq_count, forge_spm_base2 + 0x58);
		writel(irqnr,           forge_spm_base2 + 0x5c);
		writel(pc,              forge_spm_base2 + 0x60);
	}
	if (forge_spm_base) {
		writel(forge_irq_count, forge_spm_base + 0x58);
		writel(irqnr,           forge_spm_base + 0x5c);
		writel(pc,              forge_spm_base + 0x60);
	}
}
EXPORT_SYMBOL(forge_m681_irq_trace);

/*
 * forge_m681_wdt_arm - arm the MTK toprgu HW watchdog in single-mode
 * hw-reset. Called exactly once, from forge_m681_marker_late_init()
 * (post mm_init, before any initcalls), using the post-mm_init ioremap of
 * FORGE_WDT_PHYS_BASE (a pre-mm_init mapping of SoC register space does
 * not work that early).
 *
 * mtk_wdt_probe stays in the denylist: it calls request_irq before its
 * single-mode mode_config, and if that wedges the WDT is left in the
 * preloader's dual-mode+IRQ, where a later AXI bus hang cannot deliver
 * the IRQ and the device never resets.
 *
 * MODE is read-modify-written only: read the MODE the preloader left,
 * clear only DUAL_MODE (0x40) and IRQ (0x08), set
 * KEY|ENABLE|EXTEN|AUTO_RESTART, write back once. Then a single RESTART_KEY
 * pet resets any pre-boot count to the full window.
 *
 * The readback snapshot at 0xE6/0xE7 (mode/length) lets a post-reset
 * marker decode show the armed state from recovery.
 */
static void forge_m681_wdt_arm(void)
{
	void __iomem *b = forge_wdt_base;
	u32 mode;

	if (!b)
		return;

	mode = readl(b + FORGE_WDT_MODE_OFF);
	/* Clear DUAL_MODE + IRQ (the preloader arms dual-mode+IRQ). Keep ENABLE
	 * (already set by the preloader; re-asserted below). LENGTH is set
	 * separately below, on this same mapping, and latched by the pet. */
	mode &= ~(FORGE_WDT_MODE_DUAL_MODE | FORGE_WDT_MODE_IRQ);
	mode |= FORGE_WDT_MODE_KEY | FORGE_WDT_MODE_ENABLE |
		FORGE_WDT_MODE_EXTEN | FORGE_WDT_MODE_AUTO_RESTART;
	writel(mode, b + FORGE_WDT_MODE_OFF);

	/* Extend the WDT timeout to the hardware maximum: the preloader LENGTH
	 * (0x5000) gives only ~20s, too short for userspace (mediaserver, agps,
	 * NVRAM) to reach boot_completed. MTK_WDT_LENGTH = (count[10:0] << 5) |
	 * KEY(0x08); 0x7FF is the max count (~60-95s). Written on the post-mm_init
	 * mapping (same one as the MODE write above) and latched by the RESTART
	 * pet below. */
	writel((0x7FFU << 5) | FORGE_WDT_LENGTH_KEY, b + FORGE_WDT_LENGTH_OFF);

	/* pet once: reload the (now-extended) LENGTH counter to its full window */
	writel(FORGE_WDT_RESTART_KEY, b + FORGE_WDT_RESTART_OFF);

	/* snapshot armed state into the SRAM marker (rolling-stage
	 * channel); a post-reset readback from recovery decodes aux -> the
	 * raw MODE/LENGTH we just wrote, proving arm-before-hang. */
	forge_m681_mark_aux(0xE6, readl(b + FORGE_WDT_MODE_OFF));
	forge_m681_mark_aux(0xE7, readl(b + FORGE_WDT_LENGTH_OFF));

	pr_emerg("[FORGE_M681] v47 WDT armed single-mode-hwreset (RMW, no LENGTH reset): MODE=0x%x LENGTH=0x%x\n",
		 readl(b + FORGE_WDT_MODE_OFF),
		 readl(b + FORGE_WDT_LENGTH_OFF));
}

/*
 * forge_m681_wdt_disarm - disable the MTK toprgu HW watchdog.
 *
 * forge_m681_wdt_kick only pets the WDT once per do_one_initcall. When
 * do_initcalls() finishes the kicks stop, so the armed 30s HW watchdog
 * would fire ~30s into userspace (the mtk_wdt driver is denylisted, so
 * nothing in userspace pets it). Call this right after the last initcall
 * (POST_BASIC_SETUP, init/main.c) so userspace runs watchdog-free. Writes
 * MODE with the KEY but the ENABLE bit cleared. Trade-off: a userspace hang
 * no longer auto-resets.
 */
void forge_m681_wdt_disarm(void)
{
	void __iomem *b = forge_wdt_base ? forge_wdt_base : toprgu_base;

	if (!b)
		return;
	/* KEY only, ENABLE/EXTEN/AUTO_RESTART cleared -> WDT off. */
	writel(FORGE_WDT_MODE_KEY, b + FORGE_WDT_MODE_OFF);
	forge_m681_mark_aux(0xE5, readl(b + FORGE_WDT_MODE_OFF));
	pr_emerg("[FORGE_M681] v59 WDT DISARMED for userspace: MODE=0x%x\n",
		 readl(b + FORGE_WDT_MODE_OFF));
}
EXPORT_SYMBOL(forge_m681_wdt_disarm);

/*
 * Jiffies-tick-driven self-rearming WDT kicker. The per-initcall kick
 * (forge_m681_wdt_kick) only fires while do_initcalls() runs; once the
 * kernel hands over to userspace the kicks stop and the ~30s HW dog would
 * reset the phone. The timer wheel (timer_list) is serviced by the GPT
 * jiffies tick, so a self-rearming timer_list keeps the dog petted through
 * userspace without relying on the arch timer. On a hard hang (tick stops /
 * IRQs off) the timer stops firing and the dog still warm-resets, keeping
 * the DRAM markers. Kick every 8s (dog timeout ~30s). Armed lazily on the
 * first forge_m681_wdt_kick (i.e. the first initcall, after time_init()
 * so the tick is live). */
static struct timer_list forge_wdt_ticker;
static int forge_wdt_ticker_armed;

static void __maybe_unused forge_wdt_ticker_fn(unsigned long data)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	if (b) {
		/* Take the dog back from the stock WDK: it arms dual-mode+IRQ
		 * (RGU_MODE 0x15 -> 0x5d) and its own shorter LENGTH, after which a
		 * plain RESTART from this ticker no longer reloads the counter and the
		 * dog resets the SoC. Re-assert single-hwreset MODE + max LENGTH every
		 * tick so the RESTART below reliably reloads the counter. The dog stays
		 * enabled, so a genuine tick-stop hang still warm-resets with the
		 * markers intact. */
		u32 mode = readl(b + FORGE_WDT_MODE_OFF);

		{
			static int _hb126;

			if (_hb126 < 16) {
				_hb126++;
				pr_emerg("[FORGE_M681] #126 wdt-ticker tick#%d preMODE=0x%x preLEN=0x%x\n",
					 _hb126, mode, readl(b + FORGE_WDT_LENGTH_OFF));
			}
		}

		mode &= ~(FORGE_WDT_MODE_DUAL_MODE | FORGE_WDT_MODE_IRQ);
		mode |= FORGE_WDT_MODE_KEY | FORGE_WDT_MODE_ENABLE |
			FORGE_WDT_MODE_EXTEN | FORGE_WDT_MODE_AUTO_RESTART;
		writel(mode, b + FORGE_WDT_MODE_OFF);
		writel((0x7FFU << 5) | FORGE_WDT_LENGTH_KEY, b + FORGE_WDT_LENGTH_OFF);
		writel(FORGE_WDT_RESTART_KEY, b + FORGE_WDT_RESTART_OFF);
	}
	/* breadcrumb: ticker heartbeat into the kicker-init diag slot (0xE0). */
	if (forge_spm_base2)
		writel(0x71C0E000u | (forge_wdt_kick_count & 0xFFFFu),
		       forge_spm_base2 + FORGE_DIAG_WDTK_INIT_OFF);
	mod_timer(&forge_wdt_ticker, jiffies + msecs_to_jiffies(8000));
}

static void __maybe_unused forge_wdt_ticker_start(void)
{
	if (forge_wdt_ticker_armed)
		return;
	forge_wdt_ticker_armed = 1;
	setup_timer(&forge_wdt_ticker, forge_wdt_ticker_fn, 0);
	mod_timer(&forge_wdt_ticker, jiffies + msecs_to_jiffies(8000));
	pr_emerg("[FORGE_M681] v120 timer_list WDT ticker armed (jiffies tick, 8s)\n");
}

/*
 * Start the tick-driven WDT ticker at late_initcall so the dog stays
 * petted across the kernel->userspace handoff (the per-initcall kick
 * stops when do_initcalls() ends, and the dog would reset the phone ~30s
 * later). The ticker pets via the GPT jiffies tick, so a hard hang (tick
 * stops / IRQs off) still warm-resets with the DRAM markers intact.
 */
static int __init forge_wdt_ticker_late_init(void)
{
	forge_wdt_ticker_start();
	return 0;
}
late_initcall(forge_wdt_ticker_late_init);

/*
 * forge_m681_wdt_kick - pet the MTK toprgu watchdog from do_one_initcall().
 *
 * No-op until forge_m681_marker_late_init() maps the toprgu block (which happens
 * in start_kernel(), before any do_initcalls() runs).  Writing the restart key
 * reloads the WDT counter (same effect as the driver's mtk_wdt_restart()), which
 * keeps a progressing boot alive across the kicker-less level 4/5/6 initcall
 * window without masking a real per-initcall hang.
 */
void forge_m681_wdt_kick(void)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;
	u32 flags;

	forge_wdt_kick_count++;
	if (b)
		writel(FORGE_WDT_RESTART_KEY, b + FORGE_WDT_RESTART_OFF);

	/* Manual SWRST backstop. If the initcall path loops (kick_count far
	 * above the realistic initcall count of ~600) while system_state is still
	 * pre-RUNNING, the chip must not be kept alive by WDT pets from a broken
	 * do_one_initcall caller. Write the MTK_WDT_SWRST key directly, the same
	 * hardware reset as wdt_arch_reset(): a chip-wide warm reset that keeps
	 * the preloader reserved marker @0x44800000. A 0xEA mark right before the
	 * SWRST records that the reset was forced (vs the natural 30s timeout,
	 * which leaves the last-initcall aux as the wedged function). */
	if (system_state < SYSTEM_RUNNING &&
	    forge_wdt_kick_count >= FORGE_WDT_FORCE_SWRESET_THRESHOLD && b) {
		forge_m681_mark(0xEA);
		writel(FORGE_WDT_SWRST_KEY, b + FORGE_WDT_SWRST_OFF);
	}

	/* Breadcrumb: how many initcalls kicked, and which base was live
	 * (bit0 = own ioremap, bit4 = driver toprgu_base). */
	flags = 0xC0DE0000U | (toprgu_base ? 0x10 : 0) | (forge_wdt_base ? 0x01 : 0);
	if (forge_spm_base2) {
		writel(forge_wdt_kick_count, forge_spm_base2 + FORGE_DIAG_KICKCNT_OFF);
		writel(flags, forge_spm_base2 + FORGE_DIAG_KICKFLAGS_OFF);
	}
	if (forge_spm_base) {
		writel(forge_wdt_kick_count, forge_spm_base + FORGE_DIAG_KICKCNT_OFF);
		writel(flags, forge_spm_base + FORGE_DIAG_KICKFLAGS_OFF);
	}

	/* GPT2-counts probe. Runs every initcall. Latches the first GPT2
	 * reading, then on every later kick writes the delta to diag 0xDC:
	 * sentinel 0xD2 in the high byte (shows it ran), bit23 = "counter moved",
	 * low 22 bits = delta, i.e. whether 0x10008028 counts. */
	if (forge_gpt_base) {
		static u32 g0;
		static int gset;
		u32 gn = readl(forge_gpt_base + 0x28);
		u32 pk;
		if (!gset) { g0 = gn; gset = 1; }
		pk = 0xD2000000u | (((gn - g0) != 0) ? (1u << 23) : 0u) |
		     ((gn - g0) & 0x3FFFFFu);
		if (forge_spm_base2)
			writel(pk, forge_spm_base2 + 0xDC);
		if (forge_spm_base)
			writel(pk, forge_spm_base + 0xDC);
	}
}
EXPORT_SYMBOL(forge_m681_wdt_kick);

/*
 * WDT kicker via hrtimer. A kthread kicker may never get a timeslice on a
 * single-CPU boot; an hrtimer fires from the timer-interrupt path,
 * independent of thread scheduling. A genuine AXI/APB bus wedge stalls the
 * CPU on the un-acked MMIO access and takes no interrupts, so the hrtimer
 * also stops and the dog warm-resets the SoC. Pets the dog directly (not
 * via forge_m681_wdt_kick) to avoid that path's pre-RUNNING SWRST
 * backstop. heartbeat -> diag @0xDC (shows the timer fires); arm-proof ->
 * diag @0xE0 (A11E).
 */
static struct hrtimer forge_wdt_hrtimer;
static u32 forge_wdt_beat;
#define FORGE_WDT_KICK_NS	(5ULL * NSEC_PER_SEC)	/* 5s, ~6x margin under 30s WDT */

static enum hrtimer_restart forge_wdt_hrtimer_fn(struct hrtimer *t)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	if (b)
		writel(FORGE_WDT_RESTART_KEY, b + FORGE_WDT_RESTART_OFF);
	forge_wdt_beat++;
	if (forge_spm_base2)
		writel(0xC0DE0000U | (forge_wdt_beat & 0xFFFFU),
		       forge_spm_base2 + FORGE_DIAG_WDTK_HEARTBEAT_OFF);
	if (forge_spm_base)
		writel(0xC0DE0000U | (forge_wdt_beat & 0xFFFFU),
		       forge_spm_base + FORGE_DIAG_WDTK_HEARTBEAT_OFF);
	hrtimer_forward_now(t, ns_to_ktime(FORGE_WDT_KICK_NS));
	return HRTIMER_RESTART;
}

static int __init forge_wdt_hrtimer_init(void)
{
	/* Arm the hrtimer WDT kicker: it pets the dog every 5s from the timer-IRQ
	 * path, so a slow userspace boot is not cut by the armed HW WDT
	 * (MODE=0x15). A soft hang then no longer warm-resets; a bus wedge still
	 * does (the CPU stalls and the hrtimer stops). */
	if (forge_virt)
		return 0;	/* QEMU virt: no MTK toprgu, nothing to pet */
	hrtimer_init(&forge_wdt_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	forge_wdt_hrtimer.function = forge_wdt_hrtimer_fn;
	hrtimer_start(&forge_wdt_hrtimer, ns_to_ktime(FORGE_WDT_KICK_NS),
		      HRTIMER_MODE_REL);
	pr_emerg("[FORGE_M681] v184 wdt hrtimer kicker RE-ENABLED (5s pet -> start holds long for Mali bring-up)\n");
	return 0;
}
core_initcall(forge_wdt_hrtimer_init);

/*
 * SPM power-domain state dump, as a late_initcall. The DRAM marker can be
 * overwritten by recovery, and /dev/mem from recovery cannot read SPM IO
 * (STRICT_DEVMEM), so the state is printed from the booting kernel. It
 * prints the DIS MTCMOS power state; mt_scpsys_init (CLK_OF_DECLARE
 * "mediatek,mt6755-scpsys") powers DIS on at of_clk_init, so a kernel that
 * boots at all has already powered DIS.
 */
static int __init forge_spm_dump(void)
{
	void __iomem *spm;
	u32 cfg, sta, sta2, dis, mfg, isp, mm;

	if (forge_virt)
		return 0;	/* QEMU virt: 0x10006000 is in the PCIe MMIO window */
	spm = ioremap(0x10006000UL, 0x1000);

	if (!spm) {
		pr_emerg("[FORGE_M681] v169 spm_dump: ioremap(0x10006000) FAILED\n");
		return 0;
	}
	cfg  = readl(spm + 0x000);	/* POWERON_CONFIG_EN */
	sta  = readl(spm + 0x180);	/* PWR_STATUS      */
	sta2 = readl(spm + 0x184);	/* PWR_STATUS_2ND  */
	dis  = readl(spm + 0x30c);	/* DIS_PWR_CON     */
	mm   = readl(spm + 0x308);	/* (MM_PWR_CON neighbor, informational) */
	mfg  = readl(spm + 0x214);	/* MFG_PWR_CON (informational) */
	isp  = readl(spm + 0x238);	/* ISP_PWR_CON (informational) */
	pr_emerg("[FORGE_M681] v169 SPM cfg=%08x PWR_STATUS=%08x/2ND=%08x\n",
		 cfg, sta, sta2);
	pr_emerg("[FORGE_M681] v169 DIS_PWR_CON=%08x dis_on(sta b3)=%d sram_ack(b12)=%d\n",
		 dis, !!((sta & (1u<<3)) && (sta2 & (1u<<3))),
		 !!(dis & (1u<<12)));
	pr_emerg("[FORGE_M681] v169 neigh MM(0x308)=%08x MFG(0x214)=%08x ISP(0x238)=%08x\n",
		 mm, mfg, isp);
	iounmap(spm);
	return 0;
}
late_initcall(forge_spm_dump);

/*
 * ===== panic black-box (printk-ring tail into reserved DRAM) =====
 *
 * The panic net (below) issues the SWRST from the first panic notifier
 * (INT_MAX priority runs first), and panic() only calls kmsg_dump() after
 * the notifier chain, so a panic whose text is stuck behind a held console
 * lock leaves its backtrace only in __log_buf, which is lost with the
 * reset. Before the SWRST, pull the last ~32KB of the printk ring via the
 * kmsg_dump cursor API (dumper.active set manually; the locked variant: if
 * the panic happened inside the logbuf_lock holder this can deadlock, in
 * which case the armed dog still resets the SoC and the 0xF1 marker still
 * says "panic") and copy it into preloader-reserved DRAM at 0x44810000
 * (+64KB from the marker page; no other user). memremap(MEMREMAP_WB), not
 * ioremap: a second device-attribute alias of this cacheable linear-mapped
 * region breaks the boot; WB matches the linear map, and
 * __flush_dcache_area pushes the bytes to DRAM so they survive the warm
 * reset.
 * Readback from recovery:
 *   dd if=/dev/mem of=/tmp/panic.bin bs=4096 skip=$((0x44810000/4096)) count=8
 *   header 'F681PANC' + u32 len + u32 ms; text starts at +16.
 */
#include <linux/kmsg_dump.h>
#include <linux/io.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <asm/cacheflush.h>

#define FORGE_PANIC_DUMP_PHYS	0x44810000UL
#define FORGE_PANIC_DUMP_SIZE	0x8000UL
/* last 256B of the box = orderly-reboot record ('F681REBT'):
 * u32 magic x2, u32 ms, u32 action, comm[16] at +16, cmd[64] at +32.
 * The panic dump is capped at FORGE_REBT_OFF so they cannot collide. */
#define FORGE_REBT_OFF		0x7F00UL
static void *forge_panic_dump_base;	/* WB mapping -> plain memcpy */

static void forge_panic_blackbox_dump(void)
{
	struct kmsg_dumper d = { .active = true };
	size_t len = 0;
	char *dst = forge_panic_dump_base;
	u32 *hdr = (u32 *)dst;

	if (!dst)
		return;
	kmsg_dump_rewind(&d);
	kmsg_dump_get_buffer(&d, false, dst + 16,
			     FORGE_REBT_OFF - 16, &len);
	hdr[0] = 0x31383646U;	/* 'F681' (LE) */
	hdr[1] = 0x434E4150U;	/* 'PANC' (LE) */
	hdr[2] = (u32)len;
	hdr[3] = (u32)(ktime_to_ns(ktime_get()) / 1000000);
	__flush_dcache_area(dst, len + 16);
}

/*
 * Orderly-reboot black-box. A userspace-commanded reboot (e.g. init's
 * critical-service escalation or RescueParty "reboot,recovery") prints
 * "reboot: Restarting system with command ..." behind a held console lock
 * just like a panic text, and the line can be lost. kernel_restart_prepare()
 * passes the cmd string through the reboot notifier chain (kernel/reboot.c)
 * in the caller's context, so one record gives: who (comm), what (cmd:
 * NULL / "recovery" / "bootloader"), when (ms). emergency_restart()
 * bypasses the chain; kernel/reboot.c calls forge_m681_emergency_mark() on
 * that path (action=0xEE).
 */
static void forge_rebt_record(unsigned long action, const char *cmd)
{
	char *dst = forge_panic_dump_base;
	u32 *h;

	if (!dst)
		return;
	h = (u32 *)(dst + FORGE_REBT_OFF);
	h[0] = 0x31383646U;	/* 'F681' (LE) */
	h[1] = 0x54424552U;	/* 'REBT' (LE) */
	h[2] = (u32)(ktime_to_ns(ktime_get()) / 1000000);
	h[3] = (u32)action;
	memset(dst + FORGE_REBT_OFF + 16, 0, 80);
	strncpy(dst + FORGE_REBT_OFF + 16, current->comm, 15);
	if (cmd)
		strncpy(dst + FORGE_REBT_OFF + 32, cmd, 63);
	__flush_dcache_area(dst + FORGE_REBT_OFF, 96);
	pr_emerg("[FORGE_M681] #118 reboot black-box: action=0x%lx comm=%s cmd='%s' ms=%u\n",
		 action, current->comm, cmd ? cmd : "(null)", h[2]);
}

static int forge_reboot_notifier_fn(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	forge_rebt_record(action, (const char *)data);
	return NOTIFY_DONE;
}

static struct notifier_block forge_reboot_nb = {
	.notifier_call = forge_reboot_notifier_fn,
	.priority = INT_MAX,	/* record before any other shutdown work */
};

/* Called from kernel/reboot.c emergency_restart() — that path skips the
 * reboot notifier chain entirely. */
void forge_m681_emergency_mark(void)
{
	forge_rebt_record(0xEE, "emergency_restart");
}

static int __init forge_panic_blackbox_init(void)
{
	if (forge_virt)
		return 0;	/* reserved region is MTK-hw only */
	forge_panic_dump_base = memremap(FORGE_PANIC_DUMP_PHYS,
					 FORGE_PANIC_DUMP_SIZE, MEMREMAP_WB);
	register_reboot_notifier(&forge_reboot_nb);
	pr_emerg("[FORGE_M681] #117 panic black-box %s @0x%lx (32KB, WB) + #118 reboot record @+0x%lx\n",
		 forge_panic_dump_base ? "armed" : "MAP FAILED",
		 FORGE_PANIC_DUMP_PHYS, FORGE_REBT_OFF);
	return 0;
}
late_initcall(forge_panic_blackbox_init);

/*
 * Panic-notifier direct-SWRST safety net.
 *
 * The toprgu HW timer-WDT expiry does not reliably reset this SoC (the
 * preloader even carries "WDT does not trigger reboot"); a direct
 * SWRST_KEY write, as wdt_arch_reset() does on panic, does. Registering on
 * the kernel panic_notifier chain makes any panic (BUG_ON, OOPS, MCE, OOM)
 * issue it instead of relying on machine_restart plumbing.
 *
 * The notifier is atomic-context safe: it issues only a single mmio writel
 * to toprgu+0x14 with MTK_WDT_SWRST_KEY (0x1209), the constant the
 * driver's wdt_arch_reset() uses, preceded by forge_m681_mark(0xF1), so
 * the marker shows "0xF1: panicked and forced reset". A one-shot latch
 * keeps the first panic from being shadowed by a later call from
 * atomic_notifier_call_chain iterating over several notifiers. pr_emerg
 * is best-effort.
 */
static int forge_panic_latched;

static int forge_m681_panic_handler(struct notifier_block *this,
				    unsigned long ev, void *ptr)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	if (forge_panic_latched)
		return NOTIFY_DONE;
	forge_panic_latched = 1;

	forge_m681_mark(0xF1);
	pr_emerg("[FORGE_M681] panic rv48: writing SWRST_KEY to toprgu+0x14 to force warm reset\n");

	/* Save the printk ring tail into reserved DRAM before the reset: the
	 * only copy of a panic text silenced by a held console lock. 0xF1 is
	 * already latched above, so even a wedge inside the dump still
	 * classifies the reset as a panic. */
	forge_panic_blackbox_dump();

	if (b)
		writel(FORGE_WDT_SWRST_KEY, b + FORGE_WDT_SWRST_OFF);

	/* If the SWRST_KEY write worked we never return; if it didn't (toprgu
	 * unmapped, write blocked), at least the marker stuck and the next
	 * reset attempt (manual SWRST backstop in forge_m681_wdt_kick, or
	 * physical battery pull) carries the 0xF1 evidence. */
	return NOTIFY_DONE;
}

static struct notifier_block forge_m681_panic_nb = {
	.notifier_call = forge_m681_panic_handler,
	.priority = INT_MAX,	/* run last so other panic handlers log first */
};

/*
 * Dedicated "current initcall fn" tracker.
 *
 * The rolling slots only hold the last mark, and the inner platform-probe
 * marks (0xC0 attempt / 0xCD denylist skip) overwrite the per-initcall 0xE0
 * aux, so after a hang inside an initcall that probes platform drivers the
 * marker would name the last probe, not the wedged initcall. These fixed
 * diag offsets are written only here, so they always name the initcall
 * entered last (and whether it returned). fn is mirrored to four offsets
 * because single copies in the preloader region (0x44800000) can lose
 * bits; the reader ORs them. seq is the do_one_initcall ordinal.
 *
 * diag layout:  0x64/0x70/0x74/0x78 = current fn (OR these)   0x68 = seq
 *               0x6c = last fn that returned (entered != returned => wedged)
 */
static u32 forge_initcall_seq;

static void forge_init_write(void __iomem *b, u32 fn)
{
	writel(fn, b + 0x64);
	writel(fn, b + 0x70);
	writel(fn, b + 0x74);
	writel(fn, b + 0x78);
	writel(forge_initcall_seq, b + 0x68);
}

void forge_m681_set_initcall(u32 fn)
{
	forge_initcall_seq++;
	if (forge_spm_base)
		forge_init_write(forge_spm_base, fn);
	if (forge_spm_base2)
		forge_init_write(forge_spm_base2, fn);
}
EXPORT_SYMBOL(forge_m681_set_initcall);

u32 forge_m681_get_initcall_seq(void)
{
	return forge_initcall_seq;
}
EXPORT_SYMBOL(forge_m681_get_initcall_seq);

void forge_m681_set_initcall_done(u32 fn)
{
	/* Mirror the last-good seq (0xD0) + guard (0xD4) so a reader gets the
	 * seq of the last initcall that returned without cross-referencing 0x68
	 * (current seq) and 0x6c (last done fn). */
	u32 seq = forge_initcall_seq;
	if (forge_spm_base) {
		writel(fn, forge_spm_base + 0x6c);
		writel(seq, forge_spm_base + FORGE_DIAG_LASTGOOD_SEQ_OFF);
		writel(~seq, forge_spm_base + FORGE_DIAG_LASTGOOD_SEQ_INV_OFF);
	}
	if (forge_spm_base2) {
		writel(fn, forge_spm_base2 + 0x6c);
		writel(seq, forge_spm_base2 + FORGE_DIAG_LASTGOOD_SEQ_OFF);
		writel(~seq, forge_spm_base2 + FORGE_DIAG_LASTGOOD_SEQ_INV_OFF);
	}
}
EXPORT_SYMBOL(forge_m681_set_initcall_done);

/*
 * of_platform_populate node tracker: marks every node reaching
 * of_platform_device_create_pdata, so a hang in of-core device creation
 * (not in a driver probe, which marks 0xC0) names the exact node.
 *
 * diag layout:  0x80/0x84/0x88/0x8c = DFS node counter (OR the 4 copies)
 *               0x90..0x9c = 16 ASCII bytes of full_name (copy A)
 *               0xa0..0xac = same 16 ASCII bytes (copy B; reader ORs A|B)
 */
static u32 forge_ofnode_seq;

static void forge_ofnode_write(void __iomem *b, const u32 *w)
{
	writel(forge_ofnode_seq, b + 0x80);
	writel(forge_ofnode_seq, b + 0x84);
	writel(forge_ofnode_seq, b + 0x88);
	writel(forge_ofnode_seq, b + 0x8c);
	writel(w[0], b + 0x90); writel(w[1], b + 0x94);
	writel(w[2], b + 0x98); writel(w[3], b + 0x9c);
	writel(w[0], b + 0xa0); writel(w[1], b + 0xa4);
	writel(w[2], b + 0xa8); writel(w[3], b + 0xac);
}

void forge_m681_mark_ofnode(const char *name)
{
	const char *s = name ? name : "";
	u8 buf[16];
	u32 w[4];
	int i;

	forge_ofnode_seq++;
	for (i = 0; i < 16; i++) {
		buf[i] = *s ? (u8)*s : 0;
		if (*s)
			s++;
	}
	for (i = 0; i < 4; i++)
		w[i] = buf[i*4] | (buf[i*4+1] << 8) |
		       (buf[i*4+2] << 16) | (buf[i*4+3] << 24);

	if (forge_spm_base)
		forge_ofnode_write(forge_spm_base, w);
	if (forge_spm_base2)
		forge_ofnode_write(forge_spm_base2, w);
}
EXPORT_SYMBOL(forge_m681_mark_ofnode);

/*
 * Initcall-level phase markers and per-level completion counters.
 *
 * do_initcall_level() calls mark_level_enter(level) before the level's
 * initcall loop and mark_level_done(level) after it. The rolling-stage
 * channel records 0xE8/0xE9 (aux=level), so a post-reset marker decode
 * names the level (early/core/postcore/arch/subsys/fs/device/late) a hang
 * is in, complementing the per-initcall fn tracker (0x64/0x6c).
 *
 * A reader can tell whether the hang is at the start of a level
 * (enter>0, done=0) or in the middle (done>0, enter>done+1). The
 * "last-good seq" (seq of the last initcall that returned) is mirrored to
 * diag 0xD0.
 *
 * diag layout:
 *   0xB0..0xB7 = per-level enter count (u8 each, 8 levels packed)
 *   0xC0..0xC7 = per-level done count  (u8 each, 8 levels packed)
 *   0xD0       = last-good initcall seq (u32, written in set_initcall_done)
 *   0xD4       = last-good initcall seq guard (~seq)
 *   0xD8       = boot-phase owner: current initcall level (u32)
 */
static u8 forge_level_enter_count[8];
static u8 forge_level_done_count[8];

static void forge_level_write_counts(void __iomem *b)
{
	int i;
	for (i = 0; i < 8; i++) {
		writel(forge_level_enter_count[i],
		       b + FORGE_DIAG_LEVEL_ENTER_BASE + i * 4);
		writel(forge_level_done_count[i],
		       b + FORGE_DIAG_LEVEL_DONE_BASE + i * 4);
	}
}

void forge_m681_mark_level_enter(int level)
{
	if (level < 0 || level >= 8)
		return;
	forge_level_enter_count[level]++;
	forge_m681_mark_aux(FORGE_STAGE_INITCALL_LEVEL_ENTER, (u32)level);
	if (forge_spm_base) {
		writel((u32)level, forge_spm_base + FORGE_DIAG_BOOTPHASE_OFF);
		forge_level_write_counts(forge_spm_base);
	}
	if (forge_spm_base2) {
		writel((u32)level, forge_spm_base2 + FORGE_DIAG_BOOTPHASE_OFF);
		forge_level_write_counts(forge_spm_base2);
	}
}
EXPORT_SYMBOL(forge_m681_mark_level_enter);

void forge_m681_mark_level_done(int level)
{
	if (level < 0 || level >= 8)
		return;
	forge_level_done_count[level]++;
	forge_m681_mark_aux(FORGE_STAGE_INITCALL_LEVEL_DONE, (u32)level);
	if (forge_spm_base)
		forge_level_write_counts(forge_spm_base);
	if (forge_spm_base2)
		forge_level_write_counts(forge_spm_base2);
}
EXPORT_SYMBOL(forge_m681_mark_level_done);

/* Direct SWRST_KEY write as a last-resort reset. Uses the same
 * toprgu_base/forge_wdt_base fallback as the kick path. */
void forge_m681_wdt_swrst(void)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;
	if (b)
		writel(FORGE_WDT_SWRST_KEY, b + FORGE_WDT_SWRST_OFF);
}
EXPORT_SYMBOL(forge_m681_wdt_swrst);

/*
 * Called from arch/arm64/kernel/setup.c right after early_ioremap_init().
 * Maps the marker region via the fixmap so we can mark stages from A04 onward,
 * well before the SPM/ram_console drivers register.
 */
void __init forge_m681_marker_early_init(void)
{
	if (forge_spm_base || forge_spm_base2)
		return;	/* already mapped */

	forge_spm_base = early_ioremap(FORGE_MARKER_PHYS_BASE,
				       FORGE_MARKER_REGION_SIZE);
	if (!forge_spm_base)
		pr_emerg("[FORGE_M681] marker early_ioremap(0x%lx) FAILED\n",
			 FORGE_MARKER_PHYS_BASE);

	forge_spm_base2 = early_ioremap(FORGE_MARKER2_PHYS_BASE,
					FORGE_MARKER_REGION_SIZE);
	if (!forge_spm_base2)
		pr_emerg("[FORGE_M681] marker2 early_ioremap(0x%lx) FAILED\n",
			 FORGE_MARKER2_PHYS_BASE);

	pr_emerg("[FORGE_M681] marker early base=%p base2=%p\n",
		 forge_spm_base, forge_spm_base2);


	/* sentinel so we know the marker engine itself is alive */
	forge_m681_mark(FORGE_STAGE_MARKER_EARLY_INIT);
}

/*
 * Enable the MTK cpuxgpt (the ARM architected system counter) via the
 * secure SMC, as stock setup_syscnt() does (the calls are commented out in
 * mt_gpt.c here); without it CNTVCT_EL0 is frozen. The cpuxgpt CTL is in
 * write-protected MCUSYS (0x10200000): direct EL1 MMIO writes are
 * rejected, but writes routed through SMC MTK_SIP_KERNEL_MCUSYS_WRITE
 * (0x82000201) are serviced by the ATF. Sequence (=
 * __cpuxgpt_set_clk(CLK_DIV2) + __cpuxgpt_enable() from
 * mtk_cpuxgpt_mt6755.c): write INDEX_CTL(0), then CTL with clk=13MHz/DIV2
 * and EN bit0. Reads of CTL (0x10200670) are plain MMIO (only writes are
 * protected). Reading CNTVCT before/after verifies the counter runs.
 * Result -> diag 0x90 (sentinel 0xCC | CNTVCT-delta-nonzero<<23 | low
 * bits) and 0xA4 (final CTL readback). */
static noinline int forge_smc(u64 fid, u64 a0, u64 a1, u64 a2)
{
	register u64 r0 __asm__("x0") = fid;
	register u64 r1 __asm__("x1") = a0;
	register u64 r2 __asm__("x2") = a1;
	register u64 r3 __asm__("x3") = a2;

	asm volatile ("smc    #0\n" : "+r"(r0) : "r"(r1), "r"(r2), "r"(r3));
	return (int)r0;
}

static inline u64 forge_read_cntvct(void)
{
	u64 v;

	asm volatile ("mrs %0, cntvct_el0" : "=r"(v));
	return v;
}

#define FORGE_SMC_MCUSYS_WRITE	0x82000201ULL
#define FORGE_CPUXGPT_INDEX_PHY	0x10200674ULL	/* MCUSYS + 0x674 */
#define FORGE_CPUXGPT_CTL_PHY	0x10200670ULL	/* MCUSYS + 0x670 */
#define FORGE_CPUXGPT_IDX_CTL	0x000U		/* INDEX_CTL_REG */
#define FORGE_CPUXGPT_EN	0x01U		/* EN_CPUXGPT */
#define FORGE_CPUXGPT_DIV2	(0x2U << 8)	/* CLK_DIV2 = 13MHz/2 */
#define FORGE_CPUXGPT_DIV_MASK	(~(0x7U << 8))	/* CLK_DIV_MASK */

static void forge_cpuxgpt_wr(u32 addr, u32 val)
{
	forge_smc(FORGE_SMC_MCUSYS_WRITE, addr, val, 0);
}

void __init forge_enable_cpuxgpt(void)
{
	void __iomem *mcusys = ioremap(0x10200000UL, 0x1000);
	u64 c1, c2;
	u32 ctl0 = 0, ctlf = 0, tmp;
	volatile int spin;

	c1 = forge_read_cntvct();

	/* set cpuxgpt clk = 13MHz / DIV2 (read-modify-write the CTL) */
	forge_cpuxgpt_wr(FORGE_CPUXGPT_INDEX_PHY, FORGE_CPUXGPT_IDX_CTL);
	if (mcusys)
		ctl0 = readl(mcusys + 0x670);
	tmp = (ctl0 & FORGE_CPUXGPT_DIV_MASK) | FORGE_CPUXGPT_DIV2;
	forge_cpuxgpt_wr(FORGE_CPUXGPT_INDEX_PHY, FORGE_CPUXGPT_IDX_CTL);
	forge_cpuxgpt_wr(FORGE_CPUXGPT_CTL_PHY, tmp);

	/* set EN_CPUXGPT -> system counter free-runs -> CNTVCT ticks */
	forge_cpuxgpt_wr(FORGE_CPUXGPT_INDEX_PHY, FORGE_CPUXGPT_IDX_CTL);
	if (mcusys)
		tmp = readl(mcusys + 0x670);
	tmp |= FORGE_CPUXGPT_EN;
	forge_cpuxgpt_wr(FORGE_CPUXGPT_INDEX_PHY, FORGE_CPUXGPT_IDX_CTL);
	forge_cpuxgpt_wr(FORGE_CPUXGPT_CTL_PHY, tmp);

	for (spin = 0; spin < 3000000; spin++)
		cpu_relax();
	c2 = forge_read_cntvct();

	forge_cpuxgpt_wr(FORGE_CPUXGPT_INDEX_PHY, FORGE_CPUXGPT_IDX_CTL);
	if (mcusys) {
		ctlf = readl(mcusys + 0x670);
		iounmap(mcusys);
	}

	/* 0x90 = sentinel 0xCC | (CNTVCT moved)<<23 | low22 of the CNTVCT delta.
	 * 0xA4 = (initial CTL << 16) | final CTL & 0xFFFF (bit0 should be 1 = EN). */
	if (forge_spm_base2) {
		writel(0xCC000000u | (((c2 - c1) != 0) ? (1u << 23) : 0u) |
		       ((u32)(c2 - c1) & 0x3FFFFFu), forge_spm_base2 + 0x90);
		writel(((ctl0 & 0xFFFFu) << 16) | (ctlf & 0xFFFFu),
		       forge_spm_base2 + 0xA4);
	}
	pr_emerg("[FORGE_M681] v121 cpuxgpt: CTL 0x%x->0x%x, CNTVCT %llu->%llu (d=%llu) %s\n",
		 ctl0, ctlf, c1, c2, c2 - c1,
		 (c2 != c1) ? "RUNNING!" : "still frozen");
}

/*
 * Called from init/main.c after mm_init() when vmalloc and the regular
 * ioremap() are available.  We swap the fixmap mapping for a permanent
 * ioremap() so the late_initcall leak check stays quiet and the mapping
 * survives the early_ioremap_reset() flag flip.
 */
/*
 * Boot deadline. Unless a host configures the USB gadget
 * (forge_usb_configured, android.c / configfs.c) within
 * forge_boot_deadline seconds of late_initcall, send the phone to
 * recovery, instead of leaving a phone that never finished booting dark.
 * 0 disables it; the parameter is writable at runtime
 * (/sys/module/kernel/parameters/).
 *
 * LK reads the bootloader message from the "para" partition (ROM fstab
 * /dev/block/mmcblk0p2 /misc emmc; GPT p2 = para), and "boot-recovery"
 * there sends a plain reboot to recovery. The bootguard thread below arms
 * that message as soon as para exists (after checking its GPT name);
 * userspace clears it on sys.boot_completed. USB configuration switches
 * the deadline off; if the host then loses it for forge_usb_lost_deadline
 * seconds while the BCB is still armed, the phone restarts into recovery
 * as well.
 */
extern int forge_usb_configured __attribute__((weak));
static int forge_boot_deadline = 120;
core_param(forge_boot_deadline, forge_boot_deadline, int, 0644);

#define FORGE_BCB_PARTNO	2
#define FORGE_BCB_PARTNAME	"para"
/* The eMMC; a parameter only so the path can be exercised on a qemu disk. */
static char *forge_bcb_disk = "mmcblk0";
core_param(forge_bcb_disk, forge_bcb_disk, charp, 0444);
static int forge_bcb_clear_on_usb;
core_param(forge_bcb_clear_on_usb, forge_bcb_clear_on_usb, int, 0644);
/* Seconds the host may stay without a USB configuration, once it had one,
 * before the bootguard restarts into recovery; 0 disables it at runtime. */
static int forge_usb_lost_deadline = 60;
core_param(forge_usb_lost_deadline, forge_usb_lost_deadline, int, 0644);
extern void forge_usb_log_state(const char *why) __attribute__((weak));

/*
 * Where the bootloader message lives. GPT: p2 "para" = LBA 32832, 1024
 * sectors (512 KiB), whole eMMC 30535680 sectors.
 *
 *  1. p2 whose GPT name is "para" - the normal case (sector 0 of p2);
 *  2. p2 without a name but with exactly that geometry (sector 0 of p2);
 *  3. no p2 at all for 2 s on a disk of exactly that capacity: absolute
 *     sector 32832 of the whole disk - the same physical sector the LK reads.
 * 2 and 3 additionally require the first 64 bytes there to be empty or plain
 * ASCII (an old message), so they cannot scribble over foreign data.
 *
 * blk_lookup_devt() hands out the dev_t of a partition that does not exist
 * yet ("return the right devno, even if the partition doesn't exist yet"),
 * so a missing partition table shows up as disk_get_part() == NULL.
 */
#define FORGE_BCB_START_LBA	32832ULL
#define FORGE_BCB_NR_SECT	1024ULL
#define FORGE_M681_EMMC_SECT	30535680ULL

struct forge_bcb_target {
	dev_t devt;
	sector_t sector;	/* 512-byte sector within devt */
	bool check_content;
	const char *how;
};

static unsigned long forge_bcb_nopart_since;

static int forge_bcb_locate(struct forge_bcb_target *t)
{
	static bool reported;
	struct gendisk *disk;
	struct hd_struct *part;
	const char *name = "";
	dev_t devt;
	int partno, ret = -ENOENT;

	devt = blk_lookup_devt(forge_bcb_disk, FORGE_BCB_PARTNO);
	if (!devt)
		return -ENODEV;
	disk = get_gendisk(devt, &partno);
	if (!disk)
		return -ENODEV;
	part = disk_get_part(disk, partno);
	if (part) {
		forge_bcb_nopart_since = 0;
		if (part->info)
			name = (const char *)part->info->volname;
		if (!strcmp(name, FORGE_BCB_PARTNAME)) {
			*t = (struct forge_bcb_target){ devt, 0, false, "GPT name" };
			ret = 0;
		} else if (!name[0] && part->start_sect == FORGE_BCB_START_LBA &&
			   part->nr_sects == FORGE_BCB_NR_SECT) {
			*t = (struct forge_bcb_target){ devt, 0, true, "p2 geometry" };
			ret = 0;
		} else {
			if (!reported)
				pr_emerg("[FORGE_M681] BCB: %s part %d is \"%s\" @%llu+%llu - not the m681 para, not writing\n",
					 forge_bcb_disk, FORGE_BCB_PARTNO, name,
					 (unsigned long long)part->start_sect,
					 (unsigned long long)part->nr_sects);
			reported = true;
			ret = -EINVAL;
		}
		disk_put_part(part);
	} else if (get_capacity(disk) == FORGE_M681_EMMC_SECT) {
		if (!forge_bcb_nopart_since)
			forge_bcb_nopart_since = jiffies ? jiffies : 1;
		else if (time_after(jiffies, forge_bcb_nopart_since + 2 * HZ)) {
			*t = (struct forge_bcb_target){ disk_devt(disk),
				FORGE_BCB_START_LBA, true, "whole disk" };
			ret = 0;
		}
	}
	put_disk(disk);
	return ret;
}

static bool forge_bcb_plausible(const u8 *b)
{
	int i;

	for (i = 0; i < 64; i++)
		if (b[i] && b[i] != '\n' && (b[i] < 0x20 || b[i] > 0x7e))
			return false;
	return true;
}

/* one BCB access at a time: bootguard, reboot notifier, bcb_test */
static DEFINE_MUTEX(forge_bcb_lock);
/* the last write of this boot, for bcb_test's getter (no eMMC access) */
static const char *forge_bcb_last = "none";
static int forge_bcb_last_err;

static int forge_bcb_rw_locked(const char *cmd, const char *status, bool *armed);

/*
 * Write the bootloader message (cmd == NULL clears it), or with armed != NULL
 * only read it back and report whether it still says boot-recovery.
 */
static int forge_bcb_rw(const char *cmd, const char *status, bool *armed)
{
	int err;

	mutex_lock(&forge_bcb_lock);
	err = forge_bcb_rw_locked(cmd, status, armed);
	if (!armed) {
		forge_bcb_last = cmd ? status : "cleared";
		forge_bcb_last_err = err;
	}
	mutex_unlock(&forge_bcb_lock);
	return err;
}

static int forge_bcb_rw_locked(const char *cmd, const char *status, bool *armed)
{
	static const char *last_how;
	fmode_t mode = armed ? FMODE_READ : FMODE_READ | FMODE_WRITE;
	struct forge_bcb_target t;
	struct block_device *bdev;
	struct buffer_head *bh;
	unsigned int bs;
	char *msg;
	u64 pos;
	int err;

	err = forge_bcb_locate(&t);
	if (err)
		return err;
	if (t.how != last_how) {
		pr_emerg("[FORGE_M681] BCB target: %s (dev %u:%u sector %llu)\n",
			 t.how, MAJOR(t.devt), MINOR(t.devt),
			 (unsigned long long)t.sector);
		last_how = t.how;
	}
	bdev = blkdev_get_by_dev(t.devt, mode, NULL);
	if (IS_ERR(bdev))
		return PTR_ERR(bdev);
	/* Userspace may have cleared the message through another node
	 * (mmcblk0p2, by-name/para): drop our clean cached copy first. */
	if (armed)
		invalidate_bdev(bdev);
	/*
	 * __bread() must be called with the device's current block size
	 * (i_blkbits): __find_get_block_slow() indexes the page cache with it,
	 * grow_buffers() with the size passed in. A 512-byte read of block
	 * 32832 on a whole disk whose block size is 4096 spins forever
	 * (qemu, whole-disk fallback; block 0 of p2 only worked because both
	 * indices are 0). So read the containing block and edit 512 bytes.
	 */
	bs = block_size(bdev);
	pos = (u64)t.sector * 512;
	bh = __bread(bdev, div_u64(pos, bs), bs);
	if (!bh) {
		err = -EIO;
		goto out;
	}
	msg = bh->b_data + do_div(pos, bs);
	if (armed) {
		*armed = !strncmp(msg, "boot-recovery", 13);
		brelse(bh);
		err = 0;
		goto out;
	}
	if (t.check_content && !forge_bcb_plausible(msg)) {
		static bool foreign_reported;

		if (!foreign_reported)
			pr_emerg("[FORGE_M681] BCB: %s sector %llu holds foreign data - not writing\n",
				 t.how, (unsigned long long)t.sector);
		foreign_reported = true;
		brelse(bh);
		err = -EEXIST;
		goto out;
	}
	memset(msg, 0, 512);
	if (cmd) {
		strcpy(msg, cmd);			/* command[32] */
		strcpy(msg + 32, status);		/* status[32] */
		strcpy(msg + 64, "recovery\n");	/* recovery[768] */
	}
	mark_buffer_dirty(bh);
	err = sync_dirty_buffer(bh);
	brelse(bh);
	if (!err)
		err = blkdev_issue_flush(bdev, GFP_KERNEL, NULL);
out:
	blkdev_put(bdev, mode);
	return err;
}

static int forge_bcb_write(const char *cmd, const char *status)
{
	return forge_bcb_rw(cmd, status, NULL);
}

/*
 * Manual knob for the BCB path, so both return paths can be tested one at
 * a time (RTC flag alone: BCB cleared here; BCB alone: RTC flag cleared via
 * pwrap_hal_v1.rtc_flag_test).
 *   echo 1 > /sys/module/forge_m681_marker/parameters/bcb_test   arm
 *   echo 0 > /sys/module/forge_m681_marker/parameters/bcb_test   clear
 *   cat /sys/module/forge_m681_marker/parameters/bcb_test        last write
 * Same para checks as the bootguard (GPT name "para" first). The getter
 * reports the last write of this boot (status string and error) without
 * touching the eMMC: a walk over the module parameters in sysfs must not open
 * the disk.
 */
static int forge_bcb_test_set(const char *val, const struct kernel_param *kp)
{
	int v, err = kstrtoint(val, 0, &v);

	if (err)
		return err;
	if (forge_virt)
		return -ENODEV;
	err = v ? forge_bcb_write("boot-recovery", "forge-bcb-test") :
		  forge_bcb_write(NULL, NULL);
	pr_emerg("[FORGE_M681] bcb_test: BCB %s (%d)\n",
		 v ? "boot-recovery armed" : "cleared", err);
	return err;
}

static int forge_bcb_test_get(char *buffer, const struct kernel_param *kp)
{
	const char *last;
	int err;

	mutex_lock(&forge_bcb_lock);
	last = forge_bcb_last;
	err = forge_bcb_last_err;
	mutex_unlock(&forge_bcb_lock);
	return scnprintf(buffer, PAGE_SIZE, "last write this boot: %s (%d)\n",
			 last, err);
}

static const struct kernel_param_ops forge_bcb_test_ops = {
	.set = forge_bcb_test_set,
	.get = forge_bcb_test_get,
};
module_param_cb(bcb_test, &forge_bcb_test_ops, NULL, 0600);

/*
 * The first USB configuration switches the deadline off, but a later
 * UDC rebind (e.g. `adb root`) may never bring the gadget back. So keep
 * watching while the BCB is still armed (userspace clears it on
 * sys.boot_completed): if the host loses the configuration and it does
 * not return within forge_usb_lost_deadline seconds, restart into
 * recovery. The warm restart also keeps the DRAM logs that a long Power
 * press would decay.
 */
static void forge_bootguard_usb_watch(void)
{
	unsigned long lost_since = 0;
	bool armed = true;
	int err;

	while (!kthread_should_stop()) {
		msleep(1000);
		if (forge_usb_configured) {
			if (lost_since)
				pr_emerg("[FORGE_M681] bootguard: USB configured again after %u ms\n",
					 jiffies_to_msecs(jiffies - lost_since));
			lost_since = 0;
			continue;
		}
		if (!lost_since) {
			lost_since = jiffies;
			forge_m681_mark_aux(0xDE, 0);
			pr_emerg("[FORGE_M681] bootguard: USB configuration lost - TWRP in %d s unless it returns\n",
				 forge_usb_lost_deadline);
			if (forge_usb_log_state)
				forge_usb_log_state("configuration lost");
			continue;
		}
		if (forge_usb_lost_deadline <= 0 ||
		    time_before(jiffies, lost_since + forge_usb_lost_deadline * HZ))
			continue;
		err = forge_bcb_rw(NULL, NULL, &armed);
		if (!err && !armed) {
			pr_emerg("[FORGE_M681] bootguard: USB lost, but userspace cleared the BCB (boot completed) - watch off\n");
			return;
		}
		if (forge_usb_log_state)
			forge_usb_log_state("restart");
		forge_m681_mark_aux(0xDF, (u32)err);
		pr_emerg("[FORGE_M681] bootguard: USB lost for %d s, BCB %s (%d) - restarting\n",
			 forge_usb_lost_deadline, err ? "unreadable" : "armed", err);
		emergency_restart();
		BUG();	/* restart declined: take the exception-reboot path */
	}
}

/*
 * Boot guard: arm the BCB as soon as the eMMC's para partition exists, so
 * any death after that (panic, init's fatal reboot, WDT) lands in
 * recovery (LK honours "boot-recovery" in p2 on a plain reboot; recovery
 * does not clear it). Userspace clears the message on sys.boot_completed;
 * a host configuring the USB gadget only switches the deadline off;
 * without it, reset at forge_boot_deadline seconds.
 */
static int forge_bootguard_fn(void *unused)
{
	unsigned long deadline = jiffies + forge_boot_deadline * HZ;
	int armed = 0, waited_logged = 0, err = -ENODEV;

	while (!kthread_should_stop()) {
		if (!armed && time_before(jiffies, deadline)) {
			err = forge_bcb_write("boot-recovery", "forge-bootguard");
			if (!err) {
				armed = 1;
				forge_m681_mark_aux(0xDB, 0);
				pr_emerg("[FORGE_M681] bootguard: BCB boot-recovery armed in %s p%d\n",
					 forge_bcb_disk, FORGE_BCB_PARTNO);
			}
		}
		if (&forge_usb_configured && forge_usb_configured) {
			/* Success milestone for the deadline only: the BCB stays armed until
			 * userspace clears it on sys.boot_completed, so a later death before
			 * that (init's fatal reboot, a crash loop) still ends in recovery.
			 * forge_bcb_clear_on_usb=1 clears it here instead, for images
			 * without that rc. */
			err = (armed && forge_bcb_clear_on_usb) ?
				forge_bcb_write(NULL, NULL) : 0;
			forge_m681_mark_aux(0xDC, (u32)err);
			pr_emerg("[FORGE_M681] bootguard: USB configured - deadline off, BCB %s (%d)\n",
				 !armed ? "was never armed" :
				 forge_bcb_clear_on_usb ? "cleared" :
				 "left for userspace (boot_completed)", err);
			forge_bootguard_usb_watch();
			return 0;
		}
		if (forge_boot_deadline <= 0) {
			pr_emerg("[FORGE_M681] bootguard: disabled at runtime, BCB %s\n",
				 armed ? "left armed" : "not armed");
			return 0;
		}
		if (time_after_eq(jiffies, deadline)) {
			if (!armed)
				err = forge_bcb_write("boot-recovery", "forge-deadline");
			forge_m681_mark_aux(0xDD, (u32)(armed ? 0 : err));
			pr_emerg("[FORGE_M681] boot deadline %ds: no USB configuration (BCB %s, %d) - restarting\n",
				 forge_boot_deadline, armed ? "armed" : "late write", err);
			emergency_restart();
			BUG();	/* restart declined: take the exception-reboot path */
		}
		if (!armed && !waited_logged &&
		    time_after(jiffies, deadline - (forge_boot_deadline - 5) * HZ)) {
			pr_emerg("[FORGE_M681] bootguard: no %s p%d (\"%s\") after 5 s (%d), still waiting\n",
				 forge_bcb_disk, FORGE_BCB_PARTNO, FORGE_BCB_PARTNAME, err);
			waited_logged = 1;
		}
		msleep(armed ? 1000 : 100);
	}
	return 0;
}

/*
 * Restart with an explicit target, in process context (reboot_notifier_list
 * runs in kernel_restart_prepare(), before device_shutdown()). "recovery"
 * and "bootloader" re-arm the BCB: arch_reset() hands those to
 * rtc_mark_recovery()/rtc_mark_fast(), and the RTC driver is not built,
 * so without this `reboot recovery` and init's fatal reboot (target
 * "bootloader") would both come back to the normal image. This LK has no
 * usable fastboot boot, so recovery is the useful landing for "bootloader"
 * too.
 */
static int forge_bcb_reboot_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	const char *cmd = data;
	int err;

	if (action != SYS_RESTART || !cmd ||
	    (strcmp(cmd, "recovery") && strcmp(cmd, "bootloader")))
		return NOTIFY_DONE;
	err = forge_bcb_write("boot-recovery", "forge-reboot");
	pr_emerg("[FORGE_M681] reboot %s: BCB boot-recovery %d\n", cmd, err);
	return NOTIFY_DONE;
}

static struct notifier_block forge_bcb_reboot_nb = {
	.notifier_call = forge_bcb_reboot_notify,
};

static int __init forge_bootguard_init(void)
{
	struct task_struct *t;

	if (!forge_virt)
		register_reboot_notifier(&forge_bcb_reboot_nb);

	if (forge_virt || forge_boot_deadline <= 0)
		return 0;
	t = kthread_run(forge_bootguard_fn, NULL, "forge_bootguard");
	pr_emerg("[FORGE_M681] bootguard started (%s), deadline %ds\n",
		 IS_ERR(t) ? "FAILED" : "ok", forge_boot_deadline);
	return 0;
}
late_initcall(forge_bootguard_init);

/*
 * Head log. The 3.5 KB FLOG ring keeps only the tail. This console copies
 * printk from the first line (CON_PRINTBUFFER replays the log buffer to it
 * alone at registration) into 252 KB of reserved, linear-mapped RAM and
 * stops when full. Each write is cleaned to DRAM so a warm reset keeps it.
 * The window sits next to scratch page A, which survives LK and recovery.
 * Layout: 0x7f000C00 u32 'HLOG', u32 length; text from 0x7f001000.
 * Read from recovery: dd if=/dev/mem bs=4096 skip=$((0x7f000000/4096)) count=64
 */
#define FORGE_HLOG_HDR_PHYS	0x7f000c00UL
#define FORGE_HLOG_PHYS		0x7f001000UL
#define FORGE_HLOG_SIZE		0x3f000UL
#define FORGE_HLOG_MAGIC	0x474f4c48U	/* 'HLOG' */

static u32 *forge_hlog_hdr;
static char *forge_hlog;
static u32 forge_hlog_len;

static void forge_head_write(struct console *con, const char *s,
			     unsigned int count)
{
	u32 room;

	if (!forge_hlog)
		return;
	room = FORGE_HLOG_SIZE - forge_hlog_len;
	if (count > room)
		count = room;
	if (!count)
		return;
	memcpy(forge_hlog + forge_hlog_len, s, count);
	__flush_dcache_area(forge_hlog + forge_hlog_len, count);
	forge_hlog_len += count;
	forge_hlog_hdr[1] = forge_hlog_len;
	__flush_dcache_area(forge_hlog_hdr, 8);
}

static struct console forge_head_console = {
	.name	= "forgehead",
	.write	= forge_head_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

/* setup_arch, after arm64_memblock_init(): keep the window from the allocator. */
void __init forge_m681_headlog_reserve(void)
{
	memblock_reserve(FORGE_HLOG_PHYS, FORGE_HLOG_SIZE);
}

/* setup_arch, after paging_init(): the window is reachable through the
 * linear map only if the LK handed it over as plain RAM. */
void __init forge_m681_headlog_init(void)
{
	if (forge_virt ||
	    !memblock_is_map_memory(FORGE_HLOG_HDR_PHYS) ||
	    !memblock_is_map_memory(FORGE_HLOG_PHYS) ||
	    !memblock_is_map_memory(FORGE_HLOG_PHYS + FORGE_HLOG_SIZE - 1))
		return;
	forge_hlog_hdr = phys_to_virt(FORGE_HLOG_HDR_PHYS);
	forge_hlog = phys_to_virt(FORGE_HLOG_PHYS);
	forge_hlog_hdr[0] = FORGE_HLOG_MAGIC;
	forge_hlog_hdr[1] = 0;
	__flush_dcache_area(forge_hlog_hdr, 8);
	register_console(&forge_head_console);
}

/*
 * Bring the two DRAM death channels up at the start of setup_arch instead
 * of after mm_init, so a death inside setup_arch leaves more than one
 * stage byte. Called right after forge_m681_virt_detect() (the qemu-virt
 * guard is settled by then):
 *  - registers the console on the early_ioremap'd marker page, so every
 *    printk from here on (incl. an Oops) lands in the 0x44800200 ring;
 *  - early_ioremaps toprgu into forge_wdt_base and registers the panic->SWRST
 *    notifier, so an early Oops warm-resets with the 0xF1 mark instead of
 *    hanging dark.
 * forge_m681_marker_late_init() swaps in its permanent mappings and skips
 * both registrations when they already happened here.
 */
static bool forge_console_on;
static bool forge_panic_nb_on;

void __init forge_m681_marker_early_console(void)
{
	if (forge_virt)
		return;
	if (forge_spm_base2 && !forge_console_on) {
		register_console(&forge_console);
		forge_console_on = true;
	}
	if (!forge_wdt_base)
		forge_wdt_base = early_ioremap(FORGE_WDT_PHYS_BASE,
					       FORGE_WDT_REGION_SIZE);
	if (forge_wdt_base && !forge_panic_nb_on) {
		atomic_notifier_chain_register(&panic_notifier_list,
					       &forge_m681_panic_nb);
		forge_panic_nb_on = true;
	}
	pr_emerg("[FORGE_M681] G1.2 early channels: console=%d panic-swrst=%d\n",
		 forge_console_on, forge_panic_nb_on);
}

void __init forge_m681_marker_late_init(void)
{
	void __iomem *nb;

	/* QEMU virt: skip everything here: marker ioremaps (unreserved guest
	 * RAM), console, wdt/gpt ioremaps (PCIe MMIO window), the direct cpuxgpt
	 * write and the forge_enable_cpuxgpt() SMC (no EL3 -> UNDEF). See
	 * forge_m681_virt_detect(). */
	if (forge_virt) {
		pr_info("[FORGE_M681] forge_virt: late_init skipped (no MTK hw)\n");
		return;
	}

	nb = ioremap(FORGE_MARKER_PHYS_BASE, FORGE_MARKER_REGION_SIZE);
	if (nb)
		forge_spm_base = nb;
	else
		pr_emerg("[FORGE_M681] marker late ioremap(0x%lx) FAILED, keeping early\n",
			 FORGE_MARKER_PHYS_BASE);

	nb = ioremap(FORGE_MARKER2_PHYS_BASE, FORGE_MARKER_REGION_SIZE);	/* one page: a larger ioremap aliases the linear map */
	if (nb)
		forge_spm_base2 = nb;
	else
		pr_emerg("[FORGE_M681] marker2 late ioremap(0x%lx) FAILED, keeping early\n",
			 FORGE_MARKER2_PHYS_BASE);

	/* Enable cpuxgpt via the secure SMC now, before
	 * time_init()/arch_counter_register() reads CNTVCT as the clocksource.
	 * Done after forge_spm_base2 is mapped so the verification diag
	 * (0x90/0xA4) lands. */
	forge_enable_cpuxgpt();

	/* Register the console. It logs into the marker page (forge_spm_base2
	 * @0x44800000 + 0x200), already mapped above, so all printk (including
	 * userspace init's /dev/kmsg writes) lands in DRAM that survives
	 * recovery. Read at phys 0x44800200. */
	if (forge_spm_base2 && !forge_console_on) {
		register_console(&forge_console);
		forge_console_on = true;
	}
	if (forge_console_on)
		pr_emerg("[FORGE_M681] forge console registered, log@0x%lx\n",
			 FORGE_MARKER2_PHYS_BASE + FORGE_LOG_OFF);

	/* Map the toprgu watchdog so do_one_initcall can kick it across the
	 * level 4/5/6 window where no kicker kthread runs yet. */
	forge_wdt_base = ioremap(FORGE_WDT_PHYS_BASE, FORGE_WDT_REGION_SIZE);
	if (!forge_wdt_base)
		pr_emerg("[FORGE_M681] wdt ioremap(0x%lx) FAILED\n",
			 FORGE_WDT_PHYS_BASE);

	/* RGU boot forensics: latch and decode the previous reset's cause and the
	 * request-line arming the preloader/LK handed over. Must run before
	 * forge_m681_wdt_arm() below (the MODE write may clear STA).
	 *   HWWDT          -> counter timeout: the system hung
	 *   THERMAL_DIRECT -> RGU thermal line fired
	 *   SPMWDT/SPM_THERMAL/SECURITY/DEBUG -> that requester reset the SoC
	 *   0x0            -> non-RGU reset (PMIC-level) or LK cleared STA. */
	if (forge_wdt_base) {
		u32 sta = readl(forge_wdt_base + FORGE_WDT_STA_OFF);
		u32 rq  = readl(forge_wdt_base + FORGE_WDT_REQ_MODE_OFF);

		forge_rgu_boot_sta = sta;
		forge_rgu_boot_reqmode = rq;
		pr_emerg("[FORGE_M681] #113 RGU boot forensics: STA=0x%08x [%s%s%s%s%s%s%s%s] REQ_MODE=0x%08x MODE=0x%x LEN=0x%x NONRST2=0x%08x\n",
			 sta,
			 (sta & 0x80000000U) ? "HWWDT " : "",
			 (sta & 0x40000000U) ? "SWWDT " : "",
			 (sta & 0x20000000U) ? "IRQWDT " : "",
			 (sta & 0x10000000U) ? "SECURITY " : "",
			 (sta & 0x00080000U) ? "DEBUG " : "",
			 (sta & 0x00040000U) ? "THERMAL_DIRECT " : "",
			 (sta & 0x00000002U) ? "SPMWDT " : "",
			 (sta & 0x00000001U) ? "SPM_THERMAL " : "",
			 rq,
			 readl(forge_wdt_base + FORGE_WDT_MODE_OFF),
			 readl(forge_wdt_base + FORGE_WDT_LENGTH_OFF),
			 readl(forge_wdt_base + FORGE_WDT_NONRST2_OFF));
		/* stash on the aux-marker channel */
		forge_m681_mark_aux(0xEB, sta);
		forge_m681_mark_aux(0xEC, rq);
	}

	/* Map the GPT block so forge_m681_wdt_kick can sample the GPT2 free-run
	 * counter (0x10008028) and check that it counts. */
	forge_gpt_base = ioremap(0x10008000UL, 0x100);

	/* Map topckgen/apmixed here (not just lazily in the keywatch), so the
	 * gpuclk snapshots cover the kbase-probe-time power-on too. */
	forge_topck_base = ioremap(0x10000000UL, 0x1000);
	forge_apmix_base = ioremap(0x1000C000UL, 0x1000);

	/* Also try a direct write of EN_CPUXGPT (MCUSYS 0x10200000 + CTL 0x670,
	 * bit0); it only sticks if MCUSYS is not ATF write-protected. The
	 * architected counter feeding get_cycles()/ktime is started by
	 * enable_cpuxgpt(), which nothing else calls here (the timer DT node
	 * binds the generic mtk_timer.c via "mediatek,mt6577-timer" instead of
	 * the mt6755 apxgpt driver that runs setup_syscnt()). The __delay()
	 * cpu_relax fallback (arch/arm64/lib/delay.c) stays as a backstop. */
	{
		void __iomem *cx = ioremap(0x10200000UL, 0x1000);

		if (cx) {
			u32 ctl = readl(cx + 0x670);

			writel(ctl | 0x1U, cx + 0x670);
			pr_emerg("[FORGE_M681] v91 cpuxgpt CTL 0x%x -> 0x%x (direct EN_CPUXGPT)\n",
				 ctl, readl(cx + 0x670));
			iounmap(cx);
		} else {
			pr_emerg("[FORGE_M681] v91 cpuxgpt ioremap(0x10200000) FAILED\n");
		}
	}

	pr_emerg("[FORGE_M681] marker late base=%p base2=%p wdt=%p\n",
		 forge_spm_base, forge_spm_base2, forge_wdt_base);
	forge_m681_mark(FORGE_STAGE_MARKER_LATE_INIT);

	/* Arm the toprgu HW watchdog using the post-mm_init forge_wdt_base
	 * mapped above, MODE read-modify-write only; see the
	 * forge_m681_wdt_arm() header. Do not call before that ioremap
	 * succeeds. */
	forge_m681_wdt_arm();

	/* Register the panic-notifier SWRST safety net after arming the WDT, so
	 * the marker engine and the toprgu mapping are live before any panic
	 * handler can fire (see forge_m681_panic_handler()). Uses the same
	 * forge_wdt_base mapping; toprgu_base from the (denied) driver probe is
	 * NULL. */
	if (!forge_panic_nb_on) {
		atomic_notifier_chain_register(&panic_notifier_list,
					       &forge_m681_panic_nb);
		forge_panic_nb_on = true;
	}
	pr_emerg("[FORGE_M681] v48 panic-SWRST recovery net registered\n");

	/* early_ioremap_reset() already retired the fixmap path before this
	 * point; the stale early mappings are abandoned, not iounmapped. */
}

/*
 * Timer check (late_initcall, reached if the kernel completes its
 * initcalls, before userspace can overwrite the marker page): whether the
 * GPT2 free-run counter (phys 0x10008028) counts and whether ktime
 * advances. Result -> diag 0xDC, with a 0xD2 sentinel in the high byte:
 *   bits31..24 = 0xD2  (sentinel: this late_initcall ran)
 *   bit23      = 1 if 0x10008028 delta != 0  (GPT2 counter counts)
 *   bit22      = 1 if ktime delta != 0        (ktime advances)
 *   bits21..0  = low 22 bits of the ktime delta in ns (magnitude)
 */
static int __init forge_timer_verify(void)
{
	void __iomem *g;
	u64 k1, k2;
	u32 c1 = 0, c2 = 0, dk, packed;
	volatile int spin;

	if (forge_virt)
		return 0;	/* QEMU virt: 0x10008000 is in the PCIe MMIO window */
	g = ioremap(0x10008000UL, 0x100);

	k1 = ktime_to_ns(ktime_get());
	if (g)
		c1 = readl(g + 0x28);
	for (spin = 0; spin < 3000000; spin++)
		cpu_relax();
	k2 = ktime_to_ns(ktime_get());
	if (g) {
		c2 = readl(g + 0x28);
		iounmap(g);
	}
	dk = (u32)(k2 - k1);
	packed = 0xD2000000u
		| ((c2 != c1) ? (1u << 23) : 0u)
		| ((dk != 0)  ? (1u << 22) : 0u)
		| (dk & 0x3FFFFFu);
	forge_m681_diag(0xDCu, packed);
	pr_emerg("[FORGE_M681] v118 timer_verify: GPT2 %u->%u (d=%u) ktime d=%u ns packed=0x%x\n",
		 c1, c2, c2 - c1, dk, packed);
	return 0;
}
late_initcall(forge_timer_verify);

/*
 * Persistent kernel-log dump to eMMC (survives a warm reset, a battery
 * pull and recovery, unlike the small DRAM console). A kthread
 * periodically writes the whole printk log buffer (log_buf_addr_get/len)
 * to the expdb partition (/dev/block/mmcblk0p4, MTK exception-debug),
 * prefixed with an 'FLOG' magic + length so it can be found from recovery:
 *   adb shell "dd if=/dev/block/mmcblk0p4 bs=1M count=2 of=/sdcard/klog.bin" ; pull ; strings.
 * Retries filp_open until eMMC enumerates. */
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/printk.h>

/* Each flush rewrites the entire kernel log ring (log_buf_len) to expdb
 * and vfs_fsync()s it. At a 400 ms period that is a continuous ~11 MB/s
 * of eMMC writes; the fsync serialises the eMMC queue, so the whole
 * system waits on storage, and it wears the flash.
 *
 * forge_klog_ms: 0 (default) = do not touch eMMC at all. >0 = flush period
 * in ms (400 = the original behaviour). Runtime-writable. The thread is
 * kept; only the flushing is gated. */
static int forge_klog_ms;
module_param(forge_klog_ms, int, 0644);
MODULE_PARM_DESC(forge_klog_ms,
		 "expdb klog flush period in ms (0 = off, default; 400 = historical bring-up behaviour)");

/* A module_param is only writable once userspace is up, and a boot that
 * dies never gets there, so the dump can also be armed from the kernel
 * command line. core_param gives a cmdline name with no module prefix;
 * the alias is short because LK truncates the cmdline at ~99 chars:
 *     fklog=400
 * arms the flush from the bootloader for one boot. */
core_param(fklog, forge_klog_ms, int, 0644);

static int forge_log_dump_fn(void *arg)
{
	const char *path = "/dev/block/mmcblk0p4";	/* expdb */
	char hdr[16];
	while (!kthread_should_stop()) {
		struct file *f;
		char *lb = log_buf_addr_get();
		u32 ll = log_buf_len_get();
		loff_t pos = 0;
		int period = forge_klog_ms;

		/* Gated OFF by default. Poll the knob cheaply so it can be armed
		 * at runtime; do NOT open or touch eMMC while disabled. */
		if (period <= 0) {
			msleep(1000);
			continue;
		}

		msleep(period);	/* faster flush to capture the final ~1s before reset */
		f = filp_open(path, O_RDWR | O_LARGEFILE, 0);
		if (IS_ERR(f))
			continue;	/* eMMC not ready yet -> retry */
		memcpy(hdr, "FLOGm681", 8);
		memcpy(hdr + 8, &ll, 4);
		memset(hdr + 12, 0, 4);
		kernel_write(f, hdr, sizeof(hdr), pos);
		pos = sizeof(hdr);
		if (lb && ll)
			kernel_write(f, lb, ll, pos);
		vfs_fsync(f, 0);
		filp_close(f, NULL);
	}
	return 0;
}

static int __init forge_log_dump_init(void)
{
	if (forge_virt)
		return 0;	/* QEMU virt: no expdb/mmcblk0p4, filp_open would retry forever */
	kthread_run(forge_log_dump_fn, NULL, "forge_klog");
	pr_emerg("[FORGE_M681] v126 eMMC klog dumper thread up, flushing GATED OFF (forge_klog_ms=%d; set >0 to arm, 400 = historical)\n",
		 forge_klog_ms);
	return 0;
}
late_initcall(forge_log_dump_init);

/*
 * Synchronous eMMC probe marker, for a display bus hang before
 * late_initcall: recovery overwrites both DRAM marker regions, SPM IO is
 * unreadable via /dev/mem (STRICT_DEVMEM), and forge_klog (async kthread)
 * only flushes after late_initcall. So a tiny record is written
 * synchronously (filp_open + kernel_write + fsync) to a fixed offset
 * (3MB) of expdb/p4, past the klog ring dump (offset 0), right before
 * each display driver's ->probe (called from really_probe,
 * drivers/base/dd.c). Read it back from recovery:
 *   dd if=/dev/block/mmcblk0p4 bs=1 skip=$((0x300000)) count=2048 | strings
 * The last "FEMK <seq> <name>" record = the driver whose probe hung.
 *
 * Filtered to display-family driver names so other probes pay only a
 * strstr loop, never eMMC IO. eMMC enumerates long before the display
 * stack probes, so filp_open succeeds by then (if it fails, the record is
 * skipped).
 */
static u32 forge_emmc_seq;
void forge_m681_emmc_mark(const char *tag)
{
	static const char * const disp_kw[] = {
		"disp", "DISP", "dsi", "mtkfb", "lcm", "smi", "m4u", "M4U",
		"mali", "kbase", "ged", "ddp", "mdp", "ovl", "rdma", "wdma",
		NULL };
	struct file *f;
	char buf[64];
	int n, i, hit = 0;
	loff_t pos;

	if (!tag || !*tag)
		return;
	for (i = 0; disp_kw[i]; i++)
		if (strstr(tag, disp_kw[i])) { hit = 1; break; }
	if (!hit)
		return;

	f = filp_open("/dev/block/mmcblk0p4", O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(f))
		return;			/* eMMC not up yet -> skip */
	pos = (loff_t)0x300000 + (loff_t)((forge_emmc_seq & 31u) * 64u);
	n = snprintf(buf, sizeof(buf), "FEMK %05u %s", forge_emmc_seq, tag);
	if (n < 0)
		n = 0;
	if (n > (int)sizeof(buf))
		n = sizeof(buf);
	if (n < (int)sizeof(buf))
		memset(buf + n, 0, sizeof(buf) - n);
	forge_emmc_seq++;
	kernel_write(f, buf, sizeof(buf), pos);
	vfs_fsync(f, 0);
	filp_close(f, NULL);
}
EXPORT_SYMBOL(forge_m681_emmc_mark);

/*
 * Heartbeat: a kernel timer fires in softirq every 500ms and stamps a
 * monotonic ktime ms + the CPU it ran on. It keeps firing until the
 * kernel is truly wedged or reset, so the last "HB ms=" in expdb tells
 * whether the SoC reset or the kernel only stopped printing, and when.
 * Same ktime scale as BOOTPROF ms.
 */
static struct timer_list forge_hb_timer;
static void forge_hb_fn(unsigned long data)
{
	u32 ms = (u32)(ktime_to_ns(ktime_get()) / 1000000);
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	pr_emerg("[FORGE_M681] HB ms=%u cpu=%d\n", ms, raw_smp_processor_id());

	/* last-heartbeat slot (0xFB tag | uptime in 100ms units): records the
	 * time of death even if every console page is lost. */
	if (forge_spm_base2)
		writel(0xFB000000U | ((ms / 100U) & 0x00FFFFFFU),
		       forge_spm_base2 + 0x7C);

	/* RGU request-line watch: if any reset-request line got (re-)armed
	 * since the last beat, log it and clear it again. The 500ms cadence
	 * timestamps a late armer (e.g. a userspace-driven trip rewrite) that a
	 * single late_initcall clear would miss. */
	if (b) {
		u32 rq = readl(b + FORGE_WDT_REQ_MODE_OFF);

		if (rq & FORGE_WDT_REQ_LINES) {
			pr_emerg("[FORGE_M681] #113 RGU watch: REQ_MODE=0x%08x armed at ms=%u — clearing 0x%08x\n",
				 rq, ms, rq & FORGE_WDT_REQ_LINES);
			writel((rq & ~FORGE_WDT_REQ_LINES) | FORGE_WDT_REQ_MODE_KEY,
			       b + FORGE_WDT_REQ_MODE_OFF);
		}
	}
	mod_timer(&forge_hb_timer, jiffies + msecs_to_jiffies(500));
}
static int __init forge_hb_init(void)
{
	init_timer(&forge_hb_timer);
	forge_hb_timer.function = forge_hb_fn;
	forge_hb_timer.data = 0;
	forge_hb_timer.expires = jiffies + msecs_to_jiffies(500);
	add_timer(&forge_hb_timer);
	pr_emerg("[FORGE_M681] v152 heartbeat timer armed\n");
	return 0;
}
late_initcall(forge_hb_init);

/*
 * RGU request-line quiesce. Every RGU reset-request line is cleared: none
 * of them is needed here (thermal is uncalibrated and already guarded; the
 * SPM firmware is not tuned for this board; the EINT debug key and
 * DEBUG/DFD paths are unused; SYSRST is modem-side and the modem is not
 * built), and each one can reset the SoC silently. The petted counter dog
 * stays armed, so a genuine hang still warm-resets with the markers
 * intact; only the request lines are cleared. Runs at late_initcall_sync
 * so it lands after tscpu_init's own WD_REQ_DIS and prints the pre-clear
 * value, i.e. what LK and all initcalls left armed. The heartbeat watch
 * (500ms) re-clears and timestamps any later re-armer.
 */
static int __init forge_rgu_quiesce_init(void)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;
	u32 rq;

	if (forge_virt || !b)
		return 0;
	rq = readl(b + FORGE_WDT_REQ_MODE_OFF);
	writel((rq & ~FORGE_WDT_REQ_LINES) | FORGE_WDT_REQ_MODE_KEY,
	       b + FORGE_WDT_REQ_MODE_OFF);
	pr_emerg("[FORGE_M681] #113 RGU quiesce: REQ_MODE 0x%08x -> 0x%08x (boot: STA=0x%08x REQ_MODE=0x%08x)\n",
		 rq, readl(b + FORGE_WDT_REQ_MODE_OFF),
		 forge_rgu_boot_sta, forge_rgu_boot_reqmode);
	return 0;
}
late_initcall_sync(forge_rgu_quiesce_init);

/*
 * ===== MT6351 PWRKEY/HOMEKEY long-press-reset gate + keywatch =====
 *
 * The PMIC hardware long-press reset can fire on a PWRKEY net that reads
 * "pressed" with nothing pressed: a PWRKEY press interrupt with no
 * release, and exactly RG_PWRKEY_RST_TD (11 s) later a PMIC-level reset
 * (RGU STA = 0). TOP_RST_MISC 0x02B6: HOMEKEY_RST_EN=bit8,
 * PWRKEY_RST_EN=bit9, TD=bits12-13 (stock MT6351 upmu_hw.h).
 *
 * Gate: clear only bits 8+9 via the TOP_RST_MISC_CLR (0x02BA)
 * write-1-to-clear register, atomic, so the WDTRSTB bits the RGU dog
 * needs are not disturbed. Trade-off: while this kernel runs, holding the
 * physical power button does not hardware-reset; the RGU dog remains the
 * hang-recovery path, and BROM/preloader key combos are unaffected (PMIC
 * defaults are restored by the reset itself). The keywatch samples
 * TOPSTATUS every 5 s.
 *
 * All addresses are MT6351 (stock upmu_hw.h), accessed via raw
 * pwrap_read/pwrap_write, not the pmic_* accessors and their MT6353
 * tables. Runs from a kthread (pwrap ops may sleep); the first gate write
 * lands at late_initcall.
 */
#define MT6351_TOPSTATUS_ADR		0x0220U	/* PWRKEY_DEB=bit1, HOMEKEY_DEB=bit2 (0 = pressed) */
#define MT6351_TOP_RST_MISC_ADR		0x02B6U
#define MT6351_TOP_RST_MISC_CLR_ADR	0x02BAU	/* W1C companion */
#define MT6351_RST_EN_BITS		0x0300U	/* bit8 HOMEKEY_RST_EN | bit9 PWRKEY_RST_EN */
#define MT6351_INT_STATUS0_ADR		0x02E0U	/* bit0 = PWRKEY press latch (both chips) */

/* PMIC reset-reason latches and rail probe. TOP_RST_MISC WDTRSTB_EN (b0)
 * and the sticky WDTRSTB_STATUS (b2), which nothing else clears: an RGU
 * dog bite is delivered to the PMIC and ends as a PMIC-level restart that
 * wipes RGU STA, so "STA=0" does not exclude a hang. Stock 3.10 reads
 * PMIC_WDTRSTB_STATUS as its "is_wdt_reboot" flag
 * (pmic_initial_setting.c). Read the latches, print, then clear them
 * (the stock mt6311.c pulse pattern), so the next boot reports which
 * mechanism ended this one:
 *   WDTRSTB_STATUS=1   -> RGU dog bite (the system hung)
 *   JUST_PWRKEY_RST=1  -> PMIC key long-press reset
 *   both 0             -> PMIC restarted on its own: UVLO/brownout/OC class;
 *                         the per-5s VCORE/VGPU/OC samples below help then.
 * All addresses are MT6351 (stock upmu_hw.h). */
#define MT6351_STRUP_CON1_ADR		0x0002U	/* PMU_THR_STATUS = bits 8..10 */
#define MT6351_STRUP_CON12_ADR		0x0018U	/* CLR_JUST_RST=b4 (pulse), JUST_PWRKEY_RST=b14 */
#define MT6351_TOP_RST_MISC_SET_ADR	0x02B8U	/* W1S companion */
#define MT6351_WDTRSTB_STATUS_CLR_BIT	0x0008U	/* TOP_RST_MISC b3: pulse to clear b2 */
#define MT6351_BUCK_OC_CON0_ADR		0x041EU	/* per-buck OC latches (VGPU=b1) */
#define MT6351_BUCK_VCORE_CON5_ADR	0x060AU	/* VCORE VOSEL_ON */
#define MT6351_BUCK_VGPU_CON2_ADR	0x0618U	/* VGPU EN */
#define MT6351_BUCK_VGPU_CON5_ADR	0x061EU	/* VGPU VOSEL_ON */

extern s32 pwrap_read(u32 adr, u32 *rdata);
extern s32 pwrap_write(u32 adr, u32 wdata);

static int forge_mt6351_keywatch_fn(void *unused)
{
	u32 pre = 0xDEAD, post = 0xDEAD, rst = 0, ts = 0, is0 = 0;
	s32 r1, r2;

	/* Enter marker before the first pwrap op, so a pwrap wedge shows where
	 * it happened. */
	pr_emerg("[FORGE_M681] #115 MT6351 LPRST gate ENTER (raw pwrap, authentic MT6351 addrs)\n");
	r1 = pwrap_read(MT6351_TOP_RST_MISC_ADR, &pre);
	r2 = pwrap_write(MT6351_TOP_RST_MISC_CLR_ADR, MT6351_RST_EN_BITS);
	pwrap_read(MT6351_TOP_RST_MISC_ADR, &post);
	pr_emerg("[FORGE_M681] #115 MT6351 LPRST gate: TOP_RST_MISC pre=0x%04x -> post=0x%04x (rd=%d wr=%d) PWRKEY_RST_EN was %u, HOMEKEY_RST_EN was %u, TD=%u\n",
		 pre, post, r1, r2,
		 (pre >> 9) & 0x1, (pre >> 8) & 0x1, (pre >> 12) & 0x3);

	/* Report why the previous boot ended (see the defines above), then clear
	 * the latches so the next boot reports this boot's end. `pre` still
	 * holds TOP_RST_MISC from before any of these writes. */
	{
		u32 s12 = 0, s1 = 0, oc0 = 0;

		pwrap_read(MT6351_STRUP_CON12_ADR, &s12);
		pwrap_read(MT6351_STRUP_CON1_ADR, &s1);
		pwrap_read(MT6351_BUCK_OC_CON0_ADR, &oc0);
		pr_emerg("[FORGE_M681] #116 PMIC reset-reason: WDTRSTB_STATUS=%u (1=prev reset was RGU-dog via WDTRSTB) JUST_PWRKEY_RST=%u THR_STATUS=0x%x BUCK_OC_CON0=0x%04x (STRUP_CON12=0x%04x STRUP_CON1=0x%04x)\n",
			 (pre >> 2) & 0x1, (s12 >> 14) & 0x1,
			 (s1 >> 8) & 0x7, oc0, s12, s1);
		/* pulse WDTRSTB_STATUS_CLR (b3) via the W1S/W1C companions */
		pwrap_write(MT6351_TOP_RST_MISC_SET_ADR, MT6351_WDTRSTB_STATUS_CLR_BIT);
		pwrap_write(MT6351_TOP_RST_MISC_CLR_ADR, MT6351_WDTRSTB_STATUS_CLR_BIT);
		/* pulse CLR_JUST_RST (STRUP_CON12 b4), RMW-preserving (no W1S/W1C
		 * companions on STRUP_CONs; stock mt6311.c uses the same pulse) */
		pwrap_write(MT6351_STRUP_CON12_ADR, s12 | (1u << 4));
		pwrap_write(MT6351_STRUP_CON12_ADR, s12 & ~(1u << 4));
	}

	/* Stream CLK_CFG_1 (0x10000050, mfg_sel bits 25:24) and APMIXED
	 * MMPLL_CON0/CON1 (0x1000C240/244), i.e. the GPU PLL and the topckgen
	 * mfg_sel mux state. Both blocks are always-on; reads are side-effect
	 * free. */
	{
		if (!forge_topck_base)
			forge_topck_base = ioremap(0x10000000UL, 0x1000);
		if (!forge_apmix_base)
			forge_apmix_base = ioremap(0x1000C000UL, 0x1000);
	}

	while (!kthread_should_stop()) {
		u32 vcore5 = 0, vgpu2 = 0, vgpu5 = 0, oc0 = 0;
		u32 rsta = 0xDEAD, rmode = 0xDEAD;
		u32 clkcfg1 = 0xDEAD, mmpll0 = 0xDEAD, mmpll1 = 0xDEAD;
		void __iomem *rb = toprgu_base ? toprgu_base : forge_wdt_base;

		pwrap_read(MT6351_TOPSTATUS_ADR, &ts);
		pwrap_read(MT6351_TOP_RST_MISC_ADR, &rst);
		pwrap_read(MT6351_INT_STATUS0_ADR, &is0);
		/* rail probe: VCORE/VGPU VOSEL_ON + VGPU EN + OC latches; a mis-set
		 * rail or an OC latch under GPU load shows here. */
		pwrap_read(MT6351_BUCK_VCORE_CON5_ADR, &vcore5);
		pwrap_read(MT6351_BUCK_VGPU_CON2_ADR, &vgpu2);
		pwrap_read(MT6351_BUCK_VGPU_CON5_ADR, &vgpu5);
		pwrap_read(MT6351_BUCK_OC_CON0_ADR, &oc0);
		/* stream the RGU state every 5 s */
		if (rb) {
			rsta = readl(rb + FORGE_WDT_STA_OFF);
			rmode = readl(rb + FORGE_WDT_MODE_OFF);
		}
		/* GPU clock-state snapshot (see the block comment above the loop) */
		if (forge_topck_base)
			clkcfg1 = readl(forge_topck_base + 0x50);
		if (forge_apmix_base) {
			mmpll0 = readl(forge_apmix_base + 0x240);
			mmpll1 = readl(forge_apmix_base + 0x244);
		}
		pr_emerg("[FORGE_M681] #115 keywatch ms=%u TOPSTATUS=0x%04x (pwr_deb=%u home_deb=%u; 0=pressed) RST_MISC=0x%04x INT_STA0=0x%04x VCORE_ON=0x%04x VGPU_EN=0x%04x VGPU_ON=0x%04x OC0=0x%04x RGU_STA=0x%08x RGU_MODE=0x%x CLK_CFG_1=0x%08x (mfg_sel=%u) MMPLL=0x%08x/0x%08x\n",
			 (u32)(ktime_to_ns(ktime_get()) / 1000000),
			 ts, (ts >> 1) & 0x1, (ts >> 2) & 0x1, rst, is0,
			 vcore5, vgpu2, vgpu5, oc0, rsta, rmode,
			 clkcfg1, (clkcfg1 >> 24) & 0x3, mmpll0, mmpll1);
		if (rst & MT6351_RST_EN_BITS) {
			/* on 4.9 PMIC writes are dropped (read-only
			 * WACS2), so the clear never sticks - say it once */
			static bool rearm_told;
			s32 wr = pwrap_write(MT6351_TOP_RST_MISC_CLR_ADR, MT6351_RST_EN_BITS);

			if (!rearm_told || wr == 0)
				pr_emerg("[FORGE_M681] #115 LPRST RE-ARMED (RST_MISC=0x%04x) — clearing again (wr=%d)\n",
					 rst, wr);
			rearm_told = true;
		}
		msleep(5000);
	}
	return 0;
}

static int __init forge_mt6351_keywatch_init(void)
{
	struct task_struct *t;

	if (forge_virt)
		return 0;	/* no pwrap/PMIC under qemu -M virt */
	t = kthread_run(forge_mt6351_keywatch_fn, NULL, "forge_keywatch");
	if (IS_ERR(t))
		pr_emerg("[FORGE_M681] #115 keywatch kthread FAILED (%ld)\n", PTR_ERR(t));
	return 0;
}
late_initcall(forge_mt6351_keywatch_init);

/*
 * Explicitly online secondary CPUs. On this platform boot-time smp_init
 * does not bring up secondaries; the MTK HPS governor was the only caller
 * of cpu_up(N>0), and HPS is disabled (hps_init returns early). Drive it
 * from late_initcall_sync (after SMP is set up, before userspace). Logs
 * the cpu masks and the cpu_up() return: present(1)=0 means
 * smp_prepare_cpus/cpu_prepare skipped it; cpu_up<0 means the
 * mt-boot/PSCI/spm_mtcmos power path failed (and which errno).
 */
#include <linux/cpu.h>
/* Off by default: the cpu_up path is the mt6735 mt-boot/PSCI cpu_ops,
 * not validated on MT6755, so the kernel boots single-core (maxcpus=1).
 * forge_smp_up=1 force-ups CPU1..7. */
static int forge_smp_up;
core_param(forge_smp_up, forge_smp_up, int, 0444);

static int __init forge_bringup_secondary(void)
{
	unsigned int cpu;
	int ret;

	if (!forge_smp_up) {
		pr_emerg("[FORGE_M681] secondary bring-up skipped (forge_smp_up=0), online=%u\n",
			 num_online_cpus());
		return 0;
	}

	pr_emerg("[FORGE_M681] v159 pre-bringup: possible=%u present=%u online=%u\n",
		 num_possible_cpus(), num_present_cpus(), num_online_cpus());
	/* Bring up all secondaries CPU1..7. Cluster1 big cores (4-7) are a
	 * separate power domain (bypass_cl1_armpll=1). Each cpu_up is logged
	 * before and after, so if a core wedges the bus the last
	 * "cpu_up(N) ->" line (no matching return) names it. */
	for (cpu = 1; cpu <= 7; cpu++) {
		pr_emerg("[FORGE_M681] v159 cpu_up(%u) ->\n", cpu);
		ret = cpu_up(cpu);
		pr_emerg("[FORGE_M681] v159 cpu_up(%u)=%d online_now=%u\n",
			 cpu, ret, num_online_cpus());
	}
	/* Keep all 8 online: freeze the online set after the force-up so
	 * nothing (HPS/PPM/idle/thermal) can cpu_down them, as the 3.18 kernel
	 * keeps 0-7 online. Frequencies stay PPM-managed. */
	cpu_hotplug_disable();
	pr_emerg("[FORGE_M681] v167 hotplug frozen, online=%u (pinned all cores)\n",
		 num_online_cpus());
	return 0;
}
late_initcall_sync(forge_bringup_secondary);

/*
 * Self-healing display gate: enabling the display stack can wedge the
 * boot. The gate state lives in the WDT NONRST_REG2 register (toprgu
 * 0x10007024), which survives a WDT warm reset and which the display
 * cannot touch (the DRAM region at 0x44800000 is overwritten by display
 * buffers). The mt6755 wdt driver only reads it. FORGE_WDT_NONRST2_OFF is
 * defined at the top with the other RGU offsets; the RGU boot forensics
 * line also prints it. See forge_display_gate_skip() below.
 */
#define FORGE_DISP_ATTEMPT	0xD15B0001u	/* attempting display this boot */
#define FORGE_DISP_SKIP		0xD15B0002u	/* last boot wedged -> skip (sticky) */
#define FORGE_DISP_DONE		0xD15BD09Eu	/* attempted and survived to late init */
int forge_skip_display;
EXPORT_SYMBOL(forge_skip_display);
static void __iomem *forge_disp_wdt(void)
{
	return forge_wdt_base ? forge_wdt_base : toprgu_base;
}
/* Always skip the gated display set: forge_display_deny[]
 * (of/platform.c) stays denied, the rest probes. forge_disp_decided, the
 * NONRST2 flag defines and forge_disp_wdt() are kept for
 * forge_disp_mark_ok. */
int forge_should_skip_display(void)
{
	return 1;
}
EXPORT_SYMBOL(forge_should_skip_display);
/*
 * Display gate, per build, in NONRST2 (the register survives reset ->
 * preloader -> LK -> recovery -> LK).
 *
 * The first display-stack driver init (SMI/CMDQ/dispsys/mtkfb/disp_mgr)
 * calls forge_display_gate_skip():
 *   NONRST2 == ATTEMPT(tag) or SKIP(tag) (tag = 16-bit hash of this
 *     kernel's banner): a previous boot of THIS image entered the display
 *     and never reached FORGE_DISP_DONE -> skip the display stack
 *     (NONRST2 := SKIP(tag), sticky for this image), mark 0xDB aux=previous.
 *   anything else (DONE, 0, another image's tag):
 *     attempt (NONRST2 := ATTEMPT(tag)), mark 0xDA aux=tag.
 * ATTEMPT(tag) = 0xDA000000|tag<<8, SKIP(tag) = 0xDC000000|tag<<8: bits 0-7
 * stay 0 (MTK reboot-mode bits of NONRST2 on other SoCs).
 * FORGE_DISP_DONE is written FORGE_DISP_OK_UPTIME_S after boot if the kernel
 * is still alive (below the 120 s boot deadline, so a USB/userspace deadline
 * does not blame the display; the deadline itself uses emergency_restart),
 * or on a clean kernel_restart/halt/power-off before that (adb reboot,
 * init's reboot): the kernel was alive, the display did not wedge it.
 * Panic, emergency_restart and HW-WDT resets leave ATTEMPT in place.
 * A new image (new banner) always retries.
 * Re-arm the same image from adb:
 *   echo 1 > /sys/module/forge_m681_marker/parameters/disp_gate_rearm
 * State: /sys/module/forge_m681_marker/parameters/disp_gate (-1 not asked,
 * 0 attempted, 1 skipped).
 */
#define FORGE_DISP_ATTEMPT_T(t)	(0xDA000000u | ((u32)(t) << 8))
#define FORGE_DISP_SKIP_T(t)	(0xDC000000u | ((u32)(t) << 8))
#define FORGE_DISP_OK_UPTIME_S	90
static int forge_disp_gate = -1;
module_param_named(disp_gate, forge_disp_gate, int, 0444);
static DEFINE_SPINLOCK(forge_disp_gate_lock);

static u32 forge_disp_build_tag(void)
{
	const char *p = linux_banner;
	u32 h = 0x811c9dc5u;	/* FNV-1a over the banner: #N, date and tag */

	while (*p)
		h = (h ^ (u8)*p++) * 0x01000193u;
	return (h ^ (h >> 16)) & 0xFFFFu;
}

int forge_display_gate_skip(const char *who)
{
	void __iomem *w;
	u32 prev = 0, tag;
	unsigned long flags;
	bool first = false;

	spin_lock_irqsave(&forge_disp_gate_lock, flags);
	if (forge_disp_gate < 0) {
		first = true;
		tag = forge_disp_build_tag();
		w = forge_virt ? NULL : forge_disp_wdt();
		if (w)
			prev = readl(w + FORGE_WDT_NONRST2_OFF);
		if (w && (prev == FORGE_DISP_ATTEMPT_T(tag) ||
			  prev == FORGE_DISP_SKIP_T(tag))) {
			forge_disp_gate = 1;
			forge_skip_display = 1;
			writel(FORGE_DISP_SKIP_T(tag), w + FORGE_WDT_NONRST2_OFF);
		} else {
			forge_disp_gate = 0;
			if (w)
				writel(FORGE_DISP_ATTEMPT_T(tag),
				       w + FORGE_WDT_NONRST2_OFF);
		}
	}
	spin_unlock_irqrestore(&forge_disp_gate_lock, flags);

	if (first) {
		if (forge_disp_gate)
			forge_m681_mark_aux(0xDB, prev);
		else
			forge_m681_mark_aux(0xDA, forge_disp_build_tag());
		pr_emerg("[FORGE_M681] display gate(NONRST2): prev=0x%08x tag=0x%04x -> %s (first: %s)\n",
			 prev, forge_disp_build_tag(),
			 forge_disp_gate ? "SKIP display stack" : "ATTEMPT",
			 who);
	} else if (forge_disp_gate) {
		pr_emerg("[FORGE_M681] display gate: %s skipped\n", who);
	}
	return forge_disp_gate;
}
EXPORT_SYMBOL(forge_display_gate_skip);

static void forge_disp_ok_fn(struct work_struct *work)
{
	void __iomem *w = forge_disp_wdt();

	if (w && forge_disp_gate == 0)
		writel(FORGE_DISP_DONE, w + FORGE_WDT_NONRST2_OFF);
	pr_emerg("[FORGE_M681] display gate(NONRST2): alive at %ds -> DONE\n",
		 FORGE_DISP_OK_UPTIME_S);
}
static DECLARE_DELAYED_WORK(forge_disp_ok_work, forge_disp_ok_fn);

/* a clean restart/halt/power-off (not panic, not a HW-WDT reset) proves the
 * kernel was alive: the attempted display did not wedge it */
static int forge_disp_reboot_nb_fn(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	void __iomem *w = forge_disp_wdt();

	if (w && forge_disp_gate == 0)
		writel(FORGE_DISP_DONE, w + FORGE_WDT_NONRST2_OFF);
	return NOTIFY_DONE;
}
static struct notifier_block forge_disp_reboot_nb = {
	.notifier_call = forge_disp_reboot_nb_fn,
};

static int forge_disp_gate_rearm_set(const char *val, const struct kernel_param *kp)
{
	void __iomem *w = forge_disp_wdt();

	if (w)
		writel(0, w + FORGE_WDT_NONRST2_OFF);
	pr_emerg("[FORGE_M681] display gate(NONRST2): re-armed (cleared) from userspace\n");
	return 0;
}
static const struct kernel_param_ops forge_disp_gate_rearm_ops = {
	.set = forge_disp_gate_rearm_set,
};
module_param_cb(disp_gate_rearm, &forge_disp_gate_rearm_ops, NULL, 0200);

static int __init forge_disp_mark_ok(void)
{
	void __iomem *w = forge_disp_wdt();
	long left;

	if (forge_disp_gate == 0) {
		/* the display was attempted: DONE only once it has run a while */
		left = FORGE_DISP_OK_UPTIME_S * HZ - (long)(jiffies - INITIAL_JIFFIES);
		schedule_delayed_work(&forge_disp_ok_work, left > 0 ? left : 0);
		register_reboot_notifier(&forge_disp_reboot_nb);
	} else if (forge_disp_gate < 0 && !forge_skip_display && w) {
		/* no display stack in this image: old late-init DONE */
		writel(FORGE_DISP_DONE, w + FORGE_WDT_NONRST2_OFF);
	}
	pr_emerg("[FORGE_M681] display gate(NONRST2): late-init reached, gate=%d skip=%d\n",
		 forge_disp_gate, forge_skip_display);
	return 0;
}
late_initcall(forge_disp_mark_ok);
