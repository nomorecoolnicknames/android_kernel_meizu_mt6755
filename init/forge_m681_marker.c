// SPDX-License-Identifier: GPL-2.0
/*
 * forge_m681_marker.c - persistent DRAM stage markers for M681/L681 (MT6755).
 *
 * The kernel can die in early boot before the MTK ram_console driver
 * registers a console. Anything printed before that lives only in the
 * printk ring, which a WDT reset wipes, so a few breadcrumb words are
 * written to reserved DRAM that survives a warm reset.
 *
 * Two regions are written (see FORGE_MARKER_PHYS_BASE and
 * FORGE_MARKER2_PHYS_BASE below): the minirdump reserved node at
 * 0x444f0000 and the preloader reserved node at 0x44800000. Every mark
 * fills several guarded slots (signature, raw and inverted copies) in the
 * first 512 bytes, so a reader can cross-check them and ignore stale data.
 *
 * Readback from recovery (needs /dev/mem):
 *   dd if=/dev/mem bs=512 skip=$((0x444f0000/512)) count=1 | xxd
 *   +0x100: rolling stage (0xF681xxxx), +0x104: raw stage,
 *   +0x108: 0x46524745 ('FRGE') if this kernel ran.
 *
 * Timing: forge_m681_marker_early_init() must run after
 * early_ioremap_init() in setup_arch(); before that forge_m681_mark() is a
 * silent no-op. After mm_init() the marker switches to a permanent
 * ioremap(), so the late_initcall "early ioremap leak" check stays quiet.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/moduleparam.h>	/* forge_poweroff_veto knob */
#include <linux/notifier.h>	/* atomic_notifier_chain_register, panic_notifier_list */
#include <linux/notifier.h>	/* (kept; kernel.h pulls it but be explicit for the reader) */
#include <asm/early_ioremap.h>
#include <linux/of_fdt.h>	/* of_flat_dt_is_compatible for the forge_virt guard */
#include <linux/console.h>	/* console -> DRAM log that survives a warm reset */
#include <linux/string.h>
#include <linux/hrtimer.h>	/* WDT kicker from IRQ context (a kthread may never run) */
#include <linux/ktime.h>	/* ns_to_ktime, NSEC_PER_SEC */

#include "forge_m681_marker.h"

/*
 * Console: every kernel printk (including init's /dev/kmsg writes) is
 * copied into a ring inside the marker page (forge_spm_base2, 0x44800000),
 * which survives a WDT warm reset and the following recovery boot, unlike
 * the ram_console at 0x44400000. The preloader region has no "no-map", so
 * a separate ioremap of 0x44801000 is rejected as RAM; the ring uses the
 * free space after the marker slots instead (0x200..0xFFF, about 3.5 KB).
 * Layout at phys 0x44800200: u32 head, u32 magic 'FLOG', then the text.
 */
#define FORGE_LOG_OFF	0x200U				/* within the marker page */
#define FORGE_LOG_END	0x1000U				/* one page: a larger ioremap of this reserved RAM aliases the linear map with mismatched attributes (device vs cacheable) and dies early in start_kernel */
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

/* Primary marker: the minirdump reserved node at 0x444f0000 (size 0x10000,
 * "minirdump-reserved-memory@444f0000" in mt6755.dtsi). The lk-reserved
 * range at 0x46000000 is not usable: on the warm WDT -> recovery path lk
 * runs again and overwrites it. minirdump is reserved by both this kernel
 * and recovery and is written only by the minirdump driver on a crash
 * dump, so a mark survives lk and the driver probe phase (some recovery
 * builds do overwrite it, see the eMMC probe marker below).
 * Readback: dd if=/dev/mem bs=512 skip=$((0x444f0000/512)) count=1 */
#define FORGE_MARKER_PHYS_BASE	0x444f0000UL
#define FORGE_MARKER_REGION_SIZE	0x1000UL	/* one page for the header */

/*
 * Secondary marker: the preloader reserved node at 0x44800000
 * ("preloader-reserved-memory@44800000", size 0x100000). It is written
 * only by the preloader before lk and is reserved by both kernels, so it
 * gives an independent readout that survives the probe phase and the
 * recovery fallback. Not usable: 0x46100000 (overwritten by the recovery
 * ramdisk at 0x45000000) and the pstore node at 0x44410000 (rewritten by
 * the pstore driver during of_platform_populate).
 * Readback: dd if=/dev/mem bs=512 skip=$((0x44800000/512)) count=1
 */
#define FORGE_MARKER2_PHYS_BASE	0x44800000UL
#define FORGE_MARKER2_MAP_SIZE	0x8000UL	/* late ioremap size, within the 1 MB preloader-reserved region */

/*
 * MTK toprgu watchdog (0x10007000) kick channel.
 *
 * mtk_wdt_probe arms the WDT at postcore_initcall with a 30 s timeout and
 * kicks it once, but the kicker kthread starts only at late_initcall, so
 * initcall levels 4-6 run unkicked and the watchdog can cut a boot that is
 * still progressing. forge_m681_wdt_kick() is called once per initcall
 * from do_one_initcall() to reload the counter: a progressing boot reaches
 * /init, while an initcall that never returns stops the kicks, the
 * watchdog fires and the device warm-resets with the markers intact.
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

/* Manual SWRST backstop, see forge_m681_wdt_kick(). */
#define FORGE_WDT_SWRST_OFF	0x14U
#define FORGE_WDT_SWRST_KEY	0x1209U

/* RGU reset forensics and request-line quiesce.
 * WDT_STA latches which source caused the last warm reset; it is the only
 * record that tells a counter timeout (a hang: kickers starved) from a
 * request-line reset (thermal, SPM, EINT, debug: all silent, without any
 * kernel involvement). WDT_REQ_MODE shows which request lines may reset
 * the SoC right now, including whatever the preloader/lk left enabled.
 * Bit values match drivers/watchdog/mediatek/wdt/mt6755/mt_wdt.h. */
#define FORGE_WDT_STA_OFF	0x0CU
#define FORGE_WDT_NONRST2_OFF	0x24U
#define FORGE_WDT_REQ_MODE_OFF	0x30U
#define FORGE_WDT_REQ_MODE_KEY	0x33000000U
/* Every RGU reset-request line other than the counter itself:
 * SPM_THERMAL(b0) SPM_SCPSYS(b1) EINT(b2) SYSRST(b3) THERMAL_DIRECT(b18)
 * DEBUG(b19). None of the requesters is calibrated here (tscpu is
 * uncalibrated, the SPM firmware comes from MT6757/3.18, the EINT debug
 * key is unused), so a set bit is a source of silent reboots, not
 * protection. */
#define FORGE_WDT_REQ_LINES	(0x00000001U | 0x00000002U | 0x00000004U | \
				 0x00000008U | 0x00040000U | 0x00080000U)
/* The 3.18 tree has about 600 module initcalls; the threshold is far above
 * that, so the normal initcall flow never trips it, but a loop that keeps
 * re-entering do_one_initcall() cannot keep the WDT petted forever. */
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

/* Secondary marker base (preloader region at 0x44800000). */
void __iomem *forge_spm_base2;
EXPORT_SYMBOL(forge_spm_base2);

/*
 * forge_virt: QEMU virt guard.
 *
 * Under "qemu -M virt" every MTK physical address used here is wrong:
 * 0x444f0000/0x44800000 are plain guest RAM (writes there corrupt pages
 * the allocator hands out), 0x10006000/0x10007000/0x10008000/0x10200000
 * sit inside the virt PCIe MMIO window, and the raw "smc #0" in
 * forge_enable_cpuxgpt() is undefined at EL1 because virt has no EL3.
 *
 * forge_m681_virt_detect() is called from setup_arch() right after
 * setup_machine_fdt(), the earliest point where the flat DT is readable.
 * If the root compatible is "linux,dummy-virt" it sets forge_virt=1 and
 * clears both marker bases (the early fixmap slots are dropped as
 * early_ioremap_reset() would). The writes before detection (stages 0x1E
 * and 0x35) hit RAM that is not yet in memblock. Every later mark is a
 * no-op and forge_m681_marker_late_init() returns before any MTK ioremap
 * or the cpuxgpt SMC. MT6755 DTs never carry "linux,dummy-virt".
 */
int forge_virt;
EXPORT_SYMBOL(forge_virt);

/* Userspace power-off veto, consumed by sys_reboot (kernel/reboot.c).
 * 1 = reboot(2) POWER_OFF/HALT are logged and refused with -EPERM, so an
 * orderly Android shutdown cannot power the phone off; RESTART/RESTART2
 * are never vetoed. 0 = stock behaviour. In-kernel charger shutdowns have
 * their own gate (charging_hw_sm5414.forge_chg_poweroff). Runtime:
 * /sys/module/forge_m681_marker/parameters/forge_poweroff_veto */
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

/* RGU state latched at boot, before forge_m681_wdt_arm() writes MODE (a
 * MODE write may clear STA). Printed again by the late quiesce so the
 * values survive an expdb ring rollover. */
static u32 forge_rgu_boot_sta;
static u32 forge_rgu_boot_reqmode;

/* GPT2 free-running counter (phys 0x10008000, counter at +0x28), sampled in
 * forge_m681_wdt_kick() to check that it counts; that runs on every
 * initcall, so before any hang on a frozen timer. */
static void __iomem *forge_gpt_base;

/* topckgen and apmixedsys mappings for the GPU clock-state snapshot.
 * Mapped in forge_m681_marker_late_init() (start_kernel time), so the
 * power-on at kbase probe is observable too; the keywatch thread keeps a
 * lazy-map fallback. */
static void __iomem *forge_topck_base;
static void __iomem *forge_apmix_base;

/* Inline GPU clock snapshot: the 5 s keywatch cannot see the sub-second
 * window of the first GPU job, and the question is whether MMPLL_CON0.EN
 * is set while kbase_jd_submit runs. Callable from any context (three AO
 * readl()s and a pr_emerg), called from the first-N marker blocks at
 * submit and MFG-enable time. */
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
 * The watchdog driver's own DT-mapped (of_iomap) toprgu base, set by
 * mtk_wdt_probe() at postcore_initcall. The private start_kernel ioremap()
 * of 0x10007000 (forge_wdt_base) may be NULL or not functional that early
 * for SoC register space, so toprgu_base is preferred once it is set.
 */
extern void __iomem *toprgu_base;

/* Per-initcall kick counter, mirrored into the marker page at 0x40 (count)
 * and 0x44 (which bases were non-NULL), so a readback after a reset shows
 * whether the kick ran and how far the boot got. */
static u32 forge_wdt_kick_count;

/* Diagnostic offsets in the marker page (free; real slots start at 0x100). */
#define FORGE_DIAG_KICKCNT_OFF	0x40
#define FORGE_DIAG_KICKFLAGS_OFF	0x44
/* Initcall-level phase tracking. */
#define FORGE_DIAG_LEVEL_ENTER_BASE	0xB0
#define FORGE_DIAG_LEVEL_DONE_BASE	0xC0
#define FORGE_DIAG_LASTGOOD_SEQ_OFF	0xD0
#define FORGE_DIAG_LASTGOOD_SEQ_INV_OFF	0xD4
#define FORGE_DIAG_BOOTPHASE_OFF	0xD8
#define FORGE_DIAG_WDTK_HEARTBEAT_OFF	0xFC	/* kicker heartbeat; not 0xDC, which is LEVEL_DONE_BASE + 7 * 4 */
#define FORGE_DIAG_WDTK_INIT_OFF	0xE0	/* set once the kicker has run */
#define FORGE_DIAG_EMMC_CTR_BASE	0xE4	/* eMMC checkpoint counters, slot[0..5] = C7, C8, C9, CA, C3, C5 (0xE4..0xF8), below the marker slots at 0x100 */

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

	/* Write the slots only; no printk here. A printk that blocks
	 * (console_sem, logbuf, an undrained console) would stall the boot
	 * thread and hide the furthest stage reached, since the next mark
	 * would hang before recording. Plain writel() cannot block. */
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

/* Increment a checkpoint counter in the diag zone (0xE4..0xF8). The rolling
 * stage slots keep only the last marks; these counters show how many times
 * a checkpoint ran, e.g. "stuck at C7 (count 1, never C8)" vs. a C7 -> CA
 * retry loop running N times. Read-modify-write: bit errors may disturb the
 * low bits, but the magnitude (1 vs hundreds) is what matters. */
void forge_m681_bump(unsigned int slot)
{
	u32 off = FORGE_DIAG_EMMC_CTR_BASE + (slot & 0x7u) * 4u;

	if (forge_spm_base)
		writel(readl(forge_spm_base + off) + 1u, forge_spm_base + off);
	if (forge_spm_base2)
		writel(readl(forge_spm_base2 + off) + 1u, forge_spm_base2 + off);
}
EXPORT_SYMBOL(forge_m681_bump);

/* Write a raw u32 to a diag word (offset masked to the page). Unlike the
 * wrapping console ring, this keeps a value captured deep in an async
 * kworker (e.g. the VEMC pwrap readback). */
void forge_m681_diag(unsigned int off, u32 val)
{
	off &= 0x3FCu;
	if (forge_spm_base2)
		writel(val, forge_spm_base2 + off);
	if (forge_spm_base)
		writel(val, forge_spm_base + off);
}
EXPORT_SYMBOL(forge_m681_diag);

/* Record a fatal-fault snapshot from die(): PC goes to the slots as stage
 * 0xFE (aux = PC low 32 bits), LR and ESR to diag offsets 0x48/0x4c of the
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
 * Fault/panic net. die() (0xFE) only sees faults routed through it;
 * bad_mode -> panic and a direct panic() bypass it. These hooks record the
 * first fatal event from any sink (unhandled do_mem_abort = 0xFD,
 * bad_mode = 0xFC, panic = 0xFB); if nothing latches, the reset was an
 * IRQ-off hang.
 *
 * Raw slot writes only, no printk: the caller may hold locks or be wedged,
 * and printk could deadlock or fault again. A one-shot latch keeps the
 * first event instead of letting a later panic overwrite it.
 *
 * Diag layout of the snapshot:
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
 * Per-IRQ trace from the EL1 IRQ entry (gic_handle_irq): a free-running
 * count, the last hwirq and the interrupted PC go to diag offsets
 * 0x58/0x5c/0x60. Catches an IRQ storm (a peripheral asserting an
 * interrupt that no loaded driver clears yet): a huge count after the
 * reset plus the interrupted PC name the IRQ. No printk, writel() only;
 * a no-op until the marker region is mapped.
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
 * forge_m681_wdt_arm - arm the MTK toprgu watchdog in single-mode hardware
 * reset. Called once from forge_m681_marker_late_init() (after mm_init(),
 * before any initcall) through the post-mm_init ioremap of
 * FORGE_WDT_PHYS_BASE; an ioremap from start_kernel before mm_init() does
 * not work for SoC register space.
 *
 * mtk_wdt_probe() is not relied on for this: it calls request_irq() before
 * applying the mode, and if that path stalls the WDT stays in the
 * preloader's dual-mode + IRQ setting, where a later bus hang cannot
 * deliver the IRQ, so no reset happens at all.
 *
 * MODE is read-modify-written: clear DUAL_MODE (0x40) and IRQ (0x08), set
 * KEY | ENABLE | EXTEN | AUTO_RESTART. LENGTH is set to the maximum below,
 * then one RESTART reloads the counter to the full window.
 *
 * The 0xE6/0xE7 marks (MODE/LENGTH readback) record the armed state for a
 * readback after a reset; the driver's own pr_debug is not visible before
 * the console exists.
 */
static void forge_m681_wdt_arm(void)
{
	void __iomem *b = forge_wdt_base;
	u32 mode;

	if (!b)
		return;

	mode = readl(b + FORGE_WDT_MODE_OFF);
	/* Clear DUAL_MODE and IRQ (the preloader arms dual-mode + IRQ). ENABLE
	 * is already set by the preloader and is asserted again below. */
	mode &= ~(FORGE_WDT_MODE_DUAL_MODE | FORGE_WDT_MODE_IRQ);
	mode |= FORGE_WDT_MODE_KEY | FORGE_WDT_MODE_ENABLE |
		FORGE_WDT_MODE_EXTEN | FORGE_WDT_MODE_AUTO_RESTART;
	writel(mode, b + FORGE_WDT_MODE_OFF);

	/* Extend the timeout to the hardware maximum. The preloader LENGTH
	 * (0x5000) gives about 20 s, too short for the slow userspace start
	 * (mediaserver, agps, NVRAM restore) before boot_completed.
	 * MTK_WDT_LENGTH = (count[10:0] << 5) | KEY(0x08); 0x7FF is the maximum
	 * count. Written through the post-mm_init mapping used for MODE above
	 * and latched by the RESTART below. */
	writel((0x7FFU << 5) | FORGE_WDT_LENGTH_KEY, b + FORGE_WDT_LENGTH_OFF);

	/* pet once: reload the (now-extended) LENGTH counter to its full window */
	writel(FORGE_WDT_RESTART_KEY, b + FORGE_WDT_RESTART_OFF);

	/* Record the armed MODE/LENGTH in the marker slots, so a readback
	 * after a reset shows the WDT was armed before the hang. */
	forge_m681_mark_aux(0xE6, readl(b + FORGE_WDT_MODE_OFF));
	forge_m681_mark_aux(0xE7, readl(b + FORGE_WDT_LENGTH_OFF));

	pr_emerg("[FORGE_M681] v47 WDT armed single-mode-hwreset (RMW, no LENGTH reset): MODE=0x%x LENGTH=0x%x\n",
		 readl(b + FORGE_WDT_MODE_OFF),
		 readl(b + FORGE_WDT_LENGTH_OFF));
}

/*
 * forge_m681_wdt_disarm - disable the MTK toprgu watchdog.
 *
 * forge_m681_wdt_kick() pets the WDT only once per do_one_initcall(); when
 * do_initcalls() is done the kicks stop, and an armed 30 s watchdog resets
 * the device if nothing in userspace pets it. This writes MODE with the KEY
 * and ENABLE cleared. Not called at present (the ticker below keeps the
 * watchdog petted instead); a userspace hang would no longer reset.
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
 * Self-rearming WDT ticker driven by the jiffies tick. The per-initcall
 * kick (forge_m681_wdt_kick) only runs while do_initcalls() runs; after
 * the handover to userspace the kicks stop and the 30 s watchdog resets
 * the device. The timer wheel (timer_list) is serviced by the GPT jiffies
 * tick, so a self-rearming timer_list keeps the watchdog petted through
 * userspace. On a hard hang (tick stopped, IRQs off) the timer stops and
 * the watchdog still resets the device, so the DRAM markers stay usable.
 * Kicks every 8 s; armed at late_initcall (forge_wdt_ticker_late_init).
 */
static struct timer_list forge_wdt_ticker;
static int forge_wdt_ticker_armed;

static void __maybe_unused forge_wdt_ticker_fn(unsigned long data)
{
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	if (b) {
		/* Take the watchdog back from the stock WDK. The WDK switches it
		 * to dual-mode + IRQ (RGU_MODE 0x15 -> 0x5d) about 26 s into boot
		 * and sets its own shorter LENGTH; with that, a plain RESTART
		 * does not stop the bite about 30 s later. Re-assert single
		 * hardware-reset MODE and the maximum LENGTH on every tick, so
		 * the RESTART below reliably reloads the counter. The watchdog
		 * stays enabled, so a hang that stops the tick still resets. */
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
 * Arm the ticker at late_initcall, so the watchdog stays petted across the
 * kernel -> userspace handoff (after do_initcalls() the per-initcall kicks
 * stop). A hard hang (tick stopped, IRQs off) still resets the device with
 * the DRAM markers intact.
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

	/* The ticker is not started from here: it is armed at late_initcall
	 * (forge_wdt_ticker_late_init), so during the initcalls only these
	 * per-initcall kicks pet the watchdog and a hung initcall still
	 * resets. */

	/* Manual SWRST backstop. If kick_count climbs far past the realistic
	 * initcall count (see FORGE_WDT_FORCE_SWRESET_THRESHOLD) while
	 * system_state is still before SYSTEM_RUNNING, some caller is looping
	 * through do_one_initcall() and would pet the watchdog forever. Write
	 * the MTK_WDT_SWRST key directly: the same chip-wide warm reset as
	 * wdt_arch_reset(), which keeps the preloader-reserved marker at
	 * 0x44800000. A 0xEA mark right before the SWRST tells a forced reset
	 * from a normal watchdog timeout (which leaves the last initcall as
	 * aux). */
	if (system_state < SYSTEM_RUNNING &&
	    forge_wdt_kick_count >= FORGE_WDT_FORCE_SWRESET_THRESHOLD && b) {
		forge_m681_mark(0xEA);
		writel(FORGE_WDT_SWRST_KEY, b + FORGE_WDT_SWRST_OFF);
	}

	/* Breadcrumb: how many initcalls kicked, and which base was live
	 * (bit0 = our ioremap, bit4 = driver toprgu_base). */
	flags = 0xC0DE0000U | (toprgu_base ? 0x10 : 0) | (forge_wdt_base ? 0x01 : 0);
	if (forge_spm_base2) {
		writel(forge_wdt_kick_count, forge_spm_base2 + FORGE_DIAG_KICKCNT_OFF);
		writel(flags, forge_spm_base2 + FORGE_DIAG_KICKFLAGS_OFF);
	}
	if (forge_spm_base) {
		writel(forge_wdt_kick_count, forge_spm_base + FORGE_DIAG_KICKCNT_OFF);
		writel(flags, forge_spm_base + FORGE_DIAG_KICKFLAGS_OFF);
	}

	/* GPT2 counter probe, run on every initcall (well before any hang on
	 * a frozen timer in the device initcalls). Latches the first GPT2
	 * reading, then on every later kick writes the delta to diag 0xDC:
	 * 0xD2 in the high byte (probe ran), bit23 = counter moved, low 22
	 * bits = delta. Shows whether 0x10008028 actually counts. */
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
 * WDT kicker on an hrtimer. An hrtimer fires from the timer interrupt path,
 * independent of thread scheduling, so it pets the watchdog even when no
 * kicker thread gets CPU time. A real AXI/APB bus wedge stalls the CPU on
 * the unacknowledged MMIO access and takes no interrupts, so the hrtimer
 * stops too and the watchdog still resets the device. Pets the watchdog
 * directly (not through forge_m681_wdt_kick) to avoid that path's
 * pre-RUNNING SWRST backstop. Heartbeat in FORGE_DIAG_WDTK_HEARTBEAT_OFF.
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
	/* Arm the 5 s hrtimer kicker from the timer IRQ path, so the kernel
	 * stays up even when userspace never completes boot and never takes
	 * over the watchdog. Trade-off: a soft hang no longer resets; a bus
	 * wedge still does (the CPU stalls and the hrtimer stops). */
	if (forge_virt)
		return 0;	/* no MTK toprgu on QEMU virt */
	hrtimer_init(&forge_wdt_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	forge_wdt_hrtimer.function = forge_wdt_hrtimer_fn;
	hrtimer_start(&forge_wdt_hrtimer, ns_to_ktime(FORGE_WDT_KICK_NS),
		      HRTIMER_MODE_REL);
	pr_emerg("[FORGE_M681] v184 wdt hrtimer kicker RE-ENABLED (5s pet -> start holds long for Mali bring-up)\n");
	return 0;
}
core_initcall(forge_wdt_hrtimer_init);

/*
 * SPM power-domain state dump at late_initcall: prints the SPM power
 * config, PWR_STATUS/PWR_STATUS_2ND and the DIS, MM, MFG and ISP power
 * controls. Recovery cannot read SPM registers through /dev/mem
 * (STRICT_DEVMEM), so the live log is the channel for them.
 * mt_scpsys_init (CLK_OF_DECLARE "mediatek,mt6755-scpsys") powers DIS on
 * unconditionally at of_clk_init (clk-mt6755-pg.c), so any kernel that
 * gets this far should show DIS powered.
 */
static int __init forge_spm_dump(void)
{
	void __iomem *spm;
	u32 cfg, sta, sta2, dis, mfg, isp, mm;

	if (forge_virt)
		return 0;	/* 0x10006000 is in the QEMU virt PCIe MMIO window */
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
 * Panic black box: printk-ring tail in reserved DRAM.
 *
 * The panic net below issues SWRST from the first panic notifier (INT_MAX
 * is the highest priority and runs first), and panic() calls kmsg_dump()
 * only after the notifier chain. A panic whose text is stuck behind a held
 * console lock (the DISP VSync messages hold it for long stretches) would
 * leave its backtrace only in __log_buf, which the reset destroys. So
 * before the SWRST the last ~32 KB of the printk ring is pulled through
 * the kmsg_dump cursor API (dumper.active set by hand; locked variant: if
 * the panic happened while holding logbuf_lock this can deadlock, then the
 * armed watchdog still resets and the 0xF1 mark still says "panic") and
 * copied to preloader-reserved DRAM at 0x44810000 (marker page + 64 KB, no
 * other user). memremap(MEMREMAP_WB), not ioremap: a second Device-type
 * alias of this cacheable, linearly mapped region kills the kernel early;
 * WB matches the linear map, and __flush_dcache_area() pushes the bytes
 * to DRAM so they survive the warm reset.
 * Readback:
 *   dd if=/dev/mem of=/tmp/panic.bin bs=4096 skip=$((0x44810000/4096)) count=8
 *   header 'F681PANC' + u32 len + u32 ms; text starts at +16.
 */
#include <linux/kmsg_dump.h>
#include <linux/io.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>

#define FORGE_PANIC_DUMP_PHYS	0x44810000UL
#define FORGE_PANIC_DUMP_SIZE	0x8000UL
/* The last 256 bytes of the box hold the orderly-reboot record ('F681REBT'):
 * u32 magic x2, u32 ms, u32 action, comm[16] at +16, cmd[64] at +32.
 * The panic dump is capped at FORGE_REBT_OFF so the two cannot collide. */
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
 * Orderly-reboot black box. A userspace-commanded reboot (e.g. "reboot,
 * recovery" from init's critical-service handling or RescueParty) prints
 * "reboot: Restarting system with command ...", but that line can be lost
 * behind a held console lock just like panic text. kernel_restart_prepare()
 * passes the cmd string through the reboot notifier chain (kernel/reboot.c)
 * in the caller's context, so one record gives who (comm), what (cmd:
 * NULL / "recovery" / "bootloader") and when (ms). emergency_restart()
 * bypasses the chain; kernel/reboot.c calls forge_m681_emergency_mark() on
 * that path (action = 0xEE).
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
 * Panic notifier: direct SWRST.
 *
 * The toprgu watchdog timeout does not reliably reset this SoC (the
 * preloader even carries a "WDT does not trigger reboot" string), but a
 * direct SWRST_KEY write, as done by wdt_arch_reset() on panic, does. This
 * handler sits on panic_notifier_list, so any panic (BUG_ON, oops, OOM)
 * triggers that write without relying on machine_restart; the self-arm
 * and the manual SWRST backstop in forge_m681_wdt_kick() stay as further
 * fallbacks.
 *
 * Safe in atomic context: forge_m681_mark(0xF1) followed by a single
 * writel of MTK_WDT_SWRST_KEY (0x1209, as in wdt_arch_reset()) to
 * toprgu + 0x14. A one-shot latch keeps the first panic from being
 * shadowed by a later call. pr_emerg() is best effort.
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

	/* Copy the printk ring tail to reserved DRAM before the reset: it is
	 * the only copy of panic text held back by a console lock. 0xF1 is
	 * already latched above, so a wedge inside the dump still counts as
	 * a panic. */
	forge_panic_blackbox_dump();

	if (b)
		writel(FORGE_WDT_SWRST_KEY, b + FORGE_WDT_SWRST_OFF);

	/* If the SWRST_KEY write worked we never return. If it did not
	 * (toprgu unmapped, write blocked), the 0xF1 mark is still in place
	 * for the next reset (manual SWRST backstop in forge_m681_wdt_kick()
	 * or a battery pull). */
	return NOTIFY_DONE;
}

static struct notifier_block forge_m681_panic_nb = {
	.notifier_call = forge_m681_panic_handler,
	.priority = INT_MAX,	/* highest priority: runs first */
};

/*
 * "Current initcall" tracker that nothing else overwrites.
 *
 * The rolling slots hold only the last mark, and the platform-probe marks
 * (0xC0 attempt / 0xCD denylist skip) overwrite the per-initcall 0xE0 aux,
 * so after a hang inside an initcall that probes platform drivers the
 * marker names the last probe, not the initcall. These diag offsets are
 * written only here, so they always name the last initcall entered and
 * whether it returned. fn is mirrored to four offsets because single
 * copies in the preloader region (0x44800000) can lose bits; the reader
 * ORs them. seq is the do_one_initcall() ordinal.
 *
 * diag layout:  0x64/0x70/0x74/0x78 = current fn (OR these)   0x68 = seq
 *               0x6c = last fn that RETURNED (entered != returned => wedged)
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
	/* Mirror the last-good seq (0xD0) and its guard (0xD4), so a reader
	 * gets the seq of the last initcall that returned without combining
	 * 0x68 (current seq) and 0x6c (last done fn). */
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
 * of_platform_populate node tracker. A hang in of-core device creation
 * (not in a driver probe, so there is no 0xC0 mark) would otherwise leave
 * no trace; every node reaching of_platform_device_create_pdata() is
 * recorded, so after a reset the diag area names the node.
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
 * channel records 0xE8/0xE9 (aux = level), so a marker decode after a
 * reset names the level the boot stopped in (early/core/postcore/arch/
 * subsys/fs/device/late), next to the per-initcall fn tracker (0x64/0x6c)
 * that names the exact initcall.
 *
 * Per-level counts tell whether the boot stopped at the start of a level
 * (enter > 0, done = 0) or in the middle (done > 0, enter > done + 1).
 * The seq of the last initcall that returned is mirrored to diag 0xD0, so
 * the reader need not combine 0x68 (current seq) and 0x6c (last done fn).
 *
 * diag layout:
 *   0xB0 + level * 4 = enter count of that level (u32)
 *   0xC0 + level * 4 = done count of that level (u32; written after the
 *                      enter counts, so it overlays enter levels 4-7)
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

/* Direct SWRST_KEY write, using the same toprgu_base/forge_wdt_base
 * fallback as the kick path. */
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
 * Enable the MTK cpuxgpt (the ARM architected system counter) through the
 * secure SMC, as stock setup_syscnt() would if its calls were not
 * commented out (mt_gpt.c). Without it CNTVCT_EL0 stays frozen. The
 * cpuxgpt CTL is in write-protected MCUSYS (0x10200000): direct EL1 MMIO
 * writes are rejected, but writes through SMC MTK_SIP_KERNEL_MCUSYS_WRITE
 * (0x82000201) are serviced by the device's own MT6755 ATF at EL3.
 * Sequence (as __cpuxgpt_set_clk(CLK_DIV2) + __cpuxgpt_enable() in
 * mtk_cpuxgpt_mt6755.c): write INDEX_CTL(0), then CTL with clk = 13 MHz/DIV2
 * and EN (bit0). Reads of CTL (0x10200670) are plain MMIO; only writes are
 * protected. CNTVCT is read before and after: a nonzero delta shows that
 * the counter runs. Result in diag 0x90 (sentinel 0xCC |
 * CNTVCT-delta-nonzero << 23 | low bits) and 0xA4 (final CTL readback).
 */
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
void __init forge_m681_marker_late_init(void)
{
	void __iomem *nb;

	/* Under QEMU virt skip everything here: the marker ioremaps (plain
	 * guest RAM), the console, the WDT/GPT ioremaps (PCIe MMIO window),
	 * the direct cpuxgpt write and the forge_enable_cpuxgpt() SMC (no EL3,
	 * undefined). See forge_m681_virt_detect(). */
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

	nb = ioremap(FORGE_MARKER2_PHYS_BASE, FORGE_MARKER_REGION_SIZE);	/* one page, see FORGE_LOG_END */
	if (nb)
		forge_spm_base2 = nb;
	else
		pr_emerg("[FORGE_M681] marker2 late ioremap(0x%lx) FAILED, keeping early\n",
			 FORGE_MARKER2_PHYS_BASE);

	/* Enable cpuxgpt through the secure SMC now, before time_init() /
	 * arch_counter_register() use CNTVCT as the clocksource. Done after
	 * forge_spm_base2 is mapped so the diag words (0x90/0xA4) land. */
	forge_enable_cpuxgpt();

	/* Register the console. It logs into the marker page
	 * (forge_spm_base2 at 0x44800000 + 0x200), mapped above, so every
	 * printk, including init's /dev/kmsg writes, lands in DRAM that
	 * survives a warm reset. Phys address for readback: 0x44800200. */
	if (forge_spm_base2) {
		register_console(&forge_console);
		pr_emerg("[FORGE_M681] forge console registered, log@0x%lx\n",
			 FORGE_MARKER2_PHYS_BASE + FORGE_LOG_OFF);
	}

	/* Map the toprgu watchdog so do_one_initcall() can kick it across
	 * initcall levels 4-6, before any kicker thread runs. */
	forge_wdt_base = ioremap(FORGE_WDT_PHYS_BASE, FORGE_WDT_REGION_SIZE);
	if (!forge_wdt_base)
		pr_emerg("[FORGE_M681] wdt ioremap(0x%lx) FAILED\n",
			 FORGE_WDT_PHYS_BASE);

	/* RGU boot forensics: latch and decode the previous reset's cause and
	 * the request lines the preloader/lk left armed. Must run before
	 * forge_m681_wdt_arm() below (a MODE write may clear STA). When the
	 * same kernel boots again after a silent reset, this line classifies
	 * that reset:
	 *   HWWDT          -> counter timeout: the previous boot hung
	 *   THERMAL_DIRECT -> the RGU thermal line fired
	 *   SPMWDT/SPM_THERMAL/SECURITY/DEBUG -> that requester reset the SoC
	 *   0x0            -> non-RGU (PMIC-level) reset, or lk cleared STA. */
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
		/* Also stash STA on the aux-marker channel for readback. */
		forge_m681_mark_aux(0xEB, sta);
		forge_m681_mark_aux(0xEC, rq);
	}

	/* Map the GPT block so forge_m681_wdt_kick() can sample the GPT2
	 * free-running counter (0x10008028). */
	forge_gpt_base = ioremap(0x10008000UL, 0x100);

	/* Map topckgen/apmixed here, not only lazily in the keywatch, so GPU
	 * clock snapshots also cover the power-on at kbase probe. */
	forge_topck_base = ioremap(0x10000000UL, 0x1000);
	forge_apmix_base = ioremap(0x1000C000UL, 0x1000);

	/* Direct cpuxgpt enable (MCUSYS 0x10200000 + CTL 0x670, bit0 =
	 * EN_CPUXGPT) as a second attempt next to the SMC above. The
	 * architected counter behind get_cycles()/ktime is normally started by
	 * enable_cpuxgpt() in the mt6755 apxgpt driver (setup_syscnt()), but
	 * the timer DT node binds the generic mtk_timer.c through
	 * "mediatek,mt6577-timer" instead. The write only sticks if ATF does
	 * not write-protect MCUSYS; the __delay() cpu_relax fallback
	 * (arch/arm64/lib/delay.c) stays as a backstop. */
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

	/* Arm the toprgu watchdog through the post-mm_init forge_wdt_base
	 * mapped above (MODE read-modify-write, LENGTH to maximum); see
	 * forge_m681_wdt_arm(). Must not run before that ioremap succeeds. */
	forge_m681_wdt_arm();

	/* Register the panic-notifier SWRST net after arming the WDT, so the
	 * marker engine and the toprgu mapping are live before any panic
	 * handler can fire. See forge_m681_panic_handler(). It falls back to
	 * the same forge_wdt_base mapping while toprgu_base from the driver
	 * probe is NULL. */
	atomic_notifier_chain_register(&panic_notifier_list,
				       &forge_m681_panic_nb);
	pr_emerg("[FORGE_M681] v48 panic-SWRST recovery net registered\n");

	/* early_ioremap_reset() already retired the fixmap path before this
	 * point; the stale early mappings are abandoned, not iounmapped. */
}

/*
 * Timer check at late_initcall (reached whenever the initcalls complete,
 * and before userspace can overwrite the marker page). Measures whether
 * the GPT2 free-running counter (phys 0x10008028) counts and whether
 * ktime advances. Result in diag 0xDC, with a 0xD2 sentinel in the high
 * byte:
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
		return 0;	/* 0x10008000 is in the QEMU virt PCIe MMIO window */
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
 * Kernel log dump to eMMC: survives a warm reset, a battery pull and a
 * recovery boot, unlike the small DRAM console. A kthread periodically
 * writes the whole printk buffer (log_buf_addr_get/len) to the expdb
 * partition (/dev/block/mmcblk0p4, MTK exception debug), prefixed with an
 * 'FLOG' magic and the length:
 *   adb shell "dd if=/dev/block/mmcblk0p4 bs=1M count=2 of=/sdcard/klog.bin"
 * filp_open() is retried until the eMMC is enumerated. Debug only.
 */
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/printk.h>

/* Off by default. Each flush rewrites the entire log ring (log_buf_len,
 * 8 MB with the cmdline used here) to expdb and vfs_fsync()s it; at 400 ms
 * that is a continuous ~11 MB/s of writes, and the fsync serialises the
 * eMMC queue, so the whole system waits on storage (high iowait and load,
 * plus flash wear on a 10 MB partition). The thread stays and only the
 * flushing is gated, so it can be armed for a debugging session.
 *
 * forge_klog_ms: 0 (default) = do not touch the eMMC at all; > 0 = flush
 * period in ms (400 = the old behaviour). Runtime-writable.
 */
static int forge_klog_ms;
module_param(forge_klog_ms, int, 0644);
MODULE_PARM_DESC(forge_klog_ms,
		 "expdb klog flush period in ms (0 = off, default; 400 = historical bring-up behaviour)");

/* A module_param is only writable once userspace is up, and a boot that
 * dies never gets there, so the period is also a core_param with a short,
 * unprefixed cmdline name (LK truncates the cmdline at ~99 characters):
 *     fklog=400
 * arms the dump from the bootloader for one boot. Arm it before trying a
 * change that may prevent boot; otherwise expdb only holds a stale flush. */
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
		return 0;	/* no expdb on QEMU virt: filp_open would retry forever */
	kthread_run(forge_log_dump_fn, NULL, "forge_klog");
	pr_emerg("[FORGE_M681] v126 eMMC klog dumper thread up, flushing GATED OFF (forge_klog_ms=%d; set >0 to arm, 400 = historical)\n",
		 forge_klog_ms);
	return 0;
}
late_initcall(forge_log_dump_init);

/*
 * Synchronous eMMC probe marker, for a display bus hang.
 *
 * The DRAM marker regions can be overwritten in recovery, SPM registers
 * are not readable through /dev/mem (STRICT_DEVMEM), and forge_klog
 * flushes only from late_initcall, which a display wedge before
 * late_initcall never reaches. So a small record is written synchronously
 * (filp_open + kernel_write + fsync) to a fixed offset (3 MB) of expdb,
 * past the klog dump at offset 0, right before each display driver's
 * ->probe (called from really_probe(), drivers/base/dd.c). After a wedge
 * and recovery:
 *   dd if=/dev/block/mmcblk0p4 bs=1 skip=$((0x300000)) count=2048 | strings
 * The last "FEMK <seq> <name>" record is the driver whose probe hung.
 *
 * Only display-family driver names are recorded, so other probes pay only
 * a strstr loop, never eMMC I/O. The eMMC enumerates long before the
 * display stack probes, so filp_open succeeds by then; if it fails the
 * record is skipped.
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
 * Heartbeat: a kernel timer fires in softirq every 500 ms and logs the
 * monotonic ms and the CPU it ran on. It keeps firing until the kernel is
 * really wedged or reset, so the last "HB ms=" line in expdb tells whether
 * the SoC reset or the kernel only stopped printing (framework logs go to
 * logcat, not dmesg), and when. Same ktime scale as BOOTPROF. Debug only.
 */
static struct timer_list forge_hb_timer;
static void forge_hb_fn(unsigned long data)
{
	u32 ms = (u32)(ktime_to_ns(ktime_get()) / 1000000);
	void __iomem *b = toprgu_base ? toprgu_base : forge_wdt_base;

	pr_emerg("[FORGE_M681] HB ms=%u cpu=%d\n", ms, raw_smp_processor_id());

	/* Last-HB slot (0xFB tag | uptime in 100 ms units) in the preloader
	 * region: dates the stop even if every console page is lost. */
	if (forge_spm_base2)
		writel(0xFB000000U | ((ms / 100U) & 0x00FFFFFFU),
		       forge_spm_base2 + 0x7C);

	/* RGU request-line watch: if any reset-request line was (re)armed
	 * since the last beat, log it with the time and clear it again. The
	 * 500 ms cadence catches a late armer (e.g. a userspace-driven trip
	 * rewrite) that a single late_initcall clear would miss. */
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
 * RGU request-line quiesce. None of the RGU reset-request lines is needed
 * here (thermal is uncalibrated and guarded separately, the SPM firmware
 * comes from MT6757/3.18, the EINT debug key and DEBUG/DFD paths are
 * unused, SYSRST is modem-side and the modem is not built), and any of
 * them can reset the SoC silently, so they are all cleared. The watchdog
 * counter itself stays armed, so a real hang still warm-resets with the
 * markers intact. Runs at late_initcall_sync, after tscpu_init's own
 * WD_REQ_DIS, and prints the value before clearing: what lk and all
 * initcalls left armed. The 500 ms HB watch clears again and logs any
 * later re-armer.
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
 * MT6351 PWRKEY/HOMEKEY long-press reset gate and keywatch.
 *
 * With RGU STA = 0 across repeated resets, the reset came from the PMIC (a
 * full MT6351 restart also clears RGU STA). The PMIC saw a PWRKEY press
 * (debounced level = pressed in TOPSTATUS 0x0220) and no release, while
 * the AP keypad matrix reported keys on KP_COL0/COL1 pressed, and the reset
 * followed 11 s after the press: exactly the MT6351 RG_PWRKEY_RST_TD
 * long-press reset delay (TOP_RST_MISC 0x02B6: HOMEKEY_RST_EN = bit8,
 * PWRKEY_RST_EN = bit9, TD = bits12-13, per the stock 3.10 upmu_hw.h). The
 * PMIC long-press reset acts on a key net that reads held under this
 * kernel; the 3.18 kernel on the same board and lk does not see it (it
 * also skips long_press_reboot_function_setting). INT_STATUS0/INT_CON0 are
 * at 0x02E0/0x02C2 on both MT6351 and MT6353.
 *
 * Gate: clear only bits 8 and 9 through the write-1-to-clear
 * TOP_RST_MISC_CLR (0x02BA), atomically, without touching the WDTRSTB bits
 * the RGU watchdog needs. Trade-off: while this kernel runs, holding the
 * power button does not hardware-reset the phone; the RGU watchdog stays
 * the hang recovery path, and BROM/preloader key combos are unaffected
 * (the PMIC defaults come back with the next reset). The keywatch's 5 s
 * TOPSTATUS samples show when the PWRKEY net starts reading pressed.
 *
 * All addresses are MT6351 (stock upmu_hw.h), accessed with raw
 * pwrap_read/pwrap_write, not the pmic_* accessors and their MT6353
 * tables. Runs from a kthread (pwrap ops may sleep); the first gate write
 * lands at late_initcall (~10 s), well before the press window (~32 s).
 */
#define MT6351_TOPSTATUS_ADR		0x0220U	/* PWRKEY_DEB=bit1, HOMEKEY_DEB=bit2 (0 = pressed) */
#define MT6351_TOP_RST_MISC_ADR		0x02B6U
#define MT6351_TOP_RST_MISC_CLR_ADR	0x02BAU	/* W1C companion */
#define MT6351_RST_EN_BITS		0x0300U	/* bit8 HOMEKEY_RST_EN | bit9 PWRKEY_RST_EN */
#define MT6351_INT_STATUS0_ADR		0x02E0U	/* bit0 = PWRKEY press latch (both chips) */

/* PMIC reset-reason latches and rail probe. TOP_RST_MISC also carries
 * WDTRSTB_EN (b0) and the sticky WDTRSTB_STATUS (b2), which nothing else
 * clears: an RGU watchdog bite is delivered to the PMIC and ends as a
 * PMIC-level restart that also wipes RGU STA, so STA = 0 does not rule
 * out a watchdog reset. Stock 3.10 reads PMIC_WDTRSTB_STATUS as its
 * "is_wdt_reboot" flag (pmic_initial_setting.c). The latches are read,
 * printed and then cleared (the pulse pattern of stock mt6311.c), so each
 * boot reports how the previous one ended:
 *   WDTRSTB_STATUS=1   -> RGU watchdog bite (the previous boot hung)
 *   JUST_PWRKEY_RST=1  -> PMIC key long-press reset
 *   both 0             -> PMIC restarted on its own: UVLO/brownout/OC
 *                         class; the 5 s VCORE/VGPU/OC samples below help.
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

	/* Mark entry before the first pwrap op, so a pwrap stall shows up
	 * right here in the log. */
	pr_emerg("[FORGE_M681] #115 MT6351 LPRST gate ENTER (raw pwrap, authentic MT6351 addrs)\n");
	r1 = pwrap_read(MT6351_TOP_RST_MISC_ADR, &pre);
	r2 = pwrap_write(MT6351_TOP_RST_MISC_CLR_ADR, MT6351_RST_EN_BITS);
	pwrap_read(MT6351_TOP_RST_MISC_ADR, &post);
	pr_emerg("[FORGE_M681] #115 MT6351 LPRST gate: TOP_RST_MISC pre=0x%04x -> post=0x%04x (rd=%d wr=%d) PWRKEY_RST_EN was %u, HOMEKEY_RST_EN was %u, TD=%u\n",
		 pre, post, r1, r2,
		 (pre >> 9) & 0x1, (pre >> 8) & 0x1, (pre >> 12) & 0x3);

	/* Report why the previous boot ended (see the defines above), then
	 * clear the latches so the next boot reports this one.
	 * `pre` still holds TOP_RST_MISC from before any of our writes. */
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

	/* The 4.4 build links only the gpufreq stub, so nothing programs
	 * MMPLL (the GPU PLL) or the topckgen mfg_sel mux, and the first GPU
	 * job runs on the clock state lk left. Stream CLK_CFG_1 (0x10000050,
	 * mfg_sel bits 25:24) and APMIXED MMPLL_CON0/CON1 (0x1000C240/244).
	 * Both blocks are always-on; the reads have no side effects. */
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
		/* Rail probe: VCORE/VGPU VOSEL_ON, VGPU EN and the OC latches;
		 * a mis-set rail or an OC latch under GPU load shows up here
		 * within 5 s. */
		pwrap_read(MT6351_BUCK_VCORE_CON5_ADR, &vcore5);
		pwrap_read(MT6351_BUCK_VGPU_CON2_ADR, &vgpu2);
		pwrap_read(MT6351_BUCK_VGPU_CON5_ADR, &vgpu5);
		pwrap_read(MT6351_BUCK_OC_CON0_ADR, &oc0);
		/* Stream the RGU state every 5 s; the last sample before
		 * silence is the closest look at the RGU at reset time. */
		if (rb) {
			rsta = readl(rb + FORGE_WDT_STA_OFF);
			rmode = readl(rb + FORGE_WDT_MODE_OFF);
		}
		/* GPU clock-state snapshot (see the comment above the loop) */
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
			pr_emerg("[FORGE_M681] #115 LPRST RE-ARMED (RST_MISC=0x%04x) — clearing again\n", rst);
			pwrap_write(MT6351_TOP_RST_MISC_CLR_ADR, MT6351_RST_EN_BITS);
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
 * Bring up the secondary CPUs explicitly. Boot-time smp_init() does not
 * online them on this platform: the MTK HPS governor was the only caller
 * of cpu_up(N > 0), and HPS is disabled (hps_init returns early). Runs at
 * late_initcall_sync (SMP fully set up, before userspace). Logs the CPU
 * masks and every cpu_up() return: present(N) = 0 means
 * smp_prepare_cpus/cpu_prepare skipped the CPU; cpu_up() < 0 means the
 * mt-boot/PSCI/spm_mtcmos power path failed (with the errno).
 */
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/forge_hpl.h>
/*
 * forge_hp_pin - runtime switch for the hotplug freeze.
 * /sys/module/forge_m681_marker/parameters/forge_hp_pin (0644, default 1).
 * 1 = cpu_hotplug_disable() at late_initcall_sync, cpuN/online writes fail
 *     with EBUSY. 0 = hotplug allowed (L681 SODI needs only CPU0 powered,
 *     mt_idle.c).
 * cpu_hotplug_disable()/enable() in kernel/cpu.c is a counter
 * (cpu_hotplug_disabled ++/--, shared with the PM suspend notifier), so at
 * most one reference is held and tracked in forge_hp_frozen: enable is
 * called only to drop our own reference, disable only when we hold none.
 * Before late_initcall_sync a write (e.g. cmdline
 * forge_m681_marker.forge_hp_pin=0) only records the value.
 */
static int forge_hp_pin = 1;
static bool forge_hp_frozen;	/* we hold one cpu_hotplug_disable() ref */
static bool forge_hp_ready;	/* forge_bringup_secondary() has run */
static DEFINE_MUTEX(forge_hp_lock);

/* HPS-lite asks whether the hotplug freeze is held (lockless read) */
bool forge_hp_pinned(void)
{
	return READ_ONCE(forge_hp_frozen);
}

static int forge_hp_pin_set(const char *val, const struct kernel_param *kp)
{
	int v, ret;

	ret = kstrtoint(val, 0, &v);
	if (ret)
		return ret;
	if (v != 0 && v != 1)
		return -EINVAL;
	mutex_lock(&forge_hp_lock);
	forge_hp_pin = v;
	if (forge_hp_ready) {
		if (!v && forge_hp_frozen) {
			cpu_hotplug_enable();
			forge_hp_frozen = false;
			pr_emerg("[FORGE_M681] h1 forge_hp_pin=0: hotplug unfrozen, online=%u\n",
				 num_online_cpus());
		} else if (v && !forge_hp_frozen) {
			forge_hpl_restore_all();	/* never freeze with CPUs down */
			cpu_hotplug_disable();
			forge_hp_frozen = true;
			pr_emerg("[FORGE_M681] h1 forge_hp_pin=1: hotplug frozen, online=%u\n",
				 num_online_cpus());
		}
	}
	mutex_unlock(&forge_hp_lock);
	return 0;
}

static const struct kernel_param_ops forge_hp_pin_ops = {
	.set = forge_hp_pin_set,
	.get = param_get_int,
};
module_param_cb(forge_hp_pin, &forge_hp_pin_ops, &forge_hp_pin, 0644);

static int __init forge_bringup_secondary(void)
{
	unsigned int cpu;
	int ret;

	pr_emerg("[FORGE_M681] v159 pre-bringup: possible=%u present=%u online=%u\n",
		 num_possible_cpus(), num_present_cpus(), num_online_cpus());
	/* Bring up all secondaries, CPU1..7. Cluster 1 (CPU4-7) is a separate
	 * power domain (bypass_cl1_armpll=1). Each cpu_up() is logged before
	 * and after, so if a core wedges the bus the last "cpu_up(N) ->" line
	 * without a matching return names it. */
	for (cpu = 1; cpu <= 7; cpu++) {
		pr_emerg("[FORGE_M681] v159 cpu_up(%u) ->\n", cpu);
		ret = cpu_up(cpu);
		pr_emerg("[FORGE_M681] v159 cpu_up(%u)=%d online_now=%u\n",
			 cpu, ret, num_online_cpus());
	}
	/* Pin all 8 CPUs online. HPS is active on this kernel and would
	 * offline idle secondaries; the stock 3.18 kernel keeps CPU0-7 online,
	 * so the online set is frozen after bring-up and nothing (HPS, PPM,
	 * idle, thermal) can cpu_down them. Frequencies stay PPM-managed as on
	 * 3.18; the cluster-1 DVFS table error printed here is benign (3.18
	 * prints the same message). */
	mutex_lock(&forge_hp_lock);
	forge_hp_ready = true;
	if (forge_hp_pin) {
		cpu_hotplug_disable();
		forge_hp_frozen = true;
		pr_emerg("[FORGE_M681] v167 hotplug frozen, online=%u (pinned all cores)\n",
			 num_online_cpus());
	} else {
		pr_emerg("[FORGE_M681] v167 hotplug left enabled (forge_hp_pin=0), online=%u\n",
			 num_online_cpus());
	}
	mutex_unlock(&forge_hp_lock);
	return 0;
}
late_initcall_sync(forge_bringup_secondary);

/*
 * Display gate. Enabling the whole display stack (M4U/SMI/Mali/DDP/DSI/
 * mtkfb) at once could wedge the boot. A flag in the WDT NONRST_REG2
 * register (toprgu 0x10007024: always-on, survives a WDT warm reset, only
 * read by the mt6755 wdt driver) records the attempt across a reset:
 *   cleared / DONE -> this boot attempts the display (flag := ATTEMPT)
 *   ATTEMPT        -> the previous boot never cleared it, i.e. it wedged:
 *                     skip the display (flag := SKIP) and come up on adb
 *   SKIP           -> sticky skip until the flag is cleared
 * forge_disp_mark_ok() (late_initcall) sets DONE only if this boot
 * attempted the display and reached late init. Retry from adb:
 *   busybox devmem 0x10007024 32 0   (then reboot)
 * DRAM (0x44800000) cannot hold the flag: the display's own buffers
 * overwrite it once the display is enabled. FORGE_WDT_NONRST2_OFF is
 * defined with the other RGU offsets at the top.
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
/*
 * The flag-based auto-recovery is shelved: the WDT cannot be read reliably
 * this early (of_platform runs from setup_arch, before a usable toprgu
 * mapping) and the flag does not survive the preloader reset path.
 * Instead whatever is listed in forge_display_deny[] (of/platform.c) stays
 * denied and the rest probes, so this always returns "deny the listed
 * set". forge_disp_decided, the NONRST2 flag defines and forge_disp_wdt()
 * are kept for forge_disp_mark_ok() and a future reliable store.
 */
int forge_should_skip_display(void)
{
	return 1;
}
EXPORT_SYMBOL(forge_should_skip_display);
static int __init forge_disp_mark_ok(void)
{
	void __iomem *w = forge_disp_wdt();

	if (!forge_skip_display && w)
		writel(FORGE_DISP_DONE, w + FORGE_WDT_NONRST2_OFF);
	pr_emerg("[FORGE_M681] display gate(NONRST2): late-init reached, skip=%d\n",
		 forge_skip_display);
	return 0;
}
late_initcall(forge_disp_mark_ok);
