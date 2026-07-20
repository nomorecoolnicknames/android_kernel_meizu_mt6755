/*
 * Manual frequency switch for MT6755: big 806 -> 1196 MHz and LL -> 689 MHz
 * at the unchanged 1.000 V VPROC, the stock FY-table voltage for exactly
 * those steps. Ports only the raw frequency-switch recipe of mt_cpufreq.c
 * and does nothing at boot: no initcall, no MMIO until the first knob
 * write.
 *
 * Knob (/sys/module/mt_dvfs_p0/parameters/forge_dvfs_p0):
 *   echo 3  dump only: read both ARMPLL CON0/CON1, TOP_CKMUXSEL (big[5:4],
 *           LL[9:8], CCI src[13:12]), TOP_CKDIV1, INFRA CKDIV1 BIG/SML/BUS,
 *           CLK_MISC_CFG_0, no writes. Also latches the boot CON1 pair for
 *           echo 0.
 *   echo 2  big -> 663 MHz (down-step, safe at any voltage).
 *   echo 1  big -> 1196 MHz + LL -> 689 MHz.
 *   echo 0  restore the latched boot CON1 values.
 * CCI: both target rows carry cci_div=2 (half the source clock), so no CCI
 * write is needed; the dump records the CCI mux source before any write.
 *
 * Target CON1 values (_cpu_dds_calc, PLL_FREQ_STEP 13000, DDS_DIV1 0x9A000
 * at 1001 MHz, DDS_DIV2 0x010A0000 at 520 MHz):
 *   big 1196000: dds 0x0B8000 posdiv /1 -> write 0x800B8000 (bit31 = CHG)
 *   big  663000: vco 1326000 dds 0x0CC000 posdiv /2 -> write 0x810CC000
 *   LL   689000: vco 1378000 dds 0x0D4000 posdiv /2 -> write 0x810D4000
 * Switch recipe (stock): park the cluster mux on MAINPLL (transient
 * ~1092 MHz, legal at 1.000 V since < 1196), write DDS+POSDIV+CHG into
 * CON1, udelay(PLL_SETTLE_TIME = 20), mux back to ARMPLL.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define FORGE_TAG "[FORGE_DVFS0] "

/* apmixed offsets */
#define FORGE_BIG_CON0		0x200	/* ARMCA15PLL_CON0 */
#define FORGE_BIG_CON1		0x204	/* ARMCA15PLL_CON1 */
#define FORGE_LL_CON0		0x210	/* ARMCA7PLL_CON0  */
#define FORGE_LL_CON1		0x214	/* ARMCA7PLL_CON1  */
/* infracfg-ao offsets */
#define FORGE_TOP_CKMUXSEL	0x00	/* big[5:4] LL[9:8] cci[13:12] */
#define FORGE_TOP_CKDIV1	0x08
#define FORGE_CKDIV1_BIG	0x24
#define FORGE_CKDIV1_SML	0x28
#define FORGE_CKDIV1_BUS	0x2c
/* topckgen offset */
#define FORGE_CLK_MISC_CFG_0	0x104	/* [5:4] mainpll-path div bypass */

#define FORGE_MUX_ARMPLL	1
#define FORGE_MUX_MAINPLL	2
#define FORGE_PLL_SETTLE_US	20	/* PLL_SETTLE_TIME */

#define FORGE_BIG_CON1_1196	0x800B8000u
#define FORGE_BIG_CON1_663	0x810CC000u
#define FORGE_LL_CON1_689	0x810D4000u

static void __iomem *forge_apmixed;	/* mediatek,apmixed        */
static void __iomem *forge_infra;	/* mediatek,mt6755-infrasys */
static void __iomem *forge_topck;	/* mediatek,topckgen       */

static u32 forge_boot_con1_big;
static u32 forge_boot_con1_ll;
static bool forge_boot_latched;
static bool forge_pll_written;		/* any non-boot value written since boot */

int forge_dvfs_p0;	/* last accepted mode, default 0; non-static:
			 * mt_cpufreq's arming interlock reads it */
static DEFINE_MUTEX(forge_dvfs_lock);

static void __iomem *forge_map_one(const char *compat)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, compat);
	void __iomem *base = np ? of_iomap(np, 0) : NULL;

	if (!base)
		pr_emerg(FORGE_TAG "cannot map %s - knob DISABLED\n", compat);
	return base;
}

static int forge_dvfs_map(void)
{
	if (forge_apmixed && forge_infra && forge_topck)
		return 0;
	if (!forge_apmixed)
		forge_apmixed = forge_map_one("mediatek,apmixed");
	if (!forge_infra)
		forge_infra = forge_map_one("mediatek,mt6755-infrasys");
	if (!forge_topck)
		forge_topck = forge_map_one("mediatek,topckgen");
	return (forge_apmixed && forge_infra && forge_topck) ? 0 : -ENODEV;
}

/* Decode a CON1 value to kHz (posdiv shift in [26:24], dds below). */
static unsigned int forge_con1_khz(u32 con1)
{
	u32 dds = con1 & 0x001FFFFFu;
	u32 shift = (con1 >> 24) & 0x7;

	if (dds < 0x9A000u)
		return 0;
	return (1001000u + ((dds - 0x9A000u) >> 13) * 13000u) >> shift;
}

/* Stock _cpu_clock_switch (:1437-1470): open the CLK_MISC_CFG_0[5:4]
 * bypass before parking on MAINPLL, close it again once back on ARMPLL. */
static void forge_ckmux(int little, u32 sel)
{
	u32 v;

	if (sel == FORGE_MUX_MAINPLL) {
		v = readl(forge_topck + FORGE_CLK_MISC_CFG_0);
		writel(v | (0x3u << 4), forge_topck + FORGE_CLK_MISC_CFG_0);
		readl(forge_topck + FORGE_CLK_MISC_CFG_0);
	}
	v = readl(forge_infra + FORGE_TOP_CKMUXSEL);
	if (little)
		v = (v & ~(0x3u << 8)) | (sel << 8);
	else
		v = (v & ~(0x3u << 4)) | (sel << 4);
	writel(v, forge_infra + FORGE_TOP_CKMUXSEL);
	readl(forge_infra + FORGE_TOP_CKMUXSEL);
	if (sel == FORGE_MUX_ARMPLL) {
		v = readl(forge_topck + FORGE_CLK_MISC_CFG_0);
		writel(v & ~(0x3u << 4), forge_topck + FORGE_CLK_MISC_CFG_0);
		readl(forge_topck + FORGE_CLK_MISC_CFG_0);
	}
}

static void forge_pll_write(int little, u32 con1_val, const char *why)
{
	void __iomem *con1 = forge_apmixed + (little ? FORGE_LL_CON1 : FORGE_BIG_CON1);
	u32 old = readl(con1);

	forge_ckmux(little, FORGE_MUX_MAINPLL);
	writel(con1_val, con1);		/* DDS+POSDIV, bit31 = CHG */
	readl(con1);
	udelay(FORGE_PLL_SETTLE_US);
	forge_ckmux(little, FORGE_MUX_ARMPLL);
	pr_emerg(FORGE_TAG "%s CON1 0x%08x -> 0x%08x (%u kHz, %s)\n",
		 little ? "LL" : "big", old, readl(con1),
		 forge_con1_khz(readl(con1)), why);
}

static void forge_boot_latch(void)
{
	if (forge_boot_latched)
		return;
	forge_boot_con1_big = readl(forge_apmixed + FORGE_BIG_CON1);
	forge_boot_con1_ll = readl(forge_apmixed + FORGE_LL_CON1);
	forge_boot_latched = true;
	pr_emerg(FORGE_TAG "boot CON1 latched: big=0x%08x (%u kHz) LL=0x%08x (%u kHz)\n",
		 forge_boot_con1_big, forge_con1_khz(forge_boot_con1_big),
		 forge_boot_con1_ll, forge_con1_khz(forge_boot_con1_ll));
}

static void forge_dvfs_dump(void)
{
	u32 mux = readl(forge_infra + FORGE_TOP_CKMUXSEL);

	pr_emerg(FORGE_TAG "big CON0=0x%08x CON1=0x%08x (%u kHz)\n",
		 readl(forge_apmixed + FORGE_BIG_CON0),
		 readl(forge_apmixed + FORGE_BIG_CON1),
		 forge_con1_khz(readl(forge_apmixed + FORGE_BIG_CON1)));
	pr_emerg(FORGE_TAG "LL  CON0=0x%08x CON1=0x%08x (%u kHz)\n",
		 readl(forge_apmixed + FORGE_LL_CON0),
		 readl(forge_apmixed + FORGE_LL_CON1),
		 forge_con1_khz(readl(forge_apmixed + FORGE_LL_CON1)));
	pr_emerg(FORGE_TAG "TOP_CKMUXSEL=0x%08x (big=%u LL=%u cci_src=%u) TOP_CKDIV1=0x%08x\n",
		 mux, (mux >> 4) & 0x3, (mux >> 8) & 0x3, (mux >> 12) & 0x3,
		 readl(forge_infra + FORGE_TOP_CKDIV1));
	pr_emerg(FORGE_TAG "CKDIV1 big=0x%08x sml=0x%08x bus=0x%08x CLK_MISC_CFG_0=0x%08x\n",
		 readl(forge_infra + FORGE_CKDIV1_BIG),
		 readl(forge_infra + FORGE_CKDIV1_SML),
		 readl(forge_infra + FORGE_CKDIV1_BUS),
		 readl(forge_topck + FORGE_CLK_MISC_CFG_0));
}

static int forge_dvfs_p0_set(const char *val, const struct kernel_param *kp)
{
	extern int forge_mt_cpufreq_armed;	/* mt_cpufreq.c */
	int mode, ret;

	ret = kstrtoint(val, 0, &mode);
	if (ret)
		return ret;
	if (mode < 0 || mode > 3)
		return -EINVAL;

	/* stage-3 interlock: once the cpufreq governor owns the ARMPLLs, a
	 * manual CON1 write underneath it desynchronizes idx_opp_tbl from the
	 * silicon. One owner at a time; mode 3 (pure dump) stays allowed. */
	if (forge_mt_cpufreq_armed && mode != 3) {
		pr_emerg(FORGE_TAG "REFUSED mode %d: mt_cpufreq is armed and owns the PLLs\n",
			 mode);
		return -EBUSY;
	}

	mutex_lock(&forge_dvfs_lock);
	if (forge_dvfs_map()) {
		mutex_unlock(&forge_dvfs_lock);
		return -ENODEV;
	}
	forge_boot_latch();

	switch (mode) {
	case 3:
		forge_dvfs_dump();
		break;
	case 2:
		forge_pll_write(0, FORGE_BIG_CON1_663, "down-step discriminator");
		forge_pll_written = true;
		break;
	case 1:
		forge_pll_write(0, FORGE_BIG_CON1_1196, "phase-0 uplift");
		forge_pll_write(1, FORGE_LL_CON1_689, "phase-0 uplift");
		forge_pll_written = true;
		break;
	case 0:
		if (!forge_pll_written) {
			pr_emerg(FORGE_TAG "restore skipped: nothing written yet\n");
			break;
		}
		forge_pll_write(0, forge_boot_con1_big | BIT(31), "restore boot");
		forge_pll_write(1, forge_boot_con1_ll | BIT(31), "restore boot");
		break;
	}
	forge_dvfs_p0 = mode;
	mutex_unlock(&forge_dvfs_lock);
	return 0;
}

static const struct kernel_param_ops forge_dvfs_p0_ops = {
	.set = forge_dvfs_p0_set,
	.get = param_get_int,
};
module_param_cb(forge_dvfs_p0, &forge_dvfs_p0_ops, &forge_dvfs_p0, 0644);
MODULE_PARM_DESC(forge_dvfs_p0,
	"m681 DVFS phase-0: 3=dump-only 2=big 663MHz 1=big 1196+LL 689 0=restore boot (default 0 = inert)");
