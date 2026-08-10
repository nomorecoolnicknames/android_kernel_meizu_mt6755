/*
 * One-shot MMPLL DDS setter standing in for the FHCTL freq-hopping driver
 * while CONFIG_MTK_FREQ_HOPPING is off (its mt6755 driver drags in
 * cpufreq_hybrid and dramc).
 *
 * The real mt_gpufreq reaches mt_dfs_mmpll() only for a same-postdiv frequency
 * switch. In this build that happens at most from the boot-time default-OPP
 * switch: ged GPU DVFS (CONFIG_MTK_GPU_COMMON_DVFS_SUPPORT) is off and no
 * thermal throttle hook is wired, so there are no runtime hops. A silent no-op
 * here would be dangerous: gpufreq would then re-program the VGPU voltage for
 * a frequency the PLL never actually took — so this fallback performs the real
 * DDS update as a one-shot write, mirroring the in-tree pattern
 * mt_gpufreq_clock_switch_transient() uses for cross-postdiv switches:
 * MMPLL_CON1 = PCW_CHG(bit31) | postdiv(26:24, preserved) | dds(20:0).
 * FHCTL's gradual glitch-free slew is only needed for hops under load; at the
 * boot-time switch the GPU is powered off / jobless.
 *
 * Drop this file when CONFIG_MTK_FREQ_HOPPING is enabled (duplicate
 * symbol otherwise).
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/delay.h>

#define FH_APMIXED_COMPAT	"mediatek,apmixed"
#define FH_MMPLL_CON1_OFS	0x244	/* 0x1000C244, same reg mt_gpufreq.c:268 uses */

static void __iomem *fh_apmixed_base;

int mt_dfs_mmpll(unsigned int target_dds)
{
	void __iomem *con1;
	u32 old;

	if (!fh_apmixed_base) {
		struct device_node *node =
			of_find_compatible_node(NULL, NULL, FH_APMIXED_COMPAT);

		if (node)
			fh_apmixed_base = of_iomap(node, 0);
		if (!fh_apmixed_base) {
			pr_err("[FORGE_M681] p44-1 mt_dfs_mmpll: no apmixed map, dds 0x%x DROPPED\n",
			       target_dds);
			return -1;
		}
	}

	con1 = fh_apmixed_base + FH_MMPLL_CON1_OFS;
	old = readl(con1);
	writel(0x80000000u | (old & 0x07000000u) | (target_dds & 0x001FFFFFu),
	       con1);
	udelay(30);	/* PLL re-lock settle, mirrors FHCTL post-hop wait */
	pr_info("[FORGE_M681] p44-1 mt_dfs_mmpll one-shot: MMPLL_CON1 0x%08x -> 0x%08x\n",
		old, readl(con1));
	return 0;
}
EXPORT_SYMBOL(mt_dfs_mmpll);
