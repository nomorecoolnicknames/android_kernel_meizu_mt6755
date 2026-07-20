/*
 * One-shot MMPLL DDS setter standing in for the FHCTL frequency-hopping
 * driver while CONFIG_MTK_FREQ_HOPPING is off (its MT6755 driver pulls in
 * cpufreq_hybrid and dramc).
 *
 * mt_gpufreq reaches mt_dfs_mmpll() only for a same-postdiv frequency
 * switch; in this build at most the boot-time default-OPP switch (GED GPU
 * DVFS is off and no thermal throttle hook is wired). A silent no-op would
 * let gpufreq set the VGPU voltage for a frequency the PLL never took, so
 * this does the real DDS update as one write, like
 * mt_gpufreq_clock_switch_transient() does for cross-postdiv switches:
 * MMPLL_CON1 = PCW_CHG(bit31) | postdiv(26:24, preserved) | dds(20:0).
 * The glitch-free FHCTL slew is only needed for hops under load.
 *
 * Drop this file when CONFIG_MTK_FREQ_HOPPING is enabled (duplicate symbol
 * otherwise).
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
