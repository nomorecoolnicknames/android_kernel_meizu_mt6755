/*
 * CPU side of the stock ATM cooler, without porting PPM.
 *
 * thermal/mt6755 builds with ATM_USES_PPM (tscpu_settings.h), so the ATM
 * cooler hands its CPU power budget to mt_ppm_cpu_thermal_protect() and
 * sizes it with mt_ppm_thermal_get_{cur,min,max}_power(). PPM is not built
 * here, so those were weak stubs (ap_thermal_limit.c, mtk_cooler_atm.c)
 * and the ATM CPU budget limited nothing.
 *
 * Stock PPM picks the best (cores, OPP) pair of both clusters from
 * cpu_tlp_power_tbl_FY (3.10 ppm/src/mach/mt6755/mt_ppm_power_table.h, the
 * same on M681 and L681) under the budget. That table is additive per
 * cluster: the 4+4-core rows equal the LL-only plus the L-only 4-core rows
 * (2371 = 613 + 1758, 1185 = 314 + 871). With all 8 cores kept online, this
 * uses exactly those per-cluster 4-core rows: the (LL, L) pair with the
 * highest stock perf index whose power fits the budget becomes the cpufreq
 * ceilings of forge_cpufreq_thermal_cap() (mt_cpufreq.c). It never takes
 * cores offline (stock could, below the 4+4 minimum). The lowest stock
 * rows (LL 494/338/156, L 286) are left out: the floor of the choice is
 * LL 598 + L 663 (256 + 412 = 668 mW).
 */
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/module.h>

struct forge_ppm_row {
	u32 khz;
	u16 power;	/* mW, 4 cores */
	u16 perf;
};

/* stock FY: LL_ONLY / L_ONLY rows with 4 cores */
static const struct forge_ppm_row forge_ppm_ll[] = {
	{ 1001000, 613, 2002 }, { 910000, 520, 1820 }, { 819000, 436, 1638 },
	{ 689000, 314, 1378 }, { 598000, 256, 1196 },
};

static const struct forge_ppm_row forge_ppm_l[] = {
	{ 1807000, 1758, 3614 }, { 1651000, 1498, 3302 }, { 1495000, 1263, 2990 },
	{ 1196000, 871, 2392 }, { 1027000, 709, 2054 }, { 871000, 569, 1742 },
	{ 663000, 412, 1326 },
};

/* mt_cpufreq.c */
extern void forge_cpufreq_thermal_cap(unsigned int ll_khz, unsigned int big_khz);

static unsigned int forge_ppm_budget;	/* mW, 0 = none */
module_param(forge_ppm_budget, uint, 0444);
static unsigned int forge_ppm_cap_ll, forge_ppm_cap_big;
module_param(forge_ppm_cap_ll, uint, 0444);
module_param(forge_ppm_cap_big, uint, 0444);

/* power of one cluster at khz: the stock row at or above it */
static unsigned int forge_ppm_power(const struct forge_ppm_row *t, int n,
				    unsigned int khz)
{
	int i;

	for (i = n - 1; i >= 0; i--)
		if (t[i].khz >= khz)
			return t[i].power;
	return t[0].power;
}

void mt_ppm_cpu_thermal_protect(unsigned int limited_power)
{
	int i, j, bi = -1, bj = -1;
	unsigned int best = 0;

	forge_ppm_budget = limited_power;
	if (limited_power) {
		for (i = 0; i < ARRAY_SIZE(forge_ppm_ll); i++)
			for (j = 0; j < ARRAY_SIZE(forge_ppm_l); j++) {
				unsigned int pw = forge_ppm_ll[i].power + forge_ppm_l[j].power;
				unsigned int pf = forge_ppm_ll[i].perf + forge_ppm_l[j].perf;

				if (pw <= limited_power && pf > best) {
					best = pf;
					bi = i;
					bj = j;
				}
			}
		if (bi < 0) {	/* below the 4+4 minimum: the lowest rows */
			bi = ARRAY_SIZE(forge_ppm_ll) - 1;
			bj = ARRAY_SIZE(forge_ppm_l) - 1;
		}
	}
	/* a top row means no thermal ceiling for that cluster */
	forge_ppm_cap_ll = bi > 0 ? forge_ppm_ll[bi].khz : 0;
	forge_ppm_cap_big = bj > 0 ? forge_ppm_l[bj].khz : 0;
	forge_cpufreq_thermal_cap(forge_ppm_cap_ll, forge_ppm_cap_big);
}

unsigned int mt_ppm_thermal_get_max_power(void)
{
	return forge_ppm_ll[0].power + forge_ppm_l[0].power;
}

unsigned int mt_ppm_thermal_get_min_power(void)
{
	return forge_ppm_ll[ARRAY_SIZE(forge_ppm_ll) - 1].power +
	       forge_ppm_l[ARRAY_SIZE(forge_ppm_l) - 1].power;
}

unsigned int mt_ppm_thermal_get_cur_power(void)
{
	unsigned int ll = cpufreq_quick_get(0), big = cpufreq_quick_get(4);

	if (!ll || !big)	/* cpufreq not armed yet */
		return mt_ppm_thermal_get_max_power();
	return forge_ppm_power(forge_ppm_ll, ARRAY_SIZE(forge_ppm_ll), ll) +
	       forge_ppm_power(forge_ppm_l, ARRAY_SIZE(forge_ppm_l), big);
}
