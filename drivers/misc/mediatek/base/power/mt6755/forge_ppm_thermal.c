/*
 * CPU side of the stock ATM cooler without the PPM port.
 *
 * thermal/mt6755 builds with ATM_USES_PPM (tscpu_settings.h), so the ATM
 * cooler hands its CPU power budget to mt_ppm_cpu_thermal_protect() and
 * sizes it with mt_ppm_thermal_get_{cur,min,max}_power(). PPM is not built
 * here (mt_misc_stubs.c), so without this file those are the weak "E_WF"
 * stubs and the ATM CPU budget limits nothing.
 *
 * Stock PPM picks the best (cores, OPP) pair of both clusters from
 * cpu_tlp_power_tbl_FY (stock ppm/src/mach/mt6755/mt_ppm_power_table.h)
 * under the budget. That table is additive per cluster: the 4+4-core rows
 * equal the LL-only plus the L-only 4-core rows (2371 = 613 + 1758,
 * 1185 = 314 + 871), and its 4+4 rows pair only equal OPP levels: VPROC
 * is shared, so a mixed pair would run the slower cluster at the faster
 * one's voltage, above the table's power. With all 8 cores kept online,
 * the shim takes the highest equal-level pair whose power fits the budget
 * and turns it into policy ceilings through forge_cpufreq_thermal_cap()
 * (mt_cpufreq.c); while a budget is set, LL stays at or below the stock
 * 1001 (no overclock row). It never takes cores offline (stock could,
 * below the 4+4 minimum; then the lowest rows are the ceiling).
 *
 * Rows the frequency-only table hides (L 286, LL 494/338/156) are left
 * out. The LL overclock rows are not in the stock table; their power is
 * scaled from LL 1001 at the same 1.150 V (dynamic power ~ f), which is
 * an estimate, not a stock figure.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/module.h>

struct forge_ppm_row {
	u32 khz;
	u16 power;	/* mW, 4 cores */
	u16 perf;
};

/* stock FY: LL_ONLY / L_ONLY rows with 4 cores, OPP 0..7 */
static const struct forge_ppm_row forge_ppm_ll[] = {
	{ 1001000, 613, 2002 }, { 910000, 520, 1820 }, { 819000, 436, 1638 },
	{ 689000, 314, 1378 }, { 598000, 256, 1196 },
	/* 494/338/156 hidden (clk/CCI dividers) */
};

static const struct forge_ppm_row forge_ppm_l[] = {
	{ 1807000, 1758, 3614 }, { 1651000, 1498, 3302 }, { 1495000, 1263, 2990 },
	{ 1196000, 871, 2392 }, { 1027000, 709, 2054 }, { 871000, 569, 1742 },
	{ 663000, 412, 1326 },
	/* 286 hidden (clk/CCI dividers) */
};

extern void forge_cpufreq_thermal_cap(unsigned int ll_khz, unsigned int big_khz)
	__attribute__((weak));

static unsigned int forge_ppm_budget;	/* mW, 0 = none */
module_param(forge_ppm_budget, uint, 0444);
static unsigned int forge_ppm_cap_ll, forge_ppm_cap_big;
module_param(forge_ppm_cap_ll, uint, 0444);
module_param(forge_ppm_cap_big, uint, 0444);

/* power of one cluster at khz: the stock row at or above it; LL above 1001
 * (K9) scaled from 1001 */
static unsigned int forge_ppm_power(const struct forge_ppm_row *t, int n,
				    unsigned int khz)
{
	int i;

	if (khz > t[0].khz)
		return DIV_ROUND_UP(t[0].power * khz, t[0].khz);
	for (i = n - 1; i >= 0; i--)
		if (t[i].khz >= khz)
			return t[i].power;
	return t[0].power;
}

void mt_ppm_cpu_thermal_protect(unsigned int limited_power)
{
	int i, bi = -1, bj = -1;

	forge_ppm_budget = limited_power;
	if (limited_power) {
		/* equal levels, fastest first (perf falls with the level) */
		for (i = 0; i < ARRAY_SIZE(forge_ppm_ll); i++)
			if (forge_ppm_ll[i].power + forge_ppm_l[i].power <= limited_power) {
				bi = bj = i;
				break;
			}
		if (bi < 0) {	/* below the 4+4 minimum: the lowest rows */
			bi = ARRAY_SIZE(forge_ppm_ll) - 1;
			bj = ARRAY_SIZE(forge_ppm_l) - 1;
		}
	}
	/* L at its top row needs no ceiling; LL keeps 1001 while budgeted */
	forge_ppm_cap_ll = bi >= 0 ? forge_ppm_ll[bi].khz : 0;
	forge_ppm_cap_big = bj > 0 ? forge_ppm_l[bj].khz : 0;
	if (forge_cpufreq_thermal_cap)
		forge_cpufreq_thermal_cap(forge_ppm_cap_ll, forge_ppm_cap_big);
}

/* any online CPU of the cluster (cores may be offline by hand) */
static unsigned int forge_ppm_cluster_khz(unsigned int first)
{
	unsigned int cpu;

	for (cpu = first; cpu < first + 4; cpu++)
		if (cpu_online(cpu))
			return cpufreq_quick_get(cpu);
	return 0;
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
	unsigned int ll = forge_ppm_cluster_khz(0), big = forge_ppm_cluster_khz(4);

	if (!ll && !big)	/* cpufreq not armed yet */
		return mt_ppm_thermal_get_max_power();
	return (ll ? forge_ppm_power(forge_ppm_ll, ARRAY_SIZE(forge_ppm_ll), ll) : 0) +
	       (big ? forge_ppm_power(forge_ppm_l, ARRAY_SIZE(forge_ppm_l), big) : 0);
}
