/*
 * MT6755 vcore DVFS link stubs.
 *
 * The vcore DVFS subsystem (governor, manager, SPM vcore-dvfs) is not
 * built, but the eMMC/SDIO driver (msdc autok_dvfs, pulled in because sd.c
 * calls emmc_execute_dvfs_autok) references two vcorefs entry points.
 * No-op stubs let eMMC run at the fixed boot Vcore OPP:
 *
 *   - is_vcorefs_can_work() returns 0  -> msdc treats DVFS as unavailable and
 *     skips the DVFS autok path, tuning at the fixed boot voltage.
 *   - vcorefs_request_dvfs_opp() returns 0 -> any stray OPP request is a no-op.
 *
 * Drop mt_vcorefs_stub.o from the Makefile when mt_vcorefs_governor/manager
 * are built (duplicate symbols otherwise).
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <mtk_vcorefs_manager.h>
#include <mtk_spm_resource_req.h>
#include <mach/mt6351_regs.h>	/* MT6351 VCORE VOSEL_ON for the pwrap readback */

int is_vcorefs_can_work(void)
{
	return 0;
}
EXPORT_SYMBOL(is_vcorefs_can_work);

int vcorefs_request_dvfs_opp(enum dvfs_kicker kicker, enum dvfs_opp opp)
{
	return 0;
}
EXPORT_SYMBOL(vcorefs_request_dvfs_opp);

/*
 * Thermal (mtk_ts_cpu.c THERMAL_LT_SET_HPM path) polls the kicker state.
 * With vcorefs deferred there is never an active request, so report
 * OPPI_UNREQ; any resulting enter-HPM request lands in the no-op above.
 */
int vcorefs_get_kicker_opp(int kicker)
{
	return OPPI_UNREQ;
}
EXPORT_SYMBOL(vcorefs_get_kicker_opp);

/*
 * SPM resource-request stub. The real impl (spm_v2/mtk_spm_resource_req.c) is
 * not built here. The 4.4 USB PHY driver calls spm_resource_req()
 * to keep PLLs / 26M alive for SSUSB; at boot there is no aggressive SPM power
 * gating, so returning true (request granted) is a safe no-op. Drop this when
 * the SPM resource-request core is built.
 */
bool spm_resource_req(unsigned int user, unsigned int req_mask)
{
	return true;
}
EXPORT_SYMBOL(spm_resource_req);

/*
 * mt_gpufreq (MT_GPUFREQ_USE_BUCK_MT6353 segment paths) reads the live
 * VCORE through vcorefs_get_curr_vcore(). The governor is not built, but a
 * constant would mislead gpufreq's voltage bookkeeping, so read the value
 * from the MT6351 buck over pwrap and convert like vcorefs'
 * vcore_pmic_to_uv(): 600 mV base + 6.25 mV/step (boot state: VOSEL_ON
 * 0x40 = 1.000 V).
 */
int vcorefs_get_curr_vcore(void)
{
	extern s32 pwrap_read(u32 adr, u32 *rdata);
	u32 val = 0;

	if (pwrap_read(MT6351_PMIC_BUCK_VCORE_VOSEL_ON_ADDR, &val))
		return 0;
	val = (val >> MT6351_PMIC_BUCK_VCORE_VOSEL_ON_SHIFT) &
	      MT6351_PMIC_BUCK_VCORE_VOSEL_ON_MASK;
	return 600000 + val * 6250;
}
EXPORT_SYMBOL(vcorefs_get_curr_vcore);
