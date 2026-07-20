/*
 * MT6755 SPM link stubs.
 *
 * The SPM deep-idle / SODI / vcore-DVFS scenario implementations
 * (mt_spm_dpidle.c, mt_spm_sodi*.c, mt_spm_vcorefs_mt6755.c) are not
 * built without MTK_MT6755_CPUIDLE, but the SPM core and the suspend/sysfs
 * files still reference their scenario structs and a few helpers. Zeroed
 * scenario structs and no-op helpers let the kernel link.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/types.h>
#include "mt_spm.h"
#include "mt_spm_internal.h"

/* Deferred low-power scenario descriptors (consumed by mt_spm_internal/_fs/_sleep
 * for sysfs + suspend; not entered at boot). Zeroed is safe because the entry
 * paths that would dereference .pwrctrl/.pcmdesc are never reached pre-adb. */
#ifndef CONFIG_MTK_MT6755_CPUIDLE
/* With MTK_MT6755_CPUIDLE the real scenarios (mt_spm_dpidle.c,
 * mt_spm_sodi.c, mt_spm_sodi3p0.c) are built and define these. */
struct spm_lp_scen __spm_dpidle;
struct spm_lp_scen __spm_sodi;
struct spm_lp_scen __spm_sodi3;
#endif

/* __spm_vcore_dvfs is NOT zeroed: suspend reads it.  spm_go_to_sleep()
 * (mt_spm_sleep.c:931, __spm_sync_vcore_dvfs_power_control) copies the
 * vcore-DVFS request masks from __spm_vcore_dvfs.pwrctrl on every entry, so a
 * zeroed stub is a NULL dereference on the first suspend once allow_s3 is
 * lifted.  Hold the
 * default pwr_ctrl of mt_spm_vcorefs_mt6755.c (not built here), verbatim:
 * vcore DVFS disabled, so suspend copies the stock "no DVFS" masks. */
static struct pwr_ctrl vcore_dvfs_ctrl = {
	/* default VCORE DVFS is disabled */
	.pcm_flags = (SPM_FLAG_DIS_VCORE_DVS | SPM_FLAG_DIS_VCORE_DFS),
	.wake_src = WAKE_SRC_R12_PCM_TIMER,
	/* SPM general */
	.r0_ctrl_en = 1,
	.r7_ctrl_en = 1,

	/* VCORE DVFS Logic pwr_ctrl */
	.dvfs_halt_mask_b = 0x07,
	.sdio_on_dvfs_req_mask_b = 0,

	.cpu_md_dvfs_erq_merge_mask_b = 1,

	.md1_ddr_en_dvfs_halt_mask_b = 0,
	.md2_ddr_en_dvfs_halt_mask_b = 0,

	.md_srcclkena_0_dvfs_req_mask_b = 0,
	.md_srcclkena_1_dvfs_req_mask_b = 0,
	.conn_srcclkena_dvfs_req_mask_b = 0,

	.vsync_dvfs_halt_mask_b = 0x0,
	.emi_boost_dvfs_req_mask_b = 0,
	.cpu_md_emi_dvfs_req_prot_dis = 1,

	.spm_dvfs_req = 1,
	.spm_dvfs_force_down = 1,
	.cpu_md_dvfs_sop_force_on = 0,

	.emi_bw_dvfs_req_mask = 1,

	.dvfs_halt_src_chk = 1,
};

struct spm_lp_scen __spm_vcore_dvfs = {
	.pwrctrl = &vcore_dvfs_ctrl,
};


#ifndef CONFIG_MTK_MT6755_CPUIDLE
wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data)
{
	return WR_NONE;
}
#endif

/* vcore DVFS is not built (mt_spm_vcorefs_mt6755.c, the vcorefs
 * governor); say so instead of reporting success:
 * - vcorefs_get_curr_ddr(): only logged by the suspend path
 *   (spm_output_wake_reason "vcore_status"); give the DRAM rate in kHz the
 *   way the governor would, -ENODEV without the DRAMC driver;
 * - vcorefs_late_init_dvfs(): called when all SPM firmware is loaded;
 *   there is no DVFS to start. */
#if IS_BUILTIN(CONFIG_MTK_DRAMC)
#include <mt_dramc.h>	/* get_dram_data_rate(); -I dramc/$(MTK_PLATFORM) in Makefile */
#endif

int vcorefs_get_curr_ddr(void)
{
#if IS_BUILTIN(CONFIG_MTK_DRAMC)
	return get_dram_data_rate() * 1000;
#else
	return -ENODEV;
#endif
}
EXPORT_SYMBOL(vcorefs_get_curr_ddr);

int vcorefs_late_init_dvfs(void)
{
	pr_info_once("[SPM] vcore DVFS is not built on this port; not started\n");
	return -ENODEV;
}
EXPORT_SYMBOL(vcorefs_late_init_dvfs);

