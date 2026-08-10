/*
 * MT6755 SPM link stubs.
 *
 * The SPM deep-idle / SODI / dpidle / vcore-DVFS scenario implementations
 * (mt_spm_dpidle.c, mt_spm_sodi*.c, mt_spm_vcorefs_mt6755.c) are not built.
 * The SPM core + suspend/sysfs files still reference their scenario structs
 * and a few helper entry points. Provide zeroed scenario structs + no-op
 * helpers so the kernel links. Drop this file when the SPM low-power
 * scenarios are built.
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/types.h>
#include "mt_spm.h"
#include "mt_spm_internal.h"

/* Deferred low-power scenario descriptors (consumed by mt_spm_internal/_fs/_sleep
 * for sysfs + suspend; not entered at boot). Zeroed is safe because the entry
 * paths that would dereference .pwrctrl/.pcmdesc are never reached pre-adb. */
struct spm_lp_scen __spm_dpidle;
struct spm_lp_scen __spm_sodi;
struct spm_lp_scen __spm_sodi3;
struct spm_lp_scen __spm_vcore_dvfs;


wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data)
{
	return WR_NONE;
}

/* vcore-DVFS query/late-init — deferred; report a benign fixed state. */
int vcorefs_get_curr_ddr(void) { return 0; }
EXPORT_SYMBOL(vcorefs_get_curr_ddr);

int vcorefs_late_init_dvfs(void) { return 0; }
EXPORT_SYMBOL(vcorefs_late_init_dvfs);

/* clk_buf_init/clk_buf_write_afcdac come from mt_clkbuf_ctl.o. */
