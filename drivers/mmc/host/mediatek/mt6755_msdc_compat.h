#ifndef _M681_MSDC_COMPAT_H
#define _M681_MSDC_COMPAT_H
/*
 * MT6755 compile bridge. The MSDC driver (identical IP on MT6755 and
 * MT6757) calls the MT6757 SPM hardware DVFS hooks (spm_msdc_dvfs_setting
 * / MSDC2_DVFS), which the MT6755 power layer does not provide. eMMC
 * hardware DVFS is off (host->use_hw_dvfs stays 0, so the call is never
 * taken at runtime), so stub it.
 */
#ifndef MSDC2_DVFS
#define MSDC2_DVFS 2
#endif
static inline int spm_msdc_dvfs_setting(int id, int enable) { return 0; }

/*
 * msdc_tune.c queries vcore-DVFS for the current hardware operating point to
 * pick eMMC pad-tune values per voltage. With vcore-DVFS deferred for boot the
 * voltage is fixed, so report the performance OPP (highest, safest tuning).
 * Return the literal 0 (== OPPI_PERF == OPP_0); the real OPPI_PERF macro now
 * comes from the bridged mt6757 vcorefs governor header, so do NOT redefine it
 * here (that would clash under -Werror).
 */
static inline int vcorefs_get_hw_opp(void) { return 0; /* OPPI_PERF / OPP_0 */ }
#endif /* _M681_MSDC_COMPAT_H */
