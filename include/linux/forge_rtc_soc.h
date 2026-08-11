/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_FORGE_RTC_SOC_H
#define _LINUX_FORGE_RTC_SOC_H

#include <linux/types.h>

/*
 * The battery percentage in the stock RTC FG SOC field (RTC_AL_HOU bits
 * 14:8), so that it survives a reboot as on stock. Written by
 * drivers/misc/mediatek/pmic_wrap/mt6755/pwrap_hal_v1.c, used by
 * forge_chg_observe.c (weak -ENODEV stubs there when pwrap is not built).
 * Process context only.
 *
 * forge_rtc_soc_get: the field, 0 = no value, or -errno.
 * forge_rtc_soc_set: 0..100 (1 or less stored as 1), 0 or -errno.
 * forge_rtc_relock_needed: the last RTC relock failed - the write
 * interface may be open; the next forge_rtc_soc_set relocks it.
 */
int forge_rtc_soc_get(void);
int forge_rtc_soc_set(int soc);
bool forge_rtc_relock_needed(void);

#endif /* _LINUX_FORGE_RTC_SOC_H */
