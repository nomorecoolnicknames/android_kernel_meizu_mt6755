/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * MT6351 PMIC throttling / DLPT / low-battery protection (4.4 port).
 *
 * Only the entry points the common probe (common/upmu.c) calls and the
 * enable/callback symbols other layers link against are provided, wired to
 * the MT6351 thresholds from the board cust_pmic.h. The interrupt-driven
 * protection loops of the 3.10 implementation (tightly coupled to the
 * battery_meter / charging stack) are stubbed.
 *
 * Board thresholds (wt6755_66_sz_l/power/cust_pmic.h):
 *   UVLO=2600mV, INT0/1/2=3400/3250/3100mV, BAT_OC H/L=4670/5500mA,
 *   POWEROFF_BAT_CURRENT=3000mA, DLPT_POWER_OFF_THD=100, DLPT_POWER_OFF_EN.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>

#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include "include/pmic.h"
#include "include/pmic_throttling_dlpt.h"

/* board thresholds */
#define POWER_UVLO_VOLT_LEVEL	2600
#define POWER_INT0_VOLT		3400
#define POWER_INT1_VOLT		3250
#define POWER_INT2_VOLT		3100
#define POWER_BAT_OC_CURRENT_H	4670
#define POWER_BAT_OC_CURRENT_L	5500

/* ---- shared protection-state globals (declared extern in include/pmic.h) ---- */
int g_lowbat_int_bottom;
int g_low_battery_level;
int g_battery_oc_level;

/* ---- low-battery protect enable controls ----
 * MT6351 low-battery comparator lives at the NI_LBAT_INT block; the EN bits are
 * driven through the interrupt enable in pmic_irq.c.  Provided as thin shims so
 * callers link; full comparator threshold programming is TODO.
 */
void lbat_min_en_setting(int en_val)
{
	PMICLOG("[lbat_min_en_setting] en=%d (TODO: program LBAT min comparator)\n", en_val);
}

void lbat_max_en_setting(int en_val)
{
	PMICLOG("[lbat_max_en_setting] en=%d (TODO: program LBAT max comparator)\n", en_val);
}

void exec_low_battery_callback(LOW_BATTERY_LEVEL low_battery_level)
{
	g_low_battery_level = low_battery_level;
	PMICLOG("[exec_low_battery_callback] level=%d (TODO: fan out callbacks)\n",
		low_battery_level);
}

/* ---- battery over-current protect ---- */
void bat_oc_h_en_setting(int en_val)
{
	PMICLOG("[bat_oc_h_en_setting] en=%d thd=%dmA (TODO)\n", en_val, POWER_BAT_OC_CURRENT_H);
}

void bat_oc_l_en_setting(int en_val)
{
	PMICLOG("[bat_oc_l_en_setting] en=%d thd=%dmA (TODO)\n", en_val, POWER_BAT_OC_CURRENT_L);
}

void exec_battery_oc_callback(BATTERY_OC_LEVEL battery_oc_level)
{
	g_battery_oc_level = battery_oc_level;
	PMICLOG("[exec_battery_oc_callback] level=%d (TODO: fan out callbacks)\n",
		battery_oc_level);
}

/* ---- interrupt handlers referenced by pmic_irq.c / header ---- */
void bat_h_int_handler(void)	{ PMICLOG("[bat_h_int_handler] (TODO)\n"); }
void bat_l_int_handler(void)	{ PMICLOG("[bat_l_int_handler] (TODO)\n"); }
void fg_cur_h_int_handler(void)	{ PMICLOG("[fg_cur_h_int_handler] (TODO)\n"); }
void fg_cur_l_int_handler(void)	{ PMICLOG("[fg_cur_l_int_handler] (TODO)\n"); }

/* ---- sub-feature inits ---- */
void low_battery_protect_init(void)
{
	pr_info("[PMIC] low_battery_protect_init INT0/1/2=%d/%d/%d mV (stub)\n",
		POWER_INT0_VOLT, POWER_INT1_VOLT, POWER_INT2_VOLT);
}

void battery_oc_protect_init(void)
{
	pr_info("[PMIC] battery_oc_protect_init H/L=%d/%d mA (stub)\n",
		POWER_BAT_OC_CURRENT_H, POWER_BAT_OC_CURRENT_L);
}

void bat_percent_notify_init(void)	{ pr_info("[PMIC] bat_percent_notify_init (stub)\n"); }
void dlpt_notify_init(void)		{ pr_info("[PMIC] dlpt_notify_init (stub)\n"); }

/* ---- entry points called by common/upmu.c ---- */
int pmic_throttling_dlpt_init(void)
{
	pr_info("[PMIC] pmic_throttling_dlpt_init UVLO=%dmV (MT6351, partial)\n",
		POWER_UVLO_VOLT_LEVEL);
	low_battery_protect_init();
	battery_oc_protect_init();
	bat_percent_notify_init();
	dlpt_notify_init();
	return 0;
}

void pmic_throttling_dlpt_suspend(void) { }
void pmic_throttling_dlpt_resume(void)  { }

void pmic_throttling_dlpt_debug_init(struct platform_device *dev, struct dentry *debug_dir) { }
