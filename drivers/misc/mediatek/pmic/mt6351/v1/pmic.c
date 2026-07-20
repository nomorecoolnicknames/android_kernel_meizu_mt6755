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
 * MT6351 PMIC platform glue (4.4 port).
 *
 * The platform_driver + probe live in the chip-agnostic common layer
 * (drivers/misc/mediatek/pmic/common/upmu.c), which calls into this driver
 * via PMIC_INIT_SETTING_V1(), PMIC_EINT_SETTING(), mtk_regulator_init(),
 * pmic_auxadc_init() and pmic_throttling_dlpt_init(). This file therefore
 * only carries the small MT6351-specific glue (chip-id helper,
 * charger-detect ready handshake, ext-buck stubs) and the shared globals.
 *
 * Charger-detect helpers ported from MT6355 pmic.c; the chip-id helper uses
 * the MT6351 CID codes (0x5110/0x5120/0x5130, mtk_pmic_info.h).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include "include/pmic.h"

/* ---- shared globals ----
 * gPMICDbgLvl and g_pmic_pad_vbif28_vol are OWNED by the common layer
 * (common/upmu_debugfs.c, common/upmu_regulator.c) - referenced here via the
 * externs in include/pmic.h / pmic_debugfs.h, not redefined.
 * pmic_rdy / usb_rdy are chip-local (mirrors MT6355 pmic.c).
 */
int pmic_rdy;
int usb_rdy;

/* VBIF28 (PAD) voltage accessors are OWNED by common/upmu_regulator.c. */

/* ---- chip id ---- */
unsigned int get_mt6351_pmic_chip_version(void)
{
	unsigned int cid = pmic_get_register_value(PMIC_SWCID);

	switch (cid) {
	case PMIC6351_E1_CID_CODE:
	case PMIC6351_E2_CID_CODE:
	case PMIC6351_E3_CID_CODE:
		break;
	default:
		pr_err("[PMIC] unexpected MT6351 CID = 0x%x\n", cid);
		break;
	}
	return cid;
}

/* ---- charger detection ready handshake ---- */
unsigned int upmu_get_rgs_chrdet(void)
{
	unsigned int val = pmic_get_register_value(PMIC_RGS_CHRDET);

	PMICLOG("[upmu_get_rgs_chrdet] CHRDET status = %d\n", val);
	return val;
}

void pmic_enable_charger_detection_int(int x)
{
	if (x == 0)
		pmic_rdy = 1;
	else if (x == 1)
		usb_rdy = 1;

	if (pmic_rdy == 1 && usb_rdy == 1) {
#if defined(CONFIG_MTK_SMART_BATTERY)
		wake_up_bat();
#endif
		PMICLOG("[pmic_enable_charger_detection_int] enabled\n");
	}
}

bool is_charger_detection_rdy(void)
{
	return (pmic_rdy == 1 && usb_rdy == 1);
}

/* ---- external buck/swchr: none ---- */
int is_ext_buck2_exist(void)		{ return 0; }
int is_ext_vbat_boost_exist(void)	{ return 0; }
int is_ext_swchr_exist(void)		{ return 0; }
int is_ext_buck_exist(void)		{ return 0; }
int is_ext_buck_sw_ready(void)		{ return 0; }
int get_ext_buck_i2c_ch_num(void)	{ return -1; }

int is_ext_buck_gpio_exist(void)
{
	/* MT6351 has no external PMIC strap */
	return 0;
}

MODULE_AUTHOR("ReMeizu m681 bring-up");
MODULE_DESCRIPTION("MT6351 PMIC Device Driver (4.4 port)");
MODULE_LICENSE("GPL");
