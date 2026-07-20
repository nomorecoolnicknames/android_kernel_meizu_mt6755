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
 * MT6351 PMIC initial setting (4.4 "new PMIC arch" port).
 *
 * The register init sequence in PMIC_INIT_SETTING_V1() and
 * PMIC_MD_INIT_SETTING_V1() below is copied verbatim from the stock 3.10
 * kernel (drivers/misc/mediatek/power/mt6755/pmic_initial_setting.c, and
 * pmic.c for pmic_is_battery_plugout); the stock Flyme 3.18 kernel runs the
 * same sequence (same "6351 PMIC Chip = 0x%x" / "VCTCXO on MT6176 , OFF XO
 * on MT6351" log strings). The pmic_config_interface(addr,val,mask,shift)
 * accessor has the same signature in 3.10 legacy and 4.4 new-arch, so the
 * raw-hex table ports as is.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>

#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot_reason.h>	/* get_boot_reason(), BR_POWER_KEY */

#include "include/pmic.h"

#define PMIC_32K_LESS_DETECT_V1      1
#define PMIC_CO_TSX_V1               1

/*
 * is_battery_remove / is_wdt_reboot_pmic / is_wdt_reboot_pmic_chk are OWNED by
 * the common layer (common/upmu.c, declared extern in <mach/upmu_sw.h>) - do
 * NOT define them here or the link double-defines.  Requires
 * CONFIG_MTK_PMIC_COMMON=y alongside CONFIG_MTK_PMIC_CHIP_MT6351 (standard
 * new-arch combo).
 */

/*
 * PMIC_MD_INIT_SETTING_V1 - modem co-clock / 32k-less detection.
 * Verbatim from 3.10 MT6351 pmic_initial_setting.c (the *real* MT6351 branch,
 * NOT the mt6355 stub which prints "OFF XO on MT6353").
 */
int PMIC_MD_INIT_SETTING_V1(void)
{
	unsigned int ret = 0;
	unsigned int pmic_reg = 0;
#if PMIC_CO_TSX_V1
	struct device_node *modem_temp_node = NULL;
	void __iomem *modem_temp_base = NULL;
#endif

#if PMIC_32K_LESS_DETECT_V1
	/* 32k less crystal auto detect start */
	ret |= pmic_config_interface(0x701E, 0x1, 0x1, 0);
	ret |= pmic_config_interface(0x701E, 0x3, 0xF, 1);
	ret = pmic_read_interface(0x7000, &pmic_reg, 0xffff, 0);
	ret |= pmic_config_interface(0x701E, 0x0, 0x1, 0);
	ret = pmic_config_interface(0xA04, 0x1, 0x1, 3);
	if ((pmic_reg & 0x200) == 0x200) {
		/* VCTCXO on MT6176, OFF XO on MT6351, HW control, use srclken_0 */
		ret = pmic_config_interface(0xA04, 0x0, 0x7, 11);
		pr_err("[PMIC] VCTCXO on MT6176 , OFF XO on MT6351\n");
	} else {
		/* HW control, use srclken_1, for LP */
		ret = pmic_config_interface(0xA04, 0x1, 0x1, 4);
		ret = pmic_config_interface(0xA04, 0x1, 0x7, 11);
		pr_err("[PMIC] VCTCXO 0x7000=0x%x\n", pmic_reg);
	}
#endif

#if PMIC_CO_TSX_V1
	modem_temp_node = of_find_compatible_node(NULL, NULL, "mediatek,MODEM_TEMP_SHARE");
	if (modem_temp_node == NULL) {
		pr_err("PMIC get modem_temp_node failed\n");
		if (modem_temp_base)
			iounmap(modem_temp_base);
		modem_temp_base = 0;
		return ret;
	}
	modem_temp_base = of_iomap(modem_temp_node, 0);
	/* modem temp */
	writel(0x011f, modem_temp_base);
	pr_err("[PMIC] TEMP_SHARE_CTRL:0x%x\n", readl(modem_temp_base));
	writel(0x013f, modem_temp_base + 0x04);
	writel(0x0, modem_temp_base);
	pr_err("[PMIC] TEMP_SHARE_CTRL:0x%x _RATIO:0x%x\n",
	       readl(modem_temp_base), readl(modem_temp_base + 0x04));
#endif
	return ret;
}

/*
 * pmic_is_battery_plugout - used by FG to know whether the battery was removed.
 * Verbatim port from 3.10 pmic.c.
 */
unsigned short pmic_is_battery_plugout(void)
{
	int is_battery_plugout;
	int pmic_strup_pwroff_seq_en = pmic_get_register_value(PMIC_STRUP_PWROFF_SEQ_EN);
	int uvlo_rstb_status = pmic_get_register_value(PMIC_UVLO_RSTB_STATUS);
	int is_long_press = (get_boot_reason() == BR_POWER_KEY ? 1 : 0);
	int is_wdt_reboot = pmic_get_register_value(PMIC_WDTRSTB_STATUS);

	pmic_set_register_value(PMIC_UVLO_RSTB_STATUS, 1);

	if (pmic_strup_pwroff_seq_en) {
		if (uvlo_rstb_status)
			is_battery_plugout = 0;
		else {
			if (is_long_press)
				is_battery_plugout = 0;
			else {
				if (is_wdt_reboot)
					is_battery_plugout = 0;
				else
					is_battery_plugout = 1;
			}
		}
	} else
		is_battery_plugout = 1;

	pr_err("[pmic_is_battery_plugout] [%d] %d, %d, %d, %d\n", is_battery_plugout,
	       pmic_strup_pwroff_seq_en, uvlo_rstb_status, is_long_press, is_wdt_reboot);
	return is_battery_plugout;
}

/* 4.4 PMIC_COMMON helper shims (kept so common/upmu.c can call them) */
int PMIC_check_wdt_status(void)
{
	is_wdt_reboot_pmic = pmic_get_register_value(PMIC_WDTRSTB_STATUS);
	pmic_set_register_value(PMIC_TOP_RST_MISC_SET, 0x8);
	is_wdt_reboot_pmic_chk = pmic_get_register_value(PMIC_WDTRSTB_STATUS);
	pmic_set_register_value(PMIC_TOP_RST_MISC_CLR, 0x8);
	pmic_set_register_value(PMIC_TOP_RST_MISC_SET, 0x1);
	return is_wdt_reboot_pmic;
}

int PMIC_check_battery(void)
{
	/* authentic MT6351 logic: derive battery-removed flag (see 3.10 pmic.c) */
	is_battery_remove = pmic_is_battery_plugout();
	return is_battery_remove ? 0 : 1;
}

/*
 * PMIC_LP_INIT_SETTING - the MT6351 power-up register table.
 *
 * The pmic_config_interface() block below is verbatim from the 3.10 MT6351
 * PMIC_INIT_SETTING_V1(), the sequence the stock Flyme 3.18 kernel runs.
 * In the 4.4 new-arch, the generic orchestrator PMIC_INIT_SETTING_V1()
 * lives in common/upmu_initial_setting.c and calls this chip hook for the
 * chip-specific register table; battery/wdt/chip-id reads and logging are
 * done by common + PMIC_check_*().
 */
void PMIC_LP_INIT_SETTING(void)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(0x8, 0x1, 0x1, 0);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 1);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 3);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 4);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 5);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 7);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 8);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 10);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 11);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 12);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 13);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 14);
	ret = pmic_config_interface(0xA, 0x1, 0x1, 15);
	ret = pmic_config_interface(0x18, 0x1, 0x1, 5);
	ret = pmic_config_interface(0x1C, 0x1, 0x1, 7);
	ret = pmic_config_interface(0x1E, 0x1, 0x1, 0);
	ret = pmic_config_interface(0x1E, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x2C, 0x1, 0x1, 15);
	ret = pmic_config_interface(0x32, 0x1, 0x1, 2);
	ret = pmic_config_interface(0x32, 0x1, 0x1, 3);
	ret = pmic_config_interface(0x204, 0x1, 0x1, 4);
	ret = pmic_config_interface(0x204, 0x1, 0x1, 5);
	ret = pmic_config_interface(0x204, 0x1, 0x1, 6);
	ret = pmic_config_interface(0x226, 0x1, 0x1, 0);
	ret = pmic_config_interface(0x228, 0x1, 0x1, 0);
	ret = pmic_config_interface(0x228, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x228, 0x1, 0x1, 2);
	ret = pmic_config_interface(0x228, 0x1, 0x1, 3);
	ret = pmic_config_interface(0x23A, 0x0, 0x1, 9);
	ret = pmic_config_interface(0x23A, 0x1, 0x1, 11);
	ret = pmic_config_interface(0x240, 0x1, 0x1, 2);
	ret = pmic_config_interface(0x240, 0x1, 0x1, 3);
	ret = pmic_config_interface(0x240, 0x0, 0x1, 10);
	ret = pmic_config_interface(0x246, 0x1, 0x1, 13);
	ret = pmic_config_interface(0x246, 0x1, 0x1, 14);
	ret = pmic_config_interface(0x24C, 0x0, 0x1, 2);
	ret = pmic_config_interface(0x258, 0x0, 0x1, 5);
	ret = pmic_config_interface(0x258, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x258, 0x0, 0x1, 14);
	ret = pmic_config_interface(0x25E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0x25E, 0x0, 0x1, 10);
	ret = pmic_config_interface(0x282, 0x0, 0x1, 3);
	ret = pmic_config_interface(0x282, 0x0, 0x1, 4);
	ret = pmic_config_interface(0x282, 0x0, 0x1, 10);
	ret = pmic_config_interface(0x282, 0x0, 0x1, 11);
	ret = pmic_config_interface(0x28E, 0x0, 0x1, 4);
	ret = pmic_config_interface(0x410, 0x8, 0x3F, 8);
	ret = pmic_config_interface(0x414, 0x3, 0x3, 4);
	ret = pmic_config_interface(0x422, 0x1, 0x1, 7);
	ret = pmic_config_interface(0x436, 0x0, 0x3, 2);
	ret = pmic_config_interface(0x450, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x452, 0x1, 0x1, 3);
	ret = pmic_config_interface(0x456, 0x1, 0x1, 7);
	ret = pmic_config_interface(0x45C, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x45E, 0x400, 0xFFFF, 0);
	ret = pmic_config_interface(0x464, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x466, 0x1, 0x1, 3);
	ret = pmic_config_interface(0x466, 0x5, 0x7, 9);
	ret = pmic_config_interface(0x46A, 0x1, 0x1, 7);
	ret = pmic_config_interface(0x470, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x472, 0x400, 0xFFFF, 0);
	ret = pmic_config_interface(0x478, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x47A, 0x2, 0x7, 6);
	ret = pmic_config_interface(0x47E, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x480, 0x1, 0x7, 4);
	ret = pmic_config_interface(0x484, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x48C, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x48E, 0x2, 0x7, 6);
	ret = pmic_config_interface(0x492, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x494, 0x1, 0x7, 4);
	ret = pmic_config_interface(0x498, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x4A0, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x4A2, 0x2, 0x7, 6);
	ret = pmic_config_interface(0x4A6, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x4A8, 0x1, 0x7, 4);
	ret = pmic_config_interface(0x4AC, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x4B6, 0x6, 0x7, 6);
	ret = pmic_config_interface(0x4C2, 0x10, 0xFFFF, 0);
	ret = pmic_config_interface(0x4C8, 0xF, 0xF, 11);
	ret = pmic_config_interface(0x4CA, 0x5, 0x7, 6);
	ret = pmic_config_interface(0x4CE, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x4D0, 0x1, 0x7, 4);
	ret = pmic_config_interface(0x4DC, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x4DC, 0x2, 0x3, 4);
	ret = pmic_config_interface(0x4DC, 0x0, 0x1, 10);
	ret = pmic_config_interface(0x4DC, 0x1, 0x3, 14);
	ret = pmic_config_interface(0x4E0, 0x0, 0x3, 14);
	ret = pmic_config_interface(0x4E2, 0x88, 0xFF, 8);
	ret = pmic_config_interface(0x600, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x606, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x606, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x60C, 0x10, 0x7F, 0);
	ret = pmic_config_interface(0x612, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x612, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x61A, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x61A, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x620, 0x10, 0x7F, 0);
	ret = pmic_config_interface(0x626, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x626, 0x1, 0x3, 4);
	ret = pmic_config_interface(0x62E, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x62E, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x634, 0x10, 0x7F, 0);
	ret = pmic_config_interface(0x63A, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x63A, 0x1, 0x3, 4);
	ret = pmic_config_interface(0x63A, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x642, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x642, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x646, 0x30, 0x7F, 0);
	ret = pmic_config_interface(0x648, 0x10, 0x7F, 0);
	ret = pmic_config_interface(0x64E, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x64E, 0x1, 0x3, 4);
	ret = pmic_config_interface(0x64E, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x656, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x656, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x65C, 0x10, 0x7F, 0);
	ret = pmic_config_interface(0x662, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x662, 0x1, 0x3, 4);
	ret = pmic_config_interface(0x662, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x676, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x68A, 0x1, 0x1, 8);
	ret = pmic_config_interface(0x692, 0x0, 0x7F, 0);
	ret = pmic_config_interface(0x692, 0x1, 0x7F, 8);
	ret = pmic_config_interface(0x69E, 0x0, 0x3, 0);
	ret = pmic_config_interface(0x6A0, 0x1, 0x1, 1);
	ret = pmic_config_interface(0x6A6, 0x11, 0x7F, 0);
	ret = pmic_config_interface(0x6A6, 0x4, 0x7F, 8);
	ret = pmic_config_interface(0x6AC, 0x35, 0x7F, 0);
	ret = pmic_config_interface(0x6B2, 0x3, 0x3, 0);
	ret = pmic_config_interface(0x6B2, 0x1, 0x3, 4);
	ret = pmic_config_interface(0x6B2, 0x0, 0x1, 8);
	ret = pmic_config_interface(0xA00, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA00, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA02, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA04, 0x1, 0x1, 3);
	ret = pmic_config_interface(0xA04, 0x0, 0x7, 11);
	ret = pmic_config_interface(0xA06, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA08, 0x1, 0x1, 3);
	ret = pmic_config_interface(0xA08, 0x1, 0x7, 11);
	ret = pmic_config_interface(0xA0A, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA0E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA14, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA16, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA16, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA18, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA1E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA24, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA2A, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA30, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA34, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA34, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA36, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA3C, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA46, 0x1, 0x1, 3);
	ret = pmic_config_interface(0xA46, 0x1, 0x7, 11);
	ret = pmic_config_interface(0xA48, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA4C, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA4C, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA4E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA54, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA5A, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA66, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA68, 0x1, 0x1, 3);
	ret = pmic_config_interface(0xA68, 0x1, 0x7, 11);
	ret = pmic_config_interface(0xA6E, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA6E, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA74, 0x1, 0x1, 1);
	ret = pmic_config_interface(0xA74, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA74, 0x0, 0x1, 3);
	ret = pmic_config_interface(0xA74, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA7C, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA82, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA86, 0x0, 0x1, 3);
	ret = pmic_config_interface(0xA88, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA8E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA94, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xA9C, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xA9C, 0x0, 0x7, 5);
	ret = pmic_config_interface(0xA9E, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xAAC, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xB10, 0x2, 0x7, 8);
	ret = pmic_config_interface(0xB24, 0xF0, 0xFF, 2);
	ret = pmic_config_interface(0xCC4, 0x1, 0x1, 8);
	ret = pmic_config_interface(0xCC4, 0x1, 0x1, 9);
	ret = pmic_config_interface(0xCC8, 0x1F, 0xFFFF, 0);
	ret = pmic_config_interface(0xCCA, 0x14, 0xFF, 0);
	ret = pmic_config_interface(0xCCC, 0xFF, 0xFF, 8);
	ret = pmic_config_interface(0xCE2, 0x1, 0x7FFF, 0);
	ret = pmic_config_interface(0xCE4, 0xBCAC, 0xFFFF, 0);
	ret = pmic_config_interface(0xEA2, 0x0, 0x1, 13);
	ret = pmic_config_interface(0xEA2, 0x0, 0x1, 14);
	ret = pmic_config_interface(0xEA2, 0x0, 0x1, 15);
	ret = pmic_config_interface(0xEAA, 0x83, 0xFFF, 0);
	ret = pmic_config_interface(0xEAA, 0x0, 0x1, 13);
	ret = pmic_config_interface(0xEAA, 0x1, 0x1, 15);
	ret = pmic_config_interface(0xEB2, 0x1, 0x3, 4);
	ret = pmic_config_interface(0xEB2, 0x3, 0x3, 6);
	ret = pmic_config_interface(0xEB2, 0x1, 0x3, 8);
	ret = pmic_config_interface(0xEB2, 0x1, 0x3, 10);
	ret = pmic_config_interface(0xEB2, 0x1, 0x3, 12);
	ret = pmic_config_interface(0xEB2, 0x2, 0x3, 14);
	ret = pmic_config_interface(0xEB4, 0x1, 0x3, 0);
	ret = pmic_config_interface(0xEB4, 0x1, 0x3, 2);
	ret = pmic_config_interface(0xEB4, 0x1, 0x3, 4);
	ret = pmic_config_interface(0xEB4, 0x3, 0x3, 6);
	ret = pmic_config_interface(0xEC6, 0x1, 0x1, 14);
	ret = pmic_config_interface(0xF16, 0xC, 0x3FF, 0);
	ret = pmic_config_interface(0xF16, 0x0, 0x1, 15);
	ret = pmic_config_interface(0xF1C, 0xC, 0x3FF, 0);
	ret = pmic_config_interface(0xF1C, 0x1, 0x1, 15);
	ret = pmic_config_interface(0xF20, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xF7A, 0xB, 0xF, 4);
	ret = pmic_config_interface(0xF84, 0x4, 0xF, 1);
	ret = pmic_config_interface(0xF92, 0x3, 0xF, 0);
	ret = pmic_config_interface(0xFA0, 0x1, 0x1, 1);
	ret = pmic_config_interface(0xFA4, 0x0, 0x7, 4);
	ret = pmic_config_interface(0xFAA, 0x1, 0x1, 2);
	ret = pmic_config_interface(0xFAA, 0x1, 0x1, 6);
	ret = pmic_config_interface(0xFAA, 0x1, 0x1, 7);

	ret = PMIC_IMM_GetOneChannelValue(PMIC_AUX_CH11, 5, 0);
	/* PMIC_MD_INIT_SETTING_V1() is invoked separately by the common flow */
}

/*
 * MT6351 has no dedicated RG_PWRHOLD register (unlike MT6355).  Power-hold is
 * asserted by LK and by the STRUP_CON0[0] write in PMIC_LP_INIT_SETTING() above.
 * Report "already held" so the common boot flow does not try to (re)assert it
 * through a non-existent register.
 */
int PMIC_check_pwrhold_status(void)
{
	return 1;
}

int PMIC_POWER_HOLD(unsigned int hold)
{
	/* best-effort: STRUP_CON0[0] (reg 0x8) is the MT6351 strap-up latch */
	pmic_config_interface(0x8, hold ? 0x1 : 0x0, 0x1, 0);
	return 0;
}

/*
 * On MT6351 the low-power (SRCLKEN) rail config is already folded into the
 * 0xA00..0xAAC block of PMIC_LP_INIT_SETTING() above (unlike MT6355 which
 * splits it via pmic_buck_*_lp()/pmic_ldo_*_lp() helpers absent on MT6351).
 */
void PMIC_PWROFF_SEQ_SETTING(void)
{
}
