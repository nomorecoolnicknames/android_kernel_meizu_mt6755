/*
 * Copyright (C) 2016 MediaTek Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/m3note_board.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>

#include <linux/uaccess.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_wrap.h>
#if defined CONFIG_MTK_LEGACY
/*#include <mach/mt_gpio.h> TBD*/
#endif
/*#include <mach/mtk_rtc.h> TBD*/
#include <mach/mt_spm_mtcmos.h>
#include <mt-plat/battery_common.h>
#include <linux/time.h>

#if !defined(CONFIG_FPGA_EARLY_PORTING)
CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
#endif

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
bool is_dcp_type = false;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_FPGA_EARLY_PORTING)

int hw_charging_get_charger_type(void)
{
	return STANDARD_HOST;
}

#else

static bool first_connect = true;

/* BC1.2 charger type detection. The walk's RG_USB20_BC11_SW_EN=1 hands
 * DP/DM to the PMIC BC11 comparator, which drops a live USB session. On
 * M681 the battery stack starts ~90 s after boot, so the walk would run on
 * an enumerated adb session while the forge_usb_kick worker (mtk_usb.c)
 * re-runs musb_start: a PHY/controller race the stock kernel (which walks
 * at cable attach, before enumeration) never runs. Skipping the walk
 * writes no BC11/CDI register and reports STANDARD_HOST; a DCP wall
 * charger then also reads STANDARD_HOST (500 mA charging).
 * -1 = by board: L681 walks (0), M681 keeps the skip (1). L681 starts the
 * battery stack at once (forge_bat_delay_s 0), so its walk runs at cable
 * attach before enumeration, as stock, and mtk_usb.c keeps the kicker off
 * the PHY while forge_bc11_walking is set and on a wall charger.
 * Runtime: echo 0|1 > /sys/module/pmic_chr_type_det/parameters/forge_bc11_skip,
 * then replug.
 */
static int forge_bc11_skip = -1;
module_param(forge_bc11_skip, int, 0644);
MODULE_PARM_DESC(forge_bc11_skip,
	"skip the BC1.1 charger-type walk, report STANDARD_HOST: 1 skip, 0 walk, -1 by board (l681 walk, m681 skip; default)");

/* set while the walk owns DP/DM; read by mtk_usb.c's kicker */
bool forge_bc11_walking;

static void hw_bc11_init(void)
{
	int timeout = 100;

	msleep(200);

	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == KAL_FALSE) {
			pr_err("CDP, block\n");
			while (is_usb_rdy() == KAL_FALSE && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_err("CDP, timeout\n");
			else
				pr_err("CDP, free\n");
		} else
			pr_err("CDP, PASS\n");
		first_connect = false;
	}

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* Markers bracketing the three sub-steps (BC11 pwrap writes / CDI /
	 * DCD). */
	pr_emerg("[FORGE_CHG][BC11] init writes done\n");
	msleep(50);

	pr_emerg("[FORGE_CHG][BC11] CDI enter\n");
	Charger_Detect_Init();
	pr_emerg("[FORGE_CHG][BC11] CDI done\n");
}


static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;

	pr_emerg("[FORGE_CHG][BC11] DCD enter\n");
	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	/* mdelay(80); */
	pr_emerg("[FORGE_CHG][BC11] DCD writes+sleep done, reading CMP_OUT\n");
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	return wChargerAvail;
}


static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}


static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}


static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		pr_err("charger type: DCP, keep DM voltage source in stepB2\n");
	}
	return wChargerAvail;
}


static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);
	Charger_Detect_Release();
}


static void dump_charger_name(CHARGER_TYPE type)
{
	switch (type) {
	case CHARGER_UNKNOWN:
		pr_err("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case STANDARD_HOST:
		pr_err("charger type: %d, Standard USB Host\n", type);
		break;
	case CHARGING_HOST:
		pr_err("charger type: %d, Charging USB Host\n", type);
		break;
	case NONSTANDARD_CHARGER:
		pr_err("charger type: %d, Non-standard Charger\n", type);
		break;
	case STANDARD_CHARGER:
		pr_err("charger type: %d, Standard Charger\n", type);
		break;
	case APPLE_2_1A_CHARGER:
		pr_err("charger type: %d, APPLE_2_1A_CHARGER\n", type);
		break;
	case APPLE_1_0A_CHARGER:
		pr_err("charger type: %d, APPLE_1_0A_CHARGER\n", type);
		break;
	case APPLE_0_5A_CHARGER:
		pr_err("charger type: %d, APPLE_0_5A_CHARGER\n", type);
		break;
	default:
		pr_err("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}

inline void mt_bc12_dcd_release(void)
{
	if (CHR_Type_num == STANDARD_CHARGER) {
		hw_bc11_done();
		CHR_Type_num = CHARGER_UNKNOWN;
	}
}

void hw_charging_enable_dp_voltage(int ison)
{
#if 0
	static int cnt;

	if (ison == 1) {
		if (cnt == 0) {
			cnt = 1;
			hw_bc11_init();
			bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		}
	} else if (cnt == 1) {
		hw_bc11_done();
		cnt = 0;
	}
#endif
}

int hw_charging_get_charger_type(void)
{
	unsigned int dcd_result;
	unsigned int a1_result = 0;
	unsigned int a2_result = 0;
	unsigned int b2_result = 0;

	CHR_Type_num = CHARGER_UNKNOWN;

	/* see forge_bc11_skip above */
	if (forge_bc11_skip > 0 ||
	    (forge_bc11_skip < 0 && !m3note_board_known_is(M3NOTE_BOARD_L681))) {
		pr_emerg("[FORGE_CHG][BC11] walk SKIPPED (forge_bc11_skip=1) -> STANDARD_HOST\n");
		CHR_Type_num = STANDARD_HOST;
		return CHR_Type_num;
	}

	forge_bc11_walking = true;
	hw_bc11_init();

	dcd_result = hw_bc11_DCD();
	pr_err("[M6_CHG] bc11 DCD=%u\n", dcd_result);
	if (dcd_result) {
		a1_result = hw_bc11_stepA1();
		pr_err("[M6_CHG] bc11 A1=%u\n", a1_result);
		if (a1_result)
			CHR_Type_num = APPLE_2_1A_CHARGER;
		else
			CHR_Type_num = NONSTANDARD_CHARGER;
	} else {
		a2_result = hw_bc11_stepA2();
		pr_err("[M6_CHG] bc11 A2=%u\n", a2_result);
		if (a2_result) {
			b2_result = hw_bc11_stepB2();
			pr_err("[M6_CHG] bc11 B2=%u\n", b2_result);
			if (b2_result)
				CHR_Type_num = STANDARD_CHARGER;
			else
				CHR_Type_num = CHARGING_HOST;
		} else
			CHR_Type_num = STANDARD_HOST;
	}

	if (CHR_Type_num != STANDARD_CHARGER)
		hw_bc11_done();
	else
		pr_err("charger type: skip bc11 release for BC12 DCP SPEC\n");

	forge_bc11_walking = false;
	dump_charger_name(CHR_Type_num);
	pr_err("[M6_CHG] bc11 done type=%d dcd=%u a1=%u a2=%u b2=%u is_dcp=%d\n",
		CHR_Type_num, dcd_result, a1_result, a2_result, b2_result, is_dcp_type);

	return CHR_Type_num;

}
#endif
