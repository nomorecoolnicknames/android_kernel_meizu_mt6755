/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define DEBUG 1
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <mt-plat/sync_write.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include <linux/seq_file.h>
#include <linux/slab.h>
#include "mach/mtk_thermal.h"
#include <linux/bug.h>

#if defined(CONFIG_MTK_CLKMGR)
#include <mach/mtk_clkmgr.h>
#else
#include <linux/clk.h>
#endif


#include <mach/wd_api.h>
#include <mtk_gpu_utility.h>
#include <linux/time.h>

#define __MT_MTK_TS_CPU_C__
#include <tscpu_settings.h>

#if THERMAL_INFORM_OTP
#include <mt_otp.h>
#endif

/* 1: turn on RT kthread for thermal protection in this sw module; 0: turn off */
#if MTK_TS_CPU_RT
#include <linux/sched.h>
#include <linux/kthread.h>
#endif

/* RGU thermal-direct (HW comparator -> silent warm reset) arming is
 * opt-in: 0 (default) keeps it disarmed even with a valid efuse
 * calibration; see tscpu_config_all_tc_hw_protect().
 * mtk_tc.forge_therm_hwprot=1 (boot arg or sysfs) enables it. */
static int forge_therm_hwprot;
module_param(forge_therm_hwprot, int, 0644);

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#define __MT_MTK_TS_CPU_C__

#include <mt-plat/mtk_devinfo.h>
#include <linux/m3note_board.h>
#include <linux/workqueue.h>

/* devinfo.c here overlays the v2 headerless layout {data_size; data[]} on
 * the raw v1 LK atag {size; tag; data[50]; data_size}, so
 * get_devinfo_with_index() is +1 shifted vs stock numbering: kernel index 0
 * holds the atag tag id and stock word i sits at kernel index i+1 (see
 * devinfo/devinfosys.c). The calibration reader below wants stock words
 * 8/7/9 = efuse 0x10206184/0x10206180/0x10206188; unshifted it would read
 * stock words 7/6/8 and fall back to "uncalibrated" on a calibrated die.
 * Detect the v1 layout by the tag id and shift; another layout passes
 * through. */
#define FORGE_ATAG_DEVINFO_TAG 0x41000804
static u32 forge_devinfo_stock(u32 idx)
{
	if (get_devinfo_with_index(0) == FORGE_ATAG_DEVINFO_TAG)
		return get_devinfo_with_index(idx + 1);
	return get_devinfo_with_index(idx);
}

/*
 * Sensor slope. The MT6757 code uses 1528/1000 per 0.1 C with O_SLOPE
 * scaled by 10; the stock M681 and L681 kernels use 165/100 with an
 * unscaled O_SLOPE in all three formulas (read_tc_raw_and_temp,
 * temperature_to_raw_room, get_thermal_slope_intercept; 3.10 mtk_tc.c).
 * With 1528 the reading above the 25.5 C calibration point is x1.080 too
 * high. 1 (default) = stock 165 on M681/L681; 0 (cmdline
 * mtk_tc.forge_tc_stock_slope=0) = MT6757 1528. Other boards: always 1528.
 */
static int forge_tc_stock_slope = 1;
module_param(forge_tc_stock_slope, int, 0444);
MODULE_PARM_DESC(forge_tc_stock_slope, "l681/m681: 1 (default) = stock sensor slope 165, 0 = mt6757 1528");

static bool forge_tc_use_stock(void)
{
	return forge_tc_stock_slope &&
	       (m3note_board_known_is(M3NOTE_BOARD_L681) ||
		m3note_board_known_is(M3NOTE_BOARD_M681));
}

/* one-shot "[FORGE_TC] scales" line, armed from tscpu_thermal_cal_prepare_2() */
static void forge_tc_scales_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(forge_tc_scales_work, forge_tc_scales_work_fn);
/*=============================================================
 *Local variable definition
 *=============================================================
 */
/* MT6755 three-bank layout, as in the 3.18 and 3.10 MT6755 mtk_tc.c. The
 * MT6757 four-bank table selected a nonexistent bank3 for MCU3/4/5.
*Bank0 : CPU_mp0     (TS_MCU4)
*Bank1 : SOC+GPU     (TS_MCU1, TS_MCU3, TS_MCU2)
*Bank2 : CPU_mp1     (TS_MCU4)
*/

int tscpu_bank_ts[TSCPU_NR_BANKS][TS_ENUM_MAX];
int tscpu_bank_ts_r[TSCPU_NR_BANKS][TS_ENUM_MAX];

/* chip dependent */
bank_t tscpu_g_bank[TSCPU_NR_BANKS] = {
	[0] = {
	       .ts = {TS_FILL(TS_MCU4)},
	       .ts_number = 1},
	[1] = {
	       .ts = {TS_FILL(TS_MCU1), TS_FILL(TS_MCU3), TS_FILL(TS_MCU2)},
	       .ts_number = 3},
	[2] = {
	       .ts = {TS_FILL(TS_MCU4)},
	       .ts_number = 1},
};

#ifdef CONFIG_OF
const struct of_device_id mt_thermal_of_match[2] = {
	{.compatible = "mediatek,mt6755-therm_ctrl",},
	{},
};
#endif

int tscpu_debug_log;

#if MTK_TS_CPU_RT
static struct task_struct *ktp_thread_handle;
#endif

static __s32 g_adc_ge_t;
static __s32 g_adc_oe_t;
static __s32 g_o_vtsmcu1;
static __s32 g_o_vtsmcu2;
static __s32 g_o_vtsmcu3;
static __s32 g_o_vtsmcu4;
static __s32 g_o_vtsmcu5;
static __s32 g_o_vtsabb;
static __s32 g_degc_cali;
static __s32 g_adc_cali_en_t;
static __s32 g_o_slope;
static __s32 g_o_slope_sign;
static __s32 g_id;

static __s32 g_ge;
static __s32 g_oe;
static __s32 g_gain;

static __s32 g_x_roomt[TS_ENUM_MAX] = { 0 };

static __u32 calefuse1;
static __u32 calefuse2;
static __u32 calefuse3;

/**
 * If curr_temp >= tscpu_polling_trip_temp1, use interval
 * else if cur_temp >= tscpu_polling_trip_temp2 && curr_temp < tscpu_polling_trip_temp1,
 * use interval*tscpu_polling_factor1
 * else, use interval*tscpu_polling_factor2
 */
/* chip dependent */
int tscpu_polling_trip_temp1 = 40000;
int tscpu_polling_trip_temp2 = 20000;
int tscpu_polling_factor1 = 1;
int tscpu_polling_factor2 = 4;

#if MTKTSCPU_FAST_POLLING
/* Combined fast_polling_trip_temp and fast_polling_factor,
*it means polling_delay will be 1/5 of original interval
*after mtktscpu reports > 65C w/o exit point
*/
int fast_polling_trip_temp = 60000;
int fast_polling_trip_temp_high = 60000; /* deprecaed */
int fast_polling_factor = 4;
int tscpu_cur_fp_factor = 1;
int tscpu_next_fp_factor = 1;
#endif

#if PRECISE_HYBRID_POWER_BUDGET
/*	tscpu_prev_cpu_temp: previous CPUSYS temperature
*	tscpu_curr_cpu_temp: current CPUSYS temperature
*	tscpu_prev_gpu_temp: previous GPUSYS temperature
*	tscpu_curr_gpu_temp: current GPUSYS temperature
 */
int tscpu_prev_cpu_temp = 0, tscpu_prev_gpu_temp = 0;
int tscpu_curr_cpu_temp = 0, tscpu_curr_gpu_temp = 0;
#endif

thermal_bank_name g_currentBank = THERMAL_BANK0;

static int tscpu_curr_max_ts_temp;

/*=============================================================
 * Local function declartation
 *=============================================================
 */
static __s32 temperature_to_raw_room(__u32 ret, ts_e ts_name);
static void set_tc_trigger_hw_protect(int temperature, int temperature2, thermal_bank_name bank);
/*=============================================================
 *Weak functions
 *=============================================================
 */
/* EEM/PTP is not built: these run on every bank switch; print once (see
 * the weak stubs in mtk_ts_cpu.c). */
void __attribute__ ((weak))
mt_ptp_lock(unsigned long *flags)
{
	pr_err_once("[Power/CPU_Thermal]%s doesn't exist\n", __func__);
}

void __attribute__ ((weak))
mt_ptp_unlock(unsigned long *flags)
{
	pr_err_once("[Power/CPU_Thermal]%s doesn't exist\n", __func__);
}

/* The stock weak stub returned 0 (success) without setting *obj, so a build
 * without the wdk would dereference an uninitialised stack pointer at the
 * WD_REQ_EN site below. Fail instead. (The real wd_api.c:get_wd_api is
 * linked here and always sets *obj.) */
int __attribute__ ((weak))
get_wd_api(struct wd_api **obj)
{
	pr_err_once("[Power/CPU_Thermal]%s doesn't exist\n", __func__);
	*obj = NULL;
	return -1;
}


/*=============================================================*/
void thermal_set_big_core_speed(__u32 tempMonCtl1, __u32 tempMonCtl2, __u32 tempAhbPoll)
{
	/* Deprecated, because OTP doesn't need to control
	 *the polling time of Thermal Controller
	 */
/*
*	g_bigCTS.isEnable = 1;
*	g_bigCTS.tempMonCtl1 = tempMonCtl1;
*	g_bigCTS.tempMonCtl2 = tempMonCtl2;
*	g_bigCTS.tempAhbPoll = tempAhbPoll;
*	tscpu_switch_bank(THERMAL_BANK0);
*	mt_reg_sync_writel(g_bigCTS.tempMonCtl1, TEMPMONCTL1);
*	mt_reg_sync_writel(g_bigCTS.tempMonCtl2, TEMPMONCTL2);
*	mt_reg_sync_writel(g_bigCTS.tempAhbPoll, TEMPAHBPOLL);
*/
}

/* chip dependent */
int tscpu_thermal_clock_on(void)
{
	/*Use CCF instead*/
	int ret = -1;

	tscpu_dprintk("tscpu_thermal_clock_on\n");

#if defined(CONFIG_MTK_CLKMGR)
	/*ret = enable_clock(MT_CG_PERI_THERM, "THERMAL"); */
#else
	tscpu_dprintk("CCF_thermal_clock_on\n");
	ret = clk_prepare_enable(therm_main);
	if (ret)
		tscpu_printk("Cannot enable thermal clock.\n");
#endif
	return ret;
}

/* chip dependent */
int tscpu_thermal_clock_off(void)
{
/*Use CCF instead*/
	int ret = -1;

	tscpu_dprintk("tscpu_thermal_clock_off\n");

#if defined(CONFIG_MTK_CLKMGR)
	/*ret = disable_clock(MT_CG_PERI_THERM, "THERMAL"); */
#else
	tscpu_dprintk("CCF_thermal_clock_off\n");
	clk_disable_unprepare(therm_main);
#endif
	return ret;
}

/* TODO: FIXME */
void get_thermal_slope_intercept(struct TS_PTPOD *ts_info, thermal_bank_name ts_bank)
{
	unsigned int temp0, temp1, temp2;
	struct TS_PTPOD ts_ptpod;
	__s32 x_roomt;

	tscpu_dprintk("get_thermal_slope_intercept\n");

	/* chip dependent */
	/* MT6755 banks:
	*Bank0 : CPU_mp0     (TS_MCU4)
	*Bank1 : SOC+GPU     (TS_MCU1, TS_MCU3, TS_MCU2)
	*Bank2 : CPU_mp1     (TS_MCU4)
	*/

	/*
	*   If there are two or more sensors in a bank, choose the sensor calibration value of
	*   the dominant sensor. You can observe it in the thermal doc provided by Thermal DE.
	*   For example,
	*   Bank 1 is for SOC + GPU. Observe all scenarios related to GPU tests to
	*   determine which sensor is the highest temperature in all tests.
	*   Then, It is the dominant sensor.
	*   (Confirmed by Thermal DE Alfred Tsai)
	*/
	/* Dominant-sensor roomt per bank, as in the 3.18 MT6755
	 * get_thermal_slope_intercept (BANK0->[3], BANK1->[2], BANK2->[3]);
	 * x_roomt index == ts_e enum (MCU4 = 3, MCU3 = 2). */
	switch (ts_bank) {
	case THERMAL_BANK0:
		x_roomt = g_x_roomt[3];
		break;
	case THERMAL_BANK1:
		x_roomt = g_x_roomt[2];
		break;
	case THERMAL_BANK2:
		x_roomt = g_x_roomt[3];
		break;
	default:		/*choose high temp */
		x_roomt = g_x_roomt[3];
		break;
	}

	/*
	*   The equations in this function are confirmed by Thermal DE Alfred Tsai.
	*   Don't have to change until using next generation thermal sensors.
	 */

	temp0 = (10000 * 100000 / g_gain) * 15 / 18;

	if (forge_tc_use_stock()) {
		/* stock L681H (the stock kernel 0x5170c8/0x517184): temp0 / (165 +- O_SLOPE) */
		if (g_o_slope_sign == 0)
			temp1 = temp0 / (165 + g_o_slope);
		else
			temp1 = temp0 / (165 - g_o_slope);
	} else if (g_o_slope_sign == 0)
		temp1 = (temp0 * 10) / (1528 + g_o_slope * 10);
	else
		temp1 = (temp0 * 10) / (1528 - g_o_slope * 10);

	ts_ptpod.ts_MTS = temp1;

	temp0 = (g_degc_cali * 10 / 2);
	temp1 = ((10000 * 100000 / 4096 / g_gain) * g_oe + x_roomt * 10) * 15 / 18;

	if (forge_tc_use_stock()) {
		/* stock L681H (the stock kernel 0x5170ec/0x5171a8): temp1 * 10 / (165 +- O_SLOPE) */
		if (g_o_slope_sign == 0)
			temp2 = temp1 * 10 / (165 + g_o_slope);
		else
			temp2 = temp1 * 10 / (165 - g_o_slope);
	} else if (g_o_slope_sign == 0)
		temp2 = temp1 * 100 / (1528 + g_o_slope * 10);
	else
		temp2 = temp1 * 100 / (1528 - g_o_slope * 10);

	ts_ptpod.ts_BTS = (temp0 + temp2 - 250) * 4 / 10;


	ts_info->ts_MTS = ts_ptpod.ts_MTS;
	ts_info->ts_BTS = ts_ptpod.ts_BTS;
	tscpu_dprintk("ts_MTS=%d, ts_BTS=%d\n", ts_ptpod.ts_MTS, ts_ptpod.ts_BTS);
}
EXPORT_SYMBOL(get_thermal_slope_intercept);

/* chip dependent */
void mtkts_dump_cali_info(void)
{
	tscpu_printk("[cal] g_adc_ge_t      = %d\n", g_adc_ge_t);
	tscpu_printk("[cal] g_adc_oe_t      = %d\n", g_adc_oe_t);
	tscpu_printk("[cal] g_degc_cali     = %d\n", g_degc_cali);
	tscpu_printk("[cal] g_adc_cali_en_t = %d\n", g_adc_cali_en_t);
	tscpu_printk("[cal] g_o_slope       = %d\n", g_o_slope);
	tscpu_printk("[cal] g_o_slope_sign  = %d\n", g_o_slope_sign);
	tscpu_printk("[cal] g_id            = %d\n", g_id);

	tscpu_printk("[cal] g_o_vtsmcu1     = %d\n", g_o_vtsmcu1);
	tscpu_printk("[cal] g_o_vtsmcu2     = %d\n", g_o_vtsmcu2);
	tscpu_printk("[cal] g_o_vtsmcu3     = %d\n", g_o_vtsmcu3);
	tscpu_printk("[cal] g_o_vtsmcu4     = %d\n", g_o_vtsmcu4);
	tscpu_printk("[cal] g_o_vtsmcu5     = %d\n", g_o_vtsmcu5);
	tscpu_printk("[cal] g_o_vtsabb     = %d\n", g_o_vtsabb);
}

void eDataCorrector(void)
{
	/* Confirmed with DE Kj Hsiao and DS Lin
	*   ADC_GE_T [9:0]      Default:512   265 ~ 758
	*   ADC_OE_T [9:0]      Default:512   265 ~ 758
	*   O_VTSMCU1(9b)      Default:260   -8 ~ 484
	*   O_VTSMCU2(9b)      Default:260   -8 ~ 484
	*   O_VTSMCU3(9b)      Default:260   (-8) to 484
	*   O_VTSMCU4(9b)      Default:260   (-8) to 484
	*   O_VTSMCU5(9b)      Default:260   TODO : check range
	*   O_VTSABB (9b)       Default:260   -8 ~ 484
	*   DEGC_cali  (6b)       Default:40    1 ~ 63
	*   ADC_CALI_EN_T (1b)
	*   O_SLOPE_SIGN (1b)   Default:0
	*   O_SLOPE (6b)        Default:0
	*   ID (1b)
	*/
	if (g_adc_ge_t < 265 || g_adc_ge_t > 758) {
		tscpu_warn("[thermal] Bad efuse data, g_adc_ge_t\n");
		g_adc_ge_t = 512;
	}
	if (g_adc_oe_t < 265 || g_adc_oe_t > 758) {
		tscpu_warn("[thermal] Bad efuse data, g_adc_oe_t\n");
		g_adc_oe_t = 512;
	}
	if (g_o_vtsmcu1 < -8 || g_o_vtsmcu1 > 484) {
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsmcu1\n");
		g_o_vtsmcu1 = 260;
	}
	if (g_o_vtsmcu2 < -8 || g_o_vtsmcu2 > 484) {
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsmcu2\n");
		g_o_vtsmcu2 = 260;
	}
	if (g_o_vtsmcu3 < -8 || g_o_vtsmcu3 > 484) {
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsmcu3\n");
		g_o_vtsmcu3 = 260;
	}
	if (g_o_vtsmcu4 < -8 || g_o_vtsmcu4 > 484) {
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsmcu4\n");
		g_o_vtsmcu4 = 260;
	}
	if (g_o_vtsmcu5 < -8 || g_o_vtsmcu5 > 484) { /* TODO */
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsmcu5\n");
		g_o_vtsmcu5 = 260;
	}
	if (g_o_vtsabb < -8 || g_o_vtsabb > 484) {
		tscpu_warn("[thermal] Bad efuse data, g_o_vtsabb\n");
		g_o_vtsabb = 260;
	}
	if (g_degc_cali < 1 || g_degc_cali > 63) {
		tscpu_warn("[thermal] Bad efuse data, g_degc_cali\n");
		g_degc_cali = 40;
	}
}
void tscpu_thermal_cal_prepare(void)
{
	__u32 temp0 = 0, temp1 = 0, temp2 = 0;

	temp0 = forge_devinfo_stock(ADDRESS_INDEX_0);
	temp1 = forge_devinfo_stock(ADDRESS_INDEX_1);
	temp2 = forge_devinfo_stock(ADDRESS_INDEX_2);

	/* e.g. 0x7d9f20b4 / 0x176ae61 / 0x56a90231 on a tested M681 */
	pr_info("[FORGE_THERM] cal words (stock idx %d/%d/%d): temp0=0x%x temp1=0x%x temp2=0x%x\n",
		ADDRESS_INDEX_0, ADDRESS_INDEX_1, ADDRESS_INDEX_2, temp0, temp1, temp2);

	/*
	*   chip dependent
	*   ADC_GE_T    [9:0] *(0x10206184)[31:22]
	*   ADC_OE_T    [9:0] *(0x10206184)[21:12]
	*/
	g_adc_ge_t = ((temp0 & _BITMASK_(31:22)) >> 22);
	g_adc_oe_t = ((temp0 & _BITMASK_(21:12)) >> 12);

	/*
	*   O_VTSMCU1    (9b) *(0x10206180)[25:17]
	*   O_VTSMCU2    (9b) *(0x10206180)[16:8]
	*   O_VTSMCU3    (9b) *(0x10206184)[8:0]
	*   O_VTSMCU4    (9b) *(0x10206188)[31:23]
	*   O_VTSMCU5    (9b) *(0x10206188)[13:5]
	*   O_VTSABB     (9b) *(0x10206188)[22:14]
	*/
	g_o_vtsmcu1 = ((temp1 & _BITMASK_(25:17)) >> 17);
	g_o_vtsmcu2 = ((temp1 & _BITMASK_(16:8)) >> 8);
	g_o_vtsmcu3 = (temp0 & _BITMASK_(8:0));
	g_o_vtsmcu4 = ((temp2 & _BITMASK_(31:23)) >> 23);
	g_o_vtsmcu5 = ((temp2 & _BITMASK_(13:5)) >> 5);
	g_o_vtsabb = ((temp2 & _BITMASK_(22:14)) >> 14);

	/*
	*   DEGC_cali    (6b) *(0x10206180)[6:1]
	*   ADC_CALI_EN_T(1b) *(0x10206180)[0]
	*/
	g_degc_cali = ((temp1 & _BITMASK_(6:1)) >> 1);
	g_adc_cali_en_t = (temp1 & _BIT_(0));

	/*
	*   O_SLOPE_SIGN (1b) *(0x10206180)[7]
	*   O_SLOPE      (6b) *(0x10206180)[31:26]
	*/
	g_o_slope_sign = ((temp1 & _BIT_(7)) >> 7);
	g_o_slope = ((temp1 & _BITMASK_(31:26)) >> 26);

	/*ID           (1b) *(0x10206184)[9] */
	g_id = ((temp0 & _BIT_(9)) >> 9);

	/*
	*   Check ID bit
	*   If ID=0 (TSMC sample)    , ignore O_SLOPE EFuse value and set O_SLOPE=0.
	*   If ID=1 (non-TSMC sample), read O_SLOPE EFuse value for following calculation.
	*/
	if (g_id == 0)
		g_o_slope = 0;


	if (g_adc_cali_en_t == 1) {
		/*thermal_enable = true; */
		eDataCorrector();
	} else {
		tscpu_warn("This sample is not Thermal calibrated\n");
		g_adc_ge_t = 512;
		g_adc_oe_t = 512;
		g_o_vtsmcu1 = 260;
		g_o_vtsmcu2 = 260;
		g_o_vtsmcu3 = 260;
		g_o_vtsmcu4 = 260;
		g_o_vtsmcu5 = 260;
		g_o_vtsabb = 260;
		g_degc_cali = 40;
		g_o_slope_sign = 0;
		g_o_slope = 0;
	}

	mtkts_dump_cali_info();
}

void tscpu_thermal_cal_prepare_2(__u32 ret)
{
	__s32 format_1 = 0, format_2 = 0, format_3 = 0, format_4 = 0, format_5 = 0, format_6 = 0;

	/* tscpu_printk("tscpu_thermal_cal_prepare_2\n"); */
	g_ge = ((g_adc_ge_t - 512) * 10000) / 4096;	/* ge * 10000 */
	g_oe = (g_adc_oe_t - 512);

	g_gain = (10000 + g_ge);

	format_1 = (g_o_vtsmcu1 + 3350 - g_oe);
	format_2 = (g_o_vtsmcu2 + 3350 - g_oe);
	format_3 = (g_o_vtsmcu3 + 3350 - g_oe);
	format_4 = (g_o_vtsmcu4 + 3350 - g_oe);
	format_5 = (g_o_vtsmcu5 + 3350 - g_oe);
	format_6 = (g_o_vtsabb + 3350 - g_oe);


	g_x_roomt[0] = (((format_1 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */
	g_x_roomt[1] = (((format_2 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */
	g_x_roomt[2] = (((format_3 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */
	g_x_roomt[3] = (((format_4 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */
	g_x_roomt[4] = (((format_5 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */
	g_x_roomt[5] = (((format_6 * 10000) / 4096) * 10000) / g_gain;	/* x_roomt * 10000 */

	/*
	*   tscpu_printk("[cal] g_ge         = 0x%x\n",g_ge);
	*   tscpu_printk("[cal] g_gain       = 0x%x\n",g_gain);
	*
	*   tscpu_printk("[cal] g_x_roomt1   = 0x%x\n",g_x_roomt[0]);
	*   tscpu_printk("[cal] g_x_roomt2   = 0x%x\n",g_x_roomt[1]);
	*   tscpu_printk("[cal] g_x_roomt3   = 0x%x\n",g_x_roomt[2]);
	*   tscpu_printk("[cal] g_x_roomt4   = 0x%x\n",g_x_roomt[3]);
	*   tscpu_printk("[cal] g_x_roomt5   = 0x%x\n",g_x_roomt[4]);
	*/

	pr_info("[FORGE_TC] slope=%s: 165 (stock L681H/m681) / 1528 (mt6757); forge_tc_stock_slope=%d l681=%d m681=%d\n",
		forge_tc_use_stock() ? "165" : "1528", forge_tc_stock_slope,
		m3note_board_known_is(M3NOTE_BOARD_L681),
		m3note_board_known_is(M3NOTE_BOARD_M681));
	/* raw codes are cached by the zone poll; print both scales once later */
	schedule_delayed_work(&forge_tc_scales_work, 20 * HZ);
}

#if THERMAL_CONTROLLER_HW_TP
static __s32 temperature_to_raw_room(__u32 ret, ts_e ts_name)
{
	/* Ycurr = [(Tcurr - DEGC_cali/2)*(1528+O_slope*10)/10*(18/15)*(1/10000)+X_roomtabb]*Gain*4096 + OE */

	__s32 t_curr = ret;
	__s32 format_1 = 0;
	__s32 format_2 = 0;
	__s32 format_3 = 0;
	__s32 format_4 = 0;

	/* tscpu_dprintk("temperature_to_raw_room\n"); */

	if (g_o_slope_sign == 0) {	/* O_SLOPE is Positive. */
		format_1 = t_curr - (g_degc_cali * 1000 / 2);
		if (forge_tc_use_stock())	/* stock formula */
			format_2 = format_1 * (165 + g_o_slope) * 18 / 15;
		else
			format_2 = format_1 * (1528 + g_o_slope * 10) / 10 * 18 / 15;
		format_2 = format_2 - 2 * format_2;

		format_3 = format_2 / 1000 + g_x_roomt[ts_name] * 10;
		format_4 = (format_3 * 4096 / 10000 * g_gain) / 100000 + g_oe;
	} else {		/* O_SLOPE is Negative. */
		format_1 = t_curr - (g_degc_cali * 1000 / 2);
		if (forge_tc_use_stock())	/* stock formula */
			format_2 = format_1 * (165 - g_o_slope) * 18 / 15;
		else
			format_2 = format_1 * (1528 - g_o_slope * 10) / 10 * 18 / 15;
		format_2 = format_2 - 2 * format_2;

		format_3 = format_2 / 1000 + g_x_roomt[ts_name] * 10;
		format_4 = (format_3 * 4096 / 10000 * g_gain) / 100000 + g_oe;
	}

	return format_4;
}
#endif

/* stock != 0: stock L681H slope 165 (the stock kernel read_tc_raw_and_temp
 * 0x516dcc/0x516e20: format_3 * 100 / (165 +- O_SLOPE)); 0: mt6757 1528. */
static __s32 forge_raw_to_temperature_roomt(__u32 ret, ts_e ts_name, int stock)
{
	__s32 t_current = 0;
	__s32 y_curr = ret;
	__s32 format_1 = 0;
	__s32 format_2 = 0;
	__s32 format_3 = 0;
	__s32 format_4 = 0;
	__s32 xtoomt = 0;


	xtoomt = g_x_roomt[ts_name];

	/* tscpu_dprintk("raw_to_temperature_room,ts_num=%d,xtoomt=%d\n",ts_name,xtoomt); */

	if (ret == 0)
		return 0;

	format_1 = ((g_degc_cali * 10) >> 1);
	format_2 = (y_curr - g_oe);

	format_3 = (((((format_2) * 10000) >> 12) * 10000) / g_gain) - xtoomt;
	format_3 = format_3 * 15 / 18;


	if (stock) {
		if (g_o_slope_sign == 0)
			format_4 = ((format_3 * 100) / (165 + g_o_slope));	/* uint = 0.1 deg */
		else
			format_4 = ((format_3 * 100) / (165 - g_o_slope));	/* uint = 0.1 deg */
	} else if (g_o_slope_sign == 0)
		format_4 = ((format_3 * 1000) / (1528 + g_o_slope * 10));	/* uint = 0.1 deg */
	else
		format_4 = ((format_3 * 1000) / (1528 - g_o_slope * 10));	/* uint = 0.1 deg */

	format_4 = format_4 - (format_4 << 1);


	t_current = format_1 + format_4;	/* uint = 0.1 deg */

	/* tscpu_dprintk("raw_to_temperature_room,t_current=%d\n",t_current); */
	return t_current;
}

static __s32 raw_to_temperature_roomt(__u32 ret, ts_e ts_name)
{
	return forge_raw_to_temperature_roomt(ret, ts_name, forge_tc_use_stock());
}

/*
 * Scale cross-check: the CPU sensors are BANK0[MCU4] (mp0) and BANK2[MCU4]
 * (mp1). Print the last raw codes and both scales in one line; read-only
 * (no TC/PMIC register access, only the cached raw codes).
 */
static int forge_tc_scales_fmt(char *buf, size_t len)
{
	int r0 = tscpu_bank_ts_r[THERMAL_BANK0][TS_MCU4];
	int r2 = tscpu_bank_ts_r[THERMAL_BANK2][TS_MCU4];

	return scnprintf(buf, len,
		"[FORGE_TC] scales: active=%s raw mp0=%d mp1=%d | stock165 mp0=%d mp1=%d | mt6757_1528 mp0=%d mp1=%d (0.1 C)\n",
		forge_tc_use_stock() ? "165" : "1528", r0, r2,
		forge_raw_to_temperature_roomt(r0, TS_MCU4, 1),
		forge_raw_to_temperature_roomt(r2, TS_MCU4, 1),
		forge_raw_to_temperature_roomt(r0, TS_MCU4, 0),
		forge_raw_to_temperature_roomt(r2, TS_MCU4, 0));
}

static int forge_tc_scales_get(char *buffer, const struct kernel_param *kp)
{
	return forge_tc_scales_fmt(buffer, PAGE_SIZE);
}

static const struct kernel_param_ops forge_tc_scales_ops = {
	.get = forge_tc_scales_get,
};
module_param_cb(forge_tc_scales, &forge_tc_scales_ops, NULL, 0444);
MODULE_PARM_DESC(forge_tc_scales, "l681: last CPU sensor raw codes and temperature in both slopes");

static void forge_tc_scales_work_fn(struct work_struct *work)
{
	char line[192];

	forge_tc_scales_fmt(line, sizeof(line));
	pr_info("%s", line);
}

/*
*Bank0 : CPU_mp0     (TS_MCU4)
*Bank1 : SOC+GPU     (TS_MCU1, TS_MCU3, TS_MCU2)
*Bank2 : CPU_mp1     (TS_MCU4)
*/
/* All wrappers below use the MT6755 three-bank layout, as in the 3.18
 * MT6755 mtk_tc.c: cpu0 = BANK0[MCU4], gpu = max(BANK1 MCU1/MCU3/MCU2),
 * cpu1 = BANK2[MCU4], ts1/ts2/ts3 = BANK1[MCU1/MCU2/MCU3], ts4 =
 * BANK0[MCU4]. The MT6757 names (cpuL/cpuLL/mcucci) are kept for link
 * parity: cpuLL -> mp0, cpuL -> mp1, mcucci -> max(both CPU sensors). ts5
 * has no sensor (no TS_MCU5). */
/* chip dependent */
int get_immediate_gpu_wrap(void)
{
	int curr_temp;

	curr_temp = MAX(tscpu_bank_ts[THERMAL_BANK1][TS_MCU1], tscpu_bank_ts[THERMAL_BANK1][TS_MCU3]);
	curr_temp = MAX(curr_temp, tscpu_bank_ts[THERMAL_BANK1][TS_MCU2]);

	tscpu_dprintk("get_immediate_gpu_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_cpuL_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK2][TS_MCU4];
	tscpu_dprintk("get_immediate_cpuL_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_cpuLL_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK0][TS_MCU4];
	tscpu_dprintk("get_immediate_cpuLL_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_mcucci_wrap(void)
{
	int curr_temp;

	curr_temp = MAX(tscpu_bank_ts[THERMAL_BANK0][TS_MCU4], tscpu_bank_ts[THERMAL_BANK2][TS_MCU4]);
	tscpu_dprintk("get_immediate_mcucci_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}
/*
*Bank0 : CPU_mp0     (TS_MCU4)
*Bank1 : SOC+GPU     (TS_MCU1, TS_MCU3, TS_MCU2)
*Bank2 : CPU_mp1     (TS_MCU4)
*/
/* chip dependent */
int get_immediate_ts1_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK1][TS_MCU1];
	tscpu_dprintk("get_immediate_ts1_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_ts2_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK1][TS_MCU2];
	tscpu_dprintk("get_immediate_ts2_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_ts3_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK1][TS_MCU3];
	tscpu_dprintk("get_immediate_ts3_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_ts4_wrap(void)
{
	int curr_temp;

	curr_temp = tscpu_bank_ts[THERMAL_BANK0][TS_MCU4];
	tscpu_dprintk("get_immediate_ts4_wrap curr_temp=%d\n", curr_temp);

	return curr_temp;
}

int get_immediate_ts5_wrap(void)
{
	/* MT6755 has no TS_MCU5 sensor: report "no sensor". */
	return -127000;
}

int get_immediate_none_wrap(void)
{
	return -127000;
}

int (*get_immediate_tsX[TS_ENUM_MAX])(void) = {
	get_immediate_ts1_wrap,
	get_immediate_ts2_wrap,
	get_immediate_ts3_wrap,
	get_immediate_ts4_wrap,
	get_immediate_none_wrap,	/* TS_MCU5 absent */
	get_immediate_none_wrap
};

static void thermal_interrupt_handler(int bank)
{
	__u32 ret = 0;
	unsigned long flags;

#if THERMAL_INFORM_OTP
	int temp = 0;
#endif

	mt_ptp_lock(&flags);

	tscpu_switch_bank(bank);

	ret = readl(TEMPMONINTSTS);
	tscpu_dprintk("[tIRQ] thermal_interrupt_handler,bank=0x%08x,ret=0x%08x\n", bank, ret);

#if THERMAL_INFORM_OTP
	temp = get_immediate_big_wrap();
	tscpu_dprintk("[tIRQ] thermal_interrupt_handler,BIG T=%d\n", temp);

	if (ret & THERMAL_MON_LOINTSTS0 ||
		(!ret && temp < (OTP_LOW_OFFSET_TEMP + OTP_TEMP_TOLERANCE))) {
		tscpu_dprintk("[tIRQ] thermal_isr: THERMAL_MON_LOINTSTS0\n");
		/**************************************************
		*	disable OTP here
		*************************************************
		*/
		BigOTPThermIRQ(0);
		temp = readl(TEMPMONINT);
		temp &= ~0x00000004;/*disable low offset*/
		temp |=  0x00000008;/*enable high offset*/

		mt_reg_sync_writel(temp, TEMPMONINT);	/* set high offset */

	}

	if (ret & THERMAL_MON_HOINTSTS0 ||
		(!ret && (OTP_HIGH_OFFSET_TEMP - OTP_TEMP_TOLERANCE) < temp)) {
		tscpu_dprintk("[tIRQ] thermal_isr: THERMAL_MON_HOINTSTS0\n");
		/**************************************************
		*	enable OTP here
		*************************************************
		*/
		BigOTPThermIRQ(1);
		temp = readl(TEMPMONINT);
		temp &= ~0x00000008;/*disable high offset*/
		temp |=  0x00000004;/*enable low offset*/

		mt_reg_sync_writel(temp, TEMPMONINT);	/* set low offset */

	}
#endif
	/* for SPM reset debug */
	/* dump_spm_reg(); */

	/* tscpu_printk("thermal_isr: [Interrupt trigger]: status = 0x%x\n", ret); */
	if (ret & THERMAL_MON_CINTSTS0)
		tscpu_dprintk("thermal_isr: thermal sensor point 0 - cold interrupt trigger\n");

	if (ret & THERMAL_MON_HINTSTS0)
		tscpu_dprintk("<<<thermal_isr>>>: thermal sensor point 0 - hot interrupt trigger\n");

	if (ret & THERMAL_MON_HINTSTS1)
		tscpu_dprintk("<<<thermal_isr>>>: thermal sensor point 1 - hot interrupt trigger\n");

	if (ret & THERMAL_MON_HINTSTS2)
		tscpu_dprintk("<<<thermal_isr>>>: thermal sensor point 2 - hot interrupt trigger\n");

	if (ret & THERMAL_tri_SPM_State0)
		tscpu_dprintk("thermal_isr: Thermal state0 to trigger SPM state0\n");

	if (ret & THERMAL_tri_SPM_State1) {
		/* tscpu_printk("thermal_isr: Thermal state1 to trigger SPM state1\n"); */
#if MTK_TS_CPU_RT
		/* tscpu_printk("THERMAL_tri_SPM_State1, T=%d,%d,%d\n", CPU_TS_MCU2_T, GPU_TS_MCU1_T, LTE_TS_MCU3_T); */
		wake_up_process(ktp_thread_handle);
#endif
	}
	if (ret & THERMAL_tri_SPM_State2)
		tscpu_printk("thermal_isr: Thermal state2 to trigger SPM state2\n");

	mt_ptp_unlock(&flags);

}

irqreturn_t tscpu_thermal_all_bank_interrupt_handler(int irq, void *dev_id)
{
	__u32 ret = 0, i, mask = 1;


	ret = readl(THERMINTST);
	ret = ret & 0xFF;

	tscpu_dprintk("thermal_interrupt_handler : THERMINTST = 0x%x\n", ret);

	for (i = 0; i < ARRAY_SIZE(tscpu_g_bank); i++) {
		mask = 1 << i;
		if ((ret & mask) == 0)
			thermal_interrupt_handler(i);
	}

	return IRQ_HANDLED;
}

static void thermal_reset_and_initial(void)
{

	/* pr_debug( "thermal_reset_and_initial\n"); */

	/* tscpu_thermal_clock_on(); */

	/* Calculating period unit in Module clock x 256, and the Module clock */
	/* will be changed to 26M when Infrasys enters Sleep mode. */


	/*bus clock 66M counting unit is 12 * 1/66M * 256 = 12 * 3.879us = 46.545 us */
	mt_reg_sync_writel(0x0000000C, TEMPMONCTL1);
	/* filt interval is 1 * 46.545us = 46.545us,
	*sen interval is 429 * 46.545us = 19.968ms
	*/
	mt_reg_sync_writel(0x000101AD, TEMPMONCTL2);
	/*AHB polling is 781* 1/66M = 11.833us*/
	mt_reg_sync_writel(0x0000030D, TEMPAHBPOLL);	/* poll is set to 11.833us */

#if THERMAL_CONTROLLER_HW_FILTER == 2
	mt_reg_sync_writel(0x00000492, TEMPMSRCTL0);	/* temperature sampling control, 2 out of 4 samples */
#elif THERMAL_CONTROLLER_HW_FILTER == 4
	mt_reg_sync_writel(0x000006DB, TEMPMSRCTL0);	/* temperature sampling control, 4 out of 6 samples */
#elif THERMAL_CONTROLLER_HW_FILTER == 8
	mt_reg_sync_writel(0x00000924, TEMPMSRCTL0);	/* temperature sampling control, 8 out of 10 samples */
#elif THERMAL_CONTROLLER_HW_FILTER == 16
	mt_reg_sync_writel(0x00000B6D, TEMPMSRCTL0);	/* temperature sampling control, 16 out of 18 samples */
#else				/* default 1 */
	mt_reg_sync_writel(0x00000000, TEMPMSRCTL0);	/* temperature sampling control, 1 sample */
#endif
	/* polling time of a bank: 1 sensors 20.155ms, 2 sensors 40.311ms, 3 sensors  60.465ms */

	mt_reg_sync_writel(0xFFFFFFFF, TEMPAHBTO);	/* exceed this polling time, IRQ would be inserted */

	mt_reg_sync_writel(0x00000000, TEMPMONIDET0);	/* times for interrupt occurrance */
	mt_reg_sync_writel(0x00000000, TEMPMONIDET1);	/* times for interrupt occurrance */


	/* this value will be stored to TEMPPNPMUXADDR (TEMPSPARE0) automatically by hw */
	mt_reg_sync_writel(0x800, TEMPADCMUX);
	mt_reg_sync_writel((__u32) AUXADC_CON1_CLR_P, TEMPADCMUXADDR);	/* AHB address for auxadc mux selection */
	/* mt_reg_sync_writel(0x1100100C, TEMPADCMUXADDR);// AHB address for auxadc mux selection */

	mt_reg_sync_writel(0x800, TEMPADCEN);	/* AHB value for auxadc enable */
	/* AHB address for auxadc enable (channel 0 immediate mode selected) */
	mt_reg_sync_writel((__u32) AUXADC_CON1_SET_P, TEMPADCENADDR);
	/* mt_reg_sync_writel(0x11001008, TEMPADCENADDR);
	 *AHB address for auxadc enable (channel 0 immediate mode selected)
	 *this value will be stored to TEMPADCENADDR automatically by hw
	 */


	mt_reg_sync_writel((__u32) AUXADC_DAT11_P, TEMPADCVALIDADDR);	/* AHB address for auxadc valid bit */
	mt_reg_sync_writel((__u32) AUXADC_DAT11_P, TEMPADCVOLTADDR);	/* AHB address for auxadc voltage output */
	/* mt_reg_sync_writel(0x11001040, TEMPADCVALIDADDR); // AHB address for auxadc valid bit */
	/* mt_reg_sync_writel(0x11001040, TEMPADCVOLTADDR);  // AHB address for auxadc voltage output */


	mt_reg_sync_writel(0x0, TEMPRDCTRL);	/* read valid & voltage are at the same register */
	/* indicate where the valid bit is (the 12th bit is valid bit and 1 is valid) */
	mt_reg_sync_writel(0x0000002C, TEMPADCVALIDMASK);
	mt_reg_sync_writel(0x0, TEMPADCVOLTAGESHIFT);	/* do not need to shift */
	/* mt_reg_sync_writel(0x2, TEMPADCWRITECTRL);                     // enable auxadc mux write transaction */

}



/**
 *  temperature2 to set the middle threshold for interrupting CPU. -275000 to disable it.
 */
static void set_tc_trigger_hw_protect(int temperature, int temperature2, thermal_bank_name bank)
{
	int temp = 0;
	int raw_high;
	ts_e ts_name;

	/* temperature2=80000;  test only */
	tscpu_dprintk("set_tc_trigger_hw_protect t1=%d t2=%d\n", temperature, temperature2);

	ts_name = tscpu_g_bank[bank].ts[0].type;

	/* temperature to trigger SPM state2 */
	raw_high = temperature_to_raw_room(temperature, ts_name);

	temp = readl(TEMPMONINT);
	mt_reg_sync_writel(temp & 0x00000000, TEMPMONINT);	/* disable trigger SPM interrupt */


	mt_reg_sync_writel(0x20000, TEMPPROTCTL);	/* set hot to wakeup event control */

	mt_reg_sync_writel(raw_high, TEMPPROTTC);	/* set hot to HOT wakeup event */


	mt_reg_sync_writel(temp | 0x80000000, TEMPMONINT);	/* enable trigger Hot SPM interrupt */
}


static int read_tc_raw_and_temp(volatile u32 *tempmsr_name, ts_e ts_name,
				int *ts_raw)
{
	int temp = 0, raw = 0;

	raw = (tempmsr_name != 0) ? (readl((tempmsr_name)) & 0x0fff) : 0;
	temp = (tempmsr_name != 0) ? raw_to_temperature_roomt(raw, ts_name) : 0;

	*ts_raw = raw;
	tscpu_dprintk("read_tc_raw_temp,ts_raw=%d,temp=%d\n", *ts_raw, temp * 100);

	/*fake temperature for testing*/
	if ((tscpu_fake_temp_enable == 1) && (tscpu_fake_temp != 0x7FFFFFFF)) {
		/*printk_ratelimited(TSCPU_LOG_TAG "tscpu_fake_temp=%d enable=%d\n",
		*	tscpu_fake_temp, tscpu_fake_temp_enable);
		*/
		return tscpu_fake_temp;
	}

	return temp * 100;
}


void tscpu_thermal_read_bank_temp(thermal_bank_name bank, ts_e type, int order)
{

	tscpu_dprintk("%s bank %d type %d order %d\n", __func__, bank, type, order);

	switch (order) {
	case 0:
		tscpu_bank_ts[bank][type] =
		    read_tc_raw_and_temp((volatile u32 *)TEMPMSR0, type,
					 &tscpu_bank_ts_r[bank][type]);
		tscpu_dprintk("%s order %d bank %d type %d tscpu_bank_ts %d tscpu_bank_ts_r %d\n",
			      __func__, order, bank, type, tscpu_bank_ts[bank][type],
			      tscpu_bank_ts_r[bank][type]);
		break;
	case 1:
		tscpu_bank_ts[bank][type] =
		    read_tc_raw_and_temp((volatile u32 *)TEMPMSR1, type,
					 &tscpu_bank_ts_r[bank][type]);
		tscpu_dprintk("%s order %d bank %d type %d tscpu_bank_ts %d tscpu_bank_ts_r %d\n",
			      __func__, order, bank, type, tscpu_bank_ts[bank][type],
			      tscpu_bank_ts_r[bank][type]);
		break;
	case 2:
		tscpu_bank_ts[bank][type] =
		    read_tc_raw_and_temp((volatile u32 *)TEMPMSR2, type,
					 &tscpu_bank_ts_r[bank][type]);
		tscpu_dprintk("%s order %d bank %d type %d tscpu_bank_ts %d tscpu_bank_ts_r %d\n",
			      __func__, order, bank, type, tscpu_bank_ts[bank][type],
			      tscpu_bank_ts_r[bank][type]);
		break;
	case 3:
		tscpu_bank_ts[bank][type] =
		    read_tc_raw_and_temp((volatile u32 *)TEMPMSR3, type,
					 &tscpu_bank_ts_r[bank][type]);
		tscpu_dprintk("%s order %d bank %d type %d tscpu_bank_ts %d tscpu_bank_ts_r %d\n",
			      __func__, order, bank, type, tscpu_bank_ts[bank][type],
			      tscpu_bank_ts_r[bank][type]);
		break;
	default:
		tscpu_bank_ts[bank][type] =
		    read_tc_raw_and_temp((volatile u32 *)TEMPMSR0, type,
					 &tscpu_bank_ts_r[bank][type]);
		tscpu_dprintk("%s order %d bank %d type %d tscpu_bank_ts %d tscpu_bank_ts_r %d\n",
			      __func__, order, bank, type, tscpu_bank_ts[bank][type],
			      tscpu_bank_ts_r[bank][type]);
		break;
	}
}

int tscpu_thermal_fast_init(void)
{
	__u32 temp = 0;
	__u32 cunt = 0;
	/* __u32 temp1 = 0,temp2 = 0,temp3 = 0,count=0; */

	/* tscpu_printk( "tscpu_thermal_fast_init\n"); */


	temp = THERMAL_INIT_VALUE;
	mt_reg_sync_writel((0x00001000 + temp), PTPSPARE2);	/* write temp to spare register */

	mt_reg_sync_writel(1, TEMPMONCTL1);	/* counting unit is 320 * 31.25us = 10ms */
	mt_reg_sync_writel(1, TEMPMONCTL2);	/* sensing interval is 200 * 10ms = 2000ms */
	mt_reg_sync_writel(1, TEMPAHBPOLL);	/* polling interval to check if temperature sense is ready */

	mt_reg_sync_writel(0x000000FF, TEMPAHBTO);	/* exceed this polling time, IRQ would be inserted */
	mt_reg_sync_writel(0x00000000, TEMPMONIDET0);	/* times for interrupt occurrance */
	mt_reg_sync_writel(0x00000000, TEMPMONIDET1);	/* times for interrupt occurrance */

	mt_reg_sync_writel(0x0000000, TEMPMSRCTL0);	/* temperature measurement sampling control */

	/* this value will be stored to TEMPPNPMUXADDR (TEMPSPARE0) automatically by hw */
	mt_reg_sync_writel(0x1, TEMPADCPNP0);
	mt_reg_sync_writel(0x2, TEMPADCPNP1);
	mt_reg_sync_writel(0x3, TEMPADCPNP2);
	mt_reg_sync_writel(0x4, TEMPADCPNP3);

#if 0
	mt_reg_sync_writel(0x1100B420, TEMPPNPMUXADDR);	/* AHB address for pnp sensor mux selection */
	mt_reg_sync_writel(0x1100B420, TEMPADCMUXADDR);	/* AHB address for auxadc mux selection */
	mt_reg_sync_writel(0x1100B424, TEMPADCENADDR);	/* AHB address for auxadc enable */
	mt_reg_sync_writel(0x1100B428, TEMPADCVALIDADDR);	/* AHB address for auxadc valid bit */
	mt_reg_sync_writel(0x1100B428, TEMPADCVOLTADDR);	/* AHB address for auxadc voltage output */

#else
	mt_reg_sync_writel((__u32) PTPSPARE0_P, TEMPPNPMUXADDR);	/* AHB address for pnp sensor mux selection */
	mt_reg_sync_writel((__u32) PTPSPARE0_P, TEMPADCMUXADDR);	/* AHB address for auxadc mux selection */
	mt_reg_sync_writel((__u32) PTPSPARE1_P, TEMPADCENADDR);	/* AHB address for auxadc enable */
	mt_reg_sync_writel((__u32) PTPSPARE2_P, TEMPADCVALIDADDR);	/* AHB address for auxadc valid bit */
	mt_reg_sync_writel((__u32) PTPSPARE2_P, TEMPADCVOLTADDR);	/* AHB address for auxadc voltage output */
#endif
	mt_reg_sync_writel(0x0, TEMPRDCTRL);	/* read valid & voltage are at the same register */
	/* indicate where the valid bit is (the 12th bit is valid bit and 1 is valid) */
	mt_reg_sync_writel(0x0000002C, TEMPADCVALIDMASK);
	mt_reg_sync_writel(0x0, TEMPADCVOLTAGESHIFT);	/* do not need to shift */
	mt_reg_sync_writel(0x3, TEMPADCWRITECTRL);	/* enable auxadc mux & pnp write transaction */


	/* enable all interrupt except filter sense and immediate sense interrupt */
	mt_reg_sync_writel(0x00000000, TEMPMONINT);


	mt_reg_sync_writel(0x0000000F, TEMPMONCTL0);	/* enable all sensing point (sensing point 2 is unused) */


	cunt = 0;
	temp = readl(TEMPMSR0) & 0x0fff;
	while (temp != THERMAL_INIT_VALUE && cunt < 20) {
		cunt++;
		/* pr_debug("[Power/CPU_Thermal]0 temp=%d,cunt=%d\n",temp,cunt); */
		temp = readl(TEMPMSR0) & 0x0fff;
	}

	cunt = 0;
	temp = readl(TEMPMSR1) & 0x0fff;
	while (temp != THERMAL_INIT_VALUE && cunt < 20) {
		cunt++;
		/* pr_debug("[Power/CPU_Thermal]1 temp=%d,cunt=%d\n",temp,cunt); */
		temp = readl(TEMPMSR1) & 0x0fff;
	}

	cunt = 0;
	temp = readl(TEMPMSR2) & 0x0fff;
	while (temp != THERMAL_INIT_VALUE && cunt < 20) {
		cunt++;
		/* pr_debug("[Power/CPU_Thermal]2 temp=%d,cunt=%d\n",temp,cunt); */
		temp = readl(TEMPMSR2) & 0x0fff;
	}

	cunt = 0;
	temp = readl(TEMPMSR3) & 0x0fff;
	while (temp != THERMAL_INIT_VALUE && cunt < 20) {
		cunt++;
		/* pr_debug("[Power/CPU_Thermal]3 temp=%d,cunt=%d\n",temp,cunt); */
		temp = readl(TEMPMSR3) & 0x0fff;
	}




	return 0;
}

int tscpu_switch_bank(thermal_bank_name bank)
{
	/* tscpu_dprintk( "tscpu_switch_bank =bank=%d\n",bank); */

	switch (bank) {
	case THERMAL_BANK0:
		thermal_clrl(PTPCORESEL, 0xF);
		thermal_setl(PTPCORESEL, 0x000F0000);	/* bank0 */
		break;
	case THERMAL_BANK1:
		thermal_clrl(PTPCORESEL, 0xF);
		thermal_setl(PTPCORESEL, 0x000F0001);	/* bank1 */
		break;
	case THERMAL_BANK2:
		thermal_clrl(PTPCORESEL, 0xF);
		thermal_setl(PTPCORESEL, 0x000F0002);	/* bank2 */
		break;
	case THERMAL_BANK3:
		thermal_clrl(PTPCORESEL, 0xF);
		thermal_setl(PTPCORESEL, 0x000F0003);	/* bank3 */
		break;
	default:
		thermal_clrl(PTPCORESEL, 0xF);
		thermal_setl(PTPCORESEL, 0x000F0000);	/* bank0 */
		break;
	}
	if (bank < THERMAL_BANK_NUM)
		g_currentBank = bank;
	else
		g_currentBank = THERMAL_BANK0;

	return 0;
}

int tscpu_thermal_ADCValueOfMcu(enum thermal_sensor_enum type)
{
	switch (type) {
	case TS_MCU1:
		return TEMPADC_MCU1;
	case TS_MCU2:
		return TEMPADC_MCU2;
	case TS_MCU3:
		return TEMPADC_MCU3;
	case TS_MCU4:
		return TEMPADC_MCU4;
	case TS_MCU5:
		return TEMPADC_MCU5;
	case TS_ABB:
		return TEMPADC_ABB;
	default:
		return TEMPADC_MCU1;
	}
}

void tscpu_thermal_initial_all_bank(void)
{
	unsigned long flags;
	int i, j = 0;
	int ret = -1;
	__u32 temp = 0;

	/* AuxADC Initialization,ref MT6592_AUXADC.doc // TODO: check this line */
	temp = readl(AUXADC_CON0_V);	/* Auto set enable for CH11 */
	temp &= 0xFFFFF7FF;	/* 0: Not AUTOSET mode */
	mt_reg_sync_writel(temp, AUXADC_CON0_V);	/* disable auxadc channel 11 synchronous mode */
	mt_reg_sync_writel(0x800, AUXADC_CON1_CLR_V);	/* disable auxadc channel 11 immediate mode */

	for (i = 0; i < ARRAY_SIZE(tscpu_g_bank); i++) {
		mt_ptp_lock(&flags);

		ret = tscpu_switch_bank(i);
		thermal_reset_and_initial();

		for (j = 0; j < tscpu_g_bank[i].ts_number; j++) {
			tscpu_dprintk("%s i %d, j %d\n", __func__, i, j);
			tscpu_thermal_tempADCPNP(tscpu_thermal_ADCValueOfMcu
					(tscpu_g_bank[i].ts[j].type), j);
		}

		mt_reg_sync_writel(TS_CONFIGURE_P, TEMPPNPMUXADDR);
		mt_reg_sync_writel(0x3, TEMPADCWRITECTRL);

		mt_ptp_unlock(&flags);
	}

	mt_reg_sync_writel(0x800, AUXADC_CON1_SET_V);

	for (i = 0; i < ARRAY_SIZE(tscpu_g_bank); i++) {
		mt_ptp_lock(&flags);

		ret = tscpu_switch_bank(i);
		tscpu_thermal_enable_all_periodoc_sensing_point(i);

		mt_ptp_unlock(&flags);
	}
}

#if THERMAL_INFORM_OTP
void tscpu_config_tc_sw_protect(int highoffset, int lowoffset)
{
	int raw_highoffset = 0, raw_lowoffsett = 0, temp = 0;
	ts_e ts_name;
	unsigned long flags;


	ts_name = tscpu_g_bank[THERMAL_BANK0].ts[0].type;

	tscpu_warn("[tIRQ] tscpu_config_tc_sw_protect,highoffset=%d,lowoffset=%d,bank=%d,ts_name=%d\n",
		      highoffset, lowoffset, THERMAL_BANK0, ts_name);


	mt_ptp_lock(&flags);

	tscpu_switch_bank(THERMAL_BANK0);


	if (lowoffset != 0) {
		raw_lowoffsett = temperature_to_raw_room(lowoffset, ts_name);
		mt_reg_sync_writel(raw_lowoffsett, TEMPOFFSETL);

		temp = readl(TEMPMONINT);
		mt_reg_sync_writel(temp | 0x00000004, TEMPMONINT);	/* set low offset */
	}

	if (highoffset != 0) {
		raw_highoffset = temperature_to_raw_room(highoffset, ts_name);
		mt_reg_sync_writel(raw_highoffset, TEMPOFFSETH);

		temp = readl(TEMPMONINT);
		mt_reg_sync_writel(temp | 0x00000008, TEMPMONINT);	/* set high offset */
	}


	mt_ptp_unlock(&flags);
}
#endif

void tscpu_config_all_tc_hw_protect(int temperature, int temperature2)
{
	int i = 0;
	int wd_api_ret;
	unsigned long flags;
	struct wd_api *wd_api;

	tscpu_dprintk("tscpu_config_all_tc_hw_protect,temperature=%d,temperature2=%d,\n",
		      temperature, temperature2);

#if THERMAL_PERFORMANCE_PROFILE
	struct timeval begin, end;
	unsigned long val;

	do_gettimeofday(&begin);
#endif
	/*spend 860~1463 us */
	/*Thermal need to config to direct reset mode
	*this API provide by Weiqi Fu(RGU SW owner).
	*/

	wd_api_ret = get_wd_api(&wd_api);
	if (wd_api_ret >= 0) {
		wd_api->wd_thermal_direct_mode_config(WD_REQ_DIS, WD_REQ_RST_MODE);	/* reset mode */
	} else {
		tscpu_warn("%d FAILED TO GET WD API\n", __LINE__);
		WARN_ON_ONCE(1);
	}

	/* Below this point the trip temperatures are converted to raw
	 * thresholds (set_tc_trigger_hw_protect) and the RGU thermal-direct
	 * reset line is armed (WD_REQ_EN): a reset issued by the thermal
	 * controller's autonomous AUXADC sampling that bypasses all WDT
	 * petting and prints nothing. Never arm the HW protection from an
	 * uncalibrated sample; the DIS above has already dropped any earlier
	 * arming, and the SW zones keep polling. */
	if (g_adc_cali_en_t != 1) {
		pr_err("[FORGE_M681] thermal HW-protect NOT armed: uncalibrated sample (adc_cali_en_t=%d ge_t=%d) — RGU thermal-direct reset stays DISABLED\n",
		       g_adc_cali_en_t, g_adc_ge_t);
		return;
	}

	/* Even with a calibrated sample, RGU thermal-direct arming is
	 * opt-in: the late_initcall_sync quiesce clears all RGU request lines
	 * (THERMAL included) and its heartbeat watch re-clears any re-armer.
	 * The SW trip (kshutdown cooler) is the protection.
	 * mtk_tc.forge_therm_hwprot=1 enables the HW comparator. */
	if (!forge_therm_hwprot) {
		pr_info_once("[FORGE_THERM] RGU thermal-direct arming skipped (bring-up default); mtk_tc.forge_therm_hwprot=1 enables\n");
		return;
	}

#if THERMAL_PERFORMANCE_PROFILE
	do_gettimeofday(&end);

	/* Get milliseconds */
	pr_debug("resume time spent, sec : %lu , usec : %lu\n", (end.tv_sec - begin.tv_sec),
	       (end.tv_usec - begin.tv_usec));
#endif

	for (i = 0; i < ARRAY_SIZE(tscpu_g_bank); i++) {
		mt_ptp_lock(&flags);

		tscpu_switch_bank(i);
		/* Move thermal HW protection ahead... */
		set_tc_trigger_hw_protect(temperature, temperature2, i);

		mt_ptp_unlock(&flags);
	}

	/*Thermal need to config to direct reset mode
	*  this API provide by Weiqi Fu(RGU SW owner).
	*/
	/* stock called through wd_api here unguarded; with a failed
	 * get_wd_api above that is a NULL/uninitialised pointer call */
	if (wd_api_ret >= 0)
		wd_api->wd_thermal_direct_mode_config(WD_REQ_EN, WD_REQ_RST_MODE);	/* reset mode */
	else
		pr_err("[FORGE_THERM] WD_REQ_EN skipped: no wd_api (%d) — RGU thermal-direct reset NOT armed\n",
		       wd_api_ret);

#if THERMAL_INFORM_OTP
	tscpu_config_tc_sw_protect(OTP_HIGH_OFFSET_TEMP, OTP_LOW_OFFSET_TEMP);
#endif
}

void tscpu_reset_thermal(void)
{
	/*chip dependent, Have to confirm with DE*/

	int temp = 0;
	/* reset thremal ctrl */
	temp = readl(INFRA_GLOBALCON_RST_0_SET);
	temp |= 0x00000001;	/* 1: Enables thermal control software reset */
	mt_reg_sync_writel(temp, INFRA_GLOBALCON_RST_0_SET);

	/* un reset */
	temp = readl(INFRA_GLOBALCON_RST_0_CLR);
	temp |= 0x00000001;	/* 1: Enable reset Disables thermal control software reset */
	mt_reg_sync_writel(temp, INFRA_GLOBALCON_RST_0_CLR);
}

int tscpu_read_temperature_info(struct seq_file *m, void *v)
{
	seq_printf(m, "current temp:%d\n", tscpu_read_curr_temp);
	seq_printf(m, "[cal] g_adc_ge_t      = %d\n", g_adc_ge_t);
	seq_printf(m, "[cal] g_adc_oe_t      = %d\n", g_adc_oe_t);
	seq_printf(m, "[cal] g_degc_cali     = %d\n", g_degc_cali);
	seq_printf(m, "[cal] g_adc_cali_en_t = %d\n", g_adc_cali_en_t);
	seq_printf(m, "[cal] g_o_slope       = %d\n", g_o_slope);
	seq_printf(m, "[cal] g_o_slope_sign  = %d\n", g_o_slope_sign);
	seq_printf(m, "[cal] g_id            = %d\n", g_id);

	seq_printf(m, "[cal] g_o_vtsmcu1     = %d\n", g_o_vtsmcu1);
	seq_printf(m, "[cal] g_o_vtsmcu2     = %d\n", g_o_vtsmcu2);
	seq_printf(m, "[cal] g_o_vtsmcu3     = %d\n", g_o_vtsmcu3);
	seq_printf(m, "[cal] g_o_vtsmcu4     = %d\n", g_o_vtsmcu4);
	seq_printf(m, "[cal] g_o_vtsmcu5     = %d\n", g_o_vtsmcu5);
	seq_printf(m, "[cal] g_o_vtsabb     = %d\n", g_o_vtsabb);

	seq_printf(m, "calefuse1:0x%x\n", calefuse1);
	seq_printf(m, "calefuse2:0x%x\n", calefuse2);
	seq_printf(m, "calefuse3:0x%x\n", calefuse3);
	return 0;
}

int tscpu_get_curr_temp(void)
{
	tscpu_update_tempinfo();

#if PRECISE_HYBRID_POWER_BUDGET
	/*	update CPU/GPU temp data whenever TZ times out...
	*	If the update timing is aligned to TZ polling,
	*	this segment should be moved to TZ code instead of thermal controller driver
	*/
	tscpu_prev_cpu_temp = tscpu_curr_cpu_temp;
	tscpu_prev_gpu_temp = tscpu_curr_gpu_temp;

	/* It is platform dependent which TS is better to present CPU/GPU temperature */
	/* As in the 3.18 MT6755 code: CPU = ts4 (BANK0[MCU4], mp0), GPU = ts3
	 * (BANK1[MCU3], the dominant GPU sensor); mp1 (BANK2[MCU4]) is folded
	 * in via MAX for the CPU side. */
	tscpu_curr_cpu_temp = MAX(get_immediate_ts4_wrap(), get_immediate_cpuL_wrap());
	tscpu_curr_gpu_temp = get_immediate_ts3_wrap();
#endif
	/* though tscpu_max_temperature is common, put it in mtk_ts_cpu.c is weird. */
	tscpu_curr_max_ts_temp = tscpu_max_temperature();

	return tscpu_curr_max_ts_temp;
}

/**
 * this only returns latest stored max ts temp but not updated from TC.
 */
int tscpu_get_curr_max_ts_temp(void)
{
	return tscpu_curr_max_ts_temp;
}

#ifdef CONFIG_OF
int get_io_reg_base(void)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-therm_ctrl");
	WARN_ON_ONCE(node == 0);
	if (node) {
		/* Setup IO addresses */
		thermal_base = of_iomap(node, 0);
		/*tscpu_printk("[THERM_CTRL] thermal_base=0x%p\n",thermal_base); */
	}

	/*get thermal irq num */
	thermal_irq_number = irq_of_parse_and_map(node, 0);
	/*tscpu_printk("[THERM_CTRL] thermal_irq_number=%d\n", thermal_irq_number);*/
	if (!thermal_irq_number) {
		tscpu_printk("[THERM_CTRL] get irqnr failed=%d\n", thermal_irq_number);
		return 0;
	}

	/* reg index 1 is the size (0x1000) with this dtsi's #address-cells =
	 * 1; word 0 is the address (as the 3.18 MT6755 code reads it). Using
	 * the size as base sends the TC's AHB engine to dead targets
	 * (TEMPADCENADDR/TEMPADCMUXADDR/TEMPADCVALIDADDR at 0x1008/0x100C/
	 * 0x1040), so no conversion completes. */
	of_property_read_u32(node, "reg", &thermal_phy_base);
	/*tscpu_printk("[THERM_CTRL] thermal_base thermal_phy_base=0x%x\n",thermal_phy_base); */

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-auxadc");
	WARN_ON_ONCE(node == 0);
	if (node) {
		/* Setup IO addresses */
		auxadc_ts_base = of_iomap(node, 0);
		/*tscpu_printk("[THERM_CTRL] auxadc_ts_base=0x%p\n",auxadc_ts_base); */
	}
	of_property_read_u32(node, "reg", &auxadc_ts_phy_base);
	/*tscpu_printk("[THERM_CTRL] auxadc_ts_phy_base=0x%x\n",auxadc_ts_phy_base); */

	node = of_find_compatible_node(NULL, NULL, "mediatek,infracfg_ao");
	WARN_ON_ONCE(node == 0);
	if (node) {
		/* Setup IO addresses */
		infracfg_ao_base = of_iomap(node, 0);
		/*tscpu_printk("[THERM_CTRL] infracfg_ao_base=0x%p\n",infracfg_ao_base); */
	}


	node = of_find_compatible_node(NULL, NULL, "mediatek,apmixed");
	WARN_ON_ONCE(node == 0);
	if (node) {
		/* Setup IO addresses */
		th_apmixed_base = of_iomap(node, 0);
		/*tscpu_printk("[THERM_CTRL] apmixed_base=0x%p\n", th_apmixed_base); */
	}
	of_property_read_u32(node, "reg", &apmixed_phy_base);
	/*tscpu_printk("[THERM_CTRL] apmixed_phy_base=0x%x\n",apmixed_phy_base); */

	/* expected: therm=0x1100b000 auxadc=0x11001000 apmixed=0x1000c000 */
	pr_info("[FORGE_THERM] TC engine phys targets: therm=0x%x auxadc=0x%x apmixed=0x%x\n",
		thermal_phy_base, auxadc_ts_phy_base, apmixed_phy_base);

#if THERMAL_GET_AHB_BUS_CLOCK
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-infrasys"); /**/
	if (!node) {
		pr_err("[CLK_INFRACFG_AO] find node failed\n");
		return 0;
	}

	therm_clk_infracfg_ao_base = of_iomap(node, 0);
	if (!therm_clk_infracfg_ao_base) {
		pr_err("[CLK_INFRACFG_AO] base failed\n");
		return 0;
	}
#endif
	return 1;
}
#endif

#if THERMAL_GET_AHB_BUS_CLOCK
void thermal_get_AHB_clk_info(void)
{
	int cg = 0, dcm = 0, cg_freq = 0;
	int clockSource = 136, ahbClk = 0; /*Need to confirm with DE*/

	cg = readl(THERMAL_CG);
	dcm = readl(THERMAL_DCM);

	/*The following rule may change, you need to confirm with DE.
	*  These are for mt6757, and confirmed with DE Justin Gu */ /* ???
	*/
	cg_freq = dcm & _BITMASK_(9:5);

	if ((cg_freq & _BIT_(4)) == 1)
		ahbClk = clockSource / 2;
	else if ((cg_freq & _BIT_(3)) == 1)
		ahbClk = clockSource / 4;
	else if ((cg_freq & _BIT_(2)) == 1)
		ahbClk = clockSource / 8;
	else if ((cg_freq & _BIT_(1)) == 1)
		ahbClk = clockSource / 16;
	else
		ahbClk = clockSource / 32;

	/*tscpu_printk("cg %d dcm %d\n", ((cg & _BIT_(10)) >> 10), dcm);*/
	/*tscpu_printk("AHB bus clock= %d MHz\n", ahbClk);*/
}
#endif

void print_risky_temps(char *prefix, int offset, int printLevel)
{
	sprintf((prefix + offset), "L %d LL %d",
		get_immediate_cpuL_wrap(), get_immediate_cpuLL_wrap());

	switch (printLevel) {
	case 0:
		tscpu_dprintk("%s\n", prefix);
		break;
	case 1:
		tscpu_warn("%s\n", prefix);
		break;
	default:
		tscpu_dprintk("%s\n", prefix);
	}
}

