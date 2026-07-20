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

/*
 * @file    mt_cpufreq.c
 * @brief   Driver for CPU DVFS
 *
 */

#define __MT_CPUFREQ_C__
#define DEBUG 1
/* system includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/topology.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <mt-plat/sync_write.h>
#include <mt-plat/mt_io.h>
#include <mt-plat/aee.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#endif

/* project includes */
#include "mach/mt_thermal.h"
#include "mt_static_power.h"
#include "mach/mt_pmic_wrap.h"
#include "mach/mt_freqhopping.h"
#include "mach/mt_ppm_api.h"
/* #include "mach/mt_ptp.h"
#include "mach/upmu_common.h"
#include "mach/upmu_sw.h"
#include "mach/upmu_hw.h"
#include "mach/mt_hotplug_strategy.h"
#include "mach/mt_pbm.h"
#include "mt-plat/mt_devinfo.h" */

/* local includes */
#include "mt_cpufreq.h"

#include "../../../pmic/include/pmic_api.h"
#include "../../../power/mt6755/mt6311.h"

#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include "mt-plat/mt_devinfo.h"
#include "mt_cpufreq_hybrid.h"
#include <linux/m3note_board.h>
#include <linux/i2c.h>
/* The shared header defines CONFIG_HYBRID_CPU_DVFS for MT6755, which routes
 * the ops through the SPM DVFSP co-processor firmware (cpuhvfs_*, not
 * built). Undefine it for this file to keep the AP-driven PLL/mux path. */
#undef CONFIG_HYBRID_CPU_DVFS

/* Frequency-only build: the frequency path never moves VPROC, and the OPP
 * ladder is clamped to rows that are legal at the boot voltage:
 *   big <= 1196 MHz, LL <= 689 MHz at VPROC = 1.000 V, the stock FY
 *   voltage of exactly these rows.
 * VSRAM is left as set by the preloader; its assumed value only feeds
 * bypassed sanity paths. */
#define FORGE_FREQ_ONLY		1
#define FORGE_FIXED_VPROC	100000	/* mv*100, silicon readback */
#define FORGE_FIXED_VSRAM	110000	/* VPROC + NORMAL_DIFF_VRSAM_VPROC */

/*
 * Voltage DVFS (stock FY table) on top of the frequency-only limits. When
 * cpufreq arms and the rail writer forge_vrail.c takes the rails,
 * forge_full_dvfs is set and every FORGE_FREQ_ONLY limit below moves from
 * the fixed 1.000 V to MAX_VPROC_VOLT; a refused take leaves the
 * frequency-only build as it was (forge_vrail.allow = 0 by default). A
 * failed or refused rail write latches forge_dvfs_fault: no voltage is
 * raised again and the ceilings drop to the rows legal at the voltage read
 * back. Not included: overclock rows, touch boost and perf mode.
 */
static bool forge_full_dvfs;
static bool forge_dvfs_fault;
static unsigned int forge_vproc_cur = FORGE_FIXED_VPROC, forge_vsram_cur = FORGE_FIXED_VSRAM;
module_param(forge_full_dvfs, bool, 0444);
module_param(forge_dvfs_fault, bool, 0444);
module_param(forge_vproc_cur, uint, 0444);
module_param(forge_vsram_cur, uint, 0444);
/* VPROC floor, mv*100: 1.000 V (the PL rail) by default, rows below
 * 1196/689 then run above their stock voltage, never below; 0 = stock */
static unsigned int forge_vproc_floor = 100000;
module_param(forge_vproc_floor, uint, 0644);

static const char forge_dvfs_cookie[] = "mt_cpufreq";	/* the rails' user */
extern int forge_vrail_take(const void *cookie, unsigned int *vproc_uv10, unsigned int *vsram_uv10);
extern int forge_vrail_release(const void *cookie);
extern int forge_vrail_set(const void *cookie, unsigned int vproc_uv10, unsigned int *vproc_rb,
			   unsigned int *vsram_rb);
extern int forge_vrail_hold(const void *cookie, bool hold);
extern unsigned int forge_vrail_pl_vproc(void);
extern bool forge_vrail_allowed(void);

static unsigned int forge_vmax(void)
{
	return READ_ONCE(forge_full_dvfs) ? 115000 /* MAX_VPROC_VOLT */ : FORGE_FIXED_VPROC;
}

/* the ceilings (defined with the limits notifier below) */
static unsigned int forge_cpu_max_khz[NR_MT_CPU_DVFS];
/* set_cur_freq refused the row (it cannot return an error); cpufreq lock */
static bool forge_freq_refused;

/*=============================================================*/
/* Macro definition                                            */
/*=============================================================*/
#if defined(CONFIG_OF)
static unsigned long infracfg_ao_base;
static unsigned long topckgen_base;
static unsigned long apmixed_base;

#define INFRACFG_AO_NODE "mediatek,mt6755-infrasys"
#define TOPCKGEN_NODE "mediatek,topckgen"
#define APMIXED_NODE "mediatek,apmixed"

#undef INFRACFG_AO_BASE
#undef TOPCKGEN_BASE
#undef APMIXED_BASE

#define INFRACFG_AO_BASE        (infracfg_ao_base)	/* 0xF0001000 */
#define TOPCKGEN_BASE           (topckgen_base)		/* 0xF0000000 */
#define APMIXED_BASE              (apmixed_base)	/* 0xF000C000 */

#else				/* #if defined (CONFIG_OF) */
#undef INFRACFG_AO_BASE
#undef TOPCKGEN_BASE
#undef APMIXED_BASE

#define INFRACFG_AO_BASE        0xF0001000
#define TOPCKGEN_BASE           0xF0000000
#define APMIXED_BASE            0xF000C000
#endif				/* #if defined (CONFIG_OF) */

#define ARMCA15PLL_CON0         (APMIXED_BASE + 0x200)
#define ARMCA15PLL_CON1         (APMIXED_BASE + 0x204)
#define ARMCA15PLL_CON2         (APMIXED_BASE + 0x208)
#define ARMCA7PLL_CON0          (APMIXED_BASE + 0x210)
#define ARMCA7PLL_CON1          (APMIXED_BASE + 0x214)
#define ARMCA7PLL_CON2          (APMIXED_BASE + 0x218)

/* INFRASYS Register */
#define TOP_CKMUXSEL            (INFRACFG_AO_BASE + 0x00)
#define TOP_CKDIV1              (INFRACFG_AO_BASE + 0x08)
#define INFRA_TOPCKGEN_CKDIV1_BIG (INFRACFG_AO_BASE + 0x0024)
#define INFRA_TOPCKGEN_CKDIV1_SML (INFRACFG_AO_BASE + 0x0028)
#define INFRA_TOPCKGEN_CKDIV1_BUS (INFRACFG_AO_BASE + 0x002C)

/* TOPCKGEN Register */
#define CLK_MISC_CFG_0          (TOPCKGEN_BASE + 0x104)

/*
 * CONFIG
 */
/* DCM sync is provided by mt_dcm.c, which is not built; the sync_dcm_set_*
 * calls are an optimisation, not required for the frequency switch. */
/* #define DCM_ENABLE 1 */
#define CONFIG_CPU_DVFS_SHOWLOG 1
/* #define CONFIG_CPU_DVFS_BRINGUP 1 */
#ifdef CONFIG_MTK_RAM_CONSOLE
#define CONFIG_CPU_DVFS_AEE_RR_REC 1
#endif

#ifdef CONFIG_HYBRID_CPU_DVFS
/**
 * 1: get voltage from DVFSP for performance
 * 2: get voltage from PMIC through I2C
 */
static u32 enable_cpuhvfs = 1;
#endif

#include <linux/time.h>
#include <linux/m3note_board.h>
ktime_t now[NR_SET_V_F];
ktime_t delta[NR_SET_V_F];
ktime_t max[NR_SET_V_F];

#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define MIN(a, b) ((a) >= (b) ? (b) : (a))

/* used @ set_cur_volt_extBuck() */
/* #define MIN_DIFF_VSRAM_PROC        1000  */
#define NORMAL_DIFF_VRSAM_VPROC    10000
#define MAX_DIFF_VSRAM_VPROC       20000
#define MIN_VSRAM_VOLT             93000
#define MAX_VSRAM_VOLT             115000
#define MAX_VPROC_VOLT             115000

/* PMIC/PLL settle time (us) */
#define PMIC_CMD_DELAY_TIME     5
#define MIN_PMIC_SETTLE_TIME    25
#define PMIC_VOLT_UP_SETTLE_TIME(old_volt, new_volt) \
	(((((new_volt) - (old_volt)) + 1250 - 1) / 1250) + PMIC_CMD_DELAY_TIME)
#define PMIC_VOLT_DOWN_SETTLE_TIME(old_volt, new_volt) \
	(((((old_volt) - (new_volt)) * 2)  / 625) + PMIC_CMD_DELAY_TIME)
#define PLL_SETTLE_TIME         (20)

/* for DVFS OPP table LL/FY */
#define CPU_DVFS_FREQ0_LL_6353   (1001000)	/* KHz */
#define CPU_DVFS_FREQ1_LL_6353    (910000)	/* KHz */
#define CPU_DVFS_FREQ2_LL_6353    (819000)	/* KHz */
#define CPU_DVFS_FREQ3_LL_6353    (689000)	/* KHz */
#define CPU_DVFS_FREQ4_LL_6353    (598000)	/* KHz */
#define CPU_DVFS_FREQ5_LL_6353    (494000)	/* KHz */
#define CPU_DVFS_FREQ6_LL_6353    (338000)	/* KHz */
#define CPU_DVFS_FREQ7_LL_6353    (156000)	/* KHz */

/* for DVFS OPP table L/FY */
#define CPU_DVFS_FREQ0_L_6353    (1508000)	/* KHz */
#define CPU_DVFS_FREQ1_L_6353    (1430000)	/* KHz */
#define CPU_DVFS_FREQ2_L_6353    (1352000)	/* KHz */
#define CPU_DVFS_FREQ3_L_6353    (1196000)	/* KHz */
#define CPU_DVFS_FREQ4_L_6353    (1027000)	/* KHz */
#define CPU_DVFS_FREQ5_L_6353    (871000)	/* KHz */
#define CPU_DVFS_FREQ6_L_6353    (663000)	/* KHz */
#define CPU_DVFS_FREQ7_L_6353    (286000)	/* KHz */

/* for DVFS OPP table LL/FY */
#define CPU_DVFS_FREQ0_LL   (1001000)	/* KHz */
#define CPU_DVFS_FREQ1_LL    (910000)	/* KHz */
#define CPU_DVFS_FREQ2_LL    (819000)	/* KHz */
#define CPU_DVFS_FREQ3_LL    (689000)	/* KHz */
#define CPU_DVFS_FREQ4_LL    (598000)	/* KHz */
#define CPU_DVFS_FREQ5_LL    (494000)	/* KHz */
#define CPU_DVFS_FREQ6_LL    (338000)	/* KHz */
#define CPU_DVFS_FREQ7_LL    (156000)	/* KHz */

/* for DVFS OPP table L/FY */
#define CPU_DVFS_FREQ0_L    (1807000)	/* KHz */
#define CPU_DVFS_FREQ1_L    (1651000)	/* KHz */
#define CPU_DVFS_FREQ2_L    (1495000)	/* KHz */
#define CPU_DVFS_FREQ3_L    (1196000)	/* KHz */
#define CPU_DVFS_FREQ4_L    (1027000)	/* KHz */
#define CPU_DVFS_FREQ5_L    (871000)	/* KHz */
#define CPU_DVFS_FREQ6_L    (663000)	/* KHz */
#define CPU_DVFS_FREQ7_L    (286000)	/* KHz */

/* for DVFS OPP table LL/SB */
#define CPU_DVFS_FREQ0_LL_SB    (1144000)	/* KHz */
#define CPU_DVFS_FREQ1_LL_SB    (1014000)	/* KHz */
#define CPU_DVFS_FREQ2_LL_SB    (871000)	/* KHz */
#define CPU_DVFS_FREQ3_LL_SB    (689000)	/* KHz */
#define CPU_DVFS_FREQ4_LL_SB    (598000)	/* KHz */
#define CPU_DVFS_FREQ5_LL_SB    (494000)	/* KHz */
#define CPU_DVFS_FREQ6_LL_SB    (338000)	/* KHz */
#define CPU_DVFS_FREQ7_LL_SB    (156000)	/* KHz */

/* for DVFS OPP table L/SB */
#define CPU_DVFS_FREQ0_L_SB    (1950000)	/* KHz */
#define CPU_DVFS_FREQ1_L_SB    (1755000)	/* KHz */
#define CPU_DVFS_FREQ2_L_SB    (1573000)	/* KHz */
#define CPU_DVFS_FREQ3_L_SB    (1196000)	/* KHz */
#define CPU_DVFS_FREQ4_L_SB    (1027000)	/* KHz */
#define CPU_DVFS_FREQ5_L_SB    (871000)		/* KHz */
#define CPU_DVFS_FREQ6_L_SB    (663000)		/* KHz */
#define CPU_DVFS_FREQ7_L_SB    (286000)		/* KHz */

/* for DVFS OPP table LL/P15 */
#define CPU_DVFS_FREQ0_LL_P15    (1248000)	/* KHz */
#define CPU_DVFS_FREQ1_LL_P15    (1079000)	/* KHz */
#define CPU_DVFS_FREQ2_LL_P15    (910000)	/* KHz */
#define CPU_DVFS_FREQ3_LL_P15    (689000)	/* KHz */
#define CPU_DVFS_FREQ4_LL_P15    (598000)	/* KHz */
#define CPU_DVFS_FREQ5_LL_P15    (494000)	/* KHz */
#define CPU_DVFS_FREQ6_LL_P15    (338000)	/* KHz */
#define CPU_DVFS_FREQ7_LL_P15    (156000)	/* KHz */

/* for DVFS OPP table L/P15 */
#define CPU_DVFS_FREQ0_L_P15    (2145000)	/* KHz */
#define CPU_DVFS_FREQ1_L_P15    (1911000)	/* KHz */
#define CPU_DVFS_FREQ2_L_P15    (1664000)	/* KHz */
#define CPU_DVFS_FREQ3_L_P15    (1196000)	/* KHz */
#define CPU_DVFS_FREQ4_L_P15    (1027000)	/* KHz */
#define CPU_DVFS_FREQ5_L_P15    (871000)		/* KHz */
#define CPU_DVFS_FREQ6_L_P15    (663000)		/* KHz */
#define CPU_DVFS_FREQ7_L_P15    (286000)		/* KHz */

#define CPUFREQ_BOUNDARY_FOR_FHCTL   (CPU_DVFS_FREQ4_L)
#define CPUFREQ_LAST_FREQ_LEVEL    (CPU_DVFS_FREQ7_LL)

/* Debugging */
#undef TAG
#define TAG     "[Power/cpufreq] "

#define cpufreq_err(fmt, args...)       \
	pr_err(TAG"[ERROR]"fmt, ##args)
#define cpufreq_warn(fmt, args...)      \
	pr_warn(TAG"[WARNING]"fmt, ##args)
#define cpufreq_info(fmt, args...)      \
	pr_warn(TAG""fmt, ##args)
#define cpufreq_dbg(fmt, args...)       \
	pr_debug(TAG""fmt, ##args)
#define cpufreq_ver(fmt, args...)       \
	do {                                \
		if (func_lv_mask)           \
			cpufreq_info(TAG""fmt, ##args);    \
	} while (0)

#define FUNC_LV_MODULE         BIT(0)  /* module, platform driver interface */
#define FUNC_LV_CPUFREQ        BIT(1)  /* cpufreq driver interface          */
#define FUNC_LV_API                BIT(2)  /* mt_cpufreq driver global function */
#define FUNC_LV_LOCAL            BIT(3)  /* mt_cpufreq driver local function  */
#define FUNC_LV_HELP              BIT(4)  /* mt_cpufreq driver help function   */
#define FUNC_ENTER(lv) \
	do { if ((lv) & func_lv_mask) cpufreq_dbg(">> %s()\n", __func__); } while (0)
#define FUNC_EXIT(lv) \
	do { if ((lv) & func_lv_mask) cpufreq_dbg("<< %s():%d\n", __func__, __LINE__); } while (0)

#define FUNC_LV_MODULE         BIT(0)	/* module, platform driver interface */
#define FUNC_LV_CPUFREQ        BIT(1)	/* cpufreq driver interface          */
#define FUNC_LV_API                BIT(2)	/* mt_cpufreq driver global function */
#define FUNC_LV_LOCAL            BIT(3)	/* mt_cpufreq driver local function  */
#define FUNC_LV_HELP              BIT(4)	/* mt_cpufreq driver help function   */

/*
* static unsigned int func_lv_mask =
* (FUNC_LV_MODULE | FUNC_LV_CPUFREQ | FUNC_LV_API | FUNC_LV_LOCAL | FUNC_LV_HELP);
*/
static unsigned int func_lv_mask;
static unsigned int do_dvfs_stress_test;
static unsigned int dvfs_power_mode;

enum ppb_power_mode {
	Default = 0,
	Low_power_Mode = 1,
	Just_Make_Mode = 2,
	Performance_Mode = 3,
};

#ifdef CONFIG_CPU_DVFS_SHOWLOG
#define FUNC_ENTER(lv) \
	do { if ((lv) & func_lv_mask) cpufreq_dbg(">> %s()\n", __func__); } while (0)
#define FUNC_EXIT(lv) \
	do { if ((lv) & func_lv_mask) cpufreq_dbg("<< %s():%d\n", __func__, __LINE__); } while (0)
#else
#define FUNC_ENTER(lv)
#define FUNC_EXIT(lv)
#endif				/* CONFIG_CPU_DVFS_SHOWLOG */

/*
 * BIT Operation
 */
#define _BIT_(_bit_)                    (unsigned)(1 << (_bit_))
#define _BITS_(_bits_, _val_) \
	((((unsigned) -1 >> (31 - ((1) ? _bits_))) & ~((1U << ((0) ? _bits_)) - 1)) & ((_val_)<<((0) ? _bits_)))
#define _BITMASK_(_bits_)               (((unsigned) -1 >> (31 - ((1) ? _bits_))) & ~((1U << ((0) ? _bits_)) - 1))
#define _GET_BITS_VAL_(_bits_, _val_)   (((_val_) & (_BITMASK_(_bits_))) >> ((0) ? _bits_))

/*
 * REG ACCESS
 */
#define cpufreq_read(addr)         __raw_readl(IOMEM(addr))
#define cpufreq_write(addr, val)   mt_reg_sync_writel((val), ((void *)addr))

#define cpufreq_write_mask(addr, mask, val) \
cpufreq_write(addr, (cpufreq_read(addr) & ~(_BITMASK_(mask))) | _BITS_(mask, val))

/*
 * LOCK
 */
#if 0
static DEFINE_SPINLOCK(cpufreq_lock);
#define cpufreq_lock(flags) spin_lock_irqsave(&cpufreq_lock, flags)
#define cpufreq_unlock(flags) spin_unlock_irqrestore(&cpufreq_lock, flags)
#else				/* mutex */
static DEFINE_MUTEX(cpufreq_mutex);
bool is_in_cpufreq = 0;
#define cpufreq_lock(flags) \
	do { \
		flags = (unsigned long)&flags; \
		mutex_lock(&cpufreq_mutex); \
		is_in_cpufreq = 1;\
	} while (0)

#define cpufreq_unlock(flags) \
	do { \
		flags = (unsigned long)&flags; \
		is_in_cpufreq = 0;\
		mutex_unlock(&cpufreq_mutex); \
	} while (0)
#endif

/*
 * EFUSE
 */
#define CPUFREQ_EFUSE_INDEX     (3)
#define FUNC_CODE_EFUSE_INDEX	(21)
#define FUNC_CODE_EFUSE_INDEX_ENG	(19)

/*
 * The stock LK passes the raw v1 devinfo atag (data[0] = tag 0x41000804),
 * so on L681 get_devinfo_with_index(N) returns stock index N-1
 * (devinfo/devinfosys.c). The global numbering stays as is (other consumers
 * depend on it); only the FY/SB pick reads the stock word here. Other
 * boards or a v2 layout: unchanged.
 */
#define FORGE_ATAG_DEVINFO_TAG	0x41000804
static u32 forge_cpufreq_efuse(unsigned int stock_idx)
{
	if (m3note_board_known_is(M3NOTE_BOARD_L681) &&
	    get_devinfo_with_index(0) == FORGE_ATAG_DEVINFO_TAG)
		return get_devinfo_with_index(stock_idx + 1);
	return get_devinfo_with_index(stock_idx);
}

#define CPU_LEVEL_0             (0x0)
#define CPU_LEVEL_1             (0x1)
#define CPU_LEVEL_2             (0x2)
#define CPU_LEVEL_3             (0x3)
#define CPU_LV_TO_OPP_IDX(lv)   ((lv))	/* cpu_level to opp_idx */

#define cpu_dvfs_is_extbuck_valid()     (is_ext_buck_exist() && is_ext_buck_sw_ready())
unsigned int is_extbuck_valid;

#ifdef __KERNEL__
static unsigned int _mt_cpufreq_get_cpu_level_efuse(void)
{
	unsigned int lv = 0;
	unsigned int func_code_0 = _GET_BITS_VAL_(7:0, forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX));
	unsigned int func_code_1 = _GET_BITS_VAL_(31:28, forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX));
	unsigned int binLevel_eng = _GET_BITS_VAL_(15:0, forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX_ENG));

	cpufreq_ver("from efuse: function code 0 = 0x%x, function code 1 = 0x%x, binLevel_eng = 0x%x\n",
		func_code_0,
		func_code_1,
		binLevel_eng);

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	/* L681 is MT6351 + MT6311, not MT6353, so the MT6353 _6353 table (big
	 * 1508) does not apply. Skip this PMIC-keyed pick and fall through to
	 * the DT/efuse segment logic below, which is the stock one shifted by
	 * one level (stock LEVEL_0 FY = LEVEL_1 here, stock LEVEL_1 SB =
	 * LEVEL_2). is_mt6311_exist() and is_extbuck_valid are not touched:
	 * the table pick must not arm any MT6311 path. */
	if (!m3note_board_known_is(M3NOTE_BOARD_L681)) {
		if (cpu_dvfs_is_extbuck_valid()) {
			/* If FY */
			is_extbuck_valid = 1;
			/* is real 55s */
			if (func_code_0 == 0x82 || func_code_0 == 0x86)
				return CPU_LEVEL_2;
		}
		return CPU_LEVEL_0;
	}
#endif

	/* get CPU clock-frequency from DT */
#ifdef CONFIG_OF
	{
		struct device_node *node = of_find_node_by_type(NULL, "cpu");
		unsigned int cpu_speed = 0;

		if (!of_property_read_u32(node, "clock-frequency", &cpu_speed))
			cpu_speed = cpu_speed / 1000 / 1000;	/* MHz */
		else {
			cpufreq_err
			    ("@%s: missing clock-frequency property, use default CPU level\n",
			     __func__);
			return CPU_LEVEL_1;
		}

		cpufreq_ver("CPU clock-frequency from DT = %d MHz\n", cpu_speed);

		if (cpu_speed == 1001) /* FY */
			return CPU_LEVEL_1;
		else {
			if ((1 == func_code_0) || (3 == func_code_0))
				return CPU_LEVEL_1;
			else if ((2 == func_code_0) || (4 == func_code_0))
				return CPU_LEVEL_2;
#ifdef CONFIG_ARCH_MT6755_TURBO
			else if (0x20 == (func_code_0 & 0xF0))
				return CPU_LEVEL_3;
#endif
			else {
				if ((2 == ((binLevel_eng >> 4) & 0x07)) || (2 == ((binLevel_eng >> 10) & 0x07)))
					return CPU_LEVEL_1;
				return CPU_LEVEL_2;
			}
		}
	}

#endif

	return lv;
}

/*
 * Pin the FY table (LEVEL_1 here). The stock L681 kernel runs big 1807 MHz
 * at 1.0375 V, a row only the FY table has; M681 stock is FY as well. The
 * MT6353 build path (CONFIG_MTK_PMIC_CHIP_MT6353, a build setting; the PMIC
 * is MT6351) would send other boards to LEVEL_0, the _6353 table (big
 * 1508) that no M3 Note stock kernel has. The rows of the frequency-only
 * cut are the same in both tables.
 * 1 (default) = FY whatever the efuse says; 0 (cmdline
 * mt_cpufreq.forge_cpu_level_pin=0) = the efuse pick above. Both are
 * printed by forge_opp_dump_cpu_tables(). Other boards: no effect.
 */
static int forge_cpu_level_pin = 1;
module_param(forge_cpu_level_pin, int, 0444);
MODULE_PARM_DESC(forge_cpu_level_pin, "l681/m681: 1 (default) = CPU OPP table pinned to FY (LEVEL_1)");

static unsigned int _mt_cpufreq_get_cpu_level(void)
{
	unsigned int lv = _mt_cpufreq_get_cpu_level_efuse();

	if (forge_cpu_level_pin && (m3note_board_known_is(M3NOTE_BOARD_L681) ||
				    m3note_board_known_is(M3NOTE_BOARD_M681)))
		return CPU_LEVEL_1;
	return lv;
}
#else
static unsigned int _mt_cpufreq_get_cpu_level_efuse(void)
{
	return CPU_LEVEL_0;
}

static unsigned int _mt_cpufreq_get_cpu_level(void)
{
	return CPU_LEVEL_0;
}
#endif


/*
 * AEE (SRAM debug)
 */
#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
enum cpu_dvfs_state {
	CPU_DVFS_LITTLE_IS_DOING_DVFS = 0,
	CPU_DVFS_LITTLE_IS_TURBO,
	CPU_DVFS_BIG_IS_DOING_DVFS = 0,
	CPU_DVFS_BIG_IS_TURBO,
};

static void _mt_cpufreq_aee_init(void)
{
	aee_rr_rec_cpu_dvfs_vproc_big(0xFF);
	aee_rr_rec_cpu_dvfs_vproc_little(0xFF);
	aee_rr_rec_cpu_dvfs_oppidx(0xFF);
	aee_rr_rec_cpu_dvfs_status(0xFC);
}
#endif

/*
 * PMIC_WRAP
 */
#define VOLT_TO_PMIC_VAL(volt)  (((volt) - 60000 + 625 - 1) / 625)	/* ((((volt) - 700 * 100 + 625 - 1) / 625) */
#define PMIC_VAL_TO_VOLT(pmic)  (((pmic) * 625) + 60000)	/* (((pmic) * 625) / 100 + 700) */

#define VOLT_TO_EXTBUCK_VAL(volt)   VOLT_TO_PMIC_VAL(volt)	/* (((((volt) - 300) + 9) / 10) & 0x7F) */
#define EXTBUCK_VAL_TO_VOLT(val)    PMIC_VAL_TO_VOLT(val)	/* (300 + ((val) & 0x7F) * 10) */

/* cpu voltage sampler */
static cpuVoltsampler_func g_pCpuVoltSampler;

void mt_cpufreq_setvolt_registerCB(cpuVoltsampler_func pCB)
{
	g_pCpuVoltSampler = pCB;
}
EXPORT_SYMBOL(mt_cpufreq_setvolt_registerCB);

/*
 * mt_cpufreq driver
 */
#define FP(khz, pos, clk, cci) {	\
	.target_f = khz,		\
	.vco_dds = khz*pos*clk,		\
	.pos_div = pos,			\
	.clk_div = clk,			\
	.cci_div = cci,			\
}
struct mt_cpu_freq_method {
	const unsigned int target_f;
	const unsigned int vco_dds;
	const unsigned int pos_div;
	const unsigned int clk_div;
	const unsigned int cci_div;
};
#define OP(khz, volt) {            \
	.cpufreq_khz = khz,             \
	.cpufreq_volt = volt,           \
	.cpufreq_volt_org = volt,       \
}

struct mt_cpu_freq_info {
	const unsigned int cpufreq_khz;
	unsigned int cpufreq_volt;
	const unsigned int cpufreq_volt_org;
};

struct opp_idx_tbl {
	struct mt_cpu_dvfs *p;
	struct mt_cpu_freq_method *slot;
};

enum opp_idx_type {
		CUR_OPP_IDX = 0,
		TARGET_OPP_IDX = 1,
		CUR_CCI_OPP_IDX = 2,
		TARGET_CCI_OPP_IDX = 3,
		BACKUP_CCI_OPP_IDX = 4,

		NR_OPP_IDX,
};
struct opp_idx_tbl opp_tbl[NR_OPP_IDX];

struct mt_cpu_dvfs {
	const char *name;
	unsigned int cpu_id;	/* for cpufreq */
	unsigned int cpu_level;
	struct mt_cpu_dvfs_ops *ops;
	unsigned int *armpll_addr;	/* for armpll clock address */
	enum top_ckmuxsel_cci armpll_clk_src;	/* for cci switch clock source */
	struct mt_cpu_freq_method *freq_tbl;	/* Frequency table */
	struct mt_cpu_freq_info *opp_tbl;	/* OPP table */
	int nr_opp_tbl;		/* size for OPP table */
	int idx_opp_tbl;	/* current OPP idx */
	int idx_opp_ppm_base;	/* ppm update base */
	int idx_opp_ppm_limit;	/* ppm update limit */
	int armpll_is_available;	/* For CCI clock switch flag */
	int idx_normal_max_opp;	/* idx for normal max OPP */

	struct cpufreq_frequency_table *freq_tbl_for_cpufreq;	/* freq table for cpufreq */

	/* enable/disable DVFS function */
	bool dvfs_disable_by_suspend;
	bool dvfs_disable_by_procfs;

	/* turbo mode */
	unsigned int turbo_mode;

};

struct mt_cpu_dvfs_ops {
	/* for freq change (PLL/MUX) */
	unsigned int (*get_cur_phy_freq)(struct mt_cpu_dvfs *p);	/* return (physical) freq (KHz) */
	void (*set_cur_freq)(struct mt_cpu_dvfs *p, unsigned int cur_khz, unsigned int target_khz);	/* set freq  */
	/* for volt change (PMICWRAP/extBuck) */
	unsigned int (*get_cur_volt)(struct mt_cpu_dvfs *p);	/* return volt (mV * 100) */
	int (*set_cur_volt)(struct mt_cpu_dvfs *p, unsigned int volt);	/* set volt (mv * 100) */
};

/* for freq change (PLL/MUX) */
static unsigned int get_cur_phy_freq_L(struct mt_cpu_dvfs *p);
static unsigned int get_cur_phy_freq_LL(struct mt_cpu_dvfs *p);
static void set_cur_freq(struct mt_cpu_dvfs *p, unsigned int cur_khz, unsigned int target_khz);
static enum top_ckmuxsel_cci _get_cci_clock_switch(void);
static void cci_switch_from_to(enum opp_idx_type from, enum opp_idx_type to);
/* for volt change (PMICWRAP/extBuck) */
static unsigned int get_cur_volt_extbuck(struct mt_cpu_dvfs *p);
static int set_cur_volt_extbuck(struct mt_cpu_dvfs *p, unsigned int volt);	/* volt: mv * 100 */
/* CPU callback */
static int _mt_cpufreq_cpu_CB(struct notifier_block *nfb, unsigned long action,
					void *hcpu);
static unsigned int max_cpu_num = 8;

static struct mt_cpu_dvfs_ops dvfs_ops_LL = {
	.get_cur_phy_freq = get_cur_phy_freq_LL,
	.set_cur_freq = set_cur_freq,
	.get_cur_volt = get_cur_volt_extbuck,
	.set_cur_volt = set_cur_volt_extbuck,
};

static struct mt_cpu_dvfs_ops dvfs_ops_L = {
	.get_cur_phy_freq = get_cur_phy_freq_L,
	.set_cur_freq = set_cur_freq,
	.get_cur_volt = get_cur_volt_extbuck,
	.set_cur_volt = set_cur_volt_extbuck,
};

static struct mt_cpu_dvfs cpu_dvfs[] = {
	[MT_CPU_DVFS_LITTLE] = {
				.name = __stringify(MT_CPU_DVFS_LITTLE),
				.cpu_id = MT_CPU_DVFS_LITTLE,
				.cpu_level = CPU_LEVEL_0,
				.idx_normal_max_opp = -1,
				.idx_opp_ppm_base = -1,
				.idx_opp_ppm_limit = -1,
				.ops = &dvfs_ops_LL,
				},

	[MT_CPU_DVFS_BIG] = {
				.name = __stringify(MT_CPU_DVFS_BIG),
				.cpu_id = MT_CPU_DVFS_BIG,
				.cpu_level = CPU_LEVEL_0,
				.idx_normal_max_opp = -1,
				.idx_opp_ppm_base = -1,
				.idx_opp_ppm_limit = -1,
				.ops = &dvfs_ops_L,
				},
};

#define for_each_cpu_dvfs(i, p)        for (i = 0, p = cpu_dvfs; i < NR_MT_CPU_DVFS; i++, p = &cpu_dvfs[i])
#define cpu_dvfs_is(p, id)                 (p == &cpu_dvfs[id])
#define cpu_dvfs_is_available(p)      (p->opp_tbl)
#define cpu_dvfs_get_name(p)         (p->name)

#define cpu_dvfs_get_cur_freq(p)                (p->opp_tbl[p->idx_opp_tbl].cpufreq_khz)
#define cpu_dvfs_get_freq_by_idx(p, idx)        (p->opp_tbl[idx].cpufreq_khz)

#define cpu_dvfs_get_max_freq(p)                (p->opp_tbl[0].cpufreq_khz)
#define cpu_dvfs_get_normal_max_freq(p)         (p->opp_tbl[p->idx_normal_max_opp].cpufreq_khz)
#define cpu_dvfs_get_min_freq(p)                (p->opp_tbl[p->nr_opp_tbl - 1].cpufreq_khz)

#define cpu_dvfs_get_cur_volt(p)                (p->opp_tbl[p->idx_opp_tbl].cpufreq_volt)
#define cpu_dvfs_get_volt_by_idx(p, idx)        (p->opp_tbl[idx].cpufreq_volt)

static struct mt_cpu_dvfs *id_to_cpu_dvfs(enum mt_cpu_dvfs_id id)
{
	return (id < NR_MT_CPU_DVFS) ? &cpu_dvfs[id] : NULL;
}

#ifdef CONFIG_HYBRID_CPU_DVFS
static inline unsigned int id_to_cluster(enum mt_cpu_dvfs_id id)
{
	if (id == MT_CPU_DVFS_LITTLE)
		return CPU_CLUSTER_LL;
	if (id == MT_CPU_DVFS_BIG)
		return CPU_CLUSTER_L;

	BUG();
	return NUM_CPU_CLUSTER;
}

static inline unsigned int cpu_dvfs_to_cluster(struct mt_cpu_dvfs *p)
{
	if (p == &cpu_dvfs[MT_CPU_DVFS_LITTLE])
		return CPU_CLUSTER_LL;
	if (p == &cpu_dvfs[MT_CPU_DVFS_BIG])
		return CPU_CLUSTER_L;

	BUG();
	return NUM_CPU_CLUSTER;
}

static inline struct mt_cpu_dvfs *cluster_to_cpu_dvfs(unsigned int cluster)
{
	if (cluster == CPU_CLUSTER_LL)
		return &cpu_dvfs[MT_CPU_DVFS_LITTLE];
	if (cluster == CPU_CLUSTER_L)
		return &cpu_dvfs[MT_CPU_DVFS_BIG];

	BUG();
	return NULL;
}
#endif

static void aee_record_cpu_volt(struct mt_cpu_dvfs *p, unsigned int volt)
{
#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		aee_rr_rec_cpu_dvfs_vproc_little(VOLT_TO_PMIC_VAL(volt));
	else
		aee_rr_rec_cpu_dvfs_vproc_big(VOLT_TO_PMIC_VAL(volt));
#endif
}

static void notify_cpu_volt_sampler(struct mt_cpu_dvfs *p, unsigned int volt)
{
	unsigned int mv = volt / 100;
	enum mt_cpu_dvfs_id id;

	if (!g_pCpuVoltSampler)
		return;

	id = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) ? MT_CPU_DVFS_LITTLE : MT_CPU_DVFS_BIG);
	g_pCpuVoltSampler(id, mv);
}


/* DVFS OPP table */
/* Notice: Each table MUST has 8 element to avoid ptpod error */

#define NR_MAX_OPP_TBL  8
#define NR_MAX_CPU      8

/* CPU LEVEL 0 of LL, FY segment */
static struct mt_cpu_freq_info opp_tbl_little_e0_0[] = {
	OP(CPU_DVFS_FREQ0_LL_6353, 115000),
	OP(CPU_DVFS_FREQ1_LL_6353, 111250),
	OP(CPU_DVFS_FREQ2_LL_6353, 107500),
	OP(CPU_DVFS_FREQ3_LL_6353, 100000),
	OP(CPU_DVFS_FREQ4_LL_6353, 96875),
	OP(CPU_DVFS_FREQ5_LL_6353, 93750),
	OP(CPU_DVFS_FREQ6_LL_6353, 90000),
	OP(CPU_DVFS_FREQ7_LL_6353, 80000),
};

/* CPU LEVEL 1 of LL, FY segment */
static struct mt_cpu_freq_info opp_tbl_little_e1_0[] = {
	OP(CPU_DVFS_FREQ0_LL, 115000),
	OP(CPU_DVFS_FREQ1_LL, 111250),
	OP(CPU_DVFS_FREQ2_LL, 107500),
	OP(CPU_DVFS_FREQ3_LL, 100000),
	OP(CPU_DVFS_FREQ4_LL, 96875),
	OP(CPU_DVFS_FREQ5_LL, 93750),
	OP(CPU_DVFS_FREQ6_LL, 90000),
	OP(CPU_DVFS_FREQ7_LL, 80000),
};

/* CPU LEVEL 2 of LL, SB segment */
static struct mt_cpu_freq_info opp_tbl_little_e2_0[] = {
	OP(CPU_DVFS_FREQ0_LL_SB, 115000),
	OP(CPU_DVFS_FREQ1_LL_SB, 111250),
	OP(CPU_DVFS_FREQ2_LL_SB, 107500),
	OP(CPU_DVFS_FREQ3_LL_SB, 100000),
	OP(CPU_DVFS_FREQ4_LL_SB, 96875),
	OP(CPU_DVFS_FREQ5_LL_SB, 93750),
	OP(CPU_DVFS_FREQ6_LL_SB, 90000),
	OP(CPU_DVFS_FREQ7_LL_SB, 80000),
};

/* CPU LEVEL 3 of LL, P15 segment */
static struct mt_cpu_freq_info opp_tbl_little_e3_0[] = {
	OP(CPU_DVFS_FREQ0_LL_P15, 115000),
	OP(CPU_DVFS_FREQ1_LL_P15, 111250),
	OP(CPU_DVFS_FREQ2_LL_P15, 107500),
	OP(CPU_DVFS_FREQ3_LL_P15, 100000),
	OP(CPU_DVFS_FREQ4_LL_P15, 96875),
	OP(CPU_DVFS_FREQ5_LL_P15, 93750),
	OP(CPU_DVFS_FREQ6_LL_P15, 90000),
	OP(CPU_DVFS_FREQ7_LL_P15, 80000),
};

/* CPU LEVEL 0 of L, FY segment */
static struct mt_cpu_freq_info opp_tbl_big_e0_0[] = {
	OP(CPU_DVFS_FREQ0_L_6353, 115000),
	OP(CPU_DVFS_FREQ1_L_6353, 111250),
	OP(CPU_DVFS_FREQ2_L_6353, 107500),
	OP(CPU_DVFS_FREQ3_L_6353, 100000),
	OP(CPU_DVFS_FREQ4_L_6353, 96875),
	OP(CPU_DVFS_FREQ5_L_6353, 93750),
	OP(CPU_DVFS_FREQ6_L_6353, 90000),
	OP(CPU_DVFS_FREQ7_L_6353, 80000),
};

/* CPU LEVEL 1 of L, FY segment */
static struct mt_cpu_freq_info opp_tbl_big_e1_0[] = {
	OP(CPU_DVFS_FREQ0_L, 115000),
	OP(CPU_DVFS_FREQ1_L, 111250),
	OP(CPU_DVFS_FREQ2_L, 107500),
	OP(CPU_DVFS_FREQ3_L, 100000),
	OP(CPU_DVFS_FREQ4_L, 96875),
	OP(CPU_DVFS_FREQ5_L, 93750),
	OP(CPU_DVFS_FREQ6_L, 90000),
	OP(CPU_DVFS_FREQ7_L, 80000),
};

/* CPU LEVEL 2 of L, SB segment */
static struct mt_cpu_freq_info opp_tbl_big_e2_0[] = {
	OP(CPU_DVFS_FREQ0_L_SB, 115000),
	OP(CPU_DVFS_FREQ1_L_SB, 111250),
	OP(CPU_DVFS_FREQ2_L_SB, 107500),
	OP(CPU_DVFS_FREQ3_L_SB, 100000),
	OP(CPU_DVFS_FREQ4_L_SB, 96875),
	OP(CPU_DVFS_FREQ5_L_SB, 93750),
	OP(CPU_DVFS_FREQ6_L_SB, 90000),
	OP(CPU_DVFS_FREQ7_L_SB, 80000),
};

/* CPU LEVEL 3 of L, SB segment */
static struct mt_cpu_freq_info opp_tbl_big_e3_0[] = {
	OP(CPU_DVFS_FREQ0_L_P15, 115000),
	OP(CPU_DVFS_FREQ1_L_P15, 111250),
	OP(CPU_DVFS_FREQ2_L_P15, 107500),
	OP(CPU_DVFS_FREQ3_L_P15, 100000),
	OP(CPU_DVFS_FREQ4_L_P15, 96875),
	OP(CPU_DVFS_FREQ5_L_P15, 93750),
	OP(CPU_DVFS_FREQ6_L_P15, 90000),
	OP(CPU_DVFS_FREQ7_L_P15, 80000),
};

static struct mt_cpu_freq_method opp_tbl_method_L_e0[] = {
	/* Target Frequency, POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_L_6353,	1,	1,	2),
	FP(CPU_DVFS_FREQ1_L_6353,	1,	1,	2),
	FP(CPU_DVFS_FREQ2_L_6353,	1,	1,	2),
	FP(CPU_DVFS_FREQ3_L_6353,	1,	1,	2),
	FP(CPU_DVFS_FREQ4_L_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_L_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ6_L_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ7_L_6353,	2,	2,	4),
};

static struct mt_cpu_freq_method opp_tbl_method_LL_e0[] = {
	/* Target Frequency, POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_LL_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ1_LL_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ2_LL_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ3_LL_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ4_LL_6353,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_LL_6353,	2,	2,	4),
	FP(CPU_DVFS_FREQ6_LL_6353,	2,	2,	4),
	FP(CPU_DVFS_FREQ7_LL_6353,	2,	4,	6),
};

static struct mt_cpu_freq_method opp_tbl_method_L_e1[] = {
	/* Target Frequency, POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_L,	1,	1,	2),
	FP(CPU_DVFS_FREQ1_L,	1,	1,	2),
	FP(CPU_DVFS_FREQ2_L,	1,	1,	2),
	FP(CPU_DVFS_FREQ3_L,	1,	1,	2),
	FP(CPU_DVFS_FREQ4_L,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_L,	2,	1,	2),
	FP(CPU_DVFS_FREQ6_L,	2,	1,	2),
	FP(CPU_DVFS_FREQ7_L,	2,	2,	4),
};

static struct mt_cpu_freq_method opp_tbl_method_LL_e1[] = {
	/* Target Frequency, POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_LL,	2,	1,	2),
	FP(CPU_DVFS_FREQ1_LL,	2,	1,	2),
	FP(CPU_DVFS_FREQ2_LL,	2,	1,	2),
	FP(CPU_DVFS_FREQ3_LL,	2,	1,	2),
	FP(CPU_DVFS_FREQ4_LL,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_LL,	2,	2,	4),
	FP(CPU_DVFS_FREQ6_LL,	2,	2,	4),
	FP(CPU_DVFS_FREQ7_LL,	2,	4,	6),
};

static struct mt_cpu_freq_method opp_tbl_method_L_e2[] = {
	/* Target Frequency,		POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_L_SB,		1,	1,	2),
	FP(CPU_DVFS_FREQ1_L_SB,		1,	1,	2),
	FP(CPU_DVFS_FREQ2_L_SB,		1,	1,	2),
	FP(CPU_DVFS_FREQ3_L_SB,		1,	1,	2),
	FP(CPU_DVFS_FREQ4_L_SB,		2,	1,	2),
	FP(CPU_DVFS_FREQ5_L_SB,		2,	1,	2),
	FP(CPU_DVFS_FREQ6_L_SB,		2,	1,	2),
	FP(CPU_DVFS_FREQ7_L_SB,		2,	2,	4),
};

static struct mt_cpu_freq_method opp_tbl_method_LL_e2[] = {
	/* Target Frequency,		POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_LL_SB,	2,	1,	2),
	FP(CPU_DVFS_FREQ1_LL_SB,	2,	1,	2),
	FP(CPU_DVFS_FREQ2_LL_SB,	2,	1,	2),
	FP(CPU_DVFS_FREQ3_LL_SB,	2,	1,	2),
	FP(CPU_DVFS_FREQ4_LL_SB,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_LL_SB,	2,	2,	4),
	FP(CPU_DVFS_FREQ6_LL_SB,	2,	2,	4),
	FP(CPU_DVFS_FREQ7_LL_SB,	2,	4,	6),
};

static struct mt_cpu_freq_method opp_tbl_method_L_e3[] = {
	/* Target Frequency,		POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_L_P15,		1,	1,	2),
	FP(CPU_DVFS_FREQ1_L_P15,		1,	1,	2),
	FP(CPU_DVFS_FREQ2_L_P15,		1,	1,	2),
	FP(CPU_DVFS_FREQ3_L_P15,		1,	1,	2),
	FP(CPU_DVFS_FREQ4_L_P15,		2,	1,	2),
	FP(CPU_DVFS_FREQ5_L_P15,		2,	1,	2),
	FP(CPU_DVFS_FREQ6_L_P15,		2,	1,	2),
	FP(CPU_DVFS_FREQ7_L_P15,		2,	2,	4),
};

static struct mt_cpu_freq_method opp_tbl_method_LL_e3[] = {
	/* Target Frequency,		POS, CLK, CCI */
	FP(CPU_DVFS_FREQ0_LL_P15,	2,	1,	2),
	FP(CPU_DVFS_FREQ1_LL_P15,	2,	1,	2),
	FP(CPU_DVFS_FREQ2_LL_P15,	2,	1,	2),
	FP(CPU_DVFS_FREQ3_LL_P15,	2,	1,	2),
	FP(CPU_DVFS_FREQ4_LL_P15,	2,	1,	2),
	FP(CPU_DVFS_FREQ5_LL_P15,	2,	2,	4),
	FP(CPU_DVFS_FREQ6_LL_P15,	2,	2,	4),
	FP(CPU_DVFS_FREQ7_LL_P15,	2,	4,	6),
};

struct opp_tbl_info {
	struct mt_cpu_freq_info *const opp_tbl;
	const int size;
};

static struct opp_tbl_info opp_tbls_little[2][4] = {
	{
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)] = {opp_tbl_little_e0_0, ARRAY_SIZE(opp_tbl_little_e0_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_1)] = {opp_tbl_little_e1_0, ARRAY_SIZE(opp_tbl_little_e1_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_2)] = {opp_tbl_little_e2_0, ARRAY_SIZE(opp_tbl_little_e2_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_3)] = {opp_tbl_little_e3_0, ARRAY_SIZE(opp_tbl_little_e3_0),},
	},
	{
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)] = {opp_tbl_little_e0_0, ARRAY_SIZE(opp_tbl_little_e0_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_1)] = {opp_tbl_little_e1_0, ARRAY_SIZE(opp_tbl_little_e1_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_2)] = {opp_tbl_little_e2_0, ARRAY_SIZE(opp_tbl_little_e2_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_3)] = {opp_tbl_little_e3_0, ARRAY_SIZE(opp_tbl_little_e3_0),},
	},
};

static struct opp_tbl_info opp_tbls_big[2][4] = {
	{
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)] = {opp_tbl_big_e0_0, ARRAY_SIZE(opp_tbl_big_e0_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_1)] = {opp_tbl_big_e1_0, ARRAY_SIZE(opp_tbl_big_e1_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_2)] = {opp_tbl_big_e2_0, ARRAY_SIZE(opp_tbl_big_e2_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_3)] = {opp_tbl_big_e3_0, ARRAY_SIZE(opp_tbl_big_e3_0),},
	},
	{
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)] = {opp_tbl_big_e0_0, ARRAY_SIZE(opp_tbl_big_e0_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_1)] = {opp_tbl_big_e1_0, ARRAY_SIZE(opp_tbl_big_e1_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_2)] = {opp_tbl_big_e2_0, ARRAY_SIZE(opp_tbl_big_e2_0),},
		[CPU_LV_TO_OPP_IDX(CPU_LEVEL_3)] = {opp_tbl_big_e3_0, ARRAY_SIZE(opp_tbl_big_e3_0),},
	},
};

static struct notifier_block __refdata _mt_cpufreq_cpu_notifier = {
	.notifier_call = _mt_cpufreq_cpu_CB,
};

/* for freq change (PLL/MUX) */
#define PLL_FREQ_STEP		(13000)	/* KHz */

#define PLL_MIN_FREQ		(130000)	/* KHz */
#define PLL_DIV1_FREQ		(1001000)	/* KHz */
#define PLL_DIV2_FREQ		(520000)	/* KHz */
#define PLL_DIV4_FREQ		(260000)	/* KHz */
#define PLL_DIV8_FREQ		(PLL_MIN_FREQ)	/* KHz */

#define DDS_DIV1_FREQ		(0x0009A000)	/* 1001MHz */
#define DDS_DIV2_FREQ		(0x010A0000)	/* 520MHz  */
#define DDS_DIV4_FREQ		(0x020A0000)	/* 260MHz  */
#define DDS_DIV8_FREQ		(0x030A0000)	/* 130MHz  */

int get_turbo_freq(int chip_id, int freq)
{
	return freq;
}

int get_turbo_volt(int chip_id, int volt)
{
	return volt;
}

static int _search_available_freq_idx(struct mt_cpu_dvfs *p, unsigned int target_khz,
				      unsigned int relation)
{
	int new_opp_idx = -1;
	int i;

	FUNC_ENTER(FUNC_LV_HELP);

	if (CPUFREQ_RELATION_L == relation) {
		for (i = (signed)(p->nr_opp_tbl - 1); i >= 0; i--) {
			if (cpu_dvfs_get_freq_by_idx(p, i) >= target_khz) {
				new_opp_idx = i;
				break;
			}
		}
	} else {		/* CPUFREQ_RELATION_H */
		for (i = 0; i < (signed)p->nr_opp_tbl; i++) {
			if (cpu_dvfs_get_freq_by_idx(p, i) <= target_khz) {
				new_opp_idx = i;
				break;
			}
		}
	}

	FUNC_EXIT(FUNC_LV_HELP);

	return new_opp_idx;
}

/* for PTP-OD */
static mt_cpufreq_set_ptbl_funcPTP mt_cpufreq_update_private_tbl;

void mt_cpufreq_set_ptbl_registerCB(mt_cpufreq_set_ptbl_funcPTP pCB)
{
	mt_cpufreq_update_private_tbl = pCB;
}
EXPORT_SYMBOL(mt_cpufreq_set_ptbl_registerCB);

static int _set_cur_volt_locked(struct mt_cpu_dvfs *p, unsigned int volt)	/* volt: mv * 100 */
{
	int ret = -1;

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);

	if (!cpu_dvfs_is_available(p)) {
		FUNC_EXIT(FUNC_LV_HELP);
		return 0;
	}

	/* set volt */
	ret = p->ops->set_cur_volt(p, volt);

	FUNC_EXIT(FUNC_LV_HELP);

	return ret;
}

static int _restore_default_volt(struct mt_cpu_dvfs *p)
{
	unsigned long flags;
	int i;
	int ret = -1;
	unsigned int freq = 0;
	unsigned int volt = 0;
	int idx = 0;

	enum mt_cpu_dvfs_id id;
	struct mt_cpu_dvfs *p_second;

	id = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	p_second = id_to_cpu_dvfs(id);

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);
	BUG_ON(NULL == p_second);

	if (!cpu_dvfs_is_available(p) || !cpu_dvfs_is_available(p_second)) {
		FUNC_EXIT(FUNC_LV_HELP);
		return 0;
	}

	cpufreq_lock(flags);

	/* restore to default volt */
	for (i = 0; i < p->nr_opp_tbl; i++)
		p->opp_tbl[i].cpufreq_volt = p->opp_tbl[i].cpufreq_volt_org;

	if (NULL != mt_cpufreq_update_private_tbl)
			mt_cpufreq_update_private_tbl(id, 1);

	freq = p->ops->get_cur_phy_freq(p);
	volt = p->ops->get_cur_volt(p);

	if (freq > cpu_dvfs_get_max_freq(p))
		idx = 0;
	else
		idx = _search_available_freq_idx(p, freq, CPUFREQ_RELATION_L);

	/* set volt */
	if ((p->idx_opp_tbl < p_second->idx_opp_tbl) ||
		(get_turbo_volt(p->cpu_id, cpu_dvfs_get_volt_by_idx(p, idx)) > volt))
		ret = _set_cur_volt_locked(p,
					   get_turbo_volt(p->cpu_id, cpu_dvfs_get_volt_by_idx(p, idx)));

	cpufreq_unlock(flags);

	FUNC_EXIT(FUNC_LV_HELP);

	return ret;
}

unsigned int mt_cpufreq_get_freq_by_idx(enum mt_cpu_dvfs_id id, int idx)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	FUNC_ENTER(FUNC_LV_API);

	BUG_ON(NULL == p);

	if (!cpu_dvfs_is_available(p)) {
		FUNC_EXIT(FUNC_LV_API);
		return 0;
	}

	BUG_ON(idx >= p->nr_opp_tbl);

	FUNC_EXIT(FUNC_LV_API);

	return cpu_dvfs_get_freq_by_idx(p, idx);
}
EXPORT_SYMBOL(mt_cpufreq_get_freq_by_idx);

int mt_cpufreq_update_volt(enum mt_cpu_dvfs_id id, unsigned int *volt_tbl, int nr_volt_tbl)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
	unsigned long flags;
	int i;
	int ret = -1;
	unsigned int freq = 0;
	unsigned int volt = 0;
	int idx = 0;

	enum mt_cpu_dvfs_id id_sec;
	struct mt_cpu_dvfs *p_second;

	FUNC_ENTER(FUNC_LV_API);

	id_sec = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	p_second = id_to_cpu_dvfs(id_sec);

	BUG_ON(NULL == p);
	BUG_ON(NULL == p_second);

	if (!cpu_dvfs_is_available(p) || !cpu_dvfs_is_available(p_second)) {
		FUNC_EXIT(FUNC_LV_API);
		return 0;
	}

	BUG_ON(nr_volt_tbl > p->nr_opp_tbl);

	cpufreq_lock(flags);

	/* update volt table */
	for (i = 0; i < nr_volt_tbl; i++)
		p->opp_tbl[i].cpufreq_volt = PMIC_VAL_TO_VOLT(volt_tbl[i]);

	if (NULL != mt_cpufreq_update_private_tbl)
			mt_cpufreq_update_private_tbl(id, 0);

	freq = p->ops->get_cur_phy_freq(p);
	volt = p->ops->get_cur_volt(p);

	if (freq > cpu_dvfs_get_max_freq(p))
		idx = 0;
	else
		idx = _search_available_freq_idx(p, freq, CPUFREQ_RELATION_L);

	/* dump_opp_table(p); */
	/* set volt */
	if ((p->idx_opp_tbl < p_second->idx_opp_tbl) ||
		(get_turbo_volt(p->cpu_id, cpu_dvfs_get_volt_by_idx(p, idx)) > volt))
		ret = _set_cur_volt_locked(p,
					   get_turbo_volt(p->cpu_id, cpu_dvfs_get_volt_by_idx(p, idx)));

	cpufreq_unlock(flags);

	FUNC_EXIT(FUNC_LV_API);

	return ret;
}
EXPORT_SYMBOL(mt_cpufreq_update_volt);

void mt_cpufreq_restore_default_volt(enum mt_cpu_dvfs_id id)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	FUNC_ENTER(FUNC_LV_API);

	BUG_ON(NULL == p);

	if (!cpu_dvfs_is_available(p)) {
		FUNC_EXIT(FUNC_LV_API);
		return;
	}

	_restore_default_volt(p);

	FUNC_EXIT(FUNC_LV_API);
}
EXPORT_SYMBOL(mt_cpufreq_restore_default_volt);
/* for PTP-OD End*/

/* for PBM */
unsigned int mt_cpufreq_get_leakage_mw(enum mt_cpu_dvfs_id id)
{
#ifndef DISABLE_PBM_FEATURE
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
#ifdef CONFIG_THERMAL
	int temp;

	if (!p)
		return 0;

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		temp = tscpu_get_temp_by_bank(THERMAL_BANK0) / 1000;
	else
		temp = tscpu_get_temp_by_bank(THERMAL_BANK2) / 1000;
#else
	int temp = 40;

	if (!p)
		return 0;
#endif
	/* if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		return mt_spower_get_leakage(MT_SPOWER_CPU, p->ops->get_cur_volt(p) / 100, temp);
	else */
		return mt_spower_get_leakage(MT_SPOWER_CPU, p->ops->get_cur_volt(p) / 100, temp);
#else
	return 0;
#endif
}

static void _kick_PBM_by_cpu(struct mt_cpu_dvfs *p)
{
	struct mt_cpu_dvfs *p_dvfs[NR_MT_CPU_DVFS];
	struct cpumask dvfs_cpumask[NR_MT_CPU_DVFS];
	struct cpumask cpu_online_cpumask[NR_MT_CPU_DVFS];
	struct ppm_cluster_status ppm_data[NR_MT_CPU_DVFS];
	int i;

	for (i = 0; i < NR_MT_CPU_DVFS; i++) {
		arch_get_cluster_cpus(&dvfs_cpumask[i], i);
		cpumask_and(&cpu_online_cpumask[i], &dvfs_cpumask[i], cpu_online_mask);

		p_dvfs[i] = id_to_cpu_dvfs(i);
		ppm_data[i].core_num = cpumask_weight(&cpu_online_cpumask[i]);
		ppm_data[i].freq_idx = p_dvfs[i]->idx_opp_tbl;
		ppm_data[i].volt = p_dvfs[i]->ops->get_cur_volt(p_dvfs[i]) / 100;

		cpufreq_ver("%d: core = %d, idx = %d, volt = %d\n",
			i, ppm_data[i].core_num, ppm_data[i].freq_idx, ppm_data[i].volt);
	}

	mt_ppm_dlpt_kick_PBM(ppm_data, (unsigned int)arch_get_nr_clusters());
}
/* for PBM End */
static unsigned int _cpu_freq_calc(unsigned int con1, unsigned int ckdiv1)
{
	unsigned int freq = 0;

	con1 &= _BITMASK_(26:0);

	if (con1 >= DDS_DIV8_FREQ) {
		freq = DDS_DIV8_FREQ;
		freq = PLL_DIV8_FREQ + (((con1 - freq) / 0x2000) * PLL_FREQ_STEP / 8);
	} else if (con1 >= DDS_DIV4_FREQ) {
		freq = DDS_DIV4_FREQ;
		freq = PLL_DIV4_FREQ + (((con1 - freq) / 0x2000) * PLL_FREQ_STEP / 4);
	} else if (con1 >= DDS_DIV2_FREQ) {
		freq = DDS_DIV2_FREQ;
		freq = PLL_DIV2_FREQ + (((con1 - freq) / 0x2000) * PLL_FREQ_STEP / 2);
	} else if (con1 >= DDS_DIV1_FREQ) {
		freq = DDS_DIV1_FREQ;
		freq = PLL_DIV1_FREQ + (((con1 - freq) / 0x2000) * PLL_FREQ_STEP);
	} else
		BUG();

	FUNC_ENTER(FUNC_LV_HELP);

	switch (ckdiv1) {
	case 9:
		freq = freq * 3 / 4;
		break;

	case 10:
		freq = freq * 2 / 4;
		break;

	case 11:
		freq = freq * 1 / 4;
		break;

	case 17:
		freq = freq * 4 / 5;
		break;

	case 18:
		freq = freq * 3 / 5;
		break;

	case 19:
		freq = freq * 2 / 5;
		break;

	case 20:
		freq = freq * 1 / 5;
		break;

	case 25:
		freq = freq * 5 / 6;
		break;

	case 26:
		freq = freq * 4 / 6;
		break;

	case 27:
		freq = freq * 3 / 6;
		break;

	case 28:
		freq = freq * 2 / 6;
		break;

	case 29:
		freq = freq * 1 / 6;
		break;

	case 8:
	case 16:
	case 24:
	default:
		break;
	}

	FUNC_EXIT(FUNC_LV_HELP);

	return freq;
}

static unsigned int get_cur_phy_freq_L(struct mt_cpu_dvfs *p)
{
	unsigned int con1;
	unsigned int ckdiv1;
	unsigned int cur_khz;

	FUNC_ENTER(FUNC_LV_LOCAL);

	BUG_ON(NULL == p);

#if 0
	/* read from freq meter */
	cur_khz = mt_get_cpu_freq();
	cpufreq_ver("@%s: Meter = %d MHz\n", __func__, cur_khz);
#else
	con1 = cpufreq_read(ARMCA15PLL_CON1);
	ckdiv1 = cpufreq_read(INFRA_TOPCKGEN_CKDIV1_BIG);
	ckdiv1 = _GET_BITS_VAL_(4:0, ckdiv1);

	cur_khz = _cpu_freq_calc(con1, ckdiv1);

	cpufreq_ver("@%s: cur_khz = %d, con1 = 0x%x, ckdiv1_val = 0x%x\n", __func__, cur_khz, con1,
		    ckdiv1);
#endif

	FUNC_EXIT(FUNC_LV_LOCAL);

	return cur_khz;
}

static unsigned int get_cur_phy_freq_LL(struct mt_cpu_dvfs *p)
{
	unsigned int con1;
	unsigned int ckdiv1;
	unsigned int cur_khz;

	FUNC_ENTER(FUNC_LV_LOCAL);

	BUG_ON(NULL == p);

#if 0
	/* read from freq meter */
	cur_khz = mt_get_cpu_freq();
	cpufreq_ver("@%s: Meter = %d MHz\n", __func__, cur_khz);
#else
	con1 = cpufreq_read(ARMCA7PLL_CON1);
	ckdiv1 = cpufreq_read(INFRA_TOPCKGEN_CKDIV1_SML);
	ckdiv1 = _GET_BITS_VAL_(4:0, ckdiv1);

	cur_khz = _cpu_freq_calc(con1, ckdiv1);

	cpufreq_ver("@%s: cur_khz = %d, con1 = 0x%x, ckdiv1_val = 0x%x\n", __func__, cur_khz, con1,
		    ckdiv1);
#endif

	FUNC_EXIT(FUNC_LV_LOCAL);

	return cur_khz;
}

static unsigned int get_cur_phy_freq_cci(void)
{
	unsigned int con1;
	unsigned int ckdiv1;
	unsigned int cur_khz;
	enum top_ckmuxsel_cci cur_cci_cluster;

	FUNC_ENTER(FUNC_LV_LOCAL);

	cur_cci_cluster = _get_cci_clock_switch();

	if (cur_cci_cluster == TOP_CKMUXSEL_ARMPLL_L)
		con1 = cpufreq_read(ARMCA15PLL_CON1);
	else
		con1 = cpufreq_read(ARMCA7PLL_CON1);

	ckdiv1 = cpufreq_read(INFRA_TOPCKGEN_CKDIV1_BUS);
	ckdiv1 = _GET_BITS_VAL_(4:0, ckdiv1);

	cur_khz = _cpu_freq_calc(con1, ckdiv1);

	cpufreq_ver("@%s: cur_khz = %d, con1 = 0x%x, ckdiv1_val = 0x%x\n", __func__, cur_khz, con1,
		    ckdiv1);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return cur_khz;
}

unsigned int mt_cpufreq_get_cur_phy_freq(enum mt_cpu_dvfs_id id)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
	unsigned int freq = 0;
	unsigned long flags;

	FUNC_ENTER(FUNC_LV_LOCAL);

	BUG_ON(NULL == p);

	cpufreq_lock(flags);
	freq = p->ops->get_cur_phy_freq(p);
	cpufreq_unlock(flags);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return freq;
}

static unsigned int _cpu_dds_calc(unsigned int khz)
{
	unsigned int dds = 0;

	FUNC_ENTER(FUNC_LV_HELP);

	if (khz >= PLL_DIV1_FREQ)
		dds = DDS_DIV1_FREQ + ((khz - PLL_DIV1_FREQ) / PLL_FREQ_STEP) * 0x2000;
	else if (khz >= PLL_DIV2_FREQ)
		dds = DDS_DIV2_FREQ + ((khz - PLL_DIV2_FREQ) * 2 / PLL_FREQ_STEP) * 0x2000;
	else if (khz >= PLL_DIV4_FREQ)
		dds = DDS_DIV4_FREQ + ((khz - PLL_DIV4_FREQ) * 4 / PLL_FREQ_STEP) * 0x2000;
	else if (khz >= PLL_DIV8_FREQ)
		dds = DDS_DIV8_FREQ + ((khz - PLL_DIV8_FREQ) * 8 / PLL_FREQ_STEP) * 0x2000;
	else
		BUG();

	FUNC_EXIT(FUNC_LV_HELP);

	return dds;
}

static void _cpu_clock_switch(struct mt_cpu_dvfs *p, enum top_ckmuxsel sel)
{
	unsigned int mask;

	FUNC_ENTER(FUNC_LV_HELP);

	switch (sel) {
	case TOP_CKMUXSEL_CLKSQ:
		break;
	case TOP_CKMUXSEL_ARMPLL:
		break;
	case TOP_CKMUXSEL_MAINPLL:
		cpufreq_write_mask(CLK_MISC_CFG_0, 5 : 4, 0x3);
		break;
	case TOP_CKMUXSEL_UNIVPLL:
		cpufreq_write_mask(CLK_MISC_CFG_0, 5 : 4, 0x3);
		break;
	default:
		break;
	};

	mask = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? _BITMASK_(9:8) : _BITMASK_(5:4);
	BUG_ON(sel >= NR_TOP_CKMUXSEL);

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		cpufreq_write_mask(TOP_CKMUXSEL, 9 : 8, sel);
	else
		cpufreq_write_mask(TOP_CKMUXSEL, 5 : 4, sel);

	if ((sel == TOP_CKMUXSEL_CLKSQ) || (sel == TOP_CKMUXSEL_ARMPLL))
		cpufreq_write_mask(CLK_MISC_CFG_0, 5 : 4, 0x0);

	FUNC_EXIT(FUNC_LV_HELP);
}

static enum top_ckmuxsel _get_cpu_clock_switch(struct mt_cpu_dvfs *p)
{
	unsigned int val = cpufreq_read(TOP_CKMUXSEL);
	unsigned int mask;

	FUNC_ENTER(FUNC_LV_HELP);

	mask = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? _BITMASK_(9:8) : _BITMASK_(5:4);
	val &= mask;		/* _BITMASK_(1 : 0) */

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		val = val >> 8;
	else
		val = val >> 4;

	FUNC_EXIT(FUNC_LV_HELP);

	return val;
}

int mt_cpufreq_clock_switch(enum mt_cpu_dvfs_id id, enum top_ckmuxsel sel)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	if (!p)
		return -1;

	_cpu_clock_switch(p, sel);

	return 0;
}

enum top_ckmuxsel mt_cpufreq_get_clock_switch(enum mt_cpu_dvfs_id id)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	if (!p)
		return -1;

	return _get_cpu_clock_switch(p);
}

#define POS_SETTLE_TIME (2)
#if 0
static void adjust_armpll_dds(struct mt_cpu_dvfs *p, unsigned int dds, unsigned int pos_div)
{
	unsigned int cur_volt;
	int shift;

	cur_volt = p->ops->get_cur_volt(p);
	p->ops->set_cur_volt(p, cpu_dvfs_get_volt_by_idx(p, 0));
	_cpu_clock_switch(p, TOP_CKMUXSEL_MAINPLL);
	cci_switch_from_to(CUR_CCI_OPP_IDX, BACKUP_CCI_OPP_IDX);
	shift = (pos_div == 1) ? 0 :
		(pos_div == 2) ? 1 :
		(pos_div == 4) ? 2 : 0;

	dds = (dds & ~(_BITMASK_(26:24))) | (shift << 24);
	cpufreq_write(p->armpll_addr, dds | _BIT_(31)); /* CHG */
	udelay(PLL_SETTLE_TIME);
	_cpu_clock_switch(p, TOP_CKMUXSEL_ARMPLL);
	cci_switch_from_to(BACKUP_CCI_OPP_IDX, CUR_CCI_OPP_IDX);
	p->ops->set_cur_volt(p, cur_volt);
	_get_cpu_clock_switch(p);
}
#endif

/* Stand-in for the FHCTL mt_dfs_armpll(), which is not built. Same
 * structure as adjust_posdiv() below and the #if 0 stock
 * adjust_armpll_dds() minus its voltage moves: the cluster runs from
 * MAINPLL for ~PLL_SETTLE_TIME around the CON1 write, so the CPU never runs
 * from the PLL mid-relock. The posdiv bits (26:24) in the register are
 * preserved; the caller strips them from dds and adjusts posdiv separately. */
static void forge_dfs_armpll(struct mt_cpu_dvfs *p, unsigned int dds)
{
	unsigned int con1;

	_cpu_clock_switch(p, TOP_CKMUXSEL_MAINPLL);
	cci_switch_from_to(CUR_CCI_OPP_IDX, BACKUP_CCI_OPP_IDX);

	con1 = cpufreq_read(p->armpll_addr);
	cpufreq_write(p->armpll_addr,
		      (con1 & _BITMASK_(26:24)) | (dds & ~_BITMASK_(26:24)) | _BIT_(31)); /* CHG */
	udelay(PLL_SETTLE_TIME);

	_cpu_clock_switch(p, TOP_CKMUXSEL_ARMPLL);
	cci_switch_from_to(BACKUP_CCI_OPP_IDX, CUR_CCI_OPP_IDX);
	_get_cpu_clock_switch(p);
}

static void adjust_posdiv(struct mt_cpu_dvfs *p, unsigned int pos_div)
{
	unsigned int dds;
	int shift;

	BUG_ON(cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE));

	/* disable SSC */
	/* freqhopping_config(FH_ARMCA15_PLLID, 1599000, 0); */
	_cpu_clock_switch(p, TOP_CKMUXSEL_MAINPLL);
	cci_switch_from_to(CUR_CCI_OPP_IDX, BACKUP_CCI_OPP_IDX);
	dds = cpufreq_read(p->armpll_addr);
	shift = (pos_div == 1) ? 0 :
		(pos_div == 2) ? 1 :
		(pos_div == 4) ? 2 : 0;

	dds = (dds & ~(_BITMASK_(26:24))) | (shift << 24);
	cpufreq_write(p->armpll_addr, dds | _BIT_(31)); /* CHG */
	udelay(POS_SETTLE_TIME);
	_cpu_clock_switch(p, TOP_CKMUXSEL_ARMPLL);
	/* enable SSC */
	/* freqhopping_config(FH_ARMCA15_PLLID, 1599000, 1); */

	cci_switch_from_to(BACKUP_CCI_OPP_IDX, CUR_CCI_OPP_IDX);
	_get_cpu_clock_switch(p);
}

static void adjust_clkdiv(struct mt_cpu_dvfs *p, unsigned int clk_div)
{
	unsigned int sel = 0;
	unsigned int ckdiv1_val;
	unsigned int ckdiv1_mask = _BITMASK_(4 : 0);

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		ckdiv1_val = _GET_BITS_VAL_(4 : 0, cpufreq_read(INFRA_TOPCKGEN_CKDIV1_SML));
	else
		ckdiv1_val = _GET_BITS_VAL_(4 : 0, cpufreq_read(INFRA_TOPCKGEN_CKDIV1_BIG));

	sel = (clk_div == 1) ? 8 :
		(clk_div == 2) ? 10 :
		(clk_div == 4) ? 11 : 8;

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		cpufreq_write(INFRA_TOPCKGEN_CKDIV1_SML, (ckdiv1_val & ~ckdiv1_mask) | sel);
	else
		cpufreq_write(INFRA_TOPCKGEN_CKDIV1_BIG, (ckdiv1_val & ~ckdiv1_mask) | sel);

	udelay(POS_SETTLE_TIME);
}

static void adjust_cci_div(unsigned int cci_div)
{
	unsigned int sel = 0;

	sel = (cci_div == 1) ? 8 :
		(cci_div == 2) ? 10 :
		(cci_div == 4) ? 11 :
		(cci_div == 6) ? 29 : 8;

	cpufreq_write_mask(INFRA_TOPCKGEN_CKDIV1_BUS, 4 : 0, sel);
	udelay(POS_SETTLE_TIME);
}

static int armpll_exist_in_both_side(void)
{
	struct mt_cpu_dvfs *p_little = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
	struct mt_cpu_dvfs *p_big = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);

#if 0
	unsigned int con0_LL_enable = _GET_BITS_VAL_(0:0, cpufreq_read(ARMCA7PLL_CON0));
	unsigned int con0_L_enable = _GET_BITS_VAL_(0:0, cpufreq_read(ARMCA15PLL_CON0));

	if ((con0_LL_enable && con0_L_enable) == 1)
		return 1;
	else
		return 0;
#else
	if ((p_little->armpll_is_available == 1) && (p_big->armpll_is_available == 1))
		return 1;
	else
		return 0;
#endif
}

static enum top_ckmuxsel_cci _get_cci_clock_switch(void)
{
	unsigned int val = cpufreq_read(TOP_CKMUXSEL);
	unsigned int mask;

	FUNC_ENTER(FUNC_LV_HELP);

	mask = _BITMASK_(13:12);
	val &= mask;		/* _BITMASK_(1 : 0) */
	val = val >> 12;

	switch (val) {
	case TOP_CKMUXSEL_CLKSQ_CCI:
		cpufreq_ver("CCI in 26M\n");
		break;
	case TOP_CKMUXSEL_ARMPLL_L:
		cpufreq_ver("CCI in ARMPLL_L\n");
		break;
	case TOP_CKMUXSEL_ARMPLL_LL:
		cpufreq_ver("CCI in ARMPLL_LL\n");
		break;
	case TOP_CKMUXSEL_CLKSQ_CCI_2:
		cpufreq_ver("CCI in 26M\n");
		break;
	default:
		break;
	};

	FUNC_EXIT(FUNC_LV_HELP);

	return val;
}

/* fix sel */
static void _cci_clock_switch(enum top_ckmuxsel_cci sel)
{
	FUNC_ENTER(FUNC_LV_HELP);

	cpufreq_write_mask(TOP_CKMUXSEL, 13 : 12, sel);

	FUNC_EXIT(FUNC_LV_HELP);
}

static unsigned int forge_row_at_or_below(struct mt_cpu_dvfs *p, unsigned int khz,
					  unsigned int vmax);
static int _sync_opp_tbl_idx(struct mt_cpu_dvfs *p);

static int search_table_idx_by_freq(struct mt_cpu_dvfs *p, unsigned int freq)
{
	int i;
	int idx = -1;

	for (i = 0; i < p->nr_opp_tbl; i++) {
		if (p->freq_tbl[i].target_f == freq) {
			idx = i;
			break;
		}
	}

	if (idx == -1)
		idx = p->idx_opp_tbl;

	return idx;
}

static void cci_switch_from_to(enum opp_idx_type from, enum opp_idx_type to)
{
	if (opp_tbl[from].slot == opp_tbl[to].slot)
		return;

#ifdef DCM_ENABLE
	sync_dcm_set_cci_freq(0);
#endif

	/* cci_div 1 -> 2 */
	if (opp_tbl[from].slot->cci_div < opp_tbl[to].slot->cci_div)
		adjust_cci_div(opp_tbl[to].slot->cci_div);

	/* switch CCI */
	_cci_clock_switch(opp_tbl[to].p->armpll_clk_src);

	/* cci_div 2 -> 1 */
	if (opp_tbl[from].slot->cci_div > opp_tbl[to].slot->cci_div)
		adjust_cci_div(opp_tbl[to].slot->cci_div);
}

static void set_cur_freq(struct mt_cpu_dvfs *p, unsigned int cur_khz, unsigned int target_khz)
{
	enum mt_cpu_dvfs_id id_second =
	    (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	struct mt_cpu_dvfs *p_second = id_to_cpu_dvfs(id_second);
	struct mt_cpu_dvfs *p_temp;
	unsigned int dds;
	enum top_ckmuxsel_cci cur_cci_status;
#ifdef DCM_ENABLE
	unsigned int cur_cci_freq;
	unsigned int target_cci_freq;
#endif
	int idx;

	FUNC_ENTER(FUNC_LV_LOCAL);

	now[SET_FREQ] = ktime_get();
	cpufreq_ver("%s: cur_khz = %d, target_khz = %d\n",
		__func__, cur_khz, target_khz);

	/* CUR_OPP_IDX */
	opp_tbl[CUR_OPP_IDX].p = p;
	opp_tbl[CUR_OPP_IDX].slot = &p->freq_tbl[p->idx_opp_tbl];
	cpufreq_ver("[CUR_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
		cpu_dvfs_get_name(p), p->idx_opp_tbl, cpu_dvfs_get_freq_by_idx(p, p->idx_opp_tbl));

	/* TARGET_OPP_IDX */
	opp_tbl[TARGET_OPP_IDX].p = p;
	idx = search_table_idx_by_freq(opp_tbl[TARGET_OPP_IDX].p,
		target_khz);
	/* voltage DVFS: the rails go up before any PLL raise, so a row above
	 * the read-back VPROC is never a legitimate move (procfs and hotplug
	 * pick rows on their own); after a fault a move that lowers the PLL
	 * still goes */
	if (forge_full_dvfs && cpu_dvfs_get_volt_by_idx(p, idx) > forge_vproc_cur &&
	    !(forge_dvfs_fault && target_khz < p->ops->get_cur_phy_freq(p))) {
		pr_err_ratelimited("[FORGE_CPUFREQ] REFUSED %s row %d (%u kHz): needs %u > VPROC %u\n",
				   cpu_dvfs_get_name(p), idx, target_khz,
				   cpu_dvfs_get_volt_by_idx(p, idx), forge_vproc_cur);
		forge_freq_refused = true;
		return;
	}
	opp_tbl[TARGET_OPP_IDX].slot = &p->freq_tbl[idx];
	cpufreq_ver("[TARGET_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
		cpu_dvfs_get_name(p), idx, cpu_dvfs_get_freq_by_idx(p, idx));

	/* CUR_CCI_OPP_IDX */
	cur_cci_status = _get_cci_clock_switch();
	if (cur_cci_status == TOP_CKMUXSEL_ARMPLL_LL)
		p_temp = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
	else
		p_temp = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);
	opp_tbl[CUR_CCI_OPP_IDX].p = p_temp;
	opp_tbl[CUR_CCI_OPP_IDX].slot = &p_temp->freq_tbl[p_temp->idx_opp_tbl];
	cpufreq_ver("[CUR_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
		cpu_dvfs_get_name(p_temp), p_temp->idx_opp_tbl, cpu_dvfs_get_freq_by_idx(p_temp, p_temp->idx_opp_tbl));

	/* TARGET_CCI_OPP_IDX */
	if (armpll_exist_in_both_side()) {
		cpufreq_ver("ARMPLL both side\n");
		if (target_khz > p_second->ops->get_cur_phy_freq(p_second)) {
			opp_tbl[TARGET_CCI_OPP_IDX].p = p;
			opp_tbl[TARGET_CCI_OPP_IDX].slot = opp_tbl[TARGET_OPP_IDX].slot;
			cpufreq_ver("[TARGET_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:[TARGET_OPP_IDX]:%d\n",
				cpu_dvfs_get_name(p), target_khz);
		} else {
			opp_tbl[TARGET_CCI_OPP_IDX].p = p_second;
			opp_tbl[TARGET_CCI_OPP_IDX].slot = &p_second->freq_tbl[p_second->idx_opp_tbl];
			cpufreq_ver("[TARGET_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
				cpu_dvfs_get_name(p_second), p_second->idx_opp_tbl,
				cpu_dvfs_get_freq_by_idx(p_second, p_second->idx_opp_tbl));
		}
	} else {
		cpufreq_ver("ARMPLL single side\n");
		/* BUG_ON(!p->armpll_is_available); */
		if (p->armpll_is_available) {
			opp_tbl[TARGET_CCI_OPP_IDX].p = p;
			opp_tbl[TARGET_CCI_OPP_IDX].slot = opp_tbl[TARGET_OPP_IDX].slot;
			cpufreq_ver("[TARGET_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:[TARGET_OPP_IDX]:%d\n",
				cpu_dvfs_get_name(p), target_khz);
		} else {
			opp_tbl[TARGET_CCI_OPP_IDX].p = p_second;
			opp_tbl[TARGET_CCI_OPP_IDX].slot = &p_second->freq_tbl[p_second->idx_opp_tbl];
			cpufreq_ver("[TARGET_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
				cpu_dvfs_get_name(p_second), p_second->idx_opp_tbl,
				cpu_dvfs_get_freq_by_idx(p_second, p_second->idx_opp_tbl));
		}
	}

	/* BACKUP_CCI_OPP_IDX */
	opp_tbl[BACKUP_CCI_OPP_IDX].p = p_second;
	opp_tbl[BACKUP_CCI_OPP_IDX].slot = &p_second->freq_tbl[p_second->idx_opp_tbl];
	cpufreq_ver("[BACKUP_CCI_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
		cpu_dvfs_get_name(p_second), p_second->idx_opp_tbl,
		cpu_dvfs_get_freq_by_idx(p_second, p_second->idx_opp_tbl));

	if (cur_khz == target_khz && p->armpll_is_available)
		return;

	/* DCM (freq: high -> low) */
#ifdef DCM_ENABLE
	sync_dcm_set_cci_freq(0);

	if (cur_khz > target_khz) {
		if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
			sync_dcm_set_mp0_freq(target_khz/1000);
		else
			sync_dcm_set_mp1_freq(target_khz/1000);
	}
#endif

	/* post_div 1 -> 2 */
	if (opp_tbl[CUR_OPP_IDX].slot->pos_div < opp_tbl[TARGET_OPP_IDX].slot->pos_div)
		adjust_posdiv(p, opp_tbl[TARGET_OPP_IDX].slot->pos_div);

	/* armpll_div 1 -> 2 */
	if (opp_tbl[CUR_OPP_IDX].slot->clk_div < opp_tbl[TARGET_OPP_IDX].slot->clk_div)
		adjust_clkdiv(p, opp_tbl[TARGET_OPP_IDX].slot->clk_div);

	/* cci_div 1 -> 2 */
	if (opp_tbl[CUR_CCI_OPP_IDX].slot->cci_div < opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div)
		adjust_cci_div(opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div);

	/* actual FHCTL */
	dds = _cpu_dds_calc(opp_tbl[TARGET_OPP_IDX].slot->vco_dds);
	/* adjust_armpll_dds(p, dds, opp_tbl[TARGET_OPP_IDX].slot->pos_div); */
	dds &= ~(_BITMASK_(26:24));
	/* FHCTL (mt_dfs_armpll, glitch-free slew) is not built: the
	 * freqhopping driver pulls in cpufreq_hybrid and dramc. Use the stock
	 * non-hopping recipe instead: park the cluster mux on MAINPLL, write
	 * CON1 with CHG, settle, mux back, as adjust_posdiv() does above. */
	forge_dfs_armpll(p, dds);

	/* switch CCI */
	_cci_clock_switch(opp_tbl[TARGET_CCI_OPP_IDX].p->armpll_clk_src);

	/* cci_div 2 -> 1 */
	if (opp_tbl[CUR_CCI_OPP_IDX].slot->cci_div > opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div)
		adjust_cci_div(opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div);

	/* armpll_div 2 -> 1 */
	if (opp_tbl[CUR_OPP_IDX].slot->clk_div > opp_tbl[TARGET_OPP_IDX].slot->clk_div)
		adjust_clkdiv(p, opp_tbl[TARGET_OPP_IDX].slot->clk_div);

	/* post_div 2 -> 1 */
	if (opp_tbl[CUR_OPP_IDX].slot->pos_div > opp_tbl[TARGET_OPP_IDX].slot->pos_div)
		adjust_posdiv(p, opp_tbl[TARGET_OPP_IDX].slot->pos_div);

#ifdef DCM_ENABLE
	/* DCM (cci freq) */
	cur_cci_freq = opp_tbl[CUR_CCI_OPP_IDX].slot->target_f/opp_tbl[CUR_CCI_OPP_IDX].slot->cci_div;
	target_cci_freq = opp_tbl[TARGET_CCI_OPP_IDX].slot->target_f/opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div;
	sync_dcm_set_cci_freq(target_cci_freq/1000);

	/* DCM (freq: low -> high)*/
	if (cur_khz < target_khz) {
		if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
			sync_dcm_set_mp0_freq(target_khz/1000);
		else
			sync_dcm_set_mp1_freq(target_khz/1000);
	}
#endif
	delta[SET_FREQ] = ktime_sub(ktime_get(), now[SET_FREQ]);
	if (ktime_to_us(delta[SET_FREQ]) > ktime_to_us(max[SET_FREQ]))
		max[SET_FREQ] = delta[SET_FREQ];

	FUNC_EXIT(FUNC_LV_LOCAL);
}

#ifdef CONFIG_HYBRID_CPU_DVFS
static void set_cur_freq_hybrid(struct mt_cpu_dvfs *p, unsigned int cur_khz, unsigned int target_khz)
{
	int r, index;
	unsigned int cluster, volt, volt_val = 0;

	BUG_ON(!enable_cpuhvfs);

	cluster = cpu_dvfs_to_cluster(p);
	index = search_table_idx_by_freq(p, target_khz);
	volt = cpu_dvfs_get_volt_by_idx(p, index);

	cpufreq_ver("cluster%u: target_khz = %u (%u), index = %d (%u)\n",
		    cluster, target_khz, cur_khz, index, volt);

	aee_record_cpu_volt(p, volt);

	r = cpuhvfs_set_target_opp(cluster, index, &volt_val);
	BUG_ON(r);

	notify_cpu_volt_sampler(p, EXTBUCK_VAL_TO_VOLT(volt_val));
}
#endif

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
/* for volt change (PMICWRAP/extBuck) */
static unsigned int get_cur_vsram(struct mt_cpu_dvfs *p)
{
	unsigned int rdata = 0;

	FUNC_ENTER(FUNC_LV_LOCAL);

#if FORGE_FREQ_ONLY
	/* The mt6353_upmu_* helper below uses MT6353 register addresses but
	 * the PMIC is MT6351, so it would read unrelated bits and trip the
	 * vsram >= vproc sanity BUG. The rail is never moved in this build;
	 * return the preloader value. */
	FUNC_EXIT(FUNC_LV_LOCAL);
	return forge_full_dvfs ? forge_vsram_cur : FORGE_FIXED_VSRAM;
#endif
	rdata = mt6353_upmu_get_da_ni_vsram_proc_vosel();
	rdata = PMIC_VAL_TO_VOLT(rdata);
	/* cpufreq_ver("@%s: vsram = %d\n", __func__, rdata); */

	FUNC_EXIT(FUNC_LV_LOCAL);

	return rdata;		/* vproc: mv*100 */
}

unsigned int __attribute__((weak)) mt6311_read_byte(unsigned char cmd, unsigned char *returnData)
{
	return 0;
}

static unsigned int get_cur_volt_extbuck(struct mt_cpu_dvfs *p)
{
	unsigned int ret_volt = 0;	/* volt: mv * 100 */
	unsigned int retry_cnt = 5;

	FUNC_ENTER(FUNC_LV_LOCAL);

#if FORGE_FREQ_ONLY
	/* No live readback path (mt6311_read_byte is a weak stub returning 0
	 * and the pwrap fallback reads MT6353 addresses on an MT6351); the
	 * rail is set by the preloader and never moved in this build. */
	FUNC_EXIT(FUNC_LV_LOCAL);
	return forge_full_dvfs ? forge_vproc_cur : FORGE_FIXED_VPROC;
#endif
	if (cpu_dvfs_is_extbuck_valid()) {
		unsigned char ret_val_buck = 0;

		do {
			if (!mt6311_read_byte(MT6311_VDVFS11_CON13, &ret_val_buck)) {
				cpufreq_err("%s(), fail to read ext buck volt\n", __func__);
				ret_volt = 0;
			} else {
				ret_volt = EXTBUCK_VAL_TO_VOLT(ret_val_buck);
				/* cpufreq_ver("@%s: volt = %d\n", __func__, ret_volt); */
			}
		} while (ret_volt == EXTBUCK_VAL_TO_VOLT(0) && retry_cnt--);
	} else {
		unsigned int ret_val = 0;
	ret_val = mt6353_upmu_get_da_ni_vproc_vosel();
	ret_volt = PMIC_VAL_TO_VOLT(ret_val);
	}

	FUNC_EXIT(FUNC_LV_LOCAL);

	return ret_volt;
}
#else
/* for volt change (PMICWRAP/extBuck) */
static unsigned int get_cur_vsram(struct mt_cpu_dvfs *p)
{
	unsigned int rdata = 0;
	unsigned int retry_cnt = 5;

	FUNC_ENTER(FUNC_LV_LOCAL);
#if 0
	#define PMIC_ADDR_VSRAM_EN           0x00A6
	rdata = pmic_get_register_value(PMIC_ADDR_VSRAM_EN);
	rdata &= _BITMASK_(1:1);	/* enable or disable (i.e. 0mv or not) */
#else
	rdata = 1;
#endif
	if (rdata) {
		do {
			rdata = pmic_get_register_value(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON);
		} while ((rdata == 0 || rdata > 0x7F) && retry_cnt--);

		rdata = PMIC_VAL_TO_VOLT(rdata);
		/* cpufreq_ver("@%s: volt = %d\n", __func__, rdata); */
	} else
		cpufreq_err("@%s: read VSRAM_EN failed, rdata = 0x%x\n", __func__, rdata);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return rdata;		/* vproc: mv*100 */
}

static unsigned int get_cur_volt_extbuck(struct mt_cpu_dvfs *p)
{
	unsigned char ret_val = 0;
	unsigned int ret_volt = 0;	/* volt: mv * 100 */
	unsigned int retry_cnt = 5;

	FUNC_ENTER(FUNC_LV_LOCAL);

	if (cpu_dvfs_is_extbuck_valid()) {
		do {
			if (!mt6311_read_byte(MT6311_VDVFS11_CON13, &ret_val)) {
				cpufreq_err("%s(), fail to read ext buck volt\n", __func__);
				ret_volt = 0;
			} else {
				ret_volt = EXTBUCK_VAL_TO_VOLT(ret_val);
				/* cpufreq_ver("@%s: volt = %d\n", __func__, ret_volt); */
			}
		} while (ret_volt == EXTBUCK_VAL_TO_VOLT(0) && retry_cnt--);
	} else
		cpufreq_err("%s(), can not use ext buck!\n", __func__);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return ret_volt;
}
#endif

unsigned int mt_cpufreq_get_cur_volt(enum mt_cpu_dvfs_id id)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	FUNC_ENTER(FUNC_LV_API);

	BUG_ON(NULL == p);
	BUG_ON(NULL == p->ops);

	FUNC_EXIT(FUNC_LV_API);

	return p->ops->get_cur_volt(p);	/* mv * 100 */
}
EXPORT_SYMBOL(mt_cpufreq_get_cur_volt);

static unsigned int _calc_pmic_settle_time(unsigned int old_vproc, unsigned int old_vsram,
					   unsigned int new_vproc, unsigned int new_vsram)
{
	unsigned delay = 100;

	if (new_vproc == old_vproc && new_vsram == old_vsram)
		return 0;

	/* VPROC is UP */
	if (new_vproc >= old_vproc) {
		/* VSRAM is UP too, choose larger one to calculate settle time */
		if (new_vsram >= old_vsram)
			delay = MAX(PMIC_VOLT_UP_SETTLE_TIME(old_vsram, new_vsram),
				    PMIC_VOLT_UP_SETTLE_TIME(old_vproc, new_vproc)
			    );
		/* VSRAM is DOWN, it may happen at bootup stage */
		else
			delay = MAX(PMIC_VOLT_DOWN_SETTLE_TIME(old_vsram, new_vsram),
				    PMIC_VOLT_UP_SETTLE_TIME(old_vproc, new_vproc)
			    );
	}
	/* VPROC is DOWN */
	else {
		/* VSRAM is DOWN too, choose larger one to calculate settle time */
		if (old_vsram >= new_vsram)
			delay = MAX(PMIC_VOLT_DOWN_SETTLE_TIME(old_vsram, new_vsram),
				    PMIC_VOLT_DOWN_SETTLE_TIME(old_vproc, new_vproc)
			    );
		/* VSRAM is UP, it may happen at bootup stage */
		else
			delay = MAX(PMIC_VOLT_UP_SETTLE_TIME(old_vsram, new_vsram),
				    PMIC_VOLT_DOWN_SETTLE_TIME(old_vproc, new_vproc)
			    );
	}

	if (delay < MIN_PMIC_SETTLE_TIME)
		delay = MIN_PMIC_SETTLE_TIME;

	return delay;
}

static void dump_opp_table(struct mt_cpu_dvfs *p)
{
	int i;

	cpufreq_dbg("[%s/%d]\n" "cpufreq_oppidx = %d\n", p->name, p->cpu_id, p->idx_opp_tbl);

	for (i = 0; i < p->nr_opp_tbl; i++) {
		cpufreq_dbg("\tOP(%d, %d),\n",
			    cpu_dvfs_get_freq_by_idx(p, i), cpu_dvfs_get_volt_by_idx(p, i)
		    );
	}
}

unsigned int debug_vsram = 0;
unsigned int debug_vproc = 0;
unsigned int last_vsram = 0;
unsigned int last_vproc = 0;
int fail_case = 0;
int fail_times = 0;

static void forge_dvfs_fault_kick(void);
static unsigned int _search_available_volt(struct mt_cpu_dvfs *p, unsigned int target_khz);

/* called under the cpufreq lock (_cpufreq_set_locked, park) */
static int forge_set_rails_floor(unsigned int volt, unsigned int floor)
{
	unsigned int rb_p = forge_vproc_cur, rb_s = forge_vsram_cur;
	struct mt_cpu_dvfs *ll = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
	struct mt_cpu_dvfs *big = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);
	int ret;

	/* never below what the clocks running right now need, whoever asks */
	volt = max(volt, _search_available_volt(ll, ll->ops->get_cur_phy_freq(ll)));
	volt = max(volt, _search_available_volt(big, big->ops->get_cur_phy_freq(big)));
	/* the writer's floor is 1.000 V (forge_vrail.c) */
	volt = clamp_t(unsigned int, max(volt, floor), 100000, 115000);
	/* after a fault no rail is written again: a raise is refused, a
	 * lower request leaves the (higher) rail where it is */
	if (forge_dvfs_fault)
		return volt > forge_vproc_cur ? -EIO : 0;
	if (volt == forge_vproc_cur)
		return 0;
	ret = forge_vrail_set(forge_dvfs_cookie, volt, &rb_p, &rb_s);
	/* on a fault the writer hands out the lower of the old and the target
	 * code (N1): every limit below counts on what is surely there */
	forge_vproc_cur = rb_p;
	forge_vsram_cur = rb_s;
	/* -EAGAIN = nothing written (held for suspend/reboot, allow=0,
	 * sleep on), no fault: a lower request is done (the rail stays
	 * higher), a raise fails and the clock stays */
	if (ret == -EAGAIN)
		return volt > forge_vproc_cur ? -EBUSY : 0;
	if (ret && !forge_dvfs_fault) {
		forge_dvfs_fault = true;
		cpufreq_err("[FORGE_DVFS] rail write to %u failed or refused (%d), read back VPROC %u VSRAM %u: fault latched, ceilings to the rows legal there\n",
			    volt, ret, rb_p, rb_s);
		forge_dvfs_fault_kick();
	}
	return ret;
}

static int forge_set_rails(unsigned int volt)
{
	/* the knob can only raise the 1.000 V floor (min_t made it a no-op) */
	return forge_set_rails_floor(volt, max_t(unsigned int, READ_ONCE(forge_vproc_floor), 100000));
}

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
void __attribute__((weak)) mt6311_set_vdvfs11_vosel_on(unsigned char val) {}

static int set_cur_volt_extbuck(struct mt_cpu_dvfs *p, unsigned int volt)
{				/* volt: vproc (mv*100) */
	unsigned int cur_vsram;
	unsigned int cur_vproc;
	unsigned int delay_us = 0;
	int ret = 0;

	enum mt_cpu_dvfs_id id_sec;
	struct mt_cpu_dvfs *p_second;

	FUNC_ENTER(FUNC_LV_LOCAL);

#if FORGE_FREQ_ONLY
	/* In the frequency-only build both write targets are wrong or absent
	 * (the VPROC setter is a weak no-op, the VSRAM helper uses MT6353
	 * addresses on MT6351). Refuse the whole move rather than actuate
	 * partially; reporting success is safe because every reachable OPP
	 * runs at the fixed boot voltage. With voltage DVFS forge_vrail.c
	 * moves both rails. */
	if (forge_full_dvfs) {
		FUNC_EXIT(FUNC_LV_LOCAL);
		return forge_set_rails(volt);
	}
	pr_info_once("[FORGE_CPUFREQ] freq-only build: voltage moves are no-ops (VPROC fixed at PL 1.000 V)\n");
	FUNC_EXIT(FUNC_LV_LOCAL);
	return 0;
#endif

	/* First statement of any real VPROC/VSRAM write. */
	if (forge_opp_guard_check("set_cur_volt_extbuck")) {
		FUNC_EXIT(FUNC_LV_LOCAL);
		return -EPERM;
	}

	aee_record_cpu_volt(p, volt);

	now[SET_VOLT] = ktime_get();

	cur_vsram = get_cur_vsram(p);
	cur_vproc = get_cur_volt_extbuck(p);

	id_sec = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	p_second = id_to_cpu_dvfs(id_sec);

	if (unlikely
	    (!((cur_vsram >= cur_vproc) && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
#ifdef __KERNEL__
		aee_kernel_warning(TAG, "@%s():%d, cur_vsram = %d, cur_vproc = %d\n",
				   __func__, __LINE__, cur_vsram, cur_vproc);
#endif
	}

	/* UP */
	if (volt > cur_vproc) {
		unsigned int target_vsram = volt + NORMAL_DIFF_VRSAM_VPROC;
		unsigned int next_vsram;

		do {
			unsigned int old_vproc = cur_vproc;
			unsigned int old_vsram = cur_vsram;

			next_vsram = MIN(((MAX_DIFF_VSRAM_VPROC - 2500) + cur_vproc), target_vsram);

			/* update vsram */
			cur_vsram = MAX(next_vsram, MIN_VSRAM_VOLT);

			if (cur_vsram > MAX_VSRAM_VOLT) {
				cur_vsram = MAX_VSRAM_VOLT;
				target_vsram = MAX_VSRAM_VOLT;	/* to end the loop */
			}

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}
			now[SET_VSRAM] = ktime_get();
			last_vsram = cur_vsram;
			mt6353_upmu_set_ldo_vsram_proc_vosel_ctrl(1);
			mt6353_upmu_set_ldo_vsram_proc_vosel_on(VOLT_TO_PMIC_VAL(cur_vsram));
			delta[SET_VSRAM] = ktime_sub(ktime_get(), now[SET_VSRAM]);
			if (ktime_to_us(delta[SET_VSRAM]) > ktime_to_us(max[SET_VSRAM]))
				max[SET_VSRAM] = delta[SET_VSRAM];

			/* update vproc */
			if (next_vsram > MAX_VSRAM_VOLT)
				cur_vproc = volt;	/* Vsram was limited, set to target vproc directly */
			else
				cur_vproc = next_vsram - NORMAL_DIFF_VRSAM_VPROC;

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 2;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}

			last_vproc = cur_vproc;
			now[SET_VPROC] = ktime_get();
			if (is_extbuck_valid) {
				mt6311_set_vdvfs11_vosel_on(VOLT_TO_EXTBUCK_VAL(cur_vproc));
			} else {
			mt6353_upmu_set_buck_vproc_vosel_ctrl(1);
			mt6353_upmu_set_buck_vproc_vosel_on(VOLT_TO_PMIC_VAL(cur_vproc));
			}
			delta[SET_VPROC] = ktime_sub(ktime_get(), now[SET_VPROC]);
			if (ktime_to_us(delta[SET_VPROC]) > ktime_to_us(max[SET_VPROC]))
				max[SET_VPROC] = delta[SET_VPROC];

			now[SET_DELAY] = ktime_get();
			delay_us =
			    _calc_pmic_settle_time(old_vproc, old_vsram, cur_vproc, cur_vsram);
			udelay(delay_us);
			delta[SET_DELAY] = ktime_sub(ktime_get(), now[SET_DELAY]);
			if (ktime_to_us(delta[SET_DELAY]) > ktime_to_us(max[SET_DELAY]))
				max[SET_DELAY] = delta[SET_DELAY];
			cpufreq_ver
			    ("@%s(): UP --> old_vsram=%d, cur_vsram=%d, old_vproc=%d, cur_vproc=%d, delay=%d\n",
			     __func__, old_vsram, cur_vsram, old_vproc, cur_vproc, delay_us);
		} while (target_vsram > cur_vsram);
	}
	/* DOWN */
	else if (volt < cur_vproc) {
		unsigned int next_vproc;
		unsigned int next_vsram = cur_vproc + NORMAL_DIFF_VRSAM_VPROC;

		do {
			unsigned int old_vproc = cur_vproc;
			unsigned int old_vsram = cur_vsram;

			next_vproc = MAX((next_vsram - (MAX_DIFF_VSRAM_VPROC - 2500)), volt);

			/* update vproc */
			cur_vproc = next_vproc;

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 3;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}

			last_vproc = cur_vproc;
			now[SET_VPROC] = ktime_get();
			if (is_extbuck_valid) {
				mt6311_set_vdvfs11_vosel_on(VOLT_TO_EXTBUCK_VAL(cur_vproc));
			} else {
			mt6353_upmu_set_buck_vproc_vosel_ctrl(1);
			mt6353_upmu_set_buck_vproc_vosel_on(VOLT_TO_PMIC_VAL(cur_vproc));
			}
			delta[SET_VPROC] = ktime_sub(ktime_get(), now[SET_VPROC]);
			if (ktime_to_us(delta[SET_VPROC]) > ktime_to_us(max[SET_VPROC]))
				max[SET_VPROC] = delta[SET_VPROC];

			/* update vsram */
			next_vsram = cur_vproc + NORMAL_DIFF_VRSAM_VPROC;
			cur_vsram = MAX(next_vsram, MIN_VSRAM_VOLT);
			cur_vsram = MIN(cur_vsram, MAX_VSRAM_VOLT);

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 4;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}
			now[SET_VSRAM] = ktime_get();
			last_vsram = cur_vsram;
			mt6353_upmu_set_ldo_vsram_proc_vosel_ctrl(1);
			mt6353_upmu_set_ldo_vsram_proc_vosel_on(VOLT_TO_PMIC_VAL(cur_vsram));
			delta[SET_VSRAM] = ktime_sub(ktime_get(), now[SET_VSRAM]);
			if (ktime_to_us(delta[SET_VSRAM]) > ktime_to_us(max[SET_VSRAM]))
				max[SET_VSRAM] = delta[SET_VSRAM];

			now[SET_DELAY] = ktime_get();
			delay_us =
			    _calc_pmic_settle_time(old_vproc, old_vsram, cur_vproc, cur_vsram);
			udelay(delay_us);
			delta[SET_DELAY] = ktime_sub(ktime_get(), now[SET_DELAY]);
			if (ktime_to_us(delta[SET_DELAY]) > ktime_to_us(max[SET_DELAY]))
				max[SET_DELAY] = delta[SET_DELAY];
			cpufreq_ver
			    ("@%s(): DOWN --> old_vsram=%d, cur_vsram=%d, old_vproc=%d, cur_vproc=%d, delay=%d\n",
			     __func__, old_vsram, cur_vsram, old_vproc, cur_vproc, delay_us);
		} while (cur_vproc > volt);
	}

	delta[SET_VOLT] = ktime_sub(ktime_get(), now[SET_VOLT]);
	if (ktime_to_us(delta[SET_VOLT]) > ktime_to_us(max[SET_VOLT]))
		max[SET_VOLT] = delta[SET_VOLT];

	notify_cpu_volt_sampler(p, volt);

	cpufreq_ver("@%s():%d, cur_vsram = %d, cur_vproc = %d\n", __func__, __LINE__, cur_vsram,
		    cur_vproc);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return ret;
}
#else
static int set_cur_volt_extbuck(struct mt_cpu_dvfs *p, unsigned int volt)
{				/* volt: vproc (mv*100) */
	unsigned int cur_vsram;
	unsigned int cur_vproc;
	unsigned int delay_us = 0;
	int ret = 0;

	enum mt_cpu_dvfs_id id_sec;
	struct mt_cpu_dvfs *p_second;

	FUNC_ENTER(FUNC_LV_LOCAL);

#if FORGE_FREQ_ONLY
	/* In the frequency-only build both write targets are wrong or absent
	 * (the VPROC setter is a weak no-op, the VSRAM helper uses MT6353
	 * addresses on MT6351). Refuse the whole move rather than actuate
	 * partially; reporting success is safe because every reachable OPP
	 * runs at the fixed boot voltage. With voltage DVFS forge_vrail.c
	 * moves both rails. */
	if (forge_full_dvfs) {
		FUNC_EXIT(FUNC_LV_LOCAL);
		return forge_set_rails(volt);
	}
	pr_info_once("[FORGE_CPUFREQ] freq-only build: voltage moves are no-ops (VPROC fixed at PL 1.000 V)\n");
	FUNC_EXIT(FUNC_LV_LOCAL);
	return 0;
#endif

	/* First statement of any real VPROC/VSRAM write. */
	if (forge_opp_guard_check("set_cur_volt_extbuck")) {
		FUNC_EXIT(FUNC_LV_LOCAL);
		return -EPERM;
	}

	aee_record_cpu_volt(p, volt);

	now[SET_VOLT] = ktime_get();

	cur_vsram = get_cur_vsram(p);
	cur_vproc = get_cur_volt_extbuck(p);

	id_sec = (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	p_second = id_to_cpu_dvfs(id_sec);

	if (unlikely
	    (!((cur_vsram >= cur_vproc) && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
		unsigned int i, val, extbuck_chip_id = mt6311_get_chip_id();

		fail_case = 1;
		fail_times++;
		dump_opp_table(p);
		cpufreq_err("@%s():%d, cur_vsram = %d, cur_vproc = %d, extbuck_chip_id = 0x%x\n",
			    __func__, __LINE__, cur_vsram, cur_vproc, extbuck_chip_id);

		/* read extbuck chip id to verify I2C is still worked or not */
		for (i = 0; i < 10; i++) {
			val = ((mt6311_get_cid() << 8) | (mt6311_get_swcid()));
			cpufreq_err("read chip id from I2C, id: 0x%x\n", val);
		}

		/* read pmic wrap chip id */
		for (i = 0; i < 10; i++) {
			pwrap_read(0x200, &val);
			cpufreq_err("pmic wrap CID = %x\n", val);
		}

		cpufreq_info("@%s():%d, cur_vsram = %d, cur_vproc = %d, idx = (%d, %d), volt = (%d, %d)\n",
					__func__, __LINE__, cur_vsram, cur_vproc, p->idx_opp_tbl, p_second->idx_opp_tbl,
					debug_vsram, debug_vproc);

		/* Try to recover voltage*/
		cur_vsram = cur_vproc + NORMAL_DIFF_VRSAM_VPROC;
		pmic_set_register_value(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON,
						VOLT_TO_PMIC_VAL(cur_vsram));

		cur_vproc = get_cur_volt_extbuck(p);
		cur_vsram = get_cur_vsram(p);

		cpufreq_info("@%s():%d, dvfs voltage = (%d, %d), recovered cur_vsram = %d, cur_vproc = %d\n",
				   __func__, __LINE__, last_vsram, last_vproc, cur_vsram, cur_vproc);
	}

	/* UP */
	if (volt > cur_vproc) {
		unsigned int target_vsram = volt + NORMAL_DIFF_VRSAM_VPROC;
		unsigned int next_vsram;

		do {
			unsigned int old_vproc = cur_vproc;
			unsigned int old_vsram = cur_vsram;

			next_vsram = MIN(((MAX_DIFF_VSRAM_VPROC - 2500) + cur_vproc), target_vsram);

			/* update vsram */
			cur_vsram = MAX(next_vsram, MIN_VSRAM_VOLT);

			if (cur_vsram > MAX_VSRAM_VOLT) {
				cur_vsram = MAX_VSRAM_VOLT;
				target_vsram = MAX_VSRAM_VOLT;	/* to end the loop */
			}

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}
			now[SET_VSRAM] = ktime_get();
			last_vsram = cur_vsram;
			pmic_set_register_value(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON,
						VOLT_TO_PMIC_VAL(cur_vsram));
			delta[SET_VSRAM] = ktime_sub(ktime_get(), now[SET_VSRAM]);
			if (ktime_to_us(delta[SET_VSRAM]) > ktime_to_us(max[SET_VSRAM]))
				max[SET_VSRAM] = delta[SET_VSRAM];

			/* update vproc */
			if (next_vsram > MAX_VSRAM_VOLT)
				cur_vproc = volt;	/* Vsram was limited, set to target vproc directly */
			else
				cur_vproc = next_vsram - NORMAL_DIFF_VRSAM_VPROC;

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 2;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}

			last_vproc = cur_vproc;
			now[SET_VPROC] = ktime_get();
			mt6311_set_vdvfs11_vosel_on(VOLT_TO_EXTBUCK_VAL(cur_vproc));
			delta[SET_VPROC] = ktime_sub(ktime_get(), now[SET_VPROC]);
			if (ktime_to_us(delta[SET_VPROC]) > ktime_to_us(max[SET_VPROC]))
				max[SET_VPROC] = delta[SET_VPROC];

			now[SET_DELAY] = ktime_get();
			delay_us =
			    _calc_pmic_settle_time(old_vproc, old_vsram, cur_vproc, cur_vsram);
			udelay(delay_us);
			delta[SET_DELAY] = ktime_sub(ktime_get(), now[SET_DELAY]);
			if (ktime_to_us(delta[SET_DELAY]) > ktime_to_us(max[SET_DELAY]))
				max[SET_DELAY] = delta[SET_DELAY];
			cpufreq_ver
			    ("@%s(): UP --> old_vsram=%d, cur_vsram=%d, old_vproc=%d, cur_vproc=%d, delay=%d\n",
			     __func__, old_vsram, cur_vsram, old_vproc, cur_vproc, delay_us);
		} while (target_vsram > cur_vsram);
	}
	/* DOWN */
	else if (volt < cur_vproc) {
		unsigned int next_vproc;
		unsigned int next_vsram = cur_vproc + NORMAL_DIFF_VRSAM_VPROC;

		do {
			unsigned int old_vproc = cur_vproc;
			unsigned int old_vsram = cur_vsram;

			next_vproc = MAX((next_vsram - (MAX_DIFF_VSRAM_VPROC - 2500)), volt);

			/* update vproc */
			cur_vproc = next_vproc;

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 3;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}

			last_vproc = cur_vproc;
			now[SET_VPROC] = ktime_get();
			mt6311_set_vdvfs11_vosel_on(VOLT_TO_EXTBUCK_VAL(cur_vproc));
			delta[SET_VPROC] = ktime_sub(ktime_get(), now[SET_VPROC]);
			if (ktime_to_us(delta[SET_VPROC]) > ktime_to_us(max[SET_VPROC]))
				max[SET_VPROC] = delta[SET_VPROC];

			/* update vsram */
			next_vsram = cur_vproc + NORMAL_DIFF_VRSAM_VPROC;
			cur_vsram = MAX(next_vsram, MIN_VSRAM_VOLT);
			cur_vsram = MIN(cur_vsram, MAX_VSRAM_VOLT);

			if (unlikely
			    (!((cur_vsram >= cur_vproc)
			       && (MAX_DIFF_VSRAM_VPROC >= (cur_vsram - cur_vproc))))) {
				fail_case = 4;
				fail_times++;
				dump_opp_table(p);
				cpufreq_err("@%s():%d, old_vsram=%d, old_vproc=%d, cur_vsram = %d, cur_vproc = %d\n",
					__func__, __LINE__, old_vsram, old_vproc, cur_vsram, cur_vproc);
				BUG();
			}
			now[SET_VSRAM] = ktime_get();
			last_vsram = cur_vsram;
			pmic_set_register_value(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON,
						VOLT_TO_PMIC_VAL(cur_vsram));
			delta[SET_VSRAM] = ktime_sub(ktime_get(), now[SET_VSRAM]);
			if (ktime_to_us(delta[SET_VSRAM]) > ktime_to_us(max[SET_VSRAM]))
				max[SET_VSRAM] = delta[SET_VSRAM];

			now[SET_DELAY] = ktime_get();
			delay_us =
			    _calc_pmic_settle_time(old_vproc, old_vsram, cur_vproc, cur_vsram);
			udelay(delay_us);
			delta[SET_DELAY] = ktime_sub(ktime_get(), now[SET_DELAY]);
			if (ktime_to_us(delta[SET_DELAY]) > ktime_to_us(max[SET_DELAY]))
				max[SET_DELAY] = delta[SET_DELAY];
			cpufreq_ver
			    ("@%s(): DOWN --> old_vsram=%d, cur_vsram=%d, old_vproc=%d, cur_vproc=%d, delay=%d\n",
			     __func__, old_vsram, cur_vsram, old_vproc, cur_vproc, delay_us);
		} while (cur_vproc > volt);
	}

	delta[SET_VOLT] = ktime_sub(ktime_get(), now[SET_VOLT]);
	if (ktime_to_us(delta[SET_VOLT]) > ktime_to_us(max[SET_VOLT]))
		max[SET_VOLT] = delta[SET_VOLT];

	notify_cpu_volt_sampler(p, volt);

	cpufreq_ver("@%s():%d, cur_vsram = %d, cur_vproc = %d\n", __func__, __LINE__, cur_vsram,
		    cur_vproc);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return ret;
}
#endif

#ifdef CONFIG_HYBRID_CPU_DVFS
static int set_cur_volt_hybrid(struct mt_cpu_dvfs *p, unsigned int volt)
{
	BUG_ON(!enable_cpuhvfs);

	return 0;
}

static unsigned int get_cur_volt_hybrid(struct mt_cpu_dvfs *p)
{
	unsigned int volt, volt_val;

	BUG_ON(!enable_cpuhvfs);

	volt_val = cpuhvfs_get_curr_volt(cpu_dvfs_to_cluster(p));
	if (volt_val != UINT_MAX)
		volt = EXTBUCK_VAL_TO_VOLT(volt_val);
	else
		volt = get_cur_volt_extbuck(p);

	return volt;
}
#endif

/* cpufreq set (freq & volt) */
static unsigned int _search_available_volt(struct mt_cpu_dvfs *p, unsigned int target_khz)
{
	int i;

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);

	/* search available voltage */
	for (i = p->nr_opp_tbl - 1; i >= 0; i--) {
		if (target_khz <= cpu_dvfs_get_freq_by_idx(p, i))
			break;
	}

	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	if (i < 0)
		i = 0;
	#else
	BUG_ON(i < 0);		/* i.e. target_khz > p->opp_tbl[0].cpufreq_khz */
	#endif

	FUNC_EXIT(FUNC_LV_HELP);

	return cpu_dvfs_get_volt_by_idx(p, i);	/* mv * 100 */
}

static int _cpufreq_set_locked(struct mt_cpu_dvfs *p, unsigned int cur_khz, unsigned int target_khz,
			       struct cpufreq_policy *policy, unsigned int target_volt, int log)
{
	int ret = -1;

	enum mt_cpu_dvfs_id id_second =
	    (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	struct mt_cpu_dvfs *p_second = id_to_cpu_dvfs(id_second);
	unsigned int cur_volt = UINT_MAX;

#ifdef CONFIG_CPU_FREQ
	struct cpufreq_freqs freqs;
	unsigned int target_khz_orig = target_khz;
#endif
	FUNC_ENTER(FUNC_LV_HELP);

	if (cur_khz != get_turbo_freq(p->cpu_id, target_khz)) {
		if (log)
			cpufreq_dbg("@%s(), %s: (%d, %d): freq = %d (%d), volt = %d (%d), cpus = %d, cur = %d\n",
				__func__, cpu_dvfs_get_name(p), p->idx_opp_ppm_base, p->idx_opp_ppm_limit,
				target_khz, get_turbo_freq(p->cpu_id, target_khz), target_volt,
				get_turbo_volt(p->cpu_id, target_volt), num_online_cpus(), cur_khz);
	}

	target_volt = get_turbo_volt(p->cpu_id, target_volt);
	target_khz = get_turbo_freq(p->cpu_id, target_khz);

#if FORGE_FREQ_ONLY
	/* Last line of defence on every path that can move a PLL: a row that
	 * needs more than the current voltage limit must never be entered
	 * (brown-out). The clamped cpufreq table should make this
	 * unreachable; refuse loudly here. */
	if (target_volt > forge_vmax()) {
		cpufreq_err("[FORGE_CPUFREQ] REFUSED %s -> %u kHz: needs %u (mv*100) > %u (%s)\n",
			    cpu_dvfs_get_name(p), target_khz, target_volt,
			    forge_vmax(), forge_full_dvfs ? "stock max" : "freq-only build");
		ret = -EINVAL;
		goto out;
	}
#endif

	if (cur_khz == target_khz)
		goto out;

#ifdef CONFIG_HYBRID_CPU_DVFS
	if (!enable_cpuhvfs) {
#endif
		cur_volt = p->ops->get_cur_volt(p);
		/* set volt (UP); after a rail fault a frequency-lowering move
		 * goes ahead without it (lower is never worse, and the fault
		 * cap must be able to bring both clusters down) */
		if (target_volt > cur_volt &&
		    !(forge_dvfs_fault && target_khz < cur_khz)) {
			ret = p->ops->set_cur_volt(p, target_volt);

			if (ret)	/* set volt fail */
				goto out;
		}
#ifdef CONFIG_HYBRID_CPU_DVFS
	}
#endif

#ifdef CONFIG_CPU_FREQ
	freqs.old = cur_khz;
	freqs.new = target_khz_orig;
	if (policy) {
		freqs.cpu = policy->cpu;
		cpufreq_freq_transition_begin(policy, &freqs);
	}
#endif

	/* set freq (UP/DOWN) */
	if (cur_khz != target_khz)
		p->ops->set_cur_freq(p, cur_khz, target_khz);

#ifdef CONFIG_CPU_FREQ
	if (policy) {
		freqs.cpu = policy->cpu;
		cpufreq_freq_transition_end(policy, &freqs, 0);
	}
#endif

#ifdef CONFIG_HYBRID_CPU_DVFS
	if (!enable_cpuhvfs) {
#endif
		/* set volt (DOWN) */
		if (cur_volt > target_volt) {
			ret = p->ops->set_cur_volt(p, target_volt);

			if (ret)	/* set volt fail */
				goto out;
		}
#ifdef CONFIG_HYBRID_CPU_DVFS
	}
#endif

	cpufreq_ver("@%s(): Vproc = %dmv, Vsram = %dmv, freq(%s) = %dKHz\n",
		    __func__,
		    (p->ops->get_cur_volt(p)) / 100,
		    (get_cur_vsram(p) / 100), p->name, p->ops->get_cur_phy_freq(p));
	cpufreq_ver("@%s(): freq(%s) = %dKHz, freq(%s) = %dKHz, freq(cci) = %dKHz\n",
			__func__, p->name, p->ops->get_cur_phy_freq(p), p_second->name
			, p_second->ops->get_cur_phy_freq(p_second), get_cur_phy_freq_cci());

	/* trigger exception if freq/volt not correct during stress */
	if (do_dvfs_stress_test) {
		unsigned int volt = p->ops->get_cur_volt(p);
		unsigned int freq = p->ops->get_cur_phy_freq(p);

		if (volt < target_volt || freq != target_khz) {
			cpufreq_err("volt = %u, target_volt = %u, freq = %u, target_khz = %u\n",
				    volt, target_volt, freq, target_khz);
			dump_opp_table(p);
			BUG();
		}
	}

	FUNC_EXIT(FUNC_LV_HELP);

	return 0;

out:
	return ret;
}

static unsigned int _calc_new_opp_idx(struct mt_cpu_dvfs *p, int new_opp_idx);

static unsigned int calc_new_target_volt(struct mt_cpu_dvfs *p, struct mt_cpu_dvfs *p_second, int new_opp_idx)
{
	int freq_second;
	unsigned int target_voltage;

	/* freq_second = p_second->policy_online ? p_second->ops->get_cur_phy_freq(p_second) : 0; */
	freq_second = p_second->ops->get_cur_phy_freq(p_second);
	target_voltage =
	    MAX(cpu_dvfs_get_volt_by_idx(p, new_opp_idx),
		_search_available_volt(p_second, freq_second));

	return target_voltage;
}

static void _mt_cpufreq_set(struct cpufreq_policy *policy, enum mt_cpu_dvfs_id id, int new_opp_idx)
{
	unsigned long flags;
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
	unsigned int cur_freq;
	unsigned int target_freq;
	int ret = 0;

	enum mt_cpu_dvfs_id id_second =
	    (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) ? MT_CPU_DVFS_BIG : MT_CPU_DVFS_LITTLE;
	struct mt_cpu_dvfs *p_second = id_to_cpu_dvfs(id_second);
	unsigned int target_volt;	/* mv * 100 */
	int log = 1;

	FUNC_ENTER(FUNC_LV_LOCAL);

#ifndef __TRIAL_RUN__
	if (!policy)
#endif
		return;

	BUG_ON(NULL == p);
	BUG_ON(new_opp_idx >= p->nr_opp_tbl);

	cpufreq_lock(flags);

	now[SET_DVFS] = ktime_get();

	if (p->armpll_is_available != 1) {
		cpufreq_unlock(flags);
		return;
	}

	/* get current idx here to avoid idx synchronization issue */
	if (new_opp_idx == -1) {
		new_opp_idx = p->idx_opp_tbl;
		log = 0;
	}

	if (do_dvfs_stress_test)
		new_opp_idx = jiffies & 0x7;	/* 0~7 */
	else {
#if defined(CONFIG_CPU_DVFS_BRINGUP)
		new_opp_idx = id_to_cpu_dvfs(id)->idx_normal_max_opp;
#else
		new_opp_idx = _calc_new_opp_idx(id_to_cpu_dvfs(id), new_opp_idx);
#endif
	}

	target_volt = calc_new_target_volt(p, p_second, new_opp_idx);

	cur_freq = p->ops->get_cur_phy_freq(p);
	target_freq = cpu_dvfs_get_freq_by_idx(p, new_opp_idx);

	if (abs(new_opp_idx - p->idx_opp_tbl) < 3 && new_opp_idx != 0 &&
		new_opp_idx != p->nr_opp_tbl - 1)
		log = 0;

	forge_freq_refused = false;
#ifdef CONFIG_CPU_FREQ
	ret = _cpufreq_set_locked(p, cur_freq, target_freq, policy, target_volt, log);
#else
	ret = _cpufreq_set_locked(p, cur_freq, target_freq, NULL, target_volt, log);
#endif

	/* a failed or refused move leaves the index on the clock */
	if (ret || forge_freq_refused)
		_sync_opp_tbl_idx(p);
	else
		p->idx_opp_tbl = new_opp_idx;

#ifndef DISABLE_PBM_FEATURE
	if (!ret && !p->dvfs_disable_by_suspend)
		_kick_PBM_by_cpu(p);
#endif
	delta[SET_DVFS] = ktime_sub(ktime_get(), now[SET_DVFS]);
	if (ktime_to_us(delta[SET_DVFS]) > ktime_to_us(max[SET_DVFS]))
		max[SET_DVFS] = delta[SET_DVFS];

	cpufreq_unlock(flags);

	FUNC_EXIT(FUNC_LV_LOCAL);
}

static int __maybe_unused _search_available_freq_idx_under_v(struct mt_cpu_dvfs *p, unsigned int volt)
{
	int i;

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);

	/* search available voltage */
	for (i = p->nr_opp_tbl - 1; i >= 0; i--) {
		if (volt <= cpu_dvfs_get_volt_by_idx(p, i))
			break;
	}

	if (i < 0)
		return 0; /* i.e. target_khz > p->opp_tbl[0].cpufreq_khz */

	FUNC_EXIT(FUNC_LV_HELP);

	return i;
}
#define SINGLE_CORE_BOUNDARY_CPU_NUM 1
static unsigned int num_online_cpus_delta;
/* a cluster coming back picks the highest row
 * under the present voltage - never above the ceiling or the policy
 * max (cpufreq lock held, as at the stock call sites) */
static int forge_hotplug_cap(struct mt_cpu_dvfs *p, int idx)
{
	enum mt_cpu_dvfs_id id = cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) ?
				 MT_CPU_DVFS_LITTLE : MT_CPU_DVFS_BIG;
	struct cpufreq_policy *policy = cpufreq_cpu_get(p->cpu_id);
	unsigned int cap = READ_ONCE(forge_cpu_max_khz[id]);

	if (policy) {
		cap = min(cap, policy->max);
		cpufreq_cpu_put(policy);
	}
	return MAX(idx, search_table_idx_by_freq(p, forge_row_at_or_below(p, cap, UINT_MAX)));
}

static int _mt_cpufreq_cpu_CB(struct notifier_block *nfb, unsigned long action,
					void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	unsigned int online_cpus = num_online_cpus();
	struct device *dev;

	enum mt_cpu_dvfs_id cluster_id;
	struct mt_cpu_dvfs *p;
	unsigned int cur_volt;
	int freq_idx;

	/* CPU mask - Get on-line cpus per-cluster */
	struct cpumask dvfs_cpumask;
	struct cpumask cpu_online_cpumask;
	unsigned int cpus;
	unsigned long flags;

#ifdef CONFIG_CPU_FREQ
	struct cpufreq_policy *policy;
	struct cpufreq_freqs freqs;
#endif

	cluster_id = arch_get_cluster_id(cpu);
	arch_get_cluster_cpus(&dvfs_cpumask, cluster_id);
	cpumask_and(&cpu_online_cpumask, &dvfs_cpumask, cpu_online_mask);
	p = id_to_cpu_dvfs(cluster_id);
	if (!p)
		return NOTIFY_OK;

	cpufreq_ver("@%s():%d, cpu = %d, action = %lu, oppidx = %d, num_online_cpus = %d\n"
	, __func__, __LINE__, cpu, action, p->idx_opp_tbl, online_cpus);

	dev = get_cpu_device(cpu);
	/* Pull up frequency when single core */
	if (dev) {
		if ((SINGLE_CORE_BOUNDARY_CPU_NUM == online_cpus) && cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) {
			switch (action) {
			case CPU_UP_PREPARE:
			case CPU_UP_PREPARE_FROZEN:
				num_online_cpus_delta = 1;

			case CPU_DEAD:
			case CPU_DEAD_FROZEN:
				/* _mt_cpufreq_set(MT_CPU_DVFS_LITTLE, -1); */
				break;
			}
		} else {
			switch (action) {
			case CPU_ONLINE:
			case CPU_ONLINE_FROZEN:
			case CPU_UP_CANCELED:
			case CPU_UP_CANCELED_FROZEN:
				num_online_cpus_delta = 0;
				break;
			}
		}
	}

	if (dev) {
		switch (action & 0xF) {
		case CPU_ONLINE:
			cpus = cpumask_weight(&cpu_online_cpumask);
			cpufreq_ver("CPU_ONLINE -> cpus = %d\n", cpus);
			if (cpus == 1) {
				cpufreq_ver("CPU_ONLINE first CPU of %s\n",
					cpu_dvfs_get_name(p));
				cpufreq_lock(flags);
				p->armpll_is_available = 1;
				if (action == CPU_ONLINE) {
					cur_volt = p->ops->get_cur_volt(p);
					freq_idx = _search_available_freq_idx_under_v(p, cur_volt);
					freq_idx = MAX(freq_idx, _calc_new_opp_idx(p, freq_idx));
					freq_idx = forge_hotplug_cap(p, freq_idx);
#ifdef CONFIG_CPU_FREQ
					policy = cpufreq_cpu_get(cpu);
					freqs.old = cpu_dvfs_get_cur_freq(p);
					freqs.new = cpu_dvfs_get_freq_by_idx(p, freq_idx);
					if (policy) {
						freqs.cpu = policy->cpu;
						cpufreq_freq_transition_begin(policy, &freqs);
					}
#endif
					forge_freq_refused = false;
					p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p)
						, cpu_dvfs_get_freq_by_idx(p, freq_idx));
#ifdef CONFIG_CPU_FREQ
					if (policy) {
						freqs.cpu = policy->cpu;
						cpufreq_freq_transition_end(policy, &freqs, 0);
						cpufreq_cpu_put(policy);
					}
#endif
					/* a refused row leaves the index on the clock */
					if (forge_freq_refused)
						_sync_opp_tbl_idx(p);
					else
						p->idx_opp_tbl = freq_idx;

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
					if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
						aee_rr_rec_cpu_dvfs_oppidx(
							(aee_rr_curr_cpu_dvfs_oppidx() & 0xF0) | p->idx_opp_tbl);
					else
						aee_rr_rec_cpu_dvfs_oppidx(
							(aee_rr_curr_cpu_dvfs_oppidx() & 0x0F) | (p->idx_opp_tbl << 4));
#endif
				}
				cpufreq_unlock(flags);
			}
			break;
		case CPU_DOWN_PREPARE:
			cpus = cpumask_weight(&cpu_online_cpumask);
			cpufreq_ver("CPU_DOWN_PREPARE -> cpus = %d\n", cpus);
			if (cpus == 1) {
				cpufreq_ver("CPU_DOWN_PREPARE last CPU of %s\n",
					cpu_dvfs_get_name(p));
				cpufreq_lock(flags);
				p->armpll_is_available = 0;
#ifdef CONFIG_HYBRID_CPU_DVFS
				if (enable_cpuhvfs)
					cpuhvfs_notify_cluster_off(cpu_dvfs_to_cluster(p));
				else
#endif					/* take care the above code segment */
					p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p), cpu_dvfs_get_min_freq(p));

				p->idx_opp_tbl = p->nr_opp_tbl - 1;

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
				if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
					aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0xF0) |
						p->idx_opp_tbl);
				else
					aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0x0F) |
						(p->idx_opp_tbl << 4));
#endif
				cpufreq_unlock(flags);
			}
			break;
		case CPU_DOWN_FAILED:
			cpus = cpumask_weight(&cpu_online_cpumask);
			cpufreq_ver("CPU_DOWN_FAILED -> cpus = %d\n", cpus);
			if (cpus == 1) {
				cpufreq_ver("CPU_DOWN_FAILED first CPU of %s\n",
					cpu_dvfs_get_name(p));
			cpufreq_lock(flags);
			p->armpll_is_available = 1;
#ifdef CONFIG_HYBRID_CPU_DVFS
			if (enable_cpuhvfs)
				cpuhvfs_notify_cluster_on(cpu_dvfs_to_cluster(p));
#endif
			if (action == CPU_DOWN_FAILED) {
				cur_volt = p->ops->get_cur_volt(p);
				freq_idx = _search_available_freq_idx_under_v(p, cur_volt);
				freq_idx = MAX(freq_idx, _calc_new_opp_idx(p, freq_idx));
				freq_idx = forge_hotplug_cap(p, freq_idx);

#ifdef CONFIG_CPU_FREQ
				policy = cpufreq_cpu_get(cpu);
				freqs.old = cpu_dvfs_get_cur_freq(p);
				freqs.new = cpu_dvfs_get_freq_by_idx(p, freq_idx);
				if (policy) {
					freqs.cpu = policy->cpu;
					cpufreq_freq_transition_begin(policy, &freqs);
				}
#endif
				forge_freq_refused = false;
				p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p)
					, cpu_dvfs_get_freq_by_idx(p, freq_idx));

#ifdef CONFIG_CPU_FREQ
				if (policy) {
					freqs.cpu = policy->cpu;
					cpufreq_freq_transition_end(policy, &freqs, 0);
					cpufreq_cpu_put(policy);
				}
#endif
				/* a refused row leaves the index on the clock */
				if (forge_freq_refused)
					_sync_opp_tbl_idx(p);
				else
					p->idx_opp_tbl = freq_idx;

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
				if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
					aee_rr_rec_cpu_dvfs_oppidx(
						(aee_rr_curr_cpu_dvfs_oppidx() & 0xF0) | p->idx_opp_tbl);
				else
					aee_rr_rec_cpu_dvfs_oppidx(
						(aee_rr_curr_cpu_dvfs_oppidx() & 0x0F) | (p->idx_opp_tbl << 4));
#endif
			}
			cpufreq_unlock(flags);
			}
			break;
		}
#ifndef DISABLE_PBM_FEATURE
		/* Notify PBM after CPU on/off */
		if (action == CPU_ONLINE || action == CPU_ONLINE_FROZEN
			|| action == CPU_DEAD || action == CPU_DEAD_FROZEN) {

			cpufreq_lock(flags);

			if (!p->dvfs_disable_by_suspend)
				_kick_PBM_by_cpu(p);

			cpufreq_unlock(flags);
		}
#endif
	}

	cpufreq_ver("@%s():%d, num_online_cpus_delta = %d\n", __func__, __LINE__,
		    num_online_cpus_delta);

	cpufreq_ver("@%s():%d, cpu = %d, action = %lu, oppidx = %d, num_online_cpus = %d\n"
	, __func__, __LINE__, cpu, action, p->idx_opp_tbl, online_cpus);

	return NOTIFY_OK;
}

static int _sync_opp_tbl_idx(struct mt_cpu_dvfs *p)
{
	int ret = -1;
	unsigned int freq;
	int i;

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);
	BUG_ON(NULL == p->opp_tbl);
	BUG_ON(NULL == p->ops);

	freq = p->ops->get_cur_phy_freq(p);

	for (i = p->nr_opp_tbl - 1; i >= 0; i--) {
		if (freq <= cpu_dvfs_get_freq_by_idx(p, i)) {
			p->idx_opp_tbl = i;
			break;
		}
	}

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0xF0) | p->idx_opp_tbl);
	else
		aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0x0F) | (p->idx_opp_tbl << 4));
#endif

	if (i >= 0) {
		cpufreq_ver("%s freq = %d\n", cpu_dvfs_get_name(p), cpu_dvfs_get_cur_freq(p));
		ret = 0;
	} else
		cpufreq_warn("%s can't find freq = %d\n", cpu_dvfs_get_name(p), freq);

	FUNC_EXIT(FUNC_LV_HELP);

	return ret;
}

static int _mt_cpufreq_sync_opp_tbl_idx(struct mt_cpu_dvfs *p)
{
	int ret = -1;

	FUNC_ENTER(FUNC_LV_LOCAL);


	if (cpu_dvfs_is_available(p))
		ret = _sync_opp_tbl_idx(p);


	cpufreq_info("%s freq = %d\n", cpu_dvfs_get_name(p), cpu_dvfs_get_cur_freq(p));

	FUNC_EXIT(FUNC_LV_LOCAL);

	return ret;
}

/* Policy ceiling that respects the frequency-only clamp: plain
 * cpu_dvfs_get_max_freq() reads opp_tbl[0], a row the governor must never
 * reach on the clamped table. */
static unsigned int forge_max_valid_freq(struct mt_cpu_dvfs *p)
{
#if FORGE_FREQ_ONLY
	int i;

	for (i = 0; i < p->nr_opp_tbl; i++)
		if (cpu_dvfs_get_volt_by_idx(p, i) <= forge_vmax())
			return cpu_dvfs_get_freq_by_idx(p, i);
#endif
	return cpu_dvfs_get_max_freq(p);
}

struct cpumask cpumask_big;
struct cpumask cpumask_little;

static enum mt_cpu_dvfs_id _get_cpu_dvfs_id(unsigned int cpu_id)
{
	int cluster_id;

	cluster_id = arch_get_cluster_id(cpu_id);

	if (cluster_id == 0)
		return MT_CPU_DVFS_LITTLE;
	else
		return MT_CPU_DVFS_BIG;
}

static int _mt_cpufreq_setup_freqs_table(struct cpufreq_policy *policy,
					 struct mt_cpu_freq_info *freqs, int num)
{
	struct mt_cpu_dvfs *p;
	int ret = 0;

	FUNC_ENTER(FUNC_LV_LOCAL);

	BUG_ON(NULL == policy);
	BUG_ON(NULL == freqs);

	p = id_to_cpu_dvfs(_get_cpu_dvfs_id(policy->cpu));

#ifdef CONFIG_CPU_FREQ
	ret = cpufreq_frequency_table_cpuinfo(policy, p->freq_tbl_for_cpufreq);

	if (!ret)
		policy->freq_table = p->freq_tbl_for_cpufreq;

#endif
	cpumask_copy(policy->cpus, topology_core_cpumask(policy->cpu));
	cpumask_copy(policy->related_cpus, policy->cpus);

	FUNC_EXIT(FUNC_LV_LOCAL);

	return 0;
}

#define SIGLE_CORE_IDX 4
static unsigned int _calc_new_opp_idx(struct mt_cpu_dvfs *p, int new_opp_idx)
{
	unsigned int online_cpus;

	FUNC_ENTER(FUNC_LV_HELP);

	BUG_ON(NULL == p);

	cpufreq_ver("new_opp_idx = %d, idx_opp_ppm_base = %d, idx_opp_ppm_limit = %d\n",
		new_opp_idx , p->idx_opp_ppm_base, p->idx_opp_ppm_limit);

	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE)) {
		online_cpus = num_online_cpus() + num_online_cpus_delta;

		if (online_cpus == 1 && (new_opp_idx > SIGLE_CORE_IDX))
			new_opp_idx = SIGLE_CORE_IDX;
	}

	if ((p->idx_opp_ppm_limit != -1) && (new_opp_idx < p->idx_opp_ppm_limit))
		new_opp_idx = p->idx_opp_ppm_limit;

	if ((p->idx_opp_ppm_base != -1) && (new_opp_idx > p->idx_opp_ppm_base))
		new_opp_idx = p->idx_opp_ppm_base;

	if ((p->idx_opp_ppm_base == p->idx_opp_ppm_limit) && p->idx_opp_ppm_base != -1)
		new_opp_idx = p->idx_opp_ppm_base;

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	if (cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE))
		aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0xF0) | new_opp_idx);
	else
		aee_rr_rec_cpu_dvfs_oppidx((aee_rr_curr_cpu_dvfs_oppidx() & 0x0F) | (new_opp_idx << 4));
#endif
	FUNC_EXIT(FUNC_LV_HELP);

	return new_opp_idx;
}

static void ppm_limit_callback(struct ppm_client_req req)
{
	struct ppm_client_req *ppm = (struct ppm_client_req *)&req;
	unsigned long flags;

	struct mt_cpu_dvfs *p;
	int ignore_ppm[NR_MT_CPU_DVFS] = {0};
	unsigned int i;
	struct cpufreq_policy *policy[NR_MT_CPU_DVFS];
	struct cpumask dvfs_cpumask[NR_MT_CPU_DVFS];
	struct cpumask cpu_online_cpumask;
	char str1[32];
	char str2[32];
	unsigned int ret;

	for (i = 0; i < NR_MT_CPU_DVFS; i++)
		arch_get_cluster_cpus(&dvfs_cpumask[i], i);

	cpufreq_ver("get feedback from PPM module\n");
	cpufreq_ver("cluster_id = %d, cpu_id = %d, min_cpufreq_idx = %d, max_cpufreq_idx = %d\n",
		ppm->cpu_limit[0].cluster_id, ppm->cpu_limit[0].cpu_id,
		ppm->cpu_limit[0].min_cpufreq_idx, ppm->cpu_limit[0].max_cpufreq_idx);
	cpufreq_ver("has_advise_freq = %d, advise_cpufreq_idx = %d\n",
		ppm->cpu_limit[0].has_advise_freq, ppm->cpu_limit[0].advise_cpufreq_idx);
	cpufreq_ver("cluster_id = %d, cpu_id = %d, min_cpufreq_idx = %d, max_cpufreq_idx = %d\n",
		ppm->cpu_limit[1].cluster_id, ppm->cpu_limit[1].cpu_id, ppm->cpu_limit[1].min_cpufreq_idx,
			ppm->cpu_limit[1].max_cpufreq_idx);
	cpufreq_ver("has_advise_freq = %d, advise_cpufreq_idx = %d\n",
		ppm->cpu_limit[1].has_advise_freq, ppm->cpu_limit[1].advise_cpufreq_idx);

	cpufreq_lock(flags);
	for (i = 0; i < ppm->cluster_num; i++) {
		if (i == 0)
			p = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
		else
			p = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);

		/* make sure the idx is synced */
		/* _sync_opp_tbl_idx(p); */

		ignore_ppm[i] = 0;
		if (ppm->cpu_limit[i].has_advise_freq) {
			p->idx_opp_ppm_base = ppm->cpu_limit[i].advise_cpufreq_idx;
			p->idx_opp_ppm_limit = ppm->cpu_limit[i].advise_cpufreq_idx;
			if (p->idx_opp_tbl == ppm->cpu_limit[i].advise_cpufreq_idx) {
					cpufreq_ver("idx = %d, advise_cpufreq_idx = %d\n", p->idx_opp_tbl ,
						ppm->cpu_limit[i].advise_cpufreq_idx);
					ignore_ppm[i] = 1;
			}
		} else {
			p->idx_opp_ppm_base = ppm->cpu_limit[i].min_cpufreq_idx;	/* ppm update base */
			p->idx_opp_ppm_limit = ppm->cpu_limit[i].max_cpufreq_idx;	/* ppm update limit */
			if ((p->idx_opp_tbl >= ppm->cpu_limit[i].max_cpufreq_idx)
				&& (p->idx_opp_tbl <= ppm->cpu_limit[i].min_cpufreq_idx)) {
				cpufreq_ver("idx = %d, idx_opp_ppm_base = %d, idx_opp_ppm_limit = %d\n",
					p->idx_opp_tbl,	p->idx_opp_ppm_base, p->idx_opp_ppm_limit);
				ignore_ppm[i] = 1;
			}
		}
	}
	cpufreq_unlock(flags);

	get_online_cpus();
	cpumask_and(&cpu_online_cpumask, &dvfs_cpumask[MT_CPU_DVFS_LITTLE], cpu_online_mask);
	scnprintf(str1, sizeof(str1), "%*pbl", cpumask_pr_args(cpu_online_mask));
	scnprintf(str2, sizeof(str2), "%*pbl", cpumask_pr_args(&cpu_online_cpumask));
	cpufreq_ver("cpu_online_mask = %s, cpu_online_cpumask for little = %s\n", str1, str2);
	ret = -1;
	for_each_cpu(i, &cpu_online_cpumask) {
		policy[MT_CPU_DVFS_LITTLE] = cpufreq_cpu_get(i);
		if (policy[MT_CPU_DVFS_LITTLE]) {
			p = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
			if (p->idx_opp_ppm_limit == -1)
				policy[MT_CPU_DVFS_LITTLE]->max = cpu_dvfs_get_max_freq(p);
			else
				policy[MT_CPU_DVFS_LITTLE]->max = cpu_dvfs_get_freq_by_idx(p,
					p->idx_opp_ppm_limit);
			if (p->idx_opp_ppm_base == -1)
				policy[MT_CPU_DVFS_LITTLE]->min = cpu_dvfs_get_min_freq(p);
			else
				policy[MT_CPU_DVFS_LITTLE]->min = cpu_dvfs_get_freq_by_idx(p,
					p->idx_opp_ppm_base);
			cpufreq_cpu_put(policy[MT_CPU_DVFS_LITTLE]);
			ret = 0;
			break;
		}
	}
	put_online_cpus();

	if (!ignore_ppm[MT_CPU_DVFS_LITTLE]) {
		if (!ret)
			_mt_cpufreq_set(policy[MT_CPU_DVFS_LITTLE], MT_CPU_DVFS_LITTLE, -1);
		else
			goto second_limit;
	}
second_limit:
	get_online_cpus();
	cpumask_and(&cpu_online_cpumask, &dvfs_cpumask[MT_CPU_DVFS_BIG], cpu_online_mask);
	scnprintf(str1, sizeof(str1), "%*pbl", cpumask_pr_args(cpu_online_mask));
	scnprintf(str2, sizeof(str2), "%*pbl", cpumask_pr_args(&cpu_online_cpumask));
	cpufreq_ver("cpu_online_mask = %s, cpu_online_cpumask for big = %s\n", str1, str2);
	ret = -1;
	for_each_cpu(i, &cpu_online_cpumask) {
		policy[MT_CPU_DVFS_BIG] = cpufreq_cpu_get(i);
		if (policy[MT_CPU_DVFS_BIG]) {
			p = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);
			if (p->idx_opp_ppm_limit == -1)
				policy[MT_CPU_DVFS_BIG]->max = cpu_dvfs_get_max_freq(p);
			else
				policy[MT_CPU_DVFS_BIG]->max = cpu_dvfs_get_freq_by_idx(p,
					p->idx_opp_ppm_limit);
			if (p->idx_opp_ppm_base == -1)
				policy[MT_CPU_DVFS_BIG]->min = cpu_dvfs_get_min_freq(p);
			else
				policy[MT_CPU_DVFS_BIG]->min = cpu_dvfs_get_freq_by_idx(p,
					p->idx_opp_ppm_base);
			cpufreq_cpu_put(policy[MT_CPU_DVFS_BIG]);
			ret = 0;
			break;
		}
	}
	put_online_cpus();

	if (!ignore_ppm[MT_CPU_DVFS_BIG]) {
		if (!ret)
			_mt_cpufreq_set(policy[MT_CPU_DVFS_BIG], MT_CPU_DVFS_BIG, -1);
		else
			goto no_limit;
	}
no_limit:
	return;
}

/*
 * cpufreq driver
 */
static int _mt_cpufreq_verify(struct cpufreq_policy *policy)
{
	struct mt_cpu_dvfs *p;
	int ret = 0;		/* cpufreq_frequency_table_verify() always return 0 */

	FUNC_ENTER(FUNC_LV_MODULE);

	p = id_to_cpu_dvfs(_get_cpu_dvfs_id(policy->cpu));

	BUG_ON(NULL == p);

#ifdef CONFIG_CPU_FREQ
	ret = cpufreq_frequency_table_verify(policy, p->freq_tbl_for_cpufreq);
#endif

	FUNC_EXIT(FUNC_LV_MODULE);

	return ret;
}
#ifdef ICORE_BRANCH
/* static int _mt_cpufreq_target(struct cpufreq_policy *policy, unsigned int index) */
#else
static int _mt_cpufreq_target(struct cpufreq_policy *policy, unsigned int target_freq,
			      unsigned int relation)
#endif
{
	enum mt_cpu_dvfs_id id = _get_cpu_dvfs_id(policy->cpu);
	int ret = 0;
	unsigned int new_opp_idx;

	FUNC_ENTER(FUNC_LV_MODULE);

#ifdef ICORE_BRANCH
	if (policy->cpu >= num_possible_cpus()
		|| (id_to_cpu_dvfs(id) && id_to_cpu_dvfs(id)->dvfs_disable_by_procfs)
	)
		return -EINVAL;
#else
	if (policy->cpu >= num_possible_cpus()
	    || cpufreq_frequency_table_target(policy, id_to_cpu_dvfs(id)->freq_tbl_for_cpufreq,
					      target_freq, relation, &new_opp_idx)
	    || (id_to_cpu_dvfs(id) && id_to_cpu_dvfs(id)->dvfs_disable_by_procfs)
		/* || (id_to_cpu_dvfs(id) && id_to_cpu_dvfs(id)->dvfs_disable_by_suspend) */
	    )
		return -EINVAL;
#endif

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	if (id == MT_CPU_DVFS_LITTLE)
		aee_rr_rec_cpu_dvfs_status(aee_rr_curr_cpu_dvfs_status() |
			(1 << CPU_DVFS_LITTLE_IS_DOING_DVFS));
	else
		aee_rr_rec_cpu_dvfs_status(aee_rr_curr_cpu_dvfs_status() |
			(1 << CPU_DVFS_BIG_IS_DOING_DVFS));
#endif
	/* PPM => idx */
	/* _mt_cpufreq_set(policy, id, index); */
	_mt_cpufreq_set(policy, id, new_opp_idx);

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	if (id == MT_CPU_DVFS_LITTLE)
		aee_rr_rec_cpu_dvfs_status(aee_rr_curr_cpu_dvfs_status() &
			~(1 << CPU_DVFS_LITTLE_IS_DOING_DVFS));
	else
		aee_rr_rec_cpu_dvfs_status(aee_rr_curr_cpu_dvfs_status() &
			~(1 << CPU_DVFS_BIG_IS_DOING_DVFS));
#endif

	FUNC_EXIT(FUNC_LV_MODULE);

	return ret;
}

static int _mt_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret = -EINVAL;
	unsigned long flags;

	FUNC_ENTER(FUNC_LV_MODULE);

	max_cpu_num = num_possible_cpus();

	if (policy->cpu >= max_cpu_num)
		return -EINVAL;

	cpufreq_ver("@%s: max_cpu_num: %d\n", __func__, max_cpu_num);

	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_setall(policy->cpus);

	policy->cpuinfo.transition_latency = 1000;

	{
		enum mt_cpu_dvfs_id id = _get_cpu_dvfs_id(policy->cpu);
		struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
		unsigned int lv = _mt_cpufreq_get_cpu_level();
		struct opp_tbl_info *opp_tbl_info =
		    (MT_CPU_DVFS_BIG ==
		     id) ? &opp_tbls_big[0][CPU_LV_TO_OPP_IDX(lv)] :
		    &opp_tbls_little[0][CPU_LV_TO_OPP_IDX(lv)];

		BUG_ON(NULL == p);
		BUG_ON(!
		       (lv == CPU_LEVEL_0 || lv == CPU_LEVEL_1 || lv == CPU_LEVEL_2
			|| lv == CPU_LEVEL_3));

		p->cpu_level = lv;

		ret = _mt_cpufreq_setup_freqs_table(policy,
						    opp_tbl_info->opp_tbl, opp_tbl_info->size);

		policy->cpuinfo.max_freq = forge_max_valid_freq(id_to_cpu_dvfs(id));
		policy->cpuinfo.min_freq = cpu_dvfs_get_min_freq(id_to_cpu_dvfs(id));

		policy->cur = p->ops->get_cur_phy_freq(p);	/* use cur phy freq is better */
		if (p->idx_opp_ppm_limit == -1)
			policy->max = forge_max_valid_freq(id_to_cpu_dvfs(id));
		else
			policy->max = cpu_dvfs_get_freq_by_idx(id_to_cpu_dvfs(id), p->idx_opp_ppm_limit);

		if (p->idx_opp_ppm_base == -1)
			policy->min = cpu_dvfs_get_min_freq(id_to_cpu_dvfs(id));
		else
			policy->min = cpu_dvfs_get_freq_by_idx(id_to_cpu_dvfs(id), p->idx_opp_ppm_base);

		if (_mt_cpufreq_sync_opp_tbl_idx(p) >= 0)
			if (p->idx_normal_max_opp == -1)
				p->idx_normal_max_opp = p->idx_opp_tbl;

		if (MT_CPU_DVFS_BIG == id) {
			if (lv == CPU_LEVEL_0)
				p->freq_tbl = opp_tbl_method_L_e0;
			else if (lv == CPU_LEVEL_1)
				p->freq_tbl = opp_tbl_method_L_e1;
			else if (lv == CPU_LEVEL_2)
				p->freq_tbl = opp_tbl_method_L_e2;
			else
				p->freq_tbl = opp_tbl_method_L_e3;
			p->armpll_addr = (unsigned int *)ARMCA15PLL_CON1;
			p->armpll_clk_src = TOP_CKMUXSEL_ARMPLL_L;
		} else {
			if (lv == CPU_LEVEL_0)
				p->freq_tbl = opp_tbl_method_LL_e0;
			else if (lv == CPU_LEVEL_1)
				p->freq_tbl = opp_tbl_method_LL_e1;
			else if (lv == CPU_LEVEL_2)
				p->freq_tbl = opp_tbl_method_LL_e2;
			else
				p->freq_tbl = opp_tbl_method_LL_e3;
			p->armpll_addr = (unsigned int *)ARMCA7PLL_CON1;
			p->armpll_clk_src = TOP_CKMUXSEL_ARMPLL_LL;
		}

		cpufreq_lock(flags);
		p->armpll_is_available = 1;
#ifdef CONFIG_HYBRID_CPU_DVFS
		if (enable_cpuhvfs)
			cpuhvfs_notify_cluster_on(cpu_dvfs_to_cluster(p));
#endif
		cpufreq_unlock(flags);
	}

	if (ret)
		cpufreq_err("failed to setup frequency table\n");

	FUNC_EXIT(FUNC_LV_MODULE);

	return ret;
}

static int _mt_cpufreq_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static unsigned int _mt_cpufreq_get(unsigned int cpu)
{
	struct mt_cpu_dvfs *p;

	FUNC_ENTER(FUNC_LV_MODULE);

	p = id_to_cpu_dvfs(_get_cpu_dvfs_id(cpu));

	BUG_ON(NULL == p);

	FUNC_EXIT(FUNC_LV_MODULE);

	return cpu_dvfs_get_cur_freq(p);
}

#ifdef CONFIG_HYBRID_CPU_DVFS
#include <linux/syscore_ops.h>
#include <linux/smp.h>

static void __set_cpuhvfs_init_sta(struct init_sta *sta)
{
	int i, r;
	unsigned int volt;
	struct mt_cpu_dvfs *p;

	for (i = 0; i < NUM_CPU_CLUSTER; i++) {
		p = cluster_to_cpu_dvfs(i);

		r = _sync_opp_tbl_idx(p);	/* find OPP with current frequency */
		BUG_ON(r);

		volt = get_cur_volt_extbuck(p);
		BUG_ON(volt < EXTBUCK_VAL_TO_VOLT(0));

		sta->opp[i] = p->idx_opp_tbl;
		sta->freq[i] = p->ops->get_cur_phy_freq(p);
		sta->volt[i] = VOLT_TO_EXTBUCK_VAL(volt);
		sta->is_on[i] = (p->armpll_is_available ? true : false);
	}
}

static int _mt_cpufreq_syscore_suspend(void)
{
	if (enable_cpuhvfs)
		return cpuhvfs_dvfsp_suspend();

	return 0;
}

static void _mt_cpufreq_syscore_resume(void)
{
	if (enable_cpuhvfs) {
		int i;
		enum mt_cpu_dvfs_id id = _get_cpu_dvfs_id(smp_processor_id());
		struct init_sta sta;
		struct mt_cpu_dvfs *p;

		for (i = 0; i < NUM_CPU_CLUSTER; i++) {
			p = cluster_to_cpu_dvfs(i);

			sta.opp[i] = OPP_AT_SUSPEND;
			sta.freq[i] = p->ops->get_cur_phy_freq(p);
			sta.volt[i] = VOLT_AT_SUSPEND;
		}

		cpuhvfs_dvfsp_resume(id_to_cluster(id), &sta);
	}
}

static struct syscore_ops _mt_cpufreq_syscore_ops = {
	.suspend	= _mt_cpufreq_syscore_suspend,
	.resume		= _mt_cpufreq_syscore_resume,
};
#endif

#ifdef CONFIG_CPU_FREQ
static struct freq_attr *_mt_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver _mt_cpufreq_driver = {
	.verify = _mt_cpufreq_verify,
	.target = _mt_cpufreq_target,
	.init = _mt_cpufreq_init,
	.exit = _mt_cpufreq_exit,
	.get = _mt_cpufreq_get,
	.name = "mt-cpufreq",
	.attr = _mt_cpufreq_attr,
};
#endif

/*
 * Platform driver
 */
static int
_mt_cpufreq_pm_callback(struct notifier_block *nb,
		unsigned long action, void *ptr)
{
	struct mt_cpu_dvfs *p;
	int i;
	unsigned long flags;

	switch (action) {

	case PM_SUSPEND_PREPARE:
		cpufreq_ver("PM_SUSPEND_PREPARE\n");
		cpufreq_lock(flags);
		for_each_cpu_dvfs(i, p) {
			if (!cpu_dvfs_is_available(p))
				continue;
			p->dvfs_disable_by_suspend = true;
		}
		cpufreq_unlock(flags);
		break;
	case PM_HIBERNATION_PREPARE:
		break;

	case PM_POST_SUSPEND:
		cpufreq_ver("PM_POST_SUSPEND\n");
		cpufreq_lock(flags);
		for_each_cpu_dvfs(i, p) {
			if (!cpu_dvfs_is_available(p))
				continue;
			p->dvfs_disable_by_suspend = false;
		}
		cpufreq_unlock(flags);
		break;
	case PM_POST_HIBERNATION:
		break;

	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}
static int _mt_cpufreq_suspend(struct device *dev)
{
#if 0
	struct mt_cpu_dvfs *p;
	int i;

	FUNC_ENTER(FUNC_LV_MODULE);

	for_each_cpu_dvfs(i, p) {
		if (!cpu_dvfs_is_available(p))
			continue;

		p->dvfs_disable_by_suspend = true;
	}

	FUNC_EXIT(FUNC_LV_MODULE);
#endif
	return 0;
}

static int _mt_cpufreq_resume(struct device *dev)
{
#if 0
	struct mt_cpu_dvfs *p;
	int i;

	FUNC_ENTER(FUNC_LV_MODULE);

	for_each_cpu_dvfs(i, p) {
		if (!cpu_dvfs_is_available(p))
			continue;

		p->dvfs_disable_by_suspend = false;
	}

	FUNC_EXIT(FUNC_LV_MODULE);
#endif
	return 0;
}

static int _mt_cpufreq_pdrv_probe(struct platform_device *pdev)
{
	/* For table preparing*/
	unsigned int lv = _mt_cpufreq_get_cpu_level();
	struct mt_cpu_dvfs *p;
	int i, j;
	struct opp_tbl_info *opp_tbl_info;
	struct cpufreq_frequency_table *table;

	FUNC_ENTER(FUNC_LV_MODULE);

	BUG_ON(!(lv == CPU_LEVEL_0 || lv == CPU_LEVEL_1 || lv == CPU_LEVEL_2 || lv == CPU_LEVEL_3));
#if 0
	/* init static power table */
	mt_spower_init();
#endif

#ifdef CONFIG_CPU_DVFS_AEE_RR_REC
	_mt_cpufreq_aee_init();
#endif
/* Prepare OPP table for PPM in probe to avoid nested lock */
	for_each_cpu_dvfs(j, p) {
		opp_tbl_info = (MT_CPU_DVFS_BIG == j) ?
			&opp_tbls_big[0][CPU_LV_TO_OPP_IDX(lv)] : &opp_tbls_little[0][CPU_LV_TO_OPP_IDX(lv)];
		if (NULL == p->freq_tbl_for_cpufreq) {
			table = kzalloc((opp_tbl_info->size + 1) * sizeof(*table), GFP_KERNEL);

			if (NULL == table)
				return -ENOMEM;

			for (i = 0; i < opp_tbl_info->size; i++) {
				table[i].driver_data = i;
				table[i].frequency = opp_tbl_info->opp_tbl[i].cpufreq_khz;
#if FORGE_FREQ_ONLY
				/* hide every row that needs a voltage raise
				 * above the fixed 1.000 V; driver_data keeps
				 * the original opp index so the idx mapping
				 * of the remaining rows is untouched. */
				if (opp_tbl_info->opp_tbl[i].cpufreq_volt > forge_vmax())
					table[i].frequency = CPUFREQ_ENTRY_INVALID;
#endif
			}

			table[opp_tbl_info->size].driver_data = i;
			table[opp_tbl_info->size].frequency = CPUFREQ_TABLE_END;

			p->opp_tbl = opp_tbl_info->opp_tbl;
			p->nr_opp_tbl = opp_tbl_info->size;
			p->freq_tbl_for_cpufreq = table;
		}

		if (j == MT_CPU_DVFS_LITTLE) {
			if (lv == CPU_LEVEL_0 || lv == CPU_LEVEL_1)
				mt_ppm_set_dvfs_table(0, p->freq_tbl_for_cpufreq,
					opp_tbl_info->size, DVFS_TABLE_TYPE_FY);
			else
				mt_ppm_set_dvfs_table(0, p->freq_tbl_for_cpufreq,
					opp_tbl_info->size, DVFS_TABLE_TYPE_SB);
		} else {
			if (lv == CPU_LEVEL_0 || lv == CPU_LEVEL_1)
				mt_ppm_set_dvfs_table(4, p->freq_tbl_for_cpufreq,
					opp_tbl_info->size, DVFS_TABLE_TYPE_FY);
			else
				mt_ppm_set_dvfs_table(4, p->freq_tbl_for_cpufreq,
					opp_tbl_info->size, DVFS_TABLE_TYPE_SB);
		}
	}

#ifdef CONFIG_HYBRID_CPU_DVFS	/* before cpufreq_register_driver */
	if (enable_cpuhvfs) {
		int r;
		struct init_sta sta;

		__set_cpuhvfs_init_sta(&sta);

		r = cpuhvfs_kick_dvfsp_to_run(&sta);
		if (!r) {
			for_each_cpu_dvfs(i, p) {
				p->ops->set_cur_freq = set_cur_freq_hybrid;
				p->ops->set_cur_volt = set_cur_volt_hybrid;
				if (enable_cpuhvfs == 1)
					p->ops->get_cur_volt = get_cur_volt_hybrid;
			}

			register_syscore_ops(&_mt_cpufreq_syscore_ops);
		} else {
			enable_cpuhvfs = 0;
		}
	}
#endif

#ifdef CONFIG_CPU_FREQ
	cpufreq_register_driver(&_mt_cpufreq_driver);
#endif
	register_hotcpu_notifier(&_mt_cpufreq_cpu_notifier);
	mt_ppm_register_client(PPM_CLIENT_DVFS, &ppm_limit_callback);
	pm_notifier(_mt_cpufreq_pm_callback, 0);
	FUNC_EXIT(FUNC_LV_MODULE);

	return 0;
}

static int _mt_cpufreq_pdrv_remove(struct platform_device *pdev)
{
	FUNC_ENTER(FUNC_LV_MODULE);

	unregister_hotcpu_notifier(&_mt_cpufreq_cpu_notifier);
#ifdef CONFIG_CPU_FREQ
	cpufreq_unregister_driver(&_mt_cpufreq_driver);
#endif

	FUNC_EXIT(FUNC_LV_MODULE);

	return 0;
}

static const struct dev_pm_ops _mt_cpufreq_pm_ops = {
	.suspend = _mt_cpufreq_suspend,
	.resume = _mt_cpufreq_resume,
	.freeze = _mt_cpufreq_suspend,
	.thaw = _mt_cpufreq_resume,
	.restore = _mt_cpufreq_resume,
};

struct platform_device _mt_cpufreq_pdev = {
	.name = "mt-cpufreq",
	.id = -1,
};

static struct platform_driver _mt_cpufreq_pdrv = {
	.probe = _mt_cpufreq_pdrv_probe,
	.remove = _mt_cpufreq_pdrv_remove,
	.driver = {
		   .name = "mt-cpufreq",
		   .pm = &_mt_cpufreq_pm_ops,
		   .owner = THIS_MODULE,
		   },
};

#ifndef __KERNEL__
/*
* For CTP
*/
int mt_cpufreq_pdrv_probe(void)
{
	static struct cpufreq_policy policy_little;
	static struct cpufreq_policy policy_big;

	_mt_cpufreq_pdrv_probe(NULL);

	policy_little.cpu = cpu_dvfs[MT_CPU_DVFS_LITTLE].cpu_id;
	_mt_cpufreq_init(&policy_little);

	policy_big.cpu = cpu_dvfs[MT_CPU_DVFS_BIG].cpu_id;
	_mt_cpufreq_init(&policy_big);

	return 0;
}

int mt_cpufreq_set_opp_volt(enum mt_cpu_dvfs_id id, int idx)
{
	int ret = 0;
	static struct opp_tbl_info *info;
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	switch (id) {
	case MT_CPU_DVFS_LITTLE:
		info = &opp_tbls_little[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)];
		break;

	case MT_CPU_DVFS_BIG:
	default:
		info = &opp_tbls_big[CPU_LV_TO_OPP_IDX(CPU_LEVEL_0)];
		break;
	}

	if (idx >= info->size)
		return -1;

	return _set_cur_volt_locked(p, info->opp_tbl[idx].cpufreq_volt);

}

int mt_cpufreq_set_freq(enum mt_cpu_dvfs_id id, int idx)
{
	unsigned int cur_freq;
	unsigned int target_freq;
	int ret;
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	cur_freq = p->ops->get_cur_phy_freq(p);
	target_freq = cpu_dvfs_get_freq_by_idx(p, idx);

	ret = _cpufreq_set_locked(p, cur_freq, target_freq);

	if (ret < 0)
		return ret;

	return target_freq;
}

#include "dvfs.h"

static unsigned int _mt_get_cpu_freq(void)
{
	unsigned int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	DRV_WriteReg32(CLK_DBG_CFG, 2 << 16);	/* sel abist_cksw and enable freq meter sel abist */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x0000FFFF) | (0x07 << 16));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	DRV_WriteReg32(CLK26CALI_1, 0x00ff0000);	/*  */

	/* temp = DRV_Reg32(CLK26CALI_0); */
	DRV_WriteReg32(CLK26CALI_0, 0x1000);
	DRV_WriteReg32(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;

		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
	DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
	DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
	DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

	cpufreq_dbg("CLK26CALI_1 = 0x%x, CPU freq = %d KHz\n", temp, output);

	if (i > 10) {
		cpufreq_dbg("meter not finished!\n");
		return 0;
	} else
		return output;
}

unsigned int dvfs_get_cpu_freq(enum mt_cpu_dvfs_id id)
{
	return _mt_get_cpu_freq();
}

void dvfs_set_cpu_freq_FH(enum mt_cpu_dvfs_id id, int freq)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);
	int idx;

	if (!p) {
		cpufreq_err("%s(%d, %d), id is wrong\n", __func__, id, freq);
		return;
	}

	idx = _search_available_freq_idx(p, freq, CPUFREQ_RELATION_H);

	if (-1 == idx) {
		cpufreq_err("%s(%d, %d), freq is wrong\n", __func__, id, freq);
		return;
	}

	mt_cpufreq_set_freq(id, idx);
}

unsigned int cpu_frequency_output_slt(enum mt_cpu_dvfs_id id)
{
	return (MT_CPU_DVFS_LITTLE == id) ? _mt_get_smallcpu_freq() : _mt_get_bigcpu_freq();
}

void dvfs_set_cpu_volt(enum mt_cpu_dvfs_id id, int volt)	/* volt: mv * 100 */
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(id);

	cpufreq_dbg("%s(%d, %d)\n", __func__, id, volt);

	if (!p) {
		cpufreq_err("%s(%d, %d), id is wrong\n", __func__, id, volt);
		return;
	}

	if (_set_cur_volt_locked(p, volt))
		cpufreq_err("%s(%d, %d), set volt fail\n", __func__, id, volt);

	cpufreq_dbg("%s(%d, %d) Vproc = %d, Vsram = %d\n",
		    __func__, id, volt, p->ops->get_cur_volt(p), get_cur_vsram(p)
	    );
}

static unsigned int vcpu_backup;
void dvfs_disable_by_ptpod(void)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);

	cpufreq_dbg("%s()\n", __func__);

	vcpu_backup = cpu_dvfs_get_cur_volt(p);

	dvfs_set_cpu_volt(MT_CPU_DVFS_LITTLE, 100000);	/* 1V */
}

void dvfs_enable_by_ptpod(void)
{
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);

	cpufreq_dbg("%s()\n", __func__);

	dvfs_set_cpu_volt(MT_CPU_DVFS_LITTLE, vcpu_backup);

}
#endif

#ifdef CONFIG_PROC_FS
/*
* PROC
*/

static char *_copy_from_user_for_proc(const char __user *buffer, size_t count)
{
	char *buf = (char *)__get_free_page(GFP_USER);

	if (!buf)
		return NULL;

	if (count >= PAGE_SIZE)
		goto out;

	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';

	return buf;

out:
	free_page((unsigned long)buf);

	return NULL;
}

/* cpufreq_debug */
static int cpufreq_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "cpufreq debug (log level) = %d\n", func_lv_mask);

	return 0;
}

static ssize_t cpufreq_debug_proc_write(struct file *file, const char __user *buffer, size_t count,
					loff_t *pos)
{
	unsigned int dbg_lv;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &dbg_lv);
	if (rc < 0)
		cpufreq_err("echo dbg_lv (dec) > /proc/cpufreq/cpufreq_debug\n");
	else
		func_lv_mask = dbg_lv;

	free_page((unsigned long)buf);
	return count;
}

static int cpufreq_power_mode_proc_show(struct seq_file *m, void *v)
{
	switch (dvfs_power_mode) {
	case Default:
		seq_puts(m, "Default\n");
		break;
	case Low_power_Mode:
		seq_puts(m, "Low_power_Mode\n");
		break;
	case Just_Make_Mode:
		seq_puts(m, "Just_Make_Mode\n");
		break;
	case Performance_Mode:
		seq_puts(m, "Performance_Mode\n");
		break;
	};

	return 0;
}

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
int mt_cpufreq_get_chip_id_38(void)
{
	/* l681: segment pinned, see M3NOTE_L681_SEGMENT */
	unsigned int chip_code = m3note_segment_or(get_devinfo_with_index(21) & 0xFF);

	if (chip_code == 0x43 || chip_code == 0x4B)
		return 1;
	else
		return 0;
}
#else
int mt_cpufreq_get_chip_id_38(void)
{
	return 0;
}
#endif

int mt_cpufreq_get_ppb_state(void)
{
	return dvfs_power_mode;
}

static ssize_t cpufreq_power_mode_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int do_power_mode;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtoint(buf, 10, &do_power_mode))
		dvfs_power_mode = do_power_mode;
	else
		cpufreq_err("echo 0/1/2/3 > /proc/cpufreq/cpufreq_power_mode\n");

	free_page((unsigned long)buf);
	return count;
}

static int cpufreq_stress_test_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", do_dvfs_stress_test);

	return 0;
}

static ssize_t cpufreq_stress_test_proc_write(struct file *file, const char __user *buffer,
					      size_t count, loff_t *pos)
{
	unsigned int do_stress;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &do_stress);
	if (rc < 0)
		cpufreq_err("echo 0/1 > /proc/cpufreq/cpufreq_stress_test\n");
	else
		do_dvfs_stress_test = do_stress;

	free_page((unsigned long)buf);
	return count;
}

#ifdef CONFIG_HYBRID_CPU_DVFS
static int enable_cpuhvfs_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", enable_cpuhvfs);

	return 0;
}

static ssize_t enable_cpuhvfs_proc_write(struct file *file, const char __user *ubuf, size_t count,
					 loff_t *ppos)
{
	int r, i;
	unsigned int val;
	unsigned long flags;
	struct init_sta sta;
	struct mt_cpu_dvfs *p;

	r = kstrtouint_from_user(ubuf, count, 0, &val);
	if (r)
		return -EINVAL;

	cpufreq_lock(flags);
	if (!val && enable_cpuhvfs) {
		r = cpuhvfs_stop_dvfsp_running();
		if (!r) {
			for_each_cpu_dvfs(i, p) {
				p->ops->set_cur_freq = set_cur_freq;
				p->ops->set_cur_volt = set_cur_volt_extbuck;
				p->ops->get_cur_volt = get_cur_volt_extbuck;
			}
			enable_cpuhvfs = 0;
		}
	} else if (val && !enable_cpuhvfs) {
		__set_cpuhvfs_init_sta(&sta);

		r = cpuhvfs_restart_dvfsp_running(&sta);
		if (!r) {
			for_each_cpu_dvfs(i, p) {
				p->ops->set_cur_freq = set_cur_freq_hybrid;
				p->ops->set_cur_volt = set_cur_volt_hybrid;
				if (val == 1)
					p->ops->get_cur_volt = get_cur_volt_hybrid;
			}
			enable_cpuhvfs = (val == 1 ? val : 2);
		}
	}
	cpufreq_unlock(flags);

	return count;
}
#endif

/* cpufreq_ptpod_freq_volt */
static int cpufreq_ptpod_freq_volt_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)m->private;
	int j;

	for (j = 0; j < p->nr_opp_tbl; j++) {
		seq_printf(m,
			   "[%d] = { .cpufreq_khz = %d,\t.cpufreq_volt = %d,\t.cpufreq_volt_org = %d, },\n",
			   j, p->opp_tbl[j].cpufreq_khz, p->opp_tbl[j].cpufreq_volt,
			   p->opp_tbl[j].cpufreq_volt_org);
	}

	return 0;
}

/* cpufreq_oppidx */
static int cpufreq_oppidx_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)m->private;
	int j;

	seq_printf(m, "[%s/%d]\n" "cpufreq_oppidx = %d\n", p->name, p->cpu_id, p->idx_opp_tbl);

	for (j = 0; j < p->nr_opp_tbl; j++) {
		seq_printf(m, "\tOP(%d, %d),\n",
			   cpu_dvfs_get_freq_by_idx(p, j), cpu_dvfs_get_volt_by_idx(p, j)
		    );
	}

	return 0;
}

static ssize_t cpufreq_oppidx_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)PDE_DATA(file_inode(file));
	int oppidx = 0;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	BUG_ON(NULL == p);
	rc = kstrtoint(buf, 10, &oppidx);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		cpufreq_err("echo oppidx > /proc/cpufreq/cpufreq_oppidx (0 <= %d < %d)\n", oppidx,
			p->nr_opp_tbl);
	} else {
		if (0 <= oppidx && oppidx < p->nr_opp_tbl) {
			p->dvfs_disable_by_procfs = true;
			/* _mt_cpufreq_set(cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) ? MT_CPU_DVFS_LITTLE :
					MT_CPU_DVFS_BIG, oppidx); */

#ifdef __TRIAL_RUN__
			rc = cpuhvfs_set_target_opp(cpu_dvfs_to_cluster(p), oppidx, NULL);
			BUG_ON(rc);
#endif
		} else {
			p->dvfs_disable_by_procfs = false;
			cpufreq_err("echo oppidx > /proc/cpufreq/cpufreq_oppidx (0 <= %d < %d)\n", oppidx,
				p->nr_opp_tbl);
		}
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_freq */
static int cpufreq_freq_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)m->private;

	seq_printf(m, "%d KHz\n", p->ops->get_cur_phy_freq(p));

	return 0;
}

static ssize_t cpufreq_freq_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned long flags;
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)PDE_DATA(file_inode(file));
	unsigned int cur_freq;
	int freq, i, found;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	BUG_ON(NULL == p);
	rc = kstrtoint(buf, 10, &freq);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		cpufreq_err("echo khz > /proc/cpufreq/cpufreq_freq\n");
	} else {
		if (freq < CPUFREQ_LAST_FREQ_LEVEL) {
			if (freq != 0)
				cpufreq_err("frequency should higher than %dKHz!\n",
					    CPUFREQ_LAST_FREQ_LEVEL);

			p->dvfs_disable_by_procfs = false;
			goto end;
		} else {
			for (i = 0; i < p->nr_opp_tbl; i++) {
				if (freq == p->opp_tbl[i].cpufreq_khz) {
					found = 1;
					break;
				}
			}

			if (found == 1) {
				p->dvfs_disable_by_procfs = true;
				cpufreq_lock(flags);
				cur_freq = p->ops->get_cur_phy_freq(p);

				if (freq != cur_freq) {
					p->ops->set_cur_freq(p, cur_freq, freq);
					p->idx_opp_tbl = i;
				}
#ifndef DISABLE_PBM_FEATURE
				if (!p->dvfs_disable_by_suspend)
					_kick_PBM_by_cpu(p);
#endif
				cpufreq_unlock(flags);
			} else {
				p->dvfs_disable_by_procfs = false;
				cpufreq_err("frequency %dKHz! is not found in CPU opp table\n",
					    freq);
			}
		}
	}

end:
	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_volt */
static int cpufreq_volt_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)m->private;
	unsigned long flags;

	cpufreq_lock(flags);

		seq_printf(m, "Vproc: %d mv\n", p->ops->get_cur_volt(p) / 100);	/* mv */
		seq_printf(m, "Vsram: %d mv\n", get_cur_vsram(p) / 100);	/* mv */

	cpufreq_unlock(flags);

	return 0;
}

static ssize_t cpufreq_volt_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned long flags;
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)PDE_DATA(file_inode(file));
	int mv;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &mv);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		cpufreq_err("echo mv > /proc/cpufreq/cpufreq_volt\n");
	} else if (forge_full_dvfs) {
		/* the rails follow the clocks of both clusters now */
		cpufreq_err("[FORGE_CPUFREQ] procfs cpufreq_volt refused with voltage DVFS\n");
	} else {
		p->dvfs_disable_by_procfs = true;
		cpufreq_lock(flags);
		_set_cur_volt_locked(p, mv * 100);
		cpufreq_unlock(flags);
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_turbo_mode */
static int cpufreq_turbo_mode_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)m->private;

	seq_printf(m, "turbo_mode = %d\n", p->turbo_mode);

	return 0;
}

static ssize_t cpufreq_turbo_mode_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)PDE_DATA(file_inode(file));
	unsigned int turbo_mode;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &turbo_mode);
	if (rc < 0)
		cpufreq_err("echo 0/1 > /proc/cpufreq/%s/cpufreq_turbo_mode\n", p->name);
	else
		p->turbo_mode = turbo_mode;

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_time_profile */
static int cpufreq_dvfs_time_profile_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_SET_V_F; i++)
		seq_printf(m, "max[%d] = %lld us\n", i, ktime_to_us(max[i]));

	seq_printf(m, "voltage fail case = %d\n", fail_case);
	seq_printf(m, "fail times= %d\n", fail_times);
	return 0;
}

static ssize_t cpufreq_dvfs_time_profile_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = (struct mt_cpu_dvfs *)PDE_DATA(file_inode(file));
	unsigned int temp;
	int rc;
	int i;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &temp);
	if (rc < 0)
		cpufreq_err("echo 0/1 > /proc/cpufreq/%s/cpufreq_dvfs_time_profile\n", p->name);
	else {
		if (temp == 1) {
			for (i = 0; i < NR_SET_V_F; i++)
				max[i].tv64 = 0;
		}
		fail_times = 0;
		fail_case = 0;
	}
	free_page((unsigned long)buf);

	return count;
}

#define PROC_FOPS_RW(name)							\
	static int name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations name ## _proc_fops = {		\
	.owner          = THIS_MODULE,					\
	.open           = name ## _proc_open,				\
	.read           = seq_read,					\
	.llseek         = seq_lseek,					\
	.release        = single_release,				\
	.write          = name ## _proc_write,				\
}

#define PROC_FOPS_RO(name)							\
	static int name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations name ## _proc_fops = {		\
	.owner          = THIS_MODULE,					\
	.open           = name ## _proc_open,				\
	.read           = seq_read,					\
	.llseek         = seq_lseek,					\
	.release        = single_release,				\
}

#define PROC_ENTRY(name)	{__stringify(name), &name ## _proc_fops}

PROC_FOPS_RW(cpufreq_debug);
PROC_FOPS_RW(cpufreq_stress_test);
PROC_FOPS_RW(cpufreq_power_mode);
#ifdef CONFIG_HYBRID_CPU_DVFS
PROC_FOPS_RW(enable_cpuhvfs);
#endif
PROC_FOPS_RO(cpufreq_ptpod_freq_volt);
PROC_FOPS_RW(cpufreq_oppidx);
PROC_FOPS_RW(cpufreq_freq);
PROC_FOPS_RW(cpufreq_volt);
PROC_FOPS_RW(cpufreq_turbo_mode);
PROC_FOPS_RW(cpufreq_dvfs_time_profile);

static int _create_procfs(void)
{
	struct proc_dir_entry *dir = NULL;
	struct proc_dir_entry *cpu_dir = NULL;
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(0);
	int i, j;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(cpufreq_debug),
		PROC_ENTRY(cpufreq_stress_test),
		PROC_ENTRY(cpufreq_power_mode),
#ifdef CONFIG_HYBRID_CPU_DVFS
		PROC_ENTRY(enable_cpuhvfs),
#endif
	};

	const struct pentry cpu_entries[] = {
		PROC_ENTRY(cpufreq_oppidx),
		PROC_ENTRY(cpufreq_freq),
		PROC_ENTRY(cpufreq_volt),
		PROC_ENTRY(cpufreq_turbo_mode),
		PROC_ENTRY(cpufreq_dvfs_time_profile),
	};

	dir = proc_mkdir("cpufreq", NULL);

	if (!dir) {
		cpufreq_err("fail to create /proc/cpufreq @ %s()\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create
		    (entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, dir, entries[i].fops))
			cpufreq_err("%s(), create /proc/cpufreq/%s failed\n", __func__,
				    entries[i].name);
	}

	for (i = 0; i < ARRAY_SIZE(cpu_entries); i++) {
		if (!proc_create_data
		    (cpu_entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, dir, cpu_entries[i].fops, p))
			cpufreq_err("%s(), create /proc/cpufreq/%s failed\n", __func__,
				    cpu_entries[i].name);
	}

	for_each_cpu_dvfs(j, p) {
		cpu_dir = proc_mkdir(p->name, dir);

		if (!cpu_dir) {
			cpufreq_err("fail to create /proc/cpufreq/%s @ %s()\n", p->name, __func__);
			return -ENOMEM;
		}

		for (i = 0; i < ARRAY_SIZE(cpu_entries); i++) {
			if (!proc_create_data
			    (cpu_entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, cpu_dir,
			     cpu_entries[i].fops, p))
				cpufreq_err("%s(), create /proc/cpufreq/%s/%s failed\n", __func__,
					    p->name, cpu_entries[i].name);
		}
	}

	return 0;
}
#endif
#ifdef CONFIG_OF
static int mt_cpufreq_dts_map(void)
{
	struct device_node *node;

	/* topckgen */
	node = of_find_compatible_node(NULL, NULL, TOPCKGEN_NODE);
	if (!node) {
		cpufreq_info("error: cannot find node " TOPCKGEN_NODE);
		BUG();
	}
	topckgen_base = (unsigned long)of_iomap(node, 0);
	if (!topckgen_base) {
		cpufreq_info("error: cannot iomap " TOPCKGEN_NODE);
		BUG();
	}

	/* infracfg_ao */
	node = of_find_compatible_node(NULL, NULL, INFRACFG_AO_NODE);
	if (!node) {
		cpufreq_info("error: cannot find node " INFRACFG_AO_NODE);
		BUG();
	}
	infracfg_ao_base = (unsigned long)of_iomap(node, 0);
	if (!infracfg_ao_base) {
		cpufreq_info("error: cannot iomap " INFRACFG_AO_NODE);
		BUG();
	}

	/* apmixed */
	node = of_find_compatible_node(NULL, NULL, APMIXED_NODE);
	if (!node) {
		cpufreq_info("error: cannot find node " APMIXED_NODE);
		BUG();
	}
	apmixed_base = (unsigned long)of_iomap(node, 0);
	if (!apmixed_base) {
		cpufreq_info("error: cannot iomap " APMIXED_NODE);
		BUG();
	}
	return 0;
}
#else
static int mt_cpufreq_dts_map(void)
{
	return 0;
}
#endif
/*
* Module driver
*/
/* Off by default: boot does nothing. Writing 1 to
 * /sys/module/mt_cpufreq/parameters/forge_mt_cpufreq runs the full init
 * once: dts_map -> procfs -> platform device+driver -> probe -> clamped
 * tables -> cpufreq_register_driver -> policies come up under the default
 * governor. One-way: disarming takes a reboot (LK restores the boot
 * frequencies). */
int forge_mt_cpufreq_armed;	/* non-static: mt_dvfs_p0.c interlock */

static int _mt_cpufreq_pdrv_init_real(void)
{
	int ret = 0;
	struct cpumask cpu_mask;
	unsigned int cluster_num;
	int i;

	FUNC_ENTER(FUNC_LV_MODULE);

	mt_cpufreq_dts_map();
	debug_vsram = get_cur_vsram(NULL);
	debug_vproc = get_cur_volt_extbuck(NULL);

	cluster_num = (unsigned int)arch_get_nr_clusters();
	for (i = 0; i < cluster_num; i++) {
		arch_get_cluster_cpus(&cpu_mask, i);
		cpu_dvfs[i].cpu_id = cpumask_first(&cpu_mask);
		cpufreq_dbg("cluster_id = %d, cluster_cpuid = %d\n",
	       i, cpu_dvfs[i].cpu_id);
	}

#ifdef CONFIG_HYBRID_CPU_DVFS	/* before platform_driver_register */
	ret = cpuhvfs_module_init();
	if (ret)
		enable_cpuhvfs = 0;
#endif

#ifdef CONFIG_PROC_FS

	/* init proc */
	if (_create_procfs())
		goto out;

#endif

	/* register platform device/driver */
	ret = platform_device_register(&_mt_cpufreq_pdev);

	if (ret) {
		cpufreq_err("fail to register cpufreq device @ %s()\n", __func__);
		goto out;
	}

	ret = platform_driver_register(&_mt_cpufreq_pdrv);

	if (ret) {
		cpufreq_err("fail to register cpufreq driver @ %s()\n", __func__);
		platform_device_unregister(&_mt_cpufreq_pdev);
	}

out:
	FUNC_EXIT(FUNC_LV_MODULE);

	return ret;
}

static void forge_opp_guard_note(const char *who);

/*
 * Policy limits: one CPUFREQ_ADJUST notifier, max =
 * min(forge_cpu_max_{ll,big}_khz, rail-fault cap), snapped to a table row.
 * The boot ceiling is 1196/689 MHz (the rows of the preloader rail); it is
 * lifted by hand or by the ROM after boot_completed, so a hang at a high
 * row does not repeat on the next boot. Later notifiers
 * (forge_therm_guard, forge_ppm_thermal) only lower max further.
 */
static unsigned int forge_cpu_max_khz[NR_MT_CPU_DVFS] = {
	[MT_CPU_DVFS_LITTLE] = 689000,
	[MT_CPU_DVFS_BIG] = 1196000,
};
static unsigned int forge_cpu_max_saved[NR_MT_CPU_DVFS];
/* the ATM CPU budget as ceilings (forge_ppm_thermal.c), 0 = none */
static unsigned int forge_cpu_therm_khz[NR_MT_CPU_DVFS];
module_param_named(forge_cpu_therm_ll_khz, forge_cpu_therm_khz[MT_CPU_DVFS_LITTLE], uint, 0444);
module_param_named(forge_cpu_therm_big_khz, forge_cpu_therm_khz[MT_CPU_DVFS_BIG], uint, 0444);

static void forge_limits_update(void)
{
	int id;

	if (!forge_mt_cpufreq_armed)
		return;
	for (id = 0; id < NR_MT_CPU_DVFS; id++)
		cpufreq_update_policy(id_to_cpu_dvfs(id)->cpu_id);
}

static void forge_limits_work_fn(struct work_struct *work)
{
	forge_limits_update();
}
static DECLARE_WORK(forge_limits_work, forge_limits_work_fn);

static void forge_dvfs_fault_kick(void)
{
	schedule_work(&forge_limits_work);	/* not under the cpufreq mutex */
}

/* highest visible row at or below khz whose voltage <= vmax (lowest if none) */
static unsigned int forge_row_at_or_below(struct mt_cpu_dvfs *p, unsigned int khz,
					  unsigned int vmax)
{
	int i;

	for (i = 0; i < p->nr_opp_tbl; i++)
		if (cpu_dvfs_get_freq_by_idx(p, i) <= khz &&
		    cpu_dvfs_get_volt_by_idx(p, i) <= vmax)
			return cpu_dvfs_get_freq_by_idx(p, i);
	return cpu_dvfs_get_freq_by_idx(p, p->nr_opp_tbl - 1);
}

static int forge_limits_notifier(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct cpufreq_policy *policy = data;
	enum mt_cpu_dvfs_id id;
	struct mt_cpu_dvfs *p;
	unsigned int cap;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_DONE;
	id = _get_cpu_dvfs_id(policy->cpu);
	p = id_to_cpu_dvfs(id);
	if (!p || !p->opp_tbl)
		return NOTIFY_DONE;
	cap = READ_ONCE(forge_cpu_max_khz[id]);
	if (READ_ONCE(forge_cpu_therm_khz[id]))
		cap = min(cap, READ_ONCE(forge_cpu_therm_khz[id]));
	/* after a rail fault: only the rows legal at the read-back VPROC */
	cap = forge_row_at_or_below(p, cap, READ_ONCE(forge_dvfs_fault) ?
				    READ_ONCE(forge_vproc_cur) : forge_vmax());
	cpufreq_verify_within_limits(policy, 0, cap);
	return NOTIFY_OK;
}

static struct notifier_block forge_limits_nb = {
	.notifier_call = forge_limits_notifier,
};

/* stage 4: from mt_ppm_cpu_thermal_protect() (ATM thread, its mutex
 * held): store, the policies follow from a work */
void forge_cpufreq_thermal_cap(unsigned int ll_khz, unsigned int big_khz)
{
	if (READ_ONCE(forge_cpu_therm_khz[MT_CPU_DVFS_LITTLE]) == ll_khz &&
	    READ_ONCE(forge_cpu_therm_khz[MT_CPU_DVFS_BIG]) == big_khz)
		return;
	WRITE_ONCE(forge_cpu_therm_khz[MT_CPU_DVFS_LITTLE], ll_khz);
	WRITE_ONCE(forge_cpu_therm_khz[MT_CPU_DVFS_BIG], big_khz);
	schedule_work(&forge_limits_work);
}

/* forge_therm_guard.c: its 1196 MHz cold ceiling is the freq-only one */
bool forge_cpufreq_full_dvfs(void)
{
	return READ_ONCE(forge_full_dvfs);
}

static int forge_limits_param_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	if (!ret)
		schedule_work(&forge_limits_work);
	return ret;
}

static const struct kernel_param_ops forge_limits_uint_ops = {
	.set = forge_limits_param_set,
	.get = param_get_uint,
};
module_param_cb(forge_cpu_max_ll_khz, &forge_limits_uint_ops,
		&forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], 0644);
module_param_cb(forge_cpu_max_big_khz, &forge_limits_uint_ops,
		&forge_cpu_max_khz[MT_CPU_DVFS_BIG], 0644);

/*
 * Park before suspend and reboot (forge_vrail.c does not lower a rail on
 * its own): the boot ceiling, the governors step down, then the rails go
 * to 1.000 V or what the clocks still running need (forge_set_rails_floor()
 * never goes below that), and the writer is held. Suspend: the ceilings
 * come back after resume.
 */
static void forge_dvfs_park(const char *why)
{
	unsigned long flags;

	WRITE_ONCE(forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], 689000);
	WRITE_ONCE(forge_cpu_max_khz[MT_CPU_DVFS_BIG], 1196000);
	forge_limits_update();
	cpufreq_lock(flags);
	forge_set_rails_floor(100000, 100000);	/* the floor knob stays */
	forge_vrail_hold(forge_dvfs_cookie, true);
	cpufreq_unlock(flags);
	pr_err("[FORGE_DVFS] %s: ceiling 1196/689, VPROC %u VSRAM %u, rails held\n",
	       why, forge_vproc_cur, forge_vsram_cur);
}

static int forge_dvfs_reboot(struct notifier_block *nb, unsigned long code, void *unused)
{
	if (forge_full_dvfs && forge_mt_cpufreq_armed)
		forge_dvfs_park("reboot");
	return NOTIFY_DONE;
}

static struct notifier_block forge_dvfs_reboot_nb = {
	.notifier_call = forge_dvfs_reboot,
};

static int forge_dvfs_pm(struct notifier_block *nb, unsigned long ev, void *unused)
{
	int id;

	if (!forge_full_dvfs || !forge_mt_cpufreq_armed)
		return NOTIFY_DONE;
	if (ev == PM_SUSPEND_PREPARE || ev == PM_HIBERNATION_PREPARE) {
		for (id = 0; id < NR_MT_CPU_DVFS; id++)
			forge_cpu_max_saved[id] = READ_ONCE(forge_cpu_max_khz[id]);
		forge_dvfs_park("suspend");
	} else if (ev == PM_POST_SUSPEND || ev == PM_POST_HIBERNATION) {
		forge_vrail_hold(forge_dvfs_cookie, false);
		for (id = 0; id < NR_MT_CPU_DVFS; id++)
			WRITE_ONCE(forge_cpu_max_khz[id], forge_cpu_max_saved[id]);
		forge_limits_update();
	}
	return NOTIFY_DONE;
}

static struct notifier_block forge_dvfs_pm_nb = {
	.notifier_call = forge_dvfs_pm,
};

/*
 * Voltage DVFS per board, and only on a chip the stock rule calls FY. The
 * stock pick (M681 and L681 use the same tables): DT cpu clock-frequency
 * 1001 MHz -> FY, else efuse 21 func_code_0 1/3 FY, 2/4 SB, else efuse 19
 * binLevel_eng field [6:4] or [12:10] = 2 FY, else SB. The efuse words are
 * the stock ones (forge_cpufreq_efuse(): +1 on L681 with the raw v1 atag)
 * and are printed for checking. The table stays pinned to FY either way
 * (forge_cpu_level_pin); this only decides whether the rails move. Not FY,
 * or the board's switch off: frequency-only at the preloader rail. The
 * stock TURBO branch (func_code_0 0x2X, LEVEL_3) is left out:
 * CONFIG_ARCH_MT6755_TURBO is not set and the M3 Note stock has no turbo
 * table, so such a chip counts as SB (no rail moves).
 * forge_full_dvfs_{m681,l681} act at the arm only (the ROM arms at
 * boot_completed): a later write changes nothing until the next boot.
 */
static bool forge_full_dvfs_m681 = true, forge_full_dvfs_l681 = true;
module_param(forge_full_dvfs_m681, bool, 0644);
module_param(forge_full_dvfs_l681, bool, 0644);

static bool forge_efuse_fy(unsigned int *dt_mhz, u32 *w21, u32 *w19)
{
	struct device_node *node = of_find_node_by_type(NULL, "cpu");
	unsigned int f0, eng;

	*dt_mhz = 0;
	if (node && !of_property_read_u32(node, "clock-frequency", dt_mhz))
		*dt_mhz /= 1000 * 1000;
	of_node_put(node);
	*w21 = forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX);
	*w19 = forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX_ENG);
	f0 = _GET_BITS_VAL_(7:0, *w21);
	eng = _GET_BITS_VAL_(15:0, *w19);
	if (*dt_mhz == 1001 || f0 == 1 || f0 == 3)
		return true;
	if (f0 == 2 || f0 == 4)
		return false;
	return ((eng >> 4) & 0x07) == 2 || ((eng >> 10) & 0x07) == 2;
}

static bool forge_full_dvfs_board(void)
{
	bool l681 = m3note_board_known_is(M3NOTE_BOARD_L681);
	bool on = l681 ? READ_ONCE(forge_full_dvfs_l681) : READ_ONCE(forge_full_dvfs_m681);
	unsigned int dt_mhz;
	u32 w21, w19;
	bool fy = forge_efuse_fy(&dt_mhz, &w21, &w19);

	pr_err("[FORGE_DVFS] %s: stock FY rule -> %s (DT cpu %u MHz, efuse 21 0x%08x, 19 0x%08x, devinfo[0] 0x%08x), switch forge_full_dvfs_%s=%d\n",
	       l681 ? "l681" : "m681", fy ? "FY" : "SB", dt_mhz, w21, w19,
	       get_devinfo_with_index(0), l681 ? "l681" : "m681", on);
	return on && fy;
}

/*
 * pmic_init351 asks this once at its late_initcall, before it replays the
 * stock table. True = the arm will try the take (forge_vrail.allow = 1,
 * the board's switch on, an FY chip by the rule above): the stock VSRAM
 * VOSEL_CTRL = 1 (entry 124) and VOSEL_SLEEP 0x35 (entry 127) are then
 * skipped, so VSRAM stays in the VOSEL_CTRL = 0 mode forge_vrail_take()
 * checks for. False (default): the full stock table. Decided once per
 * boot: allow or forge_full_dvfs_* changed later does not redo it - 0 -> 1
 * leaves the stock CTRL = 1 and the take refuses (frequency-only); 1 -> 0
 * leaves VSRAM in CTRL = 0 with SLEEP = VOSEL, harmless while SPM sleep is
 * off.
 */
bool forge_dvfs_rails_wanted(void)
{
	if (!m3note_board_known_is(M3NOTE_BOARD_M681) &&
	    !m3note_board_known_is(M3NOTE_BOARD_L681))
		return false;
	return forge_vrail_allowed() && forge_full_dvfs_board();
}
EXPORT_SYMBOL(forge_dvfs_rails_wanted);

/* at arm: take the rails (forge_vrail.c, off unless forge_vrail.allow=1) */
static void forge_dvfs_arm(void)
{
	static bool notifiers;
	unsigned int vp = 0, vs = 0;
	int ret = -EPERM;

	if (forge_full_dvfs)
		return;
	if (!m3note_board_known_is(M3NOTE_BOARD_M681) &&
	    !m3note_board_known_is(M3NOTE_BOARD_L681))
		return;
	/* the ceilings and the thermal caps hold in freq-only too */
	if (forge_full_dvfs_board())
		ret = forge_vrail_take(forge_dvfs_cookie, &vp, &vs);
	if (!ret) {
		forge_vproc_cur = vp;
		forge_vsram_cur = vs;
		forge_full_dvfs = true;
	}
	if (!notifiers) {
		notifiers = true;
		cpufreq_register_notifier(&forge_limits_nb, CPUFREQ_POLICY_NOTIFIER);
		register_reboot_notifier(&forge_dvfs_reboot_nb);
		register_pm_notifier(&forge_dvfs_pm_nb);
	}
	pr_err("[FORGE_DVFS] %s (take %d): VPROC %u VSRAM %u (mv*100), floor %u, ceiling LL %u big %u\n",
	       forge_full_dvfs ? "voltage DVFS ON" : "voltage DVFS OFF (freq-only)", ret,
	       forge_vproc_cur, forge_vsram_cur, forge_vproc_floor,
	       forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], forge_cpu_max_khz[MT_CPU_DVFS_BIG]);
}

static int forge_mt_cpufreq_set(const char *val, const struct kernel_param *kp)
{
	extern int forge_dvfs_p0;	/* mt_dvfs_p0.c manual frequency knob */
	int v, ret;

	ret = kstrtoint(val, 0, &v);
	if (ret)
		return ret;
	if (v != 1)
		return -EINVAL;	/* only "arm" exists; disarm = reboot */
	if (forge_mt_cpufreq_armed) {
		pr_err("[FORGE_CPUFREQ] already armed\n");
		return 0;
	}
	/* interlock: the manual phase-0 knob must be back at mode 0 so the
	 * governor starts from LK/boot PLL state, not from a manual uplift
	 * it knows nothing about. */
	if (forge_dvfs_p0 != 0) {
		pr_err("[FORGE_CPUFREQ] REFUSED: forge_dvfs_p0=%d, echo 0 there first\n",
		       forge_dvfs_p0);
		return -EBUSY;
	}

	/* L681: re-read the rails right before arming, report only. The
	 * frequency-only arm writes no voltage, so the guard never blocks it;
	 * forge_opp_guard_note() returns nothing to act on. */
	if (m3note_board_known_is(M3NOTE_BOARD_L681))
		forge_opp_guard_note("forge_mt_cpufreq arm");

	forge_dvfs_arm();
	ret = _mt_cpufreq_pdrv_init_real();
	if (ret) {
		/* Not armed: the rails go back and a later arm takes them
		 * anew. First back to the preloader codes, so S3 and the SPM
		 * idles never come on above them; if the clocks still running
		 * need more (forge_set_rails_floor() never goes below that),
		 * the rails stay owned and the sleep switches stay off. */
		if (forge_full_dvfs) {
			unsigned long flags;
			unsigned int pl = forge_vrail_pl_vproc();

			bool at_pl;

			cpufreq_lock(flags);
			forge_set_rails_floor(pl, 100000);
			at_pl = forge_vproc_cur == pl && !forge_dvfs_fault;	/* read under the lock */
			cpufreq_unlock(flags);
			if (at_pl) {
				forge_full_dvfs = false;
				forge_vproc_cur = FORGE_FIXED_VPROC;
				forge_vsram_cur = FORGE_FIXED_VSRAM;
				forge_vrail_release(forge_dvfs_cookie);
			} else {
				forge_vrail_hold(forge_dvfs_cookie, true);
				pr_err("[FORGE_DVFS] arm failed: rails at VPROC %u, not the PL %u (fault %d) - kept owned and held, sleep stays off\n",
				       forge_vproc_cur, pl, forge_dvfs_fault);
			}
		}
		pr_err("[FORGE_CPUFREQ] arm FAILED ret=%d\n", ret);
		return ret;
	}
	forge_mt_cpufreq_armed = 1;
	forge_limits_update();	/* the policies came up before armed was set */
	pr_err("[FORGE_CPUFREQ] ARMED: governor DVFS, %s\n",
	       forge_full_dvfs ? "voltage DVFS (stock FY), boot ceiling big 1196000 LL 689000 kHz" :
	       "freq-only, big<=1196000 LL<=689000 kHz @ fixed 1.000 V");
	return 0;
}

static const struct kernel_param_ops forge_mt_cpufreq_ops = {
	.set = forge_mt_cpufreq_set,
	.get = param_get_int,
};
module_param_cb(forge_mt_cpufreq, &forge_mt_cpufreq_ops,
		&forge_mt_cpufreq_armed, 0644);
MODULE_PARM_DESC(forge_mt_cpufreq,
		 "m681 stage-3: write 1 to arm the freq-only cpufreq governor (default off; disarm = reboot)");

/*
 * Read-only rail and OPP dump.
 *
 * Nothing here writes a PMIC, a buck or an I2C register. MT6311 is read
 * with one combined I2C transfer per register (1-byte index out, 1 byte
 * back) on i2c-3 / 0x6b; MT6351 is read with pmic_read_interface(). Runs
 * once from this file's late_initcall and again on
 *   echo 1 > /sys/module/mt_cpufreq/parameters/forge_opp_dump
 *
 * Code -> voltage, stock formula for all three rails:
 *   PMIC_VAL_TO_VOLT = code * 625 + 60000 (mV*100), i.e. 600 mV +
 *   6.25 mV/step (VGPU likewise).
 */
#define FORGE_OPP_TAG		"[FORGE_OPP] "
#define FORGE_MT6311_BUS	3
#define FORGE_MT6311_ADDR	0x6b
/* MT6311 (3.10 mach/mt6311.h:179-192) */
#define FORGE_MT6311_CID	0x00
#define FORGE_MT6311_SWCID	0x01
#define FORGE_MT6311_CON7	0x88	/* bit1 VOSEL_CTRL */
#define FORGE_MT6311_CON12	0x8D	/* VDVFS11 VOSEL */
#define FORGE_MT6311_CON13	0x8E	/* VDVFS11 VOSEL_ON (stock setter target) */
#define FORGE_MT6311_CON14	0x8F	/* VDVFS11 VOSEL_SLEEP */
#define FORGE_MT6311_CON15	0x90	/* VDVFS11 NI_VOSEL (actual output) */
/* MT6351 over pwrap (mach/mt6351_regs.h:298-383, = 3.10 upmu_hw.h:298-383) */
#define FORGE_6351_VCORE_CON5	0x060A	/* BUCK_VCORE_VOSEL_ON */
#define FORGE_6351_VCORE_CON7	0x060E	/* DA_NI_VCORE_VOSEL */
#define FORGE_6351_VGPU_CON2	0x0618	/* bit0 BUCK_VGPU_EN, bit13 DA_QI_VGPU_EN */
#define FORGE_6351_VGPU_CON5	0x061E	/* BUCK_VGPU_VOSEL_ON */
#define FORGE_6351_VGPU_CON7	0x0622	/* DA_NI_VGPU_VOSEL */
#define FORGE_6351_VSRAM_CON5	0x06AA	/* BUCK_VSRAM_PROC_VOSEL_ON */
#define FORGE_6351_VSRAM_CON7	0x06AE	/* DA_NI_VSRAM_PROC_VOSEL */
#define FORGE_VOSEL_MASK	0x7F
#define FORGE_CODE_TO_MV100(c)	((c) * 625 + 60000)

static DEFINE_MUTEX(forge_opp_lock);
static int forge_opp_dump_val;

/*
 * Rail guard. Expected values, all from the stock kernel:
 *  - MT6311 chip id = (CID << 8) | SWCID in {0x0110, 0x0120, 0x0130};
 *  - VDVFS11 VOSEL_ON 0x8E (the stock VPROC readback) == code of the
 *    voltage every reachable OPP runs at; frequency-only: all rows <=
 *    FORGE_FIXED_VPROC = 1.000 V -> 0x40, also the stock init value.
 *    NI_VOSEL 0x90 (actual output) must show the same code;
 *  - MT6351 BUCK_VSRAM_PROC_VOSEL_ON 0x06AA (the stock VSRAM readback: code
 *    0 or > 0x7F is invalid) within [MIN_VSRAM_VOLT 93000, MAX_VSRAM_VOLT
 *    115000] and VPROC <= VSRAM <= VPROC + MAX_DIFF_VSRAM_VPROC 20000, the
 *    stock sanity test of set_cur_volt_extbuck; nominal VSRAM = VPROC +
 *    NORMAL_DIFF 10000 = 1.100 V = 0x50.
 *  - any i2c-3 or pwrap read error = FAIL.
 */
#define FORGE_GUARD_VPROC_CODE	((FORGE_FIXED_VPROC - 60000) / 625)	/* 0x40 */
#define FORGE_GUARD_VSRAM_MIN	93000
#define FORGE_GUARD_VSRAM_MAX	115000

static int forge_opp_guard_ok;			/* 0444: last guard result */
/*
 * 0644, default 1. Acts only on voltage-changing paths, i.e. on the return
 * value of forge_opp_guard_check(): set_cur_volt_extbuck (both variants)
 * and the GPU volt_switch_pmic. 1 = those paths refuse (-EPERM) on guard
 * FAIL; 0 = they only log it. The frequency-only forge_mt_cpufreq arm is
 * not affected: it calls the report-only forge_opp_guard_note().
 */
static int forge_opp_guard_enforce = 1;
module_param(forge_opp_guard_ok, int, 0444);
MODULE_PARM_DESC(forge_opp_guard_ok, "l681: 1 = last read-only rail guard passed");
module_param(forge_opp_guard_enforce, int, 0644);
MODULE_PARM_DESC(forge_opp_guard_enforce,
		 "l681: 1 (default) = voltage-writing paths refuse on rail guard FAIL; never affects the freq-only forge_mt_cpufreq arm");

struct forge_opp_snap {
	u8 cid, swcid, con7, con12, con13, con14, con15;
	int i2c_ret;
	unsigned int vsram, vsram_ni, vgpu, vgpu_ni, vgpu_en, vgpu_qi, vcore, vcore_ni;
	unsigned int pwrap_err;
};

static int forge_mt6311_read(struct i2c_adapter *adap, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 idx = reg;
	int ret;

	memset(msgs, 0, sizeof(msgs));
	msgs[0].addr = FORGE_MT6311_ADDR;
	msgs[0].flags = 0;		/* register index only, no data byte */
	msgs[0].len = 1;
	msgs[0].buf = &idx;
	msgs[1].addr = FORGE_MT6311_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret == 2)
		return 0;
	return (ret < 0) ? ret : -EIO;
}

/* "1000.00" style: mV*100 -> integer mV and hundredths */
#define FORGE_MV(v)	((v) / 100), ((v) % 100)

static void forge_opp_dump_cpu_tables(void)
{
	unsigned int lv = _mt_cpufreq_get_cpu_level();
	unsigned int lv_ef = _mt_cpufreq_get_cpu_level_efuse();
	u32 w_old = get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX);
	u32 e_old = get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX_ENG);
	u32 w_new = forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX);
	u32 e_new = forge_cpufreq_efuse(FUNC_CODE_EFUSE_INDEX_ENG);
	/* same predicate as forge_cpufreq_efuse() */
	unsigned int shift = (m3note_board_known_is(M3NOTE_BOARD_L681) &&
			      get_devinfo_with_index(0) == FORGE_ATAG_DEVINFO_TAG) ? 1 : 0;
	static const char * const lv_name[] = { "_6353", "FY", "SB", "P15" };
	int c, i;

	pr_err(FORGE_OPP_TAG "efuse: old read kernel idx %u=0x%08x idx %u=0x%08x (func_code_0=0x%x binLevel_eng=0x%x) | stock idx %u/%u = kernel idx %u/%u: 0x%08x 0x%08x (func_code_0=0x%x binLevel_eng=0x%x)\n",
	       FUNC_CODE_EFUSE_INDEX, w_old, FUNC_CODE_EFUSE_INDEX_ENG, e_old,
	       _GET_BITS_VAL_(7:0, w_old), _GET_BITS_VAL_(15:0, e_old),
	       FUNC_CODE_EFUSE_INDEX, FUNC_CODE_EFUSE_INDEX_ENG,
	       FUNC_CODE_EFUSE_INDEX + shift, FUNC_CODE_EFUSE_INDEX_ENG + shift,
	       w_new, e_new, _GET_BITS_VAL_(7:0, w_new), _GET_BITS_VAL_(15:0, e_new));
	/*
	 * The stock FY/SB rule (the stock L681 DT has clock-frequency 1950, so
	 * the efuse decides) on both readings, kernel index and kernel index +
	 * 1, with the devinfo[0] word: the +1 shift exists when the LK passes
	 * the raw v1 atag (data[0] = tag 0x41000804). One read-only line per
	 * board.
	 */
	{
		u32 d0 = get_devinfo_with_index(0);
		int s;

		for (s = 0; s <= 1; s++) {
			u32 w = get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX + s);
			u32 e = get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX_ENG + s);
			u32 fc = _GET_BITS_VAL_(7:0, w), bl = _GET_BITS_VAL_(15:0, e);
			bool fy = fc == 1 || fc == 3 ||
				  (fc != 2 && fc != 4 &&
				   (((bl >> 4) & 7) == 2 || ((bl >> 10) & 7) == 2));

			pr_err(FORGE_OPP_TAG "efuse stock rule, shift %d: kernel idx %u=0x%08x idx %u=0x%08x -> func_code_0 0x%x binLevel_eng 0x%x -> %s%s\n",
			       s, FUNC_CODE_EFUSE_INDEX + s, w,
			       FUNC_CODE_EFUSE_INDEX_ENG + s, e, fc, bl,
			       fy ? "FY" : "SB",
			       s == shift ? " (the reading this kernel uses)" : "");
		}
		pr_err(FORGE_OPP_TAG "efuse devinfo[0]=0x%08x (%s the raw v1 atag 0x%08x), board %s\n",
		       d0, d0 == FORGE_ATAG_DEVINFO_TAG ? "=" : "!=",
		       FORGE_ATAG_DEVINFO_TAG,
		       m3note_board_known_is(M3NOTE_BOARD_L681) ? "l681" :
		       m3note_board_known_is(M3NOTE_BOARD_M681) ? "m681" : "unknown");
	}
	pr_err(FORGE_OPP_TAG "CPU table lv=%u (%s%s; efuse says lv=%u %s); freq-only cut: nominal V <= %u (mv*100)\n",
	       lv, forge_cpu_level_pin ? "pinned " : "",
	       lv < ARRAY_SIZE(lv_name) ? lv_name[lv] : "?",
	       lv_ef, lv_ef < ARRAY_SIZE(lv_name) ? lv_name[lv_ef] : "?",
	       FORGE_FIXED_VPROC);
	if (lv >= ARRAY_SIZE(opp_tbls_big[0]))
		return;

	for (c = 0; c < 2; c++) {
		struct opp_tbl_info *t = c ? &opp_tbls_big[0][lv] : &opp_tbls_little[0][lv];
		char vis[96], hid[96];
		int nv = 0, nh = 0;

		vis[0] = hid[0] = '\0';
		for (i = 0; i < t->size; i++) {
			unsigned int khz = t->opp_tbl[i].cpufreq_khz;
			unsigned int v = t->opp_tbl[i].cpufreq_volt;

			/* same predicate as the probe clamp and forge_max_valid_freq */
			if (v <= FORGE_FIXED_VPROC)
				nv += scnprintf(vis + nv, sizeof(vis) - nv, " %u", khz / 1000);
			else
				nh += scnprintf(hid + nh, sizeof(hid) - nh, " %u", khz / 1000);
		}
		pr_err(FORGE_OPP_TAG "CPU %s visible MHz:%s | hidden (needs >1.000 V):%s\n",
		       c ? "big" : "LL", vis, hid);
	}
}

static void forge_opp_read(struct forge_opp_snap *r)
{
	struct i2c_adapter *adap;

	memset(r, 0, sizeof(*r));
	adap = i2c_get_adapter(FORGE_MT6311_BUS);
	if (!adap) {
		r->i2c_ret = -ENODEV;
	} else {
		r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CID, &r->cid);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_SWCID, &r->swcid);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CON7, &r->con7);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CON12, &r->con12);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CON13, &r->con13);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CON14, &r->con14);
		if (!r->i2c_ret)
			r->i2c_ret = forge_mt6311_read(adap, FORGE_MT6311_CON15, &r->con15);
		i2c_put_adapter(adap);
	}

	r->pwrap_err |= pmic_read_interface(FORGE_6351_VSRAM_CON5, &r->vsram, FORGE_VOSEL_MASK, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VSRAM_CON7, &r->vsram_ni, FORGE_VOSEL_MASK, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VGPU_CON5, &r->vgpu, FORGE_VOSEL_MASK, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VGPU_CON7, &r->vgpu_ni, FORGE_VOSEL_MASK, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VGPU_CON2, &r->vgpu_en, 0x1, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VGPU_CON2, &r->vgpu_qi, 0x1, 13);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VCORE_CON5, &r->vcore, FORGE_VOSEL_MASK, 0);
	r->pwrap_err |= pmic_read_interface(FORGE_6351_VCORE_CON7, &r->vcore_ni, FORGE_VOSEL_MASK, 0);
}

/*
 * Read-only control registers, dump only; kept out of forge_opp_read() so
 * the guard snapshot and its error fields are unchanged. MT6311: 0x88 b0
 * EN_CTRL, b1 VOSEL_CTRL; 0x8A b7 OC; 0x8B/0x8C/0x94 slew/settle (stock
 * init 0x01/0x05/[1:0]=3 [5:4]=1); 0x8D VOSEL; 0x8F VOSEL_SLEEP; 0x93 b0.
 * MT6351: 0x06A0 b1 VSRAM_PROC VOSEL_CTRL, 0x06A6, 0x06AE NI_VOSEL; 0x0614
 * b1 VGPU VOSEL_CTRL, 0x0622 VGPU NI_VOSEL. One-byte I2C reads and pwrap
 * reads only.
 */
static const u8 forge_6311_ctl_regs[] = { 0x88, 0x8A, 0x8B, 0x8C, 0x8D, 0x8F, 0x93, 0x94 };
static const u16 forge_6351_ctl_regs[] = { 0x06A0, 0x06A6, 0x06AE, 0x0614, 0x0622, 0x06A8, 0x06AC };

static void forge_opp_dump_ctl(void)
{
	u8 v11[ARRAY_SIZE(forge_6311_ctl_regs)];
	unsigned int v51[ARRAY_SIZE(forge_6351_ctl_regs)];
	struct i2c_adapter *adap;
	unsigned int perr = 0;
	int i, iret = 0;

	memset(v11, 0, sizeof(v11));
	memset(v51, 0, sizeof(v51));
	adap = i2c_get_adapter(FORGE_MT6311_BUS);
	if (!adap) {
		iret = -ENODEV;
	} else {
		for (i = 0; i < ARRAY_SIZE(forge_6311_ctl_regs) && !iret; i++)
			iret = forge_mt6311_read(adap, forge_6311_ctl_regs[i], &v11[i]);
		i2c_put_adapter(adap);
	}
	for (i = 0; i < ARRAY_SIZE(forge_6351_ctl_regs); i++)
		perr |= pmic_read_interface(forge_6351_ctl_regs[i], &v51[i], 0xFFFF, 0);

	/*
	 * t1: two lines. As one line the record was cut on the device
	 * after "6351 0x06A0=0x0000 (VSRAM VOSEL_CT"
	 * (a capture).
	 */
	pr_err(FORGE_OPP_TAG "ctl 6311: 0x88=0x%02x (EN_CTRL=b0 %u, VOSEL_CTRL=b1 %u) 0x8A=0x%02x (OC b7 %u) 0x8B=0x%02x 0x8C=0x%02x 0x8D=0x%02x 0x8F=0x%02x 0x93=0x%02x 0x94=0x%02x i2c_ret=%d\n",
	       v11[0], v11[0] & 1, (v11[0] >> 1) & 1, v11[1], (v11[1] >> 7) & 1,
	       v11[2], v11[3], v11[4], v11[5], v11[6], v11[7], iret);
	pr_err(FORGE_OPP_TAG "ctl 6351: 0x06A0=0x%04x (VSRAM VOSEL_CTRL=b1 %u) 0x06A6=0x%04x 0x06AE=0x%04x 0x0614=0x%04x (VGPU VOSEL_CTRL=b1 %u) 0x0622=0x%04x pwrap_err=%u\n",
	       v51[0], (v51[0] >> 1) & 1, v51[1], v51[2], v51[3], (v51[3] >> 1) & 1,
	       v51[4], perr);
	/* stage 1: the two VSRAM_PROC fields the variant-A writer will own
	 * (VOSEL 0x06A8, VOSEL_SLEEP 0x06AC), field 6:0 */
	pr_err(FORGE_OPP_TAG "ctl 6351 VSRAM_PROC: 0x06A8=0x%04x (VOSEL 0x%02x) 0x06AC=0x%04x (VOSEL_SLEEP 0x%02x) pwrap_err=%u\n",
	       v51[5], v51[5] & 0x7F, v51[6], v51[6] & 0x7F, perr);
}

/* pure check of one snapshot; returns 0 or -EPERM, reason in buf */
static int forge_opp_guard_eval(const struct forge_opp_snap *r, char *why, size_t n)
{
	unsigned int chip = ((unsigned int)r->cid << 8) | r->swcid;
	unsigned int vproc = FORGE_CODE_TO_MV100(r->con13 & FORGE_VOSEL_MASK);
	unsigned int vsram = FORGE_CODE_TO_MV100(r->vsram);

	if (r->i2c_ret) {
		scnprintf(why, n, "MT6311 i2c-%d 0x%02x read error %d",
			  FORGE_MT6311_BUS, FORGE_MT6311_ADDR, r->i2c_ret);
		return -EPERM;
	}
	if (r->pwrap_err) {
		scnprintf(why, n, "MT6351 pwrap read error %u", r->pwrap_err);
		return -EPERM;
	}
	if (chip != 0x0110 && chip != 0x0120 && chip != 0x0130) {
		scnprintf(why, n, "MT6311 chip id 0x%04x not in {0110,0120,0130}", chip);
		return -EPERM;
	}
	if ((r->con13 & FORGE_VOSEL_MASK) != FORGE_GUARD_VPROC_CODE) {
		scnprintf(why, n, "VDVFS11 VOSEL_ON 0x8e=0x%02x, expected 0x%02x",
			  r->con13, FORGE_GUARD_VPROC_CODE);
		return -EPERM;
	}
	if ((r->con15 & FORGE_VOSEL_MASK) != FORGE_GUARD_VPROC_CODE) {
		scnprintf(why, n, "VDVFS11 NI_VOSEL 0x90=0x%02x, expected 0x%02x",
			  r->con15, FORGE_GUARD_VPROC_CODE);
		return -EPERM;
	}
	if (r->vsram == 0 || vsram < FORGE_GUARD_VSRAM_MIN || vsram > FORGE_GUARD_VSRAM_MAX) {
		scnprintf(why, n, "VSRAM_PROC 0x06aa=0x%02x (%u mv*100) outside [%u; %u]",
			  r->vsram, vsram, FORGE_GUARD_VSRAM_MIN, FORGE_GUARD_VSRAM_MAX);
		return -EPERM;
	}
	if (vsram < vproc || vsram - vproc > MAX_DIFF_VSRAM_VPROC) {
		scnprintf(why, n, "VSRAM-VPROC = %d (mv*100) outside [0; %d]",
			  (int)vsram - (int)vproc, MAX_DIFF_VSRAM_VPROC);
		return -EPERM;
	}
	scnprintf(why, n, "chip 0x%04x, 0x8e=0x90=0x%02x (%u mv*100), VSRAM %u (+%u)",
		  chip, r->con13, vproc, vsram, vsram - vproc);
	return 0;
}

static int forge_opp_guard_report(const struct forge_opp_snap *r, const char *who)
{
	char why[112];
	int ret = forge_opp_guard_eval(r, why, sizeof(why));

	forge_opp_guard_ok = !ret;
	if (ret)
		pr_err(FORGE_OPP_TAG "guard FAIL (%s): %s\n", who, why);
	else
		pr_err(FORGE_OPP_TAG "guard OK (%s): %s\n", who, why);
	return ret;
}

/* fresh read + report; raw result (0 / -EPERM), sets forge_opp_guard_ok */
static int forge_opp_guard_run(const char *who)
{
	struct forge_opp_snap r;
	int ret;

	if (!m3note_board_known_is(M3NOTE_BOARD_L681)) {
		forge_opp_guard_ok = 0;
		pr_err(FORGE_OPP_TAG "guard FAIL (%s): board is not l681\n", who);
		return -EPERM;
	}
	might_sleep();
	mutex_lock(&forge_opp_lock);
	forge_opp_read(&r);
	ret = forge_opp_guard_report(&r, who);
	mutex_unlock(&forge_opp_lock);
	return ret;
}

/* report-only entry for paths that change no voltage (freq-only arm) */
static void forge_opp_guard_note(const char *who)
{
	forge_opp_guard_run(who);
}

/*
 * The single entry every voltage-writing path must call first, right
 * before it acts (VPROC/VSRAM in set_cur_volt_extbuck, VGPU in
 * mt_gpufreq). Re-reads the rails, never trusts a cached result. Read-only.
 * Returns -EPERM on guard FAIL (any expectation, any read error, board not
 * L681) while forge_opp_guard_enforce=1; with enforce=0 the FAIL is only
 * logged and 0 is returned. Not used by the frequency-only arm.
 */
int forge_opp_guard_check(const char *who)
{
	int ret = forge_opp_guard_run(who);

	if (ret && !forge_opp_guard_enforce) {
		pr_err(FORGE_OPP_TAG "guard FAIL ignored (%s): forge_opp_guard_enforce=0\n", who);
		return 0;
	}
	return ret;
}

static void forge_opp_dump(void)
{
	struct forge_opp_snap r;

	/* read only; L681 and M681 */
	if (!m3note_board_known_is(M3NOTE_BOARD_L681) &&
	    !m3note_board_known_is(M3NOTE_BOARD_M681))
		return;

	mutex_lock(&forge_opp_lock);

	forge_opp_dump_cpu_tables();
	forge_opp_read(&r);

	if (r.i2c_ret)
		pr_err(FORGE_OPP_TAG "MT6311 i2c-%d 0x%02x read FAILED ret=%d (CID=0x%02x SWCID=0x%02x so far)\n",
		       FORGE_MT6311_BUS, FORGE_MT6311_ADDR, r.i2c_ret, r.cid, r.swcid);

	pr_err(FORGE_OPP_TAG "MT6311 CID=0x%02x%02x VDVFS11 set=0x%02x (%u.%02u mV) act=0x%02x (%u.%02u mV); VSRAM_PROC %u.%02u mV; VGPU %u.%02u mV; VCORE %u.%02u mV%s\n",
	       r.cid, r.swcid,
	       r.con13, FORGE_MV(FORGE_CODE_TO_MV100(r.con13 & FORGE_VOSEL_MASK)),
	       r.con15, FORGE_MV(FORGE_CODE_TO_MV100(r.con15 & FORGE_VOSEL_MASK)),
	       FORGE_MV(FORGE_CODE_TO_MV100(r.vsram)),
	       FORGE_MV(FORGE_CODE_TO_MV100(r.vgpu)),
	       FORGE_MV(FORGE_CODE_TO_MV100(r.vcore)),
	       r.i2c_ret ? " [MT6311 INVALID]" : "");
	pr_err(FORGE_OPP_TAG "raw: MT6311 0x88=0x%02x 0x8d=0x%02x 0x8e=0x%02x 0x8f=0x%02x 0x90=0x%02x | MT6351 0x06aa=0x%02x ni=0x%02x 0x061e=0x%02x ni=0x%02x en=%u qi_en=%u 0x060a=0x%02x ni=0x%02x pwrap_err=%u\n",
	       r.con7, r.con12, r.con13, r.con14, r.con15,
	       r.vsram, r.vsram_ni, r.vgpu, r.vgpu_ni, r.vgpu_en, r.vgpu_qi,
	       r.vcore, r.vcore_ni, r.pwrap_err);
	forge_opp_dump_ctl();
	forge_opp_guard_report(&r, "dump");

	mutex_unlock(&forge_opp_lock);
}

static int forge_opp_dump_set(const char *val, const struct kernel_param *kp)
{
	int v, ret;

	ret = kstrtoint(val, 0, &v);
	if (ret)
		return ret;
	if (v != 1)
		return -EINVAL;	/* only "dump" exists */
	forge_opp_dump();
	return 0;
}

static const struct kernel_param_ops forge_opp_dump_ops = {
	.set = forge_opp_dump_set,
	.get = param_get_int,
};
module_param_cb(forge_opp_dump, &forge_opp_dump_ops, &forge_opp_dump_val, 0644);
MODULE_PARM_DESC(forge_opp_dump,
		 "l681/m681: write 1 to print CPU OPP tables, the efuse FY/SB verdict and READ-ONLY MT6311/MT6351 rail registers");

static int __init _mt_cpufreq_pdrv_init(void)
{
	pr_info("[FORGE_CPUFREQ] built (freq-only, clamped); arm: echo 1 > /sys/module/mt_cpufreq/parameters/forge_mt_cpufreq\n");
	forge_opp_dump();
	return 0;
}

static void __exit _mt_cpufreq_pdrv_exit(void)
{
	FUNC_ENTER(FUNC_LV_MODULE);

	platform_driver_unregister(&_mt_cpufreq_pdrv);
	platform_device_unregister(&_mt_cpufreq_pdev);

	FUNC_EXIT(FUNC_LV_MODULE);
}

late_initcall(_mt_cpufreq_pdrv_init);
module_exit(_mt_cpufreq_pdrv_exit);

MODULE_DESCRIPTION("MediaTek CPU DVFS Driver v0.3");
MODULE_LICENSE("GPL");
