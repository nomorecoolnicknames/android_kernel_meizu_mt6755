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
#include <linux/m3note_board.h>
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
#include <linux/workqueue.h>
#include <linux/input.h>
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
/* The shared header defines CONFIG_HYBRID_CPU_DVFS for ARCH_MT6755, which
 * routes ops through the SPM DVFSP co-processor firmware (cpuhvfs_*, not
 * built). Undefine it here to keep the AP-driven PLL/mux path. */
#undef CONFIG_HYBRID_CPU_DVFS

/* Frequency-only build: VPROC actuation is refused, and the OPP ladder is
 * clamped to rows legal at the boot voltage (VPROC 1.000 V, read back as
 * CON12 = CON13 = 0x40, the stock FY voltage of these rows):
 *   big <= 1196 MHz, LL <= 689 MHz.
 * VSRAM is untouched too (PL-set); its assumed value only feeds bypassed
 * sanity paths. */
#define FORGE_FREQ_ONLY		1
#define FORGE_FIXED_VPROC	100000	/* mv*100, silicon readback */
#define FORGE_FIXED_VSRAM	110000	/* VPROC + NORMAL_DIFF_VRSAM_VPROC */
/* Compile the branches built with CONFIG_MTK_PMIC_CHIP_MT6353=y, where
 * every FORGE_FREQ_ONLY guard above lives. This tree has no MT6353 symbol,
 * and its #else branches read the VSRAM/ext-buck rails without those
 * guards and pick the CPU level from DT instead of the ext-buck check. */
#define FORGE_CPUFREQ_44_PATH	1
/* The MT6353 rail helpers of those branches stay unreachable behind the
 * FORGE_FREQ_ONLY returns; declared only so the dead tails compile. They are
 * not defined anywhere in 4.9: a reachable call would fail at link time. */
extern unsigned int mt6353_upmu_get_da_ni_vsram_proc_vosel(void);
extern unsigned int mt6353_upmu_get_da_ni_vproc_vosel(void);
extern void mt6353_upmu_set_ldo_vsram_proc_vosel_ctrl(unsigned int val);
extern void mt6353_upmu_set_ldo_vsram_proc_vosel_on(unsigned int val);
extern void mt6353_upmu_set_buck_vproc_vosel_ctrl(unsigned int val);
extern void mt6353_upmu_set_buck_vproc_vosel_on(unsigned int val);

/* Stock FY voltage DVFS on top of the frequency-only build. When cpufreq
 * arms on m681, mt_extbuck_probe checks the MT6311/MT6351 guard
 * (VOSEL_CTRL bits set, codes sane, no OC) and takes the rails; only then
 * forge_full_dvfs is set and every FORGE_FREQ_ONLY limit below moves from
 * the fixed 1.000 V to MAX_VPROC_VOLT. A failed guard leaves the
 * frequency-only build as it was. A failed rail write latches
 * forge_dvfs_fault: no voltage is raised again and the policy ceilings
 * drop to the rows legal at the voltage read back. */
#define FORGE_M681_FULL_DVFS	1
static bool forge_full_dvfs;
static bool forge_dvfs_fault;
static unsigned int forge_vproc_cur = 100000, forge_vsram_cur = 110000;	/* read back */
extern int forge_extbuck_dvfs_init(unsigned int *vproc_uv10, unsigned int *vsram_uv10)
	__attribute__((weak));
extern int forge_extbuck_set_rails(unsigned int vproc_uv10, unsigned int *vproc_rb,
				   unsigned int *vsram_rb) __attribute__((weak));
extern void forge_extbuck_sh0(void) __attribute__((weak));
extern void forge_extbuck_dvfs_release(void) __attribute__((weak));

/* forge_therm_guard asks at late init, long before cpufreq arms: answer
 * whether this build drives the rails on this board. Whether the guard let
 * it is forge_full_dvfs; without it the freq-only table tops at 1196/689
 * anyway, and the boot ceiling below holds until the ROM lifts it. */
static bool forge_full_dvfs_board(void);

bool forge_cpufreq_full_dvfs(void)
{
	return FORGE_M681_FULL_DVFS && forge_full_dvfs_board();
}
EXPORT_SYMBOL(forge_cpufreq_full_dvfs);

static unsigned int forge_vmax(void)
{
	return READ_ONCE(forge_full_dvfs) ? 115000 /* MAX_VPROC_VOLT */ : FORGE_FIXED_VPROC;
}

/* VPROC floor, mv*100. 1.000 V (the PL rail) by default: rows below
 * 1196/689 then run above their stock voltage, never below it; 0 = the
 * stock voltages (0.800 V and up). The reboot path restores 1.000 V. */
static unsigned int forge_vproc_floor = 100000;
module_param(forge_vproc_floor, uint, 0644);

/* highest LL overclock row allowed (0 = stock 1001, 1..5 = 1092, 1144,
 * 1196, 1404, 1599). Rows exist in cpufreq only if forge_ll_oc > 0 when
 * cpufreq arms (forge_ll_oc_armed); after that it may move at run time. */
static unsigned int forge_ll_oc;
static bool forge_ll_oc_armed;

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
/* DCM sync is provided by mt_dcm.c, which is not built; the
 * sync_dcm_set_* calls are an optimization, not required by the frequency
 * switch. */
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

/* l681: the stock FY rule on efuse 21 / 19: func_code_0 1/3 FY, 2/4 SB,
 * else binLevel_eng field 2 FY, else SB */
static bool forge_l681_efuse_fy(void)
{
	unsigned int f0 = _GET_BITS_VAL_(7:0, get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX));
	unsigned int eng = _GET_BITS_VAL_(15:0, get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX_ENG));

	return f0 == 1 || f0 == 3 ||
	       (f0 != 2 && f0 != 4 &&
		(((eng >> 4) & 0x07) == 2 || ((eng >> 10) & 0x07) == 2));
}

/* boards whose rails this build drives: m681, and l681 (same MT6311 on
 * i2c-3 0x6b, same PL state, same stock FY table) only when its efuse
 * matches the stock FY ladder; else it stays frequency-only */
static bool forge_full_dvfs_board(void)
{
	if (of_machine_is_compatible("meizu,m681"))
		return true;
	return m3note_board_is(M3NOTE_BOARD_L681) && forge_l681_efuse_fy();
}


#define CPU_LEVEL_0             (0x0)
#define CPU_LEVEL_1             (0x1)
#define CPU_LEVEL_2             (0x2)
#define CPU_LEVEL_3             (0x3)
#define CPU_LV_TO_OPP_IDX(lv)   ((lv))	/* cpu_level to opp_idx */

#define cpu_dvfs_is_extbuck_valid()     (is_ext_buck_exist() && is_ext_buck_sw_ready())
unsigned int is_extbuck_valid;

#ifdef __KERNEL__
static unsigned int _mt_cpufreq_get_cpu_level(void)
{
	unsigned int lv = 0;
	unsigned int func_code_0 = _GET_BITS_VAL_(7:0, get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX));
	unsigned int func_code_1 = _GET_BITS_VAL_(31:28, get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX));
	unsigned int binLevel_eng = _GET_BITS_VAL_(15:0, get_devinfo_with_index(FUNC_CODE_EFUSE_INDEX_ENG));

	cpufreq_ver("from efuse: function code 0 = 0x%x, function code 1 = 0x%x, binLevel_eng = 0x%x\n",
		func_code_0,
		func_code_1,
		binLevel_eng);

#if FORGE_CPUFREQ_44_PATH
	/* The ext-buck test below lands on CPU_LEVEL_0 here (is_mt6311_exist()
	 * is a stub returning 0), i.e. the MT6353 FY ladder topping at big 1508.
	 * m681 is FY on MT6351 + MT6311: stock picks LEVEL_1 (big 1807 /
	 * LL 1001) from DT clock-frequency 1001. The efuse function code is not
	 * confirmed on this board, so the level is fixed to FY and the efuse
	 * verdict is only logged. Rows legal at 1.000 V are the same in both
	 * ladders (big 1196..663, LL 689..). */
	if (of_machine_is_compatible("meizu,m681")) {
		/* the DT-less half of the stock 4.9 rule below (-1: no verdict) */
		int efuse_lv = (func_code_0 == 1 || func_code_0 == 3) ? CPU_LEVEL_1 :
			       (func_code_0 == 2 || func_code_0 == 4) ? CPU_LEVEL_2 : -1;

		pr_info_once("[FORGE_M681] cpufreq: level 1 (FY) fixed; efuse func_code_0 0x%x would give %d\n",
			     func_code_0, efuse_lv);
		return CPU_LEVEL_1;
	}
	/*
	 * l681: FY too; stock runs it at big 1807 / LL 1001, the FY ladder. The
	 * stock rule (same efuse indices 21 / 19; its LEVEL_0 is FY, LEVEL_1 SB)
	 * is evaluated and logged as a check of the devinfo numbering: the stock
	 * DT says big 1950, so the efuse decides: func_code_0 1/3 FY, 2/4 SB,
	 * else binLevel_eng field 2 FY, else SB. A mismatch means the indices
	 * differ on this kernel; the rows below 1.000 V (big 1196.., LL 689..)
	 * are the same in every ladder.
	 */
	if (m3note_board_is(M3NOTE_BOARD_L681)) {
		bool fy = forge_l681_efuse_fy();

		pr_info_once("[FORGE_L681] cpufreq: level 1 (FY) fixed; stock rule on efuse 21 func_code_0 0x%x, 19 binLevel_eng 0x%x -> %s (%s the stock FY)\n",
			     func_code_0, binLevel_eng, fy ? "FY" : "SB",
			     fy ? "matches" : "MISMATCH with");
		return CPU_LEVEL_1;
	}
	if (cpu_dvfs_is_extbuck_valid()) {
		/* If FY */
		is_extbuck_valid = 1;
		/* is real 55s */
		if (func_code_0 == 0x82 || func_code_0 == 0x86)
			return CPU_LEVEL_2;
		else
			return CPU_LEVEL_0;
	} else
		return CPU_LEVEL_0;
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
#else
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

/* LL overclock rows above the stock FY 1001, all at the stock maximum
 * 1.150 V (the shared VPROC is never raised past it). Hidden unless
 * forge_ll_oc > 0 when cpufreq arms, and each one reachable only up to the
 * forge_ll_oc row (forge_row_ok). 1092/1144 keep the stock LL post-divider 2
 * (VCO 2184/2288; the stock SB table runs LL 1144 that way); 1196/1404/1599
 * use post-divider 1 like the L cluster's 1196..1807 rows (VCO 1196..1599). */
#define FORGE_LL_OC_ROWS	5
#define CPU_DVFS_FREQ_LL_OC5	(1599000)	/* KHz */
#define CPU_DVFS_FREQ_LL_OC4	(1404000)	/* KHz */
#define CPU_DVFS_FREQ_LL_OC3	(1196000)	/* KHz */
#define CPU_DVFS_FREQ_LL_OC2	(1144000)	/* KHz */
#define CPU_DVFS_FREQ_LL_OC1	(1092000)	/* KHz */

/* CPU LEVEL 1 of LL, FY segment (overclock rows first) */
static struct mt_cpu_freq_info opp_tbl_little_e1_0[] = {
	OP(CPU_DVFS_FREQ_LL_OC5, 115000),
	OP(CPU_DVFS_FREQ_LL_OC4, 115000),
	OP(CPU_DVFS_FREQ_LL_OC3, 115000),
	OP(CPU_DVFS_FREQ_LL_OC2, 115000),
	OP(CPU_DVFS_FREQ_LL_OC1, 115000),
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
	FP(CPU_DVFS_FREQ_LL_OC5,	1,	1,	2),
	FP(CPU_DVFS_FREQ_LL_OC4,	1,	1,	2),
	FP(CPU_DVFS_FREQ_LL_OC3,	1,	1,	2),
	FP(CPU_DVFS_FREQ_LL_OC2,	2,	1,	2),
	FP(CPU_DVFS_FREQ_LL_OC1,	2,	1,	2),
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

/* The governor-driven path of set_cur_freq() also runs adjust_clkdiv()
 * (TOP_CKDIV1) and adjust_cci_div()/_cci_clock_switch() for rows with
 * clk_div != 1 or cci_div != 2 (big FREQ7, LL FREQ5..7 of the selected
 * tables). Those transitions are not validated on this board, so hide
 * those rows: every reachable transition is a plain ARMPLL CON1 write
 * through MAINPLL, as in mt_dvfs_p0. */
#define FORGE_CPUFREQ_PROVEN_DIVS 1
/* ...and, like mt_dvfs_p0 (mt_dvfs_p0.c: "no CCI write is needed"), never
 * move the CCI mux: with FORGE_CPUFREQ_PROVEN_DIVS every reachable row has
 * cci_div 2, and the stock recipe's CCI parking/re-sourcing is not
 * validated on this board. */
#define FORGE_CPUFREQ_NO_CCI 1
static const struct mt_cpu_freq_method *forge_freq_method(enum mt_cpu_dvfs_id id,
							   unsigned int lv)
{
	if (id == MT_CPU_DVFS_BIG)
		return lv == CPU_LEVEL_0 ? opp_tbl_method_L_e0 :
		       lv == CPU_LEVEL_1 ? opp_tbl_method_L_e1 :
		       lv == CPU_LEVEL_2 ? opp_tbl_method_L_e2 : opp_tbl_method_L_e3;
	return lv == CPU_LEVEL_0 ? opp_tbl_method_LL_e0 :
	       lv == CPU_LEVEL_1 ? opp_tbl_method_LL_e1 :
	       lv == CPU_LEVEL_2 ? opp_tbl_method_LL_e2 : opp_tbl_method_LL_e3;
}

/* The cpufreq table clamp above only binds the governor. PPM limits, the
 * single-core pull-up, the hotplug notifier (last CPU down -> table min) and
 * CPU_ONLINE pick an opp index on their own and call set_cur_freq directly,
 * so they could still land on a hidden row: FREQ7 of both clusters needs
 * clk_div 2/4 and cci_div 4/6, never run on this board. Every index chosen
 * outside the governor goes through forge_snap_idx(). */
static int _sync_opp_tbl_idx(struct mt_cpu_dvfs *p);

/* the LL rows above stock FY are the first FORGE_LL_OC_ROWS of the
 * level-1 table; row r (0-based) is OC row FORGE_LL_OC_ROWS - r */
static bool forge_ll_oc_row_ok(struct mt_cpu_dvfs *p, int idx)
{
	if (!cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) || p->cpu_level != CPU_LEVEL_1 ||
	    idx >= FORGE_LL_OC_ROWS)
		return true;
	return READ_ONCE(forge_ll_oc_armed) &&
	       FORGE_LL_OC_ROWS - idx <= (int)READ_ONCE(forge_ll_oc);
}

static bool forge_row_ok(struct mt_cpu_dvfs *p, int idx)
{
	if (p->opp_tbl[idx].cpufreq_volt > forge_vmax())
		return false;
	if (!forge_ll_oc_row_ok(p, idx))
		return false;
	/* after a rail fault only the rows legal at the read-back voltage */
	if (READ_ONCE(forge_dvfs_fault) &&
	    p->opp_tbl[idx].cpufreq_volt > READ_ONCE(forge_vproc_cur))
		return false;
#if FORGE_CPUFREQ_PROVEN_DIVS
	{
		/* policy init asks before it assigns p->freq_tbl */
		const struct mt_cpu_freq_method *m = p->freq_tbl ? p->freq_tbl :
			forge_freq_method(cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) ?
					  MT_CPU_DVFS_LITTLE : MT_CPU_DVFS_BIG,
					  p->cpu_level);

		if (m[idx].clk_div != 1 || m[idx].cci_div != 2)
			return false;
	}
#endif
	return true;
}

/* nearest usable row: first toward lower frequency (higher index), then
 * toward higher frequency; a table with no usable row keeps idx. */
static int forge_snap_idx(struct mt_cpu_dvfs *p, int idx)
{
	int i;

	if (idx < 0 || idx >= p->nr_opp_tbl || forge_row_ok(p, idx))
		return idx;
	for (i = idx + 1; i < p->nr_opp_tbl; i++)
		if (forge_row_ok(p, i))
			return i;
	for (i = idx - 1; i >= 0; i--)
		if (forge_row_ok(p, i))
			return i;
	return idx;
}

/* CON1[26:24] posdiv shift -> divider (adjust_posdiv's encoding) */
static unsigned int forge_cur_posdiv(struct mt_cpu_dvfs *p)
{
	unsigned int shift = _GET_BITS_VAL_(26:24, cpufreq_read(p->armpll_addr));

	return shift == 1 ? 2 : shift == 2 ? 4 : 1;
}

/* CKDIV1[4:0] -> divider (adjust_clkdiv's encoding); the boot value 0
 *  dump: CKDIV1 big=0 sml=0) is the undivided clock like 8. */
static unsigned int forge_cur_clkdiv(struct mt_cpu_dvfs *p)
{
	unsigned int sel = _GET_BITS_VAL_(4:0, cpufreq_read(
		cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) ?
		INFRA_TOPCKGEN_CKDIV1_SML : INFRA_TOPCKGEN_CKDIV1_BIG));

	return sel == 10 ? 2 : sel == 11 ? 4 : 1;
}

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
 * structure as adjust_posdiv() below and the #if 0 stock adjust_armpll_dds()
 * minus its voltage moves: the cluster runs from MAINPLL for the
 * ~PLL_SETTLE_TIME of the CON1 write, so the CPU never runs from the PLL
 * mid-relock. posdiv bits (26:24) already in the register are preserved;
 * the caller strips them from dds and adjusts posdiv separately. */
static void forge_dfs_armpll(struct mt_cpu_dvfs *p, unsigned int dds)
{
	unsigned int con1;

	_cpu_clock_switch(p, TOP_CKMUXSEL_MAINPLL);
#if !FORGE_CPUFREQ_NO_CCI
	cci_switch_from_to(CUR_CCI_OPP_IDX, BACKUP_CCI_OPP_IDX);
#endif

	con1 = cpufreq_read(p->armpll_addr);
	cpufreq_write(p->armpll_addr,
		      (con1 & _BITMASK_(26:24)) | (dds & ~_BITMASK_(26:24)) | _BIT_(31)); /* CHG */
	udelay(PLL_SETTLE_TIME);

	_cpu_clock_switch(p, TOP_CKMUXSEL_ARMPLL);
#if !FORGE_CPUFREQ_NO_CCI
	cci_switch_from_to(BACKUP_CCI_OPP_IDX, CUR_CCI_OPP_IDX);
#endif
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
#if !FORGE_CPUFREQ_NO_CCI
	cci_switch_from_to(CUR_CCI_OPP_IDX, BACKUP_CCI_OPP_IDX);
#endif
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

#if !FORGE_CPUFREQ_NO_CCI
	cci_switch_from_to(BACKUP_CCI_OPP_IDX, CUR_CCI_OPP_IDX);
#endif
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

/* unused with FORGE_CPUFREQ_NO_CCI */
static void __maybe_unused cci_switch_from_to(enum opp_idx_type from, enum opp_idx_type to)
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
	unsigned int cur_pos_div, cur_clk_div;

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
	opp_tbl[TARGET_OPP_IDX].slot = &p->freq_tbl[idx];
	cpufreq_ver("[TARGET_OPP_IDX][NAME][IDX][FREQ] => %s:%d:%d\n",
		cpu_dvfs_get_name(p), idx, cpu_dvfs_get_freq_by_idx(p, idx));
	cur_pos_div = opp_tbl[CUR_OPP_IDX].slot->pos_div;
	cur_clk_div = opp_tbl[CUR_OPP_IDX].slot->clk_div;
#if FORGE_FREQ_ONLY
	/* last line: whatever path got here, no hidden row is programmed */
	if (!forge_row_ok(p, idx)) {
		pr_err_ratelimited("[FORGE_CPUFREQ] REFUSED %s row %d (%u kHz): hidden in this build\n",
				   cpu_dvfs_get_name(p), idx, target_khz);
		return;
	}
	/* voltage DVFS: the rails are raised before any PLL raise, so a row
	 * above the read-back VPROC is never a legitimate move (hotplug and
	 * procfs pick rows on their own) */
	/* "lowering" measured on the PLL: callers may pass a stale table
	 * frequency as cur_khz */
	if (forge_full_dvfs && p->opp_tbl[idx].cpufreq_volt > forge_vproc_cur &&
	    !(forge_dvfs_fault && target_khz < p->ops->get_cur_phy_freq(p))) {
		pr_err_ratelimited("[FORGE_CPUFREQ] REFUSED %s row %d (%u kHz): needs %u > VPROC %u\n",
				   cpu_dvfs_get_name(p), idx, target_khz,
				   p->opp_tbl[idx].cpufreq_volt, forge_vproc_cur);
		return;
	}
	/* divider state from the registers, not from the idx_opp_tbl slot:
	 * after boot (806 MHz is no table row) or a refused move the slot is
	 * only a guess, and set_cur_freq orders its steps by these two. */
	cur_pos_div = forge_cur_posdiv(p);
	cur_clk_div = forge_cur_clkdiv(p);
#endif

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
	if (cur_pos_div < opp_tbl[TARGET_OPP_IDX].slot->pos_div)
		adjust_posdiv(p, opp_tbl[TARGET_OPP_IDX].slot->pos_div);

	/* armpll_div 1 -> 2 */
	if (cur_clk_div < opp_tbl[TARGET_OPP_IDX].slot->clk_div)
		adjust_clkdiv(p, opp_tbl[TARGET_OPP_IDX].slot->clk_div);

#if !FORGE_CPUFREQ_NO_CCI
	/* cci_div 1 -> 2 */
	if (opp_tbl[CUR_CCI_OPP_IDX].slot->cci_div < opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div)
		adjust_cci_div(opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div);
#endif

	/* actual FHCTL */
	dds = _cpu_dds_calc(opp_tbl[TARGET_OPP_IDX].slot->vco_dds);
	/* adjust_armpll_dds(p, dds, opp_tbl[TARGET_OPP_IDX].slot->pos_div); */
	dds &= ~(_BITMASK_(26:24));
	/* FHCTL (mt_dfs_armpll, glitch-free slew) is not built: its mt6755
	 * driver drags in cpufreq_hybrid and dramc. Use the stock non-hopping
	 * recipe instead: park the cluster mux on MAINPLL, write CON1 with CHG,
	 * settle, mux back (the same sequence adjust_posdiv uses above). */
	forge_dfs_armpll(p, dds);

	/* switch CCI */
#if !FORGE_CPUFREQ_NO_CCI
	_cci_clock_switch(opp_tbl[TARGET_CCI_OPP_IDX].p->armpll_clk_src);
#endif

#if !FORGE_CPUFREQ_NO_CCI
	/* cci_div 2 -> 1 */
	if (opp_tbl[CUR_CCI_OPP_IDX].slot->cci_div > opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div)
		adjust_cci_div(opp_tbl[TARGET_CCI_OPP_IDX].slot->cci_div);
#endif

	/* armpll_div 2 -> 1 */
	if (cur_clk_div > opp_tbl[TARGET_OPP_IDX].slot->clk_div)
		adjust_clkdiv(p, opp_tbl[TARGET_OPP_IDX].slot->clk_div);

	/* post_div 2 -> 1 */
	if (cur_pos_div > opp_tbl[TARGET_OPP_IDX].slot->pos_div)
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

#if FORGE_CPUFREQ_44_PATH
/* for volt change (PMICWRAP/extBuck) */
static unsigned int get_cur_vsram(struct mt_cpu_dvfs *p)
{
	unsigned int rdata = 0;

	FUNC_ENTER(FUNC_LV_LOCAL);

#if FORGE_FREQ_ONLY
	/* the mt6353_upmu_* helper below carries MT6353 register addresses;
	 * the fitted PMIC is MT6351 — a pwrap read of the wrong register
	 * would return unrelated bits and feed the vsram>=vproc sanity BUG.
	 * Answer the PL value, or with voltage DVFS the last read-back. */
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
	/* mt6311_read_byte is a weak 0-stub and the pwrap fallback reads
	 * MT6353 addresses on an MT6351. Frequency-only: the rail is pinned by
	 * PL. Voltage DVFS: the code mt_extbuck_probe read back after its last
	 * write (0x8E = 0x90). */
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

/* called under the cpufreq mutex (_cpufreq_set_locked, reboot) */
static int forge_set_rails(unsigned int volt)
{
	unsigned int floor = min_t(unsigned int, READ_ONCE(forge_vproc_floor), 100000);
	unsigned int rb_p = forge_vproc_cur, rb_s = forge_vsram_cur;
	struct mt_cpu_dvfs *ll = id_to_cpu_dvfs(MT_CPU_DVFS_LITTLE);
	struct mt_cpu_dvfs *big = id_to_cpu_dvfs(MT_CPU_DVFS_BIG);
	int ret;

	/* never below what the clocks running right now need, whoever asks */
	volt = max(volt, _search_available_volt(ll, ll->ops->get_cur_phy_freq(ll)));
	volt = max(volt, _search_available_volt(big, big->ops->get_cur_phy_freq(big)));
	volt = clamp_t(unsigned int, max(volt, floor), 80000, 115000);
	/* after a fault no rail is written again: a raise is refused, a
	 * lower request leaves the (higher) rail where it is */
	if (forge_dvfs_fault)
		return volt > forge_vproc_cur ? -EIO : 0;
	if (volt == forge_vproc_cur)
		return 0;
	ret = forge_extbuck_set_rails ? forge_extbuck_set_rails(volt, &rb_p, &rb_s) : -ENODEV;
	forge_vproc_cur = rb_p;
	forge_vsram_cur = rb_s;
	if (ret && !forge_dvfs_fault) {
		forge_dvfs_fault = true;
		cpufreq_err("[FORGE_DVFS] rail write to %u failed (%d), read back VPROC %u VSRAM %u: fault latched, ceilings to the rows legal there\n",
			    volt, ret, rb_p, rb_s);
		forge_dvfs_fault_kick();
	}
	return ret;
}

#if FORGE_CPUFREQ_44_PATH
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
	/* The stock writers here are wrong or absent (VPROC setter = weak
	 * no-op, VSRAM helper = MT6353 addresses on MT6351). With voltage
	 * DVFS the m681 writer in mt_extbuck_probe moves both rails; else
	 * report success only because every reachable OPP runs at the fixed
	 * boot voltage (table clamp + set_locked guard). */
	if (forge_full_dvfs) {
		FUNC_EXIT(FUNC_LV_LOCAL);
		return forge_set_rails(volt);
	}
	pr_info_once("[FORGE_CPUFREQ] freq-only build: voltage moves are no-ops (VPROC fixed at PL 1.000 V)\n");
	FUNC_EXIT(FUNC_LV_LOCAL);
	return 0;
#endif

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
	/* Both write targets are wrong or absent in this build (VPROC setter =
	 * weak no-op, VSRAM helper = MT6353 addresses on MT6351). A silent
	 * partial actuation is the dangerous outcome — refuse the whole move
	 * and report success only because every reachable OPP runs at the
	 * fixed boot voltage (table clamp + set_locked guard). */
	pr_info_once("[FORGE_CPUFREQ] freq-only build: voltage moves are no-ops (VPROC fixed at PL 1.000 V)\n");
	FUNC_EXIT(FUNC_LV_LOCAL);
	return 0;
#endif

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

	#if FORGE_CPUFREQ_44_PATH
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
	/* last line of defence, on every path that can move a PLL: a row that
	 * needs more than the fixed boot voltage must never be entered while
	 * the voltage setters are no-ops — that combination is the brown-out.
	 * The clamped cpufreq table should make this unreachable; refusing
	 * loudly here converts a logic slip into a log line instead. */
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
		 * goes ahead without it (lower is never worse, and the
		 * fault cap must be able to bring both clusters down) */
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

#ifdef CONFIG_CPU_FREQ
	ret = _cpufreq_set_locked(p, cur_freq, target_freq, policy, target_volt, log);
#else
	ret = _cpufreq_set_locked(p, cur_freq, target_freq, NULL, target_volt, log);
#endif

#if FORGE_FREQ_ONLY
	/* a refused move left the PLL where it was: keep the index on the
	 * silicon, or the next set_cur_freq plans its divider steps from a
	 * row the cluster is not on. */
	if (ret)
		_sync_opp_tbl_idx(p);
	else
#endif
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
#if FORGE_FREQ_ONLY
					freq_idx = forge_snap_idx(p, freq_idx);
#endif
#ifdef CONFIG_CPU_FREQ
					policy = cpufreq_cpu_get(cpu);
					freqs.old = cpu_dvfs_get_cur_freq(p);
					freqs.new = cpu_dvfs_get_freq_by_idx(p, freq_idx);
					if (policy) {
						freqs.cpu = policy->cpu;
						cpufreq_freq_transition_begin(policy, &freqs);
					}
#endif
					p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p)
						, cpu_dvfs_get_freq_by_idx(p, freq_idx));
#ifdef CONFIG_CPU_FREQ
					if (policy) {
						freqs.cpu = policy->cpu;
						cpufreq_freq_transition_end(policy, &freqs, 0);
						cpufreq_cpu_put(policy);
					}
#endif
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
#if FORGE_FREQ_ONLY
				{
					int lo = forge_snap_idx(p, p->nr_opp_tbl - 1);

					p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p),
							     cpu_dvfs_get_freq_by_idx(p, lo));
					p->idx_opp_tbl = lo;
				}
#else
					p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p), cpu_dvfs_get_min_freq(p));

				p->idx_opp_tbl = p->nr_opp_tbl - 1;
#endif

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
#if FORGE_FREQ_ONLY
				freq_idx = forge_snap_idx(p, freq_idx);
#endif

#ifdef CONFIG_CPU_FREQ
				policy = cpufreq_cpu_get(cpu);
				freqs.old = cpu_dvfs_get_cur_freq(p);
				freqs.new = cpu_dvfs_get_freq_by_idx(p, freq_idx);
				if (policy) {
					freqs.cpu = policy->cpu;
					cpufreq_freq_transition_begin(policy, &freqs);
				}
#endif
				p->ops->set_cur_freq(p, cpu_dvfs_get_cur_freq(p)
					, cpu_dvfs_get_freq_by_idx(p, freq_idx));

#ifdef CONFIG_CPU_FREQ
				if (policy) {
					freqs.cpu = policy->cpu;
					cpufreq_freq_transition_end(policy, &freqs, 0);
					cpufreq_cpu_put(policy);
				}
#endif
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
 * cpu_dvfs_get_max_freq() reads opp_tbl[0], which on the clamped table is
 * a row the governor must never reach. */
static unsigned int forge_max_valid_freq(struct mt_cpu_dvfs *p)
{
#if FORGE_FREQ_ONLY
	int i;

	for (i = 0; i < p->nr_opp_tbl; i++)
		if (cpu_dvfs_get_volt_by_idx(p, i) <= forge_vmax() &&
		    (forge_ll_oc_armed || i >= FORGE_LL_OC_ROWS ||
		     !cpu_dvfs_is(p, MT_CPU_DVFS_LITTLE) || p->cpu_level != CPU_LEVEL_1))
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
		/* the stock row (598 MHz) behind the overclock rows on level 1 */
		int single = SIGLE_CORE_IDX +
			(p->cpu_level == CPU_LEVEL_1 ? FORGE_LL_OC_ROWS : 0);

		online_cpus = num_online_cpus() + num_online_cpus_delta;
		if (online_cpus == 1 && (new_opp_idx > single))
			new_opp_idx = single;
	}

	if ((p->idx_opp_ppm_limit != -1) && (new_opp_idx < p->idx_opp_ppm_limit))
		new_opp_idx = p->idx_opp_ppm_limit;

	if ((p->idx_opp_ppm_base != -1) && (new_opp_idx > p->idx_opp_ppm_base))
		new_opp_idx = p->idx_opp_ppm_base;

	if ((p->idx_opp_ppm_base == p->idx_opp_ppm_limit) && p->idx_opp_ppm_base != -1)
		new_opp_idx = p->idx_opp_ppm_base;

#if FORGE_FREQ_ONLY
	new_opp_idx = forge_snap_idx(p, new_opp_idx);
#endif

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
	    || !id_to_cpu_dvfs(id) || !policy->freq_table
	    || id_to_cpu_dvfs(id)->dvfs_disable_by_procfs
		/* || (id_to_cpu_dvfs(id) && id_to_cpu_dvfs(id)->dvfs_disable_by_suspend) */
	    )
		return -EINVAL;
	/* Linux 4.9 uses policy->freq_table and returns the selected index. */
	ret = cpufreq_frequency_table_target(policy, target_freq, relation);
	if (ret < 0)
		return ret;
	new_opp_idx = ret;
	ret = 0;
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
#if FORGE_FREQ_ONLY
		policy->cpuinfo.min_freq = cpu_dvfs_get_freq_by_idx(p,
			forge_snap_idx(p, p->nr_opp_tbl - 1));
#endif

		policy->cur = p->ops->get_cur_phy_freq(p);	/* use cur phy freq is better */
		if (p->idx_opp_ppm_limit == -1)
			policy->max = forge_max_valid_freq(id_to_cpu_dvfs(id));
		else
			policy->max = cpu_dvfs_get_freq_by_idx(id_to_cpu_dvfs(id), p->idx_opp_ppm_limit);

		if (p->idx_opp_ppm_base == -1)
			policy->min = policy->cpuinfo.min_freq;
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
				/* LL overclock rows only when armed with forge_ll_oc > 0 */
				if (j == MT_CPU_DVFS_LITTLE && lv == CPU_LEVEL_1 &&
				    i < FORGE_LL_OC_ROWS && !forge_ll_oc_armed)
					table[i].frequency = CPUFREQ_ENTRY_INVALID;
#endif
#if FORGE_FREQ_ONLY && FORGE_CPUFREQ_PROVEN_DIVS
				{
					const struct mt_cpu_freq_method *m = forge_freq_method(j, lv);

					if (m[i].clk_div != 1 || m[i].cci_div != 2)
						table[i].frequency = CPUFREQ_ENTRY_INVALID;
				}
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

#if FORGE_CPUFREQ_44_PATH
int mt_cpufreq_get_chip_id_38(void)
{
	unsigned int chip_code = get_devinfo_with_index(21) & 0xFF;

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
	int freq, i, found = 0;
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

			/* this path moves the PLL without the rails: with
			 * voltage DVFS set_cur_freq() refuses rows above the
			 * read-back VPROC (checked under the lock there) */
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
static int mt_cpufreq_map_resource(const char *compatible, unsigned long *base)
{
	struct device_node *node;

	/* A previous, unsuccessful arm attempt may have mapped this resource. */
	if (*base)
		return 0;
	node = of_find_compatible_node(NULL, NULL, compatible);
	if (!node) {
		cpufreq_err("cannot find %s\n", compatible);
		return -ENODEV;
	}
	*base = (unsigned long)of_iomap(node, 0);
	of_node_put(node);
	if (!*base) {
		cpufreq_err("cannot map %s\n", compatible);
		return -ENOMEM;
	}
	return 0;
}

static int mt_cpufreq_dts_map(void)
{
	int ret;

	ret = mt_cpufreq_map_resource(TOPCKGEN_NODE, &topckgen_base);
	if (ret)
		return ret;
	ret = mt_cpufreq_map_resource(INFRACFG_AO_NODE, &infracfg_ao_base);
	if (ret)
		return ret;
	return mt_cpufreq_map_resource(APMIXED_NODE, &apmixed_base);
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
/* Off at boot; writing 1 to
 * /sys/module/mt_cpufreq/parameters/forge_mt_cpufreq runs the full init once:
 * dts_map -> procfs -> platform device+driver -> probe -> clamped tables ->
 * cpufreq_register_driver -> policies come up under the default governor.
 * One-way: disarm = reboot (LK restores boot frequencies). */
int forge_mt_cpufreq_armed;	/* non-static: mt_dvfs_p0.c interlock */

/*
 * Policy limits. One CPUFREQ_ADJUST notifier per cluster policy:
 *   max = min(forge_cpu_max_{ll,big}_khz, ATM cap, rail-fault cap);
 *   forge_cpu_perf = 1: min = max (stock "performance" under any governor);
 *   touch boost (auto mode): min = max for forge_touch_boost_ms after a touch.
 * Later notifiers (forge_therm_guard) only lower max further and
 * cpufreq_verify_within_limits() pulls min down with it, so thermal
 * throttling stays in force in performance mode and during a boost.
 * The boot ceiling is 1196/689 (the rows of the PL rail); the ROM raises it
 * after boot_completed (persist.vendor.m681.cpu_max), so a hang at a high
 * row never repeats on the next boot.
 */
static unsigned int forge_cpu_max_khz[NR_MT_CPU_DVFS] = {
	[MT_CPU_DVFS_LITTLE] = 689000,
	[MT_CPU_DVFS_BIG] = 1196000,
};
static unsigned int forge_thermal_cap_khz[NR_MT_CPU_DVFS];	/* 0 = none */
static bool forge_cpu_perf;
static unsigned int forge_touch_boost_ms = 150;
static unsigned long forge_touch_boost_until;	/* jiffies */
static struct delayed_work forge_touch_boost_end;

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

/* highest row legal at the read-back VPROC after a rail fault */
static unsigned int forge_fault_cap_khz(struct mt_cpu_dvfs *p)
{
	int i;

	for (i = 0; i < p->nr_opp_tbl; i++)
		if (cpu_dvfs_get_volt_by_idx(p, i) <= forge_vproc_cur &&
		    forge_row_ok(p, i))
			return cpu_dvfs_get_freq_by_idx(p, i);
	return cpu_dvfs_get_freq_by_idx(p, forge_snap_idx(p, p->nr_opp_tbl - 1));
}

static void forge_dvfs_fault_kick(void)
{
	schedule_work(&forge_limits_work);	/* not under the cpufreq mutex */
}

/* highest usable row at or below khz (the lowest usable row if none) */
static unsigned int forge_row_at_or_below(struct mt_cpu_dvfs *p, unsigned int khz)
{
	int i;

	for (i = 0; i < p->nr_opp_tbl; i++)
		if (cpu_dvfs_get_freq_by_idx(p, i) <= khz && forge_row_ok(p, i))
			return cpu_dvfs_get_freq_by_idx(p, i);
	return cpu_dvfs_get_freq_by_idx(p, forge_snap_idx(p, p->nr_opp_tbl - 1));
}

static int forge_limits_notifier(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct cpufreq_policy *policy = data;
	enum mt_cpu_dvfs_id id;
	struct mt_cpu_dvfs *p;
	unsigned int cap;

	/* registered at arm, before the policies exist: the first policy
	 * already starts under the boot ceiling */
	if (event != CPUFREQ_ADJUST)
		return NOTIFY_DONE;
	id = _get_cpu_dvfs_id(policy->cpu);
	p = id_to_cpu_dvfs(id);
	cap = READ_ONCE(forge_cpu_max_khz[id]);
	/* LL rows above the forge_ll_oc one (1001 when 0) */
	if (id == MT_CPU_DVFS_LITTLE && p->cpu_level == CPU_LEVEL_1 && p->opp_tbl)
		cap = min(cap, p->opp_tbl[FORGE_LL_OC_ROWS -
			  min_t(unsigned int, READ_ONCE(forge_ll_oc), FORGE_LL_OC_ROWS)].cpufreq_khz);
	if (READ_ONCE(forge_thermal_cap_khz[id]))
		cap = min(cap, READ_ONCE(forge_thermal_cap_khz[id]));
	if (READ_ONCE(forge_dvfs_fault))
		cap = min(cap, forge_fault_cap_khz(p));
	cpufreq_verify_within_limits(policy, 0, cap);
	/* min = max must name a table row, else the table check lifts max
	 * to the next row up, past the ceilings */
	cap = forge_row_at_or_below(p, policy->max);
	cpufreq_verify_within_limits(policy, 0, cap);
	if (READ_ONCE(forge_cpu_perf) ||
	    (READ_ONCE(forge_touch_boost_ms) &&
	     time_before(jiffies, READ_ONCE(forge_touch_boost_until))))
		cpufreq_verify_within_limits(policy, cap, cap);
	return NOTIFY_OK;
}

static struct notifier_block forge_limits_nb = {
	.notifier_call = forge_limits_notifier,
};

/* the PPM-table shim in forge_ppm_thermal.c calls this (kHz, 0 = none) */
void forge_cpufreq_thermal_cap(unsigned int ll_khz, unsigned int big_khz)
{
	if (READ_ONCE(forge_thermal_cap_khz[MT_CPU_DVFS_LITTLE]) == ll_khz &&
	    READ_ONCE(forge_thermal_cap_khz[MT_CPU_DVFS_BIG]) == big_khz)
		return;
	WRITE_ONCE(forge_thermal_cap_khz[MT_CPU_DVFS_LITTLE], ll_khz);
	WRITE_ONCE(forge_thermal_cap_khz[MT_CPU_DVFS_BIG], big_khz);
	schedule_work(&forge_limits_work);
}
EXPORT_SYMBOL(forge_cpufreq_thermal_cap);

static int forge_limits_param_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	if (!ret)
		schedule_work(&forge_limits_work);
	return ret;
}

static int forge_limits_bool_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret)
		schedule_work(&forge_limits_work);
	return ret;
}

static const struct kernel_param_ops forge_limits_uint_ops = {
	.set = forge_limits_param_set,
	.get = param_get_uint,
};
static const struct kernel_param_ops forge_limits_bool_ops = {
	.set = forge_limits_bool_set,
	.get = param_get_bool,
};
module_param_cb(forge_cpu_max_ll_khz, &forge_limits_uint_ops,
		&forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], 0644);
module_param_cb(forge_cpu_max_big_khz, &forge_limits_uint_ops,
		&forge_cpu_max_khz[MT_CPU_DVFS_BIG], 0644);
module_param_cb(forge_cpu_perf, &forge_limits_bool_ops, &forge_cpu_perf, 0644);
module_param_cb(forge_ll_oc, &forge_limits_uint_ops, &forge_ll_oc, 0644);
module_param(forge_touch_boost_ms, uint, 0644);
module_param(forge_full_dvfs, bool, 0444);
module_param(forge_dvfs_fault, bool, 0444);
module_param(forge_vproc_cur, uint, 0444);
module_param(forge_vsram_cur, uint, 0444);

/* touch boost: any touchscreen event starts or extends the window */
static void forge_touch_boost_end_fn(struct work_struct *work)
{
	forge_limits_update();
}

static void forge_touch_event(struct input_handle *handle, unsigned int type,
			      unsigned int code, int value)
{
	unsigned int ms = READ_ONCE(forge_touch_boost_ms);
	bool was;

	/* once per input frame */
	if (type != EV_SYN || code != SYN_REPORT)
		return;
	if (!ms || !forge_mt_cpufreq_armed || READ_ONCE(forge_cpu_perf))
		return;
	was = time_before(jiffies, READ_ONCE(forge_touch_boost_until));
	WRITE_ONCE(forge_touch_boost_until, jiffies + msecs_to_jiffies(ms));
	if (!was)
		schedule_work(&forge_limits_work);
	mod_delayed_work(system_wq, &forge_touch_boost_end,
			 msecs_to_jiffies(ms) + 1);
}

static int forge_touch_connect(struct input_handler *handler,
			       struct input_dev *dev,
			       const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "forge_touch_boost";
	ret = input_register_handle(handle);
	if (ret)
		goto err_free;
	ret = input_open_device(handle);
	if (ret)
		goto err_unreg;
	return 0;
err_unreg:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void forge_touch_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id forge_touch_ids[] = {
	{	/* multi-touch screens */
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) },
	},
	{ },
};

static struct input_handler forge_touch_handler = {
	.event = forge_touch_event,
	.connect = forge_touch_connect,
	.disconnect = forge_touch_disconnect,
	.name = "forge_touch_boost",
	.id_table = forge_touch_ids,
};

/* before reboot/power-off: back to the boot ceiling and the PL rail, so
 * whatever starts next (LK, PL with or without a rail reset) meets the
 * voltage it booted on or more */
static int forge_dvfs_reboot(struct notifier_block *nb, unsigned long code,
			     void *unused)
{
	unsigned long flags;

	/* rails taken but the driver never came up: no clock to read */
	if (!forge_full_dvfs || !forge_mt_cpufreq_armed)
		return NOTIFY_DONE;
	WRITE_ONCE(forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], 689000);
	WRITE_ONCE(forge_cpu_max_khz[MT_CPU_DVFS_BIG], 1196000);
	WRITE_ONCE(forge_cpu_perf, false);
	WRITE_ONCE(forge_touch_boost_ms, 0);
	WRITE_ONCE(forge_vproc_floor, 100000);
	forge_limits_update();	/* the governors step down to <= 1196/689 */
	/* park at 1.000 V, or at what the clocks still running need if a
	 * governor did not step down (forge_set_rails never goes below it) */
	cpufreq_lock(flags);
	forge_set_rails(100000);
	/* variant A: back to the software VOSEL PL left, if the codes match */
	if (forge_extbuck_dvfs_release)
		forge_extbuck_dvfs_release();
	cpufreq_unlock(flags);
	pr_err("[FORGE_DVFS] reboot: ceiling 1196/689, VPROC %u VSRAM %u\n",
	       forge_vproc_cur, forge_vsram_cur);
	return NOTIFY_DONE;
}

static struct notifier_block forge_dvfs_reboot_nb = {
	.notifier_call = forge_dvfs_reboot,
};

/* at arm: read-only rail dump, then try to take the rails */
static void forge_dvfs_arm(void)
{
	static bool done;	/* a failed arm may be retried by hand */
	unsigned int vp = 0, vs = 0;
	int ret;

	if (done || !FORGE_M681_FULL_DVFS)
		return;
	if (!forge_full_dvfs_board()) {
		if (m3note_board_is(M3NOTE_BOARD_L681))
			pr_err("[FORGE_DVFS] l681: efuse does NOT match the stock FY ladder - no voltage DVFS, freq-only at the PL rail\n");
		return;
	}
	done = true;
	if (forge_extbuck_sh0)
		forge_extbuck_sh0();
	ret = forge_extbuck_dvfs_init ? forge_extbuck_dvfs_init(&vp, &vs) : -ENODEV;
	if (!ret && forge_extbuck_set_rails) {
		forge_vproc_cur = vp;
		forge_vsram_cur = vs;
		forge_full_dvfs = true;
	}
	/* the LL overclock rows are characterised on m681 only */
	forge_ll_oc_armed = forge_full_dvfs && forge_ll_oc > 0 &&
			    of_machine_is_compatible("meizu,m681");
	INIT_DELAYED_WORK(&forge_touch_boost_end, forge_touch_boost_end_fn);
	cpufreq_register_notifier(&forge_limits_nb, CPUFREQ_POLICY_NOTIFIER);
	register_reboot_notifier(&forge_dvfs_reboot_nb);
	if (input_register_handler(&forge_touch_handler))
		pr_err("[FORGE_DVFS] touch boost handler not registered\n");
	pr_err("[FORGE_DVFS] %s: VPROC %u VSRAM %u (mv*100), floor %u, ceiling LL %u big %u, LL OC rows %s (%u)\n",
	       forge_full_dvfs ? "voltage DVFS ON" : "voltage DVFS OFF (freq-only)",
	       forge_vproc_cur, forge_vsram_cur, forge_vproc_floor,
	       forge_cpu_max_khz[MT_CPU_DVFS_LITTLE], forge_cpu_max_khz[MT_CPU_DVFS_BIG],
	       forge_ll_oc_armed ? "on" : "off", forge_ll_oc);
}


static int _mt_cpufreq_pdrv_init_real(void)
{
	int ret = 0;
	struct cpumask cpu_mask;
	unsigned int cluster_num;
	int i;

	FUNC_ENTER(FUNC_LV_MODULE);

	ret = mt_cpufreq_dts_map();
	if (ret)
		return ret;
	debug_vsram = get_cur_vsram(NULL);
	debug_vproc = get_cur_volt_extbuck(NULL);

	cluster_num = (unsigned int)arch_get_nr_clusters();
	if (!cluster_num || cluster_num > ARRAY_SIZE(cpu_dvfs))
		return -EINVAL;
	for (i = 0; i < cluster_num; i++) {
		arch_get_cluster_cpus(&cpu_mask, i);
		if (cpumask_empty(&cpu_mask))
			return -ENODEV;
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
	ret = _create_procfs();
	if (ret)
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

/* The cpufreq policy kobjects are not devices, so dev_uevent_filter drops
 * their KOBJ_ADD and ueventd never applies the policy* rules: after the arm
 * policyN/scaling_min_freq stayed root 0644 and the Power HAL (user system)
 * got EACCES. A change event on every online cpuN device makes ueventd apply
 * the cpu* cpufreq/... rules of /system/etc/ueventd.rc (checked on the M681:
 * echo change > cpu0/uevent and cpu4/uevent set both policies to system
 * 0664). */
static void forge_cpufreq_cpu_uevents(void)
{
	struct device *dev;
	unsigned int cpu;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		dev = get_cpu_device(cpu);
		if (dev)
			kobject_uevent(&dev->kobj, KOBJ_CHANGE);
	}
	put_online_cpus();
}

static int forge_mt_cpufreq_set(const char *val, const struct kernel_param *kp)
{
	extern int forge_dvfs_p0;	/* mt_dvfs_p0.c manual phase-0 knob */
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

	forge_dvfs_arm();
	ret = _mt_cpufreq_pdrv_init_real();
	if (ret) {
		pr_err("[FORGE_CPUFREQ] arm FAILED ret=%d\n", ret);
		return ret;
	}
	forge_mt_cpufreq_armed = 1;
	forge_limits_update();	/* the policies came up before armed was set */
	pr_err("[FORGE_CPUFREQ] ARMED: governor DVFS, %s\n",
	       forge_full_dvfs ? "voltage DVFS (stock FY), boot ceiling big 1196000 LL 689000 kHz" :
	       "freq-only, big<=1196000 LL<=689000 kHz @ fixed 1.000 V");
	forge_cpufreq_cpu_uevents();
	pr_debug("cpufreq: cpu change uevents sent\n");
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

/* Arm the frequency-only governor automatically after userspace has
 * started. LK cuts the boot.img cmdline at ~99 bytes, so this default
 * lives here; forge_mt_cpufreq_autoarm=0 (sysfs, before the delay runs)
 * keeps manual arming. On both M681 and L681 LK leaves MT6311 VPROC at
 * 0x40 = 1.000 V, where big <= 1196 and LL <= 689 are legal. The
 * forge_dvfs_arm() rail writer runs on m681, and on l681 only when its
 * efuse matches the stock FY ladder (forge_full_dvfs_board()); otherwise
 * voltages are not touched (FORGE_FREQ_ONLY). */
static bool forge_mt_cpufreq_autoarm = true;
module_param(forge_mt_cpufreq_autoarm, bool, 0644);
MODULE_PARM_DESC(forge_mt_cpufreq_autoarm,
		 "M681/L681: arm the freq-only cpufreq forge_mt_cpufreq_autoarm_delay_s after late init (default 1)");

/* Arm well after boot_completed, so that a hang at the first frequency
 * switch meets a live adb and UI instead of a boot in progress. */
static unsigned int forge_mt_cpufreq_autoarm_delay_s = 150;
module_param(forge_mt_cpufreq_autoarm_delay_s, uint, 0444);

static void forge_mt_cpufreq_autoarm_fn(struct work_struct *work)
{
	int ret;

	if (!forge_mt_cpufreq_autoarm || forge_mt_cpufreq_armed)
		return;
	ret = forge_mt_cpufreq_set("1", NULL);
	pr_err("[FORGE_CPUFREQ] auto-arm on %s: %d\n",
	       m3note_board_is(M3NOTE_BOARD_L681) ? "L681" : "M681", ret);
}
static DECLARE_DELAYED_WORK(forge_mt_cpufreq_autoarm_work,
			    forge_mt_cpufreq_autoarm_fn);

static int __init _mt_cpufreq_pdrv_init(void)
{
	pr_info("[FORGE_CPUFREQ] built (freq-only, clamped); arm: echo 1 > /sys/module/mt_cpufreq/parameters/forge_mt_cpufreq\n");
	if (of_machine_is_compatible("meizu,m681") ||
	    m3note_board_is(M3NOTE_BOARD_L681))
		schedule_delayed_work(&forge_mt_cpufreq_autoarm_work,
				      forge_mt_cpufreq_autoarm_delay_s * HZ);
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
