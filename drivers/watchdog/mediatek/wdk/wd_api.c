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

#include <linux/kernel.h>
#include <ext_wd_drv.h>
#include <mach/wd_api.h>
#include <linux/smp.h>
/*add by debug for register restart notify*/
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <mt-plat/mtk_reboot.h>
#include <mt-plat/mtk_rtc.h>

static int wd_cpu_hot_plug_on_notify(int cpu);
static int wd_cpu_hot_plug_off_notify(int cpu);
static int spmwdt_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode);
static int thermal_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode);
static int confirm_hwreboot(void);
static void resume_notify(void);
static void suspend_notify(void);
static int mtk_wk_wdt_config(enum ext_wdt_mode mode, int timeout_val);
static unsigned int wd_get_check_bit(void);
static unsigned int wd_get_kick_bit(void);
static int disable_all_wd(void);
static int disable_ext(void);
static int disable_local(void);
static int wd_sw_reset(int type);
static int wd_restart(enum wd_restart_type type);
static int set_mode(enum ext_wdt_mode mode);
static int wd_dram_reserved_mode(bool enabled);
static int wd_mcu_cache_preserve(bool enabled);
static int thermal_direct_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode);
static int debug_key_eint_config(WD_REQ_CTL en, WD_REQ_MODE mode);
static int debug_key_sysrst_config(WD_REQ_CTL en, WD_REQ_MODE mode);
static int dfd_count_en(int value);
static int dfd_thermal1_dis(int value);
static int dfd_thermal2_dis(int value);
static int dfd_timeout(int value);

static struct wd_api g_wd_api_obj = {
	.ready = 1,
	.wd_cpu_hot_plug_on_notify = wd_cpu_hot_plug_on_notify,	/* for cpu hot plug */
	.wd_cpu_hot_plug_off_notify = wd_cpu_hot_plug_off_notify,	/* for cpu hot plug */
	.wd_spmwdt_mode_config = spmwdt_mode_config,
	.wd_thermal_mode_config = thermal_mode_config,
	.wd_sw_reset = wd_sw_reset,
	.wd_restart = wd_restart,
	.wd_config = mtk_wk_wdt_config,
	.wd_set_mode = set_mode,
	.wd_aee_confirm_hwreboot = confirm_hwreboot,
	.wd_disable_ext = disable_ext,
	.wd_disable_local = disable_local,
	.wd_suspend_notify = suspend_notify,
	.wd_resume_notify = resume_notify,
	.wd_disable_all = disable_all_wd,
	.wd_get_check_bit = wd_get_check_bit,
	.wd_get_kick_bit = wd_get_kick_bit,
	.wd_dram_reserved_mode = wd_dram_reserved_mode,
	.wd_mcu_cache_preserve = wd_mcu_cache_preserve,
	.wd_thermal_direct_mode_config = thermal_direct_mode_config,
	.wd_debug_key_eint_config = debug_key_eint_config,
	.wd_debug_key_sysrst_config = debug_key_sysrst_config,
	.wd_dfd_count_en = dfd_count_en,
	.wd_dfd_thermal1_dis = dfd_thermal1_dis,
	.wd_dfd_thermal2_dis = dfd_thermal2_dis,
	.wd_dfd_timeout = dfd_timeout,
};

/* struct wd_private_api  *g_wd_private_api_obj; */



/* apiimplimentation */
#ifdef CONFIG_MTK_WD_KICKER

static unsigned int wd_get_check_bit(void)
{
	return get_check_bit();
}

static unsigned int wd_get_kick_bit(void)
{
	return get_kick_bit();
}


static int wd_restart(enum wd_restart_type type)
{
#ifdef	CONFIG_LOCAL_WDT
#ifdef CONFIG_SMP
	on_each_cpu((smp_call_func_t) mpcore_wdt_restart, WD_TYPE_NORMAL, 0);
#else
	mpcore_wdt_restart(type);
#endif
#endif
	mtk_wdt_restart(type);
	return 0;
}


static int wd_cpu_hot_plug_on_notify(int cpu)
{
	int res = 0;

	wk_cpu_update_bit_flag(cpu, 1);
	mtk_wdt_restart(WD_TYPE_NOLOCK);	/* for KICK external wdt */
	pr_alert("WD wd_cpu_hot_plug_on_notify kick ext wd\n");

	return res;
}

static int wd_cpu_hot_plug_off_notify(int cpu)
{
	int res = 0;

	wk_cpu_update_bit_flag(cpu, 0);
	return res;
}

static int wd_sw_reset(int type)
{
	wdt_arch_reset(type);
	return 0;
}

static int mtk_wk_wdt_config(enum ext_wdt_mode mode, int timeout_val)
{

	mtk_wdt_mode_config(TRUE, TRUE, TRUE, FALSE, TRUE);
	mtk_wdt_set_time_out_value(timeout_val);
#ifdef	CONFIG_LOCAL_WDT
	mpcore_wk_wdt_config(0, 0, timeout_val - 5);	/* local 25s time out */
	/* mpcore_wdt_set_heartbeat(timeout_val - 5);//local 25s time out */
#endif

	return 0;
}

static int disable_ext(void)
{
	mtk_wdt_enable(WK_WDT_DIS);
	return 0;
}


static int disable_local(void)
{
#ifdef CONFIG_LOCAL_WDT
#ifdef CONFIG_SMP
	on_each_cpu((smp_call_func_t) local_wdt_enable, WK_WDT_DIS, 0);
#else
	local_wdt_enable(WK_WDT_DIS);
#endif
#endif
	pr_debug(" wd_api disable_local not support now\n");
	return 0;
}

static int set_mode(enum ext_wdt_mode mode)
{
	pr_debug("  support only irq mode-20140522");
	switch (mode) {
	case WDT_DUAL_MODE:
		break;

	case WDT_HW_REBOOT_ONLY_MODE:
		break;

	case WDT_IRQ_ONLY_MODE:
		pr_debug("wd set only irq mode for debug\n");
		mtk_wdt_mode_config(FALSE, TRUE, TRUE, FALSE, TRUE);
		break;
	}

	return 0;

}

static int confirm_hwreboot(void)
{
	mtk_wdt_confirm_hwreboot();
	return 0;
}

static void suspend_notify(void)
{
	mtk_wd_suspend();
}

static void resume_notify(void)
{
	mtk_wd_resume();
}

static int disable_all_wd(void)
{
	disable_ext();
#ifdef CONFIG_LOCAL_WDT
	disable_local();
#endif
	return 0;
}

static int spmwdt_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	if (en == WD_REQ_EN) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_SCPSYS_MARK,WD_REQ_EN); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SPM_SCPSYS_MARK, WD_REQ_EN);
	} else if (en == WD_REQ_DIS) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_SCPSYS_MARK,WD_REQ_DIS); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SPM_SCPSYS_MARK, WD_REQ_DIS);
	} else {
		res = -2;
	}

	if (mode == WD_REQ_IRQ_MODE) {
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SPM_SCPSYS_MARK, WD_REQ_IRQ_MODE);
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_SCPSYS_MARK,WD_REQ_IRQ_MODE); */
	} else if (mode == WD_REQ_RST_MODE) {
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SPM_SCPSYS_MARK, WD_REQ_RST_MODE);
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_SCPSYS_MARK,WD_REQ_RST_MODE); */
	} else {
		res = -3;
	}
	return res;
}

static int thermal_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	if (en == WD_REQ_EN) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_EN); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK, WD_REQ_EN);
	} else if (en == WD_REQ_DIS) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_DIS); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK, WD_REQ_DIS);
	} else {
		res = -2;
	}

	if (mode == WD_REQ_IRQ_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_IRQ_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK, WD_REQ_IRQ_MODE);
	} else if (mode == WD_REQ_RST_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_RST_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK, WD_REQ_RST_MODE);
	} else {
		res = -3;
	}
	return res;
}

static int thermal_direct_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("thermal_direct_mode_config(en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_EN); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_EN);
	} else if (en == WD_REQ_DIS) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_DIS); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_DIS);
	} else {
		res = -2;
	}

	if (mode == WD_REQ_IRQ_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_IRQ_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_IRQ_MODE);
	} else if (mode == WD_REQ_RST_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_RST_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_RST_MODE);
	} else {
		res = -3;
	}
	return res;
}


static int wd_dram_reserved_mode(bool enabled)
{
	int ret = 0;

	if (true == enabled) {
		mtk_wdt_swsysret_config(0x10000000, 1);
		mtk_rgu_dram_reserved(1);
	} else {
		mtk_wdt_swsysret_config(0x10000000, 0);
		mtk_rgu_dram_reserved(0);
	}
	return ret;
}

static int wd_mcu_cache_preserve(bool enabled)
{
	int ret = 0;

	if (true == enabled)
		mtk_rgu_mcu_cache_preserve(1);
	else
		mtk_rgu_mcu_cache_preserve(0);

	return ret;
}

static int debug_key_eint_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("debug_key_eint_config(en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_EN);
	else if (en == WD_REQ_DIS)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_DIS);
	else
		res = -2;

	if (mode == WD_REQ_IRQ_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_IRQ_MODE);
	else if (mode == WD_REQ_RST_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_RST_MODE);
	else
		res = -3;
	return res;
}

static int debug_key_sysrst_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("debug_key_sysrst_config(en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_EN);
	else if (en == WD_REQ_DIS)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_DIS);
	else
		res = -2;

	if (mode == WD_REQ_IRQ_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_IRQ_MODE);
	else if (mode == WD_REQ_RST_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_RST_MODE);
	else
		res = -3;
	return res;
}

#else
/* dummy api */

static unsigned int wd_get_check_bit(void)
{
	pr_debug("dummy wd_get_check_bit");
	return 0;
}

static unsigned int wd_get_kick_bit(void)
{
	pr_debug("dummy wd_get_kick_bit");
	return 0;
}

static int wd_restart(enum wd_restart_type type)
{
	pr_debug("dummy wd_restart");
	return 0;
}


static int wd_cpu_hot_plug_on_notify(int cpu)
{
	int res = 0;

	pr_debug("dummy wd_cpu_hot_plug_on_notify");
	return res;
}

static int wd_cpu_hot_plug_off_notify(int cpu)
{
	int res = 0;

	pr_debug("dummy wd_cpu_hot_plug_off_notify");
	return res;
}

static int wd_sw_reset(int type)
{
	pr_debug("dummy wd_sw_reset");
	#ifndef CONFIG_MEDIATEK_WATCHDOG
	wdt_arch_reset(type);
	#endif
	return 0;
}

static int mtk_wk_wdt_config(enum ext_wdt_mode mode, int timeout_val)
{

	pr_debug("dummy mtk_wk_wdt_config");
	return 0;
}

static int disable_ext(void)
{
	pr_debug("dummy disable_ext");
	return 0;
}

static int disable_local(void)
{
	pr_debug("dummy disable_local");
	return 0;
}

static int set_mode(enum ext_wdt_mode mode)
{
	pr_debug("dummy set_mode");
	return 0;

}

static int confirm_hwreboot(void)
{
	pr_debug("dummy confirm_hwreboot");
	return 0;
}

static void suspend_notify(void)
{
	pr_debug("dummy suspend_notify\n");
}

static void resume_notify(void)
{

	pr_debug("dummy resume_notify\n");

}

static int disable_all_wd(void)
{
	pr_debug("dummy disable_all_wd\n");
	return 0;
}

static int spmwdt_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("dummy spmwdt_mode_config\n");
	return res;
}

static int thermal_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("dummy thermal_mode_config\n");
	return res;
}

static int wd_dram_reserved_mode(bool enabled)
{
	int res = 0;

	pr_debug("dummy wd_dram_reserved_mode\n");
	return res;
}

static int wd_mcu_cache_preserve(bool enabled)
{
	int res = 0;

	pr_debug("dummy wd_mcu_cache_preserve\n");
	return res;
}

static int thermal_direct_mode_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("thermal_direct_mode_config in dummy driver (en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_EN); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_EN);
	} else if (en == WD_REQ_DIS) {
		/* g_ext_wd_drv.reques_en_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_DIS); */
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_DIS);
	} else {
		res = -2;
	}

	if (mode == WD_REQ_IRQ_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_IRQ_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_IRQ_MODE);
	} else if (mode == WD_REQ_RST_MODE) {
		/* g_ext_wd_drv.reques_mode_set(MTK_WDT_REQ_SPM_THERMAL_MARK,WD_REQ_RST_MODE); */
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_THERMAL_MARK, WD_REQ_RST_MODE);
	} else {
		res = -3;
	}
	return res;
}

static int debug_key_eint_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("debug_key_eint_config(en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_EN);
	else if (en == WD_REQ_DIS)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_DIS);
	else
		res = -2;

	if (mode == WD_REQ_IRQ_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_IRQ_MODE);
	else if (mode == WD_REQ_RST_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_EINT_MARK, WD_REQ_RST_MODE);
	else
		res = -3;
	return res;
}

static int debug_key_sysrst_config(WD_REQ_CTL en, WD_REQ_MODE mode)
{
	int res = 0;

	pr_debug("debug_key_sysrst_config(en:0x%x,mode:0x%x)\n", en, mode);
	if (en == WD_REQ_EN)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_EN);
	else if (en == WD_REQ_DIS)
		res = mtk_wdt_request_en_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_DIS);
	else
		res = -2;

	if (mode == WD_REQ_IRQ_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_IRQ_MODE);
	else if (mode == WD_REQ_RST_MODE)
		res = mtk_wdt_request_mode_set(MTK_WDT_REQ_SYSRST_MARK, WD_REQ_RST_MODE);
	else
		res = -3;
	return res;
}

#endif

static int dfd_count_en(int value)
{
	return mtk_wdt_dfd_count_en(value);
}

static int dfd_thermal1_dis(int value)
{
	return mtk_wdt_dfd_thermal1_dis(value);
}

static int dfd_thermal2_dis(int value)
{
	return mtk_wdt_dfd_thermal2_dis(value);
}

static int dfd_timeout(int value)
{
	return mtk_wdt_dfd_timeout(value);
}

/* public interface implimentation end */

int wd_api_init(void)
{

	int i = 0;
	long *check_p = NULL;
	int api_size = 0;

	api_size = (sizeof(g_wd_api_obj) / sizeof(long));
	pr_debug("wd api_size=%d\n", api_size);
	/* check wd api */
	check_p = (long *)&g_wd_api_obj;
	for (i = 1; i < api_size; i++) {
		pr_debug("p[%d]=%lx\n", i, *(check_p + i));
		if (check_p[i] == 0) {
			pr_debug("wd_api init fail the %d api not init\n", i);
			g_wd_api_obj.ready = 0;
			return -1;
		}

	}
	pr_debug("wd_api init ok\n");
	return 0;
}

int get_wd_api(struct wd_api **obj)
{
	int res = 0;
	*obj = &g_wd_api_obj;
	if (*obj == NULL)
		res = -1;

	if ((*obj)->ready == 0)
		res = -2;

	return res;
}

#ifndef CONFIG_MEDIATEK_WATCHDOG

/* Mark the recovery/fastboot RTC flag in process context, not in restart
 * context.
 *
 * Android 9 init does not write a misc/BCB for `reboot,recovery`; it only
 * passes the string to reboot(2). arm_pm_restart is NULL (the DT declares
 * PSCI 0.1, so psci.c never assigns it), so machine_restart() falls through
 * to do_kernel_restart(cmd) -> mtk_arch_reset_handle -> arch_reset(), and
 * the only difference from a plain reboot is the rtc_mark_recovery() below.
 * That call runs with IRQs off, after smp_send_stop(), takes rtc_lock and
 * ends in pwrap traffic: it can hang if rtc_lock is held by a CPU that
 * smp_send_stop() failed to stop, or if CBUSY never clears.
 *
 * kernel_restart_prepare() calls the reboot notifier chain
 * (blocking_notifier_call_chain(SYS_RESTART, cmd)) in normal process
 * context with IRQs on, before device_shutdown(), migrate_to_reboot_cpu()
 * and machine_restart(). Doing the RTC mark there cannot be worse than
 * doing it after smp_send_stop(), and arch_reset() then skips the redundant
 * write.
 * echo 0 > /sys/module/wd_api/parameters/forge_rtc_mark_early restores the
 * stock ordering at run time.
 */
static int forge_rtc_mark_early = 1;
module_param(forge_rtc_mark_early, int, 0644);
MODULE_PARM_DESC(forge_rtc_mark_early,
	"m681: 1 = set the RTC recovery/fastboot flag from the reboot notifier (process context)");
static bool forge_rtc_marked;

static int forge_reboot_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	const char *cmd = (const char *)data;

	/* Cleared on every restart so a later plain reboot cannot inherit a
	 * stale "already marked" and skip a mark it needs. */
	forge_rtc_marked = false;

	if (!forge_rtc_mark_early || action != SYS_RESTART || !cmd)
		return NOTIFY_DONE;

#ifdef CONFIG_MTK_RTC
	if (!strcmp(cmd, "recovery")) {
		pr_emerg("[FORGE_M681] #S2 reboot notifier: rtc_mark_recovery() in PROCESS context\n");
		rtc_mark_recovery();
		forge_rtc_marked = true;
		pr_emerg("[FORGE_M681] #S2 reboot notifier: rtc_mark_recovery() returned\n");
	} else if (!strcmp(cmd, "bootloader")) {
		pr_emerg("[FORGE_M681] #S2 reboot notifier: rtc_mark_fast() in PROCESS context\n");
		rtc_mark_fast();
		forge_rtc_marked = true;
		pr_emerg("[FORGE_M681] #S2 reboot notifier: rtc_mark_fast() returned\n");
	}
#endif
	return NOTIFY_DONE;
}

static struct notifier_block forge_reboot_nb = {
	.notifier_call = forge_reboot_notify,
};

/*register restart notify and own by debug start-------
*
*/
void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_FPGA_EARLY_PORTING
	return;
#else
	char reboot = 0;
	int res = 0;
	struct wd_api *wd_api = NULL;

	res = get_wd_api(&wd_api);
	pr_alert("arch_reset: cmd = %s\n", cmd ? : "NULL");
	dump_stack();
	if (cmd && !strcmp(cmd, "charger")) {
		/* do nothing */
	} else if (cmd && !strcmp(cmd, "recovery")) {
#ifdef CONFIG_MTK_RTC
		/* already done in process context by forge_reboot_notify();
		 * repeating it here can hang */
		if (forge_rtc_marked)
			pr_emerg("[FORGE_M681] #S2 arch_reset: RTC flag already set in process context, skipping restart-context write\n");
		else
			rtc_mark_recovery();
#endif
	} else if (cmd && !strcmp(cmd, "bootloader")) {
#ifdef CONFIG_MTK_RTC
		if (forge_rtc_marked)
			pr_emerg("[FORGE_M681] #S2 arch_reset: RTC flag already set in process context, skipping restart-context write\n");
		else
			rtc_mark_fast();
#endif
	} else if (cmd && !strcmp(cmd, "kpoc")) {
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
		rtc_mark_kpoc();
#endif
	} else {
		reboot = WD_SW_RESET_BYPASS_PWR_KEY;
	}

	if (res)
		pr_err("arch_reset, get wd api error %d\n", res);
	else
		wd_api->wd_sw_reset(reboot);
 #endif
}
static struct notifier_block mtk_restart_handler;
static int mtk_arch_reset_handle(struct notifier_block *this, unsigned long mode, void *cmd)
{
	pr_alert("ARCH_RESET happen!!!\n");
	arch_reset(mode, cmd);
	pr_alert("ARCH_RESET end!!!!\n");
	return NOTIFY_DONE;
}

static int __init mtk_arch_reset_init(void)
{
	int ret;
#ifdef CONFIG_MTK_RTC
	pm_power_off = mt_power_off;
#endif
	mtk_restart_handler.notifier_call = mtk_arch_reset_handle;
	mtk_restart_handler.priority = 128;
	pr_alert("\n register_restart_handler- 0x%p, Notify call: - 0x%p\n",
		 &mtk_restart_handler, mtk_restart_handler.notifier_call);
	ret = register_restart_handler(&mtk_restart_handler);
	if (ret)
		pr_err("ARCH_RESET cannot register mtk_restart_handler!!!!\n");
	pr_alert("ARCH_RESET register mtk_restart_handler  ok!!!!\n");
	/* reboot_notifier_list is a statically initialised blocking chain,
	 * so registering at pure_initcall is safe. */
	if (register_reboot_notifier(&forge_reboot_nb))
		pr_err("[FORGE_M681] #S2 cannot register reboot notifier\n");
	else
		pr_alert("[FORGE_M681] #S2 reboot notifier registered (forge_rtc_mark_early=%d)\n",
			 forge_rtc_mark_early);
	return ret;
}

pure_initcall(mtk_arch_reset_init);
/*register restart notify and own by debug end+++++
*
*/
#endif
