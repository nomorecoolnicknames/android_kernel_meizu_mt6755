/*
 * MT6755 cpuidle driver.
 *
 * The state table follows the MT6755 3.18 BSP (cpuidle-mt67xx_v2.c): its
 * order is the IDLE_TYPE_* order of spm_v2/mt_idle.c, because the MTK
 * governor returns mt_idle_select(), an IDLE_TYPE_*, as the state index.
 * The enter functions are the ones of spm_v2/mt_idle.c, with one exception:
 * mt_spm_mcdi is not built for MT6755, so go_to_mcidle() is the weak
 * "return false" and mcidle_enter() would report MCDI without entering it.
 * MCDI is therefore .disabled here and refused by idle_type_allowed().
 *
 * mt_idle.c decides which states can be chosen (idle_type_allowed(),
 * idle_switch[]): WFI (rgidle) by default; SODI, deep idle and SODI3 each
 * only after its own mt_idle.forge_sodi / forge_dpidle / forge_sodi3 = 1.
 *
 * Notes for 4.4:
 *  - the 4.4 MTK governor calls mtk_idle_select() and, at postcore,
 *    mtk_cpuidle_framework_init() (the MT6757 names); the MT6755 framework
 *    has mt_idle_select() / mt_cpuidle_framework_init(). The governor's
 *    weak mtk_cpuidle_framework_init() stays a no-op: the framework is set
 *    up here, at device_initcall. mtk_idle_select() below forwards to
 *    mt_idle_select(); the governor only asks once this driver is
 *    registered. Likewise spm_sodi3_init/spm_sodi_init/spm_deepidle_init
 *    are skipped in spm_module_init() (arch_initcall) and run here.
 *  - no arm_cpuidle_init(): no state here uses the generic PSCI idle ops;
 *    the SPM scenarios power the CPU down through mt_cpu_dormant()
 *    (cpuidle_v1), WFI needs nothing.
 *  - no standalone "CPU dormant" state: the stock MT6755 idle table has
 *    none (MCDI is the only CPU-only power-down and it is not built).
 *
 * Deep idle, SODI3 and SODI power the CPU down, so its arch timer stops:
 * they carry CPUIDLE_FLAG_TIMER_STOP and the next timer event goes to the
 * broadcast device, if there is one; without one cpuidle_enter_state()
 * falls back to a state without the flag (here: WFI through the wrappers).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/suspend.h>

/* spm_v2/mt_idle.c */
int dpidle_enter(int cpu);
int soidle3_enter(int cpu);
int soidle_enter(int cpu);
int mcidle_enter(int cpu);
int slidle_enter(int cpu);
int rgidle_enter(int cpu);
int mt_idle_select(int cpu);
bool mt_idle_enter_checked(int type);
void mt_cpuidle_framework_init(void);
/* spm_v2/mt_spm_sodi3p0.c, mt_spm_sodi.c, mt_spm_dpidle.c */
void spm_sodi3_init(void);
void spm_sodi_init(void);
void spm_deepidle_init(void);

/* the 4.4 MTK governor (governors/mtk_governor.c) asks this name */
int mtk_idle_select(int cpu)
{
	return mt_idle_select(cpu);
}

/*
 * Every state but WFI runs only when the MTK governor chose it for this
 * entry (mt_idle_enter_checked(), spm_v2/mt_idle.c). The cpuidle core
 * also enters states without the governor: suspend-to-idle takes the
 * deepest enabled one. Such an entry runs WFI instead and is counted in
 * redirects.
 */
static unsigned int mt6755_cpuidle_redirects;
module_param_named(redirects, mt6755_cpuidle_redirects, uint, 0444);

static int mt6755_enter_checked(int index, int (*enter)(int cpu))
{
	int cpu = smp_processor_id();
	bool chosen = mt_idle_enter_checked(index);

	/* Suspend-to-idle never asks the governor, so a match there is a
	 * choice left over from an entry that call_cpuidle() dropped with
	 * -EBUSY (need_resched); its *_can_enter() checks are stale.
	 * WFI there, always. */
	if (chosen && !idle_should_freeze())
		return enter(cpu);
	mt6755_cpuidle_redirects++;
	return rgidle_enter(cpu);
}

static int mt6755_dpidle_enter(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	return mt6755_enter_checked(index, dpidle_enter);
}

static int mt6755_soidle3_enter(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	return mt6755_enter_checked(index, soidle3_enter);
}

static int mt6755_soidle_enter(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	return mt6755_enter_checked(index, soidle_enter);
}

static int mt6755_mcidle_enter(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	return mt6755_enter_checked(index, mcidle_enter);
}

static int mt6755_slidle_enter(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	return mt6755_enter_checked(index, slidle_enter);
}

static int mt6755_rgidle_enter(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	mt_idle_enter_checked(index);	/* consume the choice */
	return rgidle_enter(smp_processor_id());
}

static struct cpuidle_driver mt6755_cpuidle_driver = {
	.name             = "mt6755_cpuidle",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter            = mt6755_dpidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIMER_STOP,
		.disabled         = true,	/* mt_idle.forge_dpidle */
		.name             = "dpidle",
		.desc             = "deepidle",
	},
	.states[1] = {
		.enter            = mt6755_soidle3_enter,
		.exit_latency     = 5000,            /* 5 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIMER_STOP,
		.disabled         = true,	/* mt_idle.forge_sodi3 */
		.name             = "SODI3",
		.desc             = "SODI3",
	},
	.states[2] = {
		.enter            = mt6755_soidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIMER_STOP,
		.name             = "SODI",
		.desc             = "SODI",
	},
	.states[3] = {
		.enter            = mt6755_mcidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.disabled         = true,	/* not built for MT6755 */
		.name             = "MCDI",
		.desc             = "MCDI",
	},
	.states[4] = {
		.enter            = mt6755_slidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.name             = "slidle",
		.desc             = "slidle",
	},
	.states[5] = {
		.enter            = mt6755_rgidle_enter,
		/* the BSP said 2 ms; WFI resumes at once, and 2 ms made it
		 * look as deep as SODI to latency limits */
		.exit_latency     = 1,
		.target_residency = 1,
		.name             = "rgidle",
		.desc             = "WFI",
	},
	.state_count = 6,
	/* used by coupled idle only, which is not built; WFI anyway (the BSP
	 * said 0, dpidle) */
	.safe_state_index = 5,
};

static int __init mt6755_cpuidle_init(void)
{
	int ret;

	/* The scenario inits (DT lookups and iomap of mcucfg/m4u/apmixed,
	 * ram_console footprint reset) that spm_module_init() would run at
	 * arch_initcall; moved here so the early path stays unchanged. */
	spm_sodi3_init();
	spm_sodi_init();
	spm_deepidle_init();
	/* iomap, debugfs, hotplug notifier, profiling; with
	 * USING_STD_TIMER_OPS (MT6755) no GPT / cpuxgpt request */
	mt_cpuidle_framework_init();

	ret = cpuidle_register(&mt6755_cpuidle_driver, NULL);
	if (ret)
		pr_err("mt6755_cpuidle: register failed (%d)\n", ret);
	else
		pr_info("mt6755_cpuidle: registered, WFI only until mt_idle.forge_* is set\n");
	return ret;
}
device_initcall(mt6755_cpuidle_init);
