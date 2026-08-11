/*
 * CPU thermal throttling without the PPM port.
 *
 * A 1 s delayed work reads tscpu_get_curr_max_ts_temp() (the value the
 * mtktscpu zone stores on each poll) and drives a two-step clamp on the
 * big cluster's max frequency through a CPUFREQ_ADJUST policy notifier +
 * cpufreq_verify_within_limits(). This is the same path stock's
 * mtk_cpufreq_thermal_protect uses, minus the PPM middle layer. It only
 * picks among existing table rows; it never touches voltage, PLLs, or the
 * frequency-only clamp in mt_cpufreq.
 *
 * Fail-safe by construction:
 *  - the guard only ever lowers a policy max (verify_within_limits with
 *    max=clamp); there is no code path that raises anything;
 *  - even when cool it imposes the 1196 MHz baseline ceiling, unless
 *    forge_tg_ceiling_khz says otherwise: 1196000 (default), 0 = no cold
 *    ceiling, set automatically on m681 when the voltage-DVFS mt_cpufreq
 *    links a strong forge_cpufreq_full_dvfs(); the 85/95 C steps stay
 *    either way, and the stock ATM (cpu_adaptive_0 from 67 C) does the
 *    regular throttling;
 *  - an invalid sample (0 or < -20 C = "sensor not answering") never
 *    relaxes the clamp: the last thermal clamp is kept and the state is
 *    marked FAILSAFE until real samples return; at level 0 the clamp is
 *    then the 1196 MHz baseline even with the cold ceiling off;
 *  - the test override can only add restriction (effective state =
 *    max(thermal, forced)), so a typo cannot release a genuine clamp.
 *
 * Always on: it acts only once cpufreq policies exist (after
 * forge_mt_cpufreq arming); with no policy, cpufreq_update_policy() is a
 * no-op and the notifier never runs.
 */

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

/* from mtk_tc.c (declared in tscpu_settings.h:401, which is not directly
 * includable here: it needs the ts_e type chain only mtk_tc.c sets up) */
extern int tscpu_get_curr_max_ts_temp(void);

#define TG_POLL_MS		1000
#define TG_BIG_FIRST_CPU	4	/* policy4 = big, */

#define TG_L1_ENGAGE_MC		85000	/* >= 85.0 C  -> 871 MHz  */
#define TG_L1_RELEASE_MC	80000	/* <  80.0 C  -> clear    */
#define TG_L2_ENGAGE_MC		95000	/* >= 95.0 C  -> 663 MHz  */
#define TG_L2_RELEASE_MC	90000	/* <  90.0 C  -> back to L1 */

#define TG_CEILING_KHZ		1196000	/* frequency-only cold point */
#define TG_L1_KHZ		871000
#define TG_L2_KHZ		663000

#define TG_INVALID_MC(t)	((t) == 0 || (t) < -20000)

/* dead CPU sensor (mtk_ts_cpu.c forge_tz_sensor_dead()): big at the
 * 1196 MHz ceiling and LL at 689 MHz, the 1.0 V points */
#define TG_DEAD_LL_KHZ		689000
#define TG_LL_CPU		0

/* test override: 0 = off (default), 1/2 = force at least L1/L2. Only adds
 * restriction; values outside 0..2 are clamped. Lets the clamp/release
 * machinery be device-verified without heating the die to 85 C. */
static int forge_tg_force;
module_param(forge_tg_force, int, 0644);

/* read-only observability */
static int forge_tg_state;		/* effective level 0/1/2 */
static int forge_tg_failsafe;		/* 1 = last sample invalid */
static int forge_tg_last_temp_mc;
static int forge_tg_dead;		/* 1 = CPU sensor dead, 1196/689 */
static int forge_tg_dead_applied;	/* forge_tg_dead the LL policy took */
module_param(forge_tg_state, int, 0444);
module_param(forge_tg_dead, int, 0444);
module_param(forge_tg_failsafe, int, 0444);
module_param(forge_tg_last_temp_mc, int, 0444);

/* cold ceiling, kHz; 0 = none (see the header). Read on every 1 s step. */
static unsigned int forge_tg_ceiling_khz = TG_CEILING_KHZ;
module_param_named(forge_tg_ceiling_khz, forge_tg_ceiling_khz, uint, 0644);

/* The strong definition lives in the voltage-DVFS mt_cpufreq (returns
 * true once the stock OPP ladder with the VPROC/VSRAM writer is built
 * in). Frequency-only images keep this weak one and with it the 1196 MHz
 * cold ceiling. The cpufreq side holds its own boot-time 1196 ceiling
 * (forge_cpu_boot_ceiling_khz) until boot_completed. */
bool __weak forge_cpufreq_full_dvfs(void)
{
	return false;
}

/* strong one in mtk_ts_cpu.c (the tzcpu sample filter) */
bool __weak forge_tz_sensor_dead(void)
{
	return false;
}

static unsigned int forge_tg_clamp_khz = TG_CEILING_KHZ;
static int forge_tg_thermal_level;	/* hysteresis memory, 0/1/2 */
static struct delayed_work forge_tg_work;

static unsigned int forge_tg_level_khz(int level)
{
	unsigned int ceil = READ_ONCE(forge_tg_ceiling_khz);

	if (level >= 2)
		return TG_L2_KHZ;
	if (level == 1)
		return TG_L1_KHZ;
	return ceil ? ceil : UINT_MAX;
}

static int forge_tg_policy_notifier(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_DONE;
	if (policy->cpu < TG_BIG_FIRST_CPU) {
		/* LITTLE: 689 < 871, left alone - but for a dead sensor */
		if (!READ_ONCE(forge_tg_dead))
			return NOTIFY_DONE;
		cpufreq_verify_within_limits(policy, 0, TG_DEAD_LL_KHZ);
		return NOTIFY_OK;
	}
	cpufreq_verify_within_limits(policy, 0,
				     READ_ONCE(forge_tg_clamp_khz));
	return NOTIFY_OK;
}

static struct notifier_block forge_tg_nb = {
	.notifier_call = forge_tg_policy_notifier,
};

static void forge_tg_work_fn(struct work_struct *work)
{
	int t = tscpu_get_curr_max_ts_temp();
	int level = forge_tg_thermal_level;
	int force = clamp_val(forge_tg_force, 0, 2);
	int eff, invalid = TG_INVALID_MC(t);
	bool dead;
	unsigned int clamp;

	if (!invalid) {
		switch (level) {
		case 0:
			level = t >= TG_L2_ENGAGE_MC ? 2 :
				(t >= TG_L1_ENGAGE_MC ? 1 : 0);
			break;
		case 1:
			level = t >= TG_L2_ENGAGE_MC ? 2 :
				(t < TG_L1_RELEASE_MC ? 0 : 1);
			break;
		default:
			level = t < TG_L1_RELEASE_MC ? 0 :
				(t < TG_L2_RELEASE_MC ? 1 : 2);
			break;
		}
		forge_tg_thermal_level = level;
	}
	/* invalid: keep the previous thermal level — never relax blind */

	eff = max(level, force);
	clamp = forge_tg_level_khz(eff);
	/* sensor not answering at level 0: never no ceiling - the baseline
	 * 1196 MHz even when full DVFS took the cold ceiling off (the ATM reads
	 * the same sensor) */
	if (invalid && clamp > TG_CEILING_KHZ)
		clamp = TG_CEILING_KHZ;
	/* dead sensor: the same ceiling for big, LL to 689 MHz (notifier) */
	dead = forge_tz_sensor_dead();
	if (dead && clamp > TG_CEILING_KHZ)
		clamp = TG_CEILING_KHZ;
	if (dead != forge_tg_dead) {
		WRITE_ONCE(forge_tg_dead, dead);
		pr_err("[FORGE_THERM_GUARD] CPU sensor %s: big <= %u kHz, LL %s\n",
		       dead ? "dead" : "back", clamp,
		       dead ? "<= 689000 kHz" : "released");
	}
	/* again on every step until the LL policy took it (-EAGAIN while
	 * cpufreq is suspended, no policy yet) */
	if (forge_tg_dead_applied != forge_tg_dead &&
	    !cpufreq_update_policy(TG_LL_CPU))
		forge_tg_dead_applied = forge_tg_dead;

	forge_tg_last_temp_mc = t;
	if (invalid != forge_tg_failsafe) {
		forge_tg_failsafe = invalid;
		pr_err("[FORGE_THERM_GUARD] %s (temp=%d mC): clamp held at %u kHz\n",
		       invalid ? "FAILSAFE: sensor not answering" :
		       "sensor back", t, clamp);
	}
	if (clamp != forge_tg_clamp_khz || eff != forge_tg_state) {
		pr_err("[FORGE_THERM_GUARD] clamp big to %u kHz (Tmax=%d mC level=%d force=%d)\n",
		       clamp, t, level, force);
		WRITE_ONCE(forge_tg_clamp_khz, clamp);
		forge_tg_state = eff;
		cpufreq_update_policy(TG_BIG_FIRST_CPU);
	}

	schedule_delayed_work(&forge_tg_work, msecs_to_jiffies(TG_POLL_MS));
}

static int __init forge_tg_init(void)
{
	int ret;

	/* only the build default is replaced: a cmdline value other than
	 * 1196000 is kept as given */
	if (forge_cpufreq_full_dvfs() &&
	    forge_tg_ceiling_khz == TG_CEILING_KHZ) {
		forge_tg_ceiling_khz = 0;
		forge_tg_clamp_khz = forge_tg_level_khz(0);
		pr_err("[FORGE_THERM_GUARD] full DVFS cpufreq on this board: cold ceiling off (forge_tg_ceiling_khz=0)\n");
	}

	ret = cpufreq_register_notifier(&forge_tg_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("[FORGE_THERM_GUARD] notifier register failed %d — NO throttling\n",
		       ret);
		return ret;
	}
	INIT_DELAYED_WORK(&forge_tg_work, forge_tg_work_fn);
	schedule_delayed_work(&forge_tg_work, msecs_to_jiffies(TG_POLL_MS));
	pr_err("[FORGE_THERM_GUARD] active: poll %d ms; L1 %dmC->%ukHz L2 %dmC->%ukHz hyst 5C; cool/failsafe ceiling %ukHz (forge_tg_ceiling_khz, 0 = none); acts once cpufreq policies exist\n",
	       TG_POLL_MS, TG_L1_ENGAGE_MC, TG_L1_KHZ, TG_L2_ENGAGE_MC,
	       TG_L2_KHZ, forge_tg_ceiling_khz);
	return 0;
}
late_initcall(forge_tg_init);
