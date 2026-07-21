/*
 * Screen-off CPU hotplug (HPS-lite) for MT6755.
 *
 * Screen off: stock L681H hps_algo_amp thresholds (up 95% x3, down 85% x1):
 * sum of busy % over online CPUs > up_thr*n for up_times periods -> one CPU
 * up; < down_thr*(n-1) -> one CPU down. Screen on / enable=0 /
 * forge_hp_pin=1: every CPU online.
 *
 * Off by default (enable=0). Only CPUs in mask (default 0x7e = cpu1-3 +
 * cpu4-6). cpu0 is never touched; cpu7 only with mp1=1 (below). mask is
 * and-ed with 0x7e: cpu7 keeps MP1 (cpusys1) powered, because
 * spm_mtcmos_ctrl_cpu4..7 power cpusys1 down when the last core of cpu4-7
 * goes. In addition a cpu4-6 offline is refused unless cpu7 is online,
 * checked under lock_device_hotplug() (the lock the cpuN/online sysfs
 * writes take).
 *
 * Context: delayed works on an unbound, freezable workqueue, so the policy
 * is parked across S3 and never races disable_nonboot_cpus() /
 * enable_nonboot_cpus(). The display/fb hooks only do atomic bit ops and
 * mod_delayed_work() (no sleep, no locks), so the display one is safe
 * after _primary_path_unlock() in primary_display_suspend/resume.
 *
 * Android does not necessarily call FBIOBLANK / MTKFB_POWEROFF at screen
 * off (the HWC2 DispDevice::setPowerMode is empty), so "screen off" is the
 * OR of three sources, each with its own counters:
 *   disp - primary_display_suspend/resume (FBIOBLANK, MTKFB_POWEROFF, S3);
 *   fb   - fb notifier FB_EVENT_BLANK (any FBIOBLANK, whoever issues it);
 *   bl   - lcd-backlight led_classdev level read by the work: 0 for bl_off_ms
 *          = off, >0 again = on (read only, no writes anywhere).
 * Any "on" report clears every source's "off". With enable=1 and the
 * screen on, a deferrable poll (bl_poll_ms) samples the backlight; with the
 * screen off the policy runs on a non-deferrable timer (period_ms, default
 * 200) because a deferrable timer on an all-idle NO_HZ_IDLE system may not
 * fire for seconds.
 * cpufreq: mt_cpufreq's hotcpu notifier only acts on the last CPU of a
 * cluster (CPU_DOWN_PREPARE cpus==1: armpll_is_available=0, min freq);
 * cpu0 and cpu7 stay online, so that branch is never reached here.
 * No PMIC access anywhere in this file.
 *
 * mp1=1 (default 0): screen off, once cpu4-6 are all offline, cpu7 may go
 * too, taking MP1 (cpusys1) down via spm_mtcmos_ctrl_cpu7; with cpu1-3 also
 * offline only cpu0 is powered, which mt_idle_cpu_criteria() needs for
 * SODI / SODI3 / dpidle. Order down 6,5,4,7,3,2,1; up 1,2,3, then cpu7
 * before any of cpu4-6, so MP1 is never powered by cpu4-6 alone. cpu7 is
 * brought back only if this driver took it (hpl_took7); a cpu7 offlined by
 * hand is left alone. With mp1=1 the MP1 power-down and the mt_cpufreq
 * last-core branch (armpll_is_available=0) run outside S3.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/forge_hpl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include "mt_spm_mtcmos.h"

#define HPL_ALLOWED	0x7eU		/* cpu1-6; never cpu0, never cpu7 */
#define HPL_MP1_KEEPER	7		/* this core keeps MP1 (cpusys1) on */
#define HPL_MP1_CORES	0x70U		/* cpu4-6: may go only while cpu7 is on */
#define HPL_PWR_CORES	0x000F9F00U	/* MP0 8, cpu0-3 9-12, MP1 15, cpu4-7 16-19 */

static int hpl_enable;
static unsigned int hpl_mask = HPL_ALLOWED;
static unsigned int hpl_period_ms = 200;	/* screen-off step period */
static unsigned int hpl_up_thr = 95, hpl_up_times = 3, hpl_down_thr = 85;
static unsigned int hpl_n_off, hpl_n_on, hpl_n_err;
static unsigned int hpl_use_bl = 1, hpl_bl_off_ms = 5000, hpl_bl_poll_ms = 1000;
static unsigned int hpl_n_kick, hpl_last_load, hpl_last_n_online;
static int hpl_bl_level = -1;
static unsigned int hpl_mp1;		/* m1: cpu7 (MP1) may go after cpu4-6 */
static bool hpl_took7;			/* cpu7 is offline because of HPS-lite */
module_param_named(period_ms, hpl_period_ms, uint, 0644);
module_param_named(use_bl, hpl_use_bl, uint, 0644);
module_param_named(bl_off_ms, hpl_bl_off_ms, uint, 0644);
module_param_named(bl_poll_ms, hpl_bl_poll_ms, uint, 0644);
module_param_named(n_kick, hpl_n_kick, uint, 0444);
module_param_named(last_load, hpl_last_load, uint, 0444);
module_param_named(last_n_online, hpl_last_n_online, uint, 0444);
module_param_named(bl_level, hpl_bl_level, int, 0444);
module_param_named(up_thr, hpl_up_thr, uint, 0644);
module_param_named(up_times, hpl_up_times, uint, 0644);
module_param_named(down_thr, hpl_down_thr, uint, 0644);
module_param_named(n_off, hpl_n_off, uint, 0444);
module_param_named(n_on, hpl_n_on, uint, 0444);
module_param_named(n_err, hpl_n_err, uint, 0444);

/* screen-off sources; "off" = any bit set in hpl_off_src */
enum { HPL_SRC_DISP, HPL_SRC_FB, HPL_SRC_BL, HPL_SRC_NR };
static const char * const hpl_src_name[HPL_SRC_NR] = { "disp", "fb", "bl" };
static unsigned long hpl_off_src;
static atomic_t hpl_calls[HPL_SRC_NR][2];	/* [src][0] off, [1] on reports */
static int hpl_last_src = -1;
static bool hpl_last_on;
static bool hpl_bl_zero, hpl_bl_off;	/* work-only state, under hpl_lock */
static unsigned long hpl_bl_zero_since;
static unsigned int hpl_up_cnt;
static u64 prev_idle[NR_CPUS], prev_wall[NR_CPUS];
static struct cpumask hpl_sampled;	/* CPUs with a valid prev_* sample */
static DEFINE_MUTEX(hpl_lock);
static struct workqueue_struct *hpl_wq;
static void hpl_work_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(hpl_work, hpl_work_fn);	  /* kicks + screen-off steps */
static DECLARE_DEFERRABLE_WORK(hpl_poll, hpl_work_fn);	  /* screen-on backlight poll */

static bool hpl_screen_off(void)
{
	return READ_ONCE(hpl_off_src) != 0;
}

/* lockless: display/fb hooks and the work; "on" from any source clears all */
static void hpl_report(int src, bool on)
{
	if (on)
		xchg(&hpl_off_src, 0UL);
	else
		set_bit(src, &hpl_off_src);
	atomic_inc(&hpl_calls[src][on]);
	WRITE_ONCE(hpl_last_on, on);
	WRITE_ONCE(hpl_last_src, src);
}

/* weak default; drivers/misc/mediatek/leds/mtk_leds_drv.c overrides it */
int __weak forge_lcd_bl_level(void)
{
	return -1;
}

static void hpl_kick(unsigned int delay_ms)
{
	if (hpl_wq)
		mod_delayed_work(hpl_wq, &hpl_work, msecs_to_jiffies(delay_ms));
}

/* caller holds hpl_lock */
static int hpl_set_cpu(unsigned int cpu, bool on)
{
	struct device *dev = get_cpu_device(cpu);
	int ret;

	if (!dev)
		return -ENODEV;
	lock_device_hotplug();
	if (cpu == HPL_MP1_KEEPER)
		/* down only with cpu4-6 already gone: MP1 goes with it */
		ret = (!on && (!hpl_mp1 ||
			       (cpumask_bits(cpu_online_mask)[0] & HPL_MP1_CORES))) ?
			-EPERM : on ? device_online(dev) : device_offline(dev);
	else if (!on && cpu >= 4 && !cpu_online(HPL_MP1_KEEPER))
		ret = -EPERM;	/* would leave MP1 to cpu4-6 only */
	else
		ret = on ? device_online(dev) : device_offline(dev);
	if (cpu == HPL_MP1_KEEPER && ret == 0)
		hpl_took7 = !on;
	unlock_device_hotplug();
	if (ret < 0) {
		hpl_n_err++;
		pr_info_ratelimited("[FORGE_HPL] cpu%u %s ret=%d\n", cpu,
				    on ? "online" : "offline", ret);
	} else if (ret == 0) {
		if (on)
			hpl_n_on++;
		else
			hpl_n_off++;
	}
	return ret;
}

/* up: lowest masked offline CPU (little first); down: highest masked online */
static int hpl_pick(bool up)
{
	int cpu;

	if (up) {
		for (cpu = 1; cpu < nr_cpu_ids; cpu++)
			if ((hpl_mask & BIT(cpu)) && !cpu_online(cpu))
				/* m1: MP1 comes back with cpu7 first */
				return (cpu >= 4 && hpl_took7 &&
					!cpu_online(HPL_MP1_KEEPER)) ?
					HPL_MP1_KEEPER : cpu;
	} else {
		for (cpu = nr_cpu_ids - 1; cpu >= 1; cpu--) {
			if (cpu == 3 && hpl_mp1 &&
			    cpu_online(HPL_MP1_KEEPER) &&
			    !(cpumask_bits(cpu_online_mask)[0] & HPL_MP1_CORES))
				return HPL_MP1_KEEPER;	/* after 6,5,4 */
			if ((hpl_mask & BIT(cpu)) && cpu_online(cpu))
				return cpu;
		}
	}
	return -1;
}

/* caller holds hpl_lock; brings back every CPU HPS-lite may have taken down */
static void hpl_online_all(void)
{
	int cpu;

	if (hpl_took7 && !cpu_online(HPL_MP1_KEEPER))
		hpl_set_cpu(HPL_MP1_KEEPER, true);	/* MP1 before cpu4-6 */
	for (cpu = 1; cpu < nr_cpu_ids; cpu++)
		if ((HPL_ALLOWED & BIT(cpu)) && !cpu_online(cpu))
			hpl_set_cpu(cpu, true);
	hpl_up_cnt = 0;
}

/* sum of busy % over online CPUs (stock hps cur_loads); fresh CPUs skipped */
static unsigned int hpl_load(void)
{
	unsigned int cpu, sum = 0;
	struct cpumask seen;

	cpumask_clear(&seen);
	for_each_online_cpu(cpu) {
		u64 wall, idle = get_cpu_idle_time(cpu, &wall, 0);
		u64 dw = wall - prev_wall[cpu], di = idle - prev_idle[cpu];
		bool valid = cpumask_test_cpu(cpu, &hpl_sampled);

		prev_wall[cpu] = wall;
		prev_idle[cpu] = idle;
		cpumask_set_cpu(cpu, &seen);
		if (!valid) {
			sum += 100;	/* just onlined: no history, count as busy */
			continue;
		}
		if (dw && dw >= di)
			sum += min_t(u64, 100, div64_u64(100 * (dw - di), dw));
	}
	cpumask_copy(&hpl_sampled, &seen);
	return sum;
}

/* caller holds hpl_lock: read-only lcd-backlight sample, bl source edges */
static void hpl_bl_sample(void)
{
	int lvl = forge_lcd_bl_level();

	hpl_bl_level = lvl;
	if (!hpl_use_bl || lvl < 0) {
		hpl_bl_zero = false;
		return;
	}
	if (hpl_bl_off && !test_bit(HPL_SRC_BL, &hpl_off_src)) {
		hpl_bl_off = false;	/* disp/fb said "on": restart the window */
		hpl_bl_zero = false;
	}
	if (lvl > 0) {
		hpl_bl_zero = false;
		if (hpl_bl_off) {
			hpl_bl_off = false;
			hpl_report(HPL_SRC_BL, true);
		}
		return;
	}
	if (!hpl_bl_zero) {
		hpl_bl_zero = true;
		hpl_bl_zero_since = jiffies;
	}
	if (!hpl_bl_off && time_after_eq(jiffies, hpl_bl_zero_since +
					 msecs_to_jiffies(hpl_bl_off_ms))) {
		hpl_bl_off = true;
		hpl_report(HPL_SRC_BL, false);
	}
}

static void hpl_work_fn(struct work_struct *w)
{
	unsigned int n, load;
	int cpu;

	mutex_lock(&hpl_lock);
	hpl_n_kick++;
	if (!READ_ONCE(hpl_enable) || forge_hp_pinned()) {
		hpl_online_all();
		cpumask_clear(&hpl_sampled);
		hpl_bl_zero = false;
		if (hpl_bl_off) {	/* stale once the poll stops */
			hpl_bl_off = false;
			clear_bit(HPL_SRC_BL, &hpl_off_src);
		}
		hpl_last_n_online = num_online_cpus();
		goto out;
	}
	hpl_bl_sample();
	if (!hpl_screen_off()) {
		hpl_online_all();
		cpumask_clear(&hpl_sampled);
		hpl_last_n_online = num_online_cpus();
		cancel_delayed_work(&hpl_work);
		if (hpl_use_bl)
			queue_delayed_work(hpl_wq, &hpl_poll,
				msecs_to_jiffies(max(hpl_bl_poll_ms, 100U)));
		goto out;
	}
	n = num_online_cpus();
	load = hpl_load();
	hpl_last_load = load;
	if (load > hpl_up_thr * n) {
		if (++hpl_up_cnt >= hpl_up_times) {
			hpl_up_cnt = 0;
			cpu = hpl_pick(true);
			if (cpu >= 0)
				hpl_set_cpu(cpu, true);
		}
	} else {
		hpl_up_cnt = 0;
		if (n > 1 && load < hpl_down_thr * (n - 1)) {
			cpu = hpl_pick(false);
			if (cpu >= 0)
				hpl_set_cpu(cpu, false);
		}
	}
	hpl_last_n_online = num_online_cpus();
	cancel_delayed_work(&hpl_poll);
	queue_delayed_work(hpl_wq, &hpl_work,	/* not deferrable: see header */
			   msecs_to_jiffies(max(hpl_period_ms, 10U)));
out:
	mutex_unlock(&hpl_lock);
}

/* end of primary_display_suspend (on=false) / primary_display_resume (on=true) */
void forge_hpl_display(bool on)
{
	hpl_report(HPL_SRC_DISP, on);
	if (READ_ONCE(hpl_enable))
		hpl_kick(on ? 0 : hpl_period_ms);
}

#ifdef CONFIG_FB
/* any FBIOBLANK; mtkfb_blank: UNBLANK/NORMAL resume, POWERDOWN suspend */
static int hpl_fb_notify(struct notifier_block *nb, unsigned long ev,
			 void *data)
{
	struct fb_event *e = data;
	int blank;

	if (ev != FB_EVENT_BLANK || !e || !e->data)
		return NOTIFY_DONE;
	blank = *(int *)e->data;
	if (blank == FB_BLANK_UNBLANK || blank == FB_BLANK_NORMAL)
		hpl_report(HPL_SRC_FB, true);
	else if (blank == FB_BLANK_POWERDOWN)
		hpl_report(HPL_SRC_FB, false);
	else
		return NOTIFY_DONE;
	if (READ_ONCE(hpl_enable))
		hpl_kick(0);
	return NOTIFY_OK;
}
static struct notifier_block hpl_fb_nb = {
	.notifier_call = hpl_fb_notify,
};
#endif

/*
 * forge_hp_pin=1 calls this before cpu_hotplug_disable(). enable is dropped
 * first: forge_hp_frozen is still false here, so a work re-queued by the
 * display hook between this restore and the freeze must not take a CPU down.
 * Writes to enable and forge_hp_pin are serialized by the built-in param_lock.
 */
void forge_hpl_restore_all(void)
{
	WRITE_ONCE(hpl_enable, 0);
	if (!hpl_wq)
		return;
	cancel_delayed_work_sync(&hpl_work);
	cancel_delayed_work_sync(&hpl_poll);
	mutex_lock(&hpl_lock);
	hpl_online_all();
	mutex_unlock(&hpl_lock);
	pr_emerg("[FORGE_HPL] restore_all: online=%u n_off=%u n_on=%u n_err=%u\n",
		 num_online_cpus(), hpl_n_off, hpl_n_on, hpl_n_err);
}

static int hpl_enable_set(const char *val, const struct kernel_param *kp)
{
	int v, ret = kstrtoint(val, 0, &v);

	if (ret)
		return ret;
	if (v != 0 && v != 1)
		return -EINVAL;
	if (v && forge_hp_pinned()) {
		pr_warn("[FORGE_HPL] forge_hp_pin=1: write 0 to forge_m681_marker.forge_hp_pin first\n");
		return -EBUSY;
	}
	WRITE_ONCE(hpl_enable, v);
	pr_emerg("[FORGE_HPL] enable=%d mask=0x%02x mp1=%u period=%ums up=%u/%u down=%u\n",
		 v, hpl_mask, hpl_mp1, hpl_period_ms, hpl_up_thr, hpl_up_times,
		 hpl_down_thr);
	hpl_kick(0);	/* enable=0: work brings every CPU back */
	return 0;
}
static const struct kernel_param_ops hpl_enable_ops = {
	.set = hpl_enable_set,
	.get = param_get_int,
};
module_param_cb(enable, &hpl_enable_ops, &hpl_enable, 0644);

static int hpl_mask_set(const char *val, const struct kernel_param *kp)
{
	unsigned int m;
	int ret = kstrtouint(val, 0, &m);

	if (ret)
		return ret;
	if (READ_ONCE(hpl_enable))
		return -EBUSY;
	mutex_lock(&hpl_lock);
	hpl_mask = m & HPL_ALLOWED;	/* never cpu0, never cpu7 (MP1) */
	mutex_unlock(&hpl_lock);
	return 0;
}
static const struct kernel_param_ops hpl_mask_ops = {
	.set = hpl_mask_set,
	.get = param_get_uint,
};
module_param_cb(mask, &hpl_mask_ops, &hpl_mask, 0644);

/* m1: changed only with enable=0, like mask */
static int hpl_mp1_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > 1)
		return -EINVAL;
	if (READ_ONCE(hpl_enable))
		return -EBUSY;
	mutex_lock(&hpl_lock);
	hpl_mp1 = v;
	mutex_unlock(&hpl_lock);
	pr_emerg("[FORGE_HPL] mp1=%u (cpu7/MP1 %s)\n", v,
		 v ? "may go after cpu4-6" : "never");
	return 0;
}
static const struct kernel_param_ops hpl_mp1_ops = {
	.set = hpl_mp1_set,
	.get = param_get_uint,
};
module_param_cb(mp1, &hpl_mp1_ops, &hpl_mp1, 0644);

static int hpl_pwr_get(char *buf, const struct kernel_param *kp)
{
	u32 st2, st = spm_mtcmos_pwr_status(&st2);
	u32 c = st & HPL_PWR_CORES;
	unsigned int cpu, spm_cpus = 0;

	for (cpu = 0; cpu < 4; cpu++)
		if (c & BIT(9 + cpu))
			spm_cpus |= BIT(cpu);
	for (cpu = 4; cpu < 8; cpu++)
		if (c & BIT(16 + cpu - 4))
			spm_cpus |= BIT(cpu);
	return scnprintf(buf, PAGE_SIZE,
			 "cores=0x%08x spm_cpus=0x%02x mp0=%u mp1=%u online=0x%02lx raw=0x%08x raw_2nd=0x%08x\n",
			 c, spm_cpus, !!(c & BIT(8)), !!(c & BIT(15)),
			 cpumask_bits(cpu_online_mask)[0] & 0xff, st, st2);
}
static const struct kernel_param_ops hpl_pwr_ops = {
	.get = hpl_pwr_get,
};
module_param_cb(pwr_stat, &hpl_pwr_ops, NULL, 0444);

static int hpl_screen_off_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", hpl_screen_off());
}
static const struct kernel_param_ops hpl_screen_off_ops = {
	.get = hpl_screen_off_get,
};
module_param_cb(screen_off, &hpl_screen_off_ops, NULL, 0444);

/* last reporter, e.g. "bl off" + which sources currently say off */
static int hpl_src_get(char *buf, const struct kernel_param *kp)
{
	int src = READ_ONCE(hpl_last_src);
	unsigned long m = READ_ONCE(hpl_off_src);

	return scnprintf(buf, PAGE_SIZE, "%s %s off_src=%s%s%s\n",
			 src >= 0 ? hpl_src_name[src] : "none",
			 src < 0 ? "-" : READ_ONCE(hpl_last_on) ? "on" : "off",
			 m & BIT(HPL_SRC_DISP) ? "disp," : "",
			 m & BIT(HPL_SRC_FB) ? "fb," : "",
			 m & BIT(HPL_SRC_BL) ? "bl," : "");
}
static const struct kernel_param_ops hpl_src_ops = {
	.get = hpl_src_get,
};
module_param_cb(src, &hpl_src_ops, NULL, 0444);

/* off/on report counts per source (n_display_calls) */
static int hpl_calls_get(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "disp=%d/%d fb=%d/%d bl=%d/%d (off/on)\n",
			 atomic_read(&hpl_calls[HPL_SRC_DISP][0]),
			 atomic_read(&hpl_calls[HPL_SRC_DISP][1]),
			 atomic_read(&hpl_calls[HPL_SRC_FB][0]),
			 atomic_read(&hpl_calls[HPL_SRC_FB][1]),
			 atomic_read(&hpl_calls[HPL_SRC_BL][0]),
			 atomic_read(&hpl_calls[HPL_SRC_BL][1]));
}
static const struct kernel_param_ops hpl_calls_ops = {
	.get = hpl_calls_get,
};
module_param_cb(n_display_calls, &hpl_calls_ops, NULL, 0444);

static int __init forge_hpl_init(void)
{
	hpl_wq = alloc_workqueue("forge_hpl", WQ_UNBOUND | WQ_FREEZABLE, 1);
	if (!hpl_wq)
		return -ENOMEM;
#ifdef CONFIG_FB
	if (fb_register_client(&hpl_fb_nb))
		pr_emerg("[FORGE_HPL] fb_register_client failed\n");
#endif
	pr_emerg("[FORGE_HPL] m1 ready: enable=%d mask=0x%02x mp1=%u period=%ums bl=%u/%ums (cpu0 never)\n",
		 hpl_enable, hpl_mask, hpl_mp1, hpl_period_ms, hpl_use_bl,
		 hpl_bl_off_ms);
	if (hpl_enable)
		hpl_kick(0);
	return 0;
}
late_initcall_sync(forge_hpl_init);
