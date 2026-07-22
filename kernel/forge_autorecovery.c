/*
 * forge_autorecovery - safety net for unattended test boots.
 *
 * "forge_ar=<seconds>" on the kernel command line arms a one-shot timer.
 * If nothing writes 0 to /proc/forge_ar before it expires, the kernel
 * syncs and restarts with the "recovery" reason, which MTK arch_reset()
 * turns into rtc_mark_recovery(): LK then boots the recovery partition
 * and the previous kernel log is left in /proc/last_kmsg.
 * Without forge_ar= (or with forge_ar=0) nothing is armed.
 *
 * /proc/forge_ar: read -> "armed <0|1> timeout <s> left <s>";
 *                 write 0 -> disarm, write N -> re-arm for N seconds.
 *
 * "forge_ar_rtc=1": at arch_initcall_sync (right after rtc_arch_init; pwrap
 * is up since postcore) the RTC recovery flag is set right away, the same
 * rtc_mark_recovery() a "reboot recovery" uses. From then on any reset -
 * panic, hardware watchdog, forge_ar expiry, a plain reboot - makes LK boot
 * the recovery partition once; LK consumes the flag. This is for test
 * kernels on boards whose stock LK has no "fastboot boot" (the L681 LK
 * answers "unknown command") and would otherwise boot-loop a crashing
 * kernel. A crash before arch_initcall_sync is not covered.
 */
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/seq_file.h>
#include <linux/syscore_ops.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <mt-plat/mtk_rtc.h>

static unsigned int forge_ar_timeout;
static unsigned int forge_ar_boot_timeout;	/* forge_ar= from the cmdline */
static unsigned long forge_ar_deadline;
static bool forge_ar_armed;
static DEFINE_MUTEX(forge_ar_lock);

static unsigned int forge_ar_rtc;

static int __init forge_ar_rtc_setup(char *s)
{
	if (kstrtouint(s, 0, &forge_ar_rtc))
		forge_ar_rtc = 0;
	return 1;
}
__setup("forge_ar_rtc=", forge_ar_rtc_setup);

static int __init forge_ar_rtc_arm(void)
{
	if (!forge_ar_rtc)
		return 0;
	rtc_mark_recovery();
	pr_emerg("forge_ar: RTC recovery flag armed at arch_initcall_sync - any reset boots recovery\n");
	return 0;
}
arch_initcall_sync(forge_ar_rtc_arm);

static void forge_ar_fire(struct work_struct *work);
static DECLARE_DELAYED_WORK(forge_ar_work, forge_ar_fire);

/*
 * Backstop for a kernel_restart() that never reaches the reset.
 * kernel_restart -> migrate_to_reboot_cpu -> cpu_hotplug_disable takes
 * cpu_maps_update_begin; a cpu_kill stuck under that lock deadlocks it,
 * and the tickers keep feeding the RGU. 20 s after forge_ar starts the
 * restart, this hrtimer calls emergency_restart(): it skips the reboot
 * notifiers and the hotplug lock and goes machine_restart(NULL)
 * -> do_kernel_restart -> arch_reset(cmd=NULL) -> wd_sw_reset ->
 * wdt_arch_reset -> PSCI SYSTEM_RESET SMC, the same SMC the panic path
 * (PANIC_TIMEOUT=1) uses. cmd=NULL does not mark the RTC, so
 * forge_ar_restart() marks it itself before arming this timer.
 */
#define FORGE_AR_BACKSTOP_SEC	20
static struct hrtimer forge_ar_backstop;

static enum hrtimer_restart forge_ar_backstop_fn(struct hrtimer *t)
{
	pr_emerg("[FORGE_AR] backstop: kernel_restart stuck %d s -> emergency_restart (path=EMERGENCY)\n",
		 FORGE_AR_BACKSTOP_SEC);
	emergency_restart();
	return HRTIMER_NORESTART;
}

/*
 * Marks the normal path. syscore_shutdown() runs in kernel_restart() right
 * after migrate_to_reboot_cpu() (i.e. after the hotplug lock was obtained)
 * and before machine_restart(); emergency_restart() never calls it. Prints
 * only once forge_ar_restart() has started.
 */
static bool forge_ar_restarting;

static void forge_ar_syscore_shutdown(void)
{
	if (forge_ar_restarting)
		pr_emerg("[FORGE_AR] path=NORMAL kernel_restart proceeded (hotplug lock obtained)\n");
}

static struct syscore_ops forge_ar_syscore_ops = {
	.shutdown = forge_ar_syscore_shutdown,
};

/* Process context. Never returns. */
static void forge_ar_restart(void)
{
	/* RTC recovery flag first, the stock way forge_ar_rtc_arm() does it:
	 * the normal path sets it in the wd_api reboot notifier (kernel_restart_prepare),
	 * the emergency_restart backstop would not. */
	rtc_mark_recovery();
	forge_ar_restarting = true;
	pr_emerg("forge_ar: RTC recovery flag set, backstop emergency_restart in %d s\n",
		 FORGE_AR_BACKSTOP_SEC);
	hrtimer_start(&forge_ar_backstop, ktime_set(FORGE_AR_BACKSTOP_SEC, 0),
		      HRTIMER_MODE_REL);
	emergency_sync();
	kernel_restart("recovery");
}

static int __init forge_ar_setup(char *s)
{
	if (kstrtouint(s, 0, &forge_ar_timeout))
		forge_ar_timeout = 0;
	forge_ar_boot_timeout = forge_ar_timeout;
	return 1;
}
__setup("forge_ar=", forge_ar_setup);

/* Caller holds forge_ar_lock. */
static void forge_ar_arm(unsigned int seconds)
{
	cancel_delayed_work(&forge_ar_work);
	forge_ar_timeout = seconds;
	forge_ar_armed = seconds > 0;
	if (!forge_ar_armed)
		return;
	forge_ar_deadline = jiffies + seconds * HZ;
	schedule_delayed_work(&forge_ar_work, seconds * HZ);
}

static void forge_ar_fire(struct work_struct *work)
{
	mutex_lock(&forge_ar_lock);
	if (!forge_ar_armed) {
		mutex_unlock(&forge_ar_lock);
		return;
	}
	forge_ar_armed = false;
	mutex_unlock(&forge_ar_lock);

	pr_emerg("forge_ar: not disarmed within %u s, restarting into recovery\n",
		 forge_ar_timeout);
	forge_ar_restart();
}

/*
 * Test knob for the hotplug-lock bypass, meant to run before any CPU has
 * been taken offline.
 * echo 1 > /sys/module/forge_autorecovery/parameters/forge_ar_locktest
 * starts kthread "forge_ar_lockhold", which takes cpu_maps_update_begin()
 * (the state a stuck cpu_kill leaves behind), holds it 60 s and then
 * releases it itself (a mutex can only be released by its owner, so the
 * writer, which goes on into kernel_restart, must not own it). Once the
 * kthread holds the lock the writer does exactly what forge_ar_fire does.
 * Outcomes:
 *  - EMERGENCY: kernel_restart blocks in migrate_to_reboot_cpu ->
 *    cpu_hotplug_disable, the 20 s hrtimer fires emergency_restart ->
 *    recovery;
 *  - NORMAL after 60 s: emergency did not reset, the kthread releases the
 *    lock, kernel_restart proceeds -> "path=NORMAL" -> recovery at ~60 s;
 *  - nothing: both failed, the phone stays up.
 * Only on a test image booted with forge_ar=N (N > 0), else -EPERM; only
 * once the system is running, else -EBUSY; one run per boot.
 */
#define FORGE_AR_LOCKHOLD_SEC	60
static DECLARE_COMPLETION(forge_ar_lock_held);
static atomic_t forge_ar_locktest_ran = ATOMIC_INIT(0);

static int forge_ar_lockhold_fn(void *unused)
{
	cpu_maps_update_begin();
	complete(&forge_ar_lock_held);
	pr_emerg("[FORGE_AR] locktest: hotplug lock HELD by kthread\n");
	msleep(FORGE_AR_LOCKHOLD_SEC * 1000);
	pr_emerg("[FORGE_AR] locktest: releasing hotplug lock after %d s (emergency backstop did NOT fire)\n",
		 FORGE_AR_LOCKHOLD_SEC);
	cpu_maps_update_done();
	return 0;
}

static int forge_ar_locktest_set(const char *val, const struct kernel_param *kp)
{
	struct task_struct *t;
	unsigned int v;

	if (kstrtouint(val, 0, &v) || v != 1)
		return -EINVAL;
	if (!forge_ar_boot_timeout)
		return -EPERM;
	if (system_state != SYSTEM_RUNNING)
		return -EBUSY;
	if (atomic_xchg(&forge_ar_locktest_ran, 1))
		return -EBUSY;

	mutex_lock(&forge_ar_lock);
	forge_ar_armed = false;		/* the delayed work must not race us */
	cancel_delayed_work(&forge_ar_work);
	mutex_unlock(&forge_ar_lock);

	pr_emerg("[FORGE_AR] locktest: starting forge_ar_lockhold (holds the hotplug lock %d s)\n",
		 FORGE_AR_LOCKHOLD_SEC);
	t = kthread_run(forge_ar_lockhold_fn, NULL, "forge_ar_lockhold");
	if (IS_ERR(t)) {
		pr_emerg("[FORGE_AR] locktest: kthread_run failed %ld, forge_ar NOT re-armed\n",
			 PTR_ERR(t));
		return PTR_ERR(t);
	}
	if (!wait_for_completion_timeout(&forge_ar_lock_held, 5 * HZ)) {
		pr_emerg("[FORGE_AR] locktest: kthread did not take the lock in 5 s, restart NOT started\n");
		return -EIO;
	}
	pr_emerg("[FORGE_AR] locktest: hotplug lock held, restarting into recovery as forge_ar_fire\n");
	forge_ar_restart();
	return 0;	/* not reached */
}

static const struct kernel_param_ops forge_ar_locktest_ops = {
	.set = forge_ar_locktest_set,
};
module_param_cb(forge_ar_locktest, &forge_ar_locktest_ops, NULL, 0200);

static int forge_ar_show(struct seq_file *m, void *v)
{
	long left = 0;

	mutex_lock(&forge_ar_lock);
	if (forge_ar_armed && time_before(jiffies, forge_ar_deadline))
		left = (long)(forge_ar_deadline - jiffies) / HZ;
	seq_printf(m, "armed %d timeout %u left %ld\n",
		   forge_ar_armed, forge_ar_timeout, left);
	mutex_unlock(&forge_ar_lock);
	return 0;
}

static int forge_ar_open(struct inode *inode, struct file *file)
{
	return single_open(file, forge_ar_show, NULL);
}

static ssize_t forge_ar_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	unsigned int seconds;
	int ret;

	ret = kstrtouint_from_user(buf, count, 0, &seconds);
	if (ret)
		return ret;
	mutex_lock(&forge_ar_lock);
	forge_ar_arm(seconds);
	mutex_unlock(&forge_ar_lock);
	pr_info("forge_ar: %s (%u s)\n", seconds ? "re-armed" : "disarmed", seconds);
	return count;
}

static const struct file_operations forge_ar_fops = {
	.open		= forge_ar_open,
	.read		= seq_read,
	.write		= forge_ar_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init forge_ar_init(void)
{
	hrtimer_init(&forge_ar_backstop, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	forge_ar_backstop.function = forge_ar_backstop_fn;
	register_syscore_ops(&forge_ar_syscore_ops);
	if (!proc_create("forge_ar", 0660, NULL, &forge_ar_fops))
		pr_err("forge_ar: cannot create /proc/forge_ar\n");
	mutex_lock(&forge_ar_lock);
	forge_ar_arm(forge_ar_timeout);
	mutex_unlock(&forge_ar_lock);
	if (forge_ar_armed)
		pr_info("forge_ar: armed, recovery in %u s unless disarmed\n",
			forge_ar_timeout);
	return 0;
}
late_initcall(forge_ar_init);
