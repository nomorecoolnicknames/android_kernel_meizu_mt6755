/*
 * M681 MT6351 vibrator. Register fields and board properties follow the
 * MediaTek 2016 mt6755 vibrator driver (GPL-2.0).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <mt-plat/upmu_common.h>

struct m681_vibrator {
	struct led_classdev led;
	struct device *timed_dev;
	struct delayed_work stop;
	struct mutex request_lock;
	struct mutex state_lock;
	unsigned int duration_ms;
	unsigned int timer_ms;		/* DT vib_timer: shortest timed_output pulse */
	unsigned int limit_ms;		/* DT vib_limit: shorter requests stay as asked */
	unsigned int vibr_vol;
	unsigned int stop_retries;
	unsigned int suspend_aborts;
	bool enabled;
	bool stopping;
	bool suspended;
};

#define VIBRATOR_OFF_TRIES	3
#define VIBRATOR_STOP_RETRIES	10	/* stop work re-runs, 100 ms apart */
#define VIBRATOR_SUSPEND_ABORTS	3	/* in a row, then sleep anyway */
#define VIBRATOR_MAX_MS		15000

/* The vendor flag API is unsigned-short, including wrapped negative errno. */
static int vibrator_write(PMU_FLAGS_LIST_ENUM field, unsigned int value)
{
	unsigned short ret = pmic_set_register_value(field, value);

	if (!ret)
		return 0;
	return ret == (unsigned short)-EPERM ? -EPERM : -EIO;
}

static int vibrator_set(struct m681_vibrator *vib, bool enabled)
{
	int ret;

	ret = vibrator_write(MT6351_PMIC_RG_VIBR_EN, enabled);
	if (!ret)
		vib->enabled = enabled;
	return ret;
}

/* under state_lock: the off write, tried again on a PMIC wrapper error
 * (-EPERM is final, though the EN row lets an off write through anyway) */
static int vibrator_off(struct m681_vibrator *vib)
{
	int ret, i;

	for (i = 0; i < VIBRATOR_OFF_TRIES; i++) {
		ret = vibrator_set(vib, false);
		if (!ret || ret == -EPERM)
			break;
		usleep_range(1000, 2000);
	}
	return ret;
}

static void vibrator_stop(struct work_struct *work)
{
	struct m681_vibrator *vib = container_of(to_delayed_work(work),
					struct m681_vibrator, stop);
	bool again = false;
	int ret;

	mutex_lock(&vib->state_lock);
	ret = vibrator_off(vib);
	if (!ret)
		vib->stop_retries = 0;
	else if (vib->stop_retries < VIBRATOR_STOP_RETRIES)
		again = ++vib->stop_retries;
	mutex_unlock(&vib->state_lock);
	if (ret)
		dev_err(vib->led.dev, "vibrator disable failed: %d%s\n", ret,
			again ? ", trying again in 100 ms" : "");
	if (again)
		schedule_delayed_work(&vib->stop, msecs_to_jiffies(100));
}

static int vibrator_request(struct m681_vibrator *vib, unsigned int duration)
{
	int ret;

	if (duration > VIBRATOR_MAX_MS)
		return -ERANGE;
	mutex_lock(&vib->request_lock);
	cancel_delayed_work_sync(&vib->stop);
	mutex_lock(&vib->state_lock);
	if (vib->stopping) {
		ret = -ESHUTDOWN;
	} else if (vib->suspended) {
		ret = -EBUSY;
	} else {
		ret = vibrator_set(vib, duration != 0);
		if (!ret && duration) {
			vib->duration_ms = duration;
			vib->stop_retries = 0;
			schedule_delayed_work(&vib->stop, msecs_to_jiffies(duration));
		} else if (ret && vib->enabled) {
			/* A failed replacement must not discard the old stop request. */
			vib->stop_retries = 0;
			schedule_delayed_work(&vib->stop, 1);
		}
	}
	mutex_unlock(&vib->state_lock);
	mutex_unlock(&vib->request_lock);
	return ret;
}

static struct m681_vibrator *vibrator_from_dev(struct device *dev)
{
	return container_of(dev_get_drvdata(dev), struct m681_vibrator, led);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct m681_vibrator *vib = vibrator_from_dev(dev);
	bool enabled;

	mutex_lock(&vib->state_lock);
	enabled = vib->enabled;
	mutex_unlock(&vib->state_lock);
	return scnprintf(buf, PAGE_SIZE, "%u\n", enabled);
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct m681_vibrator *vib = vibrator_from_dev(dev);
	unsigned int value, duration;
	int ret = kstrtouint(buf, 10, &value);

	if (ret)
		return ret;
	if (value > 1)
		return -EINVAL;
	mutex_lock(&vib->state_lock);
	duration = value ? vib->duration_ms : 0;
	mutex_unlock(&vib->state_lock);
	ret = vibrator_request(vib, duration);
	return ret ? ret : count;
}

static ssize_t duration_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned int duration;
	int ret = kstrtouint(buf, 10, &duration);

	if (!ret)
		ret = vibrator_request(vibrator_from_dev(dev), duration);
	return ret ? ret : count;
}

static DEVICE_ATTR_RO(state);
static DEVICE_ATTR(activate, 0644, state_show, activate_store);
static DEVICE_ATTR_WO(duration);
static struct attribute *vibrator_attrs[] = {
	&dev_attr_state.attr, &dev_attr_activate.attr, &dev_attr_duration.attr, NULL
};
ATTRIBUTE_GROUPS(vibrator);

static int vibrator_brightness(struct led_classdev *led,
			      enum led_brightness brightness)
{
	struct m681_vibrator *vib = container_of(led, struct m681_vibrator, led);
	unsigned int duration;

	mutex_lock(&vib->state_lock);
	duration = brightness ? vib->duration_ms : 0;
	mutex_unlock(&vib->state_lock);
	return vibrator_request(vib, duration);
}

/* The LineageOS 16 vibrator HAL (vendor/mediatek/hidl/vibrator
 * service.cpp) opens /sys/class/timed_output/vibrator/{enable,vibr_vol}
 * and exits with -ENOENT without them; IVibrator is then never registered
 * and system_server waits on it forever. 4.9 has no staging timed_output,
 * so this driver publishes the same class itself, with the
 * timed_output.c semantics: no parent, i.e.
 * /sys/devices/virtual/timed_output/vibrator as in the sysfs_vibrator
 * genfscon; enable takes ms (0 = off), rounds a pulse between vib_limit
 * and vib_timer up to vib_timer, clamps at 15 s and always reports every
 * parsed write consumed (the HAL's std::ofstream stays dead after one
 * failed write, so a refusal is logged instead; only text that is not a
 * number is rejected). vibr_vol is kept and ignored: VOSEL is fixed at
 * the stock code (the mt_pmic_stub VIBR row allows 5 only). */
static struct class *timed_output_class;

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);
	unsigned int left = 0;
	long rem;

	mutex_lock(&vib->state_lock);
	if (vib->enabled && delayed_work_pending(&vib->stop)) {
		rem = (long)(vib->stop.timer.expires - jiffies);
		if (rem > 0)
			left = jiffies_to_msecs(rem);
	}
	mutex_unlock(&vib->state_lock);
	return scnprintf(buf, PAGE_SIZE, "%u\n", left);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);
	unsigned int value, duration = 0;
	int ret = kstrtouint(buf, 0, &value);

	/* the HAL writes a uint32_t: every value it can send must parse */
	if (ret)
		return -EINVAL;
	if (value) {
		duration = min_t(unsigned int, value, VIBRATOR_MAX_MS);
		if (duration > vib->limit_ms && duration < vib->timer_ms)
			duration = vib->timer_ms;
	}
	ret = vibrator_request(vib, duration);
	if (ret)
		dev_warn_ratelimited(dev, "timed_output enable %u refused: %d\n",
				     value, ret);
	return count;
}

static ssize_t vibr_vol_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", READ_ONCE(vib->vibr_vol));
}

static ssize_t vibr_vol_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);
	unsigned int vol;

	if (kstrtouint(buf, 0, &vol) || vol > 8)
		return -EINVAL;
	WRITE_ONCE(vib->vibr_vol, vol);
	return count;
}

static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(vibr_vol);
static struct attribute *timed_output_attrs[] = {
	&dev_attr_enable.attr, &dev_attr_vibr_vol.attr, NULL
};
ATTRIBUTE_GROUPS(timed_output);

static void vibrator_timed_output_remove(void *data)
{
	struct m681_vibrator *vib = data;

	device_unregister(vib->timed_dev);
	class_destroy(timed_output_class);
	timed_output_class = NULL;
}

/* A failure here leaves the LED interface working: log, do not fail. */
static void vibrator_timed_output_add(struct platform_device *pdev,
				      struct m681_vibrator *vib)
{
	int ret;

	timed_output_class = class_create(THIS_MODULE, "timed_output");
	if (IS_ERR(timed_output_class)) {
		ret = PTR_ERR(timed_output_class);
		timed_output_class = NULL;
		goto fail;
	}
	vib->timed_dev = device_create_with_groups(timed_output_class, NULL,
						   MKDEV(0, 0), vib,
						   timed_output_groups,
						   "vibrator");
	if (IS_ERR(vib->timed_dev)) {
		ret = PTR_ERR(vib->timed_dev);
		class_destroy(timed_output_class);
		timed_output_class = NULL;
		goto fail;
	}
	ret = devm_add_action_or_reset(&pdev->dev, vibrator_timed_output_remove,
				       vib);
	if (!ret)
		return;
fail:
	dev_err(&pdev->dev, "timed_output/vibrator not created: %d\n", ret);
}

static int m681_vibrator_probe(struct platform_device *pdev)
{
	struct m681_vibrator *vib;
	u32 voltage;
	int ret;

	vib = devm_kzalloc(&pdev->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;
	ret = of_property_read_u32(pdev->dev.of_node, "vib_vol", &voltage);
	if (ret)
		return ret;
	if (voltage > 7)
		return -EINVAL;
	ret = of_property_read_u32(pdev->dev.of_node, "vib_timer", &vib->duration_ms);
	if (ret)
		return ret;
	if (!vib->duration_ms || vib->duration_ms > VIBRATOR_MAX_MS)
		return -EINVAL;
	vib->timer_ms = vib->duration_ms;
	vib->limit_ms = 9;	/* stock CUST_VIBR_LIMIT default */
	of_property_read_u32(pdev->dev.of_node, "vib_limit", &vib->limit_ms);
	vib->vibr_vol = voltage;
	mutex_init(&vib->request_lock);
	mutex_init(&vib->state_lock);
	INIT_DELAYED_WORK(&vib->stop, vibrator_stop);
	/* Do not publish a working actuator when the PMIC refuses initialization. */
	ret = vibrator_set(vib, false);
	if (ret)
		return ret;
	ret = vibrator_write(MT6351_PMIC_RG_VIBR_VOSEL, voltage);
	if (ret)
		return ret;
	vib->led.name = "vibrator";
	vib->led.max_brightness = 1;
	vib->led.brightness_set_blocking = vibrator_brightness;
	vib->led.groups = vibrator_groups;
	platform_set_drvdata(pdev, vib);
	ret = devm_led_classdev_register(&pdev->dev, &vib->led);
	if (ret)
		return ret;
	vibrator_timed_output_add(pdev, vib);
	return 0;
}

static void m681_vibrator_shutdown(struct platform_device *pdev)
{
	struct m681_vibrator *vib = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&vib->request_lock);
	cancel_delayed_work_sync(&vib->stop);
	mutex_lock(&vib->state_lock);
	vib->stopping = true;
	ret = vibrator_off(vib);
	mutex_unlock(&vib->state_lock);
	mutex_unlock(&vib->request_lock);
	if (ret)
		dev_err(&pdev->dev, "vibrator shutdown failed: %d\n", ret);
}

static int m681_vibrator_remove(struct platform_device *pdev)
{
	m681_vibrator_shutdown(pdev);
	return 0;
}

/* The motor never runs through system sleep: the stop timer is cancelled and
 * EN is written 0 before suspend; requests are refused until resume. A failed
 * off write aborts the suspend only while the motor may still run (enabled),
 * and at most VIBRATOR_SUSPEND_ABORTS times in a row: a broken PMIC wrapper
 * must not keep the phone awake for good. */
static int m681_vibrator_suspend(struct device *dev)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);
	bool abort = false;
	int ret;

	mutex_lock(&vib->request_lock);
	cancel_delayed_work_sync(&vib->stop);
	mutex_lock(&vib->state_lock);
	ret = vibrator_off(vib);
	if (ret && vib->enabled &&
	    vib->suspend_aborts < VIBRATOR_SUSPEND_ABORTS) {
		abort = true;
		vib->suspend_aborts++;
		vib->stop_retries = 0;
		schedule_delayed_work(&vib->stop, 1);
	} else {
		vib->suspend_aborts = 0;
		vib->suspended = true;
	}
	mutex_unlock(&vib->state_lock);
	mutex_unlock(&vib->request_lock);
	if (ret)
		dev_err(dev, "vibrator off before suspend failed: %d, %s\n", ret,
			abort ? "suspend aborted" : "suspending anyway");
	return abort ? ret : 0;
}

static int m681_vibrator_resume(struct device *dev)
{
	struct m681_vibrator *vib = dev_get_drvdata(dev);

	mutex_lock(&vib->state_lock);
	vib->suspended = false;
	/* slept with the off write failed ("suspending anyway"): stop again */
	if (vib->enabled) {
		vib->stop_retries = 0;
		schedule_delayed_work(&vib->stop, 1);
	}
	mutex_unlock(&vib->state_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(m681_vibrator_pm, m681_vibrator_suspend,
			 m681_vibrator_resume);

static const struct of_device_id m681_vibrator_match[] = {
	{ .compatible = "mediatek,vibrator" }, { }
};
MODULE_DEVICE_TABLE(of, m681_vibrator_match);
static struct platform_driver m681_vibrator_driver = {
	.probe = m681_vibrator_probe,
	.remove = m681_vibrator_remove,
	.shutdown = m681_vibrator_shutdown,
	.driver = {
		.name = "m681-vibrator",
		.pm = &m681_vibrator_pm,
		.of_match_table = m681_vibrator_match,
	},
};
module_platform_driver(m681_vibrator_driver);
MODULE_LICENSE("GPL");
