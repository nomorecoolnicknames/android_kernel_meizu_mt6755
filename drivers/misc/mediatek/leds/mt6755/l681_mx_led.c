/*
 * L681 (Meizu M3 Note L681H): "mx-led", the round mBack/home-key LED.
 *
 * As in the stock L681H kernel:
 *  - the board LED list has {"mx-led", MT65XX_LED_MODE_PMIC,
 *    MT65XX_LED_PMIC_NLED_ISINK0}: MT6351 current sink ISINK0;
 *  - mt_brightness_set_pmic / mt_led_blink_pmic, ISINK0 case, use the
 *    fields below (the same addresses, masks and shifts are MT6351_PMIC_*
 *    in mach/mt6351_regs.h);
 *  - the Meizu "blink" attribute: non-zero -> led_blink_set(984, 984),
 *    0 -> led_trigger_remove() + led_blink_set(0, 0);
 *  - the stock lights HAL (blink_home) writes "%d" of (onMS != 0 &&
 *    offMS != 0) to /sys/class/leds/mx-led/blink, nothing to its
 *    brightness.
 *
 * This kernel is built with CONFIG_MTK_PMIC_CHIP_MT6353, so the mtk_leds
 * PMIC paths use MT6353 fields; the L681 PMIC is an MT6351 (HWCID 0x51xx),
 * hence this small driver writes the MT6351 fields directly, and only after
 * reading HWCID. Nothing is written to the PMIC until userspace asks for
 * light.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/m3note_board.h>
#include <mt-plat/upmu_common.h>

#define L681_MT6351_HWCID		0x0200
/* MT6351 fields, stock pmu_flags_table id -> reg/mask/shift */
#define L681_TOP_CKPDN_CON0		0x023A	/* 0x13d [3] 32K_CK_PDN, 0x13e [4] ISINK0_CK_PDN */
#define L681_TOP_CKSEL_CON0		0x025E	/* 0x196 [2] ISINK0_CK_CKSEL */
#define L681_ISINK0_CON0		0x0810	/* 0x62c [15:0] DIM0_FSEL */
#define L681_ISINK0_CON1		0x0812	/* 0x62f [11:7] DIM0_DUTY, 0x630 [14:12] CH0_STEP */
#define L681_ISINK0_CON2		0x0814	/* 0x634 TR1 [15:12], 0x633 TR2 [11:8], 0x632 TF1 [7:4], 0x631 TF2 [3:0] */
#define L681_ISINK0_CON3		0x0816	/* 0x636 TON [11:8], 0x635 TOFF [3:0] */
#define L681_ISINK_EN_CTRL		0x0836	/* 0x66d [0] CH0_EN */
#define L681_ISINK_MODE_CTRL		0x0838	/* 0x67d [15:14] CH0_MODE */

#define L681_ISINK_PWM_MODE		0
#define L681_ISINK_BREATH_MODE		1
#define L681_ISINK_STEP_3		3	/* stock ISINK_3, 16 mA */
#define L681_MX_BLINK_MS		984	/* stock led_blink_mz_store */

/* stock pmic_period_array / pmic_freqsel_array (@0xffffffc0010051e0/0x200) */
static const int l681_period_ms[] = { 250, 500, 1000, 1250, 1666, 2000, 2500, 10000 };
static const int l681_freqsel[] = { 0, 4, 199, 499, 999, 1999, 1999, 1999 };

enum l681_mx_op { L681_MX_NONE, L681_MX_OFF, L681_MX_ON, L681_MX_BREATH };

struct l681_mx_led {
	struct led_classdev cdev;
	struct work_struct work;
	spinlock_t lock;
	enum l681_mx_op op;
	unsigned int duty, fsel;
	unsigned long blink_mz;
};

static bool l681_mx_pmic_is_mt6351(void)
{
	unsigned int hwcid = 0;

	pmic_read_interface(L681_MT6351_HWCID, &hwcid, 0xFFFF, 0);
	if ((hwcid >> 8) != 0x51) {
		pr_err("[FORGE_LED] mx-led: PMIC HWCID 0x%04x is not MT6351, no write\n",
		       hwcid);
		return false;
	}
	return true;
}

/* stock mt_brightness_set_pmic(ISINK0, level), same order and values */
static void l681_mx_pwm(bool on)
{
	pmic_config_interface(L681_TOP_CKPDN_CON0, 0, 0x1, 3);
	pmic_config_interface(L681_TOP_CKPDN_CON0, 0, 0x1, 4);
	pmic_config_interface(L681_TOP_CKSEL_CON0, 0, 0x1, 2);
	pmic_config_interface(L681_ISINK_MODE_CTRL, L681_ISINK_PWM_MODE, 0x3, 14);
	pmic_config_interface(L681_ISINK0_CON1, L681_ISINK_STEP_3, 0x7, 12);
	pmic_config_interface(L681_ISINK0_CON1, 15, 0x1F, 7);
	pmic_config_interface(L681_ISINK0_CON0, 0, 0xFFFF, 0);
	if (on) {
		pmic_config_interface(L681_TOP_CKPDN_CON0, 0, 0x1, 3);
		pmic_config_interface(L681_ISINK_EN_CTRL, 1, 0x1, 0);
	} else {
		pmic_config_interface(L681_ISINK_EN_CTRL, 0, 0x1, 0);
	}
}

/*
 * stock mt_led_blink_pmic(ISINK0), same order and values. The ISINK0 clock
 * un-gate (CK_PDN/CKSEL = 0, stock brightness path values) is added in
 * front: stock relies on an earlier brightness write having done it.
 */
static void l681_mx_breath(unsigned int duty, unsigned int fsel)
{
	pmic_config_interface(L681_TOP_CKPDN_CON0, 0, 0x1, 4);
	pmic_config_interface(L681_TOP_CKSEL_CON0, 0, 0x1, 2);
	pmic_config_interface(L681_TOP_CKPDN_CON0, 0, 0x1, 3);
	pmic_config_interface(L681_ISINK_MODE_CTRL, L681_ISINK_BREATH_MODE, 0x3, 14);
	pmic_config_interface(L681_ISINK0_CON1, L681_ISINK_STEP_3, 0x7, 12);
	pmic_config_interface(L681_ISINK0_CON2, 0, 0xF, 12);	/* TR1 */
	pmic_config_interface(L681_ISINK0_CON2, 1, 0xF, 8);	/* TR2 */
	pmic_config_interface(L681_ISINK0_CON2, 0, 0xF, 4);	/* TF1 */
	pmic_config_interface(L681_ISINK0_CON2, 1, 0xF, 0);	/* TF2 */
	pmic_config_interface(L681_ISINK0_CON3, 2, 0xF, 8);	/* TON */
	pmic_config_interface(L681_ISINK0_CON3, 2, 0xF, 0);	/* TOFF */
	pmic_config_interface(L681_ISINK0_CON1, duty, 0x1F, 7);
	pmic_config_interface(L681_ISINK0_CON0, fsel, 0xFFFF, 0);
	pmic_config_interface(L681_ISINK_EN_CTRL, 1, 0x1, 0);
}

static void l681_mx_work(struct work_struct *work)
{
	struct l681_mx_led *mx = container_of(work, struct l681_mx_led, work);
	unsigned int en = 0, mode = 0, con1 = 0, con0 = 0;
	unsigned long flags;
	enum l681_mx_op op;
	unsigned int duty, fsel;

	spin_lock_irqsave(&mx->lock, flags);
	op = mx->op;
	duty = mx->duty;
	fsel = mx->fsel;
	mx->op = L681_MX_NONE;
	spin_unlock_irqrestore(&mx->lock, flags);

	if (op == L681_MX_NONE || !l681_mx_pmic_is_mt6351())
		return;

	if (op == L681_MX_BREATH)
		l681_mx_breath(duty, fsel);
	else
		l681_mx_pwm(op == L681_MX_ON);

	pmic_read_interface(L681_ISINK_EN_CTRL, &en, 0x1, 0);
	pmic_read_interface(L681_ISINK_MODE_CTRL, &mode, 0x3, 14);
	pmic_read_interface(L681_ISINK0_CON1, &con1, 0xFFFF, 0);
	pmic_read_interface(L681_ISINK0_CON0, &con0, 0xFFFF, 0);
	pr_info("[FORGE_LED] mx-led %s: CH0_EN=%u MODE=%u ISINK0_CON1=0x%04x FSEL=%u\n",
		op == L681_MX_BREATH ? "breath" : (op == L681_MX_ON ? "on" : "off"),
		en, mode, con1, con0);
}

static void l681_mx_queue(struct l681_mx_led *mx, enum l681_mx_op op,
			  unsigned int duty, unsigned int fsel)
{
	unsigned long flags;

	spin_lock_irqsave(&mx->lock, flags);
	mx->op = op;
	mx->duty = duty;
	mx->fsel = fsel;
	spin_unlock_irqrestore(&mx->lock, flags);
	schedule_work(&mx->work);
}

/* may run in atomic context (software blink timer): defer the PMIC writes */
static void l681_mx_brightness_set(struct led_classdev *cdev,
				   enum led_brightness level)
{
	struct l681_mx_led *mx = container_of(cdev, struct l681_mx_led, cdev);

	l681_mx_queue(mx, level ? L681_MX_ON : L681_MX_OFF, 0, 0);
}

/* stock mt_mt65xx_blink_set for a PMIC ISINK: breath when both delays are
 * set, off when both are 0, software blink otherwise */
static int l681_mx_blink_set(struct led_classdev *cdev,
			     unsigned long *delay_on, unsigned long *delay_off)
{
	struct l681_mx_led *mx = container_of(cdev, struct l681_mx_led, cdev);
	unsigned long period;
	int i;

	if (!*delay_on && !*delay_off) {
		l681_mx_queue(mx, L681_MX_OFF, 0, 0);
		return 0;
	}
	if (!*delay_on || !*delay_off)
		return -EINVAL;

	period = *delay_on + *delay_off;
	for (i = 0; i < ARRAY_SIZE(l681_period_ms) - 1; i++)
		if (period <= l681_period_ms[i])
			break;
	l681_mx_queue(mx, L681_MX_BREATH, 32 * *delay_on / period,
		      l681_freqsel[i]);
	return 0;
}

static ssize_t blink_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct l681_mx_led *mx = container_of(cdev, struct l681_mx_led, cdev);

	return sprintf(buf, "%lu\n", mx->blink_mz);
}

/* stock led_blink_mz_store */
static ssize_t blink_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct l681_mx_led *mx = container_of(cdev, struct l681_mx_led, cdev);
	unsigned long state, delay;
	int ret;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;

	if (state == LED_OFF)
		led_trigger_remove(cdev);
	mx->blink_mz = state;
	if (!(cdev->flags & LED_SUSPENDED)) {
		delay = state ? L681_MX_BLINK_MS : 0;
		led_blink_set(cdev, &delay, &delay);
	}
	return size;
}
static DEVICE_ATTR_RW(blink);

static struct attribute *l681_mx_attrs[] = {
	&dev_attr_blink.attr,
	NULL,
};
static const struct attribute_group l681_mx_group = {
	.attrs = l681_mx_attrs,
};
static const struct attribute_group *l681_mx_groups[] = {
	&l681_mx_group,
	NULL,
};

static int __init l681_mx_led_init(void)
{
	struct l681_mx_led *mx;
	unsigned int hwcid = 0;
	int ret;

	if (!m3note_board_is(M3NOTE_BOARD_L681))
		return -ENODEV;

	pmic_read_interface(L681_MT6351_HWCID, &hwcid, 0xFFFF, 0);
	if ((hwcid >> 8) != 0x51) {
		pr_err("[FORGE_LED] mx-led: PMIC HWCID 0x%04x is not MT6351, not registered\n",
		       hwcid);
		return -ENODEV;
	}

	mx = kzalloc(sizeof(*mx), GFP_KERNEL);
	if (!mx)
		return -ENOMEM;
	spin_lock_init(&mx->lock);
	INIT_WORK(&mx->work, l681_mx_work);
	mx->cdev.name = "mx-led";
	mx->cdev.max_brightness = LED_FULL;
	mx->cdev.brightness_set = l681_mx_brightness_set;
	mx->cdev.blink_set = l681_mx_blink_set;
	mx->cdev.groups = l681_mx_groups;

	ret = led_classdev_register(NULL, &mx->cdev);
	if (ret) {
		pr_err("[FORGE_LED] mx-led: led_classdev_register %d\n", ret);
		kfree(mx);
		return ret;
	}
	pr_info("[FORGE_LED] mx-led on MT6351 ISINK0 (HWCID 0x%04x), no PMIC write until used\n",
		hwcid);
	return 0;
}
late_initcall(l681_mx_led_init);
