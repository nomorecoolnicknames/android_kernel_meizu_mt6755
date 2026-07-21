/*
 * L681 main flash: the "leds-LM3642" chip at i2c1 0x63.
 *
 * Behaviour of the stock L681H kernel (constant_flashlight/leds_strobe.c):
 *  - init: GPIO9 (DCT GPIO_FLASH_LED_EN) mode 0 / out / 1, then reg 0x01 =
 *    0x00, reg 0x08 = 0x1F, reg 0x05 = 0x00, reg 0x03 = 0x00, then reads
 *    reg 0x0C: 0x02 = "LM3643", 0x18 = "SY7806". The name says LM3642, the
 *    register map is the LM3643 one (0x01 enable, 0x03/0x04 LED1/LED2
 *    flash, 0x05/0x06 LED1/LED2 torch, 0x08 timing, 0x0C device id).
 *    GPIO16/18 (DCT flash mode/en) are not driven.
 *  - uninit: enable reg 0, then GPIO9 low.
 *  - duty clamped to 0..16; isMovieMode = {1,1,1,0...}: duty 0..2 torch,
 *    3..16 flash; torchDuty = {32,74,108}; flashDuty = {3,8,12,19,...,84}.
 *    m_duty1 (main strobe 1 = ct 0) -> LED2 regs 0x06/0x04, m_duty2 (main
 *    strobe 2, strobe_main_sid2_part1 = ct 1) -> LED1 regs 0x05/0x03; both
 *    on and not both torch -> flash codes for both.
 *  - enable reg 0x00 off, LED1 only 0x09 torch / 0x0D flash, LED2 only
 *    0x0A / 0x0E, both 0x0B / 0x0F.
 * Currents (LM3643 datasheet): torch 0.977 + 1.4 * code mA (45.8, 104.6,
 * 152.2 mA), flash 1.954 + 11.72 * code mA (37 mA .. 986 mA at code 84);
 * reg 0x08 = 0x1F = flash timeout 400 ms in hardware, torch ramp 1 ms.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/m3note_board.h>
#include <mt_gpio.h>

#include "flashlight.h"

#define LM3642_NAME		"flashlights-lm3642"
#define LM3642_DTNAME_I2C	"mediatek,strobe_main"	/* l681 cust.dtsi i2c1 strobe_main@63 */

#define LM3642_GPIO_HWEN	(9 | 0x80000000)	/* stock GPIO_FLASH_LED_EN */

#define LM3642_REG_ENABLE	0x01
#define LM3642_REG_FLASH_LED1	0x03
#define LM3642_REG_FLASH_LED2	0x04
#define LM3642_REG_TORCH_LED1	0x05
#define LM3642_REG_TORCH_LED2	0x06
#define LM3642_REG_TIMING	0x08
#define LM3642_REG_DEVICE_ID	0x0C

#define LM3642_CHANNEL_NUM	2	/* ch0 = ct 0 = LED2, ch1 = ct 1 = LED1 */
#define LM3642_DUTY_NUM		17
#define LM3642_DUTY_TORCH	3	/* duty < 3: torch */

static const u8 lm3642_torch_code[LM3642_DUTY_TORCH] = { 32, 74, 108 };
static const u8 lm3642_flash_code[LM3642_DUTY_NUM] = {
	3, 8, 12, 19, 24, 29, 34, 39, 44, 49, 54, 59, 64, 69, 74, 79, 84 };

/*
 * System torch (QS / CameraManager.setTorchMode). Stock L681H: SystemUI ->
 * MeizuCamera.openTorch -> libmeizucamera writes the platform default
 * level "3" to /sys/class/flashlightdrv/kd_camera_flashlight/flash1, and
 * the kernel sets reg 0x06 (LED2 torch) = torchDuty_T[3] = 71
 * ({17,35,53,71,89,106}), LED2 only (enable 0x3A; the TORCH/STROBE pin
 * enable bits are left out here). The vendor HAL3A (FlashHal::setTorchOnOff)
 * gives strobe 1 and, with dual flash, strobe 2 setDuty(0),
 * setTimeOutTime(0), setOnOff(1). A request that is torch duty with timeout
 * 0 on every lit channel is taken as this system torch and lit as stock:
 * LED2 at 71 (~100 mA), LED1 off. Camera torch/flash with a timeout keeps
 * the stock camera tables.
 */
#define LM3642_SYS_TORCH_CODE	71

static struct i2c_client *lm3642_client;
static DEFINE_MUTEX(lm3642_mutex);
static int lm3642_use_count;
static int lm3642_duty[LM3642_CHANNEL_NUM];
static bool lm3642_on[LM3642_CHANNEL_NUM];
static unsigned int lm3642_timeout_ms[LM3642_CHANNEL_NUM] = { 1000, 1000 };
static struct hrtimer lm3642_timer[LM3642_CHANNEL_NUM];
static struct work_struct lm3642_work[LM3642_CHANNEL_NUM];

static int lm3642_devid = -1;
module_param_named(devid, lm3642_devid, int, 0444);
MODULE_PARM_DESC(devid, "l681: flash chip reg 0x0C at probe (0x02 LM3643, 0x18 SY7806, -1 never read)");

static int lm3642_write(u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(lm3642_client, reg, val);

	if (ret < 0)
		fl_err("write 0x%02x=0x%02x failed %d\n", reg, val, ret);
	return ret;
}

static void lm3642_hwen(int on)
{
	mt_set_gpio_out(LM3642_GPIO_HWEN, on ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

/* stock init_LM3642 */
static void lm3642_chip_init(void)
{
	mt_set_gpio_mode(LM3642_GPIO_HWEN, GPIO_MODE_00);
	mt_set_gpio_dir(LM3642_GPIO_HWEN, GPIO_DIR_OUT);
	lm3642_hwen(1);
	if (!lm3642_write(LM3642_REG_ENABLE, 0x00))
		lm3642_write(LM3642_REG_TIMING, 0x1F);
	lm3642_write(LM3642_REG_TORCH_LED1, 0x00);
	lm3642_write(LM3642_REG_FLASH_LED1, 0x00);
}

/* stock FL_Uninit */
static void lm3642_chip_uninit(void)
{
	lm3642_on[0] = lm3642_on[1] = false;
	lm3642_write(LM3642_REG_ENABLE, 0x00);
	lm3642_hwen(0);
}

static bool lm3642_is_torch(int ch)
{
	return lm3642_duty[ch] < LM3642_DUTY_TORCH;
}

/* stock setDuty_LM3642_led2 + flashEnable_LM3642_led2; lm3642_mutex held */
static int lm3642_apply(void)
{
	bool led2 = lm3642_on[0], led1 = lm3642_on[1];
	bool torch;

	if (!led1 && !led2)
		return lm3642_write(LM3642_REG_ENABLE, 0x00);

	if ((!led2 || (lm3642_is_torch(0) && !lm3642_timeout_ms[0])) &&
	    (!led1 || (lm3642_is_torch(1) && !lm3642_timeout_ms[1]))) {
		fl_dbg("system torch: LED2 code %d, LED1 off\n", LM3642_SYS_TORCH_CODE);
		lm3642_write(LM3642_REG_TORCH_LED2, LM3642_SYS_TORCH_CODE);
		return lm3642_write(LM3642_REG_ENABLE, 0x0A);
	}

	torch = (!led2 || lm3642_is_torch(0)) && (!led1 || lm3642_is_torch(1));
	if (led2)
		lm3642_write(torch ? LM3642_REG_TORCH_LED2 : LM3642_REG_FLASH_LED2,
			     torch ? lm3642_torch_code[lm3642_duty[0]] :
				     lm3642_flash_code[lm3642_duty[0]]);
	if (led1)
		lm3642_write(torch ? LM3642_REG_TORCH_LED1 : LM3642_REG_FLASH_LED1,
			     torch ? lm3642_torch_code[lm3642_duty[1]] :
				     lm3642_flash_code[lm3642_duty[1]]);
	fl_dbg("ch0(LED2) %d/%d ch1(LED1) %d/%d %s\n", led2, lm3642_duty[0],
	       led1, lm3642_duty[1], torch ? "torch" : "flash");
	return lm3642_write(LM3642_REG_ENABLE,
			    (torch ? 0x08 : 0x0C) | (led2 ? 0x02 : 0) | (led1 ? 0x01 : 0));
}

static void lm3642_set_duty(int ch, int duty)
{
	if (duty < 0)
		duty = 0;
	else if (duty >= LM3642_DUTY_NUM)
		duty = LM3642_DUTY_NUM - 1;
	lm3642_duty[ch] = duty;
}

static void lm3642_work_off(struct work_struct *work)
{
	int ch = work == &lm3642_work[1];

	mutex_lock(&lm3642_mutex);
	if (lm3642_use_count && lm3642_on[ch]) {
		lm3642_on[ch] = false;
		lm3642_apply();
	}
	mutex_unlock(&lm3642_mutex);
}

static enum hrtimer_restart lm3642_timer_func(struct hrtimer *timer)
{
	schedule_work(&lm3642_work[timer == &lm3642_timer[1]]);
	return HRTIMER_NORESTART;
}

static int lm3642_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg = (struct flashlight_dev_arg *)arg;
	int ch = fl_arg->channel;
	unsigned int ms;

	if (ch < 0 || ch >= LM3642_CHANNEL_NUM)
		return -EINVAL;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		fl_dbg("ch%d timeout %d ms\n", ch, fl_arg->arg);
		lm3642_timeout_ms[ch] = fl_arg->arg;
		break;
	case FLASH_IOC_SET_DUTY:
		fl_dbg("ch%d duty %d\n", ch, fl_arg->arg);
		mutex_lock(&lm3642_mutex);
		lm3642_set_duty(ch, fl_arg->arg);
		if (lm3642_use_count && lm3642_on[ch])
			lm3642_apply();
		mutex_unlock(&lm3642_mutex);
		break;
	case FLASH_IOC_SET_ONOFF:
		fl_dbg("ch%d onoff %d\n", ch, fl_arg->arg);
		if (fl_arg->arg == 1) {
			ms = lm3642_timeout_ms[ch];
			if (ms)
				hrtimer_start(&lm3642_timer[ch],
					      ktime_set(ms / 1000, (ms % 1000) * 1000000),
					      HRTIMER_MODE_REL);
		} else {
			hrtimer_cancel(&lm3642_timer[ch]);
		}
		mutex_lock(&lm3642_mutex);
		if (lm3642_use_count) {
			lm3642_on[ch] = fl_arg->arg == 1;
			lm3642_apply();
		} else {
			fl_err("ch%d onoff without set_driver\n", ch);
		}
		mutex_unlock(&lm3642_mutex);
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static int lm3642_open(void *pArg)
{
	return 0;
}

static int lm3642_set_driver(void)
{
	mutex_lock(&lm3642_mutex);
	if (!lm3642_use_count)
		lm3642_chip_init();
	lm3642_use_count++;
	mutex_unlock(&lm3642_mutex);
	return 0;
}

static int lm3642_release(void *pArg)
{
	int ch;

	mutex_lock(&lm3642_mutex);
	if (lm3642_use_count > 0 && !--lm3642_use_count) {
		mutex_unlock(&lm3642_mutex);
		for (ch = 0; ch < LM3642_CHANNEL_NUM; ch++) {
			hrtimer_cancel(&lm3642_timer[ch]);
			cancel_work_sync(&lm3642_work[ch]);
		}
		mutex_lock(&lm3642_mutex);
		if (!lm3642_use_count)
			lm3642_chip_uninit();
	}
	mutex_unlock(&lm3642_mutex);
	return 0;
}

static ssize_t lm3642_strobe_store(struct flashlight_arg arg)
{
	int ch = arg.channel;

	if (ch < 0 || ch >= LM3642_CHANNEL_NUM)
		return -EINVAL;
	lm3642_set_driver();
	mutex_lock(&lm3642_mutex);
	lm3642_set_duty(ch, arg.level);
	lm3642_on[ch] = true;
	lm3642_apply();
	mutex_unlock(&lm3642_mutex);
	msleep(arg.dur);
	mutex_lock(&lm3642_mutex);
	lm3642_on[ch] = false;
	lm3642_apply();
	mutex_unlock(&lm3642_mutex);
	lm3642_release(NULL);
	return 0;
}

static struct flashlight_operations lm3642_ops = {
	lm3642_open,
	lm3642_release,
	lm3642_ioctl,
	lm3642_strobe_store,
	lm3642_set_driver
};

static int lm3642_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ch;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* identity read with HWEN high, as stock init_LM3642 does */
	mt_set_gpio_mode(LM3642_GPIO_HWEN, GPIO_MODE_00);
	mt_set_gpio_dir(LM3642_GPIO_HWEN, GPIO_DIR_OUT);
	lm3642_hwen(1);
	usleep_range(1000, 1500);
	lm3642_devid = i2c_smbus_read_byte_data(client, LM3642_REG_DEVICE_ID);
	lm3642_hwen(0);
	pr_info("[FORGE_FLASH] l681 flash chip i2c%d 0x%02x: reg0C=0x%02x (%s)\n",
		client->adapter->nr, client->addr, lm3642_devid,
		lm3642_devid == 0x02 ? "LM3643" : lm3642_devid == 0x18 ? "SY7806" :
		lm3642_devid < 0 ? "no ACK" : "unknown id");
	if (lm3642_devid < 0)
		return -ENODEV;

	lm3642_client = client;
	for (ch = 0; ch < LM3642_CHANNEL_NUM; ch++) {
		INIT_WORK(&lm3642_work[ch], lm3642_work_off);
		hrtimer_init(&lm3642_timer[ch], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		lm3642_timer[ch].function = lm3642_timer_func;
	}
	if (flashlight_dev_register(LM3642_NAME, &lm3642_ops)) {
		lm3642_client = NULL;
		return -EFAULT;
	}
	return 0;
}

static int lm3642_i2c_remove(struct i2c_client *client)
{
	flashlight_dev_unregister(LM3642_NAME);
	lm3642_client = NULL;
	return 0;
}

static const struct i2c_device_id lm3642_i2c_id[] = {
	{ LM3642_NAME, 0 },
	{ }
};

static const struct of_device_id lm3642_i2c_of_match[] = {
	{ .compatible = LM3642_DTNAME_I2C },
	{ },
};

static struct i2c_driver lm3642_i2c_driver = {
	.driver = {
		.name = LM3642_NAME,
		.of_match_table = lm3642_i2c_of_match,
	},
	.probe = lm3642_i2c_probe,
	.remove = lm3642_i2c_remove,
	.id_table = lm3642_i2c_id,
};

static int __init lm3642_init(void)
{
	if (!m3note_board_is(M3NOTE_BOARD_L681))
		return -ENODEV;
	return i2c_add_driver(&lm3642_i2c_driver);
}
late_initcall(lm3642_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("l681 LM3642 (LM3643 regmap) flashlight");
