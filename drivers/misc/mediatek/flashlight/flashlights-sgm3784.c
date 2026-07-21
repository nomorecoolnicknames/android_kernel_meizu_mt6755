/*
 * SGM3784 flash/torch driver (M681 part1 units).
 *
 * The part is identified by its address: the LM3643-family address 0x63
 * on i2c1 does not ACK (-121) while 0x30 does, and the stock 3.10
 * constant_flashlight driver ("//SGM3784") declares STROBE_DEVICE_ID 0x60,
 * i.e. 7-bit 0x30.
 *
 * Register usage as in the stock leds_strobe.c:
 *
 *   registers   0x0F ENABLE, 0x02 TIMING, 0x03, 0x01 MODE,
 *               0x06/0x08 LED1 flash/torch, 0x09/0x0B LED2 flash/torch
 *   torch seq   STROBE low; 0x01=0xEA (assist mode, continuous current);
 *               0x02=0xFF; 0x03=0x48; currents into 0x08/0x0B; 0x0F=0x03;
 *               then GPIO_LED_GPIO high
 *   off         0x0F=0x00, GPIO low
 *   pins        EN GPIO99, STROBE GPIO9, GPIO GPIO89
 *   currents    the stock torch table maps register 2,3,4,5,6,7 to roughly
 *               37, 55, 74, 93, 112, 131 mA per LED
 *
 * Torch only: the high-current strobe is a thermal and battery-sag risk,
 * so the flash registers are written to zero and never driven.
 * forge_sgm_torch_reg sets the brightness and is clamped.
 *
 * The flashlight core's sysfs strobe path calls strobe_store() and then
 * sleeps arg.dur (at most 400 ms), useless as a torch. Here dur == 0 means
 * "leave it on until a level <= 0 write". The ioctl path
 * (FLASH_IOC_SET_ONOFF) keeps the vendor semantics the framework expects.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <mt_gpio.h>

#include "flashlight.h"
#include "flashlight-dt.h"

#define SGM_NAME "flashlights-sgm3784"

#define SGM_REG_MODE		0x01
#define SGM_REG_TIMING		0x02
#define SGM_REG_MISC		0x03
#define SGM_REG_FLASH_LED1	0x06
#define SGM_REG_TORCH_LED1	0x08
#define SGM_REG_FLASH_LED2	0x09
#define SGM_REG_TORCH_LED2	0x0B
#define SGM_REG_ENABLE		0x0F

#define SGM_MODE_ASSIST		0xEA	/* continuous LED current (torch) */
#define SGM_TIMING_TORCH	0xFF
#define SGM_MISC_DEFAULT	0x48
#define SGM_ENABLE_BOTH		0x03
#define SGM_ENABLE_OFF		0x00

static int forge_sgm_i2c_bus = 1;
module_param(forge_sgm_i2c_bus, int, 0444);
MODULE_PARM_DESC(forge_sgm_i2c_bus, "m681: i2c bus the SGM3784 sits on");

static int forge_sgm_i2c_addr = 0x30;
module_param(forge_sgm_i2c_addr, int, 0444);
MODULE_PARM_DESC(forge_sgm_i2c_addr, "m681: SGM3784 7-bit i2c address");

/* stock torch table: 2->37 mA, 3->55, 4->74, 5->93, 6->112, 7->131 per LED */
static int forge_sgm_torch_reg = 6;
module_param(forge_sgm_torch_reg, int, 0644);
MODULE_PARM_DESC(forge_sgm_torch_reg,
	"m681: torch current register value per LED (oracle table: 2=37mA .. 7=131mA), clamped to 1..15");

static int forge_sgm_en_gpio = 99;
module_param(forge_sgm_en_gpio, int, 0444);
static int forge_sgm_strobe_gpio = 9;
module_param(forge_sgm_strobe_gpio, int, 0444);
static int forge_sgm_ctrl_gpio = 89;
module_param(forge_sgm_ctrl_gpio, int, 0444);

/* the attach verdict, readable at any time -- the lesson: a
 * boot-time finding that can only be read during boot is not a finding */
static int forge_sgm_present = -1;
module_param(forge_sgm_present, int, 0444);
MODULE_PARM_DESC(forge_sgm_present,
	"m681: -1 not attempted, 0 no ACK at addr, 1 bound, 2 no i2c adapter");

static int forge_sgm_on;
module_param(forge_sgm_on, int, 0444);
MODULE_PARM_DESC(forge_sgm_on, "m681: 1 while the torch is lit");

/* Register dump: the SGM family reports LED open/short in its flag
 * registers. Write 1 to dump with the chip powered (EN raised for the read
 * and restored afterwards). Every access is a read.
 */
static int forge_sgm_dump;
static int sgm_dump_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops sgm_dump_ops = {
	.set = sgm_dump_set,
	.get = param_get_int,
};
module_param_cb(forge_sgm_dump, &sgm_dump_ops, &forge_sgm_dump, 0644);
MODULE_PARM_DESC(forge_sgm_dump,
	"m681: write 1 to dump SGM3784 registers 0x00..0x10 to dmesg (reads only; raises EN for the read)");

static struct i2c_client *sgm_client;
static DEFINE_MUTEX(sgm_mutex);
static int sgm_use_count;

static int sgm_write(u8 reg, u8 val)
{
	int ret;

	if (!sgm_client)
		return -ENODEV;
	ret = i2c_smbus_write_byte_data(sgm_client, reg, val);
	if (ret < 0)
		pr_err("[FORGE_SGM] write reg 0x%02x = 0x%02x failed: %d\n",
		       reg, val, ret);
	return ret;
}

static void sgm_en(int on)
{
	mt_set_gpio_out(forge_sgm_en_gpio, on ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

static int sgm_torch_on(void)
{
	int cur = forge_sgm_torch_reg;

	if (cur < 1)
		cur = 1;
	if (cur > 15)
		cur = 15;

	sgm_en(1);
	usleep_range(1000, 1500);
	mt_set_gpio_out(forge_sgm_strobe_gpio, GPIO_OUT_ZERO);

	if (sgm_write(SGM_REG_MODE, SGM_MODE_ASSIST) < 0)
		return -EIO;
	sgm_write(SGM_REG_TIMING, SGM_TIMING_TORCH);
	sgm_write(SGM_REG_MISC, SGM_MISC_DEFAULT);
	/* flash channels stay at zero: this driver never strobes */
	sgm_write(SGM_REG_FLASH_LED1, 0x00);
	sgm_write(SGM_REG_FLASH_LED2, 0x00);
	sgm_write(SGM_REG_TORCH_LED1, (u8)cur);
	sgm_write(SGM_REG_TORCH_LED2, (u8)cur);
	sgm_write(SGM_REG_ENABLE, SGM_ENABLE_BOTH);

	mt_set_gpio_out(forge_sgm_ctrl_gpio, GPIO_OUT_ONE);
	forge_sgm_on = 1;
	pr_err("[FORGE_SGM] torch ON at reg 0x%02x per LED\n", cur);
	return 0;
}

static void sgm_torch_off(void)
{
	sgm_write(SGM_REG_ENABLE, SGM_ENABLE_OFF);
	mt_set_gpio_out(forge_sgm_ctrl_gpio, GPIO_OUT_ZERO);
	mt_set_gpio_out(forge_sgm_strobe_gpio, GPIO_OUT_ZERO);
	sgm_en(0);
	forge_sgm_on = 0;
	pr_err("[FORGE_SGM] torch OFF\n");
}

static int sgm_dump_set(const char *val, const struct kernel_param *kp)
{
	int req = 0, ret, reg, was_on;

	ret = kstrtoint(val, 0, &req);
	if (ret)
		return ret;
	*(int *)kp->arg = req;
	if (req != 1)
		return 0;

	if (!sgm_client) {
		pr_err("[FORGE_SGM] dump: no i2c client\n");
		return 0;
	}

	/* the chip only answers with EN asserted; leave it as we found it */
	was_on = forge_sgm_on;
	if (!was_on) {
		sgm_en(1);
		usleep_range(2000, 2500);
	}

	for (reg = 0x00; reg <= 0x10; reg++) {
		ret = i2c_smbus_read_byte_data(sgm_client, (u8)reg);
		pr_err("[FORGE_SGM] reg 0x%02x = %s0x%02x\n", reg,
		       ret < 0 ? "ERR " : "", ret < 0 ? (u8)-ret : (u8)ret);
	}

	if (!was_on)
		sgm_en(0);

	return 0;
}

static int sgm_open(void *pArg)
{
	return 0;
}

static int sgm_release(void *pArg)
{
	mutex_lock(&sgm_mutex);
	sgm_use_count--;
	if (sgm_use_count <= 0) {
		sgm_use_count = 0;
		sgm_torch_off();
	}
	mutex_unlock(&sgm_mutex);
	return 0;
}

static int sgm_set_driver(void)
{
	mutex_lock(&sgm_mutex);
	sgm_use_count++;
	mutex_unlock(&sgm_mutex);
	return 0;
}

static int sgm_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg = (struct flashlight_dev_arg *)arg;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		/* the chip's own torch mode is continuous; the framework's
		 * timeout is honoured by its matching OFF call */
		break;
	case FLASH_IOC_SET_DUTY:
		if ((int)fl_arg->arg > 0)
			forge_sgm_torch_reg = (int)fl_arg->arg;
		break;
	case FLASH_IOC_SET_ONOFF:
		mutex_lock(&sgm_mutex);
		if ((int)fl_arg->arg == 1)
			sgm_torch_on();
		else
			sgm_torch_off();
		mutex_unlock(&sgm_mutex);
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static ssize_t sgm_strobe_store(struct flashlight_arg arg)
{
	mutex_lock(&sgm_mutex);
	if (arg.level <= 0) {
		sgm_torch_off();
	} else {
		sgm_torch_on();
		/* see the deviation note at the top: dur == 0 leaves it lit */
		if (arg.dur > 0) {
			mutex_unlock(&sgm_mutex);
			msleep(arg.dur);
			mutex_lock(&sgm_mutex);
			sgm_torch_off();
		}
	}
	mutex_unlock(&sgm_mutex);
	return 0;
}

static struct flashlight_operations sgm_ops = {
	sgm_open,
	sgm_release,
	sgm_ioctl,
	sgm_strobe_store,
	sgm_set_driver
};

/* non-destructive presence read, same shape as the lm3643 glue's */
static int sgm_probe_addr(struct i2c_adapter *adap, u16 addr)
{
	union i2c_smbus_data data;

	return i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
			      I2C_SMBUS_BYTE_DATA, &data) == 0 ? 1 : 0;
}

/*
 * The DTB has no node for this chip (as for lm3643), so the client is
 * created from code once the i2c adapter exists (late_initcall).
 */
static int __init forge_sgm_attach(void)
{
	struct i2c_board_info info;
	struct i2c_adapter *adap;
	int ret;

	adap = i2c_get_adapter(forge_sgm_i2c_bus);
	if (!adap) {
		pr_err("[FORGE_SGM] i2c adapter %d missing\n", forge_sgm_i2c_bus);
		forge_sgm_present = 2;
		return 0;
	}

	/* the chip only answers with EN asserted */
	sgm_en(1);
	usleep_range(2000, 2500);

	ret = sgm_probe_addr(adap, forge_sgm_i2c_addr);
	pr_err("[FORGE_SGM] presence read 0x%02x on i2c%d: %d (%s)\n",
	       forge_sgm_i2c_addr, forge_sgm_i2c_bus, ret,
	       ret == 1 ? "ACK" : "no ACK");

	if (ret == 1) {
		memset(&info, 0, sizeof(info));
		strlcpy(info.type, SGM_NAME, sizeof(info.type));
		info.addr = forge_sgm_i2c_addr;
		sgm_client = i2c_new_device(adap, &info);
		if (!sgm_client) {
			pr_err("[FORGE_SGM] i2c_new_device failed\n");
			forge_sgm_present = 0;
		} else {
			forge_sgm_present = 1;
			if (flashlight_dev_register(SGM_NAME, &sgm_ops))
				pr_err("[FORGE_SGM] flashlight_dev_register failed\n");
			else
				pr_err("[FORGE_SGM] registered as %s at 0x%02x\n",
				       SGM_NAME, forge_sgm_i2c_addr);
		}
	} else {
		forge_sgm_present = 0;
	}

	/* leave the chip unpowered until userspace asks for light */
	sgm_en(0);
	i2c_put_adapter(adap);
	return 0;
}
late_initcall(forge_sgm_attach);

MODULE_DESCRIPTION("m681 SGM3784 torch driver (ported from the 3.10 oracle)");
MODULE_LICENSE("GPL v2");
