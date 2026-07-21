/*
 * Senodia ST480 magnetometer.
 *
 * Wire protocol (command set, register init sequence, measurement framing,
 * board-0 axis assembly) as in the Meizu 3.10 st480.c (Senodia, GPL v2).
 *
 * The stock driver was a standalone input+misc driver (it registered its
 * own "msensor" misc device, which would collide with the 4.4 mag
 * framework's /dev/msensor). This port uses the MTK mag control/data-path
 * framework, with akm09911-new as the template. The stock meizu_class
 * factory-test device is not carried over.
 *
 * Address: 0x0d on i2c1 (stock board_info). The msensor@0c DT client node
 * is a shared probe seed: akm09911 (earlier in link order) probes it, fails
 * its chip check on ST480 units (-121 NAK) and stays unbound; this probe
 * takes the client and rewrites client->addr.
 *
 * Copyright (C) 2012 Senodia.
 * Author: Tori Xu <tori.xz.xu@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */
#include <linux/module.h>
#include <linux/m3note_board.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/atomic.h>

#include <hwmsen_helper.h>
#include <hwmsen_dev.h>	/* SENSOR_STATUS_* */

#include "cust_mag.h"
#include "mag.h"
#include "st480.h"

#define ST480_DEV_NAME "st480"

#define ST480_TAG "[st480] "
#define ST480_ERR(fmt, args...) pr_err(ST480_TAG "%s %d : " fmt, __func__, __LINE__, ##args)
#define ST480_LOG(fmt, args...) pr_err(ST480_TAG fmt, ##args)

/* Raw LSB -> reported units: value * ST480_CONVERT_MUL / ST480_CONVERT_DIV uT.
 * 0.15 uT/LSB is the ST480-series datasheet-typical sensitivity for the
 * 1.6x1.6 LGA package the stock header selects; the stock driver reported raw
 * counts and let the Flyme HAL scale, so this factor is calibratable later. */
#define ST480_CONVERT_MUL 15
#define ST480_CONVERT_DIV 100

struct st480_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert cvt;
};

/* board 0 = "96085_1_1*" per stock; board 1 axis map kept for reference */
static int board_version_flag;

static struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

static struct i2c_client *st480_i2c_client;
static struct st480_i2c_data *st480_obj;

static int st480_init_flag = -1; /* 0 OK, -1 fail */

static int st480_local_init(void);
static int st480_local_uninit(void);

static struct mag_init_info st480_init_info = {
	.name = ST480_DEV_NAME,
	.init = st480_local_init,
	.uninit = st480_local_uninit,
};

static const struct i2c_device_id st480_i2c_id[] = { {ST480_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id st480_of_match[] = {
	/* shared msensor@0c seed, same compatible as akm09911-new */
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

/*
 * i2c transfer: write `len` command bytes then read `length` bytes back,
 * both phases reusing `buf` (stock ST480 framing).
 */
static int st480_i2c_transfer_data(struct i2c_client *client, int len, char *buf, int length)
{
	struct i2c_msg msgs[2];

	/* Always zero a hand-built i2c_msg: the MTK ext_flag/timing fields
	 * must not be stack garbage. */
	memset(msgs, 0, sizeof(msgs));
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = len;
	msgs[0].buf = buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = length;
	msgs[1].buf = buf;

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		ST480_ERR("i2c transfer error\n");
		return -EIO;
	}
	return 0;
}

/*
 * Presence check + register init + first single-measurement kick.
 * Stock st480_setup with its retry loops; the chip-id read is a mandatory
 * presence check (one transaction, -ENODEV on NAK) so auto-detect cannot
 * bind on AKM09911 units. The ID value is logged but not enforced (stock
 * had IC_CHECK 0).
 */
static int st480_setup(struct i2c_client *client)
{
	int ret;
	unsigned char buf[5];

	memset(buf, 0, 5);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	if (st480_i2c_transfer_data(client, 2, buf, 3) != 0) {
		ST480_ERR("presence check NAK at 0x%02x: chip absent (other dual-source variant)\n",
			  client->addr);
		return -ENODEV;
	}
	ST480_LOG("presence OK, register-0 readback 0x%02x (datasheet device id 0x%02x)\n",
		  buf[2], ST480_DEVICE_ID);

	/* init register step 1 */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	/* init register step 2 */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	/* set calibration register */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	/* kick first single measurement */
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(client, 1, buf, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 1, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	return 0;
}

/*
 * Read one measurement (stock st480_work_func framing: 1-byte command,
 * 7-byte readback, board-0 axis assembly), then kick the next single
 * measurement. Returns raw signed counts.
 */
static int st480_read_mag(int *x, int *y, int *z)
{
	char buffer[7];
	int ret;
	s16 rx, ry, rz;

	if (!st480_i2c_client)
		return -ENODEV;

	memset(buffer, 0, 7);
	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480_i2c_client, 1, buffer, 7) != 0) {
		ret++;
		if (st480_i2c_transfer_data(st480_i2c_client, 1, buffer, 7) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	if (!((buffer[0] >> 4) & 0x01)) {
		if (board_version_flag == 0) {
			rx = (s16)(-1 * ((buffer[3] << 8) | buffer[4]));
			ry = (s16)((buffer[1] << 8) | buffer[2]);
			rz = (s16)((buffer[5] << 8) | buffer[6]);
		} else {
			rx = (s16)(-1 * ((buffer[1] << 8) | buffer[2]));
			ry = (s16)(-1 * ((buffer[3] << 8) | buffer[4]));
			rz = (s16)((buffer[5] << 8) | buffer[6]);
		}
		*x = rx;
		*y = ry;
		*z = rz;
	}
	/* on "not ready" keep previous values silently, matching stock */

	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480_i2c_client, 1, buffer, 1) != 0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480_i2c_client, 1, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			break;
	}
	return 0;
}

/*------------------ mag framework control path ------------------*/
static int st480_m_open_report_data(int en)
{
	return 0;
}

static int st480_m_set_delay(u64 delay)
{
	return 0;
}

static int st480_m_enable(int en)
{
	char buffer[1];

	if (en && st480_i2c_client) {
		/* fresh conversion so the first poll has data */
		buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
		st480_i2c_transfer_data(st480_i2c_client, 1, buffer, 1);
	}
	return 0;
}

/* No orientation daemon for st480: orientation is fused in the HAL from raw
 * mag; these report nothing (status unreliable) rather than fake headings. */
static int st480_o_open_report_data(int en)
{
	return 0;
}

static int st480_o_set_delay(u64 delay)
{
	return 0;
}

static int st480_o_enable(int en)
{
	return 0;
}

static int st480_m_get_data(int *x, int *y, int *z, int *status)
{
	int raw[3] = { 0, 0, 0 };
	int remap[3];
	int res;
	struct st480_i2c_data *data = st480_obj;

	if (!data)
		return -ENODEV;

	res = st480_read_mag(&raw[0], &raw[1], &raw[2]);
	if (res < 0)
		return res;

	/* DTS-direction remap (direction 0 = identity = stock board-0 axes) */
	remap[data->cvt.map[0]] = data->cvt.sign[0] * raw[0];
	remap[data->cvt.map[1]] = data->cvt.sign[1] * raw[1];
	remap[data->cvt.map[2]] = data->cvt.sign[2] * raw[2];

	*x = remap[0] * ST480_CONVERT_MUL;
	*y = remap[1] * ST480_CONVERT_MUL;
	*z = remap[2] * ST480_CONVERT_MUL;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	if (atomic_read(&data->trace))
		ST480_LOG("mag raw %d %d %d\n", raw[0], raw[1], raw[2]);
	return 0;
}

static int st480_o_get_data(int *x, int *y, int *z, int *status)
{
	*x = 0;
	*y = 0;
	*z = 0;
	*status = SENSOR_STATUS_UNRELIABLE;
	return 0;
}

/*------------------ i2c probe ------------------*/
static int st480_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct st480_i2c_data *data;
	struct mag_control_path ctl = { 0 };
	struct mag_data_path mag_data = { 0 };
	int err = 0;

	ST480_LOG("st480_i2c_probe start\n");

	/* shared msensor@0c seed: rewrite to ST480's real address FIRST */
	client->addr = ST480_I2C_ADDRESS;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->hw = hw;
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	if (hwmsen_get_convert(data->hw->direction, &data->cvt)) {
		ST480_ERR("invalid direction %d, using identity\n", data->hw->direction);
		hwmsen_get_convert(0, &data->cvt);
	}

	data->client = client;
	i2c_set_clientdata(client, data);
	st480_i2c_client = client;
	st480_obj = data;

	err = st480_setup(client);
	if (err) {
		ST480_ERR("st480 setup error %d\n", err);
		goto exit_init_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.m_enable = st480_m_enable;
	ctl.m_set_delay = st480_m_set_delay;
	ctl.m_open_report_data = st480_m_open_report_data;
	ctl.o_enable = st480_o_enable;
	ctl.o_set_delay = st480_o_set_delay;
	ctl.o_open_report_data = st480_o_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;

	err = mag_register_control_path(&ctl);
	if (err) {
		ST480_ERR("register mag control path err\n");
		goto exit_init_failed;
	}

	mag_data.div_m = ST480_CONVERT_DIV;
	mag_data.div_o = ST480_CONVERT_DIV;
	mag_data.get_data_m = st480_m_get_data;
	mag_data.get_data_o = st480_o_get_data;

	err = mag_register_data_path(&mag_data);
	if (err) {
		ST480_ERR("register mag data path err\n");
		goto exit_init_failed;
	}

	st480_init_flag = 0;
	ST480_LOG("st480 probe done.\n"); /* stock success marker string */
	return 0;

exit_init_failed:
	st480_i2c_client = NULL;
	st480_obj = NULL;
	kfree(data);
exit:
	ST480_ERR("%s: err = %d\n", __func__, err);
	st480_init_flag = -1;
	return err;
}

static int st480_i2c_remove(struct i2c_client *client)
{
	st480_i2c_client = NULL;
	st480_obj = NULL;
	kfree(i2c_get_clientdata(client));
	return 0;
}

static struct i2c_driver st480_i2c_driver = {
	.probe = st480_i2c_probe,
	.remove = st480_i2c_remove,
	.id_table = st480_i2c_id,
	.driver = {
		.name = ST480_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = st480_of_match,
#endif
	},
};

/*------------------ mag framework glue ------------------*/
static int st480_local_init(void)
{
	if (i2c_add_driver(&st480_i2c_driver)) {
		ST480_ERR("i2c_add_driver error\n");
		return -1;
	}
	if (st480_init_flag == -1)
		return -1;
	return 0;
}

static int st480_local_uninit(void)
{
	i2c_del_driver(&st480_i2c_driver);
	return 0;
}

static int __init st480_init(void)
{
	const char *name = "mediatek,st480";

	/* ST480 is an M681 magnetometer */
	if (!m3note_board_is(M3NOTE_BOARD_M681))
		return -ENODEV;
	hw = get_mag_dts_func(name, hw);
	if (!hw) {
		/* WALL-3 class guard: NEVER mag_driver_add() with NULL hw */
		ST480_ERR("st480_init: get_mag_dts_func(%s) fail, not registering\n", name);
		return -ENODEV;
	}
	mag_driver_add(&st480_init_info);
	return 0;
}

static void __exit st480_exit(void)
{
}

module_init(st480_init);
module_exit(st480_exit);

MODULE_AUTHOR("Tori Xu <xuezhi_xu@senodia.com>");
MODULE_DESCRIPTION("senodia st480 linux driver (m681 4.4 mag-framework port)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0-m681");
