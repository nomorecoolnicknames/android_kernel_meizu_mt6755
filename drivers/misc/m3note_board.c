/*
 * Meizu M3 Note board identity (m681 / l681).
 *
 * The board is read from the machine compatible of the appended DTB, which
 * unflatten_device_tree() has parsed before the first initcall. Both or
 * neither of "meizu,m681" / "meizu,l681" means UNKNOWN. The result is
 * cached on first use. No initcalls: the board is printed on the first
 * m3note_board_id(), and the l681 battery observer calls
 * m3note_hw_report() from its own init.
 *
 * /sys/module/m3note_board/parameters/board reports the selection.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/m3note_board.h>

static int m3note_board_cached = -1;

static const char *const m3note_board_names[] = {
	[M3NOTE_BOARD_UNKNOWN] = "unknown",
	[M3NOTE_BOARD_M681] = "m681",
	[M3NOTE_BOARD_L681] = "l681",
};

enum m3note_board_id m3note_board_id(void)
{
	int id = READ_ONCE(m3note_board_cached);

	if (id < 0) {
		bool m681 = of_machine_is_compatible("meizu,m681");
		bool l681 = of_machine_is_compatible("meizu,l681");

		if (m681 == l681)
			id = M3NOTE_BOARD_UNKNOWN;
		else
			id = m681 ? M3NOTE_BOARD_M681 : M3NOTE_BOARD_L681;
		/* a race prints twice, both the same */
		if (cmpxchg(&m3note_board_cached, -1, id) == -1)
			pr_notice("m3note: board %s (DT machine compatible)\n",
				  m3note_board_names[id]);
	}
	return id;
}
EXPORT_SYMBOL(m3note_board_id);

static int m3note_board_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%s\n",
			 m3note_board_names[m3note_board_id()]);
}

static const struct kernel_param_ops m3note_board_ops = {
	.get = m3note_board_get,
};
module_param_cb(board, &m3note_board_ops, NULL, 0444);

/*
 * Charger identity on i2c1, read only (one SMBus byte read each, straight
 * on the adapter, no client):
 * - l681: BQ24196 at 0x6b (REG0A = 0x2b); matched as REG0A & 0xf8 == 0x28
 *   (vendor/part bits, revision ignored).
 * - m681: SM5414 at 0x49; present when register 0x0E reads non-zero (the
 *   stock detect, sm5414_hw_component_detect()).
 */
#define M3NOTE_CHG_BUS		1
#define M3NOTE_BQ24196_ADDR	0x6b
#define M3NOTE_BQ24196_REG0A	0x0a
#define M3NOTE_SM5414_ADDR	0x49
#define M3NOTE_SM5414_REG0E	0x0e

enum m3note_hw { M3NOTE_HW_NOT_READ, M3NOTE_HW_M681, M3NOTE_HW_L681, M3NOTE_HW_UNCLEAR };
static const char *const m3note_hw_names[] = {
	[M3NOTE_HW_NOT_READ] = "not-read",
	[M3NOTE_HW_M681] = "m681",
	[M3NOTE_HW_L681] = "l681",
	[M3NOTE_HW_UNCLEAR] = "unclear",
};
static int m3note_hw = M3NOTE_HW_NOT_READ;
static int m3note_hw_bq_reg0a = -1, m3note_hw_sm_reg0e = -1;
static DEFINE_MUTEX(m3note_hw_lock);

static int m3note_read_byte(struct i2c_adapter *adap, u16 addr, u8 reg)
{
	union i2c_smbus_data data;
	int ret = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, reg,
				 I2C_SMBUS_BYTE_DATA, &data);

	return ret < 0 ? ret : data.byte;
}

/* Returns enum m3note_hw, or -EPROBE_DEFER without the i2c1 adapter. */
static int m3note_hw_identify(void)
{
	struct i2c_adapter *adap;
	bool bq, sm;
	int ret;

	mutex_lock(&m3note_hw_lock);
	ret = m3note_hw;
	if (ret != M3NOTE_HW_NOT_READ)
		goto out;
	adap = i2c_get_adapter(M3NOTE_CHG_BUS);
	if (!adap) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	m3note_hw_bq_reg0a = m3note_read_byte(adap, M3NOTE_BQ24196_ADDR,
					      M3NOTE_BQ24196_REG0A);
	m3note_hw_sm_reg0e = m3note_read_byte(adap, M3NOTE_SM5414_ADDR,
					      M3NOTE_SM5414_REG0E);
	i2c_put_adapter(adap);
	bq = m3note_hw_bq_reg0a >= 0 && (m3note_hw_bq_reg0a & 0xf8) == 0x28;
	sm = m3note_hw_sm_reg0e > 0;
	ret = bq && !sm ? M3NOTE_HW_L681 : sm && !bq ? M3NOTE_HW_M681 :
	      M3NOTE_HW_UNCLEAR;
	m3note_hw = ret;
	pr_notice("m3note: i2c%d charger: 0x%02x REG0A %d (0x%02x), 0x%02x REG0E %d (0x%02x) -> %s hardware\n",
		  M3NOTE_CHG_BUS, M3NOTE_BQ24196_ADDR, m3note_hw_bq_reg0a,
		  m3note_hw_bq_reg0a & 0xff, M3NOTE_SM5414_ADDR,
		  m3note_hw_sm_reg0e, m3note_hw_sm_reg0e & 0xff,
		  m3note_hw_names[ret]);
out:
	mutex_unlock(&m3note_hw_lock);
	return ret;
}

int m3note_m681_gpio_owner_ok(void)
{
	int hw;

	if (!m3note_board_is(M3NOTE_BOARD_M681))
		return 0;
	hw = m3note_hw_identify();
	if (hw < 0)
		return hw;
	if (hw == M3NOTE_HW_L681) {
		pr_err_ratelimited("m3note: m681 DTB but BQ24196 and no SM5414 on i2c%d - m681 GPIO owner refused\n",
				   M3NOTE_CHG_BUS);
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(m3note_m681_gpio_owner_ok);

static int m3note_hw_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%s bq24196_reg0a=%d sm5414_reg0e=%d\n",
			 m3note_hw_names[READ_ONCE(m3note_hw)],
			 m3note_hw_bq_reg0a, m3note_hw_sm_reg0e);
}

static const struct kernel_param_ops m3note_hw_ops = {
	.get = m3note_hw_get,
};
module_param_cb(hw, &m3note_hw_ops, NULL, 0444);

/*
 * Report whether the hardware matches the DTB. Called by the l681 battery
 * observer at its init (late_initcall), so an m681 boot reads nothing on
 * i2c1 here.
 */
void m3note_hw_report(void)
{
	int hw = m3note_hw_identify();
	enum m3note_board_id dt = m3note_board_id();

	if (hw < 0) {
		pr_notice("m3note: i2c%d not registered, charger not read\n",
			  M3NOTE_CHG_BUS);
		return;
	}
	pr_notice("m3note: DT board %s, charger says %s hardware: %s\n",
		  m3note_board_names[dt], m3note_hw_names[hw],
		  (dt == M3NOTE_BOARD_M681 && hw == M3NOTE_HW_M681) ||
		  (dt == M3NOTE_BOARD_L681 && hw == M3NOTE_HW_L681) ? "match" :
		  hw == M3NOTE_HW_UNCLEAR ? "unclear" : "MISMATCH");
}
