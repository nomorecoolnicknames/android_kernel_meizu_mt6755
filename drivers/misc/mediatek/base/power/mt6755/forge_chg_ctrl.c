/* M681 charger control: the stock temperature cut-off.
 *
 * Stock (wt6755_66_sz_l cust_charging.h and battery_common_fg_20.c
 * do_batt_temp_state_machine): no charging at 46 C and above until the
 * battery is below 46 C again, and none below 0 C until it is at 0 C
 * again; an error temperature stops it too. Stock stops it with the CHGEN
 * pin, GPIO115; here only the SM5414 I2C bit CTRL.CHGEN (0x07 bit 1) is
 * written, read-modify-write, and that is the only register this file
 * writes. Charging needs the pin low and the bit set. GPIO115 and GPIO4
 * are never touched.
 *
 * The temperature comes from forge_chg_observe.c, one sample every 30 s.
 * Without a valid one for notemp_stop_s (90 s) with the cable in, charging stops
 * as for an error temperature; a single missed sample
 * keeps the last decision. The bit is set again only if this file cleared it.
 *
 * All SM5414 I2C access goes through forge_sm5414_lock here; INT1-3
 * (0x00-0x02) clear on read and are read only by the full latch in
 * forge_chg_observe.c.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>

#define FORGE_SM5414_BUS	1
#define FORGE_SM5414_ADDR	0x49
#define FORGE_SM5414_CTRL	0x07
#define FORGE_CTRL_RESET	0x08
#define FORGE_CTRL_CHGEN	0x02
#define FORGE_CTRL_ENBOOST	0x01
/* read-only snapshot for /proc */
#define FORGE_SM5414_SNAP_FIRST	0x03
#define FORGE_SM5414_SNAP_LAST	0x0E
#define FORGE_SM5414_SNAP_LEN	(FORGE_SM5414_SNAP_LAST - FORGE_SM5414_SNAP_FIRST + 1)

/* stock thresholds, 0.1 C; MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE and
 * MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE equal the limits: no hysteresis */
#define FORGE_HOT_DC		460
#define FORGE_HOT_RESUME_DC	460	/* resume below this */
#define FORGE_COLD_DC		0
#define FORGE_COLD_RESUME_DC	0	/* resume at this and above */

enum { FORGE_CTRL_NORMAL, FORGE_CTRL_HOT, FORGE_CTRL_COLD, FORGE_CTRL_NOTEMP };
static const char * const forge_ctrl_names[] = { "normal", "hot", "cold", "no temperature" };

/* on by default: a safety cut-off */
static bool forge_temp_guard = true;
module_param_named(temp_guard, forge_temp_guard, bool, 0644);
MODULE_PARM_DESC(temp_guard, "stop charging at >=46 C and <0 C as stock (1)");

/* three missed samples (30 s each) */
static unsigned int forge_notemp_stop_s = 90;
module_param_named(notemp_stop_s, forge_notemp_stop_s, uint, 0644);
MODULE_PARM_DESC(notemp_stop_s, "stop charging after this long with the cable and no valid temperature, 0 = never (90)");

/* bench check of the write path at room temperature: 1 stops charging as a
 * hot battery would; it can only stop, never allow */
static bool forge_test_stop;
module_param_named(test_stop, forge_test_stop, bool, 0644);
MODULE_PARM_DESC(test_stop, "stop charging as if hot, for a bench check (0)");

/*
 * The stock SM5414 setup (wt6755_66_sz_l charging_hw_sm5414.c
 * charging_hw_init, set_current / set_input_current, cust_charging.h;
 * switch_charging.c and battery_common_fg_20.c for the 0-9 C zone), off by
 * default. stock_init=1, cable in, every tick: only the fields of
 * forge_sm5414_fields, each read-modify-write and read back, only when it
 * differs:
 *   0x09 AICLTH 4.4 V, AUTOSTOP 1;  0x0C TOPOFF 150 mA;  0x0D TOPOFFTIMER 10 min;
 *   0x0B BATREG cv_mv (0 = not written, default; 4100-4400 mV, stock 4400);
 *   0x08 VBUSLIMIT / 0x0A FASTCHG by source: USB 500 / 500 mA; wall charger
 *        1850 / 2500 mA (stock AC 1875 by the table, 2500), at 0-9 C or with
 *        no temperature 900 / 1000 mA.
 * The source: src 0 = USB (default), 1 = wall charger, 2 = auto: wall unless
 * forge_usb_host_seen() says a USB host reset the bus since VBUS came; its
 * weak version here answers "unknown", which auto takes as USB. Stock
 * knew the type from the PMIC BC1.1 walk, which is not available here.
 * CTRL is still written only by the cut-off (CHGEN); 0x00-0x02 are never
 * read (read-to-clear latches), 0x03-0x05 never written.
 */
static bool forge_stock_init;
module_param_named(stock_init, forge_stock_init, bool, 0644);
MODULE_PARM_DESC(stock_init, "write the stock SM5414 setup each tick, cable in (0)");

static unsigned int forge_src;
module_param_named(src, forge_src, uint, 0644);
MODULE_PARM_DESC(src, "with stock_init, the source: 0 USB 500 mA (default), 1 wall charger as stock, 2 auto from the USB bus");

static unsigned int forge_cv_mv;
module_param_named(cv_mv, forge_cv_mv, uint, 0644);
MODULE_PARM_DESC(cv_mv, "with stock_init, BATREG in mV, 4100-4400 in 25 mV steps (0 = not written, default)");

/* 1 = a USB host reset the bus since VBUS came, 0 = not, -1 = unknown */
int __weak forge_usb_host_seen(void)
{
	return -1;
}

static DEFINE_MUTEX(forge_sm5414_lock);

/* the control state is the observe work's: the hooks run only from it */
static struct {
	int state, temp_dc, chrdet;
	bool temp_ok, temp_seen;
	bool held;		/* this file cleared CHGEN and read it back clear */
	int rd_ret, wr_ret;	/* last I2C results */
	u8 ctrl;		/* CTRL as last read */
	u8 regs[FORGE_SM5414_SNAP_LAST + 1];	/* snapshot, cable in */
	int regs_ret;
	unsigned int clears, sets;
	unsigned int field_writes;
	int field_ret, src;	/* src as last applied: 0 USB, 1 wall */
	ktime_t temp_at;	/* CLOCK_BOOTTIME of the last valid temperature,
				 * or of the first tick before one */
} forge_ctrl = { .rd_ret = 1, .regs_ret = 1, .chrdet = -1, .src = -1 };	/* 1: not read yet */

static int forge_sm5414_xfer(u8 reg, u8 *val, bool write)
{
	struct i2c_adapter *adap = i2c_get_adapter(FORGE_SM5414_BUS);
	u8 buf[2] = { reg, write ? *val : 0 };
	struct i2c_msg msgs[2];
	int n, ret;

	if (!adap)
		return -ENODEV;
	memset(msgs, 0, sizeof(msgs));
	msgs[0].addr = FORGE_SM5414_ADDR;
	msgs[0].buf = buf;
	if (write) {
		msgs[0].len = 2;
		n = 1;
	} else {
		msgs[0].len = 1;
		msgs[1].addr = FORGE_SM5414_ADDR;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = 1;
		msgs[1].buf = val;
		n = 2;
	}
	ret = i2c_transfer(adap, msgs, n);
	i2c_put_adapter(adap);
	return ret == n ? 0 : (ret < 0 ? ret : -EIO);
}

/* one SM5414 register read for the rest of the line (forge_chg_observe.c) */
int forge_chg_sm5414_read(u8 reg, u8 *val)
{
	int ret;

	mutex_lock(&forge_sm5414_lock);
	ret = forge_sm5414_xfer(reg, val, false);
	mutex_unlock(&forge_sm5414_lock);
	return ret;
}
EXPORT_SYMBOL(forge_chg_sm5414_read);

/*
 * The one write: CTRL with only CHGEN changed against @cur, the value just
 * read; RESET is never written as 1. Then a read back. Returns 0 if the
 * chip now has CHGEN as asked. Caller holds forge_sm5414_lock.
 */
static int forge_ctrl_write_chgen(u8 cur, bool en)
{
	u8 val = ((cur & ~FORGE_CTRL_CHGEN) | (en ? FORGE_CTRL_CHGEN : 0)) &
		 ~FORGE_CTRL_RESET;
	u8 back = 0;
	int ret;

	if ((val ^ cur) & ~(FORGE_CTRL_CHGEN | FORGE_CTRL_RESET))
		return -EPERM;
	ret = forge_sm5414_xfer(FORGE_SM5414_CTRL, &val, true);
	if (!ret)
		ret = forge_sm5414_xfer(FORGE_SM5414_CTRL, &back, false);
	if (!ret) {
		forge_ctrl.ctrl = back;
		if (!!(back & FORGE_CTRL_CHGEN) != en)
			ret = -EIO;
	}
	forge_ctrl.wr_ret = ret;
	pr_notice("[FORGE_M681] chgctrl: CHGEN %d (CTRL 0x%02x -> 0x%02x, read back 0x%02x): %d\n",
		  en, cur, val, back, ret);
	return ret;
}

/* what the chip charges with: no other code on this line writes it.
 * Caller holds forge_sm5414_lock. */
static void forge_ctrl_snapshot(void)
{
	u8 old[sizeof(forge_ctrl.regs)], reg;
	int ret = 0;

	memcpy(old, forge_ctrl.regs, sizeof(old));
	for (reg = FORGE_SM5414_SNAP_FIRST; !ret && reg <= FORGE_SM5414_SNAP_LAST;
	     reg++)
		ret = forge_sm5414_xfer(reg, &forge_ctrl.regs[reg], false);
	/* log the first read, failures and changes (a replug resets the chip) */
	if (ret != forge_ctrl.regs_ret ||
	    (!ret && memcmp(old, forge_ctrl.regs, sizeof(old))))
		pr_notice("[FORGE_M681] chgctrl: SM5414 0x03-0x0E read %d: %*ph\n",
			  ret, FORGE_SM5414_SNAP_LEN,
			  &forge_ctrl.regs[FORGE_SM5414_SNAP_FIRST]);
	forge_ctrl.regs_ret = ret;
}

struct forge_sm5414_field {
	u8 reg, mask;
	const char *name;
};

enum { F_VBUSLIMIT, F_AICLTH, F_AUTOSTOP, F_FASTCHG, F_BATREG, F_TOPOFF,
       F_TOPOFFTIMER, F_NUM };

static const struct forge_sm5414_field forge_sm5414_fields[F_NUM] = {
	[F_VBUSLIMIT]	= { 0x08, 0x3F, "VBUSLIMIT" },
	[F_AICLTH]	= { 0x09, 0x70, "AICLTH" },
	[F_AUTOSTOP]	= { 0x09, 0x08, "AUTOSTOP" },
	[F_FASTCHG]	= { 0x0A, 0x3F, "FASTCHG" },
	[F_BATREG]	= { 0x0B, 0xF0, "BATREG" },
	[F_TOPOFF]	= { 0x0C, 0x78, "TOPOFF" },
	[F_TOPOFFTIMER]	= { 0x0D, 0x03, "TOPOFFTIMER" },
};

/* Caller holds forge_sm5414_lock. */
static int forge_sm5414_set_field(int f, u8 val)
{
	const struct forge_sm5414_field *d = &forge_sm5414_fields[f];
	u8 cur = 0, nv, back = 0;
	int ret;

	if (f < 0 || f >= F_NUM || (val & ~d->mask))
		return -EPERM;
	ret = forge_sm5414_xfer(d->reg, &cur, false);
	if (ret || (cur & d->mask) == val)
		return ret;
	nv = (cur & ~d->mask) | val;
	ret = forge_sm5414_xfer(d->reg, &nv, true);
	if (!ret)
		ret = forge_sm5414_xfer(d->reg, &back, false);
	if (!ret && (back & d->mask) != val)
		ret = -EIO;
	pr_notice("[FORGE_M681] chgctrl: %s 0x%02x 0x%02x -> 0x%02x (read back 0x%02x): %d\n",
		  d->name, d->reg, cur, nv, back, ret);
	forge_ctrl.field_writes++;
	return ret;
}

static void forge_acc(int *ret, int r)
{
	if (!*ret)
		*ret = r;
}

/* 100 + 50 * code mA for both VBUSLIMIT and FASTCHG */
#define FORGE_MA(ma)	(((ma) - 100) / 50)

/* Caller holds forge_sm5414_lock. */
static void forge_sm5414_stock(void)
{
	bool cool = !forge_ctrl.temp_ok ||
		    (forge_ctrl.temp_dc >= 0 && forge_ctrl.temp_dc < 100);
	int wall = forge_src == 1 ||
		   (forge_src == 2 && forge_usb_host_seen() == 0);
	int ret = 0;

	forge_acc(&ret, forge_sm5414_set_field(F_AICLTH, 1 << 4));	/* 4.4 V */
	forge_acc(&ret, forge_sm5414_set_field(F_AUTOSTOP, 0x08));
	forge_acc(&ret, forge_sm5414_set_field(F_TOPOFF, 1 << 3));	/* 150 mA */
	forge_acc(&ret, forge_sm5414_set_field(F_TOPOFFTIMER, 0));	/* 10 min */
	if (forge_cv_mv >= 4100 && forge_cv_mv <= 4400)
		forge_acc(&ret, forge_sm5414_set_field(F_BATREG,
				((forge_cv_mv - 4100) / 25) << 4));
	if (!wall) {
		forge_acc(&ret, forge_sm5414_set_field(F_VBUSLIMIT, FORGE_MA(500)));
		forge_acc(&ret, forge_sm5414_set_field(F_FASTCHG, FORGE_MA(500)));
	} else if (cool) {
		forge_acc(&ret, forge_sm5414_set_field(F_VBUSLIMIT, FORGE_MA(900)));
		forge_acc(&ret, forge_sm5414_set_field(F_FASTCHG, FORGE_MA(1000)));
	} else {
		forge_acc(&ret, forge_sm5414_set_field(F_VBUSLIMIT, FORGE_MA(1850)));
		forge_acc(&ret, forge_sm5414_set_field(F_FASTCHG, FORGE_MA(2500)));
	}
	if (wall != forge_ctrl.src)
		pr_notice("[FORGE_M681] chgctrl: source %s (src %u, USB host seen %d)\n",
			  wall ? "wall charger" : "USB", forge_src,
			  forge_usb_host_seen());
	forge_ctrl.src = wall;
	forge_ctrl.field_ret = ret;
}

static void forge_ctrl_apply(bool stop)
{
	u8 ctrl = 0;
	int ret;

	mutex_lock(&forge_sm5414_lock);
	ret = forge_sm5414_xfer(FORGE_SM5414_CTRL, &ctrl, false);
	if (ret != forge_ctrl.rd_ret)
		pr_notice("[FORGE_M681] chgctrl: SM5414 CTRL read %d (0x%02x)\n",
			  ret, ctrl);
	forge_ctrl.rd_ret = ret;
	if (ret)
		goto out;
	forge_ctrl.ctrl = ctrl;
	forge_ctrl_snapshot();
	/* boost (OTG) mode does not charge; leave it alone */
	if (ctrl & FORGE_CTRL_ENBOOST)
		goto out;
	if (forge_stock_init)
		forge_sm5414_stock();
	if (stop && (ctrl & FORGE_CTRL_CHGEN)) {
		/* also after a cable replug reset the chip to its defaults */
		if (!forge_ctrl_write_chgen(ctrl, false)) {
			forge_ctrl.held = true;
			forge_ctrl.clears++;
		}
	} else if (!stop && !(ctrl & FORGE_CTRL_CHGEN) && forge_ctrl.held) {
		if (!forge_ctrl_write_chgen(ctrl, true)) {
			forge_ctrl.held = false;
			forge_ctrl.sets++;
		}
	} else if (ctrl & FORGE_CTRL_CHGEN) {
		/* set by someone else (or a chip reset): not held any more */
		forge_ctrl.held = false;
	}
out:
	mutex_unlock(&forge_sm5414_lock);
}

/* stock do_batt_temp_state_machine(), cold checked first */
static int forge_ctrl_next(int state, int dc)
{
	if (dc < FORGE_COLD_DC)
		return FORGE_CTRL_COLD;
	if (state == FORGE_CTRL_COLD && dc < FORGE_COLD_RESUME_DC)
		return FORGE_CTRL_COLD;
	if (dc >= FORGE_HOT_DC)
		return FORGE_CTRL_HOT;
	if (state == FORGE_CTRL_HOT && dc >= FORGE_HOT_RESUME_DC)
		return FORGE_CTRL_HOT;
	return FORGE_CTRL_NORMAL;
}

static void forge_dc_str(char *buf, size_t len, int dc)
{
	int a = dc < 0 ? -dc : dc;

	snprintf(buf, len, "%s%d.%d C", dc < 0 ? "-" : "", a / 10, a % 10);
}

/* strong versions of the weak hooks in forge_chg_observe.c */
void forge_chg_ctrl_tick(int chrdet, int temp_dc, bool temp_ok, int vbat_mv)
{
	ktime_t now = ktime_get_boottime();
	int state = forge_ctrl.state;
	char t[16];
	bool stop;

	if (!forge_ctrl.temp_seen && !ktime_to_ns(forge_ctrl.temp_at))
		forge_ctrl.temp_at = now;	/* the no-temperature clock starts */
	forge_ctrl.chrdet = chrdet;
	forge_ctrl.temp_ok = temp_ok;
	if (temp_ok) {
		forge_ctrl.temp_dc = temp_dc;
		forge_ctrl.temp_seen = true;
		forge_ctrl.temp_at = now;
		state = forge_ctrl_next(state, temp_dc);
	} else if (state == FORGE_CTRL_NORMAL && chrdet == 1 &&
		   forge_notemp_stop_s &&
		   ktime_ms_delta(now, forge_ctrl.temp_at) >=
		   (s64)forge_notemp_stop_s * 1000) {
		state = FORGE_CTRL_NOTEMP;
	}
	if (state != forge_ctrl.state) {
		forge_dc_str(t, sizeof(t), temp_dc);
		pr_notice("[FORGE_M681] chgctrl: battery %s (%s): %s -> %s (charger %d, %d mV, last valid temperature %lld s ago)\n",
			  t, temp_ok ? "valid" : "no sample",
			  forge_ctrl_names[forge_ctrl.state],
			  forge_ctrl_names[state], chrdet, vbat_mv,
			  ktime_ms_delta(now, forge_ctrl.temp_at) / 1000);
	}
	forge_ctrl.state = state;
	stop = forge_test_stop ||
	       (forge_temp_guard && state != FORGE_CTRL_NORMAL);
	/* without VBUS the chip may be unpowered: nothing to write */
	if (chrdet == 1)
		forge_ctrl_apply(stop);
}

bool forge_chg_ctrl_inhibited(void)
{
	return forge_ctrl.held;
}

static int forge_ctrl_show(struct seq_file *m, void *v)
{
	const u8 *r = forge_ctrl.regs;
	char t[16], hot[16], cold[16];

	forge_dc_str(t, sizeof(t), forge_ctrl.temp_dc);
	forge_dc_str(hot, sizeof(hot), FORGE_HOT_DC);
	forge_dc_str(cold, sizeof(cold), FORGE_COLD_DC);
	seq_printf(m, "chgctrl: temp_guard %d notemp_stop_s %u test_stop %d, state %s (stop at >= %s and < %s), charger %d\n",
		   forge_temp_guard, forge_notemp_stop_s, forge_test_stop,
		   forge_ctrl_names[forge_ctrl.state], hot, cold,
		   forge_ctrl.chrdet);
	seq_printf(m, "temperature %s, %s this tick, %s %lld s ago\n",
		   forge_ctrl.temp_seen ? t : "never valid",
		   forge_ctrl.temp_ok ? "valid" : "no valid sample",
		   forge_ctrl.temp_seen ? "last valid" : "first tick",
		   ktime_to_ns(forge_ctrl.temp_at) ?
		   ktime_ms_delta(ktime_get_boottime(), forge_ctrl.temp_at) / 1000 :
		   -1LL);
	seq_printf(m, "SM5414 CTRL 0x%02x (CHGEN %d ENBOOST %d) read %d, last write %d; CHGEN held off %d, cleared %u set %u\n",
		   forge_ctrl.ctrl, !!(forge_ctrl.ctrl & FORGE_CTRL_CHGEN),
		   !!(forge_ctrl.ctrl & FORGE_CTRL_ENBOOST), forge_ctrl.rd_ret,
		   forge_ctrl.wr_ret, forge_ctrl.held, forge_ctrl.clears,
		   forge_ctrl.sets);
	seq_printf(m, "stock_init %d src %u (applied %s, USB host seen %d) cv_mv %u: field writes %u, last %d\n",
		   forge_stock_init, forge_src,
		   forge_ctrl.src < 0 ? "never" : forge_ctrl.src ? "wall" : "USB",
		   forge_usb_host_seen(), forge_cv_mv, forge_ctrl.field_writes,
		   forge_ctrl.field_ret);
	if (forge_ctrl.regs_ret) {
		seq_printf(m, "SM5414 0x03-0x0E: not read (%d)\n",
			   forge_ctrl.regs_ret);
		return 0;
	}
	/* fields and codes as stock sm5414.h */
	seq_printf(m, "SM5414 0x03-0x0E: %*ph\n", FORGE_SM5414_SNAP_LEN,
		   &r[FORGE_SM5414_SNAP_FIRST]);
	seq_printf(m, "SM5414 (read only): VBUSLIMIT %d mA, FASTCHG %d mA, BATREG %d mV, WEAKBAT %d mV, TOPOFF %d mA, AICLEN %d (%d mV), AUTOSTOP %d, PRECHG %d mA, FASTTIMER %d (0-2 = 3.5/4.5/5.5 h, 3 off), TOPOFFTIMER %d (10/20/30/45 min)\n",
		   100 + 50 * (r[0x08] & 0x3F), 100 + 50 * (r[0x0A] & 0x3F),
		   4100 + 25 * (r[0x0B] >> 4), 3000 + 50 * (r[0x0B] & 0x0F),
		   100 + 50 * ((r[0x0C] >> 3) & 0x0F), (r[0x09] >> 2) & 1,
		   4300 + 100 * ((r[0x09] >> 4) & 0x07), (r[0x09] >> 3) & 1,
		   150 + 100 * (r[0x09] & 0x03), (r[0x0D] >> 2) & 3,
		   r[0x0D] & 3);
	return 0;
}

static int forge_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, forge_ctrl_show, NULL);
}

static const struct file_operations forge_ctrl_fops = {
	.open = forge_ctrl_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init forge_chg_ctrl_init(void)
{
	if (!proc_create("m681_chgctrl", 0444, NULL, &forge_ctrl_fops))
		pr_notice("[FORGE_M681] chgctrl: /proc/m681_chgctrl not created\n");
	pr_notice("[FORGE_M681] chgctrl: temperature cut-off %s, no-temperature stop %u s (SM5414 CTRL.CHGEN over I2C only)\n",
		  forge_temp_guard ? "on" : "off", forge_notemp_stop_s);
	return 0;
}
/* before forge_chg_observe's late_initcall runs the first tick */
device_initcall(forge_chg_ctrl_init);
