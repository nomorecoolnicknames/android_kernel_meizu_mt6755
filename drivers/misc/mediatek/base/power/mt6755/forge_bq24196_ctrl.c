/* L681 charger control: the stock temperature cut-off and the charge state
 * of the BQ24196 (i2c1 0x6b). Acts on the l681 board only (m3note_board_id()
 * from the DTB); on any other board every hook is a no-op.
 *
 * Stock behaviour (hq6755_66_b1a_l cust_charging.h, battery_common_fg_20.c
 * do_batt_temp_state_machine): an integer battery temperature above 45 C
 * stops charging until it is below 45 C again (strict ">", unlike m681's
 * ">= 46"), below 0 C until it is 0 C again; an error temperature stops it
 * too. In 0.1 C: stop at >= 46.0 C, resume below 45.0 C; stop below 0.0 C,
 * resume at 0.0 C.
 *
 * By default the only register written is REG01, and in it only CHG_CONFIG
 * (bits 5:4, 00 off / 01 charge) and WDT_RST (bit 6, self-clearing);
 * stock_init adds the fields of forge_bq_fields (see there). REG_RST (bit 7) is
 * never written as 1, OTG (CHG_CONFIG 1x) is left alone. While charging is
 * held off, every tick (30 s) also kicks the I2C watchdog (REG05: 40 s from
 * LK): an expired watchdog resets the chip to its defaults, which charge.
 * GPIO100 (BQ24196 CE on l681) is never touched.
 *
 * Without a valid temperature for notemp_stop_s (90 s) with the cable in,
 * charging stops as for an error temperature (fail-safe, as on m681); a
 * single missed sample keeps the last decision. Charging is enabled again
 * only if this file disabled it.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/m3note_board.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>

#define FORGE_BQ_BUS		1
#define FORGE_BQ_ADDR		0x6b
#define FORGE_BQ_REG01		0x01
#define FORGE_BQ_REG08		0x08
#define FORGE_BQ_REG_RST	0x80
#define FORGE_BQ_WDT_RST	0x40
#define FORGE_BQ_CHG_CONFIG	0x30	/* 00 off, 01 charge, 1x OTG */
#define FORGE_BQ_CHG_ON		0x10
#define FORGE_BQ_SNAP_LAST	0x0A	/* REG00-REG0A, read only */
/* stock full veto (forge_full_min_mv): a done below this is a top-off
 * from a starved input, not a full battery */
#define FORGE_BQ_FULL_MIN_MV	4250

#define FORGE_HOT_DC		460
#define FORGE_HOT_RESUME_DC	450	/* resume below this */
#define FORGE_COLD_DC		0
#define FORGE_COLD_RESUME_DC	0	/* resume at this and above */

enum { FORGE_CTRL_NORMAL, FORGE_CTRL_HOT, FORGE_CTRL_COLD, FORGE_CTRL_NOTEMP };
static const char * const forge_ctrl_names[] = { "normal", "hot", "cold", "no temperature" };

/* on by default: a safety cut-off */
static bool forge_temp_guard = true;
module_param_named(temp_guard, forge_temp_guard, bool, 0644);
MODULE_PARM_DESC(temp_guard, "stop charging above 45 C and below 0 C as stock (1)");

/* three missed samples (30 s each) */
static unsigned int forge_notemp_stop_s = 90;
module_param_named(notemp_stop_s, forge_notemp_stop_s, uint, 0644);
MODULE_PARM_DESC(notemp_stop_s, "stop charging after this long with the cable and no valid temperature, 0 = never (90)");

/* bench check of the write path: 1 stops charging as a hot battery would;
 * it can only stop, never allow */
static bool forge_test_stop;
module_param_named(test_stop, forge_test_stop, bool, 0644);
MODULE_PARM_DESC(test_stop, "stop charging as if hot, for a bench check (0)");

/* bench check of the cut-off without heating the battery: a battery
 * temperature in 0.1 C for a second, separate copy of the state machine. It
 * can only stop charging: the real temperature keeps its own state, and
 * charging stops if either says so. FORGE_TEST_TEMP_OFF (default) = off. */
#define FORGE_TEST_TEMP_OFF	-10000
static int forge_test_temp_dc = FORGE_TEST_TEMP_OFF;
module_param_named(test_temp_dc, forge_test_temp_dc, int, 0644);
MODULE_PARM_DESC(test_temp_dc, "bench: a stand-in battery temperature in 0.1 C that can only stop charging (-10000 = off)");

/*
 * The stock BQ24196 setup (3.10 charging_hw_bq24196.c charging_hw_init,
 * charging_set_current/input_current, cust_charging.h), off by default:
 * the kernel otherwise leaves the chip as LK set it, and nobody kicks its
 * watchdog. stock_init=1, cable in, every tick: only the fields of
 * forge_bq_fields, each read-modify-write and read back, and only when it
 * differs:
 *   REG00 VINDPM 4.44 V; IINLIM 500 mA for a USB host or an unknown source
 *         (REG08 VBUS_STAT, the chip's own D+/D- detection), an adapter's
 *         left as the chip set it (3000 mA with stock_ac);
 *   REG01 SYS_MIN 3.4 V (stock code 4; its comment says 3.5), and CHG_CONFIG 01 when charging is not held off
 *         (OTG never touched);
 *   REG02 ICHG 512 mA (adapter with stock_ac: 1728 mA, 832 mA at 0-9 C or
 *         without a temperature);
 *   REG03 IPRECHG 512 mA, ITERM 128 mA;
 *   REG04 VREG cv_mv (4336 by default; up to the stock 4400; 0 = not
 *         written), BATLOWV 3.0 V, VRECHG 100 mV;
 *   REG05 EN_TERM 1, TERM_STAT 0, WATCHDOG off, CHG_TIMER 8 h, EN_TIMER 1.
 * WATCHDOG differs from the stock 40 s on purpose: this file runs from a
 * freezable 30 s work that stops in suspend, and an expired watchdog
 * returns the chip to its defaults (ICHG 2048 mA, VREG 4.208 V, charging
 * on); with it off the chip keeps these, lower values. REG_RST, EN_HIZ,
 * BATFET and OTG are never written. The stock 1024 mA for an adapter while
 * the panel is lit is left out on purpose.
 */
static bool forge_stock_init;
module_param_named(stock_init, forge_stock_init, bool, 0644);
MODULE_PARM_DESC(stock_init, "write the stock BQ24196 setup each tick, cable in (0)");

static bool forge_stock_ac;
module_param_named(stock_ac, forge_stock_ac, bool, 0644);
MODULE_PARM_DESC(stock_ac, "with stock_init, an adapter gets the stock 1728 mA / input 3000 mA (0: 512 mA, input as detected)");

static unsigned int forge_cv_mv = 4336;
module_param_named(cv_mv, forge_cv_mv, uint, 0644);
MODULE_PARM_DESC(cv_mv, "with stock_init, VREG in mV, 4100-4400, rounded down to 16 mV (4336; 0 = not written)");

static bool forge_bq_board;	/* l681, set at init */
static DEFINE_MUTEX(forge_bq_lock);

/* the control state is the observe work's: the hooks run only from it */
static struct {
	int state, temp_dc, chrdet, vbat_mv;
	int test_state;		/* the test_temp_dc copy of the state machine */
	bool temp_ok, temp_seen;
	bool held;		/* this file set CHG_CONFIG 00 and read it back */
	int rd_ret, wr_ret, stat_ret;
	u8 reg01, reg08;
	u8 regs[FORGE_BQ_SNAP_LAST + 1];	/* snapshot, cable in */
	int regs_ret;
	unsigned int clears, sets, kicks;
	unsigned int field_writes;
	int field_ret;
	bool tfault;		/* REG09 CHRG_FAULT 11 seen, VBAT noted */
	int tfault_mv;		/* VBAT when the timer fault was first seen */
	ktime_t temp_at;
} forge_ctrl = { .rd_ret = 1, .regs_ret = 1, .stat_ret = 1, .chrdet = -1 };

static int forge_bq_xfer(u8 reg, u8 *val, bool write)
{
	struct i2c_adapter *adap = i2c_get_adapter(FORGE_BQ_BUS);
	union i2c_smbus_data data;
	int ret;

	if (!adap)
		return -ENODEV;
	data.byte = write ? *val : 0;
	ret = i2c_smbus_xfer(adap, FORGE_BQ_ADDR, 0,
			     write ? I2C_SMBUS_WRITE : I2C_SMBUS_READ, reg,
			     I2C_SMBUS_BYTE_DATA, &data);
	i2c_put_adapter(adap);
	if (!ret && !write)
		*val = data.byte;
	return ret < 0 ? ret : 0;
}

/*
 * The one write: REG01 with only CHG_CONFIG set to @chg (00 or 01) against
 * @cur, the value just read, and WDT_RST; REG_RST never 1. Then a read back.
 * Caller holds forge_bq_lock.
 */
static int forge_bq_write_reg01(u8 cur, u8 chg)
{
	u8 val = ((cur & ~(FORGE_BQ_CHG_CONFIG | FORGE_BQ_REG_RST)) | chg) |
		 FORGE_BQ_WDT_RST;
	u8 back = 0;
	int ret;

	if (chg & ~FORGE_BQ_CHG_ON ||
	    (val ^ cur) & ~(FORGE_BQ_CHG_CONFIG | FORGE_BQ_WDT_RST |
			    FORGE_BQ_REG_RST))
		return -EPERM;
	ret = forge_bq_xfer(FORGE_BQ_REG01, &val, true);
	if (!ret)
		ret = forge_bq_xfer(FORGE_BQ_REG01, &back, false);
	if (!ret) {
		forge_ctrl.reg01 = back;
		if ((back & FORGE_BQ_CHG_CONFIG) != chg)
			ret = -EIO;
	}
	forge_ctrl.wr_ret = ret;
	return ret;
}

/* Caller holds forge_bq_lock. */
static void forge_bq_snapshot(void)
{
	u8 old[sizeof(forge_ctrl.regs)], reg;
	int ret = 0;

	memcpy(old, forge_ctrl.regs, sizeof(old));
	for (reg = 0; !ret && reg <= FORGE_BQ_SNAP_LAST; reg++)
		ret = forge_bq_xfer(reg, &forge_ctrl.regs[reg], false);
	if (ret != forge_ctrl.regs_ret ||
	    (!ret && memcmp(old, forge_ctrl.regs, sizeof(old))))
		pr_notice("[FORGE_L681] chgctrl: BQ24196 REG00-0A read %d: %*ph\n",
			  ret, FORGE_BQ_SNAP_LAST + 1, forge_ctrl.regs);
	forge_ctrl.regs_ret = ret;
}

/* the only fields stock_init writes */
struct forge_bq_field {
	u8 reg, mask;
	const char *name;
};

enum { F_VINDPM, F_IINLIM, F_SYS_MIN, F_ICHG, F_IPRECHG, F_ITERM, F_VREG,
       F_BATLOWV, F_VRECHG, F_EN_TERM, F_TERM_STAT, F_WATCHDOG, F_EN_TIMER,
       F_CHG_TIMER,
       F_NUM };

static const struct forge_bq_field forge_bq_fields[F_NUM] = {
	[F_VINDPM]	= { 0x00, 0x78, "VINDPM" },
	[F_IINLIM]	= { 0x00, 0x07, "IINLIM" },
	[F_SYS_MIN]	= { 0x01, 0x0E, "SYS_MIN" },
	[F_ICHG]	= { 0x02, 0xFC, "ICHG" },
	[F_IPRECHG]	= { 0x03, 0xF0, "IPRECHG" },
	[F_ITERM]	= { 0x03, 0x0F, "ITERM" },
	[F_VREG]	= { 0x04, 0xFC, "VREG" },
	[F_BATLOWV]	= { 0x04, 0x02, "BATLOWV" },
	[F_VRECHG]	= { 0x04, 0x01, "VRECHG" },
	[F_EN_TERM]	= { 0x05, 0x80, "EN_TERM" },
	[F_TERM_STAT]	= { 0x05, 0x40, "TERM_STAT" },
	[F_WATCHDOG]	= { 0x05, 0x30, "WATCHDOG" },
	[F_EN_TIMER]	= { 0x05, 0x08, "EN_TIMER" },
	[F_CHG_TIMER]	= { 0x05, 0x06, "CHG_TIMER" },
};

/*
 * One field to @val (already in place under the mask): read, skip if equal,
 * write the register with only that field changed (REG01: REG_RST kept 0),
 * read back. Caller holds forge_bq_lock.
 */
static int forge_bq_set_field(int f, u8 val)
{
	const struct forge_bq_field *d = &forge_bq_fields[f];
	u8 cur = 0, nv, back = 0;
	int ret;

	if (f < 0 || f >= F_NUM || (val & ~d->mask))
		return -EPERM;
	ret = forge_bq_xfer(d->reg, &cur, false);
	if (ret || (cur & d->mask) == val)
		return ret;
	nv = (cur & ~d->mask) | val;
	if (d->reg == FORGE_BQ_REG01)
		nv &= ~FORGE_BQ_REG_RST;
	ret = forge_bq_xfer(d->reg, &nv, true);
	if (!ret)
		ret = forge_bq_xfer(d->reg, &back, false);
	if (!ret && (back & d->mask) != val)
		ret = -EIO;
	pr_notice("[FORGE_L681] chgctrl: %s REG%02X 0x%02x -> 0x%02x (read back 0x%02x): %d\n",
		  d->name, d->reg, cur, nv, back, ret);
	forge_ctrl.field_writes++;
	return ret;
}

/* keep the first error */
static void forge_acc(int *ret, int r)
{
	if (!*ret)
		*ret = r;
}

/* VBUS_STAT (REG08 bits 7:6): 0 unknown, 1 USB host, 2 adapter, 3 OTG */
static void forge_bq_stock(u8 r08)
{
	unsigned int vbus = r08 >> 6;
	bool adapter = vbus == 2 && forge_stock_ac;
	bool cool = !forge_ctrl.temp_ok ||
		    (forge_ctrl.temp_dc >= 0 && forge_ctrl.temp_dc < 100);
	u8 ichg = adapter ? (cool ? 5 : 19) : 0;	/* 512 + 64 * code mA */
	int ret = 0;

	if (vbus == 3)
		return;
	forge_acc(&ret, forge_bq_set_field(F_VINDPM, 0x7 << 3));	/* 3.88 + 0.08 * 7 V */
	if (vbus != 2)
		forge_acc(&ret, forge_bq_set_field(F_IINLIM, 2));	/* 500 mA */
	else if (forge_stock_ac)
		forge_acc(&ret, forge_bq_set_field(F_IINLIM, 7));	/* 3000 mA */
	forge_acc(&ret, forge_bq_set_field(F_SYS_MIN, 0x4 << 1));	/* 3.0 + 0.1 * 4 V */
	forge_acc(&ret, forge_bq_set_field(F_ICHG, ichg << 2));
	forge_acc(&ret, forge_bq_set_field(F_IPRECHG, 0x3 << 4));	/* 512 mA */
	forge_acc(&ret, forge_bq_set_field(F_ITERM, 0));		/* 128 mA */
	if (forge_cv_mv >= 4100 && forge_cv_mv <= 4400)
		forge_acc(&ret, forge_bq_set_field(F_VREG, ((forge_cv_mv - 3504) / 16) << 2));
	forge_acc(&ret, forge_bq_set_field(F_BATLOWV, 0x02));	/* 3.0 V */
	forge_acc(&ret, forge_bq_set_field(F_VRECHG, 0));		/* 100 mV */
	forge_acc(&ret, forge_bq_set_field(F_EN_TERM, 0x80));
	forge_acc(&ret, forge_bq_set_field(F_TERM_STAT, 0));
	forge_acc(&ret, forge_bq_set_field(F_WATCHDOG, 0));	/* off, see stock_init */
	/*
	 * Safety timer on, 8 h (stock writes EN_TIMER 0; this replaces the
	 * watchdog this file turns off). Per the data sheet (TI SLUSB98A) an
	 * expired timer stops charging and sets REG09 CHRG_FAULT 11; it restarts
	 * on a new charge cycle, on CHG_CONFIG 00 -> 01 and on EN_TIMER 0 -> 1;
	 * it runs at half rate in input or thermal regulation (REG07 TMR2X_EN,
	 * left at its default). Clear EN_TIMER before changing CHG_TIMER, then
	 * set it again. A field is written only when it differs, so the timer is
	 * not restarted on every tick.
	 */
	{
		u8 r05 = 0;

		if (!forge_bq_xfer(0x05, &r05, false) && (r05 & 0x06) != (1 << 1)) {
			forge_acc(&ret, forge_bq_set_field(F_EN_TIMER, 0));
			forge_acc(&ret, forge_bq_set_field(F_CHG_TIMER, 1 << 1));
		}
	}
	forge_acc(&ret, forge_bq_set_field(F_EN_TIMER, 0x08));
	forge_ctrl.field_ret = ret;
}

/*
 * Re-arm after a safety timer fault: a phone left on a USB cable with the
 * panel on can draw more than the input limit, so the cell never
 * terminates and after 8 h (16 h in regulation) the timer stops charging.
 * Per TI SLUSB98A an expired timer sets REG09 CHRG_FAULT 11 and stops
 * charging; writing REG05[3] EN_TIMER 0 -> 1 restarts the timer.
 * REG09 latches a fault until read; the snapshot reads it every tick, so
 * CHRG_FAULT 11 there means "since the last tick", and the live state is
 * REG08 CHRG_STAT (0 = not charging). The first tick with the fault and
 * CHRG_STAT 0 notes VBAT; CHRG_STAT charging again (a replug starts a new
 * cycle) clears it. The timer is re-armed only when VBAT has fallen
 * >= timer_rearm_drop_mv below that and the battery is in the charging
 * temperature zone (real and test state normal): a cell held up by the
 * load sags and gets charged again; a cell that sits at CV and never
 * terminates does not sag, and stays stopped.
 * Caller holds forge_bq_lock.
 */
static bool forge_timer_rearm = true;
module_param_named(timer_rearm, forge_timer_rearm, bool, 0644);
MODULE_PARM_DESC(timer_rearm, "with stock_init, restart the safety timer after a timer fault once VBAT sagged (1)");

static unsigned int forge_timer_rearm_drop_mv = 100;
module_param_named(timer_rearm_drop_mv, forge_timer_rearm_drop_mv, uint, 0644);
MODULE_PARM_DESC(timer_rearm_drop_mv, "VBAT drop below the fault-time VBAT that re-arms the timer (100)");

static unsigned int forge_timer_rearms;
module_param_named(timer_rearms, forge_timer_rearms, uint, 0444);
MODULE_PARM_DESC(timer_rearms, "safety timer re-arms so far (read only)");

static void forge_bq_timer_rearm(void)
{
	u8 r08 = forge_ctrl.regs[FORGE_BQ_REG08], r09 = forge_ctrl.regs[0x09];
	bool fault = ((r09 >> 4) & 3) == 3;	/* latched since the last tick */
	bool charging = (r08 >> 4) & 3;		/* CHRG_STAT, live */
	int ret;

	if (forge_ctrl.regs_ret)
		return;
	if (forge_ctrl.tfault && charging) {
		pr_notice("[FORGE_L681] chgctrl: charging again after the safety timer fault (REG08 0x%02x)\n",
			  r08);
		forge_ctrl.tfault = false;
		return;
	}
	if (!forge_ctrl.tfault) {
		if (fault && !charging) {
			forge_ctrl.tfault = true;
			forge_ctrl.tfault_mv = forge_ctrl.vbat_mv;
			pr_notice("[FORGE_L681] chgctrl: safety timer expired (REG09 0x%02x, REG08 0x%02x) at %d mV\n",
				  r09, r08, forge_ctrl.vbat_mv);
		}
		return;
	}
	if (!forge_timer_rearm || !forge_ctrl.temp_ok ||
	    forge_ctrl.state != FORGE_CTRL_NORMAL ||
	    forge_ctrl.test_state != FORGE_CTRL_NORMAL ||
	    forge_ctrl.vbat_mv > forge_ctrl.tfault_mv -
				 (int)forge_timer_rearm_drop_mv)
		return;
	ret = forge_bq_set_field(F_EN_TIMER, 0);
	forge_acc(&ret, forge_bq_set_field(F_EN_TIMER, 0x08));
	forge_timer_rearms++;
	pr_notice("[FORGE_L681] chgctrl: safety timer re-armed #%u: %d mV, %d below the fault-time %d mV: %d\n",
		  forge_timer_rearms, forge_ctrl.vbat_mv,
		  forge_ctrl.tfault_mv - forge_ctrl.vbat_mv, forge_ctrl.tfault_mv,
		  ret);
	/* a fault again later notes a new VBAT */
	forge_ctrl.tfault = false;
}

static void forge_bq_apply(bool stop)
{
	u8 r01 = 0, chg;
	int ret;

	mutex_lock(&forge_bq_lock);
	ret = forge_bq_xfer(FORGE_BQ_REG01, &r01, false);
	if (ret != forge_ctrl.rd_ret)
		pr_notice("[FORGE_L681] chgctrl: BQ24196 REG01 read %d (0x%02x)\n",
			  ret, r01);
	forge_ctrl.rd_ret = ret;
	if (ret)
		goto out;
	forge_ctrl.reg01 = r01;
	forge_bq_snapshot();
	chg = r01 & FORGE_BQ_CHG_CONFIG;
	if (chg & 0x20)		/* OTG: not charging, leave it alone */
		goto out;
	if (forge_stock_init && !forge_ctrl.regs_ret) {
		forge_bq_stock(forge_ctrl.regs[FORGE_BQ_REG08]);
		forge_bq_timer_rearm();
		/* the field writes may have changed REG01 SYS_MIN */
		if (forge_bq_xfer(FORGE_BQ_REG01, &r01, false))
			goto out;
		forge_ctrl.reg01 = r01;
	}
	if (stop && chg) {
		ret = forge_bq_write_reg01(r01, 0);
		pr_notice("[FORGE_L681] chgctrl: charging off (REG01 0x%02x -> 0x%02x): %d\n",
			  r01, forge_ctrl.reg01, ret);
		if (!ret) {
			forge_ctrl.held = true;
			forge_ctrl.clears++;
		}
	} else if (stop && forge_ctrl.held) {
		/* keep the chip in host mode, or its watchdog restores charging */
		if (!forge_bq_write_reg01(r01, 0))
			forge_ctrl.kicks++;
	} else if (!stop && !chg && (forge_ctrl.held || forge_stock_init)) {
		ret = forge_bq_write_reg01(r01, FORGE_BQ_CHG_ON);
		pr_notice("[FORGE_L681] chgctrl: charging on (REG01 0x%02x -> 0x%02x): %d\n",
			  r01, forge_ctrl.reg01, ret);
		if (!ret) {
			forge_ctrl.held = false;
			forge_ctrl.sets++;
		}
	} else if (chg) {
		/* charging by someone else (or a chip reset): not held */
		forge_ctrl.held = false;
	}
out:
	mutex_unlock(&forge_bq_lock);
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

/* strong versions of the weak l681 hooks in forge_chg_observe.c (the m681
 * SM5414 control owns forge_chg_ctrl_*) */
void forge_bq_ctrl_tick(int chrdet, int temp_dc, bool temp_ok, int vbat_mv)
{
	ktime_t now = ktime_get_boottime();
	int state = forge_ctrl.state;
	char t[16];
	bool stop;

	if (!forge_bq_board)
		return;
	if (!forge_ctrl.temp_seen && !ktime_to_ns(forge_ctrl.temp_at))
		forge_ctrl.temp_at = now;	/* the no-temperature clock starts */
	forge_ctrl.chrdet = chrdet;
	forge_ctrl.vbat_mv = vbat_mv;
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
		pr_notice("[FORGE_L681] chgctrl: battery %s (%s): %s -> %s (charger %d, %d mV, last valid temperature %lld s ago)\n",
			  t, temp_ok ? "valid" : "no sample",
			  forge_ctrl_names[forge_ctrl.state],
			  forge_ctrl_names[state], chrdet, vbat_mv,
			  ktime_ms_delta(now, forge_ctrl.temp_at) / 1000);
	}
	forge_ctrl.state = state;
	state = forge_test_temp_dc == FORGE_TEST_TEMP_OFF ? FORGE_CTRL_NORMAL :
		forge_ctrl_next(forge_ctrl.test_state, forge_test_temp_dc);
	if (state != forge_ctrl.test_state) {
		forge_dc_str(t, sizeof(t), forge_test_temp_dc);
		pr_notice("[FORGE_L681] chgctrl: test temperature %s: %s -> %s\n",
			  forge_test_temp_dc == FORGE_TEST_TEMP_OFF ? "off" : t,
			  forge_ctrl_names[forge_ctrl.test_state],
			  forge_ctrl_names[state]);
	}
	forge_ctrl.test_state = state;
	stop = forge_test_stop ||
	       (forge_temp_guard && (forge_ctrl.state != FORGE_CTRL_NORMAL ||
				     forge_ctrl.test_state != FORGE_CTRL_NORMAL));
	/* without VBUS there is nothing to charge */
	if (chrdet == 1)
		forge_bq_apply(stop);
}

bool forge_bq_ctrl_inhibited(void)
{
	return forge_bq_board && forge_ctrl.held;
}

/* REG08 CHRG_STAT (bits 5:4): 0 not charging, 1 pre-charge, 2 fast, 3 done */
int forge_bq_ctrl_status(void)
{
	u8 r08 = 0;
	int ret, stat;

	if (!forge_bq_board)
		return -ENODEV;
	mutex_lock(&forge_bq_lock);
	ret = forge_bq_xfer(FORGE_BQ_REG08, &r08, false);
	mutex_unlock(&forge_bq_lock);
	forge_ctrl.stat_ret = ret;
	if (ret)
		return ret;
	forge_ctrl.reg08 = r08;
	stat = (r08 >> 4) & 3;
	if (stat == 3)
		return forge_ctrl.vbat_mv >= FORGE_BQ_FULL_MIN_MV ?
		       POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
	return stat ? POWER_SUPPLY_STATUS_CHARGING :
	       POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int forge_ctrl_show(struct seq_file *m, void *v)
{
	const u8 *r = forge_ctrl.regs;
	char t[16];

	if (!forge_bq_board) {
		seq_puts(m, "chgctrl: not the l681 board, off\n");
		return 0;
	}
	forge_dc_str(t, sizeof(t), forge_ctrl.temp_dc);
	seq_printf(m, "chgctrl: temp_guard %d notemp_stop_s %u test_stop %d, state %s (stop at >= 46.0 C, resume < 45.0 C; stop < 0.0 C), charger %d, %d mV\n",
		   forge_temp_guard, forge_notemp_stop_s, forge_test_stop,
		   forge_ctrl_names[forge_ctrl.state], forge_ctrl.chrdet,
		   forge_ctrl.vbat_mv);
	if (forge_test_temp_dc != FORGE_TEST_TEMP_OFF) {
		forge_dc_str(t, sizeof(t), forge_test_temp_dc);
		seq_printf(m, "test temperature %s: state %s\n", t,
			   forge_ctrl_names[forge_ctrl.test_state]);
	}
	forge_dc_str(t, sizeof(t), forge_ctrl.temp_dc);
	seq_printf(m, "temperature %s, %s this tick, %s %lld s ago\n",
		   forge_ctrl.temp_seen ? t : "never valid",
		   forge_ctrl.temp_ok ? "valid" : "no valid sample",
		   forge_ctrl.temp_seen ? "last valid" : "first tick",
		   ktime_to_ns(forge_ctrl.temp_at) ?
		   ktime_ms_delta(ktime_get_boottime(), forge_ctrl.temp_at) / 1000 :
		   -1LL);
	seq_printf(m, "BQ24196 REG01 0x%02x (CHG_CONFIG %d) read %d, last write %d; held off %d, off %u on %u watchdog kicks %u; REG08 0x%02x (CHRG_STAT %d) read %d\n",
		   forge_ctrl.reg01, (forge_ctrl.reg01 >> 4) & 3,
		   forge_ctrl.rd_ret, forge_ctrl.wr_ret, forge_ctrl.held,
		   forge_ctrl.clears, forge_ctrl.sets, forge_ctrl.kicks,
		   forge_ctrl.reg08, (forge_ctrl.reg08 >> 4) & 3,
		   forge_ctrl.stat_ret);
	seq_printf(m, "safety timer: re-arm %d (drop %u mV), re-arms %u, fault %d (at %d mV)\n",
		   forge_timer_rearm, forge_timer_rearm_drop_mv, forge_timer_rearms,
		   forge_ctrl.tfault, forge_ctrl.tfault_mv);
	seq_printf(m, "stock_init %d stock_ac %d cv_mv %u: field writes %u, last %d\n",
		   forge_stock_init, forge_stock_ac, forge_cv_mv,
		   forge_ctrl.field_writes, forge_ctrl.field_ret);
	if (forge_ctrl.regs_ret) {
		seq_printf(m, "BQ24196 REG00-0A: not read (%d)\n",
			   forge_ctrl.regs_ret);
		return 0;
	}
	/* fields as in bq24196.h; codes per the BQ24196 data sheet */
	seq_printf(m, "BQ24196 REG00-0A: %*ph\n", FORGE_BQ_SNAP_LAST + 1, r);
	seq_printf(m, "BQ24196 (read only): EN_HIZ %d VINDPM %d mV IINLIM code %d (100/150/500/900/1200/1500/2000/3000 mA), CHG_CONFIG %d, ICHG %d mA, IPRECHG %d mA, ITERM %d mA, VREG %d mV, EN_TERM %d WATCHDOG %d (off/40/80/160 s) EN_TIMER %d, FAULT 0x%02x\n",
		   r[0] >> 7, 3880 + 80 * ((r[0] >> 3) & 0x0F), r[0] & 7,
		   (r[1] >> 4) & 3, 512 + 64 * (r[2] >> 2),
		   128 + 128 * (r[3] >> 4), 128 + 128 * (r[3] & 0x0F),
		   3504 + 16 * (r[4] >> 2), r[5] >> 7, (r[5] >> 4) & 3,
		   (r[5] >> 3) & 1, r[9]);
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

/* called by forge_chg_observe's late_initcall before its first tick, on
 * the l681 board only: no initcall of its own */
void __init forge_bq_ctrl_init(void)
{
	forge_bq_board = m3note_board_is(M3NOTE_BOARD_L681);
	if (!forge_bq_board)
		return;
	if (!proc_create("l681_chgctrl", 0444, NULL, &forge_ctrl_fops))
		pr_notice("[FORGE_L681] chgctrl: /proc/l681_chgctrl not created\n");
	pr_notice("[FORGE_L681] chgctrl: temperature cut-off %s, no-temperature stop %u s (BQ24196 REG01 CHG_CONFIG over I2C only)\n",
		  forge_temp_guard ? "on" : "off", forge_notemp_stop_s);
}
