/*
 * CPU rail writer for MT6755 (VOSEL_CTRL=0 mode).
 *
 * Rails (both boards): VPROC is MT6311 on i2c-3 0x6b, VSRAM_PROC is MT6351
 * on the wrap; the preloader leaves both with VOSEL_CTRL = 0 (0x88 b1,
 * 0x06A0 b1), so each output follows its software VOSEL (0x8D, 0x06A8).
 * Every write here puts ONE code into all three selectors of a rail -
 * VOSEL_ON and VOSEL_SLEEP first (no output change), VOSEL last (moves the
 * output) - so the rail is defined whatever the mode bit or a sleep signal
 * says. VOSEL_CTRL is never written.
 *
 * Writes go through the PMIC gate of pwrap_hal_v1.c: MT6351 by
 * forge_pmic_gate_write() (field and limit check, one-write token,
 * read-back), MT6311 after forge_pmic_gate_mt6311_ok(). Taking the rails
 * switches that gate to enforce. After each rail write: a pause (stock
 * _calc_pmic_settle_time, at least forge_vrail_settle_us = 200 us, since
 * the preloader's ramp registers 0x8B/0x8C = 0x88, 0x94 = 0x01 are not
 * stock) and a read-back of every selector plus the outputs (MT6311 0x90
 * twice, 50 us apart, and OC 0x8A b7; MT6351 0x06AE polled up to 1 ms -
 * the rails are taken only if 0x06AE is the Gray code of 0x06A8 then,
 * fvr_gray()). Any mismatch, a write refused by the gate or a move without
 * progress latches forge_vrail_fault: no write ever follows. The gate's
 * rows are claimed by this file only.
 *
 * Limits: VPROC 0x40..0x58 (1.000..1.150 V; 1.000 V floor, stock
 * MAX_VPROC_VOLT ceiling), VSRAM 0x50..0x58 (1.100..1.150 V); VSRAM =
 * VPROC + 100 mV clamped to 1.150 V (stock FY), and 0 <= VSRAM - VPROC <=
 * 175 mV at every write (up: VSRAM then VPROC; down: VPROC then VSRAM).
 *
 * Sleep: the rails are taken only with S3 not allowed (mt_sleep.allow_s3 =
 * 0) and deep idle / SODI3 / SODI off - SPM writes these rails through its
 * own command phases (the deep-idle phase writes 0x06AA outside the gate).
 * While the rails are owned those switches refuse to come on
 * (mt_sleep.allow_s3, mt_idle.forge_*: on only under
 * forge_vrail_sleep_lock(), which also guards the owner flag); every move
 * re-checks them anyway and is refused (no write) if one is on. This file
 * never lowers a rail on its own: only the caller knows what the running
 * clocks need, so the park before suspend or reboot is mt_cpufreq's (it
 * steps the clocks down, sets the rails, then forge_vrail_hold() blocks
 * further moves).
 *
 * One user: forge_vrail_take() binds the rails to the caller's cookie, and
 * set/hold/release refuse any other. forge_vrail_set() returns -EAGAIN
 * when it wrote nothing (held, allow = 0, sleep on): the rails are where
 * they were. Any other error is a fault, and the VPROC it hands out is then
 * the lowest of the old code, the target and the 0x8D/0x8E/0x90 read back
 * after the failure - a failed move may have landed in part, and the
 * caller must not count on the higher one. allow = 0 after the take is a
 * stop, not a fault: no write, -EAGAIN.
 *
 * Off by default: forge_vrail.allow = 0.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/m3note_board.h>
#include <mt-plat/mt_pmic_wrap.h>

#define FVR_TAG		"[FORGE_VRAIL] "
#define FVR_BUS		3
#define FVR_ADDR	0x6b
#define FVR_CID_E1	0x0110
#define FVR_CID_E2	0x0120
#define FVR_CID_E3	0x0130
#define FVR_VPROC_MIN	0x40	/* 1.000 V, the floor */
#define FVR_VPROC_MAX	0x58	/* 1.150 V, MAX_VPROC_VOLT */
#define FVR_VSRAM_MIN	0x50	/* 1.100 V */
#define FVR_VSRAM_MAX	0x58	/* 1.150 V, MAX_VSRAM_VOLT */
#define FVR_VSRAM_OVER	16	/* NORMAL_DIFF_VRSAM_VPROC, 100 mV */
#define FVR_MAX_DIFF	28	/* MAX_DIFF_VSRAM_VPROC - 2500, 175 mV */

/* pwrap_hal_v1.c, the CPU-rail gate */
extern int forge_pmic_gate_claim(const void *cookie);
extern int forge_pmic_gate_write(const void *cookie, u32 reg, u32 mask, u32 shift, u32 code,
				 u32 *readback);
extern bool forge_pmic_gate_mt6311_ok(u8 reg, u8 code);
extern void forge_pmic_gate_enforce_on(void);
/* spm_v2: the S3 permission and the idle switches ([0] deep idle,
 * [1] SODI3, [2] SODI; mt_idle_mt6755.c idle_switch[], IDLE_TYPE_DP = 0) */
extern bool m681_allow_s3;
extern int idle_switch[];

/* 0 by default. A test build passes make FORGE_VRAIL_ALLOW=1 (the Makefile
 * sets CFLAGS_forge_vrail.o only): the ROM arms cpufreq at boot and the
 * rails are taken only at that point, so a later echo 1 > allow comes too
 * late. The .config is the same either way; the boot ceiling 1196/689 MHz
 * (1.000 V) holds until forge_cpu_max_* is raised by hand. */
#ifndef FORGE_VRAIL_ALLOW_DEFAULT
#define FORGE_VRAIL_ALLOW_DEFAULT 0
#endif
static bool forge_vrail_allow = FORGE_VRAIL_ALLOW_DEFAULT;
module_param_named(allow, forge_vrail_allow, bool, 0644);
MODULE_PARM_DESC(allow, "M3 Note 4.4: let the CPU DVFS take the VPROC/VSRAM rails (0 by default; 1 only in a test build)");
/* read only; 200 us is the measured settle floor */
static unsigned int forge_vrail_settle_us = 200;
module_param_named(settle_us, forge_vrail_settle_us, uint, 0444);
static bool forge_vrail_fault;
module_param_named(fault, forge_vrail_fault, bool, 0444);
static unsigned int forge_vrail_writes, forge_vrail_errors;
module_param_named(writes, forge_vrail_writes, uint, 0444);
module_param_named(errors, forge_vrail_errors, uint, 0444);

static DEFINE_MUTEX(fvr_lock);
/* the owner flag and the sleep switches change only under this lock: a
 * switch comes on only with the rails not owned, the rails are owned only
 * with every switch off - no window where both hold */
static DEFINE_SPINLOCK(fvr_sw_lock);
static const char fvr_cookie[] = "forge_vrail";	/* the PMIC gate claim */
static struct i2c_adapter *fvr_adap;	/* held while owner */
static bool fvr_owner, fvr_held, fvr_06ae;
static const void *fvr_user;		/* the take's cookie */
static u8 fvr_vproc, fvr_vsram;		/* last read-back codes */
static u8 fvr_pl_vproc, fvr_pl_vsram;	/* codes found when taken */

static u32 fvr_uv10(u8 code)	/* both rails: 0.600 V + code * 6.25 mV */
{
	return 60000 + code * 625;
}

/*
 * 0x06AE DA_NI_VSRAM_PROC_VOSEL is the output code in Gray code; 0x06B0
 * DA_NI_VSRAM_PROC_VOSEL_SYNC is the same code in binary (the stock
 * pmic_dvt.c test expects VOSEL_SYNC == i and VOSEL == pmic_gray_code[i]
 * after VOSEL = i). Example: VOSEL 0x50 reads 0x06AE 0x78 = 0x50 ^ 0x28.
 */
static u8 fvr_gray(u8 code)
{
	return code ^ (code >> 1);
}

static u8 fvr_code(u32 uv10)	/* round up, like VOLT_TO_PMIC_VAL */
{
	return uv10 <= 60000 ? 0 : (u8)min_t(u32, (uv10 - 60000 + 624) / 625, 0x7F);
}

/* stock _calc_pmic_settle_time: up ceil(d/1250)+5, down d*2/625+5, >= 25 us */
static void fvr_pause(u8 from, u8 to)
{
	u32 a = fvr_uv10(from), b = fvr_uv10(to);
	unsigned int us = b > a ? DIV_ROUND_UP(b - a, 1250) + 5 : (a - b) * 2 / 625 + 5;

	us = max3(us, 25U, READ_ONCE(forge_vrail_settle_us));
	usleep_range(us, us + 20);
}

static bool fvr_sleep_on(void)
{
	return READ_ONCE(m681_allow_s3) || READ_ONCE(idle_switch[0]) ||
	       READ_ONCE(idle_switch[1]) || READ_ONCE(idle_switch[2]);
}

static int fvr_read(u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 wbuf = reg;
	int ret;

	memset(msgs, 0, sizeof(msgs));
	msgs[0].addr = FVR_ADDR;
	msgs[0].len = 1;
	msgs[0].buf = &wbuf;
	msgs[1].addr = FVR_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;
	ret = i2c_transfer(fvr_adap, msgs, 2);
	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

static int fvr_write(u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 wbuf[2] = { reg, val };
	int ret;

	if (!forge_pmic_gate_mt6311_ok(reg, val & 0x7F))
		return -EPERM;
	memset(&msg, 0, sizeof(msg));
	msg.addr = FVR_ADDR;
	msg.len = 2;
	msg.buf = wbuf;
	ret = i2c_transfer(fvr_adap, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static void fvr_fail(const char *what, u8 from, u8 to, int ret)
{
	forge_vrail_errors++;
	WRITE_ONCE(forge_vrail_fault, true);
	pr_emerg(FVR_TAG "%s 0x%02x -> 0x%02x failed (%d): fault latched, no more rail writes\n",
		 what, from, to, ret);
}

static const u8 fvr_vproc_rb[] = { 0x8D, 0x8E, 0x90 };

/* VPROC: 0x8E, 0x8F, then 0x8D (the output in mode 0); read back all, 0x90, OC */
static int fvr_write_vproc(u8 code)
{
	static const u8 sel[] = { 0x8E, 0x8F, 0x8D };
	u8 v = 0, out = 0, oc = 0;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(sel) && !ret; i++) {
		ret = fvr_read(sel[i], &v);
		if (!ret && (v & 0x7F) != code)
			ret = fvr_write(sel[i], (v & 0x80) | code);
	}
	fvr_pause(fvr_vproc, code);
	for (i = 0; i < ARRAY_SIZE(sel) && !ret; i++)
		ret = fvr_read(sel[i], &v) ?: ((v & 0x7F) != code ? -EIO : 0);
	if (!ret)
		ret = fvr_read(0x90, &out) ?: fvr_read(0x8A, &oc) ?:
		      ((out & 0x7F) != code || (oc & 0x80) ? -EIO : 0);
	/* L7: 0x90 may be a digital code ahead of a slow analog fall: read it
	 * again 50 us later, it must still say the code */
	if (!ret) {
		usleep_range(50, 60);
		ret = fvr_read(0x90, &out) ?: ((out & 0x7F) != code ? -EIO : 0);
	}
	forge_vrail_writes++;
	if (ret) {
		/* 0x8D may have landed and the output be anywhere between:
		 * count on the lowest of the old code, the target and whatever
		 * 0x8D/0x8E/0x90 read back now */
		u8 low = min(fvr_vproc, code);

		for (i = 0; i < ARRAY_SIZE(fvr_vproc_rb); i++)
			if (!fvr_read(fvr_vproc_rb[i], &v))
				low = min_t(u8, low, v & 0x7F);
		fvr_fail("VPROC", fvr_vproc, code, ret);
		fvr_vproc = low;
		return ret;
	}
	fvr_vproc = code;
	return 0;
}

/* VSRAM: VOSEL_ON 0x06AA, VOSEL_SLEEP 0x06AC, then VOSEL 0x06A8 (the output) */
static int fvr_write_vsram(u8 code)
{
	static const u32 sel[] = { 0x06AA, 0x06AC, 0x06A8 };
	u32 rb = 0, ae = 0;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(sel) && !ret; i++)
		ret = forge_pmic_gate_write(fvr_cookie, sel[i], 0x7F, 0, code, &rb);
	fvr_pause(fvr_vsram, code);
	/* the output (0x06AE in Gray code, which matched 0x06A8 at take -
	 * else the rails are not taken) must reach the code before VPROC may
	 * follow: polled up to 1 ms after the pause */
	for (i = 0; !ret && i <= 20; i++) {
		if (pwrap_read(0x06AE, &ae)) {
			ret = -EIO;
			break;
		}
		if ((ae & 0x7F) == fvr_gray(code))
			break;
		if (i == 20)
			ret = -ETIMEDOUT;
		else
			usleep_range(50, 60);
	}
	forge_vrail_writes++;
	if (ret) {
		/* the fault stops every write, so VSRAM is never raised again;
		 * the lower code is only what gets reported */
		fvr_fail("VSRAM", fvr_vsram, code, ret);
		fvr_vsram = min(fvr_vsram, code);
		return ret;
	}
	fvr_vsram = code;
	return 0;
}

/* one move to (tp, ts) keeping 0 <= VSRAM - VPROC <= 175 mV; fvr_lock held */
static int fvr_move(u8 tp, u8 ts)
{
	int ret = 0, rounds = 0;

	while (!ret && (fvr_vproc != tp || fvr_vsram != ts)) {
		if (++rounds > 8) {
			fvr_fail("move (no progress)", fvr_vproc, tp, -EIO);
			return -EIO;
		}
		if (tp > fvr_vproc || ts > fvr_vsram) {
			u8 s = max_t(u8, min_t(u8, ts, fvr_vproc + FVR_MAX_DIFF), fvr_vsram);

			if (s != fvr_vsram)
				ret = fvr_write_vsram(s);
			if (!ret && fvr_vproc != min(tp, s))
				ret = fvr_write_vproc(min(tp, s));
		} else {
			u8 p = max_t(u8, tp, fvr_vsram > FVR_MAX_DIFF ? fvr_vsram - FVR_MAX_DIFF : 0);

			if (p != fvr_vproc)
				ret = fvr_write_vproc(p);
			if (!ret && fvr_vsram != max(ts, p))
				ret = fvr_write_vsram(max(ts, p));
		}
	}
	return ret;
}

/*
 * Take the rails for the CPU DVFS: allowed, sleep off, MT6311 answering
 * with a known CID, VOSEL_CTRL 0 on both rails, no OC, the three selectors
 * of each rail and the outputs on one code inside the limits. Returns the
 * codes as voltages (10 uV) or -EPERM/-ENODEV/-EIO.
 */
int forge_vrail_take(const void *cookie, unsigned int *vproc_uv10, unsigned int *vsram_uv10)
{
	u8 r[0x91];
	u32 a0 = 0, a8 = 0, aa = 0, ac = 0, ae = 0, b0 = 0;
	static const u8 regs[] = { 0x00, 0x01, 0x88, 0x8A, 0x8D, 0x8E, 0x8F, 0x90 };
	unsigned int i, chip;
	u32 a6 = 0;
	unsigned long flags;
	int ret = 0;
	u8 vp, vs;

	might_sleep();
	if (!cookie)
		return -EINVAL;
	pr_emerg(FVR_TAG "take requested: allow %d (build default %d), fault %d\n",
		 READ_ONCE(forge_vrail_allow), FORGE_VRAIL_ALLOW_DEFAULT, READ_ONCE(forge_vrail_fault));
	if (!READ_ONCE(forge_vrail_allow) || READ_ONCE(forge_vrail_fault))
		return -EPERM;
	/* L1: the DT board as well as the MT6311 CID below */
	if (!m3note_board_known_is(M3NOTE_BOARD_M681) &&
	    !m3note_board_known_is(M3NOTE_BOARD_L681))
		return -ENODEV;
	if (fvr_sleep_on()) {
		pr_emerg(FVR_TAG "NOT taken: S3 allowed or deep idle/SODI on (allow_s3 %d, dpidle %d, soidle3 %d, soidle %d) - freq-only\n",
			 m681_allow_s3, idle_switch[0], idle_switch[1], idle_switch[2]);
		return -EPERM;
	}
	mutex_lock(&fvr_lock);
	if (fvr_owner) {
		ret = fvr_user == cookie ? 0 : -EBUSY;
		goto out;
	}
	fvr_adap = i2c_get_adapter(FVR_BUS);
	if (!fvr_adap) {
		ret = -ENODEV;
		goto out;
	}
	memset(r, 0, sizeof(r));
	for (i = 0; i < ARRAY_SIZE(regs) && !ret; i++)
		ret = fvr_read(regs[i], &r[regs[i]]);
	if (!ret)
		ret = pwrap_read(0x06A0, &a0) || pwrap_read(0x06A8, &a8) ||
		      pwrap_read(0x06AA, &aa) || pwrap_read(0x06AC, &ac) ||
		      pwrap_read(0x06AE, &ae) || pwrap_read(0x06A6, &a6) ? -EIO : 0;
	/* 0x06B0 is printed only: a failed read leaves 0xFFFF, not -EIO */
	if (!ret && pwrap_read(0x06B0, &b0))
		b0 = 0xFFFF;
	chip = ((unsigned int)r[0x00] << 8) | r[0x01];
	vp = r[0x90] & 0x7F;
	vs = a8 & 0x7F;
	if (!ret && (!(chip == FVR_CID_E1 || chip == FVR_CID_E2 || chip == FVR_CID_E3) ||
		     (r[0x88] & 0x02) || (a0 & 0x02) || (r[0x8A] & 0x80) ||
		     (r[0x8D] & 0x7F) != vp || (r[0x8E] & 0x7F) != vp || (r[0x8F] & 0x7F) != vp ||
		     (aa & 0x7F) != vs || (ac & 0x7F) != vs ||
		     vp < FVR_VPROC_MIN || vp > FVR_VPROC_MAX ||
		     vs < FVR_VSRAM_MIN || vs > FVR_VSRAM_MAX || vs < vp || vs - vp > FVR_MAX_DIFF))
		ret = -EPERM;
	/* VSRAM moves only with its output proven to follow 0x06A8 (Gray
	 * code); 0x06B0 is only printed - the binary copy, not proven here */
	if (!ret && (ae & 0x7F) != fvr_gray(vs))
		ret = -EPERM;
	pr_emerg(FVR_TAG "take: MT6311 chip 0x%04x 0x88 0x%02x 0x8A 0x%02x 0x8D 0x%02x 0x8E 0x%02x 0x8F 0x%02x 0x90 0x%02x | MT6351 0x06A0 0x%04x 0x06A6 0x%04x 0x06A8 0x%04x 0x06AA 0x%04x 0x06AC 0x%04x 0x06AE 0x%04x (Gray of 0x%02x) 0x06B0 0x%04x -> %d\n",
		 chip, r[0x88], r[0x8A], r[0x8D], r[0x8E], r[0x8F], r[0x90],
		 a0, a6, a8, aa, ac, ae, fvr_gray(vs), b0, ret);
	if (!ret && forge_pmic_gate_claim(fvr_cookie))
		ret = -EBUSY;
	if (ret) {
		i2c_put_adapter(fvr_adap);
		fvr_adap = NULL;
		goto out;
	}
	fvr_vproc = fvr_pl_vproc = vp;
	fvr_vsram = fvr_pl_vsram = vs;
	/* 0x06AE is checked after VSRAM writes only if it already follows
	 * 0x06A8 here: its field layout is not proven on this board */
	fvr_06ae = true;	/* checked above (M3) */
	fvr_user = cookie;
	fvr_held = false;
	/* S3/deep idle/SODI refuse to come on from now on
	 * (forge_vrail_sleep_lock()); one came on between the check above and
	 * here - give the rails back untouched */
	spin_lock_irqsave(&fvr_sw_lock, flags);
	if (!fvr_sleep_on())
		WRITE_ONCE(fvr_owner, true);
	spin_unlock_irqrestore(&fvr_sw_lock, flags);
	if (!fvr_owner) {
		fvr_user = NULL;
		i2c_put_adapter(fvr_adap);
		fvr_adap = NULL;
		ret = -EPERM;
		pr_emerg(FVR_TAG "NOT taken: S3/deep idle/SODI came on during the take\n");
		goto out;
	}
	forge_pmic_gate_enforce_on();
	pr_emerg(FVR_TAG "rails owned by cpufreq: VPROC 0x%02x VSRAM 0x%02x (mode VOSEL_CTRL=0), settle >= %u us, 0x06AE check %d\n",
		 vp, vs, forge_vrail_settle_us, fvr_06ae);
out:
	if (!ret) {
		*vproc_uv10 = fvr_uv10(fvr_vproc);
		*vsram_uv10 = fvr_uv10(fvr_vsram);
	}
	mutex_unlock(&fvr_lock);
	return ret;
}
EXPORT_SYMBOL(forge_vrail_take);

/* set VPROC (VSRAM follows the stock rule); read-back voltages out */
int forge_vrail_set(const void *cookie, unsigned int vproc_uv10, unsigned int *vproc_rb,
		    unsigned int *vsram_rb)
{
	u8 tp = fvr_code(vproc_uv10);
	u8 ts = clamp_t(u8, tp + FVR_VSRAM_OVER, FVR_VSRAM_MIN, FVR_VSRAM_MAX);
	int ret;

	might_sleep();
	if (tp < FVR_VPROC_MIN || tp > FVR_VPROC_MAX)
		return -EINVAL;
	mutex_lock(&fvr_lock);
	if (!fvr_owner || fvr_user != cookie) {
		ret = -ENODEV;
	} else if (READ_ONCE(forge_vrail_fault)) {
		ret = -EIO;
	} else if (fvr_held || !READ_ONCE(forge_vrail_allow)) {
		/* nothing written - held for suspend/reboot, or allow=0
		 * after the take (L4) */
		ret = -EAGAIN;
	} else if (fvr_sleep_on()) {
		pr_emerg_ratelimited(FVR_TAG "S3 or an SPM idle came on while the rails are owned - move refused, rails left at VPROC 0x%02x VSRAM 0x%02x\n",
				     fvr_vproc, fvr_vsram);
		ret = -EAGAIN;
	} else {
		ret = fvr_move(tp, ts);
	}
	*vproc_rb = fvr_uv10(fvr_vproc);
	*vsram_rb = fvr_uv10(fvr_vsram);
	mutex_unlock(&fvr_lock);
	return ret;
}
EXPORT_SYMBOL(forge_vrail_set);

/* block (true) or allow (false) further moves; mt_cpufreq holds the rails
 * after its own park before suspend and reboot */
int forge_vrail_hold(const void *cookie, bool hold)
{
	int ret = 0;

	mutex_lock(&fvr_lock);
	if (fvr_owner && fvr_user == cookie)	/* N6 */
		fvr_held = hold;
	else
		ret = -EPERM;
	mutex_unlock(&fvr_lock);
	if (ret)
		pr_err(FVR_TAG "hold %d refused: not the owner of the rails\n", hold);
	return ret;
}
EXPORT_SYMBOL(forge_vrail_hold);

/* give the rails back untouched (the caller's init failed after the
 * take); S3 and the SPM idles may come on again. The gate claim stays:
 * these rows are written by this file only. */
int forge_vrail_release(const void *cookie)
{
	int ret = 0;

	mutex_lock(&fvr_lock);
	if (fvr_owner && fvr_user == cookie) {
		unsigned long flags;

		spin_lock_irqsave(&fvr_sw_lock, flags);
		WRITE_ONCE(fvr_owner, false);
		spin_unlock_irqrestore(&fvr_sw_lock, flags);
		fvr_user = NULL;
		fvr_held = false;
		i2c_put_adapter(fvr_adap);
		fvr_adap = NULL;
		pr_emerg(FVR_TAG "rails released at VPROC 0x%02x VSRAM 0x%02x\n", fvr_vproc, fvr_vsram);
	} else {
		ret = -EPERM;
	}
	mutex_unlock(&fvr_lock);
	return ret;
}
EXPORT_SYMBOL(forge_vrail_release);

/* PL codes found at take (for the caller's park target), 10 uV */
unsigned int forge_vrail_pl_vproc(void)
{
	return fvr_uv10(fvr_pl_vproc);
}
EXPORT_SYMBOL(forge_vrail_pl_vproc);

/* the S3 permission and the SPM idle switches refuse to come on while
 * this is true (mt_sleep.c, mt_idle.c) */
bool forge_vrail_owned(void)
{
	return READ_ONCE(fvr_owner);
}
EXPORT_SYMBOL(forge_vrail_owned);

/* a take may follow (allow, no fault) - pmic_init351 then leaves
 * VSRAM in VOSEL_CTRL = 0 (mt_cpufreq forge_dvfs_rails_wanted()) */
bool forge_vrail_allowed(void)
{
	return READ_ONCE(forge_vrail_allow) && !READ_ONCE(forge_vrail_fault);
}
EXPORT_SYMBOL(forge_vrail_allowed);

/* a sleep switch comes on between these two, and only if this said false:
 * the owner cannot change in between */
bool forge_vrail_sleep_lock(unsigned long *flags)
{
	spin_lock_irqsave(&fvr_sw_lock, *flags);
	return READ_ONCE(fvr_owner);
}
EXPORT_SYMBOL(forge_vrail_sleep_lock);

void forge_vrail_sleep_unlock(unsigned long flags)
{
	spin_unlock_irqrestore(&fvr_sw_lock, flags);
}
EXPORT_SYMBOL(forge_vrail_sleep_unlock);

/* L5: say at boot which build this is */
static int __init forge_vrail_init(void)
{
	pr_err(FVR_TAG "built with allow default %d (%s build)\n", FORGE_VRAIL_ALLOW_DEFAULT,
	       FORGE_VRAIL_ALLOW_DEFAULT ? "TEST" : "daily");
	return 0;
}
late_initcall(forge_vrail_init);
