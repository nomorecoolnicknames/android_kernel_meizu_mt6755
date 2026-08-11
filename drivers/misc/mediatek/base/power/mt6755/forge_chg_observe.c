/* M681 MT6351 battery telemetry; ADC gate defaults off.
 * Own AUXADC clock preparation has a separate default-off gate. The SM5414
 * charger is only read (status); no charger, GPIO, RTC or voltage control. Capacity is an
 * uncalibrated OCV ID0 estimate, not a fuel-gauge measurement.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/ratelimit.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/m3note_board.h>
#include <linux/reboot.h>
#include <linux/forge_rtc_soc.h>

#define FORGE_MT6351_CHR_CON0	0x0F78	/* cable presence, not charging */
#define FORGE_CHG_TICK_MS	10000	/* CHRDET */
#define FORGE_CHG_FULL_TICKS	3	/* battery: every 30 s */
#define FORGE_BAT_FAST_TICKS	6	/* before registration: every tick, 1 min */

/* battery monitor; MT6351 fields from stock upmu_hw.h */
#define FORGE_MT6351_TOP_CKPDN_CON0	0x023A	/* b9 RG_AUXADC_SMPS_CK_PDN */
#define FORGE_MT6351_TOP_CKPDN_CON1	0x0240	/* b10 RG_AUXADC_CK_PDN */
#define FORGE_MT6351_VBIF28_CON0	0x0A86	/* b15 DA_QI_VBIF28_EN */
#define FORGE_MT6351_AUXADC_ADC3	0x0E06	/* CH3 BATON: [11:0], [15] RDY */
#define FORGE_MT6351_AUXADC_ADC25	0x0E32	/* CH1 ISENSE by AP: [14:0], [15] RDY */
#define FORGE_MT6351_AUXADC_STA0	0x0E90	/* [11:0] channel busy */
#define FORGE_MT6351_AUXADC_RQST0_SET	0x0E98	/* W1, bit n requests channel n */
#define FORGE_MT6351_AUXADC_CON0	0x0EA2	/* b15 AUXADC_CK_AON */
#define FORGE_AUXADC_CH_ISENSE	1
#define FORGE_AUXADC_CH_BATON	3
#define FORGE_AUXADC_RDY	BIT(15)
#define FORGE_AUXADC_POLLS	16	/* further reads 1.3-1.5 ms apart, as stock */
#define FORGE_BAT_MIN_MV	2500	/* a VBAT outside this is not a sample */
#define FORGE_BAT_MAX_MV	4600
#define FORGE_BAT_EMPTY_MV	3400	/* stock SHUTDOWN_SYSTEM_VOLTAGE */
#define FORGE_BAT_EMPTY_SAMPLES	3
/* BATON accepted only by proof 'A' with the same raw value this many
 * samples in a row (5 min): the temperature is stale */
#define FORGE_BAT_A_MAX		10
#define FORGE_NTC_PULLUP_OHM	24000
#define FORGE_NTC_PULLUP_MV	2800
/* VOLTAGE_NOW and TEMP older than three missed samples read as no data */
#define FORGE_BAT_STALE_MS	(3 * FORGE_CHG_FULL_TICKS * FORGE_CHG_TICK_MS)
/* no valid sample this long and no charger: STATUS UNKNOWN */
#define FORGE_BAT_LOST_MS	(10 * 60 * 1000)

extern s32 pwrap_read(u32 adr, u32 *rdata);
extern int forge_pmic_battery_adc_prepare(void);
extern unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val,
					  unsigned int MASK, unsigned int SHIFT);

static struct delayed_work forge_chg_work;
static unsigned int forge_chg_ticks;
static int forge_chg_chrdet = -1;

static bool forge_bat_enable = true;
module_param_named(battery, forge_bat_enable, bool, 0444);

/* stock Batt_Temperature_Table, BAT_NTC_10 (wt6755_66_sz_l
 * cust_battery_meter_table_multi_profile.h:90-111) */
static const struct {
	s8 c;
	u32 ohm;
} forge_ntc[] = {
	{ -25, 87559 }, { -20, 68237 }, { -15, 53650 }, { -10, 42506 },
	{ -5, 33892 }, { 0, 27219 }, { 5, 22021 }, { 10, 17926 },
	{ 15, 14674 }, { 20, 12081 }, { 25, 10000 }, { 30, 8315 },
	{ 35, 6948 }, { 40, 5834 }, { 45, 4917 }, { 50, 4161 },
	{ 55, 3535 }, { 60, 3014 }, { 65, 2586 },
};

/* stock battery_profile_t2[0], profile ID0 (D_ATL) at 25 C, DOD % : OCV mV
 * (cust_battery_meter_table_multi_profile.h:908-984) */
static const struct forge_ocv {
	u8 dod;
	u16 mv;
} forge_ocv_id0[] = {
	{ 0, 4391 }, { 2, 4350 }, { 3, 4339 }, { 5, 4315 }, { 6, 4303 },
	{ 7, 4292 }, { 9, 4268 }, { 10, 4257 }, { 12, 4234 }, { 13, 4223 },
	{ 15, 4201 }, { 17, 4179 }, { 18, 4170 }, { 19, 4158 }, { 20, 4148 },
	{ 21, 4137 }, { 23, 4117 }, { 25, 4096 }, { 26, 4086 }, { 27, 4076 },
	{ 29, 4063 }, { 29, 4055 }, { 30, 4043 }, { 31, 4027 }, { 33, 4000 },
	{ 34, 3992 }, { 36, 3982 }, { 37, 3976 }, { 38, 3969 }, { 40, 3954 },
	{ 41, 3944 }, { 42, 3935 }, { 43, 3925 }, { 45, 3904 }, { 46, 3893 },
	{ 48, 3874 }, { 49, 3867 }, { 50, 3862 }, { 51, 3856 }, { 53, 3845 },
	{ 54, 3839 }, { 56, 3831 }, { 57, 3826 }, { 59, 3818 }, { 60, 3815 },
	{ 60, 3811 }, { 62, 3805 }, { 63, 3802 }, { 64, 3799 }, { 65, 3795 },
	{ 67, 3789 }, { 68, 3787 }, { 69, 3784 }, { 70, 3782 }, { 72, 3775 },
	{ 73, 3772 }, { 75, 3764 }, { 76, 3759 }, { 78, 3753 }, { 79, 3750 },
	{ 80, 3747 }, { 81, 3743 }, { 82, 3739 }, { 84, 3731 }, { 85, 3723 },
	{ 86, 3717 }, { 87, 3712 }, { 88, 3705 }, { 89, 3692 }, { 90, 3690 },
	{ 91, 3691 }, { 92, 3688 }, { 94, 3686 }, { 95, 3679 }, { 96, 3652 },
	{ 98, 3546 }, { 100, 3322 },
};

/* l681 (M3 Note international): stock hq6755_66_b1a_l
 * battery_profile_t2[0], profile ID0 at 25 C, DOD % : OCV mV
 * (cust_battery_meter_table_multi_profile.h). Stock picks one of two
 * profiles by the battery ID on AUXADC channel 12 (0.5 V threshold); the
 * AP AUXADC is not built here, so ID0 is assumed. The NTC table and
 * pull-up of that file equal the m681 ones above. */
static const struct forge_ocv forge_ocv_l681_id0[] = {
	{ 0, 4369 }, { 1, 4351 }, { 2, 4336 }, { 3, 4322 }, { 4, 4308 },
	{ 5, 4296 }, { 6, 4283 }, { 7, 4271 }, { 8, 4259 }, { 9, 4247 },
	{ 10, 4236 }, { 11, 4224 }, { 12, 4213 }, { 13, 4201 }, { 14, 4191 },
	{ 15, 4180 }, { 16, 4169 }, { 17, 4158 }, { 18, 4147 }, { 19, 4136 },
	{ 20, 4126 }, { 21, 4115 }, { 22, 4105 }, { 23, 4094 }, { 24, 4085 },
	{ 25, 4076 }, { 26, 4071 }, { 27, 4063 }, { 28, 4049 }, { 29, 4033 },
	{ 31, 4007 }, { 32, 3997 }, { 33, 3992 }, { 34, 3987 }, { 35, 3982 },
	{ 36, 3976 }, { 37, 3968 }, { 38, 3960 }, { 39, 3952 }, { 40, 3943 },
	{ 41, 3935 }, { 42, 3926 }, { 43, 3917 }, { 44, 3906 }, { 45, 3895 },
	{ 46, 3885 }, { 47, 3875 }, { 48, 3867 }, { 49, 3860 }, { 50, 3855 },
	{ 51, 3849 }, { 52, 3844 }, { 53, 3839 }, { 54, 3834 }, { 55, 3830 },
	{ 56, 3826 }, { 57, 3821 }, { 58, 3818 }, { 59, 3814 }, { 60, 3811 },
	{ 61, 3808 }, { 62, 3805 }, { 63, 3801 }, { 64, 3799 }, { 65, 3797 },
	{ 66, 3794 }, { 67, 3791 }, { 68, 3788 }, { 69, 3787 }, { 70, 3784 },
	{ 71, 3782 }, { 72, 3779 }, { 73, 3777 }, { 74, 3774 }, { 75, 3771 },
	{ 76, 3768 }, { 77, 3765 }, { 78, 3761 }, { 79, 3756 }, { 80, 3752 },
	{ 81, 3749 }, { 82, 3746 }, { 83, 3741 }, { 84, 3736 }, { 85, 3729 },
	{ 86, 3723 }, { 87, 3718 }, { 88, 3710 }, { 89, 3701 }, { 90, 3697 },
	{ 91, 3696 }, { 92, 3695 }, { 93, 3694 }, { 94, 3692 }, { 95, 3689 },
	{ 96, 3678 }, { 97, 3643 }, { 98, 3589 }, { 99, 3517 },
	{ 100, 3400 },
};

static bool forge_bat_l681;	/* board from the DT, set at init */
/* l681 only: capacity published, until the first valid sample, when the
 * observer registers without one (healthd reads a missing battery as 0 %
 * and BatteryService then shuts down; never 0 on error) */
#define FORGE_BAT_PLACEHOLDER_CAP	50
static bool forge_bat_placeholder;
/*
 * l681 only: with no valid VBAT sample for this long and no cable,
 * publish 0 % so healthd shuts the phone down instead of a 50 %
 * placeholder (or a held capacity) running the battery flat. Never with
 * the cable in (CHRDET 1 or unknown: LK would park a phone switched off on
 * the cable in KPOC). 0 = never.
 */
static unsigned int forge_bat_starve_s = 600;
module_param_named(l681_no_sample_off_s, forge_bat_starve_s, uint, 0644);
MODULE_PARM_DESC(l681_no_sample_off_s,
		 "l681: no valid VBAT sample for this many seconds and no cable -> capacity 0 % (600; 0 = never)");
static bool forge_bat_starved;

/* one AUXADC conversion */
struct forge_adc {
	int ret;		/* 0: the value comes from this request */
	u32 pre, raw;		/* result register before the request, and after */
	u32 sta0;		/* AUXADC_STA0 right after the request, for the log */
	int polls;
	char proof;		/* why the value counts, see forge_adc_convert() */
};

struct forge_bat_sample {
	struct forge_adc vb, bt;
	int vbat_mv, baton_mv, ntc_ohm, temp_dc, temp_ret;
	int regs_ok;
	u32 adc_con0, ckpdn0, ckpdn1, sta0, vbif28;	/* read only, for the log */
};

/* what get_property reports; written by the work only */
static DEFINE_SPINLOCK(forge_bat_lock);
static struct {
	int cap, vbat_mv, temp_dc, status, chrdet;
	bool temp_ok;
	/* CLOCK_BOOTTIME of the last valid sample: it runs through suspend,
	 * so values from before a long sleep do not pass as fresh */
	ktime_t at;
} forge_bat_pub = { .status = POWER_SUPPLY_STATUS_UNKNOWN, .chrdet = -1 };

/* filter state and the last sample, for the work and /proc */
static DEFINE_MUTEX(forge_bat_mutex);
static struct {
	struct forge_bat_sample last;
	int ret, temp_ret;	/* of the last sample */
	int vavg, vavg_chrdet, est, low;
	unsigned int a_run;	/* BATON samples in a row with only proof 'A' */
	u32 a_raw;		/* and their raw value */
	unsigned int ok, failed;
	bool have, reg_failed;
} forge_bat = { .ret = 1, .temp_ret = 1 };	/* 1: nothing sampled yet */
static struct power_supply *forge_bat_psy, *forge_usb_psy;

/*
 * One conversion, as stock PMIC_IMM_GetOneChannelValue() does it
 * (pmic_auxadc.c): request the channel, read AUXADC_STA0 at once
 * (kept for the log), poll RDY. The value counts only with a proof that it
 * comes from this request, a->proof:
 *  'R' RDY read 0 after the request, then 1;
 *  'V' the result differs from the one before the request;
 *  'A' only with @converting: an earlier conversion of the same sample was
 *      proven, so the AUXADC is processing requests, and a conversion that
 *      ended before the first read with an unchanged value (a steady BATON)
 *      is taken as it is.
 * Otherwise a RDY that stays up over an unchanged value may be a result from
 * before (the AUXADC clock gated, for one): -ESTALE, not a sample. The STA0
 * busy bits are not a proof: a modem request for the same channel sets
 * them too without refreshing the AP result.
 */
static int forge_adc_convert(unsigned int ch, u32 reg, bool converting,
			     struct forge_adc *a)
{
	bool dropped = false;
	int i;

	memset(a, 0, sizeof(*a));
	if (pwrap_read(reg, &a->pre))
		return a->ret = -EIO;
	/* W1 request only; refused unless mt_pmic_stub.battery_adc=1. */
	a->ret = (int)pmic_config_interface(FORGE_MT6351_AUXADC_RQST0_SET,
					    1, 1, ch);
	if (a->ret)
		return a->ret;
	if (pwrap_read(FORGE_MT6351_AUXADC_STA0, &a->sta0))
		return a->ret = -EIO;
	for (i = 0; i <= FORGE_AUXADC_POLLS; i++) {
		if (i)
			usleep_range(1300, 1500);
		if (pwrap_read(reg, &a->raw))
			return a->ret = -EIO;
		if (a->raw & FORGE_AUXADC_RDY)
			break;
		dropped = true;
	}
	a->polls = i;
	if (!(a->raw & FORGE_AUXADC_RDY))
		a->ret = -ETIMEDOUT;
	else if (dropped)
		a->proof = 'R';
	else if (a->raw != a->pre)
		a->proof = 'V';
	else if (converting)
		a->proof = 'A';
	else
		a->ret = -ESTALE;
	return a->ret;
}

/* NTC resistance and temperature in 0.1 C; -ERANGE outside the table */
static int forge_ntc_temp(int mv, int *ohm, int *dc)
{
	int i, r;

	if (mv <= 0 || mv >= FORGE_NTC_PULLUP_MV)
		return -ERANGE;
	r = FORGE_NTC_PULLUP_OHM * mv / (FORGE_NTC_PULLUP_MV - mv);
	*ohm = r;
	if (r > forge_ntc[0].ohm || r < forge_ntc[ARRAY_SIZE(forge_ntc) - 1].ohm)
		return -ERANGE;
	for (i = 1; i < ARRAY_SIZE(forge_ntc) - 1; i++)
		if (r >= forge_ntc[i].ohm)
			break;
	/* forge_ntc[i - 1].ohm >= r >= forge_ntc[i].ohm */
	*dc = forge_ntc[i - 1].c * 10 +
	      DIV_ROUND_CLOSEST((int)(forge_ntc[i - 1].ohm - r) *
				(forge_ntc[i].c - forge_ntc[i - 1].c) * 10,
				(int)(forge_ntc[i - 1].ohm - forge_ntc[i].ohm));
	return 0;
}

/* capacity % from the OCV table, linear between rows */
static int forge_ocv_soc(int mv)
{
	const struct forge_ocv *t = forge_bat_l681 ? forge_ocv_l681_id0 :
						    forge_ocv_id0;
	int n = forge_bat_l681 ? ARRAY_SIZE(forge_ocv_l681_id0) :
				 ARRAY_SIZE(forge_ocv_id0);
	int i, dod;

	if (mv >= t[0].mv)
		return 100 - t[0].dod;
	for (i = 1; i < n; i++) {
		const struct forge_ocv *hi = &t[i - 1];
		const struct forge_ocv *lo = &t[i];

		if (mv < lo->mv)
			continue;
		dod = lo->dod;
		/* the stock table has a few flat or rising rows */
		if (hi->mv > lo->mv)
			dod -= DIV_ROUND_CLOSEST((mv - lo->mv) * (lo->dod - hi->dod),
						 hi->mv - lo->mv);
		return 100 - dod;
	}
	return 0;
}

/* returns 0 if m->vbat_mv is a valid VBAT sample */
static int forge_bat_measure(struct forge_bat_sample *m)
{
	int ret;

	memset(m, 0, sizeof(*m));
	m->temp_ret = -ENODATA;
	m->regs_ok = !pwrap_read(FORGE_MT6351_AUXADC_CON0, &m->adc_con0) &&
		     !pwrap_read(FORGE_MT6351_TOP_CKPDN_CON0, &m->ckpdn0) &&
		     !pwrap_read(FORGE_MT6351_TOP_CKPDN_CON1, &m->ckpdn1) &&
		     !pwrap_read(FORGE_MT6351_AUXADC_STA0, &m->sta0) &&
		     !pwrap_read(FORGE_MT6351_VBIF28_CON0, &m->vbif28);
	ret = forge_pmic_battery_adc_prepare();
	if (ret)
		return ret;
	/* VBAT needs its own proof: a stale one would freeze the capacity */
	ret = forge_adc_convert(FORGE_AUXADC_CH_ISENSE,
				FORGE_MT6351_AUXADC_ADC25, false, &m->vb);
	if (ret)
		return ret;
	/* stock: 15 bits, x3 divider, 1800 mV full scale */
	m->vbat_mv = (m->vb.raw & 0x7fff) * 3 * 1800 / 32768;
	if (m->vbat_mv < FORGE_BAT_MIN_MV || m->vbat_mv > FORGE_BAT_MAX_MV)
		return -ERANGE;
	/* the VBAT conversion just proved the AUXADC converts */
	m->temp_ret = forge_adc_convert(FORGE_AUXADC_CH_BATON,
					FORGE_MT6351_AUXADC_ADC3, true, &m->bt);
	if (!m->temp_ret) {
		/* stock: 12 bits, x2 divider, 1800 mV full scale */
		m->baton_mv = (m->bt.raw & 0xfff) * 2 * 1800 / 4096;
		m->temp_ret = forge_ntc_temp(m->baton_mv, &m->ntc_ohm,
					     &m->temp_dc);
	}
	return 0;
}

static void forge_bat_log_sample(const char *what, int ret,
				 const struct forge_bat_sample *m)
{
	pr_notice("[FORGE_M681] battery: %s (%d): VBAT %d mV (CH1 %d: 0x%04x -> 0x%04x, STA0 0x%04x, %d polls, proof %c) BATON %d mV %d ohm (CH3 %d: 0x%04x -> 0x%04x, STA0 0x%04x, %d polls, proof %c, temp %d) | pre-prepare AUXADC_CON0 0x%04x TOP_CKPDN_CON0 0x%04x CON1 0x%04x STA0 0x%04x VBIF28_CON0 0x%04x (rd %d)\n",
		  what, ret, m->vbat_mv, m->vb.ret, m->vb.pre, m->vb.raw,
		  m->vb.sta0, m->vb.polls, m->vb.proof ?: '-',
		  m->baton_mv, m->ntc_ohm, m->bt.ret, m->bt.pre, m->bt.raw,
		  m->bt.sta0, m->bt.polls, m->bt.proof ?: '-',
		  m->temp_ret, m->adc_con0, m->ckpdn0, m->ckpdn1, m->sta0,
		  m->vbif28, m->regs_ok);
}

/*
 * The published percentage survives a reboot, as stock keeps it in the
 * RTC FG SOC field. Without it the first valid sample (the loaded voltage
 * on the curve) became the capacity on every boot, so each reboot under
 * load set it lower. forge_rtc_soc=1 (default): the field is read at init
 * (forge_rtc_soc_get, pwrap_hal_v1.c); the first sample takes it when it
 * is 1..100, VBAT is at least 3400 mV (stock SHUTDOWN_SYSTEM_VOLTAGE) and
 * it is within forge_rtc_soc_tol (40, stock
 * CUST_POWERON_DELTA_CAPACITY_TOLRANCE) points of the estimate, otherwise
 * the estimate as before; every change of the published capacity and a
 * reboot write it back. The 0 % rule is not touched: a published 0 stays 0
 * (the field stores it as 1, as stock).
 */
static int forge_rtc_soc = 1;
module_param_named(rtc_soc, forge_rtc_soc, int, 0644);
MODULE_PARM_DESC(rtc_soc, "keep the published capacity in the RTC FG SOC field and start from it after a reboot (1)");
static int forge_rtc_soc_tol = 40;
module_param_named(rtc_soc_tol, forge_rtc_soc_tol, int, 0644);
/* the restart after the l681 no-sample 0 % takes the last valid capacity
 * of this boot, which may be old (the starve window, sleep): a narrower
 * tolerance than the boot one */
static int forge_rtc_soc_tol_restart = 10;
module_param_named(rtc_soc_tol_restart, forge_rtc_soc_tol_restart, int, 0644);
static int forge_rtc_soc_boot = -1;	/* the field at init, -1 once used */
static bool forge_rtc_soc_restart;	/* forge_rtc_soc_boot is the last one */
/* the last value tried, so a failing write is retried on the next change
 * only, not every pass - but for a failed RTC relock (forge_rtc_soc_store) */
static int forge_rtc_soc_tried = -1;
/* the last capacity from a valid sample: the reboot notifier keeps this, not
 * the published one, which may be the l681 no-sample 0 % or the
 * placeholder */
static int forge_rtc_soc_last = -1;

int __weak forge_rtc_soc_get(void)
{
	return -ENODEV;
}

int __weak forge_rtc_soc_set(int soc)
{
	return -ENODEV;
}

bool __weak forge_rtc_relock_needed(void)
{
	return false;
}

/* the first capacity: the saved one, if it is plausible */
static int forge_rtc_seed_pick(int saved, int est, int vbat_mv, int tol,
			       const char **why)
{
	if (saved < 1 || saved > 100) {
		*why = "no saved value";
		return est;
	}
	if (vbat_mv < 3400) {
		*why = "battery below 3400 mV";
		return est;
	}
	if (abs(saved - est) > tol) {
		*why = "saved value too far from the estimate";
		return est;
	}
	*why = "saved value";
	return saved;
}

static int forge_rtc_seed(int est, int vbat_mv)
{
	const char *why = "off";
	int cap = est;

	if (forge_rtc_soc && forge_rtc_soc_boot >= 0)
		cap = forge_rtc_seed_pick(forge_rtc_soc_boot, est, vbat_mv,
					  forge_rtc_soc_restart ?
					  forge_rtc_soc_tol_restart :
					  forge_rtc_soc_tol, &why);
	pr_notice("[FORGE_M681] battery: %s capacity %d %% (%s; estimate %d %%, saved %d, %d mV)\n",
		  forge_rtc_soc_restart ? "restart" : "first", cap, why, est,
		  forge_rtc_soc_boot, vbat_mv);
	forge_rtc_soc_boot = -1;
	forge_rtc_soc_restart = false;
	return cap;
}

/*
 * process context only (forge_chg_fn work, reboot notifier). force: write
 * even when this value was tried last (the reboot notifier: a hand write
 * through rtc_soc_test, or a failed try, must not be what survives);
 * forge_rtc_soc_set itself skips the unlock when the field already holds it.
 * A failed RTC relock is retried on every pass, not on the next change of
 * the capacity: the write interface must not stay open.
 */
static void forge_rtc_soc_store(int cap, bool force)
{
	if (!forge_rtc_soc || cap < 0 || cap > 100 ||
	    (!force && cap == forge_rtc_soc_tried &&
	     !forge_rtc_relock_needed()))
		return;
	forge_rtc_soc_tried = cap;
	forge_rtc_soc_set(cap);
}

/*
 * The capacity to publish after a valid sample, from the one published
 * before: the first sample sets it; then it moves one percent per sample
 * toward the estimate from the averaged VBAT, up only with a charger, down
 * only without one. 0 % only after FORGE_BAT_EMPTY_SAMPLES samples in a row
 * with the average below FORGE_BAT_EMPTY_MV (not a single reading: a load
 * dip of a battery that still has charge would shut Android down early),
 * else at least 1 %. Caller holds forge_bat_mutex.
 */
static int forge_bat_filter(int vbat_mv, int chrdet, int cap)
{
	/* a charger plugged or pulled steps VBAT: restart the average */
	if (!forge_bat.have || chrdet != forge_bat.vavg_chrdet) {
		forge_bat.vavg = vbat_mv;
		forge_bat.vavg_chrdet = chrdet;
	} else {
		forge_bat.vavg += (vbat_mv - forge_bat.vavg) / 4;
	}
	forge_bat.est = forge_ocv_soc(forge_bat.vavg);
	if (!forge_bat.have)
		cap = forge_rtc_seed(forge_bat.est, vbat_mv);
	else if (chrdet == 1 && forge_bat.est > cap)
		cap++;
	else if (chrdet != 1 && forge_bat.est < cap)
		cap--;
	forge_bat.have = true;
	forge_bat.low = forge_bat.vavg < FORGE_BAT_EMPTY_MV ?
			forge_bat.low + 1 : 0;
	if (forge_bat.low >= FORGE_BAT_EMPTY_SAMPLES)
		return 0;
	return cap < 1 ? 1 : cap;
}

/*
 * Charger state for STATUS. The SM5414 (i2c-1 0x49, stock sm5414.c; its
 * driver is not built here) is only read: STATUS 0x06 for the log,
 * INT1/INT2 for the stock full latch (sm5414_apply_int_latches): DONE or
 * TOPOFF sets full, VBUS UVLO/OVP or CHGRSTF clears it, and stock vetoes
 * a full below forge_full_min_mv = 4250 mV (a top-off from a starved
 * input). Reading INT1/INT2 clears those latches in the chip; nothing
 * else consumes them here. Cable out clears full. A charger that does not
 * answer leaves CHARGING on CHRDET alone.
 */
#define FORGE_SM5414_INT1	0x00
#define FORGE_SM5414_INT2	0x01
#define FORGE_SM5414_STATUS	0x06
#define FORGE_FULL_MIN_MV	4250
static bool forge_chg_full;
static int forge_sm_ret = -ENODEV;
static u8 forge_sm_status, forge_sm_int1, forge_sm_int2;

/* forge_chg_ctrl.c: i2c-1 0x49, one lock for all SM5414 access of the line */
extern int forge_chg_sm5414_read(u8 reg, u8 *val);

static void forge_sm5414_poll(int vbat_mv)
{
	u8 st = 0, i1 = 0, i2 = 0;
	int ret;

	ret = forge_chg_sm5414_read(FORGE_SM5414_STATUS, &st);
	if (!ret)
		ret = forge_chg_sm5414_read(FORGE_SM5414_INT1, &i1);
	if (!ret)
		ret = forge_chg_sm5414_read(FORGE_SM5414_INT2, &i2);
	if (ret != forge_sm_ret)
		pr_notice("[FORGE_M681] charger: SM5414 read %d (STATUS 0x%02x)\n", ret, st);
	forge_sm_ret = ret;
	if (ret)
		return;
	forge_sm_status = st;
	forge_sm_int1 |= i1;	/* for the log: what was ever seen */
	forge_sm_int2 |= i2;
	if ((i2 & 0x03) && vbat_mv >= FORGE_FULL_MIN_MV) {	/* DONE | TOPOFF */
		if (!forge_chg_full)
			pr_notice("[FORGE_M681] charger: full (INT1 0x%02x INT2 0x%02x, %d mV)\n",
				  i1, i2, vbat_mv);
		forge_chg_full = true;
	}
	if ((i1 & 0xC0) || (i2 & 0x04)) {	/* VBUS UVLO/OVP, CHGRSTF */
		if (forge_chg_full)
			pr_notice("[FORGE_M681] charger: full cleared (INT1 0x%02x INT2 0x%02x)\n",
				  i1, i2);
		forge_chg_full = false;
	}
}

/* charger control (forge_chg_ctrl.c: temperature cut-off
 * and the stock SM5414 settings) hooks in here; without it nothing acts */
void __weak forge_chg_ctrl_tick(int chrdet, int temp_dc, bool temp_ok, int vbat_mv)
{
}

bool __weak forge_chg_ctrl_inhibited(void)
{
	return false;
}

/* l681 charger control (forge_bq24196_ctrl.c: temperature
 * cut-off and the BQ24196 charge state). The board picks the hooks: m681
 * runs the SM5414 ones above, unchanged; l681 runs only these. Without
 * the file nothing acts. */
void __weak forge_bq_ctrl_tick(int chrdet, int temp_dc, bool temp_ok, int vbat_mv)
{
}

bool __weak forge_bq_ctrl_inhibited(void)
{
	return false;
}

/* POWER_SUPPLY_STATUS_* from the BQ24196 with the cable in, or -errno */
int __weak forge_bq_ctrl_status(void)
{
	return -ENODEV;
}

void __weak forge_bq_ctrl_init(void)
{
}

/* l681, cable in: no SM5414 on this board */
static int forge_bat_status_l681(void)
{
	int st;

	if (forge_bq_ctrl_inhibited())	/* charging off by the charger control */
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	st = forge_bq_ctrl_status();
	/* Cable presence alone cannot prove a charging current. */
	return st < 0 ? POWER_SUPPLY_STATUS_UNKNOWN : st;
}

static int forge_bat_status(int chrdet)
{
	if (chrdet == 0) {
		forge_chg_full = false;
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
	if (chrdet != 1)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	if (forge_bat_l681)
		return forge_bat_status_l681();
	forge_sm5414_poll(forge_bat_pub.vbat_mv);
	if (forge_chg_ctrl_inhibited())	/* CHGEN off by the charger control */
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	return forge_chg_full ? POWER_SUPPLY_STATUS_FULL :
		POWER_SUPPLY_STATUS_CHARGING;
}

static int forge_bat_get(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	unsigned long flags;
	bool fresh;
	int ret = 0;

	spin_lock_irqsave(&forge_bat_lock, flags);
	fresh = ktime_before(ktime_get_boottime(),
			     ktime_add_ms(forge_bat_pub.at, FORGE_BAT_STALE_MS));
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = forge_bat_pub.status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* Only a fresh, validated VBAT is evidence of presence. */
		if (fresh)
			val->intval = 1;
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* never an error: healthd reads a failed capacity as 0 % */
		val->intval = forge_bat_pub.cap;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (fresh)
			val->intval = forge_bat_pub.vbat_mv * 1000;
		else
			ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (fresh && forge_bat_pub.temp_ok)
			val->intval = forge_bat_pub.temp_dc;
		else
			ret = -ENODATA;
		break;
	default:
		ret = -EOPNOTSUPP;
	}
	spin_unlock_irqrestore(&forge_bat_lock, flags);
	return ret;
}

static int forge_usb_get(struct power_supply *psy,
			 enum power_supply_property psp,
			 union power_supply_propval *val)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&forge_bat_lock, flags);
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		ret = -EOPNOTSUPP;
	else if (forge_bat_pub.chrdet < 0)
		ret = -ENODATA;
	else
		val->intval = forge_bat_pub.chrdet;
	spin_unlock_irqrestore(&forge_bat_lock, flags);
	return ret;
}

static enum power_supply_property forge_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property forge_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc forge_bat_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = forge_bat_props,
	.num_properties = ARRAY_SIZE(forge_bat_props),
	.get_property = forge_bat_get,
	/* TEMP may be -ENODATA; no thermal zone on top of it */
	.no_thermal = true,
};

/* CHRDET only: no charger type detection (BC1.2 needs PMIC writes) */
static const struct power_supply_desc forge_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = forge_usb_props,
	.num_properties = ARRAY_SIZE(forge_usb_props),
	.get_property = forge_usb_get,
	.no_thermal = true,
};

static void forge_bat_register(void)
{
	struct power_supply *psy;

	psy = power_supply_register(NULL, &forge_bat_desc, NULL);
	if (IS_ERR(psy)) {
		forge_bat.reg_failed = true;
		pr_err("[FORGE_M681] battery: power_supply 'battery' not registered: %ld\n",
		       PTR_ERR(psy));
		return;
	}
	forge_bat_psy = psy;
	psy = power_supply_register(NULL, &forge_usb_desc, NULL);
	if (IS_ERR(psy))
		pr_err("[FORGE_M681] battery: power_supply 'usb' not registered: %ld\n",
		       PTR_ERR(psy));
	else
		forge_usb_psy = psy;
	if (forge_bat_placeholder)
		pr_err("[FORGE_M681] battery: l681: registered WITHOUT a valid sample, capacity placeholder %d %% until the first one (never 0 on error)\n",
		       forge_bat_pub.cap);
	else
		pr_notice("[FORGE_M681] battery: registered, %d mV, capacity %d %%; %s\n",
			  forge_bat_pub.vbat_mv, forge_bat_pub.cap,
			  forge_bat_l681 ?
			  "l681 OCV profile ID0 (25 C) assumed, battery ID (AUXADC CH12) not read" :
			  "OCV profile ID0 (D_ATL, 25 C) assumed, battery ID (AUXADC CH13) not read");
}

static void forge_bat_update(int chrdet)
{
	struct forge_bat_sample m;
	int status = forge_bat_status(chrdet);
	int ret = forge_bat_measure(&m);
	int cap = forge_bat_pub.cap, old_cap = cap, est = 0;
	bool changed, usb_changed, empty = false;
	unsigned long flags;
	ktime_t now;

	mutex_lock(&forge_bat_mutex);
	/*
	 * Proof 'A' says the AUXADC converts, not that CH3 does: a CH3 path
	 * that stopped (review of eb1923d1a) would keep an old BATON
	 * forever and hide a charger overheating. After FORGE_BAT_A_MAX valid
	 * samples in a row with only 'A' on the same raw value, the
	 * temperature is stale until an 'R' or 'V' proof comes back. A sample
	 * without a BATON conversion (VBAT or CH3 failed) leaves the count
	 * alone: TEMP is not published from it anyway. An 'A' on a new raw
	 * value starts again at 1: someone did convert CH3.
	 */
	if (!ret && !m.bt.ret) {
		if (m.bt.proof != 'A')
			forge_bat.a_run = 0;
		else if (m.bt.raw == forge_bat.a_raw)
			forge_bat.a_run++;
		else
			forge_bat.a_run = 1;
		forge_bat.a_raw = m.bt.raw;
		if (forge_bat.a_run >= FORGE_BAT_A_MAX && !m.temp_ret)
			m.temp_ret = -ESTALE;
	}
	forge_bat.last = m;
	/* log changes of the outcome, not every sample */
	if (ret != forge_bat.ret || (!ret && m.temp_ret != forge_bat.temp_ret))
		forge_bat_log_sample(ret ? "no sample" : "sample", ret, &m);
	forge_bat.ret = ret;
	forge_bat.temp_ret = m.temp_ret;
	if (ret) {
		forge_bat.failed++;
		/* "in a row" means valid samples in a row */
		forge_bat.low = 0;
	} else {
		forge_bat.ok++;
		/*
		 * The l681 no-sample 0 % left forge_bat.have set, so the first
		 * valid sample after it would filter down from 0 and publish 1,
		 * which would then go to the RTC. Start over as on a boot instead:
		 * the last valid capacity is taken under the same rule as the
		 * saved one (forge_rtc_seed_pick), else the estimate. With no
		 * valid sample yet in this boot (last -1) the RTC value read at
		 * boot stays the candidate.
		 */
		if (forge_bat_starved) {
			forge_bat.have = false;
			if (forge_rtc_soc_last >= 0) {
				forge_rtc_soc_boot = forge_rtc_soc_last;
				forge_rtc_soc_restart = true;
			}
		}
		cap = forge_bat_filter(m.vbat_mv, chrdet, cap);
		est = forge_bat.est;
		empty = !cap && old_cap;
	}
	mutex_unlock(&forge_bat_mutex);

	/*
	 * No valid sample for FORGE_BAT_LOST_MS (the AUXADC stopped converting,
	 * or battery_adc=0) and no charger: the capacity is only the last
	 * value, and a draining battery would never reach the 0 % rule. Say so:
	 * STATUS UNKNOWN and an error in the log. The capacity is not changed,
	 * nothing is made up; cable presence does not
	 * prove charging, and its status remains UNKNOWN. CLOCK_BOOTTIME, so
	 * a sleep counts as time without a sample.
	 */
	now = ktime_get_boottime();
	if (ret && forge_bat.have && chrdet != 1 &&
	    ktime_after(now, ktime_add_ms(forge_bat_pub.at, FORGE_BAT_LOST_MS))) {
		static DEFINE_RATELIMIT_STATE(lost_rs, FORGE_BAT_LOST_MS / 1000 * HZ, 1);

		status = POWER_SUPPLY_STATUS_UNKNOWN;
		if (__ratelimit(&lost_rs))
			pr_err("[FORGE_M681] battery: no valid sample for %lld s and no charger (last %d): capacity held at %d %%, status unknown\n",
			       ktime_ms_delta(now, forge_bat_pub.at) / 1000,
			       ret, cap);
	}

	if (ret && forge_bat_l681 && forge_bat_starve_s && chrdet == 0 &&
	    ktime_after(now, ktime_add_ms(forge_bat_pub.at,
					  (s64)forge_bat_starve_s * 1000))) {
		if (!forge_bat_starved)
			pr_emerg("[FORGE_M681] battery: l681: no valid VBAT sample for %lld s and no cable (last %d): capacity %d -> 0 %%, shutdown by the battery service\n",
				 ktime_ms_delta(now, forge_bat_pub.at) / 1000,
				 ret, cap);
		forge_bat_starved = true;
		cap = 0;
	}

	spin_lock_irqsave(&forge_bat_lock, flags);
	changed = forge_bat_pub.status != status || forge_bat_pub.cap != cap;
	usb_changed = forge_bat_pub.chrdet != chrdet;
	forge_bat_pub.status = status;
	forge_bat_pub.chrdet = chrdet;
	if (!ret) {
		forge_bat_pub.cap = cap;
		forge_bat_pub.vbat_mv = m.vbat_mv;
		forge_bat_pub.temp_ok = !m.temp_ret;
		forge_bat_pub.temp_dc = m.temp_dc;
		forge_bat_pub.at = now;
	} else if (forge_bat_starved) {
		forge_bat_pub.cap = 0;
	}
	spin_unlock_irqrestore(&forge_bat_lock, flags);
	if (!ret) {
		forge_rtc_soc_last = cap;
		forge_rtc_soc_store(cap, false);
	}
	if (!ret && forge_bat_starved) {
		forge_bat_starved = false;
		pr_notice("[FORGE_M681] battery: l681: valid sample %d mV again after the no-sample 0 %%\n",
			  m.vbat_mv);
	}

	if (empty)
		pr_notice("[FORGE_M681] battery: %d samples in a row below %d mV (last %d mV): capacity 0 %%\n",
			  FORGE_BAT_EMPTY_SAMPLES, FORGE_BAT_EMPTY_MV,
			  m.vbat_mv);
	else if (!ret && forge_bat_psy && cap != old_cap)
		pr_info("[FORGE_M681] battery: capacity %d -> %d %% (%d mV, average %d, estimate %d %%, charger %d)\n",
			old_cap, cap, m.vbat_mv, forge_bat.vavg, est, chrdet);

	if (!ret && forge_bat_placeholder) {
		forge_bat_placeholder = false;
		pr_notice("[FORGE_M681] battery: l681: first valid sample %d mV, capacity %d %% replaces the placeholder\n",
			  m.vbat_mv, cap);
	}
	if (!ret && !forge_bat_psy && !forge_bat.reg_failed)
		forge_bat_register();
	else if (forge_bat_psy && changed)
		power_supply_changed(forge_bat_psy);
	if (forge_usb_psy && usb_changed)
		power_supply_changed(forge_usb_psy);
	/* the published sample: temperature only if valid and fresh */
	if (forge_bat_l681) {
		forge_bq_ctrl_tick(chrdet, forge_bat_pub.temp_dc,
				   forge_bat_pub.temp_ok && !ret,
				   forge_bat_pub.vbat_mv);
		return;
	}
	forge_chg_ctrl_tick(chrdet, forge_bat_pub.temp_dc,
			    forge_bat_pub.temp_ok && !ret, forge_bat_pub.vbat_mv);
	/* a CHGEN change made by that tick shows now, not 30 s later */
	if (chrdet == 1 && forge_bat_psy) {
		int now_status = forge_chg_ctrl_inhibited() ?
			POWER_SUPPLY_STATUS_NOT_CHARGING :
			forge_chg_full ? POWER_SUPPLY_STATUS_FULL :
			POWER_SUPPLY_STATUS_CHARGING;

		spin_lock_irqsave(&forge_bat_lock, flags);
		changed = forge_bat_pub.status != now_status &&
			  forge_bat_pub.status != POWER_SUPPLY_STATUS_UNKNOWN;
		if (changed)
			forge_bat_pub.status = now_status;
		spin_unlock_irqrestore(&forge_bat_lock, flags);
		if (changed)
			power_supply_changed(forge_bat_psy);
	}
}

static void forge_chg_fn(struct work_struct *work)
{
	u32 con0;
	int chrdet = pwrap_read(FORGE_MT6351_CHR_CON0, &con0) ? -1 :
		     (con0 >> 5) & 1;
	bool full = !(forge_chg_ticks % FORGE_CHG_FULL_TICKS) ||
		    chrdet != forge_chg_chrdet;

	/* no battery yet: healthd looks for one only when it starts, so
	 * retry every tick for the first minute */
	if (forge_bat_enable && !forge_bat_psy && !forge_bat.reg_failed &&
	    forge_chg_ticks < FORGE_BAT_FAST_TICKS)
		full = true;
	forge_chg_ticks++;
	if (full && forge_bat_enable)
		forge_bat_update(chrdet);
	forge_chg_chrdet = chrdet;
	queue_delayed_work(system_freezable_power_efficient_wq, &forge_chg_work,
			   msecs_to_jiffies(FORGE_CHG_TICK_MS));
}

/* a cable change seen by the USB VBUS poll (mtk_usb.c, 250 ms): run the
 * tick now instead of up to FORGE_CHG_TICK_MS later; it reads CHRDET
 * itself and does the full update on a change */
void forge_chg_kick(void)
{
	if (forge_chg_work.work.func)
		mod_delayed_work(system_freezable_power_efficient_wq,
				 &forge_chg_work, 0);
}
EXPORT_SYMBOL(forge_chg_kick);

static void forge_temp_str(char *buf, size_t len, int dc)
{
	snprintf(buf, len, "%s%d.%d C", dc < 0 ? "-" : "", abs(dc) / 10,
		 abs(dc) % 10);
}

static void forge_bat_show(struct seq_file *m)
{
	const struct forge_bat_sample *l = &forge_bat.last;
	unsigned long flags;
	int cap, status, chrdet;
	ktime_t at;
	char t[16];

	if (!forge_bat_enable) {
		seq_puts(m, "battery: off (forge_chg_observe.battery=0)\n");
		return;
	}
	spin_lock_irqsave(&forge_bat_lock, flags);
	cap = forge_bat_pub.cap;
	status = forge_bat_pub.status;
	chrdet = forge_bat_pub.chrdet;
	at = forge_bat_pub.at;
	spin_unlock_irqrestore(&forge_bat_lock, flags);
	mutex_lock(&forge_bat_mutex);
	forge_temp_str(t, sizeof(t), l->temp_dc);
	seq_printf(m, "battery: %s, capacity %d %% (estimate %d %%, average %d mV, below %d mV: %d/%d), status %d, charger %d; samples ok %u failed %u, last valid %lld s ago; %s\n",
		   forge_bat_psy ? (forge_bat_placeholder ?
				    "registered with placeholder capacity" :
				    "registered") :
		   forge_bat.reg_failed ? "registration failed" : "not registered",
		   cap, forge_bat.est, forge_bat.vavg, FORGE_BAT_EMPTY_MV,
		   forge_bat.low, FORGE_BAT_EMPTY_SAMPLES, status, chrdet,
		   forge_bat.ok, forge_bat.failed,
		   forge_bat.have ?
		   ktime_ms_delta(ktime_get_boottime(), at) / 1000 : -1LL,
		   forge_bat_l681 ? "OCV profile l681 ID0 (25 C) assumed" :
		   "OCV profile ID0 (D_ATL, 25 C) assumed");
	seq_printf(m, "last sample %d: VBAT %d mV (CH1 %d: 0x%04x -> 0x%04x, STA0 0x%04x, %d polls, proof %c); BATON %d mV, %d ohm, %s (CH3 %d: 0x%04x -> 0x%04x, STA0 0x%04x, %d polls, proof %c, only 'A' %u/%u, temp %d)\n",
		   forge_bat.ret, l->vbat_mv, l->vb.ret, l->vb.pre, l->vb.raw,
		   l->vb.sta0, l->vb.polls, l->vb.proof ?: '-',
		   l->baton_mv, l->ntc_ohm, l->temp_ret ? "no temp" : t,
		   l->bt.ret, l->bt.pre, l->bt.raw, l->bt.sta0, l->bt.polls,
		   l->bt.proof ?: '-', forge_bat.a_run, FORGE_BAT_A_MAX,
		   l->temp_ret);
	seq_printf(m, "auxadc (read only): CON0 0x%04x (CK_AON %u) TOP_CKPDN_CON0 0x%04x (SMPS_CK_PDN %u) CON1 0x%04x (CK_PDN %u) STA0 0x%04x VBIF28_CON0 0x%04x (enabled %u) rd %d\n",
		   l->adc_con0, (l->adc_con0 >> 15) & 1, l->ckpdn0,
		   (l->ckpdn0 >> 9) & 1, l->ckpdn1, (l->ckpdn1 >> 10) & 1,
		   l->sta0, l->vbif28, (l->vbif28 >> 15) & 1, l->regs_ok);
	seq_printf(m, "charger SM5414 (read only): rd %d STATUS 0x%02x (SUSPEND %u BATDET %u THEMSHDN %u), INT1 seen 0x%02x INT2 seen 0x%02x, full %d\n",
		   forge_sm_ret, forge_sm_status, forge_sm_status & 1,
		   (forge_sm_status >> 1) & 1, (forge_sm_status >> 2) & 1,
		   forge_sm_int1, forge_sm_int2, forge_chg_full);
	mutex_unlock(&forge_bat_mutex);
}

static int forge_chg_show(struct seq_file *m, void *v)
{
	forge_bat_show(m);
	return 0;
}

static int forge_chg_open(struct inode *inode, struct file *file)
{
	return single_open(file, forge_chg_show, NULL);
}

static const struct file_operations forge_chg_fops = {
	.open = forge_chg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int forge_rtc_soc_reboot(struct notifier_block *nb, unsigned long code,
				void *unused)
{
	if (forge_rtc_soc_last >= 0)
		forge_rtc_soc_store(forge_rtc_soc_last, true);
	return NOTIFY_DONE;
}

static struct notifier_block forge_rtc_soc_nb = {
	.notifier_call = forge_rtc_soc_reboot,
};

static int __init forge_chg_observe_init(void)
{
	forge_bat_l681 = m3note_board_is(M3NOTE_BOARD_L681);
	if (forge_rtc_soc) {
		forge_rtc_soc_boot = forge_rtc_soc_get();
		pr_notice("[FORGE_M681] battery: saved capacity in the RTC: %d\n",
			  forge_rtc_soc_boot);
		register_reboot_notifier(&forge_rtc_soc_nb);
	}
	/* l681 charger identity report on i2c1; m681 reads nothing here */
	if (forge_bat_l681) {
		m3note_hw_report();
		forge_bq_ctrl_init();	/* before the first tick below */
	}
	if (!forge_bat_enable)
		pr_notice("[FORGE_M681] battery: monitor off (forge_chg_observe.battery=0)\n");
	if (!proc_create("m681_battery", 0444, NULL, &forge_chg_fops))
		pr_notice("[FORGE_M681] charger: /proc/m681_battery not created\n");
	INIT_DELAYED_WORK(&forge_chg_work, forge_chg_fn);
	/* Publish a valid first sample before userspace scans power supplies. */
	forge_chg_fn(&forge_chg_work.work);
	/*
	 * l681: a missing first sample must not leave Android without a
	 * battery. healthd scans once at start, reads the missing capacity as
	 * 0 % and BatteryService shuts down unless a charger is online (KPOC
	 * trap on the cable). Register now with a placeholder capacity: STATUS
	 * unknown, PRESENT/VOLTAGE/TEMP -ENODATA, the 'usb' supply carries
	 * CHRDET. The first valid sample sets the real estimate
	 * (forge_bat_filter(): first sample = estimate).
	 */
	if (forge_bat_l681 && forge_bat_enable && !forge_bat_psy &&
	    !forge_bat.reg_failed) {
		forge_bat_pub.cap = FORGE_BAT_PLACEHOLDER_CAP;
		forge_bat_placeholder = true;
		forge_bat_register();
	}
	return 0;
}
late_initcall(forge_chg_observe_init);
