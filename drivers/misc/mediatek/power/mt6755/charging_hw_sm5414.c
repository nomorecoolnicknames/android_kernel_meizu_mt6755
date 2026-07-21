/*
 * M681 charging HAL: SM5414 switching charger behind the GM20
 * chr_control_interface().
 *
 * Logic ported from the stock 3.10 charging_hw_sm5414.c; command table
 * shape and includes follow mt6757/charging_hw_bq25890.c (4.4 idiom).
 *
 * Hardware (stock DCT wt6755_66_sz_l):
 *   CHGEN net = GPIO115. This driver never drives it: all charge gating
 *     goes through the SM5414 I2C CTRL.CHGEN bit (sm5414_set_chgen; the
 *     stock #else path when the DCT pin is undefined).
 *   nSHDN = GPIO4,   1 = chip active
 *   cable detect     = MT6351 PMIC RGS_CHRDET (pmic irq 39 ->
 *                      chrdet_int_handler -> do_chrdet_int_task)
 *   BC1.1 charger type = in-tree pmic_chr_type_det.c
 *
 * Runtime gate:
 *   charging_hw_sm5414.forge_chg=1 (default) full charger control
 *   charging_hw_sm5414.forge_chg=0 observe only: every register/GPIO write
 *     is logged and skipped, reads still work (the LK/HW default, always
 *     charging).
 *   Fail-safe rule: no error path may end with charging disabled; disable
 *   is only ever executed as an explicit, gated, logged command.
 *
 * Deviation from stock: CV is clamped to 4.35 V (stock clamps to the 4.4 V
 * step); the cell is a 4.35 V part. Stock behaviour:
 * charging_hw_sm5414.forge_cv_step=12 (4.4 V).
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#ifdef CONFIG_M3NOTE_UNIFIED
/* M681/L681 kernel: charging_hw_m3note.c owns the public names and calls
 * this HAL only on M681. */
#define chr_control_interface chr_control_interface_sm5414
#define charging_type_detection_done charging_type_detection_done_sm5414
#define charging_type_det_done charging_type_det_done_sm5414
#endif

#include <linux/reboot.h>

#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/mtk_gpio.h>
#include <mach/mtk_battery_meter.h>
#include <mach/mtk_charging.h>
#include <mach/mtk_pmic.h>
/* BMT_status — the battery thread's current sample; read-only here, used by
 * the forge_full_min_mv CV-proximity guard in charging_get_charging_status().
 * MUST come after <mach/mtk_battery_meter.h>: that header defines
 * SHUTDOWN_SYSTEM_VOLTAGE unguarded, battery_common.h defines it under an
 * #ifndef, and the reverse order is a -Werror redefinition. */
#include <mt-plat/battery_common.h>
/* NOT <mtk_sleep.h>: that dispatcher keys on CONFIG_ARCH_MT6755, but this
 * tree sets CONFIG_MACH_MT6755, so it resolves to nothing. Go direct. */
#include <spm_v2/mtk_sleep.h>

#include <linux/m3note_board.h>
#include "sm5414.h"

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (ARRAY_SIZE(array))

#define FORGE_CHG_TAG "[FORGE_CHG][HW]"

#define GPIO_SM5414_SHDN_PIN	(4 | 0x80000000)	/* stock DCT */
/* No CHGEN GPIO define on purpose: charge gating is I2C CTRL.CHGEN only
 * (see the header). */

/* ============================================================ // */
/* Runtime gate */
/* ============================================================ // */
static int forge_chg = 1;
module_param(forge_chg, int, 0644);
MODULE_PARM_DESC(forge_chg,
	"m681 charger stack gate: 1=full SM5414 control (default), 0=observe-only (leave HW-default charging untouched)");

/* CHGCTRL3 BATREG step for the CV clamp: 10 = 4.35V (safe default),
 * 12 = 4.4V (stock value). */
static int forge_cv_step = BATREG_4_3_5_0_V;
module_param(forge_cv_step, int, 0644);

/* Power-off/reset gate. The GM20 battery check (check_battery_exist ->
 * CHARGING_CMD_SET_POWER_OFF -> here -> kernel_power_off) can shut the
 * board down ~3 s into boot, before any log channel flushes (PMIC
 * power-off -> charger-present auto-boot -> LK battery picture -> loop).
 * Default 0 = log the request (with BATON/CHRDET state) and refuse;
 * charging_hw_sm5414.forge_chg_poweroff=1 for stock behaviour.
 * Runtime-writable. Non-static: the same knob gates the two shutdown sites
 * outside this file, FG_DAEMON_CMD_SET_POWEROFF (battery_meter_fg_20.c,
 * netlink from the userspace gauge daemon) and the DLPT low-SOC
 * kernel_restart (battery_common_fg_20.c). */
#ifndef CONFIG_M3NOTE_UNIFIED
int forge_chg_poweroff;
module_param(forge_chg_poweroff, int, 0644);
MODULE_PARM_DESC(forge_chg_poweroff,
	"m681 chg-diag: 0=log+refuse kernel_power_off/restart requests (default), 1=allow");
#else
/* one knob for both HALs: charging_hw_m3note.forge_chg_poweroff */
extern int forge_chg_poweroff;
#endif

/* Serialize the BATON battery-detect read against PMIC AUXADC
 * conversions. The BATON pin is shared: battery temperature is measured by
 * muxing the same pin into the AUXADC (ch3, PMIC_AUX_BATON_AP, from the
 * battery thread's GetBatteryData) while the RGS_BATON_UNDET comparator
 * watches it for battery removal. check_battery_exist() can run from the
 * hv-detect thread concurrently with a ch3 conversion; an UNDET sample taken
 * while the ADC loads the pin can misread "absent", and 3 back-to-back
 * reads in one check turn one transient into a power-off. The conversion
 * side holds pmic_adc_mutex for the whole conversion
 * (PMIC_IMM_GetOneChannelValue); taking the same lock here
 * (pmic_auxadc_lock/unlock, as in pmic_throttling_dlpt.c) rules the overlap
 * out. Register behaviour is unchanged. forge_baton_lock=0 reverts to the
 * unserialized stock timing. */
extern void pmic_auxadc_lock(void);
extern void pmic_auxadc_unlock(void);
static int forge_baton_lock = 1;
module_param(forge_baton_lock, int, 0644);
MODULE_PARM_DESC(forge_baton_lock,
	"m681 chg-fix: 1=serialize BATON detect vs AUXADC conversions (default), 0=stock unserialized");

/* CV-proximity check on the charger IC's "full" verdict. An SM5414
 * TOPOFF/DONE edge only means "charge current fell below the 150 mA top-off
 * threshold"; that equals "cell full" only while the chip regulates at CV.
 * forge_bc11_skip pins the charger type to STANDARD_HOST (500 mA input), so
 * when the system draws more than that the charge current stays under the
 * top-off threshold at any pack voltage and the chip reports top-off from a
 * starved input (seen at 4178..4216 mV against the 4350 mV CV). Reject a
 * full verdict the programmed CV cannot account for, as linear_charging.c's
 * BAT_BatteryFullAction() gates its recharge decision on BMT_status.bat_vol
 * vs batt_cust_data.recharging_voltage (switch_charging.c trusts the IC
 * unconditionally). Default 4250 = CV 4350 - 100 mV; 0 disables the
 * check. */
static int forge_full_min_mv = 4250;
module_param(forge_full_min_mv, int, 0644);
MODULE_PARM_DESC(forge_full_min_mv,
	"m681 chg-full: reject a SM5414 full verdict below this pack voltage in mV (default 4250 = CV 4350 - 100); 0 disables");

static inline int forge_chg_active(const char *what)
{
	if (forge_chg)
		return 1;
	pr_notice(FORGE_CHG_TAG " gate OFF: skip %s\n", what);
	return 0;
}

/* ============================================================ // */
/* [FORGE_AMP] loudspeaker/charger CHGEN hold                     */
/* ============================================================ // */
/* While the audio side has the speaker amp engaged, keep CTRL.CHGEN=1 and
 * defer any battery FSM disable, so VBUS/charger current, not the battery
 * alone, backs the PA peaks (a VBAT collapse under load hangs the board).
 * The deferred disable is applied on hold release.
 * Caveats:
 * 1. This only helps if charging itself works: a garbage bat-temp reading
 *    leads to an under-temperature CHGEN=0, which would be deferred here.
 *    The deferral window is bounded by the amp engage.
 * 2. charging_enable_otg() writes CHGEN directly (not via charging_enable)
 *    and bypasses the deferral: boosting VBUS with charge enabled must win
 *    over the audio hold. */
static bool forge_amp_chgen_hold_on;	/* amp engaged, hold active */
static bool forge_amp_chgen_deferred_off; /* FSM asked CHGEN=0 during hold */

void forge_sm5414_amp_chgen_hold(int hold)
{
	if (!m3note_board_is(M3NOTE_BOARD_M681))
		return;
	if (hold) {
		forge_amp_chgen_hold_on = true;
		forge_amp_chgen_deferred_off = false;
		sm5414_set_chgen(CHARGE_EN);
		pr_notice(FORGE_CHG_TAG " [FORGE_AMP] CHGEN HOLD on: I2C CHGEN=1 (amp engaged; FSM offs deferred)\n");
	} else {
		forge_amp_chgen_hold_on = false;
		if (forge_amp_chgen_deferred_off) {
			forge_amp_chgen_deferred_off = false;
			sm5414_set_chgen(CHARGE_DIS);
			pr_notice(FORGE_CHG_TAG " [FORGE_AMP] CHGEN HOLD off: deferred FSM OFF applied (I2C CHGEN=0)\n");
		} else {
			pr_notice(FORGE_CHG_TAG " [FORGE_AMP] CHGEN HOLD off: no deferred OFF\n");
		}
	}
}

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */
kal_bool charging_type_det_done = KAL_TRUE;

/* CHGCTRL3 BATREG: 4.100V + 25mV/step (uV), stock table verbatim */
static const unsigned int VBAT_CV_VTH[] = {
	4100000, 4125000, 4150000, 4175000,
	4200000, 4225000, 4250000, 4275000,
	4300000, 4325000, 4350000, 4375000,
	4400000, 4425000, 4450000, 4475000
};

/* CHGCTRL2 FASTCHG: 100mA + 50mA/step, unit 0.01mA, stock table verbatim */
static const unsigned int CS_VTH[] = {
	10000, 15000, 20000, 25000,
	30000, 35000, 40000, 45000,
	50000, 55000, 60000, 65000,
	70000, 75000, 80000, 85000,
	90000, 95000, 100000, 105000,
	110000, 115000, 120000, 125000,
	130000, 135000, 140000, 145000,
	150000, 155000, 160000, 165000,
	170000, 175000, 180000, 185000,
	190000, 195000, 200000, 205000,
	210000, 215000, 220000, 225000,
	230000, 235000, 240000, 245000,
	250000
};

/* VBUSCTRL VBUSLIMIT: 100mA + 50mA/step, unit 0.01mA, stock table verbatim */
static const unsigned int INPUT_CS_VTH[] = {
	10000, 15000, 20000, 25000,
	30000, 35000, 40000, 45000,
	50000, 55000, 60000, 65000,
	70000, 75000, 80000, 85000,
	90000, 95000, 100000, 105000,
	110000, 115000, 120000, 125000,
	130000, 135000, 140000, 145000,
	150000, 155000, 160000, 165000,
	170000, 175000, 180000, 185000,
	190000, 195000, 200000, 205000
};

/* PMIC RG_VCDT_HV_VTH steps, stock table verbatim */
static const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,
	BATTERY_VOLT_04_500000_V, BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,
	BATTERY_VOLT_06_500000_V, BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,
	BATTERY_VOLT_09_500000_V, BATTERY_VOLT_10_500000_V
};

static unsigned int charging_error;
static unsigned int charging_set_error_state(void *data);

/* ============================================================ // */
static unsigned int charging_value_to_parameter(const unsigned int *parameter,
						const unsigned int array_size,
						const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

	pr_notice(FORGE_CHG_TAG " Can't find the parameter\n");
	return parameter[0];
}

static unsigned int charging_parameter_to_value(const unsigned int *parameter,
						const unsigned int array_size,
						const unsigned int val)
{
	unsigned int i;

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_notice(FORGE_CHG_TAG " NO register value match. val=%d\n", val);
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList,
					   unsigned int number,
					   unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {
			if (pList[i] <= level)
				return pList[i];
		}
		pr_notice(FORGE_CHG_TAG " Can't find closest level, small value first\n");
		return pList[0];
	}

	for (i = 0; i < number; i++) {
		if (pList[i] <= level)
			return pList[i];
	}
	pr_notice(FORGE_CHG_TAG " Can't find closest level, large value first\n");
	return pList[number - 1];
}

static unsigned int charging_hw_init(void *data)
{
	if (!forge_chg_active("hw_init"))
		return STATUS_OK;

	/* nSHDN high = chip running (idempotent, LK leaves it charging) */
	mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SM5414_SHDN_PIN, GPIO_OUT_ONE);

	sm5414_set_topoff(TOPOFF_150mA);
	sm5414_set_batreg(forge_cv_step);
	sm5414_set_aiclth(AICL_THRESHOLD_4_4_V);
#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
	sm5414_set_autostop(AUTOSTOP_EN);
	sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
	sm5414_set_autostop(AUTOSTOP_DIS);
#endif
	return STATUS_OK;
}

static unsigned int charging_dump_register(void *data)
{
	sm5414_dump_register();
	return STATUS_OK;
}

static unsigned int charging_enable(void *data)
{
	unsigned int enable = *(unsigned int *)(data);

	if (enable == KAL_TRUE) {
		/* enabling charge is ALWAYS allowed, gate or not:
		 * fail towards charging on. */
		forge_amp_chgen_deferred_off = false; /* FSM wants ON again */
		sm5414_set_chgen(CHARGE_EN);
		pr_notice(FORGE_CHG_TAG " charging_enable: ON (I2C CHGEN=1)\n");
	} else {
		if (!forge_chg_active("charging_enable(OFF)"))
			return STATUS_OK;
		/* [FORGE_AMP] amp engage CHGEN hold: defer the disable so the
		 * charger keeps sourcing the PA; applied on hold release. See
		 * forge_sm5414_amp_chgen_hold() (incl. caveat 1). */
		if (forge_amp_chgen_hold_on) {
			forge_amp_chgen_deferred_off = true;
			pr_notice(FORGE_CHG_TAG " [FORGE_AMP] charging_enable: OFF DEFERRED (amp CHGEN hold)\n");
			return STATUS_OK;
		}
		/* The stock I2C variant left CHARGE_DIS commented out (the GPIO
		 * carried the disable there); without the GPIO the I2C bit takes
		 * both directions. */
		sm5414_set_chgen(CHARGE_DIS);
		pr_notice(FORGE_CHG_TAG " charging_enable: OFF (I2C CHGEN=0)\n");
	}

	return STATUS_OK;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;
	unsigned int cv_value = *(unsigned int *)(data);
	unsigned int clamp_uv;

	if (!forge_chg_active("set_cv_voltage"))
		return STATUS_OK;

	/* clamp CV at forge_cv_step (default 4.35 V; stock used the 4.4 V
	 * step under HIGH_BATTERY_VOLTAGE_SUPPORT) */
	clamp_uv = VBAT_CV_VTH[forge_cv_step & 0xF];
	if (cv_value >= clamp_uv)
		cv_value = clamp_uv;

	/* stock nearest-value quirk kept: 4.2V request maps to 4.208V cell */
	if (cv_value == BATTERY_VOLT_04_200000_V)
		cv_value = 4208000;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size,
						cv_value);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size,
						     set_cv_voltage);
	sm5414_set_batreg(register_value);

	return STATUS_OK;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int array_size;
	unsigned char reg_value = 0;

	array_size = GETARRAYNUM(CS_VTH);
	sm5414_read_interface(SM5414_CHGCTRL2, &reg_value,
			      SM5414_CHGCTRL2_FASTCHG_MASK,
			      SM5414_CHGCTRL2_FASTCHG_SHIFT);
	*(unsigned int *)data = charging_value_to_parameter(CS_VTH,
							    array_size,
							    reg_value);
	return STATUS_OK;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *)data;

	if (!forge_chg_active("set_current"))
		return STATUS_OK;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
						 current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
						     set_chr_current);
	sm5414_set_fastchg(register_value);

	return STATUS_OK;
}

static unsigned int charging_set_input_current(void *data)
{
	unsigned int current_value = *(unsigned int *)data;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (!forge_chg_active("set_input_current"))
		return STATUS_OK;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size,
						 current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size,
						     set_chr_current);
	sm5414_set_vbuslimit(register_value);

	return STATUS_OK;
}

static unsigned int charging_get_charging_status(void *data)
{
	/* replaces the stock EINT-latched is_fullcharged: bounded INT
	 * poll, once per battery-thread tick */
	int full = (sm5414_poll_charge_done() == 0x1);

	/* See forge_full_min_mv. bat_vol == 0 means the gauge has no sample
	 * yet: do not veto on a reading we do not have. */
	if (full && forge_full_min_mv > 0 && BMT_status.bat_vol > 0 &&
	    BMT_status.bat_vol < (unsigned int)forge_full_min_mv) {
		pr_notice(FORGE_CHG_TAG " full VETOED: bat_vol=%u mV < %d mV (CV step %d) — top-off from a starved input, not a full cell\n",
			  BMT_status.bat_vol, forge_full_min_mv, forge_cv_step);
		full = 0;
	}

	*(unsigned int *)data = full ? KAL_TRUE : KAL_FALSE;

	return STATUS_OK;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	/* SM5414 has no i2c watchdog */
	return STATUS_OK;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *)(data);

	if (!forge_chg_active("set_hv_threshold"))
		return STATUS_OK;

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size,
						voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size,
						     set_hv_voltage);
	/* PMIC write manifest: PMIC_RG_VCDT_HV_VTH -> MT6351 CHR_CON1
	 * (0x0F7A), as in stock */
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);

	return STATUS_OK;
}

static unsigned int charging_get_hv_status(void *data)
{
	/* PMIC_RGS_VCDT_HV_DET -> MT6351 CHR_CON0 (0x0F78), read-only */
	*(kal_bool *)(data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
	return STATUS_OK;
}

static unsigned int charging_get_battery_status(void *data)
{
	unsigned int val;

	/* PMIC write manifest: PMIC_BATON_TDET_EN / PMIC_RG_BATON_EN ->
	 * MT6351 CHR_CON7 (0x0F86); PMIC_RGS_BATON_UNDET -> CHR_CON44
	 * (0x0FD2) read-only, as in stock. */
	/* Hold the AUXADC conversion lock across the whole detect sequence
	 * (see forge_baton_lock). Snapshot the knob so a concurrent sysfs
	 * change cannot unbalance lock/unlock. */
	int locked = forge_baton_lock;

	if (locked)
		pmic_auxadc_lock();
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	/* The two enable writes respect the forge_chg observe gate; observe
	 * mode reports battery present, matching the fail-safe rule in the
	 * file header. */
	if (val && forge_chg_active("baton_detect")) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *)(data) =
			pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *)(data) = KAL_FALSE;
	}
	if (locked)
		pmic_auxadc_unlock();
	{
		/* per-call verdict (capped) */
		static int forge_baton_hw_logn;

		if (forge_baton_hw_logn < 30) {
			forge_baton_hw_logn++;
			pr_emerg(FORGE_CHG_TAG " baton: TDET_EN(pre)=%u -> absent=%u\n",
				 val, *(kal_bool *)(data));
		}
	}

	return STATUS_OK;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	/* PMIC_RGS_CHRDET -> MT6351 CHR_CON0 (0x0F78), read-only. This is
	 * the real VBUS presence bit; the pmic irq 39 handler drives
	 * do_chrdet_int_task() off its edges. */
	*(kal_bool *)(data) = pmic_get_register_value(PMIC_RGS_CHRDET);
	return STATUS_OK;
}

kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

static unsigned int charging_get_charger_type(void *data)
{
	/* BC1.1 walk lives in the in-tree pmic_chr_type_det.c */
	/* Markers around the walk, so a hang inside is located. Rare (plug
	 * events). */
	pr_emerg(FORGE_CHG_TAG " BC1.1 charger-type walk: enter\n");
	*(CHARGER_TYPE *)(data) = hw_charging_get_charger_type();
	pr_emerg(FORGE_CHG_TAG " BC1.1 charger-type walk: exit type=%d\n",
		 *(CHARGER_TYPE *)(data));
	return STATUS_OK;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *)(data) = KAL_TRUE;
	else
		*(kal_bool *)(data) = KAL_FALSE;

	return STATUS_OK;
}

static unsigned int charging_set_platform_reset(void *data)
{
	if (!forge_chg_poweroff) {
		pr_emerg(FORGE_CHG_TAG " REFUSED platform_reset (forge_chg_poweroff=0) TDET=%u BATON_EN=%u UNDET=%u CHRDET=%u\n",
			 pmic_get_register_value(PMIC_BATON_TDET_EN),
			 pmic_get_register_value(PMIC_RG_BATON_EN),
			 pmic_get_register_value(PMIC_RGS_BATON_UNDET),
			 pmic_get_register_value(PMIC_RGS_CHRDET));
		return STATUS_OK;
	}
	pr_notice(FORGE_CHG_TAG " charging_set_platform_reset\n");
	kernel_restart("battery service reboot system");
	return STATUS_OK;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	*(unsigned int *)(data) = get_boot_mode();
	return STATUS_OK;
}

static unsigned int charging_set_power_off(void *data)
{
	if (!forge_chg_poweroff) {
		pr_emerg(FORGE_CHG_TAG " REFUSED power_off (forge_chg_poweroff=0) TDET=%u BATON_EN=%u UNDET=%u CHRDET=%u\n",
			 pmic_get_register_value(PMIC_BATON_TDET_EN),
			 pmic_get_register_value(PMIC_RG_BATON_EN),
			 pmic_get_register_value(PMIC_RGS_BATON_UNDET),
			 pmic_get_register_value(PMIC_RGS_CHRDET));
		return STATUS_OK;
	}
	pr_notice(FORGE_CHG_TAG " charging_set_power_off\n");
	kernel_power_off();
	return STATUS_OK;
}

static unsigned int charging_get_power_source(void *data)
{
	*(kal_bool *)data = KAL_FALSE;
	return STATUS_OK;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	/* Pump Express is off on M681 (stock: PE/PE+ not set) */
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_error_state(void *data)
{
	charging_error = *(unsigned int *)(data);
	pr_notice(FORGE_CHG_TAG " error_state=%u\n", charging_error);
	return STATUS_OK;
}

static unsigned int charging_diso_init(void *data)
{
	/* no dual-input hardware on M681 */
	return STATUS_OK;
}

static unsigned int charging_get_diso_state(void *data)
{
	return STATUS_OK;
}

static unsigned int charging_enable_otg(void *data)
{
	unsigned int enable = *(unsigned int *)(data);

	if (!forge_chg_active("enable_otg"))
		return STATUS_OK;

	if (enable == KAL_TRUE) {
		/* charging off before boost on (stock sm5414_otg_enable) */
		sm5414_set_chgen(CHARGE_DIS);
		sm5414_set_enboost(ENBOOST_EN);
	} else {
		sm5414_set_enboost(ENBOOST_DIS);
		/* fail towards charging enabled after OTG ends */
		sm5414_set_chgen(CHARGE_EN);
	}
	pr_notice(FORGE_CHG_TAG " enable_otg=%d\n", enable);

	return STATUS_OK;
}

static unsigned int charging_get_input_current(void *data)
{
	unsigned int array_size;
	unsigned char reg_value = 0;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	sm5414_read_interface(SM5414_VBUSCTRL, &reg_value,
			      SM5414_VBUSCTRL_VBUSLIMIT_MASK,
			      SM5414_VBUSCTRL_VBUSLIMIT_SHIFT);
	*(unsigned int *)data = charging_value_to_parameter(INPUT_CS_VTH,
							    array_size,
							    reg_value);
	return STATUS_OK;
}

static unsigned int charging_not_supported(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int (*const charging_func[CHARGING_CMD_NUMBER])(void *data) = {
	[CHARGING_CMD_INIT] = charging_hw_init,
	[CHARGING_CMD_DUMP_REGISTER] = charging_dump_register,
	[CHARGING_CMD_ENABLE] = charging_enable,
	[CHARGING_CMD_SET_CV_VOLTAGE] = charging_set_cv_voltage,
	[CHARGING_CMD_GET_CURRENT] = charging_get_current,
	[CHARGING_CMD_SET_CURRENT] = charging_set_current,
	[CHARGING_CMD_SET_INPUT_CURRENT] = charging_set_input_current,
	[CHARGING_CMD_GET_CHARGING_STATUS] = charging_get_charging_status,
	[CHARGING_CMD_RESET_WATCH_DOG_TIMER] = charging_reset_watch_dog_timer,
	[CHARGING_CMD_SET_HV_THRESHOLD] = charging_set_hv_threshold,
	[CHARGING_CMD_GET_HV_STATUS] = charging_get_hv_status,
	[CHARGING_CMD_GET_BATTERY_STATUS] = charging_get_battery_status,
	[CHARGING_CMD_GET_CHARGER_DET_STATUS] = charging_get_charger_det_status,
	[CHARGING_CMD_GET_CHARGER_TYPE] = charging_get_charger_type,
	[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER] = charging_get_is_pcm_timer_trigger,
	[CHARGING_CMD_SET_PLATFORM_RESET] = charging_set_platform_reset,
	[CHARGING_CMD_GET_PLATFORM_BOOT_MODE] = charging_get_platform_boot_mode,
	[CHARGING_CMD_SET_POWER_OFF] = charging_set_power_off,
	[CHARGING_CMD_GET_POWER_SOURCE] = charging_get_power_source,
	[CHARGING_CMD_GET_CSDAC_FALL_FLAG] = charging_get_csdac_full_flag,
	[CHARGING_CMD_SET_TA_CURRENT_PATTERN] = charging_set_ta_current_pattern,
	[CHARGING_CMD_SET_ERROR_STATE] = charging_set_error_state,
	[CHARGING_CMD_DISO_INIT] = charging_diso_init,
	[CHARGING_CMD_GET_DISO_STATE] = charging_get_diso_state,
	[CHARGING_CMD_ENABLE_OTG] = charging_enable_otg,
	[CHARGING_CMD_GET_INPUT_CURRENT] = charging_get_input_current,
};

int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	int status;

	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL) {
			status = charging_func[cmd](data);
		} else {
			pr_notice(FORGE_CHG_TAG " chr_control_interface: cmd:%d not supported\n",
				  cmd);
			status = charging_not_supported(data);
		}
	} else {
		return STATUS_UNSUPPORTED;
	}

	return status;
}
