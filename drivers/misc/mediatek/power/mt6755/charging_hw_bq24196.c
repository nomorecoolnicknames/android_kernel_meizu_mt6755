/*
 * Runtime gates (as in the SM5414 HAL, same semantics):
 *   charging_hw_bq24196.forge_chg=1 (default) full charger control, 0 =
 *     observe only (writes skipped and logged; BQ24196 then runs its
 *     default mode once its 40 s I2C watchdog expires, i.e. keeps
 *     charging);
 *   charging_hw_bq24196.forge_chg_poweroff=0 (default) log and refuse
 *     kernel power-off/restart requests from the battery stack;
 *   charging_hw_bq24196.forge_cv_step, forge_full_min_mv - see below.
 * Fail-safe rule: no error path ends with charging disabled.
 */

/*
 * L681 (hq6755_66_b1a_l) BQ24196 charger HAL for the 4.4 GM20 stack.
 * Framework = charging_hw_sm5414.c (PMIC detect/BATON/power-off gates,
 * forge_chg / forge_chg_poweroff knobs that battery_common_fg_20.c and
 * battery_meter_fg_20.c reference); chip logic = the 3.10 L681
 * charging_hw_bq24196.c (register values and tables verbatim). No GPIO is
 * driven here: on L681 GPIO100 is the BQ24196 CE pin
 * (GPIO_SWCHARGER_EN_PIN, DCT output low = charge enabled) and is left as
 * the bootloader set it; charge gating is the I2C CHG_CONFIG bit.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#ifdef CONFIG_M3NOTE_UNIFIED
/* M681/L681 kernel: charging_hw_m3note.c owns the public names and calls
 * this HAL only on L681. */
#define chr_control_interface chr_control_interface_bq24196
#define charging_type_detection_done charging_type_detection_done_bq24196
#define charging_type_det_done charging_type_det_done_bq24196
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

#include "bq24196.h"

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (ARRAY_SIZE(array))

#define FORGE_CHG_TAG "[FORGE_CHG][HW]"

/* ============================================================ // */
/* Runtime gate */
/* ============================================================ // */
static int forge_chg = 1;
module_param(forge_chg, int, 0644);
MODULE_PARM_DESC(forge_chg,
	"l681 charger stack gate: 1=full BQ24196 control (default), 0=observe-only (leave HW-default charging untouched)");

/* BQ24196 REG04 VREG index for the CV clamp: 3.504 V + 16 mV/step.
 * 53 = 4.352 V (default; the L681 cell is a 4.35 V one,
 * HIGH_BATTERY_VOLTAGE_SUPPORT in the 3.10 cust_charging.h), 56 = 4.400 V
 * (what the 3.10 HAL programs). With 53, REG04 reads 0xd2 = VREG 4.336 V
 * (the HAL rounds down), and the chip terminates there. */
static int forge_cv_step = 53;
module_param(forge_cv_step, int, 0644);
MODULE_PARM_DESC(forge_cv_step,
	"l681: BQ24196 REG04 VREG step for the CV clamp (3.504V + 16mV*step; default 53)");

/* Stock CV: the 3.10 HAL writes VREG 4.400 V at init and turns any request
 * of 4.3 V or more into 4.4 V; here the request is MTK_CV_VOLTAGE 4.35 V,
 * which the clamp alone would round down to 4.336 V. 1 = both as stock
 * (VREG step 56), whatever forge_cv_step says; 0 = forge_cv_step
 * (default). */
#define FORGE_CV_STOCK_STEP	56	/* 3.504 V + 56 * 16 mV = 4.400 V */
static int forge_cv_stock;
module_param(forge_cv_stock, int, 0644);
MODULE_PARM_DESC(forge_cv_stock,
	"l681: 1 = CV 4.400 V as the 3.10 HAL, 0 = forge_cv_step (default)");

static unsigned int forge_cv_vreg_step(void)
{
	return forge_cv_stock ? FORGE_CV_STOCK_STEP : forge_cv_step;
}

/* Meizu's current zones of the 3.10 charging_set_current, wall charger
 * only: with the panel lit and the battery at 10 C or more a 1750 mA
 * request charges at 1050 mA (1024 by the table); at 0-9 C at 832 mA. Here
 * the cold zone only lowers a request (stock also raised a smaller one to
 * 832). Follows battery_common_fg_20.forge_l681_stock_chg (1, default); the
 * wall charger is only seen with forge_bc11_skip=0. */
extern int forge_l681_stock_chg;
extern int forge_lcm_on;

/* The panel-lit cut (1750 -> 1050 mA) is off by default; 1 brings the
 * stock behaviour back. The 0-9 C zone does not depend on it. */
static int forge_l681_screen_cut;
module_param(forge_l681_screen_cut, int, 0644);
MODULE_PARM_DESC(forge_l681_screen_cut,
	"l681: stock 1050 mA on a wall charger while the panel is lit (0 = off, default)");

static unsigned int forge_l681_zone_current(unsigned int current_value)
{
	int t = BMT_status.temperature;

	if (!forge_l681_stock_chg || BMT_status.charger_type != STANDARD_CHARGER)
		return current_value;
	if (forge_l681_screen_cut && current_value == CHARGE_CURRENT_1750_00_MA &&
	    forge_lcm_on && t >= 10)
		current_value = CHARGE_CURRENT_1050_00_MA;
	if (t >= 0 && t < 10 && current_value > 83200)
		current_value = 83200;
	return current_value;
}

/* Kernel-initiated power-off/reset gate, as in the SM5414 HAL (there the
 * GM20 battery check -> CHARGING_CMD_SET_POWER_OFF -> kernel_power_off shut
 * the board down ~3 s into boot). Default 0 = log the request (with
 * BATON/CHRDET state) and refuse; 1 = stock. Non-static:
 * battery_meter_fg_20.c (FG_DAEMON_CMD_SET_POWEROFF from the gauge daemon)
 * and battery_common_fg_20.c (DLPT low-SOC kernel_restart) link against it
 * too. An empty cell still gets Android's orderly shutdown: below
 * forge_soc0_min_mv (3450 mV) a capacity 0 reaches BatteryService, which
 * powers off at 0 % unplugged. */
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

/* CV-proximity check on the charger IC's "full" verdict (as in the SM5414
 * HAL). A charge-termination/top-off edge only means "charge current fell
 * below the termination threshold"; it equals "cell full" only while the
 * chip regulates at CV. forge_bc11_skip pins the charger type to
 * STANDARD_HOST (500 mA input), so a system drawing more than that starves
 * the cell and the IC can report termination at any voltage. BQ24196
 * reports it through REG08 CHRG_STAT=11; the veto applies unchanged.
 * Default 4250 = CV 4350 - 100 mV; 0 disables the check. */
static int forge_full_min_mv = 4250;
module_param(forge_full_min_mv, int, 0644);
MODULE_PARM_DESC(forge_full_min_mv,
	"l681 chg-full: reject a BQ24196 full verdict below this pack voltage in mV (default 4250); 0 disables");

static inline int forge_chg_active(const char *what)
{
	if (forge_chg)
		return 1;
	pr_notice(FORGE_CHG_TAG " gate OFF: skip %s\n", what);
	return 0;
}

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */
kal_bool charging_type_det_done = KAL_TRUE;

/* REG04 VREG: 3.504V + 16mV/step (uV), 3.10 L681 table */
static const unsigned int VBAT_CV_VTH[] = {
	3504000, 3520000, 3536000, 3552000,
	3568000, 3584000, 3600000, 3616000,
	3632000, 3648000, 3664000, 3680000,
	3696000, 3712000, 3728000, 3744000,
	3760000, 3776000, 3792000, 3808000,
	3824000, 3840000, 3856000, 3872000,
	3888000, 3904000, 3920000, 3936000,
	3952000, 3968000, 3984000, 4000000,
	4016000, 4032000, 4048000, 4064000,
	4080000, 4096000, 4112000, 4128000,
	4144000, 4160000, 4176000, 4192000,
	4208000, 4224000, 4240000, 4256000,
	4272000, 4288000, 4304000, 4320000,
	4336000, 4352000, 4368000, 4384000,
	4400000
};

/* REG02 ICHG: 512mA + 64mA/step, unit 0.01mA, 3.10 L681 table */
static const unsigned int CS_VTH[] = {
	51200, 57600, 64000, 70400,
	76800, 83200, 89600, 96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
};

/* REG00 IINLIM, 3.10 L681 table */
static const unsigned int INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_150_00_MA, CHARGE_CURRENT_500_00_MA,
	CHARGE_CURRENT_900_00_MA, CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1500_00_MA,
	CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_MAX
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

	/* l681 3.10 charging_hw_bq24196.c charging_hw_init, values verbatim
	 * except VREG, which follows the forge_cv_step clamp */
	bq24196_set_en_hiz(0x0);
	bq24196_set_vindpm(0x7);	/* VIN DPM 4.68V */
	bq24196_set_reg_rst(0x0);
	bq24196_set_wdt_rst(0x1);	/* kick watchdog */
	bq24196_set_sys_min(0x4);	/* minimum system voltage 3.5V */
	bq24196_set_iprechg(0x3);	/* precharge current 512mA */
	bq24196_set_iterm(0x0);		/* termination current 128mA */
	bq24196_set_vreg(forge_cv_vreg_step() & 0x3F);
	bq24196_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq24196_set_vrechg(0x0);	/* VRECHG 0.1V */
	bq24196_set_en_term(0x1);	/* enable termination */
	bq24196_set_term_stat(0x0);	/* match ITERM */
	bq24196_set_watchdog(0x1);	/* WDT 40s */
	bq24196_set_en_timer(0x0);	/* charge safety timer off (3.10 value) */
	bq24196_set_int_mask(0x0);	/* no fault interrupt */
	return STATUS_OK;
}

static unsigned int charging_dump_register(void *data)
{
	bq24196_dump_register();
	return STATUS_OK;
}

static unsigned int charging_enable(void *data)
{
	unsigned int enable = *(unsigned int *)(data);

	if (enable == KAL_TRUE) {
		/* enabling charge is always allowed, gate or not: fail
		 * towards charging on */
		bq24196_set_en_hiz(0x0);
		bq24196_set_chg_config(0x1);
		pr_notice(FORGE_CHG_TAG " charging_enable: ON (CHG_CONFIG=1)\n");
	} else {
		if (!forge_chg_active("charging_enable(OFF)"))
			return STATUS_OK;
		bq24196_set_chg_config(0x0);
		/* 3.10: HIZ (power path off) only on a charging error */
		if (charging_error)
			bq24196_set_en_hiz(0x1);
		pr_notice(FORGE_CHG_TAG " charging_enable: OFF (CHG_CONFIG=0, error=%u)\n",
			  charging_error);
	}
	return STATUS_OK;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int array_size = GETARRAYNUM(VBAT_CV_VTH);
	unsigned int set_cv_voltage;
	unsigned int register_value;
	unsigned int cv_value = *(unsigned int *)(data);
	unsigned int clamp_uv;

	if (!forge_chg_active("set_cv_voltage"))
		return STATUS_OK;

	clamp_uv = VBAT_CV_VTH[min_t(unsigned int, forge_cv_vreg_step(), array_size - 1)];
	/* 3.10: a request of 4.3 V and more is 4.4 V */
	if (forge_cv_stock && cv_value >= 4300000)
		cv_value = 4400000;
	if (cv_value >= clamp_uv)
		cv_value = clamp_uv;
	/* 3.10 nearest-value quirk kept: a 4.2V request maps to 4.208V */
	if (cv_value == BATTERY_VOLT_04_200000_V)
		cv_value = 4208000;

	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
	bq24196_set_vreg(register_value);
	return STATUS_OK;
}

static unsigned int charging_get_current(void *data)
{
	unsigned char ichg = 0, force_20pct = 0;
	unsigned int ma;

	/* l681 3.10: REG02 ICHG * 64 + 512 mA, FORCE_20PCT scales by 0.2 */
	bq24196_read_interface(bq24196_CON2, &ichg, CON2_ICHG_MASK, CON2_ICHG_SHIFT);
	bq24196_read_interface(bq24196_CON2, &force_20pct, CON2_FORCE_20PCT_MASK,
			       CON2_FORCE_20PCT_SHIFT);
	ma = ichg * 64 + 512;
	*(unsigned int *)data = force_20pct ? (ma * 2) / 10 : ma;
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

	current_value = forge_l681_zone_current(current_value);
	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size,
						 current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size,
						     set_chr_current);
	bq24196_set_ichg(register_value);

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
	bq24196_set_iinlim(register_value);

	return STATUS_OK;
}

static unsigned int charging_get_charging_status(void *data)
{
	/* l681 3.10: CHRG_STAT == 3 (charge termination done) = full */
	int full = (bq24196_get_chrg_stat() == 0x3);

	/* full-voltage sanity check (see forge_full_min_mv) */
	if (full && forge_full_min_mv > 0 && BMT_status.bat_vol > 0 &&
	    BMT_status.bat_vol < (unsigned int)forge_full_min_mv) {
		pr_notice(FORGE_CHG_TAG " full VETOED: bat_vol=%u mV < %d mV\n",
			  BMT_status.bat_vol, forge_full_min_mv);
		full = 0;
	}
	*(unsigned int *)data = full ? KAL_TRUE : KAL_FALSE;
	return STATUS_OK;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	bq24196_set_wdt_rst(0x1);	/* kick the 40s I2C watchdog */
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
	 * (0x0F7A), stock charging_hw_sm5414.c */
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
	 * (0x0FD2) read-only, as in stock charging_hw_sm5414.c. */
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
	/* Pump Express is off (m681 and l681) */
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
	/* no dual-input hardware on m681 / l681 */
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

	/* BQ24196 REG01: OTG_CONFIG boosts VBUS, CHG_CONFIG must be off meanwhile */
	if (enable == KAL_TRUE) {
		bq24196_set_chg_config(0x0);
		bq24196_set_otg_config(0x1);
	} else {
		bq24196_set_otg_config(0x0);
		bq24196_set_chg_config(0x1);	/* fail towards charging enabled */
	}
	pr_notice(FORGE_CHG_TAG " enable_otg=%d\n", enable);
	return STATUS_OK;
}

static unsigned int charging_get_input_current(void *data)
{
	unsigned char reg_value = 0;

	bq24196_read_interface(bq24196_CON0, &reg_value, CON0_IINLIM_MASK, CON0_IINLIM_SHIFT);
	*(unsigned int *)data = charging_value_to_parameter(INPUT_CS_VTH,
							    GETARRAYNUM(INPUT_CS_VTH), reg_value);
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
