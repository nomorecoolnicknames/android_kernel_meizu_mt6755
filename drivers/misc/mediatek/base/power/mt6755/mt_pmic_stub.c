/*
 * MT6755 PMIC access without the PMIC driver.
 *
 * Reads go to the hardware through pwrap (pmic_read_interface,
 * pmic_get_register_value via the MT6351 pmu_flags_table). Writes are
 * dropped and counted (dropped_writes / last_dropped) unless an allowlist
 * row below owns the field. The remaining APIs
 * (pmic_config_interface_nospinlock, pmic_force_vcore_pwm,
 * upmu_is_chr_det, is_mt6311_exist, ...) are stubs for callers in
 * subsystems that are not built.
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/ratelimit.h>
#include <linux/of.h>
#include <linux/m3note_board.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/upmu_common.h>

#ifdef CONFIG_MTK_PMIC_WRAP_HAL
/*
 * READ-ONLY PMIC access. With the pwrap chip HAL built
 * (pmic_wrap/mt6755/pwrap_hal_v1.c, WACS2 on the LK-initialised wrapper),
 * reads are real: pmic_read_interface[_nolock] go through pwrap_wacs2(), and
 * pmic_get_register_value[_nolock] index the MT6351 pmu_flags_table
 * (mt_pmic_flags_mt6351.c, 3.10 stock). Writes stay no-ops here (counted;
 * the first one is logged with its caller) and the HAL drops any direct
 * pwrap_write too, so no PMIC register changes until kernel-turns
 * writes on after a hardware check - so far only the touch rail enable
 * (forge_pmic_touch_write() below). Battery requests and own AUXADC clock
 * preparation below use separate default-off, task-owned permissions. Telemetry:
 * /sys/module/mt_pmic_stub/parameters/{dropped_writes,last_dropped}.
 */
extern s32 pwrap_wacs2(u32 write, u32 adr, u32 wdata, u32 *rdata);
extern const PMU_FLAG_TABLE_ENTRY pmu_flags_table[];

static unsigned int forge_pmic_dropped_writes;
module_param_named(dropped_writes, forge_pmic_dropped_writes, uint, 0444);
static unsigned int forge_pmic_last_dropped;	/* reg << 16 | (val & 0xffff) */
module_param_named(last_dropped, forge_pmic_last_dropped, uint, 0444);

static void forge_pmic_drop(const char *api, unsigned int reg, unsigned int val,
			    void *caller)
{
	forge_pmic_dropped_writes++;
	forge_pmic_last_dropped = (reg << 16) | (val & 0xffff);
	if (forge_pmic_dropped_writes == 1)
		pr_warn("[FORGE_M681] PMIC write dropped (read-only PMIC on 4.9): %s reg/flag=0x%x val=0x%x from %pS\n",
			api, reg, val, caller);
}

static unsigned int forge_pmic_read(unsigned int RegNum, unsigned int *val,
				    unsigned int MASK, unsigned int SHIFT)
{
	u32 rdata = 0;
	unsigned int ret = pwrap_wacs2(0, RegNum, 0, &rdata);

	if (!val)
		return ret;
	/* on a failed read report 0, as the no-op stub did before */
	*val = ret ? 0 : ((rdata & (MASK << SHIFT)) >> SHIFT);
	return ret;
}

static const PMU_FLAG_TABLE_ENTRY *forge_pmic_flag(PMU_FLAGS_LIST_ENUM flagname)
{
	const PMU_FLAG_TABLE_ENTRY *f;

	if ((unsigned int)flagname >= (unsigned int)PMU_COMMAND_MAX)
		return NULL;
	f = &pmu_flags_table[flagname];
	if (f->flagname != flagname) {
		pr_warn_once("[FORGE_M681] pmu_flags_table[%d] holds flag %d - read refused\n",
			     (int)flagname, (int)f->flagname);
		return NULL;
	}
	return f;
}

unsigned int pmic_read_interface(unsigned int RegNum, unsigned int *val,
				 unsigned int MASK, unsigned int SHIFT)
{
	return forge_pmic_read(RegNum, val, MASK, SHIFT);
}
EXPORT_SYMBOL(pmic_read_interface);

unsigned int pmic_read_interface_nolock(unsigned int RegNum, unsigned int *val,
					unsigned int MASK, unsigned int SHIFT)
{
	/* pwrap_wacs2() serialises on its own spinlock */
	return forge_pmic_read(RegNum, val, MASK, SHIFT);
}
EXPORT_SYMBOL(pmic_read_interface_nolock);

unsigned short pmic_get_register_value(PMU_FLAGS_LIST_ENUM flagname)
{
	const PMU_FLAG_TABLE_ENTRY *f = forge_pmic_flag(flagname);
	unsigned int val = 0;

	if (f)
		forge_pmic_read(f->offset, &val, f->mask, f->shift);
	return val;
}
EXPORT_SYMBOL(pmic_get_register_value);

unsigned short pmic_get_register_value_nolock(PMU_FLAGS_LIST_ENUM flagname)
{
	return pmic_get_register_value(flagname);
}
EXPORT_SYMBOL(pmic_get_register_value_nolock);

/*
 * The touch rail write is let through: otherwise VLDO28 stays off as LK
 * left it and GT9XX never answers at 0x5d. These are the writes the
 * GT9XX_MZ driver uses (tpd_custom_gt9xx.h: pmic_config_interface(0x0AA2 /
 * 0x0AA4, 1, 0x1, 1) = MT6351 RG_VLDO28_EN_0/_1), at the default VLDO28
 * voltage. Only bit 1 of those two registers, as a read-modify-write; the
 * rail status (DA_QI_VLDO28_EN, CON0 bit 15) is logged whenever it or the
 * register changes. touch_rail=0 drops them again.
 */
#define FORGE_VLDO28_CON0	0x0A9C
static int forge_pmic_touch_rail = 1;
module_param_named(touch_rail, forge_pmic_touch_rail, int, 0644);

static bool forge_pmic_touch_write(unsigned int reg, unsigned int val,
				   unsigned int mask, unsigned int shift)
{
	static u32 last_st = ~0U;
	u32 old = 0, new, st0 = 0, st1 = 0;
	s32 rd, wr;

	if (!forge_pmic_touch_rail || (reg != 0x0AA2 && reg != 0x0AA4) ||
	    mask != 0x1 || shift != 1)
		return false;
	rd = pwrap_wacs2(0, reg, 0, &old);
	pwrap_wacs2(0, FORGE_VLDO28_CON0, 0, &st0);
	new = (old & ~(mask << shift)) | ((val & mask) << shift);
	wr = rd ? rd : pwrap_wacs2(1, reg, new, NULL);
	pwrap_wacs2(0, FORGE_VLDO28_CON0, 0, &st1);
	if (rd || wr || old != new || ((st1 >> 15) & 1) != last_st)
		pr_info("[FORGE_M681] VLDO28 touch rail: reg 0x%04x 0x%04x -> 0x%04x (rd %d wr %d), DA_QI_VLDO28_EN %u -> %u\n",
			reg, old, new, rd, wr, (st0 >> 15) & 1, (st1 >> 15) & 1);
	last_st = (st1 >> 15) & 1;
	return true;
}

/* MT6351 AUXADC_RQST0_SET: stock requests CH1/CH3 by W1, not RMW.
 * On by default: LK cuts the boot.img cmdline at ~99 bytes, so a boot
 * parameter never arrives. */
static bool forge_battery_adc = true;
module_param_named(battery_adc, forge_battery_adc, bool, 0444);
static DEFINE_MUTEX(forge_adc_owner_lock);
static struct task_struct *forge_adc_owner;
static u32 forge_adc_request;
static u32 forge_adc_request_addr;
static bool forge_adc_request_armed;
/* Separate gate: the ADC request gate does not grant clock writes; on by
 * default for the same reason. */
static bool forge_battery_adc_clocks = true;
module_param_named(battery_adc_clocks, forge_battery_adc_clocks, bool, 0444);

#define FORGE_ADC_AON_REG 0x0EA2
#define FORGE_ADC_AON_MASK BIT(15)
#define FORGE_ADC_SMPS_REG 0x023A
#define FORGE_ADC_SMPS_MASK BIT(9)

/* Rail setpoints the DVFS owners may write, through the same owned-token
 * route as the ADC request: only VOSEL_ON [6:0] of a listed register, only
 * a code inside its row's range, only while the row's switch is on.
 *  - VSRAM_PROC 0x06AA: CPU DVFS (mt_cpufreq via mt_extbuck_probe). Codes
 *    0x35..0x58 = 0.931..1.150 V, the stock clamp MIN_VSRAM_VOLT ..
 *    MAX_VSRAM_VOLT. Switch forge_vstep_allow: compiled in by
 *    FORGE_M681_VSTEP_ALLOW (LK cuts the cmdline), refuses every set.
 *  - VGPU 0x061E: GPU DVFS. Codes 0x35..0x54 = 0.931..1.125 V, the span of
 *    the stock GPU tables e1_0/e1_1/e1_t (0x30 only exists in e2_x).
 *    Switch forge_pmic_vgpu, run-time, default 0.
 *  - VGPU 0x061C (VOSEL): the same rail in SW control. VGPU runs in SW
 *    control here (LK leaves BUCK_VGPU_CON0 VOSEL_CTRL = 0 and the stock
 *    switch to HW is not done), so VOSEL is the live setpoint and VOSEL_ON
 *    changes nothing. Same codes, same switch.
 *  - VCN18/VCN28/VCN33 EN and ON_CTRL: connsys, single-bit rows (below).
 *  - VCAMA/VCAMD/VCAMIO/VGP3 EN and VOSEL: camera, VOSEL at the stock code only.
 *  - VIBR EN and VOSEL: vibrator, VOSEL at the stock code only.
 * Rows are fields: a register may have more than one. */
#ifndef FORGE_M681_VSTEP_ALLOW
#define FORGE_M681_VSTEP_ALLOW 1	/* the Sh-stock line; 0 elsewhere */
#endif
static bool forge_vstep_allow = FORGE_M681_VSTEP_ALLOW;

/* visible, but not settable even from the kernel command line */
static int forge_vstep_allow_set(const char *val, const struct kernel_param *kp)
{
	return -EPERM;
}

static const struct kernel_param_ops forge_vstep_allow_ops = {
	.set = forge_vstep_allow_set,
	.get = param_get_bool,
};
module_param_cb(forge_vstep_allow, &forge_vstep_allow_ops, &forge_vstep_allow, 0444);

static bool forge_pmic_vgpu;
module_param(forge_pmic_vgpu, bool, 0644);

#define FORGE_VSRAM_ON_REG 0x06AA
#define FORGE_VGPU_ON_REG 0x061E
#define FORGE_VGPU_SW_REG 0x061C
#define FORGE_VOSEL_ON_MASK 0x7F

/* connsys rails: EN bit 1 and ON_CTRL bit 3 of VCN18, VCN28 and the
 * VCN33 BT/WIFI enables, both values. The consys driver
 * (conn_soc/mt6755/mtk_wcn_consys_hw.c) checks VOSEL before it sets EN and
 * reaches these through pmic_config_interface(). Run-time switch, on by
 * default. Also on the l681 board: the same MT6351 E2 (HWCID 0x5140 /
 * SWCID 0x5120 on both boards), the same fields (mt6351 upmu_hw.h
 * LDO_VCN18_CON0 0x0A52, LDO_VCN28_CON0 0x0A0C, LDO_VCN33_CON3/4
 * 0x0A98/0x0A9A; EN bit 1, ON_CTRL bit 3). */
static bool forge_pmic_conn = true;
module_param_named(allow_conn, forge_pmic_conn, bool, 0644);

/* camera rails (stock kd_camera_hw.c / power/mt6755/pmic.c): EN bit 1 of
 * LDO_<x>_CON0 either way, VOSEL [10:8] of <x>_ANA_CON0 only at the stock
 * code - VCAMA 3 = 2.8 V, VCAMD 5 = 1.2 V, VCAMIO 6 = 1.8 V, and VGP3 3 =
 * 1.22 V (the front sensors' DVDD, own switch). kd_sensorlist.c writes them
 * through pmic_set_register_value() once kd_camera_hw.forge_cam_power is
 * armed. VLDO28 (AFVDD, the touch rail) is never a camera row. Also on the
 * l681 board: the same MT6351 E2, and the l681 DCT maps the same LDOs
 * (main A/D/IO = VCAMA/VCAMD/VCAMIO, sub DVDD = VGP3); its stock asks DVDD
 * 1.2 V of VGP3, whose MT6351 table has no 1.20, so 1.22 (code 3) is used
 * as on m681. */
static bool forge_pmic_cam = true;
module_param_named(allow_cam, forge_pmic_cam, bool, 0644);
static bool forge_pmic_cam_front = true;
module_param_named(allow_cam_front, forge_pmic_cam_front, bool, 0644);

/* vibrator LDO (vibrator/mt6755/m681_vibrator.c): RG_VIBR_EN =
 * LDO_VIBR_CON0 0x0A3A bit 1, RG_VIBR_VOSEL = VIBR_ANA_CON0 0x0AE6 [10:8]
 * only at the stock code 5 = 2.8 V (DT vib_vol = <5> on both boards).
 * Stock writes these two fields only. The off write (EN 0) passes with the
 * switch off too: turning allow_vibr off must never leave the motor
 * running. */
static bool forge_pmic_vibr = true;
module_param_named(allow_vibr, forge_pmic_vibr, bool, 0644);

struct forge_pmic_wl {
	u16 reg;
	u8 mask, shift;		/* the field */
	u8 lo, hi;		/* allowed field values */
	bool *allow;
	const char *name;
	bool api;		/* writable through pmic_config_interface() */
	bool l681;		/* also on the l681 board */
	bool off_ok;		/* value 0 passes with the switch off */
};

/* VSRAM_PROC VOSEL_CTRL (0x06A0 b1): the variant-A switch to the VOSEL_ON
 * register as stock PMIC_INIT_SETTING_V1 does (pmic_initial_setting.c:322),
 * set by the CPU DVFS only after VOSEL_ON equals VOSEL (no jump) */
#define FORGE_VSRAM_CON0_REG 0x06A0
/* VSRAM_PROC VOSEL_SLEEP (0x06AC) follows VOSEL_ON in that mode, so a sleep
 * request never moves the rail (stock's lower sleep code is a later step) */
#define FORGE_VSRAM_SLEEP_REG 0x06AC
static const struct forge_pmic_wl forge_pmic_wl[] = {
	{ FORGE_VSRAM_ON_REG, 0x7F, 0, 0x35, 0x58, &forge_vstep_allow, "VSRAM_PROC", false, true },
	{ FORGE_VSRAM_SLEEP_REG, 0x7F, 0, 0x35, 0x58, &forge_vstep_allow, "VSRAM_PROC VOSEL_SLEEP", false, true },
	{ FORGE_VSRAM_CON0_REG, 0x1, 1, 0, 1, &forge_vstep_allow, "VSRAM_PROC VOSEL_CTRL", false, true },
	{ FORGE_VGPU_ON_REG, 0x7F, 0, 0x35, 0x54, &forge_pmic_vgpu, "VGPU", true },
	{ FORGE_VGPU_SW_REG, 0x7F, 0, 0x35, 0x54, &forge_pmic_vgpu, "VGPU_SW", true },
	{ 0x0A52, 0x1, 1, 0, 1, &forge_pmic_conn, "LDO_VCN18_CON0 EN", true, true },
	{ 0x0A52, 0x1, 3, 0, 1, &forge_pmic_conn, "LDO_VCN18_CON0 ON_CTRL", true, true },
	{ 0x0A0C, 0x1, 1, 0, 1, &forge_pmic_conn, "LDO_VCN28_CON0 EN", true, true },
	{ 0x0A0C, 0x1, 3, 0, 1, &forge_pmic_conn, "LDO_VCN28_CON0 ON_CTRL", true, true },
	{ 0x0A98, 0x1, 1, 0, 1, &forge_pmic_conn, "LDO_VCN33_CON3 EN (BT)", true, true },
	{ 0x0A98, 0x1, 3, 0, 1, &forge_pmic_conn, "LDO_VCN33_CON3 ON_CTRL (BT)", true, true },
	{ 0x0A9A, 0x1, 1, 0, 1, &forge_pmic_conn, "LDO_VCN33_CON4 EN (WIFI)", true, true },
	{ 0x0A9A, 0x1, 3, 0, 1, &forge_pmic_conn, "LDO_VCN33_CON4 ON_CTRL (WIFI)", true, true },
	{ 0x0A12, 0x1, 1, 0, 1, &forge_pmic_cam, "LDO_VCAMA_CON0 EN", true, true },
	{ 0x0AD6, 0x7, 8, 3, 3, &forge_pmic_cam, "VCAMA_ANA_CON0 VOSEL (2.8 V)", true, true },
	{ 0x0A40, 0x1, 1, 0, 1, &forge_pmic_cam, "LDO_VCAMD_CON0 EN", true, true },
	{ 0x0B00, 0x7, 8, 5, 5, &forge_pmic_cam, "VCAMD_ANA_CON0 VOSEL (1.2 V)", true, true },
	{ 0x0A58, 0x1, 1, 0, 1, &forge_pmic_cam, "LDO_VCAMIO_CON0 EN", true, true },
	{ 0x0B14, 0x7, 8, 6, 6, &forge_pmic_cam, "VCAMIO_ANA_CON0 VOSEL (1.8 V)", true, true },
	{ 0x0A80, 0x1, 1, 0, 1, &forge_pmic_cam_front, "LDO_VGP3_CON0 EN (front DVDD)", true, true },
	{ 0x0AFC, 0x7, 8, 3, 3, &forge_pmic_cam_front, "VGP3_ANA_CON0 VOSEL (1.22 V, front DVDD)", true, true },
	{ 0x0A3A, 0x1, 1, 0, 1, &forge_pmic_vibr, "LDO_VIBR_CON0 EN (vibrator)", true, true, true },
	{ 0x0AE6, 0x7, 8, 5, 5, &forge_pmic_vibr, "VIBR_ANA_CON0 VOSEL (2.8 V, vibrator)", true, true },
};

/* a register may hold several rows, one per field */
static const struct forge_pmic_wl *forge_pmic_wl_row(u32 reg, u32 mask,
						     u32 shift)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(forge_pmic_wl); i++)
		if (forge_pmic_wl[i].reg == reg &&
		    forge_pmic_wl[i].mask == mask &&
		    forge_pmic_wl[i].shift == shift)
			return &forge_pmic_wl[i];
	return NULL;
}

/* every row on m681; a row marked l681 on the l681 board too (DT) */
static bool forge_pmic_wl_board(const struct forge_pmic_wl *wl)
{
	return of_machine_is_compatible("meizu,m681") ||
	       (wl->l681 && m3note_board_is(M3NOTE_BOARD_L681));
}

/* the row's switch on, or an off write (0) on an off_ok row */
static bool forge_pmic_wl_on(const struct forge_pmic_wl *wl, u32 code)
{
	return READ_ONCE(*wl->allow) || (wl->off_ok && code == 0);
}

static bool forge_pmic_wl_ok(const struct forge_pmic_wl *wl, u32 code)
{
	return wl && forge_pmic_wl_on(wl, code) && forge_pmic_wl_board(wl) &&
	       code >= wl->lo && code <= wl->hi;
}

/* The rail setpoints have their own token and lock: a DVFS write must
 * never make a battery ADC request (trylock) fail. */
static DEFINE_MUTEX(forge_wl_lock);
static struct task_struct *forge_wl_owner;
static const struct forge_pmic_wl *forge_wl_row;
static u32 forge_wl_reg, forge_wl_value;
static bool forge_wl_armed;

/* Called under WACS2's spinlock. An IRQ or another task cannot use a token. */
bool forge_pmic_adc_consume(u32 adr, u32 value)
{
	if (in_interrupt())
		return false;
	if (READ_ONCE(forge_wl_owner) == current && READ_ONCE(forge_wl_armed) &&
	    READ_ONCE(forge_wl_reg) == adr && READ_ONCE(forge_wl_value) == value) {
		const struct forge_pmic_wl *wl = READ_ONCE(forge_wl_row);

		if (!wl || wl->reg != adr ||
		    !forge_pmic_wl_ok(wl, (value >> wl->shift) & wl->mask))
			return false;
		WRITE_ONCE(forge_wl_armed, false);
		return true;
	}
	if (READ_ONCE(forge_adc_owner) != current ||
	    !READ_ONCE(forge_adc_request_armed) ||
	    READ_ONCE(forge_adc_request_addr) != adr ||
	    READ_ONCE(forge_adc_request) != value)
		return false;
	if (adr == 0x0E98) {
		if (value != BIT(1) && value != BIT(3))
			return false;
	} else if (adr != FORGE_ADC_AON_REG && adr != FORGE_ADC_SMPS_REG) {
		return false;
	}
	WRITE_ONCE(forge_adc_request_armed, false);
	return true;
}

static int forge_pmic_adc_write(unsigned int val, unsigned int mask,
				unsigned int shift)
{
	s32 ret;

	if (val != 1 || mask != 1 || (shift != 1 && shift != 3))
		return -EINVAL;
	if (!forge_battery_adc)
		return -EPERM;
	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	if (!mutex_trylock(&forge_adc_owner_lock))
		return -EBUSY;
	WRITE_ONCE(forge_adc_request_addr, 0x0E98);
	WRITE_ONCE(forge_adc_request, BIT(shift));
	WRITE_ONCE(forge_adc_request_armed, true);
	WRITE_ONCE(forge_adc_owner, current);
	ret = pwrap_wacs2(1, 0x0E98, BIT(shift), NULL);
	WRITE_ONCE(forge_adc_request_armed, false);
	WRITE_ONCE(forge_adc_owner, NULL);
	WRITE_ONCE(forge_adc_request, 0);
	WRITE_ONCE(forge_adc_request_addr, 0);
	mutex_unlock(&forge_adc_owner_lock);
	return ret;
}

/* Caller holds forge_adc_owner_lock in process context. Preserve all other
 * register bits and grant exactly the value obtained from this owned RMW. */
static int forge_pmic_adc_clock_field(u32 reg, u32 mask, u32 target)
{
	u32 old, next, readback;
	s32 ret;

	if ((reg != FORGE_ADC_AON_REG || mask != FORGE_ADC_AON_MASK ||
	     target != FORGE_ADC_AON_MASK) &&
	    (reg != FORGE_ADC_SMPS_REG || mask != FORGE_ADC_SMPS_MASK || target))
		return -EINVAL;
	ret = pwrap_wacs2(0, reg, 0, &old);
	if (ret)
		return ret < 0 ? ret : -EIO;
	next = (old & ~mask) | target;
	if (next == old)
		return 0;
	WRITE_ONCE(forge_adc_request_addr, reg);
	WRITE_ONCE(forge_adc_request, next);
	WRITE_ONCE(forge_adc_request_armed, true);
	WRITE_ONCE(forge_adc_owner, current);
	ret = pwrap_wacs2(1, reg, next, NULL);
	WRITE_ONCE(forge_adc_request_armed, false);
	WRITE_ONCE(forge_adc_owner, NULL);
	WRITE_ONCE(forge_adc_request, 0);
	WRITE_ONCE(forge_adc_request_addr, 0);
	if (ret)
		return ret < 0 ? ret : -EIO;
	ret = pwrap_wacs2(0, reg, 0, &readback);
	if (ret)
		return ret < 0 ? ret : -EIO;
	/* Field failure is real; no ADC request or provider follows it. */
	if ((readback & mask) != target)
		return -EIO;
	return 0;
}

/* Prepare the AUXADC as stock PMIC_IMM_GetOneChannelValue does: AON=1,
 * SMPS_CK_PDN=0. This grants no voltage/rail/charger write. */
int forge_pmic_battery_adc_prepare(void)
{
	u32 aon, smps;
	s32 ret;

	if (!forge_battery_adc)
		return -EPERM;
	/* same MT6351 AUXADC on both M3 Note boards (VBAT on the ISENSE IMM
	 * channel); the board is the DT machine */
	if (!of_machine_is_compatible("meizu,m681") &&
	    !m3note_board_is(M3NOTE_BOARD_L681))
		return -ENODEV;
	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	if (!mutex_trylock(&forge_adc_owner_lock))
		return -EBUSY;
	ret = pwrap_wacs2(0, FORGE_ADC_AON_REG, 0, &aon);
	if (!ret)
		ret = pwrap_wacs2(0, FORGE_ADC_SMPS_REG, 0, &smps);
	if (ret) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}
	/* LK may already have configured these fields. No write is necessary. */
	if ((aon & FORGE_ADC_AON_MASK) && !(smps & FORGE_ADC_SMPS_MASK))
		goto out;
	if (!forge_battery_adc_clocks) {
		ret = -EACCES;
		goto out;
	}
	ret = forge_pmic_adc_clock_field(FORGE_ADC_AON_REG,
					FORGE_ADC_AON_MASK, FORGE_ADC_AON_MASK);
	if (!ret)
		ret = forge_pmic_adc_clock_field(FORGE_ADC_SMPS_REG,
						FORGE_ADC_SMPS_MASK, 0);
out:
	mutex_unlock(&forge_adc_owner_lock);
	return ret;
}

bool forge_pmic_vstep_allowed(void)
{
	return forge_vstep_allow && (of_machine_is_compatible("meizu,m681") ||
				     m3note_board_is(M3NOTE_BOARD_L681));
}

/* owned RMW of one listed field; the DVFS callers pause and read back */
static int forge_pmic_wl_write(const struct forge_pmic_wl *wl, u32 code)
{
	u32 old = 0, next = 0;
	s32 ret;

	if (!wl || !forge_pmic_wl_on(wl, code))
		ret = -EPERM;
	else if (!forge_pmic_wl_ok(wl, code))
		ret = -EINVAL;
	else if (in_interrupt() || in_atomic() || irqs_disabled())
		ret = -EWOULDBLOCK;
	else
		ret = 0;
	if (ret) {
		/* the connsys and camera rows are written a few times per
		 * power cycle: every refusal is logged (as the a13i table) */
		if (wl && wl->allow != &forge_vstep_allow &&
		    wl->allow != &forge_pmic_vgpu)
			pr_warn_ratelimited("[FORGE_M681] PMIC write refused: %s 0x%04x <= %u: %d\n",
					    wl->name, wl->reg, code, ret);
		return ret;
	}
	mutex_lock(&forge_wl_lock);
	ret = pwrap_wacs2(0, wl->reg, 0, &old);
	if (ret)
		goto out;
	next = (old & ~((u32)wl->mask << wl->shift)) | (code << wl->shift);
	if (next == old)
		goto out;
	WRITE_ONCE(forge_wl_row, wl);
	WRITE_ONCE(forge_wl_reg, wl->reg);
	WRITE_ONCE(forge_wl_value, next);
	WRITE_ONCE(forge_wl_armed, true);
	WRITE_ONCE(forge_wl_owner, current);
	ret = pwrap_wacs2(1, wl->reg, next, NULL);
	WRITE_ONCE(forge_wl_armed, false);
	WRITE_ONCE(forge_wl_owner, NULL);
	WRITE_ONCE(forge_wl_value, 0);
	WRITE_ONCE(forge_wl_reg, 0);
	WRITE_ONCE(forge_wl_row, NULL);
out:
	mutex_unlock(&forge_wl_lock);
	/* the vibrator EN row flips on every buzz (touch haptics): only an
	 * error or a VOSEL change goes to the log at info level */
	if (wl->allow == &forge_pmic_vibr && !ret &&
	    (next == old || wl->reg == 0x0A3A))
		pr_debug("[FORGE_M681] %s 0x%04x: 0x%04x -> 0x%04x\n",
			 wl->name, wl->reg, old, next);
	else if (wl->allow != &forge_vstep_allow && wl->allow != &forge_pmic_vgpu)
		pr_info("[FORGE_M681] PMIC write: %s 0x%04x 0x%04x -> 0x%04x (rd/wr %d)\n",
			wl->name, wl->reg, old, next, ret);
	else if (next != old)
		pr_debug("[FORGE_M681] %s 0x%04x: 0x%04x -> 0x%04x (wr %d)\n",
			 wl->name, wl->reg, old, next, ret);
	return ret ? (ret < 0 ? ret : -EIO) : 0;
}

int forge_pmic_vsram_set(u32 code)
{
	return forge_pmic_wl_write(forge_pmic_wl_row(FORGE_VSRAM_ON_REG,
						     FORGE_VOSEL_ON_MASK, 0), code);
}

int forge_pmic_vsram_sleep_set(u32 code)
{
	return forge_pmic_wl_write(forge_pmic_wl_row(FORGE_VSRAM_SLEEP_REG,
						     FORGE_VOSEL_ON_MASK, 0), code);
}

int forge_pmic_vsram_ctrl_set(u32 on)
{
	return forge_pmic_wl_write(forge_pmic_wl_row(FORGE_VSRAM_CON0_REG, 0x1, 1),
				   on ? 1 : 0);
}

unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val,
				   unsigned int MASK, unsigned int SHIFT)
{
	const struct forge_pmic_wl *wl;

	if (RegNum == 0x0E98)
		return forge_pmic_adc_write(val, MASK, SHIFT);
	if (forge_pmic_touch_write(RegNum, val, MASK, SHIFT))
		return 0;
	/* GPU DVFS (VGPU VOSEL_ON/VOSEL) and connsys (VCN EN/ON_CTRL) write
	 * their fields through the stock API */
	wl = forge_pmic_wl_row(RegNum, MASK, SHIFT);
	if (wl && wl->api) {
		int ret = forge_pmic_wl_write(wl, val);

		/* a refusal counts as a dropped write too (dropped_writes,
		 * last_dropped); the caller gets the code, which consys checks */
		if (ret)
			forge_pmic_drop(__func__, RegNum, val,
					__builtin_return_address(0));
		return ret;
	}
	forge_pmic_drop(__func__, RegNum, val, __builtin_return_address(0));
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface);

unsigned int pmic_config_interface_nolock(unsigned int RegNum, unsigned int val,
					  unsigned int MASK, unsigned int SHIFT)
{
	/* No sleeping ownership route through the nolock API. */
	if (RegNum == 0x0E98)
		return -EOPNOTSUPP;
	forge_pmic_drop(__func__, RegNum, val, __builtin_return_address(0));
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface_nolock);

/* A flag write reaches an allowlist api row when the flag table names exactly its
 * register and field (the camera rails); anything else is dropped as
 * before. A refused row write returns its error and counts as dropped. */
unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val)
{
	const PMU_FLAG_TABLE_ENTRY *f = forge_pmic_flag(flagname);
	const struct forge_pmic_wl *wl =
		f ? forge_pmic_wl_row(f->offset, f->mask, f->shift) : NULL;
	int ret;

	if (wl && wl->api) {
		ret = forge_pmic_wl_write(wl, val);
		if (ret)
			forge_pmic_drop(__func__, flagname, val,
					__builtin_return_address(0));
		return ret;
	}
	forge_pmic_drop(__func__, flagname, val, __builtin_return_address(0));
	return 0;
}
EXPORT_SYMBOL(pmic_set_register_value);

unsigned short pmic_set_register_value_nolock(PMU_FLAGS_LIST_ENUM flagname, unsigned int val)
{
	forge_pmic_drop(__func__, flagname, val, __builtin_return_address(0));
	return 0;
}
EXPORT_SYMBOL(pmic_set_register_value_nolock);

/* battery-throttling notifiers (pmic_throttling_dlpt.c, not built): gpufreq
 * registers its low-battery/volume callbacks; nothing ever calls them here */
void register_low_battery_notify(void (*low_battery_callback)(LOW_BATTERY_LEVEL),
				 LOW_BATTERY_PRIO prio_val)
{
}
EXPORT_SYMBOL(register_low_battery_notify);

void register_battery_percent_notify(void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL),
				     BATTERY_PERCENT_PRIO prio_val)
{
}
EXPORT_SYMBOL(register_battery_percent_notify);
#endif /* CONFIG_MTK_PMIC_WRAP_HAL */

unsigned int pmic_config_interface_nospinlock(unsigned int RegNum, unsigned int val,
					      unsigned int MASK, unsigned int SHIFT)
{
#ifdef CONFIG_MTK_PMIC_WRAP_HAL
	if (RegNum == 0x0E98)
		return -EOPNOTSUPP;
	forge_pmic_drop(__func__, RegNum, val, __builtin_return_address(0));
#endif
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface_nospinlock);

unsigned short pmic_set_register_value_nospinlock(PMU_FLAGS_LIST_ENUM flagname, unsigned int val)
{
#ifdef CONFIG_MTK_PMIC_WRAP_HAL
	forge_pmic_drop(__func__, flagname, val, __builtin_return_address(0));
#endif
	return 0;
}
EXPORT_SYMBOL(pmic_set_register_value_nospinlock);

int pmic_force_vcore_pwm(bool enable)
{
	return 0;
}
EXPORT_SYMBOL(pmic_force_vcore_pwm);

#ifndef CONFIG_MTK_SMART_BATTERY
/* charger presence (PMIC): report "no charger" while the charger subsystem
 * is not built. The real chr_type_det path reads PMIC_RGS_CHRDET directly.
 * With CONFIG_MTK_SMART_BATTERY=y the real provider is
 * battery_common_fg_20.c and this stub is compiled out. */
bool upmu_is_chr_det(void)
{
	return false;
}
EXPORT_SYMBOL(upmu_is_chr_det);
#endif /* !CONFIG_MTK_SMART_BATTERY */

/*
 * PMIC-wrap chip-HAL symbols (pwrap_base, mt_pmic_wrap_eint_status,
 * mt_pmic_wrap_eint_clr) are now provided by the real chip HAL,
 * drivers/misc/mediatek/pmic_wrap/mt6755/pwrap_hal_v1.c, which compiles
 * once the mt6755/Makefile uses CONFIG_MTK_PMIC_WRAP_HAL (the Kconfig symbol
 * that actually exists) instead of the phantom CONFIG_MTK_PMIC_WRAP. The
 * stubs that used to stand in for them here were removed to avoid multiple
 * definition link errors now that the real HAL is built.
 */

/*
 * MT6311 external-buck presence queries for the PMIC ext-buck helpers;
 * mt6311.c is not built here. Reporting "absent / not ready" makes the
 * helpers fall back to the internal bucks.
 */
int is_mt6311_exist(void)
{
	return 0;
}
EXPORT_SYMBOL(is_mt6311_exist);

int is_mt6311_sw_ready(void)
{
	return 0;
}
EXPORT_SYMBOL(is_mt6311_sw_ready);

/* The ext buck is the MT6311; see is_mt6311_exist() above. */
int is_ext_buck_sw_ready(void)
{
	return is_mt6311_sw_ready() == 1;
}
EXPORT_SYMBOL(is_ext_buck_sw_ready);

int is_ext_buck_exist(void)
{
	return is_mt6311_exist() == 1;
}
EXPORT_SYMBOL(is_ext_buck_exist);

int get_mt6311_i2c_ch_num(void)
{
	return -1;
}
EXPORT_SYMBOL(get_mt6311_i2c_ch_num);

#ifndef CONFIG_MTK_PMIC_WRAP_HAL
/* PMIC interface stubs while the pmic/ driver is not built. SPM and the
 * marker code call these for PMIC/RTC access. With MTK_PMIC_WRAP_HAL the
 * read-only versions at the top of this file and the pwrap HAL replace
 * this whole block. */
unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT)
{
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface);

unsigned int pmic_config_interface_nolock(unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT)
{
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface_nolock);

signed int pwrap_write(unsigned int adr, unsigned int wdata)
{
	return -ENODEV;
}
EXPORT_SYMBOL(pwrap_write);

/* skeleton: read-side PMIC interface + pwrap base pointer stubs,
 * while pmic_wrap/mt6351 are deferred to Phase 2. REMOVE with the driver. */
unsigned int pmic_read_interface_nolock(unsigned int RegNum, unsigned int *val, unsigned int MASK, unsigned int SHIFT)
{
	if (val)
		*val = 0;
	return 0;
}
EXPORT_SYMBOL(pmic_read_interface_nolock);

void __iomem *pwrap_base;
EXPORT_SYMBOL(pwrap_base);
#endif /* !CONFIG_MTK_PMIC_WRAP_HAL */

#if !defined(CONFIG_MTK_PMIC_NEW_ARCH) && !defined(CONFIG_MTK_PMIC_WRAP_HAL)
/* leds/mt6755 (MT65XX_LED_MODE_PMIC = MT6351 ISINK channels) references
 * this. No LED here is in PMIC mode (DT: lcd-backlight led_mode 5 =
 * CUST_BLS_PWM, i.e. DISP_PWM in the display block; the others 0 = NONE),
 * so it is not reached on this board. A real ISINK write needs the MT6351
 * PMIC driver + pwrap chip HAL (MTK_PMIC_NEW_ARCH + MTK_PMIC_WRAP_HAL),
 * which are not built; the write is dropped and reported once. */
unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val)
{
	pr_warn_once("[FORGE_M681] pmic_set_register_value(%d, 0x%x) dropped: no PMIC driver on 4.9\n",
		     (int)flagname, val);
	return 0;
}
EXPORT_SYMBOL(pmic_set_register_value);
#endif
