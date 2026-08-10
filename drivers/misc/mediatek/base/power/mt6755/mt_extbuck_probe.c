/*
 * MT6311 external-buck presence probe, read only, default off.
 *
 * MT6351 has no VPROC buck (pmic/include/mt6351/mach/upmu_hw.h defines
 * VCORE/VGPU/VMODEM/VMD1/VSRAM_MD/VS1/VS2/VPA/VSRAM_PROC/K and nothing
 * else). The stock kernel drives the CPU rail through the MT6311 external
 * buck (mt6311_set_vdvfs11_vosel_on) and only VSRAM_PROC through the PMIC.
 * Without an MT6311 no register in this tree can move VPROC.
 *
 * It performs two 1-byte I2C reads and prints them. It writes nothing.
 *
 *   echo 1 > /sys/module/mt_extbuck_probe/parameters/forge_extbuck_probe
 *
 * Detection matches the stock mt6311_hw_component_detect():
 * chip_id = (CID << 8) | SWCID, valid = 0x0110 / 0x0120 / 0x0130.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/reboot.h>

#define FEB_TAG			"[FORGE_EXTBUCK] "
#define FEB_DT_COMPATIBLE	"mediatek,ext_buck"
#define FEB_REG_CID		0x00
#define FEB_REG_SWCID		0x01
#define FEB_CID_E1		0x0110
#define FEB_CID_E2		0x0120
#define FEB_CID_E3		0x0130

/* One byte out (register index), one byte back. No write anywhere. */
static int forge_extbuck_read(struct i2c_adapter *adap, u16 addr,
			      u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 wbuf = reg;
	int ret;

	/* Always memset a hand-built i2c_msg: uninitialised fields corrupt the
	 * transfer. */
	memset(msgs, 0, sizeof(msgs));

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &wbuf;

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret == 2)
		return 0;

	return (ret < 0) ? ret : -EIO;
}

static void forge_extbuck_run(void)
{
	struct device_node *np;
	struct device_node *parent;
	struct i2c_adapter *adap;
	unsigned int chip_id;
	u32 addr = 0;
	u8 cid = 0;
	u8 swcid = 0;
	int ret;

	np = of_find_compatible_node(NULL, NULL, FEB_DT_COMPATIBLE);
	if (!np) {
		pr_err(FEB_TAG "no DT node compatible=\"%s\" => nothing to probe\n",
		       FEB_DT_COMPATIBLE);
		return;
	}

	if (of_property_read_u32(np, "reg", &addr)) {
		pr_err(FEB_TAG "node %s has no reg property\n", np->full_name);
		of_node_put(np);
		return;
	}

	parent = of_get_parent(np);
	adap = parent ? of_find_i2c_adapter_by_node(parent) : NULL;
	if (!adap) {
		pr_err(FEB_TAG "node %s: parent is not a live i2c adapter\n",
		       np->full_name);
		of_node_put(parent);
		of_node_put(np);
		return;
	}

	pr_err(FEB_TAG "node=%s i2c-%d addr=0x%02x\n",
	       np->full_name, adap->nr, addr);

	ret = forge_extbuck_read(adap, (u16)addr, FEB_REG_CID, &cid);
	if (ret) {
		pr_err(FEB_TAG "CID read FAILED ret=%d => NO ACK at 0x%02x (buck ABSENT or bus/rail down)\n",
		       ret, addr);
		goto out;
	}

	ret = forge_extbuck_read(adap, (u16)addr, FEB_REG_SWCID, &swcid);
	if (ret) {
		pr_err(FEB_TAG "SWCID read FAILED ret=%d (CID=0x%02x did ACK)\n",
		       ret, cid);
		goto out;
	}

	chip_id = ((unsigned int)cid << 8) | swcid;

	if (chip_id == FEB_CID_E1 || chip_id == FEB_CID_E2 ||
	    chip_id == FEB_CID_E3)
		pr_err(FEB_TAG "CID=0x%02x SWCID=0x%02x chip_id=0x%04x => MT6311 PRESENT\n",
		       cid, swcid, chip_id);
	else
		pr_err(FEB_TAG "CID=0x%02x SWCID=0x%02x chip_id=0x%04x not in {0110,0120,0130} => UNKNOWN DEVICE\n",
		       cid, swcid, chip_id);

out:
	i2c_put_adapter(adap);
	of_node_put(parent);
	of_node_put(np);
}

/*
 * Write 2: add the skipped i2c bus 3 late, then read MT6311 CID/SWCID and
 * 0x88-0x94, MT6351 VSRAM_PROC CON0/3/4/5/6/7 and VGPU CON0/5/7 over
 * pwrap, the efuse words 0..31 through get_devinfo_with_index, the GPU and
 * CPU frequencies, and print the guard verdict: chip E1/E2/E3,
 * 0x8E = 0x90 = 0x40, 0x88 b1 = 1, 0x06A0 b1 = 1, 0x06AA = 0x50. Reads
 * only: nothing is written to any PMIC or buck.
 */
#include <linux/cpufreq.h>

extern int forge_i2c_add_deferred(int id) __attribute__((weak));
extern s32 pwrap_read(u32 adr, u32 *rdata);
#include <mt-plat/mtk_devinfo.h>
extern unsigned int mt_gpufreq_get_cur_freq(void) __attribute__((weak));
extern unsigned int mt_gpufreq_get_cur_volt(void) __attribute__((weak));

static const u16 feb_pmic_regs[] = {
	0x06A0, 0x06A6, 0x06A8, 0x06AA, 0x06AC, 0x06AE,	/* VSRAM_PROC CON0/3/4/5/6/7 */
	0x0614, 0x061E, 0x0622,				/* VGPU CON0/5/7 */
};

/* the MT6311 node's live adapter (caller puts it) and its address */
static struct i2c_adapter *forge_extbuck_adapter(u32 *addr)
{
	struct device_node *np, *parent;
	struct i2c_adapter *adap = NULL;

	np = of_find_compatible_node(NULL, NULL, FEB_DT_COMPATIBLE);
	if (np && !of_property_read_u32(np, "reg", addr)) {
		parent = of_get_parent(np);
		adap = parent ? of_find_i2c_adapter_by_node(parent) : NULL;
		of_node_put(parent);
	}
	of_node_put(np);
	return adap;
}

static void forge_opp_dump(void)
{
	struct i2c_adapter *adap;
	u8 r[0x95] = { 0 };
	bool ok[0x95] = { false };
	u32 pv[ARRAY_SIZE(feb_pmic_regs)] = { 0 };
	u32 addr = 0;
	unsigned int i, chip = 0;
	int ret;
	bool guard = true;

	ret = forge_i2c_add_deferred ? forge_i2c_add_deferred(3) : -ENOSYS;
	pr_err(FEB_TAG "sh0: i2c bus3 late add: %d\n", ret);

	adap = forge_extbuck_adapter(&addr);
	if (adap) {
		for (i = 0; i < ARRAY_SIZE(r); i++) {
			if (i > 0x01 && i < 0x88)
				continue;
			ok[i] = !forge_extbuck_read(adap, (u16)addr, i, &r[i]);
		}
		chip = ok[0] && ok[1] ? ((unsigned int)r[0] << 8) | r[1] : 0;
		pr_err(FEB_TAG "sh0: MT6311 i2c-%d 0x%02x chip_id=0x%04x\n",
		       adap->nr, addr, chip);
		i2c_put_adapter(adap);
		for (i = 0x88; i < ARRAY_SIZE(r); i++)
			pr_err(FEB_TAG "sh0: MT6311 0x%02x = %s0x%02x\n", i,
			       ok[i] ? "" : "READ FAILED ", r[i]);
	} else {
		pr_err(FEB_TAG "sh0: no live i2c adapter for the MT6311 node\n");
	}

	for (i = 0; i < ARRAY_SIZE(feb_pmic_regs); i++) {
		ret = pwrap_read(feb_pmic_regs[i], &pv[i]);
		pr_err(FEB_TAG "sh0: MT6351 0x%04x = 0x%04x (%d)\n",
		       feb_pmic_regs[i], pv[i], ret);
	}
	for (i = 0; i < 32; i++)
		pr_err(FEB_TAG "sh0: efuse devinfo[%u] = 0x%08x\n", i,
		       get_devinfo_with_index(i));
	if (mt_gpufreq_get_cur_freq)
		pr_err(FEB_TAG "sh0: GPU %u kHz, %u (10 uV)\n",
		       mt_gpufreq_get_cur_freq(),
		       mt_gpufreq_get_cur_volt ? mt_gpufreq_get_cur_volt() : 0);
	pr_err(FEB_TAG "sh0: CPU0 %u kHz, CPU4 %u kHz\n", cpufreq_quick_get(0),
	       cpufreq_quick_get(4));

	/* the plan's guard for step s0 (a9 p.3) */
	if (chip != FEB_CID_E1 && chip != FEB_CID_E2 && chip != FEB_CID_E3) {
		pr_err(FEB_TAG "sh0: guard: chip_id 0x%04x not E1/E2/E3\n", chip);
		guard = false;
	}
	if (!ok[0x8E] || r[0x8E] != 0x40 || !ok[0x90] || r[0x90] != 0x40) {
		pr_err(FEB_TAG "sh0: guard: 0x8E/0x90 not 0x40\n");
		guard = false;
	}
	if (!ok[0x88] || !(r[0x88] & 0x02)) {
		pr_err(FEB_TAG "sh0: guard: MT6311 0x88 b1 (VOSEL_CTRL) is 0\n");
		guard = false;
	}
	if (!(pv[0] & 0x02)) {
		pr_err(FEB_TAG "sh0: guard: MT6351 0x06A0 b1 (VSRAM VOSEL_CTRL) is 0\n");
		guard = false;
	}
	if ((pv[3] & 0x7F) != 0x50) {
		pr_err(FEB_TAG "sh0: guard: VSRAM 0x06AA = 0x%02x, not 0x50\n",
		       pv[3] & 0x7F);
		guard = false;
	}
	pr_err(FEB_TAG "sh0: guard %s\n", guard ? "OK (s0)" : "STOP");
}

/*
 * Stage writer, off by default. The only writes are MT6311 0x8E [6:0]
 * (VPROC VOSEL_ON, RMW) and, through forge_pmic_vsram_set(), MT6351
 * 0x06AA [6:0], both to the stock FY codes of one stage. Nothing happens
 * unless the image was built with FORGE_M681_VSTEP_ALLOW 1 (mt_pmic_stub.c,
 * read-only forge_vstep_allow).
 *
 *   echo <0..3> > /sys/module/mt_extbuck_probe/parameters/forge_vproc_step
 *
 * The current stage is read back every time, never remembered: 0x8E, 0x90
 * and 0x06AA must all match one row, with the guard (chip E1/E2/E3, 0x88 b1,
 * 0x06A0 b1, 0x8A b7 = 0) - else nothing is written. Only +-1 stage. Up:
 * VSRAM, pause, VPROC, pause; down: the reverse (stock set_cur_volt_extbuck).
 * The final readback must give the target row with no OC flag; otherwise
 * the start codes go back in the reverse order and forge_vstep_fault
 * latches until reboot. FORGE_FREQ_ONLY keeps the governor on the rows of
 * 1.000 V (big 1196, LL 689), legal at every stage, so no step needs a
 * frequency move first; unlocking higher rows must add that.
 */
extern bool forge_pmic_vstep_allowed(void) __attribute__((weak));
extern int forge_pmic_vsram_set(u32 code) __attribute__((weak));
extern int forge_pmic_vsram_ctrl_set(u32 on) __attribute__((weak));
extern int forge_pmic_vsram_sleep_set(u32 code) __attribute__((weak));

struct forge_vstep {
	u8 vproc;	/* MT6311 0x8E */
	u8 vsram;	/* MT6351 0x06AA */
	u32 big_khz, ll_khz;	/* FY rows legal at this VPROC */
};

static const struct forge_vstep forge_vsteps[] = {
	{ 0x40, 0x50, 1196000,  689000 },	/* s0: PL, 1.000 / 1.100 V */
	{ 0x4C, 0x58, 1495000,  819000 },	/* s1: 1.075 / 1.150 V */
	{ 0x52, 0x58, 1651000,  910000 },	/* s2: 1.1125 / 1.150 V */
	{ 0x58, 0x58, 1807000, 1001000 },	/* s3: 1.150 / 1.150 V */
};

/* the manual stage knob; off on the Sh-stock line, where the rail writes
 * belong to cpufreq (forge_vstep_allow is on there for that) */
#define FORGE_M681_VSTEP_KNOB	0
static DEFINE_MUTEX(forge_vstep_lock);
static bool forge_dvfs_owner;	/* full CPU DVFS is live (rail writer below) */
static bool forge_vstep_fault;
module_param(forge_vstep_fault, bool, 0444);
static bool forge_vstep_reboot_s0 = true;	/* 0 for the warm-reboot test */
module_param(forge_vstep_reboot_s0, bool, 0644);

/* both rails: 0.600 V + code * 6.25 mV, in 10 uV like the OPP tables */
static u32 forge_vcode_uv10(u8 code)
{
	return 60000 + code * 625;
}

/* stock _calc_pmic_settle_time: up ceil(d/1250)+5, down d*2/625+5, >= 25 us */
static unsigned int forge_vsettle_us(u8 from, u8 to)
{
	u32 a = forge_vcode_uv10(from), b = forge_vcode_uv10(to);
	unsigned int us = b > a ? DIV_ROUND_UP(b - a, 1250) + 5 :
				  (a - b) * 2 / 625 + 5;

	return max(us, 25U);
}

/* the table against the stock FY rule (VPROC = row volt, VSRAM =
 * min(VPROC + 100 mV, 1.150 V), p.0 #1); a mismatch disables P5 */
static bool forge_vsteps_sane(void)
{
	static const u32 row_uv10[] = { 100000, 107500, 111250, 115000 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(forge_vsteps); i++)
		if (forge_vcode_uv10(forge_vsteps[i].vproc) != row_uv10[i] ||
		    forge_vcode_uv10(forge_vsteps[i].vsram) !=
		    min(row_uv10[i] + 10000, 115000U))
			return false;
	return true;
}

static int forge_extbuck_write(struct i2c_adapter *adap, u16 addr,
			       u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 wbuf[2] = { reg, val };
	int ret;

	memset(&msg, 0, sizeof(msg));
	msg.addr = addr;
	msg.len = 2;
	msg.buf = wbuf;
	ret = i2c_transfer(adap, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

struct forge_vstate {
	u8 r[0x95];	/* MT6311 0x00/0x01, 0x88, 0x8A..0x90, 0x94 */
	u32 p06a0, p06a8, p06aa, p06ac, p06ae;
	int step;	/* -1: no row matches */
	bool guard;
};

static int forge_vstate_read(struct i2c_adapter *adap, u16 addr,
			     struct forge_vstate *st)
{
	static const u8 regs[] = { 0x00, 0x01, 0x88, 0x8A, 0x8B, 0x8C, 0x8D,
				   0x8E, 0x8F, 0x90, 0x94 };
	unsigned int i, chip;
	int ret = 0;

	memset(st, 0, sizeof(*st));
	st->step = -1;
	for (i = 0; i < ARRAY_SIZE(regs) && !ret; i++)
		ret = forge_extbuck_read(adap, addr, regs[i], &st->r[regs[i]]);
	if (!ret)
		ret = pwrap_read(0x06A0, &st->p06a0);
	if (!ret)
		ret = pwrap_read(0x06A8, &st->p06a8);
	if (!ret)
		ret = pwrap_read(0x06AA, &st->p06aa);
	if (!ret)
		ret = pwrap_read(0x06AC, &st->p06ac);
	if (!ret)
		ret = pwrap_read(0x06AE, &st->p06ae);
	if (ret)
		return ret < 0 ? ret : -EIO;
	for (i = 0; i < ARRAY_SIZE(forge_vsteps); i++)
		if ((st->r[0x8E] & 0x7F) == forge_vsteps[i].vproc &&
		    (st->r[0x90] & 0x7F) == forge_vsteps[i].vproc &&
		    (st->p06aa & 0x7F) == forge_vsteps[i].vsram)
			st->step = i;
	chip = ((unsigned int)st->r[0x00] << 8) | st->r[0x01];
	st->guard = (chip == FEB_CID_E1 || chip == FEB_CID_E2 ||
		     chip == FEB_CID_E3) &&
		    (st->r[0x88] & 0x02) && (st->p06a0 & 0x02) &&
		    !(st->r[0x8A] & 0x80);
	pr_err(FEB_TAG "vstep: chip 0x%04x 0x88 0x%02x 0x8A 0x%02x 0x8B 0x%02x 0x8C 0x%02x 0x8D 0x%02x 0x8E 0x%02x 0x8F 0x%02x 0x90 0x%02x 0x94 0x%02x 0x06A0 0x%04x 0x06AA 0x%04x 0x06AE 0x%04x -> s%d guard %d\n",
	       chip, st->r[0x88], st->r[0x8A], st->r[0x8B], st->r[0x8C],
	       st->r[0x8D], st->r[0x8E], st->r[0x8F], st->r[0x90], st->r[0x94],
	       st->p06a0, st->p06aa, st->p06ae, st->step, st->guard);
	return 0;
}

static int forge_vproc_set(struct i2c_adapter *adap, u16 addr, u8 code)
{
	u8 old = 0;
	int ret = forge_extbuck_read(adap, addr, 0x8E, &old);

	if (!ret && (old & 0x7F) != code)
		ret = forge_extbuck_write(adap, addr, 0x8E, (old & 0x80) | code);
	return ret;
}

/* one stage's codes in the stock order for the direction from -> to */
static int forge_vstep_codes(struct i2c_adapter *adap, u16 addr,
			     const struct forge_vstep *from,
			     const struct forge_vstep *to)
{
	int ret;

	if (to->vproc > from->vproc) {
		u32 aa = 0;

		ret = forge_pmic_vsram_set(to->vsram);
		udelay(forge_vsettle_us(from->vsram, to->vsram));
		/* VPROC goes up only over a VSRAM read back at the new code */
		if (!ret)
			ret = pwrap_read(0x06AA, &aa) ? -EIO :
			      (aa & 0x7F) != to->vsram ? -EIO : 0;
		if (!ret)
			ret = forge_vproc_set(adap, addr, to->vproc);
	} else {
		ret = forge_vproc_set(adap, addr, to->vproc);
		udelay(forge_vsettle_us(from->vproc, to->vproc));
		if (!ret)
			ret = forge_pmic_vsram_set(to->vsram);
	}
	udelay(max(forge_vsettle_us(from->vproc, to->vproc),
		   forge_vsettle_us(from->vsram, to->vsram)));
	return ret;
}

static int forge_vstep_apply(int to)
{
	struct i2c_adapter *adap;
	struct forge_vstate st;
	u32 addr = 0;
	int from, ret;

	if (!FORGE_M681_VSTEP_KNOB || !forge_pmic_vstep_allowed ||
	    !forge_pmic_vsram_set || !forge_pmic_vstep_allowed()) {
		pr_err(FEB_TAG "vstep: stage knob not in this image\n");
		return -EPERM;
	}
	if (to < 0 || to >= (int)ARRAY_SIZE(forge_vsteps) || !forge_vsteps_sane())
		return -EINVAL;
	if (forge_i2c_add_deferred)
		forge_i2c_add_deferred(3);
	adap = forge_extbuck_adapter(&addr);
	if (!adap)
		return -ENODEV;

	mutex_lock(&forge_vstep_lock);
	ret = -EBUSY;
	if (forge_dvfs_owner) {
		pr_err(FEB_TAG "vstep: refused, the CPU DVFS owns the rails\n");
		goto out;
	}
	ret = -EIO;
	if (forge_vstep_fault) {
		pr_err(FEB_TAG "vstep: fault latched, no write until reboot\n");
		goto out;
	}
	ret = forge_vstate_read(adap, (u16)addr, &st);
	if (ret)
		goto out;
	from = st.step;
	if (!st.guard || from < 0) {
		pr_err(FEB_TAG "vstep: STOP - guard %d, stage s%d\n", st.guard, from);
		ret = -EPERM;
		goto out;
	}
	if (to == from) {
		ret = 0;
		goto out;
	}
	if (to != from + 1 && to != from - 1) {
		pr_err(FEB_TAG "vstep: s%d -> s%d refused, one stage at a time\n", from, to);
		ret = -EINVAL;
		goto out;
	}

	ret = forge_vstep_codes(adap, (u16)addr, &forge_vsteps[from], &forge_vsteps[to]);
	if (!ret)
		ret = forge_vstate_read(adap, (u16)addr, &st);
	if (!ret && (st.step != to || !st.guard))
		ret = -EIO;
	if (ret) {
		int rb;

		forge_vstep_fault = true;
		rb = forge_vstep_codes(adap, (u16)addr, &forge_vsteps[to], &forge_vsteps[from]);
		if (forge_vstate_read(adap, (u16)addr, &st))
			st.step = -1;
		pr_err(FEB_TAG "vstep: s%d -> s%d FAILED (%d); rollback to s%d: %d, read back s%d; fault latched\n",
		       from, to, ret, from, rb, st.step);
		goto out;
	}
	pr_err(FEB_TAG "vstep: s%d -> s%d done; FY rows up to big %u / LL %u legal\n",
	       from, to, forge_vsteps[to].big_khz, forge_vsteps[to].ll_khz);
out:
	mutex_unlock(&forge_vstep_lock);
	i2c_put_adapter(adap);
	return ret;
}

static int forge_vproc_step_set(const char *val, const struct kernel_param *kp)
{
	int to;

	if (kstrtoint(val, 0, &to))
		return -EINVAL;
	return forge_vstep_apply(to);
}

static const struct kernel_param_ops forge_vproc_step_ops = {
	.set = forge_vproc_step_set,
};

module_param_cb(forge_vproc_step, &forge_vproc_step_ops, NULL, 0200);
MODULE_PARM_DESC(forge_vproc_step,
		 "DVFS P5: move VPROC/VSRAM one stage (0..3); refused unless the image allows it");

/* stage back down to s0 before a reboot or power-off, one stage at a time */
static int forge_vstep_reboot(struct notifier_block *nb, unsigned long code,
			      void *unused)
{
	struct i2c_adapter *adap;
	struct forge_vstate st;
	u32 addr = 0;
	int s;

	/* with full DVFS, mt_cpufreq's own reboot path parks the rails */
	if (!forge_vstep_reboot_s0 || forge_vstep_fault || forge_dvfs_owner)
		return NOTIFY_DONE;
	adap = forge_extbuck_adapter(&addr);
	if (!adap)
		return NOTIFY_DONE;
	if (!forge_vstate_read(adap, (u16)addr, &st))
		for (s = st.step; s > 0; s--)
			if (forge_vstep_apply(s - 1))
				break;
	i2c_put_adapter(adap);
	return NOTIFY_DONE;
}

static struct notifier_block forge_vstep_reboot_nb = {
	.notifier_call = forge_vstep_reboot,
};

static int __init forge_vstep_init(void)
{
	if (FORGE_M681_VSTEP_KNOB && forge_pmic_vstep_allowed &&
	    forge_pmic_vstep_allowed())
		register_reboot_notifier(&forge_vstep_reboot_nb);
	return 0;
}
late_initcall(forge_vstep_init);

/*
 * Rail writer under mt_cpufreq's set_cur_volt, i.e. the stock
 * set_cur_volt_extbuck on this board: one VPROC (MT6311 0x8E) for both
 * clusters plus VSRAM_PROC (MT6351 0x06AA). Called with the cpufreq mutex
 * held (sleeping i2c is fine there). Codes are stock: VSRAM = VPROC + 100 mV
 * clamped to 0.931..1.150 V; VPROC 0.800..1.150 V. A move keeps
 * 0 <= VSRAM - VPROC <= 175 mV (MAX_DIFF_VSRAM_VPROC - 2500, as stock) at
 * every write: up = VSRAM then VPROC, down = VPROC then VSRAM, in as many
 * rounds as that needs. Each write is followed by a pause (stock formula,
 * at least forge_dvfs_settle_us, since the ramp registers 0x8B/0x8C/0x94
 * may not be stock) and a readback (0x8E and 0x90, or 0x06AA) that must give
 * the code just written. Any mismatch stops the move with -EIO; the cached
 * codes are re-read so the caller works from the silicon.
 */
#define FEB_VPROC_MIN	0x20	/* 0.800 V, the lowest FY row */
#define FEB_VPROC_MAX	0x58	/* 1.150 V, MAX_VPROC_VOLT */
#define FEB_VSRAM_MIN	0x35	/* 0.931 V, MIN_VSRAM_VOLT rounded up */
#define FEB_VSRAM_MAX	0x58	/* 1.150 V, MAX_VSRAM_VOLT */
#define FEB_VSRAM_OVER	16	/* NORMAL_DIFF_VRSAM_VPROC, 100 mV */
#define FEB_MAX_DIFF	28	/* MAX_DIFF_VSRAM_VPROC - 2500, 175 mV */

static struct i2c_adapter *forge_dvfs_adap;	/* held while owner */
static u16 forge_dvfs_addr;
static u8 forge_dvfs_vproc, forge_dvfs_vsram;	/* last read-back codes */
static unsigned int forge_dvfs_settle_us = 200;
module_param(forge_dvfs_settle_us, uint, 0644);
static unsigned int forge_dvfs_writes, forge_dvfs_errors;
module_param(forge_dvfs_writes, uint, 0444);
module_param(forge_dvfs_errors, uint, 0444);

static u8 forge_vcode(u32 uv10)	/* round up, like VOLT_TO_PMIC_VAL */
{
	return uv10 <= 60000 ? 0 : (u8)min_t(u32, (uv10 - 60000 + 624) / 625, 0x7F);
}

static void forge_dvfs_pause(u8 from, u8 to)
{
	unsigned int us = max(forge_vsettle_us(from, to),
			      READ_ONCE(forge_dvfs_settle_us));

	usleep_range(us, us + 20);
}

static bool forge_dvfs_06ae;	/* 0x06AE [6:0] tracked 0x06AA at init */
/* variant A (below) switched the rail to hardware VOSEL_ON/SLEEP */
static bool forge_dvfs_ctrl_set_vproc, forge_dvfs_ctrl_set_vsram;

/* After a failed write the cache takes the lowest code the rail may be at
 * (old, attempted, every register that answered): the caller then never
 * assumes more voltage than the silicon may give. No write follows a
 * failure (mt_cpufreq latches its fault). */
static int forge_dvfs_write_vsram(u8 code)
{
	u32 aa = 0, ae = 0, sl = 0;
	int ret = forge_pmic_vsram_set(code), r1 = -EIO, r2 = -EIO;
	u8 low = min(forge_dvfs_vsram, code);

	forge_dvfs_pause(forge_dvfs_vsram, code);
	if (!ret) {
		r1 = pwrap_read(0x06AA, &aa);
		r2 = pwrap_read(0x06AE, &ae);
	}
	/* hardware VOSEL mode: the sleep code follows, so SPM's sleep
	 * signal never moves the rail (review of b9eb6df0c) */
	if (!ret && !r1 && (aa & 0x7F) == code && forge_dvfs_ctrl_set_vsram)
		ret = !forge_pmic_vsram_sleep_set ? -ENODEV :
		      forge_pmic_vsram_sleep_set(code) ?:
		      (pwrap_read(0x06AC, &sl) || (sl & 0x7F) != code ? -EIO : 0);
	forge_dvfs_writes++;
	if (!ret && !r1 && (aa & 0x7F) == code &&
	    (!forge_dvfs_06ae || (!r2 && (ae & 0x7F) == code))) {
		forge_dvfs_vsram = code;
		return 0;
	}
	forge_dvfs_errors++;
	if (!r1)
		low = min_t(u8, low, aa & 0x7F);
	if (forge_dvfs_06ae && !r2)
		low = min_t(u8, low, ae & 0x7F);
	pr_err(FEB_TAG "dvfs: VSRAM 0x%02x -> 0x%02x failed (%d, 0x06AA 0x%04x 0x06AE 0x%04x), assume 0x%02x\n",
	       forge_dvfs_vsram, code, ret, aa, ae, low);
	forge_dvfs_vsram = low;
	return -EIO;
}

static int forge_dvfs_write_vproc(u8 code)
{
	u8 on = 0, out = 0, oc = 0, sl = 0, low = min(forge_dvfs_vproc, code);
	int ret = forge_vproc_set(forge_dvfs_adap, forge_dvfs_addr, code);
	int r1 = -EIO, r2 = -EIO, r3 = -EIO;

	forge_dvfs_pause(forge_dvfs_vproc, code);
	if (!ret) {
		r1 = forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8E, &on);
		r2 = forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x90, &out);
		r3 = forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8A, &oc);
	}
	/* hardware VOSEL mode: VOSEL_SLEEP (0x8F) follows VOSEL_ON */
	if (!ret && !r1 && (on & 0x7F) == code && forge_dvfs_ctrl_set_vproc) {
		ret = forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8F, &sl);
		if (!ret && (sl & 0x7F) != code)
			ret = forge_extbuck_write(forge_dvfs_adap, forge_dvfs_addr, 0x8F,
						  (sl & 0x80) | code);
		if (!ret)
			ret = forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8F, &sl);
		if (!ret && (sl & 0x7F) != code)
			ret = -EIO;
	}
	forge_dvfs_writes++;
	if (!ret && !r1 && !r2 && !r3 && (on & 0x7F) == code &&
	    (out & 0x7F) == code && !(oc & 0x80)) {
		forge_dvfs_vproc = code;
		return 0;
	}
	forge_dvfs_errors++;
	if (!r1)
		low = min_t(u8, low, on & 0x7F);
	if (!r2)
		low = min_t(u8, low, out & 0x7F);
	pr_err(FEB_TAG "dvfs: VPROC 0x%02x -> 0x%02x failed (%d, 0x8E 0x%02x 0x90 0x%02x 0x8A 0x%02x), assume 0x%02x\n",
	       forge_dvfs_vproc, code, ret, on, out, oc, low);
	forge_dvfs_vproc = low;
	return -EIO;
}

/*
 * VOSEL_CTRL off on both rails (0x88 = 0x00 and 0x06A0 = 0x0000, as LK
 * leaves them on m681 and l681). In that mode the rails follow their
 * software VOSEL (MT6311 0x8D, MT6351 0x06A8), so a VOSEL_ON write does
 * nothing. Stock sets VOSEL_CTRL itself: MT6311 mt6311_hw_init (0x8E
 * VOSEL_ON, 0x8F VOSEL_SLEEP, 0x88 b0 = 0, 0x88 b1 = 1) and MT6351
 * PMIC_INIT_SETTING_V1 (0x06A0 b1 = 1). Done here in that order,
 * only when no output can move: VOSEL = VOSEL_ON = VOSEL_SLEEP = the output
 * code on each rail (0x8D = 0x8E = 0x8F = 0x90; 0x06A8 = 0x06AA = 0x06AC),
 * so both the ON and the sleep selection give today's voltage. The stock
 * VOSEL_SLEEP 0x10 (0.7 V in suspend) is not written: sleep on this rail is
 * not validated. VSRAM first, then VPROC; after each switch a pause and a
 * readback (MT6311 0x88 b1, 0x90 and OC 0x8A b7; MT6351 0x06A0 b1, 0x06AA
 * and the raw 0x06AE unchanged). Any mismatch: the switch goes back to 0
 * and the CPU stays freq-only.
 */
static int forge_dvfs_variant_a(struct i2c_adapter *adap, u16 addr,
				const struct forge_vstate *st)
{
	u8 vp = st->r[0x90] & 0x7F, r88 = 0, out = 0, oc = 0;
	u32 vs = st->p06a8 & 0x7F, a0 = 0, aa = 0, ae = 0;
	int ret;

	if (!forge_pmic_vsram_ctrl_set ||
	    (st->r[0x8D] & 0x7F) != vp || (st->r[0x8E] & 0x7F) != vp ||
	    (st->r[0x8F] & 0x7F) != vp || (st->r[0x88] & 0x03) ||
	    (st->p06aa & 0x7F) != vs || (st->p06ac & 0x7F) != vs ||
	    (st->p06a0 & 0x02) || (st->r[0x8A] & 0x80)) {
		pr_err(FEB_TAG "dvfs: variant A not applicable (codes differ, CTRL already set or OC)\n");
		return -EPERM;
	}

	/* MT6351 VSRAM_PROC: VOSEL_ON = VOSEL is already true; switch */
	ret = forge_pmic_vsram_ctrl_set(1);
	usleep_range(200, 220);
	if (!ret)
		ret = pwrap_read(0x06A0, &a0) || pwrap_read(0x06AA, &aa) ||
		      pwrap_read(0x06AE, &ae) ? -EIO : 0;
	if (ret || !(a0 & 0x02) || (aa & 0x7F) != vs || ae != st->p06ae) {
		forge_pmic_vsram_ctrl_set(0);
		pr_err(FEB_TAG "dvfs: variant A VSRAM failed (%d, 0x06A0 0x%04x 0x06AA 0x%04x 0x06AE 0x%04x was 0x%04x): VOSEL_CTRL back to 0\n",
		       ret, a0, aa, ae, st->p06ae);
		return -EIO;
	}
	forge_dvfs_ctrl_set_vsram = true;

	/* MT6311 VPROC: VOSEL_ON (0x8E) = VOSEL is already true; switch */
	ret = forge_extbuck_write(adap, addr, 0x88, (st->r[0x88] & ~0x03) | 0x02);
	usleep_range(200, 220);
	if (!ret)
		ret = forge_extbuck_read(adap, addr, 0x88, &r88) ||
		      forge_extbuck_read(adap, addr, 0x90, &out) ||
		      forge_extbuck_read(adap, addr, 0x8A, &oc) ? -EIO : 0;
	if (ret || (r88 & 0x03) != 0x02 || (out & 0x7F) != vp || (oc & 0x80)) {
		forge_extbuck_write(adap, addr, 0x88, st->r[0x88]);
		forge_pmic_vsram_ctrl_set(0);
		forge_dvfs_ctrl_set_vsram = false;
		pr_err(FEB_TAG "dvfs: variant A VPROC failed (%d, 0x88 0x%02x 0x90 0x%02x 0x8A 0x%02x): both VOSEL_CTRL back to 0\n",
		       ret, r88, out, oc);
		return -EIO;
	}
	forge_dvfs_ctrl_set_vproc = true;
	pr_err(FEB_TAG "dvfs: variant A done: VOSEL_CTRL on (MT6311 0x88 0x%02x, MT6351 0x06A0 0x%04x), VPROC 0x%02x VSRAM 0x%02x unchanged\n",
	       r88, a0, out, aa & 0x7F);
	return 0;
}

/* before reboot (mt_cpufreq parks the rails at the PL codes first): give
 * PL/LK the software-VOSEL mode it set, if VOSEL_ON is back on VOSEL */
void forge_extbuck_dvfs_release(void)
{
	u8 on = 0, sw = 0, r88 = 0;
	u32 aa = 0, a8 = 0;

	if (!forge_dvfs_owner)
		return;
	mutex_lock(&forge_vstep_lock);
	if (forge_dvfs_ctrl_set_vproc &&
	    !forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8E, &on) &&
	    !forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x8D, &sw) &&
	    !forge_extbuck_read(forge_dvfs_adap, forge_dvfs_addr, 0x88, &r88) &&
	    (on & 0x7F) == (sw & 0x7F))
		forge_extbuck_write(forge_dvfs_adap, forge_dvfs_addr, 0x88, r88 & ~0x02);
	if (forge_dvfs_ctrl_set_vsram && forge_pmic_vsram_ctrl_set &&
	    !pwrap_read(0x06AA, &aa) && !pwrap_read(0x06A8, &a8) &&
	    (aa & 0x7F) == (a8 & 0x7F))
		forge_pmic_vsram_ctrl_set(0);
	mutex_unlock(&forge_vstep_lock);
}
EXPORT_SYMBOL(forge_extbuck_dvfs_release);

/* the guard of p.3 for any starting stage, then take the rails */
int forge_extbuck_dvfs_init(unsigned int *vproc_uv10, unsigned int *vsram_uv10)
{
	struct forge_vstate st;
	struct i2c_adapter *adap;
	u32 addr = 0;
	u8 vp, vs;
	int ret;

	if (!forge_pmic_vstep_allowed || !forge_pmic_vsram_set ||
	    !forge_pmic_vstep_allowed())
		return -EPERM;
	if (forge_i2c_add_deferred)
		forge_i2c_add_deferred(3);
	adap = forge_extbuck_adapter(&addr);
	if (!adap)
		return -ENODEV;
	mutex_lock(&forge_vstep_lock);
	ret = forge_vstate_read(adap, (u16)addr, &st);
	/* VOSEL_CTRL off on both rails: do the stock switch, then check the
	 * guard again on the new state */
	if (!ret && !st.guard && !(st.r[0x88] & 0x02) && !(st.p06a0 & 0x02) &&
	    !forge_dvfs_variant_a(adap, (u16)addr, &st))
		ret = forge_vstate_read(adap, (u16)addr, &st);
	vp = st.r[0x8E] & 0x7F;
	vs = st.p06aa & 0x7F;
	if (!ret && (!st.guard || vp != (st.r[0x90] & 0x7F) ||
		     vp < FEB_VPROC_MIN || vp > FEB_VPROC_MAX ||
		     vs < FEB_VSRAM_MIN || vs > FEB_VSRAM_MAX ||
		     vs < vp || vs - vp > FEB_MAX_DIFF))
		ret = -EPERM;
	if (ret) {
		/* a switch made above goes back: VOSEL_ON still equals VOSEL */
		if (forge_dvfs_ctrl_set_vproc)
			forge_extbuck_write(adap, (u16)addr, 0x88, st.r[0x88] & ~0x02);
		if (forge_dvfs_ctrl_set_vsram)
			forge_pmic_vsram_ctrl_set(0);
		forge_dvfs_ctrl_set_vproc = forge_dvfs_ctrl_set_vsram = false;
		mutex_unlock(&forge_vstep_lock);
		i2c_put_adapter(adap);
		pr_err(FEB_TAG "dvfs: guard STOP (%d) - no voltage DVFS, CPU stays freq-only at the PL rail\n",
		       ret);
		return ret;
	}
	forge_dvfs_adap = adap;
	forge_dvfs_addr = (u16)addr;
	forge_dvfs_vproc = vp;
	forge_dvfs_vsram = vs;
	/* 0x06AE is checked after VSRAM writes only if it already follows
	 * 0x06AA here: its field layout is not confirmed on this board */
	forge_dvfs_06ae = (st.p06ae & 0x7F) == vs;
	forge_dvfs_owner = true;
	mutex_unlock(&forge_vstep_lock);
	*vproc_uv10 = forge_vcode_uv10(vp);
	*vsram_uv10 = forge_vcode_uv10(vs);
	pr_err(FEB_TAG "dvfs: guard OK, rails owned by cpufreq: VPROC 0x%02x VSRAM 0x%02x, settle >= %u us, 0x06AE check %d\n",
	       vp, vs, forge_dvfs_settle_us, forge_dvfs_06ae);
	return 0;
}
EXPORT_SYMBOL(forge_extbuck_dvfs_init);

int forge_extbuck_set_rails(unsigned int vproc_uv10, unsigned int *vproc_rb,
			    unsigned int *vsram_rb)
{
	u8 tp = forge_vcode(vproc_uv10);
	u8 ts = clamp_t(u8, tp + FEB_VSRAM_OVER, FEB_VSRAM_MIN, FEB_VSRAM_MAX);
	int ret = 0, rounds = 0;

	if (!forge_dvfs_owner)
		return -ENODEV;
	if (tp < FEB_VPROC_MIN || tp > FEB_VPROC_MAX)
		return -EINVAL;
	mutex_lock(&forge_vstep_lock);
	while (!ret && (forge_dvfs_vproc != tp || forge_dvfs_vsram != ts)) {
		/* from any state the guard admits (VPROC <= VSRAM <=
		 * VPROC + 175 mV) every round moves a rail; a stuck loop
		 * means some other state, so stop rather than spin */
		if (++rounds > 8) {
			ret = -EIO;
			break;
		}
		if (tp > forge_dvfs_vproc || ts > forge_dvfs_vsram) {
			u8 s = min_t(u8, ts, forge_dvfs_vproc + FEB_MAX_DIFF);

			s = max(s, forge_dvfs_vsram);	/* never lower on the way up */
			if (s != forge_dvfs_vsram)
				ret = forge_dvfs_write_vsram(s);
			if (!ret && forge_dvfs_vproc != min(tp, s))
				ret = forge_dvfs_write_vproc(min(tp, s));
		} else {
			u8 p = max_t(u8, tp, forge_dvfs_vsram > FEB_MAX_DIFF ?
				     forge_dvfs_vsram - FEB_MAX_DIFF : 0);

			if (p != forge_dvfs_vproc)
				ret = forge_dvfs_write_vproc(p);
			if (!ret && forge_dvfs_vsram != max(ts, p))
				ret = forge_dvfs_write_vsram(max(ts, p));
		}
	}
	*vproc_rb = forge_vcode_uv10(forge_dvfs_vproc);
	*vsram_rb = forge_vcode_uv10(forge_dvfs_vsram);
	mutex_unlock(&forge_vstep_lock);
	return ret;
}
EXPORT_SYMBOL(forge_extbuck_set_rails);

/* the read-only dump, run by mt_cpufreq when it arms */
void forge_extbuck_sh0(void)
{
	forge_opp_dump();
}
EXPORT_SYMBOL(forge_extbuck_sh0);

static int forge_extbuck_probe_set(const char *val,
				   const struct kernel_param *kp)
{
	int n = 0;

	if (kstrtoint(val, 0, &n))
		return -EINVAL;

	if (n == 1)
		forge_extbuck_run();
	else if (n == 2)
		forge_opp_dump();
	else
		pr_err(FEB_TAG "op=%d ignored (1: CID probe, 2: Sh0 read-only dump)\n", n);

	return 0;
}

static const struct kernel_param_ops forge_extbuck_probe_ops = {
	.set = forge_extbuck_probe_set,
};

module_param_cb(forge_extbuck_probe, &forge_extbuck_probe_ops, NULL, 0200);
MODULE_PARM_DESC(forge_extbuck_probe,
		 "1: read MT6311 CID/SWCID; 2: DVFS Sh0 dump (bus3 late add, MT6311/MT6351/efuse/GPU/CPU, guard) - read-only, no rail is touched");
