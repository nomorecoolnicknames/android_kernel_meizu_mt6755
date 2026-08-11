/*
 * SM5414 switching charger driver (i2c1 @ 0x49), from the stock
 * drivers/misc/mediatek/power/mt6755/sm5414.c:
 *  - i2c: stock used the 3.10 MTK ext_flag/WRRD idiom; this tree's
 *    i2c_client has no ext_flag, so plain SMBus byte transfers are used
 *    (the i2c core builds and memsets its own i2c_msg).
 *  - registration: stock i2c_register_board_info(bus 1, 0x49) at
 *    subsys_initcall, kept; plus a bounded late_initcall fallback that
 *    i2c_new_device()s the chip if the adapter probed before us.
 *  - EINT: stock registered EINT101 via the DTS node
 *    "mediatek, SM5414_CHARGER-eint"; the DTB here has no such node, so
 *    charge-done is polled via INT2 (clear-on-read) from
 *    charging_hw_sm5414 instead. OF registration is still attempted and
 *    logged, so adding the DTS node later enables the IRQ path.
 *
 * Stock init policy kept in sm5414_reg_init(): INTMASK 0x3F/0xF8/0xFF,
 * comparator on, TOPOFF 150mA, BATREG 4.35V (4.35V cell,
 * HIGH_BATTERY_VOLTAGE_SUPPORT), AICL TH 4.4V, AUTOSTOP on, topoff timer
 * 10min.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <mt-plat/charging.h>
#include <mt-plat/mtk_gpio.h>

#include "sm5414.h"

#define FORGE_CHG_TAG "[FORGE_CHG][SM5414]"

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define sm5414_SLAVE_ADDR_WRITE	0x92	/* 7-bit 0x49 */

/* stock cust_i2c.h: I2C_SWITHING_CHARGER_CHANNEL = I2C_CHANNEL_1 */
#define sm5414_BUSNUM 1

static struct i2c_client *new_client;
static struct i2c_board_info __initdata i2c_sm5414 = {
	I2C_BOARD_INFO("sm5414", (sm5414_SLAVE_ADDR_WRITE >> 1))
};
static const struct i2c_device_id sm5414_i2c_id[] = { {"sm5414", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;

static int sm5414_driver_probe(struct i2c_client *client,
			       const struct i2c_device_id *id);

static int is_fullcharged;

static unsigned int sm5414_irq;

/* See sm5414_apply_int_latches for the reasoning behind these two. Both
 * are runtime-writable at /sys/module/sm5414/parameters/. */
static int forge_full_clear_wins = 1;
module_param(forge_full_clear_wins, int, 0644);
MODULE_PARM_DESC(forge_full_clear_wins,
	"m681 chg-full: 1=an INT sample carrying BOTH a full edge and a restart edge resolves to NOT-full (default), 0=stock if/else-if (full wins = one-way latch)");

static int forge_int_destructive_dump;
module_param(forge_int_destructive_dump, int, 0644);
MODULE_PARM_DESC(forge_int_destructive_dump,
	"m681 chg-full: 0=register dumps skip the clear-on-read INT1/INT2 latches and report the poll's shadow (default), 1=stock destructive dump");

#ifdef CONFIG_OF
static const struct of_device_id sm5414_of_match[] = {
	{ .compatible = "sm5414", },
	{},
};

MODULE_DEVICE_TABLE(of, sm5414_of_match);
#endif

static struct i2c_driver sm5414_driver = {
	.driver = {
		.name = "sm5414",
#ifdef CONFIG_OF
		.of_match_table = sm5414_of_match,
#endif
	},
	.probe = sm5414_driver_probe,
	.id_table = sm5414_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char sm5414_reg[SM5414_REG_NUM] = {0};

static DEFINE_MUTEX(sm5414_i2c_access);

int g_sm5414_hw_exist;

/* diag state, as in the stock driver's instrumentation */
struct m681_sm5414_diag_state {
	unsigned long read_count;
	unsigned long read_fail_count;
	unsigned long write_count;
	unsigned long write_fail_count;
	unsigned long irq_count;
	unsigned long probe_count;
	int probe_done;
	int active_adapter_nr;
	int active_addr;
	int last_read_cmd;
	int last_read_ret;
	int last_read_value;
	int last_write_cmd;
	int last_write_value;
	int last_write_ret;
	int last_irq_int1;
	int last_irq_int2;
	int last_irq_int3;
	int last_irq_status;
	/* Charge-done latch accounting. int_merged_count counts samples that
	 * carried a full edge and a restart edge at once, which an EINT would
	 * never produce. */
	unsigned long poll_count;
	unsigned long full_set_count;
	unsigned long full_clr_count;
	unsigned long int_merged_count;
};

static struct m681_sm5414_diag_state m681_sm5414_diag = {
	.active_adapter_nr = -1,
	.active_addr = -1,
	.last_read_cmd = -1,
	.last_read_ret = -ENODEV,
	.last_write_cmd = -1,
	.last_write_ret = -ENODEV,
};

/**********************************************************
  *
  *   [I2C Function For Read/Write sm5414]
  *
  *********************************************************/
int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
{
	int ret;

	m681_sm5414_diag.last_read_cmd = cmd;
	if (!new_client) {
		m681_sm5414_diag.last_read_ret = -ENODEV;
		m681_sm5414_diag.read_fail_count++;
		return 0;
	}

	mutex_lock(&sm5414_i2c_access);
	ret = i2c_smbus_read_byte_data(new_client, cmd);
	mutex_unlock(&sm5414_i2c_access);
	if (ret < 0) {
		m681_sm5414_diag.last_read_ret = ret;
		m681_sm5414_diag.read_fail_count++;
		return 0;
	}

	*returnData = (unsigned char)ret;
	m681_sm5414_diag.last_read_ret = 1;
	m681_sm5414_diag.last_read_value = *returnData;
	m681_sm5414_diag.read_count++;
	return 1;
}

int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
{
	int ret;

	m681_sm5414_diag.last_write_cmd = cmd;
	m681_sm5414_diag.last_write_value = writeData;
	if (!new_client) {
		m681_sm5414_diag.last_write_ret = -ENODEV;
		m681_sm5414_diag.write_fail_count++;
		return 0;
	}

	mutex_lock(&sm5414_i2c_access);
	ret = i2c_smbus_write_byte_data(new_client, cmd, writeData);
	mutex_unlock(&sm5414_i2c_access);
	m681_sm5414_diag.last_write_ret = ret;
	if (ret < 0) {
		m681_sm5414_diag.write_fail_count++;
		return 0;
	}

	m681_sm5414_diag.write_count++;
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int sm5414_read_interface(unsigned char RegNum, unsigned char *val,
				   unsigned char MASK, unsigned char SHIFT)
{
	unsigned char reg_val = 0;
	unsigned int ret;

	ret = sm5414_read_byte(RegNum, &reg_val);

	reg_val &= (MASK << SHIFT);
	*val = (reg_val >> SHIFT);

	return ret;
}

unsigned int sm5414_config_interface(unsigned char RegNum, unsigned char val,
				     unsigned char MASK, unsigned char SHIFT)
{
	unsigned char reg_val = 0;
	unsigned int ret;

	ret = sm5414_read_byte(RegNum, &reg_val);

	reg_val &= ~(MASK << SHIFT);
	reg_val |= (val << SHIFT);

	ret = sm5414_write_byte(RegNum, reg_val);

	return ret;
}

/* write one register directly */
unsigned int sm5414_reg_config_interface(unsigned char RegNum,
					 unsigned char val)
{
	return sm5414_write_byte(RegNum, val);
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CTRL ---------------------------------------------------- */
void sm5414_set_enboost(unsigned int val)
{
	sm5414_config_interface(SM5414_CTRL, (unsigned char)val,
				SM5414_CTRL_ENBOOST_MASK,
				SM5414_CTRL_ENBOOST_SHIFT);
}

void sm5414_set_chgen(unsigned int val)
{
	sm5414_config_interface(SM5414_CTRL, (unsigned char)val,
				SM5414_CTRL_CHGEN_MASK,
				SM5414_CTRL_CHGEN_SHIFT);
}

void sm5414_set_suspen(unsigned int val)
{
	sm5414_config_interface(SM5414_CTRL, (unsigned char)val,
				SM5414_CTRL_SUSPEN_MASK,
				SM5414_CTRL_SUSPEN_SHIFT);
}

void sm5414_set_reset(unsigned int val)
{
	sm5414_config_interface(SM5414_CTRL, (unsigned char)val,
				SM5414_CTRL_RESET_MASK,
				SM5414_CTRL_RESET_SHIFT);
}

void sm5414_set_encomparator(unsigned int val)
{
	sm5414_config_interface(SM5414_CTRL, (unsigned char)val,
				SM5414_CTRL_ENCOMPARATOR_MASK,
				SM5414_CTRL_ENCOMPARATOR_SHIFT);
}

/* VBUSCTRL ------------------------------------------------- */
void sm5414_set_vbuslimit(unsigned int val)
{
	sm5414_config_interface(SM5414_VBUSCTRL, (unsigned char)val,
				SM5414_VBUSCTRL_VBUSLIMIT_MASK,
				SM5414_VBUSCTRL_VBUSLIMIT_SHIFT);
}

/* CHGCTRL1 ------------------------------------------------- */
void sm5414_set_prechg(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL1, (unsigned char)val,
				SM5414_CHGCTRL1_PRECHG_MASK,
				SM5414_CHGCTRL1_PRECHG_SHIFT);
}

void sm5414_set_aiclen(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL1, (unsigned char)val,
				SM5414_CHGCTRL1_AICLEN_MASK,
				SM5414_CHGCTRL1_AICLEN_SHIFT);
}

void sm5414_set_autostop(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL1, (unsigned char)val,
				SM5414_CHGCTRL1_AUTOSTOP_MASK,
				SM5414_CHGCTRL1_AUTOSTOP_SHIFT);
}

void sm5414_set_aiclth(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL1, (unsigned char)val,
				SM5414_CHGCTRL1_AICLTH_MASK,
				SM5414_CHGCTRL1_AICLTH_SHIFT);
}

/* CHGCTRL2 ------------------------------------------------- */
void sm5414_set_fastchg(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL2, (unsigned char)val,
				SM5414_CHGCTRL2_FASTCHG_MASK,
				SM5414_CHGCTRL2_FASTCHG_SHIFT);
}

/* CHGCTRL3 ------------------------------------------------- */
void sm5414_set_weakbat(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL3, (unsigned char)val,
				SM5414_CHGCTRL3_WEAKBAT_MASK,
				SM5414_CHGCTRL3_WEAKBAT_SHIFT);
}

void sm5414_set_batreg(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL3, (unsigned char)val,
				SM5414_CHGCTRL3_BATREG_MASK,
				SM5414_CHGCTRL3_BATREG_SHIFT);
}

/* CHGCTRL4 ------------------------------------------------- */
void sm5414_set_dislimit(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL4, (unsigned char)val,
				SM5414_CHGCTRL4_DISLIMIT_MASK,
				SM5414_CHGCTRL4_DISLIMIT_SHIFT);
}

void sm5414_set_topoff(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL4, (unsigned char)val,
				SM5414_CHGCTRL4_TOPOFF_MASK,
				SM5414_CHGCTRL4_TOPOFF_SHIFT);
}

/* CHGCTRL5 ------------------------------------------------- */
void sm5414_set_topofftimer(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL5, (unsigned char)val,
				SM5414_CHGCTRL5_TOPOFFTIMER_MASK,
				SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT);
}

void sm5414_set_fasttimer(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL5, (unsigned char)val,
				SM5414_CHGCTRL5_FASTTIMER_MASK,
				SM5414_CHGCTRL5_FASTTIMER_SHIFT);
}

void sm5414_set_votg(unsigned int val)
{
	sm5414_config_interface(SM5414_CHGCTRL5, (unsigned char)val,
				SM5414_CHGCTRL5_VOTG_MASK,
				SM5414_CHGCTRL5_VOTG_SHIFT);
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
static void sm5414_hw_component_detect(void)
{
	unsigned int ret;
	unsigned char val = 0;

	/* stock detect: a live chip reads back nonzero at 0x0E; a NAK'd /
	 * absent chip leaves val at 0. */
	ret = sm5414_read_interface(0x0E, &val, 0xFF, 0x0);

	if (val == 0)
		g_sm5414_hw_exist = 0;
	else
		g_sm5414_hw_exist = 1;

	pr_notice(FORGE_CHG_TAG " hw_component_detect exist=%d, Reg[0x0E]=0x%x\n",
		  g_sm5414_hw_exist, val);
}

int is_sm5414_exist(void)
{
	return g_sm5414_hw_exist;
}

/* True for the two clear-on-read INT latches that
 * sm5414_poll_charge_done() relies on. A debug reader must not consume the
 * state it reports; see sm5414_dump_register(). */
static bool sm5414_reg_is_int_latch(int reg)
{
	return reg == SM5414_INT1 || reg == SM5414_INT2;
}

static unsigned char sm5414_int_shadow(int reg)
{
	return (unsigned char)(reg == SM5414_INT1 ?
			       m681_sm5414_diag.last_irq_int1 :
			       m681_sm5414_diag.last_irq_int2);
}

void sm5414_dump_register(void)
{
	int i;

	pr_notice(FORGE_CHG_TAG " regs:");
	for (i = 0; i < SM5414_REG_NUM; i++) {
		/* This dump runs on every battery tick
		 * (mt_battery_charging_algorithm() ends with CHARGING_CMD_DUMP_REGISTER,
		 * a few instructions after the same tick's charge-done poll), and
		 * 0x00/0x01 are clear-on-read. Reading them here would eat every edge
		 * the chip raises between the poll and the dump. Report the poll's
		 * shadow instead, labelled as such. forge_int_destructive_dump=1
		 * restores the stock read. */
		if (!forge_int_destructive_dump && sm5414_reg_is_int_latch(i)) {
			sm5414_reg[i] = sm5414_int_shadow(i);
			pr_notice(" [0x%x]=0x%x(shadow)", i, sm5414_reg[i]);
			continue;
		}
		sm5414_read_byte(i, &sm5414_reg[i]);
		pr_notice(" [0x%x]=0x%x", i, sm5414_reg[i]);
	}
	pr_notice("\n");
}

int sm5414_get_chrg_stat(void)
{
	return is_fullcharged;
}

/*
 * Fold one INT1/INT2 sample into is_fullcharged.
 *
 * INT1/INT2 are clear-on-read edge latches: a set bit means "this event
 * happened since the last read", and the byte carries no ordering between
 * the bits it presents. Stock consumed them from the EINT101 handler, i.e.
 * essentially one edge per read, so the if/else-if chain below could not
 * see a full edge and a restart edge in the same sample. Here there is no
 * "mediatek, SM5414_CHARGER-eint" DTB node (see sm5414_irq_registration()),
 * so the same expression is fed from a poll instead, one read per
 * battery-thread tick (BAT_TASK_PERIOD = 10s), and a 10s window merges
 * every edge the chip raised inside it.
 *
 * INT2 often reads 0x5 = SM5414_INT2_TOPOFF | SM5414_INT2_CHGRSTF, both
 * bits in one byte. With if/else-if the TOPOFF arm wins and the CHGRSTF
 * arm is never evaluated, so is_fullcharged stays 1, charging_full_check()
 * (switch_charging.c) reports full on every tick, and the algorithm sits
 * in CHR_BATFULL well below the programmed CV.
 *
 * The two directions are not symmetric, and that is what picks the fix:
 *  - "full" is absorbing. In CHR_BATFULL the algorithm stops re-running
 *    BAT_ConstantCurrentModeAction()/pchr_turn_on_charging(), the charge
 *    current stays under the 150 mA TOPOFF threshold, so TOPOFF keeps
 *    re-latching and the state re-confirms itself from its own consequence.
 *  - "not full" is self-correcting. charging_full_check() needs
 *    FULL_CHECK_TIMES (6) consecutive full samples to declare full again, so
 *    a genuinely full pack re-declares within a minute; and a genuinely full
 *    and settled pack raises no new edges at all (TOPOFF and CHGRSTF are
 *    both entry edges), leaving is_fullcharged at its last value untouched.
 * An ambiguous sample must therefore resolve toward not-full: the cost is at
 * most one minute of a late "full" indication, against a whole lost charging
 * session for the opposite bias.
 *
 * forge_full_clear_wins=0 restores the stock if/else-if.
 */
static void sm5414_apply_int_latches(unsigned char int1, unsigned char int2)
{
	bool set_full = (int2 & (SM5414_INT2_DONE | SM5414_INT2_TOPOFF)) != 0;
	bool clr_full = ((int1 & (SM5414_INT1_VBUSUVLO |
				  SM5414_INT1_VBUSOVP)) ||
			 (int2 & SM5414_INT2_CHGRSTF)) != 0;

	if (set_full && clr_full)
		m681_sm5414_diag.int_merged_count++;

	if (set_full) {
		is_fullcharged = 1;
		m681_sm5414_diag.full_set_count++;
	}

	/* Evaluated last so that a merged sample ends up cleared. Gating on
	 * !set_full reproduces the stock else-if exactly. */
	if (clr_full && (forge_full_clear_wins || !set_full)) {
		is_fullcharged = 0;
		m681_sm5414_diag.full_clr_count++;
	}
}

/*
 * Poll the (clear-on-read) INT latches for charge-done / restart. This
 * replaces the stock EINT101 handler; it is called from
 * charging_hw_sm5414's CHARGING_CMD_GET_CHARGING_STATUS, i.e. once per
 * battery-thread tick (10s), bounded, no busy loop.
 *
 * This is the only place allowed to consume INT1/INT2 on the poll path:
 * every other reader (sm5414_dump_register(), /proc/m681_charger_diag)
 * reports the shadow captured here instead of re-reading and destroying
 * it.
 */
int sm5414_poll_charge_done(void)
{
	unsigned char int1 = 0, int2 = 0;

	if (!new_client || !g_sm5414_hw_exist)
		return is_fullcharged;

	sm5414_read_byte(SM5414_INT1, &int1);
	sm5414_read_byte(SM5414_INT2, &int2);
	m681_sm5414_diag.last_irq_int1 = int1;
	m681_sm5414_diag.last_irq_int2 = int2;
	m681_sm5414_diag.poll_count++;

	sm5414_apply_int_latches(int1, int2);

	if (int1 || int2)
		pr_notice(FORGE_CHG_TAG " poll INT1=0x%x INT2=0x%x full=%d merged=%lu\n",
			  int1, int2, is_fullcharged,
			  m681_sm5414_diag.int_merged_count);

	return is_fullcharged;
}

static irqreturn_t sm5414_irq_handler(int irq, void *data)
{
	unsigned char int1 = 0, int2 = 0, int3 = 0;

	m681_sm5414_diag.irq_count++;
	sm5414_read_byte(SM5414_INT1, &int1);
	sm5414_read_byte(SM5414_INT2, &int2);
	sm5414_read_byte(SM5414_INT3, &int3);
	m681_sm5414_diag.last_irq_int1 = int1;
	m681_sm5414_diag.last_irq_int2 = int2;
	m681_sm5414_diag.last_irq_int3 = int3;

	pr_notice(FORGE_CHG_TAG " IRQ INT1=0x%x INT2=0x%x INT3=0x%x\n",
		  int1, int2, int3);

	/* Same folding rule as the poll. Unreachable today (no DTB eint node),
	 * but an IRQ that coalesces while threaded-handler-pending presents the
	 * identical merged sample, so the two paths must not diverge. */
	sm5414_apply_int_latches(int1, int2);

	return IRQ_HANDLED;
}

static int sm5414_irq_registration(void)
{
	struct device_node *node;
	int ret = 0;

	/* stock DTS node name (note the space after the comma, as the stock DTS
	 * spells it). Absent in the DTB here: INT polling is used instead, this is
	 * informational only. */
	node = of_find_compatible_node(NULL, NULL,
				       "mediatek, SM5414_CHARGER-eint");
	if (node) {
		sm5414_irq = irq_of_parse_and_map(node, 0);
		ret = request_threaded_irq(sm5414_irq, NULL,
					   sm5414_irq_handler,
					   IRQF_TRIGGER_FALLING |
					   IRQF_NO_SUSPEND | IRQF_ONESHOT,
					   "SM5414_CHARGER-eint", NULL);
		if (ret < 0)
			pr_notice(FORGE_CHG_TAG " request_irq %u failed (%d)\n",
				  sm5414_irq, ret);
		else
			pr_notice(FORGE_CHG_TAG " EINT irq %u armed\n",
				  sm5414_irq);
	} else {
		pr_notice(FORGE_CHG_TAG " no SM5414_CHARGER-eint DTS node; using INT polling\n");
		ret = -ENODEV;
	}
	return ret;
}

void sm5414_reg_init(void)
{
	/* INT MASK 1/2/3: unmask VBUSOVP|VBUSUVLO, DONE|TOPOFF|CHGRSTF */
	sm5414_write_byte(SM5414_INTMASK1, 0x3F);
	sm5414_write_byte(SM5414_INTMASK2, 0xF8);
	sm5414_write_byte(SM5414_INTMASK3, 0xFF);

	sm5414_set_encomparator(ENCOMPARATOR_EN);
	sm5414_set_topoff(TOPOFF_150mA);
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	sm5414_set_batreg(BATREG_4_3_5_0_V);	/* VREG 4.35V (4.35V cell) */
#else
	sm5414_set_batreg(BATREG_4_2_0_0_V);	/* VREG 4.2V */
#endif
	sm5414_set_aiclth(AICL_THRESHOLD_4_4_V);
#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
	sm5414_set_autostop(AUTOSTOP_EN);
	sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
	sm5414_set_autostop(AUTOSTOP_DIS);
#endif
}

static int sm5414_driver_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	pr_notice(FORGE_CHG_TAG " probe: adapter=%d addr=0x%x\n",
		  client->adapter ? client->adapter->nr : -1, client->addr);
	m681_sm5414_diag.probe_count++;
	m681_sm5414_diag.active_adapter_nr =
		client->adapter ? client->adapter->nr : -1;
	m681_sm5414_diag.active_addr = client->addr;

	new_client = client;

	/* nSHDN (GPIO4) high = chip running; EINT pin (GPIO101) input+pullup
	 * (stock DCT). LK leaves nSHDN in the charging state; writing 1 here is
	 * idempotent. */
	mt_set_gpio_mode((4 | 0x80000000), GPIO_MODE_GPIO);
	mt_set_gpio_dir((4 | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((4 | 0x80000000), GPIO_OUT_ONE);

	mt_set_gpio_mode((101 | 0x80000000), GPIO_MODE_GPIO);
	mt_set_gpio_dir((101 | 0x80000000), GPIO_DIR_IN);
	mt_set_gpio_pull_enable((101 | 0x80000000), GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select((101 | 0x80000000), GPIO_PULL_UP);

	sm5414_hw_component_detect();

	sm5414_reg_init();
	chargin_hw_init_done = KAL_TRUE;

	sm5414_irq_registration();

	sm5414_dump_register();
	m681_sm5414_diag.probe_done = 1;

	pr_notice(FORGE_CHG_TAG " probe DONE (hw_exist=%d)\n",
		  g_sm5414_hw_exist);

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API — sysfs register access]
  *
  *********************************************************/
static unsigned char g_reg_value_sm5414;
static ssize_t show_sm5414_access(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_reg_value_sm5414);
}

static ssize_t store_sm5414_access(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	if (buf != NULL && size != 0) {
		reg_address = simple_strtoul(buf, &pvalue, 16);

		if (size > 3) {
			reg_value = simple_strtoul((pvalue + 1), NULL, 16);
			pr_notice(FORGE_CHG_TAG " write reg 0x%x = 0x%x\n",
				  reg_address, reg_value);
			sm5414_config_interface(reg_address, reg_value,
						0xFF, 0x0);
		} else {
			sm5414_read_interface(reg_address,
					      &g_reg_value_sm5414, 0xFF, 0x0);
			pr_notice(FORGE_CHG_TAG " read reg 0x%x = 0x%x (cat sm5414_access)\n",
				  reg_address, g_reg_value_sm5414);
		}
	}
	return size;
}

static DEVICE_ATTR(sm5414_access, 0664, show_sm5414_access,
		   store_sm5414_access);

static int sm5414_user_space_probe(struct platform_device *dev)
{
	pr_notice(FORGE_CHG_TAG " user_space_probe\n");
	device_create_file(&(dev->dev), &dev_attr_sm5414_access);
	return 0;
}

struct platform_device sm5414_user_space_device = {
	.name = "sm5414-user",
	.id = -1,
};

static struct platform_driver sm5414_user_space_driver = {
	.probe = sm5414_user_space_probe,
	.driver = {
		.name = "sm5414-user",
	},
};

/**********************************************************
  *
  *   [/proc/m681_charger_diag]
  *
  *********************************************************/
static int m681_charger_diag_proc_show(struct seq_file *m, void *v)
{
	int i;

	seq_puts(m, "[M681][CHARGER] active driver: sm5414 (4.4 port)\n");
	seq_printf(m, "probe_done: %d\n", m681_sm5414_diag.probe_done);
	seq_printf(m, "probe_count: %lu\n", m681_sm5414_diag.probe_count);
	seq_printf(m, "adapter_addr: adapter=%d addr=0x%x\n",
		   m681_sm5414_diag.active_adapter_nr,
		   m681_sm5414_diag.active_addr);
	seq_printf(m, "hw_exist: %d chargin_hw_init_done=%d is_fullcharged=%d irq=%u\n",
		   g_sm5414_hw_exist, chargin_hw_init_done,
		   is_fullcharged, sm5414_irq);
	seq_printf(m, "reads: ok=%lu fail=%lu last_cmd=0x%x last_ret=%d last_val=0x%x\n",
		   m681_sm5414_diag.read_count,
		   m681_sm5414_diag.read_fail_count,
		   m681_sm5414_diag.last_read_cmd,
		   m681_sm5414_diag.last_read_ret,
		   m681_sm5414_diag.last_read_value);
	seq_printf(m, "writes: ok=%lu fail=%lu last_cmd=0x%x last_ret=%d last_val=0x%x\n",
		   m681_sm5414_diag.write_count,
		   m681_sm5414_diag.write_fail_count,
		   m681_sm5414_diag.last_write_cmd,
		   m681_sm5414_diag.last_write_ret,
		   m681_sm5414_diag.last_write_value);
	seq_printf(m, "irq: count=%lu int1=0x%x int2=0x%x int3=0x%x status=0x%x\n",
		   m681_sm5414_diag.irq_count,
		   m681_sm5414_diag.last_irq_int1,
		   m681_sm5414_diag.last_irq_int2,
		   m681_sm5414_diag.last_irq_int3,
		   m681_sm5414_diag.last_irq_status);
	/* merged > 0 with clear_wins=1 means the fix is doing work; merged > 0
	 * with clear_wins=0 is the stock behaviour. */
	seq_printf(m, "chgfull: polls=%lu set=%lu clr=%lu merged=%lu clear_wins=%d destructive_dump=%d\n",
		   m681_sm5414_diag.poll_count,
		   m681_sm5414_diag.full_set_count,
		   m681_sm5414_diag.full_clr_count,
		   m681_sm5414_diag.int_merged_count,
		   forge_full_clear_wins, forge_int_destructive_dump);

	seq_puts(m, "regs:");
	for (i = 0; i < SM5414_REG_NUM; i++) {
		unsigned char val = 0;
		int ok;

		/* Same clear-on-read hazard as sm5414_dump_register(): a plain
		 * `cat /proc/m681_charger_diag` would consume the TOPOFF/DONE/CHGRSTF
		 * edges the charge-done poll needs. Report the poll's shadow. */
		if (!forge_int_destructive_dump && sm5414_reg_is_int_latch(i)) {
			seq_printf(m, " [%02x]=shadow:0x%02x", i,
				   sm5414_int_shadow(i));
			continue;
		}
		ok = sm5414_read_byte((unsigned char)i, &val);
		seq_printf(m, " [%02x]=%s0x%02x", i, ok ? "" : "ERR:", val);
	}
	seq_putc(m, '\n');
	return 0;
}

static int m681_charger_diag_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, m681_charger_diag_proc_show, NULL);
}

static const struct file_operations m681_charger_diag_fops = {
	.owner = THIS_MODULE,
	.open = m681_charger_diag_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * Late fallback: if the i2c1 adapter registered before our board_info (or
 * the board_info path is bypassed by an OF-populated adapter), instantiate
 * the device directly. Bounded: single attempt, logged either way.
 */
static int __init sm5414_late_attach(void)
{
	struct i2c_adapter *adap;
	struct i2c_board_info info = {
		I2C_BOARD_INFO("sm5414", (sm5414_SLAVE_ADDR_WRITE >> 1))
	};

	if (proc_create("m681_charger_diag", 0444, NULL,
			&m681_charger_diag_fops))
		pr_notice(FORGE_CHG_TAG " /proc/m681_charger_diag created\n");

	if (new_client) {
		pr_notice(FORGE_CHG_TAG " late_attach: already probed, ok\n");
		return 0;
	}

	adap = i2c_get_adapter(sm5414_BUSNUM);
	if (!adap) {
		pr_notice(FORGE_CHG_TAG " late_attach: i2c%d adapter missing\n",
			  sm5414_BUSNUM);
		return 0;
	}

	if (!i2c_new_device(adap, &info))
		pr_notice(FORGE_CHG_TAG " late_attach: i2c_new_device failed\n");
	else
		pr_notice(FORGE_CHG_TAG " late_attach: device created on i2c%d\n",
			  sm5414_BUSNUM);
	i2c_put_adapter(adap);
	return 0;
}
late_initcall(sm5414_late_attach);

static int __init sm5414_subsys_init(void)
{
	int ret;

	pr_notice(FORGE_CHG_TAG " init start. ch=%d addr=0x%x\n",
		  sm5414_BUSNUM, sm5414_SLAVE_ADDR_WRITE >> 1);
	i2c_register_board_info(sm5414_BUSNUM, &i2c_sm5414, 1);
	if (i2c_add_driver(&sm5414_driver) != 0)
		pr_notice(FORGE_CHG_TAG " failed to register i2c driver\n");

	/* sysfs register-access interface */
	ret = platform_device_register(&sm5414_user_space_device);
	if (ret) {
		pr_notice(FORGE_CHG_TAG " device register failed (%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&sm5414_user_space_driver);
	if (ret) {
		pr_notice(FORGE_CHG_TAG " driver register failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit sm5414_exit(void)
{
	i2c_del_driver(&sm5414_driver);
}

subsys_initcall(sm5414_subsys_init);
module_exit(sm5414_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5414 Driver (m681 4.4 port)");
