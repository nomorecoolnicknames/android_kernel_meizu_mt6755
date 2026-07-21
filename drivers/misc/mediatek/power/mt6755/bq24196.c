/*
 * TI BQ24196 switching charger, L681 (hq6755_66_b1a_l) 4.4 port of the 3.10
 * driver (drivers/misc/mediatek/power/mt6755/bq24196.c).
 *
 * Changes from 3.10: the MTK-private i2c ext_flag read/write became
 * i2c_smbus_read/write_byte_data; the legacy i2c_register_board_info(1,
 * 0x6b) became a device-tree bind to the node DrvGen emits from the L681
 * DWS (&i2c1 { swithing_charger@6b { compatible =
 * "mediatek,swithing_charger"; }; }); kal_* types became u8/u32. The
 * register helpers (bq24196_set_* / _get_*) are the 3.10 ones verbatim.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mt-plat/charging.h>

#include <linux/m3note_board.h>
#include "bq24196.h"

static struct i2c_client *new_client;
/* declared kal_bool (32-bit) in mt-plat/charging.h; battery_common_fg_20.c reads it */
#ifndef CONFIG_M3NOTE_UNIFIED
kal_bool chargin_hw_init_done = KAL_FALSE;
#endif

u8 bq24196_reg[bq24196_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq24196_i2c_access);

int bq24196_read_byte(u8 cmd, u8 *returnData)
{
	int ret;

	if (!new_client)
		return 0;
	mutex_lock(&bq24196_i2c_access);
	ret = i2c_smbus_read_byte_data(new_client, cmd);
	mutex_unlock(&bq24196_i2c_access);
	if (ret < 0) {
		pr_err("[bq24196] read reg 0x%x failed %d\n", cmd, ret);
		return 0;
	}
	*returnData = (u8)ret;
	return 1;
}

int bq24196_write_byte(u8 cmd, u8 writeData)
{
	int ret;

	if (!new_client)
		return 0;
	mutex_lock(&bq24196_i2c_access);
	ret = i2c_smbus_write_byte_data(new_client, cmd, writeData);
	mutex_unlock(&bq24196_i2c_access);
	if (ret < 0) {
		pr_err("[bq24196] write reg 0x%x=0x%x failed %d\n", cmd, writeData, ret);
		return 0;
	}
	return 1;
}

u32 bq24196_read_interface(u8 RegNum, u8 *val, u8 MASK,
				  u8 SHIFT)
{
	u8 bq24196_reg = 0;
	int ret = 0;

	pr_debug("--------------------------------------------------\n");

	ret = bq24196_read_byte(RegNum, &bq24196_reg);
	pr_debug("[bq24196_read_interface] Reg[%x]=0x%x\n", RegNum, bq24196_reg);

	bq24196_reg &= (MASK << SHIFT);
	*val = (bq24196_reg >> SHIFT);
	pr_debug("[bq24196_read_interface] Val=0x%x\n", *val);

	return ret;
}

u32 bq24196_config_interface(u8 RegNum, u8 val, u8 MASK,
				    u8 SHIFT)
{
	u8 bq24196_reg = 0;
	int ret = 0;

	pr_debug("--------------------------------------------------\n");

	ret = bq24196_read_byte(RegNum, &bq24196_reg);
	pr_debug("[bq24196_config_interface] Reg[%x]=0x%x\n", RegNum, bq24196_reg);

	bq24196_reg &= ~(MASK << SHIFT);
	bq24196_reg |= (val << SHIFT);

	ret = bq24196_write_byte(RegNum, bq24196_reg);
	pr_debug("[bq24196_config_interface] Write Reg[%x]=0x%x\n", RegNum, bq24196_reg);

	/* Check */
	 bq24196_read_byte(RegNum, &bq24196_reg); 
	pr_debug("[bq24196_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24196_reg);

	return ret;
}

/* write one register directly */
u32 bq24196_reg_config_interface(u8 RegNum, u8 val)
{
	u32 ret = 0;

	ret = bq24196_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq24196_set_en_hiz(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON0),
				       (u8) (val),
				       (u8) (CON0_EN_HIZ_MASK),
				       (u8) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq24196_set_vindpm(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON0),
				       (u8) (val),
				       (u8) (CON0_VINDPM_MASK),
				       (u8) (CON0_VINDPM_SHIFT)
	    );
}

void bq24196_set_iinlim(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON0),
				       (u8) (val),
				       (u8) (CON0_IINLIM_MASK),
				       (u8) (CON0_IINLIM_SHIFT)
	    );
}

/* CON1---------------------------------------------------- */

void bq24196_set_reg_rst(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_REG_RST_MASK),
				       (u8) (CON1_REG_RST_SHIFT)
	    );
}

void bq24196_set_wdt_rst(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_WDT_RST_MASK),
				       (u8) (CON1_WDT_RST_SHIFT)
	    );
}

void bq24196_set_otg_config(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_OTG_CONFIG_MASK),
				       (u8) (CON1_OTG_CONFIG_SHIFT)
	    );
}


void bq24196_set_chg_config(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_CHG_CONFIG_MASK),
				       (u8) (CON1_CHG_CONFIG_SHIFT)
	    );
}


void bq24196_set_sys_min(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_SYS_MIN_MASK),
				       (u8) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq24196_set_boost_lim(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON1),
				       (u8) (val),
				       (u8) (CON1_BOOST_LIM_MASK),
				       (u8) (CON1_BOOST_LIM_SHIFT)
	    );
}

/* CON2---------------------------------------------------- */

void bq24196_set_ichg(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON2),
				       (u8) (val),
				       (u8) (CON2_ICHG_MASK), (u8) (CON2_ICHG_SHIFT)
	    );
}

void bq24196_set_force_20pct(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON2),
				       (u8) (val),
				       (u8) (CON2_FORCE_20PCT_MASK),
				       (u8) (CON2_FORCE_20PCT_SHIFT)
	    );
}

/* CON3---------------------------------------------------- */

void bq24196_set_iprechg(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON3),
				       (u8) (val),
				       (u8) (CON3_IPRECHG_MASK),
				       (u8) (CON3_IPRECHG_SHIFT)
	    );
}

void bq24196_set_iterm(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON3),
				       (u8) (val),
				       (u8) (CON3_ITERM_MASK), (u8) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq24196_set_vreg(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON4),
				       (u8) (val),
				       (u8) (CON4_VREG_MASK), (u8) (CON4_VREG_SHIFT)
	    );
}

void bq24196_set_batlowv(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON4),
				       (u8) (val),
				       (u8) (CON4_BATLOWV_MASK),
				       (u8) (CON4_BATLOWV_SHIFT)
	    );
}

void bq24196_set_vrechg(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON4),
				       (u8) (val),
				       (u8) (CON4_VRECHG_MASK),
				       (u8) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq24196_set_en_term(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON5),
				       (u8) (val),
				       (u8) (CON5_EN_TERM_MASK),
				       (u8) (CON5_EN_TERM_SHIFT)
	    );
}

void bq24196_set_term_stat(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON5),
				       (u8) (val),
				       (u8) (CON5_TERM_STAT_MASK),
				       (u8) (CON5_TERM_STAT_SHIFT)
	    );
}

void bq24196_set_watchdog(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON5),
				       (u8) (val),
				       (u8) (CON5_WATCHDOG_MASK),
				       (u8) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq24196_set_en_timer(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON5),
				       (u8) (val),
				       (u8) (CON5_EN_TIMER_MASK),
				       (u8) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq24196_set_chg_timer(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON5),
				       (u8) (val),
				       (u8) (CON5_CHG_TIMER_MASK),
				       (u8) (CON5_CHG_TIMER_SHIFT)
	    );
}

/* CON6---------------------------------------------------- */

void bq24196_set_treg(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON6),
				       (u8) (val),
				       (u8) (CON6_TREG_MASK), (u8) (CON6_TREG_SHIFT)
	    );
}

/* CON7---------------------------------------------------- */

void bq24196_set_tmr2x_en(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON7),
				       (u8) (val),
				       (u8) (CON7_TMR2X_EN_MASK),
				       (u8) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq24196_set_batfet_disable(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON7),
				       (u8) (val),
				       (u8) (CON7_BATFET_Disable_MASK),
				       (u8) (CON7_BATFET_Disable_SHIFT)
	    );
}

void bq24196_set_int_mask(u32 val)
{
	u32 ret = 0;

	ret = bq24196_config_interface((u8) (bq24196_CON7),
				       (u8) (val),
				       (u8) (CON7_INT_MASK_MASK),
				       (u8) (CON7_INT_MASK_SHIFT)
	    );
}

/* CON8---------------------------------------------------- */

u32 bq24196_get_system_status(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = bq24196_read_interface((u8) (bq24196_CON8),
				     (&val), (u8) (0xFF), (u8) (0x0)
	    );
	return val;
}

u32 bq24196_get_vbus_stat(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = bq24196_read_interface((u8) (bq24196_CON8),
				     (&val),
				     (u8) (CON8_VBUS_STAT_MASK),
				     (u8) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

u32 bq24196_get_chrg_stat(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = bq24196_read_interface((u8) (bq24196_CON8),
				     (&val),
				     (u8) (CON8_CHRG_STAT_MASK),
				     (u8) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

u32 bq24196_get_vsys_stat(void)
{
	u32 ret = 0;
	u8 val = 0;

	ret = bq24196_read_interface((u8) (bq24196_CON8),
				     (&val),
				     (u8) (CON8_VSYS_STAT_MASK),
				     (u8) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq24196_dump_register(void)
{
	u8 i = 0;

	/* one line, debug level: the battery thread asks for this every
	 * cycle */
	for (i = 0; i < bq24196_REG_NUM; i++)
		bq24196_read_byte(i, &bq24196_reg[i]);
	pr_debug("[bq24196] %*ph\n", bq24196_REG_NUM, bq24196_reg);
}


static int bq24196_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 reg0a = 0;

	new_client = client;
	/* REG0A vendor/part/revision: BQ24196 reads 0x2x (PN[5:3] = 100 on the
	 * 24190 family, see the TI datasheet) - logged, not enforced. */
	bq24196_read_byte(0x0A, &reg0a);
	pr_info("[bq24196] probe addr=0x%x REG0A=0x%02x\n", client->addr, reg0a);
	bq24196_dump_register();
	chargin_hw_init_done = KAL_TRUE;
	return 0;
}

static void bq24196_shutdown(struct i2c_client *client)
{
	bq24196_set_chg_config(0x0);	/* charger disable, as the 3.10 driver */
}

static const struct of_device_id bq24196_of_match[] = {
	{ .compatible = "mediatek,swithing_charger" },	/* DrvGen name for i2c1@0x6b */
	{ .compatible = "ti,bq24196" },
	{},
};
MODULE_DEVICE_TABLE(of, bq24196_of_match);

static const struct i2c_device_id bq24196_i2c_id[] = { { "bq24196", 0 }, {} };

static struct i2c_driver bq24196_driver = {
	.driver = {
		.name = "bq24196",
		.of_match_table = bq24196_of_match,
	},
	.probe = bq24196_driver_probe,
	.shutdown = bq24196_shutdown,
	.id_table = bq24196_i2c_id,
};

static int __init bq24196_subsys_init(void)
{
	/* the M681 DCT also emits swithing_charger@6b; its charger is SM5414 */
	if (!m3note_board_is(M3NOTE_BOARD_L681))
		return 0;
	return i2c_add_driver(&bq24196_driver);
}
subsys_initcall(bq24196_subsys_init);

MODULE_DESCRIPTION("TI BQ24196 charger (l681)");
MODULE_LICENSE("GPL");
