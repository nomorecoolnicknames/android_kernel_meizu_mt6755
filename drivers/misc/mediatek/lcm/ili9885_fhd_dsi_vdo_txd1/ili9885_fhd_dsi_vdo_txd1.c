#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#else
/* Raw pad API for the TPS65132 bias EN pads: when the kernel inherits the
 * panel from LK nothing owns ENP (GPIO12) / ENN (GPIO7), and ENP driven low
 * leaves AVDD dead (gray/striped/black panel). */
#include <mt-plat/mtk_gpio.h>
#include <mach/gpio_const.h>
#include <linux/workqueue.h>
#include <linux/m3note_board.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif

#ifndef MACH_FPGA
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_ILI9885A	0x9885

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
		lcm_util.set_gpio_lcd_enn_bias(cmd)

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
//#define LCM_DSI_CMD_MODE									1
#define LCM_DSI_CMD_MODE									0

#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(1920)

#ifndef BUILD_LK
#define M681_TXD1_AUTO_DCS_READBACK 0
#endif

#if !defined(MACH_FPGA) && defined(CONFIG_MTK_LEGACY)
#define GPIO_65132_EN1 GPIO_LCD_BIAS_ENP_PIN
#define GPIO_65132_EN2 GPIO_LCD_BIAS_ENN_PIN
#endif

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

//the same i2c-addr can't to probe two times.
#ifdef BUILD_LK
extern int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value);
#else
extern int disp_bls_set_backlight(int level_1024);
#endif

/* ----------------------------------------------------------------------- */
/* M681 self-contained TPS65132 bias I2C provider.                         */
/* The m6-base tree builds only this single panel, so the TPS65132 bias    */
/* driver that used to be supplied by a sibling panel must live here.      */
/* Transplanted from the proven ili9881p_hd_dsi_txd.c (non-legacy path).   */
/* ----------------------------------------------------------------------- */
#ifndef BUILD_LK
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>

#if defined(CONFIG_M3NOTE_UNIFIED)
/* M681/L681 kernel: one TPS65132 provider for both panels,
 * lcm/tps65132/tps65132.c (same compatible, same two-byte write); two
 * drivers on one i2c_lcd_bias node would leave one panel without bias. */
extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
#elif !defined(CONFIG_FPGA_EARLY_PORTING)
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,i2c_lcd_bias"},
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

struct i2c_client *tps65132_i2c_client;

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ili9885_fhd_dsi_vdo_txd1",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
		   },
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LCM_LOGI("tps65132_iic_probe\n");
	tps65132_i2c_client = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	if (tps65132_i2c_client == client)
		tps65132_i2c_client = NULL;
	return 0;
}

int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { 0 };

	if (!client) {
		LCM_LOGI("tps65132 i2c client is not ready !!\n");
		return -1;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_LOGI("tps65132 write data fail !!\n");
	return ret;
}

static int __init tps65132_iic_init(void)
{
	int ret;

	LCM_LOGI("tps65132_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
#endif
	ret = i2c_add_driver(&tps65132_iic_driver);
	LCM_LOGI("tps65132_iic_init add_driver ret=%d\n", ret);
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	i2c_del_driver(&tps65132_iic_driver);
}

module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_DESCRIPTION("MTK TPS65132 I2C Driver (m681 ili9885)");
MODULE_LICENSE("GPL");
#endif /* CONFIG_FPGA_EARLY_PORTING */
#endif /* BUILD_LK */

#ifndef BUILD_LK
struct m681_txd1_diag_state {
	unsigned long init_power_calls;
	unsigned long resume_power_calls;
	unsigned long suspend_power_calls;
	unsigned long suspend_calls;
	unsigned long resume_calls;
	unsigned long suspend_preserve_bias_calls;
	unsigned long bias_enable_calls;
	unsigned long lk_handoff_bias_calls;
	unsigned long tps_write_failures;
	int tps_write00_ret;
	int tps_write01_ret;
	int tps_write00_attempts;
	int tps_write01_attempts;
	int lk_handoff_tps00_ret;
	int lk_handoff_tps01_ret;
	int lk_handoff_tps00_attempts;
	int lk_handoff_tps01_attempts;
	unsigned long late_bias_scheduled;
	unsigned long late_bias_runs;
	unsigned long late_bias_skips;
	unsigned long late_bias_manual;
	unsigned long late_bias_last_jiffies;
	unsigned int late_bias_last_delay_ms;
	unsigned int late_bias_last_skip_reason;
	unsigned int bias_enp;
	unsigned int bias_enn;
	unsigned int reset_low_before_bias;
	unsigned int dcs_attempted;
	unsigned int dcs_util_missing;
	unsigned long dcs_auto_read_skips;
	unsigned long dcs_on_demand_calls;
	unsigned long dcs_read_calls;
	unsigned long dcs_read_failures;
	unsigned long panel_wake_calls;
	unsigned long panel_wake_manual;
	unsigned long panel_wake_last_jiffies;
	unsigned long refinit_calls;
	unsigned long refinit_manual;
	unsigned long refinit_last_jiffies;
	int refinit_tps00_ret;
	int refinit_tps01_ret;
	int refinit_tps00_attempts;
	int refinit_tps01_attempts;
	int dcs_0a_ret;
	int dcs_0c_ret;
	int dcs_0d_ret;
	int dcs_0f_ret;
	unsigned char dcs_0a;
	unsigned char dcs_0c;
	unsigned char dcs_0d;
	unsigned char dcs_0f;
	unsigned long backlight_cmdq_calls;
	unsigned int backlight_last_level;
	int backlight_last_ret;
};

static struct m681_txd1_diag_state m681_txd1_diag;

static void m681_txd1_dcs_readback(void);
static void m681_txd1_auto_dcs_readback(const char *reason)
{
#if M681_TXD1_AUTO_DCS_READBACK
	m681_txd1_dcs_readback();
#else
	m681_txd1_diag.dcs_auto_read_skips++;
	LCM_LOGI("[M681][TXD1][DCS] auto readback disabled reason=%s skips=%lu\n",
		 reason ? reason : "unknown",
		 m681_txd1_diag.dcs_auto_read_skips);
#endif
}
#endif

#ifndef MACH_FPGA
static void m681_txd1_enable_bias_gpios(void)
{
#if defined(CONFIG_MTK_LEGACY)
	mt_set_gpio_mode(GPIO_65132_EN1, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN1, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN1, GPIO_OUT_ONE);
	MDELAY(5);

	mt_set_gpio_mode(GPIO_65132_EN2, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN2, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN2, GPIO_OUT_ONE);
	MDELAY(15);
#else
	set_gpio_lcd_enp(1);
	MDELAY(5);
	set_gpio_lcd_enn(1);
	MDELAY(15);
#endif

#ifndef BUILD_LK
	m681_txd1_diag.bias_enable_calls++;
	m681_txd1_diag.bias_enp = 1;
	m681_txd1_diag.bias_enn = 1;
#endif
	LCM_LOGI("[M681][TXD1][BIAS] ENP/ENN asserted before TPS65132 writes\n");
}

static int m681_txd1_tps_write_retry(unsigned char cmd, unsigned char data,
				     int *attempts_out)
{
	int attempt;
	int ret = -1;

	for (attempt = 1; attempt <= 5; attempt++) {
#ifdef BUILD_LK
		ret = TPS65132_write_byte(cmd, data);
#else
		ret = tps65132_write_bytes(cmd, data);
#endif
		if (ret >= 0) {
			LCM_LOGI("[M681][TXD1][BIAS] TPS65132 reg=0x%02x data=0x%02x ok ret=%d attempt=%d\n",
				 cmd, data, ret, attempt);
			break;
		}

		LCM_LOGI("[M681][TXD1][BIAS] TPS65132 reg=0x%02x data=0x%02x fail ret=%d attempt=%d\n",
			 cmd, data, ret, attempt);
		MDELAY(5);
	}

	if (attempts_out)
		*attempts_out = attempt > 5 ? 5 : attempt;

#ifndef BUILD_LK
	if (ret < 0)
		m681_txd1_diag.tps_write_failures++;
#endif
	return ret;
}
#endif

#ifndef BUILD_LK
void m681_txd1_ensure_lk_handoff_bias(const char *reason)
{
#ifndef MACH_FPGA
	int attempts00 = 0;
	int attempts01 = 0;
	int ret00;
	int ret01;
	const char *why = reason ? reason : "unknown";

	m681_txd1_diag.lk_handoff_bias_calls++;
	m681_txd1_enable_bias_gpios();
	ret00 = m681_txd1_tps_write_retry(0x00, 0x0F, &attempts00);
	ret01 = m681_txd1_tps_write_retry(0x01, 0x0F, &attempts01);

	m681_txd1_diag.tps_write00_ret = ret00;
	m681_txd1_diag.tps_write01_ret = ret01;
	m681_txd1_diag.tps_write00_attempts = attempts00;
	m681_txd1_diag.tps_write01_attempts = attempts01;
	m681_txd1_diag.lk_handoff_tps00_ret = ret00;
	m681_txd1_diag.lk_handoff_tps01_ret = ret01;
	m681_txd1_diag.lk_handoff_tps00_attempts = attempts00;
	m681_txd1_diag.lk_handoff_tps01_attempts = attempts01;
	m681_txd1_diag.dcs_attempted = 0;

	LCM_LOGI("[M681][TXD1][LKBIAS] reason=%s ret00=%d/%d ret01=%d/%d no-reset no-init\n",
		 why, ret00, attempts00, ret01, attempts01);
#else
	m681_txd1_diag.lk_handoff_bias_calls++;
	LCM_LOGI("[M681][TXD1][LKBIAS] reason=%s skipped on MACH_FPGA\n",
		 reason ? reason : "unknown");
#endif
}

#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};
static struct LCM_setting_table init_setting[] =
{
    {0xB0,3, {0x98,0x85,0x0A}},               
    /*
     * First D6 byte 0x80, as in the LK init table (D6,12 = 80 00 08 17 23
     * 65 77 44 87 00 00 09); the 3.18/3.10 txd1 sources have 0x00. The
     * F5-paged block below is in the LK table too.
     */
    {0xD6,12, {0x80,0x00,0x08,0x17,0x23,0x65,0x77,0x44,0x87,0x00,0x00,0x09}},
    {0x35,1, {0x00}},  

    {0xF5,1, {0x85}},	
    {0xB9,1, {0x02}},	

    {0xF5,1, {0x8D}},	
    {0xEF,1, {0x02}},	
    
    {0xF5,1, {0x9B}},	
    {0xD7,1, {0xA0}},	
    
    {0xF5,1, {0x9C}},	
    {0xD7,1, {0x40}},	    

    {0xF5,1, {0x8B}},	
    {0xD0,1, {0x0E}},	
    {0xF5,1, {0x00}},	
	
    {0x11,1, {0x00}},
    {REGFLAG_DELAY, 120, {}},        
    {0x29,1, {0x00}},  
    {REGFLAG_END_OF_TABLE, 0x00, {}}
	
};

static struct LCM_setting_table lcm_sleep_in_setting[] =
{
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

#ifndef BUILD_LK
static int m681_txd1_dcs_read(unsigned char cmd, unsigned char *value)
{
	unsigned char buf[4] = { 0xff, 0xff, 0xff, 0xff };
	int ret;

	ret = read_reg_v2(cmd, buf, 1);
	*value = buf[0];
	m681_txd1_diag.dcs_read_calls++;
	if (ret <= 0)
		m681_txd1_diag.dcs_read_failures++;

	pr_alert("[M681][TXD1][DCS] read cmd=0x%02x ret=%d byte=0x%02x\n",
		 cmd, ret, buf[0]);
	return ret;
}

static void m681_txd1_dcs_readback(void)
{
	m681_txd1_diag.dcs_attempted = 1;

	m681_txd1_diag.dcs_0a = 0xff;
	m681_txd1_diag.dcs_0c = 0xff;
	m681_txd1_diag.dcs_0d = 0xff;
	m681_txd1_diag.dcs_0f = 0xff;

	if (!lcm_util.dsi_dcs_read_lcm_reg_v2) {
		m681_txd1_diag.dcs_util_missing = 1;
		m681_txd1_diag.dcs_read_failures++;
		pr_alert("[M681][TXD1][DCS] readback skipped: util missing\n");
		return;
	}
	m681_txd1_diag.dcs_util_missing = 0;

	m681_txd1_diag.dcs_0a_ret =
		m681_txd1_dcs_read(0x0A, &m681_txd1_diag.dcs_0a);
	m681_txd1_diag.dcs_0c_ret =
		m681_txd1_dcs_read(0x0C, &m681_txd1_diag.dcs_0c);
	m681_txd1_diag.dcs_0d_ret =
		m681_txd1_dcs_read(0x0D, &m681_txd1_diag.dcs_0d);
	m681_txd1_diag.dcs_0f_ret =
		m681_txd1_dcs_read(0x0F, &m681_txd1_diag.dcs_0f);

	pr_alert("[M681][TXD1][DCS] summary 0a=%d/0x%02x 0c=%d/0x%02x 0d=%d/0x%02x 0f=%d/0x%02x\n",
		 m681_txd1_diag.dcs_0a_ret, m681_txd1_diag.dcs_0a,
		 m681_txd1_diag.dcs_0c_ret, m681_txd1_diag.dcs_0c,
		 m681_txd1_diag.dcs_0d_ret, m681_txd1_diag.dcs_0d,
		 m681_txd1_diag.dcs_0f_ret, m681_txd1_diag.dcs_0f);
}

#endif

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

#if !defined(BUILD_LK) && !defined(MACH_FPGA) && !defined(CONFIG_MTK_LEGACY)
/* Bias guard (stock DCT): ENP = GPIO12 (AVDD +5.5V), ENN = GPIO7 (AVEE
 * -5.5V), TPS65132 at i2c0 0x3e. The EN pads are re-checked every period
 * and restored; counters go to dmesg. */
#define M681_LCD_BIAS_ENP_PIN 12
#define M681_LCD_BIAS_ENN_PIN 7
#define M681_BIAS_GUARD_FIRST_MS  2000
#define M681_BIAS_GUARD_PERIOD_MS 15000

static struct delayed_work m681_bias_guard_work;
static unsigned long m681_bias_guard_runs, m681_bias_guard_restores;

static int m681_bias_pad_good(unsigned long pin)
{
	return mt_get_gpio_mode(pin) == GPIO_MODE_00 &&
	       mt_get_gpio_dir(pin) == GPIO_DIR_OUT &&
	       mt_get_gpio_out(pin) == GPIO_OUT_ONE;
}

static void m681_bias_assert_pads(void)
{
	mt_set_gpio_mode(M681_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(M681_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(M681_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	mt_set_gpio_mode(M681_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(M681_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(M681_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
}

static void m681_bias_guard_fn(struct work_struct *work)
{
	int enp_ok = m681_bias_pad_good(M681_LCD_BIAS_ENP_PIN);
	int enn_ok = m681_bias_pad_good(M681_LCD_BIAS_ENN_PIN);

	m681_bias_guard_runs++;
	if (!enp_ok || !enn_ok) {
		m681_bias_guard_restores++;
		pr_err("[M681][TXD1][BIASGUARD44] pad regression ENP_ok=%d ENN_ok=%d run=%lu restores=%lu — restoring\n",
		       enp_ok, enn_ok, m681_bias_guard_runs,
		       m681_bias_guard_restores);
		m681_bias_assert_pads();
	}
	schedule_delayed_work(&m681_bias_guard_work,
			      msecs_to_jiffies(M681_BIAS_GUARD_PERIOD_MS));
}

static void m681_bias_guard_arm(void)
{
	static int armed;

	if (armed)
		return;
	armed = 1;
	INIT_DELAYED_WORK(&m681_bias_guard_work, m681_bias_guard_fn);
	/* The periodic poll is not scheduled on 4.4: m681_bias_pad_good reads
	 * GPIO12 (ENP) / GPIO7 (ENN) through mt_get_gpio_mode/dir/out on every
	 * tick, and mt_gpio_pin_decrypt WARNs on the raw (unflagged) pin form
	 * (~6 WARNs per tick); MTK AEE picks up the WARN storm and the RGU
	 * resets the device. The work is still initialised (keeps
	 * m681_bias_guard_fn referenced) but never armed. To re-enable it, pass
	 * 0x80000000-flagged pins so mt_gpio_pin_decrypt does not WARN. */
	pr_err("[M681][TXD1][BIASGUARD44] #133 periodic poll DISABLED (WARN-flood/AEE reset test)\n");
}
#endif /* !BUILD_LK && !MACH_FPGA && !CONFIG_MTK_LEGACY */

static void lcm_get_params(LCM_PARAMS *params)
{
#if !defined(BUILD_LK) && !defined(MACH_FPGA) && !defined(CONFIG_MTK_LEGACY)
	m681_bias_guard_arm();
#endif
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
    	params->physical_width = 68;
	params->physical_height = 121;
	/*
	 * BURST video mode: SYNC_PULSE mode fails before the boot logo handoff and
	 * leaves Android without VDO words. Used together with the DCS-read
	 * suppression, the timing/HSTX settings and HSTX_CKLP.
	 */
	params->dsi.mode   					= BURST_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 0;

	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size=256;

	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active	= 8;
	params->dsi.vertical_backporch		= 16;
	params->dsi.vertical_frontporch		= 150;	/* V-blank slack for RDMA prefetch (~61 fps scanout) */
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 8;
	params->dsi.horizontal_backporch	= 80;
	params->dsi.horizontal_frontporch	= 80;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	params->dsi.cont_clock = 1;
	params->dsi.clk_lp_per_line_enable = 0;
	/*
	 * TXD1 DCS reads require a VDO->CMD->VDO window and current dumps show
	 * RX_LPDT/ESC_SYNC residue after those reads. Keep normal boot scanout
	 * free of automatic ESD reads; use /proc/m681_txd1_dcs_read_now only
	 * when DCS readback itself is the target.
	 */
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
//	params->dsi.lcm_esd_check_table[0].cmd = 0xaf;
//	params->dsi.lcm_esd_check_table[0].count = 1;
//	params->dsi.lcm_esd_check_table[0].para_list[0] = 0xad;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#if 1
	params->dsi.lcm_esd_check_table[1].cmd = 0xE5;
	params->dsi.lcm_esd_check_table[1].count =10;//10;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x36;
	params->dsi.lcm_esd_check_table[1].para_list[1] = 0x36;
	params->dsi.lcm_esd_check_table[1].para_list[2] = 0xA1;
	params->dsi.lcm_esd_check_table[1].para_list[3] = 0xF6;

	params->dsi.lcm_esd_check_table[1].para_list[4] = 0xF6;
	params->dsi.lcm_esd_check_table[1].para_list[5] = 0x47;
	params->dsi.lcm_esd_check_table[1].para_list[6] = 0x07;
	params->dsi.lcm_esd_check_table[1].para_list[7] = 0x55;
	params->dsi.lcm_esd_check_table[1].para_list[8] = 0x15;
	params->dsi.lcm_esd_check_table[1].para_list[9] = 0x63;

	params->dsi.lcm_esd_check_table[2].cmd = 0x0B;  //0B
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00; //00

	
	params->dsi.lcm_esd_check_table[3].cmd = 0xD2;
	params->dsi.lcm_esd_check_table[3].count = 4;
	params->dsi.lcm_esd_check_table[3].para_list[0] = 0x03;
	params->dsi.lcm_esd_check_table[3].para_list[1] = 0x03;
	params->dsi.lcm_esd_check_table[3].para_list[2] = 0x2A;
	params->dsi.lcm_esd_check_table[3].para_list[3] = 0x22;

#endif
	/*
	 * M681 default boot diagnostics show DSI CMDQ latched on the
	 * TXD1 ESD DCS-read shape (0x37 + 0x0a) while explicit DCS reads are
	 * disabled. Keep TXD1 ESD reads fully disabled unless a future
	 * on-demand debug path intentionally asks for DCS readback.
	 */
	memset(params->dsi.lcm_esd_check_table, 0,
	       sizeof(params->dsi.lcm_esd_check_table));

	params->dsi.PLL_CLOCK=480;
}
static void lcm_init_power(void)
{
#ifndef MACH_FPGA
	int attempts00 = 0;
	int attempts01 = 0;
	int ret00;
	int ret01;
#endif

	SET_RESET_PIN(0);
	MDELAY(10);
#ifndef BUILD_LK
	m681_txd1_diag.init_power_calls++;
	m681_txd1_diag.reset_low_before_bias = 1;
#endif
#ifndef MACH_FPGA
	m681_txd1_enable_bias_gpios();
	ret00 = m681_txd1_tps_write_retry(0x00, 0x0F, &attempts00);
	ret01 = m681_txd1_tps_write_retry(0x01, 0x0F, &attempts01);
#ifndef BUILD_LK
	m681_txd1_diag.tps_write00_ret = ret00;
	m681_txd1_diag.tps_write01_ret = ret01;
	m681_txd1_diag.tps_write00_attempts = attempts00;
	m681_txd1_diag.tps_write01_attempts = attempts01;
#endif
	MDELAY(10);
#endif
    LCM_LOGI("[M681][TXD1][BIAS] init_power done\n");
}

static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
	m681_txd1_diag.suspend_power_calls++;
#endif
    LCM_LOGI("ili9885_fhd_dsi_vdo_txd is called\n");
}

static void lcm_resume_power(void)
{
#ifndef BUILD_LK
	m681_txd1_diag.resume_power_calls++;
#endif
    lcm_init_power();
    LCM_LOGI("ili9885_fhd_dsi_vdo_txd is called\n");
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0); 
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);	

	push_table(init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
#ifndef BUILD_LK
	m681_txd1_auto_dcs_readback("lcm_init");
#endif
	LCM_LOGI("ili9885_fhd_dsi_vdo_txd1 is init\n");
}

static void lcm_suspend(void)
{
#ifndef BUILD_LK
	m681_txd1_diag.suspend_calls++;
#endif
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

#ifndef BUILD_LK
	/*
	 * m3 showed a runtime suspend/resume before late diagnostics. Do not
	 * drop reset/bias during Android display blanking; it can kill the
	 * TXD1 handoff while the DDP/DSI path remains active.
	 */
	m681_txd1_diag.suspend_preserve_bias_calls++;
	LCM_LOGI("[M681][TXD1][SUSPEND] preserve reset/bias on runtime suspend count=%lu\n",
		 m681_txd1_diag.suspend_preserve_bias_calls);
	return;
#endif

	SET_RESET_PIN(0);
	MDELAY(5);
#if defined(CONFIG_MTK_LEGACY)
	mt_set_gpio_mode(GPIO_65132_EN2, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN2, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN2, GPIO_OUT_ZERO);
    	MDELAY(2);
	mt_set_gpio_mode(GPIO_65132_EN1, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN1, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN1, GPIO_OUT_ZERO);
	MDELAY(1);
#else
	set_gpio_lcd_enn(0);
	MDELAY(2);
	set_gpio_lcd_enp(0);
	MDELAY(1);
#endif
#ifndef BUILD_LK
	m681_txd1_diag.bias_enp = 0;
	m681_txd1_diag.bias_enn = 0;
#endif
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
	m681_txd1_diag.resume_calls++;
#endif
	lcm_init();	
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
#ifndef BUILD_LK
	int ret;

	(void)handle;
	ret = disp_bls_set_backlight(level);
	m681_txd1_diag.backlight_cmdq_calls++;
	m681_txd1_diag.backlight_last_level = level;
	m681_txd1_diag.backlight_last_ret = ret;
	LCM_LOGI("[M681][TXD1][BL] set_backlight_cmdq level=%u ret=%d calls=%lu\n",
		 level, ret, m681_txd1_diag.backlight_cmdq_calls);
#else
	(void)handle;
	LCM_LOGI("[M681][TXD1][BL] LK set_backlight_cmdq level=%u skipped\n",
		 level);
#endif
}

static unsigned int lcm_compare_id(void)
{
  
    return 0;
}

#ifndef BUILD_LK
static int m681_txd1_diag_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "[M681][TXD1] persistent LCM/bias diag\n");
	seq_printf(m, "init_power_calls=%lu resume_power_calls=%lu suspend_power_calls=%lu\n",
		   m681_txd1_diag.init_power_calls,
		   m681_txd1_diag.resume_power_calls,
		   m681_txd1_diag.suspend_power_calls);
	seq_printf(m, "lcm_runtime suspend_calls=%lu resume_calls=%lu\n",
		   m681_txd1_diag.suspend_calls,
		   m681_txd1_diag.resume_calls);
	seq_printf(m, "runtime_suspend_policy=preserve_reset_bias preserve_calls=%lu\n",
		   m681_txd1_diag.suspend_preserve_bias_calls);
	seq_printf(m, "bias_enable_calls=%lu bias_enp=%u bias_enn=%u reset_low_before_bias=%u\n",
		   m681_txd1_diag.bias_enable_calls,
		   m681_txd1_diag.bias_enp,
		   m681_txd1_diag.bias_enn,
		   m681_txd1_diag.reset_low_before_bias);
	seq_printf(m, "lk_handoff_bias_calls=%lu tps00_ret=%d attempts=%d tps01_ret=%d attempts=%d\n",
		   m681_txd1_diag.lk_handoff_bias_calls,
		   m681_txd1_diag.lk_handoff_tps00_ret,
		   m681_txd1_diag.lk_handoff_tps00_attempts,
		   m681_txd1_diag.lk_handoff_tps01_ret,
		   m681_txd1_diag.lk_handoff_tps01_attempts);
	seq_printf(m, "late_bias scheduled=%lu runs=%lu skips=%lu manual=%lu last_jiffies=%lu last_delay_ms=%u skip_reason=%u\n",
		   m681_txd1_diag.late_bias_scheduled,
		   m681_txd1_diag.late_bias_runs,
		   m681_txd1_diag.late_bias_skips,
		   m681_txd1_diag.late_bias_manual,
		   m681_txd1_diag.late_bias_last_jiffies,
		   m681_txd1_diag.late_bias_last_delay_ms,
		   m681_txd1_diag.late_bias_last_skip_reason);
	seq_printf(m, "tps_write00_ret=%d attempts=%d tps_write01_ret=%d attempts=%d failures=%lu\n",
		   m681_txd1_diag.tps_write00_ret,
		   m681_txd1_diag.tps_write00_attempts,
		   m681_txd1_diag.tps_write01_ret,
		   m681_txd1_diag.tps_write01_attempts,
		   m681_txd1_diag.tps_write_failures);
	seq_printf(m, "dcs_attempted=%u auto_skips=%lu on_demand=%lu util_missing=%u calls=%lu failures=%lu\n",
		   m681_txd1_diag.dcs_attempted,
		   m681_txd1_diag.dcs_auto_read_skips,
		   m681_txd1_diag.dcs_on_demand_calls,
		   m681_txd1_diag.dcs_util_missing,
		   m681_txd1_diag.dcs_read_calls,
		   m681_txd1_diag.dcs_read_failures);
	seq_printf(m, "panel_wake calls=%lu manual=%lu last_jiffies=%lu\n",
		   m681_txd1_diag.panel_wake_calls,
		   m681_txd1_diag.panel_wake_manual,
		   m681_txd1_diag.panel_wake_last_jiffies);
	seq_printf(m, "refinit calls=%lu manual=%lu last_jiffies=%lu tps00_ret=%d attempts=%d tps01_ret=%d attempts=%d\n",
		   m681_txd1_diag.refinit_calls,
		   m681_txd1_diag.refinit_manual,
		   m681_txd1_diag.refinit_last_jiffies,
		   m681_txd1_diag.refinit_tps00_ret,
		   m681_txd1_diag.refinit_tps00_attempts,
		   m681_txd1_diag.refinit_tps01_ret,
		   m681_txd1_diag.refinit_tps01_attempts);
	seq_printf(m, "dcs_0a_ret=%d byte=0x%02x dcs_0c_ret=%d byte=0x%02x\n",
		   m681_txd1_diag.dcs_0a_ret,
		   m681_txd1_diag.dcs_0a,
		   m681_txd1_diag.dcs_0c_ret,
		   m681_txd1_diag.dcs_0c);
	seq_printf(m, "dcs_0d_ret=%d byte=0x%02x dcs_0f_ret=%d byte=0x%02x\n",
		   m681_txd1_diag.dcs_0d_ret,
		   m681_txd1_diag.dcs_0d,
		   m681_txd1_diag.dcs_0f_ret,
		   m681_txd1_diag.dcs_0f);
	seq_printf(m, "backlight_cmdq calls=%lu last_level=%u last_ret=%d note=disp_lcm_requires_set_backlight_cmdq\n",
		   m681_txd1_diag.backlight_cmdq_calls,
		   m681_txd1_diag.backlight_last_level,
		   m681_txd1_diag.backlight_last_ret);
	seq_puts(m, "expected_success_ret=2 for each TPS65132 write; negative ret means I2C ACK/power path is still broken\n");
	seq_puts(m, "DCS 0x0A=0x9c means panel command path is alive; ret<=0/0xff now means read-window/panel-command path is still unproven below SF/HWC/fbdev\n");
	return 0;
}

static int m681_txd1_dcs_read_proc_show(struct seq_file *m, void *v)
{
	m681_txd1_diag.dcs_on_demand_calls++;
	m681_txd1_dcs_readback();
	seq_puts(m, "[M681][TXD1] explicit DCS readback requested\n");
	return m681_txd1_diag_proc_show(m, v);
}

static int m681_txd1_diag_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, m681_txd1_diag_proc_show, NULL);
}

static int m681_txd1_dcs_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, m681_txd1_dcs_read_proc_show, NULL);
}

static const struct file_operations m681_txd1_diag_fops = {
	.owner = THIS_MODULE,
	.open = m681_txd1_diag_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations m681_txd1_dcs_read_fops = {
	.owner = THIS_MODULE,
	.open = m681_txd1_dcs_read_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init m681_txd1_diag_proc_init(void)
{
	/* these procs talk DCS to the ILI9885 panel */
	if (!m3note_board_is(M3NOTE_BOARD_M681))
		return 0;
	if (proc_create("m681_txd1_diag", 0444, NULL, &m681_txd1_diag_fops))
		pr_alert("[M681][TXD1][DIAG] /proc/m681_txd1_diag created\n");
	else
		pr_alert("[M681][TXD1][DIAG] /proc/m681_txd1_diag create FAILED\n");
	if (proc_create("m681_txd1_dcs_read_now", 0444, NULL,
			&m681_txd1_dcs_read_fops))
		pr_alert("[M681][TXD1][DIAG] /proc/m681_txd1_dcs_read_now created\n");
	else
		pr_alert("[M681][TXD1][DIAG] /proc/m681_txd1_dcs_read_now create FAILED\n");
	pr_alert("[M681][TXD1][MUTATE] manual TXD1 late_bias/panel_wake/refinit proc triggers disabled after v40 pipeline stop\n");
	return 0;
}
late_initcall(m681_txd1_diag_proc_init);

/*
 * M6-base primary_display.c carries Meizu-specific ili9881p stock-page diag
 * probes and calls them unconditionally.  The m681 ili9885 panel does not
 * implement those ili9881p register pages, so provide no-op stubs to satisfy
 * the vmlinux link.  (Replaces the symbols formerly exported by
 * ili9881p_hd_dsi_txd.c, which is no longer built.)
 */
void lcm_m6_diag_read_stock_pages(void)
{
	LCM_LOGD("[M681][TXD1][DIAG] lcm_m6_diag_read_stock_pages stub (ili9881p-only)\n");
}

void lcm_m6_diag_page5_2a_probe(unsigned int value, unsigned int hold_ms)
{
	LCM_LOGD("[M681][TXD1][DIAG] lcm_m6_diag_page5_2a_probe stub value=0x%x hold=%u\n",
		 value, hold_ms);
}

void lcm_m6_diag_mode_ctrl_probe(unsigned int value, unsigned int hold_ms)
{
	LCM_LOGD("[M681][TXD1][DIAG] lcm_m6_diag_mode_ctrl_probe stub value=0x%x hold=%u\n",
		 value, hold_ms);
}
#endif

LCM_DRIVER ili9885_fhd_dsi_vdo_txd_lcm_drv1 = {
	.name = "ili9885_fhd_dsi_vdo_txd1",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
};
