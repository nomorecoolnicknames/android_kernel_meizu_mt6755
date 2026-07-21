/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/types.h>
#include <mt-plat/aee.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#else
#include "disp_dts_gpio.h"
#endif

#include "m4u.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"

#include "lcm_drv.h"
#include "ddp_path.h"
#include "ddp_reg.h"
#include "ddp_drv.h"
#include "ddp_wdma.h"
#include "ddp_hal.h"
#include "ddp_aal.h"
#include "ddp_pwm.h"
#include "ddp_dither.h"
#include "ddp_info.h"
#include "ddp_dsi.h"
#include "ddp_ovl.h"
#include "ddp_rdma.h"
#include "ddp_manager.h"
#include "ddp_met.h"
#include "disp_log.h"
#include "disp_debug.h"
#include "disp_helper.h"
#include "disp_drv_ddp.h"
#include "disp_recorder.h"
#include "disp_session.h"
#include "disp_lowpower.h"
#include "disp_recovery.h"
#include "disp_assert_layer.h"
#include "mtkfb.h"
#include "mtkfb_fence.h"
#include "mtkfb_debug.h"
#include "primary_display.h"
#include "ddp_clkmgr.h"	/* dsi_clk_on debug command (ddp_clk_prepare_enable) */

#pragma GCC optimize("O0")

/* --------------------------------------------------------------------------- */
/* Global variable declarations */
/* --------------------------------------------------------------------------- */
char DDP_STR_HELP[] =
	"USAGE:\n"
	"       echo [ACTION]>/d/dispsys\n"
	"ACTION:\n"
	"       regr:addr\n              :regr:0xf400c000\n"
	"       regw:addr,value          :regw:0xf400c000,0x1\n"
	"       dbg_log:0|1|2            :0 off, 1 dbg, 2 all\n"
	"       irq_log:0|1              :0 off, !0 on\n"
	"       met_on:[0|1],[0|1],[0|1] :fist[0|1]on|off,other [0|1]direct|decouple\n"
	"       backlight:level\n"
	"       dump_aal:arg\n"
	"       mmp\n"
	"       dump_reg:moduleID\n"
	"       dump_path:mutexID\n"
	"       dpfd_ut1:channel\n";

char MTKFB_STR_HELP[] =
	"\n"
	"USAGE\n"
	"        echo [ACTION]... > /d/mtkfb\n"
	"\n"
	"ACTION\n"
	"        mtkfblog:[on|off]\n"
	"             enable/disable [MTKFB] log\n"
	"\n"
	"        displog:[on|off]\n"
	"             enable/disable [DISP] log\n"
	"\n"
	"        mtkfb_vsynclog:[on|off]\n"
	"             enable/disable [VSYNC] log\n"
	"\n"
	"        log:[on|off]\n"
	"             enable/disable above all log\n"
	"\n"
	"        fps:[on|off]\n"
	"             enable fps and lcd update time log\n"
	"\n"
	"        tl:[on|off]\n"
	"             enable touch latency log\n"
	"\n"
	"        layer\n"
	"             dump lcd layer information\n"
	"\n"
	"        suspend\n"
	"             enter suspend mode\n"
	"\n"
	"        resume\n"
	"             leave suspend mode\n"
	"\n"
	"        lcm:[on|off|init]\n"
	"             power on/off lcm\n"
	"\n"
	"        m6_lcm_reinit:[0|1]\n"
	"             Meizu M6 diagnostic Linux LCM init after boot\n"
	"\n"
	"        m6_dsi_dcs_status[:stock_pages]\n"
	"             Meizu M6 diagnostic DSI/ILI9881P DCS status dump\n"
	"        m6_dsi_dcs_status_force[:tag]\n"
	"             Meizu M6 force one manual DCS status dump even after boot limits\n"
	"        m6_dsi_hs_window:<tag>[:hold_ms]\n"
	"             Meizu M6 bounded DSI HS-video IRQ/VM/window sampler\n"
	"        m6_dsi_phy_truth[:tag]\n"
	"             Meizu M6 read-only DSI/MIPITX/LCM lane and PHY truth dump\n"
		"        m6_dsi_debug_mux[:tag]\n"
		"             Meizu M6 bounded DSI/MIPITX debug mux sweep; restores selectors\n"
		"        m6_dsi_debug_mux_stats[:tag[:samples[:delay_us]]]\n"
		"             Meizu M6 DSI/MIPITX debug mux multi-sample stats; restores selectors\n"
		"        m6_dsi_lkgold_muxstats_dump\n"
		"             Meizu M6 dump cached early LK handoff MIPITX mux stats\n"
	"        m6_dsi_clk_restore[:tag]\n"
	"             Meizu M6 force TXRX HSTX_CKLP_EN and PHY LC_HS_TX_EN back on\n"
	"        m6_dsi_pll_change:<pll>[:tag]\n"
	"             Meizu M6 runtime MIPITX PLL reprogram (pll=230 stock, 240/250/265 test); live link, revert by writing old value\n"
		"        m6_dsi_cc_probe:<0|1>[:hold_ms[:restore[:mux]]]\n"
		"             Meizu M6 isolation toggle for TXRX HSTX_CKLP_EN; mux=1 sweep, mux=2 stats\n"
		"        m6_dsi_lc_hs_probe:<0|1>[:hold_ms[:restore[:mux]]]\n"
		"             Meizu M6 isolation toggle for PHY LC_HS_TX_EN; mux=1 sweep, mux=2 stats\n"
		"        m6_dsi_mipitx_pad_window[:tag[:samples[:delay_ms]]]\n"
		"             Meizu M6 read-only repeated MIPITX pad/top/lane sampler\n"
		"        m6_dsi_mipitx_pad_probe:<field>:<value>[:hold_ms[:restore[:mux]]]\n"
		"             Meizu M6 restore-safe MIPITX field isolation; mux=1 sweep, 2 stats, 3 pad window\n"
		"        m6_dsi_mipitx_lane_group_probe:<rt|lptx|lpcd>:<value>[:hold_ms[:restore[:mux]]]\n"
		"             Meizu M6 restore-safe all-lane MIPITX analog-field isolation; mux=1 sweep, 2 stats, 3 pad window\n"
		"        m6_dsi_mipitx_phy_sel_probe:<value>[:hold_ms[:restore[:mux]]]\n"
		"             Meizu M6 restore-safe MIPITX PHY_SEL lane-map isolation; mux=1 sweep, 2 stats, 3 pad window\n"
		"        m6_dsi_mipitx_plltop_probe:<preserve>[:hold_ms[:restore[:mux[:shift]]]]\n"
		"             Meizu M6 restore-safe MIPITX PLL_TOP preserve isolation; shift defaults to 8, shift=7 tests local bitfield\n"
		"        m6_dsi_wrtrace_dump[:limit]\n"
		"             Meizu M6 dump first DSI0/MIPITX register write-order trace\n"
		"        m6_dsi_wrtrace_reset[:enable]\n"
		"             Meizu M6 clear DSI0/MIPITX write-order trace and set capture state\n"
	"        m6_dsi_wrtrace_enable:<0|1>\n"
	"             Meizu M6 enable/disable DSI0/MIPITX write-order capture\n"
	"        m6_dsi_hsa_wc:<value>[:hold_ms]\n"
	"             Meizu M6 isolation override for DSI_HSA_WC with snapshots\n"
	"        m6_dsi_vm_cmd_probe:<raw>[:hold_ms[:restore[:mux]]]\n"
	"             Meizu M6 restore-safe VM_CMD_CON isolation; try 0xff511501 to clear TS_VFP_EN\n"
	"        m6_dsi_bist_profile:<profile>:<rgb>[:hold_ms]\n"
	"             Meizu M6 manual DSI BIST profile sweep; auto-disables\n"
	"        m6_lcm_page5_2a:<value>[:hold_ms]\n"
	"             Meizu M6 isolation write/read probe for ILI9881P page5 cmd 0x2A\n"
	"        m6_lcm_mode_ctrl:<value>[:hold_ms]\n"
	"             Meizu M6 isolation write/read probe for ILI9881P cmd 0xBB mode control\n"
	"        m6_dsi_c2v_switch:<value>[:hold_ms]\n"
	"             Meizu M6 diagnostic DDP DSI C2V switch path probe using cmd 0xBB\n"
	"\n"
	"\n"
	"        m6_display_truth_window[:tag]\n"
	"             Meizu M6 read-only DDP/OVL/RDMA/DSI/MIPITX/backlight truth dump\n"
	"        m6_display_route_probe[:dump|trigger|rekick|mask]\n"
	"             Meizu M6 DDP route probe with optional manual trigger/trigger-loop rekick\n"
	"\n"
	"        m6_ovl_greq_profile:[0|1|2|3]\n"
	"             Meizu M6 isolation profiles for OVL RDMA/GREQ underflow triage\n"
	"\n"
	"        m6_ovl_bounds_profile:[0|1]\n"
	"             Meizu M6 isolation profile for OVL end-prefetch/M4U boundary triage\n"
	"\n"
	"        m6_ovl_stale_cpu_clear:[0|1]\n"
	"             Meizu M6 isolation switch for CPU mirroring stale disabled-layer clears\n"
	"\n"
	"        cabc:[ui|mov|still]\n"
	"             cabc mode, UI/Moving picture/Still picture\n"
	"\n"
	"        lcd:[on|off]\n"
	"             power on/off display engine\n"
	"\n"
	"        te:[on|off]\n"
	"             turn on/off tearing-free control\n"
	"\n"
	"        tv:[on|off]\n"
	"             turn on/off tv-out\n"
	"\n"
	"        tvsys:[ntsc|pal]\n"
	"             switch tv system\n"
	"\n"
	"        reg:[lcd|dpi|dsi|tvc|tve]\n"
	"             dump hw register values\n"
	"\n"
	"        regw:addr=val\n"
	"             write hw register\n"
	"\n"
	"        regr:addr\n"
	"             read hw register\n"
	"\n"
	"       cpfbonly:[on|off]\n"
	"             capture UI layer only on/off\n"
	"\n"
	"       esd:[on|off]\n"
	"             esd kthread on/off\n"
	"       HQA:[NormalToFactory|FactoryToNormal]\n"
	"             for HQA requirement\n"
	"\n"
	"       mmp\n"
	"             Register MMProfile events\n"
	"\n"
	"       dump_fb:[on|off[,down_sample_x[,down_sample_y,[delay]]]]\n"
	"             Start/end to capture framebuffer every delay(ms)\n"
	"\n"
	"       dump_ovl:[on|off[,down_sample_x[,down_sample_y]]]\n"
	"             Start to capture OVL only once\n"
	"\n"
	"       dump_layer:[on|off[,down_sample_x[,down_sample_y]][,layer(0:L0,1:L1,2:L2,3:L3,4:L0-3)]\n"
	"             Start/end to capture current enabled OVL layer every frame\n";

/* --------------------------------------------------------------------------- */
/* Local variable declarations */
/* --------------------------------------------------------------------------- */
static struct dentry *lowpowermode_debugfs;
static struct dentry *kickdump_debugfs;
static int low_power_cust_mode = LP_CUST_DISABLE;
static unsigned int vfp_backup;
static char LP_CUST_STR_HELP[] =
	"USAGE:\n"
	"       echo [ACTION]>/d/disp/lowpowermode\n"
	"ACTION:\n"
	"       low_power_mode:Mode\n"
	"		Mode:0(LP_CUST_DISABLE)|1(LOW_POWER_MODE)|2(JUST_MAKE_MODE)|3(PERFORMANC_MODE)\n";

static void disp_m6_copy_tag(char *dst, size_t dst_size, const char *tag)
{
	const char *src = tag ? tag : "manual";
	const char fallback[] = "manual";
	size_t i = 0;

	if (!dst_size)
		return;

	while (i + 1 < dst_size && src[i] &&
	       src[i] != '\n' && src[i] != '\r' &&
	       src[i] != ' ' && src[i] != '\t') {
		dst[i] = src[i];
		i++;
	}
	dst[i] = '\0';

	if (dst[0])
		return;

	for (i = 0; i + 1 < dst_size && fallback[i]; i++)
		dst[i] = fallback[i];
	dst[i] = '\0';
}

/* --------------------------------------------------------------------------- */
/* DDP and MTKFB Command Processor */
/* --------------------------------------------------------------------------- */
void ddp_process_dbg_opt(const char *opt)
{
	int ret = 0;
	char *buf = dbg_buf + strlen(dbg_buf);

	if (0 == strncmp(opt, "regr:", 5)) {
		char *p = (char *)opt + 5;
		unsigned long addr = 0;

		ret = kstrtoul(p, 16, &addr);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal = DISP_REG_GET(addr);

			DISPMSG("regr: 0x%lx = 0x%08X\n", addr, regVal);
			sprintf(buf, "regr: 0x%lx = 0x%08X\n", addr, regVal);
		} else {
			sprintf(buf, "regr, invalid address 0x%lx\n", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "lfr_update", 3)) {
		DSI_LFR_UPDATE(DISP_MODULE_DSI0, NULL);
	} else if (0 == strncmp(opt, "regw:", 5)) {
		unsigned long addr;
		unsigned int val;

		ret = sscanf(opt, "regw:0x%lx,0x%x\n", &addr, &val);
		if (ret != 2) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal;

			DISP_CPU_REG_SET(addr, val);
			regVal = DISP_REG_GET(addr);
			DISPMSG("regw: 0x%lx, 0x%08X = 0x%08X\n", addr, val, regVal);
			sprintf(buf, "regw: 0x%lx, 0x%08X = 0x%08X\n", addr, val, regVal);
		} else {
			sprintf(buf, "regw, invalid address 0x%lx\n", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "dbg_log:", 8)) {
		char *p = (char *)opt + 8;
		unsigned int enable;

		ret = kstrtouint(p, 0, &enable);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (enable) {
			dbg_log_level = 1;
			g_fencelog = 1;
			g_loglevel = 5;
		} else {
			dbg_log_level = 0;
			g_fencelog = 0;
			g_loglevel = 3;
		}

		sprintf(buf, "dbg_log: %d\n", dbg_log_level);
	} else if (0 == strncmp(opt, "irq_log:", 8)) {
		char *p = (char *)opt + 8;
		unsigned int enable;

		ret = kstrtouint(p, 0, &enable);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}
		if (enable) {
			irq_log_level = 1;
			g_loglevel = 6;
		} else {
			irq_log_level = 0;
			g_loglevel = 3;
		}

		sprintf(buf, "irq_log: %d\n", irq_log_level);
	} else if (0 == strncmp(opt, "met_on:", 7)) {
		int met_on, rdma0_mode, rdma1_mode;

		ret = sscanf(opt, "met_on:%d,%d,%d\n", &met_on, &rdma0_mode, &rdma1_mode);
		if (ret != 3) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}
		ddp_init_met_tag(met_on, rdma0_mode, rdma1_mode);
		DISPMSG("process_dbg_opt, met_on=%d,rdma0_mode %d, rdma1 %d\n", met_on, rdma0_mode,
		       rdma1_mode);
		sprintf(buf, "met_on:%d,rdma0_mode:%d,rdma1_mode:%d\n", met_on, rdma0_mode,
			rdma1_mode);
	} else if (0 == strncmp(opt, "backlight:", 10)) {
		char *p = (char *)opt + 10;
		unsigned int level;

		ret = kstrtouint(p, 0, &level);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (level) {
			/*disp_bls_set_backlight(level);*/
			sprintf(buf, "backlight: %d\n", level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "pwm0:", 5) || 0 == strncmp(opt, "pwm1:", 5)) {
		char *p = (char *)opt + 5;
		unsigned int level;

		ret = kstrtouint(p, 0, &level);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (level) {
			disp_pwm_id_t pwm_id = DISP_PWM0;

			if (opt[3] == '1')
				pwm_id = DISP_PWM1;

			/*disp_pwm_set_backlight(pwm_id, level);*/
			sprintf(buf, "PWM 0x%x : %d\n", pwm_id, level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "rdma_color:", 11)) {
			unsigned int red, green, blue;

			rdma_color_matrix matrix;
			rdma_color_pre pre = { 0 };
			rdma_color_post post = { 255, 0, 0 };

			memset(&matrix, 0, sizeof(matrix));

			ret = sscanf(opt, "rdma_color:%d,%d,%d\n", &red, &green, &blue);
			if (ret != 3) {
				snprintf(buf, 50, "error to parse cmd %s\n", opt);
				return;
			}

			post.ADD0 = red;
			post.ADD1 = green;
			post.ADD2 = blue;
			rdma_set_color_matrix(DISP_MODULE_RDMA0, &matrix, &pre, &post);
			rdma_enable_color_transform(DISP_MODULE_RDMA0);
	} else if (0 == strncmp(opt, "rdma_color:off", 14)) {
		rdma_disable_color_transform(DISP_MODULE_RDMA0);
	} else if (0 == strncmp(opt, "aal_dbg:", 8)) {
		char *p = (char *)opt + 8;

		ret = kstrtouint(p, 0, &aal_dbg_en);
		if (ret) {
				snprintf(buf, 50, "error to parse cmd %s\n", opt);
				return;
		}
		sprintf(buf, "aal_dbg_en = 0x%x\n", aal_dbg_en);
	} else if (0 == strncmp(opt, "aal_test:", 9)) {
		aal_test(opt + 9, buf);
	} else if (0 == strncmp(opt, "pwm_test:", 9)) {
		disp_pwm_test(opt + 9, buf);
	} else if (0 == strncmp(opt, "dither_test:", 12)) {
		dither_test(opt + 12, buf);
	} else if (0 == strncmp(opt, "corr_dbg:", 9)) {
		int i;

		i = 0;
	} else if (0 == strncmp(opt, "dump_reg:", 9)) {
		char *p = (char *)opt + 9;
		unsigned int module;

		ret = kstrtouint(p, 0, &module);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		DISPMSG("process_dbg_opt, module=%d\n", module);
		if (module < DISP_MODULE_NUM) {
			ddp_dump_reg(module);
			sprintf(buf, "dump_reg: %d\n", module);
		} else {
			DISPMSG("process_dbg_opt2, module=%d\n", module);
			goto Error;
		}
	} else if (0 == strncmp(opt, "dump_path:", 10)) {
		char *p = (char *)opt + 10;
		unsigned int mutex_idx;

		ret = kstrtouint(p, 0, &mutex_idx);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		DISPMSG("process_dbg_opt, path mutex=%d\n", mutex_idx);
		dpmgr_debug_path_status(mutex_idx);
		sprintf(buf, "dump_path: %d\n", mutex_idx);

	} else if (0 == strncmp(opt, "get_module_addr", 15)) {
		unsigned int i = 0;
		char *buf_temp = buf;

		for (i = 0; i < DISP_REG_NUM; i++) {
			DISPDMP("i=%d, module=%s, va=0x%lx, pa=0x%lx, irq(%d)\n",
				i, ddp_get_reg_module_name(i), dispsys_reg[i],
				ddp_reg_pa_base[i], dispsys_irq[i]);
			snprintf(buf_temp, 100,
				"i=%d, module=%s, va=0x%lx, pa=0x%lx, irq(%d)\n", i,
				ddp_get_reg_module_name(i), dispsys_reg[i],
				ddp_reg_pa_base[i], dispsys_irq[i]);
			buf_temp += strlen(buf_temp);
		}

	} else if (0 == strncmp(opt, "debug:", 6)) {
		char *p = (char *)opt + 6;
		unsigned int enable;

		ret = kstrtouint(p, 0, &enable);
		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		if (enable == 1) {
			DISPMSG("[DDP] debug=1, trigger AEE\n");
			/* aee_kernel_exception("DDP-TEST-ASSERT", "[DDP] DDP-TEST-ASSERT"); */
		} else if (enable == 2) {
			ddp_mem_test();
		} else if (enable == 3) {
			ddp_lcd_test();
		} else if (enable == 4) {
			/* DISPAEE("test 4"); */
		} else if (enable == 12) {
			if (gUltraEnable == 0)
				gUltraEnable = 1;
			else
				gUltraEnable = 0;
			sprintf(buf, "gUltraEnable: %d\n", gUltraEnable);
		}
	} else if (0 == strncmp(opt, "mmp", 3)) {
		init_ddp_mmp_events();
	} else if (0 == strncmp(opt, "low_power_mode:", 15)) {
		char *p = (char *)opt + 15;
		unsigned int mode;

		ret = kstrtouint(p, 0, &mode);

		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		low_power_cust_mode = mode;

	} else {
		dbg_buf[0] = '\0';
		goto Error;
	}

	return;

Error:
	DISPERR("parse command error!\n%s\n\n%s", opt, DDP_STR_HELP);
}

void mtkfb_process_dbg_opt(const char *opt)
{
	int ret;

	if (0 == strncmp(opt, "helper", 6)) {
		/*ex: echo helper:DISP_OPT_BYPASS_OVL,0 > /d/mtkfb */
		char option[100] = "";
		char *tmp;
		int value, i;

		tmp = (char *)opt + 7;
		for (i = 0; i < 100; i++) {
			if (tmp[i] != ',' && tmp[i] != ' ')
				option[i] = tmp[i];
			else
				break;
		}
		tmp += i + 1;
		ret = sscanf(tmp, "%d\n", &value);
		if (ret != 1) {
			pr_err("error to parse cmd %s: %s %s ret=%d\n", opt, option, tmp, ret);
			return;
		}

		DISPMSG("will set option %s to %d\n", option, value);
		disp_helper_set_option_by_name(option, value);
	} else if (0 == strncmp(opt, "switch_mode:", 12)) {
		int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
		int sess_mode;

		ret = sscanf(opt, "switch_mode:%d\n", &sess_mode);
		if (ret != 1) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		primary_display_switch_mode(sess_mode, session_id, 1);
	} else if (0 == strncmp(opt, "dsi_mode:cmd", 12)) {
		lcm_mode_status = 1;
		DISPMSG("switch cmd\n");
	} else if (0 == strncmp(opt, "dsi_mode:vdo", 12)) {
		DISPMSG("switch vdo\n");
		lcm_mode_status = 2;
	} else if (0 == strncmp(opt, "clk_change:", 11)) {
		char *p = (char *)opt + 11;
		unsigned int clk = 0;

		ret = kstrtouint(p, 0, &clk);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		DISPMSG("clk_change:%d\n", clk);
		primary_display_mipi_clk_change(clk);
	} else if (0 == strncmp(opt, "freeze:", 7)) {
		if (0 == strncmp(opt + 7, "on", 2))
			display_freeze_mode(1, 1);
		else if (0 == strncmp(opt + 7, "off", 3))
			display_freeze_mode(0, 1);
	} else if (0 == strncmp(opt, "dsipattern", 10)) {
		char *p = (char *)opt + 11;
		unsigned int pattern;

		ret = kstrtouint(p, 0, &pattern);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		if (pattern) {
			primary_display_manual_lock();
			DSI_BIST_Pattern_Test(DISP_MODULE_DSI0, NULL, true, pattern);
			primary_display_manual_unlock();
			DISPMSG("enable dsi pattern: 0x%08x\n", pattern);
		} else {
			primary_display_manual_lock();
			DSI_BIST_Pattern_Test(DISP_MODULE_DSI0, NULL, false, 0);
			primary_display_manual_unlock();
			return;
		}
	} else if (0 == strncmp(opt, "m6_dsi_bist_full:", 17)) {
		char *p = (char *)opt + 17;
		unsigned int pattern;

		ret = kstrtouint(p, 0, &pattern);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		primary_display_manual_lock();
		DSI_M6_BIST_Full_Test(DISP_MODULE_DSI0, NULL, pattern != 0, pattern);
		primary_display_manual_unlock();
		DISPMSG("m6 dsi bist full: 0x%08x\n", pattern);
	} else if (0 == strncmp(opt, "m6_dsi_bist_profile:", 20)) {
		unsigned int profile = 0;
		unsigned int pattern = 0;
		unsigned int hold_ms = 3000;
		int pattern_arg = 0;

		ret = sscanf(opt, "m6_dsi_bist_profile:%u:%i:%u\n",
			     &profile, &pattern_arg, &hold_ms);
		if (ret < 2 || pattern_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		pattern = (unsigned int)pattern_arg;

		primary_display_manual_lock();
		DSI_M6_BIST_Profile_Test(DISP_MODULE_DSI0, NULL, profile,
					 pattern, hold_ms);
		primary_display_manual_unlock();
		DISPMSG("m6 dsi bist profile: profile=%u pattern=0x%08x hold=%u\n",
			profile, pattern, hold_ms);
	} else if (0 == strncmp(opt, "m6_dsi_hs_window:", 17)) {
		const char *arg = opt + 17;
		const char *sep;
		char tag[32] = {0};
		unsigned int hold_ms = 1000;
		size_t tag_len;

		sep = strchr(arg, ':');
		tag_len = sep ? (size_t)(sep - arg) : strnlen(arg, sizeof(tag) - 1);
		if (tag_len == 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		if (tag_len >= sizeof(tag))
			tag_len = sizeof(tag) - 1;
		memcpy(tag, arg, tag_len);
		tag[tag_len] = '\0';
		if (sep && sep[1] != '\0') {
			ret = kstrtouint(sep + 1, 0, &hold_ms);
			if (ret) {
				pr_err("error to parse cmd %s ret=%d\n",
				       opt, ret);
				return;
			}
		}
		primary_display_manual_lock();
		dsi_m6_dump_hs_window(tag, hold_ms);
		primary_display_manual_unlock();
		DISPMSG("m6 dsi hs window: tag=%s hold=%u\n", tag, hold_ms);
	} else if (0 == strncmp(opt, "m6_dsi_phy_truth", 16)) {
		const char *tag = "manual";
		char safe_tag[32];

		if (opt[16] == ':')
			tag = opt + 17;
		disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
		primary_display_manual_lock();
		dsi_m6_dump_phy_truth(safe_tag);
		primary_display_manual_unlock();
		DISPMSG("m6 dsi phy truth: tag=%s\n", safe_tag);
		} else if (0 == strncmp(opt, "m6_dsi_debug_mux_stats",
					sizeof("m6_dsi_debug_mux_stats") - 1)) {
			const size_t prefix_len = sizeof("m6_dsi_debug_mux_stats") - 1;
			char safe_tag[32] = "manual";
			unsigned int samples = 12;
			unsigned int delay_us = 1000;

			if (opt[prefix_len] == ':') {
				const char *arg = opt + prefix_len + 1;
				const char *p = arg;
				char tag_arg[32];
				size_t i = 0;

				while (i + 1 < sizeof(tag_arg) && *p &&
				       *p != ':' && *p != '\n' && *p != '\r' &&
				       *p != ' ' && *p != '\t') {
					tag_arg[i] = *p;
					i++;
					p++;
				}
				tag_arg[i] = '\0';
				disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag_arg);
				if (*p == ':') {
					ret = sscanf(p + 1, "%u:%u", &samples, &delay_us);
					if (ret < 1) {
						pr_err("error to parse cmd %s\n", opt);
						return;
					}
				}
			}
			primary_display_manual_lock();
			dsi_m6_debug_mux_stats(safe_tag, samples, delay_us);
			primary_display_manual_unlock();
			DISPERR("M6 DSI debug_mux_stats command: tag=%s samples=%u delay_us=%u\n",
				safe_tag, samples, delay_us);
		} else if (0 == strncmp(opt, "m6_dsi_lkgold_muxstats_dump",
					sizeof("m6_dsi_lkgold_muxstats_dump") - 1)) {
			primary_display_manual_lock();
			dsi_m6_lkgold_muxstats_dump();
			primary_display_manual_unlock();
			DISPERR("M6 DSI lkgold_muxstats_dump command\n");
		} else if (0 == strncmp(opt, "m6_dsi_debug_mux", 16)) {
			const char *tag = "manual";
			char safe_tag[32];

		if (opt[16] == ':')
			tag = opt + 17;
		disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
		primary_display_manual_lock();
		dsi_m6_debug_mux_sweep(safe_tag);
		primary_display_manual_unlock();
		DISPERR("M6 DSI debug_mux command: tag=%s\n", safe_tag);
	} else if (0 == strncmp(opt, "m6_dsi_clk_restore", 18)) {
		const char *tag = "manual";
		char safe_tag[32];

		if (opt[18] == ':')
			tag = opt + 19;
		disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
		primary_display_manual_lock();
		dsi_m6_force_clk_restore(safe_tag);
		primary_display_manual_unlock();
		DISPERR("M6 DSI clk_restore command: tag=%s\n", safe_tag);
	} else if (0 == strncmp(opt, "m6_dsi_pll_change:", sizeof("m6_dsi_pll_change:") - 1)) {
		unsigned int new_pll = 0;
		const char *tag = "manual";
		char safe_tag[32];
		const char *colon;

		ret = sscanf(opt, "m6_dsi_pll_change:%u", &new_pll);
		if (ret < 1) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		colon = strchr(opt + sizeof("m6_dsi_pll_change:") - 1, ':');
		if (colon)
			tag = colon + 1;
		disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
		primary_display_manual_lock();
		dsi_m6_force_pll_change(new_pll, safe_tag);
		primary_display_manual_unlock();
		DISPERR("M6 DSI pll_change command: pll=%u tag=%s\n", new_pll, safe_tag);
	} else if (0 == strncmp(opt, "m6_dsi_cc_probe:", sizeof("m6_dsi_cc_probe:") - 1)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;

		ret = sscanf(opt, "m6_dsi_cc_probe:%i:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux);
		if (ret < 1 || value_arg < 0 || value_arg > 1) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
			dsi_m6_force_cc_probe(value, hold_ms, restore, sample_mux);
			primary_display_manual_unlock();
			DISPERR("M6 DSI cc_probe command: value=%u hold=%u restore=%u mux=%u\n",
				value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "m6_dsi_lc_hs_probe:", sizeof("m6_dsi_lc_hs_probe:") - 1)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;

		ret = sscanf(opt, "m6_dsi_lc_hs_probe:%i:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux);
		if (ret < 1 || value_arg < 0 || value_arg > 1) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
		dsi_m6_force_lc_hs_probe(value, hold_ms, restore, sample_mux);
		primary_display_manual_unlock();
		DISPERR("M6 DSI lc_hs_probe command: value=%u hold=%u restore=%u mux=%u\n",
			value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "m6_dsi_mipitx_pad_window",
				sizeof("m6_dsi_mipitx_pad_window") - 1)) {
		const size_t prefix_len = sizeof("m6_dsi_mipitx_pad_window") - 1;
		char safe_tag[32] = "manual";
		unsigned int samples = 6;
		unsigned int delay_ms = 100;

		if (opt[prefix_len] == ':') {
			const char *arg = opt + prefix_len + 1;
			const char *p = arg;
			char tag_arg[32];
			size_t i = 0;

			while (i + 1 < sizeof(tag_arg) && *p &&
			       *p != ':' && *p != '\n' && *p != '\r' &&
			       *p != ' ' && *p != '\t') {
				tag_arg[i] = *p;
				i++;
				p++;
			}
			tag_arg[i] = '\0';
			disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag_arg);
			if (*p == ':') {
				ret = sscanf(p + 1, "%u:%u", &samples, &delay_ms);
				if (ret < 1) {
					pr_err("error to parse cmd %s\n", opt);
					return;
				}
			}
		}
		primary_display_manual_lock();
		dsi_m6_mipitx_pad_window(safe_tag, samples, delay_ms);
		primary_display_manual_unlock();
		DISPERR("M6 DSI mipitx_pad_window command: tag=%s samples=%u delay_ms=%u\n",
			safe_tag, samples, delay_ms);
	} else if (0 == strncmp(opt, "m6_dsi_mipitx_pad_probe:",
				sizeof("m6_dsi_mipitx_pad_probe:") - 1)) {
		const char *p = opt + sizeof("m6_dsi_mipitx_pad_probe:") - 1;
		char field[24];
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;
		size_t i = 0;

		while (i + 1 < sizeof(field) && *p &&
		       *p != ':' && *p != '\n' && *p != '\r' &&
		       *p != ' ' && *p != '\t') {
			field[i] = *p;
			i++;
			p++;
		}
		field[i] = '\0';
		if (!field[0] || *p != ':') {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		ret = sscanf(p + 1, "%i:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
		dsi_m6_mipitx_pad_probe(field, value, hold_ms, restore, sample_mux);
		primary_display_manual_unlock();
		DISPERR("M6 DSI mipitx_pad_probe command: field=%s value=%u hold=%u restore=%u mux=%u\n",
			field, value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "m6_dsi_mipitx_lane_group_probe:",
				sizeof("m6_dsi_mipitx_lane_group_probe:") - 1)) {
		const char *p = opt + sizeof("m6_dsi_mipitx_lane_group_probe:") - 1;
		char group[16];
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;
		size_t i = 0;

		while (i + 1 < sizeof(group) && *p &&
		       *p != ':' && *p != '\n' && *p != '\r' &&
		       *p != ' ' && *p != '\t') {
			group[i] = *p;
			i++;
			p++;
		}
		group[i] = '\0';
		if (!group[0] || *p != ':') {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		ret = sscanf(p + 1, "%i:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
		dsi_m6_mipitx_lane_group_probe(group, value, hold_ms,
					       restore, sample_mux);
		primary_display_manual_unlock();
		DISPERR("M6 DSI mipitx_lane_group_probe command: group=%s value=%u hold=%u restore=%u mux=%u\n",
			group, value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "m6_dsi_mipitx_phy_sel_probe:",
				sizeof("m6_dsi_mipitx_phy_sel_probe:") - 1)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;

		ret = sscanf(opt, "m6_dsi_mipitx_phy_sel_probe:%i:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
		dsi_m6_mipitx_phy_sel_probe(value, hold_ms, restore, sample_mux);
		primary_display_manual_unlock();
		DISPERR("M6 DSI mipitx_phy_sel_probe command: value=0x%x hold=%u restore=%u mux=%u\n",
			value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "m6_dsi_mipitx_plltop_probe:",
				sizeof("m6_dsi_mipitx_plltop_probe:") - 1)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;
		unsigned int shift = 8;

		ret = sscanf(opt, "m6_dsi_mipitx_plltop_probe:%i:%u:%u:%u:%u\n",
			     &value_arg, &hold_ms, &restore, &sample_mux,
			     &shift);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		primary_display_manual_lock();
		dsi_m6_mipitx_plltop_probe(value, hold_ms, restore, sample_mux,
					   shift);
		primary_display_manual_unlock();
		DISPERR("M6 DSI mipitx_plltop_probe command: value=%u hold=%u restore=%u mux=%u shift=%u\n",
			value, hold_ms, restore ? 1 : 0, sample_mux, shift);
	} else if (0 == strncmp(opt, "m6_dsi_wrtrace_dump",
				sizeof("m6_dsi_wrtrace_dump") - 1)) {
		const unsigned int prefix = sizeof("m6_dsi_wrtrace_dump") - 1;
		unsigned int limit = 256;

		if (opt[prefix] == ':') {
			ret = kstrtouint(opt + prefix + 1, 0, &limit);
			if (ret) {
				pr_err("error to parse cmd %s ret=%d\n",
				       opt, ret);
				return;
			}
		}
		dsi_m6_wrtrace_dump(limit);
		DISPERR("M6 DSI wrtrace dump command: limit=%u\n", limit);
		return;
	} else if (0 == strncmp(opt, "m6_dsi_wrtrace_reset",
				sizeof("m6_dsi_wrtrace_reset") - 1)) {
		const unsigned int prefix = sizeof("m6_dsi_wrtrace_reset") - 1;
		unsigned int enable = 1;

		if (opt[prefix] == ':') {
			ret = kstrtouint(opt + prefix + 1, 0, &enable);
			if (ret) {
				pr_err("error to parse cmd %s ret=%d\n",
				       opt, ret);
				return;
			}
		}
		dsi_m6_wrtrace_reset(enable);
		DISPERR("M6 DSI wrtrace reset command: enable=%u\n",
			enable ? 1 : 0);
		return;
	} else if (0 == strncmp(opt, "m6_dsi_wrtrace_enable:",
				sizeof("m6_dsi_wrtrace_enable:") - 1)) {
		unsigned int enable = 0;

		ret = kstrtouint(opt + sizeof("m6_dsi_wrtrace_enable:") - 1,
				 0, &enable);
		if (ret) {
			pr_err("error to parse cmd %s ret=%d\n", opt, ret);
			return;
		}
		dsi_m6_wrtrace_enable(enable);
		DISPERR("M6 DSI wrtrace enable command: enable=%u\n",
			enable ? 1 : 0);
		return;
	} else if (0 == strncmp(opt, "m6_dsi_hsa_wc:", 14)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;

		ret = sscanf(opt, "m6_dsi_hsa_wc:%i:%u\n",
			     &value_arg, &hold_ms);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;

		primary_display_manual_lock();
		dsi_m6_force_hsa_wc(value, hold_ms);
		primary_display_manual_unlock();
		DISPMSG("m6 dsi hsa wc: value=0x%08x hold=%u\n",
			value, hold_ms);
	} else if (0 == strncmp(opt, "m6_dsi_vm_cmd_probe:",
				sizeof("m6_dsi_vm_cmd_probe:") - 1)) {
		unsigned int value = 0;
		unsigned int hold_ms = 1000;
		unsigned int restore = 1;
		unsigned int sample_mux = 0;

		ret = sscanf(opt, "m6_dsi_vm_cmd_probe:%x:%u:%u:%u\n",
			     &value, &hold_ms, &restore, &sample_mux);
		if (ret < 1) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_manual_lock();
		dsi_m6_force_vm_cmd(value, hold_ms, restore, sample_mux);
		primary_display_manual_unlock();
		DISPERR("M6 DSI vm_cmd_probe command: value=0x%x hold=%u restore=%u mux=%u\n",
			value, hold_ms, restore ? 1 : 0, sample_mux);
	} else if (0 == strncmp(opt, "bypass_blank:", 13)) {
		char *p = (char *)opt + 13;
		unsigned int blank;

		ret = kstrtouint(p, 0, &blank);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		if (blank)
			bypass_blank = 1;
		else
			bypass_blank = 0;

	} else if (0 == strncmp(opt, "force_fps:", 9)) {
		unsigned int keep;
		unsigned int skip;

		ret = sscanf(opt, "force_fps:%d,%d\n", &keep, &skip);
		if (ret != 2) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		DISPMSG("force set fps, keep %d, skip %d\n", keep, skip);
		primary_display_force_set_fps(keep, skip);
	} else if (0 == strncmp(opt, "AAL_trigger", 11)) {
		int i = 0;
		disp_session_vsync_config vsync_config;

		for (i = 0; i < 1200; i++) {
			primary_display_wait_for_vsync(&vsync_config);
			dpmgr_module_notify(DISP_MODULE_AAL, DISP_PATH_EVENT_TRIGGER);
		}
	} else if (0 == strncmp(opt, "diagnose", 8)) {
		primary_display_diagnose();
		return;
	} else if (0 == strncmp(opt, "_efuse_test", 11)) {
		primary_display_check_test();
	} else if (0 == strncmp(opt, "dprec_reset", 11)) {
		dprec_logger_reset_all();
		return;
	} else if (0 == strncmp(opt, "suspend", 7)) {
		primary_display_suspend();
		return;
	} else if (0 == strncmp(opt, "resume", 6)) {
		primary_display_resume();
	} else if (0 == strncmp(opt, "dsi_clk_on", 10)) {
		/* Debug command: ungate the DSI engine and digital clocks
		 * directly in MMSYS CG, bypassing ddp_clk_prepare_enable().
		 * MMSYS CG_CON1 bit0 = DSI_ENGINE, bit1 = DSI_DIGITAL; CG_CLR1 =
		 * 0x3 clears those gate bits (the mm_sel parent and the DISP
		 * domain must already be on). */
		pr_emerg("[FORGE_DISP] v201 dsi_clk_on: CG_CON1 before=0x%x (bit0/1 set=gated)\n",
			 DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1));
		DISP_REG_SET(NULL, DISP_REG_CONFIG_MMSYS_CG_CLR1, 0x3);
		pr_emerg("[FORGE_DISP] v201 dsi_clk_on: CG_CON1 after=0x%x (bit0/1 clear=ungated)\n",
			 DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1));
		primary_display_m6_lcm_reinit(1);
		pr_emerg("[FORGE_DISP] v201 dsi_clk_on: reinit done\n");
		return;
	} else if (0 == strncmp(opt, "m6_lcm_reinit", 13)) {
		char *p = (char *)opt + 13;
		unsigned int force_power = 1;

		if (*p == ':') {
			ret = kstrtouint(p + 1, 0, &force_power);
			if (ret) {
				pr_err("error to parse cmd %s\n", opt);
				return;
			}
		}
		DISPERR("M6 LCM debug reinit command: force=%u\n", force_power);
		primary_display_m6_lcm_reinit(force_power);
		return;
	} else if (0 == strncmp(opt, "m6_lcm_page5_2a:", 16)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;

		ret = sscanf(opt, "m6_lcm_page5_2a:%i:%u\n",
			     &value_arg, &hold_ms);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		DISPERR("M6 LCM page5_2a command: value=0x%x hold=%u\n",
			value, hold_ms);
		primary_display_m6_lcm_page5_2a(value, hold_ms);
		return;
	} else if (0 == strncmp(opt, "m6_lcm_mode_ctrl:", 17)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;

		ret = sscanf(opt, "m6_lcm_mode_ctrl:%i:%u\n",
			     &value_arg, &hold_ms);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		DISPERR("M6 LCM mode_ctrl command: value=0x%x hold=%u\n",
			value, hold_ms);
		primary_display_m6_lcm_mode_ctrl(value, hold_ms);
		return;
	} else if (0 == strncmp(opt, "m6_dsi_c2v_switch:", 18)) {
		int value_arg = 0;
		unsigned int value = 0;
		unsigned int hold_ms = 1000;

		ret = sscanf(opt, "m6_dsi_c2v_switch:%i:%u\n",
			     &value_arg, &hold_ms);
		if (ret < 1 || value_arg < 0) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		value = (unsigned int)value_arg;
		DISPERR("M6 DSI c2v_switch command: value=0x%x hold=%u\n",
			value, hold_ms);
		primary_display_m6_dsi_c2v_switch(value, hold_ms);
		return;
	} else if (0 == strncmp(opt, "m6_display_truth_window", 23)) {
		const char *tag = "manual";

		if (opt[23] == ':')
			tag = opt + 24;
		DISPERR("M6 DISPLAY truth command: tag=%s\n", tag);
		primary_display_m6_truth_window(tag);
		return;
	} else if (0 == strncmp(opt, "m6_display_route_probe", 22)) {
		const char *tag = "trigger";
		unsigned int action = 0x1;

		if (opt[22] == ':')
			tag = opt + 23;
		if (!strncmp(tag, "dump", 4)) {
			action = 0x0;
		} else if (!strncmp(tag, "trigger", 7)) {
			action = 0x1;
		} else if (!strncmp(tag, "rekick", 6)) {
			action = 0x3;
		} else {
			ret = kstrtouint((char *)tag, 0, &action);
			if (ret) {
				pr_err("error to parse cmd %s\n", opt);
				return;
			}
		}
		DISPERR("M6 DISPLAY route probe command: tag=%s action=0x%x\n",
			tag, action);
		primary_display_m6_route_probe(tag, action);
		return;
	} else if (0 == strncmp(opt, "m6_dsi_dcs_status_force",
				sizeof("m6_dsi_dcs_status_force") - 1)) {
		const unsigned int prefix = sizeof("m6_dsi_dcs_status_force") - 1;
		const char *tag = "force";
		char safe_tag[32];

		if (opt[prefix] == ':')
			tag = opt + prefix + 1;
		disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
		DISPERR("M6 DSI DCS status force command: tag=%s\n", safe_tag);
		primary_display_manual_lock();
		dsi_m6_dump_dcs_status_force(safe_tag);
		primary_display_manual_unlock();
		return;
	} else if (0 == strncmp(opt, "m6_dsi_dcs_status", 17)) {
		const char *tag = "public";
		char safe_tag[32];

		if (opt[17] == ':')
			tag = opt + 18;
		if (!strncmp(tag, "stock_pages", 11)) {
			DISPERR("M6 DSI DCS status command: stock_pages\n");
			primary_display_m6_lcm_stock_pages();
		} else {
			disp_m6_copy_tag(safe_tag, sizeof(safe_tag), tag);
			DISPERR("M6 DSI DCS status command: tag=%s\n", safe_tag);
			primary_display_manual_lock();
			dsi_m6_dump_dcs_status(safe_tag);
			primary_display_manual_unlock();
		}
		return;
	} else if (0 == strncmp(opt, "m6_ovl_greq_profile:", 20)) {
		char *p = (char *)opt + 20;
		unsigned int profile;

		ret = kstrtouint(p, 0, &profile);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_manual_lock();
		ret = ovl_m6_set_greq_profile(profile);
		primary_display_manual_unlock();
		DISPERR("M6 OVL greq profile command: profile=%u ret=%d\n",
			profile, ret);
		return;
	} else if (0 == strncmp(opt, "m6_ovl_bounds_profile:", 22)) {
		char *p = (char *)opt + 22;
		unsigned int profile;

		ret = kstrtouint(p, 0, &profile);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_manual_lock();
		ret = ovl_m6_set_bounds_profile(profile);
		primary_display_manual_unlock();
		DISPERR("M6 OVL bounds profile command: profile=%u ret=%d\n",
			profile, ret);
		return;
	} else if (0 == strncmp(opt, "m6_ovl_stale_cpu_clear:", 23)) {
		char *p = (char *)opt + 23;
		unsigned int enable;

		ret = kstrtouint(p, 0, &enable);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_manual_lock();
		ret = ovl_m6_set_stale_cpu_clear(enable);
		primary_display_manual_unlock();
		DISPERR("M6 OVL stale cpu clear command: enable=%u ret=%d\n",
			!!enable, ret);
		return;
	} else if (0 == strncmp(opt, "ata", 3)) {
		mtkfb_fm_auto_test();
		return;
	} else if (0 == strncmp(opt, "dalprintf", 9)) {
		DAL_Printf("display aee layer test\n");
	} else if (0 == strncmp(opt, "dalclean", 8)) {
		DAL_Clean();
	} else if (0 == strncmp(opt, "daltest", 7)) {
		int i = 1000;

		while (i--) {
			DAL_Printf("display aee layer test\n");
			msleep(20);
			DAL_Clean();
			msleep(20);
		}
	} else if (0 == strncmp(opt, "lfr_setting:", 12)) {
		unsigned int enable;
		unsigned int mode;
		unsigned int type = 0;
		unsigned int skip_num = 1;

		ret = sscanf(opt, "lfr_setting:%d,%d\n", &enable, &mode);
		if (ret != 2) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		DISPMSG("--------------enable/disable lfr--------------\n");
		if (enable) {
			DISPMSG("lfr enable %d mode =%d\n", enable, mode);
			enable = 1;
			DSI_Set_LFR(DISP_MODULE_DSI0, NULL, mode, type, enable, skip_num);
		} else {
			DISPMSG("lfr disable %d mode=%d\n", enable, mode);
			enable = 0;
			DSI_Set_LFR(DISP_MODULE_DSI0, NULL, mode, type, enable, skip_num);
		}
	} else if (0 == strncmp(opt, "vsync_switch:", 13)) {
		char *p = (char *)opt + 13;
		unsigned int method = 0;

		ret = kstrtouint(p, 0, &method);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_vsync_switch(method);

	} else if (0 == strncmp(opt, "dsi0_clk:", 9)) {
		char *p = (char *)opt + 9;
		uint32_t clk;

		ret = kstrtouint(p, 0, &clk);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
	} else if (0 == strncmp(opt, "detect_recovery", 15)) {
		DISPMSG("primary_display_signal_recovery\n");
		primary_display_signal_recovery();
	} else if (0 == strncmp(opt, "dst_switch:", 11)) {
		char *p = (char *)opt + 11;
		uint32_t mode;

		ret = kstrtouint(p, 0, &mode);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_switch_dst_mode(mode % 2);
		return;
	} else if (0 == strncmp(opt, "cmmva_dprec", 11)) {
		dprec_handle_option(0x7);
	} else if (0 == strncmp(opt, "cmmpa_dprec", 11)) {
		dprec_handle_option(0x3);
	} else if (0 == strncmp(opt, "dprec", 5)) {
		char *p = (char *)opt + 6;
		unsigned int option;

		ret = kstrtouint(p, 0, &option);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		dprec_handle_option(option);
	} else if (0 == strncmp(opt, "maxlayer", 8)) {
		char *p = (char *)opt + 9;
		unsigned int maxlayer;

		ret = kstrtouint(p, 0, &maxlayer);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		if (maxlayer)
			primary_display_set_max_layer(maxlayer);
		else
			DISPERR("can't set max layer to 0\n");
	} else if (0 == strncmp(opt, "primary_reset", 13)) {
		primary_display_reset();
	} else if (0 == strncmp(opt, "esd_check", 9)) {
		char *p = (char *)opt + 10;
		unsigned int enable;

		ret = kstrtouint(p, 0, &enable);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		primary_display_esd_check_enable(enable);
	} else if (0 == strncmp(opt, "esd_recovery", 12)) {
		primary_display_esd_recovery();
	} else if (0 == strncmp(opt, "lcm0_reset", 10)) {
		DISPMSG("lcm0_reset\n");
#if 1
		DISP_CPU_REG_SET(DISPSYS_CONFIG_BASE + 0x150, 1);
		msleep(20);
		DISP_CPU_REG_SET(DISPSYS_CONFIG_BASE + 0x150, 0);
		msleep(20);
		DISP_CPU_REG_SET(DISPSYS_CONFIG_BASE + 0x150, 1);
#else
#ifdef CONFIG_MTK_LEGACY
		mt_set_gpio_mode(GPIO158 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO158 | 0x80000000, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO158 | 0x80000000, GPIO_OUT_ONE);
		msleep(20);
		mt_set_gpio_out(GPIO158 | 0x80000000, GPIO_OUT_ZERO);
		msleep(20);
		mt_set_gpio_out(GPIO158 | 0x80000000, GPIO_OUT_ONE);
#else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
		msleep(20);
		ret |= disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
		msleep(20);
		ret |= disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
#endif
#endif
	} else if (0 == strncmp(opt, "lcm0_reset0", 11)) {
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG + 0x150, 0);
	} else if (0 == strncmp(opt, "lcm0_reset1", 11)) {
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG + 0x150, 1);
	} else if (0 == strncmp(opt, "dump_layer:", 11)) {
		if (0 == strncmp(opt + 11, "on", 2)) {
			ret = sscanf(opt, "dump_layer:on,%d,%d,%d\n",
				     &gCapturePriLayerDownX, &gCapturePriLayerDownY, &gCapturePriLayerNum);
			if (ret != 3) {
				pr_err("error to parse cmd %s\n", opt);
				return;
			}

			gCapturePriLayerEnable = 1;
			gCaptureWdmaLayerEnable = 0;
			if (gCapturePriLayerDownX == 0)
				gCapturePriLayerDownX = 20;
			if (gCapturePriLayerDownY == 0)
				gCapturePriLayerDownY = 20;
			DISPMSG("dump_layer En %d DownX %d DownY %d,Num %d", gCapturePriLayerEnable,
			       gCapturePriLayerDownX, gCapturePriLayerDownY, gCapturePriLayerNum);

		} else if (0 == strncmp(opt + 11, "off", 3)) {
			gCapturePriLayerEnable = 0;
			gCaptureWdmaLayerEnable = 0;
			gCapturePriLayerNum = TOTAL_OVL_LAYER_NUM;
			DISPMSG("dump_layer En %d\n", gCapturePriLayerEnable);
		}

	} else if (0 == strncmp(opt, "dump_wdma_layer:", 16)) {
		if (0 == strncmp(opt + 16, "on", 2)) {
			ret = sscanf(opt, "dump_wdma_layer:on,%d,%d\n",
				     &gCapturePriLayerDownX, &gCapturePriLayerDownY);
			if (ret != 2) {
				pr_err("error to parse cmd %s\n", opt);
				return;
			}

			gCaptureWdmaLayerEnable = 1;
			if (gCapturePriLayerDownX == 0)
				gCapturePriLayerDownX = 20;
			if (gCapturePriLayerDownY == 0)
				gCapturePriLayerDownY = 20;
			DISPMSG("dump_wdma_layer En %d DownX %d DownY %d", gCaptureWdmaLayerEnable,
			       gCapturePriLayerDownX, gCapturePriLayerDownY);

		} else if (0 == strncmp(opt + 16, "off", 3)) {
			gCaptureWdmaLayerEnable = 0;
			DISPMSG("dump_layer En %d\n", gCaptureWdmaLayerEnable);
		}
	} else if (0 == strncmp(opt, "dump_rdma_layer:", 16)) {
		if (0 == strncmp(opt + 16, "on", 2)) {
			ret = sscanf(opt, "dump_rdma_layer:on,%d,%d\n",
				     &gCapturePriLayerDownX, &gCapturePriLayerDownY);
			if (ret != 2) {
				pr_err("error to parse cmd %s\n", opt);
				return;
			}

			gCaptureRdmaLayerEnable = 1;
			if (gCapturePriLayerDownX == 0)
				gCapturePriLayerDownX = 20;
			if (gCapturePriLayerDownY == 0)
				gCapturePriLayerDownY = 20;
			DISPMSG("dump_wdma_layer En %d DownX %d DownY %d", gCaptureRdmaLayerEnable,
			       gCapturePriLayerDownX, gCapturePriLayerDownY);

		} else if (0 == strncmp(opt + 16, "off", 3)) {
			gCaptureRdmaLayerEnable = 0;
			DISPMSG("dump_layer En %d\n", gCaptureRdmaLayerEnable);
		}
	} else if (0 == strncmp(opt, "enable_idlemgr:", 15)) {
		char *p = (char *)opt + 15;
		uint32_t flg;

		ret = kstrtouint(p, 0, &flg);
		if (ret) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		enable_idlemgr(flg);
	}

	if (0 == strncmp(opt, "primary_basic_test:", 19)) {
		int layer_num, w, h, fmt, frame_num, vsync;

		ret = sscanf(opt, "primary_basic_test:%d,%d,%d,%d,%d,%d\n",
			     &layer_num, &w, &h, &fmt, &frame_num, &vsync);
		if (ret != 6) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		if (fmt == 0)
			fmt = DISP_FORMAT_RGBA8888;
		else if (fmt == 1)
			fmt = DISP_FORMAT_RGB888;
		else if (fmt == 2)
			fmt = DISP_FORMAT_RGB565;

		/*primary_display_basic_test(layer_num, w, h, fmt, frame_num, vsync);*/
	}

	if (0 == strncmp(opt, "pan_disp_test:", 13)) {
		int frame_num;
		int bpp;

		ret = sscanf(opt, "pan_disp_test:%d,%d\n", &frame_num, &bpp);
		if (ret != 2) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		pan_display_test(frame_num, bpp);
	}

}

/* --------------------------------------------------------------------------- */
/* Local debugfs Command Processor */
/* --------------------------------------------------------------------------- */
int get_lp_cust_mode(void)
{
	return low_power_cust_mode;
}
void backup_vfp_for_lp_cust(unsigned int vfp)
{
	vfp_backup = vfp;
}
unsigned int get_backup_vfp(void)
{
	return vfp_backup;
}

static char cmd_buf[512];

static void lp_cust_process_dbg_opt(const char *opt)
{
	int ret = 0;
	char *buf = cmd_buf + strlen(cmd_buf);

	if (0 == strncmp(opt, "low_power_mode:", 15)) {
		char *p = (char *)opt + 15;
		unsigned int mode;

		ret = kstrtouint(p, 0, &mode);

		if (ret) {
			snprintf(buf, 50, "error to parse cmd %s\n", opt);
			return;
		}

		low_power_cust_mode = mode;

	} else {
		cmd_buf[0] = '\0';
		goto Error;
	}

	return;

Error:
	DISPERR("parse command error!\n%s\n\n%s", opt, LP_CUST_STR_HELP);
}

static void lp_cust_process_dbg_cmd(char *cmd)
{
	char *tok;

	DISPMSG("cmd: %s\n", cmd);
	memset(cmd_buf, 0, sizeof(cmd_buf));
	while ((tok = strsep(&cmd, " ")) != NULL)
		lp_cust_process_dbg_opt(tok);
}

static ssize_t lp_cust_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&cmd_buf, ubuf, count))
		return -EFAULT;

	cmd_buf[count] = 0;

	lp_cust_process_dbg_cmd(cmd_buf);

	return ret;
}

static ssize_t lp_cust_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	char *mode0 = "low power mode(1)\n";
	char *mode1 = "just make mode(2)\n";
	char *mode2 = "performance mode(3)\n";
	char *mode4 = "unknown mode(n)\n";

	switch (low_power_cust_mode) {
	case LOW_POWER_MODE:
		return simple_read_from_buffer(ubuf, count, ppos, mode0, strlen(mode0));
	case JUST_MAKE_MODE:
		return simple_read_from_buffer(ubuf, count, ppos, mode1, strlen(mode1));
	case PERFORMANC_MODE:
		return simple_read_from_buffer(ubuf, count, ppos, mode2, strlen(mode2));
	default:
		return simple_read_from_buffer(ubuf, count, ppos, mode4, strlen(mode4));
	}

}

static int lp_cust_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations low_power_cust_fops = {
	.read = lp_cust_read,
	.write = lp_cust_write,
	.open = lp_cust_open,
};

static ssize_t kick_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(ubuf, count, ppos, get_kick_dump(), get_kick_dump_size());
}

static const struct file_operations kickidle_fops = {
	.read = kick_read,
};

void sub_debug_init(void)
{
	lowpowermode_debugfs = debugfs_create_file("lowpowermode",
						   S_IFREG | S_IRUGO, disp_debugDir, NULL, &low_power_cust_fops);
	if (!lowpowermode_debugfs)
		DISPERR("create debug file disp/lowpowermode fail!\n");

	kickdump_debugfs = debugfs_create_file("kickdump",
					       S_IFREG | S_IRUGO, disp_debugDir, NULL, &kickidle_fops);
	if (!kickdump_debugfs)
		DISPERR("create debug file disp/kickdump fail!\n");
}

void sub_debug_deinit(void)
{
	debugfs_remove(lowpowermode_debugfs);
	debugfs_remove(kickdump_debugfs);
}
