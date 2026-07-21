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

#include <linux/slab.h>
#include <linux/types.h>
#include "disp_log.h"
#include "lcm_drv.h"
#include "disp_drv_platform.h"
#include "ddp_manager.h"
#include "ddp_dsi.h"
#include "disp_lcm.h"
#ifdef CONFIG_LOG_JANK
#include <huawei_platform/log/log_jank.h>
#endif
#ifdef CONFIG_HUAWEI_LCD_DSM//lcd
#include 	<dsm/dsm_pub.h>
#endif
/* This macro and arrya is designed for multiple LCM support */
/* for multiple LCM, we should assign I/F Port id in lcm driver, such as DPI0, DSI0/1 */

#include <misc/app_info.h>
#include <mt-plat/aee.h>

#ifdef CONFIG_HUAWEI_LCD_DSM//lcd
static struct dsm_dev dsm_lcd = {
	.name 		= "dsm_lcd",
	.fops 		= NULL,
	.buff_size 	= 4096,
	};
struct dsm_client *lcd_dclient = NULL;
struct dsm_client*lcd_dsm_get_client(void)
{
	return lcd_dclient;
}
#endif
char *lcd_name_for_als = NULL;
int _lcm_count(void)
{
	return lcm_count;
}

int _is_lcm_inited(disp_lcm_handle *plcm)
{
	if (plcm) {
		if (plcm->params && plcm->drv)
			return 1;

		DISPERR("WARNING,params|drv is null!\n");
		return 0;
	}

	DISPERR("WARNING, invalid lcm handle: %p\n", plcm);
	return 0;
}
LCM_PARAMS *_get_lcm_params_by_handle(disp_lcm_handle *plcm)
{
	if (plcm)
		return plcm->params;
	DISPERR("WARNING, invalid lcm handle:%p\n", plcm);
	return NULL;
}

LCM_DRIVER *_get_lcm_driver_by_handle(disp_lcm_handle *plcm)
{
	if (plcm)
		return plcm->drv;
	DISPERR("WARNING, invalid lcm handle:%p\n", plcm);
	return NULL;
}

#ifdef CONFIG_HUAWEI_LCD_DSM//lcd
int lcd_report_dsm_errno(int errno)
{
	int size = 0;
	struct dsm_client *lcd_dclient = lcd_dsm_get_client();
	if( NULL == lcd_dclient )
	{
		printk("LCD_DSM %s: there is not lcd_dclient!\n", __func__);
		return -1;
	}
	if(dsm_client_ocuppy(lcd_dclient))
	{
		/* buffer is busy */
		printk("LCD_DSM %s: buffer is busy!\n", __func__);
		return -1;
	}
	size = dsm_client_record(lcd_dclient,"lcd mipi cmd fail,error=%d\n",errno);
	/*if device is not probe successfully or client is null, don't notify dsm work func*/
	dsm_client_notify(lcd_dclient, errno);
	return size;
}

int lcd_report_dsm_err( int type, int err_value,int add_value)
{
	struct dsm_client *lcd_dclient = lcd_dsm_get_client();

	printk("LCD_DSM %s: entry! type:%d\n", __func__, type);

	if( NULL == lcd_dclient )
	{
		printk("LCD_DSM %s: there is not lcd_dclient!\n", __func__);
		return -1;
	}

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(lcd_dclient))
	{
		/* buffer is busy */
		printk("LCD_DSM %s: buffer is busy!\n", __func__);
		return -1;
	}

	/* lcd report err according to err type */
	switch(type)
	{
		case DSM_LCD_STATUS_ERROR_NO:
			printk("DSM_LCD_STATUS_ERROR_NO 0x%08x  $$$$\n",err_value);
			dsm_client_record(lcd_dclient, "lcd register %x status wrong, value :%x\n",add_value, err_value);
			break;
		case DSM_LCD_MIPI_ERROR_NO:
			dsm_client_record(lcd_dclient, "mipi transmit register %x time out ,err number :%x\n", add_value, err_value );
			break;
		case DSM_LCD_ESD_STATUS_ERROR_NO:
			  case DSM_LCD_ESD_RESET_ERROR_NO:
			 dsm_client_record(lcd_dclient, "ESD status error register %x  ,err number :%x\n", add_value, err_value );
			break;
		default:
			break;
	}

	dsm_client_notify(lcd_dclient, type);

	return 0;
}
#endif

void _dump_lcm_info(disp_lcm_handle *plcm)
{
	LCM_DRIVER *l = NULL;
	LCM_PARAMS *p = NULL;

	if (plcm == NULL) {
		DISPERR("plcm is null\n");
		return;
	}

	l = plcm->drv;
	p = plcm->params;

	if (!l || !p)
		return;

	DISPMSG("[LCM], name: %s\n", l->name);
	DISPMSG("[LCM] resolution: %d x %d\n", p->width, p->height);
	DISPMSG("[LCM] physical size: %d x %d\n", p->physical_width, p->physical_height);
	DISPMSG("[LCM] physical size: %d x %d\n", p->physical_width, p->physical_height);

	switch (p->lcm_if) {
	case LCM_INTERFACE_DSI0:
		DISPMSG("[LCM] interface: DSI0\n");
		break;
	case LCM_INTERFACE_DSI1:
		DISPMSG("[LCM] interface: DSI1\n");
		break;
	case LCM_INTERFACE_DPI0:
		DISPMSG("[LCM] interface: DPI0\n");
		break;
	case LCM_INTERFACE_DPI1:
		DISPMSG("[LCM] interface: DPI1\n");
		break;
	case LCM_INTERFACE_DBI0:
		DISPMSG("[LCM] interface: DBI0\n");
		break;
	default:
		DISPMSG("[LCM] interface: unknown\n");
		break;
	}

	switch (p->type) {
	case LCM_TYPE_DBI:
		DISPMSG("[LCM] Type: DBI\n");
		break;
	case LCM_TYPE_DSI:
		DISPMSG("[LCM] Type: DSI\n");

		break;
	case LCM_TYPE_DPI:
		DISPMSG("[LCM] Type: DPI\n");
		break;
	default:
		DISPMSG("[LCM] TYPE: unknown\n");
		break;
	}

	if (p->type == LCM_TYPE_DSI) {
		switch (p->dsi.mode) {
		case CMD_MODE:
			DISPMSG("[LCM] DSI Mode: CMD_MODE\n");
			break;
		case SYNC_PULSE_VDO_MODE:
			DISPMSG("[LCM] DSI Mode: SYNC_PULSE_VDO_MODE\n");
			break;
		case SYNC_EVENT_VDO_MODE:
			DISPMSG("[LCM] DSI Mode: SYNC_EVENT_VDO_MODE\n");
			break;
		case BURST_VDO_MODE:
			DISPMSG("[LCM] DSI Mode: BURST_VDO_MODE\n");
			break;
		default:
			DISPMSG("[LCM] DSI Mode: Unknown\n");
			break;
		}
	}

	if (p->type == LCM_TYPE_DSI) {
		DISPMSG("[LCM] LANE_NUM: %d,data_format\n", (int)p->dsi.LANE_NUM);
		DISPMSG("[LCM] vact: %u, vbp: %u, vfp: %u, vact_line: %u, hact: %u, hbp: %u, hfp: %u, hblank: %u\n",
			 p->dsi.vertical_sync_active, p->dsi.vertical_backporch,
			 p->dsi.vertical_frontporch, p->dsi.vertical_active_line,
			 p->dsi.horizontal_sync_active, p->dsi.horizontal_backporch,
			 p->dsi.horizontal_frontporch, p->dsi.horizontal_blanking_pixel);
		DISPMSG("[LCM] pll_select: %d, pll_div1: %d, pll_div2: %d, fbk_div: %d,fbk_sel: %d, rg_bir: %d\n",
			 p->dsi.pll_select, p->dsi.pll_div1, p->dsi.pll_div2, p->dsi.fbk_div,
			 p->dsi.fbk_sel, p->dsi.rg_bir);
		DISPMSG("[LCM] rg_bic: %d, rg_bp: %d,PLL_CLOCK: %d, dsi_clock: %d, ssc_range: %d,ssc_disable: %d",
			 p->dsi.rg_bic, p->dsi.rg_bp, p->dsi.PLL_CLOCK, p->dsi.dsi_clock,
			 p->dsi.ssc_range, p->dsi.ssc_disable);
		DISPMSG("[LCM]compatibility_for_nvk: %d, cont_clock: %d\n",
			 p->dsi.compatibility_for_nvk,
			 p->dsi.cont_clock);
		DISPMSG("[LCM] lcm_ext_te_enable: %d, noncont_clock: %d, noncont_clock_period: %d\n",
			 p->dsi.lcm_ext_te_enable, p->dsi.noncont_clock,
			 p->dsi.noncont_clock_period);
	}
}

disp_lcm_handle *disp_lcm_probe(char *plcm_name, LCM_INTERFACE_ID lcm_id, int is_lcm_inited)
{

	int lcmindex = 0;
	bool isLCMFound = false;
	bool isLCMInited = false;
	int i;
	LCM_DRIVER *lcm_drv = NULL;
	LCM_PARAMS *lcm_param = NULL;
	disp_lcm_handle *plcm = NULL;
	int err = 0;
	static const char *info_node = "lcd_type";

	pr_emerg("[FORGE_DISP] disp_lcm_probe ENTRY name=%s\n", plcm_name);
	DISPMSG("plcm_name=%s is_lcm_inited %d\n", plcm_name, is_lcm_inited);
	/* Log the LCM match state in the kernel ring (DISPERR FATAL strings
	 * only reach the dprec buffer), to tell which branch leaves pgc->plcm
	 * NULL: count == 0, name mismatch, or none found. */
	pr_emerg("[FORGE_DISP] disp_lcm_probe: lcm_count=%d plcm_name=%s\n", _lcm_count(), plcm_name ? plcm_name : "(null)");
	pr_emerg("[FORGE_DISP] v195 disp_lcm_probe: count=%d name='%s' drv0='%s' inited=%d\n",
		 _lcm_count(), plcm_name ? plcm_name : "(null)",
		 (_lcm_count() > 0 && lcm_driver_list[0] && lcm_driver_list[0]->name)
			 ? lcm_driver_list[0]->name : "(none)",
		 is_lcm_inited);

	lcd_name_for_als = plcm_name;
	#ifdef CONFIG_HUAWEI_LCD_DSM//lcd
	if(!lcd_dclient){
	lcd_dclient = dsm_register_client(&dsm_lcd);
	}
	if(!lcd_dclient)
		printk("LCD_DSM lcd dsm client register error %d\n",is_lcm_inited);
	else
		printk("LCD_DSM lcd dsm client register success %d\n",is_lcm_inited);
	#endif

	if (_lcm_count() == 0) {
		DISPERR("no lcm driver defined in linux kernel driver\n");
	pr_emerg("[FORGE_DISP] disp_lcm_probe: NO LCM DRIVER in kernel list\n");
		return NULL;
	} else if (_lcm_count() == 1) {
		if (plcm_name == NULL) {
			lcm_drv = lcm_driver_list[0];

			isLCMFound = true;
		pr_emerg("[FORGE_DISP] disp_lcm_probe: LCM FOUND name=%s inited=%d\n", lcm_drv->name, isLCMInited);
			isLCMInited = false;
			DISPMSG("LCM Name NULL\n");
		} else {
			lcm_drv = lcm_driver_list[0];
			if (strcmp(lcm_drv->name, plcm_name)) {
				DISPERR
					("FATAL ERROR!!!LCM Driver defined in kernel(%s) is different with LK(%s)\n",
					 lcm_drv->name, plcm_name);
				pr_emerg("[FORGE_DISP] v195 NAME MISMATCH kernel='%s' LK='%s' -> plcm NULL (black screen)\n",
					 lcm_drv->name, plcm_name);
				return NULL;
			}

			isLCMInited = true;
			isLCMFound = true;
		pr_emerg("[FORGE_DISP] disp_lcm_probe: LCM FOUND name=%s inited=%d\n", lcm_drv->name, isLCMInited);
		}

		if (!is_lcm_inited) {
			isLCMFound = true;
		pr_emerg("[FORGE_DISP] disp_lcm_probe: LCM FOUND name=%s inited=%d\n", lcm_drv->name, isLCMInited);
			isLCMInited = false;
			DISPMSG("LCM not init\n");
		}

		lcmindex = 0;
	} else {
		if (plcm_name == NULL) {
			/* TODO: we need to detect all the lcm driver */
		} else {

			for (i = 0; i < _lcm_count(); i++) {
				lcm_drv = lcm_driver_list[i];
				if (!strcmp(lcm_drv->name, plcm_name)) {
					isLCMFound = true;
		pr_emerg("[FORGE_DISP] disp_lcm_probe: LCM FOUND name=%s inited=%d\n", lcm_drv->name, isLCMInited);
					isLCMInited = true;
					lcmindex = i;
					break;
				}
			}
			if (!isLCMFound) {
				DISPERR
					("FATAL ERROR: can't found lcm driver:%s in linux kernel driver\n",
					 plcm_name);
			} else if (!is_lcm_inited) {
				isLCMInited = false;
				DISPMSG("LCM not init\n");
			}
		}
		/* TODO: */
	}

	if (isLCMFound == false) {
		DISPERR("FATAL ERROR!!!No LCM Driver defined\n");
	pr_emerg("[FORGE_DISP] disp_lcm_probe: LCM NOT FOUND isLCMFound=false\n");
		return NULL;
	}

	if(lcm_drv != NULL && lcm_drv->set_lcm_panel_support != NULL)
		lcm_drv->set_lcm_panel_support();

	if(plcm_name != NULL){
		if(!strcmp("hx8394f_hd720_dsi_vdo_truly", plcm_name)){
			err = app_info_set(info_node,plcm_name);
		}else if(!strcmp("ili9881c_hd720_dsi_vdo_dijing", plcm_name)){
			err = app_info_set(info_node,plcm_name);
		}else if(!strcmp("ili9881c_hd720_dsi_vdo_helitec", plcm_name)){
			err = app_info_set(info_node,plcm_name);
		}
	}

	if(err){
		DISPERR("%s:%d panel app_info message failed!\n",__func__,__LINE__);
	}

	plcm = kzalloc(sizeof(uint8_t *) * sizeof(disp_lcm_handle), GFP_KERNEL);
	lcm_param = kzalloc(sizeof(uint8_t *) * sizeof(LCM_PARAMS), GFP_KERNEL);
	if (plcm && lcm_param) {
		plcm->params = lcm_param;
		plcm->drv = lcm_drv;
		plcm->is_inited = isLCMInited;
		plcm->index = lcmindex;
	} else {
		DISPERR("FATAL ERROR!!!kzalloc plcm and plcm->params failed\n");
		goto FAIL;
	}

	plcm->drv->get_params(plcm->params);
	plcm->lcm_if_id = plcm->params->lcm_if;

	/* below code is for lcm driver forward compatible */
	if (plcm->params->type == LCM_TYPE_DSI
		&& plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
		plcm->lcm_if_id = LCM_INTERFACE_DSI0;
	if (plcm->params->type == LCM_TYPE_DPI
		&& plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
		plcm->lcm_if_id = LCM_INTERFACE_DPI0;
	if (plcm->params->type == LCM_TYPE_DBI
		&& plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED)
		plcm->lcm_if_id = LCM_INTERFACE_DBI0;

	if ((lcm_id == LCM_INTERFACE_NOTDEFINED) || lcm_id == plcm->lcm_if_id) {
		plcm->lcm_original_width = plcm->params->width;
		plcm->lcm_original_height = plcm->params->height;
		_dump_lcm_info(plcm);
		return plcm;
	}
	DISPERR("the specific LCM Interface [%d] didn't define any lcm driver\n",
		lcm_id);

FAIL:

	kfree(plcm);
	kfree(lcm_param);
	return NULL;
}

static void disp_lcm_m6_sram(const char *tag, int force, int inited)
{
	static unsigned int count;

	if (count >= 12)
		return;

	count++;
	aee_sram_printk("M6L%02u %s f=%d i=%d\n",
		count, tag, force, inited);
	DISPERR("M6L%02u %s f=%d i=%d\n",
		count, tag, force, inited);
}

static void disp_lcm_m6_pm_marker(const char *tag, disp_lcm_handle *plcm,
				  int ret)
{
	static unsigned int count;
	LCM_DRIVER *lcm_drv = plcm ? plcm->drv : NULL;
	int inited = -1;

	if (count >= 64)
		return;

	if (plcm && plcm->params && plcm->drv)
		inited = plcm->is_inited;

	count++;
	aee_sram_printk("M6C%02u %s i=%d r=%d\n",
		count, tag, inited, ret);
	DISPERR("M6 LCM pm[%s]#%u plcm=%p inited=%d drv=%s ret=%d has_suspend=%d has_resume=%d has_suspend_power=%d has_resume_power=%d\n",
		tag, count, plcm, inited,
		lcm_drv && lcm_drv->name ? lcm_drv->name : "unknown",
		ret, lcm_drv && lcm_drv->suspend,
		lcm_drv && lcm_drv->resume,
		lcm_drv && lcm_drv->suspend_power,
		lcm_drv && lcm_drv->resume_power);
}

int disp_lcm_init(disp_lcm_handle *plcm, int force)
{
	LCM_DRIVER *lcm_drv = NULL;
	int inited = 0;
	pr_emerg("[FORGE_DISP] disp_lcm_init ENTRY\n");


	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		inited = disp_lcm_is_inited(plcm);
		DISPERR("M6 LCM disp_lcm_init: enter force=%d inited=%d plcm=%p drv=%s\n",
			force, inited, plcm, lcm_drv->name ? lcm_drv->name : "unknown");
		disp_lcm_m6_sram("enter", force, inited);

		if (lcm_drv->init_power) {
			if (!inited || force) {
				DISPERR("M6 LCM disp_lcm_init: call init_power force=%d inited=%d\n",
					force, inited);
				disp_lcm_m6_sram("call-power", force, inited);
				pr_debug("lcm init power()\n");
				lcm_drv->init_power();
			} else {
				DISPERR("M6 LCM disp_lcm_init: skip init_power force=%d inited=%d\n",
					force, inited);
				disp_lcm_m6_sram("skip-power", force, inited);
			}
		}

		if (lcm_drv->init) {
			if (!inited || force) {
				DISPERR("M6 LCM disp_lcm_init: call init force=%d inited=%d\n",
					force, inited);
				disp_lcm_m6_sram("call-init", force, inited);
				pr_debug("lcm init()\n");
				lcm_drv->init();
			} else {
				DISPERR("M6 LCM disp_lcm_init: skip init force=%d inited=%d\n",
					force, inited);
				disp_lcm_m6_sram("skip-init", force, inited);
			}
		} else {
			DISPERR("FATAL ERROR, lcm_drv->init is null\n");
			return -1;
		}
		if (LCM_TYPE_DSI == plcm->params->type) {
			dsi_m6_dump_dcs_status("disp-lcm-init");
		}
		/* ddp_dsi_start(DISP_MODULE_DSI0, NULL); */
		/* DSI_BIST_Pattern_Test(DISP_MODULE_DSI0,NULL,true, 0x00ffff00); */
		return 0;
	}
	DISPERR("plcm is null\n");
	return -1;
}

LCM_PARAMS *disp_lcm_get_params(disp_lcm_handle *plcm)
{
	/* DISPFUNC(); */

	if (_is_lcm_inited(plcm))
		return plcm->params;
	return NULL;
}

LCM_INTERFACE_ID disp_lcm_get_interface_id(disp_lcm_handle *plcm)
{
	DISPFUNC();

	if (_is_lcm_inited(plcm))
		return plcm->lcm_if_id;

	return LCM_INTERFACE_NOTDEFINED;
}

int disp_lcm_update(disp_lcm_handle *plcm, int x, int y, int w, int h, int force)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->update) {
			lcm_drv->update(x, y, w, h);
		} else {
			if (!disp_lcm_is_video_mode(plcm))
				DISPERR("FATAL ERROR, lcm is cmd mode lcm_drv->update is null\n");
			ret = -1;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
	}
	return ret;
}

/* return 1: esd check fail */
/* return 0: esd check pass */
int disp_lcm_esd_check(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->esd_check) {
			ret = lcm_drv->esd_check();
		} else {
			DISPERR("FATAL ERROR, lcm_drv->esd_check is null\n");
			ret = 0;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = 0;
	}
	return ret;
}



int disp_lcm_esd_recover(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->esd_recover) {
			lcm_drv->esd_recover();
		} else {
			DISPERR("FATAL ERROR, lcm_drv->esd_check is null\n");
			ret = -1;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
	}
	return ret;
}

/* Panel lit. The L681 charger HAL lowers the wall-charger current while it
 * is (Meizu's temp_rise_flag, set in these two functions in the 3.10 stock
 * disp_lcm.c). 1 at boot: LK lit the panel, and a lower current is the safe
 * side. */
int forge_lcm_on = 1;

int disp_lcm_suspend(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;
#ifdef CONFIG_LOG_JANK
   LOG_JANK_D(JLID_KERNEL_LCD_POWER_OFF, "%s", "JL_KERNEL_LCD_POWER_OFF");
#endif
	DISPFUNC();
	disp_lcm_m6_pm_marker("suspend-entry", plcm, ret);
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->suspend) {
			disp_lcm_m6_pm_marker("suspend-before-callback", plcm, ret);
			lcm_drv->suspend();
			forge_lcm_on = 0;
			disp_lcm_m6_pm_marker("suspend-after-callback", plcm, ret);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->suspend is null\n");
			ret = -1;
			disp_lcm_m6_pm_marker("suspend-missing-callback", plcm, ret);
		}
		if (lcm_drv->suspend_power) {
			disp_lcm_m6_pm_marker("suspend-before-power", plcm, ret);
			lcm_drv->suspend_power();
			disp_lcm_m6_pm_marker("suspend-after-power", plcm, ret);
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
		disp_lcm_m6_pm_marker("suspend-not-inited", plcm, ret);
	}
	disp_lcm_m6_pm_marker("suspend-exit", plcm, ret);
	return ret;
}

int disp_lcm_resume(disp_lcm_handle *plcm)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;

	DISPFUNC();
	forge_lcm_on = 1;
	disp_lcm_m6_pm_marker("resume-entry", plcm, ret);
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->resume_power) {
			disp_lcm_m6_pm_marker("resume-before-power", plcm, ret);
			lcm_drv->resume_power();
			disp_lcm_m6_pm_marker("resume-after-power", plcm, ret);
		}
		if (lcm_drv->resume) {
			disp_lcm_m6_pm_marker("resume-before-callback", plcm, ret);
			lcm_drv->resume();
			disp_lcm_m6_pm_marker("resume-after-callback", plcm, ret);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->resume is null\n");
			ret = -1;
			disp_lcm_m6_pm_marker("resume-missing-callback", plcm, ret);
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
		disp_lcm_m6_pm_marker("resume-not-inited", plcm, ret);
	}
	disp_lcm_m6_pm_marker("resume-exit", plcm, ret);
	return ret;
}

int disp_lcm_set_backlight(disp_lcm_handle *plcm, void *handle, int level)
{
	LCM_DRIVER *lcm_drv = NULL;
	int ret = 0;

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_backlight_cmdq) {
			lcm_drv->set_backlight_cmdq(handle, level);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->set_backlight is null\n");
			ret = -1;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
	}
	return ret;
}

#ifdef CONFIG_LCT_CABC_MODE_SUPPORT
int disp_lcm_set_cabc(disp_lcm_handle *plcm, void *handle, unsigned int mode)
{
	LCM_DRIVER *lcm_drv = NULL;
	DISPFUNC();
	if (_is_lcm_inited(plcm))
	{
		lcm_drv = plcm->drv;
		if (lcm_drv->set_cabc_cmdq)
		{
			lcm_drv->set_cabc_cmdq(handle, mode);
		}
		else
		{
			DISPERR("FATAL ERROR, lcm_drv->set_cabc is null\n");
			return -1;
		}
		return 0;
	}
	else
	{
		DISPERR("lcm_drv is null\n");
		return -1;
	}
}
#endif

int disp_lcm_ioctl(disp_lcm_handle *plcm, LCM_IOCTL ioctl, unsigned int arg)
{
	return 0;
}

int disp_lcm_is_inited(disp_lcm_handle *plcm)
{
	if (_is_lcm_inited(plcm))
		return plcm->is_inited;
	else
		return 0;
}

unsigned int disp_lcm_ATA(disp_lcm_handle *plcm)
{
	unsigned int ret = 0;
	LCM_DRIVER *lcm_drv = NULL;

	DISPFUNC();
	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->ata_check) {

			ret = lcm_drv->ata_check(NULL);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->ata_check is null\n");
			ret = 0;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = 0;
	}
	return ret;
}

void *disp_lcm_switch_mode(disp_lcm_handle *plcm, int mode)
{
	LCM_DRIVER *lcm_drv = NULL;
	LCM_DSI_MODE_SWITCH_CMD *lcm_cmd = NULL;

	if (_is_lcm_inited(plcm)) {
		if (plcm->params->dsi.switch_mode_enable == 0) {
			DISPERR(" ERROR, Not enable switch in lcm_get_params function\n");
			return NULL;
		}
		lcm_drv = plcm->drv;
		if (lcm_drv->switch_mode) {
			lcm_cmd = (LCM_DSI_MODE_SWITCH_CMD *) lcm_drv->switch_mode(mode);
			lcm_cmd->cmd_if = (unsigned int)(plcm->params->lcm_cmd_if);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->switch_mode is null\n");
			return NULL;
		}
		return (void *)(lcm_cmd);
	}
	DISPERR("lcm_drv is null\n");
	return NULL;
}

int disp_lcm_is_video_mode(disp_lcm_handle *plcm)
{
	LCM_PARAMS *lcm_param = NULL;

	if (_is_lcm_inited(plcm))
		lcm_param = plcm->params;
	else
		BUG();

	switch (lcm_param->type) {
	case LCM_TYPE_DBI:
		return false;
	case LCM_TYPE_DSI:
		break;
	case LCM_TYPE_DPI:
		return true;
	default:
		DISPMSG("[LCM] TYPE: unknown\n");
		break;
	}

	if (lcm_param->type == LCM_TYPE_DSI) {
		switch (lcm_param->dsi.mode) {
		case CMD_MODE:
			return false;
		case SYNC_PULSE_VDO_MODE:
		case SYNC_EVENT_VDO_MODE:
		case BURST_VDO_MODE:
			return true;
		default:
			DISPMSG("[LCM] DSI Mode: Unknown\n");
			break;
		}
	}

	BUG();
	return 0;
}

int disp_lcm_set_lcm_cmd(disp_lcm_handle *plcm, void *cmdq_handle, unsigned int *lcm_cmd,
			 unsigned int *lcm_count, unsigned int *lcm_value)
{
	int ret = 0;

	LCM_DRIVER *lcm_drv = NULL;

	if (_is_lcm_inited(plcm)) {
		lcm_drv = plcm->drv;
		if (lcm_drv->set_lcm_cmd) {
			lcm_drv->set_lcm_cmd(cmdq_handle, lcm_cmd, lcm_count, lcm_value);
		} else {
			DISPERR("FATAL ERROR, lcm_drv->set_lcm_cmd is null\n");
			ret = -1;
		}
	} else {
		DISPERR("lcm_drv is null\n");
		ret = -1;
	}
	return ret;
}
