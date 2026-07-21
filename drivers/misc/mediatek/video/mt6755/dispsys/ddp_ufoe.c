extern int ddp_debug_analysis_to_buffer(void);
/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
#include "ddp_debug.h"
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "UFOE"
#include "disp_log.h"
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif
#include <linux/delay.h>

#include "ddp_info.h"
#include "ddp_clkmgr.h"
#include "ddp_hal.h"
#include "ddp_reg.h"

static bool ufoe_enable;

static int ufoe_dump(DISP_MODULE_ENUM module, int level)
{
	(void)module;
	(void)level;

	DISPDMP("==DISP UFOE ROUTE== enable=%d\n", ufoe_enable);
	DISPDMP("DISP_UFOE_MOUT_EN=0x%x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_UFOE_MOUT_EN));
	DISPDMP("DISP_UFOE_SEL_IN=0x%x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_UFOE_SEL_IN));
	DISPDMP("MMSYS_CG_CON0=0x%x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return 0;
}

static int ufoe_init(DISP_MODULE_ENUM module, void *handle)
{
	(void)module;
	(void)handle;

	DISPMSG("M6 DDP ufoe init route-only CG=0x%x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return 0;
}

static int ufoe_deinit(DISP_MODULE_ENUM module, void *handle)
{
	(void)module;
	(void)handle;

	DISPMSG("M6 DDP ufoe deinit route-only CG=0x%x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return 0;
}

static int ufoe_start(DISP_MODULE_ENUM module, void *handle)
{
	(void)module;
	(void)handle;

	DISPMSG("M6 DDP ufoe start route-only enable=%d\n", ufoe_enable);
	return 0;
}

static int ufoe_stop(DISP_MODULE_ENUM module, void *handle)
{
	(void)module;
	(void)handle;

	DISPMSG("M6 DDP ufoe stop route-only enable=%d\n", ufoe_enable);
	return 0;
}

static int ufoe_config(DISP_MODULE_ENUM module, disp_ddp_path_config *pConfig, void *handle)
{
	LCM_PARAMS *disp_if_config = &(pConfig->dispif_config);
	LCM_DSI_PARAMS *lcm_config = &(disp_if_config->dsi);

	(void)module;
	(void)handle;

	ufoe_enable = !!lcm_config->ufoe_enable;
	DISPMSG("M6 DDP ufoe config route-only enable=%d %ux%u\n",
		ufoe_enable, disp_if_config->width, disp_if_config->height);
	return 0;
}

static int ufoe_reset(DISP_MODULE_ENUM module, void *handle)
{
	(void)module;
	(void)handle;

	DISPMSG("M6 DDP ufoe reset route-only\n");
	return 0;
}
static int ufoe_clock_on(DISP_MODULE_ENUM module, void *handle)
{
	int ret = 0;

#ifdef CONFIG_MTK_CLKMGR
	ret = enable_clock(MT_CG_DISP0_DISP_UFOE, "ufoe");
#else
	ret = ddp_clk_enable(DISP0_DISP_UFOE_MOUT);
#endif
	DISPMSG("M6 DDP clk: ufoe on ret=%d CG=0x%x\n", ret,
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return ret;
}

static int ufoe_clock_off(DISP_MODULE_ENUM module, void *handle)
{
	int ret = 0;

#ifdef CONFIG_MTK_CLKMGR
	disable_clock(MT_CG_DISP0_DISP_UFOE, "ufoe");
#else
	ret = ddp_clk_disable(DISP0_DISP_UFOE_MOUT);
#endif
	DISPMSG("M6 DDP clk: ufoe off ret=%d CG=0x%x\n", ret,
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
	return ret;
}


/* ufoe */
DDP_MODULE_DRIVER ddp_driver_ufoe = {
	.init = ufoe_init,
	.deinit = ufoe_deinit,
	.config = ufoe_config,
	.start = ufoe_start,
	.trigger = NULL,
	.stop = ufoe_stop,
	.reset = ufoe_reset,
	.power_on = ufoe_clock_on,
	.power_off = ufoe_clock_off,
	.is_idle = NULL,
	.is_busy = NULL,
	.dump_info = ufoe_dump,
	.bypass = NULL,
	.build_cmdq = NULL,
	.set_lcm_utils = NULL,
};
