/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#ifndef __MTK_COMMON_H__
#define __MTK_COMMON_H__

/* MTK device context driver private data */
struct mtk_config {

    /* mtk mfg mapped register */
    volatile void *mfg_register;

    struct clk *clk_mfg;
    struct clk *clk_mfg_main;
    struct clk *clk_smi_common;
    /* SCP_SYS_DIS is held across GPU power-on, as on 3.18 (the SMI-common
     * CG register lives in MMSYS = DIS domain). */
    struct clk *clk_display_scp;
#ifdef ENABLE_MFG_PERF
    struct clk *clk_mfg_f52m_sel;
    bool mfg_perf_enabled;
#endif

};

#ifdef ENABLE_MFG_PERF
/* MTK HAL mtk_enable_gpu_perf_monitor implementation */
bool _mtk_enable_gpu_perf_monitor(bool enable);
#endif

#define MTKCLK_prepare_enable(clk) \
	if (config->clk) { \
		if (clk_prepare_enable(config->clk)) \
		pr_alert("MALI: clk_prepare_enable failed when enabling" #clk); }

#define MTKCLK_disable_unprepare(clk) \
	if (config->clk) { \
		clk_disable_unprepare(config->clk); }



#endif /* __MTK_COMMON_H__ */
