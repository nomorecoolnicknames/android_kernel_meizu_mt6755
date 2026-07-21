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


#define LOG_TAG "IRQ"

#include "disp_log.h"
#include "disp_debug.h"
#include "mtkfb_debug.h"

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/timer.h>

#include "ddp_reg.h"
#include "ddp_irq.h"
#include "ddp_ovl.h"
#include "ddp_aal.h"
#include "ddp_drv.h"
#include "disp_helper.h"

/* IRQ log print kthread */
static struct task_struct *disp_irq_log_task;
static wait_queue_head_t disp_irq_log_wq;
static int disp_irq_log_module_en;

static int irq_init;

static unsigned int disp_irq_log_module[DISP_MODULE_NUM];
static unsigned int cnt_rdma_underflow[2];
static unsigned int cnt_rdma_abnormal[2];
static unsigned int cnt_ovl_underflow[OVL_NUM];
static unsigned int cnt_ovl_abnormal_sof[OVL_NUM];
static unsigned int cnt_wdma_underflow[2];
static unsigned int m6_ovl0_irq_diag_count;
static unsigned int m6_rdma0_irq_diag_count;
static unsigned int m6_dsi0_irq_diag_count;
static unsigned int m6_mutex_irq_diag_count;

unsigned long long rdma_start_time[2] = { 0 };
unsigned long long rdma_end_time[2] = { 0 };

unsigned int mmsys_debug[4] = {0};
unsigned int mmsys_enable = 4;



#define DISP_MAX_IRQ_CALLBACK   10
#define M6_OVL_LAYER_OFFSET     (0x20)
#define M6_OVL_RDMA_DBG_OFFSET  (0x4)

static bool disp_irq_m6_diag_sample(unsigned int *count)
{
	unsigned int n = (*count)++;

	return n < 24;
}

static const char *disp_irq_m6_ovl_fsm_name(unsigned int fsm)
{
	switch (fsm) {
	case 0x1:
		return "idle";
	case 0x2:
		return "wait_SOF";
	case 0x4:
		return "prepare";
	case 0x8:
		return "reg_update";
	case 0x10:
		return "eng_clr";
	case 0x20:
		return "eng_act";
	case 0x40:
		return "h_wait_w_rst";
	case 0x80:
		return "s_wait_w_rst";
	case 0x100:
		return "h_w_rst";
	case 0x200:
		return "s_w_rst";
	default:
		return "unknown";
	}
}

static void disp_irq_m6_dump_ovl0_layer(unsigned int idx, unsigned long base,
	unsigned int layer, const struct m6_ovl_config_snapshot *snap)
{
	unsigned long layer_off = layer * M6_OVL_LAYER_OFFSET;
	unsigned long rdma_dbg_off = layer * M6_OVL_RDMA_DBG_OFFSET;
	const struct m6_ovl_layer_snapshot *req = NULL;
	unsigned int src_on = DISP_REG_GET(base + DISP_REG_OVL_SRC_CON);
	unsigned int live_size = DISP_REG_GET(base + DISP_REG_OVL_L0_SRC_SIZE + layer_off);
	unsigned int live_pitch = DISP_REG_GET(base + DISP_REG_OVL_L0_PITCH + layer_off);
	unsigned int live_addr = DISP_REG_GET(base + DISP_REG_OVL_L0_ADDR + layer_off);

	if (snap && snap->seq && layer < ARRAY_SIZE(snap->layer) &&
	    snap->layer[layer].valid)
		req = &snap->layer[layer];

	DISPERR("M6 OVL irq diag[%u]: L%u live en=%u con=0x%x size=%ux%u off=0x%x addr=0x%x pitch=0x%x rdma_ctrl=0x%x gmc=0x%x slow=0x%x fifo=0x%x buflow=0x%x rdma_dbg=0x%x\n",
		idx, layer, !!(src_on & (1U << layer)),
		DISP_REG_GET(base + DISP_REG_OVL_L0_CON + layer_off),
		live_size & 0xfff, (live_size >> 16) & 0xfff,
		DISP_REG_GET(base + DISP_REG_OVL_L0_OFFSET + layer_off),
		live_addr, live_pitch,
		DISP_REG_GET(base + DISP_REG_OVL_RDMA0_CTRL + layer_off),
		DISP_REG_GET(base + DISP_REG_OVL_RDMA0_MEM_GMC_SETTING + layer_off),
		DISP_REG_GET(base + DISP_REG_OVL_RDMA0_MEM_SLOW_CON + layer_off),
		DISP_REG_GET(base + DISP_REG_OVL_RDMA0_FIFO_CTRL + layer_off),
		DISP_REG_GET(base + DISP_REG_OVL_RDMAn_BUF_LOW(layer)),
		DISP_REG_GET(base + DISP_REG_OVL_RDMA0_DBG + rdma_dbg_off));

	if (!req)
		return;

	DISPERR("M6 OVL irq diag[%u]: L%u req seq=%u global=%u en=%u src=%u fmt=0x%x bpp=%u sec=%u addr=0x%lx final=0x%lx visible_last=0x%lx pitch_end=0x%lx src=%u/%u/%u/%u pitch=%u dst=%u/%u/%u/%u hw_h=%u bounds=%u\n",
		idx, layer, snap->seq, req->global_layer, req->enabled,
		req->source, req->fmt, req->bpp, req->security, req->addr,
		req->final_addr, req->visible_last, req->pitch_end,
		req->src_x, req->src_y, req->src_w, req->src_h,
		req->src_pitch, req->dst_x, req->dst_y, req->dst_w,
		req->dst_h, req->hw_dst_h, req->bounds_profile);
}

static void disp_irq_m6_dump_ovl0_state(DISP_MODULE_ENUM module,
	unsigned int intsta)
{
	unsigned long base;
	unsigned int ovl_greq_num;
	unsigned int ovl_greq_urg;
	unsigned int larb0_greq;
	unsigned int flow;
	unsigned int idx;
	unsigned int layer;
	struct m6_ovl_config_snapshot snap = { 0 };
	int has_snap;

	if (module != DISP_MODULE_OVL0 ||
	    !disp_helper_get_option(DISP_OPT_BYPASS_PQ) ||
	    !(intsta & ((1U << 2) | (0xfU << 5) | (1U << 13))))
		return;

	if (!disp_irq_m6_diag_sample(&m6_ovl0_irq_diag_count))
		return;

	idx = m6_ovl0_irq_diag_count - 1;
	base = ovl_base_addr(module);
	has_snap = ovl_m6_get_last_config_snapshot(&snap);
	ovl_greq_num = DISP_REG_GET(base + DISP_REG_OVL_RDMA_GREQ_NUM);
	ovl_greq_urg = DISP_REG_GET(base + DISP_REG_OVL_RDMA_GREQ_URG_NUM);
	larb0_greq = DISP_REG_GET(DISP_REG_CONFIG_SMI_LARB0_GREQ);
	flow = DISP_REG_GET(base + DISP_REG_OVL_FLOW_CTRL_DBG);

	DISPERR("M6 OVL irq diag[%u]: intsta=0x%x sta=0x%x inten=0x%x en=0x%x src=0x%x roi=0x%x path=0x%x flow=0x%x addcon=0x%x smi=0x%x ovl_greq=0x%x ovl_urg=0x%x larb0_greq=0x%x valid=0x%x ready=0x%x mutex=0x%x/0x%x rdma=0x%x in=%u/%u out=%u/%u\n",
		idx, intsta,
		DISP_REG_GET(base + DISP_REG_OVL_STA),
		DISP_REG_GET(base + DISP_REG_OVL_INTEN),
		DISP_REG_GET(base + DISP_REG_OVL_EN),
		DISP_REG_GET(base + DISP_REG_OVL_SRC_CON),
		DISP_REG_GET(base + DISP_REG_OVL_ROI_SIZE),
		DISP_REG_GET(base + DISP_REG_OVL_DATAPATH_CON),
		flow,
		DISP_REG_GET(base + DISP_REG_OVL_ADDCON_DBG),
		DISP_REG_GET(base + DISP_REG_OVL_SMI_DBG),
		ovl_greq_num,
		ovl_greq_urg,
		larb0_greq,
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF),
		DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON),
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT));
	DISPERR("M6 OVL irq diag[%u]: flow decode fsm=0x%x/%s addcon_idle=%u blend_idle=%u out_valid=%u out_ready=%u out_idle=%u rdma_idle=%u/%u/%u/%u rst=%u trig=%u hwrst_done=%u swrst_done=%u underrun=%u done=%u running=%u start=%u clr=%u reg_update=%u upd_reg=%u\n",
		idx,
		flow & 0x3ff, disp_irq_m6_ovl_fsm_name(flow & 0x3ff),
		(flow >> 10) & 0x1, (flow >> 11) & 0x1,
		(flow >> 12) & 0x1, (flow >> 13) & 0x1,
		(flow >> 15) & 0x1, (flow >> 19) & 0x1,
		(flow >> 18) & 0x1, (flow >> 17) & 0x1,
		(flow >> 16) & 0x1, (flow >> 20) & 0x1,
		(flow >> 21) & 0x1, (flow >> 23) & 0x1,
		(flow >> 24) & 0x1, (flow >> 25) & 0x1,
		(flow >> 26) & 0x1, (flow >> 27) & 0x1,
		(flow >> 28) & 0x1, (flow >> 29) & 0x1,
		(flow >> 30) & 0x1, (flow >> 31) & 0x1);
	DISPERR("M6 OVL irq diag[%u]: ovl_greq decode layer=%u/%u/%u/%u ostd=0x%x dis=%u flush_pre=%u flush_ultra=%u urg_layer=%u/%u/%u/%u urg_th=0x%x urg_bias=%u\n",
		idx,
		ovl_greq_num & 0x7,
		(ovl_greq_num >> 4) & 0x7,
		(ovl_greq_num >> 8) & 0x7,
		(ovl_greq_num >> 12) & 0x7,
		(ovl_greq_num >> 16) & 0xff,
		(ovl_greq_num >> 24) & 0x7,
		(ovl_greq_num >> 28) & 0x1,
		(ovl_greq_num >> 29) & 0x1,
		ovl_greq_urg & 0x7,
		(ovl_greq_urg >> 4) & 0x7,
		(ovl_greq_urg >> 8) & 0x7,
		(ovl_greq_urg >> 12) & 0x7,
		(ovl_greq_urg >> 16) & 0x3ff,
		(ovl_greq_urg >> 28) & 0x1);
	if (has_snap)
		DISPERR("M6 OVL irq diag[%u]: last cfg seq=%u enabled=0x%x first_global=%u scan=0x%x->0x%x dst=%ux%u sec=%u cmdq=%u direct=%u bypass_pq=%u\n",
			idx, snap.seq, snap.enabled_layers, snap.first_global_layer,
			snap.scanned_before, snap.scanned_after, snap.dst_w,
			snap.dst_h, snap.has_sec_layer, snap.cmdq, snap.direct,
			snap.bypass_pq);
	else
		DISPERR("M6 OVL irq diag[%u]: last cfg missing\n", idx);

	for (layer = 0; layer < 4; layer++)
		disp_irq_m6_dump_ovl0_layer(idx, base, layer,
			has_snap ? &snap : NULL);
}

static DDP_IRQ_CALLBACK irq_module_callback_table[DISP_MODULE_NUM][DISP_MAX_IRQ_CALLBACK];
static DDP_IRQ_CALLBACK irq_callback_table[DISP_MAX_IRQ_CALLBACK];

/* dsi read by cpu should keep esd_check_bycmdq = 0.  */
/* dsi read by cmdq should keep esd_check_bycmdq = 1. */
atomic_t ESDCheck_byCPU = ATOMIC_INIT(0);

void disp_irq_esd_cust_bycmdq(int enable)
{
	atomic_set(&ESDCheck_byCPU, enable ? 0 : 1);
}

int disp_irq_esd_cust_get(void)
{
	return atomic_read(&ESDCheck_byCPU);
}

int disp_register_irq_callback(DDP_IRQ_CALLBACK cb)
{
	int i = 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == cb)
			break;
	}
	if (i < DISP_MAX_IRQ_CALLBACK)
		return 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == NULL)
			break;
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DISPERR("not enough irq callback entries for module\n");
		return -1;
	}
	DISPMSG("register callback on %d\n", i);
	irq_callback_table[i] = cb;
	return 0;
}

int disp_unregister_irq_callback(DDP_IRQ_CALLBACK cb)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == cb) {
			irq_callback_table[i] = NULL;
			break;
		}
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DISPERR("Try to unregister callback function %p which was not registered\n", cb);
		return -1;
	}
	return 0;
}

int disp_register_module_irq_callback(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
	int i;

	if (module >= DISP_MODULE_NUM) {
		DISPERR("Register IRQ with invalid module ID. module=%d\n", module);
		return -1;
	}
	if (cb == NULL) {
		DISPERR("Register IRQ with invalid cb.\n");
		return -1;
	}
	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == cb)
			break;
	}
	if (i < DISP_MAX_IRQ_CALLBACK)
		return 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == NULL)
			break;
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DISPERR("No enough callback entries for module %d.\n", module);
		return -1;
	}
	irq_module_callback_table[module][i] = cb;
	return 0;
}

int disp_unregister_module_irq_callback(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == cb) {
			irq_module_callback_table[module][i] = NULL;
			break;
		}
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DISPERR
		    ("Try to unregister callback function with was not registered. module=%d cb=%p\n",
		     module, cb);
		return -1;
	}
	return 0;
}

void disp_invoke_irq_callbacks(DISP_MODULE_ENUM module, unsigned int param)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {

		if (irq_callback_table[i]) {
			/* DISPERR("Invoke callback function. module=%d param=0x%X\n", module, param); */
			irq_callback_table[i] (module, param);
		}

		if (irq_module_callback_table[module][i]) {
			/* DISPERR("Invoke module callback function. module=%d param=0x%X\n", module, param); */
			irq_module_callback_table[module][i] (module, param);
		}
	}
}

static DISP_MODULE_ENUM disp_irq_module(unsigned int irq)
{
	DISP_REG_ENUM reg_module;

	for (reg_module = 0; reg_module < DISP_REG_NUM; reg_module++) {
		if (irq == dispsys_irq[reg_module])
			return ddp_get_reg_module(reg_module);
	}
	DISPERR("cannot find module for irq %d\n", irq);
	BUG();
	return DISP_MODULE_UNKNOWN;
}

/* /TODO:  move each irq to module driver */
unsigned int rdma_start_irq_cnt[2] = { 0, 0 };
unsigned int rdma_done_irq_cnt[2] = { 0, 0 };
unsigned int rdma_underflow_irq_cnt[2] = { 0, 0 };
unsigned int rdma_targetline_irq_cnt[2] = { 0, 0 };

static void disp_irq_m6_dump_primary_path_state(const char *tag,
	unsigned int intsta, unsigned int *count)
{
	unsigned int idx;

	if (!disp_irq_m6_diag_sample(count))
		return;

	idx = *count - 1;
	DISPERR("M6 DDP irq diag[%u][%s]: intsta=0x%x route valid=0x%x ready=0x%x mutex INTEN=0x%x INTSTA=0x%x M0_EN=0x%x M0_MOD=0x%x M0_SOF=0x%x\n",
		idx, tag, intsta,
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF));
	DISPERR("M6 DDP irq diag[%u][%s]: rdma0 INTEN=0x%x INTSTA=0x%x GLOBAL=0x%x SIZE=%ux%u FIFO=0x%x FIFO_CON=0x%x IN=%u/%u OUT=%u/%u irqcnt s/d/u/t=%u/%u/%u/%u\n",
		idx, tag,
		DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE),
		DISP_REG_GET(DISP_REG_RDMA_INT_STATUS),
		DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON),
		DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0),
		DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1),
		DISP_REG_GET(DISP_REG_RDMA_FIFO_LOG),
		DISP_REG_GET(DISP_REG_RDMA_FIFO_CON),
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT),
		rdma_start_irq_cnt[0], rdma_done_irq_cnt[0],
		rdma_underflow_irq_cnt[0], rdma_targetline_irq_cnt[0]);
	DISPERR("M6 DDP irq diag[%u][%s]: ovl0 INTSTA=0x%x EN=0x%x SRC=0x%x STA=0x%x FLOW=0x%x ADDCON=0x%x dsi0 START=0x%x STA=0x%x INTEN=0x%x INTSTA=0x%x MODE=0x%x VM_CMD=0x%x STATE_DBG=0x%x/0x%x/0x%x/0x%x DBG6=0x%x VACT_NL=0x%x\n",
		idx, tag,
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_INTSTA),
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_EN),
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_SRC_CON),
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_STA),
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_FLOW_CTRL_DBG),
		DISP_REG_GET(DISPSYS_OVL0_BASE + DISP_REG_OVL_ADDCON_DBG),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x000),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x004),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x008),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x00c),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x014),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x130),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x148),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x14c),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x150),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x154),
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x160),	/* DSI_STATE_DBG6 (VDO VSA/VBP/VACT/VFP phase) */
		DISP_REG_GET(DISPSYS_DSI0_BASE + 0x02c));	/* DSI_VACT_NL (programmed active-line count) */
}

irqreturn_t disp_irq_handler(int irq, void *dev_id)
{
	DISP_MODULE_ENUM module = DISP_MODULE_UNKNOWN;
	unsigned int reg_val = 0;
	unsigned int index = 0;
	unsigned int mutexID = 0;
	unsigned int reg_temp_val = 0;
	unsigned int j = 0;
	unsigned int tmp_mmsys_debug[4] = {0};

	DISPIRQ("disp_irq_handler, irq=%d, module=%s\n",
	       irq, ddp_get_module_name(disp_irq_module(irq)));

	if (irq == dispsys_irq[DISP_REG_DSI0]) {
		module = DISP_MODULE_DSI0;
		reg_val = (DISP_REG_GET(dsi_reg_va + 0xC) & 0xff);
		reg_temp_val = reg_val;
		/* rd_rdy don't clear and wait for ESD & Read LCM will clear the bit. */
		if (disp_irq_esd_cust_get() == 0)
			reg_temp_val = reg_val&0xfffe;
		reg_temp_val = reg_temp_val&0xffdf;
		if (reg_val) {
			/* Log the DSI0 raw IRQ status (INTSTA reg 0xC, low byte)
			 * for the first interrupts: does VM_DONE (bit3) ever
			 * set? */
			static int _de125;

			if (_de125 < 8) {
				_de125++;
				pr_emerg("[FORGE_DISP] #125 DSI0 IRQ raw INTSTA=0x%x VM_DONE(b3)=%d #%d\n",
					 reg_val, !!(reg_val & (1 << 3)), _de125);
			}
			disp_irq_m6_dump_primary_path_state("dsi0", reg_val,
				&m6_dsi0_irq_diag_count);
		}
		DISP_CPU_REG_SET(dsi_reg_va + 0xC, ~reg_temp_val);
	} else if (irq == dispsys_irq[DISP_REG_OVL0] ||
		   irq == dispsys_irq[DISP_REG_OVL1] ||
		   irq == dispsys_irq[DISP_REG_OVL0_2L] || irq == dispsys_irq[DISP_REG_OVL1_2L]
	    ) {
		module = disp_irq_module(irq);
		index = ovl_to_index(module);
		reg_val = DISP_REG_GET(DISP_REG_OVL_INTSTA + ovl_base_addr(module));
		if (reg_val & (1 << 0))
			DISPIRQ("IRQ: %s reg commit!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 1))
			DISPIRQ("IRQ: %s frame done!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 2))
			DISPERR("IRQ: %s frame underflow! cnt=%d\n", ddp_get_module_name(module),
			       cnt_ovl_underflow[index]++);

		if (reg_val & (1 << 3))
			DISPIRQ("IRQ: %s sw reset done\n", ddp_get_module_name(module));

		if (reg_val & (1 << 4))
			DISPERR("IRQ: %s hw reset done\n", ddp_get_module_name(module));

		if (reg_val & (1 << 5))
			DISPERR("IRQ: %s-L0 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 6))
			DISPERR("IRQ: %s-L1 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 7))
			DISPERR("IRQ: %s-L2 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 8))
			DISPERR("IRQ: %s-L3 not complete until EOF!\n",
			       ddp_get_module_name(module));
#if 0
		/* we don't care ovl underflow, it's not error */
		if (reg_val & (1 << 9))
			DISPERR("IRQ: %s-L0 fifo underflow!\n", ddp_get_module_name(module));


		if (reg_val & (1 << 10))
			DISPERR("IRQ: %s-L1 fifo underflow!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 11))
			DISPERR("IRQ: %s-L2 fifo underflow!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 12))
			DISPERR("IRQ: %s-L3 fifo underflow!\n", ddp_get_module_name(module));
#endif
		if (reg_val & (1 << 13)) {
			unsigned int abnormal = cnt_ovl_abnormal_sof[index]++;

			if (abnormal < 24 || ((abnormal & 0x3ff) == 0))
				DISPERR("IRQ: %s abnormal SOF! cnt=%u\n",
					ddp_get_module_name(module), abnormal);
		}

		disp_irq_m6_dump_ovl0_state(module, reg_val);

		DISP_CPU_REG_SET(DISP_REG_OVL_INTSTA + ovl_base_addr(module), ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->OVL_IRQ[index], MMProfileFlagPulse, reg_val,
			       0);
		if (reg_val & 0x1e0)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (index << 16) | reg_val, module);

	} else if (irq == dispsys_irq[DISP_REG_WDMA0] || irq == dispsys_irq[DISP_REG_WDMA1]) {
		index = (irq == dispsys_irq[DISP_REG_WDMA0]) ? 0 : 1;
		module =
		    (irq == dispsys_irq[DISP_REG_WDMA0]) ? DISP_MODULE_WDMA0 : DISP_MODULE_WDMA1;
		reg_val = DISP_REG_GET(DISP_REG_WDMA_INTSTA + index * DISP_WDMA_INDEX_OFFSET);
		if (reg_val & (1 << 0))
			DISPIRQ("IRQ: WDMA%d frame done!\n", index);

		if (reg_val & (1 << 1)) {
			DISPERR("IRQ: WDMA%d underrun! cnt=%d\n", index,
			       cnt_wdma_underflow[index]++);
			disp_irq_log_module[module] = 1;
		}
		/* clear intr */
		DISP_CPU_REG_SET(DISP_REG_WDMA_INTSTA + index * DISP_WDMA_INDEX_OFFSET, ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->WDMA_IRQ[index], MMProfileFlagPulse, reg_val,
			       DISP_REG_GET(DISP_REG_WDMA_CLIP_SIZE));
		if (reg_val & 0x2)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (cnt_wdma_underflow[index] << 24) | (index << 16) | reg_val,
				       module);

	} else if (irq == dispsys_irq[DISP_REG_RDMA0] || irq == dispsys_irq[DISP_REG_RDMA1]) {
		if (dispsys_irq[DISP_REG_RDMA0] == irq) {
			index = 0;
			module = DISP_MODULE_RDMA0;
		} else if (dispsys_irq[DISP_REG_RDMA1] == irq) {
			index = 1;
			module = DISP_MODULE_RDMA1;
		}

		reg_val = DISP_REG_GET(DISP_REG_RDMA_INT_STATUS + index * DISP_RDMA_INDEX_OFFSET);
		if (reg_val & (1 << 0))
			DISPIRQ("IRQ: RDMA%d reg update done!\n", index);

		if (reg_val & (1 << 2)) {
			/* Log the first RDMA frame-done (EOF) IRQs. */
			{
				static int _re125;

				if (_re125 < 6) {
					_re125++;
					pr_emerg("[FORGE_DISP] #125 RDMA%d frame-done(EOF) IRQ raw INTSTA=0x%x #%d\n",
						 index, reg_val, _re125);
				}
			}
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagEnd,
				       reg_val, 0);
			rdma_end_time[index] = sched_clock();
			tmp_mmsys_debug[2] = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0);
			tmp_mmsys_debug[3] = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW0_RST_B);

			if ((tmp_mmsys_debug[2] != mmsys_debug[2])
				|| (tmp_mmsys_debug[3] != mmsys_debug[3])) {
				mmsys_debug[2] = tmp_mmsys_debug[2];
				mmsys_debug[3] = tmp_mmsys_debug[3];
				if (mmsys_enable > 0) {
					DISPMSG("cg_e = %x, rst_e = %x", mmsys_debug[2], mmsys_debug[3]);
					mmsys_enable--;
					}
				}
			DISPIRQ("IRQ: RDMA%d frame done!\n", index);
			rdma_done_irq_cnt[index]++;
		}
		if (reg_val & (1 << 1)) {
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index],
				       MMProfileFlagStart, reg_val, 0);
			rdma_start_time[index] = sched_clock();
			tmp_mmsys_debug[0] = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0);
			tmp_mmsys_debug[1] = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW0_RST_B);

			if ((tmp_mmsys_debug[0] != mmsys_debug[0])
				|| (tmp_mmsys_debug[1] != mmsys_debug[1])) {
				mmsys_debug[0] = tmp_mmsys_debug[0];
				mmsys_debug[1] = tmp_mmsys_debug[1];
				if (mmsys_enable > 0) {
					DISPMSG("cg_s = %x, rst_s = %x", mmsys_debug[0], mmsys_debug[1]);
					mmsys_enable--;
					}
				}
			DISPIRQ("IRQ: RDMA%d frame start!\n", index);
			rdma_start_irq_cnt[index]++;
		}
		if (reg_val & (1 << 3)) {
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagPulse,
				       reg_val, 0);

			DISPERR("IRQ: RDMA%d abnormal! cnt=%d\n", index, cnt_rdma_abnormal[index]++);
			disp_irq_log_module[module] = 1;

		}
		if (reg_val & (1 << 4)) {

			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagPulse,
				       reg_val, 1);

			DISPMSG("rdma%d, pix(%d,%d,%d,%d)\n",
			       index,
			       DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT +
					    DISP_RDMA_INDEX_OFFSET * index));
			DISPERR("IRQ: RDMA%d underflow! cnt=%d\n", index, cnt_rdma_underflow[index]++);
			DISPERR("(0x030)R_M_GMC_SET0  =0x%x\n",
				DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0 + DISP_RDMA_INDEX_OFFSET * index));
			if (disp_helper_get_option(DISP_OPT_RDMA_UNDERFLOW_AEE))
				DISPAEE("RDMA%d underflow!cnt=%d\n", index, cnt_rdma_underflow[index]++);
			disp_irq_log_module[module] = 1;
			rdma_underflow_irq_cnt[index]++;
		}
			if (reg_val & (1 << 5)) {
				DISPIRQ("IRQ: RDMA%d target line!\n", index);
				rdma_targetline_irq_cnt[index]++;
			}
			if (index == 0 &&
			    (reg_val & ((1 << 1) | (1 << 2) | (1 << 3) |
					(1 << 4) | (1 << 5))))
				disp_irq_m6_dump_primary_path_state("rdma0", reg_val,
					&m6_rdma0_irq_diag_count);
			/* clear intr */
			DISP_CPU_REG_SET(DISP_REG_RDMA_INT_STATUS + index * DISP_RDMA_INDEX_OFFSET, ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->RDMA_IRQ[index], MMProfileFlagPulse, reg_val, 0);
		if (reg_val & 0x18)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (rdma_underflow_irq_cnt[index] << 24) | (index << 16) | reg_val, module);

	} else if (irq == dispsys_irq[DISP_REG_COLOR]) {
		DISPERR("color irq happens!! %d\n", irq);
	} else if (irq == dispsys_irq[DISP_REG_MUTEX]) {
		/* mutex0: perimary disp */
		/* mutex1: sub disp */
		/* mutex2: aal */
		module = DISP_MODULE_MUTEX;
		reg_val = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & 0x7C1F;
			for (mutexID = 0; mutexID < 5; mutexID++) {
				if (reg_val & (0x1 << mutexID)) {
					DISPIRQ("IRQ: mutex%d sof!\n", mutexID);
				MMProfileLogEx(ddp_mmp_get_events()->MUTEX_IRQ[mutexID],
					       MMProfileFlagPulse, reg_val, 0);
			}
			if (reg_val & (0x1 << (mutexID + DISP_MUTEX_TOTAL))) {
				DISPIRQ("IRQ: mutex%d eof!\n", mutexID);
					MMProfileLogEx(ddp_mmp_get_events()->MUTEX_IRQ[mutexID],
						       MMProfileFlagPulse, reg_val, 1);
				}
			}
			if (reg_val & ((0x1 << 0) | (0x1 << DISP_MUTEX_TOTAL)))
				disp_irq_m6_dump_primary_path_state("mutex0", reg_val,
					&m6_mutex_irq_diag_count);
			DISP_CPU_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, ~reg_val);
	} else if (irq == dispsys_irq[DISP_REG_AAL]) {
		module = DISP_MODULE_AAL;
		reg_val = DISP_REG_GET(DISP_AAL_INTSTA);
		disp_aal_on_end_of_frame();
	} else if (irq == dispsys_irq[DISP_REG_CONFIG]) {	/* MMSYS error intr */
		reg_val = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_INTSTA) & 0x7;
		if (reg_val & (1 << 0))
			DISPERR("MMSYS to MFG APB TX Error, MMSYS clock off but MFG clock on!\n");

		if (reg_val & (1 << 1))
			DISPERR("MMSYS to MJC APB TX Error, MMSYS clock off but MJC clock on!\n");

		if (reg_val & (1 << 2))
			DISPERR("PWM APB TX Error!\n");

		DISP_CPU_REG_SET(DISP_REG_CONFIG_MMSYS_INTSTA, ~reg_val);
	} else if (irq == dispsys_irq[DISP_REG_DPI0]) {
		module = DISP_MODULE_DPI;
		reg_val = DISP_REG_GET(DISP_REG_DPI_INSTA) & 0x7;
		DISP_CPU_REG_SET(DISP_REG_DPI_INSTA, 0);
	} else {
		module = DISP_MODULE_UNKNOWN;
		reg_val = 0;
		DISPERR("invalid irq=%d\n ", irq);
	}

	disp_invoke_irq_callbacks(module, reg_val);

	for (j = 0; j < DISP_MODULE_NUM; j++) {
		if (disp_irq_log_module[j] != 0) {
			disp_irq_log_module_en = 1;
			break;
		}
	}
	if (disp_irq_log_module_en != 0)
		wake_up_interruptible(&disp_irq_log_wq);

	MMProfileLogEx(ddp_mmp_get_events()->DDP_IRQ, MMProfileFlagEnd, irq, reg_val);
	return IRQ_HANDLED;
}


static int disp_irq_log_kthread_func(void *data)
{
	unsigned int i = 0;

	while (1) {
		wait_event_interruptible(disp_irq_log_wq, disp_irq_log_module_en);

		for (i = 0; i < DISP_MODULE_NUM; i++) {
			if (disp_irq_log_module[i] != 0) {
				ddp_dump_reg(i);
				disp_irq_log_module[i] = 0;
			}
		}
		disp_irq_log_module_en = 0;
	}
	return 0;
}

void disp_register_dev_irq(unsigned int irq_num, char *device_name)
{
	if (request_irq(irq_num, (irq_handler_t) disp_irq_handler,
			IRQF_TRIGGER_LOW, device_name, NULL))
		DISPERR("ddp register irq %u failed on device %s\n", irq_num, device_name);

}

int disp_init_irq(void)
{
	if (irq_init)
		return 0;

	irq_init = 1;
	DISPMSG("disp_init_irq\n");

	/* create irq log thread */
	init_waitqueue_head(&disp_irq_log_wq);
	disp_irq_log_task = kthread_create(disp_irq_log_kthread_func, NULL, "ddp_irq_log_kthread");
	if (IS_ERR(disp_irq_log_task))
		DISPERR(" can not create disp_irq_log_task kthread\n");

	/* wake_up_process(disp_irq_log_task); */
	return 0;
}
