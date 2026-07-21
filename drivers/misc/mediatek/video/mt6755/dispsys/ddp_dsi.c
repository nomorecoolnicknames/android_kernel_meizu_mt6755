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

#define LOG_TAG "DSI"

#include <linux/delay.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <linux/types.h>
#include "disp_log.h"
#include "disp_drv_platform.h"
#include "mtkfb.h"
#include "ddp_drv.h"
#include "ddp_manager.h"
#include "ddp_dump.h"
#include "ddp_irq.h"
#include "ddp_dsi.h"
#include "ddp_mmp.h"
#include "disp_helper.h"
#include "disp_lowpower.h"
#include "ddp_reg.h"
#include "disp_debug.h"
#include "mtkfb_debug.h"
#ifdef CONFIG_MTK_LEGACY
#include <mt-plat/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif
#include <mt-plat/sync_write.h>
#include <mt-plat/aee.h>
#ifndef CONFIG_MTK_CLKMGR
#include "ddp_clkmgr.h"
#endif
#ifdef CONFIG_LOG_JANK
#include <huawei_platform/log/log_jank.h>
#endif
#ifdef CONFIG_HUAWEI_LCD_DSM
#include <dsm/dsm_pub.h>
#include <asm-generic/gpio.h>
#include "ddp_reg.h"
#endif

static void dsi_m6_wrtrace_record(char op, unsigned long addr, unsigned int val,
				  unsigned int mask, unsigned int old,
				  unsigned int old_valid, void *cmdq);

#define DSI_OUTREG32(cmdq, addr, val) \
	do { \
		unsigned int __m6_val = (unsigned int)(val); \
		dsi_m6_wrtrace_record('W', (unsigned long)(addr), __m6_val, \
				      0xffffffff, 0, 0, (void *)(cmdq)); \
		DISP_REG_SET(cmdq, addr, __m6_val); \
	} while (0)
#define DSI_BACKUPREG32(cmdq, hSlot, idx, addr) DISP_REG_BACKUP(cmdq, hSlot, idx, addr)
#define DSI_POLLREG32(cmdq, addr, mask, value) DISP_REG_CMDQ_POLLING(cmdq, addr, value, mask)
#define DSI_INREG32(type, addr) INREG32(addr)
#define DSI_READREG32(type, dst, src) mt_reg_sync_writel(INREG32(src), dst)

static int dsi_reg_op_debug;
#define DSI_MASKREG32(cmdq, REG, MASK, VALUE) \
	do { \
		dsi_m6_wrtrace_record('R', (unsigned long)(REG), \
				      (unsigned int)(VALUE), (unsigned int)(MASK), \
				      0, 0, (void *)(cmdq)); \
		DISP_REG_MASK((cmdq), (REG), (VALUE), (MASK)); \
	} while (0)

#define DSI_OUTREGBIT(cmdq, TYPE, REG, bit, value)  \
	{\
		do {\
			TYPE r = {0};\
			TYPE v = {0};\
			if (cmdq) {\
				*(unsigned int *)(&r) = ((unsigned int)0x00000000); \
				r.bit = ~(r.bit);  \
				*(unsigned int *)(&v) = ((unsigned int)0x00000000); \
				v.bit = value; \
				dsi_m6_wrtrace_record('M', (unsigned long)(&REG), \
						      AS_UINT32(&v), AS_UINT32(&r), \
						      0, 0, (void *)(cmdq)); \
				DISP_REG_MASK(cmdq, &REG, AS_UINT32(&v), AS_UINT32(&r)); \
			} else { \
				unsigned int __m6_old = INREG32(&REG); \
				mt_reg_sync_writel(__m6_old, &r); \
				r.bit = (value); \
				dsi_m6_wrtrace_record('B', (unsigned long)(&REG), \
						      AS_UINT32(&r), 0xffffffff, \
						      __m6_old, 1, (void *)(cmdq)); \
				DISP_REG_SET(cmdq, &REG, INREG32(&r)); \
			} \
		} while (0);\
	}

#ifdef CONFIG_FPGA_EARLY_PORTING
#define MIPITX_Write60384(slave_addr, write_addr, write_data)			\
{	\
	DISPMSG("MIPITX_Write60384:0x%x,0x%x,0x%x\n", slave_addr, write_addr, write_data);		\
	mt_reg_sync_writel(0x2, MIPITX_BASE+0x14);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x18);		\
	mt_reg_sync_writel(((unsigned int)slave_addr << 0x1), MIPITX_BASE+0x04);		\
	mt_reg_sync_writel(write_addr, MIPITX_BASE+0x0);		\
	mt_reg_sync_writel(write_data, MIPITX_BASE+0x0);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x24);		\
	while ((INREG32(MIPITX_BASE+0xC)&0x1) != 0x1)\
		;		\
	mt_reg_sync_writel(0xFF, MIPITX_BASE+0xC);		\
	\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x14);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x18);		\
	mt_reg_sync_writel(((unsigned int)slave_addr << 0x1), MIPITX_BASE+0x04);		\
	mt_reg_sync_writel(write_addr, MIPITX_BASE+0x0);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x24);		\
	while ((INREG32(MIPITX_BASE+0xC)&0x1) != 0x1)\
		;		\
	mt_reg_sync_writel(0xFF, MIPITX_BASE+0xC);		\
	\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x14);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x18);		\
	mt_reg_sync_writel(((unsigned int)slave_addr << 0x1)+1, MIPITX_BASE+0x04);		\
	mt_reg_sync_writel(0x1, MIPITX_BASE+0x24);		\
	while ((INREG32(MIPITX_BASE+0xC)&0x1) != 0x1)\
		;		\
	mt_reg_sync_writel(0xFF, MIPITX_BASE+0xC);		\
	\
	DISPMSG("MIPI write data = 0x%x, read data = 0x%x\n", write_data, INREG32(MIPITX_BASE));		\
	if (INREG32(MIPITX_BASE) == write_data) \
		DISPMSG("MIPI write success\n");		\
	else \
		DISPMSG("MIPI write fail\n");		\
}

#define MIPITX_INREG32(addr) \
({ \
	unsigned int val = 0;\
	if (0)\
		val = INREG32(addr); \
	if (dsi_reg_op_debug) \
		DISPMSG("[mipitx/inreg]%p=0x%08x\n", (void *)addr, val); \
	val; \
})

 #define MIPITX_OUTREG32(addr, val) {\
		unsigned int __m6_val = (unsigned int)(val); \
		dsi_m6_wrtrace_record('W', (unsigned long)(addr), __m6_val, \
				      0xffffffff, 0, 0, NULL); \
		if (dsi_reg_op_debug) \
			DISPMSG("[mipitx/reg]%p=0x%08x\n", (void *)addr, __m6_val); \
		if (0) \
			mt_reg_sync_writel(__m6_val, addr); \
	}

#define MIPITX_OUTREGBIT(TYPE, REG, bit, value) {\
		do {	\
			TYPE r = {0};\
			if (0) \
				mt_reg_sync_writel(INREG32(&REG), &r); \
			*(unsigned long *)(&r) = ((unsigned long)0x00000000);	  \
			r.bit = value;	  \
			MIPITX_OUTREG32(&REG, AS_UINT32(&r));	  \
			} while (0);\
	}

#define MIPITX_MASKREG32(x, y, z)  MIPITX_OUTREG32(x, (MIPITX_INREG32(x)&~(y))|(z))
#else
#define MIPITX_INREG32(addr) \
({ \
	unsigned int val = 0; val = INREG32(addr);\
	if (dsi_reg_op_debug) \
		DISPMSG("[mipitx/inreg]%p=0x%08x\n", (void *)addr, val); \
	val; })

#define MIPITX_OUTREG32(addr, val) \
	{\
		do {	\
			unsigned int __m6_val = (unsigned int)(val); \
			dsi_m6_wrtrace_record('W', (unsigned long)(addr), __m6_val, \
					      0xffffffff, 0, 0, NULL); \
			if (dsi_reg_op_debug) {	\
				DISPMSG("[mipitx/reg]%p=0x%08x\n", (void *)addr, __m6_val);\
			} \
			mt_reg_sync_writel(__m6_val, addr);\
		} while (0);\
	}

#define MIPITX_OUTREGBIT(TYPE, REG, bit, value)  \
	{\
		do {	\
			TYPE r = {0};\
			mt_reg_sync_writel(INREG32(&REG), &r);	  \
			r.bit = value;	  \
			MIPITX_OUTREG32(&REG, AS_UINT32(&r));	  \
			} while (0);\
	}

#define MIPITX_MASKREG32(x, y, z)  MIPITX_OUTREG32(x, (MIPITX_INREG32(x)&~(y))|(z))
#endif

	typedef struct {
		unsigned int lcm_width;
		unsigned int lcm_height;
		cmdqRecHandle *handle;
		bool enable;
		DSI_REGS regBackup;
		unsigned int cmdq_size;
		LCM_DSI_PARAMS dsi_params;
	}
t_dsi_context;

t_dsi_context _dsi_context[DSI_INTERFACE_NUM];

#define DSI_MODULE_BEGIN(x)		0	/* (x == DISP_MODULE_DSIDUAL?0:DSI_MODULE_to_ID(x)) */
#define DSI_MODULE_END(x)		0	/* (x == DISP_MODULE_DSIDUAL?1:DSI_MODULE_to_ID(x)) */
#define DSI_MODULE_to_ID(x)		0	/* (x == DISP_MODULE_DSI0?0:1) */
#define DIFF_CLK_LANE_LP (0x10)
#define M6_MIPITX_RT_CAL_PHYS 0x10206190
#define M6_LK_PHY_BG_SETTLE_MS 30
#define M6_LK_PHY_PLL_EN_SETTLE_MS 20
#define M6_LK_PHY_PCW_PAD_SETTLE_MS 200
#define M6_LKGOLD_DSI_LAST 0x1b0
#define M6_LKGOLD_DSI_WORDS ((M6_LKGOLD_DSI_LAST / 4) + 1)
#define M6_LKGOLD_MIPITX_LAST 0x104
#define M6_LKGOLD_MIPITX_WORDS ((M6_LKGOLD_MIPITX_LAST / 4) + 1)
/* Diagnostic hold (ms) on the first DSI0 handoff; 0 disables it
 * (dsi_m6_takeover_hold_once returns early). */
#define M6_LK_HANDOFF_TAKEOVER_HOLD_MS 0
/*
 * PROPER-FIX: keep Linux-owned DSI config/start enabled. The old M6
 * isolation skip leaves DSI in CMD mode after any stop/restart sequence.
 */
#define M6_LK_HANDOFF_SKIP_FIRST_DSI_CONFIG 0
/*
 * Must stay 0. With 1, the first takeover (LK's MIPITX live, PMaster=0)
 * calls DSI_PHY_clk_setting() in ddp_dsi_config(), which power-cycles the
 * MIPITX PLL/PHY (MPLL off->on, BG re-enable, PAD_TIE_LOW toggle, ~250 ms
 * settle) on LK's live link and reconfigures it. The transient breaks the
 * panel's HS video lock (the image goes black a few seconds into boot)
 * while the final registers still match LK. Stock ddp_dsi_config keeps the
 * LK-initialized link here (goto done). Resume is unaffected: after
 * suspend MIPITX is off (the else branch reconfigures) or
 * dsi_force_config=1.
 */
#define M6_FORCE_FIRST_DSI_CONFIG_ON_LK_MIPITX 0

/* 0, as in the 3.18 kernel: reprogramming the PLL on the live link makes no
 * difference to VM_DONE. */
#define M6_BOOT_PLL_REPROG 0

PDSI_REGS DSI_REG[2] = {0};
PDSI_PHY_REGS DSI_PHY_REG[2] = {0};
PDSI_CMDQ_REGS DSI_CMDQ_REG[2] = {0};
PDSI_VM_CMDQ_REGS DSI_VM_CMD_REG[2] = {0};

static wait_queue_head_t _dsi_cmd_done_wait_queue[2];
static wait_queue_head_t _dsi_dcs_read_wait_queue[2];
static wait_queue_head_t _dsi_wait_bta_te[2];
static wait_queue_head_t _dsi_wait_ext_te[2];
static wait_queue_head_t _dsi_wait_vm_done_queue[2];
static wait_queue_head_t _dsi_wait_vm_cmd_done_queue[2];
static wait_queue_head_t _dsi_wait_sleep_out_done_queue[2];
static bool waitRDDone;
static bool wait_vm_cmd_done;
static bool wait_sleep_out_done;
static int s_isDsiPowerOn;
static int dsi_currect_mode;
static int dsi_force_config;
static unsigned int m6_lk_handoff_takeover_hold_done;
/* DSI selfpat/BIST bisector at boot, off by default; the m6_dsi_bist_full
 * knob still runs it at runtime (see the done: label in ddp_dsi_config). */
static int m6_selfpat_auto_on;
static int m6_selfpat_auto_done;
static int dsi0_te_enable = 1;
static const LCM_UTIL_FUNCS lcm_utils_dsi0;
unsigned int clock_lane = 0;/*MIPITX_DSI_CLOCK_LANE*/
unsigned int data_lane3 = 0;/*MIPITX_DSI_DATA_LANE3*/
unsigned int data_lane2 = 0;/*MIPITX_DSI_DATA_LANE2*/
unsigned int data_lane1 = 0;/*MIPITX_DSI_DATA_LANE1*/
unsigned int data_lane0 = 0;/*MIPITX_DSI_DATA_LANE0*/

#define M6_DSI_WRTRACE_MAX 2048

struct m6_dsi_wrtrace_entry {
	unsigned int seq;
	unsigned int val;
	unsigned int mask;
	unsigned int old;
	unsigned int old_valid;
	unsigned int off;
	unsigned long addr;
	unsigned long jiffies;
	unsigned long long ns;
	void *cmdq;
	char op;
	char blk;
	char comm[TASK_COMM_LEN];
};

static DEFINE_SPINLOCK(m6_dsi_wrtrace_lock);
static struct m6_dsi_wrtrace_entry m6_dsi_wrtrace[M6_DSI_WRTRACE_MAX];
static unsigned int m6_dsi_wrtrace_count;
static unsigned int m6_dsi_wrtrace_dropped;
static unsigned int m6_dsi_wrtrace_enabled = 1;

static char dsi_m6_wrtrace_classify(unsigned long addr, unsigned int *off)
{
	unsigned long dsi_base = (unsigned long)DDP_REG_BASE_DSI0;
	unsigned long mipitx_base = (unsigned long)MIPITX_BASE;

	if (addr >= dsi_base && addr <= dsi_base + M6_LKGOLD_DSI_LAST) {
		*off = (unsigned int)(addr - dsi_base);
		return 'D';
	}
	if (addr >= mipitx_base && addr <= mipitx_base + M6_LKGOLD_MIPITX_LAST) {
		*off = (unsigned int)(addr - mipitx_base);
		return 'M';
	}
	return 0;
}

static void dsi_m6_wrtrace_record(char op, unsigned long addr, unsigned int val,
				  unsigned int mask, unsigned int old,
				  unsigned int old_valid, void *cmdq)
{
	struct m6_dsi_wrtrace_entry *e;
	unsigned long flags;
	unsigned int off = 0;
	char blk;

	if (!m6_dsi_wrtrace_enabled)
		return;

	blk = dsi_m6_wrtrace_classify(addr, &off);
	if (!blk)
		return;

	spin_lock_irqsave(&m6_dsi_wrtrace_lock, flags);
	if (m6_dsi_wrtrace_count >= M6_DSI_WRTRACE_MAX) {
		m6_dsi_wrtrace_dropped++;
		m6_dsi_wrtrace_enabled = 0;
		spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);
		return;
	}

	e = &m6_dsi_wrtrace[m6_dsi_wrtrace_count];
	e->seq = m6_dsi_wrtrace_count++;
	e->val = val;
	e->mask = mask;
	e->old = old;
	e->old_valid = old_valid ? 1 : 0;
	e->off = off;
	e->addr = addr;
	e->jiffies = jiffies;
	e->ns = local_clock();
	e->cmdq = cmdq;
	e->op = op;
	e->blk = blk;
	strlcpy(e->comm, current->comm, sizeof(e->comm));
	spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);
}

void dsi_m6_wrtrace_reset(unsigned int enable)
{
	unsigned long flags;

	spin_lock_irqsave(&m6_dsi_wrtrace_lock, flags);
	m6_dsi_wrtrace_count = 0;
	m6_dsi_wrtrace_dropped = 0;
	m6_dsi_wrtrace_enabled = enable ? 1 : 0;
	spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);

	DISPERR("M6 DSI wrtrace reset: enable=%u max=%u\n",
		m6_dsi_wrtrace_enabled, M6_DSI_WRTRACE_MAX);
}

void dsi_m6_wrtrace_enable(unsigned int enable)
{
	unsigned long flags;

	spin_lock_irqsave(&m6_dsi_wrtrace_lock, flags);
	m6_dsi_wrtrace_enabled = enable ? 1 : 0;
	spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);

	DISPERR("M6 DSI wrtrace enable=%u count=%u dropped=%u max=%u\n",
		m6_dsi_wrtrace_enabled, m6_dsi_wrtrace_count,
		m6_dsi_wrtrace_dropped, M6_DSI_WRTRACE_MAX);
}

void dsi_m6_wrtrace_dump(unsigned int limit)
{
	struct m6_dsi_wrtrace_entry e;
	unsigned long flags;
	unsigned int count;
	unsigned int dropped;
	unsigned int enabled;
	unsigned int i;

	spin_lock_irqsave(&m6_dsi_wrtrace_lock, flags);
	count = m6_dsi_wrtrace_count;
	dropped = m6_dsi_wrtrace_dropped;
	enabled = m6_dsi_wrtrace_enabled;
	spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);

	if (limit == 0 || limit > count)
		limit = count;

	DISPERR("M6 DSI wrtrace dump: enabled=%u count=%u dropped=%u limit=%u max=%u\n",
		enabled, count, dropped, limit, M6_DSI_WRTRACE_MAX);

	for (i = 0; i < limit; i++) {
		spin_lock_irqsave(&m6_dsi_wrtrace_lock, flags);
		e = m6_dsi_wrtrace[i];
		spin_unlock_irqrestore(&m6_dsi_wrtrace_lock, flags);

		DISPERR("M6 DSI wrtrace[%04u]: blk=%c off=0x%03x op=%c cmdq=%p old_valid=%u old=0x%08x val=0x%08x mask=0x%08x j=%lu ns=%llu comm=%s addr=0x%lx\n",
			e.seq, e.blk, e.off, e.op, e.cmdq, e.old_valid,
			e.old, e.val, e.mask, e.jiffies, e.ns, e.comm, e.addr);
	}
}

static void dsi_m6_dump_irq_decode(const char *tag, uint32_t start, uint32_t status,
				   uint32_t inten, uint32_t intsta)
{
	DISPERR("M6 DSI irq_decode[%s]: START dsi/sleep/skew/vmcmd=%u/%u/%u/%u STA underrun/esc_entry/esc_sync/ctrl/content=%u/%u/%u/%u/%u INTEN rd/cmd/te/vm/frame/vmcmd/sleep/te_to/vbp/vact/vfp/skew=%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u INTSTA rd/cmd/te/vm/frame/vmcmd/sleep/te_to/vbp/vact/vfp/skew/busy=%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u raw=0x%x/0x%x/0x%x/0x%x\n",
		tag,
		(start & BIT(0)) ? 1 : 0, (start & BIT(2)) ? 1 : 0,
		(start & BIT(4)) ? 1 : 0, (start & BIT(16)) ? 1 : 0,
		(status & BIT(1)) ? 1 : 0, (status & BIT(4)) ? 1 : 0,
		(status & BIT(5)) ? 1 : 0, (status & BIT(6)) ? 1 : 0,
		(status & BIT(7)) ? 1 : 0,
		(inten & BIT(0)) ? 1 : 0, (inten & BIT(1)) ? 1 : 0,
		(inten & BIT(2)) ? 1 : 0, (inten & BIT(3)) ? 1 : 0,
		(inten & BIT(4)) ? 1 : 0, (inten & BIT(5)) ? 1 : 0,
		(inten & BIT(6)) ? 1 : 0, (inten & BIT(7)) ? 1 : 0,
		(inten & BIT(8)) ? 1 : 0, (inten & BIT(9)) ? 1 : 0,
		(inten & BIT(10)) ? 1 : 0, (inten & BIT(11)) ? 1 : 0,
		(intsta & BIT(0)) ? 1 : 0, (intsta & BIT(1)) ? 1 : 0,
		(intsta & BIT(2)) ? 1 : 0, (intsta & BIT(3)) ? 1 : 0,
		(intsta & BIT(4)) ? 1 : 0, (intsta & BIT(5)) ? 1 : 0,
		(intsta & BIT(6)) ? 1 : 0, (intsta & BIT(7)) ? 1 : 0,
		(intsta & BIT(8)) ? 1 : 0, (intsta & BIT(9)) ? 1 : 0,
		(intsta & BIT(10)) ? 1 : 0, (intsta & BIT(11)) ? 1 : 0,
		(intsta & BIT(31)) ? 1 : 0, start, status, inten, intsta);
}

static void dsi_m6_clkstate_marker(const char *tag, DISP_MODULE_ENUM module)
{
	static unsigned int count;
	unsigned int start = 0;
	unsigned int intsta = 0;
	unsigned int mode = 0;
	unsigned int txrx = 0;
	unsigned int ps = 0;
	unsigned int dsi_e = 0xffffffff;
	unsigned int dsi_p = 0xffffffff;
	unsigned int dig_e = 0xffffffff;
	unsigned int dig_p = 0xffffffff;
	unsigned int mtcmos_e = 0xffffffff;
	unsigned int mtcmos_p = 0xffffffff;

	if (count >= 192)
		return;

	count++;

	if ((module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) && DSI_REG[0]) {
		start = AS_UINT32(&DSI_REG[0]->DSI_START);
		intsta = AS_UINT32(&DSI_REG[0]->DSI_INTSTA);
		mode = AS_UINT32(&DSI_REG[0]->DSI_MODE_CTRL);
		txrx = AS_UINT32(&DSI_REG[0]->DSI_TXRX_CTRL);
		ps = AS_UINT32(&DSI_REG[0]->DSI_PSCTRL);
	}

#ifndef CONFIG_MTK_CLKMGR
	dsi_e = ddp_clk_get_enable_count(DISP1_DSI_ENGINE);
	dsi_p = ddp_clk_get_prepare_count(DISP1_DSI_ENGINE);
	dig_e = ddp_clk_get_enable_count(DISP1_DSI_DIGITAL);
	dig_p = ddp_clk_get_prepare_count(DISP1_DSI_DIGITAL);
	mtcmos_e = ddp_clk_get_enable_count(DISP_MTCMOS_CLK);
	mtcmos_p = ddp_clk_get_prepare_count(DISP_MTCMOS_CLK);
#endif
	DISPERR("M6 DSI clkstate[%s] #%u module=%d power=%d ulps=%u dsi_e/p=%u/%u dig_e/p=%u/%u mtcmos_e/p=%u/%u start=0x%x intsta=0x%x mode=0x%x txrx=0x%x ps=0x%x cg=0x%x/0x%x swrst=0x%x/%x lcm_rst=0x%x route=0x%x/%x mutex0=0x%x/%x/%x\n",
		tag, count, module, s_isDsiPowerOn, is_mipi_enterulps(),
		dsi_e, dsi_p, dig_e, dig_p, mtcmos_e, mtcmos_p,
		start, intsta, mode, txrx, ps,
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW0_RST_B),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW1_RST_B),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_LCM_RST_B),
		DISP_REG_GET(DISP_REG_CONFIG_DSI0_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_RDMA0_SOUT_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF));
}

atomic_t PMaster_enable = ATOMIC_INIT(0);

static const char *_dsi_cmd_mode_parse_state(unsigned int state)
{
	switch (state) {
	case 0x0001:
		return "idle";
	case 0x0002:
		return "Reading command queue for header";
	case 0x0004:
		return "Sending type-0 command";
	case 0x0008:
		return "Waiting frame data from RDMA for type-1 command";
	case 0x0010:
		return "Sending type-1 command";
	case 0x0020:
		return "Sending type-2 command";
	case 0x0040:
		return "Reading command queue for data";
	case 0x0080:
		return "Sending type-3 command";
	case 0x0100:
		return "Sending BTA";
	case 0x0200:
		return "Waiting RX-read data ";
	case 0x0400:
		return "Waiting SW RACK for RX-read data";
	case 0x0800:
		return "Waiting TE";
	case 0x1000:
		return "Get TE ";
	case 0x2000:
		return "Waiting SW RACK for TE";
	case 0x4000:
		return "Waiting external TE";
	default:
		return "unknown";
	}
}

static const char *_dsi_vdo_mode_parse_state(unsigned int state)
{
	switch (state) {
	case 0x0001:
		return "Video mode idle";
	case 0x0002:
		return "Sync start packet";
	case 0x0004:
		return "Hsync active";
	case 0x0008:
		return "Sync end packet";
	case 0x0010:
		return "Hsync back porch";
	case 0x0020:
		return "Video data period";
	case 0x0040:
		return "Hsync front porch";
	case 0x0080:
		return "BLLP";
	case 0x0100:
		return "--";
	case 0x0200:
		return "Mix mode using command mode transmission";
	case 0x0400:
		return "Command transmission in BLLP";
	default:
		return "unknown";
	}

}

DSI_STATUS DSI_DumpRegisters(DISP_MODULE_ENUM module, int level)
{
	uint32_t i;
	unsigned int DSI_DBG6_Status;
	unsigned int DSI_DBG7_Status;
	unsigned int DSI_DBG8_Status;
	unsigned int DSI_DBG9_Status;

	if (level >= 0) {
		if (module == DISP_MODULE_DSI0) {
			if (DSI_REG[0]->DSI_MODE_CTRL.MODE == CMD_MODE) {
				DSI_DBG6_Status =
				    (INREG32(DDP_REG_BASE_DSI0 + 0x160)) & 0xffff;
				DISPDMP("DSI0 state6(cmd mode):%s\n",
					_dsi_cmd_mode_parse_state(DSI_DBG6_Status));

			} else {
				DSI_DBG7_Status =
				    (INREG32(DDP_REG_BASE_DSI0 + 0x164)) & 0xff;
				DISPDMP("DSI0 state7(vdo mode):%s\n",
					_dsi_vdo_mode_parse_state(DSI_DBG7_Status));

			}
			DSI_DBG8_Status =
			    (INREG32(DDP_REG_BASE_DSI0 + 0x168)) & 0x3fff;
			DISPDMP("DSI0 state8 WORD_COUNTER(cmd mode):%s\n",
				_dsi_cmd_mode_parse_state(DSI_DBG8_Status));

			DSI_DBG9_Status =
			    (INREG32(DDP_REG_BASE_DSI0 + 0x16C)) & 0x3fffff;
			DISPDMP("DSI0 state9 LINE_COUNTER(cmd mode):%s\n",
				_dsi_cmd_mode_parse_state(DSI_DBG9_Status));

		}
	}
	if (level >= 1) {
		if (module == DISP_MODULE_DSI0) {
			DSI_DBG6_Status =
			    (INREG32(DDP_REG_BASE_DSI0 + 0x160)) & 0xffff;

			DISPDMP("== START: DISP DSI0 registers ==\n");

			for (i = 0; i < sizeof(DSI_REGS); i += 16) {
				DISPDMP("DSI0: 0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x\n",
					i, INREG32(DDP_REG_BASE_DSI0 + i),
					i + 0x04, INREG32(DDP_REG_BASE_DSI0 + i + 0x4),
					i + 0x08, INREG32(DDP_REG_BASE_DSI0 + i + 0x8),
					i + 0x0c, INREG32(DDP_REG_BASE_DSI0 + i + 0xc));
			}

			for (i = 0; i < sizeof(DSI_CMDQ_REGS); i += 16) {
				DISPDMP("DSI_CMD+%04x : 0x%08x  0x%08x  0x%08x  0x%08x\n", i,
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0x4)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0x8)),
					INREG32((DDP_REG_BASE_DSI0 + 0x200 + i + 0xc)));
			}

#ifndef CONFIG_FPGA_EARLY_PORTING
			for (i = 0; i < sizeof(DSI_PHY_REGS); i += 16) {
				DISPDMP("DSI_PHY: 0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x\n",
					i, INREG32((MIPITX_BASE + i)),
					i + 0x4, INREG32((MIPITX_BASE + i + 0x4)),
					i + 0x8, INREG32((MIPITX_BASE + i + 0x8)),
					i + 0xc, INREG32((MIPITX_BASE + i + 0xc)));
			}
			DISPDMP("-- END: DISP DSI0 registers --\n");
#endif
		}
	}

	return DSI_STATUS_OK;
}

static void _DSI_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	int i = 0;
	DSI_INT_STATUS_REG status = {0};
	DSI_TXRX_CTRL_REG txrx_ctrl = {0};
	static unsigned int m6_irq_dump_count;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (module == DISP_MODULE_DSI0 && DSI_REG[i] && m6_irq_dump_count < 96) {
			uint32_t start = INREG32(&DSI_REG[i]->DSI_START);
			uint32_t sta = INREG32(&DSI_REG[i]->DSI_STA);
			uint32_t inten = INREG32(&DSI_REG[i]->DSI_INTEN);
			uint32_t live_intsta = INREG32(&DSI_REG[i]->DSI_INTSTA);
			uint32_t mode = INREG32(&DSI_REG[i]->DSI_MODE_CTRL);
			uint32_t state7 = INREG32(DDP_REG_BASE_DSI0 + 0x164);
			uint32_t state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
			uint32_t state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);

			m6_irq_dump_count++;
			DISPERR("M6 DSI irq[internal] #%u module=%d param=0x%x live_intsta=0x%x inten=0x%x start=0x%x mode=0x%x state7=0x%x/%s state8=0x%x state9=0x%x waits rd/vmcmd/sleep=%u/%u/%u\n",
				m6_irq_dump_count, module, param, live_intsta, inten,
				start, mode, state7,
				_dsi_vdo_mode_parse_state(state7 & 0xff),
				state8, state9, waitRDDone, wait_vm_cmd_done,
				wait_sleep_out_done);
			dsi_m6_dump_irq_decode("internal-param", start, sta, inten, param);
			dsi_m6_dump_irq_decode("internal-live", start, sta, inten,
					       live_intsta);
		}
		status = *(PDSI_INT_STATUS_REG) & param;
		if (status.RD_RDY) {
			/* /write clear RD_RDY interrupt */

			/* / write clear RD_RDY interrupt must be before DSI_RACK */
			/* / because CMD_DONE will raise after DSI_RACK, */
			/* / so write clear RD_RDY after that will clear CMD_DONE too */
			/*do
			   {
			   ///send read ACK
			   //DSI_REG->DSI_RACK.DSI_RACK = 1;
			   DSI_OUTREGBIT(NULL, DSI_RACK_REG,DSI_REG[i]->DSI_RACK,DSI_RACK,1);
			   DISPMSG("send read ACK\n");
			   } while(DSI_REG[i]->DSI_INTSTA.BUSY); */
			waitRDDone = true;
			wake_up_interruptible(&_dsi_dcs_read_wait_queue[i]);
		}

		if (status.CMD_DONE) {
			/* DISPMSG("[callback]%s cmd dome\n", ddp_get_module_name(module)); */
			wake_up_interruptible(&_dsi_cmd_done_wait_queue[i]);
		}

		if (status.TE_RDY) {
			DSI_OUTREG32(NULL, &txrx_ctrl, INREG32(&DSI_REG[i]->DSI_TXRX_CTRL));
			if (txrx_ctrl.EXT_TE_EN == 1) {
				/* DISPMSG("[callback]%s  EXT  te\n", ddp_get_module_name(module)); */
				wake_up_interruptible(&_dsi_wait_ext_te[i]);
			} else {
				wake_up_interruptible(&_dsi_wait_bta_te[i]);
			}
		}

		if (status.VM_DONE)
			wake_up_interruptible(&_dsi_wait_vm_done_queue[i]);
		if (status.VM_CMD_DONE) {
			wait_vm_cmd_done = true;
			wake_up_interruptible(&_dsi_wait_vm_cmd_done_queue[i]);
		}
		if (status.SLEEPOUT_DONE) {
			wait_sleep_out_done = true;
			wake_up_interruptible(&_dsi_wait_sleep_out_done_queue[i]);
		}
	}
}

static DSI_STATUS DSI_Reset(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	unsigned int irq_en[2];
	/* DSI_RESET Protect: backup & disable dsi interrupt */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		irq_en[i] = AS_UINT32(&DSI_REG[i]->DSI_INTEN);
		DSI_OUTREG32(NULL, &DSI_REG[i]->DSI_INTEN, 0);
		DISPDBG("DSI_RESET backup dsi%d irq:0x%08x ", i, irq_en[i]);
	}

	/* do reset */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 1);
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 0);
	}

	/* DSI_RESET Protect: restore dsi interrupt */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(NULL, &DSI_REG[i]->DSI_INTEN, irq_en[i]);
		DISPDBG("DSI_RESET restore dsi%d irq:0x%08x ", i,
			AS_UINT32(&DSI_REG[i]->DSI_INTEN));
	}
	return DSI_STATUS_OK;
}

static int _dsi_is_video_mode(DISP_MODULE_ENUM module)
{
	int i = DSI_MODULE_BEGIN(module);

	if (DSI_REG[i]->DSI_MODE_CTRL.MODE == CMD_MODE)
		return 0;
	else
		return 1;
}

static DSI_STATUS DSI_SetMode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, MODE, mode);

	return DSI_STATUS_OK;
}
static DSI_STATUS DSI_SetSwitchMode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (mode == 0) {	/* V2C */
			DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL,
				      V2C_SWITCH_ON, 1);
		} else		/* C2V */
			DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL,
				      C2V_SWITCH_ON, 1);

	}

	return DSI_STATUS_OK;
}
void DSI_lane0_ULP_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	ASSERT(cmdq == NULL);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_RM_TRIG_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      Lx_ULPM_AS_L0, 1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_ULPM_EN, 0);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_ULPM_EN, 1);
			mdelay(1);
		} else {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_ULPM_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      Lx_ULPM_AS_L0, 0);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_WAKEUP_EN, 1);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LD0CON_REG, DSI_REG[i]->DSI_PHY_LD0CON,
				      L0_WAKEUP_EN, 0);
			mdelay(1);
		}
	}
}


void DSI_clk_ULP_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	ASSERT(cmdq == NULL);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_ULPM_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_ULPM_EN, 1);
			mdelay(1);
		} else {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_ULPM_EN, 0);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_WAKEUP_EN, 1);
			mdelay(1);
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_WAKEUP_EN, 0);
			mdelay(1);
		}
	}
}

bool DSI_clk_HS_state(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = DSI_MODULE_BEGIN(module);
	DSI_PHY_LCCON_REG tmpreg = {0};

	DSI_READREG32(PDSI_PHY_LCCON_REG, &tmpreg, &DSI_REG[i]->DSI_PHY_LCCON);
	return tmpreg.LC_HS_TX_EN ? true : false;
}

void DSI_clk_HS_mode(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enter)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_HS_TX_EN, 1);
		} else if (!enter) {
			DSI_OUTREGBIT(cmdq, DSI_PHY_LCCON_REG, DSI_REG[i]->DSI_PHY_LCCON,
				      LC_HS_TX_EN, 0);
		}
	}
}

int DSI_WaitVMDone(DISP_MODULE_ENUM module)
{
	int i = 0;
	static const long WAIT_TIMEOUT = 2 * HZ;	/* 2 sec */
	int ret = 0;

	/*...dsi video is always in busy state... */
	if (_dsi_is_video_mode(module)) {
		DISPMSG("DSI_WaitVMDone error: should set DSI to CMD mode firstly\n");
		return -1;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		ret =
		    wait_event_interruptible_timeout(_dsi_wait_vm_done_queue[i],
						     !(DSI_REG[i]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (0 == ret) {
			DISPERR("dsi wait VM done  timeout\n");
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
			return -1;
		}
	}
	return 0;
}

static void DSI_WaitForNotBusy(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	unsigned int count = 0;
	unsigned int tmp = 0;

	if (cmdq) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
			DSI_POLLREG32(cmdq, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);
		return;
	}


	/*...dsi video is always in busy state... */
	if (_dsi_is_video_mode(module))
		return;
#if defined(MTK_NO_DISP_IN_LK)
	i = DSI_MODULE_BEGIN(module);
	while (1) {
		tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
		if (!(tmp & 0x80000000))
			break;

		/* if(count %1000) */
		/* DISPMSG("dsi state:0x%08x, 0x%08x\n", tmp, INREG32(&DSI_REG[i]->DSI_STATE_DBG6)); */

		/* msleep(1); */

		if (count++ > 1000000000) {
			DISPERR("dsi wait not busy timeout\n");
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
			break;
		}
	}
#else
	/*
	 * The DSI0 GIC line (SPI 194) never fires on this board (the ISR
	 * count stays 0), so the CMD_DONE wakeup never comes and this wait
	 * would always time out, after which DSI_Reset mid-command wedges the
	 * link. DSI_INTSTA latches status regardless of GIC delivery, so poll
	 * BUSY (bit31) directly; no DSI_Reset on timeout.
	 */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		count = 0;
		while (1) {
			tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
			if (!(tmp & 0x80000000))
				break;
			if (count++ > 200000) {
				DISPERR("[M681] dsi poll not-busy timeout, INTSTA=0x%08x\n", tmp);
				break;
			}
			udelay(1);
		}
	}
#endif
}

DSI_STATUS DSI_SleepOut(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	/* wake_up_prd *1024*cycle time > 1ms */
	int wake_up_prd = (_dsi_context[i].dsi_params.PLL_CLOCK * 2 * 1000) / (1024 * 8) + 0x1;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 1);
		DSI_OUTREGBIT(cmdq, DSI_TIME_CON0_REG, DSI_REG[i]->DSI_TIME_CON0, UPLS_WAKEUP_PRD, wake_up_prd);
	}
	DISPDBG("DSI_SleepOut\n");
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_Wakeup(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	int ret = 0;
	int cnt = 0;

	DISPDBG("DSI_Wakeup+\n");
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		wait_sleep_out_done = false;
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 1);
		do {
			cnt++;
			ret =
			    wait_event_interruptible_timeout(_dsi_wait_sleep_out_done_queue[i],
							     wait_sleep_out_done, 2 * HZ);
		} while (ret <= 0 && cnt <= 2);

		if (ret == 0) {
			DISPERR("dsi wait sleep out timeout\n");
			DSI_DumpRegisters(module, 2);
			DSI_Reset(module, NULL);
		} else if (ret < 0) {
			DISPERR("dsi wait sleep out weake up by signal ret %d\n", ret);
			mdelay(5);
		}
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_MODE_CTRL_REG, DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 0);
	}
	DISPDBG("DSI_Wakeup-\n");
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_BackupRegisters(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DSI_REGS *regs = NULL;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		regs = &(_dsi_context[i].regBackup);

		DSI_OUTREG32(cmdq, &regs->DSI_INTEN, AS_UINT32(&DSI_REG[i]->DSI_INTEN));
		DSI_OUTREG32(cmdq, &regs->DSI_MODE_CTRL, AS_UINT32(&DSI_REG[i]->DSI_MODE_CTRL));
		DSI_OUTREG32(cmdq, &regs->DSI_TXRX_CTRL, AS_UINT32(&DSI_REG[i]->DSI_TXRX_CTRL));
		DSI_OUTREG32(cmdq, &regs->DSI_PSCTRL, AS_UINT32(&DSI_REG[i]->DSI_PSCTRL));

		DSI_OUTREG32(cmdq, &regs->DSI_VSA_NL, AS_UINT32(&DSI_REG[i]->DSI_VSA_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VBP_NL, AS_UINT32(&DSI_REG[i]->DSI_VBP_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VFP_NL, AS_UINT32(&DSI_REG[i]->DSI_VFP_NL));
		DSI_OUTREG32(cmdq, &regs->DSI_VACT_NL, AS_UINT32(&DSI_REG[i]->DSI_VACT_NL));

		DSI_OUTREG32(cmdq, &regs->DSI_HSA_WC, AS_UINT32(&DSI_REG[i]->DSI_HSA_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_HBP_WC, AS_UINT32(&DSI_REG[i]->DSI_HBP_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_HFP_WC, AS_UINT32(&DSI_REG[i]->DSI_HFP_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_BLLP_WC, AS_UINT32(&DSI_REG[i]->DSI_BLLP_WC));

		DSI_OUTREG32(cmdq, &regs->DSI_HSTX_CKL_WC, AS_UINT32(&DSI_REG[i]->DSI_HSTX_CKL_WC));
		DSI_OUTREG32(cmdq, &regs->DSI_MEM_CONTI, AS_UINT32(&DSI_REG[i]->DSI_MEM_CONTI));

		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON0,
			     AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON0));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON1,
			     AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON1));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON2,
			     AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON2));
		DSI_OUTREG32(cmdq, &regs->DSI_PHY_TIMECON3,
			     AS_UINT32(&DSI_REG[i]->DSI_PHY_TIMECON3));
		DSI_OUTREG32(cmdq, &regs->DSI_VM_CMD_CON, AS_UINT32(&DSI_REG[i]->DSI_VM_CMD_CON));
	}

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_RestoreRegisters(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	DSI_REGS *regs = NULL;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		regs = &(_dsi_context[i].regBackup);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_INTEN, AS_UINT32(&regs->DSI_INTEN));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MODE_CTRL, AS_UINT32(&regs->DSI_MODE_CTRL));
		/* can not restore lane_num here */
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_TXRX_CTRL,
			     AS_UINT32(&regs->DSI_TXRX_CTRL) & 0xFFFFFFC3);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PSCTRL, AS_UINT32(&regs->DSI_PSCTRL));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VSA_NL, AS_UINT32(&regs->DSI_VSA_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VBP_NL, AS_UINT32(&regs->DSI_VBP_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VFP_NL, AS_UINT32(&regs->DSI_VFP_NL));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VACT_NL, AS_UINT32(&regs->DSI_VACT_NL));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSA_WC, AS_UINT32(&regs->DSI_HSA_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC, AS_UINT32(&regs->DSI_HBP_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, AS_UINT32(&regs->DSI_HFP_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BLLP_WC, AS_UINT32(&regs->DSI_BLLP_WC));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, AS_UINT32(&regs->DSI_HSTX_CKL_WC));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI, AS_UINT32(&regs->DSI_MEM_CONTI));

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0,
			     AS_UINT32(&regs->DSI_PHY_TIMECON0));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1,
			     AS_UINT32(&regs->DSI_PHY_TIMECON1));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2,
			     AS_UINT32(&regs->DSI_PHY_TIMECON2));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3,
			     AS_UINT32(&regs->DSI_PHY_TIMECON3));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VM_CMD_CON, AS_UINT32(&regs->DSI_VM_CMD_CON));
		DISPDBG("DSI_RestoreRegisters VM_CMD_EN %d TS_VFP_EN %d\n",
		       regs->DSI_VM_CMD_CON.VM_CMD_EN, regs->DSI_VM_CMD_CON.TS_VFP_EN);
	}
	return DSI_STATUS_OK;
}

static void dsi_m6_dump_snapshot(const char *tag, DISP_MODULE_ENUM module, void *cmdq);
static void dsi_m6_dump_snapshot_limited(const char *tag, DISP_MODULE_ENUM module,
					 void *cmdq, unsigned int *count,
					 unsigned int limit);
static void dsi_m6_sram_snapshot(const char *tag, DISP_MODULE_ENUM module);

DSI_STATUS DSI_BIST_Pattern_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enable,
				 unsigned int color)
{
	int i = 0;
	static unsigned int m6_bist_dump_count;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			dsi_m6_dump_snapshot_limited("bist-pre-enable", module, cmdq,
						     &m6_bist_dump_count, 16);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_PATTERN, color);
			/* DSI_OUTREG32(&DSI_REG->DSI_BIST_CON, AS_UINT32(&temp_reg)); */
			/* DSI_OUTREGBIT(DSI_BIST_CON_REG, DSI_REG->DSI_BIST_CON, SELF_PAT_MODE, 1); */
			DSI_OUTREGBIT(cmdq, DSI_BIST_CON_REG, DSI_REG[i]->DSI_BIST_CON,
				      SELF_PAT_MODE, 1);
			dsi_m6_dump_snapshot_limited("bist-post-enable", module, cmdq,
						     &m6_bist_dump_count, 16);

			if (!_dsi_is_video_mode(module)) {
				DSI_T0_INS t0;

				t0.CONFG = 0x09;
				t0.Data_ID = 0x39;
				t0.Data0 = 0x2c;
				t0.Data1 = 0;

				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[0], AS_UINT32(&t0));
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE, 1);

				/* DSI_OUTREGBIT(DSI_START_REG,DSI_REG->DSI_START,DSI_START,0); */
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 0);
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 1);
				/* DSI_OUTREGBIT(DSI_START_REG,DSI_REG->DSI_START,DSI_START,1); */
			}
		} else {
			/* if disable dsi pattern, need enable mutex, can't just start dsi */
			/* so we just disable pattern bit, do not start dsi here */
			/* DSI_WaitForNotBusy(module,cmdq); */
			/* DSI_OUTREGBIT(cmdq, DSI_BIST_CON_REG, DSI_REG[i]->DSI_BIST_CON, SELF_PAT_MODE, 0); */
			dsi_m6_dump_snapshot_limited("bist-pre-disable", module, cmdq,
						     &m6_bist_dump_count, 16);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, 0x00);
			dsi_m6_dump_snapshot_limited("bist-post-disable", module, cmdq,
						     &m6_bist_dump_count, 16);
		}

	}
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_M6_BIST_Full_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enable,
				 unsigned int color)
{
	int i = 0;
	static unsigned int m6_bist_full_dump_count;
	DSI_BIST_CON_REG bist_con = {0};

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (enable) {
			dsi_m6_dump_snapshot_limited("bist-full-pre", module, cmdq,
						     &m6_bist_full_dump_count, 32);
			bist_con.BIST_ENABLE = 1;
			bist_con.BIST_FIX_PATTERN = 1;
			bist_con.SELF_PAT_MODE = 1;
			bist_con.BIST_LANE_NUM = 4;
			bist_con.BIST_TIMING = 0x20;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_PATTERN, color);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, AS_UINT32(&bist_con));
			dsi_m6_dump_snapshot_limited("bist-full-post", module, cmdq,
						     &m6_bist_full_dump_count, 32);
			msleep(500);
			dsi_m6_dump_snapshot_limited("bist-full-after-500ms", module, cmdq,
						     &m6_bist_full_dump_count, 32);
		} else {
			dsi_m6_dump_snapshot_limited("bist-full-pre-disable", module, cmdq,
						     &m6_bist_full_dump_count, 32);
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, 0x00);
			dsi_m6_dump_snapshot_limited("bist-full-post-disable", module, cmdq,
						     &m6_bist_full_dump_count, 32);
		}
	}

	return DSI_STATUS_OK;
}

static const char *dsi_m6_bist_profile_name(unsigned int profile)
{
	switch (profile) {
	case 0:
		return "full-current";
	case 1:
		return "full-mode";
	case 2:
		return "full-hs-free";
	case 3:
		return "full-mode-hs-free";
	case 4:
		return "selfpat-legacy";
	case 5:
		return "engine-mode-hs-free";
	default:
		return "full-current";
	}
}

static uint32_t dsi_m6_bist_profile_raw(unsigned int profile)
{
	DSI_BIST_CON_REG bist_con = {0};

	bist_con.BIST_TIMING = 0x20;
	bist_con.BIST_LANE_NUM = 4;

	switch (profile) {
	case 1:
		bist_con.BIST_MODE = 1;
		/* fall through */
	case 0:
		bist_con.BIST_ENABLE = 1;
		bist_con.BIST_FIX_PATTERN = 1;
		bist_con.SELF_PAT_MODE = 1;
		break;
	case 2:
		bist_con.BIST_HS_FREE = 1;
		bist_con.BIST_ENABLE = 1;
		bist_con.BIST_FIX_PATTERN = 1;
		bist_con.SELF_PAT_MODE = 1;
		break;
	case 3:
		bist_con.BIST_MODE = 1;
		bist_con.BIST_HS_FREE = 1;
		bist_con.BIST_ENABLE = 1;
		bist_con.BIST_FIX_PATTERN = 1;
		bist_con.SELF_PAT_MODE = 1;
		break;
	case 4:
		bist_con.BIST_LANE_NUM = 0;
		bist_con.SELF_PAT_MODE = 1;
		break;
	case 5:
		bist_con.BIST_MODE = 1;
		bist_con.BIST_HS_FREE = 1;
		bist_con.BIST_ENABLE = 1;
		bist_con.BIST_FIX_PATTERN = 1;
		break;
	default:
		bist_con.BIST_ENABLE = 1;
		bist_con.BIST_FIX_PATTERN = 1;
		bist_con.SELF_PAT_MODE = 1;
		break;
	}

	return AS_UINT32(&bist_con);
}

DSI_STATUS DSI_M6_BIST_Profile_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq,
				    unsigned int profile, unsigned int color,
				    unsigned int hold_ms)
{
	int i = 0;
	uint32_t bist_con;
	unsigned int bounded_hold = hold_ms;
	unsigned int elapsed = 0;

	if (profile > 5)
		profile = 0;
	if (bounded_hold == 0)
		bounded_hold = 3000;
	if (bounded_hold > 10000)
		bounded_hold = 10000;

	bist_con = dsi_m6_bist_profile_raw(profile);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DISPERR("M6 DSI bist_profile: begin profile=%u/%s color=0x%08x hold_ms=%u raw_con=0x%x\n",
			profile, dsi_m6_bist_profile_name(profile), color,
			bounded_hold, bist_con);
		dsi_m6_dump_snapshot("bist-profile-pre", module, cmdq);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_PATTERN, color);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, bist_con);
		dsi_m6_dump_snapshot("bist-profile-post", module, cmdq);

		if (bounded_hold > 50) {
			msleep(50);
			elapsed = 50;
			dsi_m6_dump_snapshot("bist-profile-after-50ms", module, cmdq);
		}
		if (bounded_hold > 250) {
			msleep(200);
			elapsed = 250;
			dsi_m6_dump_snapshot("bist-profile-after-250ms", module, cmdq);
		}
		if (bounded_hold > 1000) {
			msleep(750);
			elapsed = 1000;
			dsi_m6_dump_snapshot("bist-profile-after-1000ms", module, cmdq);
		}
		if (bounded_hold > elapsed)
			msleep(bounded_hold - elapsed);

		dsi_m6_dump_snapshot("bist-profile-hold-end", module, cmdq);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, 0x00);
		dsi_m6_dump_snapshot("bist-profile-post-disable", module, cmdq);
		DISPERR("M6 DSI bist_profile: end profile=%u/%s\n",
			profile, dsi_m6_bist_profile_name(profile));
	}

	return DSI_STATUS_OK;
}

int ddp_dsi_porch_setting(DISP_MODULE_ENUM module, void *handle,
		DSI_PORCH_TYPE type, unsigned int value)
{
	int ret = 0;

	if (DISP_MODULE_DSI0 == module) {
		if (DSI_VFP == type) {
			DISPMSG("set dsi vfp to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_VFP_NL, value);
		}
		if (DSI_VSA == type) {
			DISPMSG("set dsi vsa to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_VSA_NL, value);
		}
		if (DSI_VBP == type) {
			DISPMSG("set dsi vbp to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_VBP_NL, value);
		}
		if (DSI_VACT == type) {
			DISPMSG("set dsi vact to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_VACT_NL, value);
		}
		if (DSI_HFP == type) {
			DISPMSG("set dsi hfp to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_HFP_WC, value);
		}
		if (DSI_HSA == type) {
			DISPMSG("set dsi hsa to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_HSA_WC, value);
		}
		if (DSI_HBP == type) {
			DISPMSG("set dsi hbp to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_HBP_WC, value);
		}
		if (DSI_BLLP == type) {
			DISPMSG("set dsi bllp to %d\n", value);
			DSI_OUTREG32(handle, &DSI_REG[0]->DSI_BLLP_WC, value);
		}
	}
	return ret;
}

void DSI_Config_VDO_Timing(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int line_byte = 0;
	unsigned int horizontal_sync_active_byte = 0;
	unsigned int horizontal_backporch_byte = 0;
	unsigned int horizontal_frontporch_byte = 0;
	unsigned int horizontal_bllp_byte = 0;
	unsigned int dsiTmpBufBpp = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (dsi_params->data_format.format == LCM_DSI_FORMAT_RGB565)
			dsiTmpBufBpp = 2;
		else
			dsiTmpBufBpp = 3;

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VSA_NL, dsi_params->vertical_sync_active);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VBP_NL, dsi_params->vertical_backporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VFP_NL, dsi_params->vertical_frontporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VACT_NL, dsi_params->vertical_active_line);

		line_byte =
		    (dsi_params->horizontal_sync_active + dsi_params->horizontal_backporch +
		     dsi_params->horizontal_frontporch +
		     dsi_params->horizontal_active_pixel) * dsiTmpBufBpp;
		horizontal_sync_active_byte =
		    (dsi_params->horizontal_sync_active * dsiTmpBufBpp - 4);

		if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE
		    || dsi_params->switch_mode == SYNC_EVENT_VDO_MODE
		    || dsi_params->switch_mode == BURST_VDO_MODE) {
			ASSERT((dsi_params->horizontal_backporch +
				dsi_params->horizontal_sync_active) * dsiTmpBufBpp > 9);
			horizontal_backporch_byte =
			    ((dsi_params->horizontal_backporch +
			      dsi_params->horizontal_sync_active) * dsiTmpBufBpp - 10);
		} else {
			ASSERT(dsi_params->horizontal_sync_active * dsiTmpBufBpp > 9);
			horizontal_sync_active_byte =
			    (dsi_params->horizontal_sync_active * dsiTmpBufBpp - 10);

			ASSERT(dsi_params->horizontal_backporch * dsiTmpBufBpp > 9);
			horizontal_backporch_byte =
			    (dsi_params->horizontal_backporch * dsiTmpBufBpp - 10);
		}

		ASSERT(dsi_params->horizontal_frontporch * dsiTmpBufBpp > 11);
		horizontal_frontporch_byte =
		    (dsi_params->horizontal_frontporch * dsiTmpBufBpp - 12);
		horizontal_bllp_byte = (dsi_params->horizontal_bllp * dsiTmpBufBpp);

		DISPERR("M6 DSI timing_calc[before-enqueue]: cmdq=%p mode=%u switch=%u bpp=%u h=%u/%u/%u/%u raw=0x%x/0x%x/0x%x/0x%x aligned=0x%x/0x%x/0x%x/0x%x live=0x%x/0x%x/0x%x/0x%x/0x%x\n",
			cmdq, dsi_params->mode, dsi_params->switch_mode,
			dsiTmpBufBpp, dsi_params->horizontal_sync_active,
			dsi_params->horizontal_backporch,
			dsi_params->horizontal_frontporch,
			dsi_params->horizontal_active_pixel,
			horizontal_sync_active_byte, horizontal_backporch_byte,
			horizontal_frontporch_byte, horizontal_bllp_byte,
			ALIGN_TO(horizontal_sync_active_byte, 4),
			ALIGN_TO(horizontal_backporch_byte, 4),
			ALIGN_TO(horizontal_frontporch_byte, 4),
			ALIGN_TO(horizontal_bllp_byte, 4),
			INREG32(DDP_REG_BASE_DSI0 + 0x050),
			INREG32(DDP_REG_BASE_DSI0 + 0x054),
			INREG32(DDP_REG_BASE_DSI0 + 0x058),
			INREG32(DDP_REG_BASE_DSI0 + 0x05c),
			INREG32(DDP_REG_BASE_DSI0 + 0x064));
		dsi_m6_sram_snapshot("timing-before-enqueue", module);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSA_WC,
			     ALIGN_TO((horizontal_sync_active_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC,
			     ALIGN_TO((horizontal_backporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC,
			     ALIGN_TO((horizontal_frontporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BLLP_WC, ALIGN_TO((horizontal_bllp_byte), 4));

		DISPERR("M6 DSI timing_calc[after-enqueue]: cmdq=%p live=0x%x/0x%x/0x%x/0x%x/0x%x\n",
			cmdq, INREG32(DDP_REG_BASE_DSI0 + 0x050),
			INREG32(DDP_REG_BASE_DSI0 + 0x054),
			INREG32(DDP_REG_BASE_DSI0 + 0x058),
			INREG32(DDP_REG_BASE_DSI0 + 0x05c),
			INREG32(DDP_REG_BASE_DSI0 + 0x064));
		dsi_m6_sram_snapshot("timing-after-enqueue", module);
	}
}

void DSI_Set_LFR(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode,
		 unsigned int type, unsigned int enable, unsigned int skip_num)
{
	/* LFR_MODE 0 disable,1 static mode ,2 dynamic mode 3,both */
	unsigned int i = 0;

	/* DISPMSG("module=%d,mode=%d,type=%d,enable=%d,skip_num=%d\n",module,mode,type,enable,skip_num); */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_MODE, mode);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_TYPE, 0);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 1);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_VSE_DIS, 0);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_SKIP_NUM,
			      skip_num);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_EN, enable);
	}
}

void DSI_LFR_UPDATE(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	unsigned int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 0);
		DSI_OUTREGBIT(cmdq, DSI_LFR_CON_REG, DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 1);
	}
}

int DSI_LFR_Status_Check(void)
{
	unsigned int status = 0;

	DSI_LFR_STA_REG lfr_skip_sta;

	lfr_skip_sta = DSI_REG[0]->DSI_LFR_STA;
	status = lfr_skip_sta.LFR_SKIP_STA;
	DISPMSG("LFR_SKIP_CNT 0x%x LFR_SKIP_STA 0x%x,status 0x%x\n", lfr_skip_sta.LFR_SKIP_CNT,
		  lfr_skip_sta.LFR_SKIP_STA, status);

	return status;
}

int _dsi_ps_type_to_bpp(LCM_PS_TYPE ps)
{
	switch (ps) {
	case LCM_PACKED_PS_16BIT_RGB565:
		return 2;
	case LCM_LOOSELY_PS_18BIT_RGB666:
		return 3;
	case LCM_PACKED_PS_24BIT_RGB888:
		return 3;
	case LCM_PACKED_PS_18BIT_RGB666:
		return 3;
	default:
		break;
	}
	return 0;
}

DSI_STATUS DSI_PS_Control(DISP_MODULE_ENUM module, cmdqRecHandle cmdq,
				  LCM_DSI_PARAMS *dsi_params, int w, int h)
{
	int i = 0;
	unsigned int ps_sel_bitvalue = 0;
	/* /TODO: parameter checking */
	ASSERT(_dsi_ps_type_to_bpp(dsi_params->PS) <= PACKED_PS_18BIT_RGB666);

	if (_dsi_ps_type_to_bpp(dsi_params->PS) > LOOSELY_PS_18BIT_RGB666)
		ps_sel_bitvalue = (5 - dsi_params->PS);
	else
		ps_sel_bitvalue = dsi_params->PS;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_VACT_NL_REG, DSI_REG[i]->DSI_VACT_NL, VACT_NL, h);
		if (dsi_params->ufoe_enable && dsi_params->ufoe_params.lr_mode_en != 1) {
			if (dsi_params->ufoe_params.compress_ratio == 3) {
				unsigned int ufoe_internal_width = w + w % 4;

				if (ufoe_internal_width % 3 == 0) {
					DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL,
						      DSI_PS_WC,
						      (ufoe_internal_width / 3) *
						      _dsi_ps_type_to_bpp(dsi_params->PS));
				} else {
					unsigned int temp_w = ufoe_internal_width / 3 + 1;

					temp_w = ((temp_w % 2) == 1) ? (temp_w + 1) : temp_w;
					DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL,
						      DSI_PS_WC,
						      temp_w * _dsi_ps_type_to_bpp(dsi_params->PS));
				}
			} else	/* 1/2 */
				DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL,
					      DSI_PS_WC,
					      (w +
					       w % 4) / 2 * _dsi_ps_type_to_bpp(dsi_params->PS));
		} else {
			DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL, DSI_PS_WC,
				      w * _dsi_ps_type_to_bpp(dsi_params->PS));
		}

		DSI_OUTREGBIT(cmdq, DSI_PSCTRL_REG, DSI_REG[i]->DSI_PSCTRL, DSI_PS_SEL,
			      ps_sel_bitvalue);
	}

	return DSI_STATUS_OK;
}

DSI_STATUS DSI_TXRX_Control(DISP_MODULE_ENUM module, cmdqRecHandle cmdq,
			    LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int lane_num_bitvalue = 0;
	int lane_num = dsi_params->LANE_NUM;
	int vc_num = 0;
	bool null_packet_en = false;
	bool dis_eotp_en = false;
	bool hstx_cklp_en = dsi_params->cont_clock ? false : true;
	int max_return_size = 0;

	switch (lane_num) {
	case LCM_ONE_LANE:
		lane_num_bitvalue = 0x1;
		break;
	case LCM_TWO_LANE:
		lane_num_bitvalue = 0x3;
		break;
	case LCM_THREE_LANE:
		lane_num_bitvalue = 0x7;
		break;
	case LCM_FOUR_LANE:
		lane_num_bitvalue = 0xF;
		break;
	default:
		ASSERT(0);
		break;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, VC_NUM, vc_num);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, DIS_EOT,
			      dis_eotp_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, BLLP_EN,
			      null_packet_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, MAX_RTN_SIZE,
			      max_return_size);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			      hstx_cklp_en);
		DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, LANE_NUM,
			      lane_num_bitvalue);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI, DSI_WMEM_CONTI);
		if (CMD_MODE == dsi_params->mode
		    || (CMD_MODE != dsi_params->mode && dsi_params->eint_disable)) {
			if (dsi_params->ext_te_edge == LCM_POLARITY_FALLING) {
				/*use ext te falling edge */
				DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL,
					      EXT_TE_EDGE, 1);
			}
			DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN,
				      1);
		} else {
			DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EDGE, 0);
			DSI_OUTREGBIT(cmdq, DSI_TXRX_CTRL_REG, DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 0);
		}
	}
	return DSI_STATUS_OK;
}

int MIPITX_IsEnabled(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	int ret = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_PLL_EN)
			ret++;
	}

	DISPDBG("MIPITX for %s is %s\n", ddp_get_module_name(module), ret ? "on" : "off");
	return ret;
}

#ifndef CONFIG_FPGA_EARLY_PORTING
static unsigned int dsi_m6_field(uint32_t value, unsigned int shift, unsigned int width)
{
	return (value >> shift) & ((1U << width) - 1);
}

static unsigned int dsi_m6_lk_rt_code(uint32_t raw, unsigned int shift)
{
	unsigned int code = dsi_m6_field(raw, shift, 4);

	return code ? code : 8;
}

static void dsi_m6_dump_rt_cal(const char *tag)
{
	static unsigned int count;
	static void __iomem *rt_cal_base;
	static bool rt_cal_iomap_tried;
	uint32_t raw = 0;
	uint32_t raw_valid = 0;
	uint32_t live_c;
	uint32_t live_d3;
	uint32_t live_d2;
	uint32_t live_d1;
	uint32_t live_d0;

	if (count >= 96)
		return;

	count++;
	if (!rt_cal_iomap_tried) {
		rt_cal_base = ioremap_nocache(M6_MIPITX_RT_CAL_PHYS, 4);
		rt_cal_iomap_tried = true;
	}
	if (rt_cal_base) {
		raw = readl(rt_cal_base);
		raw_valid = 1;
	}
	live_c = INREG32(MIPITX_BASE + 0x004);
	live_d3 = INREG32(MIPITX_BASE + 0x014);
	live_d2 = INREG32(MIPITX_BASE + 0x010);
	live_d1 = INREG32(MIPITX_BASE + 0x00c);
	live_d0 = INREG32(MIPITX_BASE + 0x008);

	DISPERR("M6 DSI rtcal[%s]: phys10206190 raw_valid=%u raw=0x%x lk_eff c/d3/d2/d1/d0=0x%x/0x%x/0x%x/0x%x/0x%x live_rt=0x%x/0x%x/0x%x/0x%x/0x%x saved_rt=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, raw_valid, raw, dsi_m6_lk_rt_code(raw, 16),
		dsi_m6_lk_rt_code(raw, 8), dsi_m6_lk_rt_code(raw, 12),
		dsi_m6_lk_rt_code(raw, 20), dsi_m6_lk_rt_code(raw, 24),
		dsi_m6_field(live_c, 8, 4), dsi_m6_field(live_d3, 8, 4),
		dsi_m6_field(live_d2, 8, 4), dsi_m6_field(live_d1, 8, 4),
		dsi_m6_field(live_d0, 8, 4), dsi_m6_field(clock_lane, 8, 4),
		dsi_m6_field(data_lane3, 8, 4), dsi_m6_field(data_lane2, 8, 4),
		dsi_m6_field(data_lane1, 8, 4), dsi_m6_field(data_lane0, 8, 4));
}

static void dsi_m6_dump_mipitx_decode(const char *tag)
{
	uint32_t txrx = INREG32(DDP_REG_BASE_DSI0 + 0x018);
	uint32_t clock_lane = INREG32(MIPITX_BASE + 0x004);
	uint32_t lane0 = INREG32(MIPITX_BASE + 0x008);
	uint32_t lane1 = INREG32(MIPITX_BASE + 0x00c);
	uint32_t lane2 = INREG32(MIPITX_BASE + 0x010);
	uint32_t lane3 = INREG32(MIPITX_BASE + 0x014);
	uint32_t top = INREG32(MIPITX_BASE + 0x040);
	uint32_t pll0 = INREG32(MIPITX_BASE + 0x050);
	uint32_t pll2 = INREG32(MIPITX_BASE + 0x058);
	uint32_t pll_top = INREG32(MIPITX_BASE + 0x064);
	uint32_t pll_pwr = INREG32(MIPITX_BASE + 0x068);
	uint32_t gpi_en = INREG32(MIPITX_BASE + 0x074);
	uint32_t gpi_pull = INREG32(MIPITX_BASE + 0x078);
	uint32_t phy_sel = INREG32(MIPITX_BASE + 0x07c);
	uint32_t sw_ctrl = INREG32(MIPITX_BASE + 0x080);
	uint32_t sw0 = INREG32(MIPITX_BASE + 0x084);
	uint32_t sw1 = INREG32(MIPITX_BASE + 0x088);
	uint32_t dbg = INREG32(MIPITX_BASE + 0x090);
	uint32_t apb = INREG32(MIPITX_BASE + 0x094);

	DISPERR("M6 DSI phydecode[%s]: txrx_lane_mask=0x%x hstx_cklp=%u dis_eot=%u bllp=%u max_rtn=0x%x mode=0x%x ps=0x%x\n",
		tag, dsi_m6_field(txrx, 2, 4), dsi_m6_field(txrx, 16, 1),
		dsi_m6_field(txrx, 6, 1), dsi_m6_field(txrx, 7, 1),
		dsi_m6_field(txrx, 12, 4),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c));
	DISPERR("M6 DSI phydecode[%s]: lane_ldo c/d0/d1/d2/d3=%u/%u/%u/%u/%u lane_b1=%u/%u/%u/%u/%u rt=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, dsi_m6_field(clock_lane, 0, 1), dsi_m6_field(lane0, 0, 1),
		dsi_m6_field(lane1, 0, 1), dsi_m6_field(lane2, 0, 1),
		dsi_m6_field(lane3, 0, 1), dsi_m6_field(clock_lane, 1, 1),
		dsi_m6_field(lane0, 1, 1), dsi_m6_field(lane1, 1, 1),
		dsi_m6_field(lane2, 1, 1), dsi_m6_field(lane3, 1, 1),
		dsi_m6_field(clock_lane, 8, 4), dsi_m6_field(lane0, 8, 4),
		dsi_m6_field(lane1, 8, 4), dsi_m6_field(lane2, 8, 4),
		dsi_m6_field(lane3, 8, 4));
	DISPERR("M6 DSI phydecode[%s]: lptx c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x lpcd=0x%x/0x%x/0x%x/0x%x/0x%x phy_map d0/d1/d2/d3/c/lprx=%u/%u/%u/%u/%u/%u\n",
		tag, dsi_m6_field(clock_lane, 2, 3), dsi_m6_field(lane0, 2, 3),
		dsi_m6_field(lane1, 2, 3), dsi_m6_field(lane2, 2, 3),
		dsi_m6_field(lane3, 2, 3), dsi_m6_field(clock_lane, 5, 2),
		dsi_m6_field(lane0, 5, 2), dsi_m6_field(lane1, 5, 2),
		dsi_m6_field(lane2, 5, 2), dsi_m6_field(lane3, 5, 2),
		dsi_m6_field(phy_sel, 0, 3), dsi_m6_field(phy_sel, 4, 3),
		dsi_m6_field(phy_sel, 8, 3), dsi_m6_field(phy_sel, 12, 3),
		dsi_m6_field(phy_sel, 16, 3), dsi_m6_field(phy_sel, 20, 3));
	DISPERR("M6 DSI phydecode[%s]: top hs_bias=%u imp_en=%u imp=0x%x aio=0x%x pad_low=%u pll en=%u pre=%u txdiv=%u/%u pos=%u pcw=0x%x pll_top=0x%x preserve7=0x%x preserve8=0x%x pwr_on=%u iso=%u ack=%u gpi=0x%x pull=0x%x sw=%u/0x%x/0x%x dbg=0x%x apb=0x%x\n",
		tag, dsi_m6_field(top, 1, 1), dsi_m6_field(top, 2, 1),
		dsi_m6_field(top, 4, 4), dsi_m6_field(top, 8, 3),
		dsi_m6_field(top, 11, 1), dsi_m6_field(pll0, 0, 1),
		dsi_m6_field(pll0, 1, 2), dsi_m6_field(pll0, 3, 2),
		dsi_m6_field(pll0, 5, 2), dsi_m6_field(pll0, 7, 3),
		dsi_m6_field(pll2, 0, 31), pll_top,
		dsi_m6_field(pll_top, 7, 5), dsi_m6_field(pll_top, 8, 8),
		dsi_m6_field(pll_pwr, 0, 1),
		dsi_m6_field(pll_pwr, 1, 1), dsi_m6_field(pll_pwr, 8, 1),
		gpi_en, gpi_pull, dsi_m6_field(sw_ctrl, 0, 1), sw0, sw1, dbg, apb);
	DISPERR("M6 DISPLAY truth[%s][mipitx]: raw c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x lane_map d0/d1/d2/d3/c/lprx=%u/%u/%u/%u/%u/%u lptx=0x%x/0x%x/0x%x/0x%x/0x%x lpcd=0x%x/0x%x/0x%x/0x%x/0x%x pll=0x%x/0x%x/0x%x pll_top=0x%x preserve7=0x%x preserve8=0x%x pwr=0x%x sw=0x%x/0x%x/0x%x dbg=0x%x apb=0x%x\n",
		tag, clock_lane, lane0, lane1, lane2, lane3,
		dsi_m6_field(phy_sel, 0, 3), dsi_m6_field(phy_sel, 4, 3),
		dsi_m6_field(phy_sel, 8, 3), dsi_m6_field(phy_sel, 12, 3),
		dsi_m6_field(phy_sel, 16, 3), dsi_m6_field(phy_sel, 20, 3),
		dsi_m6_field(clock_lane, 2, 3), dsi_m6_field(lane0, 2, 3),
		dsi_m6_field(lane1, 2, 3), dsi_m6_field(lane2, 2, 3),
		dsi_m6_field(lane3, 2, 3), dsi_m6_field(clock_lane, 5, 2),
		dsi_m6_field(lane0, 5, 2), dsi_m6_field(lane1, 5, 2),
		dsi_m6_field(lane2, 5, 2), dsi_m6_field(lane3, 5, 2),
		pll0, pll2, pll_pwr, pll_top,
		dsi_m6_field(pll_top, 7, 5), dsi_m6_field(pll_top, 8, 8),
		INREG32(MIPITX_BASE + 0x06c),
		sw_ctrl, sw0, sw1, dbg, apb);
	DISPERR("M6 DSI phydecode[%s]: dbg_out=0x%x apb_async=0x%x\n",
		tag, INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098));
}

static void dsi_m6_dump_mipitx_block(const char *tag)
{
	static unsigned int count;
	const char *safe_tag = tag ? tag : "unknown";

	if (count >= 80)
		return;

	count++;
	DISPERR("M6 DSI mipitx_block[%s]#%u: 000=0x%x 004=0x%x 008=0x%x 00c=0x%x 010=0x%x 014=0x%x 018=0x%x 01c=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x000),
		INREG32(MIPITX_BASE + 0x004), INREG32(MIPITX_BASE + 0x008),
		INREG32(MIPITX_BASE + 0x00c), INREG32(MIPITX_BASE + 0x010),
		INREG32(MIPITX_BASE + 0x014), INREG32(MIPITX_BASE + 0x018),
		INREG32(MIPITX_BASE + 0x01c));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 020=0x%x 024=0x%x 028=0x%x 02c=0x%x 030=0x%x 034=0x%x 038=0x%x 03c=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x020),
		INREG32(MIPITX_BASE + 0x024), INREG32(MIPITX_BASE + 0x028),
		INREG32(MIPITX_BASE + 0x02c), INREG32(MIPITX_BASE + 0x030),
		INREG32(MIPITX_BASE + 0x034), INREG32(MIPITX_BASE + 0x038),
		INREG32(MIPITX_BASE + 0x03c));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 040=0x%x 044=0x%x 048=0x%x 04c=0x%x 050=0x%x 054=0x%x 058=0x%x 05c=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044), INREG32(MIPITX_BASE + 0x048),
		INREG32(MIPITX_BASE + 0x04c), INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x054), INREG32(MIPITX_BASE + 0x058),
		INREG32(MIPITX_BASE + 0x05c));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 060=0x%x 064=0x%x 068=0x%x 06c=0x%x 070=0x%x 074=0x%x 078=0x%x 07c=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x060),
		INREG32(MIPITX_BASE + 0x064), INREG32(MIPITX_BASE + 0x068),
		INREG32(MIPITX_BASE + 0x06c), INREG32(MIPITX_BASE + 0x070),
		INREG32(MIPITX_BASE + 0x074), INREG32(MIPITX_BASE + 0x078),
		INREG32(MIPITX_BASE + 0x07c));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 080=0x%x 084=0x%x 088=0x%x 08c=0x%x 090=0x%x 094=0x%x 098=0x%x 09c=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x080),
		INREG32(MIPITX_BASE + 0x084), INREG32(MIPITX_BASE + 0x088),
		INREG32(MIPITX_BASE + 0x08c), INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094), INREG32(MIPITX_BASE + 0x098),
		INREG32(MIPITX_BASE + 0x09c));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 0a0=0x%x 0a4=0x%x 0a8=0x%x 0ac=0x%x 0b0=0x%x 0b4=0x%x 0b8=0x%x 0bc=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x0a0),
		INREG32(MIPITX_BASE + 0x0a4), INREG32(MIPITX_BASE + 0x0a8),
		INREG32(MIPITX_BASE + 0x0ac), INREG32(MIPITX_BASE + 0x0b0),
		INREG32(MIPITX_BASE + 0x0b4), INREG32(MIPITX_BASE + 0x0b8),
		INREG32(MIPITX_BASE + 0x0bc));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 0c0=0x%x 0c4=0x%x 0c8=0x%x 0cc=0x%x 0d0=0x%x 0d4=0x%x 0d8=0x%x 0dc=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x0c0),
		INREG32(MIPITX_BASE + 0x0c4), INREG32(MIPITX_BASE + 0x0c8),
		INREG32(MIPITX_BASE + 0x0cc), INREG32(MIPITX_BASE + 0x0d0),
		INREG32(MIPITX_BASE + 0x0d4), INREG32(MIPITX_BASE + 0x0d8),
		INREG32(MIPITX_BASE + 0x0dc));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 0e0=0x%x 0e4=0x%x 0e8=0x%x 0ec=0x%x 0f0=0x%x 0f4=0x%x 0f8=0x%x 0fc=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x0e0),
		INREG32(MIPITX_BASE + 0x0e4), INREG32(MIPITX_BASE + 0x0e8),
		INREG32(MIPITX_BASE + 0x0ec), INREG32(MIPITX_BASE + 0x0f0),
		INREG32(MIPITX_BASE + 0x0f4), INREG32(MIPITX_BASE + 0x0f8),
		INREG32(MIPITX_BASE + 0x0fc));
	DISPERR("M6 DSI mipitx_block[%s]#%u: 100=0x%x 104=0x%x\n",
		safe_tag, count, INREG32(MIPITX_BASE + 0x100),
		INREG32(MIPITX_BASE + 0x104));
}

static void dsi_m6_dump_dsi_block(const char *tag)
{
	static unsigned int count;
	const char *safe_tag = tag ? tag : "unknown";
	unsigned int off;

	if (count >= 40)
		return;

	count++;

	for (off = 0; off <= M6_LKGOLD_DSI_LAST; off += 0x20) {
		if (off + 0x1c <= M6_LKGOLD_DSI_LAST) {
			DISPERR("M6 DSI raw_block[%s]#%u: %03x=0x%x %03x=0x%x %03x=0x%x %03x=0x%x %03x=0x%x %03x=0x%x %03x=0x%x %03x=0x%x\n",
				safe_tag, count, off,
				INREG32(DDP_REG_BASE_DSI0 + off),
				off + 0x04,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x04),
				off + 0x08,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x08),
				off + 0x0c,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x0c),
				off + 0x10,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x10),
				off + 0x14,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x14),
				off + 0x18,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x18),
				off + 0x1c,
				INREG32(DDP_REG_BASE_DSI0 + off + 0x1c));
		} else {
			unsigned int tail;

			for (tail = off; tail <= M6_LKGOLD_DSI_LAST; tail += 4)
				DISPERR("M6 DSI raw_block[%s]#%u: %03x=0x%x\n",
					safe_tag, count, tail,
					INREG32(DDP_REG_BASE_DSI0 + tail));
		}
	}
}

struct m6_lkgold_snapshot {
	bool valid;
	uint32_t dsi[M6_LKGOLD_DSI_WORDS];
	uint32_t mipitx[M6_LKGOLD_MIPITX_WORDS];
	uint32_t mmsys[6];
};

static const unsigned short m6_lkgold_mmsys_offsets[] = {
	0x06c, 0x070, 0x074, 0x07c, 0x100, 0x110,
};

static struct m6_lkgold_snapshot m6_lkgold_pre_snapshot;
static bool m6_lkgold_dumped_pre;
static bool m6_lkgold_dumped_post_config;
static bool m6_lkgold_dumped_post_start;

static void dsi_m6_dump_mipitx_debug_mux_stats_direct(const char *tag,
						      unsigned int sweep_id,
						      unsigned int samples,
						      unsigned int delay_us);

#define M6_LKGOLD_MUX_TAGS 3
#define M6_LKGOLD_MUX_SELS 16

struct m6_lkgold_mipitx_mux_cache {
	bool valid;
	char tag[24];
	unsigned int sweep_id;
	unsigned int sel;
	unsigned int samples;
	unsigned int delay_us;
	uint32_t orig;
	uint32_t now;
	uint32_t out_first;
	uint32_t out_last;
	uint32_t out_min;
	uint32_t out_max;
	uint32_t out_or;
	uint32_t out_and;
	uint32_t out_xor;
	unsigned int out_changes;
	uint32_t apb_first;
	uint32_t apb_last;
	uint32_t apb_or;
	uint32_t apb_and;
	uint32_t apb_xor;
	unsigned int apb_changes;
	unsigned int word_first;
	unsigned int word_last;
	unsigned int line_first;
	unsigned int line_last;
	uint32_t lanes[5];
	uint32_t pll[3];
};

static struct m6_lkgold_mipitx_mux_cache
	m6_lkgold_mipitx_mux_cache[M6_LKGOLD_MUX_TAGS][M6_LKGOLD_MUX_SELS];

static int dsi_m6_lkgold_mux_cache_slot(const char *tag)
{
	if (!tag)
		return -1;
	if (!strncmp(tag, "lkgold-pre-init", sizeof("lkgold-pre-init") - 1))
		return 0;
	if (!strncmp(tag, "lkgold-post-config", sizeof("lkgold-post-config") - 1))
		return 1;
	if (!strncmp(tag, "lkgold-post-start", sizeof("lkgold-post-start") - 1))
		return 2;
	return -1;
}

static void dsi_m6_lkgold_capture(struct m6_lkgold_snapshot *snap)
{
	unsigned int idx;

	memset(snap, 0, sizeof(*snap));
	if (DSI_REG[0] == NULL)
		return;

	snap->valid = true;
	for (idx = 0; idx < ARRAY_SIZE(snap->dsi); idx++)
		snap->dsi[idx] = INREG32(DDP_REG_BASE_DSI0 + (idx * 4));

#ifndef CONFIG_FPGA_EARLY_PORTING
	for (idx = 0; idx < ARRAY_SIZE(snap->mipitx); idx++)
		snap->mipitx[idx] = INREG32(MIPITX_BASE + (idx * 4));
#endif

	for (idx = 0; idx < ARRAY_SIZE(snap->mmsys); idx++)
		snap->mmsys[idx] =
			INREG32(DDP_REG_BASE_MMSYS_CONFIG +
				m6_lkgold_mmsys_offsets[idx]);
}

static void dsi_m6_lkgold_dump_raw(const char *tag,
				   const struct m6_lkgold_snapshot *snap)
{
	unsigned int idx;

	if (!snap->valid)
		return;

	for (idx = 0; idx < ARRAY_SIZE(snap->dsi); idx++)
		DISPERR("M6 lkgold[%s] blk=dsi off=0x%03x val=0x%08x\n",
			tag, idx * 4, snap->dsi[idx]);

#ifndef CONFIG_FPGA_EARLY_PORTING
	for (idx = 0; idx < ARRAY_SIZE(snap->mipitx); idx++)
		DISPERR("M6 lkgold[%s] blk=mipitx off=0x%03x val=0x%08x\n",
			tag, idx * 4, snap->mipitx[idx]);
#endif

	for (idx = 0; idx < ARRAY_SIZE(snap->mmsys); idx++)
		DISPERR("M6 lkgold[%s] blk=mmsys off=0x%03x val=0x%08x\n",
			tag, m6_lkgold_mmsys_offsets[idx], snap->mmsys[idx]);
}

static void dsi_m6_lkgold_dump_diff(const char *stage,
				    const struct m6_lkgold_snapshot *snap)
{
	unsigned int idx;

	if (!m6_lkgold_pre_snapshot.valid || !snap->valid)
		return;

	for (idx = 0; idx < ARRAY_SIZE(snap->dsi); idx++) {
		if (m6_lkgold_pre_snapshot.dsi[idx] == snap->dsi[idx])
			continue;
		DISPERR("M6 lkgold[diff] stage=%s blk=dsi off=0x%03x lk=0x%08x lin=0x%08x\n",
			stage, idx * 4, m6_lkgold_pre_snapshot.dsi[idx],
			snap->dsi[idx]);
	}

#ifndef CONFIG_FPGA_EARLY_PORTING
	for (idx = 0; idx < ARRAY_SIZE(snap->mipitx); idx++) {
		if (m6_lkgold_pre_snapshot.mipitx[idx] == snap->mipitx[idx])
			continue;
		DISPERR("M6 lkgold[diff] stage=%s blk=mipitx off=0x%03x lk=0x%08x lin=0x%08x\n",
			stage, idx * 4, m6_lkgold_pre_snapshot.mipitx[idx],
			snap->mipitx[idx]);
	}
#endif

	for (idx = 0; idx < ARRAY_SIZE(snap->mmsys); idx++) {
		if (m6_lkgold_pre_snapshot.mmsys[idx] == snap->mmsys[idx])
			continue;
		DISPERR("M6 lkgold[diff] stage=%s blk=mmsys off=0x%03x lk=0x%08x lin=0x%08x\n",
			stage, m6_lkgold_mmsys_offsets[idx],
			m6_lkgold_pre_snapshot.mmsys[idx], snap->mmsys[idx]);
	}
}

static void dsi_m6_lkgold_muxstats(const char *tag)
{
	char mux_tag[32];
	static unsigned int sweep_count;

	if (!tag || DSI_REG[0] == NULL)
		return;

	snprintf(mux_tag, sizeof(mux_tag), "lkgold-%s", tag);
	DISPERR("M6 lkgold_muxstats[%s]#%u: begin dbg=0x%x out=0x%x apb=0x%x start=0x%x mode=0x%x txrx=0x%x lccon=0x%x\n",
		tag, sweep_count + 1, INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094), INREG32(MIPITX_BASE + 0x098),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x104));
	dsi_m6_dump_mipitx_debug_mux_stats_direct(mux_tag, ++sweep_count, 4, 1000);
	DISPERR("M6 lkgold_muxstats[%s]#%u: end dbg=0x%x out=0x%x apb=0x%x start=0x%x mode=0x%x txrx=0x%x lccon=0x%x\n",
		tag, sweep_count, INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094), INREG32(MIPITX_BASE + 0x098),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x104));
}

static void dsi_m6_lkgold_snapshot_once(const char *tag)
{
	struct m6_lkgold_snapshot snap;
	bool *done;

	if (!tag)
		return;
	if (!strcmp(tag, "pre-init"))
		done = &m6_lkgold_dumped_pre;
	else if (!strcmp(tag, "post-config"))
		done = &m6_lkgold_dumped_post_config;
	else if (!strcmp(tag, "post-start"))
		done = &m6_lkgold_dumped_post_start;
	else
		return;

	if (*done)
		return;

	*done = true;
	dsi_m6_lkgold_capture(&snap);
	if (!snap.valid) {
		DISPERR("M6 lkgold[%s] invalid: DSI_REG0 is null\n", tag);
		return;
	}

	if (!strcmp(tag, "pre-init"))
		m6_lkgold_pre_snapshot = snap;

	dsi_m6_lkgold_dump_raw(tag, &snap);
	if (strcmp(tag, "pre-init"))
		dsi_m6_lkgold_dump_diff(tag, &snap);
	dsi_m6_lkgold_muxstats(tag);
}

static void dsi_m6_phy_lk_delay(const char *tag, unsigned int delay_ms)
{
	DISPERR("M6 DSI physeq[%s]: lk-delay begin ms=%u top=0x%x bg=0x%x pll0=0x%x pll_chg=0x%x pll_top=0x%x pwr=0x%x lanes=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, delay_ms, INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044), INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x060), INREG32(MIPITX_BASE + 0x064),
		INREG32(MIPITX_BASE + 0x068), INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008), INREG32(MIPITX_BASE + 0x00c),
		INREG32(MIPITX_BASE + 0x010), INREG32(MIPITX_BASE + 0x014));
	mdelay(delay_ms);
	DISPERR("M6 DSI physeq[%s]: lk-delay end ms=%u top=0x%x bg=0x%x pll0=0x%x pll_chg=0x%x pll_top=0x%x pwr=0x%x lanes=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, delay_ms, INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044), INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x060), INREG32(MIPITX_BASE + 0x064),
		INREG32(MIPITX_BASE + 0x068), INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008), INREG32(MIPITX_BASE + 0x00c),
		INREG32(MIPITX_BASE + 0x010), INREG32(MIPITX_BASE + 0x014));
	dsi_m6_dump_mipitx_decode(tag);
}

static void dsi_m6_dump_state_decode(const char *tag)
{
	uint32_t dbg0 = INREG32(DDP_REG_BASE_DSI0 + 0x148);
	uint32_t dbg1 = INREG32(DDP_REG_BASE_DSI0 + 0x14c);
	uint32_t dbg2 = INREG32(DDP_REG_BASE_DSI0 + 0x150);
	uint32_t dbg3 = INREG32(DDP_REG_BASE_DSI0 + 0x154);
	uint32_t dbg4 = INREG32(DDP_REG_BASE_DSI0 + 0x158);
	uint32_t dbg5 = INREG32(DDP_REG_BASE_DSI0 + 0x15c);
	uint32_t state6 = INREG32(DDP_REG_BASE_DSI0 + 0x160);
	uint32_t state7 = INREG32(DDP_REG_BASE_DSI0 + 0x164);
	uint32_t state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
	uint32_t state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);

	DISPERR("M6 DSI state_decode[%s]: dbg0-5=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x ctl_c=0x%x hs_c=0x%x ctl0=0x%x hs0=0x%x esc0=0x%x ctl1=0x%x hs1=0x%x ctl2=0x%x hs2=0x%x ctl3=0x%x hs3=0x%x\n",
		tag, dbg0, dbg1, dbg2, dbg3, dbg4, dbg5,
		dsi_m6_field(dbg0, 0, 9), dsi_m6_field(dbg0, 16, 5),
		dsi_m6_field(dbg1, 0, 15), dsi_m6_field(dbg1, 16, 5),
		dsi_m6_field(dbg1, 24, 8), dsi_m6_field(dbg3, 0, 5),
		dsi_m6_field(dbg3, 8, 5), dsi_m6_field(dbg3, 16, 5),
		dsi_m6_field(dbg3, 24, 5), dsi_m6_field(dbg4, 0, 5),
		dsi_m6_field(dbg4, 8, 5));
	DISPERR("M6 DSI state_decode[%s]: rx_esc=0x%x ta_t2r=0x%x ta_r2t=0x%x timer=0x%x busy=%u wake=0x%x cm=0x%x cmdq=0x%x vm=0x%x periods vfp/vact/vbp/vsa=%u/%u/%u/%u word=%u line=%u\n",
		tag, dsi_m6_field(dbg2, 0, 10), dsi_m6_field(dbg2, 16, 5),
		dsi_m6_field(dbg2, 24, 5), dsi_m6_field(dbg5, 0, 16),
		dsi_m6_field(dbg5, 16, 1), dsi_m6_field(dbg5, 28, 4),
		dsi_m6_field(state6, 0, 14), dsi_m6_field(state6, 16, 8),
		dsi_m6_field(state7, 0, 11), dsi_m6_field(state7, 12, 1),
		dsi_m6_field(state7, 13, 1), dsi_m6_field(state7, 14, 1),
		dsi_m6_field(state7, 15, 1), dsi_m6_field(state8, 0, 14),
		dsi_m6_field(state9, 0, 22));
}
#endif

static void dsi_m6_sram_snapshot(const char *tag, DISP_MODULE_ENUM module)
{
	static unsigned int count;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL || count >= 20)
		return;

	count++;
	aee_sram_printk("M6D%02u %s S=%x M=%x I=%x H=%x/%x V=%x B=%x L=%x/%x\n",
		count, tag, INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x17c),
		INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008));
	DISPERR("M6D%02u %s S=%x M=%x I=%x H=%x/%x V=%x B=%x L=%x/%x\n",
		count, tag, INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x17c),
		INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008));
}

static void dsi_m6_sram_video_snapshot(const char *tag, DISP_MODULE_ENUM module)
{
	static unsigned int count;
	const char *safe_tag = tag ? tag : "null";
	unsigned int n;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL || count >= 24)
		return;

	n = ++count;
	aee_sram_printk("M6V%02u %s S=%x/%x I=%x M=%x T=%x P=%x H=%x/%x/%x VM=%x/%x/%x ST=%x/%x/%x/%x\n",
		n, safe_tag,
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x004),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c),
		INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x134),
		INREG32(DDP_REG_BASE_DSI0 + 0x138),
		INREG32(DDP_REG_BASE_DSI0 + 0x160),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x168),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c));
#ifndef CONFIG_FPGA_EARLY_PORTING
	aee_sram_printk("M6W%02u %s L=%x/%x/%x/%x/%x T=%x/%x PLL=%x/%x/%x P=%x S=%x/%x D=%x/%x\n",
		n, safe_tag,
		INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008),
		INREG32(MIPITX_BASE + 0x00c),
		INREG32(MIPITX_BASE + 0x010),
		INREG32(MIPITX_BASE + 0x014),
		INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044),
		INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x058),
		INREG32(MIPITX_BASE + 0x068),
		INREG32(MIPITX_BASE + 0x07c),
		INREG32(MIPITX_BASE + 0x084),
		INREG32(MIPITX_BASE + 0x088),
		INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094));
#endif
}

static void dsi_m6_dump_snapshot(const char *tag, DISP_MODULE_ENUM module, void *cmdq)
{
	uint32_t start;
	uint32_t status;
	uint32_t inten;
	uint32_t intsta;
	uint32_t state6;
	uint32_t state7;
	uint32_t state8;
	uint32_t state9;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL)
		return;

	start = INREG32(DDP_REG_BASE_DSI0 + 0x000);
	status = INREG32(DDP_REG_BASE_DSI0 + 0x004);
	inten = INREG32(DDP_REG_BASE_DSI0 + 0x008);
	intsta = INREG32(DDP_REG_BASE_DSI0 + 0x00c);
	state6 = INREG32(DDP_REG_BASE_DSI0 + 0x160);
	state7 = INREG32(DDP_REG_BASE_DSI0 + 0x164);
	state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
	state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);

	DISPERR("M6 DSI snapshot[%s]: cmdq=%p START=0x%x STA=0x%x INTEN=0x%x INTSTA=0x%x MODE=0x%x TXRX=0x%x PS=0x%x\n",
		tag, cmdq, start, status, inten, intsta,
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c));
	dsi_m6_dump_irq_decode(tag, start, status, inten, intsta);
	DISPERR("M6 DSI snapshot[%s]: VSA/VBP/VFP/VACT=0x%x/0x%x/0x%x/0x%x HSA/HBP/HFP/BLLP/HSTX=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, INREG32(DDP_REG_BASE_DSI0 + 0x020),
		INREG32(DDP_REG_BASE_DSI0 + 0x024),
		INREG32(DDP_REG_BASE_DSI0 + 0x028),
		INREG32(DDP_REG_BASE_DSI0 + 0x02c),
		INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x05c),
		INREG32(DDP_REG_BASE_DSI0 + 0x064));
	DISPERR("M6 DSI snapshot[%s]: PHY_LCCON=0x%x PHY_LD0CON=0x%x PHY_SYNCON=0x%x TIM=0x%x/0x%x/0x%x/0x%x VM_CMD=0x%x\n",
		tag, INREG32(DDP_REG_BASE_DSI0 + 0x104),
		INREG32(DDP_REG_BASE_DSI0 + 0x108),
		INREG32(DDP_REG_BASE_DSI0 + 0x10c),
		INREG32(DDP_REG_BASE_DSI0 + 0x110),
		INREG32(DDP_REG_BASE_DSI0 + 0x114),
		INREG32(DDP_REG_BASE_DSI0 + 0x118),
		INREG32(DDP_REG_BASE_DSI0 + 0x11c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130));
	DISPERR("M6 DSI snapshot[%s]: VM_PAYLOAD=0x%x/0x%x/0x%x/0x%x ext=0x%x/0x%x/0x%x/0x%x\n",
		tag, INREG32(DDP_REG_BASE_DSI0 + 0x134),
		INREG32(DDP_REG_BASE_DSI0 + 0x138),
		INREG32(DDP_REG_BASE_DSI0 + 0x13c),
		INREG32(DDP_REG_BASE_DSI0 + 0x140),
		INREG32(DDP_REG_BASE_DSI0 + 0x180),
		INREG32(DDP_REG_BASE_DSI0 + 0x184),
		INREG32(DDP_REG_BASE_DSI0 + 0x188),
		INREG32(DDP_REG_BASE_DSI0 + 0x18c));
	DISPERR("M6 DSI snapshot[%s]: BIST_PATTERN=0x%x BIST_CON=0x%x self_pat=%u bist_en=%u bist_mode=%u fix=%u lane=%u timing=0x%x CKSM=0x%x DEBUG_SEL=0x%x\n",
		tag, INREG32(DDP_REG_BASE_DSI0 + 0x178),
		INREG32(DDP_REG_BASE_DSI0 + 0x17c),
		DSI_REG[0]->DSI_BIST_CON.SELF_PAT_MODE,
		DSI_REG[0]->DSI_BIST_CON.BIST_ENABLE,
		DSI_REG[0]->DSI_BIST_CON.BIST_MODE,
		DSI_REG[0]->DSI_BIST_CON.BIST_FIX_PATTERN,
		DSI_REG[0]->DSI_BIST_CON.BIST_LANE_NUM,
		DSI_REG[0]->DSI_BIST_CON.BIST_TIMING,
		INREG32(DDP_REG_BASE_DSI0 + 0x144),
		INREG32(DDP_REG_BASE_DSI0 + 0x170));
	DISPERR("M6 DSI snapshot[%s]: STATE6=0x%x/%s STATE7=0x%x/%s STATE8=0x%x STATE9=0x%x DBG0-3=0x%x/0x%x/0x%x/0x%x\n",
		tag, state6, _dsi_cmd_mode_parse_state(state6 & 0xffff),
		state7, _dsi_vdo_mode_parse_state(state7 & 0xff),
		state8, state9, INREG32(DDP_REG_BASE_DSI0 + 0x148),
		INREG32(DDP_REG_BASE_DSI0 + 0x14c),
		INREG32(DDP_REG_BASE_DSI0 + 0x150),
		INREG32(DDP_REG_BASE_DSI0 + 0x154));
#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_state_decode(tag);
	DISPERR("M6 DSI snapshot[%s]: MIPITX lanes=0x%x/0x%x/0x%x/0x%x/0x%x top/bg=0x%x/0x%x pll=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008),
		INREG32(MIPITX_BASE + 0x00c),
		INREG32(MIPITX_BASE + 0x010),
		INREG32(MIPITX_BASE + 0x014),
		INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044),
		INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x054),
		INREG32(MIPITX_BASE + 0x058),
		INREG32(MIPITX_BASE + 0x05c),
		INREG32(MIPITX_BASE + 0x060),
		INREG32(MIPITX_BASE + 0x064),
		INREG32(MIPITX_BASE + 0x068));
	DISPERR("M6 DSI snapshot[%s]: MIPITX rgs/gpi/sel/sw/dbg=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, INREG32(MIPITX_BASE + 0x070),
		INREG32(MIPITX_BASE + 0x074),
		INREG32(MIPITX_BASE + 0x078),
		INREG32(MIPITX_BASE + 0x07c),
		INREG32(MIPITX_BASE + 0x080),
		INREG32(MIPITX_BASE + 0x084),
			INREG32(MIPITX_BASE + 0x088),
			INREG32(MIPITX_BASE + 0x08c),
			INREG32(MIPITX_BASE + 0x090),
			INREG32(MIPITX_BASE + 0x094));
		dsi_m6_dump_rt_cal(tag);
		dsi_m6_dump_mipitx_decode(tag);
#endif
}

bool dsi_m6_capture_live_snapshot(struct m6_dsi_live_snapshot *snap)
{
	if (!snap)
		return false;

	memset(snap, 0, sizeof(*snap));
	if (!DSI_REG[0])
		return false;

	snap->valid = true;
	snap->start = INREG32(DDP_REG_BASE_DSI0 + 0x000);
	snap->status = INREG32(DDP_REG_BASE_DSI0 + 0x004);
	snap->inten = INREG32(DDP_REG_BASE_DSI0 + 0x008);
	snap->intsta = INREG32(DDP_REG_BASE_DSI0 + 0x00c);
	snap->mode = INREG32(DDP_REG_BASE_DSI0 + 0x014);
	snap->txrx = INREG32(DDP_REG_BASE_DSI0 + 0x018);
	snap->psctrl = INREG32(DDP_REG_BASE_DSI0 + 0x01c);
	snap->vsa = INREG32(DDP_REG_BASE_DSI0 + 0x020);
	snap->vbp = INREG32(DDP_REG_BASE_DSI0 + 0x024);
	snap->vfp = INREG32(DDP_REG_BASE_DSI0 + 0x028);
	snap->vact = INREG32(DDP_REG_BASE_DSI0 + 0x02c);
	snap->hsa = INREG32(DDP_REG_BASE_DSI0 + 0x050);
	snap->hbp = INREG32(DDP_REG_BASE_DSI0 + 0x054);
	snap->hfp = INREG32(DDP_REG_BASE_DSI0 + 0x058);
	snap->bllp = INREG32(DDP_REG_BASE_DSI0 + 0x05c);
	snap->hstx_ckl = INREG32(DDP_REG_BASE_DSI0 + 0x064);
	snap->phy_lccon = INREG32(DDP_REG_BASE_DSI0 + 0x104);
	snap->phy_ld0con = INREG32(DDP_REG_BASE_DSI0 + 0x108);
	snap->phy_syncon = INREG32(DDP_REG_BASE_DSI0 + 0x10c);
	snap->phy_timecon0 = INREG32(DDP_REG_BASE_DSI0 + 0x110);
	snap->phy_timecon1 = INREG32(DDP_REG_BASE_DSI0 + 0x114);
	snap->phy_timecon2 = INREG32(DDP_REG_BASE_DSI0 + 0x118);
	snap->phy_timecon3 = INREG32(DDP_REG_BASE_DSI0 + 0x11c);
	snap->vm_cmd = INREG32(DDP_REG_BASE_DSI0 + 0x130);
	snap->vm_payload[0] = INREG32(DDP_REG_BASE_DSI0 + 0x134);
	snap->vm_payload[1] = INREG32(DDP_REG_BASE_DSI0 + 0x138);
	snap->vm_payload[2] = INREG32(DDP_REG_BASE_DSI0 + 0x13c);
	snap->vm_payload[3] = INREG32(DDP_REG_BASE_DSI0 + 0x140);
	snap->vm_payload[4] = INREG32(DDP_REG_BASE_DSI0 + 0x180);
	snap->vm_payload[5] = INREG32(DDP_REG_BASE_DSI0 + 0x184);
	snap->vm_payload[6] = INREG32(DDP_REG_BASE_DSI0 + 0x188);
	snap->vm_payload[7] = INREG32(DDP_REG_BASE_DSI0 + 0x18c);
	snap->bist_pattern = INREG32(DDP_REG_BASE_DSI0 + 0x178);
	snap->bist_con = INREG32(DDP_REG_BASE_DSI0 + 0x17c);
	snap->debug_sel = INREG32(DDP_REG_BASE_DSI0 + 0x170);
	snap->state_dbg[0] = INREG32(DDP_REG_BASE_DSI0 + 0x148);
	snap->state_dbg[1] = INREG32(DDP_REG_BASE_DSI0 + 0x14c);
	snap->state_dbg[2] = INREG32(DDP_REG_BASE_DSI0 + 0x150);
	snap->state_dbg[3] = INREG32(DDP_REG_BASE_DSI0 + 0x154);
	snap->state_dbg[4] = INREG32(DDP_REG_BASE_DSI0 + 0x158);
	snap->state_dbg[5] = INREG32(DDP_REG_BASE_DSI0 + 0x15c);
	snap->state_dbg[6] = INREG32(DDP_REG_BASE_DSI0 + 0x160);
	snap->state_dbg[7] = INREG32(DDP_REG_BASE_DSI0 + 0x164);
	snap->state_dbg[8] = INREG32(DDP_REG_BASE_DSI0 + 0x168);
	snap->state_dbg[9] = INREG32(DDP_REG_BASE_DSI0 + 0x16c);
#ifndef CONFIG_FPGA_EARLY_PORTING
	snap->mipitx_lane_c = INREG32(MIPITX_BASE + 0x004);
	snap->mipitx_lane0 = INREG32(MIPITX_BASE + 0x008);
	snap->mipitx_lane1 = INREG32(MIPITX_BASE + 0x00c);
	snap->mipitx_lane2 = INREG32(MIPITX_BASE + 0x010);
	snap->mipitx_lane3 = INREG32(MIPITX_BASE + 0x014);
	snap->mipitx_top = INREG32(MIPITX_BASE + 0x040);
	snap->mipitx_bg = INREG32(MIPITX_BASE + 0x044);
	snap->mipitx_con = INREG32(MIPITX_BASE + 0x048);
	snap->mipitx_pll[0] = INREG32(MIPITX_BASE + 0x050);
	snap->mipitx_pll[1] = INREG32(MIPITX_BASE + 0x054);
	snap->mipitx_pll[2] = INREG32(MIPITX_BASE + 0x058);
	snap->mipitx_pll[3] = INREG32(MIPITX_BASE + 0x05c);
	snap->mipitx_pll[4] = INREG32(MIPITX_BASE + 0x060);
	snap->mipitx_pll[5] = INREG32(MIPITX_BASE + 0x064);
	snap->mipitx_pll[6] = INREG32(MIPITX_BASE + 0x068);
	snap->mipitx_rgs = INREG32(MIPITX_BASE + 0x070);
	snap->mipitx_gpi = INREG32(MIPITX_BASE + 0x074);
	snap->mipitx_pull = INREG32(MIPITX_BASE + 0x078);
	snap->mipitx_phy_sel = INREG32(MIPITX_BASE + 0x07c);
	snap->mipitx_sw_ctrl = INREG32(MIPITX_BASE + 0x080);
	snap->mipitx_sw0 = INREG32(MIPITX_BASE + 0x084);
	snap->mipitx_sw1 = INREG32(MIPITX_BASE + 0x088);
	snap->mipitx_dbg = INREG32(MIPITX_BASE + 0x090);
	snap->mipitx_apb = INREG32(MIPITX_BASE + 0x094);
	snap->mipitx_dbg_out = INREG32(MIPITX_BASE + 0x094);
	snap->mipitx_apb_async = INREG32(MIPITX_BASE + 0x098);
#endif
	return true;
}

static void dsi_m6_dump_snapshot_limited(const char *tag, DISP_MODULE_ENUM module,
					 void *cmdq, unsigned int *count,
					 unsigned int limit)
{
	if (*count >= limit)
		return;

	(*count)++;
	dsi_m6_dump_snapshot(tag, module, cmdq);
}

static void dsi_m6_takeover_hold_once(const char *where,
				      DISP_MODULE_ENUM module, void *cmdq,
				      unsigned int *dump_count)
{
	const char *safe_where = where ? where : "null";

	if (!M6_LK_HANDOFF_TAKEOVER_HOLD_MS)
		return;
	if (module != DISP_MODULE_DSI0)
		return;
	if (m6_lk_handoff_takeover_hold_done)
		return;
	if (atomic_read(&PMaster_enable) != 0 || dsi_force_config)
		return;

	m6_lk_handoff_takeover_hold_done = 1;
	DISPERR("M6 DSI takeover_hold[%s]: begin hold_ms=%u PMaster=%d force=%d jiffies=%lu\n",
		safe_where, M6_LK_HANDOFF_TAKEOVER_HOLD_MS,
		atomic_read(&PMaster_enable), dsi_force_config, jiffies);
	dsi_m6_sram_snapshot("takeover-hold-begin", module);
	dsi_m6_dump_snapshot_limited("takeover-hold-begin", module, cmdq,
				     dump_count, 4);
	msleep(M6_LK_HANDOFF_TAKEOVER_HOLD_MS);
	dsi_m6_sram_snapshot("takeover-hold-end", module);
	dsi_m6_dump_snapshot_limited("takeover-hold-end", module, cmdq,
				     dump_count, 4);
	DISPERR("M6 DSI takeover_hold[%s]: end hold_ms=%u PMaster=%d force=%d jiffies=%lu\n",
		safe_where, M6_LK_HANDOFF_TAKEOVER_HOLD_MS,
		atomic_read(&PMaster_enable), dsi_force_config, jiffies);
}

static void dsi_m6_dump_hs_video_marker(const char *tag, DISP_MODULE_ENUM module,
					void *cmdq);
static void dsi_m6_dump_hs_video_edge_marker(const char *tag,
					     DISP_MODULE_ENUM module,
					     void *cmdq);
static void dsi_m6_hs_video_after_1vsync_work(struct work_struct *work);
static void dsi_m6_hs_video_after_500ms_work(struct work_struct *work);
static void dsi_m6_hs_video_edge_window_work(struct work_struct *work);

static DECLARE_DELAYED_WORK(dsi_m6_hs_video_after_1vsync_work_item,
			    dsi_m6_hs_video_after_1vsync_work);
static DECLARE_DELAYED_WORK(dsi_m6_hs_video_after_500ms_work_item,
			    dsi_m6_hs_video_after_500ms_work);
static DECLARE_DELAYED_WORK(dsi_m6_hs_video_edge_window_work_item,
			    dsi_m6_hs_video_edge_window_work);

static void dsi_m6_sram_scanout_edge(const char *tag, DISP_MODULE_ENUM module)
{
	static unsigned int count;
	const char *safe_tag = tag ? tag : "null";
	unsigned int n;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL || count >= 32)
		return;

	n = ++count;
#ifndef CONFIG_FPGA_EARLY_PORTING
	aee_sram_printk("M6X%02u %s V=%x/%x M=%x/%x/%x R=%x %u/%u %u/%u D=%x/%x/%x/%x VM=%x/%x L=%x/%x P=%x/%x\n",
		n, safe_tag,
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF),
		DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON),
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x134),
		INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008),
		INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x058));
#else
	aee_sram_printk("M6X%02u %s V=%x/%x M=%x/%x/%x R=%x %u/%u %u/%u D=%x/%x/%x/%x VM=%x/%x\n",
		n, safe_tag,
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF),
		DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON),
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x134));
#endif
}

static bool dsi_m6_should_dump_debug_mux(const char *tag)
{
	if (!tag)
		return false;

	return !strcmp(tag, "ddp-edge-0ms") ||
	       !strcmp(tag, "edge-8ms") ||
	       !strcmp(tag, "edge-33ms");
}

static void dsi_m6_dump_vm_payload_marker(const char *tag,
					  DISP_MODULE_ENUM module,
					  void *cmdq)
{
	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL)
		return;

	DISPERR("M6 DSI vm_payload[%s]: cmdq=%p start=0x%x int=0x%x mode=0x%x txrx=0x%x ps=0x%x mem=0x%x frm=0x%x h=0x%x/0x%x/0x%x/0x%x hstx=0x%x vm=0x%x data=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x st=0x%x/0x%x/0x%x cksm=0x%x dbg=0x%x\n",
		tag, cmdq,
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c),
		INREG32(DDP_REG_BASE_DSI0 + 0x090),
		INREG32(DDP_REG_BASE_DSI0 + 0x094),
		INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x05c),
		INREG32(DDP_REG_BASE_DSI0 + 0x064),
		INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x134),
		INREG32(DDP_REG_BASE_DSI0 + 0x138),
		INREG32(DDP_REG_BASE_DSI0 + 0x13c),
		INREG32(DDP_REG_BASE_DSI0 + 0x140),
		INREG32(DDP_REG_BASE_DSI0 + 0x180),
		INREG32(DDP_REG_BASE_DSI0 + 0x184),
		INREG32(DDP_REG_BASE_DSI0 + 0x188),
		INREG32(DDP_REG_BASE_DSI0 + 0x18c),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x168),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x144),
		INREG32(DDP_REG_BASE_DSI0 + 0x170));
}

static void dsi_m6_dump_debug_mux_sweep_direct(const char *tag,
					       unsigned int sweep_id)
{
	uint32_t orig;
	uint32_t restored;
	unsigned int sel;

	if (DSI_REG[0] == NULL)
		return;

	orig = INREG32(&DSI_REG[0]->DSI_DEBUG_SEL);
	for (sel = 0; sel < 32; sel++) {
		uint32_t debug_sel = (orig & ~0x1f) | sel;
		uint32_t state6;
		uint32_t state7;
		uint32_t state8;
		uint32_t state9;

		DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_DEBUG_SEL, debug_sel);
		udelay(1);
		state6 = INREG32(DDP_REG_BASE_DSI0 + 0x160);
		state7 = INREG32(DDP_REG_BASE_DSI0 + 0x164);
		state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
		state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);
		DISPERR("M6 DSI dbg_mux[%s]#%u sel=0x%x orig=0x%x now=0x%x st=0x%x/0x%x/0x%x/0x%x cksm=0x%x int=0x%x vm=0x%x word=%u line=%u\n",
			tag, sweep_id, sel, orig,
			INREG32(&DSI_REG[0]->DSI_DEBUG_SEL),
			state6, state7, state8, state9,
			INREG32(DDP_REG_BASE_DSI0 + 0x144),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c),
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			dsi_m6_field(state8, 0, 14),
			dsi_m6_field(state9, 0, 22));
	}

	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_DEBUG_SEL, orig);
	restored = INREG32(&DSI_REG[0]->DSI_DEBUG_SEL);
	DISPERR("M6 DSI dbg_mux[%s]#%u restore orig=0x%x now=0x%x\n",
		tag, sweep_id, orig, restored);
}

static void dsi_m6_dump_debug_mux_sweep(const char *tag,
					DISP_MODULE_ENUM module)
{
	static unsigned int sweep_count;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL ||
	    !dsi_m6_should_dump_debug_mux(tag) || sweep_count >= 3)
		return;

	sweep_count++;
	dsi_m6_dump_debug_mux_sweep_direct(tag, sweep_count);
}

static void dsi_m6_dump_mipitx_debug_mux_sweep_direct(const char *tag,
						      unsigned int sweep_id)
{
	uint32_t orig;
	uint32_t restored;
	unsigned int sel;

	if (DSI_REG[0] == NULL)
		return;

	orig = INREG32(MIPITX_BASE + 0x090);
	for (sel = 0; sel < 16; sel++) {
		uint32_t debug_sel = (orig & ~0x1f) | 0x10 | sel;
		uint32_t state8;
		uint32_t state9;

		mt_reg_sync_writel(debug_sel, MIPITX_BASE + 0x090);
		udelay(1);
		state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
		state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);
		DISPERR("M6 MIPITX dbg_mux[%s]#%u sel=0x%x orig=0x%x now=0x%x out=0x%x apb=0x%x lanes=0x%x/0x%x/0x%x/0x%x/0x%x top=0x%x pll=0x%x/0x%x/0x%x dsi=0x%x/0x%x st=0x%x/0x%x word=%u line=%u\n",
			tag, sweep_id, sel, orig,
			INREG32(MIPITX_BASE + 0x090),
			INREG32(MIPITX_BASE + 0x094),
			INREG32(MIPITX_BASE + 0x098),
			INREG32(MIPITX_BASE + 0x004),
			INREG32(MIPITX_BASE + 0x008),
			INREG32(MIPITX_BASE + 0x00c),
			INREG32(MIPITX_BASE + 0x010),
			INREG32(MIPITX_BASE + 0x014),
			INREG32(MIPITX_BASE + 0x040),
			INREG32(MIPITX_BASE + 0x050),
			INREG32(MIPITX_BASE + 0x058),
			INREG32(MIPITX_BASE + 0x068),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c),
			state8, state9,
			dsi_m6_field(state8, 0, 14),
			dsi_m6_field(state9, 0, 22));
	}

	mt_reg_sync_writel(orig, MIPITX_BASE + 0x090);
	restored = INREG32(MIPITX_BASE + 0x090);
	DISPERR("M6 MIPITX dbg_mux[%s]#%u restore orig=0x%x now=0x%x out=0x%x apb=0x%x\n",
		tag, sweep_id, orig, restored,
		INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098));
}

static void dsi_m6_dump_mipitx_debug_mux_sweep(const char *tag,
					       DISP_MODULE_ENUM module)
{
	static unsigned int sweep_count;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL ||
	    !dsi_m6_should_dump_debug_mux(tag) || sweep_count >= 3)
		return;

	sweep_count++;
	dsi_m6_dump_mipitx_debug_mux_sweep_direct(tag, sweep_count);
}

static unsigned int dsi_m6_bound_mux_samples(unsigned int samples)
{
	if (samples == 0)
		return 12;
	if (samples > 32)
		return 32;
	return samples;
}

static unsigned int dsi_m6_bound_mux_delay_us(unsigned int delay_us)
{
	if (delay_us == 0)
		return 1000;
	if (delay_us > 5000)
		return 5000;
	return delay_us;
}

static void dsi_m6_dump_debug_mux_stats_direct(const char *tag,
					       unsigned int sweep_id,
					       unsigned int samples,
					       unsigned int delay_us)
{
	uint32_t orig;
	uint32_t restored;
	unsigned int sel;
	unsigned int bounded_samples;
	unsigned int bounded_delay_us;

	if (DSI_REG[0] == NULL)
		return;

	bounded_samples = dsi_m6_bound_mux_samples(samples);
	bounded_delay_us = dsi_m6_bound_mux_delay_us(delay_us);
	orig = INREG32(&DSI_REG[0]->DSI_DEBUG_SEL);
	for (sel = 0; sel < 32; sel++) {
		uint32_t debug_sel = (orig & ~0x1f) | sel;
		unsigned int sample;
		unsigned int word_first = 0;
		unsigned int word_last = 0;
		unsigned int word_min = 0;
		unsigned int word_max = 0;
		unsigned int word_or = 0;
		unsigned int word_and = 0;
		unsigned int word_xor = 0;
		unsigned int word_prev = 0;
		unsigned int word_changes = 0;
		unsigned int line_first = 0;
		unsigned int line_last = 0;
		unsigned int line_min = 0;
		unsigned int line_max = 0;
		unsigned int line_prev = 0;
		unsigned int line_changes = 0;
		uint32_t state8_first = 0;
		uint32_t state8_last = 0;
		uint32_t state9_first = 0;
		uint32_t state9_last = 0;

		DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_DEBUG_SEL, debug_sel);
		udelay(1);
		for (sample = 0; sample < bounded_samples; sample++) {
			uint32_t state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
			uint32_t state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);
			unsigned int word = dsi_m6_field(state8, 0, 14);
			unsigned int line = dsi_m6_field(state9, 0, 22);

			if (sample == 0) {
				word_first = word_last = word_min = word_max = word;
				word_or = word_and = word_xor = word;
				word_prev = word;
				line_first = line_last = line_min = line_max = line;
				line_prev = line;
				state8_first = state8_last = state8;
				state9_first = state9_last = state9;
			} else {
				if (word != word_prev)
					word_changes++;
				if (line != line_prev)
					line_changes++;
				if (word < word_min)
					word_min = word;
				if (word > word_max)
					word_max = word;
				if (line < line_min)
					line_min = line;
				if (line > line_max)
					line_max = line;
				word_or |= word;
				word_and &= word;
				word_xor ^= word;
				word_last = word;
				word_prev = word;
				line_last = line;
				line_prev = line;
				state8_last = state8;
				state9_last = state9;
			}
			if (sample + 1 < bounded_samples)
				udelay(bounded_delay_us);
		}

		DISPERR("M6 DSI mux_stats[%s]#%u sel=0x%x n=%u delay_us=%u orig=0x%x now=0x%x word=%u/%u minmax=%u/%u or=0x%x and=0x%x xor=0x%x changes=%u line=%u/%u minmax=%u/%u changes=%u st8=0x%x/0x%x st9=0x%x/0x%x int=0x%x vm=0x%x cksm=0x%x\n",
			tag, sweep_id, sel, bounded_samples, bounded_delay_us,
			orig, INREG32(&DSI_REG[0]->DSI_DEBUG_SEL),
			word_first, word_last, word_min, word_max, word_or,
			word_and, word_xor, word_changes, line_first, line_last,
			line_min, line_max, line_changes, state8_first, state8_last,
			state9_first, state9_last, INREG32(DDP_REG_BASE_DSI0 + 0x00c),
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x144));
	}

	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_DEBUG_SEL, orig);
	restored = INREG32(&DSI_REG[0]->DSI_DEBUG_SEL);
	DISPERR("M6 DSI mux_stats[%s]#%u restore orig=0x%x now=0x%x n=%u delay_us=%u\n",
		tag, sweep_id, orig, restored, bounded_samples, bounded_delay_us);
}

static void dsi_m6_dump_mipitx_debug_mux_stats_direct(const char *tag,
						      unsigned int sweep_id,
						      unsigned int samples,
						      unsigned int delay_us)
{
	uint32_t orig;
	uint32_t restored;
	unsigned int sel;
	unsigned int bounded_samples;
	unsigned int bounded_delay_us;
	int cache_slot;

	if (DSI_REG[0] == NULL)
		return;

	bounded_samples = dsi_m6_bound_mux_samples(samples);
	bounded_delay_us = dsi_m6_bound_mux_delay_us(delay_us);
	cache_slot = dsi_m6_lkgold_mux_cache_slot(tag);
	orig = INREG32(MIPITX_BASE + 0x090);
	for (sel = 0; sel < 16; sel++) {
		uint32_t debug_sel = (orig & ~0x1f) | 0x10 | sel;
		unsigned int sample;
		uint32_t out_first = 0;
		uint32_t out_last = 0;
		uint32_t out_min = 0;
		uint32_t out_max = 0;
		uint32_t out_or = 0;
		uint32_t out_and = 0;
		uint32_t out_xor = 0;
		uint32_t out_prev = 0;
		unsigned int out_changes = 0;
		uint32_t apb_first = 0;
		uint32_t apb_last = 0;
		uint32_t apb_or = 0;
		uint32_t apb_and = 0;
		uint32_t apb_xor = 0;
		uint32_t apb_prev = 0;
		unsigned int apb_changes = 0;
		unsigned int word_first = 0;
		unsigned int word_last = 0;
		unsigned int line_first = 0;
		unsigned int line_last = 0;

		mt_reg_sync_writel(debug_sel, MIPITX_BASE + 0x090);
		udelay(1);
		for (sample = 0; sample < bounded_samples; sample++) {
			uint32_t out = INREG32(MIPITX_BASE + 0x094);
			uint32_t apb = INREG32(MIPITX_BASE + 0x098);
			uint32_t state8 = INREG32(DDP_REG_BASE_DSI0 + 0x168);
			uint32_t state9 = INREG32(DDP_REG_BASE_DSI0 + 0x16c);
			unsigned int word = dsi_m6_field(state8, 0, 14);
			unsigned int line = dsi_m6_field(state9, 0, 22);

			if (sample == 0) {
				out_first = out_last = out_min = out_max = out;
				out_or = out_and = out_xor = out;
				out_prev = out;
				apb_first = apb_last = apb;
				apb_or = apb_and = apb_xor = apb;
				apb_prev = apb;
				word_first = word_last = word;
				line_first = line_last = line;
			} else {
				if (out != out_prev)
					out_changes++;
				if (apb != apb_prev)
					apb_changes++;
				if (out < out_min)
					out_min = out;
				if (out > out_max)
					out_max = out;
				out_or |= out;
				out_and &= out;
				out_xor ^= out;
				out_last = out;
				out_prev = out;
				apb_or |= apb;
				apb_and &= apb;
				apb_xor ^= apb;
				apb_last = apb;
				apb_prev = apb;
				word_last = word;
				line_last = line;
			}
			if (sample + 1 < bounded_samples)
				udelay(bounded_delay_us);
		}

		if (cache_slot >= 0) {
			struct m6_lkgold_mipitx_mux_cache *entry =
				&m6_lkgold_mipitx_mux_cache[cache_slot][sel];

			memset(entry, 0, sizeof(*entry));
			entry->valid = true;
			snprintf(entry->tag, sizeof(entry->tag), "%s", tag);
			entry->sweep_id = sweep_id;
			entry->sel = sel;
			entry->samples = bounded_samples;
			entry->delay_us = bounded_delay_us;
			entry->orig = orig;
			entry->now = INREG32(MIPITX_BASE + 0x090);
			entry->out_first = out_first;
			entry->out_last = out_last;
			entry->out_min = out_min;
			entry->out_max = out_max;
			entry->out_or = out_or;
			entry->out_and = out_and;
			entry->out_xor = out_xor;
			entry->out_changes = out_changes;
			entry->apb_first = apb_first;
			entry->apb_last = apb_last;
			entry->apb_or = apb_or;
			entry->apb_and = apb_and;
			entry->apb_xor = apb_xor;
			entry->apb_changes = apb_changes;
			entry->word_first = word_first;
			entry->word_last = word_last;
			entry->line_first = line_first;
			entry->line_last = line_last;
			entry->lanes[0] = INREG32(MIPITX_BASE + 0x004);
			entry->lanes[1] = INREG32(MIPITX_BASE + 0x008);
			entry->lanes[2] = INREG32(MIPITX_BASE + 0x00c);
			entry->lanes[3] = INREG32(MIPITX_BASE + 0x010);
			entry->lanes[4] = INREG32(MIPITX_BASE + 0x014);
			entry->pll[0] = INREG32(MIPITX_BASE + 0x050);
			entry->pll[1] = INREG32(MIPITX_BASE + 0x058);
			entry->pll[2] = INREG32(MIPITX_BASE + 0x068);
		}

		DISPERR("M6 MIPITX mux_stats[%s]#%u sel=0x%x n=%u delay_us=%u orig=0x%x now=0x%x out=0x%x/0x%x minmax=0x%x/0x%x or=0x%x and=0x%x xor=0x%x changes=%u apb=0x%x/0x%x or=0x%x and=0x%x xor=0x%x changes=%u word=%u/%u line=%u/%u lanes=0x%x/0x%x/0x%x/0x%x/0x%x pll=0x%x/0x%x/0x%x\n",
			tag, sweep_id, sel, bounded_samples, bounded_delay_us,
			orig, INREG32(MIPITX_BASE + 0x090),
			out_first, out_last, out_min, out_max, out_or, out_and,
			out_xor, out_changes, apb_first, apb_last, apb_or,
			apb_and, apb_xor, apb_changes, word_first, word_last,
			line_first, line_last, INREG32(MIPITX_BASE + 0x004),
			INREG32(MIPITX_BASE + 0x008), INREG32(MIPITX_BASE + 0x00c),
			INREG32(MIPITX_BASE + 0x010), INREG32(MIPITX_BASE + 0x014),
			INREG32(MIPITX_BASE + 0x050), INREG32(MIPITX_BASE + 0x058),
			INREG32(MIPITX_BASE + 0x068));
	}

	mt_reg_sync_writel(orig, MIPITX_BASE + 0x090);
	restored = INREG32(MIPITX_BASE + 0x090);
	DISPERR("M6 MIPITX mux_stats[%s]#%u restore orig=0x%x now=0x%x out=0x%x apb=0x%x n=%u delay_us=%u\n",
		tag, sweep_id, orig, restored, INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098), bounded_samples, bounded_delay_us);
}

void dsi_m6_lkgold_muxstats_dump(void)
{
	static const char *const names[M6_LKGOLD_MUX_TAGS] = {
		"lkgold-pre-init",
		"lkgold-post-config",
		"lkgold-post-start",
	};
	unsigned int slot;
	unsigned int sel;

	DISPERR("M6 lkgold_muxstats_cache: begin live_dbg=0x%x out=0x%x apb=0x%x start=0x%x mode=0x%x txrx=0x%x lccon=0x%x\n",
		INREG32(MIPITX_BASE + 0x090), INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098), INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x104));
	for (slot = 0; slot < M6_LKGOLD_MUX_TAGS; slot++) {
		unsigned int valid = 0;

		for (sel = 0; sel < M6_LKGOLD_MUX_SELS; sel++) {
			const struct m6_lkgold_mipitx_mux_cache *entry =
				&m6_lkgold_mipitx_mux_cache[slot][sel];

			if (!entry->valid)
				continue;
			valid++;
			DISPERR("M6 lkgold_muxstats_cache[%s]#%u sel=0x%x n=%u delay_us=%u orig=0x%x now=0x%x out=0x%x/0x%x minmax=0x%x/0x%x or=0x%x and=0x%x xor=0x%x changes=%u apb=0x%x/0x%x or=0x%x and=0x%x xor=0x%x changes=%u word=%u/%u line=%u/%u lanes=0x%x/0x%x/0x%x/0x%x/0x%x pll=0x%x/0x%x/0x%x\n",
				entry->tag, entry->sweep_id, entry->sel,
				entry->samples, entry->delay_us, entry->orig,
				entry->now, entry->out_first, entry->out_last,
				entry->out_min, entry->out_max, entry->out_or,
				entry->out_and, entry->out_xor, entry->out_changes,
				entry->apb_first, entry->apb_last, entry->apb_or,
				entry->apb_and, entry->apb_xor, entry->apb_changes,
				entry->word_first, entry->word_last,
				entry->line_first, entry->line_last,
				entry->lanes[0], entry->lanes[1], entry->lanes[2],
				entry->lanes[3], entry->lanes[4], entry->pll[0],
				entry->pll[1], entry->pll[2]);
		}
		if (!valid)
			DISPERR("M6 lkgold_muxstats_cache[%s]: missing\n",
				names[slot]);
	}
	DISPERR("M6 lkgold_muxstats_cache: end\n");
}

void dsi_m6_debug_mux_sweep(const char *tag)
{
	static unsigned int manual_count;
	const char *safe_tag = tag ? tag : "manual";
	unsigned int n;

	if (DSI_REG[0] == NULL) {
		DISPERR("M6 DSI debug_mux[%s]: DSI_REG0 missing\n", safe_tag);
		return;
	}

	n = ++manual_count;
	DISPERR("M6 DSI debug_mux[%s]#%u: begin dsi_debug_sel=0x%x mipitx_dbg=0x%x out=0x%x apb=0x%x\n",
		safe_tag, n, INREG32(&DSI_REG[0]->DSI_DEBUG_SEL),
		INREG32(MIPITX_BASE + 0x090), INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098));
	dsi_m6_dump_snapshot("debugmux-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_debug_mux_sweep_direct(safe_tag, n);
	dsi_m6_dump_mipitx_debug_mux_sweep_direct(safe_tag, n);
	dsi_m6_dump_snapshot("debugmux-after", DISP_MODULE_DSI0, NULL);
	DISPERR("M6 DSI debug_mux[%s]#%u: end dsi_debug_sel=0x%x mipitx_dbg=0x%x out=0x%x apb=0x%x\n",
		safe_tag, n, INREG32(&DSI_REG[0]->DSI_DEBUG_SEL),
		INREG32(MIPITX_BASE + 0x090), INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098));
}

void dsi_m6_debug_mux_stats(const char *tag, unsigned int samples,
			    unsigned int delay_us)
{
	static unsigned int manual_count;
	const char *safe_tag = tag ? tag : "manual";
	unsigned int n;
	unsigned int bounded_samples = dsi_m6_bound_mux_samples(samples);
	unsigned int bounded_delay_us = dsi_m6_bound_mux_delay_us(delay_us);

	if (DSI_REG[0] == NULL) {
		DISPERR("M6 DSI mux_stats[%s]: DSI_REG0 missing\n", safe_tag);
		return;
	}

	n = ++manual_count;
	DISPERR("M6 DSI mux_stats[%s]#%u: begin samples=%u delay_us=%u dsi_debug_sel=0x%x mipitx_dbg=0x%x out=0x%x apb=0x%x\n",
		safe_tag, n, bounded_samples, bounded_delay_us,
		INREG32(&DSI_REG[0]->DSI_DEBUG_SEL), INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094), INREG32(MIPITX_BASE + 0x098));
	dsi_m6_dump_snapshot("muxstats-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_debug_mux_stats_direct(safe_tag, n, bounded_samples,
					   bounded_delay_us);
	dsi_m6_dump_mipitx_debug_mux_stats_direct(safe_tag, n, bounded_samples,
						  bounded_delay_us);
	dsi_m6_dump_snapshot("muxstats-after", DISP_MODULE_DSI0, NULL);
	DISPERR("M6 DSI mux_stats[%s]#%u: end dsi_debug_sel=0x%x mipitx_dbg=0x%x out=0x%x apb=0x%x\n",
		safe_tag, n, INREG32(&DSI_REG[0]->DSI_DEBUG_SEL),
		INREG32(MIPITX_BASE + 0x090), INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098));
}

static void dsi_m6_dump_hs_video_marker(const char *tag, DISP_MODULE_ENUM module,
					 void *cmdq)
{
	LCM_DSI_PARAMS *p = &_dsi_context[0].dsi_params;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL)
		return;

	dsi_m6_sram_video_snapshot(tag, module);
	DISPERR("M6 DSI HS video[%s]: cmdq=%p mode=%u switch=%u pll=%u lanes=%u packet=%u ps=%u size=%ux%u v=%u/%u/%u/%u h=%u/%u/%u/%u start=0x%x intsta=0x%x txrx=0x%x psctrl=0x%x vm=0x%x\n",
		tag, cmdq, p->mode, p->switch_mode, p->PLL_CLOCK,
		p->LANE_NUM, p->packet_size, p->PS,
		_dsi_context[0].lcm_width, _dsi_context[0].lcm_height,
		p->vertical_sync_active, p->vertical_backporch,
		p->vertical_frontporch, p->vertical_active_line,
		p->horizontal_sync_active, p->horizontal_backporch,
		p->horizontal_frontporch, p->horizontal_active_pixel,
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130));

	dsi_m6_dump_snapshot(tag, module, cmdq);
}

static void dsi_m6_dump_hs_video_edge_marker(const char *tag,
					     DISP_MODULE_ENUM module,
					     void *cmdq)
{
	const char *safe_tag = tag ? tag : "edge";

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL)
		return;

	dsi_m6_sram_video_snapshot(safe_tag, module);
	dsi_m6_sram_scanout_edge(safe_tag, module);
	DISPERR("M6 DSI HS edge[%s]: cmdq=%p route=0x%x/0x%x mutex=0x%x/0x%x/0x%x rdma=0x%x in=%u/%u out=%u/%u dsi=0x%x/0x%x state=0x%x/0x%x vm=0x%x/0x%x\n",
		safe_tag, cmdq,
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD),
		DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF),
		DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON),
		DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT),
		DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x134));
	if (dsi_m6_should_dump_debug_mux(safe_tag))
		dsi_m6_dump_vm_payload_marker(safe_tag, module, cmdq);
	dsi_m6_dump_debug_mux_sweep(safe_tag, module);
	dsi_m6_dump_mipitx_debug_mux_sweep(safe_tag, module);
}

static void dsi_m6_dump_hs_video_limited(const char *tag, DISP_MODULE_ENUM module,
					 void *cmdq, unsigned int *count,
					 unsigned int limit)
{
	if (*count >= limit)
		return;

	(*count)++;
	dsi_m6_dump_hs_video_marker(tag, module, cmdq);
}

static void dsi_m6_hs_video_after_1vsync_work(struct work_struct *work)
{
	dsi_m6_dump_hs_video_marker("after-1vsync", DISP_MODULE_DSI0, NULL);
}

static void dsi_m6_hs_video_after_500ms_work(struct work_struct *work)
{
	dsi_m6_dump_hs_video_marker("after-500ms", DISP_MODULE_DSI0, NULL);
}

static void dsi_m6_hs_video_edge_window_work(struct work_struct *work)
{
	msleep(1);
	dsi_m6_dump_hs_video_edge_marker("edge-1ms", DISP_MODULE_DSI0, NULL);
	msleep(1);
	dsi_m6_dump_hs_video_edge_marker("edge-2ms", DISP_MODULE_DSI0, NULL);
	msleep(2);
	dsi_m6_dump_hs_video_edge_marker("edge-4ms", DISP_MODULE_DSI0, NULL);
	msleep(4);
	dsi_m6_dump_hs_video_edge_marker("edge-8ms", DISP_MODULE_DSI0, NULL);
	msleep(8);
	dsi_m6_dump_hs_video_edge_marker("edge-16ms", DISP_MODULE_DSI0, NULL);
	msleep(17);
	dsi_m6_dump_hs_video_edge_marker("edge-33ms", DISP_MODULE_DSI0, NULL);
}

static void dsi_m6_schedule_hs_video_delayed(void)
{
	static unsigned int schedule_count;

	if (schedule_count >= 4)
		return;

	schedule_count++;
	if (schedule_count <= 2) {
		dsi_m6_dump_hs_video_edge_marker("edge-0ms", DISP_MODULE_DSI0,
						 NULL);
		schedule_delayed_work(&dsi_m6_hs_video_edge_window_work_item,
				      0);
	}
	schedule_delayed_work(&dsi_m6_hs_video_after_1vsync_work_item,
			      msecs_to_jiffies(17));
	schedule_delayed_work(&dsi_m6_hs_video_after_500ms_work_item,
			      msecs_to_jiffies(500));
}

static void dsi_m6_schedule_ddp_hs_video_edge(DISP_MODULE_ENUM module,
					      void *cmdq)
{
	static unsigned int schedule_count;

	if (module != DISP_MODULE_DSI0 || DSI_REG[0] == NULL ||
	    schedule_count >= 2)
		return;

	schedule_count++;
	dsi_m6_dump_hs_video_edge_marker("ddp-edge-0ms", module, cmdq);
	schedule_delayed_work(&dsi_m6_hs_video_edge_window_work_item, 0);
}

void dsi_m6_dump_live(const char *tag)
{
	const char *safe_tag = tag ? tag : "manual";

	DISPERR("M6 DISPLAY truth[%s][dsi-host]: begin\n", safe_tag);
	dsi_m6_dump_snapshot(safe_tag, DISP_MODULE_DSI0, NULL);
	DISPERR("M6 DISPLAY truth[%s][dsi-host]: end\n", safe_tag);
}

void dsi_m6_dump_phy_truth(const char *tag)
{
	const char *safe_tag = tag ? tag : "manual";
	LCM_DSI_PARAMS *p = &_dsi_context[0].dsi_params;

	if (!DSI_REG[0]) {
		DISPERR("M6 DSI phy_truth[%s]: DSI_REG0 missing\n", safe_tag);
		return;
	}

	DISPERR("M6 DSI phy_truth[%s]: begin power=%d ulps=%u dsi_cur_mode=%d size=%ux%u\n",
		safe_tag, s_isDsiPowerOn, is_mipi_enterulps(),
		dsi_currect_mode, _dsi_context[0].lcm_width,
		_dsi_context[0].lcm_height);
	DISPERR("M6 DSI phy_truth[%s]: lcm mode=%u switch=%u lanes=%u datafmt color/trans/pad/fmt=%u/%u/%u/%u ps=%u packet=%u word=%u pll=%u/%u/%u dsi_clock=%u ssc=%u/%u cont=%u noncont=%u/%u\n",
		safe_tag, p->mode, p->switch_mode, p->LANE_NUM,
		p->data_format.color_order, p->data_format.trans_seq,
		p->data_format.padding, p->data_format.format, p->PS,
		p->packet_size, p->word_count, p->PLL_CLOCK,
		p->PLL_CK_CMD, p->PLL_CK_VDO, p->dsi_clock,
		p->ssc_disable, p->ssc_range, p->cont_clock,
		p->noncont_clock, p->noncont_clock_period);
	DISPERR("M6 DSI phy_truth[%s]: lcm timing v=%u/%u/%u/%u vfp_lp=%u h=%u/%u/%u/%u bllp=%u null_pkt=%u mix=%u/%u lfr=%u/%u/%u/%u\n",
		safe_tag, p->vertical_sync_active, p->vertical_backporch,
		p->vertical_frontporch, p->vertical_active_line,
		p->vertical_frontporch_for_low_power,
		p->horizontal_sync_active, p->horizontal_backporch,
		p->horizontal_frontporch, p->horizontal_active_pixel,
		p->horizontal_bllp, p->null_packet_en, p->mixmode_enable,
		p->mixmode_mipi_clock, p->lfr_enable, p->lfr_mode,
		p->lfr_type, p->lfr_skip_num);
	DISPERR("M6 DSI phy_truth[%s]: lcm phy hs_trail/zero/prpr/lpx=%u/%u/%u/%u ta_sack/get/sure/go=%u/%u/%u/%u clk_trail/zero/lpx_wait/cont_det=%u/%u/%u/%u clk_hs_prpr/post/da_exit/clk_exit=%u/%u/%u/%u\n",
		safe_tag, p->HS_TRAIL, p->HS_ZERO, p->HS_PRPR, p->LPX,
		p->TA_SACK, p->TA_GET, p->TA_SURE, p->TA_GO,
		p->CLK_TRAIL, p->CLK_ZERO, p->LPX_WAIT, p->CONT_DET,
		p->CLK_HS_PRPR, p->CLK_HS_POST, p->DA_HS_EXIT,
		p->CLK_HS_EXIT);
	DISPERR("M6 DSI phy_truth[%s]: lcm lane_swap_en=%u port0=%u/%u/%u/%u/%u/%u port1=%u/%u/%u/%u/%u/%u te int=%u/%u ext=%u/%u edge=%u eint_disable=%u ufoe/dsc=%u/%u\n",
		safe_tag, p->lane_swap_en,
		p->lane_swap[0][0], p->lane_swap[0][1],
		p->lane_swap[0][2], p->lane_swap[0][3],
		p->lane_swap[0][4], p->lane_swap[0][5],
		p->lane_swap[1][0], p->lane_swap[1][1],
		p->lane_swap[1][2], p->lane_swap[1][3],
		p->lane_swap[1][4], p->lane_swap[1][5],
		p->lcm_int_te_monitor, p->lcm_int_te_period,
		p->lcm_ext_te_monitor, p->lcm_ext_te_enable,
		p->ext_te_edge, p->eint_disable, p->ufoe_enable,
		p->dsc_enable);
	dsi_m6_dump_snapshot(safe_tag, DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_dsi_block(safe_tag);
#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_mipitx_block(safe_tag);
#endif
	DISPERR("M6 DSI phy_truth[%s]: end\n", safe_tag);
}

void dsi_m6_dump_takeover(const char *tag)
{
	const char *safe_tag = tag ? tag : "takeover";

	dsi_m6_sram_snapshot(safe_tag, DISP_MODULE_DSI0);
	dsi_m6_sram_video_snapshot(safe_tag, DISP_MODULE_DSI0);
	DISPERR("M6 DSI takeover[%s]: begin\n", safe_tag);
	dsi_m6_dump_snapshot(safe_tag, DISP_MODULE_DSI0, NULL);
#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_mipitx_block(safe_tag);
#endif
	DISPERR("M6 DSI takeover[%s]: end\n", safe_tag);
}

void dsi_m6_dump_hs_window(const char *tag, unsigned int hold_ms)
{
	static const unsigned int marks[] = {
		0, 17, 34, 51, 68, 85, 102, 119, 136,
		250, 500, 1000, 2000, 5000, 10000
	};
	char marker[64];
	const char *safe_tag = tag ? tag : "manual";
	unsigned int bounded = hold_ms;
	unsigned int elapsed = 0;
	unsigned int idx;

	if (bounded == 0)
		bounded = 1000;
	if (bounded > 10000)
		bounded = 10000;

	DISPERR("M6 DSI hs_window[%s]: begin hold_ms=%u jiffies=%lu\n",
		safe_tag, bounded, jiffies);
	for (idx = 0; idx < ARRAY_SIZE(marks); idx++) {
		if (marks[idx] > bounded)
			break;
		if (marks[idx] > elapsed)
			msleep(marks[idx] - elapsed);
		elapsed = marks[idx];
		snprintf(marker, sizeof(marker), "hs-window-%s-%ums",
			 safe_tag, marks[idx]);
		dsi_m6_dump_snapshot(marker, DISP_MODULE_DSI0, NULL);
	}
	if (elapsed < bounded) {
		msleep(bounded - elapsed);
		snprintf(marker, sizeof(marker), "hs-window-%s-%ums",
			 safe_tag, bounded);
		dsi_m6_dump_snapshot(marker, DISP_MODULE_DSI0, NULL);
	}
	DISPERR("M6 DSI hs_window[%s]: end hold_ms=%u jiffies=%lu\n",
		safe_tag, bounded, jiffies);
}

static unsigned int dsi_m6_bound_hold_ms(unsigned int hold_ms);
static void dsi_m6_dump_probe_mux_sweep(const char *tag);
static void dsi_m6_dump_probe_mux_stats(const char *tag, unsigned int samples,
					unsigned int delay_us);
static unsigned int dsi_m6_txrx_ctrl_raw(void);
static unsigned int dsi_m6_phy_lccon_raw(void);

struct dsi_m6_mipitx_probe_field {
	const char *name;
	unsigned int offset;
	unsigned int mask;
	unsigned int shift;
	unsigned int max;
};

struct dsi_m6_mipitx_lane_group {
	const char *name;
	unsigned int mask;
	unsigned int shift;
	unsigned int max;
};

#define M6_MIPITX_PHY_SEL_MASK 0x00777777U
#define M6_MIPITX_PHY_SEL_FIELD(_raw, _idx) (((_raw) >> ((_idx) * 4)) & 0x7U)

static const struct dsi_m6_mipitx_probe_field dsi_m6_mipitx_probe_fields[] = {
	{ "lptx_clmp", 0x000, 1U << 11, 11, 1 },
	{ "c_b1", 0x004, 1U << 1, 1, 1 },
	{ "d0_b1", 0x008, 1U << 1, 1, 1 },
	{ "d1_b1", 0x00c, 1U << 1, 1, 1 },
	{ "d2_b1", 0x010, 1U << 1, 1, 1 },
	{ "d3_b1", 0x014, 1U << 1, 1, 1 },
	{ "hs_bias", 0x040, 1U << 1, 1, 1 },
	{ "imp_en", 0x040, 1U << 2, 2, 1 },
	{ "imp", 0x040, 0xfU << 4, 4, 15 },
	{ "aio", 0x040, 0x7U << 8, 8, 7 },
	{ "pad_low", 0x040, 1U << 11, 11, 1 },
};

static const unsigned int dsi_m6_mipitx_lane_offsets[] = {
	0x004, 0x008, 0x00c, 0x010, 0x014,
};

static const struct dsi_m6_mipitx_lane_group dsi_m6_mipitx_lane_groups[] = {
	{ "rt", 0xfU << 8, 8, 15 },
	{ "lptx", 0x7U << 2, 2, 7 },
	{ "lpcd", 0x3U << 5, 5, 3 },
};

static const struct dsi_m6_mipitx_probe_field *
dsi_m6_mipitx_find_probe_field(const char *name)
{
	unsigned int i;

	if (!name)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_probe_fields); i++) {
		if (!strcmp(name, dsi_m6_mipitx_probe_fields[i].name))
			return &dsi_m6_mipitx_probe_fields[i];
	}

	return NULL;
}

static const struct dsi_m6_mipitx_lane_group *
dsi_m6_mipitx_find_lane_group(const char *name)
{
	unsigned int i;

	if (!name)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_groups); i++) {
		if (!strcmp(name, dsi_m6_mipitx_lane_groups[i].name))
			return &dsi_m6_mipitx_lane_groups[i];
	}

	return NULL;
}

static bool dsi_m6_phy_sel_valid(unsigned int value)
{
	unsigned int used = 0;
	unsigned int lane;
	unsigned int i;

	if (value & ~M6_MIPITX_PHY_SEL_MASK)
		return false;

	for (i = 0; i < 6; i++) {
		lane = M6_MIPITX_PHY_SEL_FIELD(value, i);
		if (lane > 5)
			return false;
		if (i < 5) {
			if (used & (1U << lane))
				return false;
			used |= 1U << lane;
		}
	}

	return true;
}

static void dsi_m6_log_phy_sel(const char *tag, unsigned int raw)
{
	DISPERR("M6 DSI phy_sel[%s]: raw=0x%x d0/d1/d2/d3/c/lprx=%u/%u/%u/%u/%u/%u valid=%u\n",
		tag ? tag : "unknown", raw,
		M6_MIPITX_PHY_SEL_FIELD(raw, 0),
		M6_MIPITX_PHY_SEL_FIELD(raw, 1),
		M6_MIPITX_PHY_SEL_FIELD(raw, 2),
		M6_MIPITX_PHY_SEL_FIELD(raw, 3),
		M6_MIPITX_PHY_SEL_FIELD(raw, 4),
		M6_MIPITX_PHY_SEL_FIELD(raw, 5),
		dsi_m6_phy_sel_valid(raw) ? 1 : 0);
}

static unsigned int dsi_m6_bound_pad_samples(unsigned int samples)
{
	if (samples == 0)
		return 6;
	if (samples > 20)
		return 20;
	return samples;
}

static unsigned int dsi_m6_bound_pad_delay_ms(unsigned int delay_ms)
{
	if (delay_ms == 0)
		return 100;
	if (delay_ms > 1000)
		return 1000;
	return delay_ms;
}

static void dsi_m6_dump_mipitx_pad_sample(const char *tag,
					  unsigned int seq,
					  unsigned int sample,
					  unsigned int samples,
					  unsigned int delay_ms)
{
	DISPERR("M6 DSI mipitx_pad[%s]#%u sample=%u/%u delay_ms=%u con=0x%x c=0x%x d0=0x%x d1=0x%x d2=0x%x d3=0x%x top=0x%x bg=0x%x pll=0x%x/0x%x/0x%x pll_top=0x%x pwr=0x%x rgs=0x%x gpi=0x%x pull=0x%x phy_sel=0x%x sw=0x%x/0x%x/0x%x dbg=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x st8=0x%x st9=0x%x int=0x%x vm=0x%x\n",
		tag, seq, sample + 1, samples, delay_ms,
		INREG32(MIPITX_BASE + 0x000),
		INREG32(MIPITX_BASE + 0x004),
		INREG32(MIPITX_BASE + 0x008),
		INREG32(MIPITX_BASE + 0x00c),
		INREG32(MIPITX_BASE + 0x010),
		INREG32(MIPITX_BASE + 0x014),
		INREG32(MIPITX_BASE + 0x040),
		INREG32(MIPITX_BASE + 0x044),
		INREG32(MIPITX_BASE + 0x050),
		INREG32(MIPITX_BASE + 0x058),
		INREG32(MIPITX_BASE + 0x060),
		INREG32(MIPITX_BASE + 0x064),
		INREG32(MIPITX_BASE + 0x068),
		INREG32(MIPITX_BASE + 0x070),
		INREG32(MIPITX_BASE + 0x074),
		INREG32(MIPITX_BASE + 0x078),
		INREG32(MIPITX_BASE + 0x07c),
		INREG32(MIPITX_BASE + 0x080),
		INREG32(MIPITX_BASE + 0x084),
		INREG32(MIPITX_BASE + 0x088),
		INREG32(MIPITX_BASE + 0x090),
		INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098),
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw(),
		INREG32(DDP_REG_BASE_DSI0 + 0x168),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x130));
}

void dsi_m6_mipitx_pad_window(const char *tag, unsigned int samples,
			      unsigned int delay_ms)
{
	static unsigned int window_count;
	const char *safe_tag = tag ? tag : "manual";
	unsigned int bounded_samples = dsi_m6_bound_pad_samples(samples);
	unsigned int bounded_delay_ms = dsi_m6_bound_pad_delay_ms(delay_ms);
	unsigned int n;
	unsigned int sample;

	if (!DSI_REG[0])
		return;

	n = ++window_count;
	DISPERR("M6 DSI mipitx_pad[%s]#%u: begin samples=%u delay_ms=%u\n",
		safe_tag, n, bounded_samples, bounded_delay_ms);
	dsi_m6_dump_snapshot("mipitx-pad-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode(safe_tag);

	for (sample = 0; sample < bounded_samples; sample++) {
		dsi_m6_dump_mipitx_pad_sample(safe_tag, n, sample,
					      bounded_samples, bounded_delay_ms);
		if (sample + 1 < bounded_samples)
			msleep(bounded_delay_ms);
	}

	dsi_m6_dump_snapshot("mipitx-pad-after", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode(safe_tag);
	DISPERR("M6 DSI mipitx_pad[%s]#%u: end samples=%u delay_ms=%u\n",
		safe_tag, n, bounded_samples, bounded_delay_ms);
}

void dsi_m6_mipitx_pad_probe(const char *field_name, unsigned int value,
			     unsigned int hold_ms, unsigned int restore,
			     unsigned int sample_mux)
{
	char tag[64];
	const struct dsi_m6_mipitx_probe_field *field;
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old_raw;
	unsigned int new_raw;
	unsigned int after_raw;
	unsigned int restored_raw;

	if (!DSI_REG[0])
		return;

	field = dsi_m6_mipitx_find_probe_field(field_name);
	if (!field) {
		DISPERR("M6 DSI mipitx_pad_probe: unknown field=%s allowed=lptx_clmp,c_b1,d0_b1,d1_b1,d2_b1,d3_b1,hs_bias,imp_en,imp,aio,pad_low\n",
			field_name ? field_name : "null");
		return;
	}
	if (value > field->max) {
		DISPERR("M6 DSI mipitx_pad_probe: invalid field=%s value=%u max=%u\n",
			field->name, value, field->max);
		return;
	}

	old_raw = INREG32(MIPITX_BASE + field->offset);
	new_raw = (old_raw & ~field->mask) |
		  ((value << field->shift) & field->mask);
	DISPERR("M6 DSI mipitx_pad_probe: begin field=%s value=%u old=0x%x new=0x%x off=0x%x mask=0x%x shift=%u restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		field->name, value, old_raw, new_raw, field->offset,
		field->mask, field->shift, restore ? 1 : 0, sample_mux,
		bounded, dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("mipitx-pad-probe-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("mipitx-pad-probe-before");

	MIPITX_OUTREG32(MIPITX_BASE + field->offset, new_raw);
	udelay(1);
	after_raw = INREG32(MIPITX_BASE + field->offset);
	DISPERR("M6 DSI mipitx_pad_probe: after-set field=%s value=%u live=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
		field->name, value, after_raw, INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098), dsi_m6_txrx_ctrl_raw(),
		dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("mipitx-pad-probe-after-set", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("mipitx-pad-probe-after-set");

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "pad-%s-mux", field->name);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux == 2) {
		snprintf(tag, sizeof(tag), "pad-%s-muxstats", field->name);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	} else if (sample_mux >= 3) {
		snprintf(tag, sizeof(tag), "pad-%s-window", field->name);
		dsi_m6_mipitx_pad_window(tag, 4, 50);
	}

	snprintf(tag, sizeof(tag), "pad-%s-hold", field->name);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		MIPITX_OUTREG32(MIPITX_BASE + field->offset, old_raw);
		udelay(1);
		restored_raw = INREG32(MIPITX_BASE + field->offset);
		DISPERR("M6 DSI mipitx_pad_probe: after-restore field=%s old=0x%x live=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
			field->name, old_raw, restored_raw,
			INREG32(MIPITX_BASE + 0x094),
			INREG32(MIPITX_BASE + 0x098),
			dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
		dsi_m6_dump_snapshot("mipitx-pad-probe-after-restore",
				     DISP_MODULE_DSI0, NULL);
		dsi_m6_dump_mipitx_decode("mipitx-pad-probe-after-restore");
	}

	DISPERR("M6 DSI mipitx_pad_probe: end field=%s value=%u old=0x%x final=0x%x restore=%u mux=%u\n",
		field->name, value, old_raw,
		INREG32(MIPITX_BASE + field->offset),
			restore ? 1 : 0, sample_mux);
}

void dsi_m6_mipitx_lane_group_probe(const char *group_name,
				    unsigned int value,
				    unsigned int hold_ms,
				    unsigned int restore,
				    unsigned int sample_mux)
{
	char tag[64];
	const struct dsi_m6_mipitx_lane_group *group;
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old_raw[ARRAY_SIZE(dsi_m6_mipitx_lane_offsets)];
	unsigned int new_raw[ARRAY_SIZE(dsi_m6_mipitx_lane_offsets)];
	unsigned int after_raw[ARRAY_SIZE(dsi_m6_mipitx_lane_offsets)];
	unsigned int restored_raw[ARRAY_SIZE(dsi_m6_mipitx_lane_offsets)];
	unsigned int i;

	if (!DSI_REG[0])
		return;

	group = dsi_m6_mipitx_find_lane_group(group_name);
	if (!group) {
		DISPERR("M6 DSI mipitx_lane_group_probe: unknown group=%s allowed=rt,lptx,lpcd\n",
			group_name ? group_name : "null");
		return;
	}
	if (value > group->max) {
		DISPERR("M6 DSI mipitx_lane_group_probe: invalid group=%s value=%u max=%u\n",
			group->name, value, group->max);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_offsets); i++) {
		old_raw[i] = INREG32(MIPITX_BASE +
				     dsi_m6_mipitx_lane_offsets[i]);
		new_raw[i] = (old_raw[i] & ~group->mask) |
			     ((value << group->shift) & group->mask);
	}

	DISPERR("M6 DSI mipitx_lane_group_probe: begin group=%s value=%u old c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x new=0x%x/0x%x/0x%x/0x%x/0x%x mask=0x%x shift=%u restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		group->name, value, old_raw[0], old_raw[1], old_raw[2],
		old_raw[3], old_raw[4], new_raw[0], new_raw[1], new_raw[2],
		new_raw[3], new_raw[4], group->mask, group->shift,
		restore ? 1 : 0, sample_mux, bounded, dsi_m6_txrx_ctrl_raw(),
		dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("mipitx-lane-group-probe-before",
			     DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("mipitx-lane-group-probe-before");

	for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_offsets); i++)
		MIPITX_OUTREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[i],
				new_raw[i]);
	udelay(1);
	for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_offsets); i++)
		after_raw[i] = INREG32(MIPITX_BASE +
				       dsi_m6_mipitx_lane_offsets[i]);
	DISPERR("M6 DSI mipitx_lane_group_probe: after-set group=%s value=%u live c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
		group->name, value, after_raw[0], after_raw[1],
		after_raw[2], after_raw[3], after_raw[4],
		INREG32(MIPITX_BASE + 0x094), INREG32(MIPITX_BASE + 0x098),
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("mipitx-lane-group-probe-after-set",
			     DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("mipitx-lane-group-probe-after-set");

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "lanegrp-%s-mux", group->name);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux == 2) {
		snprintf(tag, sizeof(tag), "lanegrp-%s-muxstats", group->name);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	} else if (sample_mux >= 3) {
		snprintf(tag, sizeof(tag), "lanegrp-%s-window", group->name);
		dsi_m6_mipitx_pad_window(tag, 4, 50);
	}

	snprintf(tag, sizeof(tag), "lanegrp-%s-hold", group->name);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_offsets); i++)
			MIPITX_OUTREG32(MIPITX_BASE +
					dsi_m6_mipitx_lane_offsets[i],
					old_raw[i]);
		udelay(1);
		for (i = 0; i < ARRAY_SIZE(dsi_m6_mipitx_lane_offsets); i++)
			restored_raw[i] = INREG32(MIPITX_BASE +
					dsi_m6_mipitx_lane_offsets[i]);
		DISPERR("M6 DSI mipitx_lane_group_probe: after-restore group=%s old c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x live=0x%x/0x%x/0x%x/0x%x/0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
			group->name, old_raw[0], old_raw[1], old_raw[2],
			old_raw[3], old_raw[4], restored_raw[0],
			restored_raw[1], restored_raw[2], restored_raw[3],
			restored_raw[4], INREG32(MIPITX_BASE + 0x094),
			INREG32(MIPITX_BASE + 0x098),
			dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
		dsi_m6_dump_snapshot("mipitx-lane-group-probe-after-restore",
				     DISP_MODULE_DSI0, NULL);
		dsi_m6_dump_mipitx_decode("mipitx-lane-group-probe-after-restore");
	}

	DISPERR("M6 DSI mipitx_lane_group_probe: end group=%s value=%u old c/d0/d1/d2/d3=0x%x/0x%x/0x%x/0x%x/0x%x final=0x%x/0x%x/0x%x/0x%x/0x%x restore=%u mux=%u\n",
		group->name, value, old_raw[0], old_raw[1], old_raw[2],
		old_raw[3], old_raw[4],
		INREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[0]),
		INREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[1]),
		INREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[2]),
		INREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[3]),
		INREG32(MIPITX_BASE + dsi_m6_mipitx_lane_offsets[4]),
		restore ? 1 : 0, sample_mux);
}

void dsi_m6_mipitx_phy_sel_probe(unsigned int value, unsigned int hold_ms,
				 unsigned int restore, unsigned int sample_mux)
{
	char tag[64];
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old_raw;
	unsigned int after_raw;
	unsigned int restored_raw;

	if (!DSI_REG[0])
		return;

	if (!dsi_m6_phy_sel_valid(value)) {
		DISPERR("M6 DSI phy_sel_probe: invalid value=0x%x mask=0x%x fields must be <=5 and d0/d1/d2/d3/c unique\n",
			value, M6_MIPITX_PHY_SEL_MASK);
		dsi_m6_log_phy_sel("invalid", value);
		return;
	}

	old_raw = INREG32(MIPITX_BASE + 0x07c);
	DISPERR("M6 DSI phy_sel_probe: begin value=0x%x old=0x%x restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		value, old_raw, restore ? 1 : 0, sample_mux, bounded,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_log_phy_sel("old", old_raw);
	dsi_m6_log_phy_sel("new", value);
	dsi_m6_dump_snapshot("phy-sel-probe-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("phy-sel-probe-before");

	MIPITX_OUTREG32(MIPITX_BASE + 0x07c, value);
	udelay(1);
	after_raw = INREG32(MIPITX_BASE + 0x07c);
	DISPERR("M6 DSI phy_sel_probe: after-set value=0x%x live=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
		value, after_raw, INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098), dsi_m6_txrx_ctrl_raw(),
		dsi_m6_phy_lccon_raw());
	dsi_m6_log_phy_sel("after-set", after_raw);
	dsi_m6_dump_snapshot("phy-sel-probe-after-set", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("phy-sel-probe-after-set");

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "phy-sel-0x%x-mux", value);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux == 2) {
		snprintf(tag, sizeof(tag), "phy-sel-0x%x-muxstats", value);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	} else if (sample_mux >= 3) {
		snprintf(tag, sizeof(tag), "phy-sel-0x%x-window", value);
		dsi_m6_mipitx_pad_window(tag, 4, 50);
	}

	snprintf(tag, sizeof(tag), "phy-sel-0x%x-hold", value);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		MIPITX_OUTREG32(MIPITX_BASE + 0x07c, old_raw);
		udelay(1);
		restored_raw = INREG32(MIPITX_BASE + 0x07c);
		DISPERR("M6 DSI phy_sel_probe: after-restore old=0x%x live=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
			old_raw, restored_raw, INREG32(MIPITX_BASE + 0x094),
			INREG32(MIPITX_BASE + 0x098), dsi_m6_txrx_ctrl_raw(),
			dsi_m6_phy_lccon_raw());
		dsi_m6_log_phy_sel("after-restore", restored_raw);
		dsi_m6_dump_snapshot("phy-sel-probe-after-restore",
				     DISP_MODULE_DSI0, NULL);
		dsi_m6_dump_mipitx_decode("phy-sel-probe-after-restore");
	}

	DISPERR("M6 DSI phy_sel_probe: end value=0x%x old=0x%x final=0x%x restore=%u mux=%u\n",
		value, old_raw, INREG32(MIPITX_BASE + 0x07c),
		restore ? 1 : 0, sample_mux);
}

void dsi_m6_mipitx_plltop_probe(unsigned int value, unsigned int hold_ms,
				unsigned int restore, unsigned int sample_mux,
				unsigned int shift)
{
	char tag[64];
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int max;
	unsigned int mask;
	unsigned int old_raw;
	unsigned int new_raw;
	unsigned int after_raw;
	unsigned int restored_raw;

	if (!DSI_REG[0])
		return;

	if (shift != 7 && shift != 8) {
		DISPERR("M6 DSI plltop_probe: invalid shift=%u allowed=7,8\n",
			shift);
		return;
	}

	max = (shift == 7) ? 31U : 255U;
	if (value > max) {
		DISPERR("M6 DSI plltop_probe: invalid value=%u max=%u shift=%u\n",
			value, max, shift);
		return;
	}

	mask = max << shift;
	old_raw = INREG32(MIPITX_BASE + 0x064);
	new_raw = (old_raw & ~mask) | ((value << shift) & mask);
	DISPERR("M6 DSI plltop_probe: begin value=%u shift=%u old=0x%x new=0x%x mask=0x%x restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		value, shift, old_raw, new_raw, mask, restore ? 1 : 0,
		sample_mux, bounded, dsi_m6_txrx_ctrl_raw(),
		dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("plltop-probe-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("plltop-probe-before");

	MIPITX_OUTREG32(MIPITX_BASE + 0x064, new_raw);
	udelay(1);
	after_raw = INREG32(MIPITX_BASE + 0x064);
	DISPERR("M6 DSI plltop_probe: after-set value=%u shift=%u live=0x%x preserve7=0x%x preserve8=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
		value, shift, after_raw, dsi_m6_field(after_raw, 7, 5),
		dsi_m6_field(after_raw, 8, 8), INREG32(MIPITX_BASE + 0x094),
		INREG32(MIPITX_BASE + 0x098), dsi_m6_txrx_ctrl_raw(),
		dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("plltop-probe-after-set", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_mipitx_decode("plltop-probe-after-set");

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "plltop-%u-s%u-mux", value, shift);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux == 2) {
		snprintf(tag, sizeof(tag), "plltop-%u-s%u-muxstats",
			 value, shift);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	} else if (sample_mux >= 3) {
		snprintf(tag, sizeof(tag), "plltop-%u-s%u-window",
			 value, shift);
		dsi_m6_mipitx_pad_window(tag, 4, 50);
	}

	snprintf(tag, sizeof(tag), "plltop-%u-s%u-hold", value, shift);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		MIPITX_OUTREG32(MIPITX_BASE + 0x064, old_raw);
		udelay(1);
		restored_raw = INREG32(MIPITX_BASE + 0x064);
		DISPERR("M6 DSI plltop_probe: after-restore old=0x%x live=0x%x preserve7=0x%x preserve8=0x%x out=0x%x apb=0x%x txrx=0x%x lccon=0x%x\n",
			old_raw, restored_raw,
			dsi_m6_field(restored_raw, 7, 5),
			dsi_m6_field(restored_raw, 8, 8),
			INREG32(MIPITX_BASE + 0x094),
			INREG32(MIPITX_BASE + 0x098),
			dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
		dsi_m6_dump_snapshot("plltop-probe-after-restore",
				     DISP_MODULE_DSI0, NULL);
		dsi_m6_dump_mipitx_decode("plltop-probe-after-restore");
	}

	DISPERR("M6 DSI plltop_probe: end value=%u shift=%u old=0x%x final=0x%x restore=%u mux=%u\n",
		value, shift, old_raw, INREG32(MIPITX_BASE + 0x064),
		restore ? 1 : 0, sample_mux);
}

void dsi_m6_force_hsa_wc(unsigned int value, unsigned int hold_ms)
{
	unsigned int bounded = hold_ms;

	if (!DSI_REG[0])
		return;
	if (bounded > 10000)
		bounded = 10000;

	DISPERR("M6 DSI hsa_wc: begin value=0x%x hold_ms=%u live_before=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		value, bounded, INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x05c),
		INREG32(DDP_REG_BASE_DSI0 + 0x064));
	dsi_m6_dump_snapshot("hsa-wc-before", DISP_MODULE_DSI0, NULL);
	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC, value);
	DISPERR("M6 DSI hsa_wc: after-write value=0x%x live=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		value, INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x05c),
		INREG32(DDP_REG_BASE_DSI0 + 0x064));
	dsi_m6_dump_snapshot("hsa-wc-after-write", DISP_MODULE_DSI0, NULL);
	if (bounded)
		dsi_m6_dump_hs_window("hsa-wc-hold", bounded);
	DISPERR("M6 DSI hsa_wc: end value=0x%x live=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		value, INREG32(DDP_REG_BASE_DSI0 + 0x050),
		INREG32(DDP_REG_BASE_DSI0 + 0x054),
		INREG32(DDP_REG_BASE_DSI0 + 0x058),
		INREG32(DDP_REG_BASE_DSI0 + 0x05c),
		INREG32(DDP_REG_BASE_DSI0 + 0x064));
}

static unsigned int dsi_m6_bound_hold_ms(unsigned int hold_ms)
{
	if (hold_ms == 0)
		return 1000;
	if (hold_ms > 10000)
		return 10000;
	return hold_ms;
}

void dsi_m6_force_vm_cmd(unsigned int value, unsigned int hold_ms,
			 unsigned int restore, unsigned int sample_mux)
{
	char tag[64];
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old_raw;
	unsigned int final_raw;

	if (!DSI_REG[0])
		return;

	old_raw = INREG32(DDP_REG_BASE_DSI0 + 0x130);
	DISPERR("M6 DSI vm_cmd_probe: begin value=0x%x old=0x%x restore=%u mux=%u hold=%u start=0x%x int=0x%x mode=0x%x txrx=0x%x ps=0x%x\n",
		value, old_raw, restore ? 1 : 0, sample_mux, bounded,
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x014),
		INREG32(DDP_REG_BASE_DSI0 + 0x018),
		INREG32(DDP_REG_BASE_DSI0 + 0x01c));
	dsi_m6_dump_snapshot("vmcmd-probe-before", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_vm_payload_marker("vmcmd-probe-before", DISP_MODULE_DSI0,
				      NULL);

	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_VM_CMD_CON, value);
	DISPERR("M6 DSI vm_cmd_probe: after-write value=0x%x live=0x%x start=0x%x int=0x%x state=0x%x/0x%x checksum=0x%x\n",
		value, INREG32(DDP_REG_BASE_DSI0 + 0x130),
		INREG32(DDP_REG_BASE_DSI0 + 0x000),
		INREG32(DDP_REG_BASE_DSI0 + 0x00c),
		INREG32(DDP_REG_BASE_DSI0 + 0x164),
		INREG32(DDP_REG_BASE_DSI0 + 0x16c),
		INREG32(DDP_REG_BASE_DSI0 + 0x144));
	dsi_m6_dump_snapshot("vmcmd-probe-after-write", DISP_MODULE_DSI0, NULL);
	dsi_m6_dump_vm_payload_marker("vmcmd-probe-after-write",
				      DISP_MODULE_DSI0, NULL);

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "vmcmd-0x%x-mux", value);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux >= 2) {
		snprintf(tag, sizeof(tag), "vmcmd-0x%x-muxstats", value);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	}

	snprintf(tag, sizeof(tag), "vmcmd-0x%x-hold", value);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_VM_CMD_CON, old_raw);
		DISPERR("M6 DSI vm_cmd_probe: after-restore old=0x%x live=0x%x start=0x%x int=0x%x state=0x%x/0x%x checksum=0x%x\n",
			old_raw, INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c),
			INREG32(DDP_REG_BASE_DSI0 + 0x164),
			INREG32(DDP_REG_BASE_DSI0 + 0x16c),
			INREG32(DDP_REG_BASE_DSI0 + 0x144));
		dsi_m6_dump_snapshot("vmcmd-probe-after-restore",
				     DISP_MODULE_DSI0, NULL);
	}

	final_raw = INREG32(DDP_REG_BASE_DSI0 + 0x130);
	DISPERR("M6 DSI vm_cmd_probe: end value=0x%x old=0x%x final=0x%x restore=%u mux=%u\n",
		value, old_raw, final_raw, restore ? 1 : 0, sample_mux);
}

static void dsi_m6_dump_probe_mux_sweep(const char *tag)
{
	static unsigned int probe_mux_count;
	unsigned int n;

	if (!DSI_REG[0])
		return;

	n = ++probe_mux_count;
	dsi_m6_dump_debug_mux_sweep_direct(tag, n);
	dsi_m6_dump_mipitx_debug_mux_sweep_direct(tag, n);
}

static void dsi_m6_dump_probe_mux_stats(const char *tag, unsigned int samples,
					unsigned int delay_us)
{
	static unsigned int probe_mux_stats_count;
	unsigned int n;

	if (!DSI_REG[0])
		return;

	n = ++probe_mux_stats_count;
	dsi_m6_dump_debug_mux_stats_direct(tag, n, samples, delay_us);
	dsi_m6_dump_mipitx_debug_mux_stats_direct(tag, n, samples, delay_us);
}

static unsigned int dsi_m6_txrx_ctrl_raw(void)
{
	return AS_UINT32(&DSI_REG[0]->DSI_TXRX_CTRL);
}

static unsigned int dsi_m6_phy_lccon_raw(void)
{
	return AS_UINT32(&DSI_REG[0]->DSI_PHY_LCCON);
}

void dsi_m6_force_clk_restore(const char *tag)
{
	const char *safe_tag = tag ? tag : "manual";
	unsigned int old_cc;
	unsigned int old_lc;

	if (!DSI_REG[0])
		return;

	old_cc = PanelMaster_get_CC(PM_DSI0);
	old_lc = DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0;
	DISPERR("M6 DSI clk_restore[%s]: begin old_cc=%u old_lc=%u txrx=0x%x lccon=0x%x\n",
		safe_tag, old_cc, old_lc,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("clk-restore-before", DISP_MODULE_DSI0, NULL);

	PanelMaster_set_CC(PM_DSI0, 1);
	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, true);
	DISPERR("M6 DSI clk_restore[%s]: after-force cc=%u lc=%u txrx=0x%x lccon=0x%x\n",
		safe_tag, PanelMaster_get_CC(PM_DSI0),
		DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("clk-restore-after", DISP_MODULE_DSI0, NULL);
}

void DSI_PHY_clk_change(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params);
void DSI_PHY_TIMCONFIG(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params);

void dsi_m6_force_pll_change(unsigned int new_pll, const char *tag)
{
	const char *safe_tag = tag ? tag : "manual";
	LCM_DSI_PARAMS *dsi_params = &_dsi_context[0].dsi_params;
	unsigned int old_pll = dsi_params->PLL_CLOCK;
	unsigned int data_rate = new_pll * 2;

	if (!DSI_REG[0])
		return;

	if (data_rate < 50 || data_rate > 1250) {
		DISPERR("M6 DSI pll_change[%s]: REJECTED new_pll=%u data_rate=%u out of range [25..625]\n",
			safe_tag, new_pll, data_rate);
		return;
	}

	DISPERR("M6 DSI pll_change[%s]: begin old_pll=%u new_pll=%u data_rate=%u\n",
		safe_tag, old_pll, new_pll, data_rate);
	dsi_m6_dump_phy_truth("pll-change-before");

	dsi_params->PLL_CLOCK = new_pll;
	DSI_PHY_clk_change(DISP_MODULE_DSI0, NULL, dsi_params);
	DSI_PHY_TIMCONFIG(DISP_MODULE_DSI0, NULL, dsi_params);

	DISPERR("M6 DSI pll_change[%s]: done old_pll=%u new_pll=%u data_rate=%u\n",
		safe_tag, old_pll, new_pll, data_rate);
	dsi_m6_dump_phy_truth("pll-change-after");
}

void dsi_m6_force_cc_probe(unsigned int enable, unsigned int hold_ms,
			   unsigned int restore, unsigned int sample_mux)
{
	char tag[64];
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old;

	if (!DSI_REG[0])
		return;

	old = PanelMaster_get_CC(PM_DSI0);
	DISPERR("M6 DSI cc_probe: begin enable=%u old=%u restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		enable ? 1 : 0, old, restore ? 1 : 0, sample_mux, bounded,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("cc-probe-before", DISP_MODULE_DSI0, NULL);

	PanelMaster_set_CC(PM_DSI0, enable ? 1 : 0);
	DISPERR("M6 DSI cc_probe: after-set enable=%u now=%u txrx=0x%x lccon=0x%x\n",
		enable ? 1 : 0, PanelMaster_get_CC(PM_DSI0),
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("cc-probe-after-set", DISP_MODULE_DSI0, NULL);

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "cc-probe-%u-mux", enable ? 1 : 0);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux >= 2) {
		snprintf(tag, sizeof(tag), "cc-probe-%u-muxstats", enable ? 1 : 0);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	}

	snprintf(tag, sizeof(tag), "cc-probe-%u-hold", enable ? 1 : 0);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		PanelMaster_set_CC(PM_DSI0, old ? 1 : 0);
		DISPERR("M6 DSI cc_probe: after-restore old=%u now=%u txrx=0x%x lccon=0x%x\n",
			old, PanelMaster_get_CC(PM_DSI0),
			dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
		dsi_m6_dump_snapshot("cc-probe-after-restore", DISP_MODULE_DSI0, NULL);
	}

	DISPERR("M6 DSI cc_probe: end enable=%u old=%u restore=%u mux=%u final=%u\n",
		enable ? 1 : 0, old, restore ? 1 : 0, sample_mux,
		PanelMaster_get_CC(PM_DSI0));
}

void dsi_m6_force_lc_hs_probe(unsigned int enable, unsigned int hold_ms,
			      unsigned int restore, unsigned int sample_mux)
{
	char tag[64];
	unsigned int bounded = dsi_m6_bound_hold_ms(hold_ms);
	unsigned int old;

	if (!DSI_REG[0])
		return;

	old = DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0;
	DISPERR("M6 DSI lc_hs_probe: begin enable=%u old=%u restore=%u mux=%u hold=%u txrx=0x%x lccon=0x%x\n",
		enable ? 1 : 0, old, restore ? 1 : 0, sample_mux, bounded,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("lc-hs-probe-before", DISP_MODULE_DSI0, NULL);

	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, enable ? true : false);
	DISPERR("M6 DSI lc_hs_probe: after-set enable=%u now=%u txrx=0x%x lccon=0x%x\n",
		enable ? 1 : 0,
		DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0,
		dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
	dsi_m6_dump_snapshot("lc-hs-probe-after-set", DISP_MODULE_DSI0, NULL);

	if (sample_mux == 1) {
		snprintf(tag, sizeof(tag), "lc-hs-probe-%u-mux", enable ? 1 : 0);
		dsi_m6_dump_probe_mux_sweep(tag);
	} else if (sample_mux >= 2) {
		snprintf(tag, sizeof(tag), "lc-hs-probe-%u-muxstats", enable ? 1 : 0);
		dsi_m6_dump_probe_mux_stats(tag, 12, 1000);
	}

	snprintf(tag, sizeof(tag), "lc-hs-probe-%u-hold", enable ? 1 : 0);
	dsi_m6_dump_hs_window(tag, bounded);

	if (restore) {
		DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, old ? true : false);
		DISPERR("M6 DSI lc_hs_probe: after-restore old=%u now=%u txrx=0x%x lccon=0x%x\n",
			old, DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0,
			dsi_m6_txrx_ctrl_raw(), dsi_m6_phy_lccon_raw());
		dsi_m6_dump_snapshot("lc-hs-probe-after-restore", DISP_MODULE_DSI0, NULL);
	}

	DISPERR("M6 DSI lc_hs_probe: end enable=%u old=%u restore=%u mux=%u final=%u\n",
		enable ? 1 : 0, old, restore ? 1 : 0, sample_mux,
		DSI_clk_HS_state(DISP_MODULE_DSI0, NULL) ? 1 : 0);
}

static uint32_t dsi_m6_dcs_read_noreset(uint8_t cmd, uint8_t *buffer, uint8_t buffer_size)
{
	uint32_t recv_data_cnt = 0;
	unsigned char packet_type;
	DSI_RX_DATA_REG read_data0 = {0};
	DSI_RX_DATA_REG read_data1 = {0};
	DSI_RX_DATA_REG read_data2 = {0};
	DSI_RX_DATA_REG read_data3 = {0};
	DSI_T0_INS t0 = {0};
	DSI_T0_INS t1 = {0};
	long ret;

	if (buffer == NULL || buffer_size == 0) {
		DISPERR("M6 DCS status: skip cmd=0x%x invalid buffer=%p size=%u\n",
			cmd, buffer, buffer_size);
		return 0;
	}

	memset(buffer, 0, buffer_size);

	if (DSI_REG[0]->DSI_MODE_CTRL.MODE) {
		DISPERR("M6 DCS status: skip cmd=0x%x video-mode=%u START=0x%x STA=0x%x INTSTA=0x%x\n",
			cmd, DSI_REG[0]->DSI_MODE_CTRL.MODE,
			AS_UINT32(&DSI_REG[0]->DSI_START),
			AS_UINT32(&DSI_REG[0]->DSI_TRIG_STA),
			AS_UINT32(&DSI_REG[0]->DSI_INTSTA));
		return 0;
	}

	DSI_WaitForNotBusy(DISP_MODULE_DSI0, NULL);

	DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, RD_RDY, 1);
	DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, CMD_DONE, 1);

	if (DSI_REG[0]->DSI_INTSTA.RD_RDY || DSI_REG[0]->DSI_INTSTA.CMD_DONE) {
		DSI_OUTREGBIT(NULL, DSI_INT_STATUS_REG, DSI_REG[0]->DSI_INTSTA, RD_RDY, 0);
		DSI_OUTREGBIT(NULL, DSI_INT_STATUS_REG, DSI_REG[0]->DSI_INTSTA, CMD_DONE, 0);
	}

	t1.CONFG = 0x00;
	t1.Data_ID = 0x37;
	t1.Data0 = buffer_size <= 10 ? buffer_size : 10;
	t1.Data1 = 0;

	t0.CONFG = 0x04;
	t0.Data_ID = (cmd < 0xB0) ? DSI_DCS_READ_PACKET_ID : DSI_GERNERIC_READ_LONG_PACKET_ID;
	t0.Data0 = cmd;
	t0.Data1 = 0;

	waitRDDone = false;
	DSI_OUTREG32(NULL, &DSI_CMDQ_REG[0]->data[0], AS_UINT32(&t1));
	DSI_OUTREG32(NULL, &DSI_CMDQ_REG[0]->data[1], AS_UINT32(&t0));
	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_CMDQ_SIZE, 2);
	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_START, 0);
	DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_START, 1);

	/*
	 * SPI 194 does not fire, so waitRDDone is never set by the ISR; poll
	 * RD_RDY (bit0) in DSI_INTSTA instead. Return contract kept: > 0 =
	 * ready, 0 = timeout.
	 */
	{
		unsigned int c = 0;

		while (!waitRDDone &&
		       !(INREG32(&DSI_REG[0]->DSI_INTSTA) & 0x00000001)) {
			if (c++ > 200000)
				break;
			udelay(1);
		}
		ret = (waitRDDone ||
		       (INREG32(&DSI_REG[0]->DSI_INTSTA) & 0x00000001)) ? 1 : 0;
	}
	waitRDDone = false;
	if (ret <= 0) {
		DISPERR("M6 DCS status: cmd=0x%x noreset-timeout ret=%ld START=0x%x STA=0x%x INTSTA=0x%x RX0=0x%x\n",
			cmd, ret, AS_UINT32(&DSI_REG[0]->DSI_START),
			AS_UINT32(&DSI_REG[0]->DSI_TRIG_STA),
			AS_UINT32(&DSI_REG[0]->DSI_INTSTA),
			AS_UINT32(&DSI_REG[0]->DSI_RX_DATA0));
		DSI_OUTREGBIT(NULL, DSI_RACK_REG, DSI_REG[0]->DSI_RACK, DSI_RACK, 1);
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, RD_RDY, 0);
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, CMD_DONE, 0);
		return 0;
	}

	DSI_OUTREGBIT(NULL, DSI_RACK_REG, DSI_REG[0]->DSI_RACK, DSI_RACK, 1);
	DSI_OUTREGBIT(NULL, DSI_INT_STATUS_REG, DSI_REG[0]->DSI_INTSTA, RD_RDY, 0);
	DSI_OUTREGBIT(NULL, DSI_INT_STATUS_REG, DSI_REG[0]->DSI_INTSTA, CMD_DONE, 0);
	DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, RD_RDY, 0);
	DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, CMD_DONE, 0);

	AS_UINT32(&read_data0) = INREG32(&DSI_REG[0]->DSI_RX_DATA0);
	AS_UINT32(&read_data1) = INREG32(&DSI_REG[0]->DSI_RX_DATA1);
	AS_UINT32(&read_data2) = INREG32(&DSI_REG[0]->DSI_RX_DATA2);
	AS_UINT32(&read_data3) = INREG32(&DSI_REG[0]->DSI_RX_DATA3);

	packet_type = read_data0.byte0;
	if (packet_type == 0x1A || packet_type == 0x1C) {
		recv_data_cnt = read_data0.byte1 + read_data0.byte2 * 16;
		if (recv_data_cnt > 10)
			recv_data_cnt = 10;
		if (recv_data_cnt > buffer_size)
			recv_data_cnt = buffer_size;
		if (recv_data_cnt <= 4) {
			memcpy(buffer, &read_data1, recv_data_cnt);
		} else if (recv_data_cnt <= 8) {
			memcpy(buffer, &read_data1, 4);
			memcpy(buffer + 4, &read_data2, recv_data_cnt - 4);
		} else {
			memcpy(buffer, &read_data1, 4);
			memcpy(buffer + 4, &read_data2, 4);
			memcpy(buffer + 8, &read_data3, recv_data_cnt - 8);
		}
	} else if (packet_type == 0x11 || packet_type == 0x21) {
		recv_data_cnt = buffer_size < 1 ? buffer_size : 1;
		memcpy(buffer, &read_data0.byte1, recv_data_cnt);
	} else if (packet_type == 0x12 || packet_type == 0x22) {
		recv_data_cnt = buffer_size < 2 ? buffer_size : 2;
		memcpy(buffer, &read_data0.byte1, recv_data_cnt);
	}

	DISPERR("M6 DCS status: cmd=0x%x ret=%u pkt=0x%x data=%02x %02x %02x %02x RX=0x%x/0x%x/0x%x/0x%x\n",
		cmd, recv_data_cnt, packet_type,
		buffer_size > 0 ? buffer[0] : 0,
		buffer_size > 1 ? buffer[1] : 0,
		buffer_size > 2 ? buffer[2] : 0,
		buffer_size > 3 ? buffer[3] : 0,
		AS_UINT32(&read_data0), AS_UINT32(&read_data1),
		AS_UINT32(&read_data2), AS_UINT32(&read_data3));

	return recv_data_cnt;
}

static void dsi_m6_dump_dcs_status_inner(const char *tag, bool force)
{
	static int dump_count;
	uint8_t buffer[4];
	uint8_t cmds[] = {0x0A, 0x0B, 0x0C, 0x0D, 0xDA, 0xDB, 0xDC};
	int i;

	if (!force && dump_count >= 2) {
		DISPERR("M6 DCS status[%s]: skip dump_count=%d force=0\n",
			tag ? tag : "null", dump_count);
		return;
	}
	if (!force)
		dump_count++;

	DISPERR("M6 DCS status[%s]: begin force=%u count=%d\n",
		tag ? tag : "null", force ? 1 : 0, dump_count);
	for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
		dsi_m6_dcs_read_noreset(cmds[i], buffer, sizeof(buffer));
	dsi_m6_dump_snapshot("dcs-status-after", DISP_MODULE_DSI0, NULL);
}

void dsi_m6_dump_dcs_status(const char *tag)
{
	dsi_m6_dump_dcs_status_inner(tag, false);
}

void dsi_m6_dump_dcs_status_force(const char *tag)
{
	dsi_m6_dump_dcs_status_inner(tag, true);
}

/*
 * Read the panel's own state after a resume: DCS 0x0A get_power_mode
 * (bit7 booster on, bit4 sleep out, bit2 display on; 0x9C = all of them in
 * normal mode), 0x0B get_address_mode, 0x0C get_pixel_format. Command mode
 * only: dsi_m6_dcs_read_noreset() skips (and says so) in video mode.
 * Returns 0x0A, or -1 if it could not be read.
 */
int dsi_m6_read_lcm_power_mode(const char *tag)
{
	uint8_t pm[4] = {0}, am[4] = {0}, pf[4] = {0};
	uint32_t n_pm, n_am, n_pf;

	n_pm = dsi_m6_dcs_read_noreset(0x0A, pm, sizeof(pm));
	n_am = dsi_m6_dcs_read_noreset(0x0B, am, sizeof(am));
	n_pf = dsi_m6_dcs_read_noreset(0x0C, pf, sizeof(pf));
	pr_warn("[FORGE_PROBE] lcm %s: 0x0A=%s0x%02x (booster %u sleep_out %u display_on %u) 0x0B=%s0x%02x 0x0C=%s0x%02x\n",
		tag ? tag : "null", n_pm ? "" : "?", pm[0], !!(pm[0] & 0x80), !!(pm[0] & 0x10),
		!!(pm[0] & 0x04), n_am ? "" : "?", am[0], n_pf ? "" : "?", pf[0]);
	return n_pm ? pm[0] : -1;
}

unsigned int dsi_phy_get_clk(DISP_MODULE_ENUM module)
{
	int i = 0;
	int j = 0;
	unsigned int pcw = DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2.RG_DSI0_MPPLL_SDM_PCW_H;
	unsigned int prediv = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_PREDIV));
	unsigned int posdiv = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_POSDIV));
	unsigned int txdiv0 = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_TXDIV0));
	unsigned int txdiv1 = (1 << (DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0.RG_DSI0_MPPLL_TXDIV1));

	DISPMSG("%s, pcw: %d, prediv: %d, posdiv: %d, txdiv0: %d, txdiv1: %d\n", __func__, pcw,
		prediv, posdiv, txdiv0, txdiv1);
	j = prediv * 4 * txdiv0 * txdiv1;
	if (j > 0)
		return 26 * pcw / j;
	return 0;
}

void DSI_PHY_clk_change(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int j = 0;
	unsigned int data_Rate = dsi_params->PLL_CLOCK*2;
	unsigned int txdiv = 0;
	unsigned int txdiv0 = 0;
	unsigned int txdiv1 = 0;
	unsigned int pcw = 0;
	unsigned int delta1 = 5;
	/*Delta1 is SSC range, default is 0%~-5%*/
	unsigned int pdelta1 = 0;

	DISPFUNC();
	DISPMSG("New mipitx Data Rate=%d\n", data_Rate);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (0 != data_Rate) {
			if (data_Rate > 1250) {
				DISPMSG("mipitx Data Rate exceed limitation(%d)\n", data_Rate);
				ASSERT(0);
			} else if (data_Rate >= 500) {
				txdiv = 1;
				txdiv0 = 0;
				txdiv1 = 0;
			} else if (data_Rate >= 250) {
				txdiv = 2;
				txdiv0 = 1;
				txdiv1 = 0;
			} else if (data_Rate >= 125) {
				txdiv = 4;
				txdiv0 = 2;
				txdiv1 = 0;
			} else if (data_Rate > 62) {
				txdiv = 8;
				txdiv0 = 2;
				txdiv1 = 1;
			} else if (data_Rate >= 50) {
				txdiv = 16;
				txdiv0 = 2;
				txdiv1 = 2;
			} else	{
				DISPMSG("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/*1. PLL TXDIV Config*/
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					RG_DSI0_MPPLL_TXDIV0, txdiv0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					RG_DSI0_MPPLL_TXDIV1, txdiv1);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
					RG_DSI0_MPPLL_PREDIV, 0);

			/* 2. PLL PCW Config */
			/*
			PCW bit 24~30 = floor(pcw)
			PCW bit 16~23 = (pcw - floor(pcw))*256
			PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
			PCW bit 8~15 = (pcw*256*256 - floor(pcw)*256*256)*256
			*/
			/* pcw = data_Rate*4*txdiv/(26*2);//Post DIV =4, so need data_Rate*4*/
			pcw = data_Rate*txdiv/13;

			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
						RG_DSI0_MPPLL_SDM_PCW_H, (pcw & 0x7F));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
						RG_DSI0_MPPLL_SDM_PCW_16_23, ((256*(data_Rate*txdiv%13)/13) & 0xFF));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
						RG_DSI0_MPPLL_SDM_PCW_8_15,
						((256*(256*(data_Rate*txdiv%13)%13)/13) & 0xFF));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
						RG_DSI0_MPPLL_SDM_PCW_0_7,
						((256*(256*(256*(data_Rate*txdiv%13)%13)%13)/13) & 0xFF));

			/*3. SSC Config*/
			if (1 != dsi_params->ssc_disable) {
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
							RG_DSI0_MPPLL_SDM_SSC_PH_INIT, 1);
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
							RG_DSI0_MPPLL_SDM_SSC_PRD, 0x1B1);/*//PRD=ROUND(pmod) = 433;*/
				if (0 != dsi_params->ssc_range)
					delta1 = dsi_params->ssc_range;
				ASSERT(delta1 <= 8);
				pdelta1 = (delta1*data_Rate*txdiv*262144+281664)/563329;
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
							RG_DSI0_MPPLL_SDM_SSC_DELTA, pdelta1);
				DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON3_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
							RG_DSI0_MPPLL_SDM_SSC_DELTA1, pdelta1);
				DISPMSG("PLL config:data_rate=%d,txdiv=%d,pcw=%d,delta1=%d,pdelta1=0x%x\n",
					data_Rate, txdiv, DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG,
					&DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2),
					delta1, pdelta1);
			}
		} else {/*not use*/
			/*1. PLL TXDIV Config*/
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
						RG_DSI0_MPPLL_TXDIV0, dsi_params->pll_div1);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
						RG_DSI0_MPPLL_TXDIV1, dsi_params->pll_div2);
			/*2. PLL PCW Config*/
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				RG_DSI0_MPPLL_SDM_PCW_H, ((dsi_params->fbk_div) << 2));
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				RG_DSI0_MPPLL_SDM_PCW_16_23, 0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				RG_DSI0_MPPLL_SDM_PCW_8_15, 0);
			DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON2_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
				RG_DSI0_MPPLL_SDM_PCW_0_7, 0);
			/*3. SSC Config*/
			/*why no ssc config*/
		}

		DSI_OUTREGBIT(cmdq, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
				RG_DSI0_MPPLL_SDM_FRA_EN, 1);
		/*4. delay >30ns, need check, cmdq writewithmask 80ns *  9*/
			/*for(j = 0; j < 10; j++) */
			/*//DSI_OUTREG32(cmdq,&DSI_CMDQ_REG[0]->data[127],1);*/
			/*// 5. DSI_PLL_CHG = 0 */
	    DSI_OUTREG32(cmdq, &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, 0);
		/*6. delay >20us , need check*/
		for (j = 0; j < 250; j++)
				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[0]->data[126], 1);

		/*7. DSI_PLL_CHG = 1 */
		DSI_OUTREG32(cmdq, &DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG, 1);
	}
}
void DSI_PHY_clk_setting(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{
#ifdef CONFIG_FPGA_EARLY_PORTING
#if 0
	MIPITX_Write60384(0x18, 0x00, 0x10);
	MIPITX_Write60384(0x20, 0x42, 0x01);
	MIPITX_Write60384(0x20, 0x43, 0x01);
	MIPITX_Write60384(0x20, 0x05, 0x01);
	MIPITX_Write60384(0x20, 0x22, 0x01);
	MIPITX_Write60384(0x30, 0x44, 0x83);
	MIPITX_Write60384(0x30, 0x40, 0x82);
	MIPITX_Write60384(0x30, 0x00, 0x03);
	MIPITX_Write60384(0x30, 0x68, 0x03);
	MIPITX_Write60384(0x30, 0x68, 0x01);
	MIPITX_Write60384(0x30, 0x50, 0x80);
	MIPITX_Write60384(0x30, 0x51, 0x01);
	MIPITX_Write60384(0x30, 0x54, 0x01);
	MIPITX_Write60384(0x30, 0x58, 0x00);
	MIPITX_Write60384(0x30, 0x59, 0x00);
	MIPITX_Write60384(0x30, 0x5a, 0x00);
	MIPITX_Write60384(0x30, 0x5b, (dsi_params->fbk_div) << 2);
	MIPITX_Write60384(0x30, 0x04, 0x11);
	MIPITX_Write60384(0x30, 0x08, 0x01);
	MIPITX_Write60384(0x30, 0x0C, 0x01);
	MIPITX_Write60384(0x30, 0x10, 0x01);
	MIPITX_Write60384(0x30, 0x14, 0x01);
	MIPITX_Write60384(0x30, 0x64, 0x20);
	MIPITX_Write60384(0x30, 0x50, 0x81);
	MIPITX_Write60384(0x30, 0x28, 0x00);
	mdelay(500);
	DISPMSG("PLL setting finish!!\n");

	DISP_REG_SET(NULL, DISP_REG_CONFIG_MMSYS_LCM_RST_B, 0);
	DISP_REG_SET(NULL, DISP_REG_CONFIG_MMSYS_LCM_RST_B, 1);
	DSI_OUTREG32(cmdq, &DSI_REG[0]->DSI_COM_CTRL, 0x5);
	DSI_OUTREG32(cmdq, &DSI_REG[0]->DSI_COM_CTRL, 0x0);
#endif
#else
#if 0
	MIPITX_OUTREG32(0x10215044, 0x88492483);
	MIPITX_OUTREG32(0x10215040, 0x00000002);
	mdelay(10);
	MIPITX_OUTREG32(0x10215000, 0x00000403);
	MIPITX_OUTREG32(0x10215068, 0x00000003);
	MIPITX_OUTREG32(0x10215068, 0x00000001);

	mdelay(10);
	MIPITX_OUTREG32(0x10215050, 0x00000000);
	mdelay(10);
	MIPITX_OUTREG32(0x10215054, 0x00000003);
	MIPITX_OUTREG32(0x10215058, 0x60000000);
	MIPITX_OUTREG32(0x1021505c, 0x00000000);

	MIPITX_OUTREG32(0x10215004, 0x00000803);
	MIPITX_OUTREG32(0x10215008, 0x00000801);
	MIPITX_OUTREG32(0x1021500c, 0x00000801);
	MIPITX_OUTREG32(0x10215010, 0x00000801);
	MIPITX_OUTREG32(0x10215014, 0x00000801);

	MIPITX_OUTREG32(0x10215050, 0x00000001);

	mdelay(10);


	MIPITX_OUTREG32(0x10215064, 0x00000020);
	return 0;
#endif

	int i = 0;
	unsigned int data_Rate = dsi_params->PLL_CLOCK * 2;
	unsigned int txdiv = 0;
	unsigned int txdiv0 = 0;
	unsigned int txdiv1 = 0;
	unsigned int pcw = 0;
/* unsigned int fmod = 30;//Fmod = 30KHz by default */
	unsigned int delta1 = 5;	/* Delta1 is SSC range, default is 0%~-5% */
	unsigned int pdelta1 = 0;
/*	u32 m_hw_res3 = 0;
	u32 temp1 = 0;
	u32 temp2 = 0;
	u32 temp3 = 0;
	u32 temp4 = 0;
	u32 temp5 = 0;
	u32 lnt = 0;*/

	/* temp1~5 is used for impedence calibration, not enable now */
#if 0
	m_hw_res3 = INREG32(0xF0206180);
	temp1 = (m_hw_res3 >> 28) & 0xF;
	temp2 = (m_hw_res3 >> 24) & 0xF;
	temp3 = (m_hw_res3 >> 20) & 0xF;
	temp4 = (m_hw_res3 >> 16) & 0xF;
	temp5 = (m_hw_res3 >> 12) & 0xF;
#endif

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		dsi_m6_dump_rt_cal("phy-clk-before");
		/* step 0 */
		MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
						RG_DSI_LNTC_RT_CODE, (clock_lane>>8) & 0xf);
		MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3,
						RG_DSI_LNT3_RT_CODE, (data_lane3>>8) & 0xf);
		MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2,
						RG_DSI_LNT2_RT_CODE, (data_lane2>>8) & 0xf);
		MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1,
						RG_DSI_LNT1_RT_CODE, (data_lane1>>8) & 0xf);
		MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG, DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0,
						RG_DSI_LNT0_RT_CODE, (data_lane0>>8) & 0xf);

		DISPMSG("PLL config: clk=0x%x,lan3=0x%x,lan2=0x%x,lan1=0x%x,lan0=0x%x\n",
		INREG32(&DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE),
		INREG32(&DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3),
		INREG32(&DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2),
		INREG32(&DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1),
		INREG32(&DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0));
		dsi_m6_dump_rt_cal("phy-clk-after-rt-write");
		/* step 1 */
		/* MIPITX_MASKREG32(APMIXED_BASE+0x00, (0x1<<6), 1); */

		/* step 2 */
		MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON,
				 RG_DSI_BG_CORE_EN, 1);
		MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON,
				 RG_DSI_BG_CKEN, 1);

		/* step 3 */
		dsi_m6_phy_lk_delay("phy-clk-bg-settle", M6_LK_PHY_BG_SETTLE_MS);

		/* step 4 */
		MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
				 RG_DSI_LNT_HS_BIAS_EN, 1);

		/* step 5 */
		MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON,
				 RG_DSI_CKG_LDOOUT_EN, 1);
		MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON,
				 RG_DSI_LDOCORE_EN, 1);

		/* step 6 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
				 DA_DSI_MPPLL_SDM_PWR_ON, 1);

		/* step 7 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
				 DA_DSI_MPPLL_SDM_ISO_EN, 0);
		mdelay(1);

		if (0 != data_Rate) {
			if (data_Rate > 1250) {
				DISPMSG("mipitx Data Rate exceed limitation(%d)\n", data_Rate);
				ASSERT(0);
			} else if (data_Rate >= 500) {
				txdiv = 1;
				txdiv0 = 0;
				txdiv1 = 0;
			} else if (data_Rate >= 250) {
				txdiv = 2;
				txdiv0 = 1;
				txdiv1 = 0;
			} else if (data_Rate >= 125) {
				txdiv = 4;
				txdiv0 = 2;
				txdiv1 = 0;
			} else if (data_Rate > 62) {
				txdiv = 8;
				txdiv0 = 2;
				txdiv1 = 1;
			} else if (data_Rate >= 50) {
				txdiv = 16;
				txdiv0 = 2;
				txdiv1 = 2;
			} else {
				DISPMSG("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/* step 8 */
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_TXDIV0,
					 txdiv0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_TXDIV1,
					 txdiv1);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_PREDIV,
					 0);


			/* step 10 */
			/* PLL PCW config */
			/*
			   PCW bit 24~30 = floor(pcw)
			   PCW bit 16~23 = (pcw - floor(pcw))*256
			   PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
			   PCW bit 8~15 = (pcw*256*256 - floor(pcw)*256*256)*256
			 */
			/* pcw = data_Rate*4*txdiv/(26*2);//Post DIV =4, so need data_Rate*4 */
			pcw = data_Rate * txdiv / 13;

			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_H, (pcw & 0x7F));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_16_23,
					 ((256 * (data_Rate * txdiv % 13) / 13) & 0xFF));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_8_15,
					 ((256 * (256 * (data_Rate * txdiv % 13) % 13) /
					   13) & 0xFF));
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2,
					 RG_DSI0_MPPLL_SDM_PCW_0_7,
					 ((256 *
					   (256 * (256 * (data_Rate * txdiv % 13) % 13) % 13) /
					   13) & 0xFF));

			if (1 != dsi_params->ssc_disable) {
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG,
						 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
						 RG_DSI0_MPPLL_SDM_SSC_PH_INIT, 1);
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
					RG_DSI0_MPPLL_SDM_SSC_PRD, 0x1B1);	/* PRD=ROUND(pmod) = 433; */
				if (0 != dsi_params->ssc_range)
					delta1 = dsi_params->ssc_range;
				if(delta1 > 8) {
					DISPMSG("dsi_params->ssc_range is wrong, we set it to 5, deltal =%d\n", delta1);
					delta1 = 5;
				}
				//ASSERT(delta1 <= 8);
				pdelta1 = (delta1 * data_Rate * txdiv * 262144 + 281664) / 563329;
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON3_REG,
						 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
						 RG_DSI0_MPPLL_SDM_SSC_DELTA, pdelta1);
				MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON3_REG,
						 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON3,
						 RG_DSI0_MPPLL_SDM_SSC_DELTA1, pdelta1);
				DISPMSG
				    ("PLL config:data_rate=%d,txdiv=%d,pcw=%d,delta1=%d,pdelta1=0x%x\n",
				     data_Rate, txdiv, DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG,
								   &DSI_PHY_REG[i]->
								   MIPITX_DSI_PLL_CON2), delta1,
				     pdelta1);
			}
		} else {
			DISPERR("[dsi_dsi.c] PLL clock should not be 0!!!\n");
			ASSERT(0);
		}

		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
				 RG_DSI0_MPPLL_SDM_FRA_EN, 1);

		/* step 11 */
		MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG, DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
				 RG_DSI_LNTC_LDOOUT_EN, 1);

		/* step 12 */
		if (dsi_params->LANE_NUM > 0) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0,
					 RG_DSI_LNT0_LDOOUT_EN, 1);
		}
		/* step 13 */
		if (dsi_params->LANE_NUM > 1) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1,
					 RG_DSI_LNT1_LDOOUT_EN, 1);
		}
		/* step 14 */
		if (dsi_params->LANE_NUM > 2) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2,
					 RG_DSI_LNT2_LDOOUT_EN, 1);
		}
		/* step 15 */
		if (dsi_params->LANE_NUM > 3) {
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3,
					 RG_DSI_LNT3_LDOOUT_EN, 1);
		}
		/* step 16 */
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0,
				 RG_DSI0_MPPLL_PLL_EN, 1);

		/* step 17 */
		dsi_m6_phy_lk_delay("phy-clk-pll-en-settle",
			M6_LK_PHY_PLL_EN_SETTLE_MS);

		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CHG_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG,
				 RG_DSI0_MPPLL_SDM_PCW_CHG, 0);
		MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CHG_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_CHG,
				 RG_DSI0_MPPLL_SDM_PCW_CHG, 1);

		if ((0 != data_Rate) && (1 != dsi_params->ssc_disable)) {
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
					 RG_DSI0_MPPLL_SDM_SSC_EN, 1);
		} else {
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1,
					 RG_DSI0_MPPLL_SDM_SSC_EN, 0);
		}

			/* step 18 */
			MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
					 RG_DSI_PAD_TIE_LOW_EN, 0);

			dsi_m6_phy_lk_delay("phy-clk-pcw-pad-settle",
				M6_LK_PHY_PCW_PAD_SETTLE_MS);
			dsi_m6_dump_rt_cal("phy-clk-after");
		}
	#endif
	}



void DSI_PHY_TIMCONFIG(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_DSI_PARAMS *dsi_params)
{

#ifdef CONFIG_FPGA_EARLY_PORTING
	return 0;
#endif

	DSI_PHY_TIMCON0_REG timcon0;
	DSI_PHY_TIMCON1_REG timcon1;
	DSI_PHY_TIMCON2_REG timcon2;
	DSI_PHY_TIMCON3_REG timcon3;
	int i = 0;
	unsigned int lane_no;
	unsigned int cycle_time;
	unsigned int ui;
	unsigned int hs_trail_m, hs_trail_n;
#if 0
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, 0x140f0708);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, 0x10280c20);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, 0x14280000);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, 0x00101a06);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON4, 0x00023000);
	}
	return;
#endif
	lane_no = dsi_params->LANE_NUM;
	if (0 != dsi_params->PLL_CLOCK) {
		ui = 1000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		cycle_time = 8000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		DISPMSG
		("[DISP] - kernel - DSI_PHY_TIMCONFIG, Cycle Time = %d(ns), Unit Interval = %d(ns). , lane# = %d\n",
		       cycle_time, ui, lane_no);
	} else {
		DISPERR("[dsi_dsi.c] PLL clock should not be 0!!!\n");
		ASSERT(0);
	}

	/* div2_real=div2 ? div2*0x02 : 0x1; */
	/* cycle_time = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26) + 1; */
	/* ui = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26 * 2) + 1; */
#define NS_TO_CYCLE(n, c)	((n) / (c))

	hs_trail_m = 1;
	hs_trail_n =
	    (dsi_params->HS_TRAIL == 0) ? NS_TO_CYCLE(((hs_trail_m * 0x4 * ui) + 0x50),
						      cycle_time) : dsi_params->HS_TRAIL;
	/* +3 is recommended from designer becauase of HW latency */
	timcon0.HS_TRAIL = (hs_trail_m > hs_trail_n) ? hs_trail_m : hs_trail_n;

	timcon0.HS_PRPR =
	    (dsi_params->HS_PRPR == 0) ? NS_TO_CYCLE((0x40 + 0x5 * ui),
						     cycle_time) : dsi_params->HS_PRPR;
	/* HS_PRPR can't be 1. */
	if (timcon0.HS_PRPR < 1)
		timcon0.HS_PRPR = 1;

	timcon0.HS_ZERO =
	    (dsi_params->HS_ZERO == 0) ? NS_TO_CYCLE((0xC8 + 0x0a * ui),
						     cycle_time) : dsi_params->HS_ZERO;
	if (timcon0.HS_ZERO > timcon0.HS_PRPR)
		timcon0.HS_ZERO -= timcon0.HS_PRPR;

	timcon0.LPX = (dsi_params->LPX == 0) ? NS_TO_CYCLE(0x50, cycle_time) : dsi_params->LPX;
	if (timcon0.LPX < 1)
		timcon0.LPX = 1;

	/* timcon1.TA_SACK         = (dsi_params->TA_SACK == 0) ? 1 : dsi_params->TA_SACK; */
	timcon1.TA_GET = (dsi_params->TA_GET == 0) ? (0x5 * timcon0.LPX) : dsi_params->TA_GET;
	timcon1.TA_SURE =
	    (dsi_params->TA_SURE == 0) ? (0x3 * timcon0.LPX / 0x2) : dsi_params->TA_SURE;
	timcon1.TA_GO = (dsi_params->TA_GO == 0) ? (0x4 * timcon0.LPX) : dsi_params->TA_GO;
	/* -------------------------------------------------------------- */
	/* NT35510 need fine tune timing */
	/* Data_hs_exit = 60 ns + 128UI */
	/* Clk_post = 60 ns + 128 UI. */
	/* -------------------------------------------------------------- */
	timcon1.DA_HS_EXIT =
	    (dsi_params->DA_HS_EXIT == 0) ? (0x2 * timcon0.LPX) : dsi_params->DA_HS_EXIT;

	timcon2.CLK_TRAIL =
	    ((dsi_params->CLK_TRAIL == 0) ? NS_TO_CYCLE(0x60,
							cycle_time) : dsi_params->CLK_TRAIL) + 0x01;
	/* CLK_TRAIL can't be 1. */
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

	/* timcon2.LPX_WAIT        = (dsi_params->LPX_WAIT == 0) ? 1 : dsi_params->LPX_WAIT; */
	timcon2.CONT_DET = dsi_params->CONT_DET;
	timcon2.CLK_ZERO =
	    (dsi_params->CLK_ZERO == 0) ? NS_TO_CYCLE(0x190, cycle_time) : dsi_params->CLK_ZERO;

	timcon3.CLK_HS_PRPR =
	    (dsi_params->CLK_HS_PRPR == 0) ? NS_TO_CYCLE(0x40,
							 cycle_time) : dsi_params->CLK_HS_PRPR;
	if (timcon3.CLK_HS_PRPR < 1)
		timcon3.CLK_HS_PRPR = 1;
	timcon3.CLK_HS_EXIT =
	    (dsi_params->CLK_HS_EXIT == 0) ? (0x2 * timcon0.LPX) : dsi_params->CLK_HS_EXIT;
	timcon3.CLK_HS_POST =
	    (dsi_params->CLK_HS_POST == 0) ? NS_TO_CYCLE((0x60 + 0x34 * ui),
							 cycle_time) : dsi_params->CLK_HS_POST;

	DISPMSG(
		"[DISP] - kernel - DSI_PHY_TIMCONFIG, HS_TRAIL = %d, HS_ZERO = %d, HS_PRPR = %d, LPX = %d, TA_GET = %d, TA_SURE = %d, TA_GO = %d, CLK_TRAIL = %d, CLK_ZERO = %d, CLK_HS_PRPR = %d\n",
		timcon0.HS_TRAIL, timcon0.HS_ZERO, timcon0.HS_PRPR, timcon0.LPX,
		timcon1.TA_GET, timcon1.TA_SURE, timcon1.TA_GO, timcon2.CLK_TRAIL,
		timcon2.CLK_ZERO, timcon3.CLK_HS_PRPR);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, LPX,
			      timcon0.LPX);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_PRPR,
			      timcon0.HS_PRPR);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_ZERO,
			      timcon0.HS_ZERO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON0_REG, DSI_REG[i]->DSI_PHY_TIMECON0, HS_TRAIL,
			      timcon0.HS_TRAIL);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_GO,
			      timcon1.TA_GO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_SURE,
			      timcon1.TA_SURE);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, TA_GET,
			      timcon1.TA_GET);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON1_REG, DSI_REG[i]->DSI_PHY_TIMECON1, DA_HS_EXIT,
			      timcon1.DA_HS_EXIT);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CONT_DET,
			      timcon2.CONT_DET);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CLK_ZERO,
			      timcon2.CLK_ZERO);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON2_REG, DSI_REG[i]->DSI_PHY_TIMECON2, CLK_TRAIL,
			      timcon2.CLK_TRAIL);

		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
			      timcon3.CLK_HS_PRPR);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_POST,
			      timcon3.CLK_HS_POST);
		DSI_OUTREGBIT(cmdq, DSI_PHY_TIMCON3_REG, DSI_REG[i]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
			      timcon3.CLK_HS_EXIT);
		DISPMSG("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n", __func__,
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
}


void DSI_PHY_clk_switch(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, int on)
{
	int i = 0;

	/* can't use cmdq for this */
	ASSERT(cmdq == NULL);

	if (on) {
		DSI_PHY_clk_setting(module, cmdq, &(_dsi_context[i].dsi_params));
	} else {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			/* disable mipi clock */
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_PLL_EN,
					 0);
			mdelay(1);
			MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
					 RG_DSI_PAD_TIE_LOW_EN, 1);


			MIPITX_OUTREGBIT(MIPITX_DSI_CLOCK_LANE_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_CLOCK_LANE,
					 RG_DSI_LNTC_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE0,
					 RG_DSI_LNT0_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE1_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE1,
					 RG_DSI_LNT1_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE2_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE2,
					 RG_DSI_LNT2_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_DATA_LANE3_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_DATA_LANE3,
					 RG_DSI_LNT3_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
					 DA_DSI_MPPLL_SDM_ISO_EN, 1);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_PWR_REG, DSI_PHY_REG[i]->MIPITX_DSI_PLL_PWR,
					 DA_DSI_MPPLL_SDM_PWR_ON, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_TOP_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_TOP_CON,
					 RG_DSI_LNT_HS_BIAS_EN, 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON,
					 RG_DSI_CKG_LDOOUT_EN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_CON,
					 RG_DSI_LDOCORE_EN, 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON,
					 RG_DSI_BG_CKEN, 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_BG_CON_REG, DSI_PHY_REG[i]->MIPITX_DSI_BG_CON,
					 RG_DSI_BG_CORE_EN, 0);

			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_PREDIV,
					 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_TXDIV0,
					 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_TXDIV1,
					 0);
			MIPITX_OUTREGBIT(MIPITX_DSI_PLL_CON0_REG,
					 DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON0, RG_DSI0_MPPLL_POSDIV,
					 0);


			MIPITX_OUTREG32(&DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON1, 0x00000000);
			MIPITX_OUTREG32(&DSI_PHY_REG[i]->MIPITX_DSI_PLL_CON2, 0x50000000);
			mdelay(1);
		}
	}
}

DSI_STATUS DSI_EnableClk(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
#if 0
	int i = 0;

	DISPFUNC();
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_EN, 1);
#endif
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_DisableClk(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
#if 0
	int i;

	DISPFUNC();
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		DSI_OUTREGBIT(cmdq, DSI_COM_CTRL_REG, DSI_REG[i]->DSI_COM_CTRL, DSI_EN, 0);
#endif
	return DSI_STATUS_OK;
}

DSI_STATUS DSI_Start(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	static unsigned int dump_count;

	if (module == DISP_MODULE_DSI0) {
		dsi_m6_dump_hs_video_limited("start-before", module, cmdq,
					     &dump_count, 8);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, DSI_START, 1);
		dsi_m6_dump_hs_video_limited("start-after", module, cmdq,
					     &dump_count, 16);
		dsi_m6_schedule_hs_video_delayed();
	}

	return DSI_STATUS_OK;
}

void DSI_Set_VM_CMD(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{

	int i = 0;
	static unsigned int m6_vm_set_count;
	bool m6_dump = false;

	if (module == DISP_MODULE_DSI0 && DSI_REG[0] &&
	    m6_vm_set_count < 16) {
		m6_vm_set_count++;
		m6_dump = true;
		dsi_m6_sram_video_snapshot("vm-set-entry", module);
		DISPERR("M6 DSI vm_cmd[set-entry]: count=%u cmdq=%p raw=0x%x start=0x%x intsta=0x%x\n",
			m6_vm_set_count, cmdq,
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c));
		dsi_m6_dump_vm_payload_marker("vm-set-entry", module, cmdq);
	}

	if (module != DISP_MODULE_DSIDUAL) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON,
				      TS_VFP_EN, 1);
			DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON,
				      VM_CMD_EN, 1);
			DISPMSG("DSI_Set_VM_CMD");
		}
	} else {
		DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, TS_VFP_EN, 1);
		DSI_OUTREGBIT(cmdq, DSI_VM_CMD_CON_REG, DSI_REG[i]->DSI_VM_CMD_CON, VM_CMD_EN, 1);
	}
	if (m6_dump) {
		dsi_m6_sram_video_snapshot("vm-set-after", module);
		DISPERR("M6 DSI vm_cmd[set-after-enqueue]: count=%u cmdq=%p raw=0x%x start=0x%x intsta=0x%x\n",
			m6_vm_set_count, cmdq,
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c));
		dsi_m6_dump_vm_payload_marker("vm-set-after-enqueue", module,
					       cmdq);
	}
}

DSI_STATUS DSI_EnableVM_CMD(DISP_MODULE_ENUM module, cmdqRecHandle cmdq)
{
	int i = 0;
	static unsigned int m6_vm_enable_count;
	bool m6_dump = false;

	if (module == DISP_MODULE_DSI0 && DSI_REG[0] &&
	    m6_vm_enable_count < 32) {
		m6_vm_enable_count++;
		m6_dump = true;
		dsi_m6_sram_video_snapshot("vm-enable-entry", module);
		DISPERR("M6 DSI vm_cmd[enable-entry]: count=%u cmdq=%p raw=0x%x start=0x%x intsta=0x%x\n",
			m6_vm_enable_count, cmdq,
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c));
	}

	if (cmdq)
		DSI_MASKREG32(cmdq, &DSI_REG[0]->DSI_INTSTA, 0x00000020, 0x00000000);
	else
		wait_vm_cmd_done = false;

	if (module != DISP_MODULE_DSIDUAL) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
			DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		}
	} else {
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq, DSI_START_REG, DSI_REG[0]->DSI_START, VM_CMD_START, 1);
	}
	if (cmdq) {
		DSI_POLLREG32(cmdq, &DSI_REG[0]->DSI_INTSTA, 0x00000020, 0x00000020);
		DSI_MASKREG32(cmdq, &DSI_REG[0]->DSI_INTSTA, 0x00000020, 0x00000000);
	} else {
		/*
		 * SPI 194 does not fire, so wait_vm_cmd_done is never set and
		 * this untimed wait would hang until the watchdog. Poll
		 * VM_CMD_DONE (bit5) in DSI_INTSTA directly, then W1C it (as in
		 * the cmdq branch above).
		 */
		unsigned int c = 0;

		while (!wait_vm_cmd_done &&
		       !(INREG32(&DSI_REG[0]->DSI_INTSTA) & 0x00000020)) {
			if (c++ > 200000) {
				DISPERR("[M681] dsi poll vm_cmd_done timeout\n");
				break;
			}
			udelay(1);
		}
		DSI_MASKREG32(NULL, &DSI_REG[0]->DSI_INTSTA, 0x00000020, 0x00000000);
	}

	if (m6_dump) {
		dsi_m6_sram_video_snapshot("vm-enable-after", module);
		DISPERR("M6 DSI vm_cmd[enable-after-wait]: count=%u cmdq=%p raw=0x%x start=0x%x intsta=0x%x\n",
			m6_vm_enable_count, cmdq,
			INREG32(DDP_REG_BASE_DSI0 + 0x130),
			INREG32(DDP_REG_BASE_DSI0 + 0x000),
			INREG32(DDP_REG_BASE_DSI0 + 0x00c));
	}

	return DSI_STATUS_OK;
}

unsigned int DSI_esd_check_times(LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	static unsigned int esd_check_times;

	if (esd_check_times)
		return esd_check_times;
	esd_check_times = 0;
	for (i = 0; i < ESD_CHECK_NUM; i++) {
		if (dsi_params->lcm_esd_check_table[i].cmd == 0)
			break;
		esd_check_times++;
	}
	DISPMSG("1.ESD times %d\n", esd_check_times);
	return esd_check_times;
}

int DSI_read_cmp(unsigned int index, DSI_RX_DATA_REG *read_data,
	unsigned char *para_list, unsigned int count)
{
	int ret = 0;
	int i = 0;
	unsigned char packet_type;
	uint32_t recv_data_cnt = 0;
	LCM_DSI_PARAMS *dsi_params = NULL;
	unsigned char buffer[20];

	dsi_params = &_dsi_context[0].dsi_params;
	packet_type = read_data[0].byte0;
	DISPDBG("DSI read packet_type is 0x%x\n", packet_type);

	/* 0x02: acknowledge & error report */
	/* 0x11: generic short read response(1 byte return) */
	/* 0x12: generic short read response(2 byte return) */
	/* 0x1a: generic long read response */
	/* 0x1c: dcs long read response */
	/* 0x21: dcs short read response(1 byte return) */
	/* 0x22: dcs short read response(2 byte return) */
	if (packet_type == 0x1A || packet_type == 0x1C) {
		recv_data_cnt = read_data[0].byte1 + read_data[0].byte2 * 16;
		DISPDBG("packet_type=0x%x,recv_data_cnt = %d\n",
			packet_type, recv_data_cnt);
		if (count > 20)
			count = 20;
		if (recv_data_cnt > count)
			recv_data_cnt = count;
		if (recv_data_cnt <= 4) {
			memcpy((void *)buffer, (void *)&read_data[1], recv_data_cnt);
		} else if (recv_data_cnt <= 8) {
			memcpy((void *)buffer, (void *)&read_data[1], 4);
			memcpy((void *)(buffer + 4), (void *)&read_data[2],
				recv_data_cnt - 4);
		} else {
			memcpy((void *)buffer, (void *)&read_data[1], 4);
			memcpy((void *)(buffer + 4), (void *)&read_data[2], 4);
			memcpy((void *)(buffer + 8), (void *)&read_data[3],
				recv_data_cnt - 8);
		}
		if(0x54 == dsi_params->lcm_esd_check_table[index].para_list[0] && 0x24 == dsi_params->lcm_esd_check_table[index].para_list[1] && 0x2C == dsi_params->lcm_esd_check_table[index].para_list[2])
			{
				if(buffer[0]!=0x24&&buffer[0]!=0x2C)
					{
						ret = 1;
						return ret;
						DISPDBG("[ESD] index=%d,buffer[0]=0x%x\n",index,buffer[0]);
					}
			}
		else
			{
		for (i = 0; i < recv_data_cnt; i++) {
			DISPDBG("buffer[%d]=0x%x\n", i, buffer[i]);
			if (buffer[i] != dsi_params->lcm_esd_check_table[index].para_list[i]) {
				ret = 1;
				DISPMSG("[ESD]CMP index %d return value 0x%x,para_list[%d]=0x%x\n", index,
					buffer[i], i, dsi_params->lcm_esd_check_table[index].para_list[i]);
				break;
			}
			}
		}
	} else if (packet_type == 0x11 || packet_type == 0x12 ||
					packet_type == 0x21 || packet_type == 0x22) {
		if (packet_type == 0x11 || packet_type == 0x21)
			recv_data_cnt = 1;
		else
			recv_data_cnt = 2;
		if (recv_data_cnt > count)
			recv_data_cnt = count;
		memcpy((void *)buffer, (void *)&read_data[0].byte1,
			recv_data_cnt);
		DISPDBG("packet_type=0x%x,recv_data_cnt = %d\n", packet_type, recv_data_cnt);
		if(0x54 == dsi_params->lcm_esd_check_table[index].para_list[0] && 0x24 == dsi_params->lcm_esd_check_table[index].para_list[1] && 0x2C == dsi_params->lcm_esd_check_table[index].para_list[2])
			{
				if(buffer[0]!=0x24&&buffer[0]!=0x2C)
					{
						ret = 1;
						return ret;
					}
			}
		else
			{
		for (i = 0; i < recv_data_cnt; i++) {
			DISPDBG("buffer[%d]=0x%x\n", i, buffer[i]);
			if (buffer[i] != dsi_params->lcm_esd_check_table[index].para_list[i]) {
				ret = 1;
				DISPMSG("[ESD]CMP index %d return value 0x%x,para_list[%d]=0x%x\n", index,
					buffer[i], i, dsi_params->lcm_esd_check_table[index].para_list[i]);
				break;
			}
			}
		}
	} else if (packet_type == 0x02) {
		DISPMSG("read return type is 0x02, not support re-read\n");
		ret = 1;
	} else {
		DISPMSG("read return type is non-recognite, type = 0x%x\n",
			packet_type);
		ret = 1;
	}
	return ret;
}
/* / return value: the data length we got */
uint32_t DSI_dcs_read_lcm_reg_v2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, uint8_t cmd,
			       uint8_t *buffer, uint8_t buffer_size)
{
	int d = 0;
	uint32_t max_try_count = 5;
	uint32_t recv_data_cnt;
	unsigned char packet_type;
	DSI_RX_DATA_REG read_data0;
	DSI_RX_DATA_REG read_data1;
	DSI_RX_DATA_REG read_data2;
	DSI_RX_DATA_REG read_data3;
	DSI_T0_INS t0;
	DSI_T0_INS t1;
	long ret;
	int timeout = 0;
	static unsigned int m6_core_read_count;
	unsigned int m6_core_seq = 0;
	bool m6_core_dump = false;

	if (m6_core_read_count < 32) {
		m6_core_seq = ++m6_core_read_count;
		m6_core_dump = true;
	}
	if (m6_core_dump)
		dsi_m6_clkstate_marker("read-entry", module);

	for (d = DSI_MODULE_BEGIN(module); d <= DSI_MODULE_END(module); d++) {
		if (DSI_REG[d]->DSI_MODE_CTRL.MODE) {
			/* only support cmd mode read */
			DISPERR("DSI Read Fail: DSI Mode is %d\n",
				  DSI_REG[d]->DSI_MODE_CTRL.MODE);
			return 0;
		}

		if (buffer == NULL || buffer_size == 0) {
			/* illegal parameters */
			DISPERR("DSI Read Fail: buffer=%p and buffer_size=%d\n", buffer,
				  (unsigned int)buffer_size);
			return 0;
		}

		do {
			if (max_try_count == 0) {
				DISPERR("DSI Read Fail: try 5 times\n");
				return 0;
			}

			max_try_count--;
			recv_data_cnt = 0;
			/* read_timeout_ms = 20; */

			/* 1. wait dsi not busy => can't read if dsi busy */
			DSI_WaitForNotBusy(module, cmdq);

			/* 2. Check rd_rdy & cmd_done irq */
			if (DSI_REG[d]->DSI_INTEN.RD_RDY == 0) {
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN,
					      RD_RDY, 1);

			}
			if (DSI_REG[d]->DSI_INTEN.CMD_DONE == 0) {
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN,
					      CMD_DONE, 1);
			}
			if (DSI_REG[d]->DSI_INTSTA.RD_RDY != 0
			    || DSI_REG[d]->DSI_INTSTA.CMD_DONE != 0) {
				/* dump cmdq & rxdata */
				{
					unsigned int i;

					DISPMSG("Last DSI Read Why not clear irq???\n");
					DISPMSG("DSI_CMDQ_SIZE  : %d\n",
						  AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
					for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
						DISPMSG("DSI_CMDQ_DATA%d : 0x%08x\n", i,
							  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
					}
					DISPMSG("DSI_RX_DATA0   : 0x%08x\n",
						  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
					DISPMSG("DSI_RX_DATA1   : 0x%08x\n",
						  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
					DISPMSG("DSI_RX_DATA2   : 0x%08x\n",
						  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
					DISPMSG("DSI_RX_DATA3   : 0x%08x\n",
						  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
				}
				/* clear irq */
				DSI_OUTREGBIT(cmdq, DSI_INT_STATUS_REG, DSI_REG[d]->DSI_INTSTA,
					      RD_RDY, 0);
				DSI_OUTREGBIT(cmdq, DSI_INT_STATUS_REG, DSI_REG[d]->DSI_INTSTA,
					      CMD_DONE, 0);
			}
			/* 3. Send cmd */
			t0.CONFG = 0x04;	/* /BTA */
			/* / 0xB0 is used to distinguish DCS cmd or Gerneric cmd, is that Right??? */
			t0.Data_ID =
			    (cmd <
			     0xB0) ? DSI_DCS_READ_PACKET_ID : DSI_GERNERIC_READ_LONG_PACKET_ID;
			t0.Data0 = cmd;
			t0.Data1 = 0;
			/* set max return size */
			t1.CONFG = 0x00;
			t1.Data_ID = 0x37;
			t1.Data0 = buffer_size <= 10 ? buffer_size : 10;
			t1.Data1 = 0;

			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0], AS_UINT32(&t1));
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[1], AS_UINT32(&t0));
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2);

			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 0);
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 1);
			if (m6_core_dump)
				dsi_m6_clkstate_marker("read-start", module);

			/* / the following code is to */
			/* / 1: wait read ready */
			/* / 2: ack read ready */
			/* / 3: wait for CMDQ_DONE(interrupt handler do this op) */
			/* / 4: read data */
			/*
			 * SPI 194 does not fire, so waitRDDone is never set by
			 * the ISR; poll RD_RDY (bit0) in DSI_INTSTA instead.
			 * Return contract kept: > 0 = ready, 0 = timeout.
			 */
			{
				unsigned int c = 0;

				while (!waitRDDone &&
				       !(INREG32(&DSI_REG[d]->DSI_INTSTA) & 0x00000001)) {
					if (c++ > 200000)
						break;
					udelay(1);
				}
				ret = (waitRDDone ||
				       (INREG32(&DSI_REG[d]->DSI_INTSTA) & 0x00000001)) ? 1 : 0;
			}
			waitRDDone = false;
			if (m6_core_dump)
				dsi_m6_clkstate_marker("read-wait", module);
			if (m6_core_dump)
				DISPERR("M6 DSI core read wait #%u d=%d cmd=0x%x ret=%ld intsta=0x%08x trig=0x%08x start=0x%08x cmdq=0x%08x rx=%08x/%08x/%08x/%08x mode=%u busy=%u\n",
					m6_core_seq, d, cmd, ret,
					AS_UINT32(&DSI_REG[d]->DSI_INTSTA),
					AS_UINT32(&DSI_REG[d]->DSI_TRIG_STA),
					AS_UINT32(&DSI_REG[d]->DSI_START),
					AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE),
					AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0),
					AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1),
					AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2),
					AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3),
					DSI_REG[d]->DSI_MODE_CTRL.MODE,
					DSI_REG[d]->DSI_INTSTA.BUSY);
			if (ret > 0) {
				do {
					timeout++;
					udelay(1);
					DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK,
						      DSI_RACK, 1);
				} while (DSI_REG[d]->DSI_INTSTA.BUSY && (timeout < 1000));

				if (timeout == 1000) {
					/* wait cmd done timeout */
					DISPERR("DSI Read Fail: dsi wait cmd done timeout\n");
					DSI_DumpRegisters(module, 2);

					/* /do necessary reset here */
					DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK,
						      DSI_RACK, 1);
					DSI_Reset(module, NULL);

					/* clear rd rdy interrupt */
					DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG,
						      DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
					return 0;
				}
			} else if (ret == 0) {
				/* wait read ready timeout */
				DISPERR("DSI Read Fail: dsi wait read ready timeout\n");
				if (m6_core_dump)
					dsi_m6_clkstate_marker("read-timeout", module);
				DSI_DumpRegisters(module, 2);

				/* /do necessary reset here */
				DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK,
					      1);
				DSI_Reset(module, NULL);
				return 0;
			} else if (ret < 0) {
				/* wake up by other interrupt, need try again??? */
				DISPERR("DSI Read Fail: dsi wait read ready wake up by other interrupt\n");
				/* /do necessary reset here */
				DSI_OUTREGBIT(cmdq, DSI_RACK_REG, DSI_REG[d]->DSI_RACK, DSI_RACK,
					      1);
				DSI_Reset(module, NULL);
				return 0;
			}


			/* clear interrupt */
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTSTA, CMD_DONE,
				      0);

			/* read data */
			DSI_OUTREG32(cmdq, &read_data0, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
			DSI_OUTREG32(cmdq, &read_data1, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
			DSI_OUTREG32(cmdq, &read_data2, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
			DSI_OUTREG32(cmdq, &read_data3, AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));


			{
				unsigned int i;

				DISPMSG("DSI read begin i = %d --------------------\n",
					  5 - max_try_count);
				DISPMSG("DSI_RX_STA     : 0x%08x\n",
					  AS_UINT32(&DSI_REG[d]->DSI_TRIG_STA));
				DISPMSG("DSI_CMDQ_SIZE  : %d\n",
					  AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
				for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
					DISPMSG("DSI_CMDQ_DATA%d : 0x%08x\n", i,
						  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
				}
				DISPMSG("DSI_RX_DATA0   : 0x%08x\n",
					  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
				DISPMSG("DSI_RX_DATA1   : 0x%08x\n",
					  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
				DISPMSG("DSI_RX_DATA2   : 0x%08x\n",
					  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
				DISPMSG("DSI_RX_DATA3   : 0x%08x\n",
					  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
				DISPMSG("DSI read end ----------------------------\n");
			}

			packet_type = read_data0.byte0;

			DISPMSG("DSI read packet_type is 0x%x\n", packet_type);

			/* 0x02: acknowledge & error report */
			/* 0x11: generic short read response(1 byte return) */
			/* 0x12: generic short read response(2 byte return) */
			/* 0x1a: generic long read response */
			/* 0x1c: dcs long read response */
			/* 0x21: dcs short read response(1 byte return) */
			/* 0x22: dcs short read response(2 byte return) */
			if (packet_type == 0x1A || packet_type == 0x1C) {
				recv_data_cnt = read_data0.byte1 + read_data0.byte2 * 16;
				if (recv_data_cnt > 10) {
					DISPMSG
					    ("DSI read long packet data exceeds 4 bytes return size: %d\n",
					     recv_data_cnt);
					recv_data_cnt = 10;
				}

				if (recv_data_cnt > buffer_size) {
					DISPMSG
					    ("DSI read long packet data exceeds buffer size return size %d\n",
					     recv_data_cnt);
					recv_data_cnt = buffer_size;
				}
				DISPMSG("DSI read long packet size: %d\n", recv_data_cnt);

				if (recv_data_cnt <= 4) {
					memcpy((void *)buffer, (void *)&read_data1, recv_data_cnt);
				} else if (recv_data_cnt <= 8) {
					memcpy((void *)buffer, (void *)&read_data1, 4);
					memcpy((void *)((uint8_t *) buffer + 4), (void *)&read_data2,
					       recv_data_cnt - 4);
				} else {
					memcpy((void *)buffer, (void *)&read_data1, 4);
					memcpy((void *)((uint8_t *) buffer + 4), (void *)&read_data2, 4);
					memcpy((void *)((uint8_t *) buffer + 8), (void *)&read_data3,
					       recv_data_cnt - 8);
				}
			} else if (packet_type == 0x11 || packet_type == 0x12 ||
					packet_type == 0x21 || packet_type == 0x22) {
				if (packet_type == 0x11 || packet_type == 0x21)
					recv_data_cnt = 1;
				else
					recv_data_cnt = 2;
				if (recv_data_cnt > buffer_size) {
					DISPMSG
					    ("DSI read short packet data exceeds buffer size: %d\n",
					     buffer_size);
					recv_data_cnt = buffer_size;
					memcpy((void *)buffer, (void *)&read_data0.byte1,
					       recv_data_cnt);
				} else {
					memcpy((void *)buffer, (void *)&read_data0.byte1,
					       recv_data_cnt);
				}

			} else if (packet_type == 0x02) {
				DISPMSG("read return type is 0x02, re-read\n");
			} else {
				DISPMSG("read return type is non-recognite, type = 0x%x\n",
					  packet_type);
				if (m6_core_dump)
					DISPERR("M6 DSI core read packet #%u d=%d cmd=0x%x type=0x%x recv=%u retry_left=%u data=%02x %02x %02x %02x\n",
						m6_core_seq, d, cmd, packet_type,
						recv_data_cnt, max_try_count,
						buffer_size > 0 && buffer ? buffer[0] : 0,
						buffer_size > 1 && buffer ? buffer[1] : 0,
						buffer_size > 2 && buffer ? buffer[2] : 0,
						buffer_size > 3 && buffer ? buffer[3] : 0);
				return 0;
			}
			if (m6_core_dump)
				DISPERR("M6 DSI core read packet #%u d=%d cmd=0x%x type=0x%x recv=%u retry_left=%u data=%02x %02x %02x %02x\n",
					m6_core_seq, d, cmd, packet_type,
					recv_data_cnt, max_try_count,
					buffer_size > 0 && buffer ? buffer[0] : 0,
					buffer_size > 1 && buffer ? buffer[1] : 0,
					buffer_size > 2 && buffer ? buffer[2] : 0,
					buffer_size > 3 && buffer ? buffer[3] : 0);
		} while (packet_type == 0x02);
		/* / here: we may receive a ACK packet which packet type is 0x02 (incdicates some error happened) */
		/* / therefore we try re-read again until no ACK packet */
		/* / But: if it is a good way to keep re-trying ??? */
	}

	return recv_data_cnt;
}

void DSI_set_cmdq_V2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned cmd, unsigned char count,
		     unsigned char *para_list, unsigned char force_update)
{
	uint32_t i = 0;
	int d = 0;
	unsigned long goto_addr, mask_para, set_para;
	DSI_T0_INS t0;
	DSI_T2_INS t2;

	t2.pdata = NULL;
	/* DISPFUNC(); */
	for (d = DSI_MODULE_BEGIN(module); d <= DSI_MODULE_END(module); d++) {
		if (0 != DSI_REG[d]->DSI_MODE_CTRL.MODE) {	/* not in cmd mode */
			DSI_VM_CMD_CON_REG vm_cmdq;

			memset(&vm_cmdq, 0, sizeof(DSI_VM_CMD_CON_REG));
			DSI_READREG32(PDSI_VM_CMD_CON_REG, &vm_cmdq, &DSI_REG[d]->DSI_VM_CMD_CON);
			if (cmd < 0xB0) {
				if (count > 1) {
					vm_cmdq.LONG_PKT = 1;
					vm_cmdq.CM_DATA_ID = DSI_DCS_LONG_PACKET_ID;
					vm_cmdq.CM_DATA_0 = count + 1;
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
						     AS_UINT32(&vm_cmdq));

					goto_addr =
					    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para,
						      set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].
								    byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para,
							      set_para);
					}
				} else {
					vm_cmdq.LONG_PKT = 0;
					vm_cmdq.CM_DATA_0 = cmd;
					if (count) {
						vm_cmdq.CM_DATA_ID = DSI_DCS_SHORT_PACKET_ID_1;
						vm_cmdq.CM_DATA_1 = para_list[0];
					} else {
						vm_cmdq.CM_DATA_ID = DSI_DCS_SHORT_PACKET_ID_0;
						vm_cmdq.CM_DATA_1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
						     AS_UINT32(&vm_cmdq));
				}
			} else {
				if (count > 1) {
					vm_cmdq.LONG_PKT = 1;
					vm_cmdq.CM_DATA_ID = DSI_GERNERIC_LONG_PACKET_ID;
					vm_cmdq.CM_DATA_0 = count + 1;
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
						     AS_UINT32(&vm_cmdq));

					goto_addr =
					    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para,
						      set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].
								    byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para,
							      set_para);
					}
				} else {
					vm_cmdq.LONG_PKT = 0;
					vm_cmdq.CM_DATA_0 = cmd;
					if (count) {
						vm_cmdq.CM_DATA_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
						vm_cmdq.CM_DATA_1 = para_list[0];
					} else {
						vm_cmdq.CM_DATA_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
						vm_cmdq.CM_DATA_1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
						     AS_UINT32(&vm_cmdq));
				}
			}
			/* start DSI VM CMDQ */
			if (force_update)
				DSI_EnableVM_CMD(module, cmdq);
		} else {
			DSI_WaitForNotBusy(module, cmdq);

			if (cmd < 0xB0) {
				if (count > 1) {
					/*DSI_CMDQ cmd_data;*/

					t2.CONFG = 2;
					t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
					t2.WC16 = count + 1;

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						     AS_UINT32(&t2));

					goto_addr =
					    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].byte0);
					mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
					set_para = (cmd << ((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~((unsigned long)0x3u)),
						      mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].
								    byte1) + i;
						mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3u) * 8));
						DSI_MASKREG32(cmdq,
							      goto_addr & (~((unsigned long)0x3u)),
							      mask_para, set_para);
					}

					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
						     2 + (count) / 4);
				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
						t0.Data1 = 0;
					}

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						     AS_UINT32(&t0));
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
				}
			} else {
				if (count > 1) {
					t2.CONFG = 2;
					t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
					t2.WC16 = count + 1;

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						     AS_UINT32(&t2));

					goto_addr =
					    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].byte0);
					mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
					set_para = (cmd << ((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~((unsigned long)0x3u)),
						      mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].
								    byte1) + i;
						mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3u) * 8));
						DSI_MASKREG32(cmdq,
							      goto_addr & (~((unsigned long)0x3u)),
							      mask_para, set_para);
					}

					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
						     2 + (count) / 4);

				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
						t0.Data1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						     AS_UINT32(&t0));
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
				}
			}
			if (force_update) {
				DSI_Start(module, cmdq);
				DSI_WaitForNotBusy(module, cmdq);
			}
		}
	}
}


void DSI_set_cmdq_V3(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, LCM_setting_table_V3 *para_tbl,
		     unsigned int size, unsigned char force_update)
{

	uint32_t i;
	/* UINT32 layer, layer_state, lane_num; */
	unsigned long goto_addr, mask_para, set_para;
	/* UINT32 fbPhysAddr, fbVirAddr; */
	DSI_T0_INS t0;
	/* DSI_T1_INS t1; */
	DSI_T2_INS t2;

	uint32_t index = 0;

	unsigned char data_id, cmd, count;
	unsigned char *para_list;

	uint32_t d;

	memset(&t2, 0, sizeof(t2));

	for (d = DSI_MODULE_BEGIN(module); d <= DSI_MODULE_END(module); d++) {
		do {
			data_id = para_tbl[index].id;
			cmd = para_tbl[index].cmd;
			count = para_tbl[index].count;
			para_list = para_tbl[index].para_list;

			if (data_id == REGFLAG_ESCAPE_ID && cmd == REGFLAG_DELAY_MS_V3) {
				udelay(1000 * count);
				DISPMSG("DISP/DSI " "DSI_set_cmdq_V3[%d]. Delay %d (ms)\n", index,
					count);

				continue;
			}
			if (0 != DSI_REG[d]->DSI_MODE_CTRL.MODE) {	/* not in cmd mode */
				DSI_VM_CMD_CON_REG vm_cmdq;

				OUTREG32(&vm_cmdq, AS_UINT32(&DSI_REG[d]->DSI_VM_CMD_CON));
				DISPMSG("set cmdq in VDO mode\n");
				if (count > 1) {
					vm_cmdq.LONG_PKT = 1;
					vm_cmdq.CM_DATA_ID = data_id;
					vm_cmdq.CM_DATA_0 = count + 1;
					OUTREG32(&DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));

					goto_addr =
					    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].byte0);
					mask_para = (0xFF << ((goto_addr & 0x3) * 8));
					set_para = (cmd << ((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_VM_CMD_REG[d]->data[0].
								    byte1) + i;
						mask_para = (0xFF << ((goto_addr & 0x3) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3) * 8));
						DSI_MASKREG32(cmdq, goto_addr & (~0x3), mask_para, set_para);
					}
				} else {
					vm_cmdq.LONG_PKT = 0;
					vm_cmdq.CM_DATA_0 = cmd;
					if (count) {
						vm_cmdq.CM_DATA_ID = data_id;
						vm_cmdq.CM_DATA_1 = para_list[0];
					} else {
						vm_cmdq.CM_DATA_ID = data_id;
						vm_cmdq.CM_DATA_1 = 0;
					}
					OUTREG32(&DSI_REG[d]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
				}
				/* start DSI VM CMDQ */
				if (force_update)
					DSI_EnableVM_CMD(module, cmdq);
			} else {
				DSI_WaitForNotBusy(module, cmdq);

				OUTREG32(&DSI_CMDQ_REG[d]->data[0], 0);

				if (count > 1) {
					t2.CONFG = 2;
					t2.Data_ID = data_id;
					t2.WC16 = count + 1;

					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0].byte0,
						     AS_UINT32(&t2));

					goto_addr =
					    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].byte0);
					mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
					set_para = (cmd << ((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~((unsigned long)0x3u)),
						      mask_para, set_para);

					for (i = 0; i < count; i++) {
						goto_addr =
						    (unsigned long)(&DSI_CMDQ_REG[d]->data[1].
								    byte1) + i;
						mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
						set_para =
						    (para_list[i] << ((goto_addr & 0x3u) * 8));
						DSI_MASKREG32(cmdq,
							      goto_addr & (~((unsigned long)0x3u)),
							      mask_para, set_para);
					}

					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
						     2 + (count) / 4);
				} else {
					t0.CONFG = 0;
					t0.Data0 = cmd;
					if (count) {
						t0.Data_ID = data_id;
						t0.Data1 = para_list[0];
					} else {
						t0.Data_ID = data_id;
						t0.Data1 = 0;
					}
					DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						     AS_UINT32(&t0));
					DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
				}

				if (force_update) {
					DSI_Start(module, cmdq);
					DSI_WaitForNotBusy(module, cmdq);
				}
			}
		} while (++index < size);
	}

}

void DSI_set_cmdq(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int *pdata,
		  unsigned int queue_size, unsigned char force_update)
{
	/* DISPFUNC(); */

	int j = 0;
	int i = 0;

	/* DISPMSG("DSI_set_cmdq, module=%s, cmdq=0x%08x\n", ddp_get_module_name(module), cmdq); */

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (0 != DSI_REG[i]->DSI_MODE_CTRL.MODE) {
#if 0
			/* not in cmd mode */
			DSI_VM_CMD_CON_REG vm_cmdq;

			OUTREG32(&vm_cmdq, AS_UINT32(&DSI_REG[i]->DSI_VM_CMD_CON));
			DISPMSG("set cmdq in VDO mode\n");
			if (queue_size > 1) {	/* long packet */
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = ((pdata[0] >> 8) & 0xFF);
				vm_cmdq.CM_DATA_0 = ((pdata[0] >> 16) & 0xFF);
				vm_cmdq.CM_DATA_1 = 0;
				OUTREG32(&DSI_REG[i]->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
				for (j = 0; j < queue_size - 1; j++) {
					OUTREG32(&DSI_VM_CMD_REG->data[j],
						 AS_UINT32((pdata + j + 1)));
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_ID = ((pdata[0] >> 8) & 0xFF);
				vm_cmdq.CM_DATA_0 = ((pdata[0] >> 16) & 0xFF);
				vm_cmdq.CM_DATA_1 = ((pdata[0] >> 24) & 0xFF);
				OUTREG32(&DSI_REG->DSI_VM_CMD_CON, AS_UINT32(&vm_cmdq));
			}
			/* start DSI VM CMDQ */
			if (force_update) {
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagStart,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[0]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[1]));
				DSI_EnableVM_CMD();

				/* must wait VM CMD done? */
				MMProfileLogEx(MTKFB_MMP_Events.DSICmd, MMProfileFlagEnd,
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[2]),
					       *(unsigned int *)(&DSI_VM_CMD_REG->data[3]));
			}
#endif
		} else {
			ASSERT(queue_size <= 32);
			DSI_WaitForNotBusy(module, cmdq);
#ifdef ENABLE_DSI_ERROR_REPORT
			if ((pdata[0] & 1)) {
				memcpy(_dsi_cmd_queue, pdata, queue_size * 4);
				_dsi_cmd_queue[queue_size++] = 0x4;
				pdata = (unsigned int *)_dsi_cmd_queue;
			} else {
				pdata[0] |= 4;
			}
#endif
			for (j = 0; j < queue_size; j++) {
				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[j],
					     AS_UINT32((pdata + j)));
			}

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE, queue_size);

			if (force_update) {
				DSI_Start(module, cmdq);
				DSI_WaitForNotBusy(module, cmdq);
			}
		}
	}
}

void _copy_dsi_params(LCM_DSI_PARAMS *src, LCM_DSI_PARAMS *dst)
{
	memcpy((LCM_DSI_PARAMS *) dst, (LCM_DSI_PARAMS *) src, sizeof(LCM_DSI_PARAMS));
}

int DSI_Send_ROI(DISP_MODULE_ENUM module, void *handle, unsigned int x, unsigned int y,
		 unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	DSI_set_cmdq(module, handle, data_array, 3, 1);
	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	DSI_set_cmdq(module, handle, data_array, 3, 1);
	DISPDBG("DSI_Send_ROI Done!\n");

	/* data_array[0]= 0x002c3909; */
	/* DSI_set_cmdq(module, handle, data_array, 1, 0); */
	return 0;
}



static void lcm_set_reset_pin(uint32_t value)
{
#if 0
	DSI_OUTREG32(NULL, DISPSYS_CONFIG_BASE + 0x150, value);
#else
#if !defined(CONFIG_MTK_LEGACY)
	if (value)
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	else
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
#endif
#endif
}

static void lcm_udelay(uint32_t us)
{
	udelay(us);
}

static void lcm_mdelay(uint32_t ms)
{
	if (ms < 10)
		udelay(ms * 1000);
	else
		msleep(ms);
}

void DSI_set_cmdq_V2_DSI0(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
			  unsigned char force_update)
{
	/*cmdqRecHandle cmdq_hd = (cmdqRecHandle) cmdq;*/
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_DSI1(void *cmdq, unsigned cmd, unsigned char count, unsigned char *para_list,
			  unsigned char force_update)
{
	/*cmdqRecHandle cmdq_hd = (cmdqRecHandle) cmdq;*/
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_DSIDual(void *cmdq, unsigned cmd, unsigned char count,
			     unsigned char *para_list, unsigned char force_update)
{
	/*cmdqRecHandle cmdq_hd = (cmdqRecHandle) cmdq;*/
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, cmdq, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSI0(unsigned cmd, unsigned char count, unsigned char *para_list,
				  unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSI1(unsigned cmd, unsigned char count, unsigned char *para_list,
				  unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSIDual(unsigned cmd, unsigned char count, unsigned char *para_list,
				     unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, NULL, cmd, count, para_list, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI0(LCM_setting_table_V3 *para_tbl, unsigned int size,
				  unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI0, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI1(LCM_setting_table_V3 *para_tbl, unsigned int size,
				  unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI1, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSIDual(LCM_setting_table_V3 *para_tbl, unsigned int size,
				     unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSIDUAL, NULL, para_tbl, size, force_update);
}

void DSI_set_cmdq_wrapper_DSI0(unsigned int *pdata, unsigned int queue_size,
			       unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI0, NULL, pdata, queue_size, force_update);
}

void DSI_set_cmdq_wrapper_DSI1(unsigned int *pdata, unsigned int queue_size,
			       unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI1, NULL, pdata, queue_size, force_update);
}

void DSI_set_cmdq_wrapper_DSIDual(unsigned int *pdata, unsigned int queue_size,
				  unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSIDUAL, NULL, pdata, queue_size, force_update);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI0(uint8_t cmd, uint8_t *buffer, uint8_t buffer_size)
{
	unsigned int ret;
	static unsigned int m6_wrapper_count;
	bool m6_dump = false;

	if (m6_wrapper_count < 64) {
		m6_dump = true;
		m6_wrapper_count++;
		DISPERR("M6 DSI wrapper read begin #%u cmd=0x%x size=%u buffer=%p\n",
			m6_wrapper_count, cmd, buffer_size, buffer);
	}
	ret = DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, cmd, buffer, buffer_size);
	if (m6_dump)
		DISPERR("M6 DSI wrapper read end #%u cmd=0x%x ret=%u data=%02x %02x %02x %02x\n",
			m6_wrapper_count, cmd, ret,
			buffer_size > 0 && buffer ? buffer[0] : 0,
			buffer_size > 1 && buffer ? buffer[1] : 0,
			buffer_size > 2 && buffer ? buffer[2] : 0,
			buffer_size > 3 && buffer ? buffer[3] : 0);
	return ret;
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI1(uint8_t cmd, uint8_t *buffer, uint8_t buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI1, NULL, cmd, buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSIDUAL(uint8_t cmd, uint8_t *buffer, uint8_t buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSIDUAL, NULL, cmd, buffer, buffer_size);
}
/*
long lcd_enp_bias_setting(unsigned int value)
{
	long ret = 0;

#if !defined(CONFIG_MTK_LEGACY)
	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN);
#endif
	return ret;
}
*/
extern void lcm_pinctl_gpio_output(int pin, int level);
long lcd_enp_bias_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		lcm_pinctl_gpio_output (0, 1);
	else
		lcm_pinctl_gpio_output (0, 0);


	return ret;
}
long lcd_enn_bias_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		lcm_pinctl_gpio_output (1, 1);
	else
		lcm_pinctl_gpio_output (1, 0);


	return ret;
}


static void lcm_reset_settting(unsigned int value)
{

	if (value)
		lcm_pinctl_gpio_output (2, 1);
	else
		lcm_pinctl_gpio_output (2, 0);

}

int ddp_dsi_set_lcm_utils(DISP_MODULE_ENUM module, LCM_DRIVER *lcm_drv)
{
	LCM_UTIL_FUNCS *utils = NULL;

	if (lcm_drv == NULL) {
		DISPERR("lcm_drv is null\n");
		return -1;
	}

	if (module == DISP_MODULE_DSI0) {
		utils = (LCM_UTIL_FUNCS *)&lcm_utils_dsi0;
	} else {
		DISPERR("wrong module: %d\n", module);
		return -1;
	}

	utils->set_reset_pin = lcm_set_reset_pin;
	utils->udelay = lcm_udelay;
	utils->mdelay = lcm_mdelay;
	if (module == DISP_MODULE_DSI0) {
		utils->dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0;
		utils->dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0;
		utils->dsi_set_cmdq_V3 = DSI_set_cmdq_V3_Wrapper_DSI0;
		utils->dsi_dcs_read_lcm_reg_v2 = DSI_dcs_read_lcm_reg_v2_wrapper_DSI0;
		utils->dsi_set_cmdq_V22 = DSI_set_cmdq_V2_DSI0;
	}
/*
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	//utils->set_gpio_out = mt_set_gpio_out;
	//utils->set_gpio_mode = mt_set_gpio_mode;
	//utils->set_gpio_dir = mt_set_gpio_dir;
	//utils->set_gpio_pull_enable = (int (*)(unsigned int, unsigned char))mt_set_gpio_pull_enable;
	
#else
	utils->set_gpio_lcd_enp_bias = lcd_enp_bias_setting;
#endif
#endif
*/
	utils->set_gpio_lcd_enp_bias = lcd_enp_bias_setting;
	utils->set_gpio_lcd_enn_bias = lcd_enn_bias_setting;
	utils->set_reset_pin = lcm_reset_settting;
	lcm_drv->set_util_funcs(utils);

	return 0;
}

void DSI_ChangeClk(DISP_MODULE_ENUM module, uint32_t clk)
{
	int i = 0;

	if (clk > 1250 || clk < 50)
		return;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		LCM_DSI_PARAMS *dsi_params = &_dsi_context[i].dsi_params;

		dsi_params->PLL_CLOCK = clk;
		DSI_WaitForNotBusy(module, NULL);
		DSI_PHY_clk_setting(module, NULL, dsi_params);
		DSI_PHY_TIMCONFIG(module, NULL, dsi_params);
	}
}

int ddp_dsi_init(DISP_MODULE_ENUM module, void *cmdq)
{
	DSI_STATUS ret = DSI_STATUS_OK;
	int i = 0;
	static unsigned int dump_count;
#ifndef CONFIG_FPGA_EARLY_PORTING
	int mipitx_enabled = 0;
#endif

	pr_emerg("[FORGE_DISP] ddp_dsi_init ENTRY module=%d\n", module);
	DISPFUNC();
	/* DSI_OUTREG32(cmdq, 0xf0000048, 0x80000000); */
	/* DSI_OUTREG32(cmdq, MMSYS_CONFIG_BASE+0x108, 0xffffffff); */
	/* DSI_OUTREG32(cmdq, MMSYS_CONFIG_BASE+0x118, 0xffffffff); */
	/* DSI_OUTREG32(MMSYS_CONFIG_BASE+0xC08, 0xffffffff); */
#ifdef ENABLE_CLK_MGR
#ifndef CONFIG_MTK_CLKMGR
	ddp_parse_apmixed_base();
#endif
#endif
	DSI_REG[0] = (DSI_REGS *) DISPSYS_DSI0_BASE;
	DSI_PHY_REG[0] = (DSI_PHY_REGS *) MIPITX_BASE;
	DSI_CMDQ_REG[0] = (DSI_CMDQ_REGS *) (DISPSYS_DSI0_BASE + 0x200);
	DSI_REG[1] = (DSI_REGS *) DISPSYS_DSI0_BASE;
	DSI_PHY_REG[1] = (DSI_PHY_REGS *) MIPITX_BASE;
	DSI_CMDQ_REG[1] = (DSI_CMDQ_REGS *) (DISPSYS_DSI0_BASE + 0x200);
	DSI_VM_CMD_REG[0] = (DSI_VM_CMDQ_REGS *) (DISPSYS_DSI0_BASE + 0x134);
	DSI_VM_CMD_REG[1] = (DSI_VM_CMDQ_REGS *) (DISPSYS_DSI0_BASE + 0x134);
	memset(&_dsi_context, 0, sizeof(_dsi_context));

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		init_waitqueue_head(&_dsi_cmd_done_wait_queue[i]);
		init_waitqueue_head(&_dsi_dcs_read_wait_queue[i]);
		init_waitqueue_head(&_dsi_wait_bta_te[i]);
		init_waitqueue_head(&_dsi_wait_ext_te[i]);
		init_waitqueue_head(&_dsi_wait_vm_done_queue[i]);
		init_waitqueue_head(&_dsi_wait_vm_cmd_done_queue[i]);
		init_waitqueue_head(&_dsi_wait_sleep_out_done_queue[i]);
		DISPMSG("dsi%d initializing\n", i);
	}

	dsi_m6_lkgold_snapshot_once("pre-init");
	disp_register_module_irq_callback(DISP_MODULE_DSI0, _DSI_INTERNAL_IRQ_Handler);

#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_rt_cal("init-before-is-enabled");
	mipitx_enabled = MIPITX_IsEnabled(module, cmdq);
	DISPERR("M6 DSI mipitx-decision[init]: enabled=%d PMaster=%d force=%d\n",
		mipitx_enabled, atomic_read(&PMaster_enable), dsi_force_config);
	dsi_m6_dump_rt_cal("init-after-is-enabled");
	if (mipitx_enabled) {
		s_isDsiPowerOn = true;
#ifdef ENABLE_CLK_MGR
#ifdef CONFIG_MTK_CLKMGR
		set_mipi26m(1);
#else
		ddp_set_mipi26m(1);
#endif
		if (module == DISP_MODULE_DSI0) {
#ifdef CONFIG_MTK_CLKMGR
			ret += enable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI");
			ret += enable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI");
#else
			ret += ddp_clk_enable(DISP1_DSI_ENGINE);
			ret += ddp_clk_enable(DISP1_DSI_DIGITAL);
#endif
			if (ret > 0)
				DISPMSG("DSI0 power manager API return false\n");
		}
#endif
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, CMD_DONE, 1);
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, RD_RDY, 1);
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, VM_DONE, 1);
		/* enable te_rdy when need, not here (both cmd mode & vdo mode) */
		/* DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG,DSI_REG[0]->DSI_INTEN,TE_RDY,1); */
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, VM_CMD_DONE, 0);
		DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, SLEEPOUT_DONE, 1);
		/* DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG,DSI_REG[0]->DSI_INTEN,FRAME_DONE_INT_EN,0); */
		DSI_BackupRegisters(module, NULL);
		clock_lane = (INREG32(MIPI_TX_REG_BASE + 0x4));/*MIPITX_DSI_CLOCK_LANE*/
		data_lane3 = (INREG32(MIPI_TX_REG_BASE + 0x14));/*MIPITX_DSI_DATA_LANE3*/
		data_lane2 = (INREG32(MIPI_TX_REG_BASE + 0x10));/*MIPITX_DSI_DATA_LANE2*/
		data_lane1 = (INREG32(MIPI_TX_REG_BASE + 0xc));/*MIPITX_DSI_DATA_LANE1*/
			data_lane0 = (INREG32(MIPI_TX_REG_BASE + 0x8));/*MIPITX_DSI_DATA_LANE0*/
			DISPMSG("clk=0x%x,lan3=0x%x,lan2=0x%x,lan1=0x%x,lan0=0x%x\n",
				clock_lane, data_lane3, data_lane2, data_lane1, data_lane0);
			dsi_m6_dump_rt_cal("init-after-save-lanes");
			dsi_m6_dump_snapshot_limited("init-after", module, cmdq, &dump_count, 2);
		}
#endif

	return DSI_STATUS_OK;
}

int ddp_dsi_deinit(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	return 0;
}

void _dump_dsi_params(LCM_DSI_PARAMS *dsi_config)
{

	if (dsi_config) {
		switch (dsi_config->mode) {
		case CMD_MODE:
			DISPDBG("[DDPDSI] DSI Mode: CMD_MODE\n");
			break;
		case SYNC_PULSE_VDO_MODE:
			DISPDBG("[DDPDSI] DSI Mode: SYNC_PULSE_VDO_MODE\n");
			break;
		case SYNC_EVENT_VDO_MODE:
			DISPDBG("[DDPDSI] DSI Mode: SYNC_EVENT_VDO_MODE\n");
			break;
		case BURST_VDO_MODE:
			DISPDBG("[DDPDSI] DSI Mode: BURST_VDO_MODE\n");
			break;
		default:
			DISPMSG("[DDPDSI] DSI Mode: Unknown\n");
			break;
		}

		DISPDBG
		    ("[DDPDSI] vact: %d, vbp: %d, vfp: %d, vact_line: %d, hact: %d, hbp: %d, hfp: %d, hblank: %d\n",
		     dsi_config->vertical_sync_active, dsi_config->vertical_backporch,
		     dsi_config->vertical_frontporch, dsi_config->vertical_active_line,
		     dsi_config->horizontal_sync_active, dsi_config->horizontal_backporch,
		     dsi_config->horizontal_frontporch, dsi_config->horizontal_blanking_pixel);
		DISPDBG
		    ("[DDPDSI] pll_select: %d, pll_div1: %d, pll_div2: %d, fbk_div: %d,fbk_sel: %d, rg_bir: %d\n",
		     dsi_config->pll_select, dsi_config->pll_div1, dsi_config->pll_div2,
		     dsi_config->fbk_div, dsi_config->fbk_sel, dsi_config->rg_bir);
		DISPDBG("[DDPDSI] rg_bic: %d, rg_bp: %d, PLL_CLOCK: %d, dsi_clock: %d, ssc_range: %d\n",
		     dsi_config->rg_bic, dsi_config->rg_bp, dsi_config->PLL_CLOCK,
		     dsi_config->dsi_clock, dsi_config->ssc_range);

		DISPDBG("[DDPDSI] ssc_disable: %d, compatibility_for_nvk: %d, cont_clock: %d\n",
		     dsi_config->ssc_disable, dsi_config->compatibility_for_nvk, dsi_config->cont_clock);

		DISPDBG
		    ("[DDPDSI] lcm_ext_te_enable: %d, noncont_clock: %d, noncont_clock_period: %d\n",
		     dsi_config->lcm_ext_te_enable, dsi_config->noncont_clock,
		     dsi_config->noncont_clock_period);
	}
}

static void DSI_PHY_CLK_LP_PerLine_config(DISP_MODULE_ENUM module, cmdqRecHandle cmdq,
					  LCM_DSI_PARAMS *dsi_params)
{
	int i;
	DSI_PHY_TIMCON0_REG timcon0 = {0};	/* LPX */
	DSI_PHY_TIMCON2_REG timcon2 = {0};	/* CLK_HS_TRAIL, CLK_HS_ZERO */
	DSI_PHY_TIMCON3_REG timcon3 = {0};	/* CLK_HS_EXIT, CLK_HS_POST, CLK_HS_PREP */
	DSI_HSA_WC_REG hsa = {0};
	DSI_HBP_WC_REG hbp = {0};
	DSI_HFP_WC_REG hfp = {0}, new_hfp = {0};
	DSI_BLLP_WC_REG bllp = {0};
	DSI_PSCTRL_REG ps = {0};
	uint32_t hstx_ckl_wc = 0, new_hstx_ckl_wc = 0;
	uint32_t v_a, v_b, v_c, lane_num;
	LCM_DSI_MODE_CON dsi_mode;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		lane_num = dsi_params->LANE_NUM;
		dsi_mode = dsi_params->mode;

		if (dsi_mode == CMD_MODE)
			continue;
		/* vdo mode */
		DISPERR("M6 DSI lp_per_line[before]: enable=%u mode=%u HSA/HBP/HFP/BLLP/HSTX=0x%x/0x%x/0x%x/0x%x/0x%x\n",
			dsi_params->clk_lp_per_line_enable, dsi_mode,
			INREG32(DDP_REG_BASE_DSI0 + 0x050),
			INREG32(DDP_REG_BASE_DSI0 + 0x054),
			INREG32(DDP_REG_BASE_DSI0 + 0x058),
			INREG32(DDP_REG_BASE_DSI0 + 0x05c),
			INREG32(DDP_REG_BASE_DSI0 + 0x064));
		AS_UINT32(&hsa) = INREG32(&DSI_REG[i]->DSI_HSA_WC);
		AS_UINT32(&hbp) = INREG32(&DSI_REG[i]->DSI_HBP_WC);
		AS_UINT32(&hfp) = INREG32(&DSI_REG[i]->DSI_HFP_WC);
		AS_UINT32(&bllp) = INREG32(&DSI_REG[i]->DSI_BLLP_WC);
		AS_UINT32(&ps) = INREG32(&DSI_REG[i]->DSI_PSCTRL);
		AS_UINT32(&hstx_ckl_wc) = INREG32(&DSI_REG[i]->DSI_HSTX_CKL_WC);
		AS_UINT32(&timcon0) = INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0);
		AS_UINT32(&timcon2) = INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2);
		AS_UINT32(&timcon3) = INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3);

		/* 1. sync_pulse_mode */
		/* Total    WC(A) = HSA_WC + HBP_WC + HFP_WC + PS_WC + 32 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		if (dsi_mode == SYNC_PULSE_VDO_MODE) {
			v_a = hsa.HSA_WC + hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + 32;
			v_b =
			    (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR +
			     timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPMSG("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
/* DISPMSG("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPMSG("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			AS_UINT32(&new_hfp) = cmdq ? (v_b + v_c + DIFF_CLK_LANE_LP) :
						      INREG32(&DSI_REG[i]->DSI_HFP_WC);
			v_a = hsa.HSA_WC + hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			new_hstx_ckl_wc = cmdq ? (v_a - v_b) : INREG32(&DSI_REG[i]->DSI_HSTX_CKL_WC);
			DISPMSG("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc,
				  new_hfp.HFP_WC);
		}
		/* 2. sync_event_mode */
		/* Total    WC(A) = HBP_WC + HFP_WC + PS_WC + 26 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		else if (dsi_mode == SYNC_EVENT_VDO_MODE) {
			v_a = hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + 26;
			v_b =
			    (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR +
			     timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPMSG("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
/* DISPMSG("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPMSG("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			AS_UINT32(&new_hfp) = cmdq ? (v_b + v_c + DIFF_CLK_LANE_LP) :
						      INREG32(&DSI_REG[i]->DSI_HFP_WC);
			v_a = hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + 26;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			new_hstx_ckl_wc = cmdq ? (v_a - v_b) : INREG32(&DSI_REG[i]->DSI_HSTX_CKL_WC);
			DISPMSG("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc,
				  new_hfp.HFP_WC);

		}
		/* 3. burst_mode */
		/* Total    WC(A) = HBP_WC + HFP_WC + PS_WC + BLLP_WC + 32 */
		/* CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP + CLK_HS_ZERO)*lane_num */
		/* CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL)*lane_num */
		/* HSTX_CKLP_WC = A - B */
		/* Limitation: B + C < HFP_WC */
		else if (dsi_mode == BURST_VDO_MODE) {
			v_a = hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			v_b =
			    (timcon3.CLK_HS_EXIT + timcon0.LPX + timcon3.CLK_HS_PRPR +
			     timcon2.CLK_ZERO) * lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) * lane_num;

			DISPMSG("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n", (v_a - v_b), hstx_ckl_wc);
			/* DISPMSG("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",(v_b+v_c),hfp); */
			DISPMSG("===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC, (v_b + v_c + DIFF_CLK_LANE_LP));
			AS_UINT32(&new_hfp) = cmdq ? (v_b + v_c + DIFF_CLK_LANE_LP) :
						      INREG32(&DSI_REG[i]->DSI_HFP_WC);
			v_a = hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC, (v_a - v_b));
			new_hstx_ckl_wc = cmdq ? (v_a - v_b) : INREG32(&DSI_REG[i]->DSI_HSTX_CKL_WC);
				DISPMSG("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n", new_hstx_ckl_wc,
					  new_hfp.HFP_WC);
			}
			DISPERR("M6 DSI lp_per_line[after]: enable=%u mode=%u HSA/HBP/HFP/BLLP/HSTX=0x%x/0x%x/0x%x/0x%x/0x%x\n",
				dsi_params->clk_lp_per_line_enable, dsi_mode,
				INREG32(DDP_REG_BASE_DSI0 + 0x050),
				INREG32(DDP_REG_BASE_DSI0 + 0x054),
				INREG32(DDP_REG_BASE_DSI0 + 0x058),
				INREG32(DDP_REG_BASE_DSI0 + 0x05c),
				INREG32(DDP_REG_BASE_DSI0 + 0x064));
		}

	}

int ddp_dsi_config(DISP_MODULE_ENUM module, disp_ddp_path_config *config, void *cmdq)
{
	int i = 0;
	LCM_DSI_PARAMS *dsi_config = &(config->dispif_config.dsi);
	static unsigned int dump_count;
	static unsigned int m6_force_first_lk_mipitx_config_done;
	static unsigned int m6_boot_pll_reprog_done;
#ifndef CONFIG_FPGA_EARLY_PORTING
	int mipitx_enabled = 0;
#endif

	if (!config->dst_dirty) {
		if (atomic_read(&PMaster_enable) == 0)
			return 0;
	}
	DISPFUNC();
	DISPDBG("===>run here 00 Pmaster: clk:%d\n", _dsi_context[0].dsi_params.PLL_CLOCK);
	dsi_m6_takeover_hold_once("config-entry", module, cmdq, &dump_count);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		_copy_dsi_params(dsi_config, &(_dsi_context[i].dsi_params));
		_dsi_context[i].lcm_width = config->dst_w;
		_dsi_context[i].lcm_height = config->dst_h;
		_dump_dsi_params(&(_dsi_context[i].dsi_params));
		if (dsi_config->mode != CMD_MODE) {
			/* not enable TE in vdo mode */
			if (dsi_config->eint_disable == 1) {
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN,
					      TE_RDY, 1);
				DISPDBG("DSI VDO Mode TEINT On\n");
			} else {
				DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN,
										  TE_RDY, 0);
			}
		} else {
			/*enable TE in cmd mode */
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
		}
	}
	DISPDBG("===>01Pmaster: clk:%d\n", _dsi_context[0].dsi_params.PLL_CLOCK);
	if (dsi_config->mode != CMD_MODE)
		dsi_currect_mode = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_rt_cal("config-before-is-enabled");
	mipitx_enabled = MIPITX_IsEnabled(module, cmdq);
	DISPERR("M6 DSI mipitx-decision[config]: enabled=%d PMaster=%d force=%d clk_lp_per_line=%u\n",
		mipitx_enabled, atomic_read(&PMaster_enable), dsi_force_config,
		dsi_config->clk_lp_per_line_enable);
	dsi_m6_dump_rt_cal("config-after-is-enabled");
	if (M6_LK_HANDOFF_SKIP_FIRST_DSI_CONFIG &&
	    atomic_read(&PMaster_enable) == 0 && !dsi_force_config) {
		DISPERR("M6 DSI lk-handoff[config]: skip first DSI reconfig to preserve LK bootlogo state enabled=%d\n",
			mipitx_enabled);
		dsi_m6_sram_snapshot("lk-handoff-config-skip", module);
		dsi_m6_dump_snapshot_limited("lk-handoff-config-skip", module,
					     cmdq, &dump_count, 4);
		goto done;
	}
	if ((mipitx_enabled) && (atomic_read(&PMaster_enable) == 0)) {
		DISPDBG("mipitx is already init\n");
		if (M6_FORCE_FIRST_DSI_CONFIG_ON_LK_MIPITX &&
		    !m6_force_first_lk_mipitx_config_done &&
		    !dsi_force_config) {
			m6_force_first_lk_mipitx_config_done = 1;
			DISPERR("M6 DSI mipitx-decision[config-force-first]: enabled=%d PMaster=%d replay_phy=1 cmdq=%p\n",
				mipitx_enabled, atomic_read(&PMaster_enable),
				cmdq);
			dsi_m6_sram_snapshot("config-force-first", module);
			dsi_m6_dump_snapshot_limited("config-force-before",
						     module, cmdq,
						     &dump_count, 4);
			DSI_PHY_clk_setting(module, NULL, dsi_config);
			dsi_m6_dump_rt_cal("config-force-after-phy-clk-setting");
			dsi_m6_dump_snapshot_limited("config-force-after-phy",
						     module, cmdq,
						     &dump_count, 6);
			goto force_config;
		} else if (dsi_force_config)
			goto force_config;
		else {
			if (M6_BOOT_PLL_REPROG && !m6_boot_pll_reprog_done) {
				m6_boot_pll_reprog_done = 1;
				pr_err("M6BOOTPLL: reprog PLL from LK handoff, target pll=%u data_rate=%u pcw_before=0x%x\n",
					dsi_config->PLL_CLOCK, dsi_config->PLL_CLOCK * 2,
					DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG, &DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON2));
				DSI_PHY_clk_change(module, NULL, dsi_config);
				DSI_PHY_TIMCONFIG(module, NULL, dsi_config);
				pr_err("M6BOOTPLL: reprog done, pcw_after=0x%x\n",
					DSI_INREG32(PMIPITX_DSI_PLL_CON2_REG, &DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON2));
			}
			goto done;
		}
	} else
#endif
	{
		DISPMSG("MIPITX is not inited, will config mipitx clock now\n");
		DISPMSG("===>Pmaster:CLK SETTING??==> clk:%d\n",
			  _dsi_context[0].dsi_params.PLL_CLOCK);
		DSI_PHY_clk_setting(module, NULL, dsi_config);
		dsi_m6_dump_rt_cal("config-after-phy-clk-setting");
	}

force_config:
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (dsi_config->mode == CMD_MODE
		    || ((dsi_config->switch_mode_enable == 1)
			&& (dsi_config->switch_mode == CMD_MODE))
		    || (dsi_config->mode != CMD_MODE && dsi_config->eint_disable))
			DSI_OUTREGBIT(cmdq, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
	}
	/* DSI_Reset(module, cmdq_handle); */
	DSI_TXRX_Control(module, cmdq, dsi_config);
	DSI_PS_Control(module, cmdq, dsi_config, config->dst_w, config->dst_h);
	DSI_PHY_TIMCONFIG(module, cmdq, dsi_config);

	if (dsi_config->mode != CMD_MODE
	    || ((dsi_config->switch_mode_enable == 1) && (dsi_config->switch_mode != CMD_MODE))) {
		DSI_Config_VDO_Timing(module, cmdq, dsi_config);
		dsi_m6_dump_takeover("config-before-vmcmd-enqueue");
		DSI_Set_VM_CMD(module, cmdq);
		dsi_m6_dump_takeover("config-after-vmcmd-enqueue");
	}
		/* Enable clk low power per Line ; */
		DISPERR("M6 DSI lp_per_line[config]: enable=%u mode=%u HSA/HBP/HFP/BLLP/HSTX=0x%x/0x%x/0x%x/0x%x/0x%x\n",
			dsi_config->clk_lp_per_line_enable, dsi_config->mode,
			INREG32(DDP_REG_BASE_DSI0 + 0x050),
			INREG32(DDP_REG_BASE_DSI0 + 0x054),
			INREG32(DDP_REG_BASE_DSI0 + 0x058),
			INREG32(DDP_REG_BASE_DSI0 + 0x05c),
			INREG32(DDP_REG_BASE_DSI0 + 0x064));
		if (dsi_config->clk_lp_per_line_enable)
			DSI_PHY_CLK_LP_PerLine_config(module, cmdq, dsi_config);


done:
	dsi_m6_sram_snapshot("config-done", module);
	dsi_m6_lkgold_snapshot_once("post-config");
	dsi_m6_dump_hs_video_limited("config-done", module, cmdq,
				     &dump_count, 4);
#ifndef CONFIG_FPGA_EARLY_PORTING
	dsi_m6_dump_mipitx_block("config-done");
#endif

	/* Boot-time DSI selfpat bisector (m6_selfpat_auto_on, off by
	 * default), fired once at the first config-done on DSI0 (process
	 * context, so the msleep in the BIST test is fine). It substitutes the
	 * DSI pixel input with a fixed word, bypassing the OVL/RDMA/SMI/M4U
	 * fetch: if the DSI then completes a frame (VM_DONE sets, the panel
	 * shows gray), the DSI/PHY/panel link is fine and the problem is
	 * RDMA->DSI data delivery; otherwise the link itself cannot finish a
	 * frame. The BIST test's own snapshots record the result.
	 * m6_dsi_bist_full:0 toggles it off at runtime. */
	if (m6_selfpat_auto_on && !m6_selfpat_auto_done &&
	    module == DISP_MODULE_DSI0) {
		m6_selfpat_auto_done = 1;
		pr_emerg("[FORGE_DISP] #128 auto-selfpat: firing DSI0 BIST self-pattern (gray 0x00808080) to bisect DSI-link vs RDMA-data-path\n");
		DSI_M6_BIST_Full_Test(DISP_MODULE_DSI0, NULL, true, 0x00808080);
	}

	return 0;
}

int ddp_dsi_start(DISP_MODULE_ENUM module, void *cmdq)
{

	int i = 0;
	int g_lcm_x = disp_helper_get_option(DISP_OPT_FAKE_LCM_X);
	int g_lcm_y = disp_helper_get_option(DISP_OPT_FAKE_LCM_Y);
	static unsigned int dump_count;

	DISPFUNC();
	dsi_m6_takeover_hold_once("start-entry", module, cmdq, &dump_count);
	if (M6_LK_HANDOFF_SKIP_FIRST_DSI_CONFIG &&
	    atomic_read(&PMaster_enable) == 0 && !dsi_force_config) {
		DISPERR("M6 DSI lk-handoff[start]: skip first DSI start to preserve LK bootlogo state\n");
		dsi_m6_sram_snapshot("lk-handoff-start-skip", module);
		dsi_m6_dump_snapshot_limited("lk-handoff-start-skip", module,
					     cmdq, &dump_count, 4);
		return 0;
	}
	if (module == DISP_MODULE_DSI0) {
		DSI_Send_ROI(module, cmdq, g_lcm_x, g_lcm_y, _dsi_context[i].lcm_width,
			     _dsi_context[i].lcm_height);
		DSI_SetMode(module, cmdq, _dsi_context[i].dsi_params.mode);
		DSI_clk_HS_mode(module, cmdq, true);
		dsi_m6_sram_snapshot("start-after-hs", module);
		dsi_m6_lkgold_snapshot_once("post-start");
		dsi_m6_dump_snapshot_limited("start-after-hs", module, cmdq, &dump_count, 4);
		dsi_m6_schedule_ddp_hs_video_edge(module, cmdq);
#ifndef CONFIG_FPGA_EARLY_PORTING
		dsi_m6_dump_mipitx_block("start-after-hs");
#endif
	}

	return 0;
}

int ddp_dsi_stop(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	unsigned int tmp = 0;

	DISPFUNC();
	/* ths caller should call wait_event_or_idle for frame stop event then. */
	/* DSI_SetMode(module, cmdq_handle, CMD_MODE); */

	if (_dsi_is_video_mode(module)) {
		DISPMSG("dsi is video mode\n");
		DSI_SetMode(module, cmdq_handle, CMD_MODE);

		i = DSI_MODULE_BEGIN(module);
		while (1) {
			tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
			if (!(tmp & 0x80000000))
				break;
		}

		i = DSI_MODULE_END(module);
		while (1) {
			DISPMSG("dsi%d is busy\n", i);
			tmp = INREG32(&DSI_REG[i]->DSI_INTSTA);
			if (!(tmp & 0x80000000))
				break;
		}

	} else {
		DISPMSG("dsi is cmd mode\n");
		/* TODO: modify this with wait event */
		DSI_WaitForNotBusy(module, cmdq_handle);
	}
	DSI_clk_HS_mode(module, cmdq_handle, false);
	return 0;
}

/*TUI will use the api*/
int dsi_enable_irq(DISP_MODULE_ENUM module, void *handle, unsigned int enable)
{
	if (module == DISP_MODULE_DSI0) {
		uint32_t before = INREG32(&DSI_REG[0]->DSI_INTEN);

		DSI_OUTREGBIT(handle, DSI_INT_ENABLE_REG, DSI_REG[0]->DSI_INTEN, FRAME_DONE_INT_EN, enable);
		DISPERR("M6 DSI irq_enable: frame_done=%u handle=%p before=0x%x after=0x%x intsta=0x%x\n",
			enable, handle, before, INREG32(&DSI_REG[0]->DSI_INTEN),
			INREG32(&DSI_REG[0]->DSI_INTSTA));
		dsi_m6_dump_irq_decode("enable_irq", INREG32(&DSI_REG[0]->DSI_START),
				       INREG32(&DSI_REG[0]->DSI_STA),
				       INREG32(&DSI_REG[0]->DSI_INTEN),
				       INREG32(&DSI_REG[0]->DSI_INTSTA));
	}

	return 0;
}


int ddp_dsi_switch_lcm_mode(DISP_MODULE_ENUM module, void *params)
{
	int i = 0;
	LCM_DSI_MODE_SWITCH_CMD lcm_cmd = *((LCM_DSI_MODE_SWITCH_CMD *) (params));
	int mode = (int)(lcm_cmd.mode);

	DISPERR("M6 DSI switch_lcm_mode enter: module=%d cur=%d mode=%d cmd_if=%u addr=0x%x val=%02x/%02x/%02x/%02x\n",
		module, dsi_currect_mode, mode, lcm_cmd.cmd_if, lcm_cmd.addr,
		lcm_cmd.val[0], lcm_cmd.val[1], lcm_cmd.val[2], lcm_cmd.val[3]);
	dsi_m6_dump_live("switch-lcm-enter");
	if (dsi_currect_mode == mode) {
		DISPMSG
		    ("[ddp_dsi_switch_mode] not need switch mode, current mode = %d, switch to %d\n",
		     dsi_currect_mode, mode);
		return 0;
	}
	if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0)
		i = 0;
	else if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI1)
		i = 1;
	else {
		DISPMSG("dsi switch not support this cmd IF:%d\n", lcm_cmd.cmd_if);
		return -1;
	}

	if (mode == 0) {	/* V2C */
		DSI_OUTREG32(NULL, (unsigned long)(DSI_REG[i]) + 0x130,
				0x00001521 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));	/* RM = 1 */
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(NULL, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		wait_vm_cmd_done = false;
		/*
		 * SPI 194 does not fire, so this untimed wait would hang until
		 * the watchdog. Poll VM_CMD_DONE (bit5) in DSI_INTSTA + W1C.
		 */
		{
			unsigned int c = 0;

			while (!wait_vm_cmd_done &&
			       !(INREG32(&DSI_REG[i]->DSI_INTSTA) & 0x00000020)) {
				if (c++ > 200000) {
					DISPERR("[M681] dsi poll vm_cmd_done timeout (switch_lcm)\n");
					break;
				}
				udelay(1);
			}
			DSI_MASKREG32(NULL, &DSI_REG[i]->DSI_INTSTA, 0x00000020, 0x00000000);
		}
	}
	dsi_m6_dump_live("switch-lcm-exit");
	DISPERR("M6 DSI switch_lcm_mode exit: module=%d cur=%d mode=%d ret=0\n",
		module, dsi_currect_mode, mode);
	return 0;
}

int ddp_dsi_switch_mode(DISP_MODULE_ENUM module, void *cmdq_handle, void *params)
{
	int i = 0;
	LCM_DSI_MODE_SWITCH_CMD lcm_cmd;
	int mode;
	static unsigned int m6_switch_mode_count;
	unsigned int m6_seq = ++m6_switch_mode_count;

	if (m6_seq <= 64) {
		aee_sram_printk("M6K%02u enter mod=%d q=%p p=%p S=%x M=%x I=%x\n",
			m6_seq, module, cmdq_handle, params,
			INREG32(&DSI_REG[0]->DSI_START),
			INREG32(&DSI_REG[0]->DSI_MODE_CTRL),
			INREG32(&DSI_REG[0]->DSI_INTSTA));
		DISPERR("M6 DSI switch_mode raw enter #%u module=%d handle=%p params=%p start=0x%x mode_ctrl=0x%x intsta=0x%x\n",
			m6_seq, module, cmdq_handle, params,
			INREG32(&DSI_REG[0]->DSI_START),
			INREG32(&DSI_REG[0]->DSI_MODE_CTRL),
			INREG32(&DSI_REG[0]->DSI_INTSTA));
	}
	if (!params) {
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u null-params\n", m6_seq);
		DISPERR("M6 DSI switch_mode null params #%u module=%d handle=%p\n",
			m6_seq, module, cmdq_handle);
		return -EINVAL;
	}

	lcm_cmd = *((LCM_DSI_MODE_SWITCH_CMD *) (params));
	mode = (int)(lcm_cmd.mode);
	if (m6_seq <= 64)
		aee_sram_printk("M6K%02u copied mode=%d if=%u addr=%x val=%x/%x/%x/%x\n",
			m6_seq, mode, lcm_cmd.cmd_if, lcm_cmd.addr,
			lcm_cmd.val[0], lcm_cmd.val[1], lcm_cmd.val[2],
			lcm_cmd.val[3]);

	DISPERR("M6 DSI switch_mode enter: module=%d handle=%p cur=%d mode=%d cmd_if=%u addr=0x%x val=%02x/%02x/%02x/%02x\n",
		module, cmdq_handle, dsi_currect_mode, mode, lcm_cmd.cmd_if,
		lcm_cmd.addr, lcm_cmd.val[0], lcm_cmd.val[1], lcm_cmd.val[2],
		lcm_cmd.val[3]);
	if (m6_seq <= 64)
		aee_sram_printk("M6K%02u logged cur=%d mode=%d S=%x M=%x I=%x\n",
			m6_seq, dsi_currect_mode, mode,
			INREG32(&DSI_REG[0]->DSI_START),
			INREG32(&DSI_REG[0]->DSI_MODE_CTRL),
			INREG32(&DSI_REG[0]->DSI_INTSTA));
	dsi_m6_dump_live("switch-dsi-enter");
	if (dsi_currect_mode == mode) {
		DISPMSG
		    ("[ddp_dsi_switch_mode] not need switch mode, current mode = %d, switch to %d\n",
		     dsi_currect_mode, mode);
		return 0;
	}
	if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0)
		i = 0;
	else if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI1)
		i = 1;
	else {
		DISPMSG("dsi switch not support this cmd IF:%d\n", lcm_cmd.cmd_if);
		return -1;
	}

	if (mode == 0) {	/* V2C */

		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u v2c-before-set-switch i=%d S=%x M=%x I=%x\n",
				m6_seq, i, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_SetSwitchMode(module, cmdq_handle, 0);	/*  */
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u v2c-after-set-switch S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x130,
			0x00001539 | (lcm_cmd.addr << 16) | (lcm_cmd.val[1] << 24));	/* DM = 0 */
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u v2c-after-pkt S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[i]->DSI_START, VM_CMD_START, 1);
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u v2c-after-vmstart S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x1);	/* reset mutex for V2C */
		DSI_MASKREG32(cmdq_handle, 0xF4020028, 0x1, 0x0);	/*  */
		DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x1, 0x0);	/* mutext to cmd  mode */
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u v2c-after-mutex S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		cmdqRecFlush(cmdq_handle);
		cmdqRecReset(cmdq_handle);
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
		DSI_SetMode(module, NULL, 0);
	} else {		/* C2V */

		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-before-set-mode i=%d S=%x M=%x I=%x\n",
				m6_seq, i, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_SetMode(module, cmdq_handle, mode);
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-set-mode S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_SetSwitchMode(module, cmdq_handle, 1);	/* EXT TE could not use C2V */
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-set-switch S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		if (cmdq_handle) {
			DSI_MASKREG32(cmdq_handle, 0xF4020030, 0x1, 0x1);	/* mutext to video mode */
			if (m6_seq <= 64)
				aee_sram_printk("M6K%02u c2v-after-mutex-video S=%x M=%x I=%x\n",
					m6_seq, INREG32(&DSI_REG[i]->DSI_START),
					INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
					INREG32(&DSI_REG[i]->DSI_INTSTA));
		} else {
			unsigned int m6_mutex_en = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN);
			unsigned int m6_mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD);
			unsigned int m6_mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF);

			DISPERR("M6 DSI switch_mode C2V: skip cpu-direct MUTEX0_SOF video write for WDT isolation en=%x mod=%x sof=%x\n",
				m6_mutex_en, m6_mutex_mod, m6_mutex_sof);
			if (m6_seq <= 64)
				aee_sram_printk("M6K%02u c2v-skip-mutex-video E=%x O=%x F=%x S=%x M=%x I=%x\n",
					m6_seq, m6_mutex_en, m6_mutex_mod, m6_mutex_sof,
					INREG32(&DSI_REG[i]->DSI_START),
					INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
					INREG32(&DSI_REG[i]->DSI_INTSTA));
		}
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x200 + 0,
			     0x00001500 | (lcm_cmd.addr << 16) | (lcm_cmd.val[0] << 24));
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-pkt0 S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x200 + 4, 0x00000020);
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-pkt1 S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_OUTREG32(cmdq_handle, (unsigned long)(DSI_REG[i]) + 0x60, 2);
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-vmstart-reg S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		DSI_Start(module, cmdq_handle);	/* ???????????????????????????????? */
		if (m6_seq <= 64)
			aee_sram_printk("M6K%02u c2v-after-dsi-start S=%x M=%x I=%x\n",
				m6_seq, INREG32(&DSI_REG[i]->DSI_START),
				INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
				INREG32(&DSI_REG[i]->DSI_INTSTA));
		if (cmdq_handle) {
			DSI_MASKREG32(NULL, 0xF4020020, 0x1, 0x1);	/* release mutex for video mode */
			if (m6_seq <= 64)
				aee_sram_printk("M6K%02u c2v-after-mutex-release S=%x M=%x I=%x\n",
					m6_seq, INREG32(&DSI_REG[i]->DSI_START),
					INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
					INREG32(&DSI_REG[i]->DSI_INTSTA));
		} else {
			unsigned int m6_mutex_en = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN);
			unsigned int m6_mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD);
			unsigned int m6_mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF);

			DISPERR("M6 DSI switch_mode C2V: skip cpu-direct MUTEX0_EN release write for WDT isolation en=%x mod=%x sof=%x\n",
				m6_mutex_en, m6_mutex_mod, m6_mutex_sof);
			if (m6_seq <= 64)
				aee_sram_printk("M6K%02u c2v-skip-mutex-release E=%x O=%x F=%x S=%x M=%x I=%x\n",
					m6_seq, m6_mutex_en, m6_mutex_mod, m6_mutex_sof,
					INREG32(&DSI_REG[i]->DSI_START),
					INREG32(&DSI_REG[i]->DSI_MODE_CTRL),
					INREG32(&DSI_REG[i]->DSI_INTSTA));
		}
		dsi_m6_dump_live("switch-dsi-after-c2v-start");
		if (cmdq_handle) {
			cmdqRecFlush(cmdq_handle);
			cmdqRecReset(cmdq_handle);
			cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
		} else {
			DISPERR("M6 DSI switch_mode C2V: cpu-direct path skip cmdq flush/reset/wait\n");
		}
	}
	dsi_currect_mode = mode;
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++)
		_dsi_context[i].dsi_params.mode = mode;
	dsi_m6_dump_live("switch-dsi-exit");
	DISPERR("M6 DSI switch_mode exit: module=%d cur=%d mode=%d ret=0\n",
		module, dsi_currect_mode, mode);
	return 0;
}

int ddp_dsi_clk_on(DISP_MODULE_ENUM module, void *cmdq_handle, unsigned int level)
{
	int ret = 0;
	return ret;
}

int ddp_dsi_clk_off(DISP_MODULE_ENUM module, void *cmdq_handle, unsigned int level)
{
	int ret = 0;

	return ret;
}

int ddp_dsi_ioctl(DISP_MODULE_ENUM module, void *cmdq_handle, DDP_IOCTL_NAME ioctl_cmd,
		  void *params)
{
	int ret = 0;
	/* DISPFUNC(); */
	DDP_IOCTL_NAME ioctl = (DDP_IOCTL_NAME) ioctl_cmd;
	static unsigned int m6_ioctl_count;
	/* DISPMSG("[ddp_dsi_ioctl] index = %d\n", ioctl); */
	if ((ioctl == DDP_SWITCH_DSI_MODE || ioctl == DDP_SWITCH_LCM_MODE) &&
	    m6_ioctl_count < 32) {
		m6_ioctl_count++;
		aee_sram_printk("M6J%02u enter mod=%d cmd=%d q=%p p=%p S=%x M=%x I=%x\n",
			m6_ioctl_count, module, ioctl, cmdq_handle, params,
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x000),
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x014),
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x00c));
		DISPERR("M6 DSI ioctl enter #%u module=%d cmd=%d cmdq=%p params=%p start=0x%x mode=0x%x intsta=0x%x\n",
			m6_ioctl_count, module, ioctl, cmdq_handle, params,
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x000),
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x014),
			DISP_REG_GET(DDP_REG_BASE_DSI0 + 0x00c));
	}
	switch (ioctl) {
	case DDP_STOP_VIDEO_MODE:
		{
			/* ths caller should call wait_event_or_idle for frame stop event then. */
			DSI_SetMode(module, cmdq_handle, CMD_MODE);
			/* TODO: modify this with wait event */

			if (0 != DSI_WaitVMDone(module))
				ret = -1;
			if (module == DISP_MODULE_DSIDUAL) {
				DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG,
					      DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_COM_CTRL_REG,
					      DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[0]->DSI_START,
					      DSI_START, 0);
				DSI_OUTREGBIT(cmdq_handle, DSI_START_REG, DSI_REG[1]->DSI_START,
					      DSI_START, 0);
			}
			break;
		}

	case DDP_SWITCH_DSI_MODE:
		{
			if (m6_ioctl_count < 32) {
				aee_sram_printk("M6J%02u before-switch-dsi mod=%d cmd=%d q=%p\n",
					m6_ioctl_count, module, ioctl, cmdq_handle);
				DISPERR("M6 DSI ioctl before switch_dsi module=%d cmdq=%p params=%p\n",
					module, cmdq_handle, params);
			}
			ret = ddp_dsi_switch_mode(module, cmdq_handle, params);
			if (m6_ioctl_count < 32) {
				aee_sram_printk("M6J%02u after-switch-dsi ret=%d\n",
					m6_ioctl_count, ret);
				DISPERR("M6 DSI ioctl after switch_dsi ret=%d\n", ret);
			}
			break;
		}
	case DDP_SWITCH_LCM_MODE:
		{
			if (m6_ioctl_count < 32) {
				aee_sram_printk("M6J%02u before-switch-lcm mod=%d cmd=%d q=%p\n",
					m6_ioctl_count, module, ioctl, cmdq_handle);
				DISPERR("M6 DSI ioctl before switch_lcm module=%d cmdq=%p params=%p\n",
					module, cmdq_handle, params);
			}
			ret = ddp_dsi_switch_lcm_mode(module, params);
			if (m6_ioctl_count < 32) {
				aee_sram_printk("M6J%02u after-switch-lcm ret=%d\n",
					m6_ioctl_count, ret);
				DISPERR("M6 DSI ioctl after switch_lcm ret=%d\n", ret);
			}
			break;
		}
	case DDP_BACK_LIGHT:
		{
			unsigned int cmd = 0x51;
			unsigned int count = 1;
			unsigned int *p = (unsigned int *)params;
			unsigned int level = p[0];

			DISPMSG("[ddp_dsi_ioctl] level = %d\n", level);
			DSI_set_cmdq_V2(module, cmdq_handle, cmd, count, ((unsigned char *)&level), 1);
			break;
		}
	case DDP_DSI_IDLE_CLK_CLOSED:
			break;
	case DDP_DSI_IDLE_CLK_OPEN:
			break;
	case DDP_DSI_PORCH_CHANGE:
		{
			unsigned int *p = (unsigned int *)params;
			unsigned int vfp = p[0];

			ddp_dsi_porch_setting(module,
			cmdq_handle, DSI_VFP , vfp);
			break;
		}
	case DDP_PHY_CLK_CHANGE:
		{
			LCM_DSI_PARAMS *dsi_params = &_dsi_context[0].dsi_params;
			unsigned int *p = (unsigned int *)params;

			dsi_params->PLL_CLOCK = *p;
			/*DSI_WaitForNotBusy(module, cmdq_handle);*/
			DSI_PHY_clk_change(module, cmdq_handle, dsi_params);
			DSI_PHY_TIMCONFIG(module, cmdq_handle, dsi_params);
			break;
		}
	default:
		break;
	}
	return ret;
}

int ddp_dsi_trigger(DISP_MODULE_ENUM module, void *cmdq)
{
	int i = 0;
	unsigned int data_array[16];

	if (_dsi_context[i].dsi_params.mode == CMD_MODE) {
		data_array[0] = 0x002c3909;
		DSI_set_cmdq(module, cmdq, data_array, 1, 0);
	}
	DSI_Start(module, cmdq);

	return 0;
}

int ddp_dsi_reset(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	DSI_Reset(module, cmdq_handle);

	return 0;
}


int ddp_dsi_power_on(DISP_MODULE_ENUM module, void *cmdq_handle)
{

	int ret = 0;

	DISPFUNC();
	dsi_m6_clkstate_marker("power-on-entry", module);

	/* DSI_DumpRegisters(module,1); */
	if (!s_isDsiPowerOn) {
#ifdef ENABLE_CLK_MGR
#ifdef CONFIG_MTK_CLKMGR
		set_mipi26m(1);
#else
		ddp_set_mipi26m(1);
#endif
		if (is_ipoh_bootup) {
			if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
#ifdef CONFIG_MTK_CLKMGR
				ret += enable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI");
				ret += enable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI");
#else
				ret += ddp_clk_enable(DISP1_DSI_ENGINE);
				ret += ddp_clk_enable(DISP1_DSI_DIGITAL);
#endif
				if (ret > 0)
					pr_warn("DISP/DSI " "DSI power manager API return FALSE\n");
			}
			s_isDsiPowerOn = true;
			dsi_m6_clkstate_marker("power-on-ipoh", module);
			DISPMSG("ipoh dsi power on return\n");
			return DSI_STATUS_OK;
		}
		DSI_PHY_clk_switch(module, NULL, true);
		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
#ifdef CONFIG_MTK_CLKMGR
			ret += enable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI");
			ret += enable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI");
#else
			ret += ddp_clk_enable(DISP1_DSI_ENGINE);
			ret += ddp_clk_enable(DISP1_DSI_DIGITAL);
#endif
			if (ret > 0)
				DISPMSG("DSI power manager API return false\n");
		}
		/* restore dsi register */
		DSI_RestoreRegisters(module, NULL);

		/* enable sleep-out mode */
		DSI_SleepOut(module, NULL);

		/* restore lane_num */
		{
			DSI_REGS *regs = NULL;

			regs = &(_dsi_context[0].regBackup);
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_TXRX_CTRL,
				     AS_UINT32(&regs->DSI_TXRX_CTRL));
		}
		/* enter wakeup */
		DSI_Wakeup(module, NULL);

		/* enable clock */
		DSI_EnableClk(module, NULL);

		DSI_Reset(module, NULL);
		dsi_m6_clkstate_marker("power-on-after-reset", module);
#endif
		s_isDsiPowerOn = true;
	}
	dsi_m6_clkstate_marker("power-on-exit", module);
	/* DSI_DumpRegisters(module,1); */
#ifdef CONFIG_LOG_JANK
      LOG_JANK_D(JLID_KERNEL_LCD_POWER_ON, "%s", "JL_KERNEL_LCD_POWER_ON");
#endif
	return DSI_STATUS_OK;
}


int ddp_dsi_power_off(DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	int ret = 0;
	unsigned int value = 0;

	DISPFUNC();
	dsi_m6_clkstate_marker("power-off-entry", module);
	/* DSI_DumpRegisters(module,1); */

	if (s_isDsiPowerOn) {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			/*disable TE when power off */
			DSI_OUTREGBIT(NULL, DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN, TE_RDY, 0);
		}
		DSI_BackupRegisters(module, NULL);
#ifdef ENABLE_CLK_MGR
		/* disable HS mode */
		DSI_clk_HS_mode(module, NULL, false);
		/* enter ULPS mode */
		DSI_lane0_ULP_mode(module, NULL, 1);
		DSI_clk_ULP_mode(module, NULL, 1);
		/* make sure enter ulps mode */
		while (1) {
			mdelay(1);
			value = INREG32(&DSI_REG[0]->DSI_STATE_DBG1);
			value = value >> 24;
			if (value == 0x20)
				break;
			DISPMSG("dsi not in ulps mode, try again...\n");
		}
		/* clear lane_num when enter ulps */
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[0]->DSI_TXRX_CTRL, LANE_NUM, 0);
		/* disable clock */
		DSI_DisableClk(module, NULL);
		dsi_m6_clkstate_marker("power-off-before-clk-disable", module);

		if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
#ifdef CONFIG_MTK_CLKMGR
			ret += disable_clock(MT_CG_DISP1_DSI_ENGINE, "DSI");
			ret += disable_clock(MT_CG_DISP1_DSI_DIGITAL, "DSI");
#else
			ddp_clk_disable(DISP1_DSI_ENGINE);
			ddp_clk_disable(DISP1_DSI_DIGITAL);
#endif
			if (ret > 0)
				DISPMSG("DSI power manager API return false\n");
		}
		/* disable mipi pll */
		DSI_PHY_clk_switch(module, NULL, false);
#ifdef CONFIG_MTK_CLKMGR
		set_mipi26m(0);
#else
		ddp_set_mipi26m(0);
#endif
		dsi_m6_clkstate_marker("power-off-after-clk-disable", module);

#endif
		s_isDsiPowerOn = false;
	}
	dsi_m6_clkstate_marker("power-off-exit", module);
	/* DSI_DumpRegisters(module,1); */
	return DSI_STATUS_OK;
}


int ddp_dsi_is_busy(DISP_MODULE_ENUM module)
{
	int i = 0;
	int busy = 0;
	DSI_INT_STATUS_REG status;
	/* DISPFUNC(); */

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		status = DSI_REG[i]->DSI_INTSTA;

		if (status.BUSY)
			busy++;
	}

	DISPDBG("%s is %s\n", ddp_get_module_name(module), busy ? "busy" : "idle");
	return busy;
}

int ddp_dsi_is_idle(DISP_MODULE_ENUM module)
{
	return !ddp_dsi_is_busy(module);
}

static const char *dsi_mode_spy(LCM_DSI_MODE_CON mode)
{
	switch (mode) {
	case CMD_MODE:
		return "CMD_MODE";
	case SYNC_PULSE_VDO_MODE:
		return "SYNC_PULSE_VDO_MODE";
	case SYNC_EVENT_VDO_MODE:
		return "SYNC_EVENT_VDO_MODE";
	case BURST_VDO_MODE:
		return "BURST_VDO_MODE";
	default:
		return "unknown";
	}
}

void dsi_analysis(DISP_MODULE_ENUM module)
{
	int i = 0;

	DISPDMP("==DISP DSI ANALYSIS==\n");
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DISPDMP("MIPITX Clock: %d\n", dsi_phy_get_clk(module));
		DISPDMP
		    ("DSI%d Start:%x, Busy:%d, DSI_DUAL_EN:%d, MODE:%s, High Speed:%d, FSM State:%s\n",
		     i, DSI_REG[i]->DSI_START.DSI_START, DSI_REG[i]->DSI_INTSTA.BUSY,
		     DSI_REG[i]->DSI_COM_CTRL.DSI_DUAL_EN,
		     dsi_mode_spy(DSI_REG[i]->DSI_MODE_CTRL.MODE),
		     DSI_REG[i]->DSI_PHY_LCCON.LC_HS_TX_EN,
		     _dsi_cmd_mode_parse_state(DSI_REG[i]->DSI_STATE_DBG6.CMTRL_STATE));

		DISPDMP
		    ("DSI%d IRQ,RD_RDY:%d, CMD_DONE:%d, SLEEPOUT_DONE:%d, TE_RDY:%d, VM_CMD_DONE:%d, VM_DONE:%d\n",
		     i, DSI_REG[i]->DSI_INTSTA.RD_RDY, DSI_REG[i]->DSI_INTSTA.CMD_DONE,
		     DSI_REG[i]->DSI_INTSTA.SLEEPOUT_DONE, DSI_REG[i]->DSI_INTSTA.TE_RDY,
		     DSI_REG[i]->DSI_INTSTA.VM_CMD_DONE, DSI_REG[i]->DSI_INTSTA.VM_DONE);

		DISPDMP("DSI%d Lane Num:%d, Ext_TE_EN:%d, Ext_TE_Edge:%d, HSTX_CKLP_EN:%d\n", i,
			DSI_REG[i]->DSI_TXRX_CTRL.LANE_NUM,
			DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EN,
			DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EDGE,
			DSI_REG[i]->DSI_TXRX_CTRL.HSTX_CKLP_EN);

		DISPDMP("DSI%d LFR En:%d, LFR MODE:%d, LFR TYPE:%d, LFR SKIP NUMBER:%d\n", i,
			DSI_REG[i]->DSI_LFR_CON.LFR_EN,
			DSI_REG[i]->DSI_LFR_CON.LFR_MODE,
			DSI_REG[i]->DSI_LFR_CON.LFR_TYPE, DSI_REG[i]->DSI_LFR_CON.LFR_SKIP_NUM);
	}
}

int ddp_dsi_dump(DISP_MODULE_ENUM module, int level)
{
	if (!s_isDsiPowerOn) {
		DISPMSG("sleep dump is invalid\n");
		return 0;
	}

	dsi_analysis(module);
	DSI_DumpRegisters(module, level);

	return 0;
}

int ddp_dsi_build_cmdq(DISP_MODULE_ENUM module, void *cmdq_trigger_handle, CMDQ_STATE state)
{
	int ret = 0;
	int i = 0;
	int dsi_i = 0;
	LCM_DSI_PARAMS *dsi_params = NULL;
	DSI_T0_INS t0;
	DSI_T0_INS t1;
	DSI_RX_DATA_REG read_data[4] = {{0} };

	static cmdqBackupSlotHandle hSlot[4] = {0, 0, 0, 0};

	if (DISP_MODULE_DSIDUAL == module)
		dsi_i = 0;
	else
		dsi_i = DSI_MODULE_to_ID(module);
	dsi_params = &_dsi_context[dsi_i].dsi_params;

	if (cmdq_trigger_handle == NULL) {
		DISPERR("cmdq_trigger_handle is NULL\n");
		return -1;
	}

	if (state == CMDQ_WAIT_LCM_TE) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0) {
			if (dsi0_te_enable == 0)
				return 0;

			if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
				ret =
				    cmdqRecClearEventToken(cmdq_trigger_handle, CMDQ_EVENT_DSI_TE);
				ret = cmdqRecWait(cmdq_trigger_handle, CMDQ_EVENT_DSI_TE);
			}
		} else {
			DISPERR("wrong module: %s\n", ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_CHECK_IDLE_AFTER_STREAM_EOF) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0)
			DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_INTSTA, 0x80000000,
				0);
		else {
			DISPERR("wrong module: %s\n", ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_ESD_CHECK_READ) {
		/* enable dsi interrupt: RD_RDY/CMD_DONE (need do this here?) */
		DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN,
			      RD_RDY, 1);
		DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN,
			      CMD_DONE, 1);

		for (i = 0; i < ESD_CHECK_NUM; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;
			/* 0. send read lcm command(short packet) */
			t0.CONFG = 0x04;/* BTA */
			t0.Data0 = dsi_params->lcm_esd_check_table[i].cmd;
			/* / 0xB0 is used to distinguish DCS cmd or Gerneric cmd, is that Right??? */
			t0.Data_ID =
			    (t0.Data0 <
			     0xB0) ? DSI_DCS_READ_PACKET_ID : DSI_GERNERIC_READ_LONG_PACKET_ID;
			t0.Data1 = 0;

			t1.CONFG = 0x00;
			t1.Data0 = dsi_params->lcm_esd_check_table[i].count;
			t1.Data1 = 0x00;
			t1.Data_ID = 0x37;
			/* write DSI CMDQ */
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[0],
				AS_UINT32(&t1));
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[1],
				     AS_UINT32(&t0));
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_CMDQ_SIZE, 2);

			/* start DSI */
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_START, 0);
			DSI_OUTREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_START, 1);

			/* 1. wait DSI RD_RDY(must clear, in case of cpu RD_RDY interrupt handler) */
			if (dsi_i == 0) {
				DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_INTSTA,
					      0x00000001, 0x1);
				DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_STATUS_REG,
					      DSI_REG[dsi_i]->DSI_INTSTA, RD_RDY, 0x00000000);
			}
			/* 2. save RX data */
			if (hSlot[0] && hSlot[1] && hSlot[2] && hSlot[3]) {
				DSI_BACKUPREG32(cmdq_trigger_handle, hSlot[0], i,
					&DSI_REG[0]->DSI_RX_DATA0);
				DSI_BACKUPREG32(cmdq_trigger_handle, hSlot[1], i,
					&DSI_REG[0]->DSI_RX_DATA1);
				DSI_BACKUPREG32(cmdq_trigger_handle, hSlot[2], i,
					&DSI_REG[0]->DSI_RX_DATA2);
				DSI_BACKUPREG32(cmdq_trigger_handle, hSlot[3], i,
					&DSI_REG[0]->DSI_RX_DATA3);
			}
			/* 3. write RX_RACK */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_RACK_REG, DSI_REG[dsi_i]->DSI_RACK,
				DSI_RACK, 1);
			/* 4. polling not busy(no need clear) */
			if (dsi_i == 0)
				DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[dsi_i]->DSI_INTSTA,
					 0x80000000, 0);
			/* loop: 0~4 */
		}
		/* DSI_OUTREGBIT(cmdq_trigger_handle, DSI_INT_ENABLE_REG,DSI_REG[dsi_i]->DSI_INTEN,RD_RDY,0); */
	} else if (state == CMDQ_ESD_CHECK_CMP) {

		DISPMSG("[DSI]enter cmp\n");
		/* cmp just once and only 1 return value */
		for (i = 0; i < ESD_CHECK_NUM; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;
			DISPMSG("[DSI]enter cmp i=%d\n", i);

			/* read data */
			if (hSlot[0] && hSlot[1] && hSlot[2] && hSlot[3]) {
				/* read from slot */
				cmdqBackupReadSlot(hSlot[0], i, ((uint32_t *) &read_data[0]));
				cmdqBackupReadSlot(hSlot[1], i, ((uint32_t *) &read_data[1]));
				cmdqBackupReadSlot(hSlot[2], i, ((uint32_t *) &read_data[2]));
				cmdqBackupReadSlot(hSlot[3], i, ((uint32_t *) &read_data[3]));
			} else {
				/* read from dsi , support only one cmd read */
				if (i == 0) {
					DSI_OUTREG32(NULL, &read_data[0],
						     AS_UINT32(&DSI_REG[dsi_i]->DSI_RX_DATA0));
					DSI_OUTREG32(NULL, &read_data[1],
						     AS_UINT32(&DSI_REG[dsi_i]->DSI_RX_DATA0));
					DSI_OUTREG32(NULL, &read_data[2],
						     AS_UINT32(&DSI_REG[dsi_i]->DSI_RX_DATA0));
					DSI_OUTREG32(NULL, &read_data[3],
						     AS_UINT32(&DSI_REG[dsi_i]->DSI_RX_DATA0));
				}
			}

			DISPDBG("[DSI]enter cmp read_data0 byte0=0x%x byte1=0x%x byte2=0x%x byte3=0x%x\n",
			     read_data[0].byte0, read_data[0].byte1, read_data[0].byte2,
			     read_data[0].byte3);
			DISPDBG("[DSI]enter cmp read_data0=0x%x,read_data1=0x%x\n", AS_UINT32(&read_data[0]),
					AS_UINT32(&read_data[1]));
			DISPDBG("[DSI]enter cmp read_data2=0x%x,read_data3=0x%x\n", AS_UINT32(&read_data[2]),
					AS_UINT32(&read_data[3]));
			DISPDBG("[DSI]enter cmp check_table cmd=0x%x,count=0x%x,para_list[0]=0x%x,para_list[1]=0x%x\n",
			     dsi_params->lcm_esd_check_table[i].cmd,
			     dsi_params->lcm_esd_check_table[i].count,
			     dsi_params->lcm_esd_check_table[i].para_list[0],
			     dsi_params->lcm_esd_check_table[i].para_list[1]);
			DISPDBG("[DSI]enter cmp DSI+0x200=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x200));
			DISPDBG("[DSI]enter cmp DSI+0x204=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x204));
			DISPDBG("[DSI]enter cmp DSI+0x60=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x60));
			DISPDBG("[DSI]enter cmp DSI+0x74=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x74));
			DISPDBG("[DSI]enter cmp DSI+0x88=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x88));
			DISPDBG("[DSI]enter cmp DSI+0x0c=0x%x\n",
				AS_UINT32(DDP_REG_BASE_DSI0 + 0x0c));

			ret = DSI_read_cmp(i, read_data, dsi_params->lcm_esd_check_table[i].para_list,
				dsi_params->lcm_esd_check_table[i].count);
			if (ret)
				break;
		}
	} else if (state == CMDQ_ESD_ALLC_SLOT) {
		/* create 3 slot */
		unsigned int n = 0;

		n = DSI_esd_check_times(dsi_params);
		cmdqBackupAllocateSlot(&hSlot[0], n);
		cmdqBackupAllocateSlot(&hSlot[1], n);
		cmdqBackupAllocateSlot(&hSlot[2], n);
		cmdqBackupAllocateSlot(&hSlot[3], n);
	} else if (state == CMDQ_ESD_FREE_SLOT) {
		unsigned int h = 0;

		for (h = 0; h < 4; h++)
			if (hSlot[h]) {
				cmdqBackupFreeSlot(hSlot[h]);
				hSlot[h] = 0;
			}
	} else if (state == CMDQ_STOP_VDO_MODE) {
		/* use cmdq to stop dsi vdo mode */
		/* 0. set dsi cmd mode */
		DSI_SetMode(module, cmdq_trigger_handle, CMD_MODE);
		/* 1. polling dsi not busy */
		i = DSI_MODULE_BEGIN(module);
		if (i == 0) {
			/* polling dsi busy */
			DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0);
#if 0
			i = DSI_MODULE_END(module);
			if (i == 1)	/* DUAL */
				DSI_POLLREG32(cmdq_trigger_handle, &DSI_REG[i]->DSI_INTSTA,
					      0x80000000, 0);
#endif
		}
		/* 2.dual dsi need do reset DSI_DUAL_EN/DSI_START */
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG,
				      DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG,
				      DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[0]->DSI_START,
				      DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[1]->DSI_START,
				      DSI_START, 0);
		}
		/* 3.disable HS */
		/* DSI_clk_HS_mode(module, cmdq_trigger_handle, FALSE); */

	} else if (state == CMDQ_START_VDO_MODE) {

		/* 0. dual dsi set DSI_START/DSI_DUAL_EN */
		if (module == DISP_MODULE_DSIDUAL) {
			/* must set DSI_START to 0 before set dsi_dual_en, don't know why.2014.02.15 */
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[0]->DSI_START,
				      DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_START_REG, DSI_REG[1]->DSI_START,
				      DSI_START, 0);

			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG,
				      DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
			DSI_OUTREGBIT(cmdq_trigger_handle, DSI_COM_CTRL_REG,
				      DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 1);

		}
		/* 1. set dsi vdo mode */
		DSI_SetMode(module, cmdq_trigger_handle, dsi_params->mode);

		/* 2. enable HS */
		/* DSI_clk_HS_mode(module, cmdq_trigger_handle, TRUE); */

		/* 3. enable mutex */
		/* ddp_mutex_enable(mutex_id_for_latest_trigger,0,cmdq_trigger_handle); */

		/* 4. start dsi */
		/* DSI_Start(module, cmdq_trigger_handle); */

	} else if (state == CMDQ_DSI_RESET) {
		DISPMSG("CMDQ Timeout, Reset DSI\n");
		DSI_DumpRegisters(module, 1);
		DSI_Reset(module, NULL);
	} else if (state == CMDQ_DSI_LFR_MODE) {
		if (dsi_params->lfr_mode == 2 || dsi_params->lfr_mode == 3)
			DSI_LFR_UPDATE(module, cmdq_trigger_handle);
	}

	return ret;
}

void *get_dsi_params_handle(uint32_t dsi_idx)
{
	if (dsi_idx != PM_DSI1)
		return (void *)(&_dsi_context[0].dsi_params);
	else
		return (void *)(&_dsi_context[1].dsi_params);
}

int32_t DSI_ssc_enable(uint32_t dsi_index, uint32_t en)
{
	uint32_t disable = en ? 0 : 1;

	if (dsi_index == PM_DSI0) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[0].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI1) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[1]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[1].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI_DUAL) {
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[0]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		DSI_OUTREGBIT(NULL, MIPITX_DSI_PLL_CON1_REG, DSI_PHY_REG[1]->MIPITX_DSI_PLL_CON1,
			      RG_DSI0_MPPLL_SDM_SSC_EN, en);
		_dsi_context[0].dsi_params.ssc_disable = _dsi_context[1].dsi_params.ssc_disable =
		    disable;
	}
	return 0;
}

DDP_MODULE_DRIVER ddp_driver_dsi0 = {
	.module = DISP_MODULE_DSI0,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};

const LCM_UTIL_FUNCS PM_lcm_utils_dsi0 = {
	.set_reset_pin = lcm_set_reset_pin,
	.udelay = lcm_udelay,
	.mdelay = lcm_mdelay,
	.dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0,
	.dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0
};


/* /////////////////////// Panel Master ////////////////////////////////// */
uint32_t PanelMaster_get_TE_status(uint32_t dsi_idx)
{
	if (dsi_idx == 0)
		return dsi0_te_enable ? 1 : 0;
	/* else */
	/* return dsi1_te_enable ? 1:0 ; */
	return 1;
}

uint32_t PanelMaster_get_CC(uint32_t dsi_idx)
{
	DSI_TXRX_CTRL_REG tmp_reg;

	DSI_READREG32(PDSI_TXRX_CTRL_REG, &tmp_reg, &DSI_REG[dsi_idx]->DSI_TXRX_CTRL);
	return tmp_reg.HSTX_CKLP_EN ? 1 : 0;
}

void PanelMaster_set_CC(uint32_t dsi_index, uint32_t enable)
{
	DISPMSG("set_cc :%d\n", enable);
	if (dsi_index == PM_DSI0) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			      enable);
	} else if (dsi_index == PM_DSI1) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			      enable);
	} else if (dsi_index == PM_DSI_DUAL) {
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			      enable);
		DSI_OUTREGBIT(NULL, DSI_TXRX_CTRL_REG, DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			      enable);
	}
}

void PanelMaster_DSI_set_timing(uint32_t dsi_index, MIPI_TIMING timing)
{
	uint32_t hbp_byte;
	LCM_DSI_PARAMS *dsi_params;
	int fbconfig_dsiTmpBufBpp = 0;

	if (_dsi_context[dsi_index].dsi_params.data_format.format == LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;
	dsi_params = get_dsi_params_handle(dsi_index);
	switch (timing.type) {
	case LPX:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, LPX,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, LPX,
				      timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0, LPX,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0, LPX,
				      timing.value);
		}
		break;
	case HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_PRPR, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_PRPR, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_PRPR,timing.value); */
		break;
	case HS_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_ZERO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_ZERO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_ZERO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_ZERO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_ZERO,timing.value); */
		break;
	case HS_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[0]->DSI_PHY_TIMECON0,
				      HS_TRAIL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON0_REG, DSI_REG[1]->DSI_PHY_TIMECON0,
				      HS_TRAIL, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON0_REG,DSI_REG->DSI_PHY_TIMECON0,HS_TRAIL,timing.value); */
		break;
	case TA_GO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_GO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_GO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_GO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_GO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_GO,timing.value); */
		break;
	case TA_SURE:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_SURE, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_SURE, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_SURE, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_SURE, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_SURE,timing.value); */
		break;
	case TA_GET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_GET, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_GET, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      TA_GET, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      TA_GET, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,TA_GET,timing.value); */
		break;
	case DA_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      DA_HS_EXIT, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      DA_HS_EXIT, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[0]->DSI_PHY_TIMECON1,
				      DA_HS_EXIT, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON1_REG, DSI_REG[1]->DSI_PHY_TIMECON1,
				      DA_HS_EXIT, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON1_REG,DSI_REG->DSI_PHY_TIMECON1,DA_HS_EXIT,timing.value); */
		break;
	case CONT_DET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CONT_DET, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CONT_DET, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CONT_DET, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CONT_DET, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CONT_DET,timing.value); */
		break;
	case CLK_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CLK_ZERO, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CLK_ZERO, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CLK_ZERO, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CLK_ZERO, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CLK_ZERO,timing.value); */
		break;
	case CLK_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CLK_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CLK_TRAIL, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[0]->DSI_PHY_TIMECON2,
				      CLK_TRAIL, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON2_REG, DSI_REG[1]->DSI_PHY_TIMECON2,
				      CLK_TRAIL, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON2_REG,DSI_REG->DSI_PHY_TIMECON2,CLK_TRAIL,timing.value); */
		break;
	case CLK_HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_PRPR, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_PRPR, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_PRPR, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_PRPR,timing.value); */
		break;
	case CLK_HS_POST:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_POST, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_POST, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_POST, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_POST, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_POST,timing.value); */
		break;
	case CLK_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_EXIT, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_EXIT, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[0]->DSI_PHY_TIMECON3,
				      CLK_HS_EXIT, timing.value);
			DSI_OUTREGBIT(NULL, DSI_PHY_TIMCON3_REG, DSI_REG[1]->DSI_PHY_TIMECON3,
				      CLK_HS_EXIT, timing.value);
		}
		/* OUTREGBIT(DSI_PHY_TIMCON3_REG,DSI_REG->DSI_PHY_TIMECON3,CLK_HS_EXIT,timing.value); */
		break;
	case HPW:
		if (!(dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE))
			timing.value = (timing.value * fbconfig_dsiTmpBufBpp - 10);
		timing.value = ALIGN_TO((timing.value), 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC, timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC, timing.value);
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC, timing.value);
		}
		break;
	case HFP:
		timing.value = timing.value * fbconfig_dsiTmpBufBpp - 12;
		timing.value = ALIGN_TO(timing.value, 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[0]->DSI_HFP_WC, HFP_WC,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[1]->DSI_HFP_WC, HFP_WC,
				      timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[0]->DSI_HFP_WC, HFP_WC,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_HFP_WC_REG, DSI_REG[1]->DSI_HFP_WC, HFP_WC,
				      timing.value);
		}
		break;
	case HBP:
		if (dsi_params->mode == SYNC_EVENT_VDO_MODE || dsi_params->mode == BURST_VDO_MODE) {
			hbp_byte =
			    ((timing.value +
			      dsi_params->horizontal_sync_active) * fbconfig_dsiTmpBufBpp - 10);
		} else {
			/* hsa_byte = (dsi_params->horizontal_sync_active * fbconfig_dsiTmpBufBpp - 10); */
			hbp_byte = timing.value * fbconfig_dsiTmpBufBpp - 10;
		}
		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		} else {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC, ALIGN_TO((hbp_byte), 4));
		}

		break;
	case VPW:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[0]->DSI_VACT_NL, VACT_NL,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[1]->DSI_VACT_NL, VACT_NL,
				      timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[0]->DSI_VACT_NL, VACT_NL,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_VACT_NL_REG, DSI_REG[1]->DSI_VACT_NL, VACT_NL,
				      timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VACT_NL,timing.value); */
		break;
	case VFP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[0]->DSI_VFP_NL, VFP_NL,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[1]->DSI_VFP_NL, VFP_NL,
				      timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[0]->DSI_VFP_NL, VFP_NL,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_VFP_NL_REG, DSI_REG[1]->DSI_VFP_NL, VFP_NL,
				      timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VFP_NL, timing.value); */
		break;
	case VBP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[0]->DSI_VBP_NL, VBP_NL,
				      timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[1]->DSI_VBP_NL, VBP_NL,
				      timing.value);
		} else {
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[0]->DSI_VBP_NL, VBP_NL,
				      timing.value);
			DSI_OUTREGBIT(NULL, DSI_VBP_NL_REG, DSI_REG[1]->DSI_VBP_NL, VBP_NL,
				      timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VBP_NL, timing.value); */
		break;
	case SSC_EN:
		DSI_ssc_enable(dsi_index, timing.value);
		break;
	default:
		DISPMSG("fbconfig dsi set timing :no such type!!\n");
		break;
	}
}

uint32_t PanelMaster_get_dsi_timing(uint32_t dsi_index, MIPI_SETTING_TYPE type)
{
	uint32_t dsi_val;
	PDSI_REGS dsi_reg;
	int fbconfig_dsiTmpBufBpp = 0;

	if (_dsi_context[dsi_index].dsi_params.data_format.format == LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;
	if ((dsi_index == PM_DSI0) || (dsi_index == PM_DSI_DUAL))
		dsi_reg = DSI_REG[0];
	else
		dsi_reg = DSI_REG[1];
	switch (type) {
	case LPX:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.LPX;
		return dsi_val;
	case HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_PRPR;
		return dsi_val;
	case HS_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_ZERO;
		return dsi_val;
	case HS_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_TRAIL;
		return dsi_val;
	case TA_GO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GO;
		return dsi_val;
	case TA_SURE:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_SURE;
		return dsi_val;
	case TA_GET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GET;
		return dsi_val;
	case DA_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.DA_HS_EXIT;
		return dsi_val;
	case CONT_DET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CONT_DET;
		return dsi_val;
	case CLK_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_ZERO;
		return dsi_val;
	case CLK_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_TRAIL;
		return dsi_val;
	case CLK_HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_PRPR;
		return dsi_val;
	case CLK_HS_POST:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_POST;
		return dsi_val;
	case CLK_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_EXIT;
		return dsi_val;
	case HPW:
		{
			DSI_HSA_WC_REG tmp_reg = {0};

			DSI_READREG32(PDSI_HSA_WC_REG, &tmp_reg, &dsi_reg->DSI_HSA_WC);
			dsi_val = (tmp_reg.HSA_WC + 10) / fbconfig_dsiTmpBufBpp;
			return dsi_val;
		}
	case HFP:
		{
			DSI_HFP_WC_REG tmp_hfp = {0};

			DSI_READREG32(PDSI_HFP_WC_REG, &tmp_hfp, &dsi_reg->DSI_HFP_WC);
			dsi_val = ((tmp_hfp.HFP_WC + 12) / fbconfig_dsiTmpBufBpp);
			return dsi_val;
		}
	case HBP:
		{
			DSI_HBP_WC_REG tmp_hbp = {0};
			LCM_DSI_PARAMS *dsi_params;

			dsi_params = get_dsi_params_handle(dsi_index);
			OUTREG32(&tmp_hbp, AS_UINT32(&dsi_reg->DSI_HBP_WC));
			if (dsi_params->mode == SYNC_EVENT_VDO_MODE
			    || dsi_params->mode == BURST_VDO_MODE)
				return ((tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp -
					dsi_params->horizontal_sync_active);
			else
				return (tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp;
		}
	case VPW:
		{
			DSI_VACT_NL_REG tmp_vpw = {0};

			DSI_READREG32(PDSI_VACT_NL_REG, &tmp_vpw, &dsi_reg->DSI_VACT_NL);
			dsi_val = tmp_vpw.VACT_NL;
			return dsi_val;
		}
	case VFP:
		{
			DSI_VFP_NL_REG tmp_vfp = {0};

			DSI_READREG32(PDSI_VFP_NL_REG, &tmp_vfp, &dsi_reg->DSI_VFP_NL);
			dsi_val = tmp_vfp.VFP_NL;
			return dsi_val;
		}
	case VBP:
		{
			DSI_VBP_NL_REG tmp_vbp = {0};

			DSI_READREG32(PDSI_VBP_NL_REG, &tmp_vbp, &dsi_reg->DSI_VBP_NL);
			dsi_val = tmp_vbp.VBP_NL;
			return dsi_val;
		}
	case SSC_EN:
		{
			if (_dsi_context[dsi_index].dsi_params.ssc_disable)
				dsi_val = 0;
			else
				dsi_val = 1;
			return dsi_val;
		}
	default:
		DISPMSG("fbconfig dsi set timing :no such type!!\n");
	}
	dsi_val = 0;
	return dsi_val;
}

unsigned int PanelMaster_set_PM_enable(unsigned int value)
{
	atomic_set(&PMaster_enable, value);
	return 0;
}

/* ///////////////////////////////No DSI Driver //////////////////////////////////////////////// */
int DSI_set_roi(int x, int y)
{
	DISPMSG("[DSI](x0,y0,x1,y1)=(%d,%d,%d,%d)\n", x, y, _dsi_context[0].lcm_width,
	       _dsi_context[0].lcm_height);
	return DSI_Send_ROI(DISP_MODULE_DSI0, NULL, x, y, _dsi_context[0].lcm_width - x,
			    _dsi_context[0].lcm_height - y);
}

int DSI_check_roi(void)
{
	int ret = 0;
	unsigned char read_buf[4] = { 1, 1, 1, 1 };
	unsigned int data_array[16];
	int count;
	int x0;
	int y0;

	data_array[0] = 0x00043700;	/* read id return two byte,version and id */
	DSI_set_cmdq(DISP_MODULE_DSI0, NULL, data_array, 1, 1);

	msleep(20);

	count = DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, 0x2a, read_buf, 4);

	msleep(20);
	x0 = (read_buf[0] << 8) | read_buf[1];

	DISPMSG("x0=%d count=%d,read_buf[0]=%d,read_buf[1]=%d,read_buf[2]=%d,read_buf[3]=%d\n", x0,
	       count, read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
	if ((count == 0) || (x0 != 0)) {
		DISPMSG("[DSI]x count %d read_buf[0]=%d,read_buf[1]=%d,read_buf[2]=%d,read_buf[3]=%d\n",
		     count, read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
		return -1;
	}
	msleep(20);
	count = DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, 0x2b, read_buf, 4);
	y0 = (read_buf[0] << 8) | read_buf[1];

	DISPMSG("y0=%d count %d,read_buf[0]=%d,read_buf[1]=%d,read_buf[2]=%d,read_buf[3]=%d\n", y0,
	       count, read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
	if ((count == 0) || (y0 != 0)) {
		DISPMSG("[DSI]y count %d read_buf[0]=%d,read_buf[1]=%d,read_buf[2]=%d,read_buf[3]=%d\n",
		     count, read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
		return -1;
	}
	return ret;
}

void DSI_ForceConfig(int forceconfig)
{
	dsi_force_config = forceconfig;
	/*cv switch by resume*/
	if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND)) {
		if (lcm_mode_status != 0) {
			if (0 == _dsi_context[0].dsi_params.PLL_CK_CMD)
				_dsi_context[0].dsi_params.PLL_CK_CMD = _dsi_context[0].dsi_params.PLL_CLOCK;
			if (0 == _dsi_context[0].dsi_params.PLL_CK_VDO)
				_dsi_context[0].dsi_params.PLL_CK_VDO = _dsi_context[0].dsi_params.PLL_CLOCK;
			if (CMD_MODE == lcm_dsi_mode)
				_dsi_context[0].dsi_params.PLL_CLOCK = _dsi_context[0].dsi_params.PLL_CK_CMD;
			else if (SYNC_PULSE_VDO_MODE == lcm_dsi_mode ||
				 SYNC_EVENT_VDO_MODE == lcm_dsi_mode ||
					BURST_VDO_MODE == lcm_dsi_mode)
				_dsi_context[0].dsi_params.PLL_CLOCK = _dsi_context[0].dsi_params.PLL_CK_VDO;
		}
	}
}
