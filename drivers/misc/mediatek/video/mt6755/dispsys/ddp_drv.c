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

#define LOG_TAG "ddp_drv"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
/* ION */
/* #include <linux/ion.h> */
/* #include <linux/ion_drv.h> */
/* #include <mach/m4u.h> */
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>	/* of_find_device_by_node, of_platform_device_create */
#include <linux/io.h>
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include "ddp_clkmgr.h"
#endif
#include <mt-plat/sync_write.h>
#include "m4u.h"

#include "ddp_drv.h"
#include "ddp_reg.h"
#include "ddp_hal.h"
#include "disp_log.h"
#include "ddp_irq.h"
#include "ddp_info.h"
#include "ddp_ovl.h"
#include "ddp_dpi_reg.h"
#include "disp_helper.h"
#include "fbconfig_kdebug.h"


#define DISP_DEVNAME "DISPSYS"
#define M6_OVL_FAULT_LAYER_OFFSET	0x20
#define M6_OVL_FAULT_RDMA_DBG_OFFSET	0x4
extern int m4u_query_mva_info(unsigned int mva, unsigned int size,
	unsigned int *real_mva, unsigned int *real_size);
/* device and driver */
static dev_t disp_devno;
static struct cdev *disp_cdev;
static struct class *disp_class;

typedef struct {
	pid_t open_pid;
	pid_t open_tgid;
	struct list_head testList;
	spinlock_t node_lock;
} disp_node_struct;

static struct platform_device mydev;
static struct dispsys_device *dispsys_dev;
unsigned int dispsys_irq[DISP_REG_NUM] = { 0 };
volatile unsigned long dispsys_reg[DISP_REG_NUM] = { 0 };

volatile unsigned long mipi_tx_reg = 0;
volatile unsigned long dsi_reg_va = 0;
/* from DTS, for debug */
unsigned long ddp_reg_pa_base[DISP_REG_NUM] = {
	0x14000000,	/*CONFIG*/
	0x14008000,	/*OVL0 */
	0x14009000,	/*OVL1 */
	0x1400A000,	/*RDMA0 */
	0x1400B000,	/*RDMA1 */
	0x1400C000,	/*WDMA0 */
	0x1400D000,	/*COLOR*/
	0x1400E000,	/*CCORR*/
	0x1400F000,	/*AAL*/
	0x14010000,	/*GAMMA*/
	0x14011000,	/*DITHER*/
	0x14012000,	/*DSI0 */
	0x14013000,	/*DPI0 */
	0x1100E000,	/*PWM*/
	0x14014000,	/*MUTEX*/
	0x14015000,	/*SMI_LARB0 */
	0x14016000,	/*SMI_COMMON */
	0x14017000,	/*WDMA1 */
	0x14018000,	/*OVL0_2L */
	0x14019000,	/*OVL1_2L */
	0x10215000	/*MIPITX*/
};

#ifndef CONFIG_MTK_CLKMGR
/*
 * Note: The name order of the disp_clk_name[] must be synced with
 * the enum ddp_clk_id(ddp_clkmgr.h) in case get the wrong clock.
 */
const char *disp_clk_name[MAX_DISP_CLK_CNT] = {
	"DISP0_SMI_COMMON",
	"DISP0_SMI_COMMON_M4U",
	"DISP0_SMI_COMMON_DISPSYS",
	"DISP0_SMI_LARB0",
	"DISP0_SMI_LARB0_M4U",
	"DISP0_SMI_LARB0_DISPSYS",
	"DISP0_DISP_OVL0",
	"DISP0_DISP_OVL1",
	"DISP0_DISP_RDMA0",
	"DISP0_DISP_RDMA1",
	"DISP0_DISP_WDMA0",
	"DISP0_DISP_COLOR",
	"DISP0_DISP_CCORR",
	"DISP0_DISP_AAL",
	"DISP0_DISP_GAMMA",
	"DISP0_DISP_DITHER",
	"DISP0_DISP_UFOE_MOUT",
	"DISP0_DISP_WDMA1",
	"DISP0_DISP_2L_OVL0",
	"DISP0_DISP_2L_OVL1",
	"DISP0_DISP_OVL0_MOUT",
	"DISP1_DSI_ENGINE",
	"DISP1_DSI_DIGITAL",
	"DISP1_DPI_ENGINE",
	"DISP1_DPI_PIXEL",
	"DISP_PWM",
	"DISP_MTCMOS_CLK",
	"MUX_DPI0",
	"TVDPLL_D2",
	"TVDPLL_D4",
	"TVDPLL_D8",
	"TVDPLL_D16",
	"DPI_CK",
	"MUX_PWM",
	"UNIVPLL2_D4",
	"OSC_D2",
	"OSC_D8",
	"MUX_MM",
	"MM_VENCPLL",
	"SYSPLL2_D2"
};

#endif
/*
static unsigned int ddp_ms2jiffies(unsigned long ms)
{
	return (ms * HZ + 512) >> 10;
}
*/

struct dispsys_device {
	void __iomem *regs[DISP_REG_NUM];
	struct device *dev;
	int irq[DISP_REG_NUM];
#ifndef CONFIG_MTK_CLKMGR
	struct clk *disp_clk[MAX_DISP_CLK_CNT];
#endif
};


static int disp_is_intr_enable(DISP_REG_ENUM module)
{
	switch (module) {
	case DISP_REG_OVL0:
	case DISP_REG_OVL1:
	case DISP_REG_OVL0_2L:
	case DISP_REG_OVL1_2L:
	case DISP_REG_RDMA0:
	case DISP_REG_RDMA1:
	/*case DISP_REG_WDMA0:*/
	case DISP_REG_WDMA1:
	case DISP_REG_MUTEX:
	case DISP_REG_DSI0:
	case DISP_REG_DPI0:
	case DISP_REG_AAL:
		return 1;

	case DISP_REG_COLOR:
	case DISP_REG_CCORR:
	case DISP_REG_GAMMA:
	case DISP_REG_DITHER:
	case DISP_REG_PWM:
	case DISP_REG_CONFIG:
	case DISP_REG_SMI_LARB0:
	case DISP_REG_SMI_COMMON:
	case DISP_REG_MIPI:
		return 0;

	case DISP_REG_WDMA0:
#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
		return 0;
#else
		return 1;
#endif
	default:
		return 0;
	}
}

static void disp_m6_dump_ovl0_fault_corr(int port, unsigned int fault_mva)
{
	static unsigned int m6_ovl_fault_corr_count;
	struct m6_ovl_config_snapshot snap = { 0 };
	unsigned long base;
	unsigned int src_on;
	unsigned int valid_mva = 0;
	unsigned int valid_size = 0;
	unsigned int valid_end = 0;
	unsigned int idx;
	unsigned int layer;
	int has_snap;
	int qret;

	if (port != M4U_PORT_DISP_OVL0)
		return;
	if (!disp_helper_get_option(DISP_OPT_BYPASS_PQ))
		return;

	idx = m6_ovl_fault_corr_count++;
	if (idx >= 96 && (idx & 0x3f))
		return;

	qret = m4u_query_mva_info(fault_mva ? fault_mva - 1 : fault_mva,
		0, &valid_mva, &valid_size);
	if (!qret && valid_mva && valid_size)
		valid_end = valid_mva + valid_size;

	has_snap = ovl_m6_get_last_config_snapshot(&snap);
	base = ovl_base_addr(DISP_MODULE_OVL0);
	src_on = DISP_REG_GET(base + DISP_REG_OVL_SRC_CON);

	DISPERR("M6 OVL fault corr[%u]: port=%d fault=0x%x query=%d valid=0x%x/0x%x end=0x%x fault_minus_end=0x%x src=0x%x snap=%d seq=%u enabled=0x%x bounds0=%u\n",
		idx, port, fault_mva, qret, valid_mva, valid_size, valid_end,
		fault_mva - valid_end, src_on, has_snap,
		has_snap ? snap.seq : 0, has_snap ? snap.enabled_layers : 0,
		has_snap ? snap.layer[0].bounds_profile : 0);

	for (layer = 0; layer < 4; layer++) {
		unsigned long layer_off = layer * M6_OVL_FAULT_LAYER_OFFSET;
		unsigned long rdma_dbg_off = layer * M6_OVL_FAULT_RDMA_DBG_OFFSET;
		unsigned int live_size = DISP_REG_GET(base + DISP_REG_OVL_L0_SRC_SIZE + layer_off);
		unsigned int live_addr = DISP_REG_GET(base + DISP_REG_OVL_L0_ADDR + layer_off);
		unsigned int live_pitch_reg = DISP_REG_GET(base + DISP_REG_OVL_L0_PITCH + layer_off);
		unsigned int live_pitch = live_pitch_reg & 0xffff;
		unsigned int live_w = live_size & 0xfff;
		unsigned int live_h = (live_size >> 16) & 0xfff;
		unsigned int live_end = live_addr + live_h * live_pitch;
		const struct m6_ovl_layer_snapshot *req = NULL;

		if (has_snap && layer < ARRAY_SIZE(snap.layer) &&
		    snap.layer[layer].valid)
			req = &snap.layer[layer];

		DISPERR("M6 OVL fault corr[%u]: L%u live en=%u size=%ux%u addr=0x%x pitch=0x%x/%u end=0x%x fault_minus_live_end=0x%x rdma_dbg=0x%x req_end=0x%lx fault_minus_req_end=0x%lx dst_h=%u hw_h=%u bounds=%u\n",
			idx, layer, !!(src_on & (1U << layer)), live_w, live_h,
			live_addr, live_pitch_reg, live_pitch, live_end,
			fault_mva - live_end,
			DISP_REG_GET(base + DISP_REG_OVL_RDMA0_DBG + rdma_dbg_off),
			req ? req->pitch_end : 0,
			req ? (unsigned long)fault_mva - req->pitch_end : 0,
			req ? req->dst_h : 0, req ? req->hw_dst_h : 0,
			req ? req->bounds_profile : 0);
	}
}

m4u_callback_ret_t disp_m4u_callback(int port, unsigned int mva, void *data)
{
	DISP_MODULE_ENUM module = DISP_MODULE_OVL0;
	m4u_callback_ret_t ret;

	ret = M4U_CALLBACK_HANDLED;
	DISPERR("fault call port=%d, mva=0x%x, data=0x%p\n", port, mva, data);
	switch (port) {
	case M4U_PORT_DISP_OVL0:
		module = DISP_MODULE_OVL0;
		break;
	case M4U_PORT_DISP_RDMA0:
		module = DISP_MODULE_RDMA0;
		break;
	case M4U_PORT_DISP_WDMA0:
		module = DISP_MODULE_WDMA0;
		break;
	case M4U_PORT_DISP_OVL1:
		module = DISP_MODULE_OVL1;
		break;
	case M4U_PORT_DISP_RDMA1:
		module = DISP_MODULE_RDMA1;
		break;
	case M4U_PORT_DISP_WDMA1:
		module = DISP_MODULE_WDMA1;
		break;
	case M4U_PORT_DISP_2L_OVL0:
		module = DISP_MODULE_OVL0_2L;
		break;
	case M4U_PORT_DISP_2L_OVL1:
		module = DISP_MODULE_OVL1_2L;
		break;
	default:
	ret = M4U_CALLBACK_NOT_HANDLED;
		DISPERR("unknown port=%d\n", port);
	}
	disp_m6_dump_ovl0_fault_corr(port, mva);
	ddp_dump_analysis(module);
	ddp_dump_reg(module);
	return ret;
}


struct device *disp_get_device(void)
{
	return dispsys_dev->dev;
}

static int disp_probe(struct platform_device *pdev)
{
	struct class_device;
	int i;
	static unsigned int disp_probe_cnt;
	pr_emerg("[FORGE_DISP] disp_probe ENTRY\n");

	if (disp_probe_cnt != 0)
		return 0;

	/* save pdev for disp_probe_1 */
	memcpy(&mydev, pdev, sizeof(mydev));

	if (dispsys_dev) {
		DISPERR("%s: dispsys_dev=0x%p\n", __func__, dispsys_dev);
		BUG();
	}

	dispsys_dev = kmalloc(sizeof(struct dispsys_device), GFP_KERNEL);
	pr_emerg("[FORGE_DISP] disp_probe: dispsys_dev allocated=%p\n", dispsys_dev);
	pr_emerg("[FORGE_DISP] disp_probe_1: checking dispsys_dev=%p\n", dispsys_dev);
	if (!dispsys_dev) {
		DISPERR("Unable to allocate dispsys_dev\n");
		return -ENOMEM;
	}

#ifndef CONFIG_MTK_CLKMGR
	for (i = 0; i < MAX_DISP_CLK_CNT; i++) {
		DISPMSG("DISPSYS get clock %s\n", disp_clk_name[i]);
		dispsys_dev->disp_clk[i] = devm_clk_get(&pdev->dev, disp_clk_name[i]);
		if (IS_ERR(dispsys_dev->disp_clk[i]))
			DISPERR("%s:%d, DISPSYS get %d,%s clock error!!!\n",
				   __FILE__, __LINE__, i, disp_clk_name[i]);
		else {
				if (!ddp_set_clk_handle(dispsys_dev->disp_clk[i], i)) {
					/* Display clk prepare/enable can be skipped
					 * (forge_skip_disp_clk_en): the DISP MTCMOS power
					 * domain enable (DISP_MTCMOS_CLK / SMI_COMMON /
					 * SMI_LARB0) used to spin on the SCPSYS/SPM ack. The
					 * clk handles are registered via ddp_set_clk_handle
					 * either way. */
					static volatile int forge_skip_disp_clk_en = 0;	/* 0: enable the clocks through the CCF (DISP_MTCMOS/SMI_COMMON/DSI); the DSI mm1 CG must be ungated before the engine starts */
					if (!forge_skip_disp_clk_en) {
					/* marker: last 0xC2 aux = index of the clock being enabled */
					{ extern void forge_m681_mark_aux(unsigned char, unsigned int); forge_m681_mark_aux(0xC2, (unsigned int)i); }
					switch (i) {
					case MUX_MM:
					case MM_VENCPLL:
					case SYSPLL2_D2:
						break; /* no need prepare_enable here */
					case DISP0_SMI_COMMON:
					case DISP0_SMI_COMMON_M4U:
					case DISP0_SMI_COMMON_DISPSYS:
					case DISP0_SMI_LARB0:
					case DISP0_SMI_LARB0_M4U:
					case DISP0_SMI_LARB0_DISPSYS:
					case DISP_MTCMOS_CLK:
						ddp_clk_prepare_enable(i);
						break;
					default:
						ddp_clk_prepare(i);
						break;
					}
					}
				}
		}
	}
#endif /* CONFIG_MTK_CLKMGR */
	pr_emerg("[FORGE_DISP] disp_probe DONE\n");
	disp_probe_cnt++;

	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xC4); }	/* disp_probe done */
	pr_emerg("[FORGE_DISP] disp_probe: 0xC4 mark DONE, clk enabled\n");
	return 0;
}
static int __init disp_probe_1(void)
{
	struct class_device;
	int ret;
	int i;
	struct platform_device *pdev = &mydev;

	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xC5); }	/* disp_probe_1 entry */
	{ static volatile int forge_disp_disable = 0; if (forge_disp_disable) return 0; }	/* 0: run disp_probe_1 (of_iomap of the DISPSYS/MMSYS register bases + MMSYS power-on), which mtkfb needs */
	/* QEMU virt has no dispsys DT node, so dispsys_dev is NULL and the
	 * BUG() below would fire; run headless. Device unaffected. */
	{ extern int forge_virt; if (forge_virt) return 0; }

	disp_helper_option_init();

	pr_emerg("[FORGE_DISP] disp_probe_1: checking dispsys_dev=%p\n", dispsys_dev);
	if (!dispsys_dev) {
		DISPERR("%s: dispsys_dev=NULL\n", __func__);
		BUG();
	}

	dispsys_dev->dev = &pdev->dev;

	/* iomap registers and irq */
	for (i = 0; i < DISP_REG_NUM; i++) {
		struct resource res;

		dispsys_dev->regs[i] = of_iomap(pdev->dev.of_node, i);
		if (!dispsys_dev->regs[i]) {
			DISPERR("Unable to ioremap registers, of_iomap fail, i=%d\n", i);
			return -ENOMEM;
		}
		dispsys_reg[i] = (unsigned long)dispsys_dev->regs[i];
		/* check physical register */
		of_address_to_resource(pdev->dev.of_node, i, &res);
		if (ddp_reg_pa_base[i] != res.start)
			DISPERR("DT err, i=%d, module=%s, map_addr=%p, reg_pa=0x%lx!=0x%pa\n",
			       i, ddp_get_reg_module_name(i), dispsys_dev->regs[i],
			       ddp_reg_pa_base[i], &res.start);

		/* get IRQ ID and request IRQ */
		dispsys_dev->irq[i] = irq_of_parse_and_map(pdev->dev.of_node, i);
		dispsys_irq[i] = dispsys_dev->irq[i];

		DISPMSG("DT, i=%d, module=%s, map_addr=%p, map_irq=%d, reg_pa=0x%lx\n",
		       i, ddp_get_reg_module_name(i), dispsys_dev->regs[i], dispsys_dev->irq[i],
		       ddp_reg_pa_base[i]);
	}
	/* mipi tx reg map here */
	dsi_reg_va = dispsys_reg[DISP_REG_DSI0];
	mipi_tx_reg = dispsys_reg[DISP_REG_MIPI];
	DPI_REG = (struct DPI_REGS *)dispsys_reg[DISP_REG_DPI0];

	/* //// power on MMSYS for early porting */
	/* The stock wholesale MMSYS CG ungate is under CONFIG_MTK_FPGA, so on
	 * real hardware nothing ungates the MM clocks when the clk framework
	 * enable is skipped (forge_skip_disp_clk_en), and the DSI digital block
	 * reads/writes 0. Ungate the full banks CG_CLR0 + CG_CLR1 (including
	 * the SMI_COMMON/larb/OVL/RDMA fabric in CON0), bypassing the CCF.
	 * CON0/CON1 are logged before/after to confirm the writes latch (bits
	 * still set afterwards = the MMSYS island is not powered). */
	{
		unsigned int _c0 = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0);
		unsigned int _c1 = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1);
		DISP_REG_SET(NULL, DISP_REG_CONFIG_MMSYS_CG_CLR0, 0xFFFFFFFF);
		DISP_REG_SET(NULL, DISP_REG_CONFIG_MMSYS_CG_CLR1, 0xFFFFFFFF);
		pr_emerg("[FORGE_DISP] MMSYS CG ungate: CON0 0x%x->0x%x CON1 0x%x->0x%x\n",
			_c0, DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0),
			_c1, DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1));
	}
	/* init arrays */
	ddp_path_init();

	for (i = 0; i < DISP_REG_NUM; i++) {
		if (disp_is_intr_enable(i) == 1) {
			/* IRQF_TRIGGER_NONE dose not take effect here, real trigger mode set in dts file */
			ret = request_irq(dispsys_dev->irq[i], (irq_handler_t) disp_irq_handler,
				IRQF_TRIGGER_NONE, ddp_get_reg_module_name(i), NULL);
			if (ret) {
				DISPERR
				    ("Unable to request IRQ, request_irq fail, i=%d, irq=%d\n",
				     i, dispsys_dev->irq[i]);
				return ret;
			}
			DISPMSG("irq enabled, module=%s, irq=%d\n",
			       ddp_get_reg_module_name(i), dispsys_dev->irq[i]);
		}
	}

	/* init M4U callback */
	DISPMSG("register m4u callback\n");
	m4u_register_fault_callback(M4U_PORT_DISP_OVL0, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_RDMA0, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_WDMA0, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_OVL1, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_RDMA1, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_WDMA1, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_2L_OVL0, disp_m4u_callback, 0);
	m4u_register_fault_callback(M4U_PORT_DISP_2L_OVL1, disp_m4u_callback, 0);


	DISPMSG("dispsys probe done.\n");
	/* NOT_REFERENCED(class_dev); */
	return 0;
}

static int disp_remove(struct platform_device *pdev)
{
	return 0;
}

static void disp_shutdown(struct platform_device *pdev)
{
	/* Nothing yet */
}


/* PM suspend */
static int disp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

/* PM resume */
static int disp_resume(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dispsys_of_ids[] = {
	{.compatible = "mediatek,DISPSYS",},
	/* mt6755.dtsi has the lowercase "mediatek,dispsys" (dispsys@14008000);
	 * OF compatible matching is case-sensitive, so the uppercase-only table
	 * never matched and disp_probe never ran. */
	{.compatible = "mediatek,dispsys",},
	{}
};

static struct platform_driver dispsys_of_driver = {
	.driver = {
		   .name = DISP_DEVNAME,
		   .owner = THIS_MODULE,
		   .of_match_table = dispsys_of_ids,
		   },
	.probe = disp_probe,
	.remove = disp_remove,
	.shutdown = disp_shutdown,
	.suspend = disp_suspend,
	.resume = disp_resume,
};

static int __init disp_init(void)
{
	int ret = 0;

	pr_emerg("[FORGE_DISP] disp_init ENTRY\n");
	DISPMSG("Register the disp driver\n");
	init_log_buffer();
	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xC3); }	/* dispsys init */
	{ static volatile int forge_disp_disable = 0; if (forge_disp_disable) return 0; }	/* 0: register the dispsys driver (mediatek,DISPSYS) -> disp_probe -> disp_clk handles + DISP register bases set up before mtkfb/primary_display use them */
	if (platform_driver_register(&dispsys_of_driver)) {
		DISPERR("failed to register disp driver\n");
		/* platform_device_unregister(&disp_device); */
		ret = -ENODEV;
		return ret;
	}
	pr_emerg("[FORGE_DISP] disp_init platform_driver_register DONE\n");
	DISPMSG("disp driver init done\n");

	/* The async OF match for "mediatek,dispsys" does not trigger
	 * disp_probe here (dispsys_dev stays NULL and disp_probe_1 BUGs). Find
	 * the dispsys node, find or create its platform_device, and call
	 * disp_probe directly, so dispsys_dev, disp_clk[] and mydev are set up
	 * before disp_probe_1/mtkfb run. */
	pr_emerg("[FORGE_DISP] disp_probe_1: checking dispsys_dev=%p\n", dispsys_dev);
	if (!dispsys_dev) {
		struct device_node *np =
			of_find_compatible_node(NULL, NULL, "mediatek,dispsys");
		struct platform_device *pd = NULL;

		if (np) {
			pd = of_find_device_by_node(np);
			if (!pd)
				pd = of_platform_device_create(np, NULL, NULL);
		}
		pr_emerg("[FORGE_DISP] v183 dispsys np=%p pd=%p dev_of=%p\n",
			 np, pd, pd ? pd->dev.of_node : NULL);
		if (pd)
			disp_probe(pd);
		pr_emerg("[FORGE_DISP] v183 after force-probe dispsys_dev=%p\n",
			 dispsys_dev);
	}
	return 0;
}

static void __exit disp_exit(void)
{
#ifndef CONFIG_MTK_CLKMGR
	int i = 0;

	for (i = 0; i < MAX_DISP_CLK_CNT; i++)
		ddp_clk_unprepare(i);
#endif
	cdev_del(disp_cdev);
	unregister_chrdev_region(disp_devno, 1);

	platform_driver_unregister(&dispsys_of_driver);

	device_destroy(disp_class, disp_devno);
	class_destroy(disp_class);

}

#ifndef MTK_FB_DO_NOTHING
arch_initcall(disp_init);
module_init(disp_probe_1);
module_exit(disp_exit);
#endif
MODULE_AUTHOR("Tzu-Meng, Chung <Tzu-Meng.Chung@mediatek.com>");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
