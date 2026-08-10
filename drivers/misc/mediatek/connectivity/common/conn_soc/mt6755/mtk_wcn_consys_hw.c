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

/*! \file
 *   \brief  Declaration of library functions
 *
 *   Any definitions in this file will be shared among GLUE Layer and internal Driver Stack.
*/

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG "[WMT-CONSYS-HW]"

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#if defined(CONFIG_MTK_CLKMGR)
#include <mtk_clkmgr.h>
#else
#include <linux/clk.h>
#endif /* defined(CONFIG_MTK_LEGACY) */
#include <linux/delay.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>
#include "osal_typedef.h"
#include "mtk_wcn_consys_hw.h"

#if CONSYS_EMI_MPU_SETTING
#include <mach/emi_mpu.h>
#endif

#if CONSYS_PMIC_CTRL_ENABLE
#include <upmu_common.h>
#include <linux/regulator/consumer.h>
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include <mtk_hibernate_dpm.h>
#endif

#include <linux/of_reserved_mem.h>

#include <mt_clkbuf_ctl.h>

/* Stage-0/power-on DRAM tracer (survives a WDT reset; see the header for
 * readback). It brackets the EMI-MPU SMC into ATF, the EMI remap write via
 * the of_iomap'd topckgen base and the 343K memset_io of the consys EMI
 * carveout. */
#include "../../../forge_conn_trace.h"

/* mt_clkbuf_ctl.o is not built (base/power/mt6755/Makefile), so nothing
 * else provides clk_buf_ctrl(). It is only reached on the co-clock path
 * (WMT_SOC.cfg co_clock_flag != 0); this is an RF-VCTCXO board with
 * co_clock_flag=0 (see forge_conn_dcxo_init), so this is link insurance.
 * is_clk_buf_from_pmic() comes from ccci_util_dummy.c (weak, returns
 * false); the glue's own weak TRUE copy is dropped, since weak vs weak
 * picks by link order. */
bool __weak clk_buf_ctrl(enum clk_buf_id id, bool onoff)
{
	pr_info("[FORGE_CONN] clk_buf_ctrl(%d,%d) skipped (clkbuf deferred)\n",
		id, onoff);
	return false;
}

/* The PMIC is MT6351 and there is no PMIC/regulator driver for it
 * (base/power/mt6755/mt_pmic_stub.c); the vcn18/vcn28/vcn33_* regulator
 * lookups fail because the *-supply properties in the mt6755.dtsi consys
 * node are commented out. So the rails are driven directly over pwrap with
 * the MT6351 fields from mach/mt6351_regs.h. pmic_config_interface() goes
 * through the read-only WACS2 gate in pwrap_hal_v1.c: a register that is
 * not on its whitelist is dropped and counted there, and the readbacks
 * below (forge_conn_rails_on) catch it before any MTCMOS or consys access.
 * Every write is logged: dmesg [FORGE_CONN] is the live PMIC write
 * manifest for this driver.
 * Reads use pmic_read_interface_nolock(): the base-branch stub exports only
 * that name (it reads 0 there, so the rail gate below fails closed); on the
 * display line both names are the same pwrap_wacs2() read, serialised by the
 * wrapper's own spinlock. */
#define FORGE_CONN_MT6351_DIRECT 1

#if CONSYS_PMIC_CTRL_ENABLE && FORGE_CONN_MT6351_DIRECT
/* returns the PMIC gate's result: 0, or the refusal (mt_pmic_stub.c allowlist
 * rows: -EPERM switch off, -EINVAL value, -EWOULDBLOCK atomic context) */
static int forge_conn_vcn_write(const char *what, unsigned int addr,
				unsigned int val, unsigned int mask,
				unsigned int shift)
{
	int ret = (int)pmic_config_interface(addr, val, mask, shift);

	pr_info("[FORGE_CONN] pmic %s: reg 0x%x <= %u (mask 0x%x shift %u): %d\n",
		what, addr, val, mask, shift, ret);
	return ret;
}

#define FORGE_CONN_VCN(what, rg, val) \
	forge_conn_vcn_write(what, rg##_ADDR, val, rg##_MASK, rg##_SHIFT)

/* MT6351 DCXO CW00 (0x7000) readback before the stage-1 rails. This is an
 * RF-VCTCXO board: the clkbuf detect reports "clkbuf is from RF,
 * CW00=0x21f", the DCXO is not the consys 26M source (WMT_SOC.cfg
 * co_clock_flag=0, stock DTS rf buffer2(CONN)=0), and consys powers up
 * without a CW00=0x4DFD write. The step is a logged readback; there is no
 * CW00 write in this driver. */
static void forge_conn_dcxo_init(void)
{
	static int done;
	unsigned int cw00 = 0;

	if (done)
		return;
	pmic_read_interface_nolock(0x7000, &cw00, 0xFFFF, 0);
	pr_emerg("[FORGE_CONN] MT6351 DCXO CW00=0x%x (read only; m681 consys clock is the RF VCTCXO)\n",
		 cw00);
	done = 1;
}

/* The rail gate between stage 1 (PMIC) and stage 2 (MTCMOS).
 * DA_QI_VCNxx_EN (LDO_VCNxx_CON0 bit 15) is the LDO's own enable status: it
 * reads 1 only if the rail is really on, whether the EN write passed the
 * pwrap gate or LK left the rail on. A consys that is powered up without
 * its rails, or without a clock, stalls the first AXI access to it and the
 * hardware watchdog resets the phone, so a rail that reads off stops the
 * power-on here. VCN28 feeds the TCXO and is only required in TCXO mode
 * (co_clock_type 0). Bounded: 10 x 50 us. */
static bool forge_conn_rails_on(UINT32 co_clock_type)
{
	unsigned int vcn18 = 0, vcn28 = 0;
	int i;

	for (i = 0; i < 10; i++) {
		pmic_read_interface_nolock(MT6351_PMIC_DA_QI_VCN18_EN_ADDR, &vcn18,
				    MT6351_PMIC_DA_QI_VCN18_EN_MASK,
				    MT6351_PMIC_DA_QI_VCN18_EN_SHIFT);
		pmic_read_interface_nolock(MT6351_PMIC_DA_QI_VCN28_EN_ADDR, &vcn28,
				    MT6351_PMIC_DA_QI_VCN28_EN_MASK,
				    MT6351_PMIC_DA_QI_VCN28_EN_SHIFT);
		if (vcn18 && (co_clock_type || vcn28))
			break;
		udelay(50);
	}
	pr_emerg("[FORGE_CONN] rail readback: VCN18 %s, VCN28 %s (co_clock=%u)\n",
		 vcn18 ? "on" : "OFF", vcn28 ? "on" : "OFF", co_clock_type);
	return vcn18 && (co_clock_type || vcn28);
}

/* The condition for the LDO writes: before a rail is enabled, its voltage
 * selector must already hold the stock voltage. Codes from the MT6351 LDO
 * tables of the MTK MT6755 BSP (power/mt6755/pmic.c):
 *   VCN18 VOSEL[10:8], mt6351_VCAM_IO_voltages: 6 = 1.8 V
 *   VCN28 VOSEL[9:8],  mt6351_VA18_voltages:    3 = 2.8 V
 *   VCN33 VOSEL[10:9], mt6351_VCN33_voltages:   0 = 3.3 V
 * This driver never writes VOSEL. On a mismatch, or a failed read, nothing
 * is written for that rail and consys is not powered. */
enum { FORGE_VCN18, FORGE_VCN28, FORGE_VCN33 };

static const struct {
	const char *name;
	unsigned int addr, mask, shift, want, mv;
} forge_conn_vosel[] = {
	[FORGE_VCN18] = { "VCN18", MT6351_PMIC_RG_VCN18_VOSEL_ADDR,
			  MT6351_PMIC_RG_VCN18_VOSEL_MASK,
			  MT6351_PMIC_RG_VCN18_VOSEL_SHIFT, 6, 1800 },
	[FORGE_VCN28] = { "VCN28", MT6351_PMIC_RG_VCN28_VOSEL_ADDR,
			  MT6351_PMIC_RG_VCN28_VOSEL_MASK,
			  MT6351_PMIC_RG_VCN28_VOSEL_SHIFT, 3, 2800 },
	[FORGE_VCN33] = { "VCN33", MT6351_PMIC_RG_VCN33_VOSEL_ADDR,
			  MT6351_PMIC_RG_VCN33_VOSEL_MASK,
			  MT6351_PMIC_RG_VCN33_VOSEL_SHIFT, 0, 3300 },
};

static bool forge_conn_vosel_ok(int ldo)
{
	unsigned int v = ~0U;
	unsigned int ret;

	ret = pmic_read_interface_nolock(forge_conn_vosel[ldo].addr, &v,
					 forge_conn_vosel[ldo].mask,
					 forge_conn_vosel[ldo].shift);
	if (ret || v != forge_conn_vosel[ldo].want) {
		pr_emerg("[FORGE_CONN] %s VOSEL mismatch: read %u (ret %d), want %u (%u mV) - rail left off, consys not powered\n",
			 forge_conn_vosel[ldo].name, v, (int)ret,
			 forge_conn_vosel[ldo].want, forge_conn_vosel[ldo].mv);
		return false;
	}
	return true;
}
#endif

/* Read-only snapshot of the MT6351 registers connsys depends on:
 * the VCN18/VCN28/VCN33 LDO controls (RG_EN bit 1, ON_CTRL bit 3,
 * DA_QI_EN bit 15), their voltage selectors, the EN_STATUS summaries and the
 * DCXO CW00. Reads only - it answers "what did LK leave on" before any rail
 * write is allowed. Logged once at DO_MODULE_INIT (mtk_wcn_consys_hw_init)
 * and on every read of
 * /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_pmic. */
static const struct {
	const char *name;
	unsigned int addr;
} forge_conn_pmic_regs[] = {
	{ "LDO_VCN18_CON0", 0x0A52 },
	{ "LDO_VCN28_CON0", 0x0A0C },
	{ "LDO_VCN33_CON0", 0x0A92 },
	{ "LDO_VCN33_CON3(BT)", 0x0A98 },
	{ "LDO_VCN33_CON4(WIFI)", 0x0A9A },
	{ "VCN18_ANA_CON0(VOSEL[10:8])", 0x0B18 },
	{ "VCN28_ANA_CON0(VOSEL[9:8])", 0x0ACC },
	{ "VCN33_ANA_CON0(VOSEL[10:9])", 0x0ADA },
	{ "EN_STATUS1(VCN33 b9,VCN28 b11)", 0x020E },
	{ "EN_STATUS2(VCN18 b11)", 0x0210 },
	{ "DCXO_CW00", 0x7000 },
};

static int forge_conn_pmic_snapshot(char *buf, size_t len)
{
	unsigned int v;
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(forge_conn_pmic_regs); i++) {
		v = 0;
		pmic_read_interface_nolock(forge_conn_pmic_regs[i].addr, &v, 0xFFFF, 0);
		if (buf)
			n += scnprintf(buf + n, len - n, "0x%04x %s = 0x%04x\n",
				       forge_conn_pmic_regs[i].addr,
				       forge_conn_pmic_regs[i].name, v);
		else
			pr_info("[FORGE_CONN] pmic 0x%04x %s = 0x%04x\n",
				forge_conn_pmic_regs[i].addr,
				forge_conn_pmic_regs[i].name, v);
	}
	return n;
}

static int forge_conn_pmic_get(char *buffer, const struct kernel_param *kp)
{
	return forge_conn_pmic_snapshot(buffer, PAGE_SIZE);
}

static const struct kernel_param_ops forge_conn_pmic_ops = {
	.get = forge_conn_pmic_get,
};
module_param_cb(forge_conn_pmic, &forge_conn_pmic_ops, NULL, 0444);
MODULE_PARM_DESC(forge_conn_pmic, "m681 consys PMIC rails, read only");

/* Staged consys power-on gate.
 * A failed SCP_SYS_CONN MTCMOS enable hard-resets the SoC. WMT_init also
 * registers an fb notifier + work item (wmt_dev.c) that calls
 * mtk_wcn_wmt_func_on() on the first screen blank/unblank after
 * DO_MODULE_INIT, so a bad power-on would repeat on every boot. Stage 0
 * never touches consys.
 * Stages: 0=block before any touch; 1=PMIC rails only; 2=+MTCMOS+PWR_STATUS
 * readback; 3=+chip-id poll (gated on PWR_STATUS); 4=full.
 * Runtime: /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_pwron_stage
 * cmdline: mtk_wcn_consys_hw.forge_conn_pwron_stage=N
 * Markers land in expdb/last_kmsg. */
static int forge_conn_pwron_stage;	/* 0 = SAFE */
module_param(forge_conn_pwron_stage, int, 0644);
/* Arm consys automatically on the first power-on request, in the order
 * the userspace autostart used: after the loader, forge_conn_emi=7, then
 * forge_conn_pwron_stage=4. The request comes from wmt_loader/the HALs in
 * process context, after hw_init, so the EMI steps run in process context
 * too. 0 = stage and EMI only by hand. */
static bool forge_conn_autoarm = true;
module_param(forge_conn_autoarm, bool, 0644);
static int forge_conn_mtcmos_on;
/* pwr_on/pwr_off unwind symmetry: gpio/eirq teardown in pwr_off must only
 * run if gpio/eirq setup in pwr_on actually ran. A staged abort
 * (forge_conn_pwron_stage gates) fails reg_ctrl(1) before gpio_ctrl(1);
 * the stock unconditional gpio_ctrl(0) in pwr_off would then deinit pins
 * and eirqs that were never set up (see wmt_plat_alps.c gps_lna). */
static int forge_conn_gpio_on;

/* Defer all EMI hardware work out of mtk_wcn_consys_hw_init().
 * The userspace connectivity autostart runs on every boot, so any consys
 * code it can reach must touch no hardware until explicitly staged.
 * hw_init used to run three classes of hardware access at DO_MODULE_INIT
 * time, all before the reg_ctrl stage gate could intervene:
 *   bit0 EMI-MPU region-13 SMC into ATF (emi_mpu_set_region_protection)
 *   bit1 consys->AP EMI remap write + readback (topckgen + 0x1340)
 *   bit2 343K memset_io over the consys EMI carveout
 * Default forge_conn_emi_done=0 = hw_init performs no hardware access: it
 * only of_iomap's the register windows (page tables, no device access) and
 * registers the mtk_wmt platform driver (probe = clk/regulator/pinctrl
 * handle lookups, no MMIO).
 * Each deferred sub-step runs in the writer's own context when its bit is
 * first written to /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_emi,
 * bracketed by tracer + pr_err markers, so a wedge hangs the writing shell
 * with the ENTER marker already logged. Bits are one-way (a step, once
 * done, is never re-run; writes of already-done bits are no-ops). All
 * three bits (write 7) are required before pwron stage 4: releasing the
 * CONNSYS CPU without the EMI remap would let the consys MCU run against
 * an unconfigured EMI window. */
static DEFINE_MUTEX(forge_conn_emi_lock);
static unsigned int forge_conn_emi_done;	/* bitmask of completed sub-steps */
static INT32 forge_conn_emi_apply(unsigned int req);

static int forge_conn_emi_set(const char *val, const struct kernel_param *kp)
{
	unsigned int req = 0;
	int ret = kstrtouint(val, 0, &req);

	if (ret)
		return ret;
	if (req & ~0x7u)
		return -EINVAL;
	mutex_lock(&forge_conn_emi_lock);
	ret = forge_conn_emi_apply(req);
	mutex_unlock(&forge_conn_emi_lock);
	return ret;
}

static int forge_conn_emi_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%u", forge_conn_emi_done);
}

static const struct kernel_param_ops forge_conn_emi_ops = {
	.set = forge_conn_emi_set,
	.get = forge_conn_emi_get,
};
module_param_cb(forge_conn_emi, &forge_conn_emi_ops, NULL, 0644);
MODULE_PARM_DESC(forge_conn_emi,
		 "m681 consys EMI bring-up mask: bit0=EMI-MPU SMC, bit1=EMI remap, bit2=memset_io; write runs the step NOW; read = done mask");


/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static INT32 mtk_wmt_probe(struct platform_device *pdev);
static INT32 mtk_wmt_remove(struct platform_device *pdev);

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
UINT8 __iomem *pEmibaseaddr;
UINT64 gConEmiSize;
phys_addr_t gConEmiPhyBase;
struct CONSYS_BASE_ADDRESS conn_reg;

/* CCF part */
#if !defined(CONFIG_MTK_CLKMGR)
struct clk *clk_scp_conn_main;	/*ctrl conn_power_on/off */
/* struct clk *clk_infra_conn_main; */	/*ctrl infra_connmcu_bus clk */
#endif /* !defined(CONFIG_MTK_LEGACY) */

#ifdef CONFIG_OF
static const struct of_device_id apwmt_of_ids[] = {
	{.compatible = "mediatek,mt6755-consys",},
	{}
};
#endif

static struct platform_driver mtk_wmt_dev_drv = {
	.probe = mtk_wmt_probe,
	.remove = mtk_wmt_remove,
	.driver = {
		   .name = "mtk_wmt",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = apwmt_of_ids,
#endif
		   },
};

/* PMIC part */
#if CONSYS_PMIC_CTRL_ENABLE
#if !defined(CONFIG_MTK_LEGACY)
struct regulator *reg_VCN18;
struct regulator *reg_VCN28;
struct regulator *reg_VCN33_BT;
struct regulator *reg_VCN33_WIFI;
#endif
#endif

/* GPIO part */
#if !defined(CONFIG_MTK_LEGACY)
struct pinctrl *consys_pinctrl;
#endif

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
#define DYNAMIC_DUMP_GROUP_NUM 5
#if CONSYS_ENALBE_SET_JTAG
UINT32 gJtagCtrl;

#define JTAG_ADDR1_BASE 0x10002000

char *jtag_addr1 = (char *)JTAG_ADDR1_BASE;

#define JTAG1_REG_WRITE(addr, value)	\
writel(value, ((PUINT32)(jtag_addr1+(addr-JTAG_ADDR1_BASE))))
#define JTAG1_REG_READ(addr)			\
readl(((PUINT32)(jtag_addr1+(addr-JTAG_ADDR1_BASE))))

static INT32 mtk_wcn_consys_jtag_set_for_mcu(VOID)
{
#if 0
	int iRet = -1;

	WMT_PLAT_INFO_FUNC("WCN jtag_set_for_mcu start...\n");
	jtag_addr1 = ioremap(JTAG_ADDR1_BASE, 0x5000);
	if (jtag_addr1 == 0) {
		WMT_PLAT_ERR_FUNC("remap jtag_addr1 fail!\n");
		return iRet;
	}
	WMT_PLAT_INFO_FUNC("jtag_addr1 = 0x%p\n", jtag_addr1);

	JTAG1_REG_WRITE(0x100053c4, 0x11111100);
	JTAG1_REG_WRITE(0x100053d4, 0x00111111);

	/*Enable IES of all pins */
	JTAG1_REG_WRITE(0x10002014, 0x00000003);
	JTAG1_REG_WRITE(0x10005334, 0x55000000);
	JTAG1_REG_WRITE(0x10005344, 0x00555555);
	JTAG1_REG_WRITE(0x10005008, 0xc0000000);
	JTAG1_REG_WRITE(0x10005018, 0x0000000d);
	JTAG1_REG_WRITE(0x10005014, 0x00000032);
	JTAG1_REG_WRITE(0x100020a4, 0x000000ff);
	JTAG1_REG_WRITE(0x100020d4, 0x000000b4);
	JTAG1_REG_WRITE(0x100020d8, 0x0000004b);

	WMT_PLAT_INFO_FUNC("WCN jtag set for mcu start...\n");
	kal_int32 iRet = 0;
	kal_uint32 tmp = 0;
	kal_int32 addr = 0;
	kal_int32 remap_addr1 = 0;
	kal_int32 remap_addr2 = 0;

	remap_addr1 = ioremap(JTAG_ADDR1_BASE, 0x1000);
	if (remap_addr1 == 0) {
		WMT_PLAT_ERR_FUNC("remap jtag_addr1 fail!\n");
		return -1;
	}

	remap_addr2 = ioremap(JTAG_ADDR2_BASE, 0x100);
	if (remap_addr2 == 0) {
		WMT_PLAT_ERR_FUNC("remap jtag_addr2 fail!\n");
		return -1;
	}

	/*Pinmux setting for MT6625 I/F */
	addr = remap_addr1 + 0x03C0;
	tmp = DRV_Reg32(addr);
	tmp = tmp & 0xff;
	tmp = tmp | 0x11111100;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	addr = remap_addr1 + 0x03D0;
	tmp = DRV_Reg32(addr);
	tmp = tmp & 0xff000000;
	tmp = tmp | 0x00111111;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/*AP GPIO Setting 1 <default use> */
	/*Enable IES */
	/* addr = 0x10002014; */
	addr = remap_addr2 + 0x0014;
	tmp = 0x00000003;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));
	/*GPIO mode setting */
	/* addr = 0x10005334; */
	addr = remap_addr1 + 0x0334;
	tmp = 0x55000000;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/* addr = 0x10005344; */
	addr = remap_addr1 + 0x0344;
	tmp = 0x00555555;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));
	/*GPIO direction control */
	/* addr = 0x10005008; */
	addr = remap_addr1 + 0x0008;
	tmp = 0xc0000000;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/* addr = 0x10005018; */
	addr = remap_addr1 + 0x0018;
	tmp = 0x0000000d;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/* addr = 0x10005014; */
	addr = remap_addr1 + 0x0014;
	tmp = 0x00000032;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/*PULL Enable */
	/* addr = 0x100020a4; */
	addr = remap_addr2 + 0x00a4;
	tmp = 0x000000ff;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/*PULL select enable */
	/* addr = 0x100020d4; */
	addr = remap_addr2 + 0x00d4;
	tmp = 0x000000b4;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));

	/* addr = 0x100020d8; */
	addr = remap_addr2 + 0x00d8;
	tmp = 0x0000004b;
	DRV_WriteReg32(addr, tmp);
	WMT_PLAT_INFO_FUNC("(RegAddr, RegVal):(0x%08x, 0x%08x)", addr, DRV_Reg32(addr));
#endif

	return 0;
}

UINT32 mtk_wcn_consys_jtag_flag_ctrl(UINT32 en)
{
	WMT_PLAT_INFO_FUNC("%s jtag set for MCU\n", en ? "enable" : "disable");
	gJtagCtrl = en;
	return 0;
}

#endif

static INT32 mtk_wmt_probe(struct platform_device *pdev)
{
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0FA);
#if !defined(CONFIG_MTK_CLKMGR)
	clk_scp_conn_main = devm_clk_get(&pdev->dev, "conn");
	if (IS_ERR(clk_scp_conn_main)) {
		WMT_PLAT_ERR_FUNC("[CCF]cannot get clk_scp_conn_main clock.\n");
		return PTR_ERR(clk_scp_conn_main);
	}
	WMT_PLAT_DBG_FUNC("[CCF]clk_scp_conn_main=%p\n", clk_scp_conn_main);
#if 0
	clk_infra_conn_main = devm_clk_get(&pdev->dev, "infra-sys-conn-main");
	if (IS_ERR(clk_infra_conn_main)) {
		WMT_PLAT_ERR_FUNC("[CCF]cannot get clk_infra_conn_main clock.\n");
		return PTR_ERR(clk_infra_conn_main);
	}
	WMT_PLAT_DBG_FUNC("[CCF]clk_infra_conn_main=%p\n", clk_infra_conn_main);
#endif
#endif /* !defined(CONFIG_MTK_LEGACY) */

#if CONSYS_PMIC_CTRL_ENABLE
#if !defined(CONFIG_MTK_LEGACY)
	/* There is no *-supply in the consys DT node, so these are the core's
	 * dummy regulators (enable/disable are no-ops; the rails are driven by
	 * FORGE_CONN_VCN). An ERR_PTR must not survive: the rail code only
	 * tests for NULL. */
	reg_VCN18 = regulator_get(&pdev->dev, "vcn18");
	if (IS_ERR_OR_NULL(reg_VCN18)) {
		WMT_PLAT_ERR_FUNC("Regulator_get VCN_1V8 fail\n");
		reg_VCN18 = NULL;
	}
	reg_VCN28 = regulator_get(&pdev->dev, "vcn28");
	if (IS_ERR_OR_NULL(reg_VCN28)) {
		WMT_PLAT_ERR_FUNC("Regulator_get VCN_2V8 fail\n");
		reg_VCN28 = NULL;
	}
	reg_VCN33_BT = regulator_get(&pdev->dev, "vcn33_bt");
	if (IS_ERR_OR_NULL(reg_VCN33_BT)) {
		WMT_PLAT_ERR_FUNC("Regulator_get VCN33_BT fail\n");
		reg_VCN33_BT = NULL;
	}
	reg_VCN33_WIFI = regulator_get(&pdev->dev, "vcn33_wifi");
	if (IS_ERR_OR_NULL(reg_VCN33_WIFI)) {
		WMT_PLAT_ERR_FUNC("Regulator_get VCN33_WIFI fail\n");
		reg_VCN33_WIFI = NULL;
	}
#endif
#endif

#if !defined(CONFIG_MTK_LEGACY)
	consys_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(consys_pinctrl)) {
		WMT_PLAT_ERR_FUNC("cannot find consys pinctrl.\n");
		return PTR_ERR(consys_pinctrl);
	}
#endif /* !defined(CONFIG_MTK_LEGACY) */

	/* Probe marker: this line in dmesg shows that mtk_wmt_probe ran and
	 * clk_scp_conn_main is real, which guards against the
	 * clk_prepare_enable(NULL) == 0 false pass downstream. */
	pr_info("[FORGE_CONN] mtk_wmt_probe OK: clk_scp_conn=%p (deny retirement positive marker)\n",
		clk_scp_conn_main);
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0FB);
	return 0;
}

static INT32 mtk_wmt_remove(struct platform_device *pdev)
{
	return 0;
}

/* mtk_wcn_consys_co_clock_type() is not ported. It probed the co-clock
 * mode by writing DCXO CW15 (0x701E) and restoring it; on conn_soc the
 * mode comes from WMT_SOC.cfg co_clock_flag (wmt_lib.c -> wmt_plat_init),
 * and this driver writes no DCXO register. */

INT32 mtk_wcn_consys_hw_reg_ctrl(UINT32 on, UINT32 co_clock_type)
{

#if CONSYS_PWR_ON_OFF_API_AVAILABLE
	INT32 iRet = -1;
#endif
#if CONSYS_AFE_REG_SETTING
	UINT8 *consys_afe_reg_base = NULL;
	UINT8 i = 0;
#endif
	UINT32 retry = 10;
	UINT32 consysHwChipId = 0;

	WMT_PLAT_DBG_FUNC("CONSYS-HW-REG-CTRL(0x%08x),start\n", on);
	pr_info("[FORGE_CONN] reg_ctrl %s co_clock=%u\n", on ? "ON" : "OFF", co_clock_type);
	/* Was CONSYS_REG_READ(CONSYS_EMI_MAPPING): that macro expands to the
	 * hardcoded legacy VA TOPCKGEN_BASE 0xF0000000 (+0x1340), unmapped on
	 * arm64, and would fault as soon as wmtPlatLogLvl is raised to DBG. Use
	 * the of_iomap'd base like mtk_wcn_consys_hw_init() does. */
	WMT_PLAT_DBG_FUNC("CONSYS_EMI_MAPPING dump before power on/off(0x%08x)\n",
			  CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET));

	if (on) {
		WMT_PLAT_DBG_FUNC("++\n");
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x011);
		if (forge_conn_autoarm && forge_conn_pwron_stage == 0) {
			INT32 r;

			mutex_lock(&forge_conn_emi_lock);
			r = forge_conn_emi_apply(0x7);
			mutex_unlock(&forge_conn_emi_lock);
			if (!r && forge_conn_emi_done == 0x7u)
				forge_conn_pwron_stage = 4;
			pr_emerg("[FORGE_CONN] auto-arm: emi %d (done 0x%x) -> stage %d\n",
				 r, forge_conn_emi_done, forge_conn_pwron_stage);
		}
		pr_emerg("[FORGE_CONN] pwron gate: stage=%d\n", forge_conn_pwron_stage);
		if (forge_conn_pwron_stage < 1) {
			pr_emerg("[FORGE_CONN] stage0 STOP: blocked before PMIC/MTCMOS\n");
			return -EPERM;
		}
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x012);
		/* The consys 26MHz XO must be on before any rail/MTCMOS work: without it
		 * the domain powers up but stays clockless and the first consys AXI read
		 * wedges the bus. See forge_conn_dcxo_init() above. */
		forge_conn_dcxo_init();
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x020);
		if (!forge_conn_vosel_ok(FORGE_VCN18) ||
		    (!co_clock_type && !forge_conn_vosel_ok(FORGE_VCN28))) {
			pr_emerg("[FORGE_CONN] stage1 ABORT: VOSEL mismatch, no rail written\n");
			return -EIO;
		}
/*step1.PMIC ctrl*/
#if CONSYS_PMIC_CTRL_ENABLE
		/*need PMIC driver provide new API protocol */
		/*1.AP power on VCN_1V8 LDO (with PMIC_WRAP API) VCN_1V8  */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_EN, 0);
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_CFG, 0);
#else
#if FORGE_CONN_MT6351_DIRECT
		FORGE_CONN_VCN("VCN18 ON_CTRL", MT6351_PMIC_RG_VCN18_ON_CTRL, 0);
		FORGE_CONN_VCN("VCN18 EN", MT6351_PMIC_RG_VCN18_EN, 1);
#else
		pmic_set_register_value(MT6351_PMIC_RG_VCN18_ON_CTRL, 0);
#endif
#endif
		/* VOL_DEFAULT, VOL_1200, VOL_1300, VOL_1500, VOL_1800... */
#if defined(CONFIG_MTK_LEGACY)
		hwPowerOn(MT6351_POWER_LDO_VCN18, VOL_1800 * 1000, "wcn_drv");
#else
		if (reg_VCN18) {
			regulator_set_voltage(reg_VCN18, 1800000, 1800000);
			if (regulator_enable(reg_VCN18))
				WMT_PLAT_ERR_FUNC("enable VCN18 fail\n");
			else
				WMT_PLAT_DBG_FUNC("enable VCN18 ok\n");
		}
#endif
		/* VCN33 is not enabled here: it is the BT/WiFi RF PA rail, owned by
		 * mtk_wcn_consys_hw_bt/wifi_paldo_ctrl at func-on time, as in stock. */
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x021);	/* VCN18 written */
		udelay(150);

		if (co_clock_type) {
			/*step0,clk buf ctrl */
			WMT_PLAT_INFO_FUNC("co clock type(%d),turn on clk buf\n", co_clock_type);
			if (!is_clk_buf_from_pmic())
				clk_buf_ctrl(CLK_BUF_CONN, 1);

			/*if co-clock mode: */
			/*2.set VCN28 to SW control mode (with PMIC_WRAP API) */
			/*turn on VCN28 LDO only when FMSYS is activated"  */
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_EN, 0);
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_CFG, 0);
			#else
#if FORGE_CONN_MT6351_DIRECT
			FORGE_CONN_VCN("VCN28 ON_CTRL", MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
#else
			pmic_set_register_value(MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
#endif
			#endif
		} else {
			/*if NOT co-clock: */
			/*2.1.switch VCN28 to SW control mode (with PMIC_WRAP API) */
			/*2.2.turn on VCN28 LDO (with PMIC_WRAP API)" */
			/*fix vcn28 not balance warning */
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_EN, 1);
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_CFG, 0);
			#else
#if FORGE_CONN_MT6351_DIRECT
			FORGE_CONN_VCN("VCN28 ON_CTRL", MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
			FORGE_CONN_VCN("VCN28 EN", MT6351_PMIC_RG_VCN28_EN, 1);
#else
			pmic_set_register_value(MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
#endif
			#endif
#if defined(CONFIG_MTK_LEGACY)
			hwPowerOn(MT6351_POWER_LDO_VCN28, VOL_2800 * 1000, "wcn_drv");
#else
			if (reg_VCN28) {
				regulator_set_voltage(reg_VCN28, 2800000, 2800000);
				if (regulator_enable(reg_VCN28))
					WMT_PLAT_ERR_FUNC("enable VCN_2V8 fail!\n");
				else
					WMT_PLAT_DBG_FUNC("enable VCN_2V8 ok\n");
			}
#endif
		}
#endif

		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x022);	/* stage1 rails done */
		pr_emerg("[FORGE_CONN] stage1 done: VCN18/VCN28 rails written (VCN33 deferred to paldo)\n");
		if (!forge_conn_rails_on(co_clock_type)) {
			pr_emerg("[FORGE_CONN] stage1 ABORT: rail reads off (write dropped by the pwrap gate?) - NOT touching MTCMOS/consys\n");
			return -EIO;
		}
		if (forge_conn_pwron_stage < 2) {
			pr_emerg("[FORGE_CONN] stage1 STOP: before MTCMOS\n");
			return -EPERM;
		}
/*step2.MTCMOS ctrl*/

#ifdef CONFIG_OF		/*use DT */
		/*3.assert CONNSYS CPU SW reset  0x10007018 "[12]=1'b1  [31:24]=8'h88 (key)" */
		CONSYS_REG_WRITE((conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET),
				 CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) |
				 CONSYS_CPU_SW_RST_BIT | CONSYS_CPU_SW_RST_CTRL_KEY);
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x030);	/* CPU SW-RST asserted */
		/*turn on SPM clock gating enable PWRON_CONFG_EN  0x10006000  32'h0b160001 */
		/*CONSYS_REG_WRITE((conn_reg.spm_base + CONSYS_PWRON_CONFG_EN_OFFSET), CONSYS_PWRON_CONFG_EN_VALUE);*/

#if CONSYS_PWR_ON_OFF_API_AVAILABLE
#if defined(CONFIG_MTK_CLKMGR)
		iRet = conn_power_on();	/* consult clkmgr owner. */
		if (iRet)
			WMT_PLAT_ERR_FUNC("conn_power_on fail(%d)\n", iRet);
		WMT_PLAT_DBG_FUNC("conn_power_on ok\n");
#else
		/* Never let a NULL clk read as success: clk_prepare_enable(NULL) returns
		 * 0. If mtk_wmt_probe never ran (e.g. blocked by the probe denylist in
		 * drivers/base/platform.c), clk_scp_conn_main stays NULL, SCP_SYS_CONN is
		 * never powered, and the chip-id poll that follows would touch an
		 * unpowered consys and wedge the AXI bus. Fail instead. */
		if (IS_ERR_OR_NULL(clk_scp_conn_main)) {
			pr_info("[FORGE_CONN] FATAL: clk_scp_conn_main is NULL -> mtk_wmt_probe never ran (check the probe-deny in drivers/base/platform.c). ABORT power-on; NOT touching consys regs.\n");
			return -1;
		}
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x031);	/* pre MTCMOS */
		pr_info("[FORGE_CONN] mtcmos on: clk_prepare_enable(scp_sys_conn) enter\n");
		iRet = clk_prepare_enable(clk_scp_conn_main);
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x032);	/* post MTCMOS */
		pr_info("[FORGE_CONN] mtcmos on: clk_prepare_enable ret=%d\n", iRet);
		if (iRet) {
			WMT_PLAT_ERR_FUNC("clk_prepare_enable(clk_scp_conn_main) fail(%d)\n", iRet);
			return iRet;	/* never poll chip-id on an un-MTCMOS'd consys */
		}
		WMT_PLAT_DBG_FUNC("clk_prepare_enable(clk_scp_conn_main) ok\n");
		forge_conn_mtcmos_on = 1;
		{
			UINT32 sta  = CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET);
			UINT32 sta2 = CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET);
			UINT32 pwr_con = CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET);
			UINT32 prot_en = CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_EN_OFFSET);
			UINT32 prot_sta = CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_STA1_OFFSET);
			UINT32 cw00 = 0;

			pmic_read_interface_nolock(0x7000, &cw00, 0xFFFF, 0);
			/* AP-side SPM reads: always safe, tell us if CONN really powered.
			 * IGNORE_PWR_ACK compiled the in-CCF check out, so this is the only
			 * thing standing between us and an AXI read on a dead domain. */
			pr_emerg("[FORGE_CONN] stage2 done: PWR_STATUS=0x%08x/0x%08x CONN(bit1)=%d/%d\n",
				 sta, sta2, !!(sta & CONSYS_PWR_ON_ACK_BIT),
				 !!(sta2 & CONSYS_PWR_CONN_ACK_S_BIT));
			/* Full AP-side stage-3 preflight: CONN_PWR_CON should read
			 * ISO=0 RST_B=1 CLK_DIS=0 PWR_ON/2ND=1 (0x3d-ish); TOPAXI
			 * CONN bits 13|14 must be CLEAR in both EN and STA1; CW00
			 * bit5 (XO_EXTBUF2_EN_M) must be 1 or the consys 26M is off
			 * and stage 3 WILL wedge the AXI. */
			pr_emerg("[FORGE_CONN] stage2 preflight: CONN_PWR_CON=0x%08x TOPAXI_EN=0x%08x STA1=0x%08x (conn bits13|14: en=%u sta=%u) CW00=0x%04x EXTBUF2_EN_M=%u\n",
				 pwr_con, prot_en, prot_sta,
				 !!(prot_en & ((0x1 << 13) | (0x1 << 14))),
				 !!(prot_sta & ((0x1 << 13) | (0x1 << 14))),
				 cw00, !!(cw00 & (0x1 << 5)));
			forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x033);	/* stage2 preflight read */
			if (forge_conn_pwron_stage < 3) {
				pr_emerg("[FORGE_CONN] stage2 STOP: before chip-id (MTCMOS left on)\n");
				return -EPERM;
			}
			if (!(sta & CONSYS_PWR_ON_ACK_BIT) || !(sta2 & CONSYS_PWR_CONN_ACK_S_BIT)) {
				forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x03F);
				pr_emerg("[FORGE_CONN] ABORT: CONN not powered per PWR_STATUS - NOT touching consys regs (would AXI-stall)\n");
				return -ENODEV;
			}
			/* Stage-3 go/no-go: advance to the first consys AXI read only with
			 * CONN (bit1) = 1/1 and TOPAXI bus-protect conn bits 13|14 clear in
			 * both EN and STA1. With protect still up, the 0x18070008 read stalls
			 * the AXI bus and the HW WDT fires; abort instead. */
			if ((prot_en & ((0x1 << 13) | (0x1 << 14))) ||
			    (prot_sta & ((0x1 << 13) | (0x1 << 14)))) {
				forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x03E);
				pr_emerg("[FORGE_CONN] ABORT: TOPAXI conn protect still asserted (EN=0x%08x STA1=0x%08x) - NOT reading chip-id\n",
					 prot_en, prot_sta);
				return -ENODEV;
			}
		}
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x040);	/* pre first consys AXI read */
		pr_emerg("[FORGE_CONN] stage3: chip-id poll enter (first AXI read into consys 0x18070008)\n");
#endif /* defined(CONFIG_MTK_LEGACY) */
#else
		/*2.write conn_top1_pwr_on=1, power on conn_top1 0x1000632c [2]  1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ON_BIT);
		/*3.read conn_top1_pwr_on_ack =1, power on ack ready 0x10006180 [1] */
		while (0 == (CONSYS_PWR_ON_ACK_BIT & CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET)))
			NULL;
		/*5.write conn_top1_pwr_on_s=1, power on conn_top1 0x1000632c [3]  1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ON_S_BIT);
		/*6.write conn_clk_dis=0, enable connsys clock 0x1000632c [4]  1'b0 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~CONSYS_CLK_CTRL_BIT);
		/*7.wait 1us    */
		udelay(1);
		/*8.read conn_top1_pwr_on_ack_s =1, power on ack ready 0x10006184 [1] */
		while (0 == (CONSYS_PWR_CONN_ACK_S_BIT &
			CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET)))
			NULL;
		/*9.release connsys ISO, conn_top1_iso_en=0 0x1000632c [1]  1'b0 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~CONSYS_SPM_PWR_ISO_S_BIT);
		/*10.release SW reset of connsys, conn_ap_sw_rst_b=1  0x1000632c[0]   1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_RST_BIT);
		/*disable AXI BUS protect 0x10001220[13] [14] */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_EN_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_EN_OFFSET) &
				 ~CONSYS_PROT_MASK);
		while (CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_STA1_OFFSET) & CONSYS_PROT_MASK)
			NULL;
#endif
		/*11.26M is ready now, delay 10us for mem_pd de-assert */
		udelay(10);
		/*enable AP bus clock : connmcu_bus_pd  API: enable_clock() ++?? */

		/*12.poll CONNSYS CHIP ID until chipid is returned  0x18070008 */
		while (retry-- > 0) {
			consysHwChipId = CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_CHIP_ID_OFFSET);
			if (consysHwChipId == 0x0326 /* 6755 consys chip id */) {
				forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x041);	/* chip-id OK */
				WMT_PLAT_INFO_FUNC("retry(%d)consys chipId(0x%08x)\n", retry, consysHwChipId);
				pr_emerg("[FORGE_CONN] stage3 PASS: consys chipId=0x%08x (want 0x326)\n",
					 consysHwChipId);
				break;
			}
			WMT_PLAT_ERR_FUNC("Read CONSYS chipId(0x%08x)", consysHwChipId);
			msleep(20);
		}

		if ((retry == 0) || (consysHwChipId == 0)) {
			pr_info("[FORGE_CONN] chipid poll FAILED (0x%08x)\n", consysHwChipId);
			WMT_PLAT_ERR_FUNC("Maybe has a consys power on issue,(0x%08x)\n", consysHwChipId);
			WMT_PLAT_ERR_FUNC("reg dump:CONSYS_CPU_SW_RST_REG(0x%x)\n",
				  CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET));
			WMT_PLAT_ERR_FUNC("reg dump:CONSYS_PWR_CONN_ACK_REG(0x%x)\n",
				   CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_OFFSET));
			WMT_PLAT_ERR_FUNC("reg dump:CONSYS_PWR_CONN_ACK_S_REG(0x%x)\n",
				   CONSYS_REG_READ(conn_reg.spm_base + CONSYS_PWR_CONN_ACK_S_OFFSET));
			WMT_PLAT_ERR_FUNC("reg dump:CONSYS_TOP1_PWR_CTRL_REG(0x%x)\n",
				   CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET));
			/* Stop here. Stock only logged and then carried on to write ACR
			 * (0x18070110) and de-assert the CONNSYS CPU reset, i.e. more AXI
			 * accesses into a consys that is not answering. */
			forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x04F);
			pr_emerg("[FORGE_CONN] chip-id FAILED -> ABORT (not writing ACR / not de-asserting CPU reset)\n");
			return -ENODEV;
		}

		if (forge_conn_pwron_stage < 4) {
			pr_emerg("[FORGE_CONN] stage3 STOP: chip-id OK, before ACR/CPU-reset-deassert\n");
			return -EPERM;
		}
		/* Stage 4 releases the CONNSYS CPU: without the deferred EMI steps
		 * (forge_conn_emi=7: MPU SMC + remap + memset) the consys MCU would run
		 * against an unconfigured EMI window. Refuse instead. */
		if (forge_conn_emi_done != 0x7u) {
			forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x04E);
			pr_emerg("[FORGE_CONN] stage4 ABORT: EMI init incomplete (forge_conn_emi=0x%x, need 7) - not writing ACR / not releasing CONNSYS CPU\n",
				 forge_conn_emi_done);
			return -EPERM;
		}
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x050);	/* stage4: ACR next */

		/*13.{default no need}update ROMDEL/PATCH RAM DELSEL if needed 0x18070114 */

		/*
		 *14.write 1 to conn_mcu_confg ACR[1] if real speed MBIST
		 *(default write "1") ACR 0x18070110[18] 1'b1
		 *if this bit is 0, HW will do memory auto test under low CPU frequence (26M Hz)
		 *if this bit is 0, HW will do memory auto test under high CPU frequence(138M Hz)
		 *inclulding low CPU frequence
		 */
		CONSYS_REG_WRITE(conn_reg.mcu_base + CONSYS_MCU_CFG_ACR_OFFSET,
				 CONSYS_REG_READ(conn_reg.mcu_base + CONSYS_MCU_CFG_ACR_OFFSET) |
				 CONSYS_MCU_CFG_ACR_MBIST_BIT);

#if CONSYS_AFE_REG_SETTING
		/*15.default no need,update ANA_WBG(AFE) CR if needed, CONSYS_AFE_REG */
		consys_afe_reg_base = ioremap_nocache(CONSYS_AFE_REG_BASE, 0x100);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_AFE_01_OFFSET,
				CONSYS_AFE_REG_WBG_AFE_01_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_PLL_03_OFFSET,
				CONSYS_AFE_REG_WBG_PLL_03_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_PLL_05_OFFSET,
				CONSYS_AFE_REG_WBG_PLL_05_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_WB_TX_01_OFFSET,
				CONSYS_AFE_REG_WBG_WB_TX_01_VALUE);

		WMT_PLAT_DBG_FUNC("Dump AFE register\n");
		for (i = 0; i < 64; i++) {
			WMT_PLAT_DBG_FUNC("reg:0x%08x|val:0x%08x\n",
				CONSYS_AFE_REG_BASE + 4*i, CONSYS_REG_READ(consys_afe_reg_base + 4*i));
		}
#endif
		/*16.deassert CONNSYS CPU SW reset 0x10007018 "[12]=1'b0 [31:24] =8'h88 (key)" */
		CONSYS_REG_WRITE(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET,
				 (CONSYS_REG_READ(conn_reg.ap_rgu_base + CONSYS_CPU_SW_RST_OFFSET) &
				 ~CONSYS_CPU_SW_RST_BIT) | CONSYS_CPU_SW_RST_CTRL_KEY);
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x051);	/* CPU SW-RST deasserted */
		pr_emerg("[FORGE_CONN] stage4: ACR set + CONNSYS CPU released (fw download is next, via wmt_loader/stpwmt)\n");

#else /*use HADRCODE, maybe no use.. */
		/*3.assert CONNSYS CPU SW reset  0x10007018  "[12]=1'b1  [31:24]=8'h88 (key)" */
		CONSYS_REG_WRITE(CONSYS_CPU_SW_RST_REG,
				 (CONSYS_REG_READ(CONSYS_CPU_SW_RST_REG) | CONSYS_CPU_SW_RST_BIT |
				  CONSYS_CPU_SW_RST_CTRL_KEY));
		/*turn on SPM clock gating enable PWRON_CONFG_EN 0x10006000 32'h0b160001 */
		CONSYS_REG_WRITE(CONSYS_PWRON_CONFG_EN_REG, CONSYS_PWRON_CONFG_EN_VALUE);

#if CONSYS_PWR_ON_OFF_API_AVAILABLE
#if defined(CONFIG_MTK_CLKMGR)
		iRet = conn_power_on();	/* consult clkmgr owner */
		if (iRet)
			WMT_PLAT_ERR_FUNC("conn_power_on fail(%d)\n", iRet);
		WMT_PLAT_DBG_FUNC("conn_power_on ok\n");
#else
		/* Never let a NULL clk read as success: clk_prepare_enable(NULL) returns
		 * 0. If mtk_wmt_probe never ran (e.g. blocked by the probe denylist in
		 * drivers/base/platform.c), clk_scp_conn_main stays NULL, SCP_SYS_CONN is
		 * never powered, and the chip-id poll that follows would touch an
		 * unpowered consys and wedge the AXI bus. Fail instead. */
		if (IS_ERR_OR_NULL(clk_scp_conn_main)) {
			pr_info("[FORGE_CONN] FATAL: clk_scp_conn_main is NULL -> mtk_wmt_probe never ran (check the probe-deny in drivers/base/platform.c). ABORT power-on; NOT touching consys regs.\n");
			return -1;
		}
		pr_info("[FORGE_CONN] mtcmos on: clk_prepare_enable(scp_sys_conn) enter\n");
		iRet = clk_prepare_enable(clk_scp_conn_main);
		pr_info("[FORGE_CONN] mtcmos on: clk_prepare_enable ret=%d\n", iRet);
		if (iRet) {
			WMT_PLAT_ERR_FUNC("clk_prepare_enable(clk_scp_conn_main) fail(%d)\n", iRet);
			return iRet;	/* never poll chip-id on an un-MTCMOS'd consys */
		}
		WMT_PLAT_DBG_FUNC("clk_prepare_enable(clk_scp_conn_main) ok\n");
#endif /* defined(CONFIG_MTK_LEGACY) */
#else
		/*2.write conn_top1_pwr_on=1, power on conn_top1 0x10006280 [2]  1'b1 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) | CONSYS_SPM_PWR_ON_BIT);
		/*3.read conn_top1_pwr_on_ack =1, power on ack ready 0x1000660C [1] */
		while (0 == (CONSYS_PWR_ON_ACK_BIT & CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_REG)))
			NULL;
		/*5.write conn_top1_pwr_on_s=1, power on conn_top1 0x10006280 [3]  1'b1 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) | CONSYS_SPM_PWR_ON_S_BIT);
		/*6.write conn_clk_dis=0, enable connsys clock 0x10006280 [4]  1'b0 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) & ~CONSYS_CLK_CTRL_BIT);
		/*7.wait 1us    */
		udelay(1);
		/*8.read conn_top1_pwr_on_ack_s =1, power on ack ready 0x10006610 [1]  */
		while (0 == (CONSYS_PWR_CONN_ACK_S_BIT & CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_S_REG)))
			NULL;
		/*9.release connsys ISO, conn_top1_iso_en=0 0x10006280 [1]  1'b0 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) & ~CONSYS_SPM_PWR_ISO_S_BIT);
		/*10.release SW reset of connsys, conn_ap_sw_rst_b=1 0x10006280[0] 1'b1 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) | CONSYS_SPM_PWR_RST_BIT);
		/*disable AXI BUS protect */
		CONSYS_REG_WRITE(CONSYS_TOPAXI_PROT_EN, CONSYS_REG_READ(CONSYS_TOPAXI_PROT_EN) & ~CONSYS_PROT_MASK);
		while (CONSYS_REG_READ(CONSYS_TOPAXI_PROT_STA1) & CONSYS_PROT_MASK)
			NULL;
#endif
		/*11.26M is ready now, delay 10us for mem_pd de-assert */
		udelay(10);
		/*enable AP bus clock : connmcu_bus_pd  API: enable_clock() ++?? */

		/*12.poll CONNSYS CHIP ID until 6755 is returned 0x18070008 32'h0326 */
		while (retry-- > 0) {
			WMT_PLAT_DBG_FUNC("CONSYS_CHIP_ID_REG(0x%08x)", CONSYS_REG_READ(CONSYS_CHIP_ID_REG));
			consysHwChipId = CONSYS_REG_READ(CONSYS_CHIP_ID_REG);
			if (consysHwChipId == 0x0326 /* 6755 consys chip id */) {
				WMT_PLAT_INFO_FUNC("retry(%d)consys chipId(0x%08x)\n", retry, consysHwChipId);
				break;
			}
			msleep(20);
		}

		if ((retry == 0) || (consysHwChipId == 0)) {
			pr_info("[FORGE_CONN] chipid poll FAILED (0x%08x)\n", consysHwChipId);
			WMT_PLAT_ERR_FUNC("Maybe has a consys power on issue,(0x%08x)\n", consysHwChipId);
			WMT_PLAT_INFO_FUNC("reg dump:CONSYS_CPU_SW_RST_REG(0x%x)\n",
					   CONSYS_REG_READ(CONSYS_CPU_SW_RST_REG));
			WMT_PLAT_INFO_FUNC("reg dump:CONSYS_PWR_CONN_ACK_REG(0x%x)\n",
					   CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_REG));
			WMT_PLAT_INFO_FUNC("reg dump:CONSYS_PWR_CONN_ACK_S_REG(0x%x)\n",
					   CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_S_REG));
			WMT_PLAT_INFO_FUNC("reg dump:CONSYS_TOP1_PWR_CTRL_REG(0x%x)\n",
					   CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG));
		}

		/*13.{default no need}update ROMDEL/PATCH RAM DELSEL if needed 0x18070114  */

		/*
		 *14.write 1 to conn_mcu_confg ACR[1] if real speed MBIST
		 *(default write "1") ACR 0x18070110[18] 1'b1
		 *if this bit is 0, HW will do memory auto test under low CPU frequence (26M Hz)
		 *if this bit is 0, HW will do memory auto test under high CPU frequence(138M Hz)
		 *inclulding low CPU frequence
		 */
		CONSYS_REG_WRITE(CONSYS_MCU_CFG_ACR_REG,
				 CONSYS_REG_READ(CONSYS_MCU_CFG_ACR_REG) | CONSYS_MCU_CFG_ACR_MBIST_BIT);

		/*update ANA_WBG(AFE) CR. AFE setting file:  AP Offset = 0x180B2000   */

#if CONSYS_AFE_REG_SETTING
		/*15.default no need,update ANA_WBG(AFE) CR if needed, CONSYS_AFE_REG */
		consys_afe_reg_base = ioremap_nocache(CONSYS_AFE_REG_BASE, 0x100);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_AFE_01_OFFSET,
				CONSYS_AFE_REG_WBG_AFE_01_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_PLL_03_OFFSET,
				CONSYS_AFE_REG_WBG_PLL_03_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_PLL_05_OFFSET,
				CONSYS_AFE_REG_WBG_PLL_05_VALUE);
		CONSYS_REG_WRITE(consys_afe_reg_base + CONSYS_AFE_REG_WBG_WB_TX_01_OFFSET,
				CONSYS_AFE_REG_WBG_WB_TX_01_VALUE);

		WMT_PLAT_DBG_FUNC("Dump AFE register\n");
		for (i = 0; i < 64; i++) {
			WMT_PLAT_DBG_FUNC("reg:0x%08x|val:0x%08x\n",
				CONSYS_AFE_REG_BASE + 4*i, CONSYS_REG_READ(consys_afe_reg_base + 4*i));
		}
#endif

		/*16.deassert CONNSYS CPU SW reset 0x10007018 "[12]=1'b0 [31:24] =8'h88(key)" */
		CONSYS_REG_WRITE(CONSYS_CPU_SW_RST_REG,
				 (CONSYS_REG_READ(CONSYS_CPU_SW_RST_REG) & ~CONSYS_CPU_SW_RST_BIT) |
				 CONSYS_CPU_SW_RST_CTRL_KEY);

#endif
		msleep(20);	/* msleep < 20ms can sleep for up to 20ms */
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x05F);	/* reg_ctrl ON complete */

	} else {
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x00F);	/* reg_ctrl OFF entry */

#ifdef CONFIG_OF



#if CONSYS_PWR_ON_OFF_API_AVAILABLE
#if defined(CONFIG_MTK_CLKMGR)
		/*power off connsys by API (MT6582, MT6572 are different) API: conn_power_off() */
		iRet = conn_power_off();	/* consult clkmgr owner */
		if (iRet)
			WMT_PLAT_ERR_FUNC("conn_power_off fail(%d)\n", iRet);
		WMT_PLAT_DBG_FUNC("conn_power_off ok\n");
#else
		/* Only unwind MTCMOS if it was enabled. A staged or aborted power-on
		 * leaves the CCF enable count at 0; a blind disable here would underflow
		 * it and WARN. */
		if (forge_conn_mtcmos_on && !IS_ERR_OR_NULL(clk_scp_conn_main)) {
			clk_disable_unprepare(clk_scp_conn_main);
			forge_conn_mtcmos_on = 0;
			WMT_PLAT_DBG_FUNC("clk_disable_unprepare(clk_scp_conn_main) calling\n");
		} else {
			pr_info("[FORGE_CONN] pwr_off: skip clk_disable (mtcmos not on)\n");
		}
#endif /* defined(CONFIG_MTK_LEGACY) */
#else
		{
			INT32 count = 0;

			CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_EN_OFFSET,
					 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_EN_OFFSET) |
					 CONSYS_PROT_MASK);
			while ((CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_TOPAXI_PROT_STA1_OFFSET) &
				CONSYS_PROT_MASK) != CONSYS_PROT_MASK) {
				count++;
				if (count > 1000)
					break;
			}
		}
		/*release connsys ISO, conn_top1_iso_en=1  0x1000632c [1]  1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_SPM_PWR_ISO_S_BIT);
		/*assert SW reset of connsys, conn_ap_sw_rst_b=0  0x1000632c[0] 1'b0 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~CONSYS_SPM_PWR_RST_BIT);
		/*write conn_clk_dis=1, disable connsys clock  0x1000632c [4]  1'b1 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) |
				 CONSYS_CLK_CTRL_BIT);
		/*wait 1us      */
		udelay(1);
		/*write conn_top1_pwr_on=0, power off conn_top1 0x1000632c [3:2] 2'b00 */
		CONSYS_REG_WRITE(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET,
				 CONSYS_REG_READ(conn_reg.spm_base + CONSYS_TOP1_PWR_CTRL_OFFSET) &
				 ~(CONSYS_SPM_PWR_ON_BIT | CONSYS_SPM_PWR_ON_S_BIT));

#endif

#else

#if CONSYS_PWR_ON_OFF_API_AVAILABLE


#if defined(CONFIG_MTK_CLKMGR)
		/*power off connsys by API: conn_power_off() */
		iRet = conn_power_off();	/* consult clkmgr owner */
		if (iRet)
			WMT_PLAT_ERR_FUNC("conn_power_off fail(%d)\n", iRet);
		WMT_PLAT_DBG_FUNC("conn_power_off ok\n");
#else
		clk_disable_unprepare(clk_scp_conn_main);
		WMT_PLAT_DBG_FUNC("clk_disable_unprepare(clk_scp_conn_main) calling\n");
#endif /* defined(CONFIG_MTK_LEGACY) */
#else
		{
			INT32 count = 0;

			CONSYS_REG_WRITE(CONSYS_TOPAXI_PROT_EN,
					 CONSYS_REG_READ(CONSYS_TOPAXI_PROT_EN) | CONSYS_PROT_MASK);
			while ((CONSYS_REG_READ(CONSYS_TOPAXI_PROT_STA1) & CONSYS_PROT_MASK) != CONSYS_PROT_MASK) {
				count++;
				if (count > 1000)
					break;
			}

		}
		/*release connsys ISO, conn_top1_iso_en=1 0x1000632c [1]  1'b1 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) | CONSYS_SPM_PWR_ISO_S_BIT);
		/*assert SW reset of connsys, conn_ap_sw_rst_b=0 0x1000632c[0] 1'b0  */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) & ~CONSYS_SPM_PWR_RST_BIT);
		/*write conn_clk_dis=1, disable connsys clock 0x1000632c [4]  1'b1 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG,
				 CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) | CONSYS_CLK_CTRL_BIT);
		/*wait 1us      */
		udelay(1);
		/*write conn_top1_pwr_on=0, power off conn_top1 0x1000632c [3:2] 2'b00 */
		CONSYS_REG_WRITE(CONSYS_TOP1_PWR_CTRL_REG, CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG) &
		~(CONSYS_SPM_PWR_ON_BIT | CONSYS_SPM_PWR_ON_S_BIT));
#endif

#endif

#if CONSYS_PMIC_CTRL_ENABLE
		if (co_clock_type) {
			/*VCN28 has been turned off by GPS OR FM */
			if (!is_clk_buf_from_pmic())
				clk_buf_ctrl(CLK_BUF_CONN, 0);
		} else {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_EN, 0);
			pmic_set_register_value(PMIC_RG_LDO_VCN28_HW0_OP_CFG, 0);
			#else
#if FORGE_CONN_MT6351_DIRECT
			FORGE_CONN_VCN("VCN28 ON_CTRL", MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
			FORGE_CONN_VCN("VCN28 EN", MT6351_PMIC_RG_VCN28_EN, 0);
#else
			pmic_set_register_value(MT6351_PMIC_RG_VCN28_ON_CTRL, 0);
#endif
			#endif
			/*turn off VCN28 LDO (with PMIC_WRAP API)" */
#if defined(CONFIG_MTK_LEGACY)
			hwPowerDown(MT6351_POWER_LDO_VCN28, "wcn_drv");
#else
			if (reg_VCN28) {
				if (regulator_disable(reg_VCN28))
					WMT_PLAT_ERR_FUNC("disable VCN_2V8 fail!\n");
				else
					WMT_PLAT_DBG_FUNC("disable VCN_2V8 ok\n");
			}
#endif
		}
		/* VCN33 is disabled in the paldo functions (stock shape);
		 * wmt_func_bt/wifi_off runs paldo(0) before this. */

		/*AP power off MT6351L VCN_1V8 LDO */
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_EN, 0);
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_CFG, 0);
		#else
#if FORGE_CONN_MT6351_DIRECT
		FORGE_CONN_VCN("VCN18 ON_CTRL", MT6351_PMIC_RG_VCN18_ON_CTRL, 0);
		FORGE_CONN_VCN("VCN18 EN", MT6351_PMIC_RG_VCN18_EN, 0);
#else
		pmic_set_register_value(MT6351_PMIC_RG_VCN18_ON_CTRL, 0);
#endif
		#endif
#if defined(CONFIG_MTK_LEGACY)
		hwPowerDown(MT6351_POWER_LDO_VCN18, "wcn_drv");
#else
		if (reg_VCN18) {
			if (regulator_disable(reg_VCN18))
				WMT_PLAT_ERR_FUNC("disable VCN_1V8 fail!\n");
			else
				WMT_PLAT_DBG_FUNC("disable VCN_1V8 ok\n");
		}
#endif

#endif

	}
	if (!on)
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x00E);	/* reg_ctrl OFF complete */
	WMT_PLAT_WARN_FUNC("CONSYS-HW-REG-CTRL(0x%08x),finish\n", on);
	return 0;
}

INT32 mtk_wcn_consys_hw_gpio_ctrl(UINT32 on)
{
	INT32 iRet = 0;

	WMT_PLAT_DBG_FUNC("CONSYS-HW-GPIO-CTRL(0x%08x), start\n", on);

	if (on) {

		/*if external modem used,GPS_SYNC still needed to control */
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_SYNC, PIN_STA_INIT);
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_LNA, PIN_STA_INIT);

		iRet += wmt_plat_gpio_ctrl(PIN_I2S_GRP, PIN_STA_INIT);

		/* TODO: [FixMe][GeorgeKuo] double check if BGF_INT is implemented ok */
		/* iRet += wmt_plat_gpio_ctrl(PIN_BGF_EINT, PIN_STA_MUX); */
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_INIT);
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_EINT_DIS);
		WMT_PLAT_DBG_FUNC("CONSYS-HW, BGF IRQ registered and disabled\n");

	} else {

		/* set bgf eint/all eint to deinit state, namely input low state */
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_EINT_DIS);
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_DEINIT);
		WMT_PLAT_DBG_FUNC("CONSYS-HW, BGF IRQ unregistered and disabled\n");
		/* iRet += wmt_plat_gpio_ctrl(PIN_BGF_EINT, PIN_STA_DEINIT); */

		/*if external modem used,GPS_SYNC still needed to control */
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_SYNC, PIN_STA_DEINIT);
		iRet += wmt_plat_gpio_ctrl(PIN_I2S_GRP, PIN_STA_DEINIT);
		/* deinit gps_lna */
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_LNA, PIN_STA_DEINIT);

	}
	WMT_PLAT_INFO_FUNC("CONSYS-HW-GPIO-CTRL(0x%08x), finish\n", on);
	return iRet;

}

INT32 mtk_wcn_consys_hw_pwr_on(UINT32 co_clock_type)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-ON, start\n");
	if (!gConEmiPhyBase) {
		WMT_PLAT_ERR_FUNC("EMI base address is invalid, CONNSYS can not be powered on!");
		WMT_PLAT_ERR_FUNC("To avoid the occurrence of KE!\n");
		return -1;
	}

	/* Propagate an aborted or staged power-on. Stock summed the return codes
	 * and carried on into gpio/eirq setup even when the rails/MTCMOS/chip-id
	 * step had failed. Bail out instead, and say so. */
	iRet = mtk_wcn_consys_hw_reg_ctrl(1, co_clock_type);
	if (iRet) {
		pr_emerg("[FORGE_CONN] pwr_on aborted (%d) - skipping gpio/eirq\n", iRet);
		return iRet;
	}
	iRet += mtk_wcn_consys_hw_gpio_ctrl(1);
	forge_conn_gpio_on = 1;
#if CONSYS_ENALBE_SET_JTAG
	if (gJtagCtrl)
		mtk_wcn_consys_jtag_set_for_mcu();
#endif
	forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x060);	/* pwr_on complete (gpio/eirq done) */
	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-ON, finish(%d)\n", iRet);
	return iRet;
}

INT32 mtk_wcn_consys_hw_pwr_off(UINT32 co_clock_type)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-OFF, start\n");

	iRet += mtk_wcn_consys_hw_reg_ctrl(0, co_clock_type);
	/* Only unwind gpio/eirq if pwr_on set them up (see forge_conn_gpio_on
	 * above). reg_ctrl(0) stays unconditional: its clk_disable path already
	 * checks forge_conn_mtcmos_on. */
	if (forge_conn_gpio_on) {
		iRet += mtk_wcn_consys_hw_gpio_ctrl(0);
		forge_conn_gpio_on = 0;
	} else {
		pr_emerg("[FORGE_CONN] pwr_off: gpio/eirq never inited (staged abort) - skipping their teardown\n");
	}

	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-OFF, finish(%d)\n", iRet);
	return iRet;
}

INT32 mtk_wcn_consys_hw_rst(UINT32 co_clock_type)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW, hw_rst start, eirq should be disabled before this step\n");

	/*1. do whole hw power off flow */
	iRet += mtk_wcn_consys_hw_reg_ctrl(0, co_clock_type);

	/*2. do whole hw power on flow */
	iRet += mtk_wcn_consys_hw_reg_ctrl(1, co_clock_type);

	WMT_PLAT_INFO_FUNC("CONSYS-HW, hw_rst finish, eirq should be enabled after this step\n");
	return iRet;
}

#if CONSYS_BT_WIFI_SHARE_V33
INT32 mtk_wcn_consys_hw_bt_paldo_ctrl(UINT32 enable)
{
#if 0
	/* spin_lock_irqsave(&gBtWifiV33.lock,gBtWifiV33.flags); */
	if (enable) {
		if (gBtWifiV33.counter == 1) {
			gBtWifiV33.counter++;
			WMT_PLAT_DBG_FUNC("V33 has been enabled,counter(%d)\n", gBtWifiV33.counter);
		} else if (gBtWifiV33.counter == 2) {
			WMT_PLAT_DBG_FUNC("V33 has been enabled,counter(%d)\n", gBtWifiV33.counter);
		} else {
#if CONSYS_PMIC_CTRL_ENABLE
			/*do BT PMIC on,depenency PMIC API ready */
			/*switch BT PALDO control from SW mode to HW mode:0x416[5]-->0x1 */
			/* VOL_DEFAULT, VOL_3300, VOL_3400, VOL_3500, VOL_3600 */
			hwPowerOn(MT6351_POWER_LDO_VCN33_BT, VOL_3300 * 1000, "wcn_drv");
			mt6351_upmu_set_rg_vcn33_on_ctrl(1);
#endif
			WMT_PLAT_INFO_FUNC("WMT do BT/WIFI v3.3 on\n");
			gBtWifiV33.counter++;
		}

	} else {
		if (gBtWifiV33.counter == 1) {
			/*do BT PMIC off */
			/*switch BT PALDO control from HW mode to SW mode:0x416[5]-->0x0 */
#if CONSYS_PMIC_CTRL_ENABLE
			mt6351_upmu_set_rg_vcn33_on_ctrl(0);
			hwPowerDown(MT6351_POWER_LDO_VCN33_BT, "wcn_drv");
#endif
			WMT_PLAT_INFO_FUNC("WMT do BT/WIFI v3.3 off\n");
			gBtWifiV33.counter--;
		} else if (gBtWifiV33.counter == 2) {
			gBtWifiV33.counter--;
			WMT_PLAT_DBG_FUNC("V33 no need disabled,counter(%d)\n", gBtWifiV33.counter);
		} else {
			WMT_PLAT_DBG_FUNC("V33 has been disabled,counter(%d)\n", gBtWifiV33.counter);
		}

	}
#endif
	/* spin_unlock_irqrestore(&gBtWifiV33.lock,gBtWifiV33.flags); */
	return 0;
}

INT32 mtk_wcn_consys_hw_wifi_paldo_ctrl(UINT32 enable)
{
	mtk_wcn_consys_hw_bt_paldo_ctrl(enable);
	return 0;
}

#else
/* paldo functions, as in stock: VCN33, the BT/WiFi RF PA rail, is owned
 * here at func-on, not in reg_ctrl. The regulator handles are dummies (no
 * *-supply in DT), so the rail is driven with the same MT6351
 * FORGE_CONN_VCN direct writes the reg_ctrl rails use: EN/ON_CTRL pairs at
 * LDO_VCN33_CON3 (0x0A98, BT) / CON4 (0x0A9A, WIFI), EN shift 1, ON_CTRL
 * shift 3, as in stock. ON_CTRL = 1 = HW mode (the rail follows the
 * BT/WIFI srclken). */
#if CONSYS_PMIC_CTRL_ENABLE && FORGE_CONN_MT6351_DIRECT
/* VCN33 after a BT/WIFI EN write, while the rail is still in SW mode (before
 * ON_CTRL hands it to the srclken): a refused EN write, or a rail that does
 * not come up, ends the func-on here - wmt_func stops on a paldo error -
 * instead of starting the BT/WIFI subsystem without its PA supply.
 * DA_QI_VCN33_EN is the rail itself (either EN); bounded: 10 x 50 us. */
static bool forge_conn_vcn33_on(const char *what, int wr)
{
	unsigned int on = 0;
	int i;

	for (i = 0; !wr && i < 10; i++) {
		pmic_read_interface_nolock(MT6351_PMIC_DA_QI_VCN33_EN_ADDR, &on,
				    MT6351_PMIC_DA_QI_VCN33_EN_MASK,
				    MT6351_PMIC_DA_QI_VCN33_EN_SHIFT);
		if (on)
			break;
		udelay(50);
	}
	pr_info("[FORGE_CONN] %s: EN write %d, DA_QI_VCN33_EN %s\n",
		what, wr, on ? "on" : "OFF");
	return !wr && on;
}
#endif

INT32 mtk_wcn_consys_hw_bt_paldo_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE && FORGE_CONN_MT6351_DIRECT
	if (enable) {
		if (!forge_conn_vosel_ok(FORGE_VCN33))
			return -EIO;
		if (!forge_conn_vcn33_on("VCN33_BT",
			FORGE_CONN_VCN("VCN33_BT EN", MT6351_PMIC_RG_VCN33_EN_BT, 1))) {
			FORGE_CONN_VCN("VCN33_BT EN", MT6351_PMIC_RG_VCN33_EN_BT, 0);
			return -EIO;
		}
		FORGE_CONN_VCN("VCN33_BT ON_CTRL", MT6351_PMIC_RG_VCN33_ON_CTRL_BT, 1);
		WMT_PLAT_INFO_FUNC("WMT do BT PMIC on\n");
	} else {
		FORGE_CONN_VCN("VCN33_BT ON_CTRL", MT6351_PMIC_RG_VCN33_ON_CTRL_BT, 0);
		FORGE_CONN_VCN("VCN33_BT EN", MT6351_PMIC_RG_VCN33_EN_BT, 0);
		WMT_PLAT_INFO_FUNC("WMT do BT PMIC off\n");
	}
#endif
	return 0;

}

INT32 mtk_wcn_consys_hw_wifi_paldo_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE && FORGE_CONN_MT6351_DIRECT
	if (enable) {
		if (!forge_conn_vosel_ok(FORGE_VCN33))
			return -EIO;
		if (!forge_conn_vcn33_on("VCN33_WIFI",
			FORGE_CONN_VCN("VCN33_WIFI EN", MT6351_PMIC_RG_VCN33_EN_WIFI, 1))) {
			FORGE_CONN_VCN("VCN33_WIFI EN", MT6351_PMIC_RG_VCN33_EN_WIFI, 0);
			return -EIO;
		}
		FORGE_CONN_VCN("VCN33_WIFI ON_CTRL", MT6351_PMIC_RG_VCN33_ON_CTRL_WIFI, 1);
		WMT_PLAT_INFO_FUNC("WMT do WIFI PMIC on\n");
	} else {
		FORGE_CONN_VCN("VCN33_WIFI ON_CTRL", MT6351_PMIC_RG_VCN33_ON_CTRL_WIFI, 0);
		FORGE_CONN_VCN("VCN33_WIFI EN", MT6351_PMIC_RG_VCN33_EN_WIFI, 0);
		WMT_PLAT_INFO_FUNC("WMT do WIFI PMIC off\n");
	}
#endif
	return 0;

}

#endif
INT32 mtk_wcn_consys_hw_vcn28_ctrl(UINT32 enable)
{
	if (enable) {
		/*in co-clock mode,need to turn on vcn28 when fm on */
#if CONSYS_PMIC_CTRL_ENABLE
#if defined(CONFIG_MTK_LEGACY)
		hwPowerOn(MT6351_POWER_LDO_VCN28, VOL_2800 * 1000, "wcn_drv");
#else
		if (reg_VCN28) {
			regulator_set_voltage(reg_VCN28, 2800000, 2800000);
			if (regulator_enable(reg_VCN28))
				WMT_PLAT_ERR_FUNC("WMT do VCN28 PMIC on fail!\n");
		}
#endif
#endif
#if FORGE_CONN_MT6351_DIRECT
		if (!forge_conn_vosel_ok(FORGE_VCN28))
			return -EIO;
		FORGE_CONN_VCN("VCN28 EN (fm)", MT6351_PMIC_RG_VCN28_EN, 1);
#endif
		WMT_PLAT_INFO_FUNC("turn on vcn28 for fm/gps usage in co-clock mode\n");
	} else {
		/*in co-clock mode,need to turn off vcn28 when fm off */
#if CONSYS_PMIC_CTRL_ENABLE
#if defined(CONFIG_MTK_LEGACY)
		hwPowerDown(MT6351_POWER_LDO_VCN28, "wcn_drv");
#else
		if (reg_VCN28)
			regulator_disable(reg_VCN28);
#endif
#endif
#if FORGE_CONN_MT6351_DIRECT
		FORGE_CONN_VCN("VCN28 EN (fm)", MT6351_PMIC_RG_VCN28_EN, 0);
#endif
		WMT_PLAT_INFO_FUNC("turn off vcn28 for fm/gps usage in co-clock mode\n");
	}
	return 0;
}

INT32 mtk_wcn_consys_hw_state_show(VOID)
{
	return 0;
}

INT32 mtk_wcn_consys_hw_restore(struct device *device)
{
	UINT32 addrPhy = 0;

	/* Only re-apply EMI state that the staged power-on established
	 * (CONFIG_MTK_HIBERNATION is off, so this is compiled but unregistered). */
	if (forge_conn_emi_done != 0x7u) {
		pr_err("[FORGE_CONN] hw_restore: EMI never staged (forge_conn_emi=0x%x) - skip\n",
		       forge_conn_emi_done);
		return 0;
	}

	if (gConEmiPhyBase) {

#if CONSYS_EMI_MPU_SETTING
		/*set MPU for EMI share Memory */
		WMT_PLAT_INFO_FUNC("setting MPU for EMI share memory\n");

		emi_mpu_set_region_protection(gConEmiPhyBase + SZ_1M / 2,
							gConEmiPhyBase + gConEmiSize - 1,
							13,
							SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
							FORBIDDEN, NO_PROTECTION, FORBIDDEN, NO_PROTECTION));

#endif
		/*consys to ap emi remapping register:10001340, cal remapping address */
		/* MT6755 remap granularity is 1MB: bits[11:0] = phys[31:20], as in the
		 * stock mt6755 glue ((base & 0xFFF00000) >> 20). The mt6757 formula (>>21,
		 * 2MB granularity) points the consys window at foreign RAM and the consys
		 * ROM loops in an exception. */
		addrPhy = ((UINT32)(gConEmiPhyBase & 0xFFF00000)) >> 20;

		/*enable consys to ap emi remapping bit12 */
		addrPhy = addrPhy | 0x1000;

		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET,
				 CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET) | addrPhy);

		WMT_PLAT_INFO_FUNC("CONSYS_EMI_MAPPING dump in restore cb(0x%08x)\n",
				   CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET));

#if 1
		pEmibaseaddr = ioremap_nocache(gConEmiPhyBase + SZ_1M / 2, CONSYS_EMI_MEM_SIZE);
#else
		pEmibaseaddr = ioremap_nocache(CONSYS_EMI_AP_PHY_BASE, CONSYS_EMI_MEM_SIZE);
#endif
		if (pEmibaseaddr) {
			WMT_PLAT_WARN_FUNC("EMI mapping OK(0x%p)\n", pEmibaseaddr);
			memset_io(pEmibaseaddr, 0, CONSYS_EMI_MEM_SIZE);
		} else {
			WMT_PLAT_ERR_FUNC("EMI mapping fail\n");
		}
	} else {
		WMT_PLAT_ERR_FUNC("consys emi memory address gConEmiPhyBase invalid\n");
	}

	return 0;
}

/*Reserved memory by device tree!*/
int reserve_memory_consys_fn(struct reserved_mem *rmem)
{
	WMT_PLAT_WARN_FUNC(" name: %s, base: 0x%llx, size: 0x%llx\n", rmem->name,
			   (unsigned long long)rmem->base, (unsigned long long)rmem->size);
	gConEmiPhyBase = rmem->base;
	gConEmiSize = rmem->size;
	return 0;
}

RESERVEDMEM_OF_DECLARE(reserve_memory_test, "mediatek,consys-reserve-memory", reserve_memory_consys_fn);

/* The deferred EMI executor; see the forge_conn_emi knob block above for
 * the contract. Caller holds forge_conn_emi_lock (sysfs setter) or is
 * single-threaded (hw restore). Sub-steps run in ascending bit order = the
 * stock hw_init order (MPU SMC -> remap -> memset). */
static INT32 forge_conn_emi_apply(unsigned int req)
{
	unsigned int todo = req & 0x7u & ~forge_conn_emi_done;
	UINT32 addrPhy = 0;
	UINT32 remap = 0;

	pr_err("[FORGE_CONN] emi_apply: req=0x%x done=0x%x todo=0x%x\n",
	       req, forge_conn_emi_done, todo);
	if (!todo)
		return 0;
	if (!conn_reg.topckgen_base) {
		pr_err("[FORGE_CONN] emi_apply REFUSED: hw_init has not run (no iomap) - run DO_MODULE_INIT (wmt_kick) first\n");
		return -EAGAIN;
	}
	if (!gConEmiPhyBase) {
		pr_err("[FORGE_CONN] emi_apply REFUSED: gConEmiPhyBase invalid (no mediatek,consys-reserve-memory)\n");
		return -ENODEV;
	}

	if (todo & 0x1u) {
#if CONSYS_EMI_MPU_SETTING
		pr_err("[FORGE_CONN] emi step1 (H-A) ENTER: EMI-MPU region-13 SMC, base=0x%llx size=0x%llx\n",
		       (unsigned long long)gConEmiPhyBase,
		       (unsigned long long)gConEmiSize);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F2);	/* EMI-MPU SMC pre */
		emi_mpu_set_region_protection(gConEmiPhyBase + SZ_1M / 2,
					      gConEmiPhyBase + gConEmiSize - 1,
					      13,
					      SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					      FORBIDDEN, NO_PROTECTION, FORBIDDEN, NO_PROTECTION));
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F3);	/* EMI-MPU SMC post */
		pr_err("[FORGE_CONN] emi step1 (H-A) DONE\n");
#else
		pr_err("[FORGE_CONN] emi step1 (H-A) NO-OP: CONSYS_EMI_MPU_SETTING disabled\n");
#endif
		forge_conn_emi_done |= 0x1u;
	}

	if (todo & 0x2u) {
		/*consys to ap emi remapping register, cal remapping address */
		/* MT6755 remap granularity is 1MB: bits[11:0] = phys[31:20] (stock:
		 * (base & 0xFFF00000) >> 20), not the mt6757 >>21 / 2MB formula, which
		 * points the consys EMI window outside the MPU region-13 grant. Replace
		 * bits[12:0] instead of OR so a re-run can fix a previously wrong value. */
		addrPhy = (((UINT32)(gConEmiPhyBase & 0xFFF00000)) >> 20);
		/*enable consys to ap emi remapping bit12 */
		addrPhy |= 0x1000;
		pr_err("[FORGE_CONN] emi step2 (H-B) ENTER: remap write topckgen+0x%x [12:0]= 0x%04x\n",
		       CONSYS_EMI_MAPPING_OFFSET, addrPhy);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F4);	/* EMI remap pre */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET,
				 (CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET) &
				  ~0x1FFFu) | addrPhy);
		remap = CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F5);	/* EMI remap post */
		pr_err("[FORGE_CONN] emi step2 (H-B) DONE: CONSYS_EMI_MAPPING=0x%08x\n", remap);
		forge_conn_emi_done |= 0x2u;
	}

	if (todo & 0x4u) {
		if (!pEmibaseaddr)
			pEmibaseaddr = ioremap_nocache(gConEmiPhyBase + SZ_1M / 2, CONSYS_EMI_MEM_SIZE);
		if (!pEmibaseaddr) {
			pr_err("[FORGE_CONN] emi step3 (H-D) FAILED: ioremap_nocache fail\n");
			return -ENOMEM;
		}
		pr_err("[FORGE_CONN] emi step3 (H-D) ENTER: memset_io %uK @%p\n",
		       (unsigned int)(CONSYS_EMI_MEM_SIZE >> 10), pEmibaseaddr);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F6);	/* EMI memset pre */
		memset_io(pEmibaseaddr, 0, CONSYS_EMI_MEM_SIZE);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F7);	/* EMI memset post */
		pr_err("[FORGE_CONN] emi step3 (H-D) DONE\n");
		forge_conn_emi_done |= 0x4u;
	}

	return 0;
}

INT32 mtk_wcn_consys_hw_init(void)
{

	INT32 iRet = -1;
	struct device_node *node = NULL;

	/* This runs at DO_MODULE_INIT time (wmt_kick / wmt_loader), not at boot,
	 * and touches hardware before the forge_conn_pwron_stage gate in reg_ctrl:
	 * the EMI-MPU SMC into ATF, the EMI remap write through the of_iomap'd
	 * infracfg window, and the memset_io of the consys EMI carveout. Every
	 * step is bracketed with WDT-surviving tracer marks. */
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F0);
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-consys");
	if (node) {
		/* registers base address */
		conn_reg.mcu_base = (SIZE_T) of_iomap(node, 0);
		WMT_PLAT_DBG_FUNC("Get mcu register base(0x%zx)\n", conn_reg.mcu_base);
		conn_reg.ap_rgu_base = (SIZE_T) of_iomap(node, 1);
		WMT_PLAT_DBG_FUNC("Get ap_rgu register base(0x%zx)\n", conn_reg.ap_rgu_base);
		conn_reg.topckgen_base = (SIZE_T) of_iomap(node, 2);
		WMT_PLAT_DBG_FUNC("Get topckgen register base(0x%zx)\n", conn_reg.topckgen_base);
		conn_reg.spm_base = (SIZE_T) of_iomap(node, 3);
		WMT_PLAT_DBG_FUNC("Get spm register base(0x%zx)\n", conn_reg.spm_base);
	} else {
		WMT_PLAT_ERR_FUNC("[%s] can't find CONSYS compatible node\n", __func__);
		return iRet;
	}
	/* step 0x0F1 carries a NULL-base nibble mask: bit0=mcu bit1=ap_rgu
	 * bit2=topckgen bit3=spm. Non-zero low nibble = an of_iomap failed and
	 * any later CONSYS_REG_* on that window is a NULL-deref paging panic. */
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F100 |
			 (conn_reg.mcu_base ? 0 : 1) |
			 (conn_reg.ap_rgu_base ? 0 : 2) |
			 (conn_reg.topckgen_base ? 0 : 4) |
			 (conn_reg.spm_base ? 0 : 8));
	pr_info("[FORGE_CONN] hw_init: bases mcu=%d rgu=%d topck=%d spm=%d emi_base=0x%llx size=0x%llx\n",
		!!conn_reg.mcu_base, !!conn_reg.ap_rgu_base,
		!!conn_reg.topckgen_base, !!conn_reg.spm_base,
		(unsigned long long)gConEmiPhyBase,
		(unsigned long long)gConEmiSize);
	if (!conn_reg.topckgen_base || !conn_reg.spm_base ||
	    !conn_reg.mcu_base || !conn_reg.ap_rgu_base) {
		WMT_PLAT_ERR_FUNC("consys of_iomap failed - abort hw_init\n");
		return -1;
	}
	forge_conn_pmic_snapshot(NULL, 0);

	if (gConEmiPhyBase) {
		/* No hardware access here: the stock EMI steps (EMI-MPU SMC, remap
		 * write, memset_io) are deferred behind the forge_conn_emi knob, see
		 * forge_conn_emi_apply() above. This function is reachable on every
		 * boot through the connectivity autostart as soon as anything issues
		 * DO_MODULE_INIT, so it only registers software state. */
		pr_err("[FORGE_CONN] hw_init: EMI init DEFERRED (done=0x%x; echo 7 > /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_emi before stage 4; bit0=MPU-SMC bit1=remap bit2=memset)\n",
		       forge_conn_emi_done);
		iRet = 0;
	} else {
		WMT_PLAT_ERR_FUNC("consys emi memory address gConEmiPhyBase invalid\n");
	}
#ifdef CONFIG_MTK_HIBERNATION
	WMT_PLAT_INFO_FUNC("register connsys restore cb for complying with IPOH function\n");
	register_swsusp_restore_noirq_func(ID_M_CONNSYS, mtk_wcn_consys_hw_restore, NULL);
#endif

	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F8);
	iRet = platform_driver_register(&mtk_wmt_dev_drv);
	if (iRet)
		WMT_PLAT_ERR_FUNC("WMT platform driver registered failed(%d)\n", iRet);
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F9);

	return iRet;

}

INT32 mtk_wcn_consys_hw_deinit(void)
{
	if (pEmibaseaddr) {
		iounmap(pEmibaseaddr);
		pEmibaseaddr = NULL;
	}
#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_CONNSYS);
#endif

	platform_driver_unregister(&mtk_wmt_dev_drv);
	return 0;
}

UINT8 *mtk_wcn_consys_emi_virt_addr_get(UINT32 ctrl_state_offset)
{
	UINT8 *p_virtual_addr = NULL;

	if (!pEmibaseaddr) {
		WMT_PLAT_ERR_FUNC("EMI base address is NULL\n");
		return NULL;
	}
	WMT_PLAT_DBG_FUNC("ctrl_state_offset(%08x)\n", ctrl_state_offset);
	p_virtual_addr = pEmibaseaddr + ctrl_state_offset;

	return p_virtual_addr;
}

UINT32 mtk_wcn_consys_soc_chipid(void)
{
	return PLATFORM_SOC_CHIP;
}

#if !defined(CONFIG_MTK_LEGACY)
struct pinctrl *mtk_wcn_consys_get_pinctrl()
{
	return consys_pinctrl;
}
#endif
INT32 mtk_wcn_consys_set_dbg_mode(UINT32 flag)
{
	INT32 ret = -1;
	PUINT8 vir_addr = NULL;

	vir_addr = mtk_wcn_consys_emi_virt_addr_get(EXP_APMEM_CTRL_CHIP_FW_DBGLOG_MODE);
	if (!vir_addr) {
		WMT_PLAT_ERR_FUNC("get vir address fail\n");
		return -2;
	}
	if (flag) {
		ret = 0;
		CONSYS_REG_WRITE(vir_addr, 0x1);
	} else {
		CONSYS_REG_WRITE(vir_addr, 0x0);
	}
	WMT_PLAT_ERR_FUNC("fw dbg mode register value(0x%08x)\n", CONSYS_REG_READ(vir_addr));
	return ret;
}
INT32 mtk_wcn_consys_set_dynamic_dump(PUINT32 str_buf)
{
	PUINT8 vir_addr = NULL;

	vir_addr = mtk_wcn_consys_emi_virt_addr_get(EXP_APMEM_CTRL_CHIP_DYNAMIC_DUMP);
	if (!vir_addr) {
		WMT_PLAT_ERR_FUNC("get vir address fail\n");
		return -2;
	}
	memcpy(vir_addr, str_buf, DYNAMIC_DUMP_GROUP_NUM*8);
	WMT_PLAT_INFO_FUNC("dynamic dump register value(0x%08x)\n", CONSYS_REG_READ(vir_addr));
	return 0;
}
