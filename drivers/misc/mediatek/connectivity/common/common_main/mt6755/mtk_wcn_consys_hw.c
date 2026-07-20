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

/* Connectivity power-on tracer (DRAM, survives a WDT reset; see the header
 * for readback). Brackets the EMI-MPU SMC into ATF (H-A), the EMI remap
 * write via the of_iomap'd topckgen base (H-B) and the memset_io of the
 * consys EMI carveout (H-D). */
#include "../../../forge_conn_trace.h"

/* mt_clkbuf_ctl.c provides the strong is_clk_buf_from_pmic/clk_buf_ctrl;
 * these weak versions only matter for configs without it. Do not rely on
 * weak-vs-weak ordering: ccci_util_dummy.c has a weak version returning
 * false, and link order picks between weak definitions. */
bool __weak is_clk_buf_from_pmic(void)
{
	return true;
}

bool __weak clk_buf_ctrl(enum clk_buf_id id, bool onoff)
{
	pr_info("[FORGE_CONN] clk_buf_ctrl(%d,%d) skipped (clkbuf deferred)\n",
		id, onoff);
	return false;
}

#if CONSYS_PMIC_CTRL_ENABLE && defined(CONFIG_MTK_PMIC_CHIP_MT6353)
/* The physical PMIC is MT6351, driven by the MT6353 driver
 * (CONFIG_MTK_PMIC_CHIP_MT6353=y). Neither stock rail path works here:
 *   - hwPowerOn/hwPowerDown are no-ops outside CONFIG_MTK_LEGACY
 *     (pmic/mt6353/pmic_regulator.c), and the vcn18/vcn28/vcn33_* regulator
 *     handles are dummies (the *-supply properties in the mt6755.dtsi
 *     consys node are commented out);
 *   - the MT6353 VCN *_EN flags (PMIC_LDO_VCN18_EN etc., mt6353_hw.h)
 *     point at MT6353-only registers (LDO3_* 0x0A4C page) that do not
 *     exist on MT6351.
 * So the rails are driven directly over pwrap with the MT6351 macros from
 * mach/mt6351_regs.h (matching the 3.10 stock register map). Every write
 * is logged with a [FORGE_CONN] line. */
static void forge_conn_vcn_write(const char *what, unsigned int addr,
				 unsigned int val, unsigned int mask,
				 unsigned int shift)
{
	pr_info("[FORGE_CONN] pmic %s: reg 0x%x <= %u (mask 0x%x shift %u)\n",
		what, addr, val, mask, shift);
	pmic_config_interface(addr, val, mask, shift);
}

#define FORGE_CONN_VCN(what, rg, val) \
	forge_conn_vcn_write(what, rg##_ADDR, val, rg##_MASK, rg##_SHIFT)

/* Enable the MT6351 DCXO XO_WCN (EXTBUF2) 26 MHz buffer when the clock
 * buffers come from the PMIC.
 *
 * clk_buf_init() (mt_clkbuf_ctl.c, late_initcall) already performs the full
 * init (CW00 = 0x4DFD, CW13 drive strength, pwrap DCXO_CONN srclkena
 * engine), so this one-shot is idempotent; its pre/post CW00 readback right
 * before the rails shows whether that init landed. PMIC_CW00_ADDR 0x7000
 * and the 0x4DFD init value are the stock ones.
 *
 * Only one register is written, at runtime, behind the stage gate. The
 * pwrap DCXO_CONN engine (+0x18C..0x19C) and CW13 drive strength are not
 * touched here.
 */
static void forge_conn_dcxo_init(void)
{
	static int done;
	unsigned int cw00 = 0;

	if (done)
		return;
	pmic_read_interface(0x7000, &cw00, 0xFFFF, 0);
	/* M681 is an RF-VCTCXO (TCXO) board: is_clk_buf_from_pmic() returns
	 * false ("clkbuf is from RF", CW00 = 0x21f), the MT6351 DCXO is off
	 * and XO_WCN/EXTBUF2 is not the consys 26 MHz source (the stock
	 * WMT_SOC.cfg has co_clock_flag=0). Write only when the clock buffers
	 * come from the PMIC; otherwise this is a logged readback. */
	if (!is_clk_buf_from_pmic()) {
		pr_emerg("[FORGE_CONN] DCXO CW00=0x%x, clkbuf from RF -> NOT writing 0x4dfd (DCXO is not the consys 26M source)\n",
			 cw00);
		done = 1;
		return;
	}
	pr_emerg("[FORGE_CONN] MT6351 DCXO CW00 pre=0x%x\n", cw00);
	pmic_config_interface(0x7000, 0x4DFD, 0xFFFF, 0);
	pmic_read_interface(0x7000, &cw00, 0xFFFF, 0);
	pr_emerg("[FORGE_CONN] MT6351 DCXO CW00 post=0x%x (want 0x4dfd) -> XO_WCN 26M on\n",
		 cw00);
	done = 1;
}
#endif

/* Staged consys power-on gate, default 0. A failed consys power-on resets
 * the board, and WMT_init's fb notifier (wmt_dev.c) would retry it on the
 * first screen blank after DO_MODULE_INIT, so by default consys is never
 * touched.
 * Stages: 0 = block before any access; 1 = PMIC rails only; 2 = + MTCMOS
 * and PWR_STATUS readback; 3 = + chip-id poll (gated on PWR_STATUS);
 * 4 = full.
 * Runtime: /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_pwron_stage
 * cmdline: mtk_wcn_consys_hw.forge_conn_pwron_stage=N
 * Markers land in expdb/last_kmsg. */
static int forge_conn_pwron_stage;	/* 0 = SAFE */
module_param(forge_conn_pwron_stage, int, 0644);
static int forge_conn_mtcmos_on;
/* pwr_off must only tear down gpio/eirq if pwr_on set them up: a staged
 * abort fails reg_ctrl(1) before gpio_ctrl(1), and the stock unconditional
 * gpio_ctrl(0) in pwr_off would then deinit pins/eirqs that were never
 * initialised (see gps_lna in wmt_plat_alps.c). */
static int forge_conn_gpio_on;

/* Keep all EMI hardware work out of mtk_wcn_consys_hw_init(). The userspace
 * connectivity autostart reaches DO_MODULE_INIT on every boot, so anything
 * it can reach must not touch hardware until explicitly staged. hw_init
 * used to do three hardware accesses at DO_MODULE_INIT time, all before the
 * reg_ctrl stage gate:
 *   bit0 (H-A) EMI-MPU region-13 SMC into ATF (emi_mpu_set_region_protection)
 *   bit1 (H-B) consys->AP EMI remap write + readback (topckgen + 0x1340)
 *   bit2 (H-D) 343K memset_io over the consys EMI carveout
 * With forge_conn_emi_done = 0, hw_init does no hardware access: it only
 * of_iomap's the register windows and registers the mtk_wmt platform driver
 * (probe = clk/regulator/pinctrl handle lookups, no MMIO).
 * Each deferred step runs in the writer's context when its bit is first
 * written to /sys/module/mtk_wcn_consys_hw/parameters/forge_conn_emi,
 * bracketed by tracer and pr_err markers, so a hang shows which step it
 * was. Bits are one-way (a done step is never re-run). All three bits
 * (write 7) are required before power-on stage 4: releasing the CONNSYS CPU
 * without the EMI remap would let the consys MCU run against an
 * unconfigured EMI window. */
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
	reg_VCN18 = regulator_get(&pdev->dev, "vcn18");
	if (!reg_VCN18)
		WMT_PLAT_ERR_FUNC("Regulator_get VCN_1V8 fail\n");
	reg_VCN28 = regulator_get(&pdev->dev, "vcn28");
	if (!reg_VCN28)
		WMT_PLAT_ERR_FUNC("Regulator_get VCN_2V8 fail\n");
	reg_VCN33_BT = regulator_get(&pdev->dev, "vcn33_bt");
	if (!reg_VCN33_BT)
		WMT_PLAT_ERR_FUNC("Regulator_get VCN33_BT fail\n");
	reg_VCN33_WIFI = regulator_get(&pdev->dev, "vcn33_wifi");
	if (!reg_VCN33_WIFI)
		WMT_PLAT_ERR_FUNC("Regulator_get VCN33_WIFI fail\n");
#endif
#endif

#if !defined(CONFIG_MTK_LEGACY)
	consys_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(consys_pinctrl)) {
		WMT_PLAT_ERR_FUNC("cannot find consys pinctrl.\n");
		return PTR_ERR(consys_pinctrl);
	}
#endif /* !defined(CONFIG_MTK_LEGACY) */

	/* Probe marker: proves mtk_wmt_probe ran and clk_scp_conn_main is
	 * real (clk_prepare_enable(NULL) returns 0, see reg_ctrl). */
	pr_info("[FORGE_CONN] mtk_wmt_probe OK: clk_scp_conn=%p (deny retirement positive marker)\n",
		clk_scp_conn_main);
	forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0FB);
	return 0;
}

static INT32 mtk_wmt_remove(struct platform_device *pdev)
{
	return 0;
}

INT32 mtk_wcn_consys_co_clock_type(VOID)
{
	UINT32 retval = 0;
	UINT32 back_up = 0;
	UINT32 co_clock_type = 0;

	/* co-clock auto detection:backup cw15,write cw15,read cw16,restore cw15, */
	pmic_read_interface(PMIC_DCXO_CW15, &retval, 0xFFFF, 0);
	back_up = retval;
	pmic_config_interface(PMIC_DCXO_CW15, PMIC_DCXO_CW15_VAL, 0xFFFF, 0);
	pmic_read_interface(PMIC_DCXO_CW16, &retval, 0xFFFF, 0);
	pmic_config_interface(PMIC_DCXO_CW15, back_up, 0xFFFF, 0);
	if ((retval & AP_CONSYS_NOCO_CLOCK_BITA) || (retval & AP_CONSYS_NOCO_CLOCK_BITB)) {
		co_clock_type = 0;
		WMT_PLAT_INFO_FUNC("pmic_register_val = 0x%x, co_clock_type = %d,TCXO mode\n", retval, co_clock_type);
	} else if ((retval & AP_CONSYS_CO_CLOCK_BITA) || (retval & AP_CONSYS_CO_CLOCK_BITB)) {
		co_clock_type = 1;
		WMT_PLAT_INFO_FUNC("pmic_register_val = 0x%x, co_clock_type = %d,co-TSX mode\n", retval, co_clock_type);
	}
	return co_clock_type;
}

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
	/* Was CONSYS_REG_READ(CONSYS_EMI_MAPPING), which expands to the
	 * hard-coded legacy VA TOPCKGEN_BASE 0xF0000000 (+0x1340), unmapped on
	 * arm64; evaluating it with wmtPlatLogLvl at DBG would panic. Use the
	 * of_iomap'd base like mtk_wcn_consys_hw_init() does. */
	WMT_PLAT_DBG_FUNC("CONSYS_EMI_MAPPING dump before power on/off(0x%08x)\n",
			  CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET));

	if (on) {
		WMT_PLAT_DBG_FUNC("++\n");
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x011);
		pr_emerg("[FORGE_CONN] pwron gate: stage=%d\n", forge_conn_pwron_stage);
		if (forge_conn_pwron_stage < 1) {
			pr_emerg("[FORGE_CONN] stage0 STOP: blocked before PMIC/MTCMOS\n");
			return -EPERM;
		}
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x012);
		/* The consys 26 MHz clock must be on before any rail/MTCMOS
		 * work: without it the domain powers up unclocked and the first
		 * consys AXI read stalls the bus. See forge_conn_dcxo_init(). */
		forge_conn_dcxo_init();
		forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x020);
/*step1.PMIC ctrl*/
#if CONSYS_PMIC_CTRL_ENABLE
		/*need PMIC driver provide new API protocol */
		/*1.AP power on VCN_1V8 LDO (with PMIC_WRAP API) VCN_1V8  */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6355)
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_EN, 0);
		pmic_set_register_value(PMIC_RG_LDO_VCN18_HW0_OP_CFG, 0);
#else
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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
		/* VCN33 (the BT/Wi-Fi RF PA rail) is not enabled here: as in the
		 * stock driver it is owned by mtk_wcn_consys_hw_bt/wifi_paldo_ctrl
		 * at func-on time. */
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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
		/* clk_prepare_enable(NULL) returns 0, so a NULL
		 * clk_scp_conn_main (mtk_wmt_probe never ran) would look like a
		 * powered SCP_SYS_CONN and the chip-id poll would touch an
		 * unpowered consys. Fail instead. */
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

			pmic_read_interface(0x7000, &cw00, 0xFFFF, 0);
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
			/* Stage 3 go/no-go: do the first consys AXI read only
			 * with CONN (bit1) = 1/1 and TOPAXI bus-protect conn bits
			 * 13|14 clear in both EN and STA1; with protection still
			 * up the 0x18070008 read stalls the bus. */
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
			if (consysHwChipId == 0x0326 /* MT6755 consys chip id */) {
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
			/* Stop. Stock only logged here and went on to write ACR
			 * (0x18070110) and release the CONNSYS CPU reset, i.e. more
			 * accesses into a consys that is not answering. */
			forge_conn_trace(FORGE_CONN_TRACE_PWRON, 0x04F);
			pr_emerg("[FORGE_CONN] chip-id FAILED -> ABORT (not writing ACR / not de-asserting CPU reset)\n");
			return -ENODEV;
		}

		if (forge_conn_pwron_stage < 4) {
			pr_emerg("[FORGE_CONN] stage3 STOP: chip-id OK, before ACR/CPU-reset-deassert\n");
			return -EPERM;
		}
		/* Stage 4 releases the CONNSYS CPU; without the deferred EMI
		 * steps (forge_conn_emi = 7: MPU SMC + remap + memset) the consys
		 * MCU would run against an unconfigured EMI window. Refuse. */
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
		/* clk_prepare_enable(NULL) returns 0, so a NULL
		 * clk_scp_conn_main (mtk_wmt_probe never ran) would look like a
		 * powered SCP_SYS_CONN and the chip-id poll would touch an
		 * unpowered consys. Fail instead. */
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
			if (consysHwChipId == 0x0326 /* MT6755 consys chip id */) {
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
		/* Only unwind MTCMOS if it was enabled: after a staged/aborted
		 * power-on the CCF enable count is 0 and a blind disable would
		 * underflow it. */
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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

	/* Propagate an aborted/staged power-on. Stock summed the return codes
	 * and went on into gpio/eirq setup even when the rails/MTCMOS/chip-id
	 * step had failed. */
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
	/* Only unwind gpio/eirq if pwr_on set them up (forge_conn_gpio_on).
	 * reg_ctrl(0) stays unconditional; its clk_disable path checks
	 * forge_conn_mtcmos_on. */
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
/* paldo control as in the stock driver: VCN33 (the BT/Wi-Fi RF PA rail) is
 * owned here at func-on, not in reg_ctrl. The regulator handles are dummies
 * (no *-supply in DT), so the rail is driven with the same MT6351
 * FORGE_CONN_VCN direct writes as the reg_ctrl rails: EN/ON_CTRL pairs at
 * LDO_VCN33_CON3 (0x0A98, BT) / CON4 (0x0A9A, WIFI), EN shift 1, ON_CTRL
 * shift 3, as in stock. ON_CTRL = 1 = HW mode (the rail follows the
 * BT/WIFI srclken). */
INT32 mtk_wcn_consys_hw_bt_paldo_ctrl(UINT32 enable)
{
#if CONSYS_PMIC_CTRL_ENABLE && defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	if (enable) {
		FORGE_CONN_VCN("VCN33_BT EN", MT6351_PMIC_RG_VCN33_EN_BT, 1);
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
#if CONSYS_PMIC_CTRL_ENABLE && defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	if (enable) {
		FORGE_CONN_VCN("VCN33_WIFI EN", MT6351_PMIC_RG_VCN33_EN_WIFI, 1);
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
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

	/* Only re-apply EMI state that the staged bring-up established
	 * (CONFIG_MTK_HIBERNATION is off, so this is not registered). */
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
		/* MT6755 remap granularity is 1 MB: bits[11:0] = phys[31:20],
		 * as in the stock MT6755 driver. The MT6757 formula (>> 21,
		 * 2 MB) maps the consys window onto foreign RAM. */
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

/* Deferred EMI setup, see the forge_conn_emi comment above. The caller
 * holds forge_conn_emi_lock (sysfs setter) or is single-threaded (hw
 * restore). Steps run in ascending bit order, the stock hw_init order
 * (MPU SMC -> remap -> memset). */
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
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F2);	/* H-A pre */
		emi_mpu_set_region_protection(gConEmiPhyBase + SZ_1M / 2,
					      gConEmiPhyBase + gConEmiSize - 1,
					      13,
					      SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					      FORBIDDEN, NO_PROTECTION, FORBIDDEN, NO_PROTECTION));
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F3);	/* H-A post */
		pr_err("[FORGE_CONN] emi step1 (H-A) DONE\n");
#else
		pr_err("[FORGE_CONN] emi step1 (H-A) NO-OP: CONSYS_EMI_MPU_SETTING disabled\n");
#endif
		forge_conn_emi_done |= 0x1u;
	}

	if (todo & 0x2u) {
		/*consys to ap emi remapping register, cal remapping address */
		/* MT6755 remap granularity is 1 MB: bits[11:0] = phys[31:20],
		 * as in the stock MT6755 driver, not the MT6757 >> 21 / 2 MB
		 * formula, which maps the consys window onto foreign RAM.
		 * Replace bits[12:0] instead of OR-ing, so a re-run can fix a
		 * previously wrong value. */
		addrPhy = (((UINT32)(gConEmiPhyBase & 0xFFF00000)) >> 20);
		/*enable consys to ap emi remapping bit12 */
		addrPhy |= 0x1000;
		pr_err("[FORGE_CONN] emi step2 (H-B) ENTER: remap write topckgen+0x%x [12:0]= 0x%04x\n",
		       CONSYS_EMI_MAPPING_OFFSET, addrPhy);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F4);	/* H-B pre */
		CONSYS_REG_WRITE(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET,
				 (CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET) &
				  ~0x1FFFu) | addrPhy);
		remap = CONSYS_REG_READ(conn_reg.topckgen_base + CONSYS_EMI_MAPPING_OFFSET);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F5);	/* H-B post */
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
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F6);	/* H-D pre */
		memset_io(pEmibaseaddr, 0, CONSYS_EMI_MEM_SIZE);
		forge_conn_trace(FORGE_CONN_TRACE_DETECT, 0x0F7);	/* H-D post */
		pr_err("[FORGE_CONN] emi step3 (H-D) DONE\n");
		forge_conn_emi_done |= 0x4u;
	}

	return 0;
}

INT32 mtk_wcn_consys_hw_init(void)
{

	INT32 iRet = -1;
	struct device_node *node = NULL;

	/* Runs at DO_MODULE_INIT time (wmt_loader), not at boot. The EMI steps
	 * that touch hardware are deferred, see forge_conn_emi_apply(); every
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

	if (gConEmiPhyBase) {
		/* No hardware access here: the stock EMI steps (H-A EMI-MPU SMC,
		 * H-B remap write, H-D memset_io) are deferred behind the
		 * forge_conn_emi knob (forge_conn_emi_apply()). This is reachable
		 * on every boot through the connectivity autostart as soon as
		 * anything issues DO_MODULE_INIT. */
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
