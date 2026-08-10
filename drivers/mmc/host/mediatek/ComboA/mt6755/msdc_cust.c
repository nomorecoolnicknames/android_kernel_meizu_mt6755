/* Copyright (C) 2015 MediaTek Inc.
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

/*
 * MT6755 board layer for the ComboA MSDC core; see msdc_cust.h for where
 * each value comes from. Serves msdc0 (eMMC) only.
 */

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include "mtk_sd.h"
#include "dbg.h"
struct msdc_host *mtk_msdc_host[] = {NULL, NULL};
EXPORT_SYMBOL(mtk_msdc_host);

int g_dma_debug[HOST_MAX_NUM] = {0, 0};
u32 latest_int_status[HOST_MAX_NUM] = {0, 0};

unsigned int msdc_latest_transfer_mode[HOST_MAX_NUM] = {
	/* 0 for PIO; 1 for DMA; 3 for nothing */
	MODE_NONE,
	MODE_NONE
};

unsigned int msdc_latest_op[HOST_MAX_NUM] = {
	/* 0 for read; 1 for write; 2 for nothing */
	OPER_TYPE_NUM,
	OPER_TYPE_NUM
};

/* for debug zone */
unsigned int sd_debug_zone[HOST_MAX_NUM] = {
	0,
	0
};
/* for enable/disable register dump */
unsigned int sd_register_zone[HOST_MAX_NUM] = {
	1,
	1
};
/* mode select */
u32 dma_size[HOST_MAX_NUM] = {
	512,
	512
};

u32 drv_mode[HOST_MAX_NUM] = {
	MODE_SIZE_DEP, /* using DMA or not depend on the size */
	MODE_SIZE_DEP
};

int dma_force[HOST_MAX_NUM]; /* used for sd ioctrol */

u8 msdc_clock_src[HOST_MAX_NUM] = {
	0,
	0
};

/**************************************************************/
/* Section 1: Device Tree Global Variables                    */
/**************************************************************/
const struct of_device_id msdc_of_ids[] = {
	{   .compatible = DT_COMPATIBLE_NAME, },
	{ },
};

#if !defined(FPGA_PLATFORM)
static void __iomem *gpio_base;

static void __iomem *topckgen_base;
static void __iomem *apmixed_base;
#endif

void __iomem *msdc_io_cfg_bases[HOST_MAX_NUM];

/**************************************************************/
/* Section 2: Power                                           */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
/*
 * The MT6351 PMIC sits behind a read-only pwrap here (mt_pmic_stub.c), so
 * regulator_get("vmmc") returns an ERR_PTR (probe deferral of the
 * never-registered VEMC LDO), not NULL, and dereferencing it in
 * regulator_set_voltage() oopses; guard with IS_ERR_OR_NULL. VEMC is left
 * on by the preloader/LK and is neither programmed nor switched off here.
 */
int msdc_regulator_set_and_enable(struct regulator *reg, int powerVolt)
{
#ifndef CONFIG_MTK_MSDC_BRING_UP_BYPASS
	if (IS_ERR_OR_NULL(reg))
		return 0;
	regulator_set_voltage(reg, powerVolt, powerVolt);
	return regulator_enable(reg);
#else
	return 0;
#endif
}

void msdc_ldo_power(u32 on, struct regulator *reg, int voltage_mv, u32 *status)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	int voltage_uv = voltage_mv * 1000;

	if (IS_ERR_OR_NULL(reg))
		return;

	if (on) { /* want to power on */
		if (*status == 0) {  /* can power on */
			(void)msdc_regulator_set_and_enable(reg, voltage_uv);
			*status = voltage_uv;
		} else if (*status == voltage_uv) {
			pr_info("msdc power on <%d> again!\n", voltage_uv);
		} else {
			pr_info("msdc change<%d> to <%d>\n",
				*status, voltage_uv);
			regulator_disable(reg);
			(void)msdc_regulator_set_and_enable(reg, voltage_uv);
			*status = voltage_uv;
		}
	} else {  /* want to power off */
		if (*status != 0) {  /* has been powerred on */
			pr_info("msdc power off\n");
			(void)regulator_disable(reg);
			*status = 0;
		} else {
			pr_info("msdc not power on\n");
		}
	}
#endif
}

void msdc_dump_ldo_sts(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
	/* MT6351 is not reachable (pwrap stubbed), nothing to read */
	SPREAD_PRINTF(buff, size, m,
		" msdc%d LDO status n/a: PMIC wrapper stubbed, VEMC left on by LK\n",
		host->id);
}

void msdc_sd_power_switch(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	if (host->id == 1) {
		msdc_ldo_power(on, host->mmc->supply.vqmmc, VOL_1800,
			&host->power_io);
		msdc_set_tdsel(host, MSDC_TDRDSEL_1V8, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_1V8, 0);
		host->hw->driving_applied = &host->hw->driving_sdr50;
		msdc_set_driving(host, host->hw->driving_applied);
	}
#endif
}

void msdc_power_calibration_init(struct msdc_host *host)
{
	/*
	 * The stock kernels trim VEMC (-100 mV) and VMC/VMCH through PMIC
	 * calibration registers here; with the PMIC stubbed that write cannot
	 * happen.
	 */
}

int msdc_oc_check(struct msdc_host *host, u32 en)
{
	return 0;
}

void msdc_emmc_power(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	if (on) {
		msdc_set_driving(host, &host->hw->driving);
		msdc_set_tdsel(host, MSDC_TDRDSEL_1V8, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_1V8, 0);
	}
	msdc_ldo_power(on, host->mmc->supply.vmmc,
		VOL_3000, &host->power_flash);
#endif
#ifdef MTK_MSDC_BRINGUP_DEBUG
	msdc_dump_ldo_sts(NULL, 0, NULL, host);
#endif
}

void msdc_sd_power(struct msdc_host *host, u32 on)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	u32 card_on = on;

	switch (host->id) {
	case 1:
		msdc_set_driving(host, &host->hw->driving);
		msdc_set_tdsel(host, MSDC_TDRDSEL_3V, 0);
		msdc_set_rdsel(host, MSDC_TDRDSEL_3V, 0);
		if (host->hw->flags & MSDC_SD_NEED_POWER)
			card_on = 1;

		/* VMCH VOLSEL */
		msdc_ldo_power(card_on, host->mmc->supply.vmmc, VOL_3000,
			&host->power_flash);
		/* VMC VOLSEL */
		msdc_ldo_power(on, host->mmc->supply.vqmmc, VOL_3000,
			&host->power_io);
		break;

	default:
		break;
	}
#endif
#ifdef MTK_MSDC_BRINGUP_DEBUG
	msdc_dump_ldo_sts(NULL, 0, NULL, host);
#endif
}

void msdc_sd_power_off(void)
{
#if !defined(CONFIG_MTK_MSDC_BRING_UP_BYPASS)
	struct msdc_host *host = mtk_msdc_host[1];

	if (host) {
		pr_notice("Power Off, SD card\n");

		/* power must be on */
		host->power_io = VOL_3000 * 1000;
		host->power_flash = VOL_3000 * 1000;

		host->power_control(host, 0);

		msdc_set_bad_card_and_remove(host);
	}
#endif
}
EXPORT_SYMBOL(msdc_sd_power_off);
#endif /*if !defined(FPGA_PLATFORM)*/

void msdc_pmic_force_vcore_pwm(bool enable)
{
	/* stock calls pmic_force_vcore_pwm(); a no-op with the PMIC stubbed */
}

void msdc_set_host_power_control(struct msdc_host *host)
{
	if (host->hw->host_function == MSDC_EMMC) {
		host->power_control = msdc_emmc_power;
	} else if (host->hw->host_function == MSDC_SD) {
		host->power_control = msdc_sd_power;
		host->power_switch = msdc_sd_power_switch;

		#if SD_POWER_DEFAULT_ON
		/* If SD card power is default on, turn it off so that
		 * removable card slot won't keep power when no card plugged
		 */
		if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
			/* turn on first to match HW/SW state*/
			msdc_sd_power(host, 1);
			mdelay(10);
			msdc_sd_power(host, 0);
		}
		#endif
	}

	if (host->power_control != NULL) {
		msdc_power_calibration_init(host);
	} else {
		ERR_MSG("Host function defination error for msdc%d", host->id);
		WARN_ON(1);
	}
}

#if defined(MSDC_HQA)
void msdc_HQA_set_voltage(struct msdc_host *host)
{
}
#endif

/**************************************************************/
/* Section 3: Clock                                           */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
u32 hclks_msdc0[] = {
	MSDC0_SRC_0,
	MSDC0_SRC_1,
	MSDC0_SRC_2,
	MSDC0_SRC_3,
	MSDC0_SRC_4,
	MSDC0_SRC_5,
	MSDC0_SRC_6,
	MSDC0_SRC_7,
	MSDC0_SRC_8
};

/* msdc1/2 clock source reference value is 200M */
u32 hclks_msdc1[] = {
	MSDC1_SRC_0,
	MSDC1_SRC_1,
	MSDC1_SRC_2,
	MSDC1_SRC_3,
	MSDC1_SRC_4,
	MSDC1_SRC_5,
	MSDC1_SRC_6,
	MSDC1_SRC_7
};

u32 *hclks_msdc_all[] = {
	hclks_msdc0,
	hclks_msdc1
};
u32 *hclks_msdc;

/*
 * The DT "MSDC0-CLOCK" (infrasys INFRA_MSDC0 gate, parent msdc50_0_sel) is
 * got and prepared once; msdc_clk_enable()/msdc_clk_disable() only
 * enable/disable it. The msdc50_0_sel mux and MSDCPLL rate are left as the
 * preloader/LK programmed them (clk_src in DT only selects the hclk value
 * the divider math uses).
 */
int msdc_get_ccf_clk_pointer(struct platform_device *pdev,
	struct msdc_host *host)
{
	static char const * const clk_names[] = {
		MSDC0_CLK_NAME, MSDC1_CLK_NAME
	};

	host->hclk_ctl = NULL;
	host->aes_clk_ctl = NULL;

	host->clk_ctl = devm_clk_get(&pdev->dev, clk_names[pdev->id]);
	if (IS_ERR(host->clk_ctl)) {
		pr_notice("[msdc%d] can not get clock control\n", pdev->id);
		host->clk_ctl = NULL;
		return 1;
	}
	if (clk_prepare(host->clk_ctl)) {
		pr_notice("[msdc%d] can not prepare clock control\n",
			pdev->id);
		host->clk_ctl = NULL;
		return 1;
	}

	/* first-boot evidence: CCF view of the source vs the table value */
	pr_notice("[msdc%d] %s prepared, CCF rate %lu Hz, clk_src %u -> hclk %u Hz\n",
		pdev->id, clk_names[pdev->id], clk_get_rate(host->clk_ctl),
		host->hw->clk_src, host->hclk);

	return 0;
}

void msdc_select_clksrc(struct msdc_host *host, int clksrc)
{
	host->hclk = msdc_get_hclk(host->id, clksrc);
	host->hw->clk_src = clksrc;

	pr_notice("[%s]: msdc%d select clk_src as %d(%dKHz)\n", __func__,
		host->id, clksrc, host->hclk/1000);

	pr_notice("[%s]: msdc%d not support change clksrc\n", __func__,
		host->id);
}

#include <linux/seq_file.h>

void msdc_dump_clock_sts(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
	if (topckgen_base)
		SPREAD_PRINTF(buff, size, m,
			" CLK_CFG_3[0x%p]=0x%x should: bit[19:16]=msdc50_0 src, bit[23]=0\n",
			topckgen_base + MSDC_CLK_CFG_3_OFFSET,
			MSDC_READ32(topckgen_base + MSDC_CLK_CFG_3_OFFSET));
	if (apmixed_base)
		SPREAD_PRINTF(buff, size, m,
			" MSDCPLL_CON0=0x%x PWR_CON0=0x%x should: bit[0]=1\n",
			MSDC_READ32(apmixed_base + MSDCPLL_CON0_OFFSET),
			MSDC_READ32(apmixed_base + MSDCPLL_PWR_CON0_OFFSET));
}

void msdc_clk_enable_and_stable(struct msdc_host *host)
{
	void __iomem *base = host->base;
	u32 div, mode;
	u32 val;

	msdc_clk_enable(host);

	val = MSDC_READ32(MSDC_CFG);
	GET_FIELD(val, CFG_CKDIV_SHIFT, CFG_CKDIV_MASK, div);
	GET_FIELD(val, CFG_CKMOD_SHIFT, CFG_CKMOD_MASK, mode);
	msdc_clk_stable(host, mode, div, 0);
}
#endif /*if !defined(FPGA_PLATFORM)*/

/**************************************************************/
/* Section 4: GPIO and Pad (msdc0 only)                       */
/**************************************************************/
#if !defined(FPGA_PLATFORM)
void msdc_dump_vcore(char **buff, unsigned long *size, struct seq_file *m)
{
}

/*****************************************************************************/
/* obtain dump api interface */
/*****************************************************************************/
void msdc_dump_dvfs_reg(char **buff, unsigned long *size,
	struct seq_file *m, struct msdc_host *host)
{
}

int msdc_io_check(struct msdc_host *host)
{
	return 1;
}

void msdc_dump_padctl_by_id(char **buff, unsigned long *size,
	struct seq_file *m, u32 id)
{
	if (id != 0)
		return;

	if (!gpio_base || !MSDC0_IO_PAD_BASE) {
		SPREAD_PRINTF(buff, size, m,
			"err: gpio_base=%p, msdc_io_cfg_bases[0]=%p\n",
			gpio_base, MSDC0_IO_PAD_BASE);
		return;
	}

	SPREAD_PRINTF(buff, size, m, "MSDC0 MODE18=0x%x should:0x12-- ----\n",
		MSDC_READ32(MSDC0_GPIO_MODE18));
	SPREAD_PRINTF(buff, size, m, "MSDC0 MODE19=0x%x should:0x12491249\n",
		MSDC_READ32(MSDC0_GPIO_MODE19));
	SPREAD_PRINTF(buff, size, m, "MSDC0 IES  =0x%x\n",
		MSDC_READ32(MSDC0_GPIO_IES_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 SMT  =0x%x should:0x???????f\n",
		MSDC_READ32(MSDC0_GPIO_SMT_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 TDSEL=0x%x should:0x????0000\n",
		MSDC_READ32(MSDC0_GPIO_TDSEL_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 RDSEL=0x%x should:0x???00000\n",
		MSDC_READ32(MSDC0_GPIO_RDSEL_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 DRV  =0x%x\n",
		MSDC_READ32(MSDC0_GPIO_DRV_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 PUPD0=0x%x should:0x?1111161\n",
		MSDC_READ32(MSDC0_GPIO_PUPD0_ADDR));
	SPREAD_PRINTF(buff, size, m, "MSDC0 PUPD1=0x%x should:0x????6111\n",
		MSDC_READ32(MSDC0_GPIO_PUPD1_ADDR));
}

void msdc_set_pin_mode(struct msdc_host *host)
{
	if (host->id != 0)
		return;

	/* eMMC pinmux: MODE18 DAT0/CLK, MODE19 CMD/DAT1-7/DSL/RST, mode 1 */
	MSDC_SET_FIELD(MSDC0_GPIO_MODE18, MSDC0_MODE18_MASK, MSDC0_MODE18_VAL);
	MSDC_SET_FIELD(MSDC0_GPIO_MODE19, 0xFFFFFFFF, MSDC0_MODE19_VAL);
}

void msdc_set_ies_by_id(u32 id, int set_ies)
{
	if (id == 0)
		MSDC_SET_FIELD(MSDC0_GPIO_IES_ADDR, MSDC0_IES_ALL_MASK,
			(set_ies ? 0x1F : 0));
}

void msdc_set_smt_by_id(u32 id, int set_smt)
{
	if (id == 0)
		MSDC_SET_FIELD(MSDC0_GPIO_SMT_ADDR, MSDC0_SMT_ALL_MASK,
			(set_smt ? 0x1F : 0));
}

void msdc_set_tdsel_by_id(u32 id, u32 flag, u32 value)
{
	if (id == 0)
		MSDC_SET_FIELD(MSDC0_GPIO_TDSEL_ADDR, MSDC0_TDSEL_ALL_MASK,
			(flag == MSDC_TDRDSEL_CUST) ? value : 0);
}

void msdc_set_rdsel_by_id(u32 id, u32 flag, u32 value)
{
	if (id == 0)
		MSDC_SET_FIELD(MSDC0_GPIO_RDSEL_ADDR, MSDC0_RDSEL_ALL_MASK,
			(flag == MSDC_TDRDSEL_CUST) ? value : 0);
}

void msdc_get_tdsel_by_id(u32 id, u32 *value)
{
	if (id == 0)
		MSDC_GET_FIELD(MSDC0_GPIO_TDSEL_ADDR, MSDC0_TDSEL_ALL_MASK,
			*value);
}

void msdc_get_rdsel_by_id(u32 id, u32 *value)
{
	if (id == 0)
		MSDC_GET_FIELD(MSDC0_GPIO_RDSEL_ADDR, MSDC0_RDSEL_ALL_MASK,
			*value);
}

void msdc_set_sr_by_id(u32 id, int clk, int cmd, int dat, int rst, int ds)
{
	/* msdc0: no SR to set on mt6755 */
}

void msdc_set_driving_by_id(u32 id, struct msdc_hw_driving *driving)
{
	if (id != 0)
		return;

	MSDC_SET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_DSL_MASK,
		driving->ds_drv);
	MSDC_SET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_RSTB_MASK,
		driving->rst_drv);
	MSDC_SET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_CMD_MASK,
		driving->cmd_drv);
	MSDC_SET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_CLK_MASK,
		driving->clk_drv);
	MSDC_SET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_DAT_MASK,
		driving->dat_drv);
}

void msdc_get_driving_by_id(u32 id, struct msdc_hw_driving *driving)
{
	if (id != 0)
		return;

	MSDC_GET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_DSL_MASK,
		driving->ds_drv);
	MSDC_GET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_RSTB_MASK,
		driving->rst_drv);
	MSDC_GET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_CMD_MASK,
		driving->cmd_drv);
	MSDC_GET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_CLK_MASK,
		driving->clk_drv);
	MSDC_GET_FIELD(MSDC0_GPIO_DRV_ADDR, MSDC0_DRV_DAT_MASK,
		driving->dat_drv);
}

/* msdc0 PUPD/R1/R0 per pin:
 * 0/0/0 High-Z, 0/1/0 PU 50K, 0/0/1 PU 10K, 0/1/1 PU 50K//10K,
 * 1/0/0 High-Z, 1/1/0 PD 50K, 1/0/1 PD 10K, 1/1/1 PD 50K//10K
 */
void msdc_pin_config_by_id(u32 id, u32 mode)
{
	if (id != 0)
		return;

	/* Don't pull CLK high; don't toggle RST (would enter boot mode) */
	if (mode == MSDC_PIN_PULL_NONE) {
		/* high-Z */
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_ADDR, MSDC0_PUPD0_MASK,
			0x4444444);
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_ADDR, MSDC0_PUPD1_MASK,
			0x4444);
	} else if (mode == MSDC_PIN_PULL_DOWN) {
		/* cmd/clk/dat/(rstb)/dsl: pd-50k */
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_ADDR, MSDC0_PUPD0_MASK,
			0x6666666);
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_ADDR, MSDC0_PUPD1_MASK,
			0x6666);
	} else if (mode == MSDC_PIN_PULL_UP) {
		/* clk/dsl: pd-50k, cmd/dat: pu-10k, (rstb: pu-50k) */
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD0_ADDR, MSDC0_PUPD0_MASK,
			0x1111161);
		MSDC_SET_FIELD(MSDC0_GPIO_PUPD1_ADDR, MSDC0_PUPD1_MASK,
			0x6111);
	}
}
#endif /*if !defined(FPGA_PLATFORM)*/


/**************************************************************/
/* Section 5: Device Tree Init function                       */
/*            This function is placed here so that all	      */
/*            functions and variables used by it has already  */
/*            been declared                                   */
/**************************************************************/
/*
 * parse pinctl settings
 * Driver strength
 */
#if !defined(FPGA_PLATFORM)
static int msdc_get_pinctl_settings(struct msdc_host *host,
	struct device_node *np)
{
	struct device_node *pinctl_node, *pins_node;
	static char const * const pinctl_names[] = {
		"pinctl", "pinctl_hs400", "pinctl_hs200",
		"pinctl_sdr104", "pinctl_sdr50", "pinctl_ddr50"
	};

	/* sequence shall be the same as sequence in msdc_hw_driving */
	static char const * const pins_names[] = {
		"pins_cmd", "pins_dat", "pins_clk", "pins_rst", "pins_ds"
	};
	struct msdc_hw_driving *drv;
	unsigned char *pin_drv;
	int i, j;

	host->hw->driving_applied = &host->hw->driving;
	for (i = 0; i < ARRAY_SIZE(pinctl_names); i++) {
		if (strcmp(pinctl_names[i], "pinctl") == 0)
			drv = &host->hw->driving;
		else if (strcmp(pinctl_names[i], "pinctl_hs400") == 0)
			drv = &host->hw->driving_hs400;
		else if (strcmp(pinctl_names[i], "pinctl_hs200") == 0)
			drv = &host->hw->driving_hs200;
		else if (strcmp(pinctl_names[i], "pinctl_sdr104") == 0)
			drv = &host->hw->driving_sdr104;
		else if (strcmp(pinctl_names[i], "pinctl_sdr50") == 0)
			drv = &host->hw->driving_sdr50;
		else if (strcmp(pinctl_names[i], "pinctl_ddr50") == 0)
			drv = &host->hw->driving_ddr50;
		else
			continue;

		pinctl_node = of_parse_phandle(np, pinctl_names[i], 0);
		/*
		 * The mt6755 DT only has "pinctl". The ComboA core applies
		 * driving_hs200/_hs400 when eMMC changes timing (sd.c
		 * msdc_ops_set_ios); left at 0 that would be the weakest drive.
		 * Inherit "pinctl", which the 3.18 driver uses in every mode.
		 */
		if (!pinctl_node) {
			if (drv != &host->hw->driving)
				*drv = host->hw->driving;
			continue;
		}

		pin_drv = (unsigned char *)drv;
		for (j = 0; j < ARRAY_SIZE(pins_names); j++) {
			pins_node = of_get_child_by_name(pinctl_node,
				pins_names[j]);

			if (pins_node) {
				of_property_read_u8(pins_node,
					"drive-strength", pin_drv);
				of_node_put(pins_node);
			}
			pin_drv++;
		}
		of_node_put(pinctl_node);
	}

	return 0;
}
#endif

/* Get msdc register settings
 * 1. internal data delay for tuning, FIXME: can be removed when use data tune?
 * 2. sample edge
 */
static int msdc_get_register_settings(struct msdc_host *host,
	struct device_node *np)
{
	struct device_node *register_setting_node = NULL;

	/* parse hw property settings */
	register_setting_node = of_parse_phandle(np, "register_setting", 0);
	if (register_setting_node) {
		of_property_read_u8(register_setting_node, "cmd_edge",
				&host->hw->cmd_edge);
		of_property_read_u8(register_setting_node, "rdata_edge",
				&host->hw->rdata_edge);
		of_property_read_u8(register_setting_node, "wdata_edge",
				&host->hw->wdata_edge);
	} else {
		pr_notice("[msdc%d] register_setting is not found in DT\n",
			host->id);
	}

	return 0;
}

/*
 *	msdc_of_parse() - parse host's device-tree node
 *	@host: host whose node should be parsed.
 *
 */

int msdc_of_parse(struct platform_device *pdev, struct mmc_host *mmc)
{
	struct device_node *np;
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;
	u8 id;

	np = mmc->parent->of_node; /* mmcx node in project dts */

	/* the mt6755 DT has no "index"; name -> id as the mt6755 msdc_dt_init() */
	if (of_property_read_u8(np, "index", &id)) {
		if (!strcmp(np->name, "msdc0"))
			id = 0;
		else if (!strcmp(np->name, "msdc1"))
			id = 1;
		else if (!strcmp(np->name, "msdc2"))
			id = 2;
		else {
			pr_notice("[%s] host index not specified in device tree\n",
				pdev->dev.of_node->name);
			return -1;
		}
	}

	/*
	 * Only eMMC (msdc0) is served. msdc1 (microSD) needs the MT6351 VMCH/VMC
	 * LDOs, unreachable while pwrap is read-only; msdc2 (SDIO WiFi) is also
	 * dropped by the probe denylist. Refuse them here, before any register
	 * access.
	 */
	if (id != 0) {
		pr_notice("[msdc%d] %s: not served on m681 4.9 (eMMC only)\n",
			id, np->name);
		return -ENODEV;
	}
	host->id = id;
	pdev->id = id;

	pr_notice("DT probe %s%d!\n", pdev->dev.of_node->name, id);

	ret = mmc_of_parse(mmc);
	if (ret) {
		pr_notice("%s: mmc of parse error!!: %d\n", __func__, ret);
		return ret;
	}

	/*
	 * No HS400 and no HS200: the eMMC runs HS (52 MHz SDR, fixed sampling, no
	 * tuning). In HS200 the card gives repeated "response CRC error sending
	 * r/w cmd", "[AUTOK] ... LATCH_CK ... fail" and "msdc0 tune error", and
	 * no partition table is read. The probe reports "CCF rate 283500000 Hz"
	 * for MSDC0-CLOCK while the divider math assumes the table's 400 MHz
	 * (clk_src 1). hclk stays at the table value on purpose: if the source
	 * really is lower the card clock only ends up below the requested 52 MHz,
	 * never above. HS200 needs the mt6755 tuning/pad set and a checked MSDC0
	 * source rate.
	 */
	mmc->caps2 &= ~(MMC_CAP2_HS400_1_8V | MMC_CAP2_HS400_1_2V |
			MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_HS200_1_2V_SDR);

	host->mmc = mmc;
	host->hw = kzalloc(sizeof(struct msdc_hw), GFP_KERNEL);
	if (!host->hw)
		return -ENOMEM;

	/* iomap register */
	host->base = of_iomap(np, 0);
	if (!host->base) {
		pr_notice("[msdc%d] of_iomap failed\n", mmc->index);
		return -ENOMEM;
	}
	/* get irq # */
	host->irq = irq_of_parse_and_map(np, 0);
	pr_notice("[msdc%d] get irq # %d\n", host->id, host->irq);
	WARN_ON(host->irq < 0);

#if !defined(FPGA_PLATFORM)
	/* get clk_src */
	if (of_property_read_u8(np, "clk_src", &host->hw->clk_src)) {
		pr_notice("[msdc%d] error: clk_src isn't found in device tree.\n",
			host->id);
		WARN_ON(1);
	}
#endif
	/* Returns 0 on success, -EINVAL if the property does not exist,
	 * -ENODATA if property does not have a value, and -EOVERFLOW if the
	 * property data isn't large enough.
	 */
	if (of_property_read_u8(np, "host_function", &host->hw->host_function))
		pr_notice("[msdc%d] host_function isn't found in device tree\n",
			host->id);

	msdc_get_register_settings(host, np);

#if !defined(FPGA_PLATFORM)
	msdc_get_pinctl_settings(host, np);
	mmc->supply.vmmc = regulator_get(mmc_dev(mmc), "vmmc");
	mmc->supply.vqmmc = regulator_get(mmc_dev(mmc), "vqmmc");
#else
	msdc_fpga_pwr_init();
#endif

	/* mt6755 has no MSDC TOP block: host->base_top stays NULL */

	/* No device rename to "bootdevice"; the ramdisk fstab mounts
	 * /dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/...
	 */

	return host->id;
}

int msdc_dt_init(struct platform_device *pdev, struct mmc_host *mmc)
{
	int id;

#ifndef FPGA_PLATFORM
	struct device_node *np;
#endif

	id = msdc_of_parse(pdev, mmc);
	if (id < 0) {
		pr_notice("%s: msdc_of_parse error!!: %d\n", __func__, id);
		return id;
	}

#ifndef FPGA_PLATFORM
	if (gpio_base == NULL) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,gpio");
		gpio_base = of_iomap(np, 0);
		pr_debug("of_iomap for gpio base @ 0x%p\n", gpio_base);
	}

	if (msdc_io_cfg_bases[id] == NULL) {
		np = of_find_compatible_node(NULL, NULL, MSDC0_IOCFG_NAME);
		msdc_io_cfg_bases[id] = of_iomap(np, 0);
		pr_debug("of_iomap for MSDC%d IOCFG base @ 0x%p\n",
			id, msdc_io_cfg_bases[id]);
	}

	/* pad writes go through these two; fail the probe, not an Oops */
	if (!gpio_base || !msdc_io_cfg_bases[id]) {
		pr_notice("[msdc%d] gpio/iocfg base missing (%p/%p)\n",
			id, gpio_base, msdc_io_cfg_bases[id]);
		return -ENODEV;
	}

	/* read-only, for msdc_dump_clock_sts() */
	if (topckgen_base == NULL) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,topckgen");
		topckgen_base = of_iomap(np, 0);
	}

	if (apmixed_base == NULL) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,apmixed");
		apmixed_base = of_iomap(np, 0);
	}
#endif
	return 0;
}

/**************************************************************/
/* Section 7: For msdc register dump                          */
/**************************************************************/
u16 msdc_offsets[] = {
	OFFSET_MSDC_CFG,
	OFFSET_MSDC_IOCON,
	OFFSET_MSDC_PS,
	OFFSET_MSDC_INT,
	OFFSET_MSDC_INTEN,
	OFFSET_MSDC_FIFOCS,
	OFFSET_SDC_CFG,
	OFFSET_SDC_CMD,
	OFFSET_SDC_ARG,
	OFFSET_SDC_STS,
	OFFSET_SDC_RESP0,
	OFFSET_SDC_RESP1,
	OFFSET_SDC_RESP2,
	OFFSET_SDC_RESP3,
	OFFSET_SDC_BLK_NUM,
	OFFSET_SDC_VOL_CHG,
	OFFSET_SDC_CSTS,
	OFFSET_SDC_CSTS_EN,
	OFFSET_SDC_DCRC_STS,
	OFFSET_SDC_ADV_CFG0,
	OFFSET_EMMC_CFG0,
	OFFSET_EMMC_CFG1,
	OFFSET_EMMC_STS,
	OFFSET_EMMC_IOCON,
	OFFSET_SDC_ACMD_RESP,
	OFFSET_MSDC_DMA_SA_HIGH,
	OFFSET_MSDC_DMA_SA,
	OFFSET_MSDC_DMA_CA,
	OFFSET_MSDC_DMA_CTRL,
	OFFSET_MSDC_DMA_CFG,
	OFFSET_MSDC_DMA_LEN,
	OFFSET_MSDC_DBG_SEL,
	OFFSET_MSDC_DBG_OUT,
	OFFSET_MSDC_PATCH_BIT0,
	OFFSET_MSDC_PATCH_BIT1,
	OFFSET_MSDC_PATCH_BIT2,
	OFFSET_MSDC_PAD_TUNE0,
	OFFSET_MSDC_PAD_TUNE1,
	OFFSET_MSDC_HW_DBG,
	OFFSET_MSDC_VERSION,

	OFFSET_EMMC50_PAD_DS_TUNE,
	OFFSET_EMMC50_PAD_CMD_TUNE,
	OFFSET_EMMC50_PAD_DAT01_TUNE,
	OFFSET_EMMC50_PAD_DAT23_TUNE,
	OFFSET_EMMC50_PAD_DAT45_TUNE,
	OFFSET_EMMC50_PAD_DAT67_TUNE,
	OFFSET_EMMC51_CFG0,
	OFFSET_EMMC50_CFG0,
	OFFSET_EMMC50_CFG1,
	OFFSET_EMMC50_CFG2,
	OFFSET_EMMC50_CFG3,
	OFFSET_EMMC50_CFG4,
	OFFSET_SDC_FIFO_CFG,

	0xFFFF /*as mark of end */
};

u16 msdc_offsets_top[] = {
	0xFFFF /*as mark of end */
};
