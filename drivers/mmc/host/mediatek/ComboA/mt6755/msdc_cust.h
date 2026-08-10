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

/*
 * MT6755 board layer for the ComboA MSDC core.
 *
 * Template: ComboA/mt6735. Data from:
 *  - the mt6755 msdc layer (drivers/mmc/host/mediatek/mt6755/mt6755/
 *    msdc_cust.[ch]): DT names, CCF clock, clock source table, eMMC pad
 *    map (MODE18/19, iocfg_5, 3-bit DRV, PUPD values), the IS_ERR_OR_NULL
 *    regulator guard;
 *  - the native 3.18 driver drivers/mmc/host/mediatek/mt6755: the same
 *    clock table and pad map, plus the PATCH_BIT/tune defaults (Section 6)
 *    and the autok constants (autok_cust.h).
 * Every register/bit-field that the mt6755 msdc_reg.h and ComboA's
 * msdc_reg.h both define has the same value.
 */

#ifndef _MSDC_CUST_MT6755_H_
#define _MSDC_CUST_MT6755_H_

#ifdef CONFIG_FPGA_EARLY_PORTING
#define FPGA_PLATFORM
#else
/*#define MTK_MSDC_BRINGUP_DEBUG*/
#endif

#include <dt-bindings/mmc/mt6755-msdc.h>

#ifndef CONFIG_MTK_MSDC_BRING_UP_BYPASS
#define spm_resource_req(a, b)
#define SPM_RESOURCE_USER_MSDC
#define SPM_RESOURCE_RELEASE
#define SPM_RESOURCE_ALL
#endif

/**************************************************************/
/* Section 1: Device Tree                                     */
/**************************************************************/
/* Names used for device tree lookup (mt6755.dtsi mtk-msdc.0 nodes) */
#define DT_COMPATIBLE_NAME      "mediatek,mt6755-mmc"
#define MSDC0_CLK_NAME          "MSDC0-CLOCK"
#define MSDC1_CLK_NAME          "MSDC1-CLOCK"
#define MSDC0_IOCFG_NAME        "mediatek,iocfg_5"
#define MSDC1_IOCFG_NAME        "mediatek,iocfg_0"

/**************************************************************/
/* Section 2: Power                                           */
/**************************************************************/
#define SD_POWER_DEFAULT_ON     (0)

/* VOL_* only; this layer makes no PMIC access (pwrap is read-only) */
#include <mt-plat/upmu_common.h>

/**************************************************************/
/* Section 3: Clock                                           */
/**************************************************************/
/* topckgen / apmixedsys offsets, used only by the clock-status dump */
#define MSDC_CLK_CFG_3_OFFSET   (0x070)
#define MSDCPLL_CON0_OFFSET     (0x250)
#define MSDCPLL_PWR_CON0_OFFSET (0x25c)

/*
 * indexed by DT clk_src (dt-bindings/mmc/mt6755-msdc.h MSDC50_CLKSRC_*) =
 * msdc50_0_sel parent index (clk-mt6755.c msdc50_0_parents). Index 0 is
 * clk26m: 26 MHz (the vendor tables carry an MTK typo, 260000).
 */
#define MSDC0_SRC_0             26000000
#define MSDC0_SRC_1             400000000
#define MSDC0_SRC_2             200000000
#define MSDC0_SRC_3             156000000
#define MSDC0_SRC_4             182000000
#define MSDC0_SRC_5             156000000
#define MSDC0_SRC_6             100000000
#define MSDC0_SRC_7             624000000
#define MSDC0_SRC_8             312000000

/* indexed by DT clk_src (MSDC30_CLKSRC_*); index 0 is clk26m */
#define MSDC1_SRC_0             26000000
#define MSDC1_SRC_1             208000000
#define MSDC1_SRC_2             100000000
#define MSDC1_SRC_3             156000000
#define MSDC1_SRC_4             182000000
#define MSDC1_SRC_5             156000000
#define MSDC1_SRC_6             178000000
#define MSDC1_SRC_7             200000000

#define MSDC_SRC_FPGA           12000000

/**************************************************************/
/* Section 4: GPIO and Pad                                    */
/**************************************************************/
/*--------------------------------------------------------------------------*/
/* MSDC0 GPIO and IO Pad Configuration Base                                 */
/*--------------------------------------------------------------------------*/
#define MSDC_GPIO_BASE          gpio_base               /* 0x10005000 */
#define MSDC0_IO_PAD_BASE       (msdc_io_cfg_bases[0])  /* 0x10002a00 */

/*--------------------------------------------------------------------------*/
/* MSDC0 GPIO Related Register (MT6755 values)                              */
/*--------------------------------------------------------------------------*/
/* eMMC pinmux lives on MODE18/19 (not the mt6757 MODE23/24) */
#define MSDC0_GPIO_MODE18       (MSDC_GPIO_BASE + 0x410)
#define MSDC0_GPIO_MODE19       (MSDC_GPIO_BASE + 0x420)
#define MSDC0_GPIO_IES_ADDR     (MSDC0_IO_PAD_BASE + 0x0)
#define MSDC0_GPIO_SMT_ADDR     (MSDC0_IO_PAD_BASE + 0x10)
#define MSDC0_GPIO_TDSEL_ADDR   (MSDC0_IO_PAD_BASE + 0x20)
#define MSDC0_GPIO_RDSEL_ADDR   (MSDC0_IO_PAD_BASE + 0x40)
#define MSDC0_GPIO_DRV_ADDR     (MSDC0_IO_PAD_BASE + 0xa0)
#define MSDC0_GPIO_PUPD0_ADDR   (MSDC0_IO_PAD_BASE + 0xc0)
#define MSDC0_GPIO_PUPD1_ADDR   (MSDC0_IO_PAD_BASE + 0xd0)

/* MODE18[30:25] = DAT0 (@25) + CLK (@28), MODE19 = CMD/DAT1-7/DSL/RST */
#define MSDC0_MODE18_MASK       (0x3F << 25)
#define MSDC0_MODE18_VAL        (0x9)
#define MSDC0_MODE19_VAL        (0x12491249)

#define MSDC0_IES_ALL_MASK      (0x1f <<  0)
#define MSDC0_SMT_ALL_MASK      (0x1f <<  0)
#define MSDC0_TDSEL_ALL_MASK    (0xfffff << 0)
#define MSDC0_RDSEL_ALL_MASK    (0x3fffffff << 0)

/* 3-bit DRV fields; the 4th bit of each nibble is SR on mt6755 */
#define MSDC0_DRV_DAT_MASK      (0x7  <<  0)
#define MSDC0_DRV_CLK_MASK      (0x7  <<  4)
#define MSDC0_DRV_CMD_MASK      (0x7  <<  8)
#define MSDC0_DRV_RSTB_MASK     (0x7  << 12)
#define MSDC0_DRV_DSL_MASK      (0x7  << 16)

/* PUPD0: 7 nibbles (RSTB pull left untouched), PUPD1: 4 nibbles */
#define MSDC0_PUPD0_MASK        (0x7777777 << 0)
#define MSDC0_PUPD1_MASK        (0x7777 << 0)

/**************************************************************/
/* Section 5: Adjustable Driver Parameter                     */
/**************************************************************/
#define HOST_MAX_BLKSZ          (2048)

#define MSDC_OCR_AVAIL\
	(MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 \
	| MMC_VDD_31_32 | MMC_VDD_32_33)
/* data timeout counter. 1048576 * 3 sclk. */
#define DEFAULT_DTOC            (3)

#define MAX_DMA_CNT             (4 * 1024 * 1024)
/* a WIFI transaction may be 50K */
#define MAX_DMA_CNT_SDIO        (0xFFFFFFFF - 255)
/* a LTE  transaction may be 128K */

#define MAX_HW_SGMTS            (MAX_BD_NUM)
#define MAX_PHY_SGMTS           (MAX_BD_NUM)
#define MAX_SGMT_SZ             (MAX_DMA_CNT)
#define MAX_SGMT_SZ_SDIO        (MAX_DMA_CNT_SDIO)

/* msdc0 (eMMC) and msdc1 (SD) index space; msdc2 (SDIO) is not served */
#define HOST_MAX_NUM            (2)
#ifdef CONFIG_PWR_LOSS_MTK_TEST
#define MAX_REQ_SZ              (512 * 65536)
#else
#define MAX_REQ_SZ              (512 * 1024)
#endif

#define HOST_MAX_MCLK           (200000000)
#define HOST_MIN_MCLK           (260000)

/* SD card, bad card handling settings */

/* if continuous data timeout reach the limit */
/* driver will force remove card */
#define MSDC_MAX_DATA_TIMEOUT_CONTINUOUS (100)

/* if continuous power cycle fail reach the limit */
/* driver will force remove card */
#define MSDC_MAX_POWER_CYCLE_FAIL_CONTINUOUS (3)

#define SDCARD_ESD_RECOVERY

/**************************************************************/
/* Section 6: BBChip-depenent Tunnig Parameter                */
/**************************************************************/
/*
 * Native MT6755 values: the 3.18 mt6755/msdc_tune.c
 * msdc_init_tune_setting() writes PB0 0x403C000F, PB1 0xFFFE00C9,
 * RESPWAITCNT 3, RESPSTENSEL 0, CRCSTSENSEL 0, CLKTXDLY 0 (msdc_tune.h).
 */
#define EMMC_MAX_FREQ_DIV               4 /* lower frequence to 12.5M */
#define MSDC_CLKTXDLY                   0

#define VOL_CHG_CNT_DEFAULT_VAL         0x1F4 /* =500 */

#define MSDC_PB0_DEFAULT_VAL            0x403C000F
#define MSDC_PB1_DEFAULT_VAL            0xFFFE00C9

#define MSDC_PB2_DEFAULT_RESPWAITCNT    0x3
#define MSDC_PB2_DEFAULT_RESPSTENSEL    0x0
#define MSDC_PB2_DEFAULT_CRCSTSENSEL    0x0

/* the native mt6755 drivers do not program a busy timeout */
#define MSDC_HW_NO_BUSY_CHECK
#endif /* _MSDC_CUST_MT6755_H_ */
