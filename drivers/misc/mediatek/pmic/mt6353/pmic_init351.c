/*
 * MT6351 PMIC initial setting, applied at late_initcall behind a runtime
 * knob.
 *
 * The 219-entry table below is extracted from the stock 3.10.72 kernel
 * (drivers/misc/mediatek/power/mt6755/pmic_initial_setting.c,
 * PMIC_INIT_SETTING_V1()) and matches, entry for entry, the in-tree 4.4
 * port pmic/mt6351/v1/pmic_initial_setting.c PMIC_LP_INIT_SETTING(). The
 * stock kernel runs the list unconditionally at mt6311 i2c probe time,
 * i.e. late in boot after pwrap/PMIC are up, so applying it at runtime keeps
 * the stock order. Register-name comments are from mt6351_regs.h; bit-level
 * flag names are best-effort matches of the (addr,mask,shift) accessor
 * triples in pmic/include/mt6351/mach/upmu_hw.h and may alias where several
 * flags share a triple.
 *
 * Knobs (0644; KBUILD_MODNAME "pmic_init351", so
 * /sys/module/pmic_init351/parameters/ at runtime or pmic_init351.<knob>=N
 * on the cmdline, which LK truncates at ~99 characters):
 *   forge_init351     default 219 = apply the full table at late_initcall.
 *                     Value N: apply table entries [applied, N), one
 *                     [FORGE_P351] line each with full 16-bit pre/post
 *                     readback + field verify; monotonic per boot. Group
 *                     boundaries: 21 / 29 / 47 / 89 / 131 / 180 / 189 /
 *                     206 / 219. Reaching 219 also runs the stock trailing
 *                     PMIC_IMM_GetOneChannelValue(PMIC_AUX_CH11, 5, 0).
 *   forge_init351_md  0 (default, off); 1 = run the stock MD block once
 *                     (32k-less DCXO detect + TSX modem-temp share, with a
 *                     NULL-guarded iomap; the 3.10 original writes through
 *                     a NULL base when the DT node is missing). Touches
 *                     modem-shared state.
 *
 * PMIC registers are not reset by a SoC warm reset (HW WDT), so a bad group
 * survives a reboot; a power-key long-press hard power-off resets the PMIC
 * to defaults and LK re-initializes its rails on the next power-on.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <mt-plat/upmu_common.h>

struct forge_init351_ent {
	unsigned short addr;
	unsigned short val;
	unsigned short mask;
	unsigned char shift;
};

static const struct forge_init351_ent forge_init351_tab[] = {
	/* ---- G1 STRUP/TOP (PG-arm class + top status) (first index 0) ---- */
	{ 0x0008, 0x0001, 0x0001,  0 },	/*   0 STRUP_CON4 RG_OTP_RD_TRIG */
	{ 0x000A, 0x0001, 0x0001,  1 },	/*   1 STRUP_CON5 RG_AUDMICBIAS1DCSW1PEN */
	{ 0x000A, 0x0001, 0x0001,  2 },	/*   2 STRUP_CON5 RG_AUDMICBIAS1DCSW1NEN */
	{ 0x000A, 0x0001, 0x0001,  3 },	/*   3 STRUP_CON5 AUXADC_TEST_MODE */
	{ 0x000A, 0x0001, 0x0001,  4 },	/*   4 STRUP_CON5 AUXADC_BIT_SEL */
	{ 0x000A, 0x0001, 0x0001,  5 },	/*   5 STRUP_CON5 AUXADC_START_SW */
	{ 0x000A, 0x0001, 0x0001,  7 },	/*   6 STRUP_CON5 RG_STRUP_VDRAM_PG_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001,  8 },	/*   7 STRUP_CON5 RG_STRUP_VXO22_PG_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 10 },	/*   8 STRUP_CON5 RG_STRUP_VSRAM_MD_OC_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 11 },	/*   9 STRUP_CON5 RG_STRUP_VS2_OC_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 12 },	/*  10 STRUP_CON5 RG_STRUP_VMODEM_OC_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 13 },	/*  11 STRUP_CON5 RG_STRUP_VMD1_OC_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 14 },	/*  12 STRUP_CON5 RG_STRUP_VCORE_OC_DEB_BYPASS */
	{ 0x000A, 0x0001, 0x0001, 15 },	/*  13 STRUP_CON5 RG_STRUP_VS1_OC_DEB_BYPASS */
	{ 0x0018, 0x0001, 0x0001,  5 },	/*  14 STRUP_CON12 */
	{ 0x001C, 0x0001, 0x0001,  7 },	/*  15 STRUP_CON14 */
	{ 0x001E, 0x0001, 0x0001,  0 },	/*  16 STRUP_CON15 BIF_TEST_MODE0 */
	{ 0x001E, 0x0001, 0x0001,  1 },	/*  17 STRUP_CON15 BIF_TEST_MODE1 */
	{ 0x002C, 0x0001, 0x0001, 15 },	/*  18 STRUP_CON22 */
	{ 0x0032, 0x0001, 0x0001,  2 },	/*  19 STRUP_ANA_CON1 */
	{ 0x0032, 0x0001, 0x0001,  3 },	/*  20 STRUP_ANA_CON1 */
	{ 0x0204, 0x0001, 0x0001,  4 },	/*  21 TOP_CON */
	{ 0x0204, 0x0001, 0x0001,  5 },	/*  22 TOP_CON */
	{ 0x0204, 0x0001, 0x0001,  6 },	/*  23 TOP_CON */
	{ 0x0226, 0x0001, 0x0001,  0 },	/*  24 SMT_CON0 */
	{ 0x0228, 0x0001, 0x0001,  0 },	/*  25 SMT_CON1 */
	{ 0x0228, 0x0001, 0x0001,  1 },	/*  26 SMT_CON1 */
	{ 0x0228, 0x0001, 0x0001,  2 },	/*  27 SMT_CON1 */
	{ 0x0228, 0x0001, 0x0001,  3 },	/*  28 SMT_CON1 */
	{ 0x023A, 0x0000, 0x0001,  9 },	/*  29 TOP_CKPDN_CON0 */
	{ 0x023A, 0x0001, 0x0001, 11 },	/*  30 TOP_CKPDN_CON0 */
	{ 0x0240, 0x0001, 0x0001,  2 },	/*  31 TOP_CKPDN_CON1 */
	{ 0x0240, 0x0001, 0x0001,  3 },	/*  32 TOP_CKPDN_CON1 */
	{ 0x0240, 0x0000, 0x0001, 10 },	/*  33 TOP_CKPDN_CON1 */
	{ 0x0246, 0x0001, 0x0001, 13 },	/*  34 TOP_CKPDN_CON2 */
	{ 0x0246, 0x0001, 0x0001, 14 },	/*  35 TOP_CKPDN_CON2 */
	/* No TOP_CKPDN_CON2 0x70 (bits 4-6) write here: those bits gate a
	 * clock the system needs, and setting them makes the kernel
	 * bootloop. */
	{ 0x024C, 0x0000, 0x0001,  2 },	/*  36 TOP_CKPDN_CON3 */
	{ 0x0258, 0x0000, 0x0001,  5 },	/*  37 TOP_CKPDN_CON5 */
	{ 0x0258, 0x0001, 0x0001,  8 },	/*  38 TOP_CKPDN_CON5 */
	{ 0x0258, 0x0000, 0x0001, 14 },	/*  39 TOP_CKPDN_CON5 */
	{ 0x025E, 0x0001, 0x0001,  9 },	/*  40 TOP_CKSEL_CON0 */
	{ 0x025E, 0x0000, 0x0001, 10 },	/*  41 TOP_CKSEL_CON0 */
	{ 0x0282, 0x0000, 0x0001,  3 },	/*  42 TOP_CKHWEN_CON0 */
	{ 0x0282, 0x0000, 0x0001,  4 },	/*  43 TOP_CKHWEN_CON0 */
	{ 0x0282, 0x0000, 0x0001, 10 },	/*  44 TOP_CKHWEN_CON0 */
	{ 0x0282, 0x0000, 0x0001, 11 },	/*  45 TOP_CKHWEN_CON0 */
	{ 0x028E, 0x0000, 0x0001,  4 },	/*  46 TOP_CKHWEN_CON2 */
	/* ---- G2 BUCK ctrl/OC (VCORE/VGPU/VMODEM/VS1...) (first index 47) ---- */
	{ 0x0410, 0x0008, 0x003F,  8 },	/*  47 BUCK_DLC_VPA_CON1 */
	{ 0x0414, 0x0003, 0x0003,  4 },	/*  48 BUCK_DVS_VPA_CON0 */
	{ 0x0422, 0x0001, 0x0001,  7 },	/*  49 BUCK_OC_CON2 */
	{ 0x0436, 0x0000, 0x0003,  2 },	/*  50 BUCK_OC_VPA_CON0 */
	{ 0x0450, 0x000F, 0x000F, 11 },	/*  51 VCORE_ANA_CON1 */
	{ 0x0452, 0x0001, 0x0001,  3 },	/*  52 VCORE_ANA_CON2 */
	{ 0x0456, 0x0001, 0x0001,  7 },	/*  53 VCORE_ANA_CON4 */
	{ 0x045C, 0x0001, 0x0001,  1 },	/*  54 VCORE_ANA_CON7 */
	{ 0x045E, 0x0400, 0xFFFF,  0 },	/*  55 VCORE_ANA_CON8 */
	{ 0x0464, 0x000F, 0x000F, 11 },	/*  56 VGPU_ANA_CON1 */
	{ 0x0466, 0x0001, 0x0001,  3 },	/*  57 VGPU_ANA_CON2 */
	{ 0x0466, 0x0005, 0x0007,  9 },	/*  58 VGPU_ANA_CON2 */
	{ 0x046A, 0x0001, 0x0001,  7 },	/*  59 VGPU_ANA_CON4 */
	{ 0x0470, 0x0001, 0x0001,  1 },	/*  60 VGPU_ANA_CON7 */
	{ 0x0472, 0x0400, 0xFFFF,  0 },	/*  61 VGPU_ANA_CON8 */
	{ 0x0478, 0x000F, 0x000F, 11 },	/*  62 VSRAM_MD_ANA_CON1 */
	{ 0x047A, 0x0002, 0x0007,  6 },	/*  63 VSRAM_MD_ANA_CON2 */
	{ 0x047E, 0x0001, 0x0001,  1 },	/*  64 VSRAM_MD_ANA_CON4 */
	{ 0x0480, 0x0001, 0x0007,  4 },	/*  65 VSRAM_MD_ANA_CON5 */
	{ 0x0484, 0x0001, 0x0001,  1 },	/*  66 VSRAM_MD_ANA_CON7 */
	{ 0x048C, 0x000F, 0x000F, 11 },	/*  67 VMODEM_ANA_CON1 */
	{ 0x048E, 0x0002, 0x0007,  6 },	/*  68 VMODEM_ANA_CON2 */
	{ 0x0492, 0x0001, 0x0001,  1 },	/*  69 VMODEM_ANA_CON4 */
	{ 0x0494, 0x0001, 0x0007,  4 },	/*  70 VMODEM_ANA_CON5 */
	{ 0x0498, 0x0001, 0x0001,  1 },	/*  71 VMODEM_ANA_CON7 */
	{ 0x04A0, 0x000F, 0x000F, 11 },	/*  72 VMD1_ANA_CON1 */
	{ 0x04A2, 0x0002, 0x0007,  6 },	/*  73 VMD1_ANA_CON2 */
	{ 0x04A6, 0x0001, 0x0001,  1 },	/*  74 VMD1_ANA_CON4 */
	{ 0x04A8, 0x0001, 0x0007,  4 },	/*  75 VMD1_ANA_CON5 */
	{ 0x04AC, 0x0001, 0x0001,  1 },	/*  76 VMD1_ANA_CON7 */
	{ 0x04B6, 0x0006, 0x0007,  6 },	/*  77 VS1_ANA_CON2 */
	{ 0x04C2, 0x0010, 0xFFFF,  0 },	/*  78 VS1_ANA_CON8 */
	{ 0x04C8, 0x000F, 0x000F, 11 },	/*  79 VS2_ANA_CON1 */
	{ 0x04CA, 0x0005, 0x0007,  6 },	/*  80 VS2_ANA_CON2 */
	{ 0x04CE, 0x0001, 0x0001,  1 },	/*  81 VS2_ANA_CON4 */
	{ 0x04D0, 0x0001, 0x0007,  4 },	/*  82 VS2_ANA_CON5 */
	{ 0x04DC, 0x0003, 0x0003,  0 },	/*  83 VPA_ANA_CON1 */
	{ 0x04DC, 0x0002, 0x0003,  4 },	/*  84 VPA_ANA_CON1 */
	{ 0x04DC, 0x0000, 0x0001, 10 },	/*  85 VPA_ANA_CON1 */
	{ 0x04DC, 0x0001, 0x0003, 14 },	/*  86 VPA_ANA_CON1 */
	{ 0x04E0, 0x0000, 0x0003, 14 },	/*  87 VPA_ANA_CON3 */
	{ 0x04E2, 0x0088, 0x00FF,  8 },	/*  88 VPA_ANA_CON4 */
	/* ---- G3 BUCK vsleep/mode tuning (first index 89) ---- */
	{ 0x0600, 0x0001, 0x0001,  1 },	/*  89 BUCK_VCORE_CON0 */
	{ 0x0606, 0x0011, 0x007F,  0 },	/*  90 BUCK_VCORE_CON3 */
	{ 0x0606, 0x0004, 0x007F,  8 },	/*  91 BUCK_VCORE_CON3 */
	{ 0x060C, 0x0010, 0x007F,  0 },	/*  92 BUCK_VCORE_CON6 */
	{ 0x0612, 0x0003, 0x0003,  0 },	/*  93 BUCK_VCORE_CON9 */
	{ 0x0612, 0x0001, 0x0001,  8 },	/*  94 BUCK_VCORE_CON9 */
	{ 0x061A, 0x0011, 0x007F,  0 },	/*  95 BUCK_VGPU_CON3 */
	{ 0x061A, 0x0004, 0x007F,  8 },	/*  96 BUCK_VGPU_CON3 */
	{ 0x0620, 0x0010, 0x007F,  0 },	/*  97 BUCK_VGPU_CON6 */
	{ 0x0626, 0x0003, 0x0003,  0 },	/*  98 BUCK_VGPU_CON9 */
	{ 0x0626, 0x0001, 0x0003,  4 },	/*  99 BUCK_VGPU_CON9 */
	{ 0x062E, 0x0011, 0x007F,  0 },	/* 100 BUCK_VMODEM_CON3 */
	{ 0x062E, 0x0004, 0x007F,  8 },	/* 101 BUCK_VMODEM_CON3 */
	{ 0x0634, 0x0010, 0x007F,  0 },	/* 102 BUCK_VMODEM_CON6 */
	{ 0x063A, 0x0003, 0x0003,  0 },	/* 103 BUCK_VMODEM_CON9 */
	{ 0x063A, 0x0001, 0x0003,  4 },	/* 104 BUCK_VMODEM_CON9 */
	{ 0x063A, 0x0001, 0x0001,  8 },	/* 105 BUCK_VMODEM_CON9 */
	{ 0x0642, 0x0011, 0x007F,  0 },	/* 106 BUCK_VMD1_CON3 */
	{ 0x0642, 0x0004, 0x007F,  8 },	/* 107 BUCK_VMD1_CON3 */
	{ 0x0646, 0x0030, 0x007F,  0 },	/* 108 BUCK_VMD1_CON5 */
	{ 0x0648, 0x0010, 0x007F,  0 },	/* 109 BUCK_VMD1_CON6 */
	{ 0x064E, 0x0003, 0x0003,  0 },	/* 110 BUCK_VMD1_CON9 */
	{ 0x064E, 0x0001, 0x0003,  4 },	/* 111 BUCK_VMD1_CON9 */
	{ 0x064E, 0x0001, 0x0001,  8 },	/* 112 BUCK_VMD1_CON9 */
	{ 0x0656, 0x0011, 0x007F,  0 },	/* 113 BUCK_VSRAM_MD_CON3 */
	{ 0x0656, 0x0004, 0x007F,  8 },	/* 114 BUCK_VSRAM_MD_CON3 */
	{ 0x065C, 0x0010, 0x007F,  0 },	/* 115 BUCK_VSRAM_MD_CON6 */
	{ 0x0662, 0x0003, 0x0003,  0 },	/* 116 BUCK_VSRAM_MD_CON9 */
	{ 0x0662, 0x0001, 0x0003,  4 },	/* 117 BUCK_VSRAM_MD_CON9 */
	{ 0x0662, 0x0001, 0x0001,  8 },	/* 118 BUCK_VSRAM_MD_CON9 */
	{ 0x0676, 0x0001, 0x0001,  8 },	/* 119 BUCK_VS1_CON9 */
	{ 0x068A, 0x0001, 0x0001,  8 },	/* 120 BUCK_VS2_CON9 */
	{ 0x0692, 0x0000, 0x007F,  0 },	/* 121 BUCK_VPA_CON3 */
	{ 0x0692, 0x0001, 0x007F,  8 },	/* 122 BUCK_VPA_CON3 */
	{ 0x069E, 0x0000, 0x0003,  0 },	/* 123 BUCK_VPA_CON9 */
	{ 0x06A0, 0x0001, 0x0001,  1 },	/* 124 BUCK_VSRAM_PROC_CON0 */
	{ 0x06A6, 0x0011, 0x007F,  0 },	/* 125 BUCK_VSRAM_PROC_CON3 */
	{ 0x06A6, 0x0004, 0x007F,  8 },	/* 126 BUCK_VSRAM_PROC_CON3 */
	{ 0x06AC, 0x0035, 0x007F,  0 },	/* 127 BUCK_VSRAM_PROC_CON6 */
	{ 0x06B2, 0x0003, 0x0003,  0 },	/* 128 BUCK_VSRAM_PROC_CON9 */
	{ 0x06B2, 0x0001, 0x0003,  4 },	/* 129 BUCK_VSRAM_PROC_CON9 */
	{ 0x06B2, 0x0000, 0x0001,  8 },	/* 130 BUCK_VSRAM_PROC_CON9 */
	/* ---- G4 LDO low-power/sleep bits (first index 131) ---- */
	{ 0x0A00, 0x0001, 0x0001,  2 },	/* 131 LDO_VA18_CON0 */
	{ 0x0A00, 0x0000, 0x0007,  5 },	/* 132 LDO_VA18_CON0 */
	{ 0x0A02, 0x0001, 0x0001,  9 },	/* 133 LDO_VA18_CON1 */
	{ 0x0A04, 0x0001, 0x0001,  3 },	/* 134 LDO_VTCXO24_CON0 */
	{ 0x0A04, 0x0000, 0x0007, 11 },	/* 135 LDO_VTCXO24_CON0 */
	{ 0x0A06, 0x0001, 0x0001,  9 },	/* 136 LDO_VTCXO24_CON1 */
	{ 0x0A08, 0x0001, 0x0001,  3 },	/* 137 LDO_VTCXO28_CON0 */
	{ 0x0A08, 0x0001, 0x0007, 11 },	/* 138 LDO_VTCXO28_CON0 */
	{ 0x0A0A, 0x0001, 0x0001,  9 },	/* 139 LDO_VTCXO28_CON1 */
	{ 0x0A0E, 0x0001, 0x0001,  9 },	/* 140 LDO_VCN28_CON1 */
	{ 0x0A14, 0x0001, 0x0001,  9 },	/* 141 LDO_VCAMA_CON1 */
	{ 0x0A16, 0x0001, 0x0001,  2 },	/* 142 LDO_VUSB33_CON0 */
	{ 0x0A16, 0x0000, 0x0007,  5 },	/* 143 LDO_VUSB33_CON0 */
	{ 0x0A18, 0x0001, 0x0001,  9 },	/* 144 LDO_VUSB33_CON1 */
	{ 0x0A1E, 0x0001, 0x0001,  9 },	/* 145 LDO_VSIM1_CON1 */
	{ 0x0A24, 0x0001, 0x0001,  9 },	/* 146 LDO_VSIM2_CON1 */
	{ 0x0A2A, 0x0001, 0x0001,  9 },	/* 147 LDO_VEMC_CON1 */
	{ 0x0A30, 0x0001, 0x0001,  9 },	/* 148 LDO_VMCH_CON1 */
	{ 0x0A34, 0x0001, 0x0001,  2 },	/* 149 LDO_VIO28_CON0 */
	{ 0x0A34, 0x0000, 0x0007,  5 },	/* 150 LDO_VIO28_CON0 */
	{ 0x0A36, 0x0001, 0x0001,  9 },	/* 151 LDO_VIO28_CON1 */
	{ 0x0A3C, 0x0001, 0x0001,  9 },	/* 152 LDO_VIBR_CON1 */
	{ 0x0A46, 0x0001, 0x0001,  3 },	/* 153 LDO_VRF18_CON0 */
	{ 0x0A46, 0x0001, 0x0007, 11 },	/* 154 LDO_VRF18_CON0 */
	{ 0x0A48, 0x0001, 0x0001,  9 },	/* 155 LDO_VRF18_CON1 */
	{ 0x0A4C, 0x0001, 0x0001,  2 },	/* 156 LDO_VIO18_CON0 */
	{ 0x0A4C, 0x0000, 0x0007,  5 },	/* 157 LDO_VIO18_CON0 */
	{ 0x0A4E, 0x0001, 0x0001,  9 },	/* 158 LDO_VIO18_CON1 */
	{ 0x0A54, 0x0001, 0x0001,  9 },	/* 159 LDO_VCN18_CON1 */
	{ 0x0A5A, 0x0001, 0x0001,  9 },	/* 160 LDO_VCAMIO_CON1 */
	{ 0x0A66, 0x0001, 0x0001,  9 },	/* 161 LDO_VXO22_CON1 */
	{ 0x0A68, 0x0001, 0x0001,  3 },	/* 162 LDO_VRF12_CON0 */
	{ 0x0A68, 0x0001, 0x0007, 11 },	/* 163 LDO_VRF12_CON0 */
	{ 0x0A6E, 0x0001, 0x0001,  2 },	/* 164 LDO_VA10_CON0 */
	{ 0x0A6E, 0x0000, 0x0007,  5 },	/* 165 LDO_VA10_CON0 */
	{ 0x0A74, 0x0001, 0x0001,  1 },	/* 166 LDO_VDRAM_CON0 */
	{ 0x0A74, 0x0001, 0x0001,  2 },	/* 167 LDO_VDRAM_CON0 */
	{ 0x0A74, 0x0000, 0x0001,  3 },	/* 168 LDO_VDRAM_CON0 */
	{ 0x0A74, 0x0000, 0x0007,  5 },	/* 169 LDO_VDRAM_CON0 */
	{ 0x0A7C, 0x0001, 0x0001,  9 },	/* 170 LDO_VMIPI_CON1 */
	{ 0x0A82, 0x0001, 0x0001,  9 },	/* 171 LDO_VGP3_CON1 */
	{ 0x0A86, 0x0000, 0x0001,  3 },	/* 172 LDO_VBIF28_CON0 */
	{ 0x0A88, 0x0001, 0x0001,  9 },	/* 173 LDO_VBIF28_CON1 */
	{ 0x0A8E, 0x0001, 0x0001,  9 },	/* 174 LDO_VEFUSE_CON1 */
	{ 0x0A94, 0x0001, 0x0001,  9 },	/* 175 LDO_VCN33_CON1 */
	{ 0x0A9C, 0x0001, 0x0001,  2 },	/* 176 LDO_VLDO28_CON0 */
	{ 0x0A9C, 0x0000, 0x0007,  5 },	/* 177 LDO_VLDO28_CON0 */
	{ 0x0A9E, 0x0001, 0x0001,  9 },	/* 178 LDO_VLDO28_CON1 */
	{ 0x0AAC, 0x0001, 0x0001,  9 },	/* 179 LDO_VMC_CON1 */
	/* ---- G5 AUXADC/FGADC config (first index 180) ---- */
	{ 0x0B10, 0x0002, 0x0007,  8 },	/* 180 VA10_ANA_CON0 */
	{ 0x0B24, 0x00F0, 0x00FF,  2 },	/* 181 VDRAM_ANA_CON1 */
	{ 0x0CC4, 0x0001, 0x0001,  8 },	/* 182 FGADC_CON16 */
	{ 0x0CC4, 0x0001, 0x0001,  9 },	/* 183 FGADC_CON16 */
	{ 0x0CC8, 0x001F, 0xFFFF,  0 },	/* 184 FGADC_CON18 */
	{ 0x0CCA, 0x0014, 0x00FF,  0 },	/* 185 FGADC_CON19 */
	{ 0x0CCC, 0x00FF, 0x00FF,  8 },	/* 186 FGADC_CON20 */
	{ 0x0CE2, 0x0001, 0x7FFF,  0 },	/* 187 FGADC_CON31 */
	{ 0x0CE4, 0xBCAC, 0xFFFF,  0 },	/* 188 FGADC_CON32 */
	/* ---- G6 0xEAx-0xECx block (first index 189) ---- */
	/* The G6 AUXADC channel-config block is left out: it reconfigures the
	 * MT6351 AUXADC (CON0/CON2/CON6/CON7/CON16) so ch3 (BATON/bat-temp)
	 * reads ~1640 mV instead of ~646 mV, i.e. a false -5 C,
	 * under-temperature and CHGEN=0. Without it the defaults read the
	 * temperature correctly (AUXADC_CON2 0x0083, CON6 0x2040,
	 * TOP_CKPDN_CON2 0x3070 instead of 0x8283 / 0x95D0 / 0x7000). VBAT, a
	 * different channel, is unaffected; all display/audio rail writes
	 * (G1-G5, G7) stay. */
#if 0
	{ 0x0EA2, 0x0000, 0x0001, 13 },	/* 189 AUXADC_CON0 */
	{ 0x0EA2, 0x0000, 0x0001, 14 },	/* 190 AUXADC_CON0 */
	{ 0x0EA2, 0x0000, 0x0001, 15 },	/* 191 AUXADC_CON0 */
	{ 0x0EAA, 0x0083, 0x0FFF,  0 },	/* 192 AUXADC_CON2 */
	{ 0x0EAA, 0x0000, 0x0001, 13 },	/* 193 AUXADC_CON2 */
	{ 0x0EAA, 0x0001, 0x0001, 15 },	/* 194 AUXADC_CON2 */
	{ 0x0EB2, 0x0001, 0x0003,  4 },	/* 195 AUXADC_CON6 */
	{ 0x0EB2, 0x0003, 0x0003,  6 },	/* 196 AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003,  8 },	/* 197 AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003, 10 },	/* 198 AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003, 12 },	/* 199 AUXADC_CON6 */
	{ 0x0EB2, 0x0002, 0x0003, 14 },	/* 200 AUXADC_CON6 */
	{ 0x0EB4, 0x0001, 0x0003,  0 },	/* 201 AUXADC_CON7 */
	{ 0x0EB4, 0x0001, 0x0003,  2 },	/* 202 AUXADC_CON7 */
	{ 0x0EB4, 0x0001, 0x0003,  4 },	/* 203 AUXADC_CON7 */
	{ 0x0EB4, 0x0003, 0x0003,  6 },	/* 204 AUXADC_CON7 */
	{ 0x0EC6, 0x0001, 0x0001, 14 },	/* 205 AUXADC_CON16 */
#endif
	/* ---- G7 STRUP/RG tail + thermal/dcxo-adj (first index 206) ---- */
	{ 0x0F16, 0x000C, 0x03FF,  0 },	/* 206 AUXADC_MDBG_0 */
	{ 0x0F16, 0x0000, 0x0001, 15 },	/* 207 AUXADC_MDBG_0 */
	{ 0x0F1C, 0x000C, 0x03FF,  0 },	/* 208 AUXADC_MDRT_0 */
	{ 0x0F1C, 0x0001, 0x0001, 15 },	/* 209 AUXADC_MDRT_0 */
	{ 0x0F20, 0x0001, 0x0001,  2 },	/* 210 AUXADC_MDRT_2 */
	{ 0x0F7A, 0x000B, 0x000F,  4 },	/* 211 CHR_CON1 */
	{ 0x0F84, 0x0004, 0x000F,  1 },	/* 212 CHR_CON6 */
	{ 0x0F92, 0x0003, 0x000F,  0 },	/* 213 CHR_CON13 */
	{ 0x0FA0, 0x0001, 0x0001,  1 },	/* 214 CHR_CON20 */
	{ 0x0FA4, 0x0000, 0x0007,  4 },	/* 215 CHR_CON22 */
	{ 0x0FAA, 0x0001, 0x0001,  2 },	/* 216 CHR_CON25 */
	{ 0x0FAA, 0x0001, 0x0001,  6 },	/* 217 CHR_CON25 */
	/* Entry 218 (CHR_CON25 bit7 RG_ULC_DET_EN) is left out: applying it
	 * flips the bat-temp reading to about -2 C (false under-temperature,
	 * charging off), while every other table entry is safe. The charger
	 * FSM (charging_hw_pmic.c) owns CHR_CON25 bits 2/6/7 per tick
	 * anyway. */
#if 0
	{ 0x0FAA, 0x0001, 0x0001,  7 },	/* 218 CHR_CON25 RG_ULC_DET_EN = temp-ADC breaker */
#endif
};

#define FORGE_INIT351_TOTAL	((int)ARRAY_SIZE(forge_init351_tab))

/* Applied at boot by default. Disable with pmic_init351.forge_init351=0 on
 * the cmdline (mind the LK cmdline length). */
static int forge_init351 = FORGE_INIT351_TOTAL;	/* boot: apply full table */
static int forge_init351_md;	/* 1 = run the stock MD block once */
static int forge_init351_applied;
static int forge_init351_md_done;
static bool forge_init351_boot_done;
static DEFINE_MUTEX(forge_init351_lock);

/* When the CPU rails are to be taken at the cpufreq arm (forge_vrail.allow
 * = 1 and an FY chip, forge_dvfs_rails_wanted() in mt_cpufreq.c), entries
 * 124 (VSRAM VOSEL_CTRL = 1, HW mode) and 127 (VSRAM VOSEL_SLEEP 0x35) are
 * skipped: VSRAM stays in the VOSEL_CTRL = 0 mode left by the preloader
 * with VOSEL_SLEEP = VOSEL, which forge_vrail_take() requires. Decided
 * once, before the boot apply. */
extern bool forge_dvfs_rails_wanted(void);
static bool forge_init351_vsram_sw;

static bool forge_init351_vsram_mode(const struct forge_init351_ent *e)
{
	return (e->addr == 0x06A0 && e->shift == 1) ||	/* 124 VOSEL_CTRL */
	       (e->addr == 0x06AC && e->shift == 0);	/* 127 VOSEL_SLEEP */
}

/* Apply table entries [forge_init351_applied, target).  Caller holds the
 * lock.  Every write is logged with a full 16-bit pre/post readback and a
 * field check so a single dmesg pull names the exact first divergent write. */
static void forge_init351_apply_locked(int target)
{
	int i, mism = 0, rcfail = 0;
	int first = forge_init351_applied;

	if (target > FORGE_INIT351_TOTAL)
		target = FORGE_INIT351_TOTAL;
	if (target <= first) {
		pr_err("[FORGE_P351] nothing to do (applied=%d target=%d)\n",
		       first, target);
		return;
	}

	for (i = first; i < target; i++) {
		const struct forge_init351_ent *e = &forge_init351_tab[i];
		unsigned int pre = 0xDEAD, post = 0xDEAD, field;
		unsigned int rc;
		bool contended;

		if (forge_init351_vsram_sw && forge_init351_vsram_mode(e)) {
			pmic_read_interface(e->addr, &pre, 0xFFFF, 0);
			pr_err("[FORGE_P351] %3d 0x%04x pre=0x%04x skipped: VSRAM stays VOSEL_CTRL=0 for the CPU rails\n",
			       i, e->addr, pre);
			continue;
		}
		pmic_read_interface(e->addr, &pre, 0xFFFF, 0);
		rc = pmic_config_interface(e->addr, e->val, e->mask, e->shift);
		pmic_read_interface(e->addr, &post, 0xFFFF, 0);
		field = (post >> e->shift) & e->mask;
		/* CHR_CON25 0x0FAA is owned by the charger FSM (bit2
		 * RG_CSDAC_MODE / bit6 RG_HWCV_EN / bit7 RG_ULC_DET_EN are
		 * rewritten by charging_hw_pmic.c per tick, bits 8/9 show
		 * status), so a readback mismatch on this register is expected,
		 * not a failure. */
		contended = (e->addr == 0x0FAA);
		if (rc)
			rcfail++;
		if (field != e->val && !contended)
			mism++;
		pr_info("[FORGE_P351] %3d 0x%04x pre=0x%04x post=0x%04x want=0x%x<<%u mask=0x%x%s%s\n",
			i, e->addr, pre, post, e->val, e->shift, e->mask,
			rc ? " PWRAP-FAIL" : "",
			(field != e->val) ?
				(contended ? " contended-ok(CHR_CON25)"
					   : " FIELD-MISMATCH") : "");
	}
	forge_init351_applied = target;
	pr_err("[FORGE_P351] applied [%d,%d) of %d: field-mismatch=%d pwrap-fail=%d\n",
	       first, target, FORGE_INIT351_TOTAL, mism, rcfail);

	if (target == FORGE_INIT351_TOTAL) {
		/* stock tail: prime auxadc CH11 exactly like the stock flow */
		int ch11 = PMIC_IMM_GetOneChannelValue(PMIC_AUX_CH11, 5, 0);

		pr_err("[FORGE_P351] table COMPLETE; AUX_CH11 prime read = %d\n",
		       ch11);
	}
}

/*
 * Stock MD block, verbatim from 3.10 PMIC_MD_INIT_SETTING_V1() (== the
 * in-tree mt6351/v1 copy): 32k-less crystal auto-detect (reads DCXO CW00
 * 0x7000 and picks the VCTCXO-on-MT6176 vs XO-on-MT6351 low-power branch;
 * M681 is RF-VCTCXO, so bit9 is expected set) and the TSX modem-temp share
 * MMIO. Only change: the of_iomap result is NULL-guarded (the 3.10 original
 * writes through a NULL base when the "mediatek,MODEM_TEMP_SHARE" node is
 * absent).
 */
static void forge_init351_md_run_locked(void)
{
	unsigned int ret = 0;
	unsigned int pmic_reg = 0;
	struct device_node *modem_temp_node;
	void __iomem *modem_temp_base = NULL;

	if (forge_init351_md_done) {
		pr_err("[FORGE_P351] MD block already run\n");
		return;
	}

	/* 32k less crystal auto detect start */
	ret |= pmic_config_interface(0x701E, 0x1, 0x1, 0);
	ret |= pmic_config_interface(0x701E, 0x3, 0xF, 1);
	ret = pmic_read_interface(0x7000, &pmic_reg, 0xffff, 0);
	ret |= pmic_config_interface(0x701E, 0x0, 0x1, 0);
	ret = pmic_config_interface(0xA04, 0x1, 0x1, 3);
	pr_err("[FORGE_P351] MD 32k-less: DCXO_CW00(0x7000)=0x%04x\n", pmic_reg);
	if ((pmic_reg & 0x200) == 0x200) {
		/* VCTCXO on MT6176 , OFF XO on MT6351
		    HW control, use srclken_0 */
		ret = pmic_config_interface(0xA04, 0x0, 0x7, 11);
		pr_err("[PMIC] VCTCXO on MT6176 , OFF XO on MT6351\n");
	} else {
		/*  HW control, use srclken_1, for LP */
		ret = pmic_config_interface(0xA04, 0x1, 0x1, 4);
		ret = pmic_config_interface(0xA04, 0x1, 0x7, 11);
		pr_err("[PMIC] VCTCXO 0x7000=0x%x\n", pmic_reg);
	}

	modem_temp_node = of_find_compatible_node(NULL, NULL,
						  "mediatek,MODEM_TEMP_SHARE");
	if (modem_temp_node)
		modem_temp_base = of_iomap(modem_temp_node, 0);
	if (!modem_temp_base) {
		pr_err("[FORGE_P351] MD TSX: MODEM_TEMP_SHARE %s -> skip\n",
		       modem_temp_node ? "iomap failed" : "node absent");
	} else {
		/* modem temp */
		writel(0x011f, modem_temp_base);
		pr_err("[PMIC] TEMP_SHARE_CTRL:0x%x\n", readl(modem_temp_base));
		/* modem temp */
		writel(0x013f, modem_temp_base + 0x04);
		/* modem temp */
		writel(0x0, modem_temp_base);
		pr_err("[PMIC] TEMP_SHARE_CTRL:0x%x _RATIO:0x%x\n",
		       readl(modem_temp_base), readl(modem_temp_base + 0x04));
		iounmap(modem_temp_base);
	}

	forge_init351_md_done = 1;
	pr_err("[FORGE_P351] MD block done (ret=0x%x)\n", ret);
}

static void forge_init351_run(void)
{
	mutex_lock(&forge_init351_lock);
	if (forge_init351 > 0)
		forge_init351_apply_locked(forge_init351);
	if (forge_init351_md)
		forge_init351_md_run_locked();
	mutex_unlock(&forge_init351_lock);
}

static int forge_init351_set(const char *sval, const struct kernel_param *kp)
{
	int rc = param_set_int(sval, kp);

	if (rc)
		return rc;
	/* before late_initcall the value is only stored; the initcall applies
	 * it once pwrap/PMIC are certainly up (they are postcore/fs_initcall) */
	if (forge_init351_boot_done)
		forge_init351_run();
	return 0;
}

static const struct kernel_param_ops forge_init351_ops = {
	.set = forge_init351_set,
	.get = param_get_int,
};

module_param_cb(forge_init351, &forge_init351_ops, &forge_init351, 0644);
module_param_cb(forge_init351_md, &forge_init351_ops, &forge_init351_md, 0644);

/* ==== Bat-temp runtime heal ============================================
 *
 * With the table applied, AUXADC ch3 (BATON) could read ~1550 mV instead
 * of ~646 mV, i.e. a false -5..-2 C, "Battery Under Temperature" and
 * charging off. Two causes are handled: CHR_CON25 bit7 RG_ULC_DET_EN (left
 * out of the table) and the ch11 PAD_VBIF28 measure at the end of the
 * table, whose restore block left RG_ADCIN_VSEN_MUX_EN (CHR_CON16 bit8) on,
 * hanging the 2.8 V PAD_VBIF28 sense path on the shared BATON/ADCIN network
 * (effective NTC pull-up ~5.5k instead of 24k).
 *   - the pmic_auxadc.c case 11 restore block is live
 *     (forge_ch11_restore=1), so the boot measure restores the mux as in
 *     stock and the delayed heal below only logs a correct value;
 *   - the delayed heal re-establishes the bat-temp path at runtime. It
 *     touches only the temp path (BATON/VSEN mux, TDET, ULC_DET, VBIF28
 *     mode bits, optionally the G6 AUXADC config), never the BUCK/LDO rails
 *     (a whole-table re-apply at runtime crashes the phone).
 *
 * Knobs (0644, /sys/module/pmic_init351/parameters/):
 *   forge_ch11_restore  1 = ch11 restore block live (0 = leave the mux on)
 *   forge_temp_tdet     final BATON_TDET_EN after restore/heal (1 = stock)
 *   forge_temp_heal     1 = auto-heal via delayed_work after boot-apply
 *   forge_temp_heal_s   auto-heal delay, seconds (default 25)
 *   forge_temp_heal_g6  1 = the heal also re-applies the 17 stock G6 AUXADC
 *                       config writes (stock values, stock-late timing)
 *   forge_temp_heal_now write 1 = run the heal now, synchronously, any
 *                       number of times
 * Watch: dmesg | grep -E "FORGE_THEAL|force_get_tbat|charging_enable"
 */
int forge_ch11_restore = 1;
module_param(forge_ch11_restore, int, 0644);
int forge_temp_tdet = 1;
module_param(forge_temp_tdet, int, 0644);

static int forge_temp_heal = 1;
module_param(forge_temp_heal, int, 0644);
static int forge_temp_heal_s = 25;
module_param(forge_temp_heal_s, int, 0644);
static int forge_temp_heal_g6;
module_param(forge_temp_heal_g6, int, 0644);

/* Stock G6 AUXADC channel-config block (original table entries 189-205,
 * '#if 0'-ed out of the boot table above).  Kept here so the heal can A/B
 * "stock AUXADC config applied at stock-like LATE timing" without reviving
 * the boot-time application, which is harmful. */
static const struct forge_init351_ent forge_temp_g6_tab[] = {
	{ 0x0EA2, 0x0000, 0x0001, 13 },	/* AUXADC_CON0 */
	{ 0x0EA2, 0x0000, 0x0001, 14 },	/* AUXADC_CON0 */
	{ 0x0EA2, 0x0000, 0x0001, 15 },	/* AUXADC_CON0 */
	{ 0x0EAA, 0x0083, 0x0FFF,  0 },	/* AUXADC_CON2 */
	{ 0x0EAA, 0x0000, 0x0001, 13 },	/* AUXADC_CON2 */
	{ 0x0EAA, 0x0001, 0x0001, 15 },	/* AUXADC_CON2 */
	{ 0x0EB2, 0x0001, 0x0003,  4 },	/* AUXADC_CON6 */
	{ 0x0EB2, 0x0003, 0x0003,  6 },	/* AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003,  8 },	/* AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003, 10 },	/* AUXADC_CON6 */
	{ 0x0EB2, 0x0001, 0x0003, 12 },	/* AUXADC_CON6 */
	{ 0x0EB2, 0x0002, 0x0003, 14 },	/* AUXADC_CON6 */
	{ 0x0EB4, 0x0001, 0x0003,  0 },	/* AUXADC_CON7 */
	{ 0x0EB4, 0x0001, 0x0003,  2 },	/* AUXADC_CON7 */
	{ 0x0EB4, 0x0001, 0x0003,  4 },	/* AUXADC_CON7 */
	{ 0x0EB4, 0x0003, 0x0003,  6 },	/* AUXADC_CON7 */
	{ 0x0EC6, 0x0001, 0x0001, 14 },	/* AUXADC_CON16 */
};

static DEFINE_MUTEX(forge_temp_heal_mutex);

static void forge_temp_heal_regs(const char *tag)
{
	unsigned int c7 = 0xDEAD, c16 = 0xDEAD, c25 = 0xDEAD;
	unsigned int vb0 = 0xDEAD, ck2 = 0xDEAD, a13 = 0xDEAD;

	pmic_read_interface(0x0F86, &c7,  0xFFFF, 0); /* CHR_CON7  TDET=bit2  */
	pmic_read_interface(0x0F98, &c16, 0xFFFF, 0); /* CHR_CON16 VSEN_MUX=bit8 */
	pmic_read_interface(0x0FAA, &c25, 0xFFFF, 0); /* CHR_CON25 ULC=bit7   */
	pmic_read_interface(0x0A86, &vb0, 0xFFFF, 0); /* LDO_VBIF28_CON0      */
	pmic_read_interface(0x0246, &ck2, 0xFFFF, 0); /* TOP_CKPDN_CON2       */
	pmic_read_interface(0x0EC0, &a13, 0xFFFF, 0); /* AUXADC_CON13         */
	pr_err("[FORGE_THEAL] %s CHR_CON7=0x%04x CHR_CON16=0x%04x CHR_CON25=0x%04x VBIF28_CON0=0x%04x CKPDN_CON2=0x%04x AUXADC_CON13=0x%04x\n",
	       tag, c7, c16, c25, vb0, ck2, a13);
}

/* Re-establish only the bat-temp ch3/BATON path. No rail/BUCK/LDO writes
 * (VBIF28_CON0 bits 1-3 are mode-select bits of an LDO that is already up).
 * Serialized against itself; the mux sequence runs under pmic_auxadc_lock
 * so no ch3/BATON/battery-exist sample can interleave (charging_hw_sm5414
 * forge_baton_lock=1 holds the same mutex). */
static void forge_temp_heal_run(const char *why)
{
	int pre3, post3, vbif_raw = 0, vbif_mv = -1;
	int count = 0, i;
	unsigned int rc;

	mutex_lock(&forge_temp_heal_mutex);
	pr_err("[FORGE_THEAL] heal start (%s): ch11_restore=%d tdet=%d g6=%d\n",
	       why, forge_ch11_restore, forge_temp_tdet, forge_temp_heal_g6);
	forge_temp_heal_regs("pre ");
	pre3 = PMIC_IMM_GetOneChannelValue(PMIC_AUX_BATON_AP, 5, 0);

	pmic_auxadc_lock();
	/* 1. convicted ch3 current injector OFF (boot write already removed;
	 *    idempotent insurance against any other setter) */
	pmic_set_register_value(PMIC_RG_ULC_DET_EN, 0);
	/* 2. stock-bracketed PAD_VBIF28 measure: the sequence the 3.10 kernel
	 *    runs at the PMIC_INIT_SETTING_V1 tail (pre-block, ch11
	 *    conversion, restore block), inlined because the driver-level
	 *    post-block only runs inside PMIC_IMM_GetOneChannelValue (which
	 *    takes the same mutex) and must stay bracketed with the pre-block
	 *    under one lock hold here. */
	pmic_set_register_value(PMIC_BATON_TDET_EN, 0);
	pmic_set_register_value(PMIC_CLK_CKPDN_CON2_CLR, 0x70);
	pmic_set_register_value(PMIC_RG_ADCIN_VSEN_MUX_EN, 1);
	mdelay(3);
	rc = pmic_config_interface_nolock(MT6353_AUXADC_RQST0_SET, (1 << 11),
					  0xffff, 0);
	udelay(50);
	while (pmic_get_register_value(PMIC_AUXADC_ADC_RDY_CH11) != 1) {
		usleep_range(1000, 1200);
		if ((count++) > 15) {
			pr_err("[FORGE_THEAL] ch11 conversion TIMEOUT\n");
			break;
		}
	}
	vbif_raw = pmic_get_register_value(PMIC_AUXADC_ADC_OUT_CH11);
	vbif_mv = (vbif_raw * 2 * 1800) / 4096;	/* MT6351 ch11 r_val = 2 */
	g_pmic_pad_vbif28_vol = vbif_mv;
	if ((g_pmic_pad_vbif28_vol < 2700) || (g_pmic_pad_vbif28_vol >= 2912))
		g_pmic_pad_vbif28_vol = 2770;	/* stock substitute value */
	/* 3. restore: mux off, VBIF28 back to stock hw-control mode, TDET to
	 *    its steady state. The stock CKPDN_CON2_SET 0x70 re-gate is
	 *    skipped (gating those bits bootloops this kernel). */
	pmic_set_register_value(PMIC_RG_ADCIN_VSEN_MUX_EN, 0);
	pmic_config_interface(MT6351_LDO_VBIF28_CON0, 1,
			      MT6351_PMIC_RG_VBIF28_ON_CTRL_MASK,
			      MT6351_PMIC_RG_VBIF28_ON_CTRL_SHIFT);
	pmic_config_interface(MT6351_LDO_VBIF28_CON0, 1,
			      MT6351_PMIC_RG_VBIF28_MODE_CTRL_MASK,
			      MT6351_PMIC_RG_VBIF28_MODE_CTRL_SHIFT);
	pmic_config_interface(MT6351_LDO_VBIF28_CON0, 0,
			      MT6351_PMIC_RG_VBIF28_EN_MASK,
			      MT6351_PMIC_RG_VBIF28_EN_SHIFT);
	mdelay(3);
	pmic_set_register_value(PMIC_BATON_TDET_EN, forge_temp_tdet ? 1 : 0);
	/* 4. optional: stock G6 AUXADC config at stock-like late timing */
	if (forge_temp_heal_g6) {
		for (i = 0; i < (int)ARRAY_SIZE(forge_temp_g6_tab); i++) {
			const struct forge_init351_ent *e = &forge_temp_g6_tab[i];

			pmic_config_interface(e->addr, e->val, e->mask,
					      e->shift);
		}
		pr_err("[FORGE_THEAL] G6 AUXADC block re-applied (%d writes)\n",
		       (int)ARRAY_SIZE(forge_temp_g6_tab));
	}
	pmic_auxadc_unlock();

	post3 = PMIC_IMM_GetOneChannelValue(PMIC_AUX_BATON_AP, 5, 0);
	forge_temp_heal_regs("post");
	pr_err("[FORGE_THEAL] heal done (%s): ch3 %d -> %d mV (good ~646, broken ~1550); vbif28 raw=0x%x mv=%d stored=%u rdy_polls=%d rqst_rc=%u\n",
	       why, pre3, post3, vbif_raw, vbif_mv, g_pmic_pad_vbif28_vol,
	       count, rc);
	mutex_unlock(&forge_temp_heal_mutex);
}

static void forge_temp_heal_workfn(struct work_struct *work)
{
	forge_temp_heal_run("auto");
}
static DECLARE_DELAYED_WORK(forge_temp_heal_work, forge_temp_heal_workfn);

static int forge_temp_heal_now;
static int forge_temp_heal_now_set(const char *sval,
				   const struct kernel_param *kp)
{
	int rc = param_set_int(sval, kp);

	if (rc)
		return rc;
	if (forge_temp_heal_now && forge_init351_boot_done)
		forge_temp_heal_run("manual");
	return 0;
}

static const struct kernel_param_ops forge_temp_heal_now_ops = {
	.set = forge_temp_heal_now_set,
	.get = param_get_int,
};
module_param_cb(forge_temp_heal_now, &forge_temp_heal_now_ops,
		&forge_temp_heal_now, 0644);

static int __init forge_init351_late(void)
{
	forge_init351_vsram_sw = forge_dvfs_rails_wanted();
	forge_init351_boot_done = true;
	pr_err("[FORGE_P351] ready: table=%d entries, knob forge_init351=%d md=%d heal=%d/%ds ch11_restore=%d vsram_sw=%d\n",
	       FORGE_INIT351_TOTAL, forge_init351, forge_init351_md,
	       forge_temp_heal, forge_temp_heal_s, forge_ch11_restore,
	       forge_init351_vsram_sw);
	if (forge_init351 > 0 || forge_init351_md)
		forge_init351_run();
	if (forge_temp_heal)
		schedule_delayed_work(&forge_temp_heal_work,
				      msecs_to_jiffies(forge_temp_heal_s *
						       1000));
	return 0;
}
late_initcall(forge_init351_late);
