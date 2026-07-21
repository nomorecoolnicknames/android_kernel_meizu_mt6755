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
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/


/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Ana.c
 *
 * Project:
 * --------
 *   MT6797  Audio Driver ana Register setting
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-common.h"

#ifdef AUDIO_USING_WRAP_DRIVER
/*#include <mach/mt_pmic_wrap.h>*/
#include <mach/mtk_pmic_wrap.h>
#include <linux/module.h>	/* forge_ana_log param */
#endif


#ifdef AUDIO_USING_WRAP_DRIVER
static DEFINE_SPINLOCK(ana_set_reg_lock);
#endif

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/
uint32 Ana_Get_Reg(uint32 offset)
{
	/* get pmic register */
	uint32 Rdata = 0;
#ifdef AUDIO_USING_WRAP_DRIVER
	int ret = 0;

	ret = pwrap_read(offset, &Rdata);
	PRINTK_ANA_REG("Ana_Get_Reg offset=0x%x,Rdata=0x%x,ret=%d\n", offset, Rdata, ret);
#endif

	return Rdata;
}
EXPORT_SYMBOL(Ana_Get_Reg);

/* echo 1 > /sys/module/mtk_auddrv_ana/parameters/forge_ana_log logs every
 * PMIC audio write (addr/val/mask) at pr_err, so with the kernel log
 * streamed off the device the last line before a failure names the write.
 * Default 0 = silent. */
static int forge_ana_log;
module_param(forge_ana_log, int, 0644);
MODULE_PARM_DESC(forge_ana_log, "1 = log every Ana_Set_Reg pwrap write");

/* Kernel-side dump of the PMIC codec state through this driver's pwrap
 * path (Ana_Get_Reg); userspace pmic_access readback is not reliable (it
 * has read a known nonzero register as 0). Trigger at any time:
 *   echo 1 > /sys/module/mtk_auddrv_ana/parameters/forge_ana_dump
 * Dumps the clock, codec-digital, monitor and analog registers to dmesg
 * at pr_err. Repeatable (each write is a fresh dump); pwrap reads only. */
static int forge_ana_dump_set(const char *val, const struct kernel_param *kp)
{
	static const uint32 regs[] = {
		0x0234,	/* TOP_STATUS */
		0x023A,	/* TOP_CKPDN_CON0 (bits 15:12 = aud clk pdn) */
		0x029A,	/* TOP_CLKSQ (bit0 = 26M squarer en) */
		0x0230,	/* DRV_CON2 (PMIC MISO pad driving) */
		0x2000,	/* AFE_UL_DL_CON0 */
		0x2002,	/* AFE_DL_SRC2_CON0_H */
		0x2004,	/* AFE_DL_SRC2_CON0_L (bit0 = DL on) */
		0x2006,	/* AFE_DL_SDM_CON0 */
		0x2008,	/* AFE_DL_SDM_CON1 (attenuation) */
		0x200A,	/* AFE_UL_SRC_CON0_H (UL rate) */
		0x200C,	/* AFE_UL_SRC_CON0_L (bit0 UL on, bit2 loop-from-DL) */
		0x2012,	/* PMIC_AFE_TOP_CON0 (bit0 DL-from-sgen, bit1 adc) */
		0x2014,	/* AFE_AUDIO_TOP_CON0 */
		0x201C,	/* AFUNC_AUD_CON0 (scrambler) */
		0x2020,	/* AFUNC_AUD_CON2 (sdm fifo) */
		0x2038,	/* AFE_PMIC_NEWIF_CFG0 (rxif rate/format) */
		0x203A,	/* AFE_PMIC_NEWIF_CFG1 */
		0x203C,	/* AFE_PMIC_NEWIF_CFG2 (bit15 rxif sck inverse) */
		0x203E,	/* AFE_PMIC_NEWIF_CFG3 */
		0x2016,	/* AFE_DL_SRC_MON0 (DL data monitor) */
		0x2026,	/* AFUNC_AUD_MON0 */
		0x2028,	/* AFUNC_AUD_MON1 */
		0x202A,	/* AUDRC_TUNE_MON0 */
		0x202E,	/* AFE_UP8X_FIFO_LOG_MON0 (UL fifo monitor) */
		0x2030,	/* AFE_UP8X_FIFO_LOG_MON1 */
		0x0CF2,	/* AUDDEC_ANA_CON0 (DAC/HS) */
		0x0CF4,	/* AUDDEC_ANA_CON1 */
		0x0D04,	/* AUDDEC_ANA_CON9 (IBIST/AUD_CLK/GLB) */
		0x0D06,	/* AUDDEC_ANA_CON10 (NV reg) */
		0x0806,	/* ZCD_CON3 (HS gain) */
		/* Lineout (LOL) registers: CON3 = LOL enable/mux (0x4234 =
		 * LOL on, MUX=DAC), ZCD_CON1 = LOL/R gain (0x0F89 = 0 dB). */
		0x0CF8,	/* AUDDEC_ANA_CON3 (LOL enable/mux) */
		0x0802,	/* ZCD_CON1 (lineout L/R gain) */
		/* Audio supply rails (stock prints these in its audio debug
		 * set). The init table has no 0x0Axx entries and
		 * forge_pmic_skip_regulator=1, so an EN bit that is off means
		 * the codec analog is unpowered at the rail level. */
		0x0A00,	/* LDO_VA18_CON0 */
		0x0A02,	/* LDO_VA18_CON1 */
		0x0A16,	/* LDO_VUSB33_CON0 */
	};
	/* AP-side AFE MMIO half: /dev/mem does not exist in booted Android,
	 * so this knob covers both buses. Read through the AFE driver's own
	 * accessor (Afe_Get_Reg, relative offsets, AFE_BASE=0L on CONFIG_OF).
	 * AFE MMIO with the audio bus unclocked wedges the AXI bus, so it is
	 * only read while Aud_AFE_Clk_cntr > 0 (forge_audio_hold keeps it
	 * >= 1 after InitAfeControl, and any active stream adds refs). */
	static const unsigned int afe_regs[] = {
		0x0010,	/* AFE_DAC_CON0 (bit0 AFE_ON, bit1 DL1_ON) */
		0x0014,	/* AFE_DAC_CON1 (rates) */
		0x0024,	/* AFE_CONN1 (DL1->DAC interconnect) */
		0x0028,	/* AFE_CONN2 */
		0x0034,	/* AFE_I2S_CON1 (I2S DAC out) */
		0x0108,	/* AFE_ADDA_DL_SRC2_CON0 (AP DL SRC) */
		0x010C,	/* AFE_ADDA_DL_SRC2_CON1 (AP DL gain) */
		0x0114,	/* AFE_ADDA_UL_SRC_CON0 (AP UL SRC en/rate) */
		0x0118,	/* AFE_ADDA_UL_SRC_CON1 */
		0x0120,	/* AFE_ADDA_TOP_CON0 (intf select) */
		0x0124,	/* AFE_ADDA_UL_DL_CON0 (ADDA intf, bit0) */
		0x0138,	/* AFE_ADDA_NEWIF_CFG0 (AP txif cfg) */
		0x013C,	/* AFE_ADDA_NEWIF_CFG1 */
		0x0040,	/* AFE_DL1_BASE */
		0x0044,	/* AFE_DL1_CUR  (MOVING between dumps = DL memif advancing) */
		0x0080,	/* AFE_VUL_BASE */
		0x008C,	/* AFE_VUL_CUR  (MOVING between dumps = UL memif advancing) */
		0x00D0,	/* AFE_MEMIF_MON0 */
	};
	extern uint32 Afe_Get_Reg(uint32 offset);
	extern int Aud_AFE_Clk_cntr;
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(regs); i++)
		pr_err("[FORGE_ANA] R 0x%04x = 0x%04x\n",
		       regs[i], Ana_Get_Reg(regs[i]));

	/* PMIC GPIO block window: the MT6351's own pin mux for its
	 * AUD_CLK/AUD_DAT MTKIF pads (GPIO_MODE3=0x60D0 field0 is the MISO
	 * mux; the VOW code restores it to mode 1 = AUD after VOW mode 2).
	 * Nothing here writes these and the init table stops at 0x0xxx; at
	 * the reset default (GPIO mode 0) the MTKIF link is dead in both
	 * directions. Window covers DIR/PULLEN/MODE0-4. */
	for (i = 0x60B0; i <= 0x60D8; i += 2)
		pr_err("[FORGE_ANA] G 0x%04x = 0x%04x\n",
		       i, Ana_Get_Reg(i));

	/* AUDENC window: the analog encoder block (preamp/PGA/ADC/micbias
	 * enables, AUDENC_ANA_CON0..16 at 0x0D08..0x0D28). Kept in the same
	 * one-shot dump as everything else, so it can be compared between a
	 * working capture config (e.g. in-call) and a failing one. */
	for (i = 0x0D08; i <= 0x0D28; i += 2)
		pr_err("[FORGE_ANA] E 0x%04x = 0x%04x\n",
		       i, Ana_Get_Reg(i));

	/* Codec rail voltage/enable (the regulator framework is off on 4.4)
	 * and the 4.4-only setLowPowerSetting targets, read through
	 * pmic_read_interface (pwrap read, no write). Fields per mt6351
	 * upmu_hw.h: VA18_ANA_CON0 0x0AFA RG_VA18_VOSEL [9:8] CAL [3:0];
	 * VUSB33_ANA_CON0 0x0AEA RG_VUSB33_CAL [3:0]; LDO_VA18_CON0 0x0A00 /
	 * LDO_VUSB33_CON0 0x0A16 RG_*_EN bit1, DA_QI_*_EN bit15;
	 * AUDDEC_ANA_CON8 0x0D02, AUDDEC_ANA_CON9 0x0D04. */
	{
		extern unsigned int pmic_read_interface(unsigned int RegNum,
			unsigned int *val, unsigned int MASK, unsigned int SHIFT);
		static const struct {
			unsigned int reg, mask, shift;
			const char *name;
		} pf[] = {
			{ 0x0AFA, 0xFFFF, 0, "VA18_ANA_CON0" },
			{ 0x0AEA, 0xFFFF, 0, "VUSB33_ANA_CON0" },
			{ 0x0D02, 0xFFFF, 0, "AUDDEC_ANA_CON8" },
			{ 0x0D04, 0xFFFF, 0, "AUDDEC_ANA_CON9" },
			{ 0x0AFA, 0x3, 8, "RG_VA18_VOSEL" },
			{ 0x0AFA, 0xF, 0, "RG_VA18_CAL" },
			{ 0x0A00, 0x1, 1, "RG_VA18_EN" },
			{ 0x0A00, 0x1, 15, "DA_QI_VA18_EN" },
			{ 0x0AEA, 0xF, 0, "RG_VUSB33_CAL" },
			{ 0x0A16, 0x1, 1, "RG_VUSB33_EN" },
			{ 0x0A16, 0x1, 15, "DA_QI_VUSB33_EN" },
		};
		unsigned int v, r;

		for (i = 0; i < (int)ARRAY_SIZE(pf); i++) {
			v = 0xdead;
			r = pmic_read_interface(pf[i].reg, &v, pf[i].mask,
						pf[i].shift);
			pr_err("[FORGE_ANA] P 0x%04x %s = 0x%x (ret=%u)\n",
			       pf[i].reg, pf[i].name, v, r);
		}
	}

	if (Aud_AFE_Clk_cntr > 0) {
		for (i = 0; i < (int)ARRAY_SIZE(afe_regs); i++)
			pr_err("[FORGE_ANA] A 0x%04x = 0x%08x\n",
			       afe_regs[i], Afe_Get_Reg(afe_regs[i]));
	} else {
		pr_err("[FORGE_ANA] A skipped: Aud_AFE_Clk_cntr=%d (unclocked AFE MMIO = wedge risk)\n",
		       Aud_AFE_Clk_cntr);
	}
	return 0;
}

static const struct kernel_param_ops forge_ana_dump_ops = {
	.set = forge_ana_dump_set,
};
module_param_cb(forge_ana_dump, &forge_ana_dump_ops, NULL, 0200);
MODULE_PARM_DESC(forge_ana_dump, "write 1 = dump PMIC audio regs to dmesg");

/* Sine generators to split the playback path
 * (memif data -> AP ADDA out -> MTKIF serial -> PMIC DL -> DAC -> ear):
 *   echo 1 = PMIC-side sinegen (AFE_SGEN_CFG0/1 = 0x0080/0x0101, the stock
 *            debug values) plus PMIC_AFE_TOP_CON0 (0x2012) bit0 = 1, the
 *            DL input mux (0 = MTKIF RX, 1 = sine table); tests
 *            PMIC DL -> SDM -> DAC -> amp -> ear only.
 *   echo 2 = AP-side sinegen at O03/O04 (AFE_SGEN_CON0 = 0x2c8c28c2, the
 *            platform table value); injects after memif, tests the AP
 *            output, the MTKIF link and the whole PMIC path.
 *   echo 0 = restore the saved register values.
 * Reading: 1 = tone -> PMIC and analog good, the break is MTKIF/AP output
 * (try 2); 1 = silent -> PMIC codec path dead; 2 = tone -> the whole
 * hardware path is good, the problem is on the memif data side.
 * The AP write is clock-guarded (Aud_AFE_Clk_cntr). */
static int forge_ana_sgen_state;
static uint32 forge_sgen_save[4]; /* PMIC CFG0, CFG1, AP SGEN_CON0, TOP_CON0 */

static int forge_ana_sgen_set(const char *val, const struct kernel_param *kp)
{
	extern void Afe_Set_Reg(uint32 offset, uint32 value, uint32 mask);
	extern uint32 Afe_Get_Reg(uint32 offset);
	extern int Aud_AFE_Clk_cntr;
	int n = 0;

	if (kstrtoint(val, 0, &n))
		return -EINVAL;

	if (n == 1) {
		forge_sgen_save[0] = Ana_Get_Reg(0x2040);
		forge_sgen_save[1] = Ana_Get_Reg(0x2042);
		forge_sgen_save[3] = Ana_Get_Reg(0x2012);
		Ana_Set_Reg(0x2040, 0x0080, 0xffff);
		Ana_Set_Reg(0x2042, 0x0101, 0xffff);
		Ana_Set_Reg(0x2012, 0x0001, 0x0001); /* DL-input mux -> sinegen */
		forge_ana_sgen_state = 1;
		pr_err("[FORGE_ANA] sgen PMIC ON + DL mux (saved 0x%04x/0x%04x/0x%04x)\n",
		       forge_sgen_save[0], forge_sgen_save[1],
		       forge_sgen_save[3]);
	} else if (n == 2) {
		if (Aud_AFE_Clk_cntr <= 0) {
			pr_err("[FORGE_ANA] sgen AP refused: clk cntr=%d\n",
			       Aud_AFE_Clk_cntr);
			return 0;
		}
		forge_sgen_save[2] = Afe_Get_Reg(0x01F0);
		Afe_Set_Reg(0x01F0, 0x2c8c28c2, 0xffffffff);
		forge_ana_sgen_state = 2;
		pr_err("[FORGE_ANA] sgen AP O03/O04 ON (saved 0x%08x)\n",
		       forge_sgen_save[2]);
	} else {
		if (forge_ana_sgen_state == 1) {
			Ana_Set_Reg(0x2012, forge_sgen_save[3], 0x0001);
			Ana_Set_Reg(0x2040, forge_sgen_save[0], 0xffff);
			Ana_Set_Reg(0x2042, forge_sgen_save[1], 0xffff);
		} else if (forge_ana_sgen_state == 2 && Aud_AFE_Clk_cntr > 0) {
			Afe_Set_Reg(0x01F0, forge_sgen_save[2], 0xffffffff);
		}
		pr_err("[FORGE_ANA] sgen OFF (state %d restored)\n",
		       forge_ana_sgen_state);
		forge_ana_sgen_state = 0;
	}
	return 0;
}

static const struct kernel_param_ops forge_ana_sgen_ops = {
	.set = forge_ana_sgen_set,
};
module_param_cb(forge_ana_sgen, &forge_ana_sgen_ops, NULL, 0200);
MODULE_PARM_DESC(forge_ana_sgen, "1=PMIC sinegen 2=AP sinegen@O03/O04 0=restore");

void Ana_Set_Reg(uint32 offset, uint32 value, uint32 mask)
{
	/* set pmic register or analog CONTROL_IFACE_PATH */

#ifdef AUDIO_USING_WRAP_DRIVER
	int ret = 0;
	uint32 Reg_Value;
	unsigned long flags = 0;

	if (forge_ana_log)
		pr_err("[FORGE_ANA] W 0x%x = 0x%x mask 0x%x\n",
		       offset, value, mask);
	PRINTK_ANA_REG("Ana_Set_Reg offset= 0x%x , value = 0x%x mask = 0x%x\n", offset, value,
		       mask);
	spin_lock_irqsave(&ana_set_reg_lock, flags);
	Reg_Value = Ana_Get_Reg(offset);
	Reg_Value &= (~mask);
	Reg_Value |= (value & mask);
	ret = pwrap_write(offset, Reg_Value);
	spin_unlock_irqrestore(&ana_set_reg_lock, flags);

	Reg_Value = Ana_Get_Reg(offset);
	if ((Reg_Value & mask) != (value & mask))
		pr_warn("Ana_Set_Reg  mask = 0x%x ret = %d Reg_Value = 0x%x\n", mask, ret,
			 Reg_Value);
#endif
}
EXPORT_SYMBOL(Ana_Set_Reg);

void Ana_Log_Print(void)
{
	pr_debug("AFE_UL_DL_CON0	= 0x%x\n", Ana_Get_Reg(AFE_UL_DL_CON0));
	pr_debug("AFE_DL_SRC2_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_H));
	pr_debug("AFE_DL_SRC2_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_L));
	pr_debug("AFE_DL_SDM_CON0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON0));
	pr_debug("AFE_DL_SDM_CON1  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON1));
	pr_debug("AFE_UL_SRC_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON0_H));
	pr_debug("AFE_UL_SRC_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON0_L));
	pr_debug("AFE_UL_SRC_CON1_H	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON1_H));
	pr_debug("AFE_UL_SRC_CON1_L	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON1_L));
	pr_debug("PMIC_AFE_TOP_CON0  = 0x%x\n", Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	pr_debug("AFE_AUDIO_TOP_CON0	= 0x%x\n", Ana_Get_Reg(AFE_AUDIO_TOP_CON0));
	pr_debug("PMIC_AFE_TOP_CON0  = 0x%x\n", Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	pr_debug("AFE_DL_SRC_MON0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SRC_MON0));
	pr_debug("AFE_DL_SDM_TEST0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_TEST0));
	pr_debug("AFE_MON_DEBUG0	= 0x%x\n", Ana_Get_Reg(AFE_MON_DEBUG0));
	pr_debug("AFUNC_AUD_CON0	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON0));
	pr_debug("AFUNC_AUD_CON1	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON1));
	pr_debug("AFUNC_AUD_CON2	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON2));
	pr_debug("AFUNC_AUD_CON3	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON3));
	pr_debug("AFUNC_AUD_CON4	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON4));
	pr_debug("AFUNC_AUD_MON0	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_MON0));
	pr_debug("AFUNC_AUD_MON1	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_MON1));
	pr_debug("AUDRC_TUNE_MON0  = 0x%x\n", Ana_Get_Reg(AUDRC_TUNE_MON0));
	pr_debug("AFE_UP8X_FIFO_CFG0	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_CFG0));
	pr_debug("AFE_UP8X_FIFO_LOG_MON0	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON0));
	pr_debug("AFE_UP8X_FIFO_LOG_MON1	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON1));
	pr_debug("AFE_DL_DC_COMP_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG0));
	pr_debug("AFE_DL_DC_COMP_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG1));
	pr_debug("AFE_DL_DC_COMP_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG2));
	pr_debug("AFE_PMIC_NEWIF_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0));
	pr_debug("AFE_PMIC_NEWIF_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1));
	pr_debug("AFE_PMIC_NEWIF_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2));
	pr_debug("AFE_PMIC_NEWIF_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3));
	pr_debug("AFE_SGEN_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG0));
	pr_debug("AFE_SGEN_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG1));
	pr_debug("AFE_ADDA2_PMIC_NEWIF_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG0));
	pr_debug("AFE_ADDA2_PMIC_NEWIF_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG1));
	pr_debug("AFE_ADDA2_PMIC_NEWIF_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG2));
	pr_debug("AFE_VOW_TOP  = 0x%x\n", Ana_Get_Reg(AFE_VOW_TOP));
	pr_debug("AFE_VOW_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG0));
	pr_debug("AFE_VOW_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG1));
	pr_debug("AFE_VOW_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG2));
	pr_debug("AFE_VOW_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG3));
	pr_debug("AFE_VOW_CFG4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG4));
	pr_debug("AFE_VOW_CFG5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG5));
	pr_debug("AFE_VOW_MON0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON0));
	pr_debug("AFE_VOW_MON1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON1));
	pr_debug("AFE_VOW_MON2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON2));
	pr_debug("AFE_VOW_MON3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON3));
	pr_debug("AFE_VOW_MON4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON4));
	pr_debug("AFE_VOW_MON5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON5));

	pr_debug("AFE_DCCLK_CFG0	= 0x%x\n", Ana_Get_Reg(AFE_DCCLK_CFG0));
	pr_debug("AFE_DCCLK_CFG1	= 0x%x\n", Ana_Get_Reg(AFE_DCCLK_CFG1));
	pr_debug("AFE_NCP_CFG0		= 0x%x\n", Ana_Get_Reg(AFE_NCP_CFG0));
	pr_debug("AFE_NCP_CFG1		= 0x%x\n", Ana_Get_Reg(AFE_NCP_CFG1));

	pr_debug("TOP_CON  = 0x%x\n", Ana_Get_Reg(TOP_CON));
	pr_debug("TOP_STATUS	= 0x%x\n", Ana_Get_Reg(TOP_STATUS));
	pr_debug("TOP_CKPDN_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON0));
	pr_debug("TOP_CKPDN_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON1));
	pr_debug("TOP_CKPDN_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON2));
	pr_debug("TOP_CKPDN_CON3	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON3));
	pr_debug("TOP_CKPDN_CON4	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON4));
	pr_debug("TOP_CKPDN_CON5	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON5));
	pr_debug("TOP_CKSEL_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON0));
	pr_debug("TOP_CKSEL_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON1));
	pr_debug("TOP_CKSEL_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON2));
	pr_debug("TOP_CKSEL_CON3	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON3));
	pr_debug("TOP_CKDIVSEL_CON0  = 0x%x\n", Ana_Get_Reg(TOP_CKDIVSEL_CON0));
	pr_debug("TOP_CKDIVSEL_CON1  = 0x%x\n", Ana_Get_Reg(TOP_CKDIVSEL_CON1));
	pr_debug("TOP_CKHWEN_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKHWEN_CON0));
	pr_debug("TOP_CKHWEN_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKHWEN_CON1));
	pr_debug("TOP_CKHWEN_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKHWEN_CON2));
	pr_debug("TOP_CKTST_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON0));
	pr_debug("TOP_CKTST_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON1));
	pr_debug("TOP_CKTST_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON2));
	pr_debug("TOP_CLKSQ  = 0x%x\n", Ana_Get_Reg(TOP_CLKSQ));
	pr_debug("TOP_CLKSQ_RTC  = 0x%x\n", Ana_Get_Reg(TOP_CLKSQ_RTC));
	pr_debug("TOP_CLK_TRIM  = 0x%x\n", Ana_Get_Reg(TOP_CLK_TRIM));
	pr_debug("TOP_RST_CON0  = 0x%x\n", Ana_Get_Reg(TOP_RST_CON0));
	pr_debug("TOP_RST_CON1  = 0x%x\n", Ana_Get_Reg(TOP_RST_CON1));
	pr_debug("TOP_RST_CON2  = 0x%x\n", Ana_Get_Reg(TOP_RST_CON2));
	pr_debug("TOP_RST_MISC  = 0x%x\n", Ana_Get_Reg(TOP_RST_MISC));
	pr_debug("TOP_RST_STATUS  = 0x%x\n", Ana_Get_Reg(TOP_RST_STATUS));
	pr_debug("TEST_CON0  = 0x%x\n", Ana_Get_Reg(TEST_CON0));
	pr_debug("TEST_CON1  = 0x%x\n", Ana_Get_Reg(TEST_CON1));
	pr_debug("TEST_OUT  = 0x%x\n", Ana_Get_Reg(TEST_OUT));
	pr_debug("AFE_MON_DEBUG0= 0x%x\n", Ana_Get_Reg(AFE_MON_DEBUG0));
	pr_debug("ZCD_CON0  = 0x%x\n", Ana_Get_Reg(ZCD_CON0));
	pr_debug("ZCD_CON1  = 0x%x\n", Ana_Get_Reg(ZCD_CON1));
	pr_debug("ZCD_CON2  = 0x%x\n", Ana_Get_Reg(ZCD_CON2));
	pr_debug("ZCD_CON3  = 0x%x\n", Ana_Get_Reg(ZCD_CON3));
	pr_debug("ZCD_CON4  = 0x%x\n", Ana_Get_Reg(ZCD_CON4));
	pr_debug("ZCD_CON5  = 0x%x\n", Ana_Get_Reg(ZCD_CON5));
	pr_debug("LDO_VA18_CON0  = 0x%x\n", Ana_Get_Reg(LDO_VA18_CON0));
	pr_debug("LDO_VA18_CON1  = 0x%x\n", Ana_Get_Reg(LDO_VA18_CON1));
	pr_debug("LDO_VUSB33_CON0  = 0x%x\n", Ana_Get_Reg(LDO_VUSB33_CON0));
	pr_debug("LDO_VUSB33_CON1  = 0x%x\n", Ana_Get_Reg(LDO_VUSB33_CON1));

	pr_debug("AUDDEC_ANA_CON0  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON0));
	pr_debug("AUDDEC_ANA_CON1  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON1));
	pr_debug("AUDDEC_ANA_CON2  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON2));
	pr_debug("AUDDEC_ANA_CON3  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON3));
	pr_debug("AUDDEC_ANA_CON4  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON4));
	pr_debug("AUDDEC_ANA_CON5  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON5));
	pr_debug("AUDDEC_ANA_CON6  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON6));
	pr_debug("AUDDEC_ANA_CON7  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON7));
	pr_debug("AUDDEC_ANA_CON8  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON8));
	pr_debug("AUDDEC_ANA_CON9  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON9));
	pr_debug("AUDDEC_ANA_CON10  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON10));

	pr_debug("AUDENC_ANA_CON0  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON0));
	pr_debug("AUDENC_ANA_CON1  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON1));
	pr_debug("AUDENC_ANA_CON2  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON2));
	pr_debug("AUDENC_ANA_CON3  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON3));
	pr_debug("AUDENC_ANA_CON4  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON4));
	pr_debug("AUDENC_ANA_CON5  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON5));
	pr_debug("AUDENC_ANA_CON6  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON6));
	pr_debug("AUDENC_ANA_CON7  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON7));
	pr_debug("AUDENC_ANA_CON8  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON8));
	pr_debug("AUDENC_ANA_CON9  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON9));
	pr_debug("AUDENC_ANA_CON10  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON10));
	pr_debug("AUDENC_ANA_CON11  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON11));
	pr_debug("AUDENC_ANA_CON12  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON12));
	pr_debug("AUDENC_ANA_CON13  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON13));
	pr_debug("AUDENC_ANA_CON14  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON14));
	pr_debug("AUDENC_ANA_CON15  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON15));
	pr_debug("AUDENC_ANA_CON16  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON16));

	pr_debug("AUDNCP_CLKDIV_CON0	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON0));
	pr_debug("AUDNCP_CLKDIV_CON1	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON1));
	pr_debug("AUDNCP_CLKDIV_CON2	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON2));
	pr_debug("AUDNCP_CLKDIV_CON3	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON3));
	pr_debug("AUDNCP_CLKDIV_CON4	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON4));

	pr_debug("TOP_CKPDN_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON0));
	pr_debug("GPIO_MODE3	= 0x%x\n", Ana_Get_Reg(GPIO_MODE3));

	pr_debug("DCXO_CW01	= 0x%x\n", Ana_Get_Reg(DCXO_CW01));

	pr_debug("-Ana_Log_Print\n");
}
EXPORT_SYMBOL(Ana_Log_Print);

int Ana_Debug_Read(char *buffer, const int size)
{
	int n = 0;

	n += scnprintf(buffer + n, size - n, "AFE_UL_DL_CON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_UL_DL_CON0));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SRC2_CON0_H  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SRC2_CON0_H));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SRC2_CON0_L  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SRC2_CON0_L));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SDM_CON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SDM_CON0));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SDM_CON1  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SDM_CON1));
	n += scnprintf(buffer + n, size - n, "AFE_UL_SRC_CON0_H  = 0x%x\n",
		       Ana_Get_Reg(AFE_UL_SRC_CON0_H));
	n += scnprintf(buffer + n, size - n, "AFE_UL_SRC_CON0_L  = 0x%x\n",
		       Ana_Get_Reg(AFE_UL_SRC_CON0_L));
	n += scnprintf(buffer + n, size - n, "AFE_UL_SRC_CON1_H  = 0x%x\n",
		       Ana_Get_Reg(AFE_UL_SRC_CON1_H));
	n += scnprintf(buffer + n, size - n, "AFE_UL_SRC_CON1_L  = 0x%x\n",
		       Ana_Get_Reg(AFE_UL_SRC_CON1_L));
	n += scnprintf(buffer + n, size - n, "PMIC_AFE_TOP_CON0  = 0x%x\n",
		       Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	n += scnprintf(buffer + n, size - n, "AFE_AUDIO_TOP_CON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_AUDIO_TOP_CON0));
	n += scnprintf(buffer + n, size - n, "PMIC_AFE_TOP_CON0  = 0x%x\n",
		       Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SRC_MON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SRC_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_DL_SDM_TEST0  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_SDM_TEST0));
	n += scnprintf(buffer + n, size - n, "AFE_MON_DEBUG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_MON_DEBUG0));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_CON0  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_CON0));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_CON1  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_CON1));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_CON2  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_CON2));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_CON3  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_CON3));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_CON4  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_CON4));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_MON0  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_MON0));
	n += scnprintf(buffer + n, size - n, "AFUNC_AUD_MON1  = 0x%x\n",
		       Ana_Get_Reg(AFUNC_AUD_MON1));
	n += scnprintf(buffer + n, size - n, "AUDRC_TUNE_MON0  = 0x%x\n",
		       Ana_Get_Reg(AUDRC_TUNE_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_UP8X_FIFO_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_UP8X_FIFO_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_UP8X_FIFO_LOG_MON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_UP8X_FIFO_LOG_MON1  = 0x%x\n",
		       Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON1));
	n += scnprintf(buffer + n, size - n, "AFE_DL_DC_COMP_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_DC_COMP_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_DL_DC_COMP_CFG1  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_DC_COMP_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_DL_DC_COMP_CFG2  = 0x%x\n",
		       Ana_Get_Reg(AFE_DL_DC_COMP_CFG2));
	n += scnprintf(buffer + n, size - n, "AFE_PMIC_NEWIF_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_PMIC_NEWIF_CFG1  = 0x%x\n",
		       Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_PMIC_NEWIF_CFG2  = 0x%x\n",
		       Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2));
	n += scnprintf(buffer + n, size - n, "AFE_PMIC_NEWIF_CFG3  = 0x%x\n",
		       Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3));
	n += scnprintf(buffer + n, size - n, "AFE_SGEN_CFG0  = 0x%x\n",
			Ana_Get_Reg(AFE_SGEN_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_SGEN_CFG1  = 0x%x\n",
			Ana_Get_Reg(AFE_SGEN_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_ADDA2_UP8X_FIFO_LOG_MON0  = 0x%x\n",
			Ana_Get_Reg(AFE_ADDA2_UP8X_FIFO_LOG_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_ADDA2_UP8X_FIFO_LOG_MON1  = 0x%x\n",
			Ana_Get_Reg(AFE_ADDA2_UP8X_FIFO_LOG_MON1));
	n += scnprintf(buffer + n, size - n, "AFE_ADDA2_PMIC_NEWIF_CFG0  = 0x%x\n",
			Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_ADDA2_PMIC_NEWIF_CFG1  = 0x%x\n",
			Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_ADDA2_PMIC_NEWIF_CFG2  = 0x%x\n",
			Ana_Get_Reg(AFE_ADDA2_PMIC_NEWIF_CFG2));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_TOP  = 0x%x\n", Ana_Get_Reg(AFE_VOW_TOP));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG2));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG3));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG4));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_CFG5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG5));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON1));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON2));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON3));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON4));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_MON5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON5));

	n += scnprintf(buffer + n, size - n, "AFE_VOW_POSDIV_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_POSDIV_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_TGEN_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_TGEN_CFG0));

	n += scnprintf(buffer + n, size - n, "BUCK_VOW_CON0 = 0x%x\n", Ana_Get_Reg(0x416));
	n += scnprintf(buffer + n, size - n, "BUCK_VOW_CON3 = 0x%x\n", Ana_Get_Reg(0x41C));
	n += scnprintf(buffer + n, size - n, "TOP_CKSEL_CON2 = 0x%x\n", Ana_Get_Reg(0x26A));
	n += scnprintf(buffer + n, size - n, "BUCK_VCORE_CON8 = 0x%x\n", Ana_Get_Reg(0x610));

	n += scnprintf(buffer + n, size - n, "AFE_DCCLK_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_DCCLK_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_DCCLK_CFG1  = 0x%x\n",
		       Ana_Get_Reg(AFE_DCCLK_CFG1));
	n += scnprintf(buffer + n, size - n, "AFE_HPANC_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_HPANC_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_NCP_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_NCP_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_NCP_CFG1  = 0x%x\n",
		       Ana_Get_Reg(AFE_NCP_CFG1));

	n += scnprintf(buffer + n, size - n, "TOP_CON  = 0x%x\n", Ana_Get_Reg(TOP_CON));
	n += scnprintf(buffer + n, size - n, "TOP_STATUS  = 0x%x\n", Ana_Get_Reg(TOP_STATUS));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON0  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON1  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON2  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON2));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON3  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON3));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON4  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON4));
	n += scnprintf(buffer + n, size - n, "TOP_CKPDN_CON5  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKPDN_CON5));
	n += scnprintf(buffer + n, size - n, "TOP_CKSEL_CON0  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKSEL_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_CKSEL_CON1  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKSEL_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_CKSEL_CON2  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKSEL_CON2));
	n += scnprintf(buffer + n, size - n, "TOP_CKSEL_CON3  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKSEL_CON3));
	n += scnprintf(buffer + n, size - n, "TOP_CKDIVSEL_CON0  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKDIVSEL_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_CKDIVSEL_CON1  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKDIVSEL_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_CKHWEN_CON0  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKHWEN_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_CKHWEN_CON1  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKHWEN_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_CKHWEN_CON2  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKHWEN_CON2));
	n += scnprintf(buffer + n, size - n, "TOP_CKTST_CON0  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKTST_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_CKTST_CON1  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKTST_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_CKTST_CON2  = 0x%x\n",
		       Ana_Get_Reg(TOP_CKTST_CON2));
	n += scnprintf(buffer + n, size - n, "TOP_CLKSQ  = 0x%x\n", Ana_Get_Reg(TOP_CLKSQ));
	n += scnprintf(buffer + n, size - n, "TOP_CLKSQ_RTC  = 0x%x\n",
			Ana_Get_Reg(TOP_CLKSQ_RTC));
	n += scnprintf(buffer + n, size - n, "TOP_CLK_TRIM  = 0x%x\n",
			Ana_Get_Reg(TOP_CLK_TRIM));
	n += scnprintf(buffer + n, size - n, "TOP_RST_CON0  = 0x%x\n",
			Ana_Get_Reg(TOP_RST_CON0));
	n += scnprintf(buffer + n, size - n, "TOP_RST_CON1  = 0x%x\n",
			Ana_Get_Reg(TOP_RST_CON1));
	n += scnprintf(buffer + n, size - n, "TOP_RST_CON2  = 0x%x\n",
			Ana_Get_Reg(TOP_RST_CON2));
	n += scnprintf(buffer + n, size - n, "TOP_RST_MISC  = 0x%x\n",
			Ana_Get_Reg(TOP_RST_MISC));
	n += scnprintf(buffer + n, size - n, "TOP_RST_STATUS  = 0x%x\n",
			Ana_Get_Reg(TOP_RST_STATUS));
	n += scnprintf(buffer + n, size - n, "TEST_CON0  = 0x%x\n", Ana_Get_Reg(TEST_CON0));
	n += scnprintf(buffer + n, size - n, "TEST_OUT  = 0x%x\n", Ana_Get_Reg(TEST_OUT));
	n += scnprintf(buffer + n, size - n, "AFE_MON_DEBUG0= 0x%x\n",
			Ana_Get_Reg(AFE_MON_DEBUG0));
	n += scnprintf(buffer + n, size - n, "ZCD_CON0  = 0x%x\n", Ana_Get_Reg(ZCD_CON0));
	n += scnprintf(buffer + n, size - n, "ZCD_CON1  = 0x%x\n", Ana_Get_Reg(ZCD_CON1));
	n += scnprintf(buffer + n, size - n, "ZCD_CON2  = 0x%x\n", Ana_Get_Reg(ZCD_CON2));
	n += scnprintf(buffer + n, size - n, "ZCD_CON3  = 0x%x\n", Ana_Get_Reg(ZCD_CON3));
	n += scnprintf(buffer + n, size - n, "ZCD_CON4  = 0x%x\n", Ana_Get_Reg(ZCD_CON4));
	n += scnprintf(buffer + n, size - n, "ZCD_CON5  = 0x%x\n", Ana_Get_Reg(ZCD_CON5));
	n += scnprintf(buffer + n, size - n, "LDO_VA18_CON0  = 0x%x\n",
			Ana_Get_Reg(LDO_VA18_CON0));
	n += scnprintf(buffer + n, size - n, "LDO_VA18_CON1  = 0x%x\n",
			Ana_Get_Reg(LDO_VA18_CON1));
	n += scnprintf(buffer + n, size - n, "LDO_VUSB33_CON0  = 0x%x\n",
		       Ana_Get_Reg(LDO_VUSB33_CON0));
	n += scnprintf(buffer + n, size - n, "LDO_VUSB33_CON1  = 0x%x\n",
		       Ana_Get_Reg(LDO_VUSB33_CON1));

	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON0  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON0));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON1  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON1));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON2  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON2));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON3  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON3));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON4  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON4));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON5  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON5));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON6  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON6));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON7  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON7));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON8  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON8));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON9  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON9));
	n += scnprintf(buffer + n, size - n, "AUDDEC_ANA_CON10  = 0x%x\n",
		       Ana_Get_Reg(AUDDEC_ANA_CON10));

	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON0  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON0));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON1  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON1));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON2  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON2));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON3  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON3));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON4  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON4));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON5  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON5));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON6  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON6));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON7  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON7));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON8  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON8));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON9  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON9));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON10  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON10));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON11  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON11));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON12  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON12));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON13  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON13));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON14  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON14));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON15  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON15));
	n += scnprintf(buffer + n, size - n, "AUDENC_ANA_CON16  = 0x%x\n",
		       Ana_Get_Reg(AUDENC_ANA_CON16));

	n += scnprintf(buffer + n, size - n, "AUDNCP_CLKDIV_CON0  = 0x%x\n",
		       Ana_Get_Reg(AUDNCP_CLKDIV_CON0));
	n += scnprintf(buffer + n, size - n, "AUDNCP_CLKDIV_CON1  = 0x%x\n",
		       Ana_Get_Reg(AUDNCP_CLKDIV_CON1));
	n += scnprintf(buffer + n, size - n, "AUDNCP_CLKDIV_CON2  = 0x%x\n",
		       Ana_Get_Reg(AUDNCP_CLKDIV_CON2));
	n += scnprintf(buffer + n, size - n, "AUDNCP_CLKDIV_CON3  = 0x%x\n",
		       Ana_Get_Reg(AUDNCP_CLKDIV_CON3));
	n += scnprintf(buffer + n, size - n, "AUDNCP_CLKDIV_CON4  = 0x%x\n",
		       Ana_Get_Reg(AUDNCP_CLKDIV_CON4));
	n += scnprintf(buffer + n, size - n, "GPIO_MODE3  = 0x%x\n",
		       Ana_Get_Reg(GPIO_MODE3));
	n += scnprintf(buffer + n, size - n, "DRV_CON2  = 0x%x\n",
		       Ana_Get_Reg(DRV_CON2));
	n += scnprintf(buffer + n, size - n, "AUDRC_TUNE_MON0  = 0x%x\n",
		       Ana_Get_Reg(AUDRC_TUNE_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG0  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG2  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG2));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG4  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG4));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG6  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG6));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG7  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG7));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG8  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG8));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG9  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG9));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG10  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG10));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG11  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG11));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_CFG12  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_CFG12));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_MON0  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_MON0));
	n += scnprintf(buffer + n, size - n, "AFE_VOW_PERIODIC_MON1  = 0x%x\n",
		       Ana_Get_Reg(AFE_VOW_PERIODIC_MON1));
	n += scnprintf(buffer + n, size - n, "DCXO_CW01  = 0x%x\n",
		       Ana_Get_Reg(DCXO_CW01));
	return n;
}
EXPORT_SYMBOL(Ana_Debug_Read);

/* export symbols for other module using */
