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
 *   AudDrv_Gpio.c
 *
 * Project:
 * --------
 *   MT6797  Audio Driver GPIO
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * George
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
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
/* raw EXTAMP pin driving (timed amplifier mode pulses) */
#include <mt-plat/mtk_gpio.h>
#include "mtk-auddrv-gpio.h"


#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/delay.h>

#if 1
struct pinctrl *pinctrlaud;

#define MT6755_PIN 1

enum audio_system_gpio_type {
	GPIO_AUD_CLK_MOSI_OFF,
	GPIO_AUD_CLK_MOSI_ON,
	GPIO_AUD_DAT_MISO_OFF,
	GPIO_AUD_DAT_MISO_ON,
	GPIO_AUD_DAT_MOSI_OFF,
	GPIO_AUD_DAT_MOSI_ON,
	GPIO_AUD_DAT_MISO2_OFF,
	GPIO_AUD_DAT_MISO2_ON,
	GPIO_AUD_DAT_MOSI2_OFF,
	GPIO_AUD_DAT_MOSI2_ON,
	GPIO_VOW_DAT_MISO_OFF,
	GPIO_VOW_DAT_MISO_ON,
	GPIO_VOW_CLK_MISO_OFF,
	GPIO_VOW_CLK_MISO_ON,
	GPIO_SMARTPA_RESET,
	GPIO_SMARTPA_ON,
	GPIO_TDM_MODE0,
	GPIO_TDM_MODE1,
#if MT6755_PIN
	/* GPIO_DEFAULT, */
	GPIO_PMIC_MODE0,
	GPIO_PMIC_MODE1,
	GPIO_I2S_MODE0,
	GPIO_I2S_MODE1,
	GPIO_EXTAMP_HIGH,
	GPIO_EXTAMP_LOW,
	GPIO_EXTAMP2_HIGH,
	GPIO_EXTAMP2_LOW,
	GPIO_RCVSPK_HIGH,
	GPIO_RCVSPK_LOW,
#endif
	GPIO_HPDEPOP_HIGH,
	GPIO_HPDEPOP_LOW,
	GPIO_AUD_CLK_MOSI_HIGH,
	GPIO_AUD_CLK_MOSI_LOW,
	GPIO_NUM
};

struct audio_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpioctrl;
};

static struct audio_gpio_attr aud_gpios[GPIO_NUM] = {
	[GPIO_AUD_CLK_MOSI_OFF] = {"aud_clk_mosi_off", false, NULL},
	[GPIO_AUD_CLK_MOSI_ON] = {"aud_clk_mosi_on", false, NULL},
	[GPIO_AUD_DAT_MISO_OFF] = {"aud_dat_miso_off", false, NULL},
	[GPIO_AUD_DAT_MISO_ON] = {"aud_dat_miso_on", false, NULL},
	[GPIO_AUD_DAT_MOSI_OFF] = {"aud_dat_mosi_off", false, NULL},
	[GPIO_AUD_DAT_MOSI_ON] = {"aud_dat_mosi_on", false, NULL},
	[GPIO_AUD_DAT_MISO2_OFF] = {"aud_dat_miso2_off", false, NULL},
	[GPIO_AUD_DAT_MISO2_ON] = {"aud_dat_miso2_on", false, NULL},
	[GPIO_AUD_DAT_MOSI2_OFF] = {"aud_dat_mosi2_off", false, NULL},
	[GPIO_AUD_DAT_MOSI2_ON] = {"aud_dat_mosi2_on", false, NULL},
	[GPIO_VOW_DAT_MISO_OFF] = {"vow_dat_miso_off", false, NULL},
	[GPIO_VOW_DAT_MISO_ON] = {"vow_dat_miso_on", false, NULL},
	[GPIO_VOW_CLK_MISO_OFF] = {"vow_clk_miso_off", false, NULL},
	[GPIO_VOW_CLK_MISO_ON] = {"vow_clk_miso_on", false, NULL},

	[GPIO_SMARTPA_RESET] = {"aud_smartpa_reset", false, NULL},
	[GPIO_SMARTPA_ON] = {"aud_smartpa_on", false, NULL},
	[GPIO_TDM_MODE0] = {"aud_tdm_mode0", false, NULL},
	[GPIO_TDM_MODE1] = {"aud_tdm_mode1", false, NULL},

#if MT6755_PIN
	/* [GPIO_DEFAULT] = {"default", false, NULL}, */
	[GPIO_PMIC_MODE0] = {"audpmicclk-mode0", false, NULL},
	[GPIO_PMIC_MODE1] = {"audpmicclk-mode1", false, NULL},
	[GPIO_I2S_MODE0] = {"audi2s1-mode0", false, NULL},
	[GPIO_I2S_MODE1] = {"audi2s1-mode1", false, NULL},
	[GPIO_EXTAMP_HIGH] = {"extamp-pullhigh", false, NULL},
	[GPIO_EXTAMP_LOW] = {"extamp-pulllow", false, NULL},
	[GPIO_EXTAMP2_HIGH] = {"extamp2-pullhigh", false, NULL},
	[GPIO_EXTAMP2_LOW] = {"extamp2-pulllow", false, NULL},
	[GPIO_RCVSPK_HIGH] = {"rcvspk-pullhigh", false, NULL},
	[GPIO_RCVSPK_LOW] = {"rcvspk-pulllow", false, NULL},
#endif

	[GPIO_HPDEPOP_HIGH] = {"hpdepop-pullhigh", false, NULL},
	[GPIO_HPDEPOP_LOW] = {"hpdepop-pulllow", false, NULL},
	[GPIO_AUD_CLK_MOSI_HIGH] = {"aud_clk_mosi_pull_high", false, NULL},
	[GPIO_AUD_CLK_MOSI_LOW] = {"aud_clk_mosi_pull_low", false, NULL},
};
#endif

static unsigned int extbuck_fan53526_exist;

static DEFINE_MUTEX(gpio_request_mutex);

static int forge_amp_extamp_pin;	/* defined with its module_param below */

void AudDrv_GPIO_probe(void *dev)
{
#if 1
	int ret;
	int i = 0;

	pr_warn("%s\n", __func__);

	/* The amp-enable pad is board-specific. The M681 DT has no
	 * "forge,extamp-gpio", so M681 keeps the module_param default below;
	 * the L681 DT sets 115 (GPIO_SPEAKER_EN_PIN of its DCT), since there
	 * GPIO100 is the BQ24196 CE pin and GPIO118 is muxed to F2W_CK. The
	 * sysfs knob still overrides at runtime. */
	{
		struct device_node *np = ((struct device *)dev)->of_node;
		u32 pin;

		if (np && !of_property_read_u32(np, "forge,extamp-gpio", &pin)) {
			pr_notice("[FORGE_AMP] extamp gpio from DT: %u (default was %d)\n",
				  pin, forge_amp_extamp_pin);
			forge_amp_extamp_pin = pin;
		}
	}

	pinctrlaud = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrlaud)) {
		ret = PTR_ERR(pinctrlaud);
		pr_err("Cannot find pinctrlaud!\n");
		return;
	}

	/* update hpdepop gpio by PCB version - extbuck fan53526 use gpio111 which may be used by hpdepop */
	pr_warn("%s(), extbuck_fan53526_exist = %d\n", __func__, extbuck_fan53526_exist);
	if (extbuck_fan53526_exist) { /* is e2 */
		struct audio_gpio_attr gpio_hpdepop_high = {"hpdepop-pullhigh_e2", false, NULL};
		struct audio_gpio_attr gpio_hpdepop_low = {"hpdepop-pulllow_e2", false, NULL};

		aud_gpios[GPIO_HPDEPOP_HIGH] = gpio_hpdepop_high;
		aud_gpios[GPIO_HPDEPOP_LOW] = gpio_hpdepop_low;

		pr_warn("%s(), e2 PCB, update gpio name, high = %s, low = %s\n",
			__func__,
			aud_gpios[GPIO_HPDEPOP_HIGH].name,
			aud_gpios[GPIO_HPDEPOP_LOW].name);
	}

	for (i = 0; i < ARRAY_SIZE(aud_gpios); i++) {
		aud_gpios[i].gpioctrl = pinctrl_lookup_state(pinctrlaud, aud_gpios[i].name);
		if (IS_ERR(aud_gpios[i].gpioctrl)) {
			ret = PTR_ERR(aud_gpios[i].gpioctrl);
			pr_err("%s pinctrl_lookup_state %s fail %d\n", __func__, aud_gpios[i].name,
			       ret);
		} else {
			aud_gpios[i].gpio_prepare = true;
		}
	}
	//[yuwenjiao] add start for smartpa I2S GPIO.
	pr_err("AudDrv_GPIO_SMARTPA_Select!\n");
	AudDrv_GPIO_SMARTPA_Select(true);
	//[yuwenjiao] add end

	/* Stock DCT boot init muxes the AP <-> MT6351 MTKIF audio pins
	 * GPIO149/150/151 to MODE_01 (AUD_CLK_MOSI / AUD_DAT_MOSI /
	 * AUD_DAT_MISO, cust_gpio_boot.h); this tree has no DCT init and the
	 * DT "default" state may not claim them. Force the mux here
	 * (idempotent if the DT state already applied) and log the modes
	 * before and after. */
	{
		static const unsigned int mtkif[3] = { 149, 150, 151 };
		unsigned int p, pre[3], post[3];

		for (p = 0; p < 3; p++) {
			pre[p] = mt_get_gpio_mode(mtkif[p] | 0x80000000);
			mt_set_gpio_mode(mtkif[p] | 0x80000000, GPIO_MODE_01);
			post[p] = mt_get_gpio_mode(mtkif[p] | 0x80000000);
		}
		pr_emerg("[FORGE_AUDIO] MTKIF mux 149/150/151 pre=%u/%u/%u post=%u/%u/%u (want 1/1/1)\n",
			 pre[0], pre[1], pre[2], post[0], post[1], post[2]);
	}
#endif
}

static int AudDrv_GPIO_Select(enum audio_system_gpio_type _type)
{
	int ret = 0;

	if (_type < 0 || _type >= GPIO_NUM) {
		pr_err("%s(), error, invaild gpio type %d\n", __func__, _type);
		return -EINVAL;
	}

	/* A pinctrl state that the DT does not define is a board fact (M681
	 * has no smart-PA pins): the M681 dl1 node only carries the
	 * default/extamp-pullhigh/extamp-pulllow states, so the aud_smartpa_*
	 * lookup fails and gpio_prepare stays false. Stock code then went
	 * through AUDIO_AEE into pinctrl_select_state(ERR_PTR) and crashed in
	 * probe (dl1 -> AudDrv_GPIO_probe -> SMARTPA_Select). Return -EIO
	 * instead; never AEE, never dereference the error pointer. */
	if (!aud_gpios[_type].gpio_prepare) {
		pr_err("%s(), gpio type %d (%s) not prepared - skip (no DTS state)\n",
		       __func__, _type, aud_gpios[_type].name);
		return -EIO;
	}

	if (IS_ERR_OR_NULL(pinctrlaud) ||
	    IS_ERR_OR_NULL(aud_gpios[_type].gpioctrl)) {
		pr_err("%s(), gpio type %d handle invalid - skip\n",
		       __func__, _type);
		return -EIO;
	}

	ret = pinctrl_select_state(pinctrlaud,
				   aud_gpios[_type].gpioctrl);
	if (ret)
		pr_err("%s(), error, can not set gpio type %d\n",
		       __func__, _type);

	return ret;
}

static int set_aud_clk_mosi(bool _enable)
{
/*
 * scp also need this gpio on mt6797,
 * don't switch gpio if they exist.
 */
#ifndef CONFIG_MTK_TINYSYS_SCP_SUPPORT
	static int aud_clk_mosi_counter;

	if (_enable) {
		aud_clk_mosi_counter++;
		if (aud_clk_mosi_counter == 1)
			return AudDrv_GPIO_Select(GPIO_AUD_CLK_MOSI_ON);
	} else {
		if (aud_clk_mosi_counter > 0) {
			aud_clk_mosi_counter--;
		} else {
			aud_clk_mosi_counter = 0;
			pr_warn("%s(), counter %d <= 0\n",
				__func__, aud_clk_mosi_counter);
		}

		if (aud_clk_mosi_counter == 0)
			return AudDrv_GPIO_Select(GPIO_AUD_CLK_MOSI_OFF);
	}
#endif
	return 0;
}

static int set_aud_dat_mosi(bool _enable)
{
	if (_enable)
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MOSI_ON);
	else
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MOSI_OFF);
}

static int set_aud_dat_miso(bool _enable, Soc_Aud_Digital_Block _usage)
{
	static bool adda_enable;
	static bool vow_enable;

	switch (_usage) {
	case Soc_Aud_Digital_Block_ADDA_UL:
		adda_enable = _enable;
		break;
	case Soc_Aud_Digital_Block_ADDA_VOW:
		vow_enable = _enable;
		break;
	default:
		return -EINVAL;
	}

	if (vow_enable)
		return AudDrv_GPIO_Select(GPIO_VOW_DAT_MISO_ON);
	else if (adda_enable)
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MISO_ON);
	else
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MISO_OFF);

}

static int set_aud_dat_mosi2(bool _enable)
{
	if (_enable)
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MOSI2_ON);
	else
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MOSI2_OFF);
}

static int set_aud_dat_miso2(bool _enable)
{
	if (_enable)
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MISO2_ON);
	else
		return AudDrv_GPIO_Select(GPIO_AUD_DAT_MISO2_OFF);
}

static int set_vow_clk_miso(bool _enable)
{
	if (_enable)
		return AudDrv_GPIO_Select(GPIO_VOW_CLK_MISO_ON);
	else
		return AudDrv_GPIO_Select(GPIO_VOW_CLK_MISO_OFF);
}

int AudDrv_GPIO_Request(bool _enable, Soc_Aud_Digital_Block _usage)
{
	mutex_lock(&gpio_request_mutex);
	switch (_usage) {
	case Soc_Aud_Digital_Block_ADDA_DL:
		set_aud_clk_mosi(_enable);
		set_aud_dat_mosi(_enable);
		break;
	case Soc_Aud_Digital_Block_ADDA_UL:
		set_aud_clk_mosi(_enable);
		set_aud_dat_miso(_enable, _usage);
		break;
	case Soc_Aud_Digital_Block_ADDA_VOW:
		set_vow_clk_miso(_enable);
		set_aud_dat_miso(_enable, _usage);
		break;
	case Soc_Aud_Digital_Block_ADDA_ANC:
		set_aud_clk_mosi(_enable);
		set_aud_dat_miso2(_enable);
		set_aud_dat_mosi2(_enable);
		break;
	case Soc_Aud_Digital_Block_ADDA_ALL:
		set_aud_clk_mosi(_enable);
		set_aud_dat_mosi(_enable);
		set_aud_dat_mosi2(_enable);
		set_aud_dat_miso(_enable, Soc_Aud_Digital_Block_ADDA_UL);
		set_aud_dat_miso2(_enable);
		break;
	default:
		mutex_unlock(&gpio_request_mutex);
		return -EINVAL;
	}
	mutex_unlock(&gpio_request_mutex);
	return 0;
}

int AudDrv_GPIO_SMARTPA_Select(int mode)
{
	int retval = 0;

#if 0
	switch (mode) {
	case 0:
		mt_set_gpio_mode(69 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(70 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(71 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(72 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(73 | 0x80000000, GPIO_MODE_00);
		break;
	case 1:
		mt_set_gpio_mode(69 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(70 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(71 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(72 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(73 | 0x80000000, GPIO_MODE_01);
		break;
	case 3:
		mt_set_gpio_mode(69 | 0x80000000, GPIO_MODE_03);
		mt_set_gpio_mode(70 | 0x80000000, GPIO_MODE_03);
		mt_set_gpio_mode(71 | 0x80000000, GPIO_MODE_03);
		mt_set_gpio_mode(72 | 0x80000000, GPIO_MODE_03);
		mt_set_gpio_mode(73 | 0x80000000, GPIO_MODE_03);
		break;
	default:
		pr_err("%s(), invalid mode = %d", __func__, mode);
		retval = -1;
	}
#else
	switch (mode) {
	case 0:
		AudDrv_GPIO_Select(GPIO_SMARTPA_RESET);
		break;
	case 1:
		AudDrv_GPIO_Select(GPIO_SMARTPA_ON);
		break;
	default:
		pr_err("%s(), invalid mode = %d", __func__, mode);
		retval = -1;
	}
#endif
	return retval;
}

int AudDrv_GPIO_TDM_Select(int mode)
{
	int retval = 0;

#if 0
	switch (mode) {
	case 0:
		mt_set_gpio_mode(135 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(136 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_mode(138 | 0x80000000, GPIO_MODE_00);
		break;
	case 1:
		mt_set_gpio_mode(135 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(136 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_mode(138 | 0x80000000, GPIO_MODE_01);
		break;
	default:
		pr_err("%s(), invalid mode = %d", __func__, mode);
		retval = -1;
}
#else
	switch (mode) {
	case 0:
		AudDrv_GPIO_Select(GPIO_TDM_MODE0);
		break;
	case 1:
		AudDrv_GPIO_Select(GPIO_TDM_MODE1);
		break;
	default:
		pr_err("%s(), invalid mode = %d", __func__, mode);
		retval = -1;
	}
#endif
	return retval;
}

int AudDrv_GPIO_PMIC_Select(int bEnable)
{
	int retval = 0;
#if MT6755_PIN
	if (bEnable == 1) {
		if (aud_gpios[GPIO_PMIC_MODE1].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_PMIC_MODE1].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_PMIC_MODE1] pins\n");
		}
	} else {
		if (aud_gpios[GPIO_PMIC_MODE0].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_PMIC_MODE0].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_PMIC_MODE0] pins\n");
		}

	}
#endif
	return retval;
}

int AudDrv_GPIO_I2S_Select(int bEnable)
{
	int retval = 0;

#if MT6755_PIN
	if (bEnable == 1) {
		if (aud_gpios[GPIO_I2S_MODE1].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_I2S_MODE1].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_I2S_MODE1] pins\n");
		}
	} else {
		if (aud_gpios[GPIO_I2S_MODE0].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_I2S_MODE0].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_I2S_MODE0] pins\n");
		}

	}
#endif
	return retval;
}

/* Runtime gate for the external speaker amp:
 *   echo 0 > /sys/module/mtk_auddrv_gpio/parameters/forge_audio_amp_gate
 * suppresses the amp enable writes (disable is still honoured, see the
 * fail-safe below). Default 1: the amp pin is GPIO100 (see
 * forge_amp_extamp_pin), not the SM5414 charger-enable pad GPIO115, so
 * driving it does not affect charging, and without it the loudspeaker is
 * silent. A floating PA input is covered separately by the
 * forge_amp_need_lo interlock. */
static int forge_audio_amp_gate = 1;
module_param(forge_audio_amp_gate, int, 0644);
MODULE_PARM_DESC(forge_audio_amp_gate,
		 "1 = drive ext speaker amp SHDN (default since the GPIO118 pin fix); 0 = suppress (bisect gate)");

/* Amplifier one-wire mode override. Stock 3.10 (CONFIG_MTK_LEGACY=y)
 * compiles the raw AW8736_MODEn macro branch (mt_soc_codec_63xx.c, spec
 * "0.75us<TL<10us; 0.75us<TH<10us", gap 2 us):
 *   mode N = number of rising edges on SHDN/CTRL, the train ends HIGH;
 *   MODE1 (1.2 W) = a single plain set-HIGH, no pulses.
 * Any train that ends with the pin HIGH enables the PA in some mode (the
 * modes only differ in output power, 0.8-2 W), so a hold-HIGH is always a
 * valid enable and a mis-counted train cannot leave the part off.
 *   0    = use the caller's mode;
 *   1..4 = force N rising edges (1 = hold-HIGH/no pulse = 1.2 W,
 *          2 = 1.0 W, 3 = 0.8 W, 4 = 1.5-2.0 W depending on THD). */
static int forge_amp_extamp_mode;
module_param(forge_amp_extamp_mode, int, 0644);
MODULE_PARM_DESC(forge_amp_extamp_mode,
		 "AW8736 rising-edge count: 0=caller(3), 1=hold-HIGH/no-pulse, 2..4=train");

/* Speaker amp enable pin and pulse timing.
 *
 * The amplifier is an AW8738 enabled by GPIO100. This matches the
 * shipping Flyme firmware: audio.primary.mt6755.so opens /dev/aw8738 and
 * issues ioctl(fd, 0x703, 4) for the loudspeaker path, and the kernel's
 * aw8738_set_mode(N) drives gpio 100 (label "aw8738_en", boot-init LOW)
 * with N low -> high pulses under preempt-disable, ending HIGH; N = 4 is
 * the loudspeaker mode, N = 5 (pin LOW) is off. With a stream playing and
 * the LO driver engaged, GPIO100 high makes the loudspeaker play and
 * GPIO100 low stops it.
 *
 * Not the amp: GPIO115 is the SM5414 charger-enable pad (stock DCT
 * GPIO_SM5414_CHGEN_PIN), and GPIO118 is the MTK-generic "extamp" pinctrl
 * binding of the shipping DTB, unclaimed on this board (driving it does
 * nothing). GPIO54 (extamp2) is the rework-board pin and must not be
 * driven here: on this board it is BPI_BUS12 (modem RF band interface).
 *
 * The mode-select pulses need 0.75-10 us phases. pinctrl_select_state()
 * cannot meet that (it takes the pinctrl mutex and may sleep, stretching
 * every phase far past 10 us, so the part latches an undefined mode), so
 * the pin is driven with raw legacy-API writes and IRQs off across the
 * train, as the vendor CONFIG_MTK_LEGACY branch did (mt_set_gpio_out +
 * udelay(2)).
 *
 * forge_amp_extamp_pin=118 or 115 is accepted for comparison only; 115
 * drives the charger enable.
 */
static int forge_amp_extamp_pin = 100;
module_param(forge_amp_extamp_pin, int, 0644);
MODULE_PARM_DESC(forge_amp_extamp_pin,
		 "m681 speaker amp enable GPIO: 100 = AW8738 aw8738_en, device-proven (default); 118 = generic extamp (silent); 115 = SM5414 CHGEN (wrong, charger)");

static inline unsigned int forge_extamp_gpio(void)
{
	int pin = forge_amp_extamp_pin;

	/* a bogus knob value must not become a write to a random pad */
	if (pin < 0 || pin > 202)
		pin = 100;
	return (unsigned int)pin | 0x80000000;
}

#define FORGE_M681_EXTAMP_GPIO forge_extamp_gpio()

/* VBAT sampler for the external amp: reads VBAT through the battery meter
 * every 100 ms after an engage, so a supply sag around the amp engage
 * shows up as a staircase in the last dmesg lines. The PA input is fed
 * from the codec LO (LOL -> amp); engaging the amp with the LO driver off
 * leaves its input floating (oscillation, maximum draw), see
 * forge_amp_need_lo. */
static struct delayed_work forge_amp_vbat_work;
static int forge_amp_vbat_n;
static bool forge_amp_vbat_inited;
static bool forge_extamp_engaged;	/* pin currently HIGH (amp on) */
static bool forge_amp_chgen_held;	/* we asked the charger to hold CHGEN */

/* Sampler window: 100 x 100 ms = 10 s per (re)arm. Re-armed at the dl1
 * feed edge (forge_amp_feed_mark), so the engage -> feed transition is
 * covered even if playback starts late. */
#define FORGE_AMP_VBAT_SAMPLES 100

/* 0 (default) = no charger interaction;
 * 1 = on ext-amp engage, ask the SM5414 charger to hold I2C CTRL.CHGEN=1
 *     (the charger sources the PA current instead of the battery) until
 *     disengage. See forge_sm5414_amp_chgen_hold() in charging_hw_sm5414.c
 *     for the pin/bit semantics and the FSM deferral. */
static int forge_amp_chgen_hold;
module_param(forge_amp_chgen_hold, int, 0644);
MODULE_PARM_DESC(forge_amp_chgen_hold,
		 "1 = hold SM5414 I2C CHGEN=1 while ext speaker amp engaged");

/* Strong definition lives in charging_hw_sm5414.c (CONFIG_MTK_SMART_BATTERY
 * builds). Weak stub so the audio stack links in charger-less configs. */
void __weak forge_sm5414_amp_chgen_hold(int hold)
{
	pr_err("[FORGE_AMP] no SM5414 charger in this build; CHGEN hold(%d) = no-op\n",
	       hold);
}

/* VBAT sampler gate, off by default. When on, every engage costs
 * FORGE_AMP_VBAT_SAMPLES (100) x 100 ms of
 * battery_meter_get_battery_voltage(1) (a fresh auxadc conversion over
 * pwrap, not a cached read), a battery current read, a current-sign read
 * and a pr_emerg, for 10 s, re-armed from the dl1 trigger on every
 * playback start. The vendor aw8738_set_mode() touches only the GPIO and
 * pinctrl (no rail, I2C, regulator or clock call). One write re-arms it
 * at runtime.
 */
static int forge_amp_vbat_probe;
module_param(forge_amp_vbat_probe, int, 0644);
MODULE_PARM_DESC(forge_amp_vbat_probe,
		 "1 = sample VBAT/ibat every 100ms for 10s after each ext-amp engage (default 0 = off; the sag theory it served is void)");

static void forge_amp_vbat_fn(struct work_struct *w)
{
	extern signed int battery_meter_get_battery_voltage(int update);
	extern signed int battery_meter_get_battery_current(void);
	extern unsigned int battery_meter_get_battery_current_sign(void);

	/* ibat in 0.1mA units (fuel-gauge CIC); dir=1 means CHARGING
	 * (current INTO the battery) per MTK sign convention. With the
	 * CHGEN hold active + VBUS present, "dir=1 while playing" is the
	 * positive proof the charger (not the battery) sources the PA. */
	/* pad=dout/din: the GPIO's electrical state per sample, to correlate
	 * the current direction with the pad level. */
	pr_err("[FORGE_AMP] t=+%dms vbat=%d ibat01mA=%d dir=%u pad=%d/%d\n",
	       forge_amp_vbat_n * 100,
	       battery_meter_get_battery_voltage(1),
	       battery_meter_get_battery_current(),
	       battery_meter_get_battery_current_sign(),
	       mt_get_gpio_out(FORGE_M681_EXTAMP_GPIO),
	       mt_get_gpio_in(FORGE_M681_EXTAMP_GPIO));
	if (++forge_amp_vbat_n < FORGE_AMP_VBAT_SAMPLES)
		schedule_delayed_work(&forge_amp_vbat_work,
				      msecs_to_jiffies(100));
}

/* Called from the dl1 trigger-START path: marks the exact feed edge (first
 * real signal into the engaged PA) in the same dmesg stream as the vbat
 * samples, and restarts the sampling window from t=0 so the sag (if any)
 * is guaranteed to be inside it. No-op while the amp is not engaged. */
void forge_amp_feed_mark(void)
{
	if (!forge_extamp_engaged)
		return;
	/* gated: see the forge_amp_vbat_probe block. Off by default, so the
	 * dl1 trigger path costs nothing on every playback start. */
	if (!forge_amp_vbat_probe)
		return;
	pr_err("[FORGE_AMP] feed-start (dl1 trigger) — vbat window re-armed\n");
	forge_amp_vbat_n = 0;
	schedule_delayed_work(&forge_amp_vbat_work, 0);
}

int AudDrv_GPIO_EXTAMP_Select(int bEnable, int mode)
{
	int retval = 0;

#if MT6755_PIN
	int extamp_mode;
	int i;
	unsigned long flags;

	if (!forge_audio_amp_gate) {
		/* Fail-safe: the disable path is never suppressed. Flipping
		 * the gate to 0 while the pin is HIGH would otherwise latch
		 * the amp on with no way to turn it off. Acts only if this
		 * boot engaged the pin; a boot with gate=0 never touches the
		 * pad. */
		if (bEnable == 0 && forge_extamp_engaged) {
			mt_set_gpio_out(FORGE_M681_EXTAMP_GPIO, GPIO_OUT_ZERO);
			forge_extamp_engaged = false;
			if (forge_amp_chgen_held) {
				forge_sm5414_amp_chgen_hold(0);
				forge_amp_chgen_held = false;
			}
			pr_err("[FORGE_AUDIO] amp gate=0 but pin engaged: forced LOW (amp off)\n");
			return 0;
		}
		pr_err("[FORGE_AUDIO] amp gate=0: EXTAMP_Select(%d,%d) suppressed\n",
		       bEnable, mode);
		return 0;
	}
	pr_emerg("[FORGE_AUDIO] EXTAMP_Select(en=%d, mode=%d) raw-GPIO%d (m681 SHDN from shipping DTB extamp state; GPIO115 is SM5414 CHGEN, not the amp)\n",
		 bEnable, mode, forge_amp_extamp_pin);

	if (bEnable == 1) {
		if (mode == 1)
			extamp_mode = 1;
		else if (mode == 2)
			extamp_mode = 2;
		else
			/* Mode 4 matches the vendor HAL's loudspeaker call
			 * (ioctl(fd, 0x703, 4) on /dev/aw8738). A wrong pulse
			 * count only changes the gain: a plain hold-HIGH is
			 * already audible. */
			extamp_mode = 4;

		/* Runtime override (see forge_amp_extamp_mode): 1..4 forces
		 * the rising-edge count; 1 = hold-HIGH/no pulse (plain
		 * set-HIGH as in the stock legacy macro). */
		if (forge_amp_extamp_mode >= 1 && forge_amp_extamp_mode <= 4)
			extamp_mode = forge_amp_extamp_mode;

		mt_set_gpio_mode(FORGE_M681_EXTAMP_GPIO, GPIO_MODE_GPIO);
		mt_set_gpio_dir(FORGE_M681_EXTAMP_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(FORGE_M681_EXTAMP_GPIO, GPIO_OUT_ZERO);
		/* caller (Ext_Speaker_Amp_Change) already held the pin low
		 * >=1ms via EXTAMP_Select(false) + usleep, so the AW8736
		 * mode counter is reset; just settle briefly. */
		udelay(10);
		local_irq_save(flags);
		for (i = 0; i < extamp_mode; i++) {
			if (i) {
				mt_set_gpio_out(FORGE_M681_EXTAMP_GPIO,
						GPIO_OUT_ZERO);
				udelay(2);
			}
			mt_set_gpio_out(FORGE_M681_EXTAMP_GPIO, GPIO_OUT_ONE);
			udelay(2);
		}
		local_irq_restore(flags);
		/* Pad readback: din reflects the real pad level in output
		 * mode only if IES=1 on this pad (check the mtgpio row of
		 * forge_amp_extamp_pin in /sys/devices/virtual/misc/mtgpio/pin);
		 * with IES=0, din is not a measurement. */
		pr_emerg("[FORGE_AUDIO] EXTAMP train done: pin=%d edges=%d pad mode=%d dir=%d dout=%d din=%d (din=1 => pad measured HIGH only if IES=1 on THIS pad)\n",
			 forge_amp_extamp_pin,
			 extamp_mode,
			 mt_get_gpio_mode(FORGE_M681_EXTAMP_GPIO),
			 mt_get_gpio_dir(FORGE_M681_EXTAMP_GPIO),
			 mt_get_gpio_out(FORGE_M681_EXTAMP_GPIO),
			 mt_get_gpio_in(FORGE_M681_EXTAMP_GPIO));
		/* The one-time sampler init must come before the engaged=true
		 * store: forge_amp_feed_mark() (dl1 trigger, atomic context)
		 * checks forge_extamp_engaged and schedules this work, so a
		 * racer must never see engaged=true with an uninitialized
		 * work. */
		if (!forge_amp_vbat_inited) {
			INIT_DELAYED_WORK(&forge_amp_vbat_work, forge_amp_vbat_fn);
			forge_amp_vbat_inited = true;
		}
		forge_extamp_engaged = true;
		/* Optional (forge_amp_chgen_hold): ask the charger to keep
		 * I2C CTRL.CHGEN=1 for the whole engage so VBUS current backs
		 * the PA peaks. Only useful while charging works; otherwise
		 * the FSM's under-temperature CHGEN=0 is what this hold
		 * overrides. */
		if (forge_amp_chgen_hold && !forge_amp_chgen_held) {
			forge_sm5414_amp_chgen_hold(1);
			forge_amp_chgen_held = true;
		}
		/* Arm the VBAT sampler (every 100 ms for 10 s, re-armed at the
		 * dl1 feed edge) if forge_amp_vbat_probe is set; the one-time
		 * init is done above, before the engaged=true store. */
		if (forge_amp_vbat_probe) {
			forge_amp_vbat_n = 0;
			schedule_delayed_work(&forge_amp_vbat_work, 0);
		}
	} else {
		mt_set_gpio_out(FORGE_M681_EXTAMP_GPIO, GPIO_OUT_ZERO);
		forge_extamp_engaged = false;
		/* release the CHGEN hold if WE took it (tracked locally so a
		 * knob flip mid-engage cannot leak the latch) */
		if (forge_amp_chgen_held) {
			forge_sm5414_amp_chgen_hold(0);
			forge_amp_chgen_held = false;
		}
		pr_emerg("[FORGE_AUDIO] EXTAMP pin=%d LOW (amp off) din=%d\n",
			 forge_amp_extamp_pin,
			 mt_get_gpio_in(FORGE_M681_EXTAMP_GPIO));
	}
#endif
	return retval;
}

int AudDrv_GPIO_EXTAMP2_Select(int bEnable, int mode)
{
	int retval = 0;

#if MT6755_PIN
	int extamp_mode;
	int i;

	if (!forge_audio_amp_gate) {
		pr_err("[FORGE_AUDIO] amp gate=0: EXTAMP2_Select(%d,%d) suppressed\n",
		       bEnable, mode);
		return 0;
	}

	if (bEnable == 1) {
		if (mode == 1)
			extamp_mode = 1;
		else if (mode == 2)
			extamp_mode = 2;
		else
			extamp_mode = 3;	/* default mode is 3 */

		if (aud_gpios[GPIO_EXTAMP2_HIGH].gpio_prepare) {
			for (i = 0; i < extamp_mode; i++) {
				retval = pinctrl_select_state(pinctrlaud,
						aud_gpios[GPIO_EXTAMP2_LOW].gpioctrl);
				if (retval)
					pr_err("could not set aud_gpios[GPIO_EXTAMP2_LOW] pins\n");
				udelay(2);
				retval = pinctrl_select_state(pinctrlaud,
						aud_gpios[GPIO_EXTAMP2_HIGH].gpioctrl);
				if (retval)
					pr_err("could not set aud_gpios[GPIO_EXTAMP2_HIGH] pins\n");
				udelay(2);
			}
		}
	} else {
		if (aud_gpios[GPIO_EXTAMP2_LOW].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_EXTAMP2_LOW].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_EXTAMP2_LOW] pins\n");
		}

	}
#endif
	return retval;
}

int AudDrv_GPIO_RCVSPK_Select(int bEnable)
{
	int retval = 0;

#if MT6755_PIN
	if (bEnable == 1) {
		if (aud_gpios[GPIO_RCVSPK_HIGH].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_RCVSPK_HIGH].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_RCVSPK_HIGH] pins\n");
		}
	} else {
		if (aud_gpios[GPIO_RCVSPK_LOW].gpio_prepare) {
			retval =
			    pinctrl_select_state(pinctrlaud, aud_gpios[GPIO_RCVSPK_LOW].gpioctrl);
			if (retval)
				pr_err("could not set aud_gpios[GPIO_RCVSPK_LOW] pins\n");
		}

	}
#endif
	return retval;
}

int AudDrv_GPIO_HPDEPOP_Select(int bEnable)
{
	int retval = 0;

	if (bEnable == 1)
		AudDrv_GPIO_Select(GPIO_HPDEPOP_LOW);
	else
		AudDrv_GPIO_Select(GPIO_HPDEPOP_HIGH);

	return retval;
}

int audio_drv_gpio_aud_clk_pull(bool high)
{
	int retval = 0;

	pr_debug("%s, high = %d\n", __func__, high);

	if (high == 1)
		AudDrv_GPIO_Select(GPIO_AUD_CLK_MOSI_HIGH);
	else
		AudDrv_GPIO_Select(GPIO_AUD_CLK_MOSI_LOW);

	return retval;
}

static int __init dt_get_extbuck_info(unsigned long node, const char *uname, int depth, void *data)
{
	struct devinfo_extbuck_tag {
		u32 size;
		u32 tag;
		u32 extbuck_fan53526_exist;
	} *tags;
	unsigned int size = 0;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	tags = (struct devinfo_extbuck_tag *) of_get_flat_dt_prop(node, "atag,extbuck_fan53526", &size);

	if (tags) {
		extbuck_fan53526_exist = tags->extbuck_fan53526_exist;
		pr_warn("[%s] fan53526_exist = %d\n", __func__, extbuck_fan53526_exist);
	}
	return 0;
}

static int __init audio_drv_gpio_init(void)
{
	of_scan_flat_dt(dt_get_extbuck_info, NULL);

	return 0;
}

arch_initcall(audio_drv_gpio_init);
