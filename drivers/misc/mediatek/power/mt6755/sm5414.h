/*
 * SM5414 switching charger register map and interface (M681).
 *
 * Ported from the stock 3.10 sm5414.h. Register map, value tables and init
 * policy are verbatim; the function prototypes use plain C types (this 4.4
 * tree only defines kal_bool in mt-plat/charging.h).
 *
 * Hardware (stock DCT wt6755_66_sz_l):
 *   i2c bus 1, 7-bit addr 0x49 (0x92 write / 0x93 read)
 *   nSHDN  = GPIO4   (drive 1 = chip active)
 *   CHGEN  = GPIO115 (drive 0 = charging enabled, 1 = disabled)
 *   nINT   = GPIO101 / EINT101, falling edge (not wired on 4.4)
 */

#ifndef _SM5414_SW_H_
#define _SM5414_SW_H_

#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define SM5414_TOPOFF_TIMER_SUPPORT

#define SM5414_INT1		0x00
#define SM5414_INT2		0x01
#define SM5414_INT3		0x02
#define SM5414_INTMASK1		0x03
#define SM5414_INTMASK2		0x04
#define SM5414_INTMASK3		0x05
#define SM5414_STATUS		0x06
#define SM5414_CTRL		0x07
#define SM5414_VBUSCTRL		0x08
#define SM5414_CHGCTRL1		0x09
#define SM5414_CHGCTRL2		0x0A
#define SM5414_CHGCTRL3		0x0B
#define SM5414_CHGCTRL4		0x0C
#define SM5414_CHGCTRL5		0x0D

#define SM5414_REG_NUM 0x0E

/**********************************************************
  *
  *   [MASK/SHIFT]
  *
  *********************************************************/
/* INT1 */
#define SM5414_INT1_THEMREG	0x01
#define SM5414_INT1_THEMSHDN	0x02
#define SM5414_INT1_BATOVP	0x04
#define SM5414_INT1_VBUSLIMIT	0x08
#define SM5414_INT1_AICL	0x10
#define SM5414_INT1_VBUSINOK	0x20
#define SM5414_INT1_VBUSUVLO	0x40
#define SM5414_INT1_VBUSOVP	0x80
#define SM5414_INT1_MASK	0xFF
#define SM5414_INT1_SHIFT	0
/* INT2 */
#define SM5414_INT2_TOPOFF	0x01
#define SM5414_INT2_DONE	0x02
#define SM5414_INT2_CHGRSTF	0x04
#define SM5414_INT2_PRETMROFF	0x08
#define SM5414_INT2_OTGFAIL	0x10
#define SM5414_INT2_WEAKBAT	0x20
#define SM5414_INT2_NOBAT	0x40
#define SM5414_INT2_FASTTMROFF	0x80
#define SM5414_INT2_MASK	0xFF
#define SM5414_INT2_SHIFT	0
/* INT3 */
#define SM5414_INT3_DISLIMIT	0x01
#define SM5414_INT3_VSYSOLP	0x02
#define SM5414_INT3_VSYSNG	0x04
#define SM5414_INT3_VSYSOK	0x08
#define SM5414_INT3_MASK	0x0F
#define SM5414_INT3_SHIFT	0
/* INTMSK1 */
#define SM5414_INTMSK1_THEMREGM		0x01
#define SM5414_INTMSK1_THEMSHDNM	0x02
#define SM5414_INTMSK1_BATOVPM		0x04
#define SM5414_INTMSK1_VBUSLIMITM	0x08
#define SM5414_INTMSK1_AICLM		0x10
#define SM5414_INTMSK1_VBUSINOKM	0x20
#define SM5414_INTMSK1_VBUSUVLOM	0x40
#define SM5414_INTMSK1_VBUSOVPM		0x80
/* INTMSK2 */
#define SM5414_INTMSK2_TOPOFFM		0x01
#define SM5414_INTMSK2_DONEM		0x02
#define SM5414_INTMSK2_CHGRSTFM		0x04
#define SM5414_INTMSK2_PRETMROFFM	0x08
#define SM5414_INTMSK2_OTGFAILM		0x10
#define SM5414_INTMSK2_WEAKBATM		0x20
#define SM5414_INTMSK2_NOBATM		0x40
#define SM5414_INTMSK2_FASTTMROFFM	0x80
/* INTMSK3 */
#define SM5414_INTMSK3_DISLIMITM	0x01
#define SM5414_INTMSK3_VSYSOLPM		0x02
#define SM5414_INTMSK3_VSYSNGM		0x04
#define SM5414_INTMSK3_VSYSOKM		0x08
/* STATUS */
#define SM5414_STATUS_VSYSOLP_MASK	0x1
#define SM5414_STATUS_VSYSOLP_SHIFT	4
#define SM5414_STATUS_DISLIMIT_MASK	0x1
#define SM5414_STATUS_DISLIMIT_SHIFT	3
#define SM5414_STATUS_THEMSHDN_MASK	0x1
#define SM5414_STATUS_THEMSHDN_SHIFT	2
#define SM5414_STATUS_BATDET_MASK	0x1
#define SM5414_STATUS_BATDET_SHIFT	1
#define SM5414_STATUS_SUSPEND_MASK	0x1
#define SM5414_STATUS_SUSPEND_SHIFT	0
/* CTRL */
#define SM5414_CTRL_ENCOMPARATOR_MASK	0x1
#define SM5414_CTRL_ENCOMPARATOR_SHIFT	6
#define SM5414_CTRL_RESET_MASK		0x1
#define SM5414_CTRL_RESET_SHIFT		3
#define SM5414_CTRL_SUSPEN_MASK		0x1
#define SM5414_CTRL_SUSPEN_SHIFT	2
#define SM5414_CTRL_CHGEN_MASK		0x1
#define SM5414_CTRL_CHGEN_SHIFT		1
#define SM5414_CTRL_ENBOOST_MASK	0x1
#define SM5414_CTRL_ENBOOST_SHIFT	0
/* VBUSCTRL */
#define SM5414_VBUSCTRL_VBUSLIMIT_MASK	0x3F
#define SM5414_VBUSCTRL_VBUSLIMIT_SHIFT	0
/* CHGCTRL1 */
#define SM5414_CHGCTRL1_AICLTH_MASK	0x7
#define SM5414_CHGCTRL1_AICLTH_SHIFT	4
#define SM5414_CHGCTRL1_AUTOSTOP_MASK	0x1
#define SM5414_CHGCTRL1_AUTOSTOP_SHIFT	3
#define SM5414_CHGCTRL1_AICLEN_MASK	0x1
#define SM5414_CHGCTRL1_AICLEN_SHIFT	2
#define SM5414_CHGCTRL1_PRECHG_MASK	0x3
#define SM5414_CHGCTRL1_PRECHG_SHIFT	0
/* CHGCTRL2 */
#define SM5414_CHGCTRL2_FASTCHG_MASK	0x3F
#define SM5414_CHGCTRL2_FASTCHG_SHIFT	0
/* CHGCTRL3 */
#define SM5414_CHGCTRL3_BATREG_MASK	0xF
#define SM5414_CHGCTRL3_BATREG_SHIFT	4
#define SM5414_CHGCTRL3_WEAKBAT_MASK	0xF
#define SM5414_CHGCTRL3_WEAKBAT_SHIFT	0
/* CHGCTRL4 */
#define SM5414_CHGCTRL4_TOPOFF_MASK	0xF
#define SM5414_CHGCTRL4_TOPOFF_SHIFT	3
#define SM5414_CHGCTRL4_DISLIMIT_MASK	0x7
#define SM5414_CHGCTRL4_DISLIMIT_SHIFT	0
/* CHGCTRL5 */
#define SM5414_CHGCTRL5_VOTG_MASK	0x3
#define SM5414_CHGCTRL5_VOTG_SHIFT	4
#define SM5414_CHGCTRL5_FASTTIMER_MASK	0x3
#define SM5414_CHGCTRL5_FASTTIMER_SHIFT	2
#define SM5414_CHGCTRL5_TOPOFFTIMER_MASK	0x3
#define SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT	0

/* FAST charge current: 100mA + 50mA/step */
#define FASTCHG_100mA	0
#define FASTCHG_500mA	8
#define FASTCHG_1000mA	18
#define FASTCHG_1500mA	28
#define FASTCHG_2000mA	38
#define FASTCHG_2500mA	48

/* Input current limit: 100mA + 50mA/step */
#define VBUSLIMIT_100mA		0
#define VBUSLIMIT_500mA		8
#define VBUSLIMIT_1000mA	18
#define VBUSLIMIT_1500mA	28
#define VBUSLIMIT_2000mA	38
#define VBUSLIMIT_2050mA	39

/* AICL TH */
#define AICL_THRESHOLD_4_3_V	0
#define AICL_THRESHOLD_4_4_V	1
#define AICL_THRESHOLD_4_5_V	2
#define AICL_THRESHOLD_4_6_V	3
#define AICL_THRESHOLD_4_7_V	4
#define AICL_THRESHOLD_4_8_V	5
#define AICL_THRESHOLD_4_9_V	6
#define AICL_THRESHOLD_MASK	0x0F

/* AUTOSTOP */
#define AUTOSTOP_EN	(1)
#define AUTOSTOP_DIS	(0)
/* AICLEN */
#define AICL_EN		(1)
#define AICL_DIS	(0)
/* PRECHG */
#define PRECHG_150mA	0
#define PRECHG_250mA	1
#define PRECHG_350mA	2
#define PRECHG_450mA	3
#define PRECHG_MASK	0xFC

/* Battery regulation voltage: 4.100V + 25mV/step */
#define BATREG_4_1_0_0_V	0
#define BATREG_4_2_0_0_V	4
#define BATREG_4_3_0_0_V	8
#define BATREG_4_3_5_0_V	10
#define BATREG_4_4_0_0_V	12
#define BATREG_4_4_7_5_V	15
#define BATREG_MASK		0x0F

/* Weak battery voltage: 3.00V + 50mV/step */
#define WEAKBAT_3_0_0_V		0
#define WEAKBAT_3_6_0_V		12
#define WEAKBAT_3_7_5_V		15
#define WEAKBAT_MASK		0xF0

/* Top-off charge current: 100mA + 50mA/step */
#define TOPOFF_100mA	0
#define TOPOFF_150mA	1
#define TOPOFF_200mA	2
#define TOPOFF_650mA	11
#define TOPOFF_MASK	0x07

/* Discharge current limit */
#define DISCHARGELIMIT_DISABLED	0
#define DISCHARGELIMIT_2_0_A	1
#define DISCHARGELIMIT_5_0_A	7
#define DISCHARGELIMIT_MASK	0xF8

/* OTG voltage */
#define VOTG_5_0_V	0
#define VOTG_5_1_V	1
#define VOTG_5_2_V	2
#define VOTG_MASK	0x0F

/* Fast timer */
#define FASTTIMER_3_5_HOUR	0
#define FASTTIMER_4_5_HOUR	1
#define FASTTIMER_5_5_HOUR	2
#define FASTTIMER_DISABLED	3
#define FASTTIMER_MASK		0xF3

/* Topoff timer */
#define TOPOFFTIMER_10MIN	0
#define TOPOFFTIMER_20MIN	1
#define TOPOFFTIMER_30MIN	2
#define TOPOFFTIMER_45MIN	3
#define TOPOFFTIMER_MASK	0xFC

/* Enable charger */
#define CHARGE_EN	1
#define CHARGE_DIS	0
/* Enable OTG boost */
#define ENBOOST_EN	1
#define ENBOOST_DIS	0
/* Enable comparator */
#define ENCOMPARATOR_EN		1
#define ENCOMPARATOR_DIS	0
/* Suspend */
#define SUSPEND_EN	1
#define SUSPEND_DIS	0

/**********************************************************
  *
  *   [Extern Function]
  *
  *********************************************************/
extern void sm5414_set_enboost(unsigned int val);
extern void sm5414_set_chgen(unsigned int val);
extern void sm5414_set_suspen(unsigned int val);
extern void sm5414_set_reset(unsigned int val);
extern void sm5414_set_encomparator(unsigned int val);
extern void sm5414_set_vbuslimit(unsigned int val);
extern void sm5414_set_prechg(unsigned int val);
extern void sm5414_set_aiclen(unsigned int val);
extern void sm5414_set_autostop(unsigned int val);
extern void sm5414_set_aiclth(unsigned int val);
extern void sm5414_set_fastchg(unsigned int val);
extern void sm5414_set_weakbat(unsigned int val);
extern void sm5414_set_batreg(unsigned int val);
extern void sm5414_set_dislimit(unsigned int val);
extern void sm5414_set_topoff(unsigned int val);
extern void sm5414_set_topofftimer(unsigned int val);
extern void sm5414_set_fasttimer(unsigned int val);
extern void sm5414_set_votg(unsigned int val);

/*---------------------------------------------------------*/
extern void sm5414_reg_init(void);
extern void sm5414_dump_register(void);
extern int sm5414_get_chrg_stat(void);
extern int sm5414_poll_charge_done(void);
extern int is_sm5414_exist(void);
extern unsigned int sm5414_reg_config_interface(unsigned char RegNum,
						unsigned char val);
extern unsigned int sm5414_read_interface(unsigned char RegNum,
					  unsigned char *val,
					  unsigned char MASK,
					  unsigned char SHIFT);
extern unsigned int sm5414_config_interface(unsigned char RegNum,
					    unsigned char val,
					    unsigned char MASK,
					    unsigned char SHIFT);
#endif /* _SM5414_SW_H_ */
