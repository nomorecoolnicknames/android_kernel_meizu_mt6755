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

#ifndef __MTK_CHARGER_INIT_H__
#define __MTK_CHARGER_INIT_H__

#define BATTERY_CV 4350000
#ifndef V_CHARGER_MAX	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define V_CHARGER_MAX 6500000				/* 6.5 V */
#endif

#ifndef USB_CHARGER_CURRENT_SUSPEND	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define USB_CHARGER_CURRENT_SUSPEND			0		/* def CONFIG_USB_IF */
#endif
#ifndef USB_CHARGER_CURRENT_UNCONFIGURED	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define USB_CHARGER_CURRENT_UNCONFIGURED	70000	/* 70mA */
#endif
#ifndef USB_CHARGER_CURRENT_CONFIGURED	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define USB_CHARGER_CURRENT_CONFIGURED		500000	/* 500mA */
#endif
#ifndef USB_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define USB_CHARGER_CURRENT					500000	/* 500mA */
#endif
#ifndef AC_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define AC_CHARGER_CURRENT					2050000
#endif
#ifndef AC_CHARGER_INPUT_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define AC_CHARGER_INPUT_CURRENT			3200000
#endif
#ifndef NON_STD_AC_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define NON_STD_AC_CHARGER_CURRENT			500000
#endif
#ifndef CHARGING_HOST_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define CHARGING_HOST_CHARGER_CURRENT		650000
#endif
#ifndef APPLE_1_0A_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define APPLE_1_0A_CHARGER_CURRENT		650000
#endif
#ifndef APPLE_2_1A_CHARGER_CURRENT	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define APPLE_2_1A_CHARGER_CURRENT		800000
#endif
#define TA_AC_CHARGING_CURRENT	3000000

/* sw jeita */
#define JEITA_TEMP_ABOVE_T4_CV_VOLTAGE	4240000
#define JEITA_TEMP_T3_TO_T4_CV_VOLTAGE	4240000
#define JEITA_TEMP_T2_TO_T3_CV_VOLTAGE	4340000
#define JEITA_TEMP_T1_TO_T2_CV_VOLTAGE	4240000
#define JEITA_TEMP_T0_TO_T1_CV_VOLTAGE	4040000
#define JEITA_TEMP_BELOW_T0_CV_VOLTAGE	4040000
#define TEMP_T4_THRESHOLD  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRESHOLD  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRESHOLD  10
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRESHOLD  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRESHOLD  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRESHOLD 0

/* Battery Temperature Protection */
#ifndef MIN_CHARGE_TEMPERATURE	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define MIN_CHARGE_TEMPERATURE  0
#endif
#ifndef MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE	6
#endif
#ifndef MAX_CHARGE_TEMPERATURE	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define MAX_CHARGE_TEMPERATURE  50
#endif
#ifndef MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE	/* mach/mtk_charging.h (old-framework cust header) may define first */
#define MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE	47
#endif

/* [lidebiao start] Modify best efficiency for PEP1.1 */
/* pe */
#define PE_ICHG_LEAVE_THRESHOLD 1000 /* mA */
#define TA_AC_12V_INPUT_CURRENT 3200000
#define TA_AC_9V_INPUT_CURRENT	2000000
#define TA_AC_7V_INPUT_CURRENT	2500000
#define TA_9V_SUPPORT
//#define TA_12V_SUPPORT
/* [lidebiao end] */

/* pe2.0 */
#define PE20_ICHG_LEAVE_THRESHOLD 1000 /* mA */
#define TA_START_BATTERY_SOC	0
#define TA_STOP_BATTERY_SOC	85

/* dual charger */
#define TA_AC_MASTER_CHARGING_CURRENT 1500000
#define TA_AC_SLAVE_CHARGING_CURRENT 1500000


/* cable measurement impedance */
#define CABLE_IMP_THRESHOLD 699
#define VBAT_CABLE_IMP_THRESHOLD 3900

/* bif */
#define BIF_THRESHOLD1 4250000	/* UV */
#define BIF_THRESHOLD2 4300000	/* UV */
#define BIF_CV_UNDER_THRESHOLD2 4450000	/* UV */
#define BIF_CV BATTERY_CV /* UV */

/* battery warning */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
#define BATTERY_NOTIFY_CASE_0002_VBATTEMP

#endif /*__MTK_CHARGER_INIT_H__*/
