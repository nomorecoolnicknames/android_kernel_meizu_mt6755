/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MTK_PMIC_INFO_MT6351_H_
#define _MTK_PMIC_INFO_MT6351_H_

/*
 * MT6351 chip CID codes (stock 3.10 mach/pmic.h PMIC6351_E{1,2,3}_CID_CODE).
 * PMIC_INIT_SETTING_V1() reads and logs PMIC_SWCID; a mismatch is not
 * fatal, so these only feed the "6351 PMIC Chip = 0x%x" log.
 */
#define PMIC6351_E1_CID_CODE    0x5110
#define PMIC6351_E2_CID_CODE    0x5120
#define PMIC6351_E3_CID_CODE    0x5130

/* Debug log level (defined in pmic.c) */
#define PMICTAG                "[PMIC] "
extern unsigned int gPMICDbgLvl;

#define PMIC_LOG_DBG     4
#define PMIC_LOG_INFO    3
#define PMIC_LOG_NOT     2
#define PMIC_LOG_WARN    1
#define PMIC_LOG_ERR     0

#define PMICLOG(fmt, arg...) do { \
	if (gPMICDbgLvl >= PMIC_LOG_DBG) \
		pr_err(PMICTAG "%s: " fmt, __func__, ##arg); \
} while (0)

#endif /* _MTK_PMIC_INFO_MT6351_H_ */
