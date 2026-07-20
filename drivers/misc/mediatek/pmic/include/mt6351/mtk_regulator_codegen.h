/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MTK_REGULATOR_CODEGEN_MT6351_H_
#define _MTK_REGULATOR_CODEGEN_MT6351_H_

/*
 * MT6351 regulator tables consumed by the common regulator framework
 * (common/upmu_regulator.c).  The actual definitions live in
 * mt6351/v1/pmic_regulator.c.  Unlike the MT6355 codegen header, MT6351 uses a
 * single shared mtk_regulator_ops (no per-rail generated ops), so only the
 * table externs are declared here.
 */
extern struct mtk_regulator mt_ldos[], mt_bucks[];
extern struct of_regulator_match pmic_regulator_ldo_matches[], pmic_regulator_buck_matches[];
extern int mt_ldos_size, mt_bucks_size;
extern int pmic_regulator_ldo_matches_size, pmic_regulator_buck_matches_size;

#endif /* _MTK_REGULATOR_CODEGEN_MT6351_H_ */
