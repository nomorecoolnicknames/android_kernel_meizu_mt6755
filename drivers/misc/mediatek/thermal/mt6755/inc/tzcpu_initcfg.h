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

#ifndef __TZCPU_INITCFG_H__
#define __TZCPU_INITCFG_H__

#define TZCPU_SET_INIT_CFG										(1)

#define TZCPU_INITCFG_INTERVAL									(40)
#define TZCPU_INITCFG_NUM_TRIPS									(4)

/* Trip policy for unknown boards (shutdown only, no DVFS coupling); M681
 * and L681 replace this table with their stock policy (117 C on the same
 * cooler, mtk_ts_cpu.c).
 * - trip0 105 C -> mtk-cl-kshutdown00, the only acting trip: margin under
 *   the stock 117 C Tj shutdown because the SW poll is the only protection
 *   while RGU thermal-direct stays disarmed (mtk_tc.c
 *   forge_therm_hwprot=0). The cooler powers the phone off
 *   (mtk_cooler_kshutdown.forge_therm_enforce = 1 by default, 0 = log
 *   only).
 * - stock trip0 was "mtktscpu-sysrst": a no-op on LineageOS (it only
 *   SIGIOs a Meizu thermald that never registers a pid).
 * - trips 1-3 keep stock temperatures as unbound markers ("" never matches
 *   a cdev). */
#define TZCPU_INITCFG_TRIP_0_TEMP								(105000)
#define TZCPU_INITCFG_TRIP_0_COOLER								"mtk-cl-kshutdown00"

#define TZCPU_INITCFG_TRIP_1_TEMP								(95000)
#define TZCPU_INITCFG_TRIP_1_COOLER								""

#define TZCPU_INITCFG_TRIP_2_TEMP								(85000)
#define TZCPU_INITCFG_TRIP_2_COOLER								""

#define TZCPU_INITCFG_TRIP_3_TEMP								(65000)
#define TZCPU_INITCFG_TRIP_3_COOLER								""

#define TZCPU_INITCFG_TRIP_4_TEMP								(63000)
#define TZCPU_INITCFG_TRIP_4_COOLER								""

#define TZCPU_INITCFG_TRIP_5_TEMP								(60000)
#define TZCPU_INITCFG_TRIP_5_COOLER								""

#define TZCPU_INITCFG_TRIP_6_TEMP								(55000)
#define TZCPU_INITCFG_TRIP_6_COOLER								""

#define TZCPU_INITCFG_TRIP_7_TEMP								(50000)
#define TZCPU_INITCFG_TRIP_7_COOLER								""

#define TZCPU_INITCFG_TRIP_8_TEMP								(45000)
#define TZCPU_INITCFG_TRIP_8_COOLER								""

#define TZCPU_INITCFG_TRIP_9_TEMP								(40000)
#define TZCPU_INITCFG_TRIP_9_COOLER								""


#endif	/* __TZCPU_INITCFG_H__ */


