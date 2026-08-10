#ifndef _MTK_GPUFREQ_H
#define _MTK_GPUFREQ_H

/*
 * No static-inline stubs of the mt_gpufreq_* API here: they would redefine
 * the real global definitions and collide with the weak fallbacks the
 * thermal code carries (e.g. mtk_thermal_platform.c, mtk_cooler_atm.c)
 * once CONFIG_THERMAL=y compiles them. Follow the mt6757 pattern instead:
 * extern declarations only (mt_gpufreq.h).
 */
#include "mt_gpufreq.h"

#endif
