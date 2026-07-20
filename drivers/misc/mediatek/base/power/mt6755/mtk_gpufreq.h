#ifndef _MTK_GPUFREQ_H
#define _MTK_GPUFREQ_H

/*
 * Extern declarations only (mt_gpufreq.h), as on MT6757: static inline
 * stubs here would redefine the real mt_gpufreq_* definitions and collide
 * with the weak fallbacks in the thermal code (mtk_thermal_platform.c,
 * mtk_cooler_atm.c) once CONFIG_THERMAL=y.
 */
#include "mt_gpufreq.h"

#endif
