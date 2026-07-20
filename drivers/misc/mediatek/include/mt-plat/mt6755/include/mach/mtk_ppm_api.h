/*
 * mtk_-named header for the MT6757-era thermal code (ap_thermal_limit.c,
 * mtk_ts_cpu.c include <mach/mtk_ppm_api.h>). The MT6755 platform headers
 * keep the old mt_ names; mt_ppm_api.h declares the same API
 * (mt_ppm_cpu_thermal_protect, mt_ppm_thermal_get_max_power,
 * mt_ppm_set_5A_limit_throttle, ...).
 */
#ifndef __MTK_PPM_API_SHIM_H__
#define __MTK_PPM_API_SHIM_H__

#include "mach/mt_ppm_api.h"

#endif
