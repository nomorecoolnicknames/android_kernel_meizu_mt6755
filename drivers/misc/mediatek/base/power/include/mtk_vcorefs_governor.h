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

#ifndef _COMMON_MT_VCOREFS_GOVERNOR_H
#define _COMMON_MT_VCOREFS_GOVERNOR_H

/* MT6755 uses the MT6757 vcorefs governor declarations (KIR_*, OPPI_*
 * enums, used by msdc autok_dvfs); the governor itself is not built. */
#if defined(CONFIG_MACH_MT6757) || defined(CONFIG_MACH_KIBOPLUS) || defined(CONFIG_MACH_MT6755)

#include "vcorefs_v1/mtk_vcorefs_governor_mt6757.h"

#endif

#endif /* _COMMON_MT_VCOREFS_MANAGER_H */

