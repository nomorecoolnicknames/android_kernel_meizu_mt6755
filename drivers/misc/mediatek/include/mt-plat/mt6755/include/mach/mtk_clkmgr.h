/*
 * mtk_-named header for the MT6757-era thermal code (mtk_ts_cpu.c,
 * mtk_cooler_atm.c include <mach/mtk_clkmgr.h>). The MT6757 version is
 * empty (guard only) and the thermal consumers use no clkmgr symbols.
 * Does not include mach/mt_clkmgr.h (distinct guard; _MT_CLKMGR_H is taken
 * by mt_clkmgr.h).
 */
#ifndef __MTK_CLKMGR_SHIM_H__
#define __MTK_CLKMGR_SHIM_H__

#endif
