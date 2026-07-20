/*
 * MT6755 compat header for the MT6757-era name. The shared 4.4 eccci core
 * includes <mach/mtk_pbm.h> unconditionally (modem_cldma.c,
 * modem_ccif_c2k.c); the MT6755 base ships the same content as mt_pbm.h
 * (same enum pbm_kicker, same init_md_section_level()/kicker_pbm_by_md()
 * prototypes). Forwarding keeps the shared eccci core unchanged.
 *
 * PBM itself is not built for MT6755 (pbm_v1/ is MT6757 only, mt_pbm.o is
 * commented out), so init_md_section_level() resolves to the no-op in
 * mt_misc_stubs.c.
 */
#ifndef __M681_MACH_MTK_PBM_COMPAT_H__
#define __M681_MACH_MTK_PBM_COMPAT_H__

#include <mach/mt_pbm.h>

#endif /* __M681_MACH_MTK_PBM_COMPAT_H__ */
