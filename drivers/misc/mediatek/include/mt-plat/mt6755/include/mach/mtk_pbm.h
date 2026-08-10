/*
 * mt6755 compat header for the mt6757 header name.
 *
 * The shared eccci core includes <mach/mtk_pbm.h> unconditionally
 * (modem_cldma.c, modem_ccif_c2k.c); that is the newer MTK naming, and the
 * header only exists under mt-plat/mt6757/. The mt6755 base ships the same
 * content under the older name mt_pbm.h (same enum pbm_kicker, same
 * init_md_section_level()/kicker_pbm_by_md() prototypes). Forwarding here
 * keeps the shared eccci core unforked.
 *
 * The PBM implementation is not built (pbm_v1/ is MACH_MT6757-only and
 * mt6755's mt_pbm.o is not built), so init_md_section_level() resolves to
 * the no-op in mt_misc_stubs.c and the modem does not take part in power
 * budgeting.
 */
#ifndef __M681_MACH_MTK_PBM_COMPAT_H__
#define __M681_MACH_MTK_PBM_COMPAT_H__

#include <mach/mt_pbm.h>

#endif /* __M681_MACH_MTK_PBM_COMPAT_H__ */
