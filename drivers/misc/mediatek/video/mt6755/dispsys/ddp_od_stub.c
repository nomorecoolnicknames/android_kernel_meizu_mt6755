/*
 * ddp_od_stub.c — zero-stub for the OD (overdrive) module.
 *
 * od10/ is not compiled for mt6755 (DISP0_SMI_LARB5 / M4U_PORT_DISP_OD_*
 * enum values are mt6757-only).  ddp_info.c and ddp_debug.c reference the
 * two symbols below; a zero-initialised driver struct and a no-op test
 * function satisfy the linker without pulling in any platform-specific code.
 */
#include "ddp_info.h"

struct DDP_MODULE_DRIVER ddp_driver_od;
EXPORT_SYMBOL(ddp_driver_od);

void od_test(const char *cmd, char *debug_output)
{
	(void)cmd;
	(void)debug_output;
}
EXPORT_SYMBOL(od_test);
