#ifndef __MTK_DRAMC_H__
#define __MTK_DRAMC_H__

#include "mt_dramc.h"

#define PDEF_SPM_AP_SEMAPHORE IOMEM((SLEEP_BASE_ADDR + 0x0484))

/* defined in mt_dramc.c; declared here for the performance/boost_ctrl
 * dram_ctrl.c consumer (the mt6735 header in this tree declares it too) */
extern int get_ddr_type(void);

#endif
