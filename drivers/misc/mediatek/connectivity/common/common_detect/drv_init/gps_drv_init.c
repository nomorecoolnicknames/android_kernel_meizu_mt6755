/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG         "[GPS-MOD-INIT]"

#include "wmt_detect.h"
#include "gps_drv_init.h"

int do_gps_drv_init(int chip_id)
{
	int i_ret = -1;
#ifdef CONFIG_MTK_COMBO_GPS
	WMT_DETECT_INFO_FUNC("start to do gps driver init\n");
	i_ret = mtk_wcn_stpgps_drv_init();
	WMT_DETECT_INFO_FUNC("finish gps driver init, i_ret:%d\n", i_ret);
#else
	/* "Not configured" is not a failure: do_connectivity_driver_init()
	 * sums its sub-results, so returning -1 here made DO_MODULE_INIT fail
	 * even when BT and WLAN succeeded. wmt_loader then skips its HIF setup
	 * (it calls configure_soc_sdio_hif() only on 0), and every WMT func_on
	 * fails with "WMT-CORE: no hif info!". The __weak stubs in
	 * conn_drv_init.c return 0 for "not supported" as well.
	 */
	WMT_DETECT_INFO_FUNC("CONFIG_MTK_COMBO_GPS is not defined\n");
	i_ret = 0;
#endif
	return i_ret;

}
