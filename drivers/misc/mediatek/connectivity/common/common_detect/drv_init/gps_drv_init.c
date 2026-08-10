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
	/* "Not built" is not a failure: do_connectivity_driver_init() sums the
	 * sub-driver results, so -1 would fail COMBO_IOCTL_DO_MODULE_INIT with BT
	 * and WLAN fine, the combo loader would skip configure_soc_sdio_hif(), and
	 * WMT would be left with no HIF. */
	WMT_DETECT_INFO_FUNC("CONFIG_MTK_COMBO_GPS is not defined\n");
	i_ret = 0;
#endif
	return i_ret;

}
