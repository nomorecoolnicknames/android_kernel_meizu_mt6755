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

#include <linux/types.h>
#include <linux/printk.h>
#include <linux/m3note_board.h>
#include "hacc_tee.h"


int masp_hal_get_uuid(unsigned int *uuid)
{
	uuid[0] = get_devinfo_with_index(12);
	uuid[1] = get_devinfo_with_index(13);
	uuid[2] = get_devinfo_with_index(12);
	uuid[3] = get_devinfo_with_index(13);

	return 0;
}

/*
 * Stock reads the secure-boot word at stock devinfo index 6; the v1 LK
 * atag shifts 4.4 by +1, so that is kernel index 7 (on L681 and M681).
 * Only this answer (ioctl /dev/sec) changes.
 */
int masp_hal_sbc_enabled(void)
{
	static bool printed;
	u32 word = get_devinfo_with_index(6);

	if ((m3note_board_known_is(M3NOTE_BOARD_L681) ||
	     m3note_board_known_is(M3NOTE_BOARD_M681)) &&
	    get_devinfo_with_index(0) == 0x41000804) {
		u32 stock = get_devinfo_with_index(6 + 1);

		if (!printed) {
			printed = true;
			pr_info("[MASP] m3note efuse sbc: old kernel idx 6 (stock 5) = 0x%x (sbc=%u) -> stock idx 6 = kernel idx 7 = 0x%x (sbc=%u)\n",
				word, (word & 0x2) ? 1 : 0, stock, (stock & 0x2) ? 1 : 0);
		}
		word = stock;
	}
	return (word & 0x00000002) ? 1 : 0;
}
