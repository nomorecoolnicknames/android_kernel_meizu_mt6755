#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# common driver code for CMDQ
ifeq ($(CONFIG_MTK_CMDQ_TAB), y)
  obj-y += v2/
else
	# phone
	ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6735" "mt6580"))
		obj-y += v2/
	else ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6755"))
		# MT6755 CMDQ v2 core + mt6755 platform; its event table and DISP
		# scenarios differ from the mt6735 v2 core.
		obj-y += v2_mt6755/
	else
		obj-y += v3/
	endif
endif
# mdp_sw_sync
obj-y += mdp_sync/
# EOF
