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

obj-y := cameraisp_dummy.o

ifeq ($(CONFIG_MACH_MT6757),y)
  obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif

# The upstream Makefile only built the MT6757 ISP driver; on MT6755 only
# cameraisp_dummy.o was built, so there was no /dev/camera-isp and the MtkCam
# HAL found no cameras. mt6755/ is the MT6755 3.18 driver; its DT contract
# matches the ispsys@15004000 node.
ifeq ($(CONFIG_MACH_MT6755),y)
  obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
#this moment do not have other folder
#obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
