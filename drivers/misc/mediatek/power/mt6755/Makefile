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

# GM20 charger/fuel-gauge stack. This directory is only entered when
# CONFIG_MTK_SMART_BATTERY=y (see drivers/misc/mediatek/Makefile).

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(subst ",,$(CONFIG_MTK_PLATFORM))/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include

ifeq ($(CONFIG_MTK_GAUGE_VERSION),20)
obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o
endif

# SM5414 switching charger (i2c1 @ 0x49, CHGEN=GPIO115, nSHDN=GPIO4)
obj-$(CONFIG_MTK_SM5414_SUPPORT) += sm5414.o charging_hw_sm5414.o
