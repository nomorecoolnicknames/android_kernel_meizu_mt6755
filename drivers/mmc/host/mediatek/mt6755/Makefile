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

# MT6755 MSDC driver (the MSDC IP is identical to MT6757: same controller at
# 0x11230000, same register map), built natively under mt6755/. The
# per-chip board layer lives in mt6755/mt6755/ (msdc_cust/autok_dvfs) and
# carries the 3.18 MT6755 values in 4.4 format.
#add kernel source code path as head file search path
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
#workaround for include mmc/card/queue.h and mmc/core/core.h
ccflags-y += -I$(srctree)/drivers
ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/mt6755/$(MTK_PLATFORM)
# SPM/vcorefs hardware DVFS is not built; supply stubs.
ccflags-y += -include $(srctree)/drivers/mmc/host/mediatek/mt6755_msdc_compat.h

obj-$(CONFIG_MMC_MTK_PRO) += msdc.o

msdc-y += sd.o
msdc-y += dbg.o
msdc-y += mtk_sd_misc.o
msdc-y += msdc_tune.o
msdc-y += msdc_partition.o
msdc-y += autok.o
msdc-y += $(MTK_PLATFORM)/msdc_cust.o
msdc-y += $(MTK_PLATFORM)/autok_dvfs.o

ifeq ($(CONFIG_FPGA_EARLY_PORTING),y)
msdc-y += $(MTK_PLATFORM)/msdc_fpga_io.o
endif

ifeq ($(CONFIG_MTK_EMMC_SUPPORT_OTP),y)
msdc-y += emmc_otp.o
endif
