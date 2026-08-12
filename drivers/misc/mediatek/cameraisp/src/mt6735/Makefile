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

#
# Makefile for misc devices that really don't fit anywhere else.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)/
ccflags-y += $(call cc-option,-Wno-error=maybe-uninitialized)

# camera ISP requires CMDQ and SMI/MMDVFS; honour CONFIG_MTK_CAMERA_ISP so a
# minimal kernel can build without them. The arm32/arm64 branches were
# identical. The dummy object stays unconditional so built-in.o is always
# regenerated (an empty obj-y leaves a stale built-in.o behind).
obj-y += cameraisp_dummy2.o
ifeq ($(CONFIG_MTK_CAMERA_ISP),y)
  ifeq ($(strip $(CONFIG_MACH_MT6735M)),y)
    obj-y += camera_isp_D2.o
    obj-y += camera_sysram_D2.o
    obj-y += camera_pipe_mgr_D2.o
  else
    obj-y += camera_isp_D1.o
    obj-y += camera_sysram_D1.o
    obj-y += camera_fdvt_D1.o
  endif
endif

# $(info cameraisp drv by platform $(platform_drv))
