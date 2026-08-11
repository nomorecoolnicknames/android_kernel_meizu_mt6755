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

# drivers/dispsys

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6755/include/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/rdma10/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/wdma10/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/color20/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/videox/ \
             -I$(srctree)/drivers/misc/mediatek/video/mt6755/dispsys/ \
             -I$(srctree)/drivers/misc/mediatek/base/power/include/ \
             -I$(srctree)/drivers/misc/mediatek/smi/mt6755/ \
             -I$(srctree)/drivers/misc/mediatek/gpu/ged/include/


subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/mt6755 \
		    -I$(srctree)/drivers/misc/mediatek/m4u/2.0

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755/ \
		    -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755/mt6755/

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/

subdir-ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/mt6755/inc


ifeq ($(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT),y)
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/trustzone/tplay/Tltplay/public/

include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif
endif

#driver
obj-y += ddp_ovl.o
obj-y += ddp_rdma_ex.o
obj-y += ddp_wdma_ex.o
obj-y += ddp_dsi.o
obj-y += ddp_dpi.o
obj-y += ddp_ufoe.o
obj-y += ddp_clkmgr.o
#common
obj-y += ddp_drv.o
obj-y += ddp_path.o
obj-y += ddp_debug.o
obj-y += ddp_dump.o

obj-y += ddp_manager.o
obj-y += ddp_irq.o
obj-y += ddp_info.o
obj-y += ddp_color_format.o
obj-y += ddp_pwm_mux.o

obj-y += ddp_mmp.o
obj-y += ddp_met.o
# obj-y += ddp_od_stub.o
# obj-y += display_recorder.o
# obj-y += ddp_od_stub.o

ifeq ($(CONFIG_MMPROFILE),y)
ccflags-y += -DDEFAULT_MMP_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),user)
ccflags-y += -D_MTK_USER_
endif

ifeq ($(MTK_MIRAVISION_SUPPORT), yes)
    ccflags-y += -DCONFIG_FOR_SOURCE_PQ
endif

ccflags-y += -DMET_USER_EVENT_SUPPORT

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI), "MT8193")
ccflags-y +=	-DHDMI_MT8193_SUPPORT
endif

# EOF
