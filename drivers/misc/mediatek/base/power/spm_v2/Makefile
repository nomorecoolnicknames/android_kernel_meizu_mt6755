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

ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/cpuidle_v2/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/irq/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/thermal/fakeHeader/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/power_gs/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cameraisp/src/$(MTK_PLATFORM)/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/met/$(MTK_PLATFORM)/

ifeq ($(CONFIG_ARCH_MT6797), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(MTK_PLATFORM)/
endif

#ifeq ($(CONFIG_ARCH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6757/videox/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6757/dispsys/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6757/dispsys/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/lcm/inc/
#endif

ccflags-$(CONFIG_MTK_MET) += -DMET_USER_EVENT_SUPPORT

# MT6755: build the mt6755 SPM (mt_* files; they coexist with the mtk_*
# set by name), including the MTCMOS code with its larb0 and bounded-poll
# fixes.
ifeq ($(CONFIG_MACH_MT6755), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpuidle_v1/
# With the MT6353 PMIC (NEW_ARCH) on, mt_spm.c includes pmic_api_buck.h /
# pmic_api_ldo.h (vcore/vsram buck control); add the pmic dispatcher and
# mt6353 chip-header include paths.
ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6353),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6353/
endif
obj-y += mt_spm_boot_stubs.o
obj-y += mt_sleep.o
obj-y += mt_spm.o
obj-y += mt_spm_fs.o
obj-y += mt_spm_internal.o
obj-y += mt_spm_pmic_wrap.o
obj-y += mt_spm_sleep.o
endif

ifeq ($(CONFIG_MACH_MT6757), y)
#ccflags-y += -I$(srctree)/sound/soc/mediatek/mt_soc_audio_6757/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/power_gs_v1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/mt6757/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cameraisp/src/$(MTK_PLATFORM)/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/

ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6355),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355/
endif

obj-y += mtk_sleep.o
obj-y += mtk_spm_init.o
obj-y += mtk_spm.o
obj-y += mtk_spm_internal.o
obj-y += mtk_spm_pmic_wrap.o
obj-y += mtk_spm_sleep.o
obj-y += mtk_spm_fs.o
obj-y += mtk_idle.o
obj-y += mtk_idle_profile.o
obj-y += mtk_idle_mt6757.o
obj-y += mtk_spm_dpidle.o
obj-y += mtk_spm_dpidle_mt6757.o
obj-y += mtk_spm_sodi.o
obj-y += mtk_spm_sodi3p0.o
obj-y += mtk_spm_sodi_cmdq.o
obj-y += mtk_spm_mcsodi.o
obj-y += mtk_spm_vcorefs_mt6757.o
obj-y += mtk_spm_resource_req.o
endif

#ifeq ($(CONFIG_ARCH_MT6797), y)
#obj-y += mt_idle.o
#obj-y += mt_idle_profile.o
#obj-y += mt_idle_mt6797.o
#obj-y += mt_spm.o
#obj-y += mt_spm_sleep.o
#obj-y += mt_sleep.o
#obj-y += mt_spm_internal.o
#obj-y += mt_spm_dpidle.o
#obj-y += mt_spm_dpidle_mt6797.o
#obj-y += mt_spm_fs.o
#obj-y += mt_spm_sodi.o
#obj-y += mt_spm_sodi3p0.o
#obj-y += mt_spm_sodi_cmdq.o
#obj-y += mt_spm_pmic_wrap.o
#obj-y += mt_spm_vcorefs_mt6797.o
#endif

ifeq ($(CONFIG_MACH_KIBOPLUS), y)
#ccflags-y += -I$(srctree)/sound/soc/mediatek/mt_soc_audio_6757/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/power_gs_v1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/mt6757/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cameraisp/src/$(MTK_PLATFORM)/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/

obj-y += mtk_sleep.o
obj-y += mtk_spm_init.o
obj-y += mtk_spm.o
obj-y += mtk_spm_internal.o
obj-y += mtk_spm_pmic_wrap.o
obj-y += mtk_spm_sleep.o
obj-y += mtk_spm_fs.o
obj-y += mtk_idle.o
obj-y += mtk_idle_profile.o
obj-y += mtk_idle_mt6757.o
obj-y += mtk_spm_dpidle.o
obj-y += mtk_spm_dpidle_mt6757.o
obj-y += mtk_spm_sodi.o
obj-y += mtk_spm_sodi3p0.o
obj-y += mtk_spm_sodi_cmdq.o
obj-y += mtk_spm_mcsodi.o
obj-y += mtk_spm_vcorefs_mt6757.o
obj-y += mtk_spm_resource_req.o
endif
