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

# mach/mt_thermal.h (pulled by mt_gpufreq.c) includes "mt_gpufreq.h" from here
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpuidle_v1/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/irq/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/thermal/fakeHeader/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/lcm/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/2.0
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/2.0
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
# mt_thermal.h (included by mt_cpufreq.c) includes mtk_thermal_typedefs.h
# by bare name; add the mt-plat root.
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

obj-y += mt_pm_init.o
obj-y += mt_spm_mtcmos.o
# mt_static_power is built with the GPU (mt_gpufreq is its only user).
obj-$(CONFIG_MTK_GPU_SUPPORT) += mt_static_power.o
# Read-only MT6311 ext-buck presence probe; runs only when
# forge_extbuck_probe is written.
obj-y += mt_extbuck_probe.o  # read-only MT6311/MT6351/efuse dump (forge_extbuck_probe=2)
obj-y += forge_ppm_thermal.o  # ATM CPU budget -> cpufreq ceilings from the stock PPM power table
# Raw ARMPLL frequency switch, inert by default.
obj-y += mt_dvfs_p0.o  # interlock target of mt_cpufreq (forge_dvfs_p0); no initcall, inert
obj-y += mt_cpufreq.o  # frequency-only port (forge_mt_cpufreq gate)
obj-$(CONFIG_MTK_MT6755_HYBRID_CPU_DVFS) += mt_cpufreq_hybrid.o
# The real gpufreq comes with Mali (kbase links mt_gpufreq_*). It runs the
# stock MT6351 VGPU branch over the read-only PMIC.
obj-$(CONFIG_MTK_GPU_SUPPORT) += mt_gpufreq.o
obj-$(CONFIG_MTK_GPU_SUPPORT) += mt_gpufreq_fh_fallback.o
obj-$(CONFIG_MTK_MT6750TT) += mt_eem_turbo.o

obj-y += mt_vcorefs_stub.o  # no-op is_vcorefs_can_work/vcorefs_request_dvfs_opp (needed by msdc autok_dvfs)
obj-y += mt_pmic_stub.o  # no-op PMIC API while CONFIG_MTK_PMIC_NEW_ARCH is off
# MT6351 pmu_flags_table for the read-only pmic_get_register_value
ifneq ($(CONFIG_MTK_PMIC_NEW_ARCH),y)
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += mt_pmic_flags_mt6351.o
endif
obj-y += mt_misc_stubs.o  # no-op stubs for symbols of subsystems not built here

ifneq ($(CONFIG_ARM64), y)

obj-y += mt-smp.o
obj-y += hotplug.o
obj-y += mt-headsmp.o
obj-y += mt_hotplug.o

else

obj-$(CONFIG_MTK_PSCI) += mt_psci.o  # 4.9 mt-boot cpu ops (body from mt6735)

endif

obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += forge_chg_observe.o
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += forge_chg_ctrl.o  # stock temperature cut-off via SM5414 CTRL.CHGEN (I2C only)
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += forge_bq24196_ctrl.o  # stock temperature cut-off and charge state via BQ24196 REG01/REG08 (I2C only)
