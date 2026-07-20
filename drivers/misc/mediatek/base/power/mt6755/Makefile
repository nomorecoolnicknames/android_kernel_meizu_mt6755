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
# by bare name.
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

obj-y += mt_pm_init.o
# clk_buf_init() (late_initcall, mt_pm_init.c) programs the MT6351 DCXO
# XO_WCN 26 MHz buffer and the pwrap DCXO_CONN srclkena engine, which the
# CONN domain needs before its first access.
obj-y += mt_clkbuf_ctl.o
# obj-y += mt_dcm.o  # needs dramc
#obj-y += mt_spm.o
#obj-y += mt_spm_fs.o
#obj-y += mt_spm_internal.o
obj-y += mt_spm_mtcmos.o
# Screen-off hotplug of cpu1-6, off by default (forge_hpl.enable=0)
obj-$(CONFIG_HOTPLUG_CPU) += forge_hpl.o
#obj-y += mt_pbm.o
#obj-$(CONFIG_DEBUG_FS) += mt_clkdbg.o
# obj-y += mt_picachu.o
obj-y += mt_static_power.o  # leakage model (mt_gpufreq uses mt_spower_*)
# Read-only MT6311 external buck presence probe, runs only when
# forge_extbuck_probe is written.
obj-y += mt_extbuck_probe.o
# Manual ARMPLL frequency switch, inert by default.
obj-y += mt_dvfs_p0.o
obj-y += mt_cpufreq.o  # armed at runtime through forge_mt_cpufreq
obj-y += forge_ppm_thermal.o  # ATM CPU budget -> cpufreq ceilings from the stock PPM table
obj-y += forge_vrail.o  # VPROC/VSRAM writer, off by default (forge_vrail.allow)
# Test builds only: make FORGE_VRAIL_ALLOW=1; normal builds leave it unset (0)
ifneq ($(FORGE_VRAIL_ALLOW),)
CFLAGS_forge_vrail.o += -DFORGE_VRAIL_ALLOW_DEFAULT=$(FORGE_VRAIL_ALLOW)
endif
# obj-y += mt_cpufreq_hybrid.o
# Real gpufreq: its PMIC writes go through the address-based
# pmic_config_interface() with the MT6351 addresses from mt_gpufreq.c.
obj-y += mt_gpufreq.o
obj-y += mt_gpufreq_fh_fallback.o  # one-shot mt_dfs_mmpll while FHCTL is not built
# obj-y += mt_ptp.o
obj-$(CONFIG_MTK_MT6750TT) += mt_eem_turbo.o

# obj-y += mt_vcorefs_governor.o
# obj-y += mt_vcorefs_manager.o
obj-y += mt_vcorefs_stub.o  # no-op is_vcorefs_can_work/request_dvfs_opp (msdc autok_dvfs needs them)
obj-y += mt_pmic_stub.o  # no-op PMIC API while the pmic/ driver is not built (CONFIG_MTK_PMIC_NEW_ARCH off)
obj-y += mt_misc_stubs.o  # no-op stubs for symbols of subsystems that are not built

ifneq ($(CONFIG_ARM64), y)

obj-y += mt-smp.o
obj-y += hotplug.o
obj-y += mt-headsmp.o
obj-y += mt_hotplug.o

else

# obj-$(CONFIG_MTK_PSCI) += mt_psci.o  # arm64 uses the generic PSCI code

endif

# obj-y += mt_hotplug_strategy_main.o
# obj-y += mt_hotplug_strategy_core.o
# obj-y += mt_hotplug_strategy_procfs.o
# obj-y += mt_hotplug_strategy_cpu.o
# obj-y += mt_hotplug_strategy_api.o
# obj-y += mt_hotplug_strategy_algo.o
