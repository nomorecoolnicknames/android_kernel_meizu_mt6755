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

obj-$(CONFIG_MACH_MT6757) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_MACH_KIBOPLUS) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
# The MT6755 base/power is monolithic: every object lives in the mt6755/
# subdir, unlike the MT6757 pbm_v1/spm_v2/vcorefs_v1/... split.
obj-$(CONFIG_MACH_MT6755) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
obj-$(CONFIG_MACH_MT6757) += pbm_v1/
obj-$(CONFIG_MACH_KIBOPLUS) += pbm_v1/
obj-$(CONFIG_MACH_MT6757) += spm_v2/
obj-$(CONFIG_MACH_MT6755) += spm_v2/
# CPU dormant (mt_cpu_dormant), as in the MT6755 3.18 BSP; suspend needs it
# (the weak fallback in spm_v2/mt_spm_sleep.c returns 0 without WFI).
# mt6755/mt_pm_init.c calls mt_cpu_dormant_init().
obj-$(CONFIG_MACH_MT6755) += cpuidle_v1/
obj-$(CONFIG_MACH_MT6757) += vcorefs_v1/
obj-$(CONFIG_MACH_KIBOPLUS) += vcorefs_v1/
obj-$(CONFIG_MACH_KIBOPLUS) += spm_v2/
obj-$(CONFIG_MACH_MT6757) += cpuidle_v2/
obj-$(CONFIG_MACH_MT6757) += power_gs_v1/
obj-$(CONFIG_MACH_MT6757) += hps_v2/
obj-$(CONFIG_MACH_KIBOPLUS) += hps_v2/
obj-$(CONFIG_MACH_MT6757) += ppm_v2/
obj-$(CONFIG_MACH_KIBOPLUS) += ppm_v2/
obj-$(CONFIG_MACH_MT8167) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
