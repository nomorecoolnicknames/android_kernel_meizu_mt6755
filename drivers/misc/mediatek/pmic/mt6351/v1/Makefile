# MT6351 PMIC (4.4 new-arch port)
#
# NOTE: include/mt6351 must be searched FIRST so that <mach/upmu_hw.h>,
# <mach/upmu_sw.h>, <mach/upmu_common.h> and <mach/mt_pm_ldo.h> resolve to the
# MT6351 register map shipped under include/mt6351/mach/ rather than the
# platform (mt6757 / MT6355) mach headers.
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6351
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(CONFIG_MTK_PLATFORM)

obj-$(CONFIG_MTK_PMIC_CHIP_MT6351) += pmic.o pmic_common.o \
		pmic_regulator.o pmic_auxadc.o \
		pmic_initial_setting.o pmic_irq.o \
		pmic_throttling_dlpt.o
