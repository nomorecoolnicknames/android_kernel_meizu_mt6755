#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
#include $(srctree)/drivers/misc/mediatek/Makefile.custom

# MTK_PLATFORM is normally set only inside drivers/misc/mediatek; derive it
# here so the include path resolves.
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif


obj-$(CONFIG_GOODIX_FINGERPRINT)	+= gf_spi_tee.o
