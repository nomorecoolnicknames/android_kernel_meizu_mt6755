ifneq ($(CONFIG_MTK_LCM), y)
	obj-y += mtkfb.o

	ccflags-y += \
		-I$(srctree)/drivers/misc/mediatek/video/include     \
		-I$(srctree)/drivers/misc/mediatek/video/mt6757/dispsys \
		-I$(srctree)/drivers/misc/mediatek/video/mt6757/videox \
		-I$(srctree)/drivers/misc/mediatek/sync/

endif
