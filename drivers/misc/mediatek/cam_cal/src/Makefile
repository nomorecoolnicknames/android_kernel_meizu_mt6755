#$(info cam_cal: makefile start)
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

# MT6755 (M3 Note): the legacy EEPROM drivers with their headers,
# like imgsensor/inc_mt6755
ifeq ($(CONFIG_MACH_MT6755),y)
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc_mt6755 \
                    -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw \
                    -I$(MTK_CUSTOM_PATH)/cam_cal/inc_mt6755
else
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc \
                    -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw \
                    -I$(MTK_CUSTOM_PATH)/cam_cal/inc
endif
subdir-ccflags-y += -I$(srctree)/drivers/i2c/busses/

LEGACY_PLATFORM = mt6572 mt6580 mt6582 mt6735 mt6752 mt6795 mt6755
CURRENT_PLATFORM = $(subst ",,$(CONFIG_MTK_PLATFORM))

ifneq (, $(findstring $(CURRENT_PLATFORM), $(LEGACY_PLATFORM)))
#$(info cam_cal: use legacy folder)
obj-y += legacy/$(subst ",,$(CONFIG_MTK_PLATFORM))/
else
# $(info cam_cal: use common folder)
obj-y += eeprom_driver.o
obj-y += cam_cal_list.o
endif
#$(info cam_cal: makefile end)

