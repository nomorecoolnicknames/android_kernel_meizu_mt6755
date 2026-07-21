$(info cam_cal: makefile start)
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc \
                    -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw \
                    -I$(MTK_CUSTOM_PATH)/cam_cal/inc
# MT6755 uses the legacy per-driver layout: the stock 3.10 kernel builds
# standalone old-arch EEPROM drivers (gt24c64a x3 + s5k5e8) which also export
# the OTP symbols (selective_read_*, camera_b_info, lsc*) the ported
# ov13853/s5k5e8yx imgsensor drivers link against. The new-arch common/ path
# stays unbuilt, avoiding a duplicate /dev/CAM_CAL_DRV registration.
LEGACY_PLATFORM = mt6572 mt6580 mt6582 mt6735 mt6752 mt6755 mt6795
CURRENT_PLATFORM = $(subst ",,$(CONFIG_MTK_PLATFORM))

ifneq (, $(findstring $(CURRENT_PLATFORM), $(LEGACY_PLATFORM)))
$(info cam_cal: use legacy folder)
obj-y += legacy/$(subst ",,$(CONFIG_MTK_PLATFORM))/
else
$(info cam_cal: use common folder)
obj-y += cam_cal_drv.o
obj-y += cam_cal_list.o
obj-y += common/
endif
$(info cam_cal: makefile end)

