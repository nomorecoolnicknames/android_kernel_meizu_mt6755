# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o
# stk3x1x registers with hw == NULL when its DT node is missing and binds the
# shared "mediatek,alsps" client (the ltr579 compatible), then hangs probing
# an absent chip. Build it only when selected.
obj-$(CONFIG_MTK_STK3X1X_NEW)   +=  stk3x1x-new/

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
else
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
obj-$(CONFIG_MTK_CM36652_NEW)	+=  cm36652-new/
# [yanlin start] Porting sensors
obj-$(CONFIG_MTK_LTR579)	+=  ltr579/
# [yanlin end]
# PA12200001 (0x1e) is the second ALS/PS source. Link order is detect
# order: keep it after ltr579 so ltr579 gets the shared alsps@53 client
# first.
obj-$(CONFIG_MTK_PA12200001)	+=  pa12200001/
endif
