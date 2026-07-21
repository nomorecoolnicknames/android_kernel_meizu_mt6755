# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/magnetometer/inc
obj-y   +=  mag.o
obj-y   +=  mag_factory.o

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_MAGHUB)        +=  maghub/
else
obj-$(CONFIG_MTK_AKM09911_NEW)   +=  akm09911-new/
obj-$(CONFIG_MTK_QMCX983)	+=  qmcX983/
# [yanlin start] Porting sensors
obj-$(CONFIG_MTK_AKM09916_NEW)   +=  akm09916-new/
# [yanlin end]
# ST480 (0x0d) is the second magnetometer source. Link order is detect
# order: keep it after akm09911-new, so akm gets the shared msensor@0c
# client first and fails its chip check on ST480 units.
obj-$(CONFIG_MTK_ST480)	+=  st480/
endif
