# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-y   +=  barometer.o barometer_factory.o

# Only BMP280-new is carried (the L681 barometer).
obj-$(CONFIG_MTK_BMP280_NEW) += BMP280-new/
