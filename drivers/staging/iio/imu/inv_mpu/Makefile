#
# Makefile for Invensense inv-mpu-iio device.
#
# INV_KERNEL_3_10 selects the modern-API branches (iio_device_alloc,
# iio_trigger_alloc, iio_push_to_buffers) which match 4.4 as well; the
# stock VERSION/PATCHLEVEL detection cannot see 4.4 so it is hardcoded.

obj-$(CONFIG_INV_MPU_IIO) += inv-mpu-iio.o
inv-mpu-iio-objs := inv_mpu_core.o
inv-mpu-iio-objs += inv_mpu_ring.o
inv-mpu-iio-objs += inv_mpu_trigger.o
inv-mpu-iio-objs += inv_mpu_misc.o
inv-mpu-iio-objs += inv_mpu3050_iio.o
inv-mpu-iio-objs += dmp3Default_20608D.o

ccflags-y := -I$(srctree)/include/linux/iio -DINV_KERNEL_3_10
# mt_gpio.h + mach/gpio_const.h for the stock GPIO17 EINT pad setup in
# inv_mpu_ring.c (same include pattern as GT9XX_MZ/Makefile)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
# vendor code predates drivers/Makefile's blanket -Werror; the kernel's
# explicit -Werror= forms (implicit-int etc.) remain in force
ccflags-y += -Wno-error
