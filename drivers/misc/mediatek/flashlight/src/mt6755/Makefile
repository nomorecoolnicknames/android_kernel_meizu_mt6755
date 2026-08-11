#
# M3 Note (MT6755): the stock legacy flashlight stack
# (/dev/kd_camera_flashlight, the ioctl ABI of the N libcameracustom /
# libfeatureiodrv), with the SGM3784 constant_flashlight driver.
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/flashlight/inc_mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/imgsensor/inc_mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/imgsensor/src/mt6755/camera_hw

obj-y				+= kd_flashlightlist.o
obj-y				+= strobe_main_sid2_part1.o
obj-y				+= strobe_main_sid2_part2.o
obj-y				+= strobe_sub_sid2_part1.o
obj-y				+= strobe_sub_sid2_part2.o
obj-y				+= sub_strobe.o
obj-y				+= strobe_main_sid1_part2.o
obj-y				+= strobe_part_id.o
obj-y				+= strobe_sub_sid1_part2.o
obj-y				+= constant_flashlight/
# stock debug printks pass unsigned long to %d and keep dead locals
ccflags-y += -Wno-format -Wno-unused-variable
