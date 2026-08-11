ccflags-y += -I$(srctree)/drivers/misc/mediatek/flashlight/inc_mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/imgsensor/inc_mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/imgsensor/src/mt6755/camera_hw

obj-y				+= leds_strobe.o
# stock debug printks pass unsigned long to %d and keep dead locals
ccflags-y += -Wno-format -Wno-unused-variable
