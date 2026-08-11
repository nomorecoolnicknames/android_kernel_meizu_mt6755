#
# M3 Note (m681): the stock GT9762AF voice-coil drivers (standalone
# /dev/GT9762AF and /dev/GT9763AF_ char devices the camera HAL opens by
# lens name), from stock lens/mt6755/gt9762af and gt9762_sunny2_af.
# DW9714AF, the other stock VCM, is in the MAINAF dispatcher (lens/main).
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/lens/m681/inc

obj-y += GT9762AF.o
obj-y += GT9762AF_SUNNY2.o
