#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-$(CONFIG_MTK_LENS)   += main/
obj-$(CONFIG_MTK_LENS)   += main2/
obj-$(CONFIG_MTK_LENS)   += sub/
# M3 Note (m681): stock standalone GT9762AF VCM drivers
ifeq ($(CONFIG_MACH_MT6755),y)
obj-$(CONFIG_MTK_LENS)   += m681/
endif
