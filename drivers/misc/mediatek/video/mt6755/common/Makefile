#
# MT6755: the video/common pieces the mt6755 display uses: fence + debug in
# this dir, AAL/PWM (aal20), COLOR (color20), CCORR/GAMMA/DITHER (corr10).
# rdma10/wdma10 carry only headers: MT6755 has its own ddp_rdma_ex/ddp_wdma_ex.
#
obj-$(CONFIG_MTK_FB) += mtkfb_fence.o
obj-$(CONFIG_MTK_FB) += mtkfb_debug.o

obj-y += aal20/
obj-y += corr10/
obj-y += color20/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/wdma10/ \
	     -I$(srctree)/drivers/misc/mediatek/video/mt6755/common/rdma10/ \
	     -I$(srctree)/drivers/misc/mediatek/sync/ \
	     -I$(srctree)/drivers/misc/mediatek/mmp/ \
	     -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM) \
	     -I$(srctree)/drivers/staging/android/ion \
	     -I$(srctree)/drivers/staging/android/ion/mtk

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/mt6755/inc

ifeq ($(CONFIG_MTK_DITHERING_SUPPORT), y)
ccflags-y += -DDITHERING_SUPPORT
endif
