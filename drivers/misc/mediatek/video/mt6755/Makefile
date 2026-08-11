#
# MT6755 display stack. It is self-contained: the headers of video/include
# and the video/common pieces it needs (fence, debug, AAL/PWM, COLOR,
# CCORR/GAMMA/DITHER) are kept as private copies under mt6755/include and
# mt6755/common, so the mt6735 display that uses the shared
# video/include + video/common is not touched. These paths come first on
# every mt6755 compile line (subdir-ccflags precede ccflags-y).
#
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6755/include \
		    -I$(srctree)/drivers/misc/mediatek/lcm/mt6755/inc \
		    -I$(srctree)/drivers/misc/mediatek/video/mt6755/common \
		    -I$(srctree)/drivers/misc/mediatek/video/mt6755/dispsys \
		    -I$(srctree)/drivers/misc/mediatek/video/mt6755/videox \
		    -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755 \
		    -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755/mt6755 \
		    -I$(srctree)/drivers/misc/mediatek/smi/mt6755 \
		    -I$(srctree)/drivers/misc/mediatek/mmp \
		    -I$(srctree)/drivers/misc/mediatek/m4u/mt6755 \
		    -I$(srctree)/drivers/misc/mediatek/m4u/2.0 \
		    -I$(srctree)/drivers/misc/mediatek/leds \
		    -I$(srctree)/drivers/misc/mediatek/leds/mt6755

obj-y += dispsys/
obj-y += videox/
obj-y += common/
