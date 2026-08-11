#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

obj-$(CONFIG_MTK_LEDS) := mtk_leds.o

obj-$(CONFIG_BACKLIGHT_SUPPORT_LM3697) += mtk_lm3697_driver.o

# the MT6755 display headers are the private mt6755 copies
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/mt6755/include \
	     -I$(srctree)/drivers/misc/mediatek/video/mt6755/dispsys \
	     -I$(srctree)/drivers/misc/mediatek/video/mt6755/videox \
	     -I$(srctree)/drivers/misc/mediatek/video/mt6755/common \
	     -I$(srctree)/drivers/misc/mediatek/lcm/mt6755/inc \
	     -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/cmdq/v2_mt6755/mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/m4u/mt6755 \
	     -I$(srctree)/drivers/misc/mediatek/mmp
