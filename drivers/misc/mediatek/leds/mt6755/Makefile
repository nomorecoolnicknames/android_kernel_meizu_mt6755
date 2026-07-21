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
# l681: mx-led (home-key LED) on MT6351 ISINK0
obj-$(CONFIG_MTK_LEDS_L681_MX) += l681_mx_led.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
