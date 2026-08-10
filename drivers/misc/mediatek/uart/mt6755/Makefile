#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

ccflags-y += -I$(srctree)/drivers/misc/mediatek/uart
ccflags-y += -I$(srctree)/drivers/misc/mediatek/uart/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/uart/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)

obj-$(CONFIG_MTK_SERIAL) := platform_uart.o
