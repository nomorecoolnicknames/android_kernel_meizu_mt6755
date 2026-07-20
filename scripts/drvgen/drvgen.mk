# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.

ifdef MTK_PLATFORM
DRVGEN_PATH := drivers/misc/mediatek/dws/$(MTK_PLATFORM)

# The DrvGen scripts under tools/dct are the python3-compatible versions
# from the 3.18 tree. Default the recipe interpreter to python3 (as in that
# tree's scripts/drvgen/drvgen.mk) so cust.dtsi generation does not fall
# back to a python2 /usr/bin/python.
python ?= python3

ifndef DRVGEN_OUT
DRVGEN_OUT := $(objtree)/arch/$(SRCARCH)/boot/dts
endif
export DRVGEN_OUT

ALL_DRVGEN_FILE := cust.dtsi

DWS_FILE := $(srctree)/$(DRVGEN_PATH)/$(MTK_PROJECT).dws
ifneq ($(wildcard $(DWS_FILE)),)
DRVGEN_FILE_LIST := $(addprefix $(DRVGEN_OUT)/,$(ALL_DRVGEN_FILE))
else
DRVGEN_FILE_LIST :=
endif
DRVGEN_TOOL := $(srctree)/tools/dct/DrvGen.py

# Every board DWS of the platform also gets its own
# $(DRVGEN_OUT)/<project>/cust.dtsi, and each board DTS includes
# "<project>/cust.dtsi", so one tree builds the M681 and the L681 DTB with
# their own DCT. The top-level cust.dtsi of MTK_PROJECT is kept as before.
ifneq ($(DRVGEN_FILE_LIST),)
DRVGEN_BOARDS := $(basename $(notdir $(wildcard $(srctree)/$(DRVGEN_PATH)/*.dws)))
DRVGEN_FILE_LIST += $(foreach b,$(DRVGEN_BOARDS),$(DRVGEN_OUT)/$(b)/cust.dtsi)
endif

.PHONY: drvgen
drvgen: $(DRVGEN_FILE_LIST)
$(DRVGEN_OUT)/cust.dtsi: $(DRVGEN_TOOL) $(DWS_FILE)
	@mkdir -p $(dir $@)
	$(python) $(DRVGEN_TOOL) $(DWS_FILE) $(dir $@) $(dir $@) cust_dtsi

$(DRVGEN_OUT)/%/cust.dtsi: $(DRVGEN_TOOL) $(srctree)/$(DRVGEN_PATH)/%.dws
	@mkdir -p $(dir $@)
	$(python) $(DRVGEN_TOOL) $(srctree)/$(DRVGEN_PATH)/$*.dws $(dir $@) $(dir $@) cust_dtsi

endif#MTK_PLATFORM
