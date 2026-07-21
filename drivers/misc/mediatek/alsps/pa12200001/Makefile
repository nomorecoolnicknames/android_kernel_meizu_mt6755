ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
# Stock-3.10-vintage code under the mediatek subdir-wide -Werror: demote the
# STYLE warning classes only (real bugs the compiler found are fixed in-code).
ccflags-y += -Wno-error=declaration-after-statement -Wno-error=unused-variable \
             -Wno-error=unused-value -Wno-error=unused-function \
             -Wno-error=parentheses -Wno-error=unused-but-set-variable
obj-y	:=  pa12200001.o
