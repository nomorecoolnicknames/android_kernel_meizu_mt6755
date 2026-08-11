# vendor sensor code: keep warnings, do not turn them into errors.
# The case-label-exceeds-u16 warning (dead long-shutter cases from stock) has
# no dedicated -W flag on GCC 4.9, hence the blanket -Wno-error.
ccflags-y += -Wno-error
ccflags-y += -Wno-error=declaration-after-statement -Wno-error=strict-prototypes -Wno-error=format-extra-args -Wno-error=format -Wno-error=unused-variable -Wno-error=unused-function -Wno-error=unused-but-set-variable -Wno-error=parentheses -Wno-error=unused-value -Wno-error=unused-label -Wno-error=maybe-uninitialized
obj-y += s5k5e8yxmipiraw_Sensor.o
