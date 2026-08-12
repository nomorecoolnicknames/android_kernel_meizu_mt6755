# Linux kernel for the Meizu M3 Note (M681 / L681) — LineageOS 20

Linux 4.9 kernel for the Meizu M3 Note (MediaTek MT6755 / Helio P10), used
for Android 13 / LineageOS 20. One kernel serves both hardware revisions:

| Device | Board | Device tree |
|---|---|---|
| M3 Note M681 | `wt6755_66_sz_l` | `arch/arm64/boot/dts/mediatek/wt6755_66_sz_l.dts` |
| M3 Note L681 | `hq6755_66_b1a_l` | `arch/arm64/boot/dts/mediatek/hq6755_66_b1a_l.dts` |

Board-specific drivers select themselves at run time.

## Origin

- Base: the MediaTek ALPS Linux 4.9 release published at
  [nyancrimew/mtk-t-alps-release-q0-kernel-4.9-lc](https://github.com/nyancrimew/mtk-t-alps-release-q0-kernel-4.9-lc).
- MT6755 platform, board and driver code ported from MediaTek and Meizu
  kernel sources released under the GPL (Linux 3.10 / 3.18 / 4.4 trees for
  this phone), plus new work for Linux 4.9 and Android 13.
- Original authorship and copyright notices are kept. The kernel is licensed
  under the GPL version 2; see [COPYING](COPYING). Upstream notes are in
  [README](README).

Some driver parameters and log messages carry a `forge_` / `FORGE` prefix.
They are part of the interface used by the device configuration and are kept
as they are.

## Status

Android 13 boots on the M681 with this kernel (checked on 16 October 2026):
USB/ADB, CPU frequency scaling, the display, rotation, 19 sensors, the
vibrator driver and charging work, and the front camera sensor is detected.
Not working yet: Wi-Fi and Bluetooth (the connectivity regulator is not set
up), the main camera (I2C bus 2 is skipped at boot), audio (no sound card in
this configuration), the modem and the RTC. The L681 is not tested with this
snapshot.

## Building

Use the AOSP GCC 4.9 toolchain (`aarch64-linux-android-4.9`).

```sh
make ARCH=arm64 MTK_PLATFORM=mt6755 CROSS_COMPILE=aarch64-linux-android- m3note_defconfig
make ARCH=arm64 MTK_PLATFORM=mt6755 CROSS_COMPILE=aarch64-linux-android- -j8 Image.gz-dtb
```

`m3note_defconfig` appends the M681 device tree. For the L681, build
`mediatek/hq6755_66_b1a_l.dtb` and append it to `Image.gz`.

The device-tree rules use the prebuilt host tools `scripts/dtc/dtc_overlay`
and `scripts/dtc/ufdt_apply_overlay` from the upstream release. The board
pin configuration (`cust.dtsi`) is committed as generated text.

## Not included

- Touch controller firmware. The Goodix GT9xx driver is included with its
  header-firmware update disabled (`GTP_HEADER_FW_UPDATE 0`); the controller
  keeps the firmware it was shipped with.
- Binary board-configuration files (`*.dws`) and the DCT executable that turns
  them into `cust.dtsi`; the generated `cust.dtsi` files are used instead.
- Prebuilt binaries of the upstream tree that this kernel does not use:
  firmware images (`firmware/*.ihex`), `scripts/mkimage`, an audio archive,
  other touch firmware headers and the module-signing test key.

No kernel image, proprietary firmware, device identity or calibration data is
included.
