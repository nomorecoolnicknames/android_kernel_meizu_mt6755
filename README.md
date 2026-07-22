# Linux kernel for the Meizu M3 Note (M681 / L681) — LineageOS 16.0

Linux 4.4 kernel for the Meizu M3 Note (MediaTek MT6755 / Helio P10), used
for Android 9 / LineageOS 16.0. One kernel image serves both hardware
revisions; append the device tree of the phone you build for:

| Device | Board | Device tree |
|---|---|---|
| M3 Note M681 | `wt6755_66_sz_l` | `arch/arm64/boot/dts/mediatek/wt6755_66_sz_l.dts` |
| M3 Note L681 | `hq6755_66_b1a_l` | `arch/arm64/boot/dts/mediatek/hq6755_66_b1a_l.dts` |

The kernel recognises the board from the device tree at boot.

## Origin

- Base: [hyperion70/kernel-4.4](https://github.com/hyperion70/kernel-4.4),
  a MediaTek Linux 4.4 tree for MT6755; its history is kept.
- MT6755 platform, board and driver code ported from MediaTek and Meizu
  kernel sources released under the GPL for this phone, plus new work for
  Android 9.
- Original authorship and copyright notices are kept. The kernel is licensed
  under the GPL version 2; see [COPYING](COPYING). Upstream notes are in
  [README](README).

Some driver parameters and log messages carry a `forge_` / `FORGE` prefix.
They are part of the interface used by the device configuration and are kept
as they are.

## Status

Android 9 runs on both revisions. Display, touch, charging, Wi-Fi and
Bluetooth, sensors and the barometer are integrated; power management and
audio are still being refined.

This snapshot was tested on the L681 on 16 October 2026: the camera HAL
sees and powers both camera sensors, the display survives locking and
unlocking, rotation and the navigation bar are correct, Wi-Fi scans, the
sound card is present and thermal protection is armed. The camera app does
not show a preview yet (a userspace problem). The M681 has not been tested
with this snapshot.

## Building

Use the AOSP GCC 4.9 toolchain (`aarch64-linux-android-4.9`).

```sh
mkdir -p out
make O="$PWD/out" ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- \
    HOSTCFLAGS="-w -fcommon -O2" m3note_defconfig
make O="$PWD/out" ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- \
    HOSTCFLAGS="-w -fcommon -O2" DTC_FLAGS=-f -j8 Image.gz dtbs
cat out/arch/arm64/boot/Image.gz \
    out/arch/arm64/boot/dts/mediatek/wt6755_66_sz_l.dtb > Image.gz-dtb   # M681
```

For the L681 append `hq6755_66_b1a_l.dtb` instead. The board pin
configuration (`cust.dtsi`) is committed as generated text.

## Not included

- Touch controller firmware. The Goodix GT9xx driver is included with its
  header-firmware update disabled (`GTP_HEADER_FW_UPDATE 0`); the controller
  keeps the firmware it was shipped with. The Synaptics TD4322 driver and its
  firmware headers come unchanged from the upstream tree and are not enabled.
- Binary board-configuration files (`*.dws`) and the DCT executables that turn
  them into `cust.dtsi`; the generated `cust.dtsi` files are used instead.

No kernel image, proprietary firmware, device identity or calibration data is
included beyond what the upstream tree already contains.
