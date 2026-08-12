// SPDX-License-Identifier: GPL-2.0
/*
 * forge_m681_fetch.c - neofetch-style ReMeizu boot banner, printed by the KERNEL
 * itself at late_initcall with LIVE data (kernel/cpu/ram/eMMC/model). Lands in the
 * kernel console -> pstore/ramoops + the console, so the "flex" is captured
 * from recovery via /dev/mem even when adb is not up yet.
 *
 * Wire it in init/Makefile:  obj-y += forge_m681_fetch.o
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/cpumask.h>
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/major.h>

static u64 __init forge_blk_gib(int major, int minor)
{
	struct block_device *bdev;
	u64 bytes = 0;

	bdev = blkdev_get_by_dev(MKDEV(major, minor), FMODE_READ, NULL);
	if (!IS_ERR(bdev)) {
		bytes = (u64)i_size_read(bdev->bd_inode);
		blkdev_put(bdev, FMODE_READ);
	}
	return bytes >> 30; /* GiB */
}

static int __init forge_m681_fetch(void)
{
	const char *model = "Meizu M3 Note (m681)";
	unsigned long ram_mib = (totalram_pages * (PAGE_SIZE >> 10)) >> 10;
	u64 emmc_gib;

	if (of_root)
		of_property_read_string(of_root, "model", &model);

	emmc_gib = forge_blk_gib(MMC_BLOCK_MAJOR, 0); /* mmcblk0 = eMMC */

	pr_info("\n");
	pr_info("  #######     reMeizu  -  MT6755 Helio P10  (3.18 -> 4.4 forward-port)\n");
	pr_info("  ##   ##     ---------------------------------------------------------\n");
	pr_info("  #######     OS      LineageOS 15.1 (Android 8.1)\n");
	pr_info("  ##  ##      Device  %s\n", model);
	pr_info("  ##   ##     Kernel  Linux %s %s\n",
		init_uts_ns.name.release, init_uts_ns.name.machine);
	pr_info("  .-----.     SoC     MediaTek MT6755 (Helio P10)\n");
	pr_info("  |o   o|     CPU     %u x Cortex-A53\n", num_possible_cpus());
	pr_info("  |     |     RAM     %lu MiB\n", ram_mib);
	pr_info("  | === |     eMMC    %llu GiB (mmcblk0)\n", emmc_gib);
	pr_info("  |[###]|     GPU     Mali-T860 Midgard [next]\n");
	pr_info("  '-----'     Walls   timer GICv2 SMP HMP eMMC i2c USB AEE init->userspace\n");
	pr_info("  First Meizu M3 Note (m681) Linux-4.4 boot-to-userspace.  #ReMeizu\n");
	pr_info("\n");
	return 0;
}
late_initcall_sync(forge_m681_fetch);
