/*
 * Legacy "dev_info" platform-driver sysfs file, as in the stock 3.10
 * devinfosys.c. The 4.4 devinfo.c only has the /dev/devinfo cdev+ioctl
 * interface. The vendor audio blob chain libbessound_hd_mtk.so ->
 * libmtk_drvb.so (used by MtkAudioBitConverter on every playback write)
 * reads
 *   /sys/bus/platform/drivers/dev_info/dev_info
 * as a binary u32 array: [count, v0, v1, ...] (it needs at least idx 20 =
 * hw_code, checked against MT6755's 0x0326). Without the file the blob's
 * failure path crashes the audio HAL and audioserver restarts in a loop.
 *
 * Read-only, no hardware access, no ISR.
 * Knob: devinfosys.forge_dev_info=0 skips registration.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>

/* provided by devinfo.c (no shared header in this vintage) */
extern u32 devinfo_get_size(void);
extern u32 get_devinfo_with_index(u32 index);

static int forge_dev_info = 1;
module_param(forge_dev_info, int, 0444);
MODULE_PARM_DESC(forge_dev_info,
		 "1 = register legacy /sys/bus/platform/drivers/dev_info/dev_info (binary devinfo array; libmtk_drvb consumer)");

static struct platform_driver dev_info_drv = {
	.driver  = {
		.name = "dev_info",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	}
};

/*
 * v1 ATAG layout compensation (see the stock 3.10 devinfo.h):
 *
 * The stock LK stores the raw v1 tag in chosen/atag,devinfo:
 *   word0 = atag hdr size (0x35)      word1 = tag id 0x41000804
 *   word2..word51 = data[50]          word52 = data_size (=50)
 * Stock 3.10 struct devinfo_tag { u32 size; u32 tag; u32 data[50];
 * u32 data_size; } -> stock data[] starts at word2 and count comes from
 * the last word. The 4.4 devinfo.c uses the v2 headerless struct
 * { u32 data_size; u32 data[]; }, so on this LK its g_devinfo_data is
 * shifted: data[0] = tag id, real payload at data[1..50], stock data_size
 * at data[51], data[52] = 1-word FDT overread junk (hw_code 0x326 sits at
 * raw property word22 = data[21]).
 *
 * This re-emits the stock userspace ABI ([count, payload...], count = 50,
 * payload from data[1]) so libmtk_drvb's get_dev_info_data(idx) uses stock
 * numbering (idx 20 == 0x326). The in-kernel get_devinfo_with_index()
 * numbering stays shifted: changing it globally flips the Mali gpu_efuse
 * (spare3 idx7 bit6 -> MP1 core mask) and other consumers
 * (masp/dfd/smi/segcode).
 */
#define ATAG_DEVINFO_DATA_TAG	0x41000804

static int devinfo_v1_layout(void)
{
	return devinfo_get_size() >= 3 &&
	       get_devinfo_with_index(0) == ATAG_DEVINFO_DATA_TAG;
}

/* value at STOCK numbering index i (what the vendor blobs expect) */
static u32 devinfo_stock_value(u32 i)
{
	if (devinfo_v1_layout())
		return get_devinfo_with_index(i + 1);
	return get_devinfo_with_index(i);
}

static u32 devinfo_stock_count(void)
{
	u32 size = devinfo_get_size();

	if (devinfo_v1_layout()) {
		/* stock data_size = last real word (before the overread) */
		u32 payload = get_devinfo_with_index(size - 2);

		if (payload == 0 || payload > size - 3)
			payload = size - 3;
		return payload;
	}
	return size;
}

static ssize_t devinfo_sysfs_show(struct device_driver *driver, char *buf)
{
	unsigned int i;
	unsigned int *output = (unsigned int *)buf;
	u32 count = devinfo_stock_count();

	/* keep [count + values] inside one page */
	if ((count + 1) * sizeof(u32) > PAGE_SIZE)
		count = PAGE_SIZE / sizeof(u32) - 1;

	output[0] = count;
	for (i = 0; i < count; i++)
		output[i + 1] = devinfo_stock_value(i);

	return (count + 1) * sizeof(unsigned int);
}

static DRIVER_ATTR(dev_info, 0444, devinfo_sysfs_show, NULL);

static int __init devinfo_sysfs_init(void)
{
	int ret;

	if (!forge_dev_info) {
		pr_info("[FORGE_DEVINFO] legacy dev_info sysfs disabled by knob\n");
		return 0;
	}

	ret = driver_register(&dev_info_drv.driver);
	if (ret) {
		pr_warn("[FORGE_DEVINFO] driver_register fail %d\n", ret);
		return 0;	/* never block boot */
	}

	ret = driver_create_file(&dev_info_drv.driver, &driver_attr_dev_info);
	if (ret) {
		pr_warn("[FORGE_DEVINFO] create sysfs file fail %d\n", ret);
		driver_unregister(&dev_info_drv.driver);
		return 0;	/* never block boot */
	}

	/* marker derives from the SAME path the blob reads (self-validating) */
	pr_info("[FORGE_DEVINFO] legacy dev_info sysfs up (%s count=%u hw_code=0x%x)\n",
		devinfo_v1_layout() ? "v1-atag" : "v2",
		devinfo_stock_count(),
		devinfo_stock_count() > 20 ? devinfo_stock_value(20) : 0);
	return 0;
}
late_initcall(devinfo_sysfs_init);
