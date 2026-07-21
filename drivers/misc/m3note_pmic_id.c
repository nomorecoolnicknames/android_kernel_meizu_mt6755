/*
 * Meizu M3 Note: print the PMIC chip ID once at boot (read-only).
 *
 * The 4.4 PMIC driver is built as CONFIG_MTK_PMIC_CHIP_MT6353 while the
 * board code writes MT6351 register addresses (camera VGP3 on L681, the
 * MT6351 tables elsewhere). HWCID (0x0200) and SWCID (0x0202) sit at the
 * same address on both chips: 0x51xx = MT6351, 0x53xx = MT6353. The stock
 * L681 kernel reports chip 0x5120.
 */
#include <linux/init.h>
#include <linux/kernel.h>

/* mt-plat/upmu_common.h needs the mach include paths of drivers/misc/mediatek */
extern unsigned int pmic_read_interface(unsigned int RegNum, unsigned int *val,
					unsigned int MASK, unsigned int SHIFT);

#define M3NOTE_PMIC_HWCID	0x0200
#define M3NOTE_PMIC_SWCID	0x0202

static int __init m3note_pmic_id_report(void)
{
	unsigned int hwcid = 0, swcid = 0;

	pmic_read_interface(M3NOTE_PMIC_HWCID, &hwcid, 0xFFFF, 0);
	pmic_read_interface(M3NOTE_PMIC_SWCID, &swcid, 0xFFFF, 0);
	pr_notice("m3note: PMIC HWCID=0x%04x SWCID=0x%04x (%s)\n", hwcid, swcid,
		  (hwcid >> 8) == 0x51 ? "MT6351" :
		  (hwcid >> 8) == 0x53 ? "MT6353" : "unknown");
	return 0;
}
late_initcall(m3note_pmic_id_report);
