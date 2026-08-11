#ifndef _LINUX_M3NOTE_BOARD_H
#define _LINUX_M3NOTE_BOARD_H

#include <linux/types.h>

/*
 * Meizu M3 Note board identity (m681 / l681).
 *
 * The board is the machine compatible of the DTB the boot image carries:
 * the same Image.gz ships with the m681 DTB (wt6755_66_sz_l, "meizu,m681")
 * or with the l681 one (hq6755_66_b1a_l, "meizu,l681"). Both or neither
 * means UNKNOWN, and no board-specific code acts then.
 */
enum m3note_board_id {
	M3NOTE_BOARD_UNKNOWN,
	M3NOTE_BOARD_M681,
	M3NOTE_BOARD_L681,
};

#ifdef CONFIG_MACH_MT6755
enum m3note_board_id m3note_board_id(void);
#else
static inline enum m3note_board_id m3note_board_id(void)
{
	return M3NOTE_BOARD_UNKNOWN;
}
#endif

static inline bool m3note_board_is(enum m3note_board_id board)
{
	return m3note_board_id() == board;
}

/*
 * Second check against an m681 DTB on l681 hardware. Every driver that
 * takes an m681 GPIO that has another meaning on l681 - GPIO4 (SM5414
 * nSHDN / l681 gyro EINT), GPIO19 (fp RST, FDD band detect / l681 microSD
 * LDO), GPIO99 (flash EN), GPIO100 (m681 amp / l681 BQ24196 CE), GPIO115
 * (SM5414 CHGEN / l681 amp) - must call
 * this before touching the pad and refuse to start unless it returns 1:
 *   1            the DT board is m681 and i2c1 does not show the l681
 *                charger (BQ24196 at 0x6b without an SM5414 at 0x49);
 *   0            refuse: DT board is not m681, or BQ24196 found and no
 *                SM5414 (l681 hardware under an m681 DTB);
 *   -EPROBE_DEFER i2c1 is not registered yet.
 * Read-only: one SMBus byte read per chip (BQ24196 REG0A, SM5414 0x0E).
 */
#ifdef CONFIG_MACH_MT6755
int m3note_m681_gpio_owner_ok(void);
/* "m3note: DT board X, charger says Y hardware: match"; reads i2c1, so
 * only the l681 battery observer calls it */
void m3note_hw_report(void);
#else
static inline int m3note_m681_gpio_owner_ok(void)
{
	return 0;
}
static inline void m3note_hw_report(void)
{
}
#endif

/*
 * 180-degree OVL output rotation (CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW):
 * m681 needs it, l681 does not. The l681 LK draws its logo unrotated
 * (hx8399 MADCTL 0x36 = 0x00), so with the flip the picture would turn
 * over when the kernel takes the display, until SurfaceFlinger
 * (ro.sf.hwrotation 180 on l681) sets it right; stock l681 has no HW
 * rotation and turns Android by 180 in SF. An unknown board keeps the
 * flip.
 */
static inline bool m3note_lcm_hw_rot180(void)
{
	return !m3note_board_is(M3NOTE_BOARD_L681);
}

#endif
