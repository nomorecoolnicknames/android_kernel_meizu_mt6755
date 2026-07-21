#ifndef _LINUX_M3NOTE_BOARD_H
#define _LINUX_M3NOTE_BOARD_H

#include <linux/types.h>

/*
 * Meizu M3 Note board identity for the M681/L681 kernel
 * (CONFIG_M3NOTE_UNIFIED). The board is the machine compatible of the DTB
 * the boot image carries: the same Image ships twice, once with the M681
 * DTB (wt6755_66_sz_l) and once with the L681 one (hq6755_66_b1a_l).
 *
 * Board-specific drivers call m3note_board_is() before touching their
 * hardware and stay inert on M3NOTE_BOARD_UNKNOWN. CONFIG_M3NOTE_SINGLE_L681
 * names the board of an L681-only build at compile time. Without either
 * option every m3note_board_is() check passes (single-board builds).
 */
enum m3note_board_id {
	M3NOTE_BOARD_UNKNOWN,
	M3NOTE_BOARD_M681,
	M3NOTE_BOARD_L681,
};

#if defined(CONFIG_M3NOTE_UNIFIED)
enum m3note_board_id m3note_board_id(void);
#define M3NOTE_BOARD_KNOWN 1
#elif defined(CONFIG_M3NOTE_SINGLE_L681)
/* l681-only build (no m3note_board.c, no DT lookup) */
static inline enum m3note_board_id m3note_board_id(void)
{
	return M3NOTE_BOARD_L681;
}
#define M3NOTE_BOARD_KNOWN 1
#endif

#ifdef M3NOTE_BOARD_KNOWN
static inline bool m3note_board_is(enum m3note_board_id board)
{
	return m3note_board_id() == board;
}
#else
static inline bool m3note_board_is(enum m3note_board_id board)
{
	return true;
}
#endif

/*
 * For pads whose meaning differs between the boards: true only when the
 * build knows its board and it is this one, false in a board-agnostic
 * single-board build (which keeps its old, m681 pads).
 */
static inline bool m3note_board_known_is(enum m3note_board_id board)
{
#ifdef M3NOTE_BOARD_KNOWN
	return m3note_board_is(board);
#else
	return false;
#endif
}

/*
 * Segment pin. The MT6755 BSP takes a segment from
 * get_devinfo_with_index(21) & 0xFF. With the v1 LK devinfo atag (+1 shift)
 * that is stock index 20, which reads 0x26 (the low byte of hw_code
 * 0x0326), not the stock segment word. Every segment branch on L681 takes
 * this constant, the value all of them see today, so none of them (MT6353
 * PMIC writes, VMD1 GPIO, new SPM firmware, ARMPLL HW mode, ROSA DRAM
 * steps, GPU turbo) can follow the efuse, whatever a later devinfo index
 * fix returns. On other boards the efuse expression is used as is.
 */
#define M3NOTE_L681_SEGMENT	0x26
/*
 * M681 is pinned the same way: its LK hands the same v1 devinfo atag, and
 * index 21 & 0xFF reads 0x26 there as well.
 */
static inline bool m3note_segment_pinned(void)
{
	return m3note_board_known_is(M3NOTE_BOARD_L681) ||
	       m3note_board_known_is(M3NOTE_BOARD_M681);
}
#define m3note_segment_or(efuse_expr) \
	(m3note_segment_pinned() ? M3NOTE_L681_SEGMENT : (efuse_expr))

/*
 * 180-degree OVL output rotation (CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW): the
 * board default before the composer runs. The flip itself follows
 * forge_disp_rot180() / forge_disp_rot180_caps() (mtk_disp_mgr.c); L681 is
 * flipped from the composer's first frame. The L681 LK draws its logo
 * unrotated and the HX8399 panel scans in the Android orientation (MADCTL
 * 0x36 = 0x00), so a flip from boot would turn the logo over when the
 * kernel takes the display. Stock L681 3.10 has no HW rotation
 * (MTK_LCM_PHYSICAL_ROTATION="180", _HW not set): SF turns Android by 180.
 */
static inline bool m3note_lcm_hw_rot180(void)
{
	return !m3note_board_known_is(M3NOTE_BOARD_L681);
}

bool forge_disp_rot180(void);		/* OVL flip now */
bool forge_disp_rot180_caps(void);	/* is_output_rotated for the HWC */

#endif
