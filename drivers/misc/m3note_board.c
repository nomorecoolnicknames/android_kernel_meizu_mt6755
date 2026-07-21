/*
 * Meizu M3 Note board identity (m681 / l681) for the unified kernel.
 *
 * The board is read from the machine compatible of the appended DTB, which
 * unflatten_device_tree() has parsed before the first initcall. Both or
 * neither of "meizu,m681" / "meizu,l681" means UNKNOWN, and then no
 * board-specific driver acts. The result is cached on first use.
 *
 * /sys/module/m3note_board/parameters/board reports the selection.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/m3note_board.h>

static int m3note_board_cached = -1;

enum m3note_board_id m3note_board_id(void)
{
	int id = READ_ONCE(m3note_board_cached);

	if (id < 0) {
		bool m681 = of_machine_is_compatible("meizu,m681");
		bool l681 = of_machine_is_compatible("meizu,l681");

		if (m681 == l681)
			id = M3NOTE_BOARD_UNKNOWN;
		else
			id = m681 ? M3NOTE_BOARD_M681 : M3NOTE_BOARD_L681;
		WRITE_ONCE(m3note_board_cached, id);
	}
	return id;
}
EXPORT_SYMBOL(m3note_board_id);

static const char *const m3note_board_names[] = {
	[M3NOTE_BOARD_UNKNOWN] = "unknown",
	[M3NOTE_BOARD_M681] = "m681",
	[M3NOTE_BOARD_L681] = "l681",
};

static int m3note_board_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%s\n",
			 m3note_board_names[m3note_board_id()]);
}

static const struct kernel_param_ops m3note_board_ops = {
	.get = m3note_board_get,
};
module_param_cb(board, &m3note_board_ops, NULL, 0444);

static int __init m3note_board_report(void)
{
	pr_notice("m3note: board %s (DT machine compatible)\n",
		  m3note_board_names[m3note_board_id()]);
	return 0;
}
early_initcall(m3note_board_report);
