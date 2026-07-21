/*
 * Meizu M3 Note unified kernel (CONFIG_M3NOTE_UNIFIED): the GM20 battery
 * stack calls one chr_control_interface(); this file routes it to the
 * SM5414 HAL on m681 and to the BQ24196 HAL on l681 (board from the DT,
 * linux/m3note_board.h). Unknown board: every command is unsupported, so
 * no charger register is written.
 *
 * Also the single definitions both HALs used to carry: chargin_hw_init_done
 * (set by whichever charger probes) and the forge_chg_poweroff knob, now
 * /sys/module/charging_hw_m3note/parameters/forge_chg_poweroff.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/m3note_board.h>

#include <mt-plat/charging.h>

#define STATUS_UNSUPPORTED	-1

kal_bool chargin_hw_init_done = KAL_FALSE;

int forge_chg_poweroff;
module_param(forge_chg_poweroff, int, 0644);
MODULE_PARM_DESC(forge_chg_poweroff,
	"chg-diag: 0=log+refuse kernel_power_off/restart requests (default), 1=allow");

signed int chr_control_interface_sm5414(CHARGING_CTRL_CMD cmd, void *data);
signed int chr_control_interface_bq24196(CHARGING_CTRL_CMD cmd, void *data);
kal_bool charging_type_detection_done_sm5414(void);
kal_bool charging_type_detection_done_bq24196(void);

signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	switch (m3note_board_id()) {
	case M3NOTE_BOARD_M681:
		return chr_control_interface_sm5414(cmd, data);
	case M3NOTE_BOARD_L681:
		return chr_control_interface_bq24196(cmd, data);
	default:
		return STATUS_UNSUPPORTED;
	}
}

kal_bool charging_type_detection_done(void)
{
	switch (m3note_board_id()) {
	case M3NOTE_BOARD_M681:
		return charging_type_detection_done_sm5414();
	case M3NOTE_BOARD_L681:
		return charging_type_detection_done_bq24196();
	default:
		return KAL_TRUE;
	}
}
