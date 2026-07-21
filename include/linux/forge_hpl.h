#ifndef _LINUX_FORGE_HPL_H
#define _LINUX_FORGE_HPL_H
/*
 * HPS-lite: forge_hpl.c lives in drivers/misc/mediatek/base/power/mt6755
 * and is built only with CONFIG_HOTPLUG_CPU; everyone else gets the no-op
 * stubs.
 */
#include <linux/types.h>

bool forge_hp_pinned(void);		/* init/forge_m681_marker.c */
/*  lcd-backlight led_classdev level, read only; -1 if no such LED.
 * mediatek/leds/mtk_leds_drv.c; weak -1 default in forge_hpl.c. */
int forge_lcd_bl_level(void);

#if defined(CONFIG_HOTPLUG_CPU) && defined(CONFIG_MACH_MT6755)
void forge_hpl_display(bool on);	/* end of primary_display_suspend/resume */
void forge_hpl_restore_all(void);	/* online every CPU HPS-lite took down */
#else
static inline void forge_hpl_display(bool on) { }
static inline void forge_hpl_restore_all(void) { }
#endif

#endif
