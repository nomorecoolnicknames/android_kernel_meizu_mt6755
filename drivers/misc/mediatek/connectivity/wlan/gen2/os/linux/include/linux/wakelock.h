/* SPDX-License-Identifier: GPL-2.0 */
/*
 * 4.9 compat shim — <linux/wakelock.h> was removed after 3.18.
 * Map the legacy Android wake_lock API used by the 3.18 wlan gen2 driver
 * onto the wakeup_source API (linux/pm_wakeup.h).
 *
 * A thin wrapper struct (not a bare #define) is used so that both
 * "struct wake_lock" field declarations and the wake_lock*() calls keep
 * working unchanged. Semantics match 3.18:
 *   wake_lock_init/destroy  <-> wakeup_source_init/trash (ws is embedded)
 *   wake_lock/unlock        <-> __pm_stay_awake/__pm_relax
 *   wake_lock_timeout(jiff) <-> __pm_wakeup_event(msecs)
 */
#ifndef _FORGE_LINUX_WAKELOCK_H
#define _FORGE_LINUX_WAKELOCK_H

#include <linux/device.h>	/* pm_wakeup.h refuses to load without _DEVICE_H_ */
#include <linux/pm_wakeup.h>
#include <linux/jiffies.h>

struct wake_lock {
	struct wakeup_source ws;
};

#define WAKE_LOCK_SUSPEND	0

static inline void wake_lock_init(struct wake_lock *wl, int type, const char *name)
{
	wakeup_source_init(&wl->ws, name);
}

static inline void wake_lock_destroy(struct wake_lock *wl)
{
	wakeup_source_trash(&wl->ws);
}

static inline void wake_lock(struct wake_lock *wl)
{
	__pm_stay_awake(&wl->ws);
}

static inline void wake_unlock(struct wake_lock *wl)
{
	__pm_relax(&wl->ws);
}

static inline void wake_lock_timeout(struct wake_lock *wl, long timeout)
{
	__pm_wakeup_event(&wl->ws, jiffies_to_msecs(timeout));
}

#endif /* _FORGE_LINUX_WAKELOCK_H */
