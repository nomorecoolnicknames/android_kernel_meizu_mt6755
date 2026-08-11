/*
 * kbase fence support on a 4.9 kernel (shared with the r7p0 port).
 *
 * The DDK was written against the pre-4.6 Android sync driver
 * (sync_timeline / sync_pt / sync_fence, CONFIG_SYNC). Upstream replaced it
 * with sync_file in 4.6, so every CONFIG_SYNC block here compiled out and
 * KBASE_FUNC_STREAM_CREATE answered MALI_ERROR_FUNCTION_FAILED, after
 * which libGLES_mali dereferenced NULL inside eglp_swap_buffers.
 *
 * 4.9 still carries sync_timeline/sync_pt for sw_sync
 * (drivers/dma-buf/sync_debug.h), and MTK's display sync driver
 * (drivers/misc/mediatek/sync) already drives them with
 * timeline_create()/fence_create()/timeline_inc(). This declares the small
 * shim built on top of those in mali_kbase_sync_49.c.
 */

#ifndef _MALI_KBASE_SYNC_COMPAT_H_
#define _MALI_KBASE_SYNC_COMPAT_H_

#include <linux/fence.h>
#include <linux/file.h>
#include <linux/sync_file.h>

struct kbase_jd_atom;

/* Stream (timeline) handling — the KBASE_FUNC_STREAM_CREATE path. */
int kbase_stream_create(const char *name, int *const out_fd);
int kbase_stream_create_fence(int tl_fd);
int kbase_fence_validate(int fd);

/* Signal the fence a SOFT_FENCE_TRIGGER atom carries. Returns 0 when the
 * fence belonged to one of our streams, -EINVAL otherwise.
 */
int kbase_sync_fence_signal(struct sync_file *sync_file);

/* The two old helpers the soft-job code still spells out, mapped onto the
 * sync_file equivalents so the DDK source keeps its original shape. */
static inline struct sync_file *kbase_sync_fdget(int fd)
{
	struct file *file = fget(fd);
	struct fence *fence;

	if (!file)
		return NULL;

	/* Validate that the fd really is a fence; the caller keeps the file
	 * reference and drops it with sync_fence_put(), as before. */
	fence = sync_file_get_fence(fd);
	if (!fence) {
		fput(file);
		return NULL;
	}
	fence_put(fence);

	return file->private_data;
}

static inline void kbase_sync_fput(struct sync_file *sync_file)
{
	if (sync_file && sync_file->file)
		fput(sync_file->file);
}

#define sync_fence_fdget(fd)	kbase_sync_fdget(fd)
#define sync_fence_put(sf)	kbase_sync_fput(sf)

#endif /* _MALI_KBASE_SYNC_COMPAT_H_ */
