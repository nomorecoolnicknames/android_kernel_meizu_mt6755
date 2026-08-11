/*
 * fence streams for kbase on Linux 4.9 (same stream/fence ABI as r7p0).
 *
 * See mali_kbase_sync_compat.h for why this exists. Everything here is
 * expressed with primitives this kernel already has: sw_sync timelines
 * (drivers/dma-buf/sync_debug.h) driven through MTK's display sync helpers
 * (drivers/misc/mediatek/sync/mtk_sync.c), and sync_file for the fds handed
 * to userspace.
 */

#include <linux/anon_inodes.h>
#include <linux/atomic.h>
#include <linux/fence.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sync_file.h>

#include <../drivers/dma-buf/sync_debug.h>

#include "mali_kbase_sync_compat.h"

/* Provided by drivers/misc/mediatek/sync/mtk_sync.c (built-in). */
struct fence_data {
	__u32 value;
	char name[32];
	__s32 fence;
};
extern struct sync_timeline *timeline_create(const char *name);
extern void timeline_destroy(struct sync_timeline *obj);
extern void timeline_inc(struct sync_timeline *obj, u32 value);
extern int fence_create(struct sync_timeline *obj, struct fence_data *data);

struct kbase_sync_stream {
	struct sync_timeline	*timeline;
	atomic_t		seqno;
};

static int kbase_stream_release(struct inode *inode, struct file *file)
{
	struct kbase_sync_stream *stream = file->private_data;

	if (stream) {
		timeline_destroy(stream->timeline);
		kfree(stream);
	}
	return 0;
}

static const struct file_operations kbase_stream_fops = {
	.owner		= THIS_MODULE,
	.release	= kbase_stream_release,
};

int kbase_stream_create(const char *name, int *const out_fd)
{
	struct kbase_sync_stream *stream;

	if (!out_fd)
		return -EINVAL;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->timeline = timeline_create(name);
	if (!stream->timeline) {
		kfree(stream);
		return -ENOMEM;
	}
	atomic_set(&stream->seqno, 0);

	*out_fd = anon_inode_getfd(name, &kbase_stream_fops, stream,
				   O_RDONLY | O_CLOEXEC);
	if (*out_fd < 0) {
		timeline_destroy(stream->timeline);
		kfree(stream);
		return -EINVAL;
	}

	return 0;
}

int kbase_stream_create_fence(int tl_fd)
{
	struct kbase_sync_stream *stream;
	struct fence_data data = { 0 };
	struct file *tl_file;
	int err;

	tl_file = fget(tl_fd);
	if (!tl_file)
		return -EBADF;

	if (tl_file->f_op != &kbase_stream_fops) {
		fput(tl_file);
		return -EBADF;
	}

	stream = tl_file->private_data;

	/* Each fence sits one step further along its stream's timeline; the
	 * atom that owns it advances the timeline by one when it completes.
	 */
	data.value = (u32)atomic_inc_return(&stream->seqno);
	strlcpy(data.name, "mali_fence", sizeof(data.name));

	err = fence_create(stream->timeline, &data);
	fput(tl_file);

	if (err)
		return err;

	return data.fence;
}

int kbase_fence_validate(int fd)
{
	struct fence *fence = sync_file_get_fence(fd);

	if (!fence)
		return -EINVAL;

	fence_put(fence);
	return 0;
}

int kbase_sync_fence_signal(struct sync_file *sync_file)
{
	struct sync_timeline *timeline;
	struct fence *fence;

	if (!sync_file || !sync_file->fence)
		return -EINVAL;

	fence = sync_file->fence;

	/* A merged fence (fence_array) has no single timeline and cannot have
	 * come from one of our streams.
	 */
	if (!fence->lock)
		return -EINVAL;

	timeline = container_of(fence->lock, struct sync_timeline, lock);
	timeline_inc(timeline, 1);

	return 0;
}
