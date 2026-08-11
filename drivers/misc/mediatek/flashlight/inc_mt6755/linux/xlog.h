/*
 * M3 Note 4.9: the 3.10 MTK xlog API used by the stock flashlight sources
 * (xlog_printk with an Android log priority and a tag) is gone; route it to
 * printk, keeping the kernel level the callers pass in the format.
 */
#ifndef _FORGE_LINUX_XLOG_H
#define _FORGE_LINUX_XLOG_H

#include <linux/printk.h>

#define ANDROID_LOG_UNKNOWN	0
#define ANDROID_LOG_DEFAULT	1
#define ANDROID_LOG_VERBOSE	2
#define ANDROID_LOG_DEBUG	3
#define ANDROID_LOG_INFO	4
#define ANDROID_LOG_WARN	5
#define ANDROID_LOG_WARNING	5
#define ANDROID_LOG_ERROR	6
#define ANDROID_LOG_FATAL	7

#define xlog_printk(prio, tag, fmt, ...) printk(fmt, ##__VA_ARGS__)

#endif
