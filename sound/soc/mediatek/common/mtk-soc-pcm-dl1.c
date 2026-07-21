/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_afe.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio dl1 data1 playback
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "mtk-auddrv-common.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-platform.h"
#include "mtk-auddrv-gpio.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/atomic.h>	/* forge_dl1_* counters */
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#endif

static AFE_MEM_CONTROL_T *pMemControl;
static unsigned int mPlaybackDramState;
static struct snd_dma_buffer *Dl1_Playback_dma_buf;

static DEFINE_SPINLOCK(auddrv_DLCtl_lock);

static struct device *mDev;

/* Counters for the copy path's two blind spots. mtk_pcm_copy() below skips
 * copy_from_user() when access_ok() fails but still advances u4WriteIdx
 * and u4DataRemained, and the only report of that was PRINTK_AUDDRV, which
 * compiles to nothing (DEBUG_AUDDRV is not defined in mtk-auddrv-def.h).
 * A rejected user pointer therefore gave a silent stream of the correct
 * duration, with AFE_DL1_CUR advancing and IRQs firing. These counters and
 * the pr_err below make that visible; they change no behaviour. */
static atomic_t forge_dl1_badptr = ATOMIC_INIT(0);
static atomic_t forge_dl1_copies = ATOMIC_INIT(0);

/* Program AFE_DAC_CON1[11:8] (I2S/DAC-out block rate) from the stream
 * rate, as the stock 3.10 DL1 prepare does (mt_soc_pcm_dl1.c:
 * "SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, runtime->rate)"). This 4.4
 * DL1 driver never writes that field, so it keeps whatever the previous
 * stream left, e.g. AFE_DAC_CON1 = 0x20a90a: DL1_MODE[3:0] = 0xa = 48k but
 * I2S_MODE[11:8] = 0x9 = 44.1k. Default 0 = current behaviour. */
static int forge_dl1_i2s_rate;
module_param(forge_dl1_i2s_rate, int, 0644);
MODULE_PARM_DESC(forge_dl1_i2s_rate,
		 "1 = also set AFE_DAC_CON1[11:8] I2S rate from the stream rate (3.10 oracle parity)");


/*
 *    function implementation
 */

/*void StartAudioPcmHardware(void);*/
/*void StopAudioPcmHardware(void);*/
static int mtk_soc_dl1_probe(struct platform_device *pdev);
static int mtk_soc_pcm_dl1_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_dl1_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_asoc_dl1_probe(struct snd_soc_platform *platform);

static bool mPrepareDone;

#define USE_RATE        (SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        192000
#define USE_CHANNELS_MIN     1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     8192

static struct snd_pcm_hardware mtk_pcm_dl1_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_ADV_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl1_MAX_BUFFER_SIZE,
	.period_bytes_max = Dl1_MAX_PERIOD_SIZE,
	.periods_min = SOC_NORMAL_USE_PERIODS_MIN,
	.periods_max = SOC_NORMAL_USE_PERIODS_MAX,
	.fifo_size = 0,
};

/* DL1 buffer census, read-only:
 *
 *   cat /sys/module/mtk_soc_pcm_dl1/parameters/forge_dl1_probe
 *
 * Shows whether the AFE fetches the bytes the CPU wrote: AFE_DL1_CUR
 * advancing only proves the DMA engine walks BASE..END, not that the
 * walked bytes are audio. /dev/mem is not available (CONFIG_DEVMEM is not
 * set), so the census runs in the kernel over pucVirtBufAddr, the pointer
 * mtk_pcm_copy() writes through.
 *
 * lo00_01 counts s16 samples whose low byte is 0x00 or 0x01: about 1/128
 * of samples on real 16-bit PCM, near 100% if the low byte is stuck;
 * peak_abs shows whether any amplitude survives.
 *
 * Bracketed in AudDrv_Clk_On/Off: with the audio bus clock-gated, both the
 * AFE register window and the SRAM window hang the AXI bus on this SoC
 * (see forge_audio_hold). */
static int forge_dl1_probe_get(char *buffer, const struct kernel_param *kp)
{
	AFE_MEM_CONTROL_T *ctl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL1);
	AFE_BLOCK_T *blk;
	kal_int32 size;
	int n = 0;
	unsigned int nonzero = 0, lo00_01 = 0, samples = 0;
	int peak_abs = 0;
	s16 first[8] = {0};
	u8 chunk[256];
	kal_int32 off;

	if (ctl == NULL)
		return scnprintf(buffer, PAGE_SIZE, "no DL1 mem control\n");

	blk = &ctl->rBlock;
	size = blk->u4BufferSize;

	AudDrv_Clk_On();

	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "prepared=%d dram=%u phys=0x%08x virt=%p size=%d\n",
		       mPrepareDone, mPlaybackDramState,
		       blk->pucPhysBufAddr, blk->pucVirtBufAddr, size);
	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "ring  wr=%d dmard=%d remained=%d reset=%u mask=0x%x\n",
		       blk->u4WriteIdx, blk->u4DMAReadIdx, blk->u4DataRemained,
		       blk->uResetFlag, blk->u4SampleNumMask);
	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "copy  calls=%d access_ok_fail=%d\n",
		       atomic_read(&forge_dl1_copies),
		       atomic_read(&forge_dl1_badptr));
	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "memif BASE=0x%08x CUR=0x%08x END=0x%08x MSB=0x%08x PBUF=0x%08x\n",
		       Afe_Get_Reg(AFE_DL1_BASE), Afe_Get_Reg(AFE_DL1_CUR),
		       Afe_Get_Reg(AFE_DL1_END), Afe_Get_Reg(AFE_MEMIF_MSB),
		       Afe_Get_Reg(AFE_MEMIF_PBUF_SIZE));
	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "afe   DAC_CON0=0x%08x DAC_CON1=0x%08x CONN1=0x%08x CONN2=0x%08x\n",
		       Afe_Get_Reg(AFE_DAC_CON0), Afe_Get_Reg(AFE_DAC_CON1),
		       Afe_Get_Reg(AFE_CONN1), Afe_Get_Reg(AFE_CONN2));
	n += scnprintf(buffer + n, PAGE_SIZE - n,
		       "afe   CONN_24BIT=0x%08x I2S_CON1=0x%08x MEMIF_MON0=0x%08x SGEN=0x%08x\n",
		       Afe_Get_Reg(AFE_CONN_24BIT), Afe_Get_Reg(AFE_I2S_CON1),
		       Afe_Get_Reg(AFE_MEMIF_MON0), Afe_Get_Reg(AFE_SGEN_CON0));

	/* content census over the buffer the AFE was told to fetch */
	if (blk->pucVirtBufAddr != NULL && size > 0) {
		for (off = 0; off + (kal_int32)sizeof(chunk) <= size;
		     off += (kal_int32)sizeof(chunk)) {
			unsigned int i;

			memcpy_fromio(chunk, blk->pucVirtBufAddr + off,
				      sizeof(chunk));
			for (i = 0; i < sizeof(chunk); i++)
				if (chunk[i])
					nonzero++;
			for (i = 0; i + 1 < sizeof(chunk); i += 2) {
				s16 s = (s16)(chunk[i] | (chunk[i + 1] << 8));
				int a = (s < 0) ? -(int)s : (int)s;

				if (chunk[i] == 0x00 || chunk[i] == 0x01)
					lo00_01++;
				if (a > peak_abs)
					peak_abs = a;
				if (samples < ARRAY_SIZE(first))
					first[samples] = s;
				samples++;
			}
		}
		n += scnprintf(buffer + n, PAGE_SIZE - n,
			       "census bytes_scanned=%d nonzero=%u samples=%u lo00_01=%u peak_abs=%d\n",
			       off, nonzero, samples, lo00_01, peak_abs);
		n += scnprintf(buffer + n, PAGE_SIZE - n,
			       "first8 %d %d %d %d %d %d %d %d\n",
			       first[0], first[1], first[2], first[3],
			       first[4], first[5], first[6], first[7]);
	} else {
		n += scnprintf(buffer + n, PAGE_SIZE - n,
			       "census skipped (virt=%p size=%d)\n",
			       blk->pucVirtBufAddr, size);
	}

	AudDrv_Clk_Off();

	return n;
}

static const struct kernel_param_ops forge_dl1_probe_ops = {
	.get = forge_dl1_probe_get,
};
module_param_cb(forge_dl1_probe, &forge_dl1_probe_ops, NULL, 0444);
MODULE_PARM_DESC(forge_dl1_probe,
		 "read: DL1 ring indices, memif window, and a content census of the fetched buffer");

static int mtk_pcm_dl1_stop(struct snd_pcm_substream *substream)
{
	pr_emerg("[FORGE_AUDIO] dl1 trigger STOP\n"); /* breadcrumb */
	pr_warn("%s\n", __func__);

	irq_remove_user(substream, Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, false);

	/* here start digital part */
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_I2S1_DAC);
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_I2S1_DAC_2);

	ClearMemBlock(Soc_Aud_Digital_Block_MEM_DL1);
	return 0;
}

static snd_pcm_uframes_t mtk_pcm_pointer(struct snd_pcm_substream *substream)
{
	return get_mem_frame_index(substream,
		pMemControl, Soc_Aud_Digital_Block_MEM_DL1);
}

static int mtk_pcm_dl1_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	/* struct snd_dma_buffer *dma_buf = &substream->dma_buffer; */
	int ret = 0;

	pr_emerg("[FORGE_AUDIO] dl1 hw_params\n"); /* breadcrumb */
	PRINTK_AUDDRV("mtk_pcm_dl1_params\n");

	/* runtime->dma_bytes has to be set manually to allow mmap */
	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);

	if (AllocateAudioSram(&substream->runtime->dma_addr,	&substream->runtime->dma_area,
		substream->runtime->dma_bytes, substream) == 0) {
		AudDrv_Allocate_DL1_Buffer(mDev, substream->runtime->dma_bytes,
			substream->runtime->dma_addr, substream->runtime->dma_area);
		SetHighAddr(Soc_Aud_Digital_Block_MEM_DL1, false, substream->runtime->dma_addr);
		/* pr_warn("dma_bytes = %d\n",substream->runtime->dma_bytes); */
	} else {
		substream->runtime->dma_area = Dl1_Playback_dma_buf->area;
		substream->runtime->dma_addr = Dl1_Playback_dma_buf->addr;
		SetHighAddr(Soc_Aud_Digital_Block_MEM_DL1, true, substream->runtime->dma_addr);
		set_mem_block(substream, hw_params,
			pMemControl, Soc_Aud_Digital_Block_MEM_DL1);
		mPlaybackDramState = true;
		AudDrv_Emi_Clk_On();
	}

	PRINTK_AUDDRV("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
		      substream->runtime->dma_bytes, substream->runtime->dma_area,
		      (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_pcm_dl1_hw_free(struct snd_pcm_substream *substream)
{
	pr_warn("%s substream = %p\n", __func__, substream);
	if (mPlaybackDramState == true) {
		AudDrv_Emi_Clk_Off();
		mPlaybackDramState = false;
	} else
		freeAudioSram((void *)substream);
	return 0;
}


static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static int mtk_pcm_dl1_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	mPlaybackDramState = false;

	pr_emerg("[FORGE_AUDIO] dl1 open\n"); /* breadcrumb */
	PRINTK_AUDDRV("mtk_pcm_dl1_open\n");
	mtk_pcm_dl1_hardware.buffer_bytes_max = GetPLaybackSramFullSize();

	pr_warn("mtk_pcm_dl1_hardware.buffer_bytes_max = %zu mPlaybackDramState = %d\n",
	       mtk_pcm_dl1_hardware.buffer_bytes_max, mPlaybackDramState);
	runtime->hw = mtk_pcm_dl1_hardware;

	AudDrv_Clk_On();
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_pcm_dl1_hardware,
	       sizeof(struct snd_pcm_hardware));
	pMemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL1);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);

	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_integer failed\n");

	if (ret < 0) {
		pr_err("ret < 0 mtk_soc_pcm_dl1_close\n");
		mtk_soc_pcm_dl1_close(substream);
		return ret;
	}
	/* PRINTK_AUDDRV("mtk_pcm_dl1_open return\n"); */
	return 0;
}

static int mtk_soc_pcm_dl1_close(struct snd_pcm_substream *substream)
{
	pr_emerg("[FORGE_AUDIO] dl1 close\n"); /* breadcrumb */
	pr_warn("%s\n", __func__);

	if (mPrepareDone == true) {
		/* stop DAC output */
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);
		if (GetI2SDacEnable() == false)
			SetI2SDacEnable(false);

		RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);
		EnableAfe(false);
		mPrepareDone = false;
	}
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_pcm_prepare(struct snd_pcm_substream *substream)
{
	bool mI2SWLen;
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_emerg("[FORGE_AUDIO] dl1 prepare\n"); /* breadcrumb */

	if (mPrepareDone == false) {
		pr_warn
		    ("%s format = %d SNDRV_PCM_FORMAT_S32_LE = %d SNDRV_PCM_FORMAT_U32_LE = %d\n",
		     __func__, runtime->format, SNDRV_PCM_FORMAT_S32_LE, SNDRV_PCM_FORMAT_U32_LE);
		SetMemifSubStream(Soc_Aud_Digital_Block_MEM_DL1, substream);

		if (runtime->format == SNDRV_PCM_FORMAT_S32_LE
		    || runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
			SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
						     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
					Soc_Aud_AFE_IO_Block_I2S1_DAC);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
					Soc_Aud_AFE_IO_Block_I2S1_DAC_2);
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_32BITS;
		} else {
			SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL1,
						     AFE_WLEN_16_BIT);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
					Soc_Aud_AFE_IO_Block_I2S1_DAC);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
					Soc_Aud_AFE_IO_Block_I2S1_DAC_2);
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_16BITS;
		}

		/* Stock 3.10 parity for AFE_DAC_CON1[11:8]; see
		 * forge_dl1_i2s_rate at the top of this file. */
		if (forge_dl1_i2s_rate) {
			SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, runtime->rate);
			pr_warn("[FORGE_AUDIO] dl1: I2S-out rate <- %d, AFE_DAC_CON1 = 0x%x\n",
				runtime->rate, Afe_Get_Reg(AFE_DAC_CON1));
		}

		/* start I2S DAC out */
		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
			SetI2SDacOut(substream->runtime->rate, false, mI2SWLen);
			SetI2SDacEnable(true);
		} else {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
		}

		EnableAfe(true);
		mPrepareDone = true;
	}
	return 0;
}


/* Marks the feed edge and re-arms the VBAT sampler when the external amp
 * is engaged (mtk-auddrv-gpio.c). */
extern void forge_amp_feed_mark(void);

static int mtk_pcm_dl1_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_emerg("[FORGE_AUDIO] dl1 trigger START\n"); /* breadcrumb */
	forge_amp_feed_mark();
	pr_warn("%s\n", __func__);
	/* here start digital part */

	SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_I2S1_DAC);
	SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_MEM_DL1, Soc_Aud_AFE_IO_Block_I2S1_DAC_2);

	/* here to set interrupt */
	irq_add_user(substream,
		     Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE,
		     substream->runtime->rate,
		     substream->runtime->period_size);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_DL1, runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_DL1, runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL1, true);

	EnableAfe(true);

	return 0;
}

static int mtk_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	PRINTK_AUDDRV("mtk_pcm_trigger cmd = %d\n", cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_dl1_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_dl1_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_copy(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t pos,
			void __user *dst, snd_pcm_uframes_t count)
{
	AFE_BLOCK_T *Afe_Block = NULL;
	int copy_size = 0, Afe_WriteIdx_tmp;
	unsigned long flags;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	char *data_w_ptr = (char *)dst;

	PRINTK_AUD_DL1("mtk_pcm_copy pos = %lu count = %lu\n ", pos, count);
	/* get total bytes to copy */
	count = audio_frame_to_bytes(substream, count);

	/* check which memif nned to be write */
	Afe_Block = &pMemControl->rBlock;

	PRINTK_AUD_DL1("AudDrv_write WriteIdx=0x%x, ReadIdx=0x%x, DataRemained=0x%x\n",
		       Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

	if (Afe_Block->u4BufferSize == 0) {
		pr_err("AudDrv_write: u4BufferSize=0 Error");
		return 0;
	}

	AudDrv_checkDLISRStatus();
	atomic_inc(&forge_dl1_copies);

	spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;	/* free space of the buffer */
	spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
	if (count <= copy_size) {
		if (copy_size < 0)
			copy_size = 0;
		else
			copy_size = count;
	}

	copy_size = word_size_align(copy_size);
	PRINTK_AUD_DL1("copy_size=0x%x, count=0x%x\n", copy_size, (unsigned int)count);

	if (copy_size != 0) {
		spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

		/* copy once */
		if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) {

			if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
				/* Was PRINTK_AUDDRV (compiled out): the ring
				 * still advances below, so this path gives
				 * silence with no trace at all. */
				atomic_inc(&forge_dl1_badptr);
				pr_err_ratelimited(
					"[FORGE_AUDIO] dl1 copy: access_ok REJECTED ptr=%p size=%d (ring advances, stream will be SILENT)\n",
					data_w_ptr, copy_size);
			} else {
				PRINTK_AUD_DL1
				    ("memcpy VirtBufAddr+Afe_WriteIdx= %p,data_w_ptr = %p copy_size = 0x%x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr,
				     copy_size);
				if (copy_from_user
				    ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				     copy_size)) {
					PRINTK_AUDDRV("AudDrv_write Fail copy from user\n");
					return -1;
				}
			}

			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
			Afe_Block->u4DataRemained += copy_size;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
			data_w_ptr += copy_size;
			count -= copy_size;

			PRINTK_AUD_DL1
			    ("AudDrv_write finish1, copy:%x, WriteIdx:%x,ReadIdx=%x,Remained:%x, count=%d \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained, (int)count);

		} else {
		/* copy twice */
			kal_uint32 size_1 = 0, size_2 = 0;

			size_1 = word_size_align((Afe_Block->u4BufferSize - Afe_WriteIdx_tmp));
			size_2 = word_size_align((copy_size - size_1));
			PRINTK_AUD_DL1("size_1=0x%x, size_2=0x%x\n", size_1, size_2);
			if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
				pr_err("AudDrv_write 1ptr invalid data_w_ptr=%p, size_1=%d",
				       data_w_ptr, size_1);
				pr_err("AudDrv_write u4BufferSize=%d, u4DataRemained=%d",
				       Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL1
				    ("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %p data_w_ptr = %p size_1 = %x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr,
				     size_1);
				if ((copy_from_user
				     ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				      (unsigned int)size_1))) {
					PRINTK_AUDDRV("AudDrv_write Fail 1 copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);
			Afe_Block->u4DataRemained += size_1;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);

			if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2)) {
				/* see the size_1 site above */
				atomic_inc(&forge_dl1_badptr);
				pr_err_ratelimited(
					"[FORGE_AUDIO] dl1 copy: access_ok REJECTED ptr=%p size_2=%u (ring advances, stream will be SILENT)\n",
					data_w_ptr + size_1, size_2);
			} else {

				PRINTK_AUD_DL1
				    ("mcmcpy VirtBufAddr+Afe_WriteIdx= %p,data_w_ptr+size_1 = %p size_2 = %x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
				     data_w_ptr + size_1, (unsigned int)size_2);
				if ((copy_from_user
				     ((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp),
				      (data_w_ptr + size_1), size_2))) {
					PRINTK_AUDDRV("AudDrv_write Fail 2  copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_DLCtl_lock, flags);

			Afe_Block->u4DataRemained += size_2;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DLCtl_lock, flags);
			count -= copy_size;
			data_w_ptr += copy_size;

			PRINTK_AUD_DL1
			    ("AudDrv_write finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained);
		}
	}
	return 0;
}

static int mtk_pcm_silence(struct snd_pcm_substream *substream,
			   int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	PRINTK_AUDDRV("%s\n", __func__);
	return 0;		/* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_pcm_page(struct snd_pcm_substream *substream, unsigned long offset)
{
	PRINTK_AUDDRV("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);	/* the same page */
}

static struct snd_pcm_ops mtk_afe_ops = {
	.open = mtk_pcm_dl1_open,
	.close = mtk_soc_pcm_dl1_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_pcm_dl1_params,
	.hw_free = mtk_pcm_dl1_hw_free,
	.prepare = mtk_pcm_prepare,
	.trigger = mtk_pcm_trigger,
	.pointer = mtk_pcm_pointer,
	.copy = mtk_pcm_copy,
	.silence = mtk_pcm_silence,
	.page = mtk_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops = &mtk_afe_ops,
	.pcm_new = mtk_asoc_pcm_dl1_new,
	.probe = mtk_asoc_dl1_probe,
};


static int mtk_asoc_pcm_dl1_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	PRINTK_AUDDRV("%s\n", __func__);
	return ret;
}


static int mtk_asoc_dl1_probe(struct snd_soc_platform *platform)
{
	PRINTK_AUDDRV("mtk_asoc_dl1_probe\n");
	/* allocate dram */
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_DL1,
				   Dl1_MAX_BUFFER_SIZE);
	Dl1_Playback_dma_buf = Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_DL1);
	return 0;
}

static int mtk_afe_remove(struct platform_device *pdev)
{
	PRINTK_AUDDRV("%s\n", __func__);

	AudDrv_Clk_Deinit(&pdev->dev);

	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
/*extern void *AFE_BASE_ADDRESS;*/
u32 afe_irq_number;

static const struct of_device_id mt_soc_pcm_dl1_of_ids[] = {
	/* mt_soc_pcm_dl1: The "_" needs to match the name in the device tree! */
	{.compatible = "mediatek,mt_soc_pcm_dl1",},
	{}
};

static int auddrv_get_irqline(void *dev)
{
	struct device *pdev = dev;

	if (!pdev->of_node) {
		pr_err("%s invalid of_node\n", __func__);
		return -ENODEV;
	}

	/*get afe irq num */
	afe_irq_number = irq_of_parse_and_map(pdev->of_node, 0);

	pr_warn("[ge_mt_soc_pcm_dl1] afe_irq_number=%d\n", afe_irq_number);

	if (!afe_irq_number) {
		pr_err("[ge_mt_soc_pcm_dl1] get afe_irq_number failed!!!\n");
		return -ENODEV;
	}

	return 0;
}

#endif

static void DL1GlobalVarInit(void)
{
	pMemControl = NULL;

	mPlaybackDramState = 0;

	Dl1_Playback_dma_buf = NULL;

	mDev = NULL;

	mPrepareDone = false;

}

static int mtk_soc_dl1_probe(struct platform_device *pdev)
{
	int ret = 0;

	mDev = &pdev->dev;

	PRINTK_AUDDRV("%s\n", __func__);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node) {
		dev_set_name(&pdev->dev, "%s", MT_SOC_DL1_PCM);
	} else {
		pr_err("%s invalid of_node\n", __func__);
		return -ENODEV;
	}

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

	DL1GlobalVarInit();

	/* Breadcrumbs: pr_emerg reaches console-ramoops synchronously, so on
	 * a hard AXI hang the last line names the sub-operation. */
#ifdef CONFIG_OF
	pr_emerg("[FORGE_AUDIO] dl1: clk_probe\n");
	AudDrv_Clk_probe(&pdev->dev);

#ifndef CONFIG_MTK_LEGACY
	pr_emerg("[FORGE_AUDIO] dl1: gpio_probe\n");
	AudDrv_GPIO_probe(&pdev->dev);
#endif
#endif

	pr_emerg("[FORGE_AUDIO] dl1: InitAfeControl\n");
	InitAfeControl(&pdev->dev);

#ifdef CONFIG_OF
	pr_emerg("[FORGE_AUDIO] dl1: get_irqline\n");
	auddrv_get_irqline(&pdev->dev);
	pr_emerg("[FORGE_AUDIO] dl1: Register_Aud_Irq(%d)\n", afe_irq_number);
	ret = Register_Aud_Irq(&pdev->dev, afe_irq_number);
#else
	ret = Register_Aud_Irq(&pdev->dev, MT6735_AFE_MCU_IRQ_LINE);
#endif

	pr_emerg("[FORGE_AUDIO] dl1: register_platform (irq ret=%d)\n", ret);
	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}


static struct platform_driver mtk_afe_driver = {
	.driver = {
		   .name = MT_SOC_DL1_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_dl1_of_ids,
#endif
		   },
	.probe = mtk_soc_dl1_probe,
	.remove = mtk_afe_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_dev;
#endif

static int __init mtk_soc_platform_init(void)
{
	int ret;

	PRINTK_AUDDRV("%s\n", __func__);

#ifndef CONFIG_OF

	soc_mtkafe_dev = platform_device_alloc(MT_SOC_DL1_PCM, -1);

	if (!soc_mtkafe_dev)
		return -ENOMEM;


	ret = platform_device_add(soc_mtkafe_dev);

	if (ret != 0) {
		platform_device_put(soc_mtkafe_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_afe_driver);
	return ret;

}
module_init(mtk_soc_platform_init);

static void __exit mtk_soc_platform_exit(void)
{
	PRINTK_AUDDRV("%s\n", __func__);

	platform_driver_unregister(&mtk_afe_driver);
}
module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
