/* linux/drivers/char/ingenic_pwm_audio.c
 *
 * Ingenic pwm_audio driver, use kernel char device framework
 *
 * Copyright (c) 2021 Ingenic
 * Author:liuhaibo <haibo.liu@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pwm.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/circ_buf.h>

#include <asm/div64.h>

//#define PWM_DBUG
#include <pwm-ingenic-v3.h>

/**
 * usage:
 * dts:
 *    添加设备节点,例如:
 *  pwm_audio {
 *      status = "okay";
 *      compatible = "ingenic,x1600-pwm-audio";
 *
 *      pwms = <&pwm 0 1>;                // 0:pwm channel  1:pwm property
 *      pwm-names = "pwm_audio_channel";  // device name
 *      pwm-audio-channel = <0>;
 *      mute-gpio = <&gpb 8 GPIO_ACTIVE_LOW INGENIC_GPIO_NOBIAS>; // speaker enable
 *  };
 *
 */

struct pwm_cfg_info {
	int mode;
	int init_level;
	int finish_level;
	int period_ns;
	int duty_ns;
	int channel;
	int clk_in;
	int buffer_size;
	int frame_size;
};

struct audio_convert {

	/*pcm format to pwm data*/
	unsigned int clk_in;
	unsigned int half;
	unsigned int base;
};

struct pcm_head_info {
	/*wav head info*/
	char reserve[20];
	short audio_format;
	short num_channels;
	unsigned int sample_rate;
	unsigned int byte_rate;
	short block_align;
	short bitsper_sample;
	unsigned int sub_chunk2_id;
	unsigned int sub_chunk2_size;
	struct audio_convert audio_convert;
};

struct ingenic_pwm_audio {
	int major;
	int minor;
	dev_t dev_num;
	int nr_devs;

	struct class *class;
	struct cdev cdev;
	struct device *dev;
	struct platform_device *pdev;

	struct mutex mutex;
	struct completion done_tx_dma;
	struct pwm_cfg_info pwm_cfg_info;
	struct pcm_head_info pcm_head_info;
	struct audio_convert audio_convert;
	struct pwm_device *pwm;
	struct ingenic_pwm_chan *chan;
	char *src_buf;

	int circ_buf_head;  // 软件填充的指针
	int circ_buf_tail;  // 硬件使用的指针
	int circ_buf_size;  // 总共的大小

	int async_stop_stream;
	int async_stop_cnt;
	struct completion stop_stream_done;

	int pwm_enabled;

	char *stream_buffer;
	dma_addr_t dma_buffer;
	unsigned int buffer_size;
	unsigned int frame_size;

	struct gpio_desc *mute_gpio;
	int mute_level;

	int do_init_silence;
	int busy;
};

enum pwm_audio_ioctl_cmd {
	PWM_AUDIO_GET_INFO,
	PWM_AUDIO_SET_INFO,
	PWM_AUDIO_CONFIG,
	PWM_AUDIO_START,
	PWM_AUDIO_STOP,
	PWM_AUDIO_PAUSE
};

static int pcm_fill_one_frame_data(struct ingenic_pwm_audio *pwm_audio, char *dst_buf);

static void dump_pcm_head_info(struct pcm_head_info *pcm_head_info)
{

	print_dbg("\n<------------------------[debug]---------------------------->\n");
	print_dbg(">>-----pcm_head_info->audio_format : %d\n", pcm_head_info->audio_format);
	print_dbg(">>-----pcm_head_info->num_channels : %d\n", pcm_head_info->num_channels);
	print_dbg(">>-----pcm_head_info->sample_rate : %d\n", pcm_head_info->sample_rate);
	print_dbg(">>-----pcm_head_info->byte_rate : %d\n", pcm_head_info->byte_rate);
	print_dbg(">>-----pcm_head_info->block_align : %d\n", pcm_head_info->block_align);
	print_dbg(">>-----pcm_head_info->bitsper_sample : %d\n", pcm_head_info->bitsper_sample);
	print_dbg(">>-----pcm_head_info->sub_chunk2_id : %d\n", pcm_head_info->sub_chunk2_id);
	print_dbg(">>-----pcm_head_info->sub_chunk2_size : %d\n", pcm_head_info->sub_chunk2_size);
	print_dbg(">>-----audio_convert->clk_in : %d\n", pcm_head_info->audio_convert.clk_in);
	print_dbg(">>-----audio_convert->half : %d\n", pcm_head_info->audio_convert.half);
	print_dbg(">>-----audio_convert->base : %x\n", pcm_head_info->audio_convert.base);
	print_dbg("\n<------------------------[debug]---------------------------->\n");
}

static void dump_pwm_cfg_info(struct pwm_cfg_info *pwm_cfg_info)
{

	print_dbg("\n<------------------------[debug]---------------------------->\n");
	print_dbg(">>-----pwm_cfg_info->mode : %d\n", pwm_cfg_info->mode);
	print_dbg(">>-----pwm_cfg_info->channel : %d\n", pwm_cfg_info->channel);
	print_dbg(">>-----pwm_cfg_info->clk_in : %d\n", pwm_cfg_info->clk_in);
	print_dbg(">>-----pwm_cfg_info->buffer_size : %d\n", pwm_cfg_info->buffer_size);
	print_dbg(">>-----pwm_cfg_info->frame_size : %d\n", pwm_cfg_info->frame_size);
	print_dbg("\n<------------------------[debug]---------------------------->\n");
}

static void set_private_data(struct file *filp, struct ingenic_pwm_audio *pwm_audio)
{
	filp->private_data = pwm_audio;
}
static struct ingenic_pwm_audio *get_private_data(struct file *filp)
{
	return filp->private_data;
}

static void dma_tx_callback(void *data)
{
	struct ingenic_pwm_audio *pwm_audio = (struct ingenic_pwm_audio *)data;
	struct ingenic_pwm_chan *chan = pwm_audio->chan;
	char *dst_buf = NULL;

	/*
	    如果出现 dma overflow了
	    1. 说明 dma 搬运的速度太快.
	    2. 软件还没来得及填充数据据，此时为了不会重复播放之前的声音，使用静音数据填充.
	    3. 最终效果，在系统特别卡顿的时候，可能出现短暂的断音，但是不会出现错音.

	    dma_tx_callback 在线程上下文，此时可以使用mutex lock。
	    注意: 需要注意dmaengine 部分的代码实现.
	*/

	if (CIRC_CNT(pwm_audio->circ_buf_head, pwm_audio->circ_buf_tail, pwm_audio->circ_buf_size) <= 1) {
		print_dbg("< pwm dma hwptr overflow <ignore> head: %d, tail: %d>\n", pwm_audio->circ_buf_head, pwm_audio->circ_buf_tail);
	}

	/*
	   原则上，每个中断回调circ_buf_tail 指针加1,相当于是时间片驱动.

	   1. 防止dma 出现overflow之后，播放旧的音频数据，影响体验。
	   2. 此处将dma消耗过的buffer，填充静音数据，即使即使出现overflow了，播放的也是静音数据.
	   3. 坏处、效率有些低，因为每次回调之后都需要填充静音数据.
	*/
	dst_buf = (char *)chan->buffer + (pwm_audio->circ_buf_tail * pwm_audio->frame_size);
	pcm_fill_one_frame_data(pwm_audio, dst_buf);

	pwm_audio->circ_buf_tail ++;
	pwm_audio->circ_buf_tail %= pwm_audio->circ_buf_size;

	complete(&pwm_audio->done_tx_dma);

	if (pwm_audio->async_stop_stream == 1 && pwm_audio->async_stop_cnt-- == 0) {
		complete(&pwm_audio->stop_stream_done);
	}
}

static int ingenic_pwm_audio_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct ingenic_pwm_audio *pwm_audio = container_of(cdev, struct ingenic_pwm_audio, cdev);
	struct pwm_cfg_info *pwm_cfg_info = &pwm_audio->pwm_cfg_info;
	struct ingenic_pwm_chan *chan = NULL;
	int ret = 0;

	print_dbg("-------------[%s]-------->\n", __func__);
	mutex_lock(&pwm_audio->mutex);

	if (pwm_audio->busy) {
		mutex_unlock(&pwm_audio->mutex);
		return -EBUSY;
	}

	set_private_data(filp, pwm_audio);
	struct device *dev = &pwm_audio->pdev->dev;
	pwm_audio->pwm = pwm_get(dev, "pwm_audio_channel");
	if (IS_ERR(pwm_audio->pwm)) {
		ret = PTR_ERR(pwm_audio->pwm);
		printk("[error]: pwm audio channel request failed!!! ret : %d\n", ret);
		mutex_unlock(&pwm_audio->mutex);
		return -EINVAL;
	}

	chan = (struct ingenic_pwm_chan *)pwm_get_chip_data(pwm_audio->pwm);
	chan->mode = pwm_cfg_info->mode;
	chan->period_ns = pwm_cfg_info->period_ns;
	chan->duty_ns = pwm_cfg_info->duty_ns;
	chan->callback = dma_tx_callback;
	chan->callback_param = pwm_audio;
	chan->buffer  = pwm_audio->stream_buffer;
	chan->buffer_dma = pwm_audio->dma_buffer;
	chan->total_buffer_size = pwm_audio->buffer_size;
	chan->frame_size    = pwm_audio->frame_size;

	pwm_audio->chan = chan;
	pwm_audio->circ_buf_head = 0;
	pwm_audio->circ_buf_tail = 0;
	pwm_audio->circ_buf_size = pwm_audio->buffer_size / pwm_audio->frame_size;

	pwm_audio->do_init_silence = 1;

	pwm_audio->src_buf = (char *)kzalloc(pwm_audio->frame_size, GFP_KERNEL);
	if (pwm_audio->src_buf == NULL) {
		printk("[error] %d kzalloc!!!\n", ret);
		ret = -1;
		goto err_alloc;
	}

	pwm_audio->busy = 1;

	mutex_unlock(&pwm_audio->mutex);

	return ret;

err_alloc:
	pwm_put(pwm_audio->pwm);
	mutex_unlock(&pwm_audio->mutex);
	return ret;
}

static int ingenic_pwm_audio_read(struct file *filp, char *user_buf, size_t count, loff_t *f_pos)
{
	print_dbg("-------------[%s]-------->\n", __func__);
	return 0;
}

static int pcm_fill_one_frame_data(struct ingenic_pwm_audio *pwm_audio, char *dst_buf)
{
	struct pcm_head_info *pcm_head_info = &pwm_audio->pcm_head_info;
	struct audio_convert *audio_convert = &pcm_head_info->audio_convert;
	int i = 0;
	int byte_width = 0;
	int *pwm_buf;
	unsigned int data = 0;

	byte_width = pcm_head_info->block_align / pcm_head_info->num_channels;

	pwm_buf = (int *)dst_buf;

	data = (audio_convert->half + 0) | ((audio_convert->half - 0) << 16);

	for (i = 0; i < pwm_audio->frame_size / sizeof(int); i ++) {
		*pwm_buf++ = data;
	}

	return pwm_audio->frame_size;
}

static int pcm_fill_silence_data(struct ingenic_pwm_audio *pwm_audio, unsigned int size, int wait)
{
	struct ingenic_pwm_chan *chan = pwm_audio->chan;
	char *dst_buf = NULL;
	int ret = 0;
	unsigned int count = size;

	do {
		if (wait && CIRC_SPACE(pwm_audio->circ_buf_head, pwm_audio->circ_buf_tail, pwm_audio->circ_buf_size) == 0) {
			reinit_completion(&pwm_audio->done_tx_dma);
			wait_for_completion_interruptible(&pwm_audio->done_tx_dma);
		}

		dst_buf = (char *)chan->buffer + (pwm_audio->circ_buf_head * pwm_audio->frame_size);
		ret = pcm_fill_one_frame_data(pwm_audio, dst_buf);
		count -= ret;

		pwm_audio->circ_buf_head ++;
		pwm_audio->circ_buf_head %= pwm_audio->circ_buf_size;

	} while (count != 0);

	return 0;

}

static int pcm_convert_pwm(struct ingenic_pwm_audio *pwm_audio, char *src_buf, char *dst_buf, int count)
{
	struct pcm_head_info *pcm_head_info = &pwm_audio->pcm_head_info;
	struct audio_convert *audio_convert = &pcm_head_info->audio_convert;
	int bitsper_sample = 0;
	int byte_width = 0;
	int i = 0;
	uint8_t pcm_symbol = 0;
	int *pwm_buf = (int *)dst_buf;
	bitsper_sample = pcm_head_info->bitsper_sample;
	byte_width = pcm_head_info->block_align / pcm_head_info->num_channels;

	print_dbg("----------------->>>src_buf : %p,dst_buf : %p, byte_width : %d,bitsper_sample : %d\n", src_buf, dst_buf, byte_width, bitsper_sample);
	switch (bitsper_sample) {
		case 16: {
			int16_t pcm_buf_tmp = 0;
			for (i = 0; i < count; i += byte_width) {
				pcm_buf_tmp = 0;
				pcm_buf_tmp = ((src_buf[i + 1] & 0xff) << 8 | (src_buf[i] & 0xff));
				pcm_symbol = ((uint16_t)pcm_buf_tmp >> (pcm_head_info->bitsper_sample - 1));
				if (pcm_symbol) {
					pcm_buf_tmp = ~pcm_buf_tmp + 1;
				}

				pcm_buf_tmp = (pcm_buf_tmp * audio_convert->half) / audio_convert->base;
				if (pcm_symbol) {
					*pwm_buf = (audio_convert->half + pcm_buf_tmp) | ((audio_convert->half - pcm_buf_tmp) << 16);
				} else {
					*pwm_buf = (audio_convert->half - pcm_buf_tmp) | ((audio_convert->half + pcm_buf_tmp) << 16);
				}
				pwm_buf++;
			}

			/* 如果一次convert 不够一帧数据，剩余的填充50% 占空比? */
			for (; i < pwm_audio->frame_size / sizeof(int) * byte_width; i += byte_width) {

				//printk("----------Last Frame not full, fill silience!\n");
				*pwm_buf++ = (audio_convert->half + 0) | ((audio_convert->half - 0) << 16);
			}
		}
		break;
		case 24: {
			int32_t pcm_buf_tmp = 0;
			uint64_t tmp = 0;
			for (i = 0; i < count; i += byte_width) {
				pcm_buf_tmp = 0;
				pcm_buf_tmp = (((src_buf[i + 3] & 0xff) << 24) | ((src_buf[i + 2] & 0xff) << 16) | \
				               ((src_buf[i + 1] & 0xff) << 8) | (src_buf[i] & 0xff));
				pcm_symbol = (((uint32_t)pcm_buf_tmp & (0x1 << 23)) >> 23);
				if (pcm_symbol) {
					pcm_buf_tmp = (uint32_t)(0x1000000 - pcm_buf_tmp); /*~pcm_buf_tmp + 1*/
				}
				tmp = (unsigned long long int)pcm_buf_tmp * (unsigned long long int)audio_convert->half * 1ULL;
				pcm_buf_tmp = do_div(tmp, (unsigned long long int)audio_convert->base);
				if (pcm_buf_tmp > audio_convert->base / 2) {
					pcm_buf_tmp = (uint32_t)tmp + 1;
				} else {
					pcm_buf_tmp = (uint32_t)tmp;
				}
				if (pcm_symbol) {
					*pwm_buf = (audio_convert->half + pcm_buf_tmp) | ((audio_convert->half - pcm_buf_tmp) << 16);
				} else {
					*pwm_buf = (audio_convert->half - pcm_buf_tmp) | ((audio_convert->half + pcm_buf_tmp) << 16);
				}
				pwm_buf++;
			}
			for (; i < pwm_audio->frame_size / sizeof(int) * byte_width; i += byte_width) {
				*pwm_buf++ = (audio_convert->half + 0) | ((audio_convert->half - 0) << 16);
			}
		}
		break;
		case 32: {
			int32_t pcm_buf_tmp = 0;
			uint64_t tmp = 0; /*unsigned long long int*/
			for (i = 0; i < count; i += byte_width) {
				pcm_buf_tmp = 0;
				pcm_buf_tmp = (((src_buf[i + 3] & 0xff) << 24) | ((src_buf[i + 2] & 0xff) << 16) | \
				               ((src_buf[i + 1] & 0xff) << 8) | (src_buf[i] & 0xff));
				pcm_symbol = ((uint32_t)pcm_buf_tmp >> (pcm_head_info->bitsper_sample - 1));
				if (pcm_symbol) {
					pcm_buf_tmp = ~pcm_buf_tmp + 1;
				}
				tmp = (unsigned long long int)pcm_buf_tmp * (unsigned long long int)audio_convert->half * 1ULL;
				pcm_buf_tmp = do_div(tmp, (unsigned long long int)audio_convert->base);
				if (pcm_buf_tmp > audio_convert->base / 2) {
					pcm_buf_tmp = (uint32_t)tmp + 1;
				} else {
					pcm_buf_tmp = (uint32_t)tmp;
				}
				if (pcm_symbol) {
					*pwm_buf = (audio_convert->half + pcm_buf_tmp) | ((audio_convert->half - pcm_buf_tmp) << 16);
				} else {
					*pwm_buf = (audio_convert->half - pcm_buf_tmp) | ((audio_convert->half + pcm_buf_tmp) << 16);
				}
				pwm_buf++;
			}
			for (; i < pwm_audio->frame_size / sizeof(int) * byte_width; i += byte_width) {
				*pwm_buf++ = (audio_convert->half + 0) | ((audio_convert->half - 0) << 16);
			}
		}
		break;
		default:
			break;
	}
	return 0;
}

static int pwm_audio_try_enable(struct ingenic_pwm_audio *pwm_audio)
{
	int ret = 0;

	if (pwm_audio->pwm_enabled == 0) {
		ret = pwm_enable(pwm_audio->pwm);
		if (ret < 0) {
			return -EINVAL;
		}
		pwm_audio->pwm_enabled = 1;

		msleep(1);
		/* Speaker enable! */
		if (IS_ERR(pwm_audio->mute_gpio)) {
			gpiod_direction_output(pwm_audio->mute_gpio, !pwm_audio->mute_level);
		}

	}

	return 0;
}

static int ingenic_pwm_audio_write(struct file *filp, const char *user_buf, size_t count, loff_t *f_pos)
{
	struct ingenic_pwm_audio *pwm_audio = get_private_data(filp);
	struct ingenic_pwm_chan *chan = pwm_audio->chan;
	char *src_buf = NULL;
	char *dst_buf = NULL;
	int ret = 0;
	int ret1 = 0;

	mutex_lock(&pwm_audio->mutex);

	/* 1. 填充完 init_silence_frames 之后，使能pwm，开始播放静音数据.*/
	if (pwm_audio->do_init_silence) {

		pcm_fill_silence_data(pwm_audio, pwm_audio->buffer_size, 0);
		ret = pwm_audio_try_enable(pwm_audio);
		if (ret < 0) {
			printk("pwm_audio enabled with error!\n");
			goto err0;
		}

		pwm_audio->do_init_silence = 0;
	}

	/*Wait Until Buffer Available*/
	if (CIRC_SPACE(pwm_audio->circ_buf_head, pwm_audio->circ_buf_tail, pwm_audio->circ_buf_size) == 0) {

		reinit_completion(&pwm_audio->done_tx_dma);
		wait_for_completion_interruptible(&pwm_audio->done_tx_dma);
	}

	src_buf = (char *)pwm_audio->src_buf;
	dst_buf = (char *)chan->buffer;
	dst_buf = dst_buf + (pwm_audio->circ_buf_head * pwm_audio->frame_size);

	ret = copy_from_user(src_buf, user_buf, count);
	if (ret == 0 && count != 0) {
		print_dbg("[warn] %d bytes be copied!!!\n", count);
		ret = count;
	} else if (count == 0) {
		printk("[error] copy from user failed!!! count : %d\n", count);
		ret = count - ret;
		goto err0;
	} else if (ret != 0) {
		printk("[error] %d bytes not be copied!!!\n", ret);
		ret = count - ret;
		goto err0;
	}
	ret1 = pcm_convert_pwm(pwm_audio, src_buf, dst_buf, count);

	if (ret1 != 0) {
		printk("[error] pcm convert pwm failed!\n");
		mutex_unlock(&pwm_audio->mutex);
		return ret1;
	}

	pwm_audio->circ_buf_head ++;
	pwm_audio->circ_buf_head %= pwm_audio->circ_buf_size;

err0:
	mutex_unlock(&pwm_audio->mutex);
	return ret;
}

static long ingenic_pwm_audio_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	struct ingenic_pwm_audio *pwm_audio = get_private_data(filp);
	struct pwm_cfg_info *pwm_cfg_info = &pwm_audio->pwm_cfg_info;
	struct pcm_head_info *pcm_head_info = &pwm_audio->pcm_head_info;
	struct ingenic_pwm_chan *chan = pwm_audio->chan;
	int ret = 0;

	mutex_lock(&pwm_audio->mutex);
	switch (cmd) {
		case PWM_AUDIO_GET_INFO:
			/*pwm audio info config*/
			pwm_audio->pwm_cfg_info.duty_ns = 5000;
			pwm_audio->pwm_cfg_info.period_ns = 10000;
			pwm_audio->pwm_cfg_info.clk_in = chan->clk_in;
			pwm_audio->pwm_cfg_info.buffer_size = pwm_audio->buffer_size;
			pwm_audio->pwm_cfg_info.frame_size = pwm_audio->frame_size;
			ret = copy_to_user((void *)args, pwm_cfg_info, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: pwm audio get info failed!!! ret : %d\n", ret);
			} else {
				dump_pwm_cfg_info(pwm_cfg_info);
			}
			break;
		case PWM_AUDIO_SET_INFO:
			/* setup PCM stream head info config*/
			memset(pcm_head_info, 0, sizeof(struct pcm_head_info));
			ret = copy_from_user(pcm_head_info, (void *)args, sizeof(struct pcm_head_info));
			if (ret) {
				printk("[error]: pwm audio get info failed!!! ret : %d\n", ret);
			} else {
				dump_pcm_head_info(pcm_head_info);
			}

			pcm_head_info->audio_convert.clk_in = pwm_cfg_info->clk_in;
			pcm_head_info->audio_convert.base = 1 << (pcm_head_info->bitsper_sample - 1);
			pcm_head_info->audio_convert.half = (pwm_cfg_info->clk_in / pcm_head_info->sample_rate) >> pcm_head_info->num_channels;
			break;
		case PWM_AUDIO_CONFIG:
			ret = pwm_config(pwm_audio->pwm, pwm_cfg_info->duty_ns, pwm_cfg_info->period_ns);
			if (ret < 0) {
				printk("[error]: pwm audio config failed!!! ret : %d\n", ret);
			}
			break;
		case PWM_AUDIO_START:
			print_dbg("---- start <TODO:UNDEF>----!\n");
			break;
		case PWM_AUDIO_STOP:
			print_dbg("---- stop <TODO:UNDEF>---!\n");

			break;
		case PWM_AUDIO_PAUSE:
			print_dbg("---- pause <TODO:UNDEF>---!\n");
			break;

		default:
			break;
	}

	mutex_unlock(&pwm_audio->mutex);

	return ret;
}

static int ingenic_pwm_audio_close(struct inode *inode, struct file *filp)
{
	struct ingenic_pwm_audio *pwm_audio = get_private_data(filp);
	int ret = 0;

	/*
	如何安全的停止数据流:
	1. 将buffer全部填充为静音数据.
	2. 设置 stop_stream 状态.
	3. 中断中消耗 静音数据.
	4. 等待静音数据消耗完成.
	需要将所有的buffer填充为静音数据
	*/
	mutex_lock(&pwm_audio->mutex);
	if (pwm_audio->pwm_enabled) {
		pcm_fill_silence_data(pwm_audio, pwm_audio->buffer_size, 1);
		reinit_completion(&pwm_audio->stop_stream_done);
		pwm_audio->async_stop_cnt = pwm_audio->circ_buf_size;   //等4个frame之后，停止播放，确保数据播放出去了.
		pwm_audio->async_stop_stream = 1;
		ret = wait_for_completion_interruptible(&pwm_audio->stop_stream_done);

		pwm_audio->async_stop_stream = 0;
		pwm_audio->async_stop_cnt = 0;

		pwm_disable(pwm_audio->pwm);

		pwm_audio->pwm_enabled = 0;

		if (IS_ERR(pwm_audio->mute_gpio)) {
			gpiod_direction_output(pwm_audio->mute_gpio, pwm_audio->mute_level);
		}
	}

	if (pwm_audio->pwm) {
		pwm_put(pwm_audio->pwm);
	}

	if (pwm_audio->src_buf) {
		kfree(pwm_audio->src_buf);
	}

	pwm_audio->busy = 0;
	mutex_unlock(&pwm_audio->mutex);

	return 0;
}

static struct file_operations ingenic_pwm_audio_ops = {
	.owner = THIS_MODULE,
	.write = ingenic_pwm_audio_write,
	.read = ingenic_pwm_audio_read,
	.open = ingenic_pwm_audio_open,
	.release = ingenic_pwm_audio_close,
	.unlocked_ioctl = ingenic_pwm_audio_ioctl,
};

static int get_param_from_dts(struct ingenic_pwm_audio *pwm_audio)
{
	struct platform_device *pdev = pwm_audio->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int channel = 0;
	int ret = 0;

	pwm_audio->pwm_cfg_info.mode = DMA_MODE_CYCLIC;

	ret = of_property_read_u32(np, "pwm-audio-channel", &channel);
	if (ret < 0) {
		dev_err(&pdev->dev, "get pwm-audio-mode from dts failed!\n");
		return ret;
	} else {
		pwm_audio->pwm_cfg_info.channel = channel;
	}

	pwm_audio->mute_gpio = devm_gpiod_get_optional(dev, "mute-gpio", GPIOD_OUT_LOW);
	if (IS_ERR(pwm_audio->mute_gpio)) {
		dev_err(dev, "pwm audio gpio invalid! %ld\n",
		        PTR_ERR(pwm_audio->mute_gpio));
		return PTR_ERR(pwm_audio->mute_gpio);
	}
	pwm_audio->mute_level = 0;
	return ret;
}

static int ingenic_pwm_audio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ingenic_pwm_audio *pwm_audio;
	dev_t devno;

	pwm_audio = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_pwm_audio), GFP_KERNEL);
	if (!pwm_audio) {
		dev_err(&pdev->dev, "alloc pwm_audio failed!\n");
		return -ENOMEM;
	}
	pwm_audio->pdev = pdev;

	ret = get_param_from_dts(pwm_audio);
	if (ret != 0) {
		dev_err(&pdev->dev, "parse dts failed!\n");
		goto __err0;
	}
	platform_set_drvdata(pdev, pwm_audio);

	pwm_audio->async_stop_stream = 0;
	pwm_audio->async_stop_cnt = 0;
	init_completion(&pwm_audio->done_tx_dma);
	init_completion(&pwm_audio->stop_stream_done);
	mutex_init(&pwm_audio->mutex);

	pwm_audio->minor = 0;
	pwm_audio->nr_devs = 1;
	ret = alloc_chrdev_region(&devno, pwm_audio->minor, pwm_audio->nr_devs, "ingenic_pwm_audio");
	if (ret) {
		dev_err(&pdev->dev, "alloc chrdev failed!\n");
		goto __err0;
	}
	pwm_audio->major = MAJOR(devno);
	pwm_audio->dev_num = MKDEV(pwm_audio->major, pwm_audio->minor);
	dev_dbg(&pdev->dev, "%s():%d >>> pwm_audio->major = %d, pwm_audio->minor = %d, pwm_audio->dev_num = %x\n",
	        __func__, __LINE__, pwm_audio->major, pwm_audio->minor, pwm_audio->dev_num);

	cdev_init(&pwm_audio->cdev, &ingenic_pwm_audio_ops);
	pwm_audio->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pwm_audio->cdev, pwm_audio->dev_num, 1);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed!\n");
		goto __err1;
	}

	pwm_audio->class = class_create("ingenic_pwm_audio");
	if (IS_ERR(pwm_audio->class)) {
		dev_err(&pdev->dev, "class_create failed!\n");
		ret = PTR_ERR(pwm_audio->class);
		goto __err2;
	}

	pwm_audio->dev = device_create(pwm_audio->class, NULL, pwm_audio->dev_num, NULL, "ingenic_pwm_audio");
	if (IS_ERR(pwm_audio->dev)) {
		dev_err(&pdev->dev, "device_create failed!\n");
		ret = PTR_ERR(pwm_audio->dev);
		goto __err3;
	}

	pwm_audio->dev->coherent_dma_mask = pdev->dev.coherent_dma_mask;

	pwm_audio->buffer_size = BUFFER_SIZE;
	pwm_audio->frame_size  = 4096; //FRAME_SIZE;
	pwm_audio->stream_buffer = dma_alloc_coherent(pwm_audio->dev, pwm_audio->buffer_size, &pwm_audio->dma_buffer, GFP_KERNEL);
	if (!pwm_audio->stream_buffer) {
		dev_err(&pdev->dev, "Failed to alloc pwm stream buffer!\n");
		ret = -ENOMEM;

		goto __err4;
	}

	dev_info(pwm_audio->dev, "Ingenic pwm audio driver init successfully!\n");

	return 0;
__err4:
	device_destroy(pwm_audio->class, pwm_audio->dev_num);
__err3:
	class_destroy(pwm_audio->class);
__err2:
	cdev_del(&pwm_audio->cdev);
__err1:
	unregister_chrdev_region(pwm_audio->dev_num, pwm_audio->nr_devs);
__err0:
	kfree(pwm_audio);
	return ret;
}

static int ingenic_pwm_audio_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_audio *pwm_audio = platform_get_drvdata(pdev);

	if (pwm_audio->stream_buffer) {
		dma_free_coherent(pwm_audio->dev, pwm_audio->buffer_size,
		                  pwm_audio->stream_buffer, pwm_audio->dma_buffer);
	}

	device_destroy(pwm_audio->class, pwm_audio->dev_num);
	class_destroy(pwm_audio->class);
	cdev_del(&pwm_audio->cdev);
	unregister_chrdev_region(pwm_audio->dev_num, pwm_audio->nr_devs);

	return 0;
}

static const struct of_device_id ingenic_pwm_audio_match[] = {
	{ .compatible = "ingenic,x1600-pwm-audio", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_pwm_audio_match);

static struct platform_driver ingenic_pwm_audio_driver = {
	.probe          = ingenic_pwm_audio_probe,
	.remove         = ingenic_pwm_audio_remove,
	.driver     = {
		.name   = "ingenic_pwm_audio",
		.owner  = THIS_MODULE,
		.of_match_table = ingenic_pwm_audio_match,
	},
};

static int __init ingenic_pwm_audio_init(void)
{
	int ret;

	ret = platform_driver_register(&ingenic_pwm_audio_driver);
	if (ret) {
		platform_driver_unregister(&ingenic_pwm_audio_driver);
	}

	return ret;
}

static void __exit ingenic_pwm_audio_exit(void)
{
	platform_driver_unregister(&ingenic_pwm_audio_driver);
}

module_init(ingenic_pwm_audio_init);
module_exit(ingenic_pwm_audio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic pwm audio driver");
MODULE_AUTHOR("haibo.liu@ingenic.com");
