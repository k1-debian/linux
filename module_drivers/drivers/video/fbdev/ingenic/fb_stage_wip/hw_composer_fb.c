#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>

#include "dpu_reg.h"
#include "hw_composer.h"
#include "hw_composer_fb.h"

#include <libdmmu.h>

static int hw_compfb_update_vidmem(struct hw_compfb_device *compfb);
static int hw_compfb_update_laycfg_by_var_ext(struct hw_compfb_device *compfb, int layer, struct ingenicfb_var_screeninfo_extended *var_ext);

const struct fb_fix_screeninfo compfb_layer_fix[]  = {
	{
		.id = "layer0",
		.type = FB_TYPE_PACKED_PIXELS,
		.visual = FB_VISUAL_TRUECOLOR,
		.xpanstep = 0,
		.ypanstep = 1,
		.ywrapstep = 0,
		.accel = FB_ACCEL_NONE,
	},
	{
		.id = "layer1",
		.type = FB_TYPE_PACKED_PIXELS,
		.visual = FB_VISUAL_TRUECOLOR,
		.xpanstep = 0,
		.ypanstep = 1,
		.ywrapstep = 0,
		.accel = FB_ACCEL_NONE,
	},
	{
		.id = "layer2",
		.type = FB_TYPE_PACKED_PIXELS,
		.visual = FB_VISUAL_TRUECOLOR,
		.xpanstep = 0,
		.ypanstep = 1,
		.ywrapstep = 0,
		.accel = FB_ACCEL_NONE,
	},
	{
		.id = "layer3",
		.type = FB_TYPE_PACKED_PIXELS,
		.visual = FB_VISUAL_TRUECOLOR,
		.xpanstep = 0,
		.ypanstep = 1,
		.ywrapstep = 0,
		.accel = FB_ACCEL_NONE,
	},
};

static struct compfb_colormode compfb_colormodes[] = {
	{
		.mode = LAYER_CFG_FORMAT_RGB888,
		.name = "rgb888",
		.bits_per_pixel = 32,
		.nonstd = 0,
#ifdef CONFIG_FB_FORMAT_X8B8G8R8
		.color = LAYER_CFG_COLOR_BGR,
#else
		.color = LAYER_CFG_COLOR_RGB,
#endif
		.red    = { .length = 8, .offset = 16, .msb_right = 0 },
		.green  = { .length = 8, .offset = 8, .msb_right = 0 },
		.blue   = { .length = 8, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = LAYER_CFG_FORMAT_ARGB8888,
		.name = "argb888",
		.bits_per_pixel = 32,
		.nonstd = 0,
		.color = LAYER_CFG_COLOR_RGB,
		.red    = { .length = 8, .offset = 16, .msb_right = 0 },
		.green  = { .length = 8, .offset = 8, .msb_right = 0 },
		.blue   = { .length = 8, .offset = 0, .msb_right = 0 },
		.transp = { .length = 8, .offset = 24, .msb_right = 0 },
	}, {
		.mode = LAYER_CFG_FORMAT_RGB555,
		.name = "rgb555",
		.color = LAYER_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 10, .msb_right = 0 },
		.green  = { .length = 5, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = LAYER_CFG_FORMAT_ARGB1555,
		.name = "argb1555",
		.color = LAYER_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 10, .msb_right = 0 },
		.green  = { .length = 5, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 1, .offset = 15, .msb_right = 0 },
	}, {
		.mode = LAYER_CFG_FORMAT_RGB565,
		.name = "rgb565",
		.color = LAYER_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 11, .msb_right = 0 },
		.green  = { .length = 6, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = LAYER_CFG_FORMAT_YUV422,
		.name = "yuv422",
		.bits_per_pixel = 16,
		.nonstd = LAYER_CFG_FORMAT_YUV422,
	}, {
		.mode = LAYER_CFG_FORMAT_NV12,
		.name = "nv12",
		.bits_per_pixel = 12,
		.nonstd = LAYER_CFG_FORMAT_NV12,
	}, {
		.mode = LAYER_CFG_FORMAT_NV21,
		.name = "nv21",
		.bits_per_pixel = 12,
		.nonstd = LAYER_CFG_FORMAT_NV21,
	},
};
#if 0
static struct compfb_colormode compfb_wback_colormodes[] = {
	{
		.mode = DC_WB_FORMAT_888,
		.name = "rgb888",
		.color = LAYER_CFG_COLOR_RGB, //TODO
		.bits_per_pixel = 32,
		.nonstd = 0,
		.color = LAYER_CFG_COLOR_RGB,
		.red    = { .length = 8, .offset = 16, .msb_right = 0 },
		.green  = { .length = 8, .offset = 8, .msb_right = 0 },
		.blue   = { .length = 8, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = DC_WB_FORMAT_8888,
		.name = "argb888",
		.bits_per_pixel = 32,
		.nonstd = 0,
		.color = LAYER_CFG_COLOR_RGB,
		.red    = { .length = 8, .offset = 16, .msb_right = 0 },
		.green  = { .length = 8, .offset = 8, .msb_right = 0 },
		.blue   = { .length = 8, .offset = 0, .msb_right = 0 },
		.transp = { .length = 8, .offset = 24, .msb_right = 0 },
	}, {
		.mode = DC_WB_FORMAT_555,
		.name = "rgb555",
		.color = LAYER_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 10, .msb_right = 0 },
		.green  = { .length = 5, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = DC_WB_FORMAT_565,
		.name = "rgb565",
		.color = LAYER_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 11, .msb_right = 0 },
		.green  = { .length = 6, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	},
};
#endif

/* export hw_composer layers to /dev/fbX */

static int compfb_open(struct fb_info *info, int user)
{
	return 0;
}
static int compfb_release(struct fb_info *info, int user)
{
	return 0;
}

static int compfb_set_par(struct fb_info *info)
{
	//TODO:

	return 0;
}

static void compfb_videomode_to_var(struct fb_var_screeninfo *var,
                                    const struct fb_videomode *mode)
{
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = mode->vmode & FB_VMODE_MASK;
	var->pixclock = mode->pixclock;
}

static void export_info_to_var(struct fb_var_screeninfo *var,
                               struct export_info *info, int layer)
{
	struct ingenicfb_frm_cfg *frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];

	int nframes = info->layer_frames[layer];

	if (var->xres != lay_cfg->source_w) {
		var->xres = lay_cfg->source_w;
		var->xres_virtual = lay_cfg->source_w;
		var->xoffset = 0;
	}

	if (var->yres != lay_cfg->source_h) {
		int old_yres = var->yres;
		var->yres = lay_cfg->source_h;
		if (old_yres == 0) {
			var->yoffset = 0;
		} else {
			// From old var to new var yoffset.
			var->yoffset = var->yoffset / old_yres * var->yres;
		}
		var->yres_virtual = var->yres * nframes;
	}
}

static void compfb_colormode_to_var(struct fb_var_screeninfo *var,
                                    struct compfb_colormode *color)
{
	var->bits_per_pixel = color->bits_per_pixel;
	var->nonstd = color->nonstd;
	var->red = color->red;
	var->green = color->green;
	var->blue = color->blue;
	var->transp = color->transp;
}

static bool cmp_var_to_colormode(struct fb_var_screeninfo *var,
                                 struct compfb_colormode *color)
{
	bool cmp_component(struct fb_bitfield * f1, struct fb_bitfield * f2) {
		return f1->length == f2->length &&
		       f1->offset == f2->offset &&
		       f1->msb_right == f2->msb_right;
	}

	if (var->bits_per_pixel == 0 ||
	    var->red.length == 0 ||
	    var->blue.length == 0 ||
	    var->green.length == 0) {
		return 0;
	}

	return var->bits_per_pixel == color->bits_per_pixel &&
	       cmp_component(&var->red, &color->red) &&
	       cmp_component(&var->green, &color->green) &&
	       cmp_component(&var->blue, &color->blue) &&
	       cmp_component(&var->transp, &color->transp);
}

static int compfb_check_colormode(struct fb_var_screeninfo *var, uint32_t *mode)
{
	int i;

	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(compfb_colormodes); ++i) {
			struct compfb_colormode *m = &compfb_colormodes[i];
			if (var->nonstd == m->nonstd) {
				compfb_colormode_to_var(var, m);
				*mode = m->mode;
				return 0;
			}
		}

		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(compfb_colormodes); ++i) {
		struct compfb_colormode *m = &compfb_colormodes[i];
		if (cmp_var_to_colormode(var, m)) {
			compfb_colormode_to_var(var, m);
			*mode = m->mode;
			return 0;
		}
	}
	/* To support user libraries that only support RGB format */
	for (i = 0; i < ARRAY_SIZE(compfb_colormodes); ++i) {
		struct compfb_colormode *m = &compfb_colormodes[i];
		if (var->bits_per_pixel == m->bits_per_pixel) {
			compfb_colormode_to_var(var, m);
			*mode = m->mode;
			return 0;
		}
	}

	return -EINVAL;
}

static int compfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct hw_compfb_device *compfb = info->par;
	struct fb_videomode *mode = compfb->active_video_mode;
	uint32_t colormode;
	int ret;

	compfb_videomode_to_var(var, mode);

	ret = compfb_check_colormode(var, &colormode);
	if (ret) {
		dev_err(info->dev, "Check colormode failed!\n");
		return  ret;
	}

	return 0;
}

static inline uint32_t convert_color_to_hw(unsigned val, struct fb_bitfield *bf)
{
	return (((val << bf->length) + 0x7FFF - val) >> 16) << bf->offset;
}

static int compfb_setcolreg(unsigned regno, unsigned red, unsigned green,
                            unsigned blue, unsigned transp, struct fb_info *fb)
{
	if (regno >= 16) {
		return -EINVAL;
	}

	((uint32_t *)(fb->pseudo_palette))[regno] =
	    convert_color_to_hw(red, &fb->var.red) |
	    convert_color_to_hw(green, &fb->var.green) |
	    convert_color_to_hw(blue, &fb->var.blue) |
	    convert_color_to_hw(transp, &fb->var.transp);

	return 0;
}

static int compfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct hw_compfb_device *compfb = info->par;
	struct comp_setup_info *comp_info = &compfb->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct ingenicfb_var_screeninfo_extended *var_ext = NULL;
	int ret = 0;
	int layer = -1;
	int next_frm = 0;
	int i = 0;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (info == compfb->fbs[i]) {
			layer = i;
			break;
		}
	}

	if (var->xoffset != 0 || layer < 0) {
		return -EINVAL;
	}

	var_ext = &compfb->var_ext[layer];
	ret = copy_from_user(var_ext, (uint32_t *)var->reserved[0], sizeof(struct ingenicfb_var_screeninfo_extended));
	if (ret < 0) {
		return -EFAULT;
	}
	hw_composer_lock(compfb->comp_ctx);
	/*可能和unexport产生冲突，当comp_ctx被释放时，此时应该返回非法.*/
	if (!compfb->comp_ctx) {
		hw_composer_unlock(compfb->comp_ctx);
		return -EINVAL;
	}

	next_frm = var->yoffset / var->yres;

	/*更新lay_cfg信息.*/
	/*修改layer信息，需要加锁.*/

	hw_compfb_update_vidmem(compfb);

	ret = hw_compfb_update_laycfg_by_var_ext(compfb, layer, var_ext);
	if (ret < 0) {
		hw_composer_unlock(compfb->comp_ctx);
		return ret;
	}

	if (compfb->vidmem_phys[layer][next_frm]) {
		lay_cfg[layer].addr[0] = compfb->vidmem_phys[layer][next_frm];
		lay_cfg[layer].uv_addr[0] = compfb->vidmem_phys[layer][next_frm] + lay_cfg[layer].stride * lay_cfg[layer].source_h;
	} else {
		dev_err(compfb->dev, "Error address for pan display!, please check\n");
	}

	if (compfb->fps[layer] == 0) {
		ktime_t now = ktime_get();
		compfb->fps_ts_start[layer] = now;
	}

	compfb->fps[layer]++;

	if (compfb->fps[layer] >= 120) {
		ktime_t now = ktime_get();

		u64 diff_ms = ktime_ms_delta(now, compfb->fps_ts_start[layer]);
		int fps = 120 * 1000 / (u32)diff_ms;

		printk("layer: %d, fps: %d, cost: %lld\n", layer, fps, diff_ms);
		compfb->fps[layer] = 0;
	}

	compfb->comp_info.layer_to_update[layer] = 1;
	compfb->comp_info.layer_update_frame[layer] = next_frm;

#ifdef CONFIG_FB_USING_CACHABLE
	dma_cache_wback_inv((phys_addr_t)compfb->vidmem[layer][next_frm], compfb->comp_info.layer_framesize[layer]);
#endif

	/*更新comp_setup_info 并且启动 composer. */
	hw_composer_setup(compfb->comp_ctx, &compfb->comp_info);
	hw_composer_start(compfb->comp_ctx);

#if 0
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
#endif

	hw_composer_unlock(compfb->comp_ctx);

	return 0;
}

static int compfb_wait_for_vsync(struct fb_info *info, int layer, int vsync)
{
	struct hw_compfb_device *compfb = info->par;

	struct fb_var_screeninfo *var = &info->var;
	struct buffer_addr_area next_addr_area;

	unsigned int next_offset = var->yoffset + var->yres;

	next_offset = next_offset >= var->yres_virtual ? 0 : next_offset;
	int next_frm = next_offset / var->yres;

	next_addr_area.start[0] = compfb->vidmem_phys[layer][next_frm];
	next_addr_area.end[0] = next_addr_area.start[0] + compfb->comp_info.layer_framesize[layer];
	next_addr_area.cnt = 1;

	//printk("wait for index: %d var->yoffset:%d\n", var->yoffset, var->yoffset / var->yres);
	hw_composer_wait_layer_vsync(compfb->comp_ctx, layer, &next_addr_area);

#if 0
	hw_composer_wait_layer_updated(compfb->comp_ctx, layer);
#endif

	return 0;
}

static int compfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	//void __user *argp = (void __user *)arg;
	struct hw_compfb_device *compfb = info->par;
	int layer = -1;
	int i = 0;

	void __user *argp = (void __user *)arg;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (info == compfb->fbs[i]) {
			layer = i;
			break;
		}
	}

	switch (cmd) {
		/*composer 在调用hw_composer_start时,能保证vsync, 此时不需要做处理.*/
		case FBIO_WAITFORVSYNC: {
			unsigned int vsync;
			if (unlikely(copy_from_user(&vsync, argp, sizeof(int)))) {
				return -EFAULT;
			}

			return compfb_wait_for_vsync(info, layer, vsync);
		}
		break;
		default:
			break;
	}

	return 0;
}

static struct fb_ops compfb_layerx_ops = {
	.owner      = THIS_MODULE,
	.fb_open    = compfb_open,
	.fb_release     = compfb_release,
	.fb_check_var   = compfb_check_var,
	.fb_set_par     = compfb_set_par,
	.fb_setcolreg   = compfb_setcolreg,
	.fb_pan_display = compfb_pan_display,
	.fb_write   = ingenicfb_write,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_ioctl   = compfb_ioctl,
	.fb_mmap    = ingenicfb_mmap,
};

#if 0
static ssize_t
compfb_wback_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	struct hw_compfb_device *compfb = info->par;
	u8 *buffer, *dst;
	u8 __iomem *src;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	int ret = 0;

	if (!info || ! info->screen_base) {
		return -ENODEV;
	}

	if (info->state != FBINFO_STATE_RUNNING) {
		return -EPERM;
	}

	/*Wait wback complete.*/
	ret = hw_composer_start(compfb->comp_ctx);
	if (ret < 0) {
		return ret;
	}

	return 0;

	total_size = info->screen_size;

	if (total_size == 0) {
		total_size = info->fix.smem_len;
	}

	if (p >= total_size) {
		return 0;
	}

	if (count >= total_size) {
		count = total_size;
	}

	if (count + p > total_size) {
		count = total_size - p;
	}

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
	                 GFP_KERNEL);
	if (!buffer) {
		return -ENOMEM;
	}

	src = (u8 __iomem *)(info->screen_base + p);

	if (info->fbops->fb_sync) {
		info->fbops->fb_sync(info);
	}

	while (count) {
		c  = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		dst = buffer;
		fb_memcpy_fromio(dst, src, c);
		dst += c;
		src += c;

		if (copy_to_user(buf, buffer, c)) {
			err = -EFAULT;
			break;
		}
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (err) ? err : cnt;
}

static struct fb_ops compfb_wback_ops = {
	.owner = THIS_MODULE,
	.fb_open = compfb_open,
	.fb_release = compfb_release,
	.fb_read = compfb_wback_read,
	.fb_check_var = compfb_check_var,
	.fb_set_par = compfb_set_par,
	.fb_setcolreg = compfb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = ingenicfb_mmap,
};

static int hw_compfb_directout_comp_info(struct hw_compfb_device *compfb)
{
	struct comp_setup_info *comp_info = &compfb->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct fb_videomode *mode = compfb->active_video_mode;

	compfb->comp_ctx->block = 0;    //使用不阻塞的方式.
	comp_info->nframes = 1;
	comp_info->out_mode = COMP_DIRECT_OUT;

	frm_cfg->width = mode->xres;
	frm_cfg->height = mode->yres;
	frm_cfg->wback_info.en = 0; //关闭写回功能.

	return 0;
}

static int hw_compfb_wback_comp_info(struct hw_compfb_device *compfb)
{
	struct comp_setup_info *comp_info = &compfb->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct fb_videomode *mode = compfb->active_video_mode;

	compfb->comp_ctx->block = 1;    //使用不阻塞的方式.
	comp_info->nframes = 1;
	//comp_info->out_mode = COMP_WRITE_BACK;

	frm_cfg->width = mode->xres;
	frm_cfg->height = mode->yres;

	frm_cfg->wback_info.addr = compfb->wback_phys;
	frm_cfg->wback_info.stride = 1280 * 2;
	frm_cfg->wback_info.fmt = compfb->wb_color_mode->color;

	frm_cfg->wback_info.en = 1; //关闭写回功能.

	return 0;
}
#endif
static int hw_compfb_update_laycfg_by_var_ext(struct hw_compfb_device *compfb, int layer, struct ingenicfb_var_screeninfo_extended *var_ext)
{
	int i = 0;
	struct comp_setup_info *comp_info = &compfb->comp_info;
	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = &frm_cfg->lay_cfg[layer];
	struct ingenicfb_lay_cfg *update_lay_cfg = &var_ext->lay_cfg;
	struct ingenicfb_frm_cfg *info_frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *info_lay_cfg = &info_frm_cfg->lay_cfg[layer];

	if (var_ext->magic != VAR_INFO_EXT_MAGIC && var_ext->use_extended != 1) {
		return 0;
	}
	if (!update_lay_cfg->lay_en) {
		return 0;
	}

	lay_cfg->lay_en        = update_lay_cfg->lay_en;
	if ((update_lay_cfg->source_w * update_lay_cfg->source_h) > (info->layer_frame_size_w[layer] * info->layer_frame_size_h[layer])) {
		lay_cfg->source_w = info->layer_frame_size_w[layer];
		lay_cfg->source_h = info->layer_frame_size_h[layer];
		printk("[Warning] src_size should smaller than frame_size");
		return -EINVAL;
	} else {
		lay_cfg->source_w = update_lay_cfg->source_w;
		lay_cfg->source_h = update_lay_cfg->source_h;
	}

	lay_cfg->scale_w    = update_lay_cfg->scale_w;
	lay_cfg->scale_h    = update_lay_cfg->scale_h;

	lay_cfg->g_alpha_en     = update_lay_cfg->g_alpha_en;
	lay_cfg->g_alpha_val    = update_lay_cfg->g_alpha_val;
	lay_cfg->disp_pos_x = update_lay_cfg->disp_pos_x;
	lay_cfg->disp_pos_y = update_lay_cfg->disp_pos_y;
	lay_cfg->src_crop_x = update_lay_cfg->src_crop_x;
	lay_cfg->src_crop_y = update_lay_cfg->src_crop_y;
	lay_cfg->src_crop_w = update_lay_cfg->src_crop_w;
	lay_cfg->src_crop_h = update_lay_cfg->src_crop_h;
	lay_cfg->format     = update_lay_cfg->format;
	lay_cfg->color      = update_lay_cfg->color;
	lay_cfg->lay_z_order    = update_lay_cfg->lay_z_order;
	if (update_lay_cfg->stride) {
		lay_cfg->stride   = update_lay_cfg->stride;
	} else {
		lay_cfg->stride   = lay_cfg->source_w;
	}
	if (update_lay_cfg->uv_stride) {
		lay_cfg->uv_stride = update_lay_cfg->uv_stride;
	} else {
		lay_cfg->uv_stride = lay_cfg->source_w;
	}
	info_lay_cfg->lay_en = lay_cfg->lay_en;
	info_lay_cfg->source_w = lay_cfg->source_w;
	info_lay_cfg->source_h = lay_cfg->source_h;
	info_lay_cfg->scale_w = lay_cfg->scale_w;
	info_lay_cfg->scale_h = lay_cfg->scale_h;
	info_lay_cfg->g_alpha_en = lay_cfg->g_alpha_en;
	info_lay_cfg->g_alpha_val = lay_cfg->g_alpha_val;
	info_lay_cfg->disp_pos_x = lay_cfg->disp_pos_x;
	info_lay_cfg->disp_pos_y = lay_cfg->disp_pos_y;
	info_lay_cfg->src_crop_x = lay_cfg->src_crop_x;
	info_lay_cfg->src_crop_y = lay_cfg->src_crop_y;
	info_lay_cfg->src_crop_w = lay_cfg->src_crop_w;
	info_lay_cfg->src_crop_h = lay_cfg->src_crop_h;
	info_lay_cfg->format = lay_cfg->format;
	info_lay_cfg->color = lay_cfg->color;
	info_lay_cfg->lay_z_order = lay_cfg->lay_z_order;
	info_lay_cfg->stride = lay_cfg->stride;
	info_lay_cfg->uv_stride = lay_cfg->uv_stride;
	for (i = 0; i < ARRAY_SIZE(compfb_colormodes); i++) {
		if (lay_cfg->format == compfb_colormodes[i].mode) {
			info->layer_color_mode[layer] = i;
		}
	}
	return 0;
}

static int hw_compfb_update_comp_info(struct hw_compfb_device *compfb)
{
	struct comp_setup_info *comp_info = &compfb->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct export_info *info = &compfb->export_info;
	struct ingenicfb_frm_cfg *update_frm_cfg = &info->frm_cfg;
	struct ingenicfb_lay_cfg *update_lay_cfg = update_frm_cfg->lay_cfg;
	int i = 0;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		lay_cfg[i].lay_z_order  = update_lay_cfg[i].lay_z_order; /* top */
		lay_cfg[i].lay_en   = update_lay_cfg[i].lay_en;
		lay_cfg[i].source_w     = update_lay_cfg[i].source_w;
		lay_cfg[i].source_h     = update_lay_cfg[i].source_h;
		lay_cfg[i].disp_pos_x   = update_lay_cfg[i].disp_pos_x;
		lay_cfg[i].disp_pos_y   = update_lay_cfg[i].disp_pos_y;

		lay_cfg[i].scale_w  = update_lay_cfg[i].scale_w;
		lay_cfg[i].scale_h  = update_lay_cfg[i].scale_h;

		lay_cfg[i].src_crop_x   = update_lay_cfg[i].src_crop_x;
		lay_cfg[i].src_crop_y   = update_lay_cfg[i].src_crop_y;

		lay_cfg[i].src_crop_w   = update_lay_cfg[i].src_crop_w;
		lay_cfg[i].src_crop_h   = update_lay_cfg[i].src_crop_h;

		lay_cfg[i].domain_multi   = update_lay_cfg[i].domain_multi;
		lay_cfg[i].g_alpha_en   = update_lay_cfg[i].g_alpha_en;
		lay_cfg[i].g_alpha_val  = update_lay_cfg[i].g_alpha_val;
		lay_cfg[i].stride   = update_lay_cfg[i].stride;
		lay_cfg[i].uv_stride    = update_lay_cfg[i].uv_stride;

	}

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (compfb->export_info.layer_exported[i] == 1) {

			/*update lay cfg.*/
			lay_cfg[i].color  = update_lay_cfg[i].color
			                    = compfb->color_modes[i]->color;
			lay_cfg[i].format = update_lay_cfg[i].format
			                    = compfb->color_modes[i]->mode;

		} else {
			lay_cfg[i].color = 0;
			lay_cfg[i].format = 0;
			lay_cfg[i].addr[0] = 0;
			lay_cfg[i].uv_addr[0] = 0;
		}

		if (update_lay_cfg[i].addr[0]) {
			lay_cfg[i].addr[0] = update_lay_cfg[i].addr[0];
			lay_cfg[i].uv_addr[0] = update_lay_cfg[i].uv_addr[0];
			lay_cfg[i].tlb_en = update_lay_cfg[i].tlb_en =
			                        update_lay_cfg[i].addr[0] < 0x80000000 ? 1 : 0;
		} else {
			lay_cfg[i].addr[0] = lay_cfg[i].addr[0] == 0 ? compfb->vidmem_phys[i][0] : lay_cfg[i].addr[0];
			lay_cfg[i].uv_addr[0] = lay_cfg[i].addr[0] + lay_cfg[i].stride * lay_cfg[i].source_h;
		}

	}

	return 0;
}

static int hw_compfb_update_fb_info(struct hw_compfb_device *compfb)
{
	struct export_info *info = &compfb->export_info;
	int i;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		int color_mode;
		if (!compfb->fbs[i]) {
			continue;
		}

		/*update fb infos*/
		export_info_to_var(&compfb->fbs[i]->var, info, i);

		color_mode = info->layer_color_mode[i];
		/*update fb_info var color mode*/
		compfb->color_modes[i] = &compfb_colormodes[color_mode];

		compfb_colormode_to_var(&compfb->fbs[i]->var, compfb->color_modes[i]);

		compfb->fbs[i]->fix.line_length = (compfb->fbs[i]->var.bits_per_pixel * compfb->fbs[i]->var.xres) >> 3;
	}
	return 0;

}

int hw_compfb_update(struct hw_compfb_device *compfb)
{

	hw_composer_lock(compfb->comp_ctx);

	/*lock*/
	hw_compfb_update_fb_info(compfb);

	hw_compfb_update_vidmem(compfb);

	hw_compfb_update_comp_info(compfb);

	hw_composer_setup(compfb->comp_ctx, &compfb->comp_info);
	hw_composer_start(compfb->comp_ctx);

	hw_composer_unlock(compfb->comp_ctx);
	return 0;
}

static int hw_compfb_export_layer(struct hw_compfb_device *compfb, int layer)
{
	struct export_info *info = &compfb->export_info;
	int color_mode = info->layer_color_mode[layer];
	struct compfb_colormode *fb_color_mode = &compfb_colormodes[color_mode];
	struct fb_info *fb_layer = NULL;
	unsigned int frame_size = info->layer_frame_size_w[layer] * info->layer_frame_size_h[layer] * ((fb_color_mode->bits_per_pixel + 7) / 8); //TODO: rgb申请的buffer， 32bit 每个pixel, 对于nv12的只需要 w * h * 3 / 2. 能够节省很多空间.

	int nframes = info->layer_frames[layer];
	unsigned int buff_size = 0;

	int ret = 0;

	/*限制最大帧数，并且更新到export info.*/
	info->layer_frames[layer] = nframes = nframes > CONFIG_FB_INGENIC_NR_FRAMES ? CONFIG_FB_INGENIC_NR_FRAMES : nframes;
	buff_size = nframes * frame_size;

	compfb->vidmem[layer][0] = dma_alloc_coherent(compfb->dev, buff_size, &compfb->vidmem_phys[layer][0], GFP_KERNEL);

	if (compfb->vidmem[layer][0] == NULL) {
		dev_err(compfb->dev, "Failed to alloc vidmem for layer %d, buff_size: %d\n", layer, buff_size);
		//TODO error handle....
	}
	compfb->vidmem_size[layer] = buff_size;

	compfb->fbs[layer] = framebuffer_alloc(sizeof(struct hw_compfb_device), compfb->dev);
	if (compfb->fbs[layer] == NULL) {
		dev_err(compfb->dev, "Failed to alloc framebuffer for layer: %d\n", layer);
		return -ENOMEM;
	}

	compfb->fbs[layer]->par = compfb;

	fb_layer = compfb->fbs[layer];

	//fb_videomode_to_modelist(compfb->active_video_mode, 1, &fb_layer->modelist);
	compfb_videomode_to_var(&fb_layer->var, compfb->active_video_mode);
	export_info_to_var(&fb_layer->var, info, layer);

	fb_layer->fbops = &compfb_layerx_ops;
	fb_layer->flags = 0;
	fb_layer->var.width = compfb->panel->width; //TODO, 这里指尺寸.
	fb_layer->var.height = compfb->panel->height;

	compfb->color_modes[layer] = &compfb_colormodes[color_mode];
	compfb_colormode_to_var(&fb_layer->var, compfb->color_modes[layer]);

	fb_layer->fix           = compfb_layer_fix[layer];
	fb_layer->fix.line_length   = (fb_layer->var.bits_per_pixel * fb_layer->var.xres) >> 3;
	fb_layer->fix.smem_start    = compfb->vidmem_phys[layer][0];
	fb_layer->fix.smem_len      = compfb->vidmem_size[layer];
	fb_layer->screen_size       = fb_layer->fix.line_length * fb_layer->var.yres;
	fb_layer->screen_base       = compfb->vidmem[layer][0];
	fb_layer->pseudo_palette    = compfb->pseudo_palette;
	ret = register_framebuffer(fb_layer);
	if (ret) {
		dev_err(compfb->dev, "Failed to register framebuffer layer : %d\n", layer);
	}

	return ret;
}

static int hw_compfb_update_vidmem(struct hw_compfb_device *compfb)
{
	/*update vidmem info.*/
	struct export_info *info = &compfb->export_info;
	struct comp_setup_info *comp_info = &compfb->comp_info;
	int i, j;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		struct fb_info *fb_layer = compfb->fbs[i];
		int nframes = info->layer_frames[i];
		unsigned int layer_size;
		unsigned char *vidmem = compfb->vidmem[i][0];
		unsigned int vidmem_phys = compfb->vidmem_phys[i][0];

		if (!fb_layer) {
			continue;
		}
		layer_size = fb_layer->fix.line_length * fb_layer->var.yres;
		for (j = 0; j < nframes; j++) {
			compfb->vidmem[i][j] = vidmem + j * layer_size;
			compfb->vidmem_phys[i][j] = vidmem_phys + j * layer_size;
		}

		comp_info->layer_frames[i] = info->layer_frames[i];
		comp_info->layer_framesize[i] = layer_size;
	}

	return 0;
}

int hw_compfb_export2(struct hw_compfb_device *compfb, struct export_info *info)
{

	int i = 0;
	int ret = 0;

	if (compfb->exported) {
		return -EBUSY;
	}

	compfb->comp_ctx = hw_composer_create(compfb);
	if (IS_ERR_OR_NULL(compfb->comp_ctx)) {
		dev_err(compfb->dev, "Faield to create ctx for compfb!\n");
		return -ENOMEM;
	}

	hw_composer_lock(compfb->comp_ctx);

	/* export layer. */
	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (info->layer_exported[i] == 1) {
			ret = hw_compfb_export_layer(compfb, i);
			if (ret < 0) {
				dev_err(compfb->dev, "Failed to export layer %d\n", i);
			}
		}
	}

	/* export wback. */

	compfb->exported = 1;

	hw_composer_unlock(compfb->comp_ctx);

	return ret;

}

void hw_compfb_unexport(struct hw_compfb_device *compfb)
{
	int i = 0;

	if (compfb->exported == 0) {
		return;
	}

	hw_composer_lock(compfb->comp_ctx);

	if (compfb->comp_ctx) {
		hw_composer_stop(compfb->comp_ctx);
		hw_composer_destroy(compfb->comp_ctx);
		compfb->comp_ctx = NULL;
	}
	hw_composer_unlock(compfb->comp_ctx);

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (compfb->fbs[i]) {
			unregister_framebuffer(compfb->fbs[i]);
			framebuffer_release(compfb->fbs[i]);
			compfb->fbs[i] = NULL;

			if (compfb->vidmem[i][0]) {
				dma_free_coherent(compfb->dev, compfb->vidmem_size[i], compfb->vidmem[i][0], compfb->vidmem_phys[i][0]);
			}
		}
	}

	compfb->exported = 0;
}

/* 主设备初始化compfb数据结构. */
struct hw_compfb_device *hw_compfb_init(void *data)
{
	struct hw_compfb_device *compfb = kzalloc(sizeof(struct hw_compfb_device), GFP_KERNEL);
	if (IS_ERR_OR_NULL(compfb)) {
		return NULL;
	}

	/*xxx Init Stuffs.. */

	return compfb;
}

int hw_compfb_exit(struct hw_compfb_device *compfb)
{

	if (compfb) {
		kfree(compfb);
	}

	return 0;
}
