/*
 * drivers/video/fbdev/ingenic/x2000_v12/ingenicfb.c
 *
 * Copyright (c) 2020 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#include <linux/dma-buf.h>
#include "dpu_reg.h"
#include "../include/ingenicfb.h"
#include "hw_composer_fb.h"
#include "hw_composer_v4l2.h"
#include <libdmmu.h>
#include <linux/of_reserved_mem.h>

#ifdef CONFIG_SOC_X2600
	#include "x2600_rotate.h"
	#define X2600_DDR_CCHC0 0xb3012024
	#define X2600_DDR_CCHC1 0xb3012028
#endif

#define dpu_debug 1
#define print_dbg(f, arg...) if(dpu_debug) printk(KERN_INFO "dpu: %s, %d: " f "\n", __func__, __LINE__, ## arg)

#define COMPOSER_DIRECT_OUT_EN

#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"
#include "../jz_mipi_dsi/jz_mipi_dsi_regs.h"
#include "../jz_mipi_dsi/jz_mipi_dsi_lowlevel.h"
extern struct dsi_device *jzdsi_init(struct jzdsi_data *pdata);
extern int dsi_change_video_config(struct dsi_device *dsi, struct lcd_panel *panel);
extern void jzdsi_remove(struct dsi_device *dsi);
extern void dump_dsi_reg(struct dsi_device *dsi);
int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power);
static const struct of_device_id ingenicfb_of_match[];
#ifdef CONFIG_TRUE_COLOR_LOGO
	#include <video/ingenic_logo.h>
	extern unsigned char logo_buf_initdata[] __initdata; /* obj/drivers/video/logo-ingenic/logo.c */
	extern void show_logo(struct fb_info *info);
	static unsigned char *copyed_logo_buf = NULL;
#endif

struct lcd_panel *fbdev_panel = NULL;
struct platform_device *fbdev_pdev = NULL;

static int showFPS = 0;

static bool panel_mipi_init_ok = false;
static struct ingenicfb_device *fbdev;

#ifdef CONFIG_FB_VSYNC_SKIP_DISABLE
	static unsigned int old_desc_addr = 0;
#endif

/* #define TEST_IRQ */

const struct fb_fix_screeninfo ingenicfb_fix  = {
	.id = "ingenicfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 1,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

struct ingenicfb_colormode {
	uint32_t mode;
	const char *name;
	uint32_t color;
	uint32_t bits_per_pixel;
	uint32_t nonstd;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

struct ingenic_priv {
	unsigned int dsi_iobase;
	unsigned int dsi_phy_iobase;
	bool support_comp;
};

/* 这个实际是comp支持的, 应该放在compfb中。这里应该只记录rdma支持的格式.*/
static struct ingenicfb_colormode ingenicfb_colormodes[] = {
	{
		.mode = RDMA_CHAIN_CFG_FORMAT_888,
		.name = "rgb888",
		.bits_per_pixel = 32,
		.nonstd = 0,
#ifdef CONFIG_FB_FORMAT_X8B8G8R8
		.color = RDMA_CHAIN_CFG_COLOR_BGR,
#else
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
#endif
		.red    = { .length = 8, .offset = 16, .msb_right = 0 },
		.green  = { .length = 8, .offset = 8, .msb_right = 0 },
		.blue   = { .length = 8, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_555,
		.name = "rgb555",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 10, .msb_right = 0 },
		.green  = { .length = 5, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_ARGB1555,
		.name = "argb1555",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 10, .msb_right = 0 },
		.green  = { .length = 5, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 1, .offset = 15, .msb_right = 0 },
	}, {
		.mode = RDMA_CHAIN_CFG_FORMAT_565,
		.name = "rgb565",
		.color = RDMA_CHAIN_CFG_COLOR_RGB,
		.bits_per_pixel = 16,
		.nonstd = 0,
		.red    = { .length = 5, .offset = 11, .msb_right = 0 },
		.green  = { .length = 6, .offset = 5, .msb_right = 0 },
		.blue   = { .length = 5, .offset = 0, .msb_right = 0 },
		.transp = { .length = 0, .offset = 0, .msb_right = 0 },
	},
};

static int ingenicfb_dmmu_mm_release(void *data)
{
	struct ingenicfb_device *fbdev = (struct ingenicfb_device *)data;

	hw_composer_lock(fbdev->comp_ctx);
	hw_composer_stop(fbdev->comp_ctx);
	hw_composer_unlock(fbdev->comp_ctx);

	return 0;
}

static struct dmmu_mm_ops ingenicfb_dmmu_mm_ops = {
	.mm_release = ingenicfb_dmmu_mm_release,
};

static void
ingenicfb_videomode_to_var(struct fb_var_screeninfo *var,
                           const struct fb_videomode *mode)
{
	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = mode->xres;
	var->yres_virtual = mode->yres * CONFIG_FB_INGENIC_NR_FRAMES;
	var->xoffset = 0;
	var->yoffset = 0;
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

static struct fb_videomode *ingenicfb_get_mode(struct fb_var_screeninfo *var,
        struct fb_info *info)
{
	size_t i;
	struct ingenicfb_device *fbdev = info->par;
	struct lcd_panel *panel = fbdev->panel;
	struct fb_videomode *mode = NULL;
	for (i = 0; i < fbdev->video_modes_num; i++) {
		mode = fbdev->video_mode[i];
		if (mode->xres == var->xres && mode->yres == var->yres
		    && mode->vmode == var->vmode
		    && mode->right_margin == var->right_margin) {
			if (fbdev->panel->lcd_type != LCD_TYPE_SLCD) {
				if (mode->pixclock == var->pixclock) {
					panel->cur_active_videomode = i;
					return mode;
				}
			} else {
				panel->cur_active_videomode = i;
				return mode;
			}
		}
	}

	return NULL;
}

static void ingenicfb_colormode_to_var(struct fb_var_screeninfo *var,
                                       struct ingenicfb_colormode *color)
{
	var->bits_per_pixel = color->bits_per_pixel;
	var->nonstd = color->nonstd;
	var->red = color->red;
	var->green = color->green;
	var->blue = color->blue;
	var->transp = color->transp;
}

static bool cmp_var_to_colormode(struct fb_var_screeninfo *var,
                                 struct ingenicfb_colormode *color)
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

static struct ingenicfb_colormode *ingenicfb_check_colormode(struct fb_var_screeninfo *var)
{
	int i;

	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
			struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
			if (var->nonstd == m->nonstd) {
				ingenicfb_colormode_to_var(var, m);
				return m;
			}
		}

		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
		struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
		if (cmp_var_to_colormode(var, m)) {
			ingenicfb_colormode_to_var(var, m);
			return m;
		}
	}
	/* To support user libraries that only support RGB format */
	for (i = 0; i < ARRAY_SIZE(ingenicfb_colormodes); ++i) {
		struct ingenicfb_colormode *m = &ingenicfb_colormodes[i];
		if (var->bits_per_pixel == m->bits_per_pixel) {
			ingenicfb_colormode_to_var(var, m);
			return m;
		}
	}

	return NULL;
}

static int ingenicfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode;
	struct ingenicfb_colormode *color_mode;
	mode = ingenicfb_get_mode(var, info);
	if (mode == NULL) {
		dev_err(info->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}

	ingenicfb_videomode_to_var(var, mode);

	/*TODO:不应该修改var.*/
	color_mode = ingenicfb_check_colormode(var);
	if (color_mode == NULL) {
		dev_err(info->dev, "%s Check colormode failed!\n", __func__);
		return  -EINVAL;
	}

	fbdev->color_mode = color_mode;

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2
static int jzfb_dmabuf_get_phy(struct device *dev, int fd, unsigned int *phyaddr)
{
	struct dma_buf_attachment *attach;
	struct dma_buf *dbuf;
	struct sg_table *sgt;
	int err = 0;
	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf)) {
		return -EINVAL;
	}
	attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attach)) {
		err = -EINVAL;
		goto fail_attach;
	}
	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = -EINVAL;
		goto fail_map;
	}

	*phyaddr = sg_dma_address(sgt->sgl);

	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
fail_map:
	dma_buf_detach(dbuf, attach);
fail_attach:
	dma_buf_put(dbuf);
	return err;
}
#endif

#define DPU_WAIT_IRQ_TIME 8000
static int ingenicfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct comp_setup_info comp_info;
	struct ingenicfb_device *fbdev = info->par;
	int ret;
	csc_mode_t csc_mode;
	int value;
	switch (cmd) {
		case JZFB_SET_VSYNCINT:
			if (unlikely(copy_from_user(&value, argp, sizeof(int)))) {
				return -EFAULT;
			}
			break;
		case FBIO_WAITFORVSYNC:
			if (fbdev->dctrl.chan == DATA_CH_COMP) {
				break;
			}
#ifndef CONFIG_FB_VSYNC_SKIP_DISABLE
			unlock_fb_info(info);
			if (fbdev->timestamp.wp == fbdev->timestamp.rp) {
				ret = wait_event_interruptible_timeout(fbdev->vsync_wq,
				                                       fbdev->timestamp.wp != fbdev->timestamp.rp,
				                                       msecs_to_jiffies(DPU_WAIT_IRQ_TIME));
			} else {
				ret = 1;
			}
			lock_fb_info(info);
			if (ret == 0) {
				dev_err(info->dev, "DPU wait vsync timeout!\n");
				return -EFAULT;
			}

			ret = copy_to_user(argp, fbdev->timestamp.value + fbdev->timestamp.rp,
			                   sizeof(u64));

			if (unlikely(ret)) {
				return -EFAULT;
			}
#else
			{
				unlock_fb_info(info);
				fbdev->timestamp.rp = fbdev->timestamp.wp;
				ret = wait_event_interruptible_timeout(fbdev->vsync_wq,
				                                       fbdev->timestamp.wp != fbdev->timestamp.rp,
				                                       msecs_to_jiffies(DPU_WAIT_IRQ_TIME));
				lock_fb_info(info);
				if (ret == 0) {
					dev_err(info->dev, "DPU wait vsync timeout!\n");
					return -EFAULT;
				}
			}
#endif
			break;
		case JZFB_PUT_FRM_CFG:
			ret = copy_from_user(&comp_info.frm_cfg, (void *)argp, sizeof(struct ingenicfb_frm_cfg));
			if (unlikely(ret)) {
				return -EFAULT;
			}

			comp_info.nframes = 1;
			//comp_info.out_mode = COMP_WRITE_BACK;

			/*设置composer帧信息，并且启动一次comp设备进行合成.*/
			hw_composer_lock(fbdev->comp_ctx);

			if (fbdev->dctrl.blank == 0) {
				hw_composer_setup(fbdev->comp_ctx, &comp_info);
				hw_composer_start(fbdev->comp_ctx);
			}

			hw_composer_unlock(fbdev->comp_ctx);
			break;
		case JZFB_GET_FRM_CFG:
			ret = copy_to_user((void *)argp,
			                   &fbdev->comp_ctx->comp_info.frm_cfg,
			                   sizeof(struct ingenicfb_frm_cfg));
			if (unlikely(ret)) {
				return -EFAULT;
			}
			break;
		case JZFB_GET_LAYERS_NUM: {
			unsigned int layers_num = CONFIG_FB_INGENIC_NR_LAYERS;
			ret = copy_to_user((void *)argp,
			                   &layers_num,
			                   sizeof(unsigned int));
			if (unlikely(ret)) {
				return -EFAULT;
			}
		}
		break;
		case JZFB_SET_CSC_MODE:
			if (unlikely(copy_from_user(&csc_mode, argp, sizeof(csc_mode_t)))) {
				return -EFAULT;
			}

			//TODO:
			//csc_mode_set(fbdev, csc_mode);
			break;
#ifdef CONFIG_MMU_NOTIFIER
		case JZFB_USE_TLB:
			if ((unsigned int)arg != 0) {
				fbdev->dctrl.tlba = (unsigned int)arg;

				dmmu_register_mm_notifier(&fbdev->dmn);
			} else {
				printk("tlb err!!!\n");
			}
			break;
		case JZFB_DMMU_MEM_SMASH: {
			struct smash_mode sm;
			if (copy_from_user(&sm, (void *)arg, sizeof(sm))) {
				ret = -EFAULT;
				break;
			}
			return dmmu_memory_smash(sm.vaddr, sm.mode);
			break;
		}
		case JZFB_DMMU_DUMP_MAP: {
			unsigned long vaddr;
			if (copy_from_user(&vaddr, (void *)arg, sizeof(unsigned long))) {
				ret = -EFAULT;
				break;
			}
			return dmmu_dump_map(vaddr);
			break;
		}
		case JZFB_DMMU_MAP: {
			struct dpu_dmmu_map_info di;
			if (copy_from_user(&di, (void *)arg, sizeof(di))) {
				ret = -EFAULT;
				break;
			}
			fbdev->user_addr = di.addr;
			return dmmu_map(fbdev->dev, di.addr, di.len);
			break;
		}
		case JZFB_DMMU_UNMAP: {
			struct dpu_dmmu_map_info di;
			if (copy_from_user(&di, (void *)arg, sizeof(di))) {
				ret = -EFAULT;
				break;
			}
			if (dpu_ctrl_comp_stop(&fbdev->dctrl,  GEN_STOP)) {
				printk("*** comp stop fail\n");
			}
			return dmmu_unmap(fbdev->dev, di.addr, di.len);
			break;
		}
		case JZFB_DMMU_UNMAP_ALL:
			dmmu_unmap_all(fbdev->dev);
			break;
		case JZFB_DMMU_FLUSH_CACHE: {
			struct dpu_dmmu_map_info di;
			if (copy_from_user(&di, (void *)arg, sizeof(di))) {
				ret = -EFAULT;
				break;
			}
			return dmmu_flush_cache(di.addr, di.len);
			break;
		}
#endif
#ifdef CONFIG_VIDEO_V4L2
		case JZFB_GET_DMA_PHY: {
			unsigned int phyaddr = 0;
			int fd = 0;
			if (copy_from_user(&fd, (void *)arg, sizeof(fd))) {
				dev_err(info->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			ret = jzfb_dmabuf_get_phy(info->dev, fd, &phyaddr);
			if (copy_to_user((void *)arg, &phyaddr, sizeof(phyaddr))) {
				return -EFAULT;
			}
		}
		break;
#endif
		default:
			printk("Command:%x Error!\n", cmd);
			break;
	}
	return 0;
}

int ingenicfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;

	start = info->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len) {
		return -EINVAL;
	}
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vm_flags_set(vma, VM_IO);
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	/* Write-Acceleration */
#ifdef CONFIG_FB_USING_CACHABLE
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;
#else
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_WA;
#endif
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
	                       vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static void ingenicfb_set_vsync_value(void *data)
{
	struct ingenicfb_device *fbdev = (struct ingenicfb_device *)data;

	fbdev->vsync_skip_map = (fbdev->vsync_skip_map >> 1 |
	                         fbdev->vsync_skip_map << 9) & 0x3ff;
	if (likely(fbdev->vsync_skip_map & 0x1)) {
		fbdev->timestamp.value[fbdev->timestamp.wp] =
		    ktime_to_ns(ktime_get());
		fbdev->timestamp.wp = (fbdev->timestamp.wp + 1) % TIMESTAMP_CAP;
		wake_up_interruptible(&fbdev->vsync_wq);
#ifndef CONFIG_FB_VSYNC_SKIP_DISABLE
	} else {
		fbdev->timestamp.wp = fbdev->timestamp.rp + 1;
		wake_up_interruptible(&fbdev->vsync_wq);
#endif
	}
}

static inline uint32_t convert_color_to_hw(unsigned val, struct fb_bitfield *bf)
{
	return (((val << bf->length) + 0x7FFF - val) >> 16) << bf->offset;
}

static int ingenicfb_setcolreg(unsigned regno, unsigned red, unsigned green,
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

static int calc_refresh_ratio(struct lcd_panel *panel)
{
	struct smart_config *smart_config = panel->smart_config;

	switch (smart_config->smart_type) {
		case SMART_LCD_TYPE_8080:
		case SMART_LCD_TYPE_6800:
			break;
		case SMART_LCD_TYPE_SPI_3:
			return 9;
		case SMART_LCD_TYPE_SPI_4:
			return 8;
		default:
			printk("%s %d err!\n", __func__, __LINE__);
			break;
	}

	switch (smart_config->pix_fmt) {
		case SMART_LCD_FORMAT_888:
			if (smart_config->dwidth == SMART_LCD_DWIDTH_8_BIT) {
				return 4;
			}
			if (smart_config->dwidth == SMART_LCD_DWIDTH_24_BIT) {
				return 2;
			}
			break;
		case SMART_LCD_FORMAT_565:
			if (smart_config->dwidth == SMART_LCD_DWIDTH_8_BIT) {
				return 3;
			}
			if (smart_config->dwidth == SMART_LCD_DWIDTH_16_BIT) {
				return 2;
			}
			break;
		default:
			printk("%s %d err!\n", __func__, __LINE__);
			break;
	}

	return 1;

}

static int refresh_pixclock_auto_adapt(struct fb_info *info, struct fb_videomode *mode)
{
	struct ingenicfb_device *fbdev = info->par;
	struct lcd_panel *panel = fbdev->panel;
	uint16_t hds, vds;
	uint16_t hde, vde;
	uint16_t ht, vt;
	unsigned long rate;
	unsigned int refresh_ratio = 1;

	if (mode == NULL) {
		dev_err(fbdev->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}

	if (fbdev->panel->lcd_type == LCD_TYPE_SLCD) {
		refresh_ratio = calc_refresh_ratio(panel);
	}

	hds = mode->hsync_len + mode->left_margin;
	hde = hds + mode->xres;
	ht = hde + mode->right_margin;

	vds = mode->vsync_len + mode->upper_margin;
	vde = vds + mode->yres;
	vt = vde + mode->lower_margin;

	if (mode->refresh) {
		rate = mode->refresh * vt * ht * refresh_ratio;

		mode->pixclock = KHZ2PICOS(rate / 1000 + 1);    //KHz
	} else if (mode->pixclock) {
		rate = PICOS2KHZ(mode->pixclock) * 1000;    //Hz
		mode->refresh = rate / vt / ht / refresh_ratio;
	} else {
		dev_err(fbdev->dev, "%s error:lcd important config info is absenced\n", __func__);
		return -EINVAL;
	}

	dev_info(fbdev->dev, "mode->refresh: %d, mode->pixclock: %d, rate: %ld, modex->pixclock: %d\n", mode->refresh, mode->pixclock, rate, mode->pixclock);

	return 0;
}

static int ingenicfb_update_vidmem(struct ingenicfb_device *fbdev, unsigned int x_res, unsigned int y_res, unsigned int bits_per_pixel)
{
	unsigned int alloc_size = 0;
	int i = 0;

	fbdev->vidmem_size = 0;

	alloc_size = x_res * y_res;
	fbdev->frm_size = (alloc_size * bits_per_pixel) >> 3;
	alloc_size *= MAX_BITS_PER_PIX >> 3;
	alloc_size = alloc_size * CONFIG_FB_INGENIC_NR_FRAMES;

	if (IS_ERR_OR_NULL(fbdev->vidmem[0])) {
		return -ENOMEM;
	}
	fbdev->vidmem_size = alloc_size;

	for (i = 1; i < CONFIG_FB_INGENIC_NR_FRAMES; i++) {
		fbdev->vidmem[i] = fbdev->vidmem[0] + i * fbdev->frm_size;
		fbdev->vidmem_phys[i] = fbdev->vidmem_phys[0] + i * fbdev->frm_size;
	}

	return 0;
}

static int ingenicfb_set_par(struct fb_info *info)
{
	int ret = 0;
	struct ingenicfb_device *fbdev = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	struct lcd_panel *panel = fbdev->panel;
	struct fb_videomode *mode;
	struct ingenicfb_colormode *color_mode;
	struct rdma_setup_info *rdma_info = &fbdev->rdma_info;

	/*根据var xres,yres信息，查找panel支持的mode*/
	mode = ingenicfb_get_mode(var, info);
	if (mode == NULL) {
		dev_err(info->dev, "%s get video mode failed\n", __func__);
		return -EINVAL;
	}
	info->mode = mode;
	fbdev->active_video_mode = mode;
	dctrl->active_video_mode = fbdev->active_video_mode;
	/*根据var信息, 查找到colormode.*/
	color_mode = ingenicfb_check_colormode(var);
	if (color_mode == NULL) {
		dev_err(info->dev, "%s Check colormode failed!\n", __func__);
		return  -EINVAL;
	}
	ingenicfb_videomode_to_var(var, fbdev->active_video_mode);
	/*修改当前指向的color_mode? 那check_var需要修改吗?*/
	fbdev->color_mode = color_mode;

	ret = ingenicfb_update_vidmem(fbdev, info->var.xres, info->var.yres, info->var.bits_per_pixel);
	if (ret) {
		dev_err(info->dev, "Failed to update video memory\n");
	}

	info->fix = ingenicfb_fix;
	info->fix.line_length = (info->var.bits_per_pixel * info->var.xres) >> 3;
	info->fix.smem_start = fbdev->vidmem_phys[0];
	info->fix.smem_len = fbdev->vidmem_size;
	info->screen_size = info->fix.line_length * info->var.yres;
	info->screen_base = fbdev->vidmem[0];
	info->pseudo_palette = fbdev->pseudo_palette;

	/*构建rdma信息，设置并启动.*/
	rdma_info->nframes = CONFIG_FB_INGENIC_NR_FRAMES;   //TODO:
	rdma_info->format = color_mode->mode;
	rdma_info->color = color_mode->color;
	rdma_info->stride = var->xres;
	rdma_info->continuous = 1; //TODO:
	rdma_info->vidmem = (unsigned char **)&fbdev->vidmem;
	rdma_info->vidmem_phys = (unsigned int **)&fbdev->vidmem_phys;
	if (panel->lcd_type == LCD_TYPE_LVDS_JEIDA || panel->lcd_type == LCD_TYPE_LVDS_VESA) {
		ingenic_set_lvds_display_clk(dctrl);
	} else {
		ingenic_set_pixclk(dctrl, PICOS2KHZ(dctrl->active_video_mode->pixclock));
	}

	if (panel->lcd_type == LCD_TYPE_MIPI_SLCD || panel->lcd_type == LCD_TYPE_MIPI_TFT) {
		if (panel->cur_active_videomode < panel->num_modes) {
			dsi_change_video_config(dctrl->dsi, panel);
		}
	}

	if (&dctrl->rdma_info != rdma_info) {
		memcpy(&dctrl->rdma_info, rdma_info, sizeof(struct rdma_setup_info));
	}

	return 0;
}

static inline int timeval_sub_to_us(struct timespec64 lhs,
                                    struct timespec64 rhs)
{
	int sec, nsec;
	sec = lhs.tv_sec - rhs.tv_sec;
	nsec = lhs.tv_nsec - rhs.tv_nsec;

	return (sec * 1000000 + nsec / 1000);
}

static inline int time_us2ms(int us)
{
	return (us / 1000);
}

static void calculate_frame_rate(void)
{
	static struct timespec64 time_now, time_last;
	unsigned int interval_in_us;
	unsigned int interval_in_ms;
	static unsigned int fpsCount = 0;

	switch (showFPS) {
		case 1:
			fpsCount++;
			ktime_get_real_ts64(&time_now);
			interval_in_us = timeval_sub_to_us(time_now, time_last);
			if (interval_in_us > (USEC_PER_SEC)) {   /* 1 second = 1000000 us. */
				printk(" Pan display FPS: %d\n", fpsCount);
				fpsCount = 0;
				time_last = time_now;
			}
			break;
		case 2:
			ktime_get_real_ts64(&time_now);
			interval_in_us = timeval_sub_to_us(time_now, time_last);
			interval_in_ms = time_us2ms(interval_in_us);
			printk(" Pan display interval ms: %d\n", interval_in_ms);
			time_last = time_now;
			break;
		default:
			if (showFPS > 3) {
				int d, f;
				fpsCount++;
				ktime_get_real_ts64(&time_now);
				interval_in_us = timeval_sub_to_us(time_now, time_last);
				if (interval_in_us > USEC_PER_SEC * showFPS) {  /* 1 second = 1000000 us. */
					d = fpsCount / showFPS;
					f = (fpsCount * 10) / showFPS - d * 10;
					printk(" Pan display FPS: %d.%01d\n", d, f);
					fpsCount = 0;
					time_last = time_now;
				}
			}
			break;
	}
}

static int ingenicfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int next_frm;

	if (var->xoffset - info->var.xoffset) {
		dev_err(info->dev, "No support for X panning for now\n");
		return -EINVAL;
	}
	fbdev->pan_display_count++;
	if (showFPS) {
		calculate_frame_rate();
	}
	next_frm = var->yoffset / var->yres;
#ifdef CONFIG_FB_USING_CACHABLE
	dma_cache_wback_inv(fbdev->vidmem[next_frm], var->yres * var->xres);
#endif
	dpu_ctrl_rdma_change(dctrl, next_frm);

	return 0;
}

static int ingenicfb_do_resume(struct ingenicfb_device *fbdev)
{
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int ret = 0;

	ret = dpu_ctrl_resume(dctrl);
	dctrl->blank = 0;
	return ret;
}

static int ingenicfb_do_suspend(struct ingenicfb_device *fbdev)
{
	struct dpu_ctrl *dctrl = &fbdev->dctrl;
	int ret = 0;

	ret = dpu_ctrl_suspend(dctrl);
	dctrl->blank = 1;

	return ret;
}

static int ingenicfb_blank(int blank_mode, struct fb_info *info)
{
	struct ingenicfb_device *fbdev = info->par;
	int ret = 0;

	if (blank_mode == FB_BLANK_UNBLANK) {
		ret = ingenicfb_do_resume(fbdev);
	} else {
		ret = ingenicfb_do_suspend(fbdev);
	}

	return ret;
}

static int ingenicfb_open(struct fb_info *info, int user)
{
	struct ingenicfb_device *fbdev = info->par;
#if 0 //TODO:

	if (!fbdev->is_lcd_en && fbdev->vidmem_phys) {
		fbdev->timestamp.rp = 0;
		fbdev->timestamp.wp = 0;
		ingenicfb_set_fix_par(info);
		ret = ingenicfb_set_par(info);
		if (ret) {
			dev_err(info->dev, "Set par failed!\n");
			return ret;
		}
		memset(fbdev->vidmem[fbdev->current_frm_desc][0], 0, fbdev->frm_size);
		ingenicfb_enable(info);
	}
#endif
	dev_dbg(info->dev, "####open count : %d\n", ++fbdev->open_cnt);

	return 0;
}

static int ingenicfb_release(struct fb_info *info, int user)
{
	struct ingenicfb_device *fbdev = info->par;

	dev_dbg(info->dev, "####close count : %d\n", fbdev->open_cnt--);
	if (!fbdev->open_cnt) {
		//      fbdev->timestamp.rp = 0;
		//      fbdev->timestamp.wp = 0;
	}
	return 0;
}

ssize_t ingenicfb_write(struct fb_info *info, const char __user *buf,
                        size_t count, loff_t *ppos)
{
	u8 *buffer, *src;
	u8 __iomem *dst;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	unsigned long p = *ppos;
	int screen_base_offset = 0;
	int next_frm = 0;

	next_frm = info->var.yoffset / info->var.yres;

	screen_base_offset = next_frm * info->screen_size;

	total_size = info->screen_size;

	if (total_size == 0) {
		total_size = info->fix.smem_len;
	}

	if (p > total_size) {
		return -EFBIG;
	}

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err) {
			err = -ENOSPC;
		}

		count = total_size - p;
	}

	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
	                 GFP_KERNEL);
	if (!buffer) {
		return -ENOMEM;
	}

	dst = (u8 __iomem *)(info->screen_base + p + screen_base_offset);

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		src = buffer;

		if (copy_from_user(src, buf, c)) {
			err = -EFAULT;
			break;
		}

		fb_memcpy_toio(dst, src, c);
		dst += c;
		src += c;
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (cnt) ? cnt : err;
}

static ssize_t
ingenicfb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	u8 *buffer, *dst;
	u8 __iomem *src;
	int c, cnt = 0, err = 0;
	unsigned long total_size;

	int screen_base_offset = 0;
	int next_frm = 0;

	next_frm = info->var.yoffset / info->var.yres;

	if (!info || ! info->screen_base) {
		return -ENODEV;
	}

	if (info->state != FBINFO_STATE_RUNNING) {
		return -EPERM;
	}

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

	src = (u8 __iomem *)(info->screen_base + p + screen_base_offset);

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

static struct fb_ops ingenicfb_ops = {
	.owner      = THIS_MODULE,
	.fb_open    = ingenicfb_open,
	.fb_release     = ingenicfb_release,
	.fb_write   = ingenicfb_write,
	.fb_read    = ingenicfb_read,
	.fb_check_var   = ingenicfb_check_var,
	.fb_set_par     = ingenicfb_set_par,
	.fb_setcolreg   = ingenicfb_setcolreg,
	.fb_blank   = ingenicfb_blank,
	.fb_pan_display = ingenicfb_pan_display,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_ioctl   = ingenicfb_ioctl,
	.fb_mmap    = ingenicfb_mmap,
};

static int vsync_skip_set(struct ingenicfb_device *fbdev, int vsync_skip)
{
#if 0
	unsigned int map_wide10 = 0;
	int rate, i, p, n;
	int fake_float_1k;

	if (vsync_skip < 0 || vsync_skip > 9) {
		return -EINVAL;
	}

	rate = vsync_skip + 1;
	fake_float_1k = 10000 / rate;   /* 10.0 / rate */

	p = 1;
	n = (fake_float_1k * p + 500) / 1000;   /* +0.5 to int */

	for (i = 1; i <= 10; i++) {
		map_wide10 = map_wide10 << 1;
		if (i == n) {
			map_wide10++;
			p++;
			n = (fake_float_1k * p + 500) / 1000;
		}
	}
	mutex_lock(&fbdev->lock);
	fbdev->vsync_skip_map = map_wide10;
	fbdev->vsync_skip_ratio = rate - 1;     /* 0 ~ 9 */
	mutex_unlock(&fbdev->lock);

	printk("vsync_skip_ratio = %d\n", fbdev->vsync_skip_ratio);
	printk("vsync_skip_map = 0x%08x\n", fbdev->vsync_skip_map);
#endif
	return 0;
}

static int ingenicfb_alloc_vidmem(struct ingenicfb_device *fbdev, unsigned int x_res, unsigned int y_res, unsigned int bits_per_pixel)
{
	unsigned int alloc_size = 0;
	int i = 0;

	fbdev->vidmem_size = 0;

	alloc_size = x_res * y_res;
	fbdev->frm_size = (alloc_size * bits_per_pixel) >> 3;
	alloc_size *= MAX_BITS_PER_PIX >> 3;
	alloc_size = alloc_size * CONFIG_FB_INGENIC_NR_FRAMES;
	fbdev->vidmem[0] = dma_alloc_coherent(fbdev->dev, alloc_size, &fbdev->vidmem_phys[0], GFP_KERNEL);

	if (IS_ERR_OR_NULL(fbdev->vidmem[0])) {
		return -ENOMEM;
	}
	fbdev->vidmem_size = alloc_size;

	for (i = 1; i < CONFIG_FB_INGENIC_NR_FRAMES; i++) {
		fbdev->vidmem[i] = fbdev->vidmem[0] + i * fbdev->frm_size;
		fbdev->vidmem_phys[i] = fbdev->vidmem_phys[0] + i * fbdev->frm_size;
	}

	dev_info(fbdev->dev, "vidmem @ %p size %d\n", fbdev->vidmem[0], fbdev->vidmem_size);

	return 0;
}

int ingenicfb_release_vidmem(struct ingenicfb_device *fbdev)
{
	if (fbdev->vidmem_size) {
		dma_free_coherent(fbdev->dev, fbdev->vidmem_size, fbdev->vidmem[0], fbdev->vidmem_phys[0]);
		fbdev->vidmem_size = 0;
	}

	return 0;
}

int pan_init_logo(struct fb_info *fb)
{

#ifdef CONFIG_TRUE_COLOR_LOGO
	if (!copyed_logo_buf || !fb) {
		pr_err("logo not support multi layer!\n");
		return 0;
	}

	logo_info.p8 = copyed_logo_buf;
	show_logo(fb);
	/* free logo mem */
	kfree(copyed_logo_buf);
	copyed_logo_buf = NULL;
#endif

	return 0;
}

#ifdef CONFIG_SOC_X2600
static int dy_rot_angle = -1;
static int __init fbdev_rotate_setup(char *arg)
{
	dy_rot_angle = simple_strtol(arg, NULL, 10);
	if (dy_rot_angle != 0 && dy_rot_angle != 90 &&
	    dy_rot_angle != 180 && dy_rot_angle != 270) {
		printk("fbdev: 'rotate' parameter out of range[0, 90, 180,270], ignoring.\n");
		dy_rot_angle = -1;
	}
	return 0;
}
early_param("rot_angle", fbdev_rotate_setup); //get nv.rot_angle from kernel cmdargs.
#endif

static int ingenicfb_do_probe(struct platform_device *pdev, struct lcd_panel *panel)
{
	struct fb_info *fb;
	struct dpu_ctrl *dctrl;
	int ret = 0;
	int i = 0;
	const struct of_device_id *match;
	unsigned int reg;
	fb = framebuffer_alloc(sizeof(struct ingenicfb_device), &pdev->dev);
	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		return -ENOMEM;
	}

	of_reserved_mem_device_init(&pdev->dev);

	fbdev = fb->par;
	fbdev->fb = fb;
	fbdev->dev = &pdev->dev;

	fbdev->dmn.dev = fbdev->dev;
	fbdev->dmn.data = fbdev;
	fbdev->dmn.ops = &ingenicfb_dmmu_mm_ops;
	fbdev->swap_dims = 0;

	dctrl = &fbdev->dctrl;
	of_property_read_u32(fbdev->dev->of_node, "ingenic,disable-rdma-fb", &fbdev->disable_rdma_fb);
#ifdef CONFIG_SOC_X2600
	if (of_property_read_u32(fbdev->dev->of_node, "ingenic,rot_angle", &fbdev->rot_angle)) {
		dev_err(fbdev->dev, "read rot_angle failed! please check dts.");
	}
	dev_info(fbdev->dev, "dts rot_angle = %d, nv.rot_angle = %d\n", fbdev->rot_angle, dy_rot_angle);
	if (dy_rot_angle != -1 && fbdev->rot_angle != dy_rot_angle) {
		int angle_diff = (dy_rot_angle - fbdev->rot_angle + 360) % 360;
		if (angle_diff == 90 || angle_diff == 270) {
			fbdev->swap_dims = 1;
		} else {
			fbdev->swap_dims = 0;
		}
		fbdev->rot_angle = dy_rot_angle;
	}
#endif
	if (panel->dsi_pdata) {
		if (!of_property_read_u32(fbdev->dev->of_node, "dsi-host-reg", &reg)) {
			panel->dsi_pdata->dsi_iobase = reg;
		}
		if (!of_property_read_u32(fbdev->dev->of_node, "dsi-phy-reg", &reg)) {
			panel->dsi_pdata->dsi_phy_iobase = reg;
		}
		if (of_property_read_bool(fbdev->dev->of_node, "lane0_pn_swap")) {
			panel->dsi_pdata->video_config.lane0_pn_swap = 1;
		}
		if (of_property_read_bool(fbdev->dev->of_node, "lane1_pn_swap")) {
			panel->dsi_pdata->video_config.lane1_pn_swap = 1;
		}
		if (of_property_read_bool(fbdev->dev->of_node, "lane2_pn_swap")) {
			panel->dsi_pdata->video_config.lane2_pn_swap = 1;
		}
		if (of_property_read_bool(fbdev->dev->of_node, "lane3_pn_swap")) {
			panel->dsi_pdata->video_config.lane3_pn_swap = 1;
		}
		if (of_property_read_bool(fbdev->dev->of_node, "clk_lane_pn_swap")) {
			panel->dsi_pdata->video_config.clk_lane_pn_swap = 1;
		}

	}
	if (of_property_read_bool(fbdev->dev->of_node, "composer-support")) {
		dctrl->support_comp = true;
	} else {
		dctrl->support_comp = false;
	}

	match = of_match_node(ingenicfb_of_match, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}
	fbdev->panel = panel;

	for (i = 0; i < panel->num_modes; i++) {
		ret = refresh_pixclock_auto_adapt(fb, &panel->modes[i]);
		if (ret) {
			goto err_calc_pixclk;
		}
		fbdev->video_mode[i] = &panel->modes[i];
		fbdev->video_modes_num ++;
	}
	/*一个屏幕可能由多个modes, 将所有的modes转换成modelist.*/
	fb_videomode_to_modelist(panel->modes, panel->num_modes, &fb->modelist);

	/*同一时刻应该只有一个video_mode, 所以要找到active_video_mode.*/
	fbdev->active_video_mode = &panel->modes[0];

	dctrl->dev = &pdev->dev;
	dctrl->pdev = pdev;
	dctrl->active_video_mode = fbdev->active_video_mode;

	dctrl->set_vsync_value = ingenicfb_set_vsync_value;
	dctrl->vsync_data = fbdev;

#ifdef CONFIG_SOC_X2600
	dctrl->rot_init = 0;
	/*由于rotate在prot1 使用旋转时将port1优先级设置为最高*/
	int ddr_cchc0 = 0x88404010;
	int ddr_cchc1 = 0x88404030;
	if (!fbdev->disable_rdma_fb) {
		fbdev->rot_angle = 0;
	}
	if (fbdev->rot_angle != 0) {
		writel(ddr_cchc0, (void __iomem *)X2600_DDR_CCHC0);
		writel(ddr_cchc1, (void __iomem *)X2600_DDR_CCHC1);
		rot_ctrl_init(dctrl, fbdev->rot_angle);
	}
#endif
	ret = dpu_ctrl_init(dctrl, panel);
	if (ret) {
		goto err_dctrl_init;
	}
	panel_mipi_init_ok = true;

	if (dctrl->support_comp) {
		// composer master
		fbdev->comp_master = hw_composer_init(dctrl);
		if (IS_ERR_OR_NULL(fbdev->comp_master)) {
			goto err_comp_init;
		}

		// one composer instance for fbdev.
		// using by ioctl procedure.
		fbdev->comp_ctx = hw_composer_create(fbdev);
		if (IS_ERR_OR_NULL(fbdev->comp_ctx)) {
			goto err_comp_ctx;
		}
	}

	/*width只是物理尺寸，以mm为单位。可以用于计算PPI*/
	fb->fbops = &ingenicfb_ops;
	fb->flags = 0;
	fb->var.width = panel->width;
	fb->var.height = panel->height;

	/*默认使用第一个colormode. */
	fbdev->color_mode = &ingenicfb_colormodes[0];
	ingenicfb_videomode_to_var(&fb->var, fbdev->active_video_mode);
	ingenicfb_colormode_to_var(&fb->var, fbdev->color_mode);

	ret = ingenicfb_check_var(&fb->var, fb);
	if (ret) {
		goto err_check_var;
	}

	vsync_skip_set(fbdev, CONFIG_FB_VSYNC_SKIP);
	init_waitqueue_head(&fbdev->vsync_wq);

	fbdev->open_cnt = 0;
	fbdev->is_lcd_en = 0;
	fbdev->timestamp.rp = 0;
	fbdev->timestamp.wp = 0;

	if (!fbdev->disable_rdma_fb) {

		ret = ingenicfb_alloc_vidmem(fbdev, fb->var.xres, fb->var.yres, fb->var.bits_per_pixel);
		if (ret) {
			dev_err(&pdev->dev, "Failed to allocate video memory\n");
			goto err_alloc_vidmem;
		}

		fb->fix = ingenicfb_fix;
		fb->fix.line_length = (fb->var.bits_per_pixel * fb->var.xres) >> 3;
		fb->fix.smem_start = fbdev->vidmem_phys[0];
		fb->fix.smem_len = fbdev->vidmem_size;
		fb->screen_size = fb->fix.line_length * fb->var.yres;
		fb->screen_base = fbdev->vidmem[0];
		fb->pseudo_palette = fbdev->pseudo_palette;

		ret = register_framebuffer(fb);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register framebuffer: %d\n",
			        ret);
			goto err_register_framebuffer;
		}
	}

	/*1. Common setup. */
	dctrl->chan = DATA_CH_RDMA;
	dpu_ctrl_setup(dctrl);

	if (!fbdev->disable_rdma_fb) {
		struct rdma_setup_info *rdma_info = &fbdev->rdma_info;
		rdma_info->nframes = CONFIG_FB_INGENIC_NR_FRAMES;   //TODO:
		rdma_info->format = fbdev->color_mode->mode;
		rdma_info->color = fbdev->color_mode->color;
		rdma_info->stride = fb->var.xres;
		rdma_info->continuous = 1; //TODO:
		rdma_info->vidmem = (unsigned char **)&fbdev->vidmem;
		rdma_info->vidmem_phys = (unsigned int **)&fbdev->vidmem_phys;

		dpu_ctrl_rdma_setup(dctrl, rdma_info);

		dpu_ctrl_rdma_start(dctrl);

		pan_init_logo(fbdev->fb);

		dctrl->work_in_rdma = true;
	} else {
		dctrl->work_in_rdma = false;
	}

	if (dctrl->support_comp) {
		/*2. compfb init and export.*/
		fbdev->compfb = hw_compfb_init(fbdev);
		if (IS_ERR_OR_NULL(fbdev->compfb)) {
			dev_err(&pdev->dev, "Failed to init compfb!\n");
			goto err_compfb_init;
		}

		/*3. export composer.*/
		fbdev->compfb->active_video_mode = fbdev->active_video_mode;
		fbdev->compfb->dev = &pdev->dev;
		fbdev->compfb->panel = panel;
	}
#ifdef CONFIG_HW_COMPOSER_V4L2_M2M
	{
		fbdev->comp_v4l2 = hw_comp_v4l2_init(&pdev->dev);
	}
#endif
	platform_set_drvdata(pdev, fbdev);
#ifdef CONFIG_SYSFS
	ret = dpu_sysfs_init(&fbdev->sysfs, dctrl, fbdev->compfb);
	if (ret < 0) {
		dev_err(fbdev->dev, "failed to init sysfs!\n");
	}
#endif
	return 0;

err_compfb_init:
err_register_framebuffer:
err_alloc_vidmem:
err_check_var:
err_comp_ctx:
	if (dctrl->support_comp) {
		hw_composer_exit(fbdev->comp_master);
	}
err_comp_init:
	dpu_ctrl_exit(dctrl);
err_dctrl_init:
	fb_delete_videomode(panel->modes, &fb->modelist);
	fb_destroy_modelist(&fb->modelist);
err_calc_pixclk:
	framebuffer_release(fb);
	return ret;
}

int ingenicfb_register_panel(struct lcd_panel *panel)
{
	WARN_ON(fbdev_panel != NULL);

	if (fbdev_pdev != NULL) {
		if (panel_mipi_init_ok == false) {
			return ingenicfb_do_probe(fbdev_pdev, panel);
		} else {
			return -EBUSY;
		}
	}

	fbdev_panel = panel;

	return 0;
}
EXPORT_SYMBOL_GPL(ingenicfb_register_panel);

static int ingenicfb_probe(struct platform_device *pdev)
{
	WARN_ON(fbdev_pdev != NULL);

	fbdev_pdev = pdev;

	if (fbdev_panel != NULL) {
		return ingenicfb_do_probe(fbdev_pdev, fbdev_panel);
	}

	return 0;
}

static int ingenicfb_remove(struct platform_device *pdev)
{
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	// TODO:
	// release comp_ctx, compfb, comp_v4l2.

	hw_compfb_exit(fbdev->compfb);
#ifdef CONFIG_HW_COMPOSER_V4L2_M2M
	hw_comp_v4l2_exit(fbdev->comp_v4l2);
#endif
	hw_composer_destroy(fbdev->comp_ctx);
	hw_composer_exit(fbdev->comp_master);

	dpu_ctrl_exit(&fbdev->dctrl);

	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_SYSFS
	dpu_sysfs_exit(&fbdev->sysfs);
#endif
	unregister_framebuffer(fbdev->fb);
	ingenicfb_release_vidmem(fbdev);
	framebuffer_release(fbdev->fb);

	return 0;
}

static void ingenicfb_shutdown(struct platform_device *pdev)
{
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(fbdev)) {
		fb_blank(fbdev->fb, FB_BLANK_POWERDOWN);
	}
};

#ifdef CONFIG_PM

static int ingenicfb_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);
	if (!IS_ERR_OR_NULL(fbdev)) {
		fb_blank(fbdev->fb, FB_BLANK_POWERDOWN);
	}

	return 0;
}

static int ingenicfb_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenicfb_device *fbdev = platform_get_drvdata(pdev);

	if (!IS_ERR_OR_NULL(fbdev)) {
		fb_blank(fbdev->fb, FB_BLANK_UNBLANK);
	}

	return 0;
}

static const struct dev_pm_ops ingenicfb_pm_ops = {
	.suspend = ingenicfb_suspend,
	.resume = ingenicfb_resume,
};
#endif

static const struct of_device_id ingenicfb_of_match[] = {
	{ .compatible = "ingenic,dpu",},
};

static struct platform_driver ingenicfb_driver = {
	.probe = ingenicfb_probe,
	.remove = ingenicfb_remove,
	.shutdown = ingenicfb_shutdown,
	.driver = {
		.name = "ingenic-fb",
		.of_match_table = ingenicfb_of_match,
#ifdef CONFIG_PM
		.pm = &ingenicfb_pm_ops,
#endif

	},
};

static int __init ingenicfb_init(void)
{
#ifdef CONFIG_TRUE_COLOR_LOGO
	/* copy logo_buf from .init.data section */
	int size;
	size = logo_info.width * logo_info.height * (logo_info.bpp / 8);
	printk(KERN_INFO "copy logo_buf from .init.data section, size=%d\n", size);
	copyed_logo_buf = kmalloc(size, GFP_KERNEL);
	memcpy(copyed_logo_buf, &logo_buf_initdata[0], size);
#endif
	platform_driver_register(&ingenicfb_driver);
	return 0;
}

static void __exit ingenicfb_cleanup(void)
{
	platform_driver_unregister(&ingenicfb_driver);
}

#ifdef CONFIG_EARLY_INIT_RUN
	rootfs_initcall(ingenicfb_init);
#else
	module_init(ingenicfb_init);
#endif

module_exit(ingenicfb_cleanup);

MODULE_DESCRIPTION("JZ LCD Controller driver");
MODULE_AUTHOR("qipenzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL");
