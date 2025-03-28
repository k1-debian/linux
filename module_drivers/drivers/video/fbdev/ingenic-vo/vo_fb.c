/*
 *
 * Copyright (c) 2022 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <linux/videodev2.h>

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/of_address.h>

#include "vo_fb.h"

/*
 *  RAM we reserve for the frame buffer. This defines the maximum screen
 *  size
 *
 *  The default can be overridden if the driver is compiled as a module
 */

#define _WIDTH      640
#define _HEIGHT     480
#define _NR_FRAMES  3

#define VIDEOMEMSIZE    (_WIDTH * _HEIGHT *_NR_FRAMES * 3 / 2)  /* 1 MB */

#define VO_DUMP_REG 0

/**********************************************************************
 *
 * Memory management
 *
 **********************************************************************/
static int rvmalloc(struct ingenic_vo *vofb, unsigned long size)
{
	unsigned long adr;

	vofb->videomemorysize = size;
	vofb->videomemory = dma_alloc_coherent(vofb->dev, size, &vofb->videomemory_phys, GFP_KERNEL);

	if (IS_ERR_OR_NULL(vofb->videomemory)) {
		dev_err(vofb->dev, "Failed to alloc video memory size!\n");
		return -ENOMEM;
	}

	memset(vofb->videomemory, 0, vofb->videomemorysize);

	return 0;
}

static void rvfree(struct ingenic_vo *vofb, void *mem, unsigned long size)
{
	dma_free_coherent(vofb->dev, vofb->videomemorysize, vofb->videomemory, vofb->videomemory_phys);
}

static struct fb_var_screeninfo vfb_default = {
	.xres =     _WIDTH,
	.yres =     _HEIGHT,
	.xres_virtual = _WIDTH,
	.yres_virtual = _HEIGHT * _NR_FRAMES,
	.bits_per_pixel = 16,
	.activate = FB_ACTIVATE_TEST,
	.height =   -1,
	.width =    -1,
	.pixclock = 20000,
	.left_margin =  64,
	.right_margin = 64,
	.upper_margin = 32,
	.lower_margin = 32,
	.hsync_len =    64,
	.vsync_len =    2,
	.vmode =    FB_VMODE_NONINTERLACED,
	.grayscale = V4L2_PIX_FMT_NV12, //Default NV12.
	.nonstd     = 8, // 8: nv12, 9: nv21, 私有约定.
};

static struct fb_fix_screeninfo vfb_fix = {
	.id =       "Ingenic VO",
	.type =     FB_TYPE_FOURCC,
	.visual =   FB_VISUAL_FOURCC,
	.xpanstep = 0,
	.ypanstep = 1,
	.ywrapstep =    0,
	.accel =    FB_ACCEL_NONE,
};

static int vfb_check_var(struct fb_var_screeninfo *var,
                         struct fb_info *info);
static int vfb_set_par(struct fb_info *info);
static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
                         u_int transp, struct fb_info *info);
static int vfb_pan_display(struct fb_var_screeninfo *var,
                           struct fb_info *info);
static int vfb_mmap(struct fb_info *info,
                    struct vm_area_struct *vma);

struct vo_reg_struct ingenic_vo_regs_name[] = {
	{" VO_ADDR_VER		     ", VO_ADDR_VER          },
	{" VO_ADDR_TOP_CON	     ", VO_ADDR_TOP_CON      },
	{" VO_ADDR_TOP_STATUS    ", VO_ADDR_TOP_STATUS   },
	{ " VO_ADDR_INT_EN       ", VO_ADDR_INT_EN       },
	{ " VO_ADDR_INT_VAL      ", VO_ADDR_INT_VAL      },
	{ " VO_ADDR_INT_CLC      ", VO_ADDR_INT_CLC      },
	{ " VO_ADDR_VC0_SIZE     ", VO_ADDR_VC0_SIZE     },
	{ " VO_ADDR_VC0_STRIDE   ", VO_ADDR_VC0_STRIDE   },
	{ " VO_ADDR_VC0_Y_ADDR   ", VO_ADDR_VC0_Y_ADDR   },
	{ " VO_ADDR_VC0_UV_ADDR  ", VO_ADDR_VC0_UV_ADDR  },
	{ " VO_ADDR_VC0_BLK_INFO ", VO_ADDR_VC0_BLK_INFO },
	{ " VO_ADDR_VC0_CON_PAR  ", VO_ADDR_VC0_CON_PAR  },
	{ " VO_ADDR_VC0_VBLK_PAR ", VO_ADDR_VC0_VBLK_PAR },
	{ " VO_ADDR_VC0_HBLK_PAR ", VO_ADDR_VC0_HBLK_PAR },
	{ " VO_ADDR_VC0_TRAN_EN  ", VO_ADDR_VC0_TRAN_EN  },
	{ " VO_ADDR_VC0_TIME_OUT ", VO_ADDR_VC0_TIME_OUT },
	{ " VO_ADDR_VC0_ADDR_INFO", VO_ADDR_VC0_ADDR_INFO},
	{ " VO_ADDR_VC0_YUV_CLIP ", VO_ADDR_VC0_YUV_CLIP },
};
#if VO_DUMP_REG
static int vo_dump_regs(struct ingenic_vo *vofb)
{
	int i = 0;
	int num = 0;

	if (vofb == NULL) {
		dev_err(vofb->dev, "vo is NULL!\n");
		return -1;
	}
	printk("----- dump regs -----\n");
	num = sizeof(ingenic_vo_regs_name) / sizeof(struct vo_reg_struct);

	for (i = 0; i < num; i++) {
		printk("vo_reg: %s: \t0x%08x\r\n", ingenic_vo_regs_name[i].name, reg_readl(vofb, ingenic_vo_regs_name[i].addr));
	}

	return 0;

}
#endif
static void reg_bit_set(struct ingenic_vo *vo, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = reg_readl(vo, offset);
	reg |= bit;
	reg_writel(vo, offset, reg);
}

static void reg_bit_clr(struct ingenic_vo *vo, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = reg_readl(vo, offset);
	reg &= ~(bit);
	reg_writel(vo, offset, reg);
}

static int vo_hw_reset(struct ingenic_vo *vofb)
{

	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000001);
	while ((reg_readl(vofb, VO_ADDR_TOP_STATUS) & 0x1) != 1);
	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000003);
	mdelay(5);
	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000001);
	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000000);
	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000004);

	return 0;
}

static int vo_hw_start(struct ingenic_vo *vofb)
{
	reg_writel(vofb, VO_ADDR_INT_EN, 0x00000003);

	__start_vo();

	return 0;
}

static int vo_hw_stop(struct ingenic_vo *vofb)
{
	reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000001);
	if ((reg_readl(vofb, VO_ADDR_TOP_STATUS) & 0x1) == 1) {
		reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000003);
		reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000001);
		reg_writel(vofb, VO_ADDR_TOP_CON, 0x00000000);
	}

	return 0;
}

static void vo_hw_cfg(struct ingenic_vo *vofb, struct vo_param *vo_param)
{
	reg_writel(vofb, VO_ADDR_VC0_SIZE, (vo_param->src_h << 16 | vo_param->src_w));
	reg_writel(vofb, VO_ADDR_VC0_STRIDE, (vo_param->src_uv_strid << 16 | vo_param->src_y_strid));

	//reg_writel(vofb, VO_ADDR_VC0_CON_PAR, (vo_param->line_full_thr << 16 | vo_param->bt_mode | (3 << 6)));  //12bit 0:display  1:transmit
	reg_writel(vofb, VO_ADDR_VC0_CON_PAR, (vo_param->line_full_thr << 16 | vo_param->bt_mode | 1 << 3));   //12bit 0:display  1:transmit

	reg_writel(vofb, VO_ADDR_VC0_VBLK_PAR, (vo_param->vbb_num << 16 | vo_param->vfb_num));
	reg_writel(vofb, VO_ADDR_VC0_HBLK_PAR, vo_param->hfb_num);

	//4. Input image addrand
	reg_writel(vofb, VO_ADDR_VC0_Y_ADDR, vo_param->src_addr_y);
	reg_writel(vofb, VO_ADDR_VC0_UV_ADDR, vo_param->src_addr_uv);

}

static int vo_hw_update_addr(struct ingenic_vo *vofb, unsigned int y_addr, unsigned int uv_addr)
{
	unsigned int addr_info = 0;
	int timeout = 1000;
	/*Special Sequence must be take care, first y_addr, then uv_addr*/

	/*1. wait until addr fifo not full.*/

	while (--timeout) {
		addr_info = reg_readl(vofb, VO_ADDR_VC0_ADDR_INFO);

		if ((addr_info & (1 << 3)) != 1) {
			break;
		}

		udelay(10);
	}

	if (!timeout) {
		dev_err(vofb->dev, "waiting vo addr fifo not empty timedout!\n");
	}
	/*2. update addr fifo.*/

	reg_writel(vofb, VO_ADDR_VC0_Y_ADDR, y_addr);
	reg_writel(vofb, VO_ADDR_VC0_UV_ADDR, uv_addr);

	return 0;
}

ssize_t vofb_write(struct fb_info *info, const char __user *buf,
                   size_t count, loff_t *ppos)
{
	struct ingenic_vo *vofb = info->par;
	u8 *buffer, *src;
	u8 __iomem *dst;
	int c, cnt = 0, err = 0;
	unsigned long total_size;
	unsigned long p = *ppos;
	int screen_base_offset = 0;
	int next_frm = 0;
	struct vo_param *param = &vofb->param;

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

		fb_memcpy_tofb(dst, src, c);
		dst += c;
		src += c;
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	vo_hw_update_addr(vofb, param->src_addr_y, param->src_addr_uv);
end:
	return (cnt) ? cnt : err;
}

static struct fb_ops vfb_ops = {
	.fb_read        = fb_sys_read,
	.fb_write       = vofb_write,
	.fb_check_var   = vfb_check_var,
	.fb_set_par = vfb_set_par,
	.fb_setcolreg   = vfb_setcolreg,
	.fb_pan_display = vfb_pan_display,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_mmap    = vfb_mmap,
};

/*
 *  Internal routines
 */

static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return (length);
}

/*
 *  Setting the video mode has been split into two parts.
 *  First part, xxxfb_check_var, must not write anything
 *  to hardware, it should only verify and adjust var.
 *  This means it doesn't alter par but it does use hardware
 *  data from it to check this var.
 */

static int vfb_check_var(struct fb_var_screeninfo *var,
                         struct fb_info *info)
{

	printk("---check var:\n");

	printk("var->width: %d var->height: %d\n", var->width, var->height);
	printk("var->xres: %d var->yres: %d\n", var->xres, var->yres);

	return 0;
}

/* This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 */
static int vfb_set_par(struct fb_info *info)
{
	struct ingenic_vo *vofb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct vo_param *param = &vofb->param;

	info->fix.line_length = get_line_length(info->var.xres_virtual,
	                                        info->var.bits_per_pixel);

	printk("var->width: %d var->height: %d\n", var->width, var->height);
	printk("var->xres: %d var->yres: %d\n", var->xres, var->yres);

	param->src_w = var->xres;
	param->src_h = var->yres;

	param->src_uv_strid = var->xres;
	param->src_y_strid = var->xres;

	param->line_full_thr = 2;
	param->bt_mode = 1;

	param->vfb_num = 16;
	param->vbb_num = 16;
	param->hfb_num = 16;

	param->src_addr_y = vofb->videomemory_phys;
	param->src_addr_uv = param->src_addr_y + var->xres * var->yres;

	return 0;
}

/*
 *  Set a single color register. The values supplied are already
 *  rounded down to the hardware's capabilities (according to the
 *  entries in the var structure). Return != 0 for invalid regno.
 */

static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
                         u_int transp, struct fb_info *info)
{
	if (regno >= 256) { /* no. of hw registers */
		return 1;
	}

	return 0;
}

/*
 *  Pan or Wrap the Display
 *
 *  This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 */

static int vfb_pan_display(struct fb_var_screeninfo *var,
                           struct fb_info *info)
{
	struct ingenic_vo *vofb = info->par;
	struct vo_param *param = &vofb->param;
	int nframe = 0;
	unsigned int addr_y = 0;
	unsigned int addr_uv = 0;

	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset >= info->var.yres_virtual ||
		    var->xoffset) {
			return -EINVAL;
		}
	} else {
		if (var->xoffset + info->var.xres > info->var.xres_virtual ||
		    var->yoffset + info->var.yres > info->var.yres_virtual) {
			return -EINVAL;
		}
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	nframe = var->yoffset / var->yres;

	addr_y = param->src_addr_y + nframe * info->var.xres * info->var.yres * 3 / 2;
	addr_uv = addr_y + info->var.xres * info->var.yres;

	vo_hw_update_addr(vofb, addr_y, addr_uv);

	return 0;
}

/*
 *  Most drivers don't need their own mmap function
 */

static int vfb_mmap(struct fb_info *info,
                    struct vm_area_struct *vma)
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
	vma->vm_flags |= VM_IO;

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	/* Write-Acceleration */
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_WA;

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
	                       vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static irqreturn_t vofb_irq_handler(int irq, void *data)
{
	struct ingenic_vo *vofb = (struct ingenic_vo *)data;
	unsigned int status = 0;
	struct vo_param *param = &vofb->param;

	status = reg_readl(vofb, VO_ADDR_INT_VAL);

	if (status & 0x1) {
		//complete(&vo->done_vo);
		reg_bit_set(vofb, VO_ADDR_INT_CLC, 1 << 0);
		//vo_hw_update_addr(vofb, param->src_addr_y, param->src_addr_uv);
	}
	if (status & 0x2) {

		printk("--vo-error---!\n");
		reg_bit_set(vofb, VO_ADDR_INT_CLC, 1 << 1);
	}

	return IRQ_HANDLED;
}

/*
 *  Initialisation
 */

static int vo_fb_probe(struct platform_device *dev)
{
	struct ingenic_vo *vofb = NULL;
	struct fb_info *info = NULL;
	int retval = -ENOMEM;

	/*
	 * For real video cards we use ioremap.
	 */

	info = framebuffer_alloc(sizeof(struct ingenic_vo), &dev->dev);
	if (!info) {
		goto err;
	}

	vofb = info->par;
	vofb->info = info;

	vofb->dev = &dev->dev;
	vofb->iomem = of_iomap(vofb->dev->of_node, 0);
	if (!vofb->iomem) {
		dev_err(vofb->dev, "Failed to ioremap register VO memory region\n");
		goto err;
	}

	vofb->irq = platform_get_irq(dev, 0);
	if (devm_request_irq(vofb->dev, vofb->irq, vofb_irq_handler, 0, "ingenic vo", vofb)) {
		dev_err(vofb->dev, "request irq failed!\n");
		retval = -EINVAL;
		goto err_irq_req;
	}

	vofb->clk_div = devm_clk_get(vofb->dev, "div_bt0");
	if (IS_ERR_OR_NULL(vofb->clk_div)) {
		dev_err(vofb->dev, "vofb clk get div_bt0 failed\n");
	}

	vofb->clk = devm_clk_get(vofb->dev, "gate_vo");
	if (IS_ERR_OR_NULL(vofb->clk)) {
		dev_err(vofb->dev, "vofb clk get gate_vo failed!\n");

		retval = -EINVAL;
		//TODO goto:
	}

	//clk_set_rate(vofb->clk_div, 74000000);
	clk_set_rate(vofb->clk_div, 10000000);

	clk_prepare_enable(vofb->clk_div);

	clk_prepare_enable(vofb->clk);

	if (rvmalloc(vofb, VIDEOMEMSIZE) < 0) {
		return retval;
	}

	info->screen_base = (char __iomem *)vofb->videomemory;
	info->fbops = &vfb_ops;

	retval = fb_find_mode(&info->var, info, NULL,
	                      NULL, 0, NULL, 8);

	info->var = vfb_default;

	vfb_fix.smem_start = (unsigned long) vofb->videomemory_phys;
	vfb_fix.smem_len = vofb->videomemorysize;
	info->fix = vfb_fix;
	info->flags = FBINFO_FLAG_DEFAULT;

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0) {
		goto err1;
	}

	retval = register_framebuffer(info);
	if (retval < 0) {
		goto err2;
	}
	platform_set_drvdata(dev, info);

	fb_info(info, "Virtual frame buffer device, using %ldK of video memory\n",
	        vofb->videomemorysize >> 10);

	vfb_set_par(vofb->info);
	vo_hw_reset(vofb);
	vo_hw_cfg(vofb, &vofb->param);
	vo_hw_start(vofb);
#if VO_DUMP_REG
	vo_dump_regs(vofb);
#endif
	return 0;
err2:
	fb_dealloc_cmap(&info->cmap);
err1:
err_irq_req:
	framebuffer_release(info);
err:
	rvfree(vofb, vofb->videomemory, vofb->videomemorysize);
	return retval;
}

static int vo_fb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct ingenic_vo *vofb = info->par;

	if (info) {
		unregister_framebuffer(info);
		rvfree(vofb, vofb->videomemory, vofb->videomemorysize);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int vo_suspend(struct device *dev)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct ingenic_vo *vofb = info->par;
	clk_disable_unprepare(vofb->clk_div);
	clk_disable_unprepare(vofb->clk);
	return 0;
}

static int vo_resume(struct device *dev)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct ingenic_vo *vofb = info->par;
	clk_prepare_enable(vofb->clk);
	clk_prepare_enable(vofb->clk_div);
	return 0;
}
static SIMPLE_DEV_PM_OPS(vo_pm_ops, vo_suspend, vo_resume);

#endif

static const struct of_device_id ingenic_vo_dt_match[] = {
	{ .compatible = "ingenic,x2500-vo", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_vo_dt_match);

static struct platform_driver ingenic_vo_driver = {
	.probe  = vo_fb_probe,
	.remove = vo_fb_remove,
	.driver = {
		.name = "ingenic-vo",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_vo_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &vo_pm_ops,
#endif

	},
};

module_platform_driver(ingenic_vo_driver)

MODULE_DESCRIPTION("Ingenic VO driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
