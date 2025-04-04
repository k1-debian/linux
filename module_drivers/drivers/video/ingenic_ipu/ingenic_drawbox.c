/*
 * Copyright (c) 2015 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Input file for Ingenic DBOX driver
 *
 * This  program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
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
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/time.h>
#include <soc/base.h>
#include <linux/dma-buf.h>
#include <linux/version.h>
#include <linux/dma-map-ops.h>
#include "ingenic_drawbox.h"

// #define DEBUG
#ifdef  DEBUG
	static int debug_dbox = 1;

	#define DBOX_DEBUG(format, ...) { if (debug_dbox) printk(format, ## __VA_ARGS__);}
#else
	#define DBOX_DEBUG(format, ...) do{ } while(0)
#endif

#define DBOX_BUF_SIZE (1024 * 1024 * 2)

struct dbox_reg_struct jz_dbox_regs_name[] = {
	{"DBOX_CTRL", DBOX_CTRL},
	{"DBOX_YB", DBOX_YB},
	{"DBOX_CB", DBOX_CB},
	{"DBOX_STRIDE", DBOX_STRIDE},
	{"DBOX_IMG_WH", DBOX_IMG_WH},
	{"DBOX_COLOR_Y", DBOX_COLOR_Y},
	{"DBOX_COLOR_U", DBOX_COLOR_U},
	{"DBOX_COLOR_V", DBOX_COLOR_V},
	{"DBOX_RAM", DBOX_RAM},
	{"DBOX_TIMEOUT", DBOX_TIMEOUT},
};

static void reg_bit_set(struct jz_dbox *dbox, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = dbox_reg_read(dbox, offset);
	reg |= bit;
	dbox_reg_write(dbox, offset, reg);
}

static void reg_bit_clr(struct jz_dbox *dbox, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = dbox_reg_read(dbox, offset);
	reg &= ~(bit);
	dbox_reg_write(dbox, offset, reg);
}

static int _dbox_dump_regs(struct jz_dbox *dbox)
{
	int i = 0;
	int num = 0;

	if (dbox == NULL) {
		dev_err(dbox->dev, "dbox is NULL!\n");
		return -1;
	}
	printk("----- dump regs -----\n");

	num = sizeof(jz_dbox_regs_name) / sizeof(struct dbox_reg_struct);
	for (i = 0; i < num; i++) {
		printk("dbox_reg: %s: \t0x%08x\r\n", jz_dbox_regs_name[i].name, dbox_reg_read(dbox, jz_dbox_regs_name[i].addr));
	}

	return 0;
}

static void _dbox_dump_param(struct jz_dbox *dbox)
{
	return;
}

static int dbox_dump_info(struct jz_dbox *dbox)
{
	int ret = 0;
	if (dbox == NULL) {
		dev_err(dbox->dev, "dbox is NULL\n");
		return -1;
	}
	printk("dbox: dbox->base: %p\n", dbox->iomem);
	_dbox_dump_param(dbox);
	ret = _dbox_dump_regs(dbox);

	return ret;
}

static int dbox_reg_set(struct jz_dbox *dbox, struct dbox_param *dbox_param)
{
	unsigned int imgw = 0;
	unsigned int imgh = 0;
	unsigned int rectw[DBOX_RAM_SIZE];
	unsigned int recth[DBOX_RAM_SIZE];
	unsigned int rectx[DBOX_RAM_SIZE];
	unsigned int recty[DBOX_RAM_SIZE];
	unsigned int linew[DBOX_RAM_SIZE];
	unsigned int linel[DBOX_RAM_SIZE];
	// unsigned int imgstride = 0;
	unsigned int img_pbuf_y = 0;
	unsigned int img_pbuf_uv = 0;
	unsigned int boxs_num = 0;
	unsigned int is_rgba = 0;
	unsigned int i = 0;

	unsigned int dbox_colormode[DBOX_RAM_SIZE];
	unsigned int dbox_mode[DBOX_RAM_SIZE];

	struct dbox_param *ip = dbox_param;
	if (dbox == NULL) {
		dev_err(dbox->dev, "dbox: dbox is NULL or dbox_param is NULL\n");
		return -1;
	}

	img_pbuf_y = ip->box_pbuf;

	imgw = ip->img_w;
	imgh = ip->img_h;
	boxs_num = ip->boxs_num;
	is_rgba = ip->is_rgba;

	if (1 == is_rgba) {
		reg_bit_set(dbox, DBOX_CTRL, DBOX_IS_BGRA);
	} else {
		reg_bit_clr(dbox, DBOX_CTRL, DBOX_IS_BGRA);
	}

	img_pbuf_y = ip->box_pbuf;
	img_pbuf_uv = img_pbuf_y + imgw * imgh;
	//printk("imgw = %d imgh = %d\n", imgw, imgh);

	dbox_reg_write(dbox, DBOX_IMG_WH, (imgw << DBOX_WIDTH | imgh << DBOX_HEIGHT));

	dbox_reg_write(dbox, DBOX_STRIDE, ((imgw << DBOX_Y_STRIDE) | (imgw << DBOX_C_STRIDE)));

	/* YUV[0] is red
	 * YUV[1] is black
	 * YUV[2] is green
	 * YUV[3] is yellow
	 * rgb2yuv ==> https://www.mikekohn.net/file_formats/yuv_rgb_converter.php */
	dbox_reg_write(dbox, DBOX_COLOR_Y, 0xe195004c);
	dbox_reg_write(dbox, DBOX_COLOR_U, 0x002b8054);
	dbox_reg_write(dbox, DBOX_COLOR_V, 0x941580ff);

	dbox_reg_write(dbox, DBOX_YB, (img_pbuf_y << DBOX_LUMA_BASE));
	dbox_reg_write(dbox, DBOX_CB, (img_pbuf_uv << DBOX_CONCENTRATION_BASE));

	for (i = 0; i < boxs_num; i++) {
		rectx[i] = ip->ram_para[i].box_x;
		recty[i] = ip->ram_para[i].box_y;
		rectw[i] = ip->ram_para[i].box_w;
		recth[i] = ip->ram_para[i].box_h;
		linew[i] = ip->ram_para[i].line_w;
		linel[i] = ip->ram_para[i].line_l;
		dbox_mode[i] = ip->ram_para[i].box_mode;
		dbox_colormode[i] = ip->ram_para[i].color_mode;
		dbox_reg_write(dbox, DBOX_RAM, (rectx[i] << DBOX_BOX_X) | (recty[i] << DBOX_BOX_Y) | (rectw[i] << DBOX_BOX_WIDTH));
		dbox_reg_write(dbox, DBOX_RAM, (((rectw[i] >> 8) << 0) | (recth[i] << 4) | (dbox_colormode[i] << 16) | (dbox_mode[i] << 18) | (linew[i] << 20) | (linel[i] << 23)));
	}

	return 0;
}

static int dbox_start(struct jz_dbox *dbox, struct dbox_param *dbox_param)
{
	int ret = 0;
	struct dbox_param *ip = dbox_param;

	if ((dbox == NULL) || (dbox_param == NULL)) {
		dev_err(dbox->dev, "dbox: dbox is NULL or dbox_param is NULL\n");
		return -1;
	}
	DBOX_DEBUG("dbox: enter dbox_start %d\n", current->pid);

#ifndef CONFIG_FPGA_TEST
	clk_prepare_enable(dbox->clk);
#endif

	__reset_dbox();
	/* wait reset complete */
	while (dbox_reg_read(dbox, DBOX_CTRL) & 0x2) {
		udelay(10);
	}

	ret = dbox_reg_set(dbox, ip);
	__dbox_mask_irq();//mask 0

	DBOX_DEBUG("dbox_start\n");
	/* start dbox */
	__start_dbox();

#ifdef DEBUG
	//dbox_dump_info(dbox);
#endif
	//DBOX_DEBUG("dbox_start\n");

	ret = wait_for_completion_interruptible_timeout(&dbox->done_dbox, msecs_to_jiffies(5000));
	if (ret < 0) {
		printk("dbox: done_dbox wait_for_completion_interruptible_timeout err %d\n", ret);
		goto err_dbox_wait_for_done;
	} else if (ret == 0) {
		ret = -1;
		printk("dbox: done_dbox wait_for_completion_interruptible_timeout timeout %d\n", ret);
		dbox_dump_info(dbox);
		goto err_dbox_wait_for_done;
	} else {
		;
	}

	DBOX_DEBUG("dbox: exit dbox_start %d\n", current->pid);

#ifndef CONFIG_FPGA_TEST
	clk_disable_unprepare(dbox->clk);
#endif

	return 0;

err_dbox_wait_for_done:
#ifndef CONFIG_FPGA_TEST
	clk_disable_unprepare(dbox->clk);
#endif

	return ret;

}

#ifdef CONFIG_VIDEO_DEV
static int dbox_dmabuf_get_phy(struct device *dev, int fd, unsigned int *phyaddr)
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
static long dbox_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct dbox_param iparam;
	struct miscdevice *dev = filp->private_data;
	struct jz_dbox *dbox = container_of(dev, struct jz_dbox, misc_dev);

	DBOX_DEBUG("dbox: %s pid: %d, tgid: %d file: %p, cmd: 0x%08x\n",
	           __func__, current->pid, current->tgid, filp, cmd);

	if (_IOC_TYPE(cmd) != JZDBOX_IOC_MAGIC) {
		dev_err(dbox->dev, "invalid cmd!\n");
		return -EFAULT;
	}

	mutex_lock(&dbox->mutex);

	switch (cmd) {
		case IOCTL_DBOX_START:
			if (copy_from_user(&iparam, (void *)arg, sizeof(struct dbox_param))) {
				dev_err(dbox->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			ret = dbox_start(dbox, &iparam);
			if (ret) {
				printk("dbox: error dbox start ret = %d\n", ret);
			}
			break;
		case IOCTL_DBOX_GET_PBUFF:
			if (dbox->pbuf.vaddr_alloc == 0) {
				unsigned int size = DBOX_BUF_SIZE;
				dbox->pbuf.vaddr_alloc = (unsigned int)kmalloc(size, GFP_KERNEL);
				if (!dbox->pbuf.vaddr_alloc) {
					printk("dbox kmalloc is error\n");
					ret = -ENOMEM;
				}
				memset((void *)(dbox->pbuf.vaddr_alloc), 0x00, size);
				dbox->pbuf.size = size;
				dbox->pbuf.paddr = virt_to_phys((void *)(dbox->pbuf.vaddr_alloc));
				dbox->pbuf.paddr_align = ((unsigned long)(dbox->pbuf.paddr));
			}
			DBOX_DEBUG("dbox: %s dbox->pbuf.vaddr_alloc = 0x%08x\ndbox->pbuf.vaddr_align = 0x%08x\ndbox->pbuf.size = 0x%x\ndbox->pbuf.paddr = 0x%08x\n"
			           , __func__, dbox->pbuf.vaddr_alloc, dbox->pbuf.paddr_align, dbox->pbuf.size, dbox->pbuf.paddr);
			if (copy_to_user((void *)arg, &dbox->pbuf, sizeof(struct dbox_buf_info))) {
				dev_err(dbox->dev, "copy_to_user error!!!\n");
				ret = -EFAULT;
			}
			break;
		case IOCTL_DBOX_RES_PBUFF:
			if (dbox->pbuf.vaddr_alloc != 0) {
				kfree((void *)dbox->pbuf.vaddr_alloc);
				dbox->pbuf.vaddr_alloc = 0;
				dbox->pbuf.size = 0;
				dbox->pbuf.paddr = 0;
				dbox->pbuf.paddr_align = 0;
			} else {
				dev_warn(dbox->dev, "buffer wanted to free is null\n");
			}
			break;
		case IOCTL_DBOX_BUF_LOCK:
			ret = wait_for_completion_interruptible_timeout(&dbox->done_buf, msecs_to_jiffies(2000));
			if (ret < 0) {
				printk("dbox: done_buf wait_for_completion_interruptible_timeout err %d\n", ret);
			} else if (ret == 0) {
				printk("dbox: done_buf wait_for_completion_interruptible_timeout timeout %d\n", ret);
				ret = -1;
				dbox_dump_info(dbox);
			} else {
				ret = 0;
			}
			break;
		case IOCTL_DBOX_BUF_UNLOCK:
			complete(&dbox->done_buf);
			break;
		case IOCTL_DBOX_BUF_FLUSH_CACHE: {
			struct dbox_flush_cache_para fc;
			if (copy_from_user(&fc, (void *)arg, sizeof(fc))) {
				dev_err(dbox->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			//dma_sync_single_for_device(NULL, fc.addr, fc.size, DMA_TO_DEVICE);
			//dma_sync_single_for_device(NULL, fc.addr, fc.size, DMA_FROM_DEVICE);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10, 0)
			dma_cache_wback_inv((phys_addr_t)fc.addr, fc.size);
#else
			dma_cache_sync(NULL, fc.addr, fc.size, DMA_BIDIRECTIONAL);
#endif
		}
		break;
#ifdef CONFIG_VIDEO_DEV
		case IOCTL_DBOX_GET_DMA_PHY: {
			unsigned int phyaddr = 0;
			int fd = 0;
			if (copy_from_user(&fd, (void *)arg, sizeof(fd))) {
				dev_err(dbox->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			ret = dbox_dmabuf_get_phy(dbox->dev, fd, &phyaddr);
			if (copy_to_user((void *)arg, &phyaddr, sizeof(phyaddr))) {
				return -EFAULT;
			}
		}
		break;
#endif
		default:
			dev_err(dbox->dev, "invalid command: 0x%08x\n", cmd);
			ret = -EINVAL;
	}

	mutex_unlock(&dbox->mutex);
	return ret;
}

static int dbox_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct miscdevice *dev = filp->private_data;
	struct jz_dbox *dbox = container_of(dev, struct jz_dbox, misc_dev);

	DBOX_DEBUG("dbox: %s pid: %d, tgid: %d filp: %p\n",
	           __func__, current->pid, current->tgid, filp);
	mutex_lock(&dbox->mutex);

	mutex_unlock(&dbox->mutex);
	return ret;
}

static int dbox_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct miscdevice *dev = filp->private_data;
	struct jz_dbox *dbox = container_of(dev, struct jz_dbox, misc_dev);

	DBOX_DEBUG("dbox: %s  pid: %d, tgid: %d filp: %p\n",
	           __func__, current->pid, current->tgid, filp);
	mutex_lock(&dbox->mutex);

	mutex_unlock(&dbox->mutex);
	return ret;
}

static struct file_operations dbox_ops = {
	.owner = THIS_MODULE,
	.open = dbox_open,
	.release = dbox_release,
	.unlocked_ioctl = dbox_ioctl,
};

static irqreturn_t dbox_irq_handler(int irq, void *data)
{
	struct jz_dbox *dbox;
	unsigned int status;

	DBOX_DEBUG("dbox: %s\n", __func__);
	dbox = (struct jz_dbox *)data;

	status = dbox_reg_read(dbox, DBOX_CTRL);

	DBOX_DEBUG("----- %s, status= 0x%08x\n", __func__, status);
	/* this status doesn't do anything including trigger interrupt,
	* just give a hint */
	if (status & 0x10) {
		complete(&dbox->done_dbox);
	}

	__dbox_irq_clear();

	return IRQ_HANDLED;
}

static int dbox_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct jz_dbox *dbox;

	DBOX_DEBUG("%s\n", __func__);
	dbox = (struct jz_dbox *)kzalloc(sizeof(struct jz_dbox), GFP_KERNEL);
	if (!dbox) {
		dev_err(&pdev->dev, "alloc jz_dbox failed!\n");
		return -ENOMEM;
	}

	sprintf(dbox->name, "dbox");

	dbox->misc_dev.minor = MISC_DYNAMIC_MINOR;
	dbox->misc_dev.name = dbox->name;
	dbox->misc_dev.fops = &dbox_ops;
	dbox->dev = &pdev->dev;

	mutex_init(&dbox->mutex);
	init_completion(&dbox->done_dbox);
	init_completion(&dbox->done_buf);
	complete(&dbox->done_buf);

	dbox->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!dbox->res) {
		dev_err(&pdev->dev, "failed to get dev resources: %d\n", ret);
		ret = -EINVAL;
		goto err_get_platform_res;
	}

	dbox->res = request_mem_region(dbox->res->start,
	                               dbox->res->end - dbox->res->start + 1,
	                               pdev->name);
	if (!dbox->res) {
		dev_err(&pdev->dev, "failed to request regs memory region");
		ret = -EINVAL;
		goto err_get_mem_region;
	}
	dbox->iomem = ioremap(dbox->res->start, resource_size(dbox->res));
	if (!dbox->iomem) {
		dev_err(&pdev->dev, "failed to remap regs memory region: %d\n", ret);
		ret = -EINVAL;
		goto err_ioremap;
	}

	dbox->irq = platform_get_irq(pdev, 0);
	if (request_irq(dbox->irq, dbox_irq_handler, IRQF_SHARED, dbox->name, dbox)) {
		dev_err(&pdev->dev, "request irq failed\n");
		ret = -EINVAL;
		goto err_req_irq;
	}

#ifndef CONFIG_FPGA_TEST
	dbox->clk = devm_clk_get(dbox->dev, "gate_drawbox");
	if (IS_ERR(dbox->clk)) {
		dev_err(&pdev->dev, "dbox clk get failed!\n");
		ret = -EINVAL;
		goto err_get_dbox_clk;
	}
#endif

	dev_set_drvdata(&pdev->dev, dbox);

	__reset_dbox();
	/* wait reset complete */
	while (dbox_reg_read(dbox, DBOX_CTRL) & 0x2) {
		udelay(10);
	}

	ret = misc_register(&dbox->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register misc device failed!\n");
		goto err_misc_register;
	}

	printk("JZ DBOX probe ok!!!!\n");
	return 0;

err_misc_register:
#ifndef CONFIG_FPGA_TEST
	devm_clk_put(dbox->dev, dbox->clk);
err_get_dbox_clk:
#endif
	free_irq(dbox->irq, dbox);
err_req_irq:
	iounmap(dbox->iomem);
err_ioremap:
	release_mem_region(dbox->res->start, dbox->res->end - dbox->res->start + 1);
err_get_platform_res:
err_get_mem_region:
	kfree(dbox);

	return ret;
}

static int dbox_remove(struct platform_device *pdev)
{
	struct jz_dbox *dbox;
	struct resource *res;
	DBOX_DEBUG("%s\n", __func__);

	dbox = dev_get_drvdata(&pdev->dev);
	misc_deregister(&dbox->misc_dev);
#ifndef CONFIG_FPGA_TEST
	devm_clk_put(dbox->dev, dbox->clk);
#endif
	res = dbox->res;
	free_irq(dbox->irq, dbox);
	iounmap(dbox->iomem);
	release_mem_region(res->start, res->end - res->start + 1);

	if (dbox->pbuf.vaddr_alloc) {
		kfree((void *)(dbox->pbuf.vaddr_alloc));
		dbox->pbuf.vaddr_alloc = 0;
	}
	if (dbox) {
		kfree(dbox);
	}

	return 0;
}

static const struct of_device_id ingenic_dbox_dt_match[] = {
	{ .compatible = "ingenic,x2500-drawbox", .data = NULL },
	{},
};

static struct platform_driver jz_dbox_driver = {
	.probe  = dbox_probe,
	.remove = dbox_remove,
	.driver = {
		.name = "jz-drawbox",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_dbox_dt_match),
	},
};

static int __init dboxdev_init(void)
{
	DBOX_DEBUG("%s\n", __func__);
	platform_driver_register(&jz_dbox_driver);
	return 0;
}

static void __exit dboxdev_exit(void)
{
	DBOX_DEBUG("%s\n", __func__);
	platform_driver_unregister(&jz_dbox_driver);
}

module_init(dboxdev_init);
module_exit(dboxdev_exit);
MODULE_DESCRIPTION("JZ IPU driver");
MODULE_AUTHOR("Ferdinand Jia <bcjia@ingenic.cn>");
MODULE_LICENSE("GPL");
