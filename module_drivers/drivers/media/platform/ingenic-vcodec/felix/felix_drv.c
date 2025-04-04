/*
 * Copyright (c) 2014 Ingenic Inc.
 * Author: qipengzhen <aric.pzqi@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
//#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <media/ingenic_video_nr.h>

#include "felix_drv.h"
#include "felix_ops.h"

#include "h264dec.h"



#ifdef CONFIG_FELIX_INIT_BY_USER
	#include <user_triggered_module_init.h>
#endif

#undef pr_debug
#define pr_debug pr_info

extern unsigned int max_frame_buffers;
extern unsigned int max_out_frame_buffers;

/*
TODO: do not support multi ctx for now.
*/

static int vpu_on(struct ingenic_vdec_dev *dev);
static int vpu_off(struct ingenic_vdec_dev *dev);
extern struct vpu_ops ingenic_vpu_ops;

static int fops_vcodec_open(struct file *file)
{
	struct ingenic_vdec_dev *dev = video_drvdata(file);
	struct ingenic_vdec_ctx *ctx = NULL;
	devmem_ctx_t tmp_dev_ctx;
	int ret = 0;

	//  printk("%s:%d sizeof(*ctx): %d, sizeof(AVCodecContext):%d, sizeof(H264Context):%d\n", __func__, __LINE__, sizeof(*ctx), sizeof(AVCodecContext), sizeof(H264Context));

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		return -ENOMEM;
	}

	ctx->avctx = kzalloc(sizeof(AVCodecContext), GFP_KERNEL);
	if (!ctx->avctx) {
		kfree(ctx);
		return -ENOMEM;
	}

	{
		/* Because of H264Context size too big, alloc buffer for reserved memory! */
		tmp_dev_ctx.dev = dev->dev;
		ctx->h_avbuf = av_buffer_alloc(&tmp_dev_ctx, sizeof(H264Context));
		if (ctx->h_avbuf) {
			ctx->h = (H264Context *)ctx->h_avbuf->buffer;
		} else {
			ctx->h = NULL;
		}
	}
	if (!ctx->h) {
		kfree(ctx->avctx);
		kfree(ctx);
		return -ENOMEM;
	}
	memset(ctx->h, 0, sizeof(H264Context));

	ctx->avctx->priv_data = ctx->h;
	//ctx->avctx->flags2 |= AV_CODEC_FLAG2_CHUNKS;
	//ctx->avctx->debug |= FF_DEBUG_PICT_INFO;
	ctx->h->devmem_ctx.dev = dev->dev;

	ret = h264_decode_init(ctx->avctx);
	if (ret < 0) {
		av_buffer_free(&tmp_dev_ctx, ctx->h_avbuf);
		kfree(ctx->avctx);
		kfree(ctx);
		return -EINVAL;
	}

	h264_set_vpu_ops(ctx->h, ctx, &ingenic_vpu_ops);

	mutex_lock(&dev->dev_mutex);

	mutex_init(&ctx->lock);

	if (!dev->id_counter) {
		vpu_on(dev);
	}

	ctx->id = dev->id_counter++;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	//INIT_LIST_HEAD(&ctx->list);
	ctx->dev = dev;
	init_waitqueue_head(&ctx->queue);


	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &ingenic_vcodec_vdec_queue_init);
	if (IS_ERR((__force void *)ctx->m2m_ctx)) {
		ret = PTR_ERR((__force void *)ctx->m2m_ctx);

		goto err_m2m_ctx_init;
	}

	ingenic_vcodec_init_default_params(ctx);
	pr_debug("Create instance [%d]@%p m2m_ctx=%p\n",
	         ctx->id, ctx, ctx->m2m_ctx);

	mutex_unlock(&dev->dev_mutex);

	pr_debug("%s vcodec [%d]\n", dev_name(dev->dev), ctx->id);

	return ret;
err_m2m_ctx_init:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	av_buffer_free(&tmp_dev_ctx, ctx->h_avbuf);
	kfree(ctx->avctx);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int fops_vcodec_release(struct file *file)
{

	struct ingenic_vdec_dev *dev = video_drvdata(file);
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(file->private_data);

	pr_debug("[%d] vcodec release\n", ctx->id);

	mutex_lock(&dev->dev_mutex);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);


	ingenic_vcodec_deinit_default_params(ctx);

	h264_decode_end(ctx->avctx);

	if (ctx->avctx) {
		kfree(ctx->avctx);
	}

	if (ctx->h) {
		devmem_ctx_t tmp_dev_ctx;
		tmp_dev_ctx.dev = ctx->h->devmem_ctx.dev;
		av_buffer_free(&tmp_dev_ctx, ctx->h_avbuf);
	}

	kfree(ctx);

	if (!(--dev->id_counter)) {
		vpu_off(dev);
	}


	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int fops_vcodec_poll(struct file *file,
                                     struct poll_table_struct *wait)
{
	struct ingenic_vdec_dev *dev = video_drvdata(file);
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(file->private_data);
	unsigned int ret = 0;

	if (mutex_lock_interruptible(&dev->dev_mutex)) {
		return -ERESTARTSYS;
	}

	ret = v4l2_m2m_poll(file, ctx->m2m_ctx, wait);

	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int fops_vcodec_mmap(struct file *file,
                            struct vm_area_struct *vma)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations ingenic_vdec_fops = {
	.owner      = THIS_MODULE,
	.open       = fops_vcodec_open,
	.release    = fops_vcodec_release,
	.poll       = fops_vcodec_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap       = fops_vcodec_mmap,
};


#define vpu_readl(dev, offset)  \
	readl(dev->reg_base + (offset))

#define vpu_writel(dev, offset, data)   \
	writel((data), dev->reg_base + (offset))

static void inline vpu_clear_bits(struct ingenic_vdec_dev *dev, unsigned int offset, unsigned int bits)
{
	unsigned int val = vpu_readl(dev, offset);
	val &= ~bits;
	vpu_writel(dev, offset, val);
}


#define REG_VPU_STATUS ( *(volatile unsigned int*)0xb3200034 )
#define REG_VPU_LOCK ( *(volatile unsigned int*)0xb329004c )
#define REG_VPUCDR ( *(volatile unsigned int*)0xb0000030 )
#define REG_CPM_VPU_SWRST ( *(volatile unsigned int*)0xb00000c4 )
#define REG_VPU_TLBBASE (*(volatile unsigned int *)(0x30 + 0xb3200000))
#define CPM_VPU_SR           (0x1<<31)
#define CPM_VPU_STP          (0x1<<30)
#define CPM_VPU_ACK          (0x1<<29)



#define REG_VPU_GLBC      0x00000
#define VPU_INTE_ACFGERR     (0x1<<20)
#define VPU_INTE_TLBERR      (0x1<<18)
#define VPU_INTE_BSERR       (0x1<<17)
#define VPU_INTE_ENDF        (0x1<<16)

#define REG_VPU_STAT      0x00034
#define VPU_STAT_ENDF    (0x1<<0)
#define VPU_STAT_BPF     (0x1<<1)
#define VPU_STAT_ACFGERR (0x1<<2)
#define VPU_STAT_TIMEOUT (0x1<<3)
#define VPU_STAT_JPGEND  (0x1<<4)
#define VPU_STAT_BSERR   (0x1<<7)

#define VPU_STAT_TLBERR  (0x1F<<10)
#define VPU_STAT_SLDERR  (0x1<<16)

#define REG_VPU_JPGC_STAT 0xE0008
#define JPGC_STAT_ENDF   (0x1<<31)

#define REG_VPU_SDE_STAT  0x90000
#define SDE_STAT_BSEND   (0x1<<1)

#define REG_VPU_DBLK_STAT 0x70070
#define DBLK_STAT_DOEND  (0x1<<0)

#define REG_VPU_AUX_STAT  0xA0010
#define AUX_STAT_MIRQP   (0x1<<0)

static irqreturn_t ingenic_vpu_irq(int irq, void *priv)
{
	struct ingenic_vdec_dev *dev = (struct ingenic_vdec_dev *)priv;
	struct ingenic_vdec_ctx *ctx = dev->curr_ctx;
	H264Context *h = ctx->h;
	vpu_ctx_t *vpu_ctx = &h->vpu_ctx;
	unsigned long flags;

	unsigned int vpu_stat;
	unsigned int sde_stat;
	int err = 0;

#define check_vpu_status(STAT, fmt, args...) do {       \
		if(vpu_stat & STAT)                     \
			dev_err(dev->dev, fmt, ##args);     \
	}while(0)

	spin_lock_irqsave(&dev->spinlock, flags);

	vpu_stat = vpu_readl(dev, REG_SCH_STAT);
	sde_stat = vpu_readl(dev, REG_SDE_STAT);

	if (vpu_stat) {
		if (vpu_stat & VPU_STAT_ENDF) {
			if (vpu_stat & VPU_STAT_JPGEND) {
				dev_dbg(dev->dev, "JPG successfully done!\n");
				vpu_stat = vpu_readl(dev, REG_VPU_JPGC_STAT);
				vpu_clear_bits(dev, REG_VPU_JPGC_STAT,
				               JPGC_STAT_ENDF);
			} else {
				dev_dbg(dev->dev, "SCH successfully done!\n");
				vpu_clear_bits(dev, REG_VPU_SDE_STAT,
				               SDE_STAT_BSEND);
				vpu_clear_bits(dev, REG_VPU_DBLK_STAT,
				               DBLK_STAT_DOEND);
			}
		} else {
			err = 1;
			check_vpu_status(VPU_STAT_SLDERR, "SHLD error!\n");
			check_vpu_status(VPU_STAT_TLBERR, "TLB error! Addr is 0x%08x\n",
			                 vpu_readl(dev, REG_VPU_STAT));
			check_vpu_status(VPU_STAT_BSERR, "BS error!\n");
			check_vpu_status(VPU_STAT_ACFGERR, "ACFG error!\n");
			check_vpu_status(VPU_STAT_TIMEOUT, "TIMEOUT error!\n");
			vpu_clear_bits(dev, REG_VPU_GLBC, (VPU_INTE_ACFGERR |
			                                   VPU_INTE_TLBERR | VPU_INTE_BSERR |
			                                   VPU_INTE_ENDF));
		}
	} else {
		if (vpu_readl(dev, REG_VPU_AUX_STAT) & AUX_STAT_MIRQP) {
			dev_dbg(dev->dev, "AUX successfully done!\n");
			vpu_clear_bits(dev, REG_VPU_AUX_STAT, AUX_STAT_MIRQP);
		} else {
			dev_dbg(dev->dev, "illegal interrupt happened!\n");
			err = 1;
			return IRQ_HANDLED;
		}
	}

	ctx->int_cond = 1;
	vpu_ctx->error = err;
	vpu_ctx->sch_stat = vpu_stat;
	vpu_ctx->sde_stat = sde_stat;
	//ctx->int_status = vpu_stat;
	spin_unlock_irqrestore(&dev->spinlock, flags);

	wake_up_interruptible(&ctx->queue);

	return IRQ_HANDLED;
}


static void vpu_dump_regs(struct ingenic_vdec_dev *dev)
{

#if 1
	/*SDE*/
	printk("REG_SDE_STAT:		0x%x\n", vpu_readl(dev, REG_SDE_STAT));
	printk("REG_SDE_CFG0:		0x%x\n", vpu_readl(dev, REG_SDE_CFG0));
	printk("REG_SDE_CFG1:		0x%x\n", vpu_readl(dev, REG_SDE_CFG1));
	printk("REG_SDE_CFG13:		0x%x\n", vpu_readl(dev, REG_SDE_CFG13));
	printk("REG_SDE_CFG14:		0x%x\n", vpu_readl(dev, REG_SDE_CFG14));
	printk("REG_DBLK_GSTA:		0x%x\n", vpu_readl(dev, REG_DBLK_GSTA));
	printk("REG_VMAU_POS:		0x%x\n", vpu_readl(dev, REG_VMAU_POS));
	printk("REG_SCH_STAT:		0x%x\n", vpu_readl(dev, REG_SCH_STAT));
#endif
}


#define VPU_RUN_TIMEOUT_MS  (3000)

#ifdef CONFIG_SOC_M200
static int vpu_reset_m200(struct ingenic_vdec_dev *dev)
{
	int timeout = 0xffffff;

	REG_CPM_VPU_SWRST |= CPM_VPU_STP;
	while (!(REG_CPM_VPU_SWRST & CPM_VPU_ACK) && --timeout)
		;

	if (!timeout) {
		dev_err(dev->dev,
		        "[%d:%d] wait stop ack timeout when stop VPU\n",
		        current->tgid, current->pid);
	}

	REG_CPM_VPU_SWRST = ((REG_CPM_VPU_SWRST | CPM_VPU_SR) & ~CPM_VPU_STP);
	REG_CPM_VPU_SWRST = (REG_CPM_VPU_SWRST & ~CPM_VPU_SR & ~CPM_VPU_STP);
	REG_VPU_LOCK = 0;

	return 0;
}
#endif

/********************************************
  SW_RESET (VPU software reset)
*********************************************/
#define REG_CFGC_SW_RESET    0x00000
#define REG_CFGC_RST         (0x1<<30)
#define REG_CFGC_RST_CLR     (0x0<<30)
#define REG_CFGC_EARB_STAT   0x0000d
#define REG_CFGC_EARB_EMPT   (0x20000)

static int vpu_reset_x2000(struct ingenic_vdec_dev *dev)
{
	int timeout = 0xffff;
	vpu_writel(dev, REG_CFGC_SW_RESET, REG_CFGC_RST);
	while ((vpu_readl(dev, REG_CFGC_EARB_STAT) & REG_CFGC_EARB_EMPT) && --timeout);

	if (!timeout) {
		printk("vpu reset timeout!\n");
		return -EINVAL;
	}

	return 0;
}
static int vpu_reset(struct ingenic_vdec_dev *dev)
{
#ifdef CONFIG_SOC_M200
	return vpu_reset_m200(dev);
#else
	return vpu_reset_x2000(dev);
#endif
}

/*global clk on.*/
static int vpu_on(struct ingenic_vdec_dev *dev)
{
	clk_prepare_enable(dev->clk_gate);
	__asm__ __volatile__(
	    "mfc0  $2, $16,  7   \n\t"
	    "ori   $2, $2, 0x340 \n\t"
	    "andi  $2, $2, 0x3ff \n\t"
	    "mtc0  $2, $16,  7  \n\t"
	    "nop                  \n\t");

	return 0;
}

/* shutdown */
static int vpu_off(struct ingenic_vdec_dev *dev)
{
	clk_disable_unprepare(dev->clk_gate);

	return 0;
}


static int ingenic_vpu_start(void *priv)
{
	struct ingenic_vdec_ctx *ctx = (struct ingenic_vdec_ctx *)priv;
	struct ingenic_vdec_dev *dev = ctx->dev;
	H264Context *h = ctx->h;
	vpu_ctx_t *vpu_ctx = &h->vpu_ctx;

	unsigned int sch_glbc = 0;
	unsigned int des_pa;
	unsigned long flags;
	int ret = 0;


	mutex_lock(&ctx->lock);

	des_pa = vpu_ctx->desc_pa;

	spin_lock_irqsave(&dev->spinlock, flags);

	/* 是不是将vpu_start 和 wait放在一起，更加合理一些
	 * 针对ctx 做加锁处理, 保证curr_ctx在一次处理期间不会发生变化.
	 * */
	dev->curr_ctx = ctx;
	ctx->int_cond = 0;


	/* vpu reset ... */
	vpu_reset(dev);

	/*TODO: 如果使用tlb，此处要修改.*/
	sch_glbc = SCH_GLBC_HIAXI | SCH_INTE_ACFGERR | SCH_INTE_BSERR | SCH_INTE_ENDF;
	/*|SCH_INTE_ENDF | SCH_INTE_TLBERR | SCH_INTE_BSFULL;*/

	vpu_writel(dev, REG_SCH_GLBC, sch_glbc);

	/*trigger start.*/
	vpu_writel(dev, REG_VDMA_TASKRG, VDMA_ACFG_DHA(des_pa) | VDMA_ACFG_RUN);

	/* wait event time out ... */
	spin_unlock_irqrestore(&dev->spinlock, flags);

	ret = wait_event_interruptible_timeout(ctx->queue, ctx->int_cond, msecs_to_jiffies(VPU_RUN_TIMEOUT_MS));
	if (!ret) {
		pr_err("wait vpu run timeout!\n");
		printk("efe_stat %x, sch_stat %x\n", vpu_readl(dev, REG_EFE_STAT), vpu_readl(dev, REG_SCH_STAT));

		vpu_dump_regs(dev);
		ret = -ETIMEDOUT;
	} else if (ret == -ERESTARTSYS) {
		pr_err("vpu interrupted by a signal!\n");

	}

	mutex_unlock(&ctx->lock);
	return ret;
}

static int ingenic_vpu_wait(void *priv)
{
	return 0;
}

static int ingenic_vpu_stop(void *priv)
{
	return 0;
}

struct vpu_ops ingenic_vpu_ops = {
	.start = ingenic_vpu_start,
	.wait = ingenic_vpu_wait,
	.end = ingenic_vpu_stop,
};

static int ingenic_vcodec_probe(struct platform_device *pdev)
{
	struct ingenic_vdec_dev *dev;
	struct video_device *vfd;
	struct device_node *np;
	int dts_v;

	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	dev->dev = &pdev->dev;
	dev->plat_dev = pdev;

	/*io,clk,irq*/

	dev->reg_base = of_iomap(pdev->dev.of_node, 0);;
	if (IS_ERR(dev->reg_base)) {
		ret = -ENODEV;
		goto err_ioremap;
	}

	/*TODO: clk pm ...*/
	dev->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, dev->irq, ingenic_vpu_irq, 0, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request vpu irq!\n");
		goto err_irq;
	}
#if ((defined CONFIG_SOC_X2600) || (defined CONFIG_SOC_AD100))
	dev->clk_gate = clk_get(&pdev->dev, "gate_felix");
#else
	dev->clk_gate = clk_get(&pdev->dev, "power_felix");
#endif
	if (IS_ERR(dev->clk_gate)) {
		ret = PTR_ERR(dev->clk_gate);
		goto err_get_clk_gate;
	}

	spin_lock_init(&dev->spinlock);
	mutex_init(&dev->dev_mutex);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s", "ingenic-v4l2-felix");

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2_dev\n");
		goto err_v4l2;
	}

	vfd = video_device_alloc();
	if (!vfd) {
		dev_err(&pdev->dev, "Failed to alloc video device!\n");
		ret = -ENOMEM;
		goto err_vdev;
	}


	vfd->fops   = &ingenic_vdec_fops;
	vfd->ioctl_ops  = &ingenic_vdec_ioctl_ops;
	vfd->release    = video_device_release;
	vfd->lock   = &dev->dev_mutex;
	vfd->v4l2_dev   = &dev->v4l2_dev;
	vfd->vfl_dir    = VFL_DIR_M2M;
	vfd->device_caps    = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	snprintf(vfd->name, sizeof(vfd->name), "%s", INGENIC_VCODEC_DEC_NAME);

	video_set_drvdata(vfd, dev);
	dev->vfd = vfd;
	platform_set_drvdata(pdev, dev);

	dev->m2m_dev = v4l2_m2m_init(&ingenic_vdec_m2m_ops);
	if (IS_ERR((__force void *)dev->m2m_dev)) {
		dev_err(&pdev->dev, "Failed to init m2m device!\n");
		ret = PTR_ERR((__force void *)dev->m2m_dev);
		goto err_m2m;
	}

	dev->dec_workqueue = alloc_ordered_workqueue(INGENIC_VCODEC_DEC_NAME,
	                     WQ_MEM_RECLAIM | WQ_FREEZABLE);
	if (!dev->dec_workqueue) {
		dev_err(&pdev->dev, "Failed to create decode workqueue\n");
		ret = -EINVAL;
		goto err_workq;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, INGENIC_FELIX_VIDEO_NR);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device!\n");
		goto err_video_reg;
	}

	ret = of_reserved_mem_device_init(dev->dev);
	if (ret) {
		dev_warn(&pdev->dev, "failed to init reserved mem\n");
	}

	np = dev->dev->of_node;
	if (of_property_read_u32(np, "ingenic,max_out_frame_buffers", &dts_v) == 0) {
		max_out_frame_buffers = dts_v;
	}

	if (of_property_read_u32(np, "ingenic,max_frame_buffers", &dts_v) == 0) {
		max_frame_buffers = dts_v;
	}

	dev_info(&pdev->dev, "h264decoder(felix) registered as /dev/video%d\n",
	         vfd->num);

	return 0;
err_video_reg:
err_workq:
err_m2m:
err_vdev:
err_v4l2:
err_get_clk_gate:
err_irq:
err_ioremap:
	return ret;
}

static int ingenic_vcodec_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id felix_of_match[] = {
	{ .compatible = "ingenic,x2000-felix"},
	{ .compatible = "ingenic,m300-felix"},
	{ .compatible = "ingenic,x2600-felix"},
	{ .compatible = "ingenic,ad100-felix"},
	{},
};

static struct platform_driver ingenic_vcodec_driver = {
	.probe = ingenic_vcodec_probe,
	.remove = ingenic_vcodec_remove,
	.driver = {
		.name = INGENIC_VCODEC_DEC_NAME,
		.of_match_table = felix_of_match,
	},
};

static int __init felix_device_init(void)
{
	platform_driver_register(&ingenic_vcodec_driver);
	return 0;
}

static void __exit felix_device_exit(void)
{
	platform_driver_unregister(&ingenic_vcodec_driver);
}

#ifdef CONFIG_FELIX_INIT_BY_USER
	user_triggered_module_init(felix_device_init);
#else
	module_init(felix_device_init);
#endif

module_exit(felix_device_exit);



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic Video Codec v4l2 encoder driver.");
