/*
* Copyright© 2014 Ingenic Semiconductor Co.,Ltd
*
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
#include <media/videobuf2-dma-contig-ingenic.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <media/ingenic_video_nr.h>

#include "helix_drv.h"
#include "helix_ops.h"

static int vpu_on(struct ingenic_venc_dev *dev);
static int vpu_off(struct ingenic_venc_dev *dev);

struct ingenic_venc_dev *g_dev = NULL;

static int fops_vcodec_open(struct file *file)
{
	struct ingenic_venc_dev *dev = video_drvdata(file);
	struct ingenic_venc_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		return -ENOMEM;
	}

	mutex_lock(&dev->dev_mutex);

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

	ingenic_vcodec_enc_init_default_params(ctx);

	ret = ingenic_vcodec_enc_ctrls_setup(ctx);
	if (ret) {
		dev_err(ctx->dev->dev, "Failed to setup ctrls() %d\n", ret);
		goto err_ctrls_setup;
	}

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev_enc, ctx, &ingenic_vcodec_enc_queue_init);
	if (IS_ERR((__force void *)ctx->m2m_ctx)) {
		ret = PTR_ERR((__force void *)ctx->m2m_ctx);

		goto err_m2m_ctx_init;
	}

	mutex_unlock(&dev->dev_mutex);

	dev_info(ctx->dev->dev, "Create instance [%d]@%p m2m_ctx=%p\n",
	         ctx->id, ctx, ctx->m2m_ctx);

	dev_info(ctx->dev->dev, "%s vcodec [%d]\n", dev_name(dev->dev), ctx->id);

	return ret;
err_m2m_ctx_init:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_ctrls_setup:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int fops_vcodec_release(struct file *file)
{

	struct ingenic_venc_dev *dev = video_drvdata(file);
	struct ingenic_venc_ctx *ctx = fh_to_ctx(file->private_data);

	dev_info(ctx->dev->dev, "[%d] encoder release\n", ctx->id);

	mutex_lock(&dev->dev_mutex);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	ingenic_vcodec_enc_deinit_default_params(ctx);

	if (!(--dev->id_counter)) {
		vpu_off(dev);
	}

	kfree(ctx);

	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int fops_vcodec_poll(struct file *file,
                                     struct poll_table_struct *wait)
{
	struct ingenic_venc_dev *dev = video_drvdata(file);
	struct ingenic_venc_ctx *ctx = fh_to_ctx(file->private_data);
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
	struct ingenic_venc_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations ingenic_venc_fops = {
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

static void inline vpu_clear_bits(struct ingenic_venc_dev *dev, unsigned int offset, unsigned int bits)
{
	unsigned int val = vpu_readl(dev, offset);
	val &= ~bits;
	vpu_writel(dev, offset, val);
}

static irqreturn_t ingenic_vpu_irq(int irq, void *priv)
{
	struct ingenic_venc_dev *dev = (struct ingenic_venc_dev *)priv;
	struct ingenic_venc_ctx *ctx = dev->curr_ctx;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;

	unsigned long flags;

	unsigned int efe_stat;
	unsigned int sch_stat;

	spin_lock_irqsave(&dev->spinlock, flags);

	ctx->int_cond = 1;

	efe_stat = vpu_readl(dev, REG_EFE_STAT);
	sch_stat = vpu_readl(dev, REG_SCH_STAT);

	/* disable all interrupts. */
	vpu_clear_bits(dev, REG_SCH_GLBC, 0x3f << 16);
	if (sch_stat & SCH_STAT_ENDF) {

		if (sch_stat & SCH_STAT_JPGEND) {

			jpge_ctx->bslen = vpu_readl(dev, REG_JPGC_STAT) & 0xffffff;

			vpu_clear_bits(dev, REG_JPGC_STAT, JPGC_STAT_ENDF);
		} else if (sch_stat & SCH_STAT_ENDF) {
			h264e_ctx->r_bs_len = vpu_readl(dev, REG_SDE_CFG9);
			h264e_ctx->r_rbsp_len = vpu_readl(dev, REG_SDE_CFG10);
			h264e_ctx->encoded_bs_len = h264e_ctx->r_bs_len;

			vpu_clear_bits(dev, REG_SDE_STAT, SDE_STAT_BSEND);
			vpu_clear_bits(dev, REG_DBLK_GSTA, DBLK_STAT_DOEND);
			/*wakeup ...*/
		} else if (sch_stat & SCH_STAT_TIMEOUT) {
			dev_err(ctx->dev->dev, "Sch h264 Timeout !\n");
		} else {
			/*Error handling ...!*/

			dev_err(ctx->dev->dev, "stat error %x\n", sch_stat);
			/*wakeup ...*/
		}

	}
	ctx->int_status = sch_stat;
	spin_unlock_irqrestore(&dev->spinlock, flags);

	wake_up_interruptible(&ctx->queue);

	return IRQ_HANDLED;
}

static void vpu_dump_regs(struct ingenic_venc_dev *dev)
{

	/* EMC */
	dev_info(dev->dev, "REG_EMC_FRM_SIZE :	%d\n", vpu_readl(dev, REG_EMC_FRM_SIZE));
	dev_info(dev->dev, "REG_EMC_BS_ADDR :	0x%x\n", vpu_readl(dev, REG_EMC_BS_ADDR));
	dev_info(dev->dev, "REG_EMC_DBLK_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_DBLK_ADDR));
	dev_info(dev->dev, "REG_EMC_RECON_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_RECON_ADDR));
	dev_info(dev->dev, "REG_EMC_MV_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_MV_ADDR));
	dev_info(dev->dev, "REG_EMC_SE_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_SE_ADDR));
	dev_info(dev->dev, "REG_EMC_QPT_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_QPT_ADDR));
	dev_info(dev->dev, "REG_EMC_RC_RADDR:	0x%x\n", vpu_readl(dev, REG_EMC_RC_RADDR));
	dev_info(dev->dev, "REG_EMC_MOS_ADDR:	0x%x\n", vpu_readl(dev, REG_EMC_MOS_ADDR));
	dev_info(dev->dev, "REG_EMC_SLV_INIT:	0x%x\n", vpu_readl(dev, REG_EMC_SLV_INIT));
	dev_info(dev->dev, "REG_EMC_BS_SIZE:	0x%x\n", vpu_readl(dev, REG_EMC_BS_SIZE));
	dev_info(dev->dev, "REG_EMC_BS_STAT:	0x%x\n", vpu_readl(dev, REG_EMC_BS_STAT));
}

#define REG_VPU_STATUS ( *(volatile unsigned int*)0xb3200034 )
#define REG_VPU_LOCK ( *(volatile unsigned int*)0xb329004c )
#define REG_VPUCDR ( *(volatile unsigned int*)0xb0000030 )
#define REG_CPM_VPU_SWRST ( *(volatile unsigned int*)0xb00000c4 )
#define REG_VPU_TLBBASE (*(volatile unsigned int *)(0x30 + 0xb3200000))
#define CPM_VPU_SR           (0x1<<31)
#define CPM_VPU_STP          (0x1<<30)
#define CPM_VPU_ACK          (0x1<<29)

#if 0
static int vpu_reset(struct ingenic_venc_dev *dev)
{
	int timeout = 0xffffff;

	REG_CPM_VPU_SWRST |= CPM_VPU_STP;
	while (!(REG_CPM_VPU_SWRST & CPM_VPU_ACK) && --timeout)
		;

	if (!timeout) {
		printk("wait reset timeout!\n");
	}

	REG_CPM_VPU_SWRST = ((REG_CPM_VPU_SWRST | CPM_VPU_SR) & ~CPM_VPU_STP);
	REG_CPM_VPU_SWRST = (REG_CPM_VPU_SWRST & ~CPM_VPU_SR & ~CPM_VPU_STP);
	REG_VPU_LOCK = 0;

	return 0;
}
#endif
/*global clk on.*/
static int vpu_on(struct ingenic_venc_dev *dev)
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

static int vpu_off(struct ingenic_venc_dev *dev)
{
	clk_disable_unprepare(dev->clk_gate);

	return 0;
}

#define VPU_RUN_TIMEOUT_MS  (3000)
int ingenic_vpu_start(void *priv)
{
	struct ingenic_venc_ctx *ctx = (struct ingenic_venc_ctx *)priv;
	struct ingenic_venc_dev *dev = ctx->dev;
	struct h264e_ctx *h264e_ctx = &ctx->h264e_ctx;
	struct jpge_ctx *jpge_ctx = &ctx->jpge_ctx;
	struct jpgd_ctx *jpgd_ctx = &ctx->jpgd_ctx;
	unsigned int sch_glbc = 0;
	unsigned int des_pa;
	unsigned long flags;
	int timeout = 0xffff;
	int ret = 0;

	/* TODO: add lock.*/
	dev->curr_ctx = ctx;

	if (ctx->codec_id == CODEC_ID_H264E) {
		des_pa = h264e_ctx->desc_pa;
	} else if (ctx->codec_id == CODEC_ID_JPGE) {
		des_pa = jpge_ctx->desc_pa;
	} else if (ctx->codec_id == CODEC_ID_JPGD) {
		des_pa = jpgd_ctx->desc_pa;
	}

	vpu_on(dev);

	spin_lock_irqsave(&dev->spinlock, flags);
	ctx->int_cond = 0;

	/*X2000 RESET*/
	/* vpu reset ... */
	vpu_writel(dev, REG_CFGC_SW_RESET, CFGC_SW_RESET_RST);
	while (!(vpu_readl(dev, REG_CFGC_SW_RESET) & CFGC_SW_RESET_EARB_EMPT) && --timeout);
	if (!timeout) {
		dev_err(ctx->dev->dev, "%s, vpu_reset timeout!\n", __func__);
		spin_unlock_irqrestore(&dev->spinlock, flags);
		return -EINVAL;
	}

	sch_glbc = SCH_GLBC_HIAXI | SCH_INTE_ACFGERR | SCH_INTE_BSERR |
	           SCH_INTE_ENDF | SCH_INTE_TLBERR | SCH_INTE_BSFULL;

	/* type jpege, jpegd, h264e.*/
	vpu_writel(dev, REG_SCH_GLBC, sch_glbc);

	/*trigger start.*/
	vpu_writel(dev, REG_CFGC_ACM_CTRL, VDMA_ACFG_DHA(des_pa) | VDMA_ACFG_RUN);

	/* wait event time out ... */
	spin_unlock_irqrestore(&dev->spinlock, flags);

	ret = wait_event_interruptible_timeout(ctx->queue, ctx->int_cond, msecs_to_jiffies(VPU_RUN_TIMEOUT_MS));
	if (!ret) {
		dev_err(ctx->dev->dev, "wait vpu run timeout!\n");
		dev_err(ctx->dev->dev, "efe_stat %x, sch_stat %x\n", vpu_readl(dev, REG_EFE_STAT), vpu_readl(dev, REG_SCH_STAT));

		vpu_dump_regs(dev);
		ret = -ETIMEDOUT;
	} else if (ret == -ERESTARTSYS) {
		dev_err(ctx->dev->dev, "vpu interrupted by a signal!\n");

	}

	vpu_off(dev);

	return ret;
}

int ingenic_vpu_stop(struct ingenic_venc_dev *dev)
{
	return 0;
}

static int ingeic_vcodec_probe(struct platform_device *pdev)
{
	struct ingenic_venc_dev *dev;
	struct video_device *vfd_enc;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	ingenic_venc_dev_set(dev);

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

	/*clk_gate, power*/
	dev->clk_gate = clk_get(&pdev->dev, "power_helix");
	if (IS_ERR(dev->clk_gate)) {
		ret = PTR_ERR(dev->clk_gate);
		goto err_get_clk_gate;
	}

#if 0
	dev->clk = clk_get(dev->dev, "cgu_vpu");
	if (IS_ERR(dev->clk)) {
		ret = PTR_ERR(dev->clk);
		goto err_get_clk_cgu;
	}

	clk_set_rate(dev->clk, 300000000);
#endif

	spin_lock_init(&dev->spinlock);
	mutex_init(&dev->dev_mutex);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s", "ingenic-v4l2-helix");

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2_dev\n");
		goto err_v4l2;
	}

	vfd_enc = video_device_alloc();
	if (!vfd_enc) {
		dev_err(&pdev->dev, "Failed to alloc video device!\n");
		ret = -ENOMEM;
		goto err_vdev;
	}

	vfd_enc->fops       = &ingenic_venc_fops;
	vfd_enc->ioctl_ops  = &ingenic_venc_ioctl_ops;
	vfd_enc->release    = video_device_release;
	vfd_enc->lock       = &dev->dev_mutex;
	vfd_enc->v4l2_dev   = &dev->v4l2_dev;
	vfd_enc->vfl_dir    = VFL_DIR_M2M;

	snprintf(vfd_enc->name, sizeof(vfd_enc->name), "%s", INGENIC_VCODEC_ENC_NAME);

	video_set_drvdata(vfd_enc, dev);
	dev->vfd_enc = vfd_enc;
	platform_set_drvdata(pdev, dev);

	dev->m2m_dev_enc = v4l2_m2m_init(&ingenic_venc_m2m_ops);
	if (IS_ERR((__force void *)dev->m2m_dev_enc)) {
		dev_err(&pdev->dev, "Failed to init m2m device!\n");
		ret = PTR_ERR((__force void *)dev->m2m_dev_enc);
		goto err_m2m;
	}

	dev->encode_workqueue = alloc_ordered_workqueue(INGENIC_VCODEC_ENC_NAME,
	                        WQ_MEM_RECLAIM | WQ_FREEZABLE);
	if (!dev->encode_workqueue) {
		dev_err(&pdev->dev, "Failed to create encode workqueue\n");
		ret = -EINVAL;
		goto err_workq;
	}

	vfd_enc->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	ret = video_register_device(vfd_enc, VFL_TYPE_VIDEO, INGENIC_HELIX_VIDEO_NR);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device!\n");
		goto err_video_reg;
	}

	ret = of_reserved_mem_device_init(dev->dev);
	if (ret) {
		dev_warn(&pdev->dev, "failed to init reserved mem\n");
	}

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_NETLINK
	h264e_netlink_test();
#endif

	dev_info(&pdev->dev, "encoder(helix) registered as /dev/video%d\n",
	         vfd_enc->num);

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

static int ingeic_vcodec_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id helix_of_match[] = {
	{ .compatible = "ingenic,x2000-helix"},
	{ .compatible = "ingenic,m300-helix"},
	{},
};

static struct platform_driver ingenic_vcodec_driver = {
	.probe = ingeic_vcodec_probe,
	.remove = ingeic_vcodec_remove,
	.driver = {
		.name = INGENIC_VCODEC_ENC_NAME,
		.of_match_table = helix_of_match,
	},
};

static int __init helix_driver_init(void)
{
	return platform_driver_register(&ingenic_vcodec_driver);
}

static void __exit helix_driver_exit(void)
{
	platform_driver_unregister(&ingenic_vcodec_driver);
}
module_init(helix_driver_init);
module_exit(helix_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic Video Codec v4l2 encoder driver.");
