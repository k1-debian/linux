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
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/time.h>

#include "jpege_reg.h"
#include "jpege_drv.h"
#include "jpeg_dmabuf.h"
#if 0
static int _dump_jpege_reg(void)
{
	printk("---------------- dump reg start --------------------\r\n");
	printk("JPEGE_VERSION			0x%x  \n", JPEGE_READ_REG(JPEGE_VERSION));
	printk("JPEGE_CTL_STA			0x%x  \n", JPEGE_READ_REG(JPEGE_CTL_STA));
	printk("JPEGE_FRM_SIZE			0x%x  \n", JPEGE_READ_REG(JPEGE_FRM_SIZE));
	printk("JPEGE_CAP_CTL_STA		0x%x  \n", JPEGE_READ_REG(JPEGE_CAP_CTL_STA));
	printk("JPEGE_PIX_IN_STA		0x%x  \n", JPEGE_READ_REG(JPEGE_PIX_IN_STA));
	printk("JPEGE_PIX_IN_ADDR		0x%x  \n", JPEGE_READ_REG(JPEGE_PIX_IN_ADDR));
	printk("JPEGE_OUT_STA			0x%x  \n", JPEGE_READ_REG(JPEGE_OUT_STA));
	printk("JPEGE_OUT_ADDR			0x%x  \n", JPEGE_READ_REG(JPEGE_OUT_ADDR));
	printk("JPEGE_OUT_BUFFER_SIZE	0x%x  \n", JPEGE_READ_REG(JPEGE_OUT_BUFFER_SIZE));
	printk("JPEGE_IMGE_SIZE			0x%x  \n", JPEGE_READ_REG(JPEGE_IMGE_SIZE));
	printk("JPEGE_NEXT_OUT_ADDR		0x%x  \n", JPEGE_READ_REG(JPEGE_NEXT_OUT_ADDR));
	printk("JPEGE_STA_FLAG1			0x%x  \n", JPEGE_READ_REG(JPEGE_STA_FLAG1));
	printk("JPEGE_STA_FLAG2			0x%x  \n", JPEGE_READ_REG(JPEGE_STA_FLAG2));
	printk("---------------- dump reg end --------------------\r\n");
	return 0;
}
#endif

static int reg_bit_set(struct jz_jpeg_enc *jpege, unsigned int offset, unsigned int val)
{
	unsigned int tmp = JPEGE_READ_REG(offset);
	tmp |= val;
	JPEGE_WRITE_REG(offset, tmp);

	return 0;
}

static int reg_bit_clr(struct jz_jpeg_enc *jpege, unsigned int offset, unsigned int val)
{
	unsigned int tmp = JPEGE_READ_REG(offset);
	tmp &= ~(val);
	JPEGE_WRITE_REG(offset, tmp);

	return 0;
}

static int jpege_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	return ret;
}

static int jpege_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	return ret;
}

static void __jpege_soft_reset(void)
{
	unsigned int tmp = readl((unsigned int *)0xb00000C4);
	tmp |= (1 << 26);
	writel(tmp, (unsigned int *)0xb00000C4);
	tmp &= ~(1 << 26);
	writel(tmp, (unsigned int *)0xb00000C4);
	printk("%s end ======\r\n", __func__);
}

static irqreturn_t jpege_irq_handler(int irq, void *data)
{
	struct  jz_jpeg_enc *jpege = (struct jz_jpeg_enc *)data;
	unsigned int tmp = 0;
	tmp = JPEGE_READ_REG(JPEGE_CTL_STA);
	//printk("%s %d tmp = 0x%x ##########\r\n",__func__,__LINE__,tmp);
	if (tmp & ERR_FLAG) {
		printk("some error happend !!!!!\r\n");
		_jpege_clr_int();
		disable_irq_nosync(jpege->irq);
		__jpege_soft_reset();
		return IRQ_HANDLED;
	}
	if (tmp & DATAENC_IDLE) {
		complete(&jpege->done_encoder);
	}
	_jpege_clr_int();
	disable_irq_nosync(jpege->irq);
	return IRQ_HANDLED;
}

static int _jpeg_enc_start(struct jz_jpeg_enc *jpege, struct jpege_info *info)
{
	// set src paddr
	int ret = 0;

	unsigned int src_addr = info->in.src_paddr;
	unsigned int dst_addr = info->in.dst_paddr;
	unsigned short width = info->in.frame_width;
	unsigned short height = info->in.frame_height;
	unsigned int dst_bufsize = info->in.dst_bufsize;
	char quality = info->in.qa;
	int srcfmt = info->in.pix_fmt;
	unsigned int tmp = 0;
	char subsamp = info->in.subsamp;

	tmp = JPEGE_READ_REG(JPEGE_CTL_STA);
	tmp &= ~(1 << 0);   // clear status
	if (info->in.rstm_request) {
		tmp &= ~(0xffff << 16);
		tmp |= (info->in.rstm_request << 16);
	} else {
		tmp &= ~(0xffff << 16);
	}
	JPEGE_WRITE_REG(JPEGE_CTL_STA, tmp);

	//_dump_jpege_reg();

	// 设置原图像宽高
	if (width && height) {
		tmp = ((height - 1) << 16) | (width - 1);
		JPEGE_WRITE_REG(JPEGE_FRM_SIZE, tmp);
	} else {
		return -1;
	}

	switch (subsamp) {
		case JPEGE_SUBSAMP_TYPE_NONE:
			_set_input_subsamp(0); break;
		case JPEGE_SUBSAMP_TYPE_422H:
			_set_input_subsamp(1); break;
		case JPEGE_SUBSAMP_TYPE_420:
			_set_input_subsamp(3); break;
		default:
			_set_input_subsamp(0); break;
	}

#if 1
	switch (srcfmt) {
		case IMPP_PIX_FMT_YUV444:
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, 0);
			_input_is_yuv(); break;
		case IMPP_PIX_FMT_YUYV:
			tmp = (0x01 << 8);
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, tmp);
			_input_is_yuv(); break;
		case IMPP_PIX_FMT_NV12:
			tmp = (0x2 << 8) | (0x4 << 4);
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, tmp);
			tmp = width * height;
			JPEGE_WRITE_REG(JPEGE_UV_OFFSET, tmp);
			_input_is_yuv(); break;
		case IMPP_PIX_FMT_NV21:
			tmp = (0x2 << 8) | (0x0 << 4);
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, tmp);
			tmp = width * height;
			JPEGE_WRITE_REG(JPEGE_UV_OFFSET, tmp);
			_input_is_yuv(); break;
		case IMPP_PIX_FMT_RGB_888:
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, 0);
			_input_is_rgb(); break;
		case IMPP_PIX_FMT_RGBA_8888:
			tmp = (0x3 << 8) | (0x0 << 4);
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, tmp);
			_input_is_rgb(); break;
		case IMPP_PIX_FMT_BGRA_8888:
			tmp = (0x3 << 8) | (0x6 << 4);
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, tmp);
			_input_is_rgb(); break;
		case IMPP_PIX_FMT_GREY:
			JPEGE_WRITE_REG(JPEGE_CAP_CTL_STA, 0);
			_input_is_grayscale();
			break;
		default:
			printk("src fmt is error \r\n");
			return -1;
	}
#endif
	// 设置read addr
	if (src_addr) {
		src_addr = src_addr >> 12;  // the addr must 4K align
		tmp = src_addr | (1 << 31);
		JPEGE_WRITE_REG(JPEGE_PIX_IN_ADDR, tmp);
	} else {
		printk("src addr is invalid \r\n");
		return -1;
	}
	// 设置 write addr
	if (dst_addr) {
		dst_addr = dst_addr >> 12;  // the addr must 4K align
		tmp = dst_addr | (1 << 31);
		JPEGE_WRITE_REG(JPEGE_OUT_ADDR, tmp);
	} else {
		printk("dst addr is invalid \r\n");
		return -1;
	}

	// 设置 输出buffer大小
	if (dst_bufsize) {
		JPEGE_WRITE_REG(JPEGE_OUT_BUFFER_SIZE, dst_bufsize);
	}

	// 设置编码质量参数 其他参数默
	if (quality <= 100) {
		_set_quality(quality);
	} else {
		printk("quality value is invalid \r\n");
		return -1;
	}

	// 启动编码
	_start_jpege();
	enable_irq(jpege->irq); //TODO

	ret = wait_for_completion_interruptible_timeout(&jpege->done_encoder, msecs_to_jiffies(3000));
	if (ret < 0) {
		printk("wait completion error !!!!\r\n");
		return -EFAULT;
	} else if (ret == 0) {
		printk("wait completion time out !!!!!\r\n");
	} else {
		tmp = JPEGE_READ_REG(JPEGE_IMGE_SIZE);
		if (tmp) {
			//printk("enc out image size = %d \r\n",tmp);
			info->out.imagesize = tmp & 0x3FFFFFFF;
			ret = 0;
		} else {
			ret = -1;
		}
	}
	return ret;
}

static long jpege_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_jpeg_enc *jpege = container_of(dev, struct jz_jpeg_enc, misc_dev);
	int ret = 0;
	struct jpege_info info;
	mutex_lock(&jpege->mutex);
	switch (cmd) {
		case JZ_JPEGE_START:
			if (copy_from_user(&info, (struct jpege_info *)arg, sizeof(struct jpege_info))) {
				return -EFAULT;
			}
			ret = _jpeg_enc_start(jpege, &info);
			if (copy_to_user((void *)arg, &info, sizeof(struct jpege_info))) {
				ret = -EFAULT;
			}
			break;
		case JZ_JPEGE_ALLOC_DMABUF:
			ret = jpeg_ioctl_get_dma_fd(jpege->dev, arg);
			break;
		case JZ_JPEGE_GET_DMABUF_PADDR:
			ret = jpeg_ioctl_get_dmabuf_dma_addr(jpege->dev, arg);
			break;
		case JZ_JPEGE_FREE_BUFFER:
			jpeg_ioctl_free_dmabuf(jpege->dev, arg);
			break;
		default :
			ret = -EFAULT;
	}
	mutex_unlock(&jpege->mutex);
	return ret;
}

static struct file_operations jpege_ops = {
	.owner = THIS_MODULE,
	.open = jpege_open,
	.release = jpege_release,
	.unlocked_ioctl = jpege_ioctl,
};

static int jpege_probe(struct platform_device *pdev)
{
	struct jz_jpeg_enc *jpege;
	int ret = 0;
	jpege = (struct jz_jpeg_enc *)kzalloc(sizeof(struct jz_jpeg_enc), GFP_KERNEL);
	if (!jpege) {
		dev_err(&pdev->dev, "alloc jz_jpeg_enc failed");
		return -ENOMEM;
	}

	sprintf(jpege->name, "jpegenc");

	jpege->misc_dev.minor = MISC_DYNAMIC_MINOR;
	jpege->misc_dev.name = jpege->name;
	jpege->misc_dev.fops = &jpege_ops;
	jpege->dev = &pdev->dev;

	jpege->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!jpege->res) {
		dev_err(&pdev->dev, "failed to get jpege resource \n");
		ret = -EINVAL;
		goto err_get_platform_res;
	}

	jpege->iomem = ioremap(jpege->res->start, resource_size(jpege->res));
	if (!jpege->iomem) {
		dev_err(&pdev->dev, "failed to remap mem region\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	jpege->clk = clk_get(&pdev->dev, "gate_jpege");
	if (IS_ERR(jpege->clk)) {
		dev_err(&pdev->dev, "failed to get jpege clk gate\n");
		ret = -EINVAL;
		goto err_get_clk;
	}

	jpege->irq = platform_get_irq(pdev, 0);
	printk("jpege irq = %d \r\n", jpege->irq);
	if (request_irq(jpege->irq, jpege_irq_handler, IRQF_TRIGGER_HIGH
	                , jpege->name, jpege)) {
		dev_err(&pdev->dev, "request jpeg decoder irq failed\n");
		ret = -EINVAL;
		goto err_request_irq;
	}
	disable_irq_nosync(jpege->irq);
	dev_set_drvdata(&pdev->dev, jpege);

	ret = misc_register(&jpege->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register misc device failed\n");
		goto err_misc_register;
	}
	mutex_init(&jpege->mutex);
	init_completion(&jpege->done_encoder);
	init_completion(&jpege->done_buf);
	clk_prepare_enable(jpege->clk);

	ret = of_reserved_mem_device_init(jpege->dev);
	if (ret) {
		printk("failed to init reserved mem !!!!!!!!!!!!!!!!!!!!\n");
	}

	//_jpege_reset();
	printk("JZ JPEG ENCODER TEST DRIVER Probe end !!!!!!!!!!!!!!\r\n");

	return 0;
err_misc_register:
	free_irq(jpege->irq, jpege);
err_request_irq:
	iounmap(jpege->iomem);
err_get_clk:
err_ioremap:
err_get_platform_res:
	kfree(jpege);
	return ret;

}

static int jpege_remove(struct platform_device *pdev)
{
	struct jz_jpeg_enc *jpege;
	struct resource *res;
	jpege = dev_get_drvdata(&pdev->dev);
	misc_deregister(&jpege->misc_dev);
	clk_disable_unprepare(jpege->clk);
	res = jpege->res;
	free_irq(jpege->irq, jpege);
	iounmap(jpege->iomem);
	release_mem_region(res->start, res->end - res->start + 1);
	if (jpege) {
		kfree(jpege);
	}
	return 0;
}

static const struct of_device_id ingenic_jpege_dt_match[] = {
	{ .compatible = "ingenic,x2600-jpege", .data = NULL },
	{ .compatible = "ingenic,ad100-jpege", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_jpege_dt_match);

static struct platform_driver jz_jpege_driver = {
	.probe  = jpege_probe,
	.remove = jpege_remove,
	.driver = {
		.name = "jz-jpege",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_jpege_dt_match),
	},
};

static int __init jpegedev_init(void)
{
	platform_driver_register(&jz_jpege_driver);
	return 0;
}

static void __exit jpegedev_exit(void)
{
	platform_driver_unregister(&jz_jpege_driver);
}

module_init(jpegedev_init);
module_exit(jpegedev_exit);

MODULE_LICENSE("GPL");
