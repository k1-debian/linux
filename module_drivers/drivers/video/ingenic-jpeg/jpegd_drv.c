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

#include "jpegd_drv.h"
#include "jpegd_reg.h"
#include "jpeg_dmabuf.h"
#if 0
static int _dump_jpegd_reg(void)
{
	printk("JPEGD_CTL_STA(0x00H)	0x%x  \n", JPEGD_READ_REG(JPEGD_CTL_STA));
	printk("JPEGD_ROW_COL(0x04H)	0x%x  \n", JPEGD_READ_REG(JPEGD_ROW_COL));
	printk("JPEGD_RD_ADDR(0x08H)	0x%x  \n", JPEGD_READ_REG(JPEGD_RD_ADDR));
	printk("JPEGD_WR_ADDR(0x0CH)	0x%x  \n", JPEGD_READ_REG(JPEGD_WR_ADDR));
	printk("JPEGD_FILE_SIZE(0x10H)	0x%x  \n", JPEGD_READ_REG(JPEGD_FILE_SIZE));
	printk("JPEGD_FRM_CNT(0x14H)	0x%x  \n", JPEGD_READ_REG(JPEGD_FRM_CNT));
	printk("JPEGD_WORD_CNT(0x18H)	0x%x  \n", JPEGD_READ_REG(JPEGD_WORD_CNT));
	printk("JPEGD_ERR_CNT(0x1CH)	0x%x  \n", JPEGD_READ_REG(JPEGD_ERR_CNT));
	printk("JPEGD_PIX_DEC_CNT(0x20H)	0x%x  \n", JPEGD_READ_REG(JPEGD_PIX_DEC_CNT));
	printk("JPEGD_PIX_OUT_CNT(0x24H)	0x%x  \n", JPEGD_READ_REG(JPEGD_PIX_OUT_CNT));
	printk("JPEGD_RESVER_STA(0x30H)	0x%x  \n", JPEGD_READ_REG(JPEGD_RESVER_STA));
	printk("JPEGD_FB_ROW_COL(0x34H)	0x%x  \n", JPEGD_READ_REG(JPEGD_FB_ROW_COL));
	printk("JPEGD_WDT_PERIOD(0x78H)	0x%x  \n", JPEGD_READ_REG(JPEGD_WDT_PERIOD));
	printk("JPEGD_ERR_HW_FLG(0x7CH)	0x%x  \n", JPEGD_READ_REG(JPEGD_ERR_HW_FLG));
	printk("READ REG-70	0x%x  \n", JPEGD_READ_REG(0x70));
	printk("READ REG-74	0x%x  \n", JPEGD_READ_REG(0x74));
	return 0;
}
#endif

static int reg_bit_set(struct jz_jpeg_dec *jpegd, unsigned int offset, unsigned int val)
{
	unsigned int tmp = JPEGD_READ_REG(offset);
	tmp |= val;
	JPEGD_WRITE_REG(offset, tmp);

	return 0;
}

static int reg_bit_clr(struct jz_jpeg_dec *jpegd, unsigned int offset, unsigned int val)
{
	unsigned int tmp = JPEGD_READ_REG(offset);
	tmp &= ~(val);
	JPEGD_WRITE_REG(offset, tmp);

	return 0;
}

static int jpegd_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	return ret;
}

static int jpegd_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	return ret;
}

__attribute__((__unused__)) static void __jpegd_soft_reset(void)
{
	unsigned int tmp = readl((void __iomem *)0xb00000C4);
	tmp |= (1 << 27);
	writel(tmp, (void __iomem *)0xb00000C4);
	tmp &= ~(1 << 27);
	writel(tmp, (void __iomem *)0xb00000C4);
	printk("%s end ============\r\n", __func__);
}

static void _jpegd_set_output_fmt(int fmt)
{
	unsigned int tmp = JPEGD_READ_REG(JPEGD_CTL_STA);
	tmp &= ~(7 << 12);
	tmp |= (fmt << 12);
	JPEGD_WRITE_REG(JPEGD_CTL_STA, tmp);
}

static int _jpeg_get_reserved_statu(struct jz_jpeg_dec *jpegd, int *other_subsample)
{
	unsigned int tmp = JPEGD_READ_REG(JPEGD_RESVER_STA);
	*other_subsample = tmp & 0xffff;

	return 0;
}

static irqreturn_t jpegd_irq_handler(int irq, void *data)
{
	struct  jz_jpeg_dec *jpegd = (struct jz_jpeg_dec *)data;
	unsigned int tmp = JPEGD_READ_REG(JPEGD_CTL_STA);
	_jpegd_clear_int();
	if ((tmp & (1 << 7))) {
		complete(&jpegd->done_decoder);
	} else if (tmp & ERR_DETECT) {
		// err happend
		//printk("[jpeg decoder] : some error happend ######\r\n");
		// we can write 0 to reg0 bit1 to clear err , this take care of soft-reset
		// soft reset : write 1 to bit2
		_jpegd_clear_err();
	}
	_jpegd_finish_irq_disable();
	_jpegd_err_irq_disable();
	//__jpegd_soft_reset();
	//disable_irq_nosync(jpegd->irq);
	return IRQ_HANDLED;
}

static int _jpeg_dec_start(struct jz_jpeg_dec *jpegd, struct jpegd_info *param_info)
{
	// set src paddr
	int ret = 0;
	unsigned int src_paddr = param_info->in_param.src_paddr;
	unsigned int dst_paddr = param_info->in_param.dst_paddr;
	unsigned int filesize = param_info->in_param.file_size;
	unsigned int tmp;
	struct jpegd_out_info *out = &param_info->out_info;

	_jpegd_soft_reset();
	if (filesize) {
		JPEGD_WRITE_REG(JPEGD_FILE_SIZE, filesize);
	} else {
		printk("JPEGD : file size is err \r\n");
		return -EFAULT;
	}

	if (src_paddr) {
		JPEGD_WRITE_REG(JPEGD_RD_ADDR, src_paddr);
	} else {
		printk("JPEGD : src addr error\r\n");
		return -EFAULT;
	}

	if (dst_paddr) {
		JPEGD_WRITE_REG(JPEGD_WR_ADDR, dst_paddr);
	} else {
		printk("JPEGD : dst addr error\r\n");
		return -EFAULT;
	}
	_jpegd_out_normal_order();
	switch (param_info->in_param.out_fmt) {
		case IMPP_PIX_FMT_YUV444:
			_jpegd_set_output_fmt(JPEGD_OUT_DEFAULT);
			_jpegd_out_yuv(); break;
		case IMPP_PIX_FMT_RGB_888:
			_jpegd_set_output_fmt(JPEGD_OUT_DEFAULT);
			_jpegd_out_rgb(); break;
		case IMPP_PIX_FMT_YUYV:
			_jpegd_set_output_fmt(JPEGD_OUT_YUV422P);
			_jpegd_out_yuv(); break;
		case IMPP_PIX_FMT_BGRA_8888:
			_jpegd_set_output_fmt(JPEGD_OUT_ARGB32);
			_jpegd_out_rgb(); break;
		case IMPP_PIX_FMT_NV12:
			_jpegd_set_output_fmt(JPEGD_OUT_NV12);
			_jpegd_out_yuv(); break;
		case IMPP_PIX_FMT_NV21:
			_jpegd_set_output_fmt(JPEGD_OUT_NV21);
			_jpegd_out_yuv(); break;
		default:
			printk("input fmt not support \r\n");
			break;
	}
	_jpegd_finish_irq_enable();
	_jpegd_err_irq_enable();
	_jpegd_start();         //// start start

	ret = wait_for_completion_interruptible_timeout(&jpegd->done_decoder, msecs_to_jiffies(3000));
	if (ret < 0) {
		printk("wait completion error !!!!\r\n");
		_jpegd_finish_irq_disable();
		_jpegd_err_irq_disable();
		return -EFAULT;
	} else if (ret == 0) {
		printk("wait completion time out !!!!!\r\n");
	} else {
		// wiat ok
		// 1.读取状态寄存器
		tmp = JPEGD_READ_REG(JPEGD_CTL_STA);
		out->yuv_type = (tmp & 0xf0000) >> 16;
		//读取宽高信息
		tmp = JPEGD_READ_REG(JPEGD_ROW_COL);
		out->width = tmp & 0xffff;
		out->height = (tmp >> 16) & 0xffff;
		out->size = JPEGD_READ_REG(JPEGD_DEC_SIZE);
		if (out->width == 0 || out->height == 0) {
			ret = -EFAULT;
		} else {
			ret = 0;
		}
	}

	return ret;
}

static long jpegd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_jpeg_dec *jpegd = container_of(dev, struct jz_jpeg_dec, misc_dev);
	int ret = 0;
	struct jpegd_info param_info;
	mutex_lock(&jpegd->mutex);
	switch (cmd) {
		case JZ_JPEGD_START:
			if (copy_from_user(&param_info, (struct jpegd_info *)arg, sizeof(struct jpegd_info))) {
				return -EFAULT;
			}
			ret = _jpeg_dec_start(jpegd, &param_info);
			if (copy_to_user((void *)arg, &param_info, sizeof(struct jpegd_info))) {
				ret = -EFAULT;
			}
			break;
		case JZ_JPEGD_ALLOC_DMABUF:
			ret = jpeg_ioctl_get_dma_fd(jpegd->dev, arg);
			break;
		case JZ_JPEGD_GET_DMABUF_PADDR:
			ret = jpeg_ioctl_get_dmabuf_dma_addr(jpegd->dev, arg);
			break;
		case JZ_JPEGD_FREE_BUFFER:
			jpeg_ioctl_free_dmabuf(jpegd->dev, arg);
			break;
		case JZ_JPEGD_GET_RESERVED_STA:
			ret = _jpeg_get_reserved_statu(jpegd, (int *)arg);
			break;
		default :
			ret = -EFAULT;
	}
	mutex_unlock(&jpegd->mutex);
	return ret;
}

static struct file_operations jpegd_ops = {
	.owner = THIS_MODULE,
	.open = jpegd_open,
	.release = jpegd_release,
	.unlocked_ioctl = jpegd_ioctl,
};

static int jpegd_probe(struct platform_device *pdev)
{
	struct jz_jpeg_dec *jpegd;
	int ret = 0;
	jpegd = (struct jz_jpeg_dec *)kzalloc(sizeof(struct jz_jpeg_dec), GFP_KERNEL);
	if (!jpegd) {
		dev_err(&pdev->dev, "alloc jz_jpeg_dec failed");
		return -ENOMEM;
	}

	sprintf(jpegd->name, "jpegdec");

	jpegd->misc_dev.minor = MISC_DYNAMIC_MINOR;
	jpegd->misc_dev.name = jpegd->name;
	jpegd->misc_dev.fops = &jpegd_ops;
	jpegd->dev = &pdev->dev;

	jpegd->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!jpegd->res) {
		dev_err(&pdev->dev, "failed to get jpegd resource \n");
		ret = -EINVAL;
		goto err_get_platform_res;
	}

	jpegd->iomem = ioremap(jpegd->res->start, resource_size(jpegd->res));
	if (!jpegd->iomem) {
		dev_err(&pdev->dev, "failed to remap mem region\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	// TODO clk get
	jpegd->clk = clk_get(&pdev->dev, "gate_jpegd");
	if (IS_ERR(jpegd->clk)) {
		dev_err(&pdev->dev, "failed to get jpegd clk gate\n");
		ret = -EINVAL;
		goto err_get_clk;
	}

	jpegd->irq = platform_get_irq(pdev, 0);
	if (request_irq(jpegd->irq, jpegd_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_HIGH
	                , jpegd->name, jpegd)) {
		dev_err(&pdev->dev, "request jpeg decoder irq failed\n");
		ret = -EINVAL;
		goto err_request_irq;
	}

	dev_set_drvdata(&pdev->dev, jpegd);

	ret = misc_register(&jpegd->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register misc device failed\n");
		goto err_misc_register;
	}
	mutex_init(&jpegd->mutex);
	init_completion(&jpegd->done_decoder);
	init_completion(&jpegd->done_buf);

	clk_prepare_enable(jpegd->clk);

	ret = of_reserved_mem_device_init(jpegd->dev);
	if (ret) {
		printk("failed to init reserved mem !!!!!!!!!!!!!!!!!!!!\n");
	}

	printk("JZ JPEG DECODER TEST DRIVER Probe end !!!!!!!!!!!!!!\r\n");

	return 0;
err_misc_register:
	free_irq(jpegd->irq, jpegd);
err_request_irq:
	iounmap(jpegd->iomem);
err_get_clk:
err_ioremap:
err_get_platform_res:
	kfree(jpegd);
	return ret;

}

static int jpegd_remove(struct platform_device *pdev)
{
	struct jz_jpeg_dec *jpegd;
	struct resource *res;
	jpegd = dev_get_drvdata(&pdev->dev);
	misc_deregister(&jpegd->misc_dev);
	clk_disable_unprepare(jpegd->clk);
	res = jpegd->res;
	free_irq(jpegd->irq, jpegd);
	iounmap(jpegd->iomem);
	release_mem_region(res->start, res->end - res->start + 1);
	if (jpegd) {
		kfree(jpegd);
	}
	return 0;
}

static const struct of_device_id ingenic_jpegd_dt_match[] = {
	{ .compatible = "ingenic,x2600-jpegd", .data = NULL },
	{ .compatible = "ingenic,ad100-jpegd", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_jpegd_dt_match);

static struct platform_driver jz_jpegd_driver = {
	.probe  = jpegd_probe,
	.remove = jpegd_remove,
	.driver = {
		.name = "jz-jpegd",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_jpegd_dt_match),
	},
};

static int __init jpegddev_init(void)
{
	platform_driver_register(&jz_jpegd_driver);
	return 0;
}

static void __exit jpegddev_exit(void)
{
	platform_driver_unregister(&jz_jpegd_driver);
}

module_init(jpegddev_init);
module_exit(jpegddev_exit);

MODULE_LICENSE("GPL");
