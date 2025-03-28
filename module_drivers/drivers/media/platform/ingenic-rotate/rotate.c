#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/kfifo.h>
//#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include "rotate.h"
struct ingenic_rot_dev *global_dev = NULL;
#if defined(CONFIG_FB_INGENIC_STAGE_WIP) || defined(CONFIG_FB_INGENIC_STAGE)
extern struct mutex rot_dpu_mutex;
#else
DEFINE_MUTEX(rot_dpu_mutex);
#endif

void rot_do_rotate(struct ingenic_rot_ctx *ctx)
{

	int ret;
	mutex_lock(&rot_dpu_mutex);
	clk_prepare_enable(ctx->dev->clk);
	ctx->rot_end = 0;
	ctx->dev->rot_curr = ctx;
	rot_reset(ctx->dev);
	rot_set_src_desc(ctx->dev, ctx->src_addr);
	rot_set_dst_desc(ctx->dev, ctx->dst_addr);
	rot_set_src_cfg(ctx->dev, &ctx->in);
	rot_set_dst_cfg(ctx->dev, &ctx->out);
	rot_set_angle(ctx->dev,ctx->angle);
	rot_set_hflip(ctx->dev,ctx->hflip);
	rot_set_vflip(ctx->dev,ctx->vflip);
	rot_start(ctx->dev);
	ret = wait_for_completion_timeout(&ctx->dev->ctx_done, msecs_to_jiffies(3000));
	if(ret < 0) {
		printk("wait rot done error !!!!!\r\n");
	} else if(ret == 0) {
		printk("wait rot done time out !!!!\r\n");
	}
	clk_disable_unprepare(ctx->dev->clk);
	mutex_unlock(&rot_dpu_mutex);
}

static int rot_reqdesc(struct ingenic_rot_ctx *ctx)
{
	struct ingenic_rot_dev *dev = ctx->dev;
	int i;
	size_t size;

	size = sizeof(struct ingenic_rot_desc) * ROT_DESC_NUM;
	ctx->desc[0] = (struct ingenic_rot_desc *)dma_alloc_coherent(dev->dev, size,
			&ctx->desc_phys[0], GFP_KERNEL);
	if (!ctx->desc[0]) {
		dev_err(dev->dev, "dma_alloc_coherent of size %d failed\n", size);
		return -ENOMEM;
	}

	for(i = 1; i < ROT_DESC_NUM; i++) {
		ctx->desc[i] = ctx->desc[0] + i;
		ctx->desc_phys[i] = ctx->desc_phys[0] + i*sizeof(struct ingenic_rot_desc);
	}
	return 0;
}

static void rot_freedesc(struct ingenic_rot_ctx *ctx)
{
	struct ingenic_rot_dev *dev = ctx->dev;
	int size;

	size = sizeof(struct ingenic_rot_desc) * ROT_DESC_NUM;
	dma_free_coherent(dev->dev, size, ctx->desc[0], ctx->desc_phys[0]);
}

static void rot_update_info(struct ingenic_rot_ctx *ctx)
{
	struct rot_frm_cfg *in, *out;
	in = &ctx->in;
	out = &ctx->out;

	in->width = DEFAULT_WIDTH;
	in->height = DEFAULT_HEIGHT;
	in->rot_format = ROT_RDMA_FMT_ARGB8888;
	in->rot_color = ROT_RDMA_ORDER_RGB;

	out->width = DEFAULT_WIDTH;
	out->height = DEFAULT_HEIGHT;
	out->rot_format = ROT_WDMA_FMT_ARGB8888;

}
struct ingenic_rot_ctx *rot_create_ctx(void)
{
	int ret;
	struct ingenic_rot_ctx *ctx = NULL;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL|GFP_ATOMIC);
	if (!ctx) {
		dev_err(global_dev->dev, "rot_ctx alloc faield\n");
		return NULL;
	}
	ctx->dev = global_dev;
	init_completion(&global_dev->ctx_done);
	rot_update_info(ctx);
	ret = rot_reqdesc(ctx);
	if(ret) {
		kfree(ctx);
		return NULL;
	}
	ctx->rot_end = 0;
	return ctx;
}
EXPORT_SYMBOL(rot_create_ctx);

int rot_destory_ctx(struct ingenic_rot_ctx *ctx)
{
	rot_freedesc(ctx);
	kfree(ctx);
	return 0;
}
EXPORT_SYMBOL(rot_destory_ctx);

static irqreturn_t rot_irq_handler(int irq, void *prv)
{
	struct ingenic_rot_dev *dev = prv;
	rot_clr_irq(dev);
	complete(&global_dev->ctx_done);
	return IRQ_HANDLED;
}

static int ingenic_rot_probe(struct platform_device *pdev)
{
	struct ingenic_rot_dev *dev;
	struct resource *res;
	int ret = 0;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->dev = &pdev->dev;
	mutex_init(&dev->rot_mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs))
		return PTR_ERR(dev->regs);

	dev->irq = ret = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return ret;
	}

	dev->name = pdev->name;
	if(devm_request_irq(dev->dev, dev->irq, rot_irq_handler, IRQF_SHARED, dev->name, dev)){
		dev_err(dev->dev, "request irq failed\n");
		ret = -EINVAL;
	}

	dev->clk = clk_get(&pdev->dev, "gate_rot");
	if (IS_ERR(dev->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(dev->clk);
		goto clk_get_rollback;
	}
	dev_dbg(&pdev->dev, "rot clock source %p\n", dev->clk);
	platform_set_drvdata(pdev, dev);
	ret = of_reserved_mem_device_init(dev->dev);
	if (ret)
		dev_warn(dev->dev, "Could not get reserved memory\n");
	global_dev = dev;
	global_dev->rot_v4l2 = rot_v4l2_init(&pdev->dev);
	return 0;

clk_get_rollback:
	clk_put(dev->clk);
	return ret;
}

static int ingenic_rot_remove(struct platform_device *pdev)
{
	struct ingenic_rot_dev *dev = (struct ingenic_rot_dev *)platform_get_drvdata(pdev);
	rot_v4l2_remove(dev->rot_v4l2);
	clk_put(dev->clk);
	return 0;
}

static const struct of_device_id ingenic_rotate_match[] = {
	{
		.compatible = "ingenic,x2000-rotate",
		.data = NULL,
	},
	{
		.compatible = "ingenic,m300-rotate",
		.data = NULL,
	},
	{
		.compatible = "ingenic,x2600-rotate",
		.data = NULL,
	},
	{ },
};

MODULE_DEVICE_TABLE(of, ingenic_rotate_match);


static struct platform_driver rot_pdrv = {
	.probe		= ingenic_rot_probe,
	.remove		= ingenic_rot_remove,
	.driver		= {
		.owner = THIS_MODULE,
		.of_match_table	= of_match_ptr(ingenic_rotate_match),
		.name = JZ_ROT_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(rot_pdrv);

MODULE_DESCRIPTION("X2000 rotate driver");
MODULE_LICENSE("GPL");
