#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/syscalls.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <asm/delay.h>
#include <linux/proc_fs.h>
#include <linux/of.h>

#define JZ_BSCALER_NUM      2

#define IOCTL_BSCALER0_IRQ_GET_STAT 0x000
#define IOCTL_BSCALER1_IRQ_GET_STAT 0x100

#define REG_BSCALER_STAT0       (0x00)
#define REG_BSCALER_CTRL0       (0xb0)  //44*4 FRMT
#define REG_BSCALER_STAT1       (0x80)
#define REG_BSCALER_CTRL1       (0x30)  //12*4  FRMC

struct jz_bscaler_context {
	int         irq;
	struct completion   done;
	spinlock_t      slock;
	unsigned int        status;
};

struct jz_bscaler {
	char                name[16];
	char                irq_name[32];;
	int             idx;
	void __iomem            *iomem;
	struct clk          *clk;
	struct clk          *clk_gate;
	struct clk          *ahb0_gate;
	pid_t               owner_pid;
	struct jz_bscaler_context   ctx[JZ_BSCALER_NUM];
	struct miscdevice       mdev;
	struct mutex            openmutex;
	int             opencnt;
};

#define jz_bscaler_readl(bscaler, offset)       __raw_readl((bscaler)->iomem + offset)
#define jz_bscaler_writel(bscaler, offset, value)   __raw_writel((value), (bscaler)->iomem + offset)

#define CLEAR_RADIX_BIT(bscaler,offset,bm)              \
	do {                            \
		unsigned int stat;              \
		stat = jz_bscaler_readl(bscaler,offset);            \
		jz_bscaler_writel(bscaler,offset,stat & ~(bm));     \
	} while(0)

static irqreturn_t jz_bscaler_interrupt_t(int irq, void *data)
{
	struct jz_bscaler *bscaler = (struct jz_bscaler *)data;
	struct jz_bscaler_context *ctx = NULL;
	unsigned int reg_ctrl = 0, reg_stat = 0;

	spin_lock(&bscaler->ctx[0].slock);
	if (irq == bscaler->ctx[0].irq) {
		ctx = &(bscaler->ctx[0]);
		reg_ctrl = REG_BSCALER_CTRL0;
	} else {
		printk("irq error!\n");
		return -1;
	}

	if (jz_bscaler_readl(bscaler, 0x80) & 0x100) {
		jz_bscaler_writel(bscaler, 0x80, (jz_bscaler_readl(bscaler, 0x80) & ~0x100));
	}

	if (jz_bscaler_readl(bscaler, 0x84) & 0x4) {
		jz_bscaler_writel(bscaler, 0x84, (jz_bscaler_readl(bscaler, 0x84) & ~0x4));
	}
	if (jz_bscaler_readl(bscaler, 0xb0) & 0x50) {
		if (jz_bscaler_readl(bscaler, 0xb0) & 0x40) {
			printk("ERROR: bscaler T irq timeout!!!\n");
		}
		jz_bscaler_writel(bscaler, 0xb0, (jz_bscaler_readl(bscaler, 0xb0) & ~0x50));
	}

	ctx->status = jz_bscaler_readl(bscaler, reg_stat);
	dev_dbg(bscaler->mdev.this_device, "In bscaler0_interrupt : irq=%d, status = 0x%08x\n", irq, ctx->status);
	complete(&ctx->done);

	spin_unlock(&bscaler->ctx[0].slock);

	return IRQ_HANDLED;
}

static irqreturn_t jz_bscaler_interrupt_c(int irq, void *data)
{
	struct jz_bscaler *bscaler = (struct jz_bscaler *)data;
	struct jz_bscaler_context *ctx = NULL;
	unsigned int reg_ctrl = 0, reg_stat = 0;

	spin_lock(&bscaler->ctx[1].slock);
	if (irq == bscaler->ctx[1].irq) {
		ctx = &(bscaler->ctx[1]);
		reg_ctrl = REG_BSCALER_CTRL1;
	} else {
		printk("irq error!\n");
		return -1;
	}

	if (jz_bscaler_readl(bscaler, 0x0) & 0x100) {
		jz_bscaler_writel(bscaler, 0x0, (jz_bscaler_readl(bscaler, 0x0) & ~0x100));
	}

	if (jz_bscaler_readl(bscaler, 0x30) & 0x50) {
		if (jz_bscaler_readl(bscaler, 0x30) & 0x40) {
			printk("ERROR: bscaler C irq timeout!!!\n");
		}
		jz_bscaler_writel(bscaler, 0x30, (jz_bscaler_readl(bscaler, 0x30) & ~0x50));
	}

	ctx->status = jz_bscaler_readl(bscaler, reg_stat);
	dev_dbg(bscaler->mdev.this_device, "In bscaler0_interrupt : irq=%d, status = 0x%08x\n", irq, ctx->status);
	complete(&ctx->done);

	spin_unlock(&bscaler->ctx[1].slock);

	return IRQ_HANDLED;
}

static int jz_bscaler_open(struct inode *inode, struct file *file)
{
	int i = 0;
	struct miscdevice *mdev = file->private_data;
	struct jz_bscaler *bscaler = list_entry(mdev, struct jz_bscaler, mdev);

	clk_prepare_enable(bscaler->clk);
	clk_prepare_enable(bscaler->clk_gate);
	mutex_lock(&bscaler->openmutex);
	if (!bscaler->opencnt) {
		for (i = 0; i < JZ_BSCALER_NUM; i++) {
			bscaler->ctx[i].done.done = 0;
			enable_irq(bscaler->ctx[i].irq);
		}
	}
	bscaler->opencnt++;
	mutex_unlock(&bscaler->openmutex);

	dev_dbg(mdev->this_device, "%s open successful, opencnt=%d!\n", __func__, bscaler->opencnt);

	return 0;
}

static int jz_bscaler_release(struct inode *inode, struct file *file)
{
	int i = 0;
	struct miscdevice *mdev = file->private_data;
	struct jz_bscaler *bscaler = list_entry(mdev, struct jz_bscaler, mdev);

	mutex_lock(&bscaler->openmutex);
	if (bscaler->opencnt == 1) {
		for (i = 0; i < JZ_BSCALER_NUM; i++) {
			disable_irq(bscaler->ctx[i].irq);
			bscaler->ctx[i].done.done = 0;
		}
		clk_disable(bscaler->clk);
		clk_disable(bscaler->clk_gate);
		bscaler->opencnt--;
	}
	mutex_unlock(&bscaler->openmutex);

	dev_dbg(mdev->this_device, "%s close successful, opencnt=%d!\n", __func__, bscaler->opencnt);

	return 0;
}

static ssize_t jz_bscaler_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t jz_bscaler_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static long jz_bscaler_wait_irq(struct jz_bscaler *bscaler, int bscaler_idx, unsigned long arg)
{
	long ret = 0;
	dev_dbg(bscaler->mdev.this_device, "%s:bscaler index=%d start\n", __func__, bscaler_idx);
	ret = wait_for_completion_timeout(&bscaler->ctx[bscaler_idx].done, msecs_to_jiffies(10 * 1000));
	if (ret <= 0) {
		dev_err(bscaler->mdev.this_device, "copy_to_user bscaler index=%d timeout!\n", bscaler_idx);
		return -EFAULT;
	}
	if (copy_to_user((void *)arg, &bscaler->ctx[bscaler_idx].status, sizeof(unsigned int))) {
		dev_err(bscaler->mdev.this_device, "copy_to_user bscaler index=%d, stat=0x%x failed!\n", bscaler_idx, bscaler->ctx[bscaler_idx].status);
		return -EFAULT;
	}
	dev_dbg(bscaler->mdev.this_device, "%s:bscaler index=%d end\n", __func__, bscaler_idx);

	return 0;
}

static long jz_bscaler_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = file->private_data;
	struct jz_bscaler *bscaler = list_entry(mdev, struct jz_bscaler, mdev);

	switch (cmd) {
		case IOCTL_BSCALER0_IRQ_GET_STAT:
			return jz_bscaler_wait_irq(bscaler, 0, arg);
		case IOCTL_BSCALER1_IRQ_GET_STAT:
			return jz_bscaler_wait_irq(bscaler, 1, arg);
		default:
			dev_err(mdev->this_device, "%s:invalid cmd %u\n", __func__, cmd);
			return -1;
	}

	return 0;
}

const struct file_operations jz_bscaler_fops = {
	.owner = THIS_MODULE,
	.open = jz_bscaler_open,
	.release = jz_bscaler_release,
	.read = jz_bscaler_read,
	.write = jz_bscaler_write,
	.unlocked_ioctl = jz_bscaler_ioctl,
};

static int jz_bscaler_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0;
	struct resource *regs = NULL;
	struct jz_bscaler *bscaler = NULL;

	bscaler = devm_kzalloc(&pdev->dev, sizeof(struct jz_bscaler), GFP_KERNEL);
	if (!bscaler) {
		dev_err(&pdev->dev, "kzalloc struct jz_bscaler failed\n");
		ret = -ENOMEM;
		goto err_kzalloc_bscaler;
	}

	pdev->id = of_alias_get_id(pdev->dev.of_node, "bscaler");
	bscaler->idx = pdev->id;
	sprintf(bscaler->name, "%s%d", "bscaler", pdev->id);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "No iomem resource\n");
		ret = -ENXIO;
		goto err_get_bscaler_resource;
	}

	bscaler->iomem = devm_ioremap_resource(&pdev->dev, regs);
	if (!bscaler->iomem) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENXIO;
		goto err_get_bscaler_iomem;
	}
	bscaler->clk_gate = devm_clk_get(&pdev->dev, "gate_bscaler");
	if (IS_ERR(bscaler->clk_gate)) {
		dev_err(&pdev->dev, "clk_gate get bscaler failed\n");
		ret = PTR_ERR(bscaler->clk_gate);
		goto err_get_bscaler_clk_gate;
	}
	bscaler->clk = devm_clk_get(&pdev->dev, "div_bscaler");
	if (IS_ERR(bscaler->clk)) {
		dev_err(&pdev->dev, "clk get cgu_bscaler failed\n");
		ret = PTR_ERR(bscaler->clk);
		goto err_get_bscaler_clk_cgu;
	}

	clk_set_parent(bscaler->clk, clk_get(NULL, "epll"));
	clk_set_rate(bscaler->clk, 400000000);

	for (i = 0; i < JZ_BSCALER_NUM; i++) {
		bscaler->ctx[i].irq = platform_get_irq(pdev, i);
		if (bscaler->ctx[i].irq < 0) {
			dev_err(&pdev->dev, "get irq %d failed\n", i);
			goto err_get_bscaler_irq;
		}
		sprintf(bscaler->irq_name, "%s%d", bscaler->name, bscaler->ctx[i].irq);

		ret = devm_request_irq(&pdev->dev, bscaler->ctx[i].irq, i == 0 ? jz_bscaler_interrupt_t : jz_bscaler_interrupt_c, 0, bscaler->irq_name, bscaler);
		if (ret < 0) {
			dev_err(&pdev->dev, "request_irq i=%d, irq=%d failed\n", i, bscaler->ctx[i].irq);
			goto err_bscaler_helix_irq;
		}
		disable_irq_nosync(bscaler->ctx[i].irq);

		init_completion(&bscaler->ctx[i].done);
		spin_lock_init(&bscaler->ctx[i].slock);
	}
	mutex_init(&bscaler->openmutex);
	bscaler->opencnt = 0;

	bscaler->mdev.minor = MISC_DYNAMIC_MINOR;
	bscaler->mdev.fops = &jz_bscaler_fops;
	bscaler->mdev.name = bscaler->name;
	ret = misc_register(&bscaler->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "request bscaler misc device failed!\n");
		goto err_bscaler_register;
	}
	platform_set_drvdata(pdev, bscaler);
	return 0;

err_bscaler_register:
	i = JZ_BSCALER_NUM;
err_bscaler_helix_irq:
err_get_bscaler_irq:
	for (--i; i >= 0; i--) {
		free_irq(bscaler->ctx[i].irq, bscaler);
	}
	clk_put(bscaler->clk);
err_get_bscaler_clk_cgu:
	clk_put(bscaler->clk_gate);
err_get_bscaler_clk_gate:
	iounmap(bscaler->iomem);
err_get_bscaler_iomem:
err_get_bscaler_resource:
	kfree(bscaler);
	bscaler = NULL;
err_kzalloc_bscaler:
	return ret;
}

static int jz_bscaler_remove(struct platform_device *pdev)
{
	int i;
	struct jz_bscaler *bscaler = platform_get_drvdata(pdev);

	misc_deregister(&bscaler->mdev);

	for (i = JZ_BSCALER_NUM - 1; i >= 0; i--) {
		free_irq(bscaler->ctx[i].irq, bscaler);
	}

	clk_disable_unprepare(bscaler->clk);
	clk_disable_unprepare(bscaler->clk_gate);

	devm_iounmap(&pdev->dev, bscaler->iomem);

	if (bscaler) {
		kfree(bscaler);
	}

	bscaler = NULL;

	return 0;
}

static const struct of_device_id ingenic_bscaler_dt_match[] = {
	{ .compatible = "ingenic,x2500-bscaler", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_bscaler_dt_match);

static struct platform_driver jz_bscaler_driver = {
	.probe      = jz_bscaler_probe,
	.remove     = jz_bscaler_remove,
	.driver     = {
		.name   = "jz-bscaler",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_bscaler_dt_match),
	},
};

module_platform_driver(jz_bscaler_driver);

MODULE_DESCRIPTION("JZ BSCALER driver");
MODULE_AUTHOR("Justin <pengtao.kang@ingenic.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20200820");
