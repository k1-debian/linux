
/*
 * dtrng driver of Ingenic's SoC X2000
 *
 * Copyright (C) 2020 Ingenic Semiconductor Co., Ltd.
 *      http://www.ingenic.com
 * Author:      wenshuo.song <wenshuo.song@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>
#include <irq.h>
#include <linux/random.h>
#include <linux/interrupt.h>
#include <linux/of.h>

#define TRNG_INT_SWITCH
#undef  TRNG_INT_SWITCH

#define TRNG_CFG    0x00
#define TRNG_RANDOMNUM  0x04
#define TRNG_STATUS 0x08

#define TRNG_CFG_RDY_CLR    (1 << 12)
#define TRNG_CFG_GEN_EN     (1 << 0)
#define TRNG_CFG_INT_MASK   (1 << 11)

/*f(clk)=f(pclk)/(div_num+1)*/
#define TRNG_CFG_DIV_NUM 0x00

struct ingenic_trng {
	int irq;
	struct clk *clk_gate;
	void __iomem *base;
	struct hwrng rng;
};

#ifdef TRNG_INT_SWITCH
static irqreturn_t ingenic_trng_interrupt(int irq, void *dev_id)
{

	unsigned int trng_cfg;
	unsigned int trng_randomnum;
	struct ingenic_trng *trng = (struct ingenic_trng *)(dev_id);

	//TODO?

	trng_randomnum = readl(trng->base + TRNG_RANDOMNUM);

	/* Clear the interupt */
	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg |= TRNG_CFG_RDY_CLR;
	writel(trng_cfg, trng->base + TRNG_CFG);
	trng_cfg &= ~TRNG_CFG_RDY_CLR;
	writel(trng_cfg, trng->base + TRNG_CFG);

	return IRQ_HANDLED;
}
#endif

static void ingenic_trng_enable(struct hwrng *rng)
{
	unsigned int trng_cfg;
	struct ingenic_trng *trng = container_of(rng, struct ingenic_trng, rng);

	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg |= (1 << 0);
	writel(trng_cfg, trng->base + TRNG_CFG);

}

static void ingenic_trng_disable(struct hwrng *rng)
{
	unsigned int trng_cfg;
	struct ingenic_trng *trng = container_of(rng, struct ingenic_trng, rng);

	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg &= ~(1 << 0);
	writel(trng_cfg, trng->base + TRNG_CFG);

}

static int ingenic_trng_init(struct hwrng *rng)
{

	unsigned int trng_cfg;
	struct ingenic_trng *trng = container_of(rng, struct ingenic_trng, rng);

	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg |= (1 << 0);
	writel(trng_cfg, trng->base + TRNG_CFG);

	return 0;
}

static void ingenic_trng_cleanup(struct hwrng *rng)
{
	unsigned int trng_cfg;
	struct ingenic_trng *trng = container_of(rng, struct ingenic_trng, rng);

	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg &= ~TRNG_CFG_GEN_EN;
	writel(trng_cfg, trng->base + TRNG_CFG);
}

static int ingenic_trng_read(struct hwrng *rng, void *buf, size_t max,
                             bool wait)
{

	struct ingenic_trng *trng = container_of(rng, struct ingenic_trng, rng);
	unsigned int trng_cfg;
	u32 val, tmp_data;
	u32 *data = buf;
	int ret;

	ingenic_trng_enable(rng);

	/* data ready? */
	if (readl(trng->base + TRNG_STATUS) & 1) {
		get_random_bytes(&val, sizeof(u32));
		tmp_data = readl(trng->base + TRNG_RANDOMNUM);
		tmp_data ^= val;
		*data = tmp_data;

		/* Clear the ramdom_rdy */
		trng_cfg = readl(trng->base + TRNG_CFG);
		trng_cfg |= TRNG_CFG_RDY_CLR;
		writel(trng_cfg, trng->base + TRNG_CFG);
		trng_cfg &= ~TRNG_CFG_RDY_CLR;
		writel(trng_cfg, trng->base + TRNG_CFG);

		ret = 4;
	} else {
		ret = 0;
	}

	ingenic_trng_disable(rng);

	return ret;
}

static int ingenic_trng_probe(struct platform_device *pdev)
{
	struct ingenic_trng *trng;
	struct resource *res;
	int ret;

	trng = devm_kzalloc(&pdev->dev, sizeof(*trng), GFP_KERNEL);
	if (!trng) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	trng->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(trng->base)) {
		return PTR_ERR(trng->base);
	}

	trng->irq = platform_get_irq(pdev, 0);
	if (trng->irq < 0) {
		dev_err(&pdev->dev, "Cannot get %d  IORESOURCE_IRQ\n", trng->irq);
		return  -ENOENT;
	}

	trng->clk_gate = devm_clk_get(&pdev->dev, "gate_dtrng");
	if (IS_ERR(trng->clk_gate)) {
		return PTR_ERR(trng->clk_gate);
	}

	ret = clk_prepare_enable(trng->clk_gate);
	if (ret) {
		return ret;
	}

	trng->rng.name = pdev->name;
	trng->rng.init = ingenic_trng_init;
	trng->rng.cleanup = ingenic_trng_cleanup;
	trng->rng.read = ingenic_trng_read;

#ifdef TRNG_INT_SWITCH
	writel(readl(TRNG_CFG) & ~TRNG_CFG_INT_MASK, TRNG_CFG);
	ret = request_irq(trng->irq, ingenic_trng_interrupt,
	                  IRQF_SHARED | IRQF_TRIGGER_LOW, "trng-interrupt", trng);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed !! %d-\n", trng->irq);
		goto err_register;
	}
#endif
	ret = hwrng_register(&trng->rng);
	if (ret) {
		goto err_register;
	}

	platform_set_drvdata(pdev, trng);

	ingenic_trng_disable(&trng->rng);

	dev_info(&pdev->dev, "ingenic HW dtrng Probe success!\n");
	return 0;

err_register:
	clk_disable(trng->clk_gate);
	return ret;
}

static int ingenic_trng_remove(struct platform_device *pdev)
{
	struct ingenic_trng *trng = platform_get_drvdata(pdev);
	unsigned int trng_cfg;

	hwrng_unregister(&trng->rng);

	trng_cfg = readl(trng->base + TRNG_CFG);
	trng_cfg &= ~(1 << 0);
	writel(trng_cfg, trng->base + TRNG_CFG);

	clk_disable_unprepare(trng->clk_gate);

	return 0;
}

#ifdef CONFIG_PM
static int ingenic_trng_suspend(struct device *dev)
{
	struct ingenic_trng *trng = dev_get_drvdata(dev);

	clk_disable_unprepare(trng->clk_gate);

	return 0;
}

static int ingenic_trng_resume(struct device *dev)
{
	struct ingenic_trng *trng = dev_get_drvdata(dev);

	return clk_prepare_enable(trng->clk_gate);
}

static const struct dev_pm_ops ingenic_trng_pm_ops = {
	.suspend    = ingenic_trng_suspend,
	.resume     = ingenic_trng_resume,
};
#endif /* CONFIG_PM */

static const struct of_device_id ingenic_trng_dt_ids[] = {
	{ .compatible = "ingenic,dtrng" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ingenic_trng_dt_ids);

static struct platform_driver ingenic_trng_driver = {
	.probe      = ingenic_trng_probe,
	.remove     = ingenic_trng_remove,
	.driver     = {
		.name   = "ingenic-trng",
#ifdef CONFIG_PM
		.pm = &ingenic_trng_pm_ops,
#endif /* CONFIG_PM */
		.of_match_table = ingenic_trng_dt_ids,
	},
};

module_platform_driver(ingenic_trng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wenshuo.song <wenshuo.song@ingenic.com>");
MODULE_DESCRIPTION("ingenic true random number generator driver");
