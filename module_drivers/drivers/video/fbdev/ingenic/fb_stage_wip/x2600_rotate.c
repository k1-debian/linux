#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "x2600_rotate.h"
#include <linux/dma-mapping.h>

void dump_rot_registers(void)
{
	printk("==================Rotator Descriptor register start======================\n");
	printk("BUFF_CFG_ADDR(002c): 0x%08x\n", ROT_READ_REG(BUFF_CFG_ADDR));
	printk("WDMA_CFG_ADDR(003c): 0x%08x\n", ROT_READ_REG(WDMA_CFG_ADDR));
	printk("FRAM_SIZE(0004):     0x%08x\n", ROT_READ_REG(ROT_FRM_SIZE));
	printk("DC_GLB_CFG(0008):    0x%08x\n", ROT_READ_REG(ROT_GLB_CFG));
	printk("CTRL(000C):          0x%08x\n", ROT_READ_REG(ROT_CTRL));
	printk("DC_ST(0010):         0x%08x\n", ROT_READ_REG(ROT_STATUS));
	printk("DC_CSR(0014):        0x%08x\n", ROT_READ_REG(ROT_CLR_STATUS));
	printk("DC_MASK(0018):       0x%08x\n", ROT_READ_REG(ROT_INT_MASK));
	printk("RDMA_SITE(0020):     0x%08x\n", ROT_READ_REG(ROT_RDMA_SITE));
	printk("ROT_QOS_CTRL(0030):  0x%08x\n", ROT_READ_REG(ROT_QOS_CTRL));
	printk("ROT_COUNT(0040):     0x%08x\n", ROT_READ_REG(ROT_COUNT));
	printk("==================Rotator Descriptor register end======================\n");

}

static int rot_alloc_buf(struct dpu_ctrl *dctrl)
{
	struct fb_videomode *mode = dctrl->active_video_mode;
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	int i = 0;
	int linesize = mode->xres * 4;
	linesize = rotator_aligned_size(linesize);
	size = linesize * mode->yres;

	size = ALIGN(size, ROTBUF_ALIGN);
	alloc_size = size * CONFIG_ROT_INGENIC_BUFS;
	dctrl->rot_wbbuf[0] = dma_alloc_coherent(dctrl->dev, alloc_size, &dctrl->rot_wbbuf_phy[0], GFP_KERNEL);
	for (i = 0; i < CONFIG_ROT_INGENIC_BUFS; i++) {
		dctrl->rot_wbbuf[i] = (int32_t *)(dctrl->rot_wbbuf[0] + size * i);
		dctrl->rot_wbbuf_phy[i] = dctrl->rot_wbbuf_phy[0] + i * size;
	}

	return 0;
}

static int rot_release_buf(struct dpu_ctrl *dctrl)
{
	struct fb_videomode *mode = dctrl->active_video_mode;
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	size = (mode->xres / 128 + 1) * mode->yres * 512;
	alloc_size = size * CONFIG_ROT_INGENIC_BUFS;
	dma_free_coherent(dctrl->dev, alloc_size, dctrl->rot_wbbuf[0], dctrl->rot_wbbuf_phy[0]);
	return 0;
}

void rotator_cfg_init(struct rot_cfg *rot_cfg)
{
	uint32_t frm_size = 0;
	uint32_t cfg = 0;
	if (rot_cfg->rot_angle == 180) {
		frm_size |= (rot_cfg->xres) << ROT_FRM_WIDTH_LBIT;
		frm_size |= (rot_cfg->yres) << ROT_FRM_HEIGHT_LBIT;
		cfg |= ROT_ANGLE_180;
	} else {
		frm_size |= (rot_cfg->yres) << ROT_FRM_WIDTH_LBIT;
		frm_size |= (rot_cfg->xres) << ROT_FRM_HEIGHT_LBIT;
		if (rot_cfg->rot_angle == 90) {
			cfg |= ROT_ANGLE_90;
		} else {
			cfg |= ROT_ANGLE_270;
		}
	}
	ROT_WRITE_REG(ROT_FRM_SIZE, frm_size);//设置图像的宽和高

	cfg |= ROT_RDMA_BURST_4; //burst len 4
	cfg |= ROT_RDMA_ORDER_RGB;
	if (rot_cfg->rot_manual_auto) {
		cfg |= ROT_MANUAL_AUTO;
	} else {
		cfg &= ~ROT_MANUAL_AUTO;
	}
	cfg |= ROT_RDMA_FMT_CUSTOM;
	cfg &= ~ROT_FIFO_GATE;
	ROT_WRITE_REG(ROT_GLB_CFG, cfg);
	//    ROT_WRITE_REG(ROT_CTRL, 0);
	rot_cfg->rot_stop = 0;
}

void rot_manual_start(struct dpu_ctrl *dctrl)
{
	ROT_WRITE_REG(BUFF_CFG_ADDR, dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf] | 1);
	ROT_WRITE_REG(ROT_CTRL, 0);
	ROT_WRITE_REG(ROT_CLR_STATUS, 0xff);
	//    dump_rot_registers();
}

void rot_quick_stop(void)
{
	int cfg = 0;
	ROT_WRITE_REG(ROT_CTRL, ROT_QCK_STP);
	cfg = ROT_READ_REG(ROT_GLB_CFG);
	cfg |= ROT_FIFO_GATE;
	ROT_WRITE_REG(ROT_GLB_CFG, cfg);
	ROT_WRITE_REG(ROT_CTRL, ROT_FLUSH_FIFO);
}

void rot_normal_stop(void)
{

	unsigned int timeout = 1000;
	int cfg = 0;
	cfg = ROT_READ_REG(ROT_GLB_CFG);
	cfg |= ROT_FIFO_GATE;
	ROT_WRITE_REG(ROT_GLB_CFG, cfg);

	ROT_WRITE_REG(ROT_CLR_STATUS, ROT_GEN_STOP_ACK | ROT_FLUSH_FIFO);
	ROT_WRITE_REG(ROT_CTRL, ROT_GEN_STP | ROT_FLUSH_FIFO);

	while ((ROT_READ_REG(ROT_STATUS) & ROT_BUF_WORKING)) {
		timeout--;
		udelay(20);
		if (timeout == 0) {
			break;
		}
	}
	if (timeout == 0) {
		printk("rot normal stop failed!\n");
	}
}

EXPORT_SYMBOL_GPL(rot_normal_stop);

static irqreturn_t rotator_ctrl_irq_handler(int irq, void *data)
{
	if (ROT_READ_REG(ROT_STATUS) & ROT_GEN_STOP_ACK) {
		ROT_WRITE_REG(ROT_CLR_STATUS, ROT_CLR_GEN_STOP_ACK);
		return IRQ_HANDLED;
	}

	if (ROT_READ_REG(ROT_STATUS) & ROT_EOF) {
		ROT_WRITE_REG(ROT_CLR_STATUS, ROT_CLR_EOF);
		return IRQ_HANDLED;
	}

	if (ROT_READ_REG(ROT_STATUS) & ROT_SOF) {
		ROT_WRITE_REG(ROT_CLR_STATUS, ROT_CLR_SOF);
		return IRQ_HANDLED;
	}

	if (ROT_READ_REG(ROT_STATUS) & ROT_FETCG_ADDR) {
		ROT_WRITE_REG(ROT_CLR_STATUS, ROT_CLR_FETCG_ADDR);
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}

int rot_ctrl_init(struct dpu_ctrl *dctrl, int angle, enum rot_mode mode)
{
	int ret;
	struct fb_videomode *fbmode = dctrl->active_video_mode;
	dctrl->rot_clk = devm_clk_get(dctrl->dev, "gate_rot");
	if (IS_ERR(dctrl->rot_clk)) {
		ret = PTR_ERR(dctrl->rot_clk);
		dev_err(dctrl->dev, "Failed to get rot clk\n");

		goto err_clk;
	}
	ROT_WRITE_REG(ROT_INT_MASK, ROT_GSA_MASK | ROT_EOF_MASK | ROT_SOF_MASK | ROT_FA_MASK);

	ROT_WRITE_REG(ROT_QOS_CTRL, QOS_FIX_EN | QOS_FIX_3);//设置固定最高优先级
	dctrl->roirq = platform_get_irq(dctrl->pdev, 1);
	dctrl->rot_cfg.rot_angle = angle;
	dctrl->rot_cfg.rot_angle_set = angle;
	if (mode == ROT_MODE_WDMA_AUTO) {
		dctrl->rot_cfg.rot_manual_auto = 1;
	} else if (mode == ROT_MODE_RDMA_MANUAL) {
		dctrl->rot_cfg.rot_manual_auto = 0;
	} else {
		printk("%s not support mode: %d\n", __func__, mode);
	}

	dctrl->rot_cfg.next_wbbuf = 0;
	dctrl->rot_cfg.rot_stop = 0;
	dctrl->rot_cfg.xres = fbmode->xres;
	dctrl->rot_cfg.yres = fbmode->yres;
	dctrl->rot_init = 1;

	sprintf(dctrl->roirq_name, "x2600-rotate");
	if (request_irq(dctrl->roirq, rotator_ctrl_irq_handler, 0,
	                dctrl->roirq_name, dctrl)) {
		ret = -EINVAL;
	}

	clk_prepare_enable(dctrl->rot_clk);
	if (mode == ROT_MODE_WDMA_AUTO) {
		rot_alloc_buf(dctrl);
	} else if (mode == ROT_MODE_RDMA_MANUAL) {
		ROT_WRITE_REG(ROT_INT_MASK, 0);
	} else {
		printk("%s not support mode: %d\n", __func__, mode);
	}

	rotator_cfg_init(&dctrl->rot_cfg);

	printk("rot_ctrl_init success\n");
	return 0;
err_clk:
	devm_clk_put(dctrl->dev, dctrl->rot_clk);
	return ret;
}

int rot_ctrl_exit(struct dpu_ctrl *dctrl)
{
	free_irq(dctrl->roirq, dctrl);
	rot_release_buf(dctrl);
	devm_clk_put(dctrl->dev, dctrl->rot_clk);
	return 0;
}
