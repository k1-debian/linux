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
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/suspend.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>
#include <linux/atomic.h>
#ifdef CONFIG_MMU_NOTIFIER
	#include <libdmmu.h>
#endif
#include "dpu_reg.h"
#include "dpu_ctrl.h"
#include "sysfs.h"

#ifdef CONFIG_SOC_X2600
	#include "x2600_rotate.h"
	#include "../jz_mipi_dsi/jz_mipi_dsi_phy.h"
#endif

static int _do_comp_start(struct dpu_ctrl *dctrl);
static int _dpu_ctrl_comp_try_start(struct dpu_ctrl *dctrl);


DEFINE_MUTEX(rot_dpu_mutex);
//#define TEST_COMPOSER_2_RDMA //测试Composer写回，使用RDMA通道显示。60 fps

extern irqreturn_t dsi_irq_handler(int irq, void *data);
static void ingenic_dsi_worker(struct work_struct *work);
#ifdef CONFIG_SOC_X2600
struct x2600_lvds_pclk_param {
	unsigned int pclk_freq;
	unsigned int pll_freq;
};

struct x2600_lvds_pclk_param pclk_epll_tab[] = {

	{30000000, 420000000},
	{31000000, 434000000},
	{31500000, 441000000},
	{32000000, 448000000},
	{33000000, 462000000},
	{34000000, 476000000},
	{34500000, 483000000},
	{35000000, 490000000},
	{36000000, 504000000},
	{37000000, 518000000},
	{37500000, 525000000},
	{38000000, 532000000},
	{39000000, 546000000},
	{40500000, 567000000},
	{42000000, 588000000},
	{43500000, 609000000},
	{45000000, 630000000},
	{46500000, 651000000},
	{48000000, 672000000},
	{48000000, 672000000},
	{49500000, 693000000},
	{50000000, 700000000},
	{51000000, 714000000},
	{52000000, 728000000},
	{54000000, 756000000},
	{56000000, 784000000},
	{57000000, 798000000},
	{58000000, 812000000},
	{60000000, 840000000},
	{62000000, 868000000},
	{63000000, 882000000},
	{64000000, 896000000},
	{66000000, 924000000},
	{68000000, 952000000},
	{69000000, 966000000},
	{70000000, 980000000},
	{72000000, 1008000000},
	{74000000, 1036000000},
	{75000000, 1050000000},
	{76000000, 1064000000},
	{78000000, 1092000000},
	{80000000, 1120000000},
	{81000000, 1134000000},
	{82000000, 1148000000},
	{84000000, 1176000000},
	{86000000, 1204000000},
	{87000000, 1218000000},
	{88000000, 1232000000},
	{90000000, 1260000000},
};
#endif

#ifdef CONFIG_FB_INGENIC_DUMP
static void dump_dc_reg(struct dpu_ctrl *dctrl)
{
	printk("-----------------dc_reg------------------\n");
	printk("DC_FRM_CFG_ADDR(0x%04x):    0x%08lx\n", DC_FRM_CFG_ADDR, reg_read(dctrl, DC_FRM_CFG_ADDR));
	printk("DC_FRM_CFG_CTRL(0x%04x):    0x%08lx\n", DC_FRM_CFG_CTRL, reg_read(dctrl, DC_FRM_CFG_CTRL));
	printk("DC_RDMA_CHAIN_ADDR(0x%04x):    0x%08lx\n", DC_RDMA_CHAIN_ADDR, reg_read(dctrl, DC_RDMA_CHAIN_ADDR));
	printk("DC_RDMA_CHAIN_CTRL(0x%04x):    0x%08lx\n", DC_RDMA_CHAIN_CTRL, reg_read(dctrl, DC_RDMA_CHAIN_CTRL));
	printk("DC_CTRL(0x%04x):            0x%08lx\n", DC_CTRL, reg_read(dctrl, DC_CTRL));
	printk("DC_CSC_MULT_YRV(0x%04x):    0x%08lx\n", DC_CSC_MULT_YRV, reg_read(dctrl, DC_CSC_MULT_YRV));
	printk("DC_CSC_MULT_GUGV(0x%04x):   0x%08lx\n", DC_CSC_MULT_GUGV, reg_read(dctrl, DC_CSC_MULT_GUGV));
	printk("DC_CSC_MULT_BU(0x%04x):     0x%08lx\n", DC_CSC_MULT_BU, reg_read(dctrl, DC_CSC_MULT_BU));
	printk("DC_CSC_SUB_YUV(0x%04x):     0x%08lx\n", DC_CSC_SUB_YUV, reg_read(dctrl, DC_CSC_SUB_YUV));
	printk("DC_ST(0x%04x):              0x%08lx\n", DC_ST, reg_read(dctrl, DC_ST));
	printk("DC_INTC(0x%04x):            0x%08lx\n", DC_INTC, reg_read(dctrl, DC_INTC));
	printk("DC_INT_FLAG(0x%04x):	   0x%08lx\n", DC_INT_FLAG, reg_read(dctrl, DC_INT_FLAG));
	printk("DC_COM_CONFIG(0x%04x):      0x%08lx\n", DC_COM_CONFIG, reg_read(dctrl, DC_COM_CONFIG));
	printk("DC_TLB_GLBC(0x%04x):        0x%08lx\n", DC_TLB_GLBC, reg_read(dctrl, DC_TLB_GLBC));
	printk("DC_TLB_TLBA(0x%04x):        0x%08lx\n", DC_TLB_TLBA, reg_read(dctrl, DC_TLB_TLBA));
	printk("DC_TLB_TLBC(0x%04x):        0x%08lx\n", DC_TLB_TLBC, reg_read(dctrl, DC_TLB_TLBC));
	printk("DC_TLB0_VPN(0x%04x):        0x%08lx\n", DC_TLB0_VPN, reg_read(dctrl, DC_TLB0_VPN));
	printk("DC_TLB1_VPN(0x%04x):        0x%08lx\n", DC_TLB1_VPN, reg_read(dctrl, DC_TLB1_VPN));
	printk("DC_TLB2_VPN(0x%04x):        0x%08lx\n", DC_TLB2_VPN, reg_read(dctrl, DC_TLB2_VPN));
	printk("DC_TLB3_VPN(0x%04x):        0x%08lx\n", DC_TLB3_VPN, reg_read(dctrl, DC_TLB3_VPN));
	printk("DC_TLB_TLBV(0x%04x):        0x%08lx\n", DC_TLB_TLBV, reg_read(dctrl, DC_TLB_TLBV));
	printk("DC_TLB_STAT(0x%04x):        0x%08lx\n", DC_TLB_STAT, reg_read(dctrl, DC_TLB_STAT));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL, reg_read(dctrl, DC_PCFG_RD_CTRL));
	printk("DC_PCFG_WR_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_WR_CTRL, reg_read(dctrl, DC_PCFG_WR_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG, reg_read(dctrl, DC_OFIFO_PCFG));
	printk("DC_WDMA_PCFG(0x%04x):	   0x%08lx\n", DC_WDMA_PCFG, reg_read(dctrl, DC_WDMA_PCFG));
	printk("DC_CMPW_PCFG_CTRL(0x%04x): 0x%08lx\n", DC_CMPW_PCFG_CTRL, reg_read(dctrl, DC_CMPW_PCFG_CTRL));
	printk("DC_CMPW_PCFG0(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG0, reg_read(dctrl, DC_CMPW_PCFG0));
	printk("DC_CMPW_PCFG1(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG1, reg_read(dctrl, DC_CMPW_PCFG1));
	printk("DC_CMPW_PCFG2(0x%04x):	   0x%08lx\n", DC_CMPW_PCFG2, reg_read(dctrl, DC_CMPW_PCFG2));
	printk("DC_PCFG_RD_CTRL(0x%04x):    0x%08lx\n", DC_PCFG_RD_CTRL, reg_read(dctrl, DC_PCFG_RD_CTRL));
	printk("DC_OFIFO_PCFG(0x%04x):	   0x%08lx\n", DC_OFIFO_PCFG, reg_read(dctrl, DC_OFIFO_PCFG));
	printk("DC_DISP_COM(0x%04x):        0x%08lx\n", DC_DISP_COM, reg_read(dctrl, DC_DISP_COM));
	printk("-----------------dc_reg------------------\n");
}

static void dump_tft_reg(struct dpu_ctrl *dctrl)
{
	printk("----------------tft_reg------------------\n");
	printk("TFT_TIMING_HSYNC(0x%04x):   0x%08lx\n", DC_TFT_HSYNC, reg_read(dctrl, DC_TFT_HSYNC));
	printk("TFT_TIMING_VSYNC(0x%04x):   0x%08lx\n", DC_TFT_VSYNC, reg_read(dctrl, DC_TFT_VSYNC));
	printk("TFT_TIMING_HDE(0x%04x):     0x%08lx\n", DC_TFT_HDE, reg_read(dctrl, DC_TFT_HDE));
	printk("TFT_TIMING_VDE(0x%04x):     0x%08lx\n", DC_TFT_VDE, reg_read(dctrl, DC_TFT_VDE));
	printk("TFT_TRAN_CFG(0x%04x):       0x%08lx\n", DC_TFT_CFG, reg_read(dctrl, DC_TFT_CFG));
	printk("TFT_ST(0x%04x):             0x%08lx\n", DC_TFT_ST, reg_read(dctrl, DC_TFT_ST));
	printk("----------------tft_reg------------------\n");
}

static void dump_slcd_reg(struct dpu_ctrl *dctrl)
{
	printk("---------------slcd_reg------------------\n");
	printk("SLCD_CFG(0x%04x):           0x%08lx\n", DC_SLCD_CFG, reg_read(dctrl, DC_SLCD_CFG));
	printk("SLCD_WR_DUTY(0x%04x):       0x%08lx\n", DC_SLCD_WR_DUTY, reg_read(dctrl, DC_SLCD_WR_DUTY));
	printk("SLCD_TIMING(0x%04x):        0x%08lx\n", DC_SLCD_TIMING, reg_read(dctrl, DC_SLCD_TIMING));
	printk("SLCD_FRM_SIZE(0x%04x):      0x%08lx\n", DC_SLCD_FRM_SIZE, reg_read(dctrl, DC_SLCD_FRM_SIZE));
	printk("SLCD_SLOW_TIME(0x%04x):     0x%08lx\n", DC_SLCD_SLOW_TIME, reg_read(dctrl, DC_SLCD_SLOW_TIME));
	printk("SLCD_REG_IF(0x%04x):	    0x%08lx\n", DC_SLCD_REG_IF, reg_read(dctrl, DC_SLCD_REG_IF));
	printk("SLCD_ST(0x%04x):            0x%08lx\n", DC_SLCD_ST, reg_read(dctrl, DC_SLCD_ST));
	printk("---------------slcd_reg------------------\n");
}

static void dump_frm_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(dctrl, DC_CTRL, ctrl);

	printk("--------Frame Descriptor register--------\n");
	printk("FrameNextCfgAddr:   %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("FrameSize:          %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("FrameCtrl:          %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("WritebackAddr:      %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("WritebackStride:    %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("Layer0CfgAddr:      %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("Layer1CfgAddr:      %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("Layer2CfgAddr:      %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("Layer3CfgAddr:      %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("LayCfgEn:	    %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("InterruptControl:   %lx\n", reg_read(dctrl, DC_FRM_DES));
	printk("--------Frame Descriptor register--------\n");
}

static void dump_layer_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= DC_DES_CNT_RST;
	reg_write(dctrl, DC_CTRL, ctrl);

	printk("--------layer0 Descriptor register-------\n");
	printk("LayerSize:          %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerCfg:           %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerBufferAddr:    %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerScale:         %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerRotation:      %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerScratch:       %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerPos:           %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerResizeCoef_X:  %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerResizeCoef_Y:  %lx\n", reg_read(dctrl, DC_LAY0_DES));
	printk("LayerStride:        %lx\n", reg_read(dctrl, DC_LAY0_DES));
	//  printk("Layer_UV_addr:      %lx\n",reg_read(dctrl, DC_LAY0_DES));
	//  printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY0_DES));
	printk("--------layer0 Descriptor register-------\n");

	printk("--------layer1 Descriptor register-------\n");
	printk("LayerSize:          %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerCfg:           %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerBufferAddr:    %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerScale:         %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerRotation:      %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerScratch:       %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerPos:           %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerResizeCoef_X:  %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerResizeCoef_Y:  %lx\n", reg_read(dctrl, DC_LAY1_DES));
	printk("LayerStride:        %lx\n", reg_read(dctrl, DC_LAY1_DES));
	//  printk("Layer_UV_addr:      %lx\n",reg_read(dctrl, DC_LAY1_DES));
	//  printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY1_DES));
	printk("--------layer1 Descriptor register-------\n");

	printk("--------layer2 Descriptor register-------\n");
	printk("LayerSize:          %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerCfg:           %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerBufferAddr:    %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerScale:         %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerRotation:      %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerScratch:       %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerPos:           %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerResizeCoef_X:  %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerResizeCoef_Y:  %lx\n", reg_read(dctrl, DC_LAY2_DES));
	printk("LayerStride:        %lx\n", reg_read(dctrl, DC_LAY2_DES));
	//  printk("Layer_UV_addr:      %lx\n",reg_read(dctrl, DC_LAY2_DES));
	//  printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY2_DES));
	printk("--------layer2 Descriptor register-------\n");

	printk("--------layer3 Descriptor register-------\n");
	printk("LayerSize:          %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerCfg:           %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerBufferAddr:    %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerScale:         %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerRotation:      %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerScratch:       %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerPos:           %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerResizeCoef_X:  %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerResizeCoef_Y:  %lx\n", reg_read(dctrl, DC_LAY3_DES));
	printk("LayerStride:        %lx\n", reg_read(dctrl, DC_LAY3_DES));
	//  printk("Layer_UV_addr:      %lx\n",reg_read(dctrl, DC_LAY3_DES));
	//  printk("Layer_UV_stride:    %lx\n",reg_read(dctrl, DC_LAY3_DES));
	printk("--------layer3 Descriptor register-------\n");
}

static void dump_rdma_desc_reg(struct dpu_ctrl *dctrl)
{
	unsigned int ctrl;
	ctrl = reg_read(dctrl, DC_CTRL);
	ctrl |= (1 << 2);
	reg_write(dctrl, DC_CTRL, ctrl);
	printk("====================rdma Descriptor register======================\n");
	printk("RdmaNextCfgAddr:    %lx\n", reg_read(dctrl, DC_RDMA_DES));
	printk("FrameBufferAddr:    %lx\n", reg_read(dctrl, DC_RDMA_DES));
	printk("Stride:             %lx\n", reg_read(dctrl, DC_RDMA_DES));
	printk("ChainCfg:           %lx\n", reg_read(dctrl, DC_RDMA_DES));
	printk("InterruptControl:   %lx\n", reg_read(dctrl, DC_RDMA_DES));
	printk("==================rdma Descriptor register end======================\n");
}

void dump_lcdc_registers(struct dpu_ctrl *dctrl)
{
	dump_dc_reg(dctrl);
	dump_tft_reg(dctrl);
	dump_slcd_reg(dctrl);
	if (dctrl->support_comp) {
		dump_frm_desc_reg(dctrl);
		dump_layer_desc_reg(dctrl);
	}
	dump_rdma_desc_reg(dctrl);
}
#endif
static unsigned int _format_depth_in_bytes(unsigned int format)
{
	unsigned int bytes = 1;
	switch (format) {
		case LAYER_CFG_FORMAT_NV12:
		case LAYER_CFG_FORMAT_NV21:
		case LAYER_CFG_FORMAT_MONO8:
			bytes = 1;
			break;

		case LAYER_CFG_FORMAT_RGB555:
		case LAYER_CFG_FORMAT_ARGB1555:
		case LAYER_CFG_FORMAT_RGB565:
		case LAYER_CFG_FORMAT_MONO16:
		case LAYER_CFG_FORMAT_YUV422:
			bytes = 2;
			break;

		case LAYER_CFG_FORMAT_RGB888:
		case LAYER_CFG_FORMAT_ARGB8888:
			bytes = 4;
			break;
		default:
			break;
	}

	return bytes;
}

static unsigned int _stride_in_bytes(unsigned int stride_in_pixel, unsigned int format)
{
	return stride_in_pixel * _format_depth_in_bytes(format);
}

static unsigned int list_len(struct list_head *head)
{
	unsigned int count = 0;
	struct list_head *pos;

	list_for_each(pos, head) {
		count++;
	}

	return count;
}
static int _dpu_eod_int_handler(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;

	if ((dctrl->wback_end == 1 && comp_info->out_mode == COMP_WRITE_BACK_2_ROTATOR) || comp_info->out_mode == COMP_DIRECT_OUT) {
		_do_comp_start(dctrl);
	} else {
		printk("composer is working while new comp requested!, dctrl->wback_end: %d\n", dctrl->wback_end);
	}

	if (comp_info->out_mode == COMP_DIRECT_OUT) {

		unsigned int next_hw_framedesc = reg_read(dctrl, DC_FRM_CFG_ADDR);  // 下一个要搬的描述符，需要想办法记录当前正在搬运的描述符。
		struct ingenicfb_framedesc *next_framedesc = (struct ingenicfb_framedesc *)KSEG1ADDR(next_hw_framedesc);
		while (1) {
			struct ingenicfb_framedesc *desc_finished = NULL;
			// 5. 判断desc_finished 与当前硬件正在传输的描述符是否相等。
			desc_finished = list_first_entry_or_null(&dctrl->comp_desc_queued_list, struct ingenicfb_framedesc, list);

			//printk("----desc_finished: %x, next_framedesc: %x\n", desc_finished, next_framedesc);
			if (desc_finished && desc_finished->phy_addr != next_framedesc->phy_addr) {
				//                  printk("--------------done: %d, --phy_addr: %x, curr_addr: %x, list_len: %d\n", desc_finished->index, desc_finished->phy_addr, reg_read(dctrl, DC_FRM_CHAIN_SITE), list_len(&dctrl->comp_desc_queued_list));
				list_del(&desc_finished->list);
				list_add_tail(&desc_finished->list, &dctrl->comp_desc_done_list);
			} else {
				break;
			}
		}
	}

	return 0;
}

/* END of composer handler*/
static int _dpu_eoc_int_handler(struct dpu_ctrl *dctrl)
{
#if 0
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	// Do Composer restart.

	if (comp_info->out_mode != COMP_DIRECT_OUT) {
		/* 如果是非直通模式，也就是写回模式，在此不做处理. */
		return 0;
	}


	dctrl->vblank_count++;

	if (dctrl->comp_info_changed) {
		//      dctrl->comp_hw_framedesc = reg_read(dctrl, DC_FRM_CHAIN_SITE);
		//      if(dctrl->comp_hw_framedesc != dctrl->framedesc_phys[dctrl->comp_next_frame]) {
		_do_comp_start(dctrl);

		//      } else {
		printk("composer is working while new comp requested!\n");
		//      }
	}

#endif
	return 0;
}


/*
    Start of Composer interrupt handler.

    do:
      fetch next frame desc to start. if new compose request.

    int direct out mode.

*/
static int _dpu_soc_int_handler(struct dpu_ctrl *dctrl)
{
	//_dpu_eod_int_handler(dctrl);
#if 0
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	// Do Composer restart.

	if (comp_info->out_mode != COMP_DIRECT_OUT) {
		/* 如果是非直通模式，也就是写回模式，在此不做处理. */
		return 0;
	}

	// Do Composer restart.
	dctrl->vblank_count++;

	if (dctrl->comp_info_changed) {
		dctrl->comp_hw_framedesc = reg_read(dctrl, DC_FRM_CHAIN_SITE);
		//      if(dctrl->comp_hw_framedesc != dctrl->framedesc_phys[dctrl->comp_next_frame]) {
		_do_comp_start(dctrl);

#if 0
		if (dctrl->comp_hw_framedesc == dctrl->framedesc_phys[dctrl->comp_next_frame]) {
			//printk("----using next frame desc: %d\n", dctrl->comp_hw_framedesc);
			dctrl->comp_next_frame = (dctrl->comp_next_frame + 1) % DPU_COMP_MAX_FRAMEDESC;
		}
#endif

		//      } else {
		//          printk("composer is working while new comp requested!\n");
		//      }
	}
#endif
	return 0;

}

static int _dpu_eow_int_handler(struct dpu_ctrl *dctrl)
{

#ifdef CONFIG_SOC_X2600
	if (dctrl->rot_cfg.rot_angle != 0) {

		if (dctrl->rot_cfg.rot_manual_auto == 0) {
			rot_manual_start(dctrl);
		}
		dctrl->rot_cfg.next_wbbuf += 1;
		dctrl->rot_cfg.next_wbbuf %= CONFIG_ROT_INGENIC_BUFS;

	} else {
		/* setup rdma*/
#ifdef TEST_COMPOSER_2_RDMA
		dpu_ctrl_rdma_change(dctrl, dctrl->rdma_next_frame);
#endif

	}
#endif

	return 0;
}

static irqreturn_t dpu_ctrl_irq_handler(int irq, void *data)
{
	unsigned int irq_flag;
	struct dpu_ctrl *dctrl = (struct dpu_ctrl *)data;
	spin_lock(&dctrl->irq_lock);

	dbg_irqcnt_inc(dctrl->dbg_irqcnt, irqcnt);

	irq_flag = reg_read(dctrl, DC_INT_FLAG);
	if (likely(irq_flag & DC_CMP_START)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_START);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, cmp_start);
		//TODO:
		//ingenicfb_set_vsync_value(dctrl);
		//dctrl->set_vsync_value(dctrl->vsync_data);
		dctrl->comp_hw_framedesc = reg_read(dctrl, DC_FRM_CHAIN_SITE);
		_dpu_soc_int_handler(dctrl);
		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
#define IRQ_P_N 50

	if (unlikely(irq_flag & DC_STOP_CMP_ACK)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_STOP_CMP_ACK);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, stop_disp_ack);

		dctrl->comp_stopped = 1;
		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	if (unlikely(irq_flag & DC_DISP_END)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_DISP_END);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, disp_end);
		//dev_dbg(dctrl->dev, "DC_DISP_END");
		//dctrl->dsi->master_ops->query_te(dctrl->dsi);
		if (!dctrl->work_in_rdma) {
			if (dctrl->chan != DATA_CH_RDMA || dctrl->comp_info.out_mode == COMP_WRITE_BACK_2_RDMA) {
				_dpu_eod_int_handler(dctrl);
			}
		}
		dctrl->vblank_count++;
		wake_up_interruptible(&dctrl->wq);

		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}


	if (unlikely(irq_flag & DC_TFT_UNDR)) {
		reg_write(dctrl, DC_CLR_ST, DC_CLR_TFT_UNDR);
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, tft_under);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	if (likely(irq_flag & DC_WDMA_OVER)) {
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, wdma_over);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_WDMA_OVER);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_WDMA_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, wdma_end);

		dctrl->wback_end = 1;
		dctrl->comp_stopped = 1;
		if (!dctrl->work_in_rdma) {
			_dpu_eow_int_handler(dctrl);
		}
		reg_write(dctrl, DC_CLR_ST, DC_CLR_WDMA_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_WDMA_END irq came here!!!!!!!!!!!!!!");
		}

		wake_up_interruptible(&dctrl->wq);
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_LAY3_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer3_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY3_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_LAY3_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_LAY2_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer2_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY2_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_LAY2_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_LAY1_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer1_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY1_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_LAY1_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_LAY0_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, layer0_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_LAY0_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_LAY0_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_CMP_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, clr_cmp_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_END);

		_dpu_eoc_int_handler(dctrl);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_CMP_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_STOP_SRD_ACK)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, stop_wrbk_ack);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_STOP_SRD_ACK);
		dctrl->srd_stopped = 1;
		wake_up_interruptible(&dctrl->wq);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_STOP_SRD irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_SRD_START)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, srd_start);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_SRD_START);

#ifdef TEST_COMPOSER_2_RDMA
		if (dctrl->rdma_changed) {
			dctrl->rdma_changed = 0;
			dctrl->rdma_next_frame += 1;
			dctrl->rdma_next_frame %= dctrl->wback_rdma_info.nframes;
		}
#endif


#if 0
		if (dctrl->rdma_desc_changing) {
			dctrl->rdma_desc_changing = 0;
			/*SRD_START
			 *
			 * 1. descriptor is done loading, it's save to  update rdma_last_frame descriptor.
			 *
			 *  [rdma_last_frame] --     [current_refresh_frame]
			 *      ^      |
			 *      |----------|
			 * */
			dctrl->sreadesc[dctrl->rdma_last_frame]->RdmaNextCfgAddr = dctrl->sreadesc_phys[dctrl->rdma_last_frame];
			dctrl->rdma_last_frame = dctrl->rdma_changing_frame;
		}
#endif
		dctrl->set_vsync_value(dctrl->vsync_data);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_SRD_START irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_SRD_END)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, srd_end);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_SRD_END);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_SRD_END irq came here!!!!!!!!!!!!!!");
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}
	if (likely(irq_flag & DC_CMP_W_SLOW)) {
		static int cnt = 0;
		cnt++;
		dbg_irqcnt_inc(dctrl->dbg_irqcnt, cmp_w_slow);
		reg_write(dctrl, DC_CLR_ST, DC_CLR_CMP_W_SLOW);
		if (cnt < IRQ_P_N) {
			dev_dbg(dctrl->dev, "DC_CMP_W_SLOW came here!!!!!!!!!!!!!! DC_ST = 0x%lx cnt = %d", reg_read(dctrl, DC_ST), cnt);
		}
		if (cnt > 10) {
			reg_write(dctrl, DC_CMPW_PCFG_CTRL, 0 << 10 | 30);
		}
		spin_unlock(&dctrl->irq_lock);
		return IRQ_HANDLED;
	}

	dev_err(dctrl->dev, "DPU irq nothing do, please check!!! DC_ST = 0x%lx\n", reg_read(dctrl, DC_ST));
	spin_unlock(&dctrl->irq_lock);
	return IRQ_HANDLED;
}
#ifdef CONFIG_MMU_NOTIFIER
static void dctrl_tlb_invalidate_ch(struct dpu_ctrl *dctrl, int ch)
{
	int val;

	val = ch & 0xF;
	/*Invalid for ch layers.*/
	reg_write(dctrl, DC_TLB_TLBC, val);
}

static void dctrl_tlb_enable(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	unsigned int glbc, val;
	int i;

	lay_cfg = frm_cfg->lay_cfg;

	val = reg_read(dctrl, DC_TLB_GLBC);
	glbc = val;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (lay_cfg[i].lay_en) {
			if (lay_cfg[i].tlb_en != (glbc >> i & 0x1)) {
				glbc = lay_cfg[i].tlb_en ?
				       (glbc | (0x1 << i)) : (glbc & ~(0x1 << i));
			}
		} else {
			glbc &= ~(0x1 << i);
		}
	}
	if (val != glbc) {
		dctrl_tlb_invalidate_ch(dctrl, glbc);   /*invalidate tlb*/
		if (val > glbc) {           /*When a TLB channel is disable, val is greater than glbc*/
			dctrl->tlb_disable = true;  /*The TLB should be disable after the current descriptor execution completes*/
			dctrl->tlb_disable_ch = glbc;
		} else {
			reg_write(dctrl, DC_TLB_GLBC, glbc);
		}
	}
}

static void dctrl_tlb_configure(struct dpu_ctrl *dctrl)
{
	unsigned int tlbv = 0;

	tlbv |= (1 << DC_CNM_LBIT);
	tlbv |= (1 << DC_GCN_LBIT);

	/*mutex_lock(dctrl->lock);*/
	reg_write(dctrl, DC_TLB_TLBV, tlbv);
	reg_write(dctrl, DC_TLB_TLBA, dctrl->tlba);
#if 0
	reg_write(dctrl, DC_TLB_TLBC, DC_CH0_INVLD | DC_CH1_INVLD |
	          DC_CH2_INVLD | DC_CH3_INVLD);
#endif
	/*mutex_unlock(dctrl->lock);*/
}

static void dctrl_tlb_disable(struct dpu_ctrl *dctrl)
{
	/*Invalid and disable for all layers.*/
	reg_write(dctrl, DC_TLB_TLBC, DC_CH0_INVLD | DC_CH1_INVLD |
	          DC_CH2_INVLD | DC_CH3_INVLD);
	reg_write(dctrl, DC_TLB_GLBC, 0);
}
#endif

static void csc_mode_set(struct dpu_ctrl *dctrl, csc_mode_t mode)
{
	switch (mode) {
		case CSC_MODE_0:
			reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD0 | DC_CSC_MULT_RV_MD0);
			reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD0 | DC_CSC_MULT_GV_MD0);
			reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD0);
			reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD0 | DC_CSC_SUB_UV_MD0);
			break;
		case CSC_MODE_1:
			reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD1 | DC_CSC_MULT_RV_MD1);
			reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD1 | DC_CSC_MULT_GV_MD1);
			reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD1);
			reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD1 | DC_CSC_SUB_UV_MD1);
			break;
		case CSC_MODE_2:
			reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD2 | DC_CSC_MULT_RV_MD2);
			reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD2 | DC_CSC_MULT_GV_MD2);
			reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD2);
			reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD2 | DC_CSC_SUB_UV_MD2);
			break;
		case CSC_MODE_3:
			reg_write(dctrl, DC_CSC_MULT_YRV, DC_CSC_MULT_Y_MD3 | DC_CSC_MULT_RV_MD3);
			reg_write(dctrl, DC_CSC_MULT_GUGV, DC_CSC_MULT_GU_MD3 | DC_CSC_MULT_GV_MD3);
			reg_write(dctrl, DC_CSC_MULT_BU, DC_CSC_MULT_BU_MD3);
			reg_write(dctrl, DC_CSC_SUB_YUV, DC_CSC_SUB_Y_MD3 | DC_CSC_SUB_UV_MD3);
			break;
		default:
			dev_err(dctrl->dev, "Set csc mode err!\n");
			break;
	}
}

static void slcd_send_mcu_command(struct dpu_ctrl *dctrl, unsigned long cmd)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg & ~DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_CMD | (cmd & ~DC_SLCD_REG_IF_FLAG_MASK));
}

static void slcd_send_mcu_data(struct dpu_ctrl *dctrl, unsigned long data)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg | DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_DATA | (data & ~DC_SLCD_REG_IF_FLAG_MASK));
}

static void slcd_send_mcu_prm(struct dpu_ctrl *dctrl, unsigned long data)
{
	int count = 10000;
	uint32_t slcd_cfg;

	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);
	reg_write(dctrl, DC_SLCD_CFG, (slcd_cfg & ~DC_FMT_EN));
	reg_write(dctrl, DC_SLCD_REG_IF, DC_SLCD_REG_IF_FLAG_PRM | (data & ~DC_SLCD_REG_IF_FLAG_MASK));
}

__attribute__((__unused__)) static void wait_slcd_busy(struct dpu_ctrl *dctrl)
{
	int count = 100000;
	while ((reg_read(dctrl, DC_SLCD_ST) & DC_SLCD_ST_BUSY)
	       && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dctrl->dev, "SLCDC wait busy state wrong");
	}
}

static int wait_dc_state(struct dpu_ctrl *dctrl, uint32_t state, uint32_t flag)
{
	unsigned long timeout = 20000;
	while (((!(reg_read(dctrl, DC_ST) & state)) == flag) && timeout) {
		timeout--;
		udelay(10);
	}
	if (timeout <= 0) {
		printk("LCD wait state timeout! state = 0x%08x, DC_ST = 0x%08lx\n", state, reg_read(dctrl, DC_ST));
		return -1;
	}

	return 0;
}

static void ingenicfb_slcd_mcu_init(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct smart_config *smart_config;
	struct smart_lcd_data_table *data_table;
	uint32_t length_data_table;
	uint32_t i;

	smart_config = panel->smart_config;
	if (panel->lcd_type != LCD_TYPE_SLCD || smart_config == NULL) {
		return;
	}

	data_table = smart_config->data_table;
	length_data_table = smart_config->length_data_table;
	if (length_data_table && data_table) {
		for (i = 0; i < length_data_table; i++) {
			switch (data_table[i].type) {
				case SMART_CONFIG_DATA:
					slcd_send_mcu_data(dctrl, data_table[i].value);
					break;
				case SMART_CONFIG_PRM:
					slcd_send_mcu_prm(dctrl, data_table[i].value);
					break;
				case SMART_CONFIG_CMD:
					slcd_send_mcu_command(dctrl, data_table[i].value);
					break;
				case SMART_CONFIG_UDELAY:
					udelay(data_table[i].value);
					break;
				default:
					printk("Unknow SLCD data type\n");
					break;
			}
		}
	}
}

static void tft_timing_init(struct dpu_ctrl *dctrl, struct fb_videomode *modes)
{
	uint32_t hps;
	uint32_t hpe;
	uint32_t vps;
	uint32_t vpe;
	uint32_t hds;
	uint32_t hde;
	uint32_t vds;
	uint32_t vde;

	hps = modes->hsync_len;
	hpe = hps + modes->left_margin + modes->xres + modes->right_margin;
	vps = modes->vsync_len;
	vpe = vps + modes->upper_margin + modes->yres + modes->lower_margin;

	hds = modes->hsync_len + modes->left_margin;
	hde = hds + modes->xres;
	vds = modes->vsync_len + modes->upper_margin;
	vde = vds + modes->yres;

	reg_write(dctrl, DC_TFT_HSYNC,
	          (hps << DC_HPS_LBIT) |
	          (hpe << DC_HPE_LBIT));
	reg_write(dctrl, DC_TFT_VSYNC,
	          (vps << DC_VPS_LBIT) |
	          (vpe << DC_VPE_LBIT));
	reg_write(dctrl, DC_TFT_HDE,
	          (hds << DC_HDS_LBIT) |
	          (hde << DC_HDE_LBIT));
	reg_write(dctrl, DC_TFT_VDE,
	          (vds << DC_VDS_LBIT) |
	          (vde << DC_VDE_LBIT));
}

void tft_cfg_init(struct dpu_ctrl *dctrl, struct tft_config *tft_config)
{
	uint32_t tft_cfg;
	uint32_t lcd_cgu;

	lcd_cgu = *(volatile unsigned int *)(0xb0000064);
	if (tft_config->pix_clk_inv) {
		lcd_cgu |= (0x1 << 26);
	} else {
		lcd_cgu &= ~(0x1 << 26);
	}
	*(volatile unsigned int *)(0xb0000064) = lcd_cgu;

	tft_cfg = reg_read(dctrl, DC_TFT_CFG);
	if (tft_config->de_dl) {
		tft_cfg |= DC_DE_DL;
	} else {
		tft_cfg &= ~DC_DE_DL;
	}

	if (tft_config->sync_dl) {
		tft_cfg |= DC_SYNC_DL;
	} else {
		tft_cfg &= ~DC_SYNC_DL;
	}
	if (tft_config->vsync_dl) {
		tft_cfg |= DC_VSYNC_DL;
	} else {
		tft_cfg &= ~DC_VSYNC_DL;
	}

	tft_cfg &= ~DC_COLOR_EVEN_MASK;
	switch (tft_config->color_even) {
		case TFT_LCD_COLOR_EVEN_RGB:
			tft_cfg |= DC_EVEN_RGB;
			break;
		case TFT_LCD_COLOR_EVEN_RBG:
			tft_cfg |= DC_EVEN_RBG;
			break;
		case TFT_LCD_COLOR_EVEN_BGR:
			tft_cfg |= DC_EVEN_BGR;
			break;
		case TFT_LCD_COLOR_EVEN_BRG:
			tft_cfg |= DC_EVEN_BRG;
			break;
		case TFT_LCD_COLOR_EVEN_GBR:
			tft_cfg |= DC_EVEN_GBR;
			break;
		case TFT_LCD_COLOR_EVEN_GRB:
			tft_cfg |= DC_EVEN_GRB;
			break;
		default:
			printk("err!\n");
			break;
	}

	tft_cfg &= ~DC_COLOR_ODD_MASK;
	switch (tft_config->color_odd) {
		case TFT_LCD_COLOR_ODD_RGB:
			tft_cfg |= DC_ODD_RGB;
			break;
		case TFT_LCD_COLOR_ODD_RBG:
			tft_cfg |= DC_ODD_RBG;
			break;
		case TFT_LCD_COLOR_ODD_BGR:
			tft_cfg |= DC_ODD_BGR;
			break;
		case TFT_LCD_COLOR_ODD_BRG:
			tft_cfg |= DC_ODD_BRG;
			break;
		case TFT_LCD_COLOR_ODD_GBR:
			tft_cfg |= DC_ODD_GBR;
			break;
		case TFT_LCD_COLOR_ODD_GRB:
			tft_cfg |= DC_ODD_GRB;
			break;
		default:
			printk("err!\n");
			break;
	}

	tft_cfg &= ~DC_MODE_MASK;
	switch (tft_config->mode) {
		case TFT_LCD_MODE_PARALLEL_888:
			tft_cfg |= DC_MODE_PARALLEL_888;
			break;
		case TFT_LCD_MODE_PARALLEL_666:
			tft_cfg |= DC_MODE_PARALLEL_666;
			break;
		case TFT_LCD_MODE_PARALLEL_565:
			tft_cfg |= DC_MODE_PARALLEL_565;
			break;
		case TFT_LCD_MODE_SERIAL_RGB:
			tft_cfg |= DC_MODE_SERIAL_8BIT_RGB;
			break;
		case TFT_LCD_MODE_SERIAL_RGBD:
			tft_cfg |= DC_MODE_SERIAL_8BIT_RGBD;
			break;
		default:
			printk("err!\n");
			break;
	}
	reg_write(dctrl, DC_TFT_CFG, tft_cfg);
}

static int ingenicfb_tft_set_par(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct lcd_panel_ops *panel_ops;
	struct fb_videomode *mode = dctrl->active_video_mode;

	panel_ops = panel->ops;

	tft_timing_init(dctrl, mode);
	tft_cfg_init(dctrl, panel->tft_config);
	if (panel_ops && panel_ops->enable) {
		panel_ops->enable(panel);
	}

	return 0;
}

static void slcd_cfg_init(struct dpu_ctrl *dctrl,
                          struct smart_config *smart_config)
{
	uint32_t slcd_cfg;

	if (smart_config == NULL) {
		dev_info(dctrl->dev, "SLCD use default config\n");
		return;
	}

	slcd_cfg = reg_read(dctrl, DC_SLCD_CFG);

	if (smart_config->rdy_switch) {
		slcd_cfg |= DC_RDY_SWITCH;
	} else {
		slcd_cfg &= ~DC_RDY_SWITCH;
	}

	if (smart_config->cs_en) {
		slcd_cfg |= DC_CS_EN;
	} else {
		slcd_cfg &= ~DC_CS_EN;
	}

	if (smart_config->te_switch) {
		slcd_cfg |= DC_TE_SWITCH;

		if (smart_config->te_dp) {
			slcd_cfg |= DC_TE_DP;
		} else {
			slcd_cfg &= ~DC_TE_DP;
		}
		if (smart_config->te_md) {
			slcd_cfg |= DC_TE_MD;
		} else {
			slcd_cfg &= ~DC_TE_MD;
		}
		if (smart_config->te_anti_jit) {
			slcd_cfg |= DC_TE_ANTI_JIT;
		} else {
			slcd_cfg &= ~DC_TE_ANTI_JIT;
		}
	} else {
		slcd_cfg &= ~DC_TE_SWITCH;
	}

	if (smart_config->te_mipi_switch) {
		slcd_cfg |= DC_TE_MIPI_SWITCH;
	} else {
		slcd_cfg &= ~DC_TE_MIPI_SWITCH;
	}

	if (smart_config->dc_md) {
		slcd_cfg |= DC_DC_MD;
	} else {
		slcd_cfg &= ~DC_DC_MD;
	}

	if (smart_config->wr_md) {
		slcd_cfg |= DC_WR_DP;
	} else {
		slcd_cfg &= ~DC_WR_DP;
	}

	slcd_cfg &= ~DC_DBI_TYPE_MASK;
	switch (smart_config->smart_type) {
		case SMART_LCD_TYPE_8080:
			slcd_cfg |= DC_DBI_TYPE_B_8080;
			break;
		case SMART_LCD_TYPE_6800:
			slcd_cfg |= DC_DBI_TYPE_A_6800;
			break;
		case SMART_LCD_TYPE_SPI_3:
			slcd_cfg |= DC_DBI_TYPE_C_SPI_3;
			break;
		case SMART_LCD_TYPE_SPI_4:
			slcd_cfg |= DC_DBI_TYPE_C_SPI_4;
			break;
		default:
			printk("err!\n");
			break;
	}

	slcd_cfg &= ~DC_DATA_FMT_MASK;
	switch (smart_config->pix_fmt) {
		case SMART_LCD_FORMAT_888:
			slcd_cfg |= DC_DATA_FMT_888;
			break;
		case SMART_LCD_FORMAT_666:
			slcd_cfg |= DC_DATA_FMT_666;
			break;
		case SMART_LCD_FORMAT_565:
			slcd_cfg |= DC_DATA_FMT_565;
			break;
		default:
			printk("err!\n");
			break;
	}

	slcd_cfg &= ~DC_DWIDTH_MASK;
	switch (smart_config->dwidth) {
		case SMART_LCD_DWIDTH_8_BIT:
			slcd_cfg |= DC_DWIDTH_8BITS;
			break;
		case SMART_LCD_DWIDTH_9_BIT:
			slcd_cfg |= DC_DWIDTH_9BITS;
			break;
		case SMART_LCD_DWIDTH_16_BIT:
			slcd_cfg |= DC_DWIDTH_16BITS;
			break;
		case SMART_LCD_DWIDTH_18_BIT:
			slcd_cfg |= DC_DWIDTH_18BITS;
			break;
		case SMART_LCD_DWIDTH_24_BIT:
			slcd_cfg |= DC_DWIDTH_24BITS;
			break;
		default:
			printk("err!\n");
			break;
	}

	slcd_cfg &= ~DC_CWIDTH_MASK;
	switch (smart_config->cwidth) {
		case SMART_LCD_CWIDTH_8_BIT:
			slcd_cfg |= DC_CWIDTH_8BITS;
			break;
		case SMART_LCD_CWIDTH_9_BIT:
			slcd_cfg |= DC_CWIDTH_9BITS;
			break;
		case SMART_LCD_CWIDTH_16_BIT:
			slcd_cfg |= DC_CWIDTH_16BITS;
			break;
		case SMART_LCD_CWIDTH_18_BIT:
			slcd_cfg |= DC_CWIDTH_18BITS;
			break;
		case SMART_LCD_CWIDTH_24_BIT:
			slcd_cfg |= DC_CWIDTH_24BITS;
			break;
		default:
			printk("err!\n");
			break;
	}

	reg_write(dctrl, DC_SLCD_CFG, slcd_cfg);

	return;
}

static int slcd_timing_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel,
                            struct fb_videomode *mode)
{
	uint32_t width = mode->xres;
	uint32_t height = mode->yres;
	uint32_t dhtime = 0;
	uint32_t dltime = 0;
	uint32_t chtime = 0;
	uint32_t cltime = 0;
	uint32_t tah = 0;
	uint32_t tas = 0;
	uint32_t slowtime = 0;

	/*frm_size*/
	reg_write(dctrl, DC_SLCD_FRM_SIZE,
	          ((width << DC_SLCD_FRM_H_SIZE_LBIT) |
	           (height << DC_SLCD_FRM_V_SIZE_LBIT)));

	/* wr duty */
	reg_write(dctrl, DC_SLCD_WR_DUTY,
	          ((dhtime << DC_DSTIME_LBIT) |
	           (dltime << DC_DDTIME_LBIT) |
	           (chtime << DC_CSTIME_LBIT) |
	           (cltime << DC_CDTIME_LBIT)));

	/* slcd timing */
	reg_write(dctrl, DC_SLCD_TIMING,
	          ((tah << DC_TAH_LBIT) |
	           (tas << DC_TAS_LBIT)));

	/* slow time */
	reg_write(dctrl, DC_SLCD_SLOW_TIME, slowtime);

	return 0;
}

static int ingenicfb_slcd_set_par(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	struct lcd_panel_ops *panel_ops;
	struct fb_videomode *mode = dctrl->active_video_mode;

	panel_ops = panel->ops;

	slcd_cfg_init(dctrl, panel->smart_config);
	slcd_timing_init(dctrl, panel, mode);

	if (panel_ops && panel_ops->enable) {
		panel_ops->enable(panel);
	}
	ingenicfb_slcd_mcu_init(dctrl);

	return 0;
}

void disp_interface_change(struct dpu_ctrl *dctrl, int type)
{

}

static void disp_common_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel)
{
	uint32_t disp_com;

	disp_com = reg_read(dctrl, DC_DISP_COM);
	disp_com &= ~DC_DP_IF_SEL_MASK;
	if (panel->lcd_type == LCD_TYPE_SLCD) {
		disp_com |= DC_DISP_COM_SLCD;
	} else if (panel->lcd_type == LCD_TYPE_MIPI_SLCD) {
		disp_com |= DC_DISP_COM_MIPI_SLCD;
	} else if (panel->lcd_type == LCD_TYPE_LVDS_VESA ||
	           panel->lcd_type == LCD_TYPE_LVDS_JEIDA) {
		disp_com |= DC_DISP_COM_LVDS;
	} else {
		disp_com |= DC_DISP_COM_TFT;
	}
	if (panel->dither_enable) {
		disp_com |= DC_DP_DITHER_EN;
		disp_com &= ~DC_DP_DITHER_DW_MASK;
		disp_com |= panel->dither.dither_red
		            << DC_DP_DITHER_DW_RED_LBIT;
		disp_com |= panel->dither.dither_green
		            << DC_DP_DITHER_DW_GREEN_LBIT;
		disp_com |= panel->dither.dither_blue
		            << DC_DP_DITHER_DW_BLUE_LBIT;
	} else {
		disp_com &= ~DC_DP_DITHER_EN;
	}
	reg_write(dctrl, DC_DISP_COM, disp_com);

	/* QOS */
#ifdef CONFIG_SOC_X2600
	reg_write(dctrl, DC_PCFG_RD_CTRL, DC_COMP_ARQOS_CTRL | DC_COMP_ARQOS_VAL_2 | DC_RDMA_ARQOS_CTRL | DC_RDMA_ARQOS_VAL_3);
	reg_write(dctrl, DC_PCFG_WR_CTRL, DC_AWQOS_CTRL | DC_AWQOS_VAL_3);
#else
	reg_write(dctrl, DC_PCFG_RD_CTRL, DC_COMP_ARQOS_CTRL | DC_COMP_ARQOS_VAL_3);
#endif
#if 0
	reg_write(dctrl, DC_PCFG_RD_CTRL, 7);
	reg_write(dctrl, DC_PCFG_WR_CTRL, 1);
	reg_write(dctrl, DC_WDMA_PCFG, 0x1ff << 18 | 0x1f0 << 9 | 0x1e0 << 0);
	reg_write(dctrl, DC_OFIFO_PCFG, 0x1ff << 18 | 0x1f0 << 9 | 0x1e0 << 0);
	reg_write(dctrl, DC_CMPW_PCFG_CTRL, 1 << 10);
#endif
}

static void common_cfg_init(struct dpu_ctrl *dctrl)
{
	unsigned com_cfg = 0;

	com_cfg = reg_read(dctrl, DC_COM_CONFIG);
	/*Keep COM_CONFIG reg first bit 0 */

	com_cfg &= ~DC_OUT_SEL_MASK;
	if (dctrl->chan == DATA_CH_RDMA) {
#ifdef CONFIG_SOC_X2600
		com_cfg |= DC_OUT_SEL_ROT;
		com_cfg &= ~DC_OUT_SEL_RDMA;
#else
		com_cfg |= DC_OUT_SEL_RDMA;
#endif
	} else {
		com_cfg |= DC_OUT_SEL_CMP;
	}

	/* set burst length 16*/
#ifdef CONFIG_SOC_X2600
	com_cfg &= ~DC_BURST_LEN_SCALE_MASK;
	com_cfg |= DC_BURST_LEN_SCALE_16;
#endif
	com_cfg &= ~DC_BURST_LEN_BDMA_MASK;
	com_cfg |= DC_BURST_LEN_BDMA_16;
	com_cfg &= ~DC_BURST_LEN_WDMA_MASK;
	com_cfg |= DC_BURST_LEN_WDMA_16;
	com_cfg &= ~DC_BURST_LEN_RDMA_MASK;
	com_cfg |= DC_BURST_LEN_RDMA_16;

	reg_write(dctrl, DC_COM_CONFIG, com_cfg);
}

int ingenic_set_lvds_display_clk(struct dpu_ctrl *dctrl)
{
#ifdef CONFIG_SOC_X2600
	unsigned int pclk_rate = 0;
	struct clk *pclk_parent;
	int i = 0;
	pclk_rate =  dctrl->panel->dsi_pdata->video_config.pixel_clock * 1000;
	for (i = 0; i < sizeof(pclk_epll_tab) / sizeof(struct x2600_lvds_pclk_param); i++) {
		if (pclk_rate == pclk_epll_tab[i].pclk_freq) {
			break;
		}
	}
	if (i < 0) {
		printk("ERROR: cannot get pclk epll param \n");
		return -1;
	}
	pclk_parent = clk_get(NULL, "epll");
	clk_set_rate(pclk_parent, pclk_epll_tab[i].pll_freq);
	clk_set_parent(clk_get(NULL, "mux_lcd"), pclk_parent);
	clk_set_rate(dctrl->pclk, pclk_rate);
	return 0;
#else
	return 0;
#endif
}

void ingenic_set_pixclk(struct dpu_ctrl *dctrl, unsigned int pixclock_khz)
{
	unsigned long rate, prate;
	struct clk *clk;

	rate = pixclock_khz * 1000;
	clk = clk_get_parent(dctrl->pclk);
	prate = clk_get_rate(clk);

	if (prate % rate) {
		rate = prate / (prate / rate) + 1;
	}

	clk_set_rate(dctrl->pclk, rate);
}

void dump_rdma_desc(struct dpu_ctrl *dctrl)
{
	struct ingenicfb_sreadesc **sreadesc;
	struct rdma_setup_info *rdma_info = &dctrl->rdma_info;
	int i = 0;

	sreadesc = dctrl->sreadesc;
	for (i = 0; i < rdma_info->nframes; i++) {

		dev_info(dctrl->dev, "sreadesc[%d]->RdmaNextCfgAddr: 0x%x\n", i, sreadesc[i]->RdmaNextCfgAddr);
		dev_info(dctrl->dev, "sreadesc[%d]->FrameBufferAddr: 0x%x\n", i, sreadesc[i]->FrameBufferAddr);
		dev_info(dctrl->dev, "sreadesc[%d]->Stride	: %d\n", i, sreadesc[i]->Stride);
		dev_info(dctrl->dev, "sreadesc[%d]->ChainCfg	: %x\n", i, sreadesc[i]->ChainCfg.d32);
		dev_info(dctrl->dev, "sreadesc[%d]->InterruptControl: %x\n", i, sreadesc[i]->InterruptControl.d32);
	}

}
/*
 *
 *  配置寄LCD控制器, 可以配置到寄存器.
 *
 *  1. 根据rdma需求的配置信息，配置rdma描述符.
 *  2. 控制器需要处于停止状态?
 * */
int dpu_ctrl_rdma_setup(struct dpu_ctrl *dctrl, struct rdma_setup_info *rdma_info)
{

	struct ingenicfb_sreadesc **sreadesc;
	int i = 0;

	/* Init RDMA DESC as continous. */
	sreadesc = dctrl->sreadesc;
	for (i = 0; i < rdma_info->nframes; i++) {
		sreadesc[i]->RdmaNextCfgAddr = dctrl->sreadesc_phys[i];
		sreadesc[i]->FrameBufferAddr = rdma_info->vidmem_phys[i];
		sreadesc[i]->Stride = rdma_info->stride;
		sreadesc[i]->ChainCfg.d32 = 0;
		sreadesc[i]->ChainCfg.b.format = rdma_info->format;
		sreadesc[i]->ChainCfg.b.color = rdma_info->color;
		sreadesc[i]->ChainCfg.b.change_2_cmp = 0;
		sreadesc[i]->ChainCfg.b.chain_end = !rdma_info->continuous;
#ifdef TEST_IRQ
		sreadesc[i]->InterruptControl.d32 = DC_SOS_MSK | DC_EOS_MSK | DC_EOD_MSK;
#else
		sreadesc[i]->InterruptControl.d32 = DC_SOS_MSK;
#endif
		sreadesc[i]->InterruptControl.d32 = DC_SOS_MSK | DC_EOS_MSK | DC_EOD_MSK;
	}
	sreadesc[0]->RdmaNextCfgAddr = dctrl->sreadesc_phys[0];
	dctrl->rdma_last_frame = 0;
	if (&dctrl->rdma_info != rdma_info) {
		memcpy(&dctrl->rdma_info, rdma_info, sizeof(struct rdma_setup_info));
	}

	//dump_rdma_desc(dctrl);

	return 0;
}

int dpu_ctrl_rdma_change(struct dpu_ctrl *dctrl, int frame)
{
	/* TODO: 可能改变的内容，包括格式，大小等。这里只是切换frame, 什么时候真正切换还不一定.*/
	/*如果此时正处于直通显示, 需要停止直通，切换到rdma通道.*/
#ifdef CONFIG_ROT_INGENIC
	if (dctrl->work_in_rdma && dctrl->rot_init) {
		if (dctrl->rot_cfg.rot_angle != dctrl->rot_cfg.rot_angle_set) {
			if (dctrl->rot_cfg.rot_angle == 0) {
				dctrl->rot_cfg.rot_angle = dctrl->rot_cfg.rot_angle_set;
				rotator_cfg_init(&dctrl->rot_cfg);
				reg_write(dctrl, DC_CLR_ST, DC_STOP_SRD_ACK);
				reg_write(dctrl, DC_CTRL, DC_GEN_STP_RDMA);
				wait_dc_state(dctrl, DC_STOP_SRD_ACK, DC_STOP_SRD_ACK);
				{
					unsigned long FrameBufferAddr = dctrl->sreadesc[frame]->FrameBufferAddr;
					dctrl->rot_cfg.next_wbbuf = 0;
					dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf] = FrameBufferAddr;
					rot_manual_start(dctrl);
					dctrl->rdma_last_frame = frame;
				}

			} else {
				rot_normal_stop();
				dctrl->rot_cfg.rot_angle = dctrl->rot_cfg.rot_angle_set;
				if (dctrl->rot_cfg.rot_angle_set == 0) {
					reg_write(dctrl, DC_RDMA_CHAIN_ADDR, dctrl->sreadesc_phys[frame]);
					reg_write(dctrl, DC_RDMA_CHAIN_CTRL, DC_RDMA_START);
					dctrl->rdma_last_frame = frame;
				} else {
					unsigned long FrameBufferAddr = dctrl->sreadesc[frame]->FrameBufferAddr;
					rotator_cfg_init(&dctrl->rot_cfg);
					dctrl->rot_cfg.next_wbbuf = 0;
					dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf] = FrameBufferAddr;
					rot_manual_start(dctrl);
					dctrl->rdma_last_frame = frame;
				}
			}

			//printk("rot_angle: %d config: %lx\n",dctrl->rot_cfg.rot_angle_set,reg_read(dctrl, DC_COM_CONFIG));
			return 0;
		}
		if (dctrl->rot_cfg.rot_angle != 0) {
			unsigned long FrameBufferAddr = dctrl->sreadesc[frame]->FrameBufferAddr;
			dctrl->rot_cfg.next_wbbuf = 0;
			dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf] = FrameBufferAddr;
			rot_manual_start(dctrl);
			dctrl->rdma_last_frame = frame;
		} else {
			if (reg_read(dctrl, DC_RDMA_CHAIN_SITE) != dctrl->sreadesc_phys[dctrl->rdma_last_frame]) {
				dev_warn(dctrl->dev, "[rdma] last frame [%d] still in changing!\n", dctrl->rdma_changing_frame);
#ifdef CONFIG_MMU_NOTIFIER
				dctrl_tlb_invalidate_ch(dctrl, DC_ALL_INVLD);
#endif
			}
			reg_write(dctrl, DC_RDMA_CHAIN_ADDR, dctrl->sreadesc_phys[frame]);
			reg_write(dctrl, DC_RDMA_CHAIN_CTRL, DC_RDMA_START);
			dctrl->rdma_last_frame = frame;
		}
	} else
#endif
	{
		if (reg_read(dctrl, DC_RDMA_CHAIN_SITE) != dctrl->sreadesc_phys[dctrl->rdma_last_frame]) {
			dev_warn(dctrl->dev, "[rdma] last frame [%d] still in changing!\n", dctrl->rdma_changing_frame);
#ifdef CONFIG_MMU_NOTIFIER
			dctrl_tlb_invalidate_ch(dctrl, DC_ALL_INVLD);
#endif
		}
		reg_write(dctrl, DC_RDMA_CHAIN_ADDR, dctrl->sreadesc_phys[frame]);
		reg_write(dctrl, DC_RDMA_CHAIN_CTRL, DC_RDMA_START);
		dctrl->rdma_last_frame = frame;

	}
	return 0;
}

int dpu_ctrl_rdma_start(struct dpu_ctrl *dctrl)
{
	reg_write(dctrl, DC_RDMA_CHAIN_ADDR, dctrl->sreadesc_phys[0]);
	reg_write(dctrl, DC_RDMA_CHAIN_CTRL, DC_RDMA_START);
	dctrl->rdma_last_frame = 0;
	//dump_rdma_desc_reg(dctrl);

	return 0;
}

int dpu_ctrl_rdma_stop(struct dpu_ctrl *dctrl, enum stop_mode mode)
{
	int ret = 0;

	if (mode == QCK_STOP) {
		reg_write(dctrl, DC_CTRL, DC_QCK_STP_RDMA);
		wait_dc_state(dctrl, DC_SRD_WORKING, 0);
	} else {
		dctrl->srd_stopped = 0;
		reg_write(dctrl, DC_CTRL, DC_GEN_STP_RDMA);
		dev_info(dctrl->dev, "stopping rdma ...\n");
		ret = wait_event_interruptible_timeout(dctrl->wq,
		                                       dctrl->srd_stopped == 1, msecs_to_jiffies(1000 / dctrl->active_video_mode->refresh + 3));
		if ((ret == 0) && reg_read(dctrl, DC_ST) & DC_SRD_WORKING) {
			dev_err(dctrl->dev, "dpu rdma wait gen stop timeout!!!\n");
#ifdef CONFIG_FB_INGENIC_DUMP
			dump_lcdc_registers(dctrl);
#endif
		}
	}

	return 0;
}

int dpu_ctrl_rdma_wait_for_vsync(struct dpu_ctrl *dctrl)
{
	int ret = 0;
	int vblank = dctrl->vblank_count;
	ret = wait_event_interruptible_timeout(dctrl->wq,
	                                       vblank != dctrl->vblank_count, msecs_to_jiffies(3000));
	if (ret == 0) {
		dev_err(dctrl->dev, "wait vblank timeout\n");
	}

	return ret;
}

#ifdef CONFIG_FB_INGENIC_DUMP
static void dump_frm_desc(struct ingenicfb_framedesc *framedesc, int index)
{
	printk("-------User Frame Descriptor index[%d]-----\n", index);
	printk("FramedescAddr:	    0x%x\n", (uint32_t)framedesc);
	printk("FrameNextCfgAddr:   0x%x\n", framedesc->FrameNextCfgAddr);
	printk("FrameSize:          0x%x\n", framedesc->FrameSize.d32);
	printk("FrameCtrl:          0x%x\n", framedesc->FrameCtrl.d32);
	printk("WritebackAddr:	    0x%x\n", framedesc->WritebackAddr);
	printk("WritebackStride:    0X%x\n", framedesc->WritebackStride);
	printk("Layer0CfgAddr:      0x%x\n", framedesc->Layer0CfgAddr);
	printk("Layer1CfgAddr:      0x%x\n", framedesc->Layer1CfgAddr);
	printk("Layer2CfgAddr:      0x%x\n", framedesc->Layer2CfgAddr);
	printk("Layer3CfgAddr:      0x%x\n", framedesc->Layer3CfgAddr);
	printk("LayerCfgEn:	    0x%x\n", framedesc->LayCfgEn.d32);
	printk("InterruptControl:   0x%x\n", framedesc->InterruptControl.d32);
	printk("-------User Frame Descriptor index[%d]-----\n", index);
}

static void dump_layer_desc(struct ingenicfb_layerdesc *layerdesc, int row, int col)
{
	printk("------User layer Descriptor index[%d][%d]------\n", row, col);
	printk("LayerdescAddr:	    0x%x\n", (uint32_t)layerdesc);
	printk("LayerSize:          0x%x\n", layerdesc->LayerSize.d32);
	printk("LayerCfg:           0x%x\n", layerdesc->LayerCfg.d32);
	printk("LayerBufferAddr:    0x%x\n", layerdesc->LayerBufferAddr);
	printk("LayerScale:         0x%x\n", layerdesc->LayerScale.d32);
	printk("LayerResizeCoef_X:  0x%x\n", layerdesc->layerresizecoef_x);
	printk("LayerResizeCoef_Y:  0x%x\n", layerdesc->layerresizecoef_y);
	printk("LayerPos:           0x%x\n", layerdesc->LayerPos.d32);
	printk("LayerStride:        0x%x\n", layerdesc->LayerStride);
	printk("Layer_UV_addr:	    0x%x\n", layerdesc->UVBufferAddr);
	printk("Layer_UV_stride:    0x%x\n", layerdesc->UVStride);
	printk("------User layer Descriptor index[%d][%d]------\n", row, col);
}

void dump_lay_cfg(struct ingenicfb_lay_cfg *lay_cfg, int index)
{
	printk("------User disp set index[%d]------\n", index);
	printk("lay_en:		   0x%x\n", lay_cfg->lay_en);
	printk("tlb_en:		   0x%x\n", lay_cfg->tlb_en);
	printk("lay_scale_en:	   0x%x\n", lay_cfg->lay_scale_en);
	printk("lay_z_order:	   0x%x\n", lay_cfg->lay_z_order);
	printk("source_w:	   0x%x\n", lay_cfg->source_w);
	printk("source_h:	   0x%x\n", lay_cfg->source_h);
	printk("src_crop_x:	   0x%x\n", lay_cfg->src_crop_x);
	printk("src_crop_y:	   0x%x\n", lay_cfg->src_crop_y);
	printk("src_crop_w:	   0x%x\n", lay_cfg->src_crop_w);
	printk("src_crop_h:	   0x%x\n", lay_cfg->src_crop_h);
	printk("disp_pos_x:	   0x%x\n", lay_cfg->disp_pos_x);
	printk("disp_pos_y:	   0x%x\n", lay_cfg->disp_pos_y);
	printk("scale_w:	   0x%x\n", lay_cfg->scale_w);
	printk("scale_h:	   0x%x\n", lay_cfg->scale_h);
	printk("g_alpha_en:	   0x%x\n", lay_cfg->g_alpha_en);
	printk("g_alpha_val:	   0x%x\n", lay_cfg->g_alpha_val);
	printk("color:		   0x%x\n", lay_cfg->color);
	printk("format:		   0x%x\n", lay_cfg->format);
	printk("stride:		   0x%x\n", lay_cfg->stride);
	printk("addr[0]:	   0x%x\n", lay_cfg->addr[0]);
	printk("addr[1]:	   0x%x\n", lay_cfg->addr[1]);
	printk("addr[2]:	   0x%x\n", lay_cfg->addr[2]);

	printk("------User disp set index[%d]------\n", index);
}

__attribute__((__unused__)) static void dump_comp_info(struct comp_setup_info *comp_info)
{
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	struct wback_cfg *wback_info    = &frm_cfg->wback_info;
	int i;

	printk("\n\n");
	printk("width: %d, height: %d\n", frm_cfg->width, frm_cfg->height);

	printk("wback_info: en:%d  dither_en:%d  fmt:%d  stride:%d  addr:%08x \n", wback_info->en, wback_info->dither_en, wback_info->fmt, wback_info->stride, wback_info->addr);

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {

		dump_lay_cfg(&lay_cfg[i], i);
	}
	printk("\n\n");
}

void dump_comp_desc(struct dpu_ctrl *dctrl)
{
	int i, j;
	for (i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		for (j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
			dump_layer_desc(dctrl->layerdesc[i][j], i, j);
		}
		dump_frm_desc(dctrl->framedesc[i], i);
	}
}
#endif
static int dpu_comp_framedesc_setup_frame(struct dpu_ctrl *dctrl, struct ingenicfb_framedesc *last, struct ingenicfb_framedesc *new)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct wback_cfg *wback_info    = &frm_cfg->wback_info;
	// 先保留旧的配置.
#if 0
	if (last) {
		memcpy(new, last, sizeof(struct ingenicfb_framedesc));
	}
#endif

#if 1
	if (last) {
		/*Keep Last Layer Info*/
#if 0
		new->Layer0CfgAddr = last->Layer0CfgAddr;
		new->Layer1CfgAddr = last->Layer1CfgAddr;
		new->Layer2CfgAddr = last->Layer2CfgAddr;
		new->Layer3CfgAddr = last->Layer3CfgAddr;
		new->LayCfgEn.d32 = last->LayCfgEn.d32;
#endif
		int i = 0;
		for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
			struct ingenicfb_layerdesc *old_layer_desc = dctrl->layerdesc[last->index][i];
			struct ingenicfb_layerdesc *new_layer_desc = dctrl->layerdesc[new->index][i];

			// 先保持旧的配置，后面再单独更新某一个Layer的配置.

			new_layer_desc->LayerSize   = old_layer_desc->LayerSize;
			new_layer_desc->LayerCfg    = old_layer_desc->LayerCfg;
			new_layer_desc->LayerBufferAddr = old_layer_desc->LayerBufferAddr;
			new_layer_desc->LayerScale  = old_layer_desc->LayerScale;
			new_layer_desc->LayerRotation   = old_layer_desc->LayerRotation;
			new_layer_desc->LayerScratch    = old_layer_desc->LayerScratch;
			new_layer_desc->LayerPos    = old_layer_desc->LayerPos;
			new_layer_desc->layerresizecoef_x = old_layer_desc->layerresizecoef_x;
			new_layer_desc->layerresizecoef_y = old_layer_desc->layerresizecoef_y;
			new_layer_desc->LayerStride = old_layer_desc->LayerStride;
			new_layer_desc->UVBufferAddr    = old_layer_desc->UVBufferAddr;
			new_layer_desc->UVStride    = old_layer_desc->UVStride;
		}

		new->LayCfgEn.d32  = last->LayCfgEn.d32;
	}

	new->Layer0CfgAddr = dctrl->layerdesc[new->index][0]->phy_addr;
	new->Layer1CfgAddr = dctrl->layerdesc[new->index][1]->phy_addr;
	new->Layer2CfgAddr = dctrl->layerdesc[new->index][2]->phy_addr;
	new->Layer3CfgAddr = dctrl->layerdesc[new->index][3]->phy_addr;


#endif
	/* setup new cfg. */
	new->FrameNextCfgAddr = new->phy_addr;
	new->FrameSize.d32 = 0;
	new->FrameSize.b.width = frm_cfg->width;
	new->FrameSize.b.height = frm_cfg->height;
	new->FrameCtrl.d32 = 0;
	new->FrameCtrl.b.stop = comp_info->out_mode == COMP_DIRECT_OUT ? 0 : 1; //当直接显示时，帧描述符不停止.
	new->FrameCtrl.b.wb_en = !!wback_info->en;
	new->FrameCtrl.b.direct_en = comp_info->out_mode == COMP_DIRECT_OUT ? 1 : 0;
	new->FrameCtrl.b.change_2_rdma = 0;
	new->FrameCtrl.b.wb_dither_en = 0;
	new->FrameCtrl.b.wb_dither_auto = 0;
	new->FrameCtrl.b.wb_dither_auto = 0;
	new->FrameCtrl.b.wb_dither_b_dw = 0;
	new->FrameCtrl.b.wb_dither_g_dw = 0;
	new->FrameCtrl.b.wb_dither_r_dw = 0;

	new->WritebackAddr = wback_info->addr;
	new->WritebackStride = wback_info->stride;
	new->FrameCtrl.b.wb_format = wback_info->fmt;
	if (!dctrl->work_in_rdma) {
#ifdef CONFIG_SOC_X2600
		struct fb_videomode *mode = dctrl->active_video_mode;
		if (dctrl->rot_cfg.rot_angle != 0) {
			new->WritebackAddr = dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf];
			new->FrameCtrl.b.wb_en = 1;
			new->FrameCtrl.b.direct_en = 0;
			new->FrameCtrl.b.wb_format = 7;
			new->WritebackStride = mode->xres;

#ifdef TEST_COMPOSER_2_RDMA
		} else {
			// 将rot_wbbuf_phy 当作中间buffer, 作为RDMA 的输入.
			new->WritebackAddr = dctrl->sreadesc[dctrl->rdma_next_frame]->FrameBufferAddr;//dctrl->rot_wbbuf_phy[dctrl->rot_cfg.next_wbbuf];
			new->FrameCtrl.b.wb_en = 0;
			new->FrameCtrl.b.direct_en = 1;
			new->FrameCtrl.b.wb_format = 0; //ARGB888.
			//new->FrameCtrl.b.wb_format = 7;
			new->WritebackStride = mode->xres;
#endif
		}

#endif
	}
	if (comp_info->out_mode == COMP_WRITE_BACK || comp_info->out_mode == COMP_WRITE_BACK_2_ROTATOR) {
		new->InterruptControl.d32 = DC_EOW_MSK;
	} else {
		new->InterruptControl.d32 = DC_SOC_MSK;
	}
	if (!dctrl->work_in_rdma) {
		new->InterruptControl.d32 = DC_SOC_MSK | DC_EOD_MSK | DC_EOW_MSK | DC_EOC_MSK;
	}
	return 0;
}
static int dpu_comp_framedesc_setup_layer(struct dpu_ctrl *dctrl, struct ingenicfb_framedesc *new_frame, struct ingenicfb_layerdesc *layerdesc, int layer, struct ingenicfb_lay_cfg *lay_cfg)
{


	if (layerdesc->phy_addr != dctrl->layerdesc_phys[new_frame->index][layer]) {
		printk("----layerdesc->phy_addr: %x, %x\n", layerdesc->phy_addr, dctrl->layerdesc_phys[new_frame->index][layer]);
	}
	switch (layer) {
		case 0:
			/*只修改更改过的Layer信息*/
			//new_frame->Layer0CfgAddr = layerdesc->phy_addr;
			new_frame->LayCfgEn.b.lay0_scl_en = lay_cfg->lay_scale_en;
			new_frame->LayCfgEn.b.lay0_en = lay_cfg->lay_en;
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay1_z_order) {
				new_frame->LayCfgEn.b.lay1_z_order = new_frame->LayCfgEn.b.lay0_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay2_z_order) {
				new_frame->LayCfgEn.b.lay2_z_order = new_frame->LayCfgEn.b.lay0_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay3_z_order) {
				new_frame->LayCfgEn.b.lay3_z_order = new_frame->LayCfgEn.b.lay0_z_order;
			}
			new_frame->LayCfgEn.b.lay0_z_order = lay_cfg->lay_z_order;
			break;
		case 1:
			//new_frame->Layer1CfgAddr = layerdesc->phy_addr;
			new_frame->LayCfgEn.b.lay1_scl_en = lay_cfg->lay_scale_en;
			new_frame->LayCfgEn.b.lay1_en = lay_cfg->lay_en;
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay0_z_order) {
				new_frame->LayCfgEn.b.lay0_z_order = new_frame->LayCfgEn.b.lay1_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay2_z_order) {
				new_frame->LayCfgEn.b.lay2_z_order = new_frame->LayCfgEn.b.lay1_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay3_z_order) {
				new_frame->LayCfgEn.b.lay3_z_order = new_frame->LayCfgEn.b.lay1_z_order;
			}
			new_frame->LayCfgEn.b.lay1_z_order = lay_cfg->lay_z_order;
			break;
		case 2:
			//new_frame->Layer2CfgAddr = layerdesc->phy_addr;
			new_frame->LayCfgEn.b.lay2_scl_en = lay_cfg->lay_scale_en;
			new_frame->LayCfgEn.b.lay2_en = lay_cfg->lay_en;
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay0_z_order) {
				new_frame->LayCfgEn.b.lay0_z_order = new_frame->LayCfgEn.b.lay2_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay1_z_order) {
				new_frame->LayCfgEn.b.lay1_z_order = new_frame->LayCfgEn.b.lay2_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay3_z_order) {
				new_frame->LayCfgEn.b.lay3_z_order = new_frame->LayCfgEn.b.lay2_z_order;
			}
			new_frame->LayCfgEn.b.lay2_z_order = lay_cfg->lay_z_order;
			break;
		case 3:
			//new_frame->Layer3CfgAddr = layerdesc->phy_addr;
			new_frame->LayCfgEn.b.lay3_scl_en = lay_cfg->lay_scale_en;
			new_frame->LayCfgEn.b.lay3_en = lay_cfg->lay_en;
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay0_z_order) {
				new_frame->LayCfgEn.b.lay0_z_order = new_frame->LayCfgEn.b.lay3_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay1_z_order) {
				new_frame->LayCfgEn.b.lay1_z_order = new_frame->LayCfgEn.b.lay3_z_order;
			}
			if (lay_cfg->lay_z_order == new_frame->LayCfgEn.b.lay2_z_order) {
				new_frame->LayCfgEn.b.lay2_z_order = new_frame->LayCfgEn.b.lay3_z_order;
			}
			new_frame->LayCfgEn.b.lay3_z_order = lay_cfg->lay_z_order;
			break;
		default:
			printk("----Error update with no layer info !!!!\n");
			break;
	}
	/*layer2和layer3不支持NV12、NV21、YUV422*/
	if (layer >= 2 && layerdesc->LayerCfg.b.format >= LAYER_CFG_FORMAT_NV12) {
		printk("[Error] Layer2 and Layer3 do not support NV12、NV21、YUV422");
	}
	//framedesc[next_frame]->LayCfgEn.d32 = 0;
	return 0;
}

#ifdef CONFIG_MMU_NOTIFIER
static int dpu_dmmu_mm_release(void *data)
{
	struct dpu_ctrl *dctrl = (struct dpu_ctrl *)data;
	struct comp_setup_info setup_info;
	// TODO: add lock.

	struct ingenicfb_frm_cfg *frm_cfg = NULL;
	struct ingenicfb_lay_cfg *lay_cfg = NULL;
	int j = 0;
	/*关闭所有的使用TLB的Layer.*/

	memcpy(&setup_info, &dctrl->comp_info, sizeof(struct comp_setup_info));

	frm_cfg = &setup_info.frm_cfg;
	lay_cfg = frm_cfg->lay_cfg;

	for (j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
		if (lay_cfg[j].tlb_en) {
			lay_cfg[j].tlb_en = 0;
			lay_cfg[j].lay_en = 0;
		}
	}

	if (likely(dctrl->tlba)) {
		dctrl->tlba = 0;
	}

	dpu_ctrl_comp_setup(dctrl, &setup_info);
	dpu_ctrl_comp_start(dctrl, 0);

	return 0;

}

static struct dmmu_mm_ops dpu_dmmu_mm_ops = {
	.mm_release = dpu_dmmu_mm_release,
};

int dpu_ctrl_comp_dmmu_map(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info)
{
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;
	int j = 0;

	for (j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
		unsigned int start_addr = lay_cfg[j].addr[0];
		unsigned int start_uvaddr = lay_cfg[j].uv_addr[0];

		if (lay_cfg[j].tlb_en) {

			unsigned int size = 0;
			unsigned int uv_size = 0;
			unsigned int tlba = 0;
			size = _stride_in_bytes(lay_cfg[j].stride, lay_cfg[j].format) * lay_cfg[j].source_h;
			uv_size = _stride_in_bytes(lay_cfg[j].uv_stride, lay_cfg[j].format) * lay_cfg[j].source_h / 2;


			if (start_addr) {
				tlba = dmmu_map(dctrl->dev, start_addr, size);
			}

			if (start_uvaddr) {
				tlba = dmmu_map(dctrl->dev, start_uvaddr, uv_size);
			}

			if (tlba == 0) {
				dev_info(dctrl->dev, "%d dmmu map get 0, disable this layer \n", j);
				lay_cfg[j].lay_en = 0;
				lay_cfg[j].lay_scale_en = 0;
				lay_cfg[j].tlb_en = 0;
				continue;
			}

			if (dctrl->tlba != tlba) {
				dev_info(dctrl->dev, "setting dpu tlb address from [%x] to [%x]\n", dctrl->tlba, tlba);
				dctrl->tlba = tlba;

				dctrl->dmn.dev = dctrl->dev;
				dctrl->dmn.data = dctrl;
				dctrl->dmn.ops = &dpu_dmmu_mm_ops;
				dmmu_register_mm_notifier(&dctrl->dmn);
			}
		}
	}

	return 0;
}
#endif

int compare_setup_info(struct dpu_ctrl *dctrl, struct comp_setup_info *new_info, struct comp_setup_info *old_info)
{
	struct ingenicfb_frm_cfg *new_frm_cfg = &new_info->frm_cfg;
	struct wback_cfg *new_wback_info    = &new_frm_cfg->wback_info;
	struct ingenicfb_lay_cfg *new_lay_cfg = new_frm_cfg->lay_cfg;


	struct ingenicfb_frm_cfg *old_frm_cfg = &old_info->frm_cfg;
	struct wback_cfg *old_wback_info    = &old_frm_cfg->wback_info;
	struct ingenicfb_lay_cfg *old_lay_cfg = old_frm_cfg->lay_cfg;

	int ret = 0;
	int i = 0;

	unsigned int changed = 0;

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		ret = memcmp(&old_lay_cfg[i], &new_lay_cfg[i], sizeof(struct ingenicfb_lay_cfg));
		//      printk("----layer: %d, ret: %d, layer_uptodate: %d\n", i, ret, dctrl->layer_uptodate[i]);
		if (ret) {
			dctrl->layer_info_changed[i] = 1;
			changed = 1;
#if 0
		} else {
			dctrl->layer_info_changed[i] = 0;
#endif
		}

	}
	ret = memcmp(old_wback_info, new_wback_info, sizeof(struct wback_cfg));
	if (ret) {
		changed = 1;
	}

	//      printk("wback---:ret: %d\n", ret);

	ret = memcmp(old_frm_cfg, new_frm_cfg, sizeof(struct ingenicfb_frm_cfg));
	if (ret) {
		changed = 1;
	}
	//      printk("frmcfg---:ret: %d\n", ret);

#if 0
	if (!changed) {

		dump_stack();
	}
#endif

	dctrl->comp_info_changed = changed;

	return changed;
}

static int dpu_comp_layerdesc_setup_layer(struct dpu_ctrl *dctrl, struct ingenicfb_layerdesc *layerdesc, struct ingenicfb_lay_cfg *lay_cfg)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;

	if (!lay_cfg->lay_en) {
		layerdesc->LayerScale.b.target_width = lay_cfg->scale_w = 0;
		layerdesc->LayerScale.b.target_height = lay_cfg->scale_h = 0;
		layerdesc->layerresizecoef_x = 0;
		layerdesc->layerresizecoef_y = 0;
		lay_cfg->lay_scale_en = 0;
		lay_cfg->tlb_en = 0;
		return 0;
	}

	if ((lay_cfg->src_crop_w == 0) || (lay_cfg->src_crop_w >= lay_cfg->source_w)) {
		lay_cfg->src_crop_w = lay_cfg->source_w;
	}

	if ((lay_cfg->src_crop_h == 0) || (lay_cfg->src_crop_h >= lay_cfg->source_h)) {
		lay_cfg->src_crop_h = lay_cfg->source_h;
	}

#define MIN(a,b) ((a) > (b)? (b):(a))
#define MAX(a,b) ((a) > (b)? (a):(b))
	/* If crop != scale , enable scale. Ensure:
	   crop-size (for disable scale) or scale-size (for enable scale) <= screen-size
	*/
	lay_cfg->scale_w = MIN(lay_cfg->scale_w, frm_cfg->width);
	lay_cfg->scale_h = MIN(lay_cfg->scale_h, frm_cfg->height);
	lay_cfg->lay_scale_en = 0;

	if (lay_cfg->src_crop_w != lay_cfg->scale_w || lay_cfg->src_crop_h != lay_cfg->scale_h) {
		if (lay_cfg->scale_w && lay_cfg->scale_h) {
			lay_cfg->lay_scale_en = 1;
			lay_cfg->scale_w = MAX(lay_cfg->scale_w, 4);
			lay_cfg->scale_h = MAX(lay_cfg->scale_h, 4);
		} else {
			lay_cfg->src_crop_w = MIN(lay_cfg->src_crop_w, frm_cfg->width);
			lay_cfg->src_crop_h = MIN(lay_cfg->src_crop_h, frm_cfg->height);
		}
	}
	/*
	    大小，位置，缩放 限制.
	    目标位置 + 目标缩放宽高，不能超出显示区域.
	    防止 pos_x 设置超出范围.
	    src_crop_x + src_crop_w <= src_width;
	*/
	{
		unsigned int src_width = lay_cfg->source_w;
		unsigned int src_height = lay_cfg->source_h;
		unsigned int src_crop_w = lay_cfg->src_crop_w;
		unsigned int src_crop_h = lay_cfg->src_crop_h;
		unsigned int src_crop_x = lay_cfg->src_crop_x;
		unsigned int src_crop_y = lay_cfg->src_crop_y;

		lay_cfg->src_crop_x = src_crop_w + src_crop_x >= src_width ? src_width  - src_crop_w : src_crop_x;
		lay_cfg->src_crop_y = src_crop_h + src_crop_y >= src_height ? src_height - src_crop_h : src_crop_y;
		//printk("crop xy: %dx%d, wh: %dx%d\n", lay_cfg->src_crop_x, lay_cfg->src_crop_y, lay_cfg->src_crop_w, lay_cfg->src_crop_h);
	}
	{
		unsigned int scale_w = lay_cfg->scale_w;
		unsigned int scale_h = lay_cfg->scale_h;

		unsigned int pos_x = lay_cfg->disp_pos_x;
		unsigned int pos_y = lay_cfg->disp_pos_y;

		if (lay_cfg->lay_scale_en == 0) {
			lay_cfg->disp_pos_x = (pos_x + lay_cfg->src_crop_w) >= frm_cfg->width ? frm_cfg->width - lay_cfg->src_crop_w : pos_x;
			lay_cfg->disp_pos_y = (pos_y + lay_cfg->src_crop_h) >= frm_cfg->height ? frm_cfg->height - lay_cfg->src_crop_h : pos_y;
		} else {
			lay_cfg->disp_pos_x = (pos_x + scale_w) >= frm_cfg->width ? frm_cfg->width - scale_w : pos_x;
			lay_cfg->disp_pos_y = (pos_y + scale_h) >= frm_cfg->height ? frm_cfg->height - scale_h : pos_y;
		}
		if (lay_cfg->disp_pos_x < 0 || lay_cfg->disp_pos_y < 0) {
			printk("**error pos: display(%dx%d)pos(%dx%d),scale(%dx%d),frm_cfg(%dx%d),scale:%d\n",
			       lay_cfg->disp_pos_x, lay_cfg->disp_pos_y,
			       pos_x, pos_y,  scale_w, scale_h,
			       frm_cfg->width, frm_cfg->height, lay_cfg->lay_scale_en);
		}
	}

	layerdesc->LayerSize.d32        = 0;
	layerdesc->LayerSize.b.width    = lay_cfg->src_crop_w;
	layerdesc->LayerSize.b.height   = lay_cfg->src_crop_h;
	layerdesc->LayerPos.d32         = 0;
	layerdesc->LayerPos.b.x_pos     = lay_cfg->disp_pos_x;
	layerdesc->LayerPos.b.y_pos     = lay_cfg->disp_pos_y;
	layerdesc->LayerCfg.d32         = 0;
	layerdesc->LayerCfg.b.g_alpha_en    = lay_cfg->g_alpha_en;
	layerdesc->LayerCfg.b.g_alpha   = lay_cfg->g_alpha_val;
	layerdesc->LayerCfg.b.color     = lay_cfg->color;

	if (lay_cfg->version > 0) {
		layerdesc->LayerCfg.b.domain_multi = lay_cfg->domain_multi;
	} else if (lay_cfg->version == 0) {
		layerdesc->LayerCfg.b.domain_multi = 1;
	} else {
		printk("lay_cfg[0].version[%x] isn't support\n", lay_cfg->version);
		layerdesc->LayerCfg.b.domain_multi = 1;
	}

	layerdesc->LayerCfg.b.format    = lay_cfg->format;
	layerdesc->LayerCfg.b.sharpl    = 0;
	layerdesc->LayerStride      = lay_cfg->stride;
	layerdesc->LayerScale.d32   = 0;
	if (lay_cfg->lay_scale_en) {
		layerdesc->LayerScale.b.target_width    = lay_cfg->scale_w;
		layerdesc->LayerScale.b.target_height   = lay_cfg->scale_h;
		layerdesc->layerresizecoef_x        = (512 * lay_cfg->src_crop_w) / lay_cfg->scale_w;
		layerdesc->layerresizecoef_y        = (512 * lay_cfg->src_crop_h) / lay_cfg->scale_h;
	} else {
		layerdesc->LayerScale.b.target_width    = 0;
		layerdesc->LayerScale.b.target_height   = 0;
		layerdesc->layerresizecoef_x        = 0;
		layerdesc->layerresizecoef_y        = 0;
	}
	{
		unsigned int start_addr = lay_cfg->addr[0];
		unsigned int start_uvaddr = lay_cfg->uv_addr[0];

		start_addr += lay_cfg->src_crop_y * _stride_in_bytes(lay_cfg->stride, lay_cfg->format)
		              + lay_cfg->src_crop_x * _format_depth_in_bytes(lay_cfg->format);

		start_uvaddr += lay_cfg->src_crop_y / 2 * _stride_in_bytes(lay_cfg->uv_stride, lay_cfg->format)
		                + lay_cfg->src_crop_x * _format_depth_in_bytes(lay_cfg->format);

		layerdesc->LayerBufferAddr              = start_addr;
		layerdesc->UVBufferAddr                     = start_uvaddr;
		layerdesc->UVStride                     = lay_cfg->uv_stride;
	}

	//  dctrl->layerdesc[next_frame][layer]->buffer_index = comp_info->layer_update_frame[layer];

	return 0;

}

int dpu_ctrl_comp_setup(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info)
{
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct wback_cfg *wback_info    = &frm_cfg->wback_info;
	struct fb_videomode *mode = dctrl->active_video_mode;
	int ret = 0;
	unsigned long flags;
	int i = 0;

	spin_lock_irqsave(&dctrl->irq_lock, flags);

	ret = compare_setup_info(dctrl, comp_info, &dctrl->saved_comp_info);
	if (!dctrl->comp_info_changed) {
		spin_unlock_irqrestore(&dctrl->irq_lock, flags);
		return 0;
	}

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		if (dctrl->layer_info_changed[i] == 1) {
			struct comp_layer_request *layer_req = list_first_entry_or_null(&dctrl->comp_layers_req_free_list[i], struct comp_layer_request, list);

			struct comp_layer_request *old_req = list_last_entry(&dctrl->comp_layers_req_queued_list[i], struct comp_layer_request, list);
			//printk("i: %d old_req: %x, req_queue: %x\n", i, old_req, &dctrl->comp_layers_req_queued_list[i]);
			if (&old_req->list == &dctrl->comp_layers_req_queued_list[i]) {
				old_req = NULL;
			}

			/*
			    判断并和并请求.
			*/
			if (old_req && comp_info->frm_cfg.lay_cfg[i].addr[0] == old_req->cfg.addr[0]) {

				//printk("--- keep  layer :%d old cfg!\n", i);
				memcpy(&old_req->cfg, &comp_info->frm_cfg.lay_cfg[i], sizeof(struct ingenicfb_lay_cfg));
			} else {
				memcpy(&layer_req->cfg, &comp_info->frm_cfg.lay_cfg[i], sizeof(struct ingenicfb_lay_cfg));
				list_del(&layer_req->list);
				list_add_tail(&layer_req->list, &dctrl->comp_layers_req_queued_list[i]);
			}

			dctrl->layer_info_changed[i] = 0;

			//printk("=========>>sw update> layer: %d, queued req list len: %d, free req list: %d\n", i, list_len(&dctrl->comp_layers_req_queued_list[i]), list_len(&dctrl->comp_layers_req_free_list[i]));
		}
	}

	memcpy(&dctrl->saved_comp_info, comp_info, sizeof(struct comp_setup_info));

	if (wback_info->en || dctrl->work_in_rdma) {
		comp_info->out_mode = COMP_WRITE_BACK;
	} else {
#ifndef CONFIG_SOC_X2600
		comp_info->out_mode = COMP_DIRECT_OUT;
		frm_cfg->width = mode->xres;
		frm_cfg->height = mode->yres;
#else
		if (dctrl->rot_cfg.rot_angle != 0) {
			comp_info->out_mode = COMP_WRITE_BACK_2_ROTATOR;
		} else {
			comp_info->out_mode = COMP_DIRECT_OUT;
		}
		if (dctrl->rot_cfg.rot_angle == 90 || dctrl->rot_cfg.rot_angle == 270) {
			frm_cfg->width = mode->yres;
			frm_cfg->height = mode->xres;
		} else {
			frm_cfg->width = mode->xres;
			frm_cfg->height = mode->yres;
		}
#endif
	}

	memcpy(&dctrl->comp_info, comp_info, sizeof(struct comp_setup_info));

	/*Only Composer Need CSC settings.*/
	dctrl->csc_mode = CSC_MODE_1;
	csc_mode_set(dctrl, dctrl->csc_mode);

	ret = _dpu_ctrl_comp_try_start(dctrl);

	spin_unlock_irqrestore(&dctrl->irq_lock, flags);

	return ret;
}

static inline long timeval_sub_to_us(struct timespec64 lhs,
                                     struct timespec64 rhs)
{
	long sec, nsec;
	sec = lhs.tv_sec - rhs.tv_sec;
	nsec = lhs.tv_nsec - rhs.tv_nsec;

	return (sec * 1000000 + nsec / 1000);
}

#ifdef CONFIG_MMU_NOTIFIER
static void dctrl_flush_cache(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_frm_cfg *frm_cfg = &comp_info->frm_cfg;
	struct ingenicfb_lay_cfg *lay_cfg = frm_cfg->lay_cfg;

	int i = 0;

	/* TODO:
	 * 如果使用的是tlb方式，则每次在更新描述符时，需要flush cache, 保证dpu取道的是正确的数据.
	 * */

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		unsigned int size = 0;
		unsigned int uv_size = 0;
		if ((!lay_cfg[i].lay_en)) {
			continue;
		}
		size = _stride_in_bytes(lay_cfg[i].stride, lay_cfg[i].format) * lay_cfg[i].source_h;
		uv_size = _stride_in_bytes(lay_cfg[i].uv_stride, lay_cfg[i].format) * lay_cfg[i].source_h / 2;
		if (lay_cfg[i].tlb_en) {
			if (lay_cfg[i].addr[0]) {
				dma_cache_wback_inv(lay_cfg[i].addr[0], size);
			}
			if (lay_cfg[i].uv_addr[0] && uv_size) {
				dma_cache_wback_inv(lay_cfg[i].uv_addr[0], uv_size);
			}
		}
#ifdef CONFIG_FB_USING_CACHABLE
		else {
			if (lay_cfg[i].addr[0] && size) {
				dma_sync_single_for_device(dctrl->dev, lay_cfg[i].addr[0], size, DMA_BIDIRECTIONAL);
			}
			if (lay_cfg[i].uv_addr[0] && uv_size) {
				dma_sync_single_for_device(dctrl->dev, lay_cfg[i].uv_addr[0], uv_size, DMA_BIDIRECTIONAL);
			}
		}
#endif
	}
}
#endif

static int _link_all_queued_desc(struct dpu_ctrl *dctrl)
{
	struct ingenicfb_framedesc *queued_desc = NULL;

	int cnt = 0;

	int list_cnt = list_len(&dctrl->comp_desc_queued_list);

	if (list_cnt == 0) {
		return 0;
	}

	list_for_each_entry_reverse(queued_desc, &dctrl->comp_desc_queued_list, list) {
		struct ingenicfb_layerdesc *layer_desc;
		struct ingenicfb_framedesc *prev = list_prev_entry(queued_desc, list);

#if 0
		printk("====prev: %x, &dctrl->comp_desc_queued_list: %x\n", &prev->list, &dctrl->comp_desc_queued_list);

		printk("---queued_desc: %d, prev: %d\n", queued_desc->index, prev->index);
#endif

		if (&prev->list != &dctrl->comp_desc_queued_list) {
			prev->FrameNextCfgAddr = queued_desc->phy_addr;
			cnt++;
		} else {
			//break;
		}
		//      printk("queued_desc->index: %d\n", queued_desc->index);
		//      printk("Layer1CfgAddr: %x\n", queued_desc->Layer1CfgAddr);
		layer_desc = (struct ingenicfb_layerdesc *)KSEG1ADDR(queued_desc->Layer1CfgAddr);

		//      printk("layer_desc->LayerBufferAddr: %x, index: %d\n", layer_desc->LayerBufferAddr, layer_desc->buffer_index);
	}

	if (list_cnt > 1) {
		//  printk("--- linked descriptors: %d\n", cnt);
	}
	return 0;
}
static int _do_comp_start_try(struct dpu_ctrl *dctrl)
{

	int i = 0;
	int lay_scale_en_num = 0;
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	struct ingenicfb_framedesc *desc_to_start = NULL;
	struct ingenicfb_framedesc *last_queued_desc = NULL;
	struct ingenicfb_framedesc *desc_finished = NULL;
	int has_new_req = 0;
	ktime_t start, end;
	struct ingenicfb_framedesc *hw_desc;
	// 保持一个描述符安全距离，防止软件更新硬件正在使用的描述符.
	if (list_len(&dctrl->comp_desc_done_list) == 1) {
		return 0;
	}

	desc_to_start = list_first_entry_or_null(&dctrl->comp_desc_done_list, struct ingenicfb_framedesc, list);

	if (!desc_to_start) {
		// error?
		//printk("????? no desc_to_start ????? why ????\n");
		//printk(" --- queued: %d, done: %d\n", list_len(&dctrl->comp_desc_queued_list), list_len(&dctrl->comp_desc_done_list));
		return 0;
	}

	if (!list_empty(&dctrl->comp_desc_queued_list)) {
		last_queued_desc = list_last_entry(&dctrl->comp_desc_queued_list, struct ingenicfb_framedesc, list);
	}

	hw_desc = (struct ingenicfb_framedesc *)KSEG1ADDR(reg_read(dctrl, DC_FRM_CFG_ADDR));

	if (desc_to_start->index == hw_desc->index || (desc_to_start->index + 1) % DPU_COMP_MAX_FRAMEDESC == hw_desc->index) {
		printk("- may error -desc_to_start: %x, hw desc: %x\n", desc_to_start->index,  hw_desc->index);
	}

	dpu_comp_framedesc_setup_frame(dctrl, last_queued_desc, desc_to_start);

	start = ktime_get();

	/*Setup Each Layer from layer request!*/
	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		struct comp_layer_request *new_req = list_first_entry_or_null(&dctrl->comp_layers_req_queued_list[i], struct comp_layer_request, list);
		struct ingenicfb_layerdesc *layer_desc;
		if (!new_req) {
			continue;
		}

		//修改desc_to_start 描述符的layer i 部分。 其他layer保持不变？如何保持不变？
		layer_desc = dctrl->layerdesc[desc_to_start->index][i];

		dpu_comp_layerdesc_setup_layer(dctrl, layer_desc, &new_req->cfg);

		dpu_comp_framedesc_setup_layer(dctrl, desc_to_start, layer_desc, i, &new_req->cfg);
		// 只修改帧描述符中的一个地址，其他的保持不变,
		//desc_to_start->Layer0CfgAddr = dctr->layerdesc_phys[i];

		list_del(&new_req->list);
		list_add_tail(&new_req->list, &dctrl->comp_layers_req_free_list[i]);
		has_new_req = 1;
#if 0
		/*TODO: Try to Merge request into one request.*/
		struct comp_layer_request *new_req = list_first_entry_or_null(&dctrl->comp_layers_req_queued_list[i], struct comp_layer_request, list);

		if (!new_req) {
			continue;
		}

		list_for_each_entry(new_req, &dctrl->comp_layers_req_free_list[i], struct comp_layer_request, list) {

			struct comp_layer_request *next_req = list_next_entry(new_req, list);

			if (&next_req->list == &dctrl->comp_layers_req_queued_list[i]) {
				break;
			}

		}
#endif

		//printk("---- setup_layer: %d\n", i);
	}
	lay_scale_en_num = desc_to_start->LayCfgEn.b.lay0_scl_en + desc_to_start->LayCfgEn.b.lay1_scl_en + desc_to_start->LayCfgEn.b.lay2_scl_en + desc_to_start->LayCfgEn.b.lay3_scl_en;
	if (lay_scale_en_num > 2) {
		printk("[Error] Supports up to two layers of scaling");
	}
	end = ktime_get();
	//  printk("-----update layer cause: %lld ms\n", ktime_ms_delta(end, start));

	if (!has_new_req && comp_info->out_mode == COMP_DIRECT_OUT) {
		return 0;
	}

	desc_to_start->FrameNextCfgAddr = desc_to_start->phy_addr;

	// 2. 从done list 删除.
	list_del(&desc_to_start->list);
	// 3. 从comp_desc_queued_list 取出第一个描述符.

#if 1
	desc_finished = list_first_entry_or_null(&dctrl->comp_desc_queued_list, struct ingenicfb_framedesc, list);

	// 6.  将desc_to_start 添加到queued_list末尾。
	list_add_tail(&desc_to_start->list, &dctrl->comp_desc_queued_list);

	//WriteBack Mode.
	if (comp_info->out_mode == COMP_WRITE_BACK_2_ROTATOR || comp_info->out_mode == COMP_WRITE_BACK) {

		// 4. 启动硬件进行合成。
		// DC_FRM_START 开始之后，硬件需要等一段时间才能将描述符搬走，此时直接放在comp_desc_done_list存在风险。
		reg_write(dctrl, DC_FRM_CFG_ADDR, desc_to_start->phy_addr);
		reg_write(dctrl, DC_FRM_CFG_CTRL, DC_FRM_START);

		dctrl->wback_end = 0;

		if (desc_finished) {
			list_del(&desc_finished->list);
			list_add_tail(&desc_finished->list, &dctrl->comp_desc_done_list);
		}
		// 对于写回模式 + 旋转模式上屏显示，每次eod 只处理一个请求.
		return 0;

	} else {
		//Composer Directout Mode.
		if (!desc_finished) {
			//如果没有完成的描述符，说明硬件没有启动。
			//软件需要按照第一次启动composer设置。
			reg_write(dctrl, DC_FRM_CFG_ADDR, desc_to_start->phy_addr);
			reg_write(dctrl, DC_FRM_CFG_CTRL, DC_FRM_START);
		}
	}
#endif
	//printk("--- add to comp_desc_queued_list!\n");

#if 0
	dctrl->wback_end = 0;
	dctrl->comp_info_changed = 0;    //
#endif

	return has_new_req;
}

static int _do_comp_start(struct dpu_ctrl *dctrl)
{
	int cnt = 0;
	int ret = 0;
	ktime_t end;
	ktime_t start = ktime_get();
	u64 diff;
	while (1) {

		ret = _do_comp_start_try(dctrl);
		if (!ret) {
			break;
		}

		cnt++;
	}

	_link_all_queued_desc(dctrl);

	end = ktime_get();

	diff = ktime_ms_delta(end, start);
	//  printk("--- cause: %lld ms, cnt: %d\n", diff, cnt);

	if (cnt) {
		//  printk("--gatherd %d requests!\n", cnt);
	}

	return 0;
}

int dpu_ctrl_comp_start(struct dpu_ctrl *dctrl, int block)
{
	//Nothing ?
	return 0;
}

int dpu_ctrl_comp_wait_done(struct dpu_ctrl *dctrl)
{
	int ret = 0;

	ret = wait_event_interruptible_timeout(dctrl->wq,
	                                       dctrl->wback_end == 1, msecs_to_jiffies(3000));
	if (ret == 0) {
		dev_err(dctrl->dev, "wait composer end timeout\n");
	}
	return ret;
}

static int __should_wait(struct dpu_ctrl *dctrl, int layer, struct buffer_addr_area *next_buffer)
{

	unsigned int layer_pixel_site = reg_read(dctrl, DC_LAY0_SITE + layer * 4);
	int ret = 0;
	int i = 0;

	//printk("---layer: %d-layer_pixel_site: %x, next_buffer->start[%d]: %x, end: %x, queued_list: %d, done_list: %d\n", layer, layer_pixel_site, i, next_buffer->start[i], next_buffer->end[i], list_len(&dctrl->comp_desc_queued_list), list_len(&dctrl->comp_desc_done_list));

	for (i = 0; i < next_buffer->cnt; i++) {
		if (layer_pixel_site >= next_buffer->start[i] && layer_pixel_site <= next_buffer->end[i]) {
			//      printk("----layer_pixel_site: %x, next_buffer->start[%d]: %x, end: %x, queued_list: %d, done_list: %d\n", layer_pixel_site, i, next_buffer->start[i], next_buffer->end[i], list_len(&dctrl->comp_desc_queued_list), list_len(&dctrl->comp_desc_done_list));
			ret = 1;
			break;
		}
	}

	return ret;
}

int dpu_ctrl_comp_wait_for_vsync(struct dpu_ctrl *dctrl, int layer, struct buffer_addr_area *next_area)
{
	int ret = 0;
	struct timespec64 time_start, time_end;
	unsigned int interval_in_us;
	unsigned int interval_in_ms;
	unsigned int max_ms;
	struct fb_videomode *mode = dctrl->active_video_mode;

	int vblank = dctrl->vblank_count;
	int wait = 0;
	if (!dctrl->refresh_time) {
		if (mode->refresh <= 0) {
			max_ms = 100;
		} else {
			max_ms = (3 * 1000) / mode->refresh;
		}
		dctrl->refresh_time = max_ms;
	}

	ktime_get_real_ts64(&time_start);
	do {
		//vblank = dctrl->vblank_count;
		ret = wait_event_interruptible_timeout(dctrl->wq,
		                                       vblank != dctrl->vblank_count, msecs_to_jiffies(3000));
		if (ret == 0) {
			dev_err(dctrl->dev, "wait vblank timeout\n");
		}

		wait = __should_wait(dctrl, layer, next_area);
		if (!wait) {
			break;
		}
		ktime_get_real_ts64(&time_end);
		interval_in_us = timeval_sub_to_us(time_end, time_start);
		interval_in_ms = interval_in_us / 1000;
		if (interval_in_ms > dctrl->refresh_time) {
			break;
		}

		schedule();

	} while (1);
	return ret;
}

static int _dpu_ctrl_comp_try_start(struct dpu_ctrl *dctrl)
{
	struct comp_setup_info *comp_info = &dctrl->comp_info;
	int ret = 0;
	unsigned int com_cfg = 0;
#ifdef CONFIG_MMU_NOTIFIER
	dctrl_tlb_configure(dctrl);
	dctrl_tlb_enable(dctrl);

	/*Flush cache if tlb en.*/
	dctrl_flush_cache(dctrl);
#endif
	if (dctrl->tlb_disable == true) {
		reg_write(dctrl, DC_TLB_GLBC, dctrl->tlb_disable_ch);
		dctrl->tlb_disable = false;
	}

	dctrl->comp_fps++;

	/*切换到comp 之前，先停止rdma.*/
	if ((comp_info->out_mode == COMP_DIRECT_OUT) && (dctrl->chan == DATA_CH_RDMA)) {
		/*stop rdma ??*/
		if (reg_read(dctrl, DC_ST) & DC_SRD_WORKING) {
			dpu_ctrl_rdma_stop(dctrl, GEN_STOP);
		}
	}

	//out_mode
	//COMP_DIRECT_OUT   直通模式，EOD更新合成请求队列
	//COMP_WRITE_BACK   写回模式，需要阻塞等待执行完成。与显示无关。
	//COMP_WRITE_BACK_2_ROTATOR 间接直通模式，EOD 更新合成请求队列
	//COMP_WRITE_BACK_2_RDMA 间接直通模式，EOD 更新合成请求队列。

	if (comp_info->out_mode == COMP_WRITE_BACK) {
		_do_comp_start(dctrl);
	} else {
		if (!(reg_read(dctrl, DC_ST) & DC_WORKING)) {
			printk("==============>>>>>>>> DC not Working , restart composer !!!!\n");
			_do_comp_start(dctrl);
		}
	}

	/* 切换显示数据通路到composer */
	if ((comp_info->out_mode == COMP_DIRECT_OUT) && (dctrl->chan == DATA_CH_RDMA)) {

		com_cfg = reg_read(dctrl, DC_COM_CONFIG);
		com_cfg &= ~DC_OUT_SEL_MASK;

		reg_write(dctrl, DC_COM_CONFIG, com_cfg);
		dctrl->chan = DATA_CH_COMP;
	}
#ifdef CONFIG_SOC_X2600
	if (!dctrl->work_in_rdma) {
		// test for rot
		if (dctrl->rot_cfg.rot_angle != 0) {
			com_cfg = reg_read(dctrl, DC_COM_CONFIG);
			com_cfg |= DC_OUT_SEL_ROT;
			com_cfg &= ~DC_OUT_SEL;
			reg_write(dctrl, DC_COM_CONFIG, com_cfg);
			dctrl->chan = DATA_CH_ROT;
		}
		if ((comp_info->out_mode == COMP_DIRECT_OUT) && (dctrl->rot_cfg.rot_angle == 0)) {
			com_cfg = reg_read(dctrl, DC_COM_CONFIG);
			com_cfg &= ~DC_OUT_SEL_MASK;

			reg_write(dctrl, DC_COM_CONFIG, com_cfg);
			dctrl->chan = DATA_CH_COMP;
		}
	}
#endif
	//dump_lcdc_registers(dctrl);

	return ret;
}

int dpu_ctrl_comp_stop(struct dpu_ctrl *dctrl, enum stop_mode mode)
{
	struct ingenicfb_framedesc **framedesc = dctrl->framedesc;
	int ret = 0;
	int i = 0;

	if (!dctrl->support_comp) {
		return 0;
	}
	for (i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		/*当comp stop时，将描述符的LayCfgEn 都设置为0, 防止TLB没有初始化时出错.*/
		framedesc[i]->LayCfgEn.d32 = 0;
	}

	if (mode == QCK_STOP) {
		reg_write(dctrl, DC_CTRL, DC_QCK_STP_CMP);
		wait_dc_state(dctrl, DC_DIRECT_WORKING | DC_WRBK_WORKING, 0);
		dctrl->comp_stopped = 1;
	} else {
		dctrl->comp_stopped = 0;
		reg_write(dctrl, DC_CTRL, DC_GEN_STP_CMP);
		ret = wait_event_interruptible_timeout(dctrl->wq,
		                                       dctrl->comp_stopped == 1, msecs_to_jiffies(3000));
		if (ret == 0) {
			dev_err(dctrl->dev, "dpu composer gen stop timeout!!!\n");
		}
	}
#ifdef CONFIG_MMU_NOTIFIER
	dctrl_tlb_disable(dctrl);
#endif
	dctrl->comp_fps = 0;

	return ret > 0 ? 0 : -EINVAL;
}

/* 1. slcd/tft 接口初始化.
 * 2. 设置pixclk.
 * */
int dpu_ctrl_setup(struct dpu_ctrl *dctrl)
{
	struct lcd_panel *panel = dctrl->panel;
	unsigned int intc = 0;
	int ret = 0;

	disp_common_init(dctrl, dctrl->panel);
	common_cfg_init(dctrl);

	reg_write(dctrl, DC_CLR_ST, 0x01FFFFFE);

	switch (panel->lcd_type) {
		case LCD_TYPE_TFT:
		case LCD_TYPE_LVDS_JEIDA:
		case LCD_TYPE_LVDS_VESA:
			ingenicfb_tft_set_par(dctrl);
			break;
		case LCD_TYPE_MIPI_TFT:
			ingenicfb_tft_set_par(dctrl);
			dctrl->dsi->master_ops->mode_cfg(dctrl->dsi, 1);
			break;
		case LCD_TYPE_SLCD:
			ingenicfb_slcd_set_par(dctrl);
			break;
		case LCD_TYPE_MIPI_SLCD:
			ingenicfb_slcd_set_par(dctrl);
			dctrl->dsi->master_ops->mode_cfg(dctrl->dsi, 0);
			break;
	}

	/*       disp end      gen_stop    tft_under    frm_start    frm_end     wback over  GEN_STOP_SRD*/
	//  intc = DC_EOD_MSK | DC_SDA_MSK | DC_UOT_MSK | DC_SOC_MSK | DC_EOF_MSK | DC_OOW_MSK | DC_SSA_MSK;

	//intc = DC_EOD_MSK | DC_SDA_MSK | DC_UOT_MSK | DC_SOC_MSK | DC_OOW_MSK | DC_EOW_MSK | DC_SOS_MSK | DC_STOP_SRD_ACK;
	intc = DC_EOD_MSK | DC_SDA_MSK | DC_UOT_MSK | DC_OOW_MSK | DC_EOW_MSK | DC_SOS_MSK;
	reg_write(dctrl, DC_INTC, intc);

#ifdef CONFIG_MMU_NOTIFIER
	/* tlb init disabled. */
	dctrl_tlb_configure(dctrl);
#endif
#ifdef CONFIG_SOC_X2600
	if (panel->lcd_type == LCD_TYPE_LVDS_JEIDA || panel->lcd_type == LCD_TYPE_LVDS_VESA) {
		lvds_rx_enable(dctrl->dsi);
	}
#endif
	return ret;
}

/* 申请和释放rdma的描述符. */
static int dpu_ctrl_alloc_rdma_desc(struct dpu_ctrl *dctrl)
{
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	dma_addr_t addr_phy;
	uint8_t *addr = 0;
	int i = 0;

	dctrl->sreadesc_size = 0;
	size = sizeof(struct ingenicfb_sreadesc);

	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * CONFIG_FB_INGENIC_NR_FRAMES * 2;
	//  addr = devm_kzalloc(dctrl->dev,alloc_size,GFP_KERNEL);
	//  addr_phy = virt_to_phys(addr);
	//  addr = CKSEG1ADDR(addr);

	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);

	if (IS_ERR_OR_NULL(addr)) {
		dev_err(dctrl->dev, "Failed to alloc dma desc for rdma!\n");
		return -ENOMEM;
	}
	dma_cache_wback_inv((unsigned long)addr, alloc_size);
	memset(addr, 0, alloc_size);

	for (i = 0; i < CONFIG_FB_INGENIC_NR_FRAMES * 2; i++) {
		dctrl->sreadesc[i] =
		    (struct ingenicfb_sreadesc *)(addr + i * size);
		dctrl->sreadesc_phys[i] = addr_phy + i * size;
	}

	dctrl->sreadesc_size = alloc_size;

	dev_info(dctrl->dev, "rdma_desc @ 0x%lx size: %d\n", (unsigned long)dctrl->sreadesc[0], dctrl->sreadesc_size);

	return 0;
}

static int dpu_ctrl_release_rdma_desc(struct dpu_ctrl *dctrl)
{
	if (dctrl->sreadesc_size) {
		//dma_free_coherent(dctrl->dev, dctrl->sreadesc_size, dctrl->sreadesc[0], dctrl->sreadesc_phys[0]);
		dctrl->sreadesc_size = 0;
	}

	return 0;
}

/* 申请和释放composer的描述符
 * dpu控制器所拥有的描述符是固定的，当启动时，每个描述的内容需要由外部填充.
 * 例如:
 *  1. 当用户调用composer进行合成时，可以先修改上层的内容，最后再反馈到实际的buffer.
 * */
static int dpu_ctrl_alloc_comp_desc(struct dpu_ctrl *dctrl)
{
	unsigned int size = 0;
	unsigned int alloc_size = 0;
	dma_addr_t addr_phy = 0;
	uint8_t *addr = 0;
	int ret = 0;
	int i = 0, j = 0;

	/*Composer Frame desc. */
	dctrl->framedesc_size = 0;
	size = sizeof(struct ingenicfb_framedesc);

	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * DPU_COMP_MAX_FRAMEDESC;
	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);
	//addr_phy = virt_to_phys(addr);
	//addr = CKSEG1ADDR(addr);

	if (IS_ERR_OR_NULL(addr)) {
		dev_err(dctrl->dev, "Failed to alloc dma desc for rdma!\n");
		return -ENOMEM;
	}
	dma_cache_wback_inv((unsigned long)addr, alloc_size);
	memset(addr, 0, alloc_size);

	INIT_LIST_HEAD(&dctrl->comp_desc_queued_list);
	INIT_LIST_HEAD(&dctrl->comp_desc_done_list);

	for (i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		dctrl->framedesc[i] =
		    (struct ingenicfb_framedesc *)(addr + i * size);
		dctrl->framedesc_phys[i] = addr_phy + i * size;

		dctrl->framedesc[i]->index = i;
		dctrl->framedesc[i]->phy_addr = addr_phy + i * size;

		//printk("===========>>>>>dctrl->framedesc[%d]->phy_addr: %x\n", i, dctrl->framedesc[i]->phy_addr);

		INIT_LIST_HEAD(&dctrl->framedesc[i]->list);
		list_add_tail(&dctrl->framedesc[i]->list, &dctrl->comp_desc_done_list);

	}

	dctrl->framedesc_size = alloc_size;

	/*Composer Layer Desc*/
	size = sizeof(struct ingenicfb_layerdesc);
	size = ALIGN(size, DESC_ALIGN);
	alloc_size = size * DPU_COMP_MAX_FRAMEDESC * DPU_SUPPORT_MAX_LAYERS;

	//addr = devm_kzalloc(dctrl->dev, alloc_size,GFP_KERNEL);
	addr = dma_alloc_coherent(dctrl->dev, alloc_size, &addr_phy, GFP_KERNEL);
	//addr_phy = virt_to_phys(addr);

	//addr = CKSEG1ADDR(addr);
	if (addr == NULL) {
		ret = -ENOMEM;
		goto err_layer_desc;
	}
	dma_cache_wback_inv((unsigned long)addr, alloc_size);
	memset(addr, 0, alloc_size);

	for (i = 0; i < DPU_COMP_MAX_FRAMEDESC; i++) {
		for (j = 0; j < DPU_SUPPORT_MAX_LAYERS; j++) {
			dctrl->layerdesc[i][j] = (struct ingenicfb_layerdesc *)
			                         (addr + j * size + i * size * DPU_SUPPORT_MAX_LAYERS);
			dctrl->layerdesc_phys[i][j] =
			    addr_phy + j * size + i * size * DPU_SUPPORT_MAX_LAYERS;

			dctrl->layerdesc[i][j]->phy_addr = dctrl->layerdesc_phys[i][j];

		}
	}

	for (i = 0; i < DPU_SUPPORT_MAX_LAYERS; i++) {
		int j = 0;

		INIT_LIST_HEAD(&dctrl->comp_layers_req_free_list[i]);
		INIT_LIST_HEAD(&dctrl->comp_layers_req_queued_list[i]);

		for (j = 0; j < DPU_MAX_REQUESTS_PER_LAYER; j++) {
			list_add_tail(&dctrl->layer_requests[i][j].list, &dctrl->comp_layers_req_free_list[i]);
		}
	}

	dctrl->layerdesc_size = alloc_size;

	dctrl->comp_stopped = 1;
	dctrl->srd_stopped = 0;


	dev_info(dctrl->dev, "composer framedesc @ 0x%08lx size %d\n", (unsigned long)dctrl->framedesc[0], dctrl->framedesc_size);
	dev_info(dctrl->dev, "composer layerdesc @ 0x%08lx size %d\n", (unsigned long)dctrl->layerdesc[0][0], dctrl->layerdesc_size);
	return ret;

err_layer_desc:
	//dma_free_coherent(dctrl->dev, dctrl->framedesc_size, dctrl->framedesc[0], dctrl->framedesc_phys[0]);
	return ret;
}

static int dpu_ctrl_release_comp_desc(struct dpu_ctrl *dctrl)
{
#if 0
	if (dctrl->framedesc_size) {
		dma_free_coherent(dctrl->dev, dctrl->framedesc_size, dctrl->framedesc[0], dctrl->framedesc_phys[0]);
	}

	if (dctrl->layerdesc_size) {
		dma_free_coherent(dctrl->dev, dctrl->layerdesc_size, dctrl->layerdesc[0][0], dctrl->layerdesc_phys[0][0]);
	}
#endif
	return 0;
}

int dpu_ctrl_try_rdma_release(struct dpu_ctrl *dctrl)
{
	unsigned int stat = reg_read(dctrl, DC_ST);
	int ret = 0;
	/*如果当前rdma没有使用，可以尝试释放rdma申请的dma描述符资源，成功返回0.*/
	if ((dctrl->srd_stopped == 1) || !(stat & DC_SRD_WORKING)) {
		dpu_ctrl_release_rdma_desc(dctrl);
	} else {
		ret = -EBUSY;
	}

	return ret;
}

/*
 *  初始化LCD相关数据结构资源. 与具体硬件的操作无关.
 *
 * */
int dpu_ctrl_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel)
{
	int ret = 0;

	dev_info(dctrl->dev, "dpu_ctrl_init start!\n");

	spin_lock_init(&dctrl->irq_lock);

	sprintf(dctrl->clk_name, "gate_lcd");
	sprintf(dctrl->pclk_name, "div_lcd");

	dctrl->refresh_time = 0;
	dctrl->clk = devm_clk_get(dctrl->dev, dctrl->clk_name);
	if (IS_ERR(dctrl->clk)) {
		ret = PTR_ERR(dctrl->clk);
		dev_err(dctrl->dev, "Failed to get lcd clk\n");

		goto err_clk;
	}
	dctrl->pclk = devm_clk_get(dctrl->dev, dctrl->pclk_name);
	if (IS_ERR(dctrl->pclk)) {
		ret = PTR_ERR(dctrl->pclk);
		dev_err(dctrl->dev, "Failed to get lcd pclk\n");

		goto err_pclk;
	}

	dctrl->base = of_iomap(dctrl->dev->of_node, 0);
	if (!dctrl->base) {
		dev_err(dctrl->dev,
		        "Failed to ioremap register memory region\n");
		ret = -EBUSY;
		goto err_put_clk;
	}

	dctrl->irq = platform_get_irq(dctrl->pdev, 0);
	sprintf(dctrl->irq_name, "lcdc%d", dctrl->pdev->id);
	if (devm_request_irq(dctrl->dev, dctrl->irq, dpu_ctrl_irq_handler, 0,
	                     dctrl->irq_name, dctrl)) {
		dev_err(dctrl->dev, "request irq failed\n");
		ret = -EINVAL;
		goto err_irq_req;
	}

	dctrl->panel = panel;

	if (panel->lcd_type == LCD_TYPE_LVDS_JEIDA || panel->lcd_type == LCD_TYPE_LVDS_VESA) {
		ingenic_set_lvds_display_clk(dctrl);
	} else {
		ingenic_set_pixclk(dctrl, PICOS2KHZ(dctrl->active_video_mode->pixclock));
	}

	clk_prepare_enable(dctrl->clk);
	clk_prepare_enable(dctrl->pclk);

	ret = dpu_ctrl_alloc_rdma_desc(dctrl);
	if (ret < 0) {
		goto err_rdma_desc;
	}

	if (dctrl->support_comp) {
		ret = dpu_ctrl_alloc_comp_desc(dctrl);
		if (ret < 0) {
			goto err_comp_desc;
		}
	}

	init_waitqueue_head(&dctrl->wq);

	dpu_ctrl_rdma_stop(dctrl, QCK_STOP);//关闭rdma
	dpu_ctrl_comp_stop(dctrl, QCK_STOP);//关闭composer
	/*DSI 初始化会依赖pixclk.所以要先设置pixclk.**/
	if (panel->lcd_type == LCD_TYPE_MIPI_SLCD || panel->lcd_type == LCD_TYPE_MIPI_TFT) {
#ifdef CONFIG_SOC_X2600
		dctrl->dsi_irq = platform_get_irq(dctrl->pdev, 2);
#else
		dctrl->dsi_irq = platform_get_irq(dctrl->pdev, 1);
#endif
		sprintf(dctrl->dsi_irq_name, "dsi-irq");
		if (devm_request_irq(dctrl->dev, dctrl->dsi_irq, dsi_irq_handler, 0,
		                     dctrl->dsi_irq_name, dctrl)) {
			dev_err(dctrl->dev, "request dsi irq failed\n");
			ret = -EINVAL;
			goto err_irq_req;
		}
		INIT_WORK(&dctrl->dsi_work, ingenic_dsi_worker);

		dctrl->dsi = jzdsi_init(panel->dsi_pdata);
		if (!dctrl->dsi) {
			ret = -EINVAL;
			goto err_iounmap;
		}
		mipi_dsi_sysfs_init(dctrl);
	} else if (panel->lcd_type == LCD_TYPE_LVDS_JEIDA || panel->lcd_type == LCD_TYPE_LVDS_VESA) {
		dctrl->dsi = jz_lvds_init(panel->dsi_pdata, panel->lcd_type);
		if (!dctrl->dsi) {
			ret = -EINVAL;
			goto err_iounmap;
		}
		mipi_dsi_sysfs_init(dctrl);
	}

#ifdef TEST_COMPOSER_2_RDMA
	// 设置RDMA描述符
	struct fb_videomode *mode = dctrl->active_video_mode;
	struct rdma_setup_info *rdma_info = &dctrl->wback_rdma_info;

	rdma_info->nframes = CONFIG_ROT_INGENIC_BUFS;
	rdma_info->format = RDMA_CHAIN_CFG_FORMAT_888;
	rdma_info->color = RDMA_CHAIN_CFG_COLOR_RGB;
	rdma_info->stride = mode->xres;
	rdma_info->continuous = 1;

	int i = 0;
	for (i = 0; i < rdma_info->nframes; i++) {
		rdma_info->vidmem[i] = (unsigned char *)dctrl->rot_wbbuf[i];
		rdma_info->vidmem_phys[i] = dctrl->rot_wbbuf_phy[i];
	}

	printk("##### mode->xres: %x\n", mode->xres);
	dpu_ctrl_rdma_setup(dctrl, rdma_info);
#endif

	dev_info(dctrl->dev, "dpu_ctrl_init success\n");

	return 0;
err_comp_desc:
	dpu_ctrl_release_rdma_desc(dctrl);
err_rdma_desc:
	jzdsi_remove(dctrl->dsi);
err_iounmap:
	clk_disable_unprepare(dctrl->pclk);
	clk_disable_unprepare(dctrl->clk);
	devm_free_irq(dctrl->dev, dctrl->irq, dctrl);
	devm_free_irq(dctrl->dev, dctrl->dsi_irq, dctrl);
err_irq_req:
	iounmap(dctrl->base);
err_put_clk:
	devm_clk_put(dctrl->dev, dctrl->pclk);
err_pclk:
	devm_clk_put(dctrl->dev, dctrl->clk);
err_clk:
	return ret;
}

void dpu_ctrl_exit(struct dpu_ctrl *dctrl)
{
	dpu_ctrl_rdma_stop(dctrl, QCK_STOP);
	dpu_ctrl_comp_stop(dctrl, QCK_STOP);

	dpu_ctrl_release_rdma_desc(dctrl);
	dpu_ctrl_release_comp_desc(dctrl);
	if (dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD || dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT) {
		jzdsi_remove(dctrl->dsi);
		mipi_dsi_sysfs_exit(dctrl);
	}
	iounmap(dctrl->base);
	devm_free_irq(dctrl->dev, dctrl->irq, dctrl);
	devm_free_irq(dctrl->dev, dctrl->dsi_irq, dctrl);
	devm_clk_put(dctrl->dev, dctrl->pclk);
	devm_clk_put(dctrl->dev, dctrl->clk);
}

int dpu_ctrl_suspend(struct dpu_ctrl *dctrl)
{
	struct ingenicfb_framedesc *comp_queued_list = NULL;

	if (dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD || dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT) {
		dctrl->dsi->master_ops->set_blank_mode(dctrl->dsi, FB_BLANK_POWERDOWN);
	}

	//TODO: Lock??
	if (dctrl->chan == DATA_CH_RDMA) {
		dpu_ctrl_rdma_stop(dctrl, QCK_STOP);
#ifdef CONFIG_SOC_X2600
	} else if (dctrl->chan == DATA_CH_ROT) {
		rot_quick_stop();
		dpu_ctrl_comp_stop(dctrl, QCK_STOP);
#endif
	} else {
		dpu_ctrl_comp_stop(dctrl, QCK_STOP);
	}

	memset(&dctrl->saved_comp_info, 0, sizeof(struct comp_setup_info));
	while (1) {
		comp_queued_list = list_first_entry_or_null(&dctrl->comp_desc_queued_list, struct ingenicfb_framedesc, list);
		if (comp_queued_list == NULL) {
			break;
		}
		list_del(&comp_queued_list->list);
		list_add_tail(&comp_queued_list->list, &dctrl->comp_desc_done_list);
	}

	clk_disable_unprepare(dctrl->clk);
	clk_disable_unprepare(dctrl->pclk);

	return 0;
}

int dpu_ctrl_resume(struct dpu_ctrl *dctrl)
{
	clk_prepare_enable(dctrl->pclk);
	clk_prepare_enable(dctrl->clk);

	/*!! dpu must not enabled before dsi UNBLANK.*/
	if (dctrl->panel->lcd_type == LCD_TYPE_MIPI_SLCD || dctrl->panel->lcd_type == LCD_TYPE_MIPI_TFT) {
		dctrl->dsi->master_ops->set_blank_mode(dctrl->dsi, FB_BLANK_UNBLANK);
	}

	dpu_ctrl_rdma_stop(dctrl, QCK_STOP);
	dpu_ctrl_comp_stop(dctrl, QCK_STOP);

	//TODO: lock??
	dpu_ctrl_setup(dctrl);

	if (dctrl->chan == DATA_CH_RDMA) {
		dpu_ctrl_rdma_setup(dctrl, &dctrl->rdma_info);
		dpu_ctrl_rdma_start(dctrl);
#ifdef CONFIG_SOC_X2600
	} else if (dctrl->chan == DATA_CH_ROT) {
		rotator_cfg_init(&dctrl->rot_cfg);
		dpu_ctrl_comp_setup(dctrl, &dctrl->comp_info);
		dpu_ctrl_comp_start(dctrl, 0);
#endif
	} else {
		dpu_ctrl_comp_setup(dctrl, &dctrl->comp_info);
		dpu_ctrl_comp_start(dctrl, 0);
	}

	return 0;
}

static void ingenic_dsi_worker(struct work_struct *work)
{
	struct dpu_ctrl *dctrl = container_of(work, struct dpu_ctrl, dsi_work);
	struct dsi_device *dsi = dctrl->dsi;
	if (atomic_read(&dsi->need_recovery)) {
		dpu_ctrl_suspend(dctrl);
		dpu_ctrl_resume(dctrl);
		atomic_set(&dsi->need_recovery, 0);
	}
}

