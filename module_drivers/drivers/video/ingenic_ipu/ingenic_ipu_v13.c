/*
 * Copyright (c) 2015 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Input file for Ingenic IPU driver
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
#include <linux/version.h>
#include <linux/dma-map-ops.h>
#include "ingenic_regs_v13.h"
#include "ingenic_ipu_v13.h"
#include "ingenic_drawbox.h"
#include "ipu_dmabuf.h"
// #define DEBUG
#ifdef  DEBUG
	static int debug_ipu = 1;

	#define IPU_DEBUG(format, ...) { if (debug_ipu) printk(format, ## __VA_ARGS__);}
#else
	#define IPU_DEBUG(format, ...) do{ } while(0)
#endif

#define IPU_BUF_SIZE (1024 * 1024 * 2)

struct ipu_reg_struct jz_ipu_regs_name[] = {
	{"IPU_FM_CTRL", IPU_FM_CTRL},
	{"IPU_STATUS", IPU_STATUS},
	{"IPU_D_FMT", IPU_D_FMT},
	{"IPU_Y_ADDR", IPU_Y_ADDR},
	{"IPU_U_ADDR", IPU_U_ADDR},
	{"IPU_IN_FM_GS", IPU_IN_FM_GS},
	{"IPU_Y_STRIDE", IPU_Y_STRIDE},
	{"IPU_UV_STRIDE", IPU_UV_STRIDE},
	{"IPU_OUT_ADDR", IPU_OUT_ADDR},
	{"IPU_OUT_STRIDE", IPU_OUT_STRIDE},
	{"IPU_OUT_V_ADDR", IPU_OUT_V_ADDR},
	{"IPU_OUT_V_STRIDE", IPU_OUT_V_STRIDE},
	{"IPU_REG_CTRL", IPU_REG_CTRL},
	{"IPU_TRIGGER", IPU_TRIGGER},
	{"IPU_GLB_CTRL", IPU_GLB_CTRL},

	{"IPU_NV_OUT_ADDR", IPU_NV_OUT_ADDR},
	{"IPU_NV_OUT_STRIDE", IPU_NV_OUT_STRIDE},
	{"IPU_OSD_IN_CH0_Y_ADDR", IPU_OSD_IN_CH0_Y_ADDR},
	{"IPU_OSD_IN_CH0_Y_STRIDE", IPU_OSD_IN_CH0_Y_STRIDE},
	{"IPU_OSD_IN_CH0_UV_ADDR", IPU_OSD_IN_CH0_UV_ADDR},
	{"IPU_OSD_IN_CH0_UV_STRIDE", IPU_OSD_IN_CH0_UV_STRIDE},
	{"IPU_OSD_IN_CH1_Y_ADDR", IPU_OSD_IN_CH1_Y_ADDR},
	{"IPU_OSD_IN_CH1_Y_STRIDE", IPU_OSD_IN_CH1_Y_STRIDE},
	{"IPU_OSD_IN_CH1_UV_ADDR", IPU_OSD_IN_CH1_UV_ADDR},
	{"IPU_OSD_IN_CH1_UV_STRIDE", IPU_OSD_IN_CH1_UV_STRIDE},
	{"IPU_OSD_IN_CH2_Y_ADDR", IPU_OSD_IN_CH2_Y_ADDR},
	{"IPU_OSD_IN_CH2_Y_STRIDE", IPU_OSD_IN_CH2_Y_STRIDE},
	{"IPU_OSD_IN_CH2_UV_ADDR", IPU_OSD_IN_CH2_UV_ADDR},
	{"IPU_OSD_IN_CH2_UV_STRIDE", IPU_OSD_IN_CH2_UV_STRIDE},
	{"IPU_OSD_IN_CH3_Y_ADDR", IPU_OSD_IN_CH3_Y_ADDR},
	{"IPU_OSD_IN_CH3_Y_STRIDE", IPU_OSD_IN_CH3_Y_STRIDE},
	{"IPU_OSD_IN_CH3_UV_ADDR", IPU_OSD_IN_CH3_UV_ADDR},
	{"IPU_OSD_IN_CH3_UV_STRIDE", IPU_OSD_IN_CH3_UV_STRIDE},
	{"IPU_OSD_CH0_GS", IPU_OSD_CH0_GS},
	{"IPU_OSD_CH1_GS", IPU_OSD_CH1_GS},
	{"IPU_OSD_CH2_GS", IPU_OSD_CH2_GS},
	{"IPU_OSD_CH3_GS", IPU_OSD_CH3_GS},
	{"IPU_OSD_CH0_POS", IPU_OSD_CH0_POS},
	{"IPU_OSD_CH1_POS", IPU_OSD_CH1_POS},
	{"IPU_OSD_CH2_POS", IPU_OSD_CH2_POS},
	{"IPU_OSD_CH3_POS", IPU_OSD_CH3_POS},
	{"IPU_OSD_CH0_PARA", IPU_OSD_CH0_PARA},
	{"IPU_OSD_CH1_PARA", IPU_OSD_CH1_PARA},
	{"IPU_OSD_CH2_PARA", IPU_OSD_CH2_PARA},
	{"IPU_OSD_CH3_PARA", IPU_OSD_CH3_PARA},
	{"IPU_OSD_CH_BK_PARA", IPU_OSD_CH_BK_PARA},
	{"IPU_OSD_CH0_BAK_ARGB", IPU_OSD_CH0_BAK_ARGB},
	{"IPU_OSD_CH1_BAK_ARGB", IPU_OSD_CH1_BAK_ARGB},
	{"IPU_OSD_CH2_BAK_ARGB", IPU_OSD_CH2_BAK_ARGB},
	{"IPU_OSD_CH3_BAK_ARGB", IPU_OSD_CH3_BAK_ARGB},
	{"IPU_OSD_CH_B_BAK_ARGB", IPU_OSD_CH_B_BAK_ARGB},
	{"IPU_CH0_CSC_C0_COEF", IPU_CH0_CSC_C0_COEF},
	{"IPU_CH0_CSC_C1_COEF", IPU_CH0_CSC_C1_COEF},
	{"IPU_CH0_CSC_C2_COEF", IPU_CH0_CSC_C2_COEF},
	{"IPU_CH0_CSC_C3_COEF", IPU_CH0_CSC_C3_COEF},
	{"IPU_CH0_CSC_C4_COEF", IPU_CH0_CSC_C4_COEF},
	{"IPU_CH0_CSC_OFSET_PARA", IPU_CH0_CSC_OFSET_PARA},
	{"IPU_CH1_CSC_C0_COEF", IPU_CH1_CSC_C0_COEF},
	{"IPU_CH1_CSC_C1_COEF", IPU_CH1_CSC_C1_COEF},
	{"IPU_CH1_CSC_C2_COEF", IPU_CH1_CSC_C2_COEF},
	{"IPU_CH1_CSC_C3_COEF", IPU_CH1_CSC_C3_COEF},
	{"IPU_CH1_CSC_C4_COEF", IPU_CH1_CSC_C4_COEF},
	{"IPU_CH1_CSC_OFSET_PARA", IPU_CH1_CSC_OFSET_PARA},
	{"IPU_CH2_CSC_C0_COEF", IPU_CH2_CSC_C0_COEF},
	{"IPU_CH2_CSC_C1_COEF", IPU_CH2_CSC_C1_COEF},
	{"IPU_CH2_CSC_C2_COEF", IPU_CH2_CSC_C2_COEF},
	{"IPU_CH2_CSC_C3_COEF", IPU_CH2_CSC_C3_COEF},
	{"IPU_CH2_CSC_C4_COEF", IPU_CH2_CSC_C4_COEF},
	{"IPU_CH2_CSC_OFSET_PARA", IPU_CH2_CSC_OFSET_PARA},
	{"IPU_CH3_CSC_C0_COEF", IPU_CH3_CSC_C0_COEF},
	{"IPU_CH3_CSC_C1_COEF", IPU_CH3_CSC_C1_COEF},
	{"IPU_CH3_CSC_C2_COEF", IPU_CH3_CSC_C2_COEF},
	{"IPU_CH3_CSC_C3_COEF", IPU_CH3_CSC_C3_COEF},
	{"IPU_CH3_CSC_C4_COEF", IPU_CH3_CSC_C4_COEF},
	{"IPU_CH3_CSC_OFSET_PARA", IPU_CH3_CSC_OFSET_PARA},
	{"IPU_RD_ARB_CTL", IPU_RD_ARB_CTL},
	{"IPU_CLK_NUM_ONE_FRA", IPU_CLK_NUM_ONE_FRA},
	{"IPU_OSD_LAY_PADDING_ARGB", IPU_OSD_LAY_PADDING_ARGB},
	{"IPU_TEST_1B4", IPU_TEST_1B4},
};

static void reg_bit_set(struct jz_ipu *ipu, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = ipu_reg_read(ipu, offset);
	reg |= bit;
	ipu_reg_write(ipu, offset, reg);
}

static void reg_bit_clr(struct jz_ipu *ipu, int offset, unsigned int bit)
{
	unsigned int reg = 0;
	reg = ipu_reg_read(ipu, offset);
	reg &= ~(bit);
	ipu_reg_write(ipu, offset, reg);
}

static unsigned int _hal_to_ipu_infmt(int hal_fmt, int *isrgb)
{
	unsigned int ipu_fmt = IN_FMT_YUV420;
	int rgb = 0;

	switch (hal_fmt) {
	case HAL_PIXEL_FORMAT_YCbCr_422_SP:
		ipu_fmt = IN_FMT_YUV422;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_SP:
		ipu_fmt = IN_FMT_YUV420;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_P:
		ipu_fmt = IN_FMT_YUV422;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_I:
		ipu_fmt = IN_FMT_YUV422;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_I:
		ipu_fmt = IN_FMT_YUV420;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_420_B:
	case HAL_PIXEL_FORMAT_JZ_YUV_420_B:
		ipu_fmt = IN_FMT_YUV420_B;
		rgb = 0;
		break;
	case HAL_PIXEL_FORMAT_RGBA_5551: //background is not support 5551 format
	case HAL_PIXEL_FORMAT_BGRA_5551:
		ipu_fmt = IN_FMT_RGB_555;
		rgb = 1;
		break;
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_ARGB_8888:
	case HAL_PIXEL_FORMAT_ABGR_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
		ipu_fmt = IN_FMT_RGB_888;
		rgb = 1;
		break;
	case HAL_PIXEL_FORMAT_RGB_565: //background is not support 565 format
		ipu_fmt = IN_FMT_RGB_565;
		rgb = 1;
		break;
	case HAL_PIXEL_FORMAT_NV12:
	case HAL_PIXEL_FORMAT_NV21:
	case HAL_PIXEL_FORMAT_YCbCr_420_P:
	case HAL_PIXEL_FORMAT_JZ_YUV_420_P:
	default:
		ipu_fmt = IN_FMT_YUV420;
		rgb = 0;
		break;
	}
	if (isrgb != 0) {
		*isrgb = rgb;
	}
	return ipu_fmt;
}

static unsigned int _hal_to_ipu_outfmt(int hal_fmt)
{
	unsigned int ipu_fmt = OUT_FMT_RGB888;

	switch (hal_fmt) {
	case HAL_PIXEL_FORMAT_ARGB_8888:
	case HAL_PIXEL_FORMAT_ABGR_8888:
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
		ipu_fmt = OUT_FMT_RGB888;
		break;
	case HAL_PIXEL_FORMAT_RGB_565:  //It is not support
		ipu_fmt = OUT_FMT_RGB565;
		break;
	case HAL_PIXEL_FORMAT_RGBA_5551:  //It is not support
	case HAL_PIXEL_FORMAT_BGRA_5551:
		ipu_fmt = OUT_FMT_RGB555;
		break;
	case HAL_PIXEL_FORMAT_YCbCr_422_I:
		ipu_fmt = OUT_FMT_YUV422;
		break;
	case HAL_PIXEL_FORMAT_NV12:
		ipu_fmt = OUT_FMT_NV12;
		break;
	case HAL_PIXEL_FORMAT_HSV:/*Add HSV*/
		ipu_fmt = OUT_FMT_HSV;
		break;
	case HAL_PIXEL_FORMAT_NV21:
		ipu_fmt = OUT_FMT_NV21;
		break;
	}
	return ipu_fmt;
}

#define IPU_OSD_CH_BK_PARA_GLO_A(p, x) ((p)=(((p)&(~(0xff<<3)))|((x&0xff)<<3)))
#define IPU_OSD_CH_BK_PARA_ALPHA_SEL(p, x) ((p)=(((p)&(~(0x3<<1)))|((x&0x3)<<1)))
static int _ipu_set_bg_route(struct jz_ipu *ipu, struct ipu_param *ipu_param)
{
	int ret = 0;
	unsigned int reg_val = 0;
	unsigned int bgrgb = 0;
	unsigned int srcw = 0;
	unsigned int srch = 0;
	unsigned int outw = 0;
	unsigned int outh = 0;
	unsigned int infmt = 0;
	unsigned int outfmt = 0;

	unsigned int outwstride = 0;

	unsigned int infmt_bits = 0;
	unsigned int outfmt_bits = 0;
	unsigned int outrgb_bits = 0;

	struct ipu_param *ip = ipu_param;

	srcw = ip->bg_w;
	srch = ip->bg_h;
	outw = ip->bg_w;
	outh = ip->bg_h;

	IPU_DEBUG("ipu: %s, bgrgb = %d, srcw = %d, srch = %d, outw = %d, outh = %d \n",
	          __func__, bgrgb, srcw, srch, outw, outh);
	infmt  = _hal_to_ipu_infmt(ip->bg_fmt, &bgrgb);
	outfmt = _hal_to_ipu_outfmt(ip->out_fmt);
	switch (outfmt) {
	case OUT_FMT_RGB888:
		outwstride = outw << 2;
		break;
	case OUT_FMT_RGB555:    //not used
		outwstride = outw << 1;
		break;
	case OUT_FMT_RGB565:    //not used
		outwstride = outw << 1;
		break;
	default:
		outwstride = outw;
		break;
	}
	ipu_reg_write(ipu, IPU_IN_FM_GS, (IN_FM_W(srcw) | IN_FM_H(srch)));

	if (ip->cmd & IPU_CMD_OSD) {
		ipu_reg_write(ipu, IPU_OSD_CH_B_BAK_ARGB, 0xffaa7733);
	}

	switch (ip->out_fmt) {
	case HAL_PIXEL_FORMAT_RGBX_8888:
		outrgb_bits = RGB_OUT_OFT_BGR;
		break;
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_RGB_565:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
	case HAL_PIXEL_FORMAT_RGBA_5551:
	default:
		outrgb_bits = RGB_OUT_OFT_RGB;    //outrgb_bits set 22st bit,this bit in IPU_D_FMT was not used
		break;
	}
	infmt_bits = infmt;
	outfmt_bits = outfmt;     //outfmt_bits set 19st bit,IPU_D_FMT was not used
	if (infmt == IN_FMT_YUV422) {
		infmt_bits |= IN_OFT_Y1UY0V;
	}
	if (outfmt == OUT_FMT_YUV422) {
		outfmt_bits |= YUV_PKG_OUT_OFT_Y1UY0V;
	}

	/* not support RGBA5551 and BGRA5551 format */
	switch (ip->out_fmt) {
	case HAL_PIXEL_FORMAT_ARGB_8888:
		reg_val = 0x0;                      // 4'b0000: ARGB
		break;
	case HAL_PIXEL_FORMAT_ABGR_8888:
		reg_val = 0x28;                     // 4'b0101: ABGR
		break;
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
		reg_val = 0x40;                     // 4'b1000: RGBA
		break;
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
		reg_val = 0x68;                     // 4'b1101: BGRA
		break;
	case HAL_PIXEL_FORMAT_NV12:
		reg_val = 0x2;                      // 3'b010: NV12
		break;
	case HAL_PIXEL_FORMAT_NV21:
		reg_val = 0x3;                      // 3'b011: NV21
		break;
	case HAL_PIXEL_FORMAT_HSV:
		reg_val = 0x4;                      // 3'b100: HSV
		break;
	default:
		dev_err(ipu->dev, "Error: Output data format isn't support.\n");
		return -1;
	}

	ipu_reg_write(ipu, IPU_D_FMT, reg_val);

	if (ip->cmd & IPU_CMD_OSD) {
		if (ip->bg_fmt == HAL_PIXEL_FORMAT_NV12) {
			reg_val = 0x2;
		} else if (ip->bg_fmt == HAL_PIXEL_FORMAT_NV21) {
			reg_val = 0x3;
		} else {
			reg_val = infmt_bits | outfmt_bits | outrgb_bits;
		}
		ipu_reg_write(ipu, IPU_D_FMT, reg_val);
	}
	/*
	* IPU_D_FMT just use 0~6 bit, if IPU_OSD_CH_BK_PARA set CH_BK_PIC_TYPE ,
	* and the IPU was use to do OSD, this register OUT_FMT bits was valueless,
	* the OUT_FMT are automatically set equal to CH_BK_PIC_TYPE value;
	* and if IPU was use to do conversion format, the OUT_FMT bits must be set to the
	* format that you want to conversion.
	*/
	if (infmt == IN_FMT_YUV420_B) {
		__enable_blk_mode();
	}

	if ((ip->out_fmt == HAL_PIXEL_FORMAT_NV12) || (ip->out_fmt == HAL_PIXEL_FORMAT_NV21)) {/*Add NV12 or NV21*/
		ipu_reg_write(ipu, IPU_OUT_STRIDE,  srcw);
		ipu_reg_write(ipu, IPU_NV_OUT_STRIDE, srcw);
	} else if (ip->out_fmt == HAL_PIXEL_FORMAT_HSV) {/* Add HSV */
		ipu_reg_write(ipu, IPU_OUT_STRIDE, srcw);
		ipu_reg_write(ipu, IPU_NV_OUT_STRIDE, srcw * 2);
		ipu_reg_write(ipu, IPU_OUT_V_STRIDE, srcw);
	} else if (ip->out_fmt == HAL_PIXEL_FORMAT_BGRA_8888 || ip->out_fmt == HAL_PIXEL_FORMAT_RGBA_8888) {/* Add ARGB */
		ipu_reg_write(ipu, IPU_OUT_STRIDE, srcw << 2);
	} else {
		ipu_reg_write(ipu, IPU_OUT_STRIDE, outwstride);
	}

	if (infmt == IN_FMT_YUV420_B) {
		ipu_reg_write(ipu, IPU_Y_STRIDE, srcw * 16);
	} else if (ipu_reg_read(ipu, IPU_FM_CTRL) & (1 << 10)) {
		ipu_reg_write(ipu, IPU_Y_STRIDE, srcw * 2);
	} else {
		ipu_reg_write(ipu, IPU_Y_STRIDE, srcw);
	}
	switch (infmt) {
	case IN_FMT_YUV420:
		reg_val = U_STRIDE(srcw) | V_STRIDE(srcw);
		ipu_reg_write(ipu, IPU_UV_STRIDE, reg_val);
		break;
	case IN_FMT_YUV422:
		reg_val = U_STRIDE(srcw / 2) | V_STRIDE(srcw / 2);
		ipu_reg_write(ipu, IPU_UV_STRIDE, reg_val);
		break;
	case IN_FMT_YUV420_B:
		reg_val = U_STRIDE(8 * srcw) | V_STRIDE(8 * srcw);
		ipu_reg_write(ipu, IPU_UV_STRIDE, reg_val);
		break;
	case IN_FMT_YUV444:
		reg_val = U_STRIDE(srcw) | V_STRIDE(srcw);
		ipu_reg_write(ipu, IPU_UV_STRIDE, reg_val);
		break;
	case IN_FMT_YUV411:
		reg_val = U_STRIDE(srcw / 4) | V_STRIDE(srcw / 4);
		ipu_reg_write(ipu, IPU_UV_STRIDE, reg_val);
		break;
	default:
		dev_err(ipu->dev, "Error: 222 Input data format isn't support\n");
	}

	reg_val = 0;
	/*select bk ch input fmt*/
	if (bgrgb) {
		switch (ip->bg_fmt) {
		case HAL_PIXEL_FORMAT_ARGB_8888:
			reg_val |= CH_BK_ARGB_TYPE_ARGB;
			break;
		case HAL_PIXEL_FORMAT_ABGR_8888:
			reg_val |= CH_BK_ARGB_TYPE_ABGR;
			break;
		case HAL_PIXEL_FORMAT_RGBA_8888:
			reg_val |= CH_BK_ARGB_TYPE_RGBA;
			break;
		case HAL_PIXEL_FORMAT_BGRA_8888:
			reg_val |= CH_BK_ARGB_TYPE_BGRA;
			break;
		//          case HAL_PIXEL_FORMAT_BGRA_5551:   //bg_fmt is not support 5551
		default:
			reg_val |= CH_BK_ARGB_TYPE_BGRA;
			break;
		}
		reg_val |= CH_BK_PIC_TYPE_ARGB;
		//      reg_val |= CH_BK_ARGB_TYPE_BGRA;    //Maybe it's have an other choice, but here I fix it.You can change it.
	} else {
		switch (ip->bg_fmt) {
		case HAL_PIXEL_FORMAT_NV12:
			reg_val |= CH_BK_PIC_TYPE_NV12;
			break;
		case HAL_PIXEL_FORMAT_NV21:
			reg_val |= CH_BK_PIC_TYPE_NV21;
			break;
		default:
			reg_val |= CH_BK_PIC_TYPE_NV12;
			break;
		}
	}
	if (ip->cmd & IPU_CMD_OSD) {
		IPU_DEBUG("ipu: Attention!! Have some OSD CH will ENABLE!!!!\n");
		reg_val |= CH_BK_PREM;
	}

	IPU_DEBUG("ipu: enable background ch!!! \n");
	reg_val |= CH_BK_EN;

	ipu_reg_write(ipu, IPU_OSD_CH_BK_PARA, reg_val);/* input bk format */
	IPU_DEBUG("ipu: Background reg val = 0x%08x\n", ipu_reg_read(ipu, IPU_OSD_CH_BK_PARA));

	return ret;
}

static int _ipu_osd_isrgb(unsigned int fmt)
{
	int isrgb = 0;
	switch (fmt) {
	case HAL_PIXEL_FORMAT_ARGB_8888:
	case HAL_PIXEL_FORMAT_ABGR_8888:
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
	case HAL_PIXEL_FORMAT_BGRA_8888:
	case HAL_PIXEL_FORMAT_BGRX_8888:
		isrgb = 1;
		break;
	case HAL_PIXEL_FORMAT_NV12:
		isrgb = 0;
		break;
	case HAL_PIXEL_FORMAT_HSV:/*Add HSV*/
		isrgb = 0;
		break;
	case HAL_PIXEL_FORMAT_NV21:
		isrgb = 0;
		break;
	case HAL_PIXEL_FORMAT_BGRA_5551:
	case HAL_PIXEL_FORMAT_RGBA_5551:
		isrgb = 2;
		break;
	default:
		isrgb = -1;
		break;
	}
	return isrgb;
}

static int _ipu_set_osd_chx_route(struct jz_ipu *ipu, struct ipu_param *ip, int ch)
{
	unsigned int isrgb = 0;
	unsigned int posx = 0;
	unsigned int posy = 0;
	unsigned int srcw = 0;
	unsigned int srch = 0;
	unsigned int para = 0;
	unsigned int fmt = 0;
	unsigned int bak_argb = 0;
	unsigned int src_stride = 0;

	if ((ch < 0) || (ch > 3)) {
		printk("ipu: osd channel num err ch = %d\n", ch);
		return -1;
	}
	switch (ch) {
	case 0:
		fmt = ip->osd_ch0_fmt;
		posx  = ip->osd_ch0_pos_x;
		posy  = ip->osd_ch0_pos_y;
		srcw  = (ip->bg_w > (ip->osd_ch0_pos_x + ip->osd_ch0_src_w)) ? (ip->osd_ch0_src_w) : (ip->bg_w - ip->osd_ch0_pos_x);
		srch  = (ip->bg_h > (ip->osd_ch0_pos_y + ip->osd_ch0_src_h)) ? (ip->osd_ch0_src_h) : (ip->bg_h - ip->osd_ch0_pos_y);
		para  = ip->osd_ch0_para;
		bak_argb  = ip->osd_ch0_bak_argb;
		src_stride = ip->osd_ch0_src_w;
		break;
	case 1:
		fmt = ip->osd_ch1_fmt;
		posx  = ip->osd_ch1_pos_x;
		posy  = ip->osd_ch1_pos_y;
		srcw  = (ip->bg_w > (ip->osd_ch1_pos_x + ip->osd_ch1_src_w)) ? (ip->osd_ch1_src_w) : (ip->bg_w - ip->osd_ch1_pos_x);
		srch  = (ip->bg_h > (ip->osd_ch1_pos_y + ip->osd_ch1_src_h)) ? (ip->osd_ch1_src_h) : (ip->bg_h - ip->osd_ch1_pos_y);
		para  = ip->osd_ch1_para;
		bak_argb  = ip->osd_ch1_bak_argb;
		src_stride = ip->osd_ch1_src_w;
		break;
	case 2:
		fmt = ip->osd_ch2_fmt;
		posx  = ip->osd_ch2_pos_x;
		posy  = ip->osd_ch2_pos_y;
		srcw  = (ip->bg_w > (ip->osd_ch2_pos_x + ip->osd_ch2_src_w)) ? (ip->osd_ch2_src_w) : (ip->bg_w - ip->osd_ch2_pos_x);
		srch  = (ip->bg_h > (ip->osd_ch2_pos_y + ip->osd_ch2_src_h)) ? (ip->osd_ch2_src_h) : (ip->bg_h - ip->osd_ch2_pos_y);
		para  = ip->osd_ch2_para;
		bak_argb  = ip->osd_ch2_bak_argb;
		src_stride = ip->osd_ch2_src_w;
		break;
	case 3:
		fmt = ip->osd_ch3_fmt;
		posx  = ip->osd_ch3_pos_x;
		posy  = ip->osd_ch3_pos_y;
		srcw  = (ip->bg_w > (ip->osd_ch3_pos_x + ip->osd_ch3_src_w)) ? (ip->osd_ch3_src_w) : (ip->bg_w - ip->osd_ch3_pos_x);
		srch  = (ip->bg_h > (ip->osd_ch3_pos_y + ip->osd_ch3_src_h)) ? (ip->osd_ch3_src_h) : (ip->bg_h - ip->osd_ch3_pos_y);
		para  = ip->osd_ch3_para;
		bak_argb  = ip->osd_ch3_bak_argb;
		src_stride = ip->osd_ch3_src_w;
		break;
	default :
		printk("ipu: osd channel num err ch = %d\n", ch);
		return -1;
		break;
	}

	isrgb = _ipu_osd_isrgb(fmt);
	if (isrgb < 0) {
		printk("ipu: osd fmt err fmt = %d\n", fmt);
		return -1;
	}
	IPU_DEBUG("ipu: ch = %d, isrgb=%d, posx=%d, posy=%d, srcw=%d, srch=%d, para=%d src_stride= %d\n",
	          ch, isrgb, posx, posy, srcw, srch, para, src_stride);
	if (isrgb == 1) {
		ipu_reg_write(ipu, IPU_OSD_IN_CH0_Y_STRIDE + 0x10 * ch, src_stride * 4);
	} else if (isrgb == 2) {       //The format is RGBA5551
		ipu_reg_write(ipu, IPU_OSD_IN_CH0_Y_STRIDE + 0x10 * ch, src_stride * 2);
	} else {
		ipu_reg_write(ipu, IPU_OSD_IN_CH0_Y_STRIDE + 0x10 * ch, src_stride);
		ipu_reg_write(ipu, IPU_OSD_IN_CH0_UV_STRIDE + 0x10 * ch, src_stride);
	}

	if ((srcw > 0) && (srch > 0)) {
		if (isrgb == 1) {
			ipu_reg_write(ipu, IPU_OSD_CH0_GS + 4 * ch, (srcw << 16 << 2) | (srch << 0));
		} else if (isrgb == 2) {   //The format is RGBA1555
			ipu_reg_write(ipu, IPU_OSD_CH0_GS + 4 * ch, (srcw << 16 << 1) | (srch << 0));
		} else {
			ipu_reg_write(ipu, IPU_OSD_CH0_GS + 4 * ch, (srcw << 16) | (srch << 0));
		}
	} else {
		ipu_reg_write(ipu, IPU_OSD_CH0_GS + 4 * ch, (100 << 16) | (100 << 0));
	}

	ipu_reg_write(ipu, IPU_OSD_CH0_POS + 4 * ch, (posx << 16) | (posy << 0));

	if (isrgb > 0) {
		ipu_reg_write(ipu, IPU_CH0_CSC_C0_COEF + 0x18 * ch, (0x106 << 0) | (0x202 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C1_COEF + 0x18 * ch, (0x64 << 0)  | (0x97 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C2_COEF + 0x18 * ch, (0x129 << 0) | (0x1c0 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C3_COEF + 0x18 * ch, (0x1c0 << 0) | (0x177 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C4_COEF + 0x18 * ch, 0x49);
		ipu_reg_write(ipu, IPU_CH0_CSC_OFSET_PARA + 0x18 * ch, (0x10 << 0) | (0x80 << 16));
	} else {
		ipu_reg_write(ipu, IPU_CH0_CSC_C0_COEF + 0x18 * ch, (0x400 << 0) | (0 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C1_COEF + 0x18 * ch, (0x490 << 0) | (0x400 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C2_COEF + 0x18 * ch, (0x190 << 0) | (0x252 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C3_COEF + 0x18 * ch, (0x400 << 0) | (0x81f << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C4_COEF + 0x18 * ch, 0);
		ipu_reg_write(ipu, IPU_CH0_CSC_OFSET_PARA + 0x18 * ch, (0x10 << 0) | (0x80 << 16));
	}
	ipu_reg_write(ipu, IPU_OSD_CH0_BAK_ARGB + 4 * ch, bak_argb);
	if (para > 0) {
		ipu_reg_write(ipu, IPU_OSD_CH0_PARA + 4 * ch, para);
	} else {
		ipu_reg_write(ipu, IPU_OSD_CH0_PARA + 4 * ch, (0x1 | (0x2 << 11) | (0x01 << 1) | (0x32 << 3) | (0 << 19) | (0 << 18)));
	}

	return 0;
}

static int _ipu_set_bg_buffer(struct jz_ipu *ipu, struct ipu_param *ipu_param)
{
	unsigned int bg_y_pbuf = 0;
	unsigned int bg_u_pbuf = 0;
	unsigned int bg_v_pbuf = 0;
	unsigned int out_y_pbuf = 0;
	unsigned int out_uv_pbuf = 0;
	unsigned int out_v_pbuf = 0;
	unsigned int out_nv21_y = 0;
	unsigned int out_nv21_uv = 0;
	unsigned int out_nv12_y = 0;
	unsigned int out_nv12_uv = 0;

	struct ipu_param *ip = ipu_param;

	IPU_DEBUG("ipu: %s:%d \n", __func__, __LINE__);
	if (ipu == NULL) {
		dev_err(ipu->dev, "ipu is NULL\n");
		return -1;
	}

	/* nv12 */
	bg_y_pbuf = ((unsigned int)ip->bg_buf_p);
	bg_u_pbuf = ((unsigned int)ip->bg_buf_p) + ip->bg_w * ip->bg_h;
	bg_v_pbuf = bg_u_pbuf;

	if (ip->cmd & IPU_CMD_CSC) {
		/* OUT HSV */
		out_y_pbuf = ((unsigned int)ip->osd_ch0_buf_p) + (ip->bg_w * ip->bg_h) * 2;
		out_uv_pbuf = ((unsigned int)ip->osd_ch0_buf_p);
		out_v_pbuf = ((unsigned int)ip->osd_ch0_buf_p) + (ip->bg_w * ip->bg_h) * 3;

		/* OUT NV21 */
		out_nv21_y = ((unsigned int)ip->osd_ch0_buf_p);
		out_nv21_uv = ((unsigned int)ip->osd_ch0_buf_p) + (ip->bg_w * ip->bg_h);

		/* OUT NV12 */
		out_nv12_y = ((unsigned int)ip->osd_ch0_buf_p);
		out_nv12_uv = ((unsigned int)ip->osd_ch0_buf_p) + (ip->bg_w * ip->bg_h);
	}

	IPU_DEBUG("ipu: bg_y_pbuf = 0x%08x, bg_u_pbuf = 0x%08x, bg_v_pbuf = 0x%08x\n", bg_y_pbuf, bg_u_pbuf, bg_v_pbuf);
	ipu_reg_write(ipu, IPU_Y_ADDR, bg_y_pbuf);
	ipu_reg_write(ipu, IPU_U_ADDR, bg_u_pbuf);

	/* set out buff */
	if (ip->cmd & IPU_CMD_CSC) {/* out addr */
		if (ip->out_fmt == HAL_PIXEL_FORMAT_HSV) {
			ipu_reg_write(ipu, IPU_OUT_ADDR, out_y_pbuf);
			ipu_reg_write(ipu, IPU_NV_OUT_ADDR, out_uv_pbuf);
			ipu_reg_write(ipu, IPU_OUT_V_ADDR, out_v_pbuf);
		} else if (ip->out_fmt == HAL_PIXEL_FORMAT_NV21) {
			ipu_reg_write(ipu, IPU_OUT_ADDR, out_nv21_y);
			ipu_reg_write(ipu, IPU_NV_OUT_ADDR, out_nv21_uv);
		} else if (ip->out_fmt == HAL_PIXEL_FORMAT_NV12) {
			ipu_reg_write(ipu, IPU_OUT_ADDR, out_nv12_y);
			ipu_reg_write(ipu, IPU_NV_OUT_ADDR, out_nv12_uv);
		} else {
			ipu_reg_write(ipu, IPU_OUT_ADDR, out_uv_pbuf);/* OUT BGRA */
		}
	} else {
		ipu_reg_write(ipu, IPU_OUT_ADDR, bg_y_pbuf);
		ipu_reg_write(ipu, IPU_NV_OUT_ADDR, bg_u_pbuf);
	}

	return 0;
}

static int _ipu_set_osdx_buffer(struct jz_ipu *ipu, struct ipu_param *ipu_param, int ch)
{
	unsigned int osdx_y_pbuf = 0;
	unsigned int osdx_uv_pbuf = 0;
	struct ipu_param *ip = ipu_param;

	IPU_DEBUG("ipu: %s:%d \n", __func__, __LINE__);
	if (ipu == NULL) {
		dev_err(ipu->dev, "ipu is NULL\n");
		return -1;
	}
	if ((ch < 0) || (ch > 3)) {
		printk("ipu: osd channel num err ch = %d\n", ch);
		return -1;
	}
	switch (ch) {
	case 0:
		osdx_y_pbuf = (unsigned int)ip->osd_ch0_buf_p;
		osdx_uv_pbuf = (unsigned int)ip->osd_ch0_buf_p + ip->osd_ch0_src_w * ip->osd_ch0_src_h;
		break;
	case 1:
		osdx_y_pbuf = (unsigned int)ip->osd_ch1_buf_p;
		osdx_uv_pbuf = (unsigned int)ip->osd_ch1_buf_p + ip->osd_ch1_src_w * ip->osd_ch1_src_h;
		break;
	case 2:
		osdx_y_pbuf = (unsigned int)ip->osd_ch2_buf_p;
		osdx_uv_pbuf = (unsigned int)ip->osd_ch2_buf_p + ip->osd_ch2_src_w * ip->osd_ch2_src_h;
		break;
	case 3:
		osdx_y_pbuf = (unsigned int)ip->osd_ch3_buf_p;
		osdx_uv_pbuf = (unsigned int)ip->osd_ch3_buf_p + ip->osd_ch3_src_w * ip->osd_ch3_src_h;
		break;
	default :
		printk("ipu: osd channel num err ch = %d\n", ch);
		return -1;
		break;
	}

	/* set osd chx addr */
	IPU_DEBUG("ipu: ch = %d, osdx_y_pbuf = 0x%08x, osdx_uv_pbuf = 0x%08x\n", ch, osdx_y_pbuf, osdx_uv_pbuf);
	ipu_reg_write(ipu, IPU_OSD_IN_CH0_Y_ADDR + 0x10 * ch, osdx_y_pbuf);
	ipu_reg_write(ipu, IPU_OSD_IN_CH0_UV_ADDR + 0x10 * ch, osdx_uv_pbuf);

	return 0;
}

static int _ipu_dump_regs(struct jz_ipu *ipu)
{
	int i = 0;
	int num = 0;

	if (ipu == NULL) {
		dev_err(ipu->dev, "ipu is NULL!\n");
		return -1;
	}
	printk("----- dump regs -----\n");

	num = sizeof(jz_ipu_regs_name) / sizeof(struct ipu_reg_struct);
	for (i = 0; i < num; i++) {
		printk("ipu_reg: %s: \t0x%08x\r\n", jz_ipu_regs_name[i].name, ipu_reg_read(ipu, jz_ipu_regs_name[i].addr));
	}

	return 0;
}

static void _ipu_dump_param(struct jz_ipu *ipu)
{
	return;
}

static int ipu_dump_info(struct jz_ipu *ipu)
{
	int ret = 0;
	if (ipu == NULL) {
		dev_err(ipu->dev, "ipu is NULL\n");
		return -1;
	}
	printk("ipu: ipu->base: %p\n", ipu->iomem);
	_ipu_dump_param(ipu);
	ret = _ipu_dump_regs(ipu);

	return ret;
}

static int ipu_start(struct jz_ipu *ipu, struct ipu_param *ipu_param)
{
	int ret = 0;
	struct ipu_param *ip = ipu_param;

	if ((ipu == NULL) || (ipu_param == NULL)) {
		dev_err(ipu->dev, "ipu: ipu is NULL or ipu_param is NULL\n");
		return -1;
	}
	if ((0 == (ip->cmd & IPU_CMD_OSD)) & ((0 == (ip->cmd & IPU_CMD_CSC)))) {

		printk("ipu: error cmd 0x%08x\n", ip->cmd);
		ret = -1;
		goto err_cmd;
	}
	IPU_DEBUG("ipu: enter ipu_start %d\n", current->pid);

#ifndef CONFIG_FPGA_TEST
	clk_prepare_enable(ipu->clk);
#endif

	__reset_ipu();

	ret = _ipu_set_bg_route(ipu, ip);
	if (ret) {
		printk("ipu: error _ipu_set_bg_route %d\n", ret);
		goto err_ipu_set_bg_route;
	}
	ret = _ipu_set_bg_buffer(ipu, ip);
	if (ret) {
		printk("ipu: error _ipu_set_bg_buffer %d\n", ret);
		goto err_ipu_set_bg_buffer;
	}

	if (ip->cmd & IPU_CMD_CSC) {
		ipu_reg_write(ipu, IPU_REG_CTRL, 0xffffffff);
		ipu_reg_write(ipu, IPU_OSD_CH0_PARA, 0x010417fa);
		ipu_reg_write(ipu, IPU_OSD_CH_BK_PARA, 0x1417F8);

		ipu_reg_write(ipu, IPU_CH0_CSC_C0_COEF, (0x4ad << 0) | (0 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C1_COEF, (0x669 << 0) | (0x4ad << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C2_COEF, (0x193 << 0) | (0x344 << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C3_COEF, (0x4ad << 0) | (0x81a << 20));
		ipu_reg_write(ipu, IPU_CH0_CSC_C4_COEF, 0);
		ipu_reg_write(ipu, IPU_CH0_CSC_OFSET_PARA, (0x10 << 0) | (0x80 << 16));
	}

	if (ip->cmd & IPU_CMD_OSD0) {
		ret = _ipu_set_osd_chx_route(ipu, ip, 0);
		if (ret) {
			printk("ipu: error _ipu_set_osd_ch0_route %d\n", ret);
			goto err_ipu_set_osd_chx_route;
		}
		ret = _ipu_set_osdx_buffer(ipu, ip, 0);
		if (ret) {
			printk("ipu: error _ipu_set_osdx_buffer %d\n", ret);
			goto err_ipu_set_osdx_buffer;
		}
	}
	if (ip->cmd & IPU_CMD_OSD1) {
		ret = _ipu_set_osd_chx_route(ipu, ip, 1);
		if (ret) {
			printk("ipu: error _ipu_set_osd_ch1_route %d\n", ret);
			goto err_ipu_set_osd_chx_route;
		}
		ret = _ipu_set_osdx_buffer(ipu, ip, 1);
		if (ret) {
			printk("ipu: error _ipu_set_osdx_buffer %d\n", ret);
			goto err_ipu_set_osdx_buffer;
		}
	}
	if (ip->cmd & IPU_CMD_OSD2) {
		ret = _ipu_set_osd_chx_route(ipu, ip, 2);
		if (ret) {
			printk("ipu: error _ipu_set_osd_ch2_route %d\n", ret);
			goto err_ipu_set_osd_chx_route;
		}
		ret = _ipu_set_osdx_buffer(ipu, ip, 2);
		if (ret) {
			printk("ipu: error _ipu_set_osdx_buffer %d\n", ret);
			goto err_ipu_set_osdx_buffer;
		}
	}
	if (ip->cmd & IPU_CMD_OSD3) {
		ret = _ipu_set_osd_chx_route(ipu, ip, 3);
		if (ret) {
			printk("ipu: error _ipu_set_osd_ch3_route %d\n", ret);
			goto err_ipu_set_osd_chx_route;
		}
		ret = _ipu_set_osdx_buffer(ipu, ip, 3);
		if (ret) {
			printk("ipu: error _ipu_set_osdx_buffer %d\n", ret);
			goto err_ipu_set_osdx_buffer;
		}
	}

	__clear_ipu_out_end();
	__ipu_enable_irq();

	ipu_reg_write(ipu, IPU_REG_CTRL, 0xffffffff);

	/* reg_bit_set(ipu, IPU_TRIGGER, 6<<6); */
	/* start ipu */
	__start_ipu();

#ifdef DEBUG
	ipu_dump_info(ipu);
#endif
	IPU_DEBUG("ipu_start\n");

	ret = wait_for_completion_timeout(&ipu->done_ipu, msecs_to_jiffies(2000));
	if (ret < 0) {
		printk("ipu: done_ipu wait_for_completion_timeout err %d\n", ret);
		goto err_ipu_wait_for_done;
	} else if (ret == 0) {
		ret = -1;
		printk("ipu: done_ipu wait_for_completion_timeout timeout %d\n", ret);
		ipu_dump_info(ipu);
		goto err_ipu_wait_for_done;
	} else {
		;
	}

	IPU_DEBUG("ipu: exit ipu_start %d\n", current->pid);

#ifndef CONFIG_FPGA_TEST
	clk_disable_unprepare(ipu->clk);
#endif

	return 0;

err_ipu_wait_for_done:
err_ipu_set_osdx_buffer:
err_ipu_set_bg_buffer:
err_ipu_set_osd_chx_route:
err_ipu_set_bg_route:
#ifndef CONFIG_FPGA_TEST
	clk_disable_unprepare(ipu->clk);
#endif
err_cmd:
	return ret;
}

static long ipu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct ipu_param iparam;
	struct miscdevice *dev = filp->private_data;
	struct jz_ipu *ipu = container_of(dev, struct jz_ipu, misc_dev);

	IPU_DEBUG("ipu: %s pid: %d, tgid: %d file: %p, cmd: 0x%08x\n",
	          __func__, current->pid, current->tgid, filp, cmd);

	if (_IOC_TYPE(cmd) != JZIPU_IOC_MAGIC) {
		dev_err(ipu->dev, "invalid cmd!\n");
		return -EFAULT;
	}

	mutex_lock(&ipu->mutex);

	switch (cmd) {
	case IOCTL_IPU_START:
		if (copy_from_user(&iparam, (void *)arg, sizeof(struct ipu_param))) {
			dev_err(ipu->dev, "copy_from_user error!!!\n");
			ret = -EFAULT;
			break;
		}
		ret = ipu_start(ipu, &iparam);
		if (ret) {
			printk("ipu: error ipu start ret = %d\n", ret);
		}
		break;
	case IOCTL_IPU_GET_PBUFF:
		if (ipu->pbuf.vaddr_alloc == 0) {
			unsigned int size = IPU_BUF_SIZE;
			ipu->pbuf.vaddr_alloc = (unsigned int)kmalloc(size, GFP_KERNEL);
			if (!ipu->pbuf.vaddr_alloc) {
				printk("ipu kmalloc is error\n");
				ret = -ENOMEM;
			}
			memset((void *)(ipu->pbuf.vaddr_alloc), 0x00, size);
			ipu->pbuf.size = size;
			ipu->pbuf.paddr = virt_to_phys((void *)(ipu->pbuf.vaddr_alloc));
			ipu->pbuf.paddr_align = ((unsigned long)(ipu->pbuf.paddr));
		}
		IPU_DEBUG("ipu: %s ipu->pbuf.vaddr_alloc = 0x%08x\nipu->pbuf.vaddr_align = 0x%08x\nipu->pbuf.size = 0x%x\nipu->pbuf.paddr = 0x%08x\n"
		          , __func__, ipu->pbuf.vaddr_alloc, ipu->pbuf.paddr_align, ipu->pbuf.size, ipu->pbuf.paddr);
		if (copy_to_user((void *)arg, &ipu->pbuf, sizeof(struct ipu_buf_info))) {
			dev_err(ipu->dev, "copy_to_user error!!!\n");
			ret = -EFAULT;
		}
		break;
	case IOCTL_IPU_RES_PBUFF:
		if (ipu->pbuf.vaddr_alloc != 0) {
			kfree((void *)ipu->pbuf.vaddr_alloc);
			ipu->pbuf.vaddr_alloc = 0;
			ipu->pbuf.size = 0;
			ipu->pbuf.paddr = 0;
			ipu->pbuf.paddr_align = 0;
		} else {
			dev_warn(ipu->dev, "buffer wanted to free is null\n");
		}
		break;
	case IOCTL_IPU_BUF_LOCK:
		ret = wait_for_completion_interruptible_timeout(&ipu->done_buf, msecs_to_jiffies(2000));
		if (ret < 0) {
			printk("ipu: done_buf wait_for_completion_interruptible_timeout err %d\n", ret);
		} else if (ret == 0) {
			printk("ipu: done_buf wait_for_completion_interruptible_timeout timeout %d\n", ret);
			ret = -1;
			ipu_dump_info(ipu);
		} else {
			ret = 0;
		}
		break;
	case IOCTL_IPU_BUF_UNLOCK:
		complete(&ipu->done_buf);
		break;
	case IOCTL_IPU_BUF_FLUSH_CACHE: {
		struct ipu_flush_cache_para fc;
		if (copy_from_user(&fc, (void *)arg, sizeof(fc))) {
			dev_err(ipu->dev, "copy_from_user error!!!\n");
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
	case IOCTL_IPU_GET_DMA_FD:
		ret = ipu_ioctl_get_dma_fd(ipu->dev, arg); break;
	case IOCTL_IPU_GET_DMA_PHY:
		ret = ipu_ioctl_get_dmabuf_dma_addr(ipu->dev, arg); break;
	case IOCTL_IPU_FLUSH_DMA_FD:
		ret = ipu_ioctl_dmabuf_cache_sync(ipu->dev, arg); break;
	default:
		dev_err(ipu->dev, "invalid command: 0x%08x\n", cmd);
		ret = -EINVAL;
	}

	mutex_unlock(&ipu->mutex);
	return ret;
}

static int ipu_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct miscdevice *dev = filp->private_data;
	struct jz_ipu *ipu = container_of(dev, struct jz_ipu, misc_dev);

	IPU_DEBUG("ipu: %s pid: %d, tgid: %d filp: %p\n",
	          __func__, current->pid, current->tgid, filp);
	mutex_lock(&ipu->mutex);

	mutex_unlock(&ipu->mutex);
	return ret;
}

static int ipu_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct miscdevice *dev = filp->private_data;
	struct jz_ipu *ipu = container_of(dev, struct jz_ipu, misc_dev);

	IPU_DEBUG("ipu: %s  pid: %d, tgid: %d filp: %p\n",
	          __func__, current->pid, current->tgid, filp);
	mutex_lock(&ipu->mutex);

	mutex_unlock(&ipu->mutex);
	return ret;
}

static struct file_operations ipu_ops = {
	.owner = THIS_MODULE,
	.open = ipu_open,
	.release = ipu_release,
	.unlocked_ioctl = ipu_ioctl,
};

static irqreturn_t ipu_irq_handler(int irq, void *data)
{
	struct jz_ipu *ipu;
	unsigned int status;

	IPU_DEBUG("ipu: %s\n", __func__);
	ipu = (struct jz_ipu *)data;
	__ipu_disable_irq();
	status = ipu_reg_read(ipu, IPU_STATUS);
	IPU_DEBUG("----- %s, status= 0x%08x\n", __func__, status);
	/* t10 supports 720P and max width 1280, but t20 supports 1080P and max width
	 * is 1920; the width is limited to 1280 in hardware, so t20 status is 0x4,
	 * and this status doesn't do anything including trigger interrupt,
	 * just give a hint */
	if (status & 0x1) {
		complete(&ipu->done_ipu);
	}
	return IRQ_HANDLED;
}

static int ipu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct jz_ipu *ipu;

	IPU_DEBUG("%s\n", __func__);
	ipu = (struct jz_ipu *)kzalloc(sizeof(struct jz_ipu), GFP_KERNEL);
	if (!ipu) {
		dev_err(&pdev->dev, "alloc jz_ipu failed!\n");
		return -ENOMEM;
	}

	sprintf(ipu->name, "ipu");

	ipu->misc_dev.minor = MISC_DYNAMIC_MINOR;
	ipu->misc_dev.name = ipu->name;
	ipu->misc_dev.fops = &ipu_ops;
	ipu->dev = &pdev->dev;

	mutex_init(&ipu->mutex);
	init_completion(&ipu->done_ipu);
	init_completion(&ipu->done_buf);
	complete(&ipu->done_buf);

	ipu->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ipu->res) {
		dev_err(&pdev->dev, "failed to get dev resources: %d\n", ret);
		ret = -EINVAL;
		goto err_get_platform_res;
	}
	ipu->res = request_mem_region(ipu->res->start,
	                              ipu->res->end - ipu->res->start + 1,
	                              pdev->name);
	if (!ipu->res) {
		dev_err(&pdev->dev, "failed to request regs memory region");
		ret = -EINVAL;
		goto err_get_mem_region;
	}
	ipu->iomem = ioremap(ipu->res->start, resource_size(ipu->res));
	if (!ipu->iomem) {
		dev_err(&pdev->dev, "failed to remap regs memory region: %d\n", ret);
		ret = -EINVAL;
		goto err_ioremap;
	}

	ipu->irq = platform_get_irq(pdev, 0);
	if (request_irq(ipu->irq, ipu_irq_handler, IRQF_SHARED, ipu->name, ipu)) {
		dev_err(&pdev->dev, "request irq failed\n");
		ret = -EINVAL;
		goto err_req_irq;
	}

#ifndef CONFIG_FPGA_TEST
	ipu->clk = devm_clk_get(ipu->dev, "gate_ipu");
	if (IS_ERR(ipu->clk)) {
		dev_err(&pdev->dev, "ipu clk get failed!\n");
		ret = -EINVAL;
		goto err_get_ipu_clk;
	}
#endif

	dev_set_drvdata(&pdev->dev, ipu);

	__reset_ipu();
	ret = misc_register(&ipu->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "register misc device failed!\n");
		goto err_misc_register;
	}

	ret = of_reserved_mem_device_init(ipu->dev);
	if (ret) {
		dev_err(ipu->dev, "Could not get reserved memory!\n");
		goto deregister_misc;
	}

	return 0;

deregister_misc:
	misc_deregister(&ipu->misc_dev);
err_misc_register:
#ifndef CONFIG_FPGA_TEST
	devm_clk_put(ipu->dev, ipu->clk);
err_get_ipu_clk:
#endif
	free_irq(ipu->irq, ipu);
err_req_irq:
	iounmap(ipu->iomem);
err_ioremap:
	release_mem_region(ipu->res->start, ipu->res->end - ipu->res->start + 1);
err_get_platform_res:
err_get_mem_region:
	kfree(ipu);

	return ret;
}

static int ipu_remove(struct platform_device *pdev)
{
	struct jz_ipu *ipu;
	struct resource *res;
	IPU_DEBUG("%s\n", __func__);

	ipu = dev_get_drvdata(&pdev->dev);
	misc_deregister(&ipu->misc_dev);
#ifndef CONFIG_FPGA_TEST
	devm_clk_put(ipu->dev, ipu->clk);
#endif
	res = ipu->res;
	free_irq(ipu->irq, ipu);
	iounmap(ipu->iomem);
	release_mem_region(res->start, res->end - res->start + 1);

	if (ipu->pbuf.vaddr_alloc) {
		kfree((void *)(ipu->pbuf.vaddr_alloc));
		ipu->pbuf.vaddr_alloc = 0;
	}
	if (ipu) {
		kfree(ipu);
	}

	return 0;
}

static const struct of_device_id ingenic_ipu_dt_match[] = {
	{ .compatible = "ingenic,x2500-ipu", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_ipu_dt_match);

static struct platform_driver jz_ipu_driver = {
	.probe  = ipu_probe,
	.remove = ipu_remove,
	.driver = {
		.name = "jz-ipu",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_ipu_dt_match),
	},
};

static int __init ipudev_init(void)
{
	IPU_DEBUG("%s\n", __func__);
	platform_driver_register(&jz_ipu_driver);
	return 0;
}

static void __exit ipudev_exit(void)
{
	IPU_DEBUG("%s\n", __func__);
	platform_driver_unregister(&jz_ipu_driver);
}

module_init(ipudev_init);
module_exit(ipudev_exit);

MODULE_DESCRIPTION("JZ IPU driver");
MODULE_AUTHOR("Ferdinand Jia <bcjia@ingenic.cn>");
MODULE_LICENSE("GPL");
