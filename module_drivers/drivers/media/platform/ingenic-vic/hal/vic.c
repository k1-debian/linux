/*
 * Copyright (C) 2020 Ingenic Semiconductor Co., Ltd.
 *
 * Camera Driver for the Ingenic VIC controller
 *
 */
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>

#include "csi.h"
#include "vic_sensor_info.h"
#include "vic_regs.h"

unsigned long vic_iobase_array[2];

static void vic_start(int index)
{
	/* start vic 控制器 */
	vic_set_bit(index, VIC_CONTROL, VIC_START, 1);
}

void vic_reset(int index)
{
	/* reset vic 控制器 */
	vic_set_bit(index, VIC_CONTROL, VIC_GLB_RST, 1);
}

static void vic_dma_reset(int index)
{
	/* reset dma */
	vic_write_reg(index, VIC_DMA_RESET, 1);
}

static void vic_register_enable(int index)
{
	/* VIC 开始初始化 */
	vic_set_bit(index, VIC_CONTROL, VIC_REG_ENABLE, 1);
#if 0
	int timeout = 3000;
	while (vic_get_bit(index, VIC_CONTROL, VIC_REG_ENABLE)) {
		if (--timeout == 0) {
			printk(KERN_ERR "timeout while wait vic_reg_enable: %x\n", vic_read_reg(info->index, VIC_CONTROL));
			break;
		}
	}
#endif
}

static void vic_data_path_select_route(int index, int route)
{
	if (route) {
		/* ISP Route */
		vic_set_bit(index, VIC_CONTROL_DMA_ROUTE, VC_DMA_ROUTE_dma_out, 0);
		vic_set_bit(index, VIC_CONTROL_TIZIANO_ROUTE, VC_TIZIANO_ROUTE_isp_out, 1);
	} else {
		/* DMA Route */
		vic_set_bit(index, VIC_CONTROL_TIZIANO_ROUTE, VC_TIZIANO_ROUTE_isp_out, 0);
		vic_set_bit(index, VIC_CONTROL_DMA_ROUTE, VC_DMA_ROUTE_dma_out, 1);
	}
}

static void vic_init_dvp_timing(struct vic_hal_info *info)
{
	unsigned long vic_input_dvp = vic_read_reg(info->index, VIC_INPUT_DVP);
	unsigned long yuv_data_order = info->dvp.yuv_data_order;

	if (info->is_y8) {
		vic_input_dvp = set_bit_field(vic_input_dvp, DVP_DATA_FORMAT, 6);
	} else if (info->dvp.data_fmt <= DVP_RAW12) {
		vic_input_dvp = set_bit_field(vic_input_dvp, DVP_DATA_FORMAT, info->dvp.data_fmt);
	} else if (info->dvp.data_fmt == DVP_YUV422) {
		vic_input_dvp = set_bit_field(vic_input_dvp, DVP_DATA_FORMAT, 6);    // YUV422(8bit IO)
	}

	if (info->is_y8) { // 不改变四个raw8的先后顺序
		yuv_data_order = order_1_2_3_4;
	}

	vic_input_dvp = set_bit_field(vic_input_dvp, YUV_DATA_ORDER, yuv_data_order);
	vic_input_dvp = set_bit_field(vic_input_dvp, DVP_TIMING_MODE, info->dvp.timing_mode);
	vic_input_dvp = set_bit_field(vic_input_dvp, HSYNC_POLAR, info->dvp.hsync_polarity);
	vic_input_dvp = set_bit_field(vic_input_dvp, VSYNC_POLAR, info->dvp.vsync_polarity);
	vic_input_dvp = set_bit_field(vic_input_dvp, INTERLACE_EN, info->dvp.img_scan_mode);

	if (info->dvp.gpio_mode == DVP_PA_HIGH_8BIT ||
	    info->dvp.gpio_mode == DVP_PA_HIGH_10BIT) {
		vic_input_dvp = set_bit_field(vic_input_dvp, DVP_RAW_ALIGN, 1);
	} else {
		vic_input_dvp = set_bit_field(vic_input_dvp, DVP_RAW_ALIGN, 0);
	}

	vic_write_reg(info->index, VIC_INPUT_DVP, vic_input_dvp);

	unsigned long vic_ctrl_delay = 0;
	vic_ctrl_delay = set_bit_field(vic_ctrl_delay, VC_CONTROL_delay_hdelay, 1);
	vic_ctrl_delay = set_bit_field(vic_ctrl_delay, VC_CONTROL_delay_vdelay, 1);
	vic_write_reg(info->index, VIC_CONTROL_DELAY, vic_ctrl_delay);
}

static void vic_init_mipi_timing(struct vic_hal_info *info)
{
	unsigned long horizontal_resolution = info->width;

	if (info->is_y8) {
		vic_write_reg(info->index, VIC_INPUT_MIPI, MIPI_YUV422);
		horizontal_resolution /= 2;
	} else {
		vic_write_reg(info->index, VIC_INPUT_MIPI, info->mipi.data_fmt);
	}

	int width_4byte;
	int pixel_wdith;

	switch (info->mipi.data_fmt) {
	case MIPI_RAW8:
		pixel_wdith = 8;
		break;
	case MIPI_RAW10:
		pixel_wdith = 10;
		break;
	case MIPI_RAW12:
		pixel_wdith = 12;
		break;
	default:
		pixel_wdith = 8;
		break;
	}

	/* 每行前有0个无效像素, 每行之后有0个无效像素 */
	width_4byte = ((horizontal_resolution + 0 + 0) * pixel_wdith + 31) / 32;
	vic_write_reg(info->index, MIPI_ALL_WIDTH_4BYTE, width_4byte);

	unsigned long hcrop_ch0 = 0;
	hcrop_ch0 = set_bit_field(hcrop_ch0, MIPI_HCROP_CHO_all_image_width, horizontal_resolution);
	hcrop_ch0 = set_bit_field(hcrop_ch0, MIPI_HCROP_CHO_start_pixel, 0);
	vic_write_reg(info->index, MIPI_HCROP_CH0, hcrop_ch0);

	unsigned long vic_ctrl_delay = 0;
	vic_ctrl_delay = set_bit_field(vic_ctrl_delay, VC_CONTROL_delay_hdelay, 10);
	vic_ctrl_delay = set_bit_field(vic_ctrl_delay, VC_CONTROL_delay_vdelay, 10);
	vic_write_reg(info->index, VIC_CONTROL_DELAY, vic_ctrl_delay);
}

static void vic_init_common_setting(struct vic_hal_info *info)
{
	unsigned long resolution = 0;
	unsigned long horizontal_resolution = info->width;
	unsigned long vorizontal_resolution = info->height;

	/*
	 * sensor输出的图像数据是raw8的，但我们是使用yuv422的格式输入和输出的，
	 * 因为raw8一个像素1个字节 yuv422一个像素占2个字节。
	 * 所以填入寄存器的像素点为raw8像素点的1/2。
	*/
	if (info->is_y8) {
		horizontal_resolution /= 2;
	}

	resolution = set_bit_field(resolution, HORIZONTAL_RESOLUTION, horizontal_resolution);
	resolution = set_bit_field(resolution, VERTICAL_RESOLUTION, vorizontal_resolution);
	vic_write_reg(info->index, VIC_RESOLUTION, resolution);

	int vic_interface = 0;
	switch (info->bus_type) {
	case SENSOR_DATA_BUS_BT656:
		vic_interface = 0;
		break;
	case SENSOR_DATA_BUS_BT601:
		vic_interface = 1;
		break;
	case SENSOR_DATA_BUS_MIPI:
		vic_interface = 2;
		break;
	case SENSOR_DATA_BUS_DVP:
		vic_interface = 3;
		break;
	case SENSOR_DATA_BUS_BT1120:
		vic_interface = 4;
		break;
	default:
		printk(KERN_ERR "vic%d unknown dbus_type: %d\n",  info->index, info->bus_type);
	}
	vic_write_reg(info->index, VIC_INPUT_INTF, vic_interface);
}

static void init_dvp_dma(struct vic_hal_info *info)
{
	unsigned long dma_resolution = 0;
	unsigned long horizontal_resolution = info->width;

	if (info->is_y8) {
		horizontal_resolution /= 2;
	}

	dma_resolution = set_bit_field(dma_resolution, DMA_HORIZONTAL_RESOLUTION, horizontal_resolution);
	dma_resolution = set_bit_field(dma_resolution, DMA_VERTICAL_RESOLUTION, info->height);
	vic_write_reg(info->index, VIC_DMA_RESOLUTION, dma_resolution);

	unsigned int base_mode = 0;
	unsigned int y_stride = 0;
	unsigned int uv_stride = 0;
	unsigned int horizon_time = 0;

	switch (info->dvp.data_fmt) {
	case DVP_RAW8:
	case DVP_RAW10:
	case DVP_RAW12:
		base_mode = 0;
		y_stride = info->width * 2;
		horizon_time = info->width;
		if (info->is_y8) {
			base_mode = 3;
			y_stride = info->width;
		}
		break;

	case DVP_YUV422:
		base_mode = 3;
		y_stride = info->width * 2;
		horizon_time = info->width * 2;
		break;

	default:
		break;
	}

	vic_set_bit(info->index, VIC_IN_HOR_PARA0, HACT_NUM, horizon_time);
	vic_write_reg(info->index, VIC_DMA_Y_CH_LINE_STRIDE, y_stride);
	vic_write_reg(info->index, VIC_DMA_UV_CH_LINE_STRIDE, uv_stride);

	unsigned long dma_configure = vic_read_reg(info->index, VIC_DMA_CONFIGURE);
	dma_configure = set_bit_field(dma_configure, Buffer_number, 2 - 1);
	dma_configure = set_bit_field(dma_configure, Base_mode, base_mode);
	dma_configure = set_bit_field(dma_configure, Yuv422_order, 2);
	vic_write_reg(info->index, VIC_DMA_CONFIGURE, dma_configure);

	/* default DMA Route */
	vic_data_path_select_route(info->index, 0);
}

static void init_mipi_dma(struct vic_hal_info *info)
{
	unsigned long dma_resolution = 0;
	unsigned long horizontal_resolution = info->width;
	unsigned long vorizontal_resolution = info->height;

	if (info->is_y8) {
		horizontal_resolution /= 2;
	}

	dma_resolution = set_bit_field(dma_resolution, DMA_HORIZONTAL_RESOLUTION, horizontal_resolution);
	dma_resolution = set_bit_field(dma_resolution, DMA_VERTICAL_RESOLUTION, vorizontal_resolution);
	vic_write_reg(info->index, VIC_DMA_RESOLUTION, dma_resolution);

	unsigned int base_mode = 0;
	unsigned int y_stride = 0;
	unsigned int uv_stride = 0;
	unsigned int yuv422_order_mode = 0;

	switch (info->mipi.data_fmt) {
	case MIPI_RAW8:
	case MIPI_RAW10:
	case MIPI_RAW12:
		base_mode = 0;
		y_stride = horizontal_resolution * 2;

		if (info->is_y8) {
			base_mode = 3;          /* YUV422 packey */
			yuv422_order_mode = 3;  /* =3, U1Y1V1Y2 */
			y_stride = horizontal_resolution * 2;
		}
		break;
	case MIPI_YUV422:
		base_mode = 3;
		y_stride = info->width * 2;
		break;

	default:
		break;
	}

	vic_write_reg(info->index, VIC_DMA_Y_CH_LINE_STRIDE, y_stride);
	vic_write_reg(info->index, VIC_DMA_UV_CH_LINE_STRIDE, uv_stride);

	unsigned long dma_configure = vic_read_reg(info->index, VIC_DMA_CONFIGURE);
	dma_configure = set_bit_field(dma_configure, Buffer_number, 2 - 1);
	dma_configure = set_bit_field(dma_configure, Base_mode, base_mode);
	dma_configure = set_bit_field(dma_configure, Yuv422_order, yuv422_order_mode);

	vic_write_reg(info->index, VIC_DMA_CONFIGURE, dma_configure);

	/* default DMA Route */
	vic_data_path_select_route(info->index, 0);
}

static void init_dvp_irq(int index)
{
	unsigned long vic_int_mask = 0;

	vic_int_mask = set_bit_field(vic_int_mask, VIC_FRM_START, 1);
	vic_int_mask = set_bit_field(vic_int_mask, VIC_FRM_RST, 1);

	vic_write_reg(index, VIC_INT_CLR, vic_int_mask);
	vic_write_reg(index, VIC_INT_MASK, vic_int_mask);
}

static void init_mipi_irq(int index)
{
	unsigned long vic_int_mask = 0xFFFFF;

	vic_int_mask = set_bit_field(vic_int_mask, VIC_DONE, 0);
	vic_int_mask = set_bit_field(vic_int_mask, MIPI_VCOMP_ERR_CH0, 0);
	vic_int_mask = set_bit_field(vic_int_mask, MIPI_HCOMP_ERR_CH0, 0);
	vic_int_mask = set_bit_field(vic_int_mask, DMA_FRD, 0);
	vic_int_mask = set_bit_field(vic_int_mask, VIC_HVRES_ERR, 0);
	vic_int_mask = set_bit_field(vic_int_mask, VIC_FRM_START, 0);
	vic_int_mask = set_bit_field(vic_int_mask, VIC_FRD, 0);

	vic_write_reg(index, VIC_INT_CLR, vic_int_mask);
	vic_write_reg(index, VIC_INT_MASK, vic_int_mask);
}

void vic_dump_reg(int index)
{
	printk("==========dump vic%d register============\n", index);

	printk("VIC_CONTROL                 : 0x%08x\n", vic_read_reg(index, VIC_CONTROL));
	printk("VIC_RESOLUTION              : 0x%08x\n", vic_read_reg(index, VIC_RESOLUTION));
	printk("VIC_FRM_ECC                 : 0x%08x\n", vic_read_reg(index, VIC_FRM_ECC));
	printk("VIC_INPUT_INTF              : 0x%08x\n", vic_read_reg(index, VIC_INPUT_INTF));
	printk("VIC_INPUT_DVP               : 0x%08x\n", vic_read_reg(index, VIC_INPUT_DVP));
	printk("VIC_INPUT_MIPI              : 0x%08x\n", vic_read_reg(index, VIC_INPUT_MIPI));
	printk("VIC_IN_HOR_PARA0            : 0x%08x\n", vic_read_reg(index, VIC_IN_HOR_PARA0));
	printk("VIC_IN_HOR_PARA1            : 0x%08x\n", vic_read_reg(index, VIC_IN_HOR_PARA1));
	printk("VIC_BK_CB_CTRL              : 0x%08x\n", vic_read_reg(index, VIC_BK_CB_CTRL));
	printk("VIC_BK_CB_BLK               : 0x%08x\n", vic_read_reg(index, VIC_BK_CB_BLK));
	printk("VIC_IN_VER_PARA0            : 0x%08x\n", vic_read_reg(index, VIC_INPUT_VPARA0));
	printk("VIC_IN_VER_PARA1            : 0x%08x\n", vic_read_reg(index, VIC_INPUT_VPARA1));
	printk("VIC_IN_VER_PARA2            : 0x%08x\n", vic_read_reg(index, VIC_INPUT_VPARA2));
	printk("VIC_IN_VER_PARA3            : 0x%08x\n", vic_read_reg(index, VIC_INPUT_VPARA3));
	printk("VIC_VLD_LINE_SAV            : 0x%08x\n", vic_read_reg(index, VIC_VLD_LINE_SAV));
	printk("VIC_VLD_LINE_EAV            : 0x%08x\n", vic_read_reg(index, VIC_VLD_LINE_EAV));
	printk("VIC_VLD_FRM_SAV             : 0x%08x\n", vic_read_reg(index, VIC_VLD_FRM_SAV));
	printk("VIC_VLD_FRM_EAV             : 0x%08x\n", vic_read_reg(index, VIC_VLD_FRM_EAV));
	printk("VIC_VC_CONTROL_FSM          : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_FSM));
	printk("VIC_VC_CONTROL_CH0_PIX      : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH0_PIX));
	printk("VIC_VC_CONTROL_CH1_PIX      : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH1_PIX));
	printk("VIC_VC_CONTROL_CH2_PIX      : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH2_PIX));
	printk("VIC_VC_CONTROL_CH3_PIX      : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH3_PIX));
	printk("VIC_VC_CONTROL_CH0_LINE     : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH0_LINE));
	printk("VIC_VC_CONTROL_CH1_LINE     : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH1_LINE));
	printk("VIC_VC_CONTROL_CH2_LINE     : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH2_LINE));
	printk("VIC_VC_CONTROL_CH3_LINE     : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_CH3_LINE));
	printk("VIC_VC_CONTROL_FIFO_USE     : 0x%08x\n", vic_read_reg(index, VIC_VC_CONTROL_FIFO_USE));
	printk("VIC_CB_1ST                  : 0x%08x\n", vic_read_reg(index, VIC_CB_1ST));
	printk("VIC_CB_2ND                  : 0x%08x\n", vic_read_reg(index, VIC_CB_2ND));
	printk("VIC_CB_3RD                  : 0x%08x\n", vic_read_reg(index, VIC_CB_3RD));
	printk("VIC_CB_4TH                  : 0x%08x\n", vic_read_reg(index, VIC_CB_4TH));
	printk("VIC_CB_5TH                  : 0x%08x\n", vic_read_reg(index, VIC_CB_5TH));
	printk("VIC_CB_6TH                  : 0x%08x\n", vic_read_reg(index, VIC_CB_6TH));
	printk("VIC_CB_7TH                  : 0x%08x\n", vic_read_reg(index, VIC_CB_7TH));
	printk("VIC_CB_8TH                  : 0x%08x\n", vic_read_reg(index, VIC_CB_8TH));
	printk("VIC_CB2_1ST                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_1ST));
	printk("VIC_CB2_2ND                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_2ND));
	printk("VIC_CB2_3RD                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_3RD));
	printk("VIC_CB2_4TH                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_4TH));
	printk("VIC_CB2_5TH                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_5TH));
	printk("VIC_CB2_6TH                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_6TH));
	printk("VIC_CB2_7TH                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_7TH));
	printk("VIC_CB2_8TH                 : 0x%08x\n", vic_read_reg(index, VIC_CB2_8TH));
	printk("MIPI_ALL_WIDTH_4BYTE        : 0x%08x\n", vic_read_reg(index, MIPI_ALL_WIDTH_4BYTE));
	printk("MIPI_VCROP_DEL01            : 0x%08x\n", vic_read_reg(index, MIPI_VCROP_DEL01));
	printk("MIPI_SENSOR_CONTROL         : 0x%08x\n", vic_read_reg(index, MIPI_SENSOR_CONTROL));
	printk("MIPI_HCROP_CH0              : 0x%08x\n", vic_read_reg(index, MIPI_HCROP_CH0));
	printk("MIPI_VCROP_SHADOW_CFG       : 0x%08x\n", vic_read_reg(index, MIPI_VCROP_SHADOW_CFG));
	printk("VIC_CONTROL_LIMIT           : 0x%08x\n", vic_read_reg(index, VIC_CONTROL_LIMIT));
	printk("VIC_CONTROL_DELAY           : 0x%08x\n", vic_read_reg(index, VIC_CONTROL_DELAY));
	printk("VIC_CONTROL_TIZIANO_ROUTE   : 0x%08x\n", vic_read_reg(index, VIC_CONTROL_TIZIANO_ROUTE));
	printk("VIC_CONTROL_DMA_ROUTE       : 0x%08x\n", vic_read_reg(index, VIC_CONTROL_DMA_ROUTE));
	printk("VIC_INT_STA                 : 0x%08x\n", vic_read_reg(index, VIC_INT_STA));
	printk("VIC_INT_MASK                : 0x%08x\n", vic_read_reg(index, VIC_INT_MASK));
	printk("VIC_INT_CLR                 : 0x%08x\n", vic_read_reg(index, VIC_INT_CLR));

	printk("VIC_DMA_CONFIGURE           : 0x%08x\n", vic_read_reg(index, VIC_DMA_CONFIGURE));
	printk("VIC_DMA_RESOLUTION          : 0x%08x\n", vic_read_reg(index, VIC_DMA_RESOLUTION));
	printk("VIC_DMA_RESET               : 0x%08x\n", vic_read_reg(index, VIC_DMA_RESET));
	printk("DMA_Y_CH_LINE_STRIDE        : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_LINE_STRIDE));
	printk("VIC_DMA_Y_CH_BUF0_ADDR      : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_BUF0_ADDR));
	printk("VIC_DMA_Y_CH_BUF1_ADDR      : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_BUF1_ADDR));
	printk("VIC_DMA_Y_CH_BUF2_ADDR      : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_BUF2_ADDR));
	printk("VIC_DMA_Y_CH_BUF3_ADDR      : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_BUF3_ADDR));
	printk("VIC_DMA_Y_CH_BUF4_ADDR      : 0x%08x\n", vic_read_reg(index, VIC_DMA_Y_CH_BUF4_ADDR));
	printk("VIC_DMA_UV_CH_LINE_STRIDE   : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_LINE_STRIDE));
	printk("VIC_DMA_UV_CH_BUF0_ADDR     : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_BUF0_ADDR));
	printk("VIC_DMA_UV_CH_BUF1_ADDR     : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_BUF1_ADDR));
	printk("VIC_DMA_UV_CH_BUF2_ADDR     : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_BUF2_ADDR));
	printk("VIC_DMA_UV_CH_BUF3_ADDR     : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_BUF3_ADDR));
	printk("VIC_DMA_UV_CH_BUF4_ADDR     : 0x%08x\n", vic_read_reg(index, VIC_DMA_UV_CH_BUF4_ADDR));
	printk("=========================================\n");
}

static void vic_dvp_init(struct vic_hal_info *info)
{
	vic_reset(info->index);

	vic_init_common_setting(info);

	vic_init_dvp_timing(info);

	vic_dma_reset(info->index);

	init_dvp_dma(info);

	init_dvp_irq(info->index);

	vic_register_enable(info->index);

	vic_start(info->index);

	// vic_dump_reg(info->index);
}

static void vic_mipi_init(struct vic_hal_info *info)
{
	vic_dma_reset(info->index);

	init_mipi_dma(info);

	init_mipi_irq(info->index);

	vic_reset(info->index);

	int csi_ret = mipi_csi_phy_initialization(info->index, &info->mipi);  // 1,init phy and stream on
	if (csi_ret < 0) {
		panic("mipi_csi_phy_initialization failed\n");
	}

	vic_register_enable(info->index); // 2,vic enable; 3, wait vic enable

	vic_init_common_setting(info);

	vic_init_mipi_timing(info);  // 4, config vic register

	vic_start(info->index);   // 5, start vic

	// vic_dump_reg(info->index);
}

void vic_hal_stream_on(struct vic_hal_info *info)
{
	if (info->bus_type == SENSOR_DATA_BUS_DVP) {
		vic_dvp_init(info);
	} else if (info->bus_type == SENSOR_DATA_BUS_MIPI) {
		vic_mipi_init(info);
	}
}

void vic_hal_stream_off(struct vic_hal_info *info)
{
	vic_reset(info->index);
	usleep_range(1000, 1000);

	if (info->bus_type == SENSOR_DATA_BUS_MIPI) {
		mipi_csi_phy_stop(info->index);
	}
}

unsigned int vic_enable_ts(struct vic_hal_info *info, int ms_ch, unsigned int offset)
{
	unsigned int ts_enbale;
	unsigned int ts_counter = 250;

	/*
	 * 1.counter max 255
	 * 2.offset need 64bit align
	 * 3.vic timestamp 64bit, hight 32bit timestamp, low 32bit vic status
	 * 4.vic1 timestamp is from vic0
	 */

	ts_enbale = vic_read_reg(info->index, VIC_TS_ENABLE);
	ts_enbale |= TS_COUNTER_EN;

	vic_write_reg(info->index, VIC_TS_COUNTER, ts_counter - 1);

	switch (ms_ch) {
	case 0:
		vic_write_reg(info->index, VIC_TS_MS_CH0_OFFSET, offset);
		ts_enbale |= TS_MS_CH0_EN;
		vic_write_reg(info->index, VIC_TS_ENABLE, ts_enbale);
		break;
	case 1:
		vic_write_reg(info->index, VIC_TS_MS_CH1_OFFSET, offset);
		ts_enbale |= TS_MS_CH1_EN;
		vic_write_reg(info->index, VIC_TS_ENABLE, ts_enbale);
		break;
	case 2:
		vic_write_reg(info->index, VIC_TS_MS_CH2_OFFSET, offset);
		ts_enbale |= TS_MS_CH2_EN;
		vic_write_reg(info->index, VIC_TS_ENABLE, ts_enbale);
		break;
	}

	/* 返回每秒计数数 */
	return info->isp_clk_rate / ts_counter;
}

void vic_disable_ts(struct vic_hal_info *info)
{
	vic_write_reg(info->index,  VIC_TS_COUNTER, 0);
	vic_write_reg(info->index,  VIC_TS_ENABLE, 0);
}

void vic_hal_init_iodata(int index, unsigned int iobase)
{
	if (index >= ARRAY_SIZE(vic_iobase_array)) {
		panic("vic: index too large: %d\n", index);
	}

	vic_iobase_array[index] = iobase;
}

static unsigned int vic_dma_addr[][2] = {
	{VIC_DMA_Y_CH_BUF0_ADDR, VIC_DMA_UV_CH_BUF0_ADDR},
	{VIC_DMA_Y_CH_BUF1_ADDR, VIC_DMA_UV_CH_BUF1_ADDR},
	{VIC_DMA_Y_CH_BUF2_ADDR, VIC_DMA_UV_CH_BUF2_ADDR},
	{VIC_DMA_Y_CH_BUF3_ADDR, VIC_DMA_UV_CH_BUF3_ADDR},
	{VIC_DMA_Y_CH_BUF4_ADDR, VIC_DMA_UV_CH_BUF4_ADDR},
};

void vic_set_dma_addr(int index, unsigned long y_addr, unsigned long uv_addr, int frame_index)
{
	vic_write_reg(index, vic_dma_addr[frame_index][0], y_addr);
	vic_write_reg(index, vic_dma_addr[frame_index][1], uv_addr);
}
