#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/of_reserved_mem.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <linux/delay.h>

#include "isp-drv.h"
#include "isp-regs.h"
#include "tiziano-isp.h"

int isp_clk = 350000000;
module_param(isp_clk, int, S_IRUGO);
MODULE_PARM_DESC(isp_clk, "isp core clock");

struct isp_device *g_isp_device[2] = {0};
unsigned int MAIN_ISP_INDEX = 0;
unsigned int SEC_ISP_INDEX = 1;
unsigned int flag;
static int isp_enabled;

static inline unsigned int isp_reg_readl(struct isp_device *isp, unsigned int reg)
{
	return readl(isp->iobase + reg);
}

static inline void isp_reg_writel(struct isp_device *isp, unsigned int reg, unsigned int val)
{
	writel(val, isp->iobase + reg);
}

/* interface used by isp-core.*/
int system_reg_write(unsigned int reg, unsigned int value)
{
	struct isp_device *isp = g_isp_device[0];
	isp_reg_writel(isp, reg, value);
	return 0;
}

unsigned int  system_reg_read(unsigned int reg)
{
	struct isp_device *isp = g_isp_device[0];
	return isp_reg_readl(isp, reg);
}

int system_irq_func_set(int irq, void *func)
{
	if (irq < 42) {
		g_isp_device[MAIN_ISP_INDEX]->irq_func_cb[irq] = func;
	} else {
		g_isp_device[SEC_ISP_INDEX]->irq_func_cb[irq - 42] = func;
	}

	return 0;
}

static int isp_cpm_stop(struct isp_device *isp)
{
	int timeout = 0xffffff;
	unsigned int value = 0;

	/*stop request*/
	value = readl(isp->cpm_reset);
	value |= 1 << isp->bit_stp;
	writel(value, isp->cpm_reset);

	/*stop ack*/
	while (!(readl(isp->cpm_reset) & (1 << isp->bit_ack)) && --timeout);

	if (timeout == 0) {
		dev_err(isp->dev, "isp wait stop timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

int isp_cpm_reset(struct isp_device *isp)
{
	int timeout = 0xffffff;
	unsigned int value = 0;

	/*stop request*/
	value = readl(isp->cpm_reset);
	value |= 1 << isp->bit_stp;
	writel(value, isp->cpm_reset);

	/*stop ack*/
	while (!(readl(isp->cpm_reset) & (1 << isp->bit_ack)) && --timeout);

	if (timeout == 0) {
		dev_err(isp->dev, "isp wait stop timeout\n");
		return -ETIMEDOUT;
	}

	/* activate reset */
	value = readl(isp->cpm_reset);
	value &= ~(1 << isp->bit_stp);
	value |= 1 << isp->bit_sr;
	writel(value, isp->cpm_reset);

	/* deactive reset */
	value = readl(isp->cpm_reset);
	value &= ~(1 << isp->bit_sr);
	writel(value, isp->cpm_reset);

	return 0;
}

static int ingenic_isp_parse_dt(struct isp_device *isp)
{
	struct device *dev = isp->dev;
	unsigned int cpm_reset = 0;
	int ret = 0;

	of_property_read_u32(dev->of_node, "ingenic,index", &isp->index);
	of_property_read_u32(dev->of_node, "ingenic,cpm_reset", &cpm_reset);
	of_property_read_u32(dev->of_node, "ingenic,bit_sr", &isp->bit_sr);
	of_property_read_u32(dev->of_node, "ingenic,bit_stp", &isp->bit_stp);
	of_property_read_u32(dev->of_node, "ingenic,bit_ack", &isp->bit_ack);

	isp->cpm_reset = (void __iomem *)cpm_reset;

	return ret;
}
#if 0
int isp_load_params(tisp_init_param_t *iparam, char *bpath)
{
	struct file *file = NULL;
	struct inode *inode = NULL;
	mm_segment_t old_fs;
	loff_t fsize = 0;
	loff_t *pos = NULL;
	tisp_bin_t attr;
	unsigned int ret = 0;

	char file_name[64] = {0};

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (strlen(bpath)) {
		sprintf(attr.bname, bpath);
	} else {
		snprintf(attr.bname, sizeof(attr.bname), "/etc/sensor/%s-x2500.bin", iparam->sensor);
	}
	//sprintf(bpath, attr.bname);
	sprintf(file_name, attr.bname);

	/* open file */
	file = filp_open(file_name, O_RDONLY, 0);
	if (file < 0 || IS_ERR(file)) {
		printk("ISP: open %s file for isp calibrate read failed\n", file_name);
		ret = -1;
		iparam->tuned_params = NULL;
		goto failed_open_file;
	}

	/* read file */
	inode = file->f_inode;
	fsize = inode->i_size;
	pos = &(file->f_pos);

	if (iparam->tuned_params == NULL) {
		iparam->tuned_params = kzalloc(fsize, GFP_KERNEL);
		if (iparam->tuned_params == NULL) {
			printk("%s[%d]: Failed to alloc %lld KB buffer!\n", __func__, __LINE__, fsize >> 10);
			ret = -1;
			goto failed_malloc_data;
		}
		iparam->tuned_params_size = fsize;
	}

	vfs_read(file, iparam->tuned_params, fsize, pos);

failed_malloc_data:
	filp_close(file, NULL);
	set_fs(old_fs);
failed_open_file:
	return ret;
}

int isp_release_params(tisp_init_param_t *iparam)
{
	if (iparam->tuned_params) {
		kfree(iparam->tuned_params);
		iparam->tuned_params = NULL;
		iparam->tuned_params_size = 0;
	}

	return 0;
}
#endif
int iparam_init(struct v4l2_subdev *sd, tisp_init_param_t *iparam)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct ispcam_device *ispcam = isp->ispcam;
	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];

	iparam->width = input_fmt->format.width;
	iparam->height = input_fmt->format.height;

	iparam->WdrEn = isp->sensor_info->wdr_en;
	iparam->sensorId = isp->index;  //the sensor id:mark the struct for sensor 0/1
	memcpy(&iparam->multi_mode, &isp->multi_mode, sizeof(multisensor_mode_t));

	switch (input_fmt->format.code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			iparam->bayer = 0;
			break;
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			iparam->bayer = 1 << 16;
			break;
		case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			iparam->bayer = 2 << 16;
			break;
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_YVYU8_2X8:
		case MEDIA_BUS_FMT_VYUY8_2X8:
			iparam->bayer = 3 << 16;
			break;
		default:
			dev_err(isp->dev, "%s[%d] the format(0x%08x) of input couldn't be handled!\n",
			        __func__, __LINE__, input_fmt->format.code);
			return -EINVAL;
			break;
	}
	switch (input_fmt->format.code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8:
		case MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE:
		case MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE:
		case MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE:
		case MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE:
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SBGGR12_1X12:
			iparam->bayer += 1;
			break;
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			iparam->bayer += 3;
			break;
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
			iparam->bayer += 2;
			break;
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			iparam->bayer += 0;
			break;
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_YVYU8_2X8:
		case MEDIA_BUS_FMT_VYUY8_2X8:
			iparam->bayer += 0;
			break;
		default:
			dev_err(isp->dev, "%s[%d] the format(0x%08x) of input couldn't be handled!\n",
			        __func__, __LINE__, input_fmt->format.code);
			return -EINVAL;
			break;
	}

	strncpy(iparam->sensor, ispcam->isd[0]->sd->dev->driver->name, sizeof(iparam->sensor));
	iparam->sensor_info.fps = isp->sensor_info->fps;
	iparam->sensor_info.total_width = isp->sensor_info->total_width;
	iparam->sensor_info.total_height = isp->sensor_info->total_height;

	return 0;

}

static int isp_subdev_init(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

/* interface should be removed. */
static int isp_subdev_reset(struct v4l2_subdev *sd, u32 val)
{
	//struct isp_device *isp = v4l2_get_subdevdata(sd);
	int ret = 0;

	//  ret = isp_cpm_reset(isp);

	return ret;
}

static const struct v4l2_subdev_core_ops isp_subdev_core_ops = {
	.init   = isp_subdev_init,
	.reset = isp_subdev_reset,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static int isp_subdev_get_fmt(struct v4l2_subdev *sd,
                              struct v4l2_subdev_state *state,
                              struct v4l2_subdev_format *format)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt = {0};
	int ret = 0;

	remote = media_pad_remote_pad_unique(&isp->pads[ISP_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	/*获取源当前格式，复制到输出格式.*/
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	/*本身ISP时没有格式区别的， 这里必须从VIC获取，即ISP的SINK_PAD.*/
	if (format->pad == ISP_PAD_SOURCE) {
		memcpy(&format->format, &remote_subdev_fmt.format, sizeof(format->format));
	} else {
		dev_warn(isp->dev, "ISP_PAD_SOURCE should be set!\n");
	}

	isp->formats[ISP_PAD_SINK] = isp->formats[ISP_PAD_SOURCE] = *format;
	isp->sensor_info = (struct sensor_info *) * (unsigned int *)format->format.reserved;

	//printk("----%s, %d, format->pad: %d\n", __func__, __LINE__, format->pad);
	return 0;
}

static int isp_subdev_set_fmt(struct v4l2_subdev *sd,
                              struct v4l2_subdev_state *state,
                              struct v4l2_subdev_format *format)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	int ret = 0;

	remote = media_pad_remote_pad_unique(&isp->pads[ISP_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	ret = v4l2_subdev_call(remote_sd, pad, set_fmt, NULL, format);
	if (ret < 0) {
		dev_dbg(isp->dev, "Failed to set_fmt from remote pad\n");
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops isp_subdev_pad_ops = {
	.set_fmt                = isp_subdev_set_fmt,
	.get_fmt                = isp_subdev_get_fmt,
	/*
	    .init_cfg               = isp_subdev_init_cfg,
	    .enum_mbus_code         = isp_subdev_enum_mbus_code,
	    .enum_frame_size        = isp_subdev_enum_frame_size,
	*/
};

static int isp_fw_process(void *data)
{
	while (!kthread_should_stop()) {
		tisp_fw_process(0);
	}
	return 0;
}

static int isp_fw_process1(void *data)
{
	while (!kthread_should_stop()) {
		tisp_fw_process(1);
	}
	return 0;
}

dma_addr_t wdr_paddr;
void *wdr_vaddr;
int wdr_size;

int isp_set_wdr(struct isp_device *isp)
{
	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];
	int width = input_fmt->format.width;
	int height = input_fmt->format.height;
	uint32_t tsize = width * height * 2 * 2;
	wdr_size = tsize;
	wdr_vaddr = dma_alloc_noncoherent(isp->dev, tsize, &wdr_paddr, DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (!wdr_vaddr || !wdr_paddr) {
		return -ENOMEM;
	}
	system_reg_write(IPC_ADDR_DF_CH0_ADDR, wdr_paddr);
	system_reg_write(IPC_ADDR_DF_CH0_SIZE, tsize / 2);
	system_reg_write(IPC_ADDR_DF_CHN_COMP_EN, 0x0);
	system_reg_write(IPC_ADDR_DF_CH1_ADDR, wdr_paddr);
	system_reg_write(IPC_ADDR_DF_CH1_SIZE, tsize / 2);
	return 0;
}

int isp_set_mdns(struct isp_device *isp)
{

	struct v4l2_subdev_format *input_fmt = &isp->formats[ISP_PAD_SINK];
	int width = input_fmt->format.width;
	int height = input_fmt->format.height;
	uint32_t tsize = 0;
	uint32_t tmpsize = 0;
	uint32_t stride = 0;
	uint32_t mdns_res_end_point_y;
	uint32_t mdns_res_end_point_c;
	int vinum = isp->index;
	dma_addr_t mdns_paddr = 0;

	/*get mdns bufinfo*/
	//Ref Y
	stride = ((width + 15) / 16) * 16;
	tmpsize = stride * height + ((stride * height) >> 5) ;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	mdns_res_end_point_y = tmpsize / 256;
	tsize += tmpsize;

	//Ref UV
	stride = ((width + 31) / 32) * 32;
	tmpsize = stride * height / 2 + ((stride * height / 2) >> 6);
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	mdns_res_end_point_c = tmpsize / 256;
	tsize += tmpsize;

	//Bsn
	stride = (width + 7) / 8;
	stride = (((stride + 1) / 2 + 7) / 8) * 8;
	tmpsize = (stride * height) / 8;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	tsize += tmpsize;

	//Ass
	stride = ((width + 31) / 32) * 32;
	tmpsize = stride * height / 2;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	tsize += tmpsize;

	//Lynne
	stride = ((width + 31) / 32) * 16;
	tmpsize = stride * height / 4;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	tsize += tmpsize;

	//  bufinfo.paddr = 0;
	//  bufinfo.size = tsize;
	isp->buf_info[MDNS_BUF].size = tsize;
	isp->buf_info[MDNS_BUF].vaddr = dma_alloc_noncoherent(isp->dev, tsize, &isp->buf_info[MDNS_BUF].paddr, DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (!isp->buf_info[MDNS_BUF].vaddr || !isp->buf_info[MDNS_BUF].paddr) {
		return -ENOMEM;
	}

	mdns_paddr = isp->buf_info[MDNS_BUF].paddr;
	tsize = 0;

	/*set mdns bufinfo*/
	//Ref Y
	stride = ((width + 15) / 16) * 16;
	tmpsize = stride * height + ((stride * height) >> 5) ;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	mdns_res_end_point_y = tmpsize / 256;
	system_reg_write(MDNS_ADDR_YRESENDPOINT(vinum), mdns_res_end_point_y);
	system_reg_write(MDNS_ADDR_YREF_ADDR(vinum), mdns_paddr + tsize);
	system_reg_write(MDNS_ADDR_YREF_STRIDE(vinum), stride);
	tsize += tmpsize;
	if (tsize > isp->buf_info[MDNS_BUF].size) {
		dev_err(isp->dev, "[ %s:%d ] buf size too small\n", __func__, __LINE__);
		return -EFAULT;
	}

	//REF UV
	stride = ((width + 31) / 32) * 32;
	tmpsize = stride * height / 2 + ((stride * height / 2) >> 6);
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	mdns_res_end_point_c = tmpsize / 256;
	system_reg_write(MDNS_ADDR_CRESENDPOINT(vinum), mdns_res_end_point_c);
	system_reg_write(MDNS_ADDR_CREF_ADDR(vinum), mdns_paddr + tsize);
	system_reg_write(MDNS_ADDR_CREF_STRIDE(vinum), stride);
	tsize += tmpsize;
	if (tsize > isp->buf_info[MDNS_BUF].size) {
		dev_err(isp->dev, "[ %s:%d ] buf size too small\n", __func__, __LINE__);
		return -EFAULT;
	}

	//Bsn
	stride = (width + 7) / 8;
	stride = (((stride + 1) / 2 + 7) / 8) * 8;
	tmpsize = (stride * height) / 8;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	system_reg_write(MDNS_ADDR_BSN_ADDR(vinum), mdns_paddr + tsize);
	system_reg_write(MDNS_ADDR_BSN_STRIDE(vinum), stride);
	tsize += tmpsize;
	if (tsize > isp->buf_info[MDNS_BUF].size) {
		dev_err(isp->dev, "[ %s:%d ] buf size too small\n", __func__, __LINE__);
		return -EFAULT;
	}

	//Ass
	stride = ((width + 31) / 32) * 32;
	tmpsize = stride * height / 2;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	system_reg_write(MDNS_ADDR_ASS_ADDR(vinum), mdns_paddr + tsize);
	system_reg_write(MDNS_ADDR_ASS_STRIDE(vinum), stride);
	tsize += tmpsize;
	if (tsize > isp->buf_info[MDNS_BUF].size) {
		dev_err(isp->dev, "[ %s:%d ] buf size too small\n", __func__, __LINE__);
		return -EFAULT;
	}

	//Lynne
	stride = ((width + 31) / 32) * 16;
	tmpsize = stride * height / 4;
	tmpsize = ((tmpsize + 1023) / 1024 * 1024);
	system_reg_write(MDNS_ADDR_LYN_ADDR(vinum), mdns_paddr + tsize);
	system_reg_write(MDNS_ADDR_LYN_STRIDE(vinum), stride);
	tsize += tmpsize;
	if (tsize > isp->buf_info[MDNS_BUF].size) {
		dev_err(isp->dev, "[ %s:%d ] buf size too small\n", __func__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

int isp_set_dualsensor_buf(struct isp_device *isp)
{
	struct v4l2_subdev_format *input_fmt = NULL;
	int width = 0;
	int height = 0;
	int bitdepth = 0;
	unsigned int mbus_code = 0;
	int buf_size = 0;

	input_fmt = &isp->formats[ISP_PAD_SINK];
	width = input_fmt->format.width;
	height = input_fmt->format.height;
	mbus_code = input_fmt->format.code;
	if (mbus_code <= MEDIA_BUS_FMT_SRGGB12_1X12 && mbus_code >= MEDIA_BUS_FMT_SBGGR12_1X12) {
		bitdepth = 12;
	} else {
		bitdepth = 10;
	}

	buf_size = ((width * height * bitdepth / 8 + width * height / 32 + 7) / 8 * 8) * 2;
	isp->buf_info[DUALSENSOR_BUF].size = buf_size;

	if (!isp->buf_info[DUALSENSOR_BUF].vaddr && !isp->buf_info[DUALSENSOR_BUF].paddr) {
		isp->buf_info[DUALSENSOR_BUF].vaddr = dma_alloc_noncoherent(isp->dev, buf_size, &isp->buf_info[DUALSENSOR_BUF].paddr, DMA_BIDIRECTIONAL, GFP_KERNEL);
		if (!isp->buf_info[DUALSENSOR_BUF].vaddr || !isp->buf_info[DUALSENSOR_BUF].paddr) {
			return -ENOMEM;
		}
	}
	//printk("~~~%s %d %d %d %d isp->index = %d\n", __func__, __LINE__, width, height, bitdepth, isp->index);

	if (isp->index == MAIN_ISP_INDEX) { /*main*/
		system_reg_write(IPC_ADDR_DUAL_M3_ORI_MODE, 0x0);
		system_reg_write(IPC_ADDR_M3_V0_URGENT_THRES, 0x60);

		system_reg_write(IPC_ADDR_DF_CH1_ADDR, isp->buf_info[DUALSENSOR_BUF].paddr);
		system_reg_write(IPC_ADDR_DF_CH1_SIZE, isp->buf_info[DUALSENSOR_BUF].size);
		system_reg_write(IPC_ADDR_M3_V0_V0_THRES, (width * height * 9 / 8) / 256); /* width * height * 1.5 * 0.75 */
		system_reg_write(IPC_ADDR_M3_V0_V1_THRES, 0x0);

		system_reg_write(0x101c, width * height / 169); //IPC OVER FLOW /*(tsize >> 8 >> 1) - 10 ??*/
		system_reg_write(0x1020, width * height / 169);
	} else if (isp->index == SEC_ISP_INDEX) {
		system_reg_write(IPC_ADDR_M3_V1_URGENT_THRES, 0x60);

		system_reg_write(IPC_ADDR_DF_CH3_ADDR, isp->buf_info[DUALSENSOR_BUF].paddr);
		system_reg_write(IPC_ADDR_DF_CH3_SIZE, isp->buf_info[DUALSENSOR_BUF].size);
		system_reg_write(IPC_ADDR_M3_V1_V0_THRES, 0x0);
		system_reg_write(IPC_ADDR_M3_V1_V1_THRES, (width * height * 9 / 8) / 256);

		system_reg_write(0x1024, width * height / 169);
		system_reg_write(0x1028, width * height / 169);
	}

	return 0;
}

static int isp_subdev_streamon(struct v4l2_subdev *sd)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	tisp_init_param_t iparam = {0};
	unsigned long flags = 0;
	unsigned int top_bypass = 0;
	int ret = 0;

	if (isp->enabled++ > 0) {
		return 0;
	}

	ret = iparam_init(sd, &iparam);

	if (isp->index == 0) {
		ret = tisp_main_init(&iparam, isp->bpath.path);
	} else if (isp->index == 1) {
		ret = tisp_sec_init(&iparam, isp->bpath.path);
	}

	if (ret < 0) {
		/*如果出错，释放在init阶段可能申请的buffer*/
		goto tisp_init_err;
	}

	top_bypass = tisp_get_top_bypass(isp->index);
	if (!(top_bypass & MDNS_BYPASS)) {
		ret = isp_set_mdns(isp);
		if (ret) {
			goto set_mdns_err;
		}
	}

	if (isp->sensor_info->wdr_en) {
		ret = isp_set_wdr(isp);
		if (ret) {
			goto set_wdr_err;
		}
	}

#ifndef CONFIG_INGENIC_ISP_V2_SENSOR_NUM_ONE
	ret = isp_set_dualsensor_buf(isp); /*dual sensor all cached mode*/
	if (ret) {
		goto set_dualsensor_buf_err;
	}
#endif

	tisp_stream_on(&iparam);
	system_reg_write(SEC_MSCA_ADDR_FORCE, 1);

	if (isp_enabled == 0) {
		tisp_activate_all();
		tisp_process_init();
		ret = tisp_enable_tuning();
		if (ret) {
			goto enable_tuning_err;
		}
	}

	/*For event engine.*/
	if (isp->index == 0) {
		isp->process_thread = kthread_run(isp_fw_process, isp, "isp_fw_process");
		if (IS_ERR_OR_NULL(isp->process_thread)) {
			dev_err(isp->dev, "%s[%d] kthread_run was failed!\n", __func__, __LINE__);
			ret = -EINVAL;
		}
		if (ret) {
			goto process_thread_err;
		}
	} else if (isp->index == 1) {
		isp->process_thread = kthread_run(isp_fw_process1, isp, "isp_fw_process1");
		if (IS_ERR_OR_NULL(isp->process_thread)) {
			dev_err(isp->dev, "%s[%d] kthread_run was failed!\n", __func__, __LINE__);
			ret = -EINVAL;
		}
		if (ret) {
			goto process_thread_err;
		}
	}

	/*enable irq*/
	spin_lock_irqsave(&isp->lock, flags);
	system_reg_write(RESP_ADDR_INT_COMMON_EN(isp->index), 0xffffffff); //common intp
	//system_reg_write(RESP_ADDR_INT_UNUSUAL_EN(isp->index), 0xffffffff); //unusual intp
	//system_reg_write(RESP_ADDR_INT_BACKUP_EN(isp->index), 1 << 27 | 1 << 29); //back0 intp
	system_reg_write(RESP_ADDR_INT_BACKUP_EN(isp->index), 1 << 20); //back0 intp
	spin_unlock_irqrestore(&isp->lock, flags);

	tisp_ipc_triger();
	isp_enabled++;

	return 0;

process_thread_err:

enable_tuning_err:
	tisp_process_deinit(isp->index);
	tisp_deinit(isp->index);

	if (isp_enabled == 0) {
		tisp_disable_tuning();
		tisp_slake_all();
	}

#ifndef CONFIG_INGENIC_ISP_V2_SENSOR_NUM_ONE
	if (isp->buf_info[DUALSENSOR_BUF].vaddr && isp->buf_info[DUALSENSOR_BUF].paddr) {
		dma_free_noncoherent(isp->dev, isp->buf_info[DUALSENSOR_BUF].size, isp->buf_info[DUALSENSOR_BUF].vaddr, isp->buf_info[DUALSENSOR_BUF].paddr, DMA_BIDIRECTIONAL);
	}
set_dualsensor_buf_err:
#endif
	isp->buf_info[DUALSENSOR_BUF].vaddr = 0;
	isp->buf_info[DUALSENSOR_BUF].paddr = 0;
	if (isp->sensor_info->wdr_en && wdr_vaddr && wdr_paddr) {
		dma_free_noncoherent(isp->dev, wdr_size, wdr_vaddr, wdr_paddr, DMA_BIDIRECTIONAL);
	}
set_wdr_err:
	wdr_vaddr = 0;
	wdr_paddr = 0;
	if (isp->buf_info[MDNS_BUF].vaddr && isp->buf_info[MDNS_BUF].paddr) {
		dma_free_noncoherent(isp->dev, isp->buf_info[MDNS_BUF].size, isp->buf_info[MDNS_BUF].vaddr, isp->buf_info[MDNS_BUF].paddr, DMA_BIDIRECTIONAL);
	}
set_mdns_err:
	isp->buf_info[MDNS_BUF].vaddr = 0;
	isp->buf_info[MDNS_BUF].paddr = 0;
	isp->enabled--;
tisp_init_err:
	tisp_deinit(isp->index);
	return ret;
}

unsigned int sensor1_deinit_flag;
static int isp_subdev_streamoff(struct v4l2_subdev *sd)
{
	struct isp_device *isp = v4l2_get_subdevdata(sd);
	unsigned long flags = 0;
	unsigned int top_val = 0;
	unsigned int tmp0, tmp1, tmp2;
	int timeout = 0xffffff;

	if (--isp->enabled > 0) {
		return 0;
	}

	unsigned int top_bypass = 0;
	top_bypass = tisp_get_top_bypass(isp->index);
	if ((!(top_bypass & DEFOG_BYPASS)) || (!(top_bypass & AWB0_BYPASS)) || (!(top_bypass & AWB1_BYPASS)) || (!(top_bypass & ADR_BYPASS))) {
		system_reg_write(TOP_ADDR_BYPASS(isp->index), top_bypass | DEFOG_BYPASS | AWB0_BYPASS | AWB1_BYPASS | ADR_BYPASS);
		msleep(50);
	}

	if (isp->sensor_info->wdr_en && wdr_vaddr && wdr_paddr) {
		dma_free_noncoherent(isp->dev, wdr_size, wdr_vaddr, wdr_paddr, DMA_BIDIRECTIONAL);
		wdr_vaddr = 0;
		wdr_paddr = 0;
	}

	/*disable irq*/
	spin_lock_irqsave(&isp->lock, flags);
	system_reg_write(RESP_ADDR_INT_COMMON_EN(isp->index), 0x0); //common intp
	isp_reg_writel(isp, RESP_ADDR_INT_COMMON_CLR(isp->index), 0xffffffff);
	spin_unlock_irqrestore(&isp->lock, flags);
	kthread_stop(isp->process_thread);

	if (--isp_enabled == 0) {
		top_val = system_reg_read(TOP_ADDR_STOP_CON);
		tmp0 = *(volatile unsigned int *)0xb3380300;
		tmp1 = *(volatile unsigned int *)0xb3390300;
		tmp2 = *(volatile unsigned int *)0xb33a0300;
		system_reg_write(TOP_ADDR_STOP_CON, top_val | 0x1);
		while (timeout--) {
			top_val = system_reg_read(TOP_ADDR_STOP_STATE);
			if (top_val & 0x01) {
				break;
			}
		}

		if (timeout == 0) {
			dev_err(isp->dev, "isp wait stop timeout\n");
			return -ETIMEDOUT;
		}
		*(volatile unsigned int *)0xb3380300 = tmp0;
		*(volatile unsigned int *)0xb3390300 = tmp1;
		*(volatile unsigned int *)0xb33a0300 = tmp2;

		top_val = system_reg_read(TOP_ADDR_TOP_RST);
		system_reg_write(TOP_ADDR_TOP_RST, top_val | ALL_RST);
		system_reg_write(TOP_ADDR_TOP_RST, top_val & (~ALL_RST));
	}

	tisp_deinit(isp->index);
	if (isp_enabled == 0) {
		tisp_disable_tuning();
		tisp_slake_all();
	}
	tisp_process_deinit(isp->index);

#ifndef CONFIG_INGENIC_ISP_V2_SENSOR_NUM_ONE
	if (isp->buf_info[DUALSENSOR_BUF].vaddr && isp->buf_info[DUALSENSOR_BUF].paddr) {
		dma_free_noncoherent(isp->dev, isp->buf_info[DUALSENSOR_BUF].size, isp->buf_info[DUALSENSOR_BUF].vaddr, isp->buf_info[DUALSENSOR_BUF].paddr, DMA_BIDIRECTIONAL);
		isp->buf_info[DUALSENSOR_BUF].vaddr = 0;
		isp->buf_info[DUALSENSOR_BUF].paddr = 0;
	}
#endif

	if (isp->buf_info[MDNS_BUF].vaddr && isp->buf_info[MDNS_BUF].paddr) {
		dma_free_noncoherent(isp->dev, isp->buf_info[MDNS_BUF].size, isp->buf_info[MDNS_BUF].vaddr, isp->buf_info[MDNS_BUF].paddr, DMA_BIDIRECTIONAL);
		isp->buf_info[MDNS_BUF].vaddr = 0;
		isp->buf_info[MDNS_BUF].paddr = 0;
	}

	system_reg_write(SEC_MSCA_ADDR_FORCE, 1);

	return 0;
}

static int isp_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable) {
		return isp_subdev_streamon(sd);
	} else {
		return isp_subdev_streamoff(sd);
	}
}

static const struct v4l2_subdev_video_ops isp_subdev_video_ops = {
	.s_stream = isp_subdev_s_stream,
};

static const struct v4l2_subdev_ops isp_subdev_ops = {
	.core = &isp_subdev_core_ops,
	.pad = &isp_subdev_pad_ops,
	.video = &isp_subdev_video_ops,
};

#ifdef CONFIG_ISP_DUMP
static ssize_t
dump_isp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	char *p = buf;

	p += sprintf(p, "TOP_ADDR_VERSION            :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_VERSION));
	p += sprintf(p, "TOP_ADDR_TOP_CON            :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_TOP_CON));
	p += sprintf(p, "TOP_ADDR_TOP_RST             :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_VERSION));
	p += sprintf(p, "TOP_ADDR_TOP_INFO            :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_TOP_CON));
	p += sprintf(p, "TOP_ADDR_BYPASS(0) :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_BYPASS(0)));
	p += sprintf(p, "TOP_ADDR_BYPASS(1) :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_BYPASS(1)));
	p += sprintf(p, "TOP_ADDR_REG_CON :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_REG_CON));
	p += sprintf(p, "TOP_ADDR_STATIC_EN1 :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_EN1));
	p += sprintf(p, "TOP_ADDR_STATIC_EN      :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_EN));
	p += sprintf(p, "TOP_ADDR_STATIC_STOP    :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_STOP));
	p += sprintf(p, "TOP_ADDR_STATIC_RST     :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_RST));
	p += sprintf(p, "TOP_ADDR_STATIC_POS(0) :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_POS(0)));
	p += sprintf(p, "TOP_ADDR_STATIC_POS(1) :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STATIC_POS(1)));
	p += sprintf(p, "TOP_ADDR_S0_FRM_SIZE    :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S0_FRM_SIZE));
	p += sprintf(p, "TOP_ADDR_S1_FRM_SIZE    :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S1_FRM_SIZE));
	p += sprintf(p, "TOP_ADDR_S0_BAYER_TYPE  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S0_BAYER_TYPE));
	p += sprintf(p, "TOP_ADDR_S1_BAYER_TYPE  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S1_BAYER_TYPE));
	p += sprintf(p, "TOP_ADDR_S0_LINE_SPACE_0 :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S0_LINE_SPACE_0));
	p += sprintf(p, "TOP_ADDR_S0_LINE_SPACE_1 :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S0_LINE_SPACE_1));
	p += sprintf(p, "TOP_ADDR_S1_LINE_SPACE_0 :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S1_LINE_SPACE_0));
	p += sprintf(p, "TOP_ADDR_S1_LINE_SPACE_1 :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_S1_LINE_SPACE_1));
	p += sprintf(p, "TOP_ADDR_STOP_CON       :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STOP_CON));
	p += sprintf(p, "TOP_ADDR_STOP_STATE     :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_STOP_STATE));
	p += sprintf(p, "TOP_ADDR_DMA_RD_CON     :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_RD_CON));
	p += sprintf(p, "TOP_ADDR_DMA_RD_INFO    :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_RD_INFO));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_0   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_0));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_1   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_1));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_2   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_2));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_3   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_3));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_4   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_4));
	p += sprintf(p, "TOP_ADDR_DMA_WR_CON_5   :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_CON_5));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_0  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_0));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_1  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_1));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_2  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_2));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_3  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_3));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_4  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_4));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_5  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_5));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_6  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_6));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_7  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_7));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_8  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_8));
	p += sprintf(p, "TOP_ADDR_DMA_WR_INFO_9  :0x%08x\n", isp_reg_readl(isp, TOP_ADDR_DMA_WR_INFO_9));

	p += sprintf(p, "IPC_ADDR_IP_TRIG                :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_IP_TRIG));
	p += sprintf(p, "IPC_ADDR_SHD_CTRL               :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_SHD_CTRL));
	p += sprintf(p, "IPC_ADDR_CONTROL                :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_CONTROL));
	p += sprintf(p, "IPC_ADDR_CHK_TRIG               :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_CHK_TRIG));
	p += sprintf(p, "IPC_ADDR_DUAL_CHN_SWITCH_CON    :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DUAL_CHN_SWITCH_CON));
	p += sprintf(p, "IPC_ADDR_DUAL_CHN_SWITCH_CON_NUM:0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DUAL_CHN_SWITCH_CON_NUM));
	p += sprintf(p, "IPC_ADDR_DUAL_CHN_SEL           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DUAL_CHN_SEL));
	p += sprintf(p, "IPC_ADDR_V0_NEAR_FULL           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_NEAR_FULL));
	p += sprintf(p, "IPC_ADDR_V0_NEAR_FULL_THRES     :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_NEAR_FULL_THRES));
	p += sprintf(p, "IPC_ADDR_V1_NEAR_FULL           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_NEAR_FULL));
	p += sprintf(p, "IPC_ADDR_V1_NEAR_FULL_THRES     :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_NEAR_FULL_THRES));
	p += sprintf(p, "IPC_ADDR_DF_CHN_COMP_EN         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CHN_COMP_EN));
	p += sprintf(p, "IPC_ADDR_DF_CH0_ADDR            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH0_ADDR));
	p += sprintf(p, "IPC_ADDR_DF_CH1_ADDR            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH1_ADDR));
	p += sprintf(p, "IPC_ADDR_DF_CH2_ADDR            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH2_ADDR));
	p += sprintf(p, "IPC_ADDR_DF_CH3_ADDR            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH3_ADDR));
	p += sprintf(p, "IPC_ADDR_DF_CH0_SIZE            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH0_SIZE));
	p += sprintf(p, "IPC_ADDR_DF_CH1_SIZE            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH1_SIZE));
	p += sprintf(p, "IPC_ADDR_DF_CH2_SIZE            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH2_SIZE));
	p += sprintf(p, "IPC_ADDR_DF_CH3_SIZE            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DF_CH3_SIZE));
	p += sprintf(p, "IPC_ADDR_YUV_OFFSET             :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_YUV_OFFSET));
	p += sprintf(p, "IPC_ADDR_WDMA_THRES_0           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_WDMA_THRES_0));
	p += sprintf(p, "IPC_ADDR_WDMA_THRES_1           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_WDMA_THRES_1));
	p += sprintf(p, "IPC_ADDR_RDMA_THRES             :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_RDMA_THRES));
	p += sprintf(p, "IPC_ADDR_DUAL_M3_ORI_MODE       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_DUAL_M3_ORI_MODE));
	p += sprintf(p, "IPC_ADDR_M3_V0_URGENT_THRES     :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V0_URGENT_THRES));
	p += sprintf(p, "IPC_ADDR_M3_V1_URGENT_THRES     :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V1_URGENT_THRES));
	p += sprintf(p, "IPC_ADDR_M3_V0_V0_THRES         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V0_V0_THRES));
	p += sprintf(p, "IPC_ADDR_M3_V0_V1_THRES         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V0_V1_THRES));
	p += sprintf(p, "IPC_ADDR_M3_V1_V0_THRES         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V1_V0_THRES));
	p += sprintf(p, "IPC_ADDR_M3_V1_V1_THRES         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_M3_V1_V1_THRES));
	p += sprintf(p, "IPC_ADDR_V0_CH0_IN_CNT          :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH0_IN_CNT));
	p += sprintf(p, "IPC_ADDR_V0_CH1_IN_CNT          :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH1_IN_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH0_IN_CNT          :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH0_IN_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH1_IN_CNT          :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH1_IN_CNT));
	p += sprintf(p, "IPC_ADDR_V0_CH0_ERR_CNT         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH0_ERR_CNT));
	p += sprintf(p, "IPC_ADDR_V0_CH1_ERR_CNT         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH1_ERR_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH0_ERR_CNT         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH0_ERR_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH1_ERR_CNT         :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH1_ERR_CNT));
	p += sprintf(p, "IPC_ADDR_ERR_CHN_INDEX          :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_ERR_CHN_INDEX));
	p += sprintf(p, "IPC_ADDR_V0_CH0_CHK_FR_CNT      :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH0_CHK_FR_CNT));
	p += sprintf(p, "IPC_ADDR_V0_CH1_CHK_FR_CNT      :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V0_CH1_CHK_FR_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH0_CHK_FR_CNT      :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH0_CHK_FR_CNT));
	p += sprintf(p, "IPC_ADDR_V1_CH1_CHK_FR_CNT      :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_V1_CH1_CHK_FR_CNT));
	p += sprintf(p, "IPC_ADDR_CHN_OUT_CNT            :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_CHN_OUT_CNT));
	p += sprintf(p, "IPC_ADDR_IN_OUT_STATE           :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_IN_OUT_STATE));
	p += sprintf(p, "IPC_ADDR_IN_CH01_FIFO_CNT       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_IN_CH01_FIFO_CNT));
	p += sprintf(p, "IPC_ADDR_IN_CH2_FIFO_CNT        :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_IN_CH2_FIFO_CNT));
	p += sprintf(p, "IPC_ADDR_COMP_CH0_DMA_CNT       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_COMP_CH0_DMA_CNT));
	p += sprintf(p, "IPC_ADDR_COMP_CH1_DMA_CNT       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_COMP_CH1_DMA_CNT));
	p += sprintf(p, "IPC_ADDR_COMP_CH2_DMA_CNT       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_COMP_CH2_DMA_CNT));
	p += sprintf(p, "IPC_ADDR_COMP_CH3_DMA_CNT       :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_COMP_CH3_DMA_CNT));
	p += sprintf(p, "IPC_ADDR_FR_PRO_NUM             :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_FR_PRO_NUM));
	p += sprintf(p, "IPC_ADDR_CH0_FR_INTERVAL        :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_CH0_FR_INTERVAL));
	p += sprintf(p, "IPC_ADDR_CH1_FR_INTERVAL        :0x%08x\n", isp_reg_readl(isp, IPC_ADDR_CH1_FR_INTERVAL));

	return p - buf;
}

static DEVICE_ATTR(dump_isp, S_IRUGO | S_IWUSR, dump_isp, NULL);

static struct attribute *isp_debug_attrs[] = {
	&dev_attr_dump_isp.attr,
	NULL,
};

static struct attribute_group isp_debug_attr_group = {
	.name   = "debug",
	.attrs  = isp_debug_attrs,
};
#endif

static int isp_comp_bind(struct device *comp, struct device *master,
                         void *master_data)
{
	struct isp_device *isp = dev_get_drvdata(comp);
	struct ispcam_device *ispcam = (struct ispcam_device *)master_data;
	struct v4l2_device *v4l2_dev = &ispcam->v4l2_dev;
	struct v4l2_subdev *sd = &isp->sd;
	int ret = 0;

	//dev_info(comp, "----dev_name(comp): %s----%s, %d \n", dev_name(comp), __func__, __LINE__);

	/* link subdev to master.*/
	isp->ispcam = (void *)ispcam;
	ispcam->isp = isp;

	/*1. register supported subdev ctrls.*/

	/*2. init v4l2_subdev*/

	v4l2_subdev_init(sd, &isp_subdev_ops);

	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(comp), sizeof(sd->name));
	v4l2_set_subdevdata(sd, isp);

	/* init isp pads. */
	isp->pads = kzalloc(sizeof(struct media_pad) * ISP_NUM_PADS, GFP_KERNEL);
	if (!isp->pads) {
		ret = -ENOMEM;
		goto err_alloc_pads;
	}
	isp->pads[0].index = ISP_PAD_SINK;
	isp->pads[0].flags = MEDIA_PAD_FL_SINK;
	isp->pads[1].index = ISP_PAD_SOURCE;
	isp->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, ISP_NUM_PADS, isp->pads);

	/*3. register v4l2_subdev*/
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_COMPOSER;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(comp, "Failed to register v4l2_subdev for isp\n");
		goto err_subdev_register;
	}

	return 0;
err_subdev_register:
err_alloc_pads:
	return ret;
}

static void isp_comp_unbind(struct device *comp, struct device *master,
                            void *master_data)
{
	struct isp_device *isp = dev_get_drvdata(comp);

	dev_info(comp, "---TODO:--%p---%s, %d \n", isp,  __func__, __LINE__);

}

static const struct component_ops isp_comp_ops = {
	.bind = isp_comp_bind,
	.unbind = isp_comp_unbind,
};

static irqreturn_t isp_irq_handler(struct isp_device *isp, unsigned int status, unsigned status_back, unsigned int status_unusual)
{
	bool handled = 0;
	int ret = 0;
	int i = 0;

	spin_lock(&isp->lock);

	if (isp_reg_readl(isp, RESP_ADDR_INT_COMMON_EN(isp->index)) == 0) {
		spin_unlock(&isp->lock);
		return IRQ_HANDLED;
	}

	/*1. process irq by subdev.*/
	if (status) {
		ret = v4l2_subdev_call(&isp->ispcam->mscaler->sd, core, interrupt_service_routine, status, &handled);
		if (ret < 0) {

		}
	}

	if (status & CH0_FRM_DONE_BF_INT) {
		tisp_hardware_reg_refresh(isp->index);
	}

	/*2. isp-core irq callbacks */
	for (i = 0; i < 32; i++) {
		if ((status & (0x1 << i)) && (NULL != isp->irq_func_cb[i])) {
			ret = isp->irq_func_cb[i]();
			if (ret < 0) {

			}
		}
	}

	if ((status_back & (0x1 << 29)) && (NULL != isp->irq_func_cb[33])) {
		ret = isp->irq_func_cb[33]();
		if (ret < 0) {

		}
	}

	if ((status_back & (0x1 << 20)) && (NULL != isp->irq_func_cb[ISP_IRQ_BCSH_FRM_DONE])) {
		ret = isp->irq_func_cb[ISP_IRQ_BCSH_FRM_DONE]();
		if (ret < 0) {

		}
	}

	/*clear irq_flags*/
	isp_reg_writel(isp, RESP_ADDR_INT_COMMON_CLR(isp->index), status);
	isp_reg_writel(isp, RESP_ADDR_INT_BACKUP_CLR(isp->index), status_back);
	isp_reg_writel(isp, RESP_ADDR_INT_UNUSUAL_CLR(isp->index), status_unusual);
	spin_unlock(&isp->lock);
	return IRQ_HANDLED;
}

static irqreturn_t isp_top_irq_handler(int irq, void *data)
{
	struct isp_device *isp = (struct isp_device *)data;
	unsigned int status = 0;
	unsigned int status_sec = 0;
	unsigned int status_back0 = 0;
	unsigned int status_back1 = 0;
	unsigned int status_unusual0 = 0;
	unsigned int status_unusual1 = 0;
	unsigned int ret_0 = 0;
	unsigned int ret_1 = 0;

	status = isp_reg_readl(isp, RESP_ADDR_INT_COMMON_INFO(0));
	status_sec = isp_reg_readl(isp, RESP_ADDR_INT_COMMON_INFO(1));
	status_back0 = isp_reg_readl(isp, RESP_ADDR_INT_BACKUP_INFO(0));
	status_back1 = isp_reg_readl(isp, RESP_ADDR_INT_BACKUP_INFO(1));
	status_unusual0 = isp_reg_readl(isp, RESP_ADDR_INT_UNUSUAL_INFO(0));
	status_unusual1 = isp_reg_readl(isp, RESP_ADDR_INT_UNUSUAL_INFO(1));
	//  printk("0x%08x 0x%08x\n", status,status_sec);
	//  printk("0x%08x 0x%08x\n", status_unusual0,status_unusual1);

	if (status || status_back0 || status_unusual0) {
		ret_0 = isp_irq_handler(g_isp_device[MAIN_ISP_INDEX], status, status_back0, status_unusual0);
	}
	if (status_sec || status_back1 || status_unusual1) {
		ret_1 = isp_irq_handler(g_isp_device[SEC_ISP_INDEX], status_sec, status_back1, status_unusual1);
	}

	return IRQ_HANDLED;
}

static int ingenic_isp_probe(struct platform_device *pdev)
{

	struct isp_device *isp = NULL;
	int ret = 0;

	isp = kzalloc(sizeof(struct isp_device), GFP_KERNEL);
	if (!isp) {
		pr_err("Failed to alloc isp dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	isp->dev = &pdev->dev;
	platform_set_drvdata(pdev, isp);

	ingenic_isp_parse_dt(isp);

#ifdef CONFIG_INGENIC_ISP_V2_SENSOR_NUM_ONE
	isp->multi_mode.sensor_num = IMPISP_TOTAL_ONE;
#else
	isp->multi_mode.sensor_num = IMPISP_TOTAL_TWO;
#endif
	isp->multi_mode.dual_mode = IMPISP_DUALSENSOR_DUAL_ALLCACHED_MODE;
	isp->multi_mode.joint_mode = IMPISP_NOT_JOINT;

	ret = component_add(isp->dev, &isp_comp_ops);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to add component isp!\n");
	}

	g_isp_device[isp->index] = isp;

	isp->iobase = (unsigned int *)0xb3300000;
	isp->irq = 31 + 8;

	spin_lock_init(&isp->lock);

	if (!flag) {
		//  ret = devm_request_irq(isp->dev, isp->irq, isp_irq_handler, 0,
		//          dev_name(isp->dev), isp);

		ret = request_irq(isp->irq, isp_top_irq_handler, 0,
		                  dev_name(isp->dev), isp);

		if (ret) {
			dev_err(isp->dev, "request irq failed!\n");
			goto err_request_irq;
		}

		isp->div_clk = of_clk_get(isp->dev->of_node, 0);
		if (!isp->div_clk) {
			dev_err(isp->dev, "failed to get isp div_clk\n");
			goto err_div_clk;
		}

		clk_set_rate(isp->div_clk, isp_clk);

		clk_prepare_enable(isp->div_clk);

		isp->gate_clk = of_clk_get(isp->dev->of_node, 1);
		if (!isp->gate_clk) {
			dev_err(isp->dev, "failed to get isp gate_clk\n");
			goto err_gate_clk;
		}

		clk_prepare_enable(isp->gate_clk);

#ifdef CONFIG_ISP_DUMP
		ret = sysfs_create_group(&isp->dev->kobj, &isp_debug_attr_group);
		if (ret) {
			dev_err(isp->dev, "device create sysfs group failed\n");

			ret = -EINVAL;
			goto err_sys_group;
		}
#endif

		flag = 1;
	}

	ret = of_reserved_mem_device_init(isp->dev);
	if (ret) {
		dev_warn(isp->dev, "failed to init reserved mem\n");
	}

	isp_cpm_reset(isp);

	return 0;
err_sys_group:
err_gate_clk:
err_div_clk:
err_request_irq:
	return ret;
}

static int ingenic_isp_remove(struct platform_device *pdev)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(isp->power_clk);
	clk_disable_unprepare(isp->gate_clk);
	clk_disable_unprepare(isp->div_clk);
	return 0;
}

static const struct of_device_id ingenic_isp_dt_match[] = {
	{ .compatible = "ingenic,x2500-isp" },
	{ }
};

MODULE_DEVICE_TABLE(of, ingenic_isp_dt_match);

static int __maybe_unused ingenic_isp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	if (isp->enabled) {
		dev_err(isp->dev, "faild to suspend, isp is streaming on\n");
		return -EBUSY;
	}

	isp_cpm_stop(isp);
	clk_disable_unprepare(isp->power_clk);
	clk_disable_unprepare(isp->gate_clk);
	clk_disable_unprepare(isp->div_clk);

	return 0;
}

static int __maybe_unused ingenic_isp_resume(struct platform_device *pdev)
{
	struct isp_device *isp = dev_get_drvdata(&pdev->dev);

	clk_prepare_enable(isp->div_clk);
	clk_prepare_enable(isp->gate_clk);
	clk_prepare_enable(isp->power_clk);
	isp_cpm_reset(isp);

	return 0;
}

static struct platform_driver ingenic_isp_driver = {
	.probe = ingenic_isp_probe,
	.remove = ingenic_isp_remove,
	.suspend = ingenic_isp_suspend,
	.resume = ingenic_isp_resume,
	.driver = {
		.name = "ingenic-isp",
		.of_match_table = ingenic_isp_dt_match,
	},
};

module_platform_driver(ingenic_isp_driver);

MODULE_ALIAS("platform:ingenic-isp");
MODULE_DESCRIPTION("ingenic isp subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");
