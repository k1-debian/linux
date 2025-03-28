#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "ingenic-adc.h"

static void adc_write_reg(struct ingenic_adc *adc, unsigned int reg,
                          uint32_t val)
{
	spin_lock(&adc->lock);
	writel(val, adc->base + reg);
	spin_unlock(&adc->lock);
}

static unsigned int  adc_read_reg(struct ingenic_adc *adc, unsigned int reg)
{
	return readl(adc->base + reg);
}

static void adc_set_bits(struct ingenic_adc *adc, unsigned int reg, unsigned int bits)
{

	adc_write_reg(adc, reg, adc_read_reg(adc, reg) | bits);
}

static int adc_read_bits(struct ingenic_adc *adc, unsigned int reg, unsigned int bits)
{

	return adc_read_reg(adc, reg)   & bits;
}

static void adc_clr_bits(struct ingenic_adc *adc, unsigned int reg, unsigned int bits)
{

	adc_write_reg(adc, reg, adc_read_reg(adc, reg) & ~bits);
}

void dump_reg(struct iio_dev *indio_dev)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);

	int i = 0;
	printk("ADC_SR(%#x)             = %#x\n", ADC_SR, adc_read_reg(adc, ADC_SR));
	printk("ADC_IE(%#x)             = %#x\n", ADC_IE, adc_read_reg(adc, ADC_IE));
	printk("ADC_IE_FORRISCV(%#x)    = %#x\n", ADC_IE_FORRISCV, adc_read_reg(adc, ADC_IE_FORRISCV));
	printk("ADC_IR(%#x)             = %#x\n", ADC_IR, adc_read_reg(adc, ADC_IR));
	printk("ADC_IR_FORRISCV(%#x)    = %#x\n", ADC_IR_FORRISCV, adc_read_reg(adc, ADC_IR_FORRISCV));
	printk("ADC_CR(%#x)             = %#x\n", ADC_CR, adc_read_reg(adc, ADC_CR));
	printk("ADC_CFR(%#x)            = %#x\n", ADC_CFR, adc_read_reg(adc, ADC_CFR));
	printk("ADC_CLK0(%#x)           = %#x\n", ADC_CLK0, adc_read_reg(adc, ADC_CLK0));
	printk("ADC_CLK1(%#x)           = %#x\n", ADC_CLK1, adc_read_reg(adc, ADC_CLK1));
	printk("ADC_CLK2(%#x)           = %#x\n", ADC_CLK2, adc_read_reg(adc, ADC_CLK2));
	printk("ADC_EXT_GPIO_CR(%#x)    = %#x\n", ADC_EXT_GPIO_CR, adc_read_reg(adc, ADC_EXT_GPIO_CR));

	for (i = 0; i < 16; i++) {
		printk("ADC_AWD_CR%d(%#x)        = %#x\n", i, ADC_AWD_CR(i), adc_read_reg(adc, ADC_AWD_CR(i)));
	}

	printk("ADC_AWD_SR(%#x)         = %#x\n", ADC_AWD_SR, adc_read_reg(adc, ADC_AWD_SR));
	printk("ADC_AWD_IM(%#x)         = %#x\n", ADC_AWD_IM, adc_read_reg(adc, ADC_AWD_IM));
	printk("ADC_AWD_IM_FORRISCV(%#x)                = %#x\n", ADC_AWD_IM_FORRISCV, adc_read_reg(adc, ADC_AWD_IM_FORRISCV));
	printk("ADC_AWD_IF(%#x)         = %#x\n", ADC_AWD_IF, adc_read_reg(adc, ADC_AWD_IF));
	printk("ADC_AWD_IF_FORRISCV(%#x)                = %#x\n", ADC_AWD_IF_FORRISCV, adc_read_reg(adc, ADC_AWD_IF_FORRISCV));
	printk("ADC_DBG_CR(%#x)         = %#x\n", ADC_DBG_CR, adc_read_reg(adc, ADC_DBG_CR));
	printk("ADC_DBG_FSM(%#x)                = %#x\n", ADC_DBG_FSM, adc_read_reg(adc, ADC_DBG_FSM));
	printk("ADC_SEQ0_CR(%#x)                = %#x\n", ADC_SEQ0_CR, adc_read_reg(adc, ADC_SEQ0_CR));
	printk("ADC_SEQ0_CNR0(%#x)              = %#x\n", ADC_SEQ0_CNR0, adc_read_reg(adc, ADC_SEQ0_CNR0));
	printk("ADC_SEQ0_DCR(%#x)               = %#x\n", ADC_SEQ0_DCR, adc_read_reg(adc, ADC_SEQ0_DCR));
	printk("ADC_SEQ0_DR0(%#x)               = %#x\n", ADC_SEQ0_DR0, adc_read_reg(adc, ADC_SEQ0_DR0));
	printk("ADC_SEQ0_DR1(%#x)               = %#x\n", ADC_SEQ0_DR1, adc_read_reg(adc, ADC_SEQ0_DR1));
	printk("ADC_SEQ1_CR(%#x)                = %#x\n", ADC_SEQ1_CR, adc_read_reg(adc, ADC_SEQ1_CR));
	printk("ADC_SEQ1_CNR0(%#x)              = %#x\n", ADC_SEQ1_CNR0, adc_read_reg(adc, ADC_SEQ1_CNR0));
	printk("ADC_SEQ1_CNR1(%#x)              = %#x\n", ADC_SEQ1_CNR1, adc_read_reg(adc, ADC_SEQ1_CNR1));
	printk("ADC_SEQ1_DCR(%#x)               = %#x\n", ADC_SEQ1_DCR, adc_read_reg(adc, ADC_SEQ1_DCR));
	printk("ADC_SEQ1_DR(%#x)                = %#x\n", ADC_SEQ1_DR, adc_read_reg(adc, ADC_SEQ1_DR));

	for (i = 0; i < 8; i++) {
		printk("ADC_SEQ1_DLY%d(%#x)      = %#x\n", i, ADC_SEQ1_DLY(i), adc_read_reg(adc, ADC_SEQ1_DLY(i)));
	}

	for (i = 0; i < 8; i++) {
		printk("ADC_SEQ1_CONT_CR%d(%#x)  = %#x\n", i, ADC_SEQ1_CONT_CR(i), adc_read_reg(adc, ADC_SEQ1_CONT_CR(i)));
	}

	printk("ADC_SEQ1_RCNT(%#x)              = %#x\n", ADC_SEQ1_RCNT, adc_read_reg(adc, ADC_SEQ1_RCNT));
	printk("ADC_SEQ1_DMA_RCNT(%#x)          = %#x\n", ADC_SEQ1_DMA_RCNT, adc_read_reg(adc, ADC_SEQ1_DMA_RCNT));
	printk("ADC_SEQ2_CR(%#x)                = %#x\n", ADC_SEQ2_CR, adc_read_reg(adc, ADC_SEQ2_CR));
	printk("ADC_SEQ2_CNR0(%#x)              = %#x\n", ADC_SEQ2_CNR0, adc_read_reg(adc, ADC_SEQ2_CNR0));
	printk("ADC_SEQ2_DCR(%#x)               = %#x\n", ADC_SEQ2_DCR, adc_read_reg(adc, ADC_SEQ2_DCR));
	printk("ADC_SEQ2_DR(%#x)                = %#x\n", ADC_SEQ2_DR, adc_read_reg(adc, ADC_SEQ2_DR));

	for (i = 0; i < 2; i++) {
		printk("ADC_SEQ2_DLY%d(%#x)      = %#x\n", i, ADC_SEQ2_DLY(i), adc_read_reg(adc, ADC_SEQ2_DLY(i)));
	}

	printk("ADC_SEQ2_RCNT(%#x)              = %#x\n", ADC_SEQ2_RCNT, adc_read_reg(adc, ADC_SEQ2_RCNT));
	printk("ADC_SEQ2_DMA_RCNT(%#x)          = %#x\n", ADC_SEQ2_DMA_RCNT, adc_read_reg(adc, ADC_SEQ2_DMA_RCNT));

}

void ingenic_adc_dma_end(void *dma_async_param)
{
	struct iio_dev *indio_dev = (struct iio_dev *)dma_async_param;
	struct ingenic_adc *adc = iio_priv(indio_dev);
	int i = 0;

	for (i = 0; i < DMA_BUFFER_SIZE / 4;) {
		iio_push_to_buffers(indio_dev, &adc->seq1rx_buf[i]);
		i += adc->seq1_len;
	}

	return;
}

static int  ingenic_adc_dma_rx(struct iio_dev *indio_dev, int seq_num)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	struct dma_slave_config seq1_rx_config;
	struct dma_async_tx_descriptor *seq1_rxdesc;
	struct dma_chan *rxchan = adc->seq1rxchan;

	if (!rxchan) {
		dev_err(adc->dev, "no dma rx channel\n");
		return -ENODEV;
	}

	if (seq_num == SEQ1) {
		seq1_rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		seq1_rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		seq1_rx_config.src_maxburst = 1;
		seq1_rx_config.dst_maxburst = 1;
		seq1_rx_config.src_addr = (dma_addr_t)(adc->base_phy + ADC_SEQ1_DR);
		seq1_rx_config.direction = DMA_DEV_TO_MEM;
		dmaengine_slave_config(rxchan, &seq1_rx_config);

		seq1_rxdesc = dmaengine_prep_dma_cyclic(rxchan,
		                                        adc->seq1rx_dma_buf,
		                                        DMA_BUFFER_SIZE,
		                                        DMA_BUFFER_SIZE,
		                                        DMA_DEV_TO_MEM,
		                                        DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

		if (!seq1_rxdesc) {
			dev_err(adc->dev, "device_prep_slave_sg error\n");
			printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
			dmaengine_terminate_all(rxchan);
			return -1;
		}

		seq1_rxdesc->callback = ingenic_adc_dma_end;
		seq1_rxdesc->callback_param = indio_dev;

		dmaengine_submit(seq1_rxdesc);

		dma_async_issue_pending(adc->seq1rxchan);
	}
	return 0;

}

static int ingenic_adc_setup(struct iio_dev *indio_dev, struct  iio_chan_spec const *chan, int seq_num)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	unsigned int tmp = 0;

	adc_clr_bits(adc, ADC_CFR, PHY_PD);
	adc_set_bits(adc, ADC_CFR, PHY_SEL_EN);
	adc_clr_bits(adc, ADC_CFR, PHY_RESET);

	if (seq_num == SEQ0) {
		tmp = adc_read_reg(adc, ADC_SEQ0_CR);
		tmp = (tmp & (~SEQ0_CR_LEN_MASK)) | ((adc->seq0_len - 1) << SEQ0_CR_LEN_SHIFT);
		adc_write_reg(adc, ADC_SEQ0_CR, tmp);

		adc_set_bits(adc, ADC_SEQ0_CR, SEQ0_CR_CH_NUM_EN);

	} else if (seq_num == SEQ1) {
		adc->record = 0;
		tmp = adc_read_reg(adc, ADC_SEQ1_CR);
		tmp = (tmp & (~SEQ1_CR_LEN_MASK)) | ((adc->seq1_len - 1) << SEQ1_CR_LEN_SHIFT);

		tmp = (tmp & (~ CONT_EN_MASK)) | CONT_EN;
		adc_write_reg(adc, ADC_SEQ1_CR, tmp);

		/* 连续转换时钟分频 在30M的设备时间下，分频后的时钟周期 1ms */
		tmp = 0x7530;
		adc_write_reg(adc, ADC_CLK0, tmp);

		tmp = adc->seq1_cont_interval;
		tmp = (tmp << CONT_CNT_SHIFT) | ((adc->seq1_len - 1) << SEQ1_CONT_LEN_SHIFT);
		adc_write_reg(adc, ADC_SEQ1_CONT_CR(0), tmp);

		adc_set_bits(adc, ADC_SEQ1_CR, SEQ1_CR_CH_NUM_EN);
		if (adc->use_dma) {
			/* 暂不使用seq1的dma finish终端 */
			//          adc_set_bits(adc,ADC_IE,IE_SEQ1_DMA_FIN);
			adc_set_bits(adc, ADC_SEQ1_DCR, DMA_MD);

			/*clear rcnt*/
			adc_write_reg(adc, ADC_SEQ1_RCNT, 0);
			tmp = adc_read_reg(adc, ADC_SEQ1_RCNT);
			tmp = (tmp & (~MAX_MASK)) | (DMA_BUFFER_SIZE << MAX_SHIFT);
			adc_write_reg(adc, ADC_SEQ1_RCNT, tmp);

			/*init dma*/
			ingenic_adc_dma_rx(indio_dev, SEQ1);

		} else {
			adc_set_bits(adc, ADC_IE, IE_SEQ1_DR);
		}
	} else if (seq_num == SEQ2) {
		tmp = adc_read_reg(adc, ADC_SEQ2_CR);
		tmp = (tmp & (~SEQ2_CR_LEN_MASK)) | ((adc->seq2_len - 1) << SEQ2_CR_LEN_SHIFT);
		adc_write_reg(adc, ADC_SEQ2_CR, tmp);

		adc_set_bits(adc, ADC_SEQ2_CR, SEQ2_CR_CH_NUM_EN);

		adc_set_bits(adc, ADC_IE, IE_SEQ2_DR);
	} else {
		//      dev_err();
	}
	return 0;
}
irqreturn_t ingenic_pollfunc(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ingenic_adc *adc = iio_priv(indio_dev);

	/* 未开启连续转换 */
	if (!adc->conversion_en) {
		ingenic_adc_setup(indio_dev, NULL, SEQ1);
		adc_set_bits(adc, ADC_CR, SEQ1_START);
		adc->conversion_en++;
	} else if (adc->conversion_en) {
		adc_clr_bits(adc, ADC_SEQ1_CR, CONT_EN);
		if (adc->use_dma) {
			dmaengine_terminate_all(adc->seq1rxchan);
			memset(adc->seq1rx_buf, 0, DMA_BUFFER_SIZE);
		}
		adc->conversion_en--;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ingenic_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ingenic_adc_set_seq(struct iio_dev *indio_dev, int channel, int seq_num)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	int tmp = 0;

	switch (seq_num) {
	case SEQ0:

		tmp = adc_read_reg(adc, ADC_SEQ0_CNR0);
		tmp = (tmp & (~(0xf  << (adc->seq0_len * 4)))) | (channel   << (adc->seq0_len * 4));
		adc_write_reg(adc, ADC_SEQ0_CNR0, tmp);
		adc->seq0_len ++;

		break;
	case SEQ1:

		if (adc->seq1_len < 8) {
			tmp = adc_read_reg(adc, ADC_SEQ1_CNR0);
			tmp  = (tmp & (~(0xf    << (adc->seq1_len * 4)))) | (channel   << (adc->seq1_len * 4));
			adc_write_reg(adc, ADC_SEQ1_CNR0, tmp);
		} else {
			tmp = adc_read_reg(adc, ADC_SEQ1_CNR1);
			tmp = (tmp & (~(0xf  << ((adc->seq1_len - 8) * 4)))) | (channel << ((adc->seq1_len - 8) * 4));
			adc_write_reg(adc, ADC_SEQ1_CNR1, tmp);
		}
		adc->seq1_len++;

		break;
	case SEQ2:

		tmp = adc_read_reg(adc, ADC_SEQ2_CNR0);
		tmp = (tmp & (~(0xf  << (adc->seq2_len * 4)))) | (channel   << (adc->seq2_len * 4));
		adc_write_reg(adc, ADC_SEQ2_CNR0, tmp);
		adc->seq2_len ++;

		break;
	}

	return 0;
}

static int ingenic_adc_read_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	unsigned int raw_data;
	unsigned int channel;
	unsigned int data;
	unsigned int avail_datas = 0;
	int num_shift = 0;

	channel = chan->channel;
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		channel = chan->channel;
		adc->seq0_len = 0;
		adc_set_bits(adc, ADC_CR, SEQ0_RESET);
		ingenic_adc_set_seq(indio_dev, channel, SEQ0);
		ingenic_adc_setup(indio_dev, chan, SEQ0);

		adc_set_bits(adc, ADC_CR, SEQ0_START);
		/* 轮询等待seq0转换结束 */
		while (!adc_read_bits(adc, ADC_SR, SR_SEQ0_DR));
		avail_datas = adc_read_reg(adc, ADC_SEQ0_DCR) & SEQ0_DCR_CNT_MASK;

		while (avail_datas--) {
			num_shift %= 32;
			if (avail_datas >= 2) {
				raw_data = adc_read_reg(adc, ADC_SEQ0_DR1) & (0xffff << (num_shift));
			}

			else {
				raw_data = adc_read_reg(adc, ADC_SEQ0_DR0) & (0xffff << (num_shift));
			}

			channel = ((raw_data & GENMASK(15, 12)) >> 12);
			data = (raw_data & GENMASK(11, 0));
			num_shift += 16;
		}

#ifdef CONFIG_ADC_DEBUG
		data = 1800 * data / 4096;
		dev_info(adc->dev, "channel = %02d, data = %04dmv\n", channel, data);
#endif
		*val = raw_data;
	}
	return IIO_VAL_INT;

}

int ingenic_adc_reg_access(struct iio_dev *indio_dev, unsigned reg, unsigned writeval, unsigned *readval)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	struct device *dev = adc->dev;

	reg |= 0xa0000000;
	if (readval != NULL) {
		if (ADC_REG_MIN <= reg && reg <= ADC_REG_MAX) {
			*readval = readl((void *)reg);
		} else {
			dump_reg(indio_dev);
		}
	} else {
		if (ADC_REG_MIN <= reg && reg <= ADC_REG_MAX) {
			writel(writeval, (void *)reg);
		} else {
			dev_err(dev, "%s-%d: reg address error\n", __func__, __LINE__);
		}
	}
	return 0;
}

static irqreturn_t ingenic_sadc_irq(int irq, void *dev)
{
	struct iio_dev *indio_dev = dev;
	struct ingenic_adc *adc = iio_priv(indio_dev);
	unsigned int  status;
	unsigned int fifo_status = 0;
	unsigned int fifo_val = 0;
	unsigned int i = 0;
	unsigned int temp_data = 0;
	unsigned int index = 0;
	int pos = 0;

	status = adc_read_reg(adc, ADC_IR);
	if (!status) {
		printk("sadc irq_flag is NULL! \n");
		goto out ;
	}

	if (status & IR_SEQ1_DR) {
		fifo_status = adc_read_reg(adc, ADC_SEQ1_DCR);
		fifo_val  = fifo_status & 0x3f;

		if (fifo_val < adc->seq1_len) {
			adc_set_bits(adc, ADC_IR, IR_SEQ1_DR);
			goto out;
		}

		fifo_val = adc->seq1_len;
		for (i = 0; i < fifo_val; i++) {
			adc->seq1_buf[i] = adc_read_reg(adc, ADC_SEQ1_DR);
		}
		iio_push_to_buffers(indio_dev, &adc->seq1_buf);
		memset(&adc->seq1_buf, 0, 32 * 4);
		adc_set_bits(adc, ADC_IR, IR_SEQ1_DR);
		goto out;
	}
	if (status & IR_SEQ1_DMA_FIN) {
		unsigned int dtc;

		adc_set_bits(adc, ADC_IR, IR_SEQ1_DMA_FIN);

		dtc = adc_read_reg(adc, ADC_SEQ1_DMA_RCNT);
		pos = (dtc - (adc->seq1_len * 4)) / 4;
		if (pos < 0) {
			pos = (DMA_BUFFER_SIZE / 4) + pos;
		}

		for (i = 0; i <  adc->seq1_len; i++, pos++) {
			if (pos >= (DMA_BUFFER_SIZE / 4)) {
				pos = pos % (DMA_BUFFER_SIZE / 4);
			}
			temp_data = adc->seq1rx_buf[pos];
			index = (((temp_data) & GENMASK(15, 12)) >> 12);
			adc->seq1_buf[index] = adc->seq1rx_buf[pos];
		}
		iio_push_to_buffers(indio_dev, &adc->seq1_buf);

		goto out;

	}

out:
	return IRQ_HANDLED;

}

static const unsigned long ingenic_scan_masks[] = {
	0xffffffff,
	0,
};

static const struct iio_info ingenic_adc_iio_info = {
	.read_raw = &ingenic_adc_read_raw,
	.debugfs_reg_access = &ingenic_adc_reg_access,
};

static const struct of_device_id ingenic_adc_of_match_table[] = {
	{ .compatible = "ingenic,x2600-sadc",},
	{ .compatible = "ingenic,ad100-sadc",},
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_adc_of_match_table);

static int ingenic_adc_configure_dma(struct iio_dev *indio_dev)
{
	struct ingenic_adc *adc = iio_priv(indio_dev);
	struct device *dev = adc->dev;
	int ret;

	adc->seq1rxchan = dma_request_slave_channel(dev, "seq1rx");
	if (IS_ERR(adc->seq1rxchan)) {
		dev_err(dev, "SADC request dma seq1_rx channel failed");
		ret = -EBUSY;
		goto error;
	}

	adc->seq1rx_buf = dma_alloc_coherent(dev, DMA_BUFFER_SIZE,
	                                     &adc->seq1rx_dma_buf, GFP_KERNEL);
	if (!adc->seq1rx_buf) {
		dev_err(dev, "SADC request temp dma seq1_buffer failed");
		ret = -ENOMEM;
		goto error;
	}
	memset(adc->seq1rx_buf, 0, DMA_BUFFER_SIZE);

	adc->seq1_sg_rx = devm_kmalloc(adc->dev, sizeof(struct scatterlist), GFP_KERNEL);
	if (!adc->seq1_sg_rx) {
		dev_err(adc->dev, "Failed to alloc seq1_sg_rx scatterlist\n");
		goto error;
	}
	printk("SADC DMA succeeded\n");
	return 0;

error:
	if (adc->seq1rxchan) {
		dma_free_coherent(adc->seq1rxchan->device->dev,
		                  DMA_BUFFER_SIZE,
		                  adc->seq1rx_buf, adc->seq1rx_dma_buf);
		dma_release_channel(adc->seq1rxchan);
	}
	return ret;
}

static int ingenic_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ingenic_adc *adc;
	unsigned int value;
	int ret;
	int i = 0;
	int count = 0;
	unsigned int channel = 0;
	struct iio_chan_spec *chan_array;
	struct iio_chan_spec *chan;

	if (!pdev->dev.of_node) {
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev) {
		return -ENOMEM;
	}
	adc = iio_priv(indio_dev);

	mutex_init(&adc->mutex);
	spin_lock_init(&adc->lock);

	adc->dev = &pdev->dev;
	adc->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adc->base = devm_ioremap_resource(&pdev->dev, adc->mem);
	adc->base_phy = (void __iomem *)adc->mem->start;
	adc->record = 0;

	count = of_property_count_elems_of_size(pdev->dev.of_node, "seq1", 4);

	chan_array = kcalloc(count, sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (chan_array == NULL) {
		return -ENOMEM;
	}

	if (of_property_read_u32(pdev->dev.of_node, "seq1-cont-interval", &value)) {
		adc->seq1_cont_interval = 1;
	} else {
		adc->seq1_cont_interval = value;
	}

	if (IS_ERR(adc->base)) {
		return PTR_ERR(adc->base);
	}

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,has_dma_support", &value)) {
		adc->use_dma = 0;
	} else {
		adc->use_dma = value;
	}

	adc->irq = platform_get_irq(pdev, 0);
	if (adc->irq < 0) {
		return adc->irq;
	}
	ret = devm_request_irq(&pdev->dev,  adc->irq, ingenic_sadc_irq,
	                       0, pdev->name, indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto err_irq;
	}
	adc->clk = devm_clk_get(&pdev->dev, "div_sadc");
	if (IS_ERR(adc->clk)) {
		dev_err(&pdev->dev, "Cannot get div_sadc clock\n");
		ret = -ENOENT;
		goto err_clk;
	}
	adc->gate_clk = devm_clk_get(&pdev->dev, "gate_sadc");
	if (IS_ERR(adc->gate_clk)) {
		dev_err(&pdev->dev, "Cannot get sadc clock\n");
		ret = -ENOENT;
		goto err_clk;
	}

	clk_set_rate(adc->clk, 30000000);

	if (clk_prepare_enable(adc->clk)) {
		dev_err(&pdev->dev, "cgu clk error\n");
		ret = -ENOENT;
		goto err_clk;

	}
	if (clk_prepare_enable(adc->gate_clk)) {
		dev_err(&pdev->dev, "gate clk error\n");
		ret = -ENOENT;
		goto err_clk;
	}

	if (count > 0 && count <= 16) {
		chan = chan_array;
		for (i = 0; i < count; i++, chan++) {
			ret = of_property_read_u32_index(pdev->dev.of_node, "seq1", i, &channel);
			if (ret != 0) {
				return ret;
			}
			ingenic_adc_set_seq(indio_dev, channel, SEQ1);
			chan->type = IIO_VOLTAGE;
			chan->channel = channel;
			chan->indexed = 1;
			chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
			chan->scan_index = i;
			chan->scan_type.sign = 'u';
			chan->scan_type.realbits = 12;
			chan->scan_type.storagebits = 32;
			chan->scan_type.shift = 0;
			chan->scan_type.endianness = IIO_CPU;
		}
	}

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &ingenic_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chan_array;
	indio_dev->num_channels = adc->seq1_len;
	indio_dev->available_scan_masks = ingenic_scan_masks;

	platform_set_drvdata(pdev, indio_dev);


	if (adc->use_dma) {
		ingenic_adc_configure_dma(indio_dev);
	}

	ret = iio_triggered_buffer_setup(indio_dev, &ingenic_pollfunc,
	                                 &ingenic_trigger_handler, NULL);
	if (ret) {
		goto err_clk;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "iio dev register failed\n");
		goto err_device_register;
	}

	dev_info(&pdev->dev, "ingenic sadc dev init succeeded\n");
	return 0;
err_device_register:
	if (adc->clk) {
		clk_put(adc->clk);
	}
	if (adc->gate_clk) {
		clk_put(adc->gate_clk);
	}
err_clk:
	free_irq(adc->irq, adc);
err_irq:
	iounmap(adc->base);
	release_resource(adc->mem);
	return ret;

}

static int ingenic_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ingenic_adc *adc = iio_priv(indio_dev);
	int irq = platform_get_irq(pdev, 0);

	iio_device_unregister(indio_dev);
	free_irq(irq, indio_dev);
	clk_disable(adc->gate_clk);
	clk_disable(adc->clk);
	clk_put(adc->gate_clk);
	clk_put(adc->clk);

	if (adc->seq1rxchan) {
		dma_release_channel(adc->seq1rxchan);
	}

	memset(adc->seq0_buf, 0, sizeof(adc->seq2_buf));
	memset(adc->seq1_buf, 0, sizeof(adc->seq2_buf));
	memset(adc->seq2_buf, 0, sizeof(adc->seq2_buf));

	return 0;
}

static struct platform_driver ingenic_adc_driver = {
	.probe = ingenic_adc_probe,
	.remove = ingenic_adc_remove,
	.driver = {
		.name = "ingenic_adc",
		.of_match_table = ingenic_adc_of_match_table,
		.owner  = THIS_MODULE,
	},
};
module_platform_driver(ingenic_adc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ingenic SADC IIO driver");
