#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/random.h>
//#include <mach/jzdma_v13.h>
#include <linux/vmalloc.h>
#include <linux/list.h>
#include <ingenic_dma.h>

#include "multithread_test.h"

struct cpu_dma_test {
	void *multithread;
	struct list_head dma_top;
} cpu_dma;
struct dma_channel {
	struct list_head list;
	struct dma_chan *chan;
	struct hdma_desc *desc;
	void *from;
	unsigned long pfrom;
	void *to;
	unsigned long pto;
	unsigned long test_len;
	struct mutex    lock;
	unsigned long test_cnt;
};
static void init_buf(struct dma_channel *dc)
{
	int i;

	unsigned long *p = dc->from;
	struct device *dev;
	dev = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);
	for (i = 0; i < dc->test_len / 4; i++) {
		p[i] = (unsigned int)p + i * 4;
	}

	p = dc->to;
	for (i = 0; i < dc->test_len / 4; i++) {
		p[i] = 0x5a5a5a5a;
	}
	dma_sync_single_for_device(dev, (dma_addr_t)(virt_to_phys(dc->from)), dc->test_len, DMA_TO_DEVICE);
	dma_sync_single_for_device(dev, (dma_addr_t)(virt_to_phys(dc->to)), dc->test_len, DMA_FROM_DEVICE);
}

static int cpu_dma_test_func_nowait(void *param)
{
	struct dma_channel *dc = (struct dma_channel *)param;
	int i;
	mutex_lock(&dc->lock);
	init_buf(dc);
	{
		struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(dc->chan);
		writel((unsigned int)virt_to_phys(dc->desc), dmac->iomem + CH_DDA);
		/* initiate descriptor fetch */
		writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
		writel(1 | (1 << 30), dmac->iomem + CH_DCS);
	}

	dc->test_cnt ++;
	{
		volatile unsigned long *p = (unsigned long *)CKSEG1ADDR(dc->to);
		unsigned long e = (unsigned long)dc->from;
		int timeout = 0x10000;
		for (i = 0; i < dc->test_len / 4; i++) {
			timeout = 0x10000;
			while ((p[i] != e) && --timeout);
			if (timeout == 0) {
				printk("xxxxxx error data is not ready! xxxxxx\n");
				while (1);
			}
			e += 4;
		}
	}
	mutex_unlock(&dc->lock);
#ifdef CONFIG_FPGA_TEST
	mdelay(100);
#endif
	return 0;
}
static int cpu_dma_test_func_wait(void *param)
{
	struct dma_channel *dc = (struct dma_channel *)param;
	unsigned long *p = (unsigned long *)((unsigned long)dc->to + dc->test_len - 4);
	unsigned long e = (unsigned long)dc->from + dc->test_len - 4;
	unsigned long r;
	int timeout = 0x10000;

	struct device *dev;
	dev = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);

	mutex_lock(&dc->lock);
	init_buf(dc);
	{
		struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(dc->chan);

		writel((unsigned int)virt_to_phys(dc->desc), dmac->iomem + CH_DDA);
		/* initiate descriptor fetch */
		writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
		writel(1 | (1 << 30), dmac->iomem + CH_DCS);
		while (!(readl(dmac->iomem + CH_DCS) & (1 << 3)) && timeout--);
	}

	dc->test_cnt ++;

	{
		do {
			r = *p;
			if (r != e) {
				unsigned long *uc = (unsigned long *)((unsigned long)p | 0xa0000000);

				printk("error:from:%px to: %px data[%px] real: 0x%08lx, uc:0x%08lx expect: 0x%08lx, test_cnt: %ld\n",
				       dc->from, dc->to, p, r, *uc, e, dc->test_cnt);
				break;
			}
			p--;
			e -= 4;
		} while ((unsigned long)p != (unsigned long)dc->to - 4);
	}
	if (timeout == 0) {
		printk("xxxxxx error dmac is not ready! xxxxxx\n");
		while (1);
	}

	mutex_unlock(&dc->lock);
#ifdef CONFIG_FPGA_TEST
	mdelay(100);
#endif
	return 0;
}
static bool filter(struct dma_chan *chan, void *param)
{
#if 0
	if (!((unsigned int)chan->private == 8)) {
		return false;
	}
#endif
	return true;
}
const static char dcm_tsz[8] = { 1, 2, 0, 0, 3, 4, 5, 6 };
static inline unsigned int get_max_tsz(unsigned long val, unsigned long *dcmp)
{

	int ord;

	ord = ffs(val) - 1;
	if (ord < 0) {
		ord = 0;
	} else if (ord > 7) {
		ord = 7;
	}

	*dcmp &= ~DCM_TSZ_MSK;
	*dcmp |= dcm_tsz[ord] << DCM_TSZ_SFT;

	/* if tsz == 8, set it to 4 */
	return ord == 3 ? 4 : 1 << ord;
}
static struct dma_channel *new_dma_channel(struct cpu_dma_test *cd)
{
	struct dma_channel *ch;
	struct device *dev;
	dev = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);
	ch = (struct dma_channel *)vmalloc(sizeof(struct dma_channel));
	memset(ch, 0, sizeof(struct dma_channel));
	if (ch == NULL) {
		printk("mem alloc failed! %s %d\n", __FILE__, __LINE__);
		return 0;
	}

	mutex_init(&ch->lock);

	ch->test_len = 4096;
	ch->from = (void *)kmalloc(ch->test_len, GFP_KERNEL);
	ch->to = (void *)kmalloc(ch->test_len, GFP_KERNEL);
	if (ch->to == 0 || ch->from == 0) {
		printk("mem alloc failed! %s %d\n", __FILE__, __LINE__);
		goto error_exit;
	}
	ch->pfrom = virt_to_phys(ch->from);
	ch->pto = virt_to_phys(ch->to);

	{
		dma_cap_mask_t mask;
		struct hdma_desc *desc;
		unsigned long burst_len = 128;
		unsigned long burst_bits = 0;
		dma_cap_zero(mask);
		dma_cap_set(DMA_MEMCPY, mask);
		ch->chan = dma_request_channel(mask, filter, 0);
		if (!ch->chan) {
			printk("dma channel request failed! %s %d\n", __FILE__, __LINE__);
			goto error_exit;
		}
		ch->desc = (struct hdma_desc *)kmalloc(128, GFP_KERNEL);
		if (!ch->desc) {
			printk("dma channel request desc failed! %s %d\n", __FILE__, __LINE__);
			goto error_exit;
		}
		desc = ch->desc;
		burst_len = get_max_tsz(burst_len, &burst_bits);
		desc->dcm = DCM_SAI | DCM_DAI | burst_bits;
		desc->dsa = ch->pfrom;
		desc->dta = ch->pto;
		desc->dtc = ch->test_len / burst_len;
		desc->drt = 8;
		dma_sync_single_for_device(dev, (dma_addr_t)(virt_to_phys(desc)), 128, DMA_TO_DEVICE);

	}

	list_add_tail(&ch->list, &cd->dma_top);
	return ch;
error_exit:
	if (ch->desc) {
		kfree(ch->desc);
	}
	if (ch->from) {
		kfree(ch->from);
	}
	if (ch->to) {
		kfree(ch->to);
	}
	vfree(ch);
	return 0;
}
static void free_dma_channel(struct dma_channel *dc)
{
	if (dc->desc) {
		kfree(dc->desc);
	}
	if (dc->from) {
		kfree(dc->from);
	}
	if (dc->to) {
		kfree(dc->to);
	}
	vfree(dc);
}
static int __init cpu_dma_test_init(void)
{
	struct dma_channel *dc;
	INIT_LIST_HEAD(&cpu_dma.dma_top);
	cpu_dma.multithread = multithread_test_init("cpu_dma_test", 2);
	if (cpu_dma.multithread) {
		dc = new_dma_channel(&cpu_dma);
		if (dc) {
			multithread_test_add_func(cpu_dma.multithread, "cpu-dma-nowait", cpu_dma_test_func_nowait, (void *)dc);
		}

		dc = new_dma_channel(&cpu_dma);
		if (dc) {
			multithread_test_add_func(cpu_dma.multithread, "cpu-dma-wait", cpu_dma_test_func_wait, (void *)dc);
		}

		printk("cpu_dma_test register ok.\n");
	}
	return 0;
}
static void __exit cpu_dma_test_deinit(void)
{
	struct dma_channel *dc, *_dc;

	list_for_each_entry_safe(dc, _dc, &cpu_dma.dma_top, list) {
		list_del(&dc->list);
		free_dma_channel(dc);
	}
	multithread_test_deinit(cpu_dma.multithread);
}
late_initcall(cpu_dma_test_init);
module_exit(cpu_dma_test_deinit);
