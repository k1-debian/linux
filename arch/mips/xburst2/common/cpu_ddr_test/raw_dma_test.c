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
#include <ingenic_dma.h>


const int dma_test_burst_len[] = {1, 4, 8, 16, 32, 64, 128};

struct dma_channel {
	struct list_head list;
	struct dma_chan *chan;
	unsigned char *desc;
	unsigned int blen_per_desc;
};

struct cycle_raw_dma_test {
	struct dentry       *root;
	int run;
	struct mutex        lock;
	struct list_head    top;
};

static struct cycle_raw_dma_test g_raw_cycle_dma;

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
static void dump_desc(char *title, struct hdma_desc *desc)
{
	printk("=========== %s =========== \n", title);
	printk("dcm:0x%08lx\n", desc->dcm);
	printk("dsa:0x%08x\n", (unsigned int)desc->dsa);
	printk("dta:0x%08x\n", (unsigned int)desc->dta);
	printk("dtc:0x%08lx\n", desc->dtc);
	printk("drt:0x%08lx\n", desc->drt);

}
static void build_desc(unsigned char *desc, int burst, int len)
{
	/**
	 *    4 series A  B  A' B' desc.
	 *    A desc is transmite (A B) to (A' B')
	 *    A next desc is B'
	 *    B desc is transmite (A' B') to (A B)
	 *    B next desc is A
	 */
	unsigned long burst_bits = 0;
	struct hdma_desc *A = (struct hdma_desc *)desc;
	struct hdma_desc *B = (struct hdma_desc *)(desc + len * 1);
	struct hdma_desc *A1 = (struct hdma_desc *)(desc + len * 2);
	struct hdma_desc *B1 = (struct hdma_desc *)(desc + len * 3);
	struct device *dev;

	dev = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);

	burst = get_max_tsz(burst, &burst_bits);
	A->dcm = DCM_SAI | DCM_DAI | DCM_LINK | burst_bits;
	A->dsa = virt_to_phys(A);
	A->dta = virt_to_phys(A1);
	A->dtc = (len * 2 / burst) | ((((unsigned int)B1 & 0xff0) >> 4) << 24);
	A->drt = 8;

	B->dcm = DCM_SAI | DCM_DAI | DCM_LINK | burst_bits;
	B->dsa = virt_to_phys(A1);
	B->dta = virt_to_phys(A);
	B->dtc = (len * 2 / burst) | ((((unsigned int)A & 0xff0) >> 4) << 24);
	B->drt = 8;

	dump_desc("A", A);
	dump_desc("B", B);
	dma_sync_single_for_device(dev, (dma_addr_t)(virt_to_phys(A)), len * 4, DMA_TO_DEVICE);
}
static void start_dma_chans(struct cycle_raw_dma_test *dma)
{
	struct dma_channel *ch;
	list_for_each_entry(ch, &dma->top, list) {
		struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(ch->chan);
		writel((unsigned int)virt_to_phys(ch->desc), dmac->iomem + CH_DDA);
		printk("dda:%px->0x%08lx\n", dmac->iomem + CH_DDA, virt_to_phys(ch->desc));
		/* initiate descriptor fetch */
		writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
		printk("ddrs:%px->0x%08lx\n", dmac->engine->iomem + DDRS, BIT(dmac->id));
		writel(1 | (1 << 30), dmac->iomem + CH_DCS);
	}
}
static void stop_dma_chans(struct cycle_raw_dma_test *dma)
{
	struct dma_channel *ch;
	struct device *dev;

	dev = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);

	list_for_each_entry(ch, &dma->top, list) {
		struct hdma_desc *A, *B;
		struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(ch->chan);
		int len = ch->blen_per_desc;
		A = (struct hdma_desc *)(ch->desc);
		B = (struct hdma_desc *)(ch->desc + len * 1);
		A->dtc &= ~(0xff << 24);
		B->dtc &= ~(0xff << 24);
		dma_sync_single_for_device(dev, (dma_addr_t)(virt_to_phys(A)), len * 4, DMA_TO_DEVICE);
		while (readl(dmac->iomem + CH_DCS) & (1 << 3));
		writel(0, dmac->iomem + CH_DCS);
	}

}
static void start_cycle_raw_dma(struct cycle_raw_dma_test *dma)
{
	int i;
	int max;
	struct dma_channel *ch;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	for (i = 0; i < ARRAY_SIZE(dma_test_burst_len); i++) {
		chan = dma_request_channel(mask, filter, dma);
		if (!chan) {
			printk("dma channel request failed! curid = %d, burstlen: %d ignored\n", i, dma_test_burst_len[i]);
			//          break;
			continue;
		}
		if (dma_test_burst_len[i] < 32) {
			max = 32;
		} else {
			max = dma_test_burst_len[i];
		}

		//extra add 16 byte for descriptor align.
		ch = kmalloc(sizeof(struct dma_channel) + max * 4 + 16, GFP_KERNEL); // max * 4 is  (4's descs)
		if (IS_ERR_OR_NULL(ch)) {
			pr_err("dma request channel failed!\n");
			dma_release_channel(chan);
			break;
		}
		ch->chan = chan;
		ch->desc = (unsigned char *)(((unsigned int)(ch + 1) + 15) / 16 * 16);  // aligned 16 byte for dma descriptor
		ch->blen_per_desc = max;
		build_desc(ch->desc, dma_test_burst_len[i], max);
		list_add_tail(&ch->list, &dma->top);
	}
	start_dma_chans(dma);
}
static void stop_cycle_raw_dma(struct cycle_raw_dma_test *dma)
{

	struct list_head *pos;
	struct list_head *next;
	struct dma_channel *ch;
	stop_dma_chans(dma);
	list_for_each_safe(pos, next, &dma->top) {
		ch = list_entry(pos, struct dma_channel, list);
		dma_release_channel(ch->chan);
		list_del(pos);
		kfree(ch);
	}
}
static int is_stop_cycle_raw_dma(struct cycle_raw_dma_test *dma)
{
	return list_empty(&dma->top);
}
static ssize_t cycle_raw_dma_test_read_run(struct file *file, char __user *user_buf,
        size_t count, loff_t *ppos)
{
	struct cycle_raw_dma_test *dma = file->private_data;
	char *buf;
	mutex_lock(&dma->lock);
	if (!is_stop_cycle_raw_dma(dma)) {
		buf = "Runing\n";
	} else {
		buf = "Stop\n";
	}
	mutex_unlock(&dma->lock);
	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}
static ssize_t cycle_raw_dma_test_write_run(struct file *file, const char __user *user_buf,
        size_t count, loff_t *ppos)
{
	struct cycle_raw_dma_test *dma = file->private_data;
	char buf[16];
	bool bv;
	int ret = 0;
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	if (strtobool(buf, &bv) == 0) {
		mutex_lock(&dma->lock);
		if (bv) {
			start_cycle_raw_dma(dma);
		} else {
			stop_cycle_raw_dma(dma);
		}
		mutex_unlock(&dma->lock);
	}

	return ret ? ret : count;
}

static const struct file_operations cycle_raw_dma_test_run_fops = {
	.read   = cycle_raw_dma_test_read_run,
	.write  = cycle_raw_dma_test_write_run,
	.open   = simple_open,
	.llseek = default_llseek,
};
static int __init cycle_raw_dma_test_init(void)
{
	struct dentry *d;

	struct cycle_raw_dma_test *dma = &g_raw_cycle_dma;

	memset(dma, 0, sizeof(struct cycle_raw_dma_test));
	INIT_LIST_HEAD(&dma->top);
	mutex_init(&dma->lock);
	d = debugfs_create_dir("cycle_raw_dma_test", NULL);
	if (IS_ERR(d)) {
		pr_err("create debugfs for cycle_raw_dma_test failed.\n");
		return PTR_ERR(d);
	}
	dma->root = d;

	d = debugfs_create_file("run", S_IWUSR | S_IRUGO, dma->root,
	                        dma, &cycle_raw_dma_test_run_fops);
	if (IS_ERR_OR_NULL(d)) {
		pr_err("debugfs create run node failed\n");
		goto err_node;
	}
	pr_info("cycle raw dma_test debugfs init ok.\n");
	return 0;
err_node:
	debugfs_remove_recursive(dma->root);
	pr_err("cycle_raw_dma_test debugfs init failed.\n");
	return -1;
}
late_initcall(cycle_raw_dma_test_init);

static void __exit cycle_raw_dma_test_deinit(void)
{
	struct cycle_raw_dma_test *dma = &g_raw_cycle_dma;
	debugfs_remove_recursive(dma->root);
}
module_exit(cycle_raw_dma_test_deinit);
