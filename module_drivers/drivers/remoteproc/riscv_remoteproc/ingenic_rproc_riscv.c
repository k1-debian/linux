#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "remoteproc_internal.h"
#include "ingenic_rproc_riscv.h"
#include "ingenic_rproc_riscv_sysfs.h"
#ifdef CONFIG_INGENIC_RISCV_IPC
	#include "../ingenic_ipc/ingenic_ipc.h"
#endif

#define CCU_MASG_REG    (0x2 * 4)
#define CCU_RECV_REG    (0x3 * 4)

#define TX_CH0_MASK (1 << 31)
#define RX_CH0_MASK (1 << 15)

#define REMOTE_REC_OK   0x1231

#define MAX_VQRING_NUM  2

#define VQRING_TX_INDEX 0
#define VQRING_RX_INDEX 1

#define KICK_REMOTE_TX  0xAA
#define KICK_REMOTE_RX  0x55

#define RISCV_RUN   0x0
#define RISCV_STATE_MASK    0x80000000

extern unsigned int arch_get_ddr_size(void);

void riscv_writel(struct ingenic_riscv_rproc *jz_rproc, unsigned int val, unsigned int offset)
{
	writel(val, jz_rproc->riscv_base + offset);
}

unsigned int riscv_readl(struct ingenic_riscv_rproc *jz_rproc, unsigned int offset)
{
	return readl(jz_rproc->riscv_base + offset);
}

void ccu_writel(struct ingenic_riscv_rproc *jz_rproc, unsigned int val, unsigned int offset)
{
	writel(val, jz_rproc->ccu_base + offset);
}

unsigned int ccu_readl(struct ingenic_riscv_rproc *jz_rproc, unsigned int offset)
{
	return readl(jz_rproc->ccu_base + offset);
}

static RAW_NOTIFIER_HEAD(riscv_notifier_chain);

int register_riscv_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&riscv_notifier_chain, nb);
}
EXPORT_SYMBOL(register_riscv_notifier);

int unregister_riscv_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&riscv_notifier_chain, nb);
}
EXPORT_SYMBOL(unregister_riscv_notifier);

int riscv_notifier_call_chain(unsigned long val)
{
	return raw_notifier_call_chain(&riscv_notifier_chain, val, NULL);
}
EXPORT_SYMBOL(riscv_notifier_call_chain);



static irqreturn_t ingenic_rproc_irq_handler(int irq, void *data)
{
	struct rproc *rproc = (struct rproc *)data;
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int tmp = 0;
	// process msg
	//disable_irq_nosync(irq);
	tmp = riscv_readl(jz_rproc, CCU_TO_HOST);
	if (tmp & 0x80000000) {
		printk("shared mailbox irq\n");
		return 0;
	}
	riscv_writel(jz_rproc, 0, CCU_TO_HOST);
#ifdef CONFIG_RPMSG
	int vqid = 0;
	vqid = tmp & 0xff;
	if (vqid > 0) {
		vqid -= 1;
	}
	rproc_vq_interrupt(rproc, vqid);
#endif

	riscv_notifier_call_chain(tmp);

	return IRQ_HANDLED;
}

#ifdef CONFIG_SOC_X2600
static irqreturn_t ingenic_rproc_wdt_irq_handler(int irq, void *data)
{
	struct rproc *rproc = (struct rproc *)data;
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	if (readl(jz_rproc->wdt_base + INGENIC_REG_WDT_TIMER_FLAG_RD) & INGENIC_WDT_TIMER_FFLAG) {
		unsigned int val = readl(jz_rproc->wdt_base + INGENIC_REG_WDT_TIMER_CONTROL);
		writel(val | INGENIC_WDT_CLOCK_CLRZ, jz_rproc->wdt_base + INGENIC_REG_WDT_TIMER_CONTROL);
		writel(0, jz_rproc->wdt_base + INGENIC_REG_WDT_COUNTER_ENABLE);

		writel(INGENIC_WDT_TIMER_FFLAG, jz_rproc->wdt_base + INGENIC_REG_WDT_TIMER_FLAG_CLR);
		rproc_report_crash(rproc, RPROC_WATCHDOG);
	}
	return IRQ_HANDLED;
}
#endif

static void ingenic_rproc_kick(struct rproc *rproc, int vqid)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	if (vqid >= MAX_VQRING_NUM || vqid < 0) {
		printk("warning vqid error ,vqid = %d ,rporc kick failed\n", vqid);
		return;
	}

	riscv_writel(jz_rproc, (vqid + 1), CCU_FROM_HOST);
	riscv_readl(jz_rproc, CCU_FROM_HOST); //sync write
}

static int ingenic_rproc_mem_alloc(struct rproc *rproc,
                                   struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_err(dev, "map memory: %pa+%x\n", &mem->dma, mem->len);
	va = ioremap(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pa+%x\n",
		        &mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;
	printk("%s : va = %p mem = %p \n ", __func__, va, mem);
	return 0;
}

static int ingenic_rproc_mem_release(struct rproc *rproc,
                                     struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

// parse reserved mem  && init virtio ring buffer
static int ingenic_rproc_parse_mem_regions(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	int index = 0;

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}
		if (strcmp(it.node->name, "vdev0buffer")) {
			// vring buffer
			mem = rproc_mem_entry_init(dev, NULL,
			                           (dma_addr_t)rmem->base,
			                           rmem->size, rmem->base,
			                           ingenic_rproc_mem_alloc,
			                           ingenic_rproc_mem_release,
			                           it.node->name);

			if (mem)
				rproc_coredump_add_segment(rproc, rmem->base,
				                           rmem->size);
		} else {
			// for vdev0buffer
			mem = rproc_of_resm_mem_entry_init(dev, index,
			                                   rmem->size,
			                                   rmem->base,
			                                   it.node->name);
		}
		printk("%s : rmem-base = 0x%x rmem->size = %d va = %lx\n", __func__, rmem->base, rmem->size, (unsigned long)mem->va);
		if (!mem) {
			return -ENOMEM;
		}
		rproc_add_carveout(rproc, mem);
		index++;
	}
	// add risv0vring0 for TX,riscv-core SRAM
	//
	return 0;

}

#if 0
static int ingenic_rproc_start_x2500(struct rproc *rproc)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int val = 0;
	unsigned int reset_entry = jz_rproc->load_addr + 0x80;

	val = riscv_readl(jz_rproc, CCU_CCSR);
	val &= ~(1 << 1);
	riscv_writel(jz_rproc, val, CCU_CCSR);

	/*XBURST2_LEP_STOP*/
	val = ccu_readl(jz_rproc, XBURST2_CCU_CFCR);
	val |= (1 << 31);
	ccu_writel(jz_rproc, val, XBURST2_CCU_CFCR);

	// Reset value
	riscv_writel(jz_rproc, 0x43, CCU_CCSR);
	riscv_writel(jz_rproc, reset_entry, CCU_CRER);
	riscv_writel(jz_rproc, 0, CCU_FROM_HOST);
	riscv_writel(jz_rproc, 0, CCU_TO_HOST);
	riscv_writel(jz_rproc, 0, CCU_INTC_MASK_L);
	riscv_writel(jz_rproc, 0, CCU_INTC_MASK_H);
	riscv_writel(jz_rproc, 0, CCU_TIME_L);
	riscv_writel(jz_rproc, 0, CCU_TIME_H);
	riscv_writel(jz_rproc, -1, CCU_TIME_CMP_L);
	riscv_writel(jz_rproc, -1, CCU_TIME_CMP_H);
	riscv_writel(jz_rproc, 0x1e01, CCU_PMA_CFG0);
	riscv_writel(jz_rproc, 0x2F03, CCU_PMA_CFG1);
	riscv_writel(jz_rproc, 0x1f01, CCU_PMA_CFG2);
	riscv_writel(jz_rproc, 0x1e01, CCU_PMA_CFG3);
	riscv_writel(jz_rproc, 0x04000000, CCU_PMA_ADR0);
	riscv_writel(jz_rproc, 0x05ffffff, CCU_PMA_ADR1);
	riscv_writel(jz_rproc, 0x1fffffff, CCU_PMA_ADR2);
	riscv_writel(jz_rproc, 0x00000000, CCU_PMA_ADR3);
	/*XBURST2_LEP_RUN*/
	val = ccu_readl(jz_rproc, XBURST2_CCU_CFCR);
	val &= ~(1 << 31);
	ccu_writel(jz_rproc, val, XBURST2_CCU_CFCR);
	return 0;
}
#endif

static int ingenic_rproc_start_x2600(struct rproc *rproc)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int val = 0;
	unsigned int reset_entry = jz_rproc->load_addr + 0x80;

	val = riscv_readl(jz_rproc, CCU_CCSR);
	val &= ~(1 << 1);
	riscv_writel(jz_rproc, val, CCU_CCSR);

	/*XBURST2_LEP_STOP*/
	val = ccu_readl(jz_rproc, XBURST2_CCU_CFCR);
	//If the status is RUNNING, subsequent operations are performed by the JDI debugger
	if ((val >> 31) == 0) {
		return 0;
	}
	val |= (1 << 31);
	ccu_writel(jz_rproc, val, XBURST2_CCU_CFCR);

	// Reset value
	riscv_writel(jz_rproc, 0x23, CCU_CCSR);
	riscv_writel(jz_rproc, reset_entry, CCU_CRER);
	riscv_writel(jz_rproc, 0, CCU_FROM_HOST);
	riscv_writel(jz_rproc, 0, CCU_TO_HOST);
	riscv_writel(jz_rproc, 0, CCU_INTC_MASK_L);
	riscv_writel(jz_rproc, 0, CCU_INTC_MASK_H);
	riscv_writel(jz_rproc, 0, CCU_TIME_L);
	riscv_writel(jz_rproc, 0, CCU_TIME_H);
	riscv_writel(jz_rproc, -1, CCU_TIME_CMP_L);
	riscv_writel(jz_rproc, -1, CCU_TIME_CMP_H);
	/*XBURST2_LEP_RUN*/
	val = ccu_readl(jz_rproc, XBURST2_CCU_CFCR);
	val &= ~(1 << 31);
	ccu_writel(jz_rproc, val, XBURST2_CCU_CFCR);

	return 0;
}

#ifdef CONFIG_INGENIC_RISCV_IPC
static void *riscv_to_host_addr(unsigned long addr)
{
	return (void *)CKSEG1ADDR(addr);
}

static struct ring_mem *riscv_get_ring_mem_for_host_write(void *data)
{
	struct ingenic_riscv_rproc *jzproc = (struct ingenic_riscv_rproc *)data;
	struct ingenic_ipc_rsc_table *rsc_table = jzproc->ipc_dev->rsc_table;

	if (!rsc_table) {
		return NULL;
	}

	return (void *)CKSEG1ADDR(rsc_table->data_from_host);
}
static struct ring_mem *riscv_get_ring_mem_for_host_read(void *data)
{
	struct ingenic_riscv_rproc *jzproc = (struct ingenic_riscv_rproc *)data;
	struct ingenic_ipc_rsc_table *rsc_table = jzproc->ipc_dev->rsc_table;

	if (!rsc_table) {
		return NULL;
	}

	return (void *)CKSEG1ADDR(rsc_table->data_to_host);
}

static void riscv_notify_remote(struct ingenic_ipc_device *ipc_dev)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)ipc_dev->data;
	while (riscv_readl(jz_rproc, CCU_FROM_HOST));
	riscv_writel(jz_rproc, 1, CCU_FROM_HOST);
}

static struct ingenic_ipc_ops ipc_ops = {
	.notify_remote = riscv_notify_remote,
	.get_ring_mem_for_host_write = riscv_get_ring_mem_for_host_write,
	.get_ring_mem_for_host_read = riscv_get_ring_mem_for_host_read,
	.to_host_addr = riscv_to_host_addr,
	.notifier_register = register_riscv_notifier,
	.notifier_unregister = unregister_riscv_notifier,

};
#endif

static int ingenic_rproc_start(struct rproc *rproc)
{
	int ret = 0;
#ifdef CONFIG_SOC_X2500
	ret = ingenic_rproc_start_x2500(rproc);
#elif CONFIG_SOC_X2600
	ret = ingenic_rproc_start_x2600(rproc);
#endif
#ifdef CONFIG_INGENIC_RISCV_IPC
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	struct resource_table *table_ptr = rproc->table_ptr;

	jz_rproc->ipc_dev = ingenic_ipc_device_register(jz_rproc, &ipc_ops, "riscv");

	if (!table_ptr) {
		jz_rproc->ipc_dev->rsc_table = NULL;
	} else {
		jz_rproc->ipc_dev->rsc_table = (struct ingenic_ipc_rsc_table *)((void *)table_ptr + table_ptr->offset[0]);
	}

#endif
	return ret;
}

static int ingenic_rproc_stop(struct rproc *rproc)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int val = 0;

	/*XBURST2_LEP_STOP*/
	val = ccu_readl(jz_rproc, XBURST2_CCU_CFCR);
	val |= (1 << 31);
	ccu_writel(jz_rproc, val, XBURST2_CCU_CFCR);

#ifdef CONFIG_INGENIC_RISCV_IPC
	ingenic_ipc_device_unregister(jz_rproc->ipc_dev);

#endif

	return 0;
}

static int ingenic_rproc_attach(struct rproc *rproc)
{
	return 0;
}

static void *ingenic_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	void *vaddr = NULL;

	if (da >= 0 && da <= 0x20000000) {
		vaddr = (void *)CKSEG1ADDR(da);
	} else {
		printk("ERROR: %llu address no support\n", da);
	}
	return vaddr;
}

static unsigned int ingenic_get_riscv_state(struct ingenic_riscv_rproc *jz_rproc)
{
	return readl(jz_rproc->ccu_base + 0xFE0);
}

static int ingenic_rproc_load_bin(struct rproc *rproc, const struct firmware *fw)
{
	struct ingenic_riscv_rproc *jzproc = (struct ingenic_riscv_rproc *)rproc->priv;
	void *load_vaddr = (void *) phys_to_virt(jzproc->load_addr);

	const unsigned char *bin_data = fw->data;
	if (fw->size) {
		printk(" %s firmware size = %d load_vaddr = %p #########\n", __func__, fw->size, load_vaddr);
		memcpy(load_vaddr, bin_data, fw->size);
		dma_cache_wback_inv((phys_addr_t)load_vaddr, fw->size);
	}
	return 0;
}

static int ingenic_rproc_load(struct rproc *rproc, const struct firmware *fw)
{
	const char *firmware = rproc->firmware;
	if (!strcmp(strrchr(firmware, '.'), ".elf")) {
		return rproc_elf_load_segments(rproc, fw);
	} else if (!strcmp(strrchr(firmware, '.'), ".bin")) {
		return ingenic_rproc_load_bin(rproc, fw);
	} else {
		dev_err(&rproc->dev, "Unsupported firmware format.\n");
		return -EINVAL;
	}
}

static int ingenic_rproc_bin_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int rsc_table_start, rsc_table_end;
	struct resource_table *table = NULL;
	size_t tablesz;

	ingenic_rproc_load_bin(rproc, fw);

	rsc_table_start = (*(unsigned int *)phys_to_virt(jz_rproc->load_addr + RSC_TABLE_START_OFFSET));
	rsc_table_end = (*(unsigned int *)phys_to_virt(jz_rproc->load_addr + RSC_TABLE_END_OFFSET));

	if (rsc_table_start == rsc_table_end) {
		return -EINVAL;
	}

	table = (struct resource_table *)phys_to_virt(rsc_table_start);
	tablesz = rsc_table_end - rsc_table_start;

	/*
	 * Create a copy of the resource table. When a virtio device starts
	 * and calls vring_new_virtqueue() the address of the allocated vring
	 * will be stored in the cached_table. Before the device is started,
	 * cached_table will be copied into device memory.
	 */
	rproc->cached_table = kmemdup(table, tablesz, GFP_KERNEL);
	if (!rproc->cached_table) {
		return -ENOMEM;
	}

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = tablesz;

	return 0;
}

static int ingenic_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	const char *firmware = rproc->firmware;
	int ret = 0;
	ingenic_rproc_parse_mem_regions(rproc);

	if (!strcmp(strrchr(firmware, '.'), ".elf")) {
		ret = rproc_elf_load_rsc_table(rproc, fw);
	} else if (!strcmp(strrchr(firmware, '.'), ".bin")) {
		ret = ingenic_rproc_bin_load_rsc_table(rproc, fw);
	} else {
		dev_err(&rproc->dev, "Unsupported firmware format.\n");
		return -EINVAL;
	}
	if (ret) {
		dev_warn(&rproc->dev, "no resource table found for this firmware\n");
	}
	return 0;
}

static struct resource_table *ingenic_rproc_bin_find_loaded_rsc_table(struct rproc *rproc,
        const struct firmware *fw)
{
	struct ingenic_riscv_rproc *jz_rproc = (struct ingenic_riscv_rproc *)rproc->priv;
	unsigned int rsc_table_start, rsc_table_end;
	u64 sh_addr, sh_size;

	rsc_table_start = (*(unsigned int *)phys_to_virt(jz_rproc->load_addr + RSC_TABLE_START_OFFSET));
	rsc_table_end = (*(unsigned int *)phys_to_virt(jz_rproc->load_addr + RSC_TABLE_END_OFFSET));

	if (rsc_table_start == rsc_table_end) {
		return NULL;
	}

	sh_addr = (u64)rsc_table_start;
	sh_size = rsc_table_end - rsc_table_start;

	if (!rproc_u64_fit_in_size_t(sh_size)) {
		dev_err(&rproc->dev, "size (%llx) does not fit in size_t type\n",
		        sh_size);
		return NULL;
	}

	return rproc_da_to_va(rproc, sh_addr, sh_size, NULL);
}

static struct resource_table *ingenic_rproc_find_loaded_rsc_table(struct rproc *rproc,
        const struct firmware *fw)
{
	const char *firmware = rproc->firmware;
	if (!strcmp(strrchr(firmware, '.'), ".elf")) {
		return rproc_elf_find_loaded_rsc_table(rproc, fw);
	} else if (!strcmp(strrchr(firmware, '.'), ".bin")) {
		return ingenic_rproc_bin_find_loaded_rsc_table(rproc, fw);
	} else {
		dev_err(&rproc->dev, "Unsupported firmware format.\n");
		return NULL;
	}
}

static struct rproc_ops ingenic_rproc_ops = {
	.start   = ingenic_rproc_start,
	.stop    = ingenic_rproc_stop,
	.attach  = ingenic_rproc_attach,
	.kick    = ingenic_rproc_kick,
	.da_to_va = ingenic_rproc_da_to_va,
	.load    = ingenic_rproc_load,
	.parse_fw = ingenic_rproc_parse_fw,
	.find_loaded_rsc_table = ingenic_rproc_find_loaded_rsc_table,
};

static int ingenic_rproc_riscv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct rproc *rproc;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ingenic_riscv_rproc *jz_rproc;
	struct resource *wdt_res;

	unsigned int state = 0;
	rproc = rproc_alloc(dev, np->name, &ingenic_rproc_ops, NULL, sizeof(struct ingenic_riscv_rproc));
	if (!rproc) {
		dev_err(&pdev->dev, "alloc rproc failed !!\n");
		return -ENOMEM;
	}

	jz_rproc = rproc->priv;

	// test
	jz_rproc->rproc = rproc;
	jz_rproc->dev = &pdev->dev;

	init_completion(&jz_rproc->remote_msg_done);
	jz_rproc->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!jz_rproc->res) {
		dev_err(&pdev->dev, "get dev resource failed\n");
		ret = -EINVAL;
		goto err_get_platform_res;
	}
	jz_rproc->res = request_mem_region(jz_rproc->res->start,
	                                   jz_rproc->res->end - jz_rproc->res->start + 1,
	                                   pdev->name);
	if (!jz_rproc->res) {
		dev_err(&pdev->dev, "request regs memory failed\n");
		ret = -EINVAL;
		goto err_get_mem_region;
	}

	jz_rproc->riscv_base = ioremap(jz_rproc->res->start, resource_size(jz_rproc->res));
	if (!jz_rproc->riscv_base) {
		dev_err(&pdev->dev, "remap regbase failed \n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	struct device_node *mcu_ram_node = of_parse_phandle(np, "mcu-ram", 0);
	if (!mcu_ram_node) {
		dev_err(jz_rproc->dev, "No RAM Region defined for RISCV, please check dts!\n");
		goto err_riscv_ram;
	}

	unsigned int reg[2];
	unsigned char reg_name[16] = {0};
	unsigned int ddr_size = arch_get_ddr_size();
	if (ddr_size > 256) {
		ddr_size = 256;
	}
	sprintf(reg_name, "reg-%dM", ddr_size);
	if (of_property_read_u32_array(mcu_ram_node, reg_name, reg, 2)) {
		if (of_property_read_u32_array(mcu_ram_node, "reg", reg, 2)) {
			dev_err(jz_rproc->dev, "reserved memory riscv_mcu_ram defined with error, please check!\n");
			goto err_get_loadaddr;
		}
	}

	jz_rproc->load_addr = reg[0];

	jz_rproc->ccu_base = (void *)CCU_BASE;

	jz_rproc->irq = platform_get_irq(pdev, 0);

	ret = devm_request_threaded_irq(jz_rproc->dev, jz_rproc->irq, NULL, ingenic_rproc_irq_handler, IRQF_ONESHOT | IRQF_SHARED, rproc->name, rproc);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		ret = -EINVAL;
		goto err_request_irq;
	}

#ifdef CONFIG_SOC_X2600
	wdt_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (wdt_res) {
		jz_rproc->wdt_base = ioremap(wdt_res->start, resource_size(wdt_res));
		if (jz_rproc->wdt_base) {
			jz_rproc->irq = platform_get_irq(pdev, 1);
			if (jz_rproc->irq >= 0) {
				ret = devm_request_irq(dev, jz_rproc->irq, ingenic_rproc_wdt_irq_handler, IRQF_SHARED, "rproc-wdt", rproc);
				if (ret) {
					dev_err(&pdev->dev, "request wdt-irq failed\n");
				}
			}
		}
	}
#endif

	//rproc_coredump_set_elf_info(rproc, ELFCLASS32,25);

	// todo add reserved mem init,
	// riscv boot addr prased from dts
	//ingenic_jz_rproc_prase_dt(pdev,jz_rproc);

	ingenic_rproc_parse_mem_regions(rproc);

	state = ingenic_get_riscv_state(jz_rproc);
	if ((state & RISCV_STATE_MASK) == RISCV_RUN) {
		// uboot has runing the riscv-core
		ingenic_rproc_parse_mem_regions(rproc);
		// TODO get elf info
	} else {

	}

	platform_set_drvdata(pdev, jz_rproc);

	ret = rproc_add(rproc);
	if (ret) {
		goto err_add_rproc;
	}

#if CONFIG_SOC_X2600
	ret = ingenic_rproc_riscv_sysfs_init(jz_rproc);
	if (ret) {
		goto err_sysfs;
	}
#endif

	printk("[rproc  %d ] %s end !!!!\n", __LINE__, __func__);
	return 0;
err_sysfs:
err_add_rproc:
err_request_irq:
err_get_loadaddr:
err_riscv_ram:
	iounmap(jz_rproc->ccu_base);
err_ioremap:
	release_mem_region(jz_rproc->res->start, jz_rproc->res->end - jz_rproc->res->start + 1);
err_get_mem_region:
err_get_platform_res:
	rproc_free(rproc);
	return ret;
}

static const struct of_device_id ingenic_rproc_riscv_of_matches[] = {
	{ .compatible = "ingenic,x2500-riscv-rproc", },
	{ .compatible = "ingenic,x2600-riscv-rproc", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_rproc_riscv_of_matches);

static struct platform_driver ingenic_rproc_riscv_driver = {
	.probe = ingenic_rproc_riscv_probe,
	.driver = {
		.name = "ingenic-riscv",
		.of_match_table = ingenic_rproc_riscv_of_matches,
	},
};
module_platform_driver(ingenic_rproc_riscv_driver);

MODULE_LICENSE("GPL");
