#ifndef __INGENIC_RPROC_RISCV_H__
#define __INGENIC_RPROC_RISCV_H__

#define CCU_BASE    0xB2200000

#define XBURST2_CCU_CFCR            (0x0fe0)

#define CCU_CCSR                    (0x0 * 4)
#define CCU_CRER                    (0x1 * 4)
#define CCU_FROM_HOST               (0x2 * 4)
#define CCU_TO_HOST                 (0x3 * 4)
#define CCU_TIME_L                  (0x4 * 4)
#define CCU_TIME_H                  (0x5 * 4)
#define CCU_TIME_CMP_L              (0x6 * 4)
#define CCU_TIME_CMP_H              (0x7 * 4)
#define CCU_INTC_MASK_L             (0x8 * 4)
#define CCU_INTC_MASK_H             (0x9 * 4)
#define CCU_INTC_PEND_L             (0xa * 4)
#define CCU_INTC_PEND_H             (0xb * 4)

#define CCU_PMA_ADR(n)              ((0x10 + n) * 4)
#define CCU_PMA_ADR0                (0x10 * 4)
#define CCU_PMA_ADR1                (0x11 * 4)
#define CCU_PMA_ADR2                (0x12 * 4)
#define CCU_PMA_ADR3                (0x13 * 4)
#ifdef CONFIG_SOC_X2600
	#define CCU_PMA_ADR4                (0x14 * 4)
	#define CCU_PMA_ADR5                (0x15 * 4)
	#define CCU_PMA_ADR6                (0x16 * 4)
	#define CCU_PMA_ADR7                (0x17 * 4)
#endif
#define CCU_PMA_CFG(n)              ((0x18 + n) * 4)
#define CCU_PMA_CFG0                (0x18 * 4)
#define CCU_PMA_CFG1                (0x19 * 4)
#define CCU_PMA_CFG2                (0x1a * 4)
#define CCU_PMA_CFG3                (0x1b * 4)
#ifdef CONFIG_SOC_X2600
	#define CCU_PMA_CFG4                (0x1c * 4)
	#define CCU_PMA_CFG5                (0x1d * 4)
	#define CCU_PMA_CFG6                (0x1e * 4)
	#define CCU_PMA_CFG7                (0x1f * 4)
#endif

#define PMA_CFG_READ_BIT 1
#define PMA_CFG_WRITE_BIT 2
#define PMA_CFG_EXECUT_BIT 0
#define PMA_CFG_CACHE_BIT 5
#define PMA_CFG_ADDR_BIT 3

#ifdef CONFIG_SOC_X2600
	#define INGENIC_REG_WDT_COUNTER_ENABLE (0x4)
	#define INGENIC_REG_WDT_TIMER_CONTROL  (0xC)
	#define INGENIC_REG_WDT_TIMER_FLAG_RD  (0x20)
	#define INGENIC_REG_WDT_TIMER_FLAG_CLR (0x28)

	#define INGENIC_WDT_TIMER_FFLAG        BIT(25)
	#define INGENIC_WDT_CLOCK_CLRZ         BIT(10)
#endif

/*libbare-cpu*/
#define RSC_TABLE_START_OFFSET  0x200
#define RSC_TABLE_END_OFFSET    0x204

struct ingenic_riscv_rproc {
	struct reset_control *rst;
	struct rproc *rproc;
	void __iomem *wdt_base;
	int wdt_irq;

	int irq;
	int vqid;
	void __iomem *ccu_base;
	void __iomem *riscv_base;
	unsigned int load_addr;
	struct device *dev;
	struct resource *res;
	struct completion remote_msg_done;
	struct work_struct mbx_work;

	struct kobject *sysfs_kobj;
#ifdef CONFIG_INGENIC_RISCV_IPC
	struct ingenic_ipc_device *ipc_dev;
#endif
};

void riscv_writel(struct ingenic_riscv_rproc *jz_rproc, unsigned int val, unsigned int offset);
unsigned int riscv_readl(struct ingenic_riscv_rproc *jz_rproc, unsigned int offset);
void ccu_writel(struct ingenic_riscv_rproc *jz_rproc, unsigned int val, unsigned int offset);
unsigned int ccu_readl(struct ingenic_riscv_rproc *jz_rproc, unsigned int offset);

#endif /*__INGENIC_RPROC_RISCV_H__*/
