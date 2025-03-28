#ifndef __INGENIC_MCU_H__
#define __INGENIC_MCU_H__

#include <soc/base.h>
/* MCU controller Regs */
#define DMCS    0x1030
#define DMNMB   0x1034
#define DMSMB   0x1038
#define DMINT   0x103c

#define DMINT_N_IP  (1 << 16)
#define DMINT_N_IMSK    (1 << 0)

#define TCSM_BASE_ADDR  0xb3422000

#define MCU_STATE_SHUTDOWN  1
#define MCU_STATE_BOOTUP    2

#define CPM_SFTINT 0xbc

#define TCSM_MAX_SIZE (32 * 1024)
#define TCSM_IO_BASE 0x13422000
#define TCSM_MCU_VIRT_BASE 0xF4000000

#define TCSM_ADDR ((volatile unsigned long *)CKSEG1ADDR(TCSM_IO_BASE))

#define MCU_ADDR(reg) ((volatile unsigned long *)CKSEG1ADDR(PDMA_IOBASE+reg))

#define CPM_ADDR(reg) ((volatile unsigned long *)CKSEG1ADDR(CPM_IOBASE+reg))


struct ingenic_mcu {
	void __iomem    *reg_base;

	struct device   *dev;
#ifdef CONFIG_INGENIC_MCU_IPC
	struct ingenic_ipc_device *ipc_dev;
#endif
	struct clk  *clk;
	struct clk  *intc_clk;
	int     irq_pdmam;
	unsigned int    tcsm_size;
	int     state;


};

int register_mcu_notifier(struct notifier_block *nb);
int unregister_mcu_notifier(struct notifier_block *nb);

#endif /*__INGENIC_MCU_H__*/
