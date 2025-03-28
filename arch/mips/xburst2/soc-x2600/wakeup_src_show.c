#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/printk.h>

struct intc_regs {
	volatile unsigned int ICSR0;    /* 0x00*/
	volatile unsigned int ICMR0;    /* 0x04 */
	volatile unsigned int ICMSR0;   /* 0x08 */
	volatile unsigned int ICMCR0;   /* 0x0C */
	volatile unsigned int ICPR0;    /* 0x10 */
	volatile unsigned int reserved[3];
	volatile unsigned int ICSR1;    /* 0x20 */
	volatile unsigned int ICMR1;    /* 0x24 */
	volatile unsigned int ICMSR1;   /* 0x28 */
	volatile unsigned int ICMCR1;   /* 0x2C */
	volatile unsigned int ICPR1;    /* 0x30 */
	volatile unsigned int DSR0;     /* 0x34 */
	volatile unsigned int DMR0;     /* 0x38 */
	volatile unsigned int DPR0;     /* 0x3C */
	volatile unsigned int DSR1;     /* 0x40 */
	volatile unsigned int DMR1;     /* 0x44 */
	volatile unsigned int DPR1;     /* 0x48 */
};

#define INTC_IO_BASE                    0xB2300000
#define INTC_IO_OFFSET                  0x1000

extern void show_gpio_wakeup_sources(int port);

static const char *intc0_src_name[32] = {
	"Audio",
	"OTG",
	"USB",
	"PDMA",
	"PDMA Desc",
	"PDMA MCU",
	"SSI SLV",
	"SFC",
	"SSI1",
	"SSI0",
	"MIPI DSI",
	"SADC",
	"DMIC",
	"GPIOE",    /* GPIO4 */
	"GPIOD",    /* GPIO3 */
	"GPIOC",    /* GPIO2 */
	"GPIOB",    /* GPIO1 */
	"GPIOA",    /* GPIO0 */
	"LEP",      /* RISC-V */
	"TPC",
	"PWM",
	"DTRNG",
	"HASH",
	"AES",
	"WDT1_TIMER",
	"TCU1",
	"TCU0",
	"FELIX",
	"G2D",
	"ROTATE",
	"CIM",
	"LCD",
};

static const char *intc1_src_name[32] = {
	"RTC",
	"SOFT",     /* software */
	"CPM",
	"RESERVED",
	"MSC1",
	"MSC0",
	"CAN1",
	"CAN0",
	"UART7",
	"UART6",
	"UART5",
	"UART4",
	"UART3",
	"UART2",
	"UART1",
	"UART0",
	"HMAN_MCU",
	"HMAN2",
	"HMAN0",
	"DDR",
	"PCM",
	"RESERVED",
	"EFUSE",
	"MAC",
	"I2C3",
	"I2C2",
	"I2C1",
	"I2C0",
	"MCU_PDMA",
	"MCU_PDMA_Desc",
	"JPEGE",
	"JPEGD",
};


void show_wakeup_sources(void)
{
	int cpu_id = smp_processor_id();
	struct intc_regs *intc = (void *)INTC_IO_BASE + cpu_id * INTC_IO_OFFSET;
	int i;

	unsigned int reg = intc->ICPR0;

	printk("-----------------\n");
	/* 当有多个中断时，具体那个中断唤醒请查看 pm_wakeup_irq() 返回的中断号 */
	for (i = 0; i < 32; ++i) {
		if (reg & (1 << i)) {
			printk("WAKE UP by INTC0: bit[%d] -> %s\n", i, intc0_src_name[i]);
			if (i >= 13 && i <= 17) {
				show_gpio_wakeup_sources(17 - i);
			}
		}
	}

	reg = intc->ICPR1;

	for (i = 0; i < 32; ++i) {
		if (reg & (1 << i)) {
			printk("WAKE UP by INTC1: bit[%d] -> %s\n", i, intc1_src_name[i]);
		}
	}
	printk("-----------------\n");
}
