#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/printk.h>

struct intc_regs {
	volatile unsigned int ICSR0;    /* 0x00*/
	volatile unsigned int ICMR0;	/* 0x04 */
	volatile unsigned int ICMSR0;	/* 0x08 */
	volatile unsigned int ICMCR0;	/* 0x0C */
	volatile unsigned int ICPR0;	/* 0x10 */
	volatile unsigned int reserved[3];
	volatile unsigned int ICSR1;	/* 0x20 */
	volatile unsigned int ICMR1;	/* 0x24 */
	volatile unsigned int ICMSR1;	/* 0x28 */
	volatile unsigned int ICMCR1;	/* 0x2C */
	volatile unsigned int ICPR1;	/* 0x30 */
	volatile unsigned int DSR0;		/* 0x34 */
	volatile unsigned int DMR0;		/* 0x38 */
	volatile unsigned int DPR0;		/* 0x3C */
	volatile unsigned int DSR1;		/* 0x40 */
	volatile unsigned int DMR1;		/* 0x44 */
	volatile unsigned int DPR1;		/* 0x48 */
};

#define INTC_IO_BASE					0xB0001000

extern void show_gpio_wakeup_sources(int port);

static const char *intc0_src_name[32] = {
	"DMIC",
	"AIC0",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"SFC",
	"SSI1",
	"RESERVED",
	"PDMA",
	"PDMA Desc",
	"RESERVED",
	"RESERVED",
	"GPIOD",	/* GPIO3 */
	"GPIOC",	/* GPIO2 */
	"GPIOB",	/* GPIO1 */
	"GPIOA",	/* GPIO0 */
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"OTG",
	"RESERVED",
	"AES",
	"RESERVED",
	"TCU2"
	"TCU1",
	"TCU0",
	"RESERVED",
	"RESERVED",
	"CIM",
	"LCD",
};

static const char *intc1_src_name[32] = {
	"RTC",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"MSC1",
	"MSC0",
	"SCC",
	"RESERVED",
	"PCM0",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"HARB2",
	"RESERVED",
	"HARB0",
	"RESERVED",
	"UART2",
	"UART1",
	"UART0",
	"DDR",
	"RESERVED",
	"EFUSE",
	"MAC",
    "RESERVED",
    "RESERVED",
	"I2C2",
	"I2C1",
	"I2C0",
	"PDMA MCU",
	"JPEG",
	"RESERVED",
};


void show_wakeup_sources(void)
{
	struct intc_regs *intc = (void *)INTC_IO_BASE;
	int i;

	unsigned int reg = intc->ICPR0;

	printk("-----------------\n");
	/* 当有多个中断时，具体那个中断唤醒请查看 pm_wakeup_irq() 返回的中断号 */
	for (i = 0; i < 32; ++i) {
		if (reg & (1 << i)) {
			printk("WAKE UP by INTC0: bit[%d] -> %s\n", i, intc0_src_name[i]);
			if (i >= 14 && i <= 17)
				show_gpio_wakeup_sources(17 - i);
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
