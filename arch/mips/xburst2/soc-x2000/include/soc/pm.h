#ifndef __PM_H__
#define __PM_H__

#include <soc/base.h>
#include <linux/suspend.h>

/*
 *
 *  |-----------------------|
 *  |           |
 *  |           |
 *  |   RESUME_SP   |
 *  |           |
 *  |-----------------------|
 *  |           |
 *  |     PARAM     |
 *  |           |
 *  |-----------------------|
 *  |           |
 *  |           |
 *  |   SLEEP_TEXT      |
 *  |           |
 *  |           |
 *  |-----------------------|
 *  |           |
 *  |           |
 *  |   RESUME_TEXT |
 *  |           |
 *  |           |
 *  |-----------------------|
 *  |           |
 *  |  RESUME_BOOTU_TEXTP   |
 *  |           |
 *  |-----------------------|     <------------ TCSM_START 0xb2400000
 *
 */
#define SRAM_MEMORY_START   0xb2400000
#define SRAM_MEMORY_END     0xb2407ff8
#define RTC_MEMORY_START    0xb0004000
#define RTC_MEMORY_END          0xb0005000


#define NORMAL_RESUME_SP        (SRAM_MEMORY_END - 4)
#define NORMAL_RESUME_CODE1_ADDR    SRAM_MEMORY_START
#define NORMAL_RESUME_CODE1_LEN     64
#define NORMAL_RESUME_CODE2_ADDR    (NORMAL_RESUME_CODE1_ADDR + NORMAL_RESUME_CODE1_LEN)
#define NORMAL_RESUME_CODE2_LEN     (4096)
#define NORMAL_SLEEP_CODE_ADDR      (NORMAL_RESUME_CODE2_ADDR + NORMAL_RESUME_CODE2_LEN)
#define NORMAL_SLEEP_CODE_LEN       (3072)
#define NORMAL_PARAM_ADDR       (NORMAL_SLEEP_CODE_ADDR + NORMAL_SLEEP_CODE_LEN)
#define NORMAL_PARAM_LEN        (sizeof(struct sleep_param))


#define FASTBOOT_RESUME_SP      (FASTBOOT_DATA_ADDR - 4)
#define FASTBOOT_RESUME_CODE1_ADDR  RTC_MEMORY_START
#define FASTBOOT_RESUME_CODE1_LEN   64
#define FASTBOOT_RESUME_CODE2_ADDR  (FASTBOOT_RESUME_CODE1_ADDR + FASTBOOT_RESUME_CODE1_LEN)
#define FASTBOOT_RESUME_CODE_LEN    0xb00
#define FASTBOOT_SLEEP_CODE_ADDR    SRAM_MEMORY_START
#define FASTBOOT_SLEEP_CODE_LEN     4096
#define FASTBOOT_DATA_ADDR      0xb0004c00
#define FASTBOOT_DATA_LEN       1024





struct sleep_param {
	suspend_state_t state;
	unsigned int pdt;
	unsigned int dpd;
	unsigned int dlp;
	unsigned int autorefresh;
	unsigned int cpu_div;
	unsigned int uart_base;
	unsigned int sleep_level;
	unsigned int cpccr;
};

#define sleep_param ((struct sleep_param *)NORMAL_PARAM_ADDR)

enum {
	IDLE,
	IDLE_PD,
	SLEEP,
	FASTBOOT,
};


void load_func_to_tcsm(unsigned int *tcsm_addr, unsigned int *f_addr, unsigned int size);



long long save_goto(unsigned int func);
int restore_goto(unsigned int func);


static inline void rtc_write_reg(unsigned int reg, unsigned int val)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1));
	*(volatile unsigned int *)0xb000303c = 0xa55a;
	while (!((*(volatile unsigned int *)0xb000303c >> 31) & 0x1));
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1));

	*(volatile unsigned int *)reg = val;

	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1));
}

static inline unsigned int  rtc_read_reg(unsigned int reg)
{
	while (!((*(volatile unsigned int *)0xb0003000 >> 7) & 0x1));

	return *(volatile unsigned int *)reg;
}




#define reg_ddr_phy(x)   (*(volatile unsigned int *)(0xb3011000 + ((x) << 2)))



extern int bc_idx;

int pm_get_sleep_level(void);


//#define X2000_SLEEP_GPIO  GPIO_PD(05)

#define readb(addr)     (*(volatile unsigned char *)(addr))
#define readw(addr)     (*(volatile unsigned short *)(addr))
#define readl(addr)     (*(volatile unsigned int *)(addr))
#define writeb(b, addr) (*(volatile unsigned char *)(addr)) = (b)
#define writew(b, addr) (*(volatile unsigned short *)(addr)) = (b)
#define writel(b, addr) (*(volatile unsigned int *)(addr)) = (b)

#define GPIO_PXPIN(n)   (0x00 + (n) * 0x100) /* PIN Level Register */
#define GPIO_PXINT(n)   (0x10 + (n) * 0x100) /* Port Interrupt Register */
#define GPIO_PXINTS(n)  (0x14 + (n) * 0x100) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)  (0x18 + (n) * 0x100) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)  (0x20 + (n) * 0x100) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n) (0x24 + (n) * 0x100) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n) (0x28 + (n) * 0x100) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)  (0x30 + (n) * 0x100) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n) (0x34 + (n) * 0x100) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n) (0x38 + (n) * 0x100) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)  (0x40 + (n) * 0x100) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n) (0x44 + (n) * 0x100) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n) (0x48 + (n) * 0x100) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)   (0x50 + (n) * 0x100) /* Port Flag Register */
#define GPIO_PXFLGC(n)  (0x58 + (n) * 0x100) /* Port Flag clear Register */
#define GPIO_PXPU(n)    (0x80 + (n) * 0x100) /* PORT PULL-UP State Register */
#define GPIO_PXPUS(n)   (0x84 + (n) * 0x100) /* PORT PULL-UP State Set Register*/
#define GPIO_PXPUC(n)   (0x88 + (n) * 0x100) /* PORT PULL-UP State Clear Register */
#define GPIO_PXPD(n)    (0x90 + (n) * 0x100) /* PORT PULL-DOWN State Register */
#define GPIO_PXPDS(n)   (0x94 + (n) * 0x100) /* PORT PULL-DOWN State Set Register */
#define GPIO_PXPDC(n)   (0x98 + (n) * 0x100) /* PORT PULL-DOWN State Set Register */

#define gpio_readl(o)       readl(0xb0010000 + (o))
#define gpio_writel(b, o)   writel(b, 0xb0010000 + (o))

#define cpm_readl(o)        readl(0xb0000000 + (o))
#define cpm_writel(b, o)    writel(b, 0xb0000000 + (o))

/************************************************
 *      debug interface
 ***********************************************/

#define DEBUG_PM
#define PRINT_DEBUG


#define U_IOBASE    (sleep_param->uart_base)
#define UART_OFF        (0x1000)


#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)
#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)


#ifdef PRINT_DEBUG
#define TCSM_PCHAR(x)                                                   \
	*((volatile unsigned int*)(U_IOBASE+OFF_TDR)) = x;              \
	while ((*((volatile unsigned int*)(U_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))
#else
#define TCSM_PCHAR(x)
#endif

#define TCSM_DELAY(x)                       \
	do{                         \
		register unsigned int i = x;                \
		while(i--)                      \
			__asm__ volatile("nop\n\t");            \
	}while(0)

static inline void serial_put_hex(unsigned int x)
{
	int i;
	unsigned int d;
	for (i = 7; i >= 0; i--) {
		d = (x  >> (i * 4)) & 0xf;
		if (d < 10) {
			d += '0';
		} else {
			d += 'A' - 10;
		}
		TCSM_PCHAR(d);
	}
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
}


#endif
