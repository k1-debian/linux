#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <soc/base.h>
#include <soc/gpio.h>

#define GPIO_PORT_OFF    0x100
#define GPIO_SHADOW_OFF  0x700

#define PXPIN      0x00   /* PIN Level Register */
#define PXINT      0x10   /* Port Interrupt Register */
#define PXINTS     0x14   /* Port Interrupt Set Register */
#define PXINTC     0x18   /* Port Interrupt Clear Register */
#define PXMSK      0x20   /* Port Interrupt Mask Reg */
#define PXMSKS     0x24   /* Port Interrupt Mask Set Reg */
#define PXMSKC     0x28   /* Port Interrupt Mask Clear Reg */
#define PXPAT1     0x30   /* Port Pattern 1 Set Reg. */
#define PXPAT1S    0x34   /* Port Pattern 1 Set Reg. */
#define PXPAT1C    0x38   /* Port Pattern 1 Clear Reg. */
#define PXPAT0     0x40   /* Port Pattern 0 Register */
#define PXPAT0S    0x44   /* Port Pattern 0 Set Register */
#define PXPAT0C    0x48   /* Port Pattern 0 Clear Register */
#define PXFLG      0x50   /* Port Flag Register */
#define PXFLGC     0x58   /* Port Flag clear Register */
#define PXPU       0x80   /* Port PULL-UP State Register */
#define PXPUS      0x84   /* Port PULL-UP State Set Register */
#define PXPUC      0x88   /* Port PULL-UP State Clear Register */
#define PXPD       0x90   /* Port PULL-DOWN State Register */
#define PXPDS      0x94   /* Port PULL-DOWN State Set Register */
#define PXPDC      0x98   /* Port PULL-DOWN State Clear Register */

#define PZGID2LD   0xF0   /* GPIOZ Group ID to load */

#define SHADOW 5

#define PE_PORT     4
#define PE_TYPED    (BIT(22) | BIT(23) | BIT(24) | BIT(25) | BIT(26) | BIT(27))

static const unsigned long gpiobase[] = {
	[0] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 0 * GPIO_PORT_OFF),
	[1] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 1 * GPIO_PORT_OFF),
	[2] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 2 * GPIO_PORT_OFF),
	[3] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 3 * GPIO_PORT_OFF),
	[4] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 4 * GPIO_PORT_OFF),

	[SHADOW] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + GPIO_SHADOW_OFF),
};

#define GPIO_ADDR(port, reg) ((volatile unsigned long *)(gpiobase[port] + reg))

static inline void gpio_write(int port, unsigned int reg, int val)
{
	*GPIO_ADDR(port, reg) = val;
}

static inline unsigned int gpio_read(int port, unsigned int reg)
{
	return *GPIO_ADDR(port, reg);
}

static void hal_gpio_port_set_func(int port, unsigned int pins, enum gpio_function func)
{
	/* func option */
	if (func & 0x10) {
		if (func & 0x8) {
			gpio_write(SHADOW, PXINTS, pins);
		} else {
			gpio_write(SHADOW, PXINTC, pins);
		}

		if (func & 0x4) {
			gpio_write(SHADOW, PXMSKS, pins);
		} else {
			gpio_write(SHADOW, PXMSKC, pins);
		}

		if (func & 0x2) {
			gpio_write(SHADOW, PXPAT1S, pins);
		} else {
			gpio_write(SHADOW, PXPAT1C, pins);
		}

		if (func & 0x1) {
			gpio_write(SHADOW, PXPAT0S, pins);
		} else {
			gpio_write(SHADOW, PXPAT0C, pins);
		}

		/* configure PzGID2LD to specify which port group to load */
		gpio_write(SHADOW, PZGID2LD, port);
	}

	if (func & 0x80) {
		int pull = (func >> 5) & 0x3;

		/* TYPED only support pull-down function and the setting in the pull-up register */
		if (pull && port == PE_PORT && pins & PE_TYPED) {
			pull--;
		}

		if (pull == 0) { // no pull
			gpio_write(port, PXPUC, pins);
			gpio_write(port, PXPDC, pins);
		}
		if (pull == 1) { // pull up
			gpio_write(port, PXPDC, pins);
			gpio_write(port, PXPUS, pins);
		}
		if (pull == 2) { // pull down
			gpio_write(port, PXPUC, pins);
			gpio_write(port, PXPDS, pins);
		}
	}
}

unsigned long ingenic_pinctrl_lock(int port);
void ingenic_pinctrl_unlock(int port, unsigned long flags);

int jzgpio_set_func(int port, enum gpio_function func, unsigned long pins)
{
	unsigned long flags;

	if (port < 0 || port > 4) {
		printk(KERN_ERR "gpio: invalid gpio port for x2000: %d\n", port);
		return -EINVAL;
	}

	flags = ingenic_pinctrl_lock(port);

	hal_gpio_port_set_func(port, pins, func);

	ingenic_pinctrl_unlock(port, flags);

	return 0;
}
EXPORT_SYMBOL(jzgpio_set_func);
