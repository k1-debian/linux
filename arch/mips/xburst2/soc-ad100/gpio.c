#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <soc/base.h>
#include <soc/gpio.h>

#define GPIO_PORT_OFF    0x1000

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

#define PXSPCFG    0x200
#define PXSPSEL0   0x204
#define PXMPCFG0   0x208

#define PxGLD      0x8F0
#define PXINTSS    0x814
#define PXINTCS    0x818

#define PXMSKSS    0x824
#define PXMSKCS    0x828

#define PXPAT1SS   0x834
#define PXPAT1CS   0x838
#define PXPAT0SS   0x844
#define PXPAT0CS   0x848

#define PIN_NUM    16, 20
#define EDG        5, 5
#define INT        4, 4
#define MSK        3, 3
#define MMSK       2, 2
#define PAT1       1, 1
#define PAT0       0, 0

#define PZGID2LD   0xF0   /* GPIOZ Group ID to load */

#define PE_PORT     4

static const unsigned long gpiobase[] = {
	[0] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 0 * GPIO_PORT_OFF),
	[1] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 1 * GPIO_PORT_OFF),
	[2] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 2 * GPIO_PORT_OFF),
	[3] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 3 * GPIO_PORT_OFF),
	[4] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 4 * GPIO_PORT_OFF),
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

static inline unsigned long bit_field_max(int start, int end)
{
	return (1ul << (end - start + 1)) - 1;
}

static inline unsigned long bit_field_mask(int start, int end)
{
	return bit_field_max(start, end) << start;
}

static inline void set_bit_field(volatile unsigned long *reg, int start, int end, unsigned long val)
{
	unsigned long mask = bit_field_mask(start, end);
	*reg = (*reg & ~mask) | ((val << start) & mask);
}

static void hal_gpio_set_pull_func(int port, enum gpio_function func, unsigned int pins)
{
	int pull = (func >> 5) & 0x3;

	if (port == PE_PORT) {
		if (pull == 0) { // no pull
			gpio_write(port, PXPUC, pins);
		} else {
			gpio_write(port, PXPUS, pins);
		}
		if (pull == 1) { // pull up
			gpio_write(port, PXPDC, pins);
		}
		if (pull == 2) { // pull down
			gpio_write(port, PXPDS, pins);
		}
	} else {
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

static void hal_gpio_set_func(int port, unsigned int pins, enum gpio_function func)
{
	/* func option */
	if (func & 0x10) {
		if (func & 0x8) {
			gpio_write(port, PXINTSS, pins);
		} else {
			gpio_write(port, PXINTCS, pins);
		}

		if (func & 0x4) {
			gpio_write(port, PXMSKSS, pins);
		} else {
			gpio_write(port, PXMSKCS, pins);
		}

		if (func & 0x2) {
			gpio_write(port, PXPAT1SS, pins);
		} else {
			gpio_write(port, PXPAT1CS, pins);
		}

		if (func & 0x1) {
			gpio_write(port, PXPAT0SS, pins);
		} else {
			gpio_write(port, PXPAT0CS, pins);
		}

		/* configure PzGID2LD to specify which port group to load */
		gpio_write(port, PxGLD, 0x01);
	}

	if (func & 0x80) {
		hal_gpio_set_pull_func(port, func, pins);
	}
}

static void hal_gpio_set_func1(int port, unsigned int pin, enum gpio_function func)
{
	/* func option */
	if (func & 0x10) {
		unsigned long pxspcfg = 0;
		if (func & 0x8) {
			set_bit_field(&pxspcfg, INT, 1);
		}

		if (func & 0x4) {
			set_bit_field(&pxspcfg, MSK, 1);
		}

		if (func & 0x2) {
			set_bit_field(&pxspcfg, PAT1, 1);
		}

		if (func & 0x1) {
			set_bit_field(&pxspcfg, PAT0, 1);
		}

		set_bit_field(&pxspcfg, PIN_NUM, pin);
		gpio_write(port, PXSPCFG, pxspcfg);
	}

	if (func & 0x80) {
		hal_gpio_set_pull_func(port, func, BIT(pin));
	}
}

static void hal_gpio_port_set_func(int port, unsigned long pins, enum gpio_function func)
{
	/* func option */
	if (func & 0x10) {
		unsigned long pxmpcfg0 = 0;
		if (func & 0x8) {
			set_bit_field(&pxmpcfg0, INT, 1);
		}

		if (func & 0x4) {
			set_bit_field(&pxmpcfg0, MSK, 1);
		}

		if (func & 0x2) {
			set_bit_field(&pxmpcfg0, PAT1, 1);
		}

		if (func & 0x1) {
			set_bit_field(&pxmpcfg0, PAT0, 1);
		}

		gpio_write(port, PXSPSEL0, pins);
		gpio_write(port, PXMPCFG0, pxmpcfg0);
	}

	if (func & 0x80) {
		hal_gpio_set_pull_func(port, func, pins);
	}
}

typedef int (*gpio_cb)(int port, enum gpio_function func, unsigned long pin);
struct gpio_set {
	gpio_cb single;
	gpio_cb multi;
};

unsigned long ingenic_pinctrl_lock(int port);
void ingenic_pinctrl_unlock(int port, unsigned long flags);

int jzgpio_set_func(int port, enum gpio_function func, unsigned long pins)
{
	unsigned long flags;

	if (port < 0 || port > 4) {
		printk(KERN_ERR "gpio: invalid gpio port for ad100: %d\n", port);
		return -EINVAL;
	}

	flags = ingenic_pinctrl_lock(port);

	hal_gpio_set_func(port, pins, func);

	ingenic_pinctrl_unlock(port, flags);

	return 0;
}
EXPORT_SYMBOL(jzgpio_set_func);

static int jzgpio_set_func1(int port, enum gpio_function func, unsigned long pin)
{
	unsigned long flags;

	if (port < 0 || port > 4) {
		printk(KERN_ERR "gpio: invalid gpio port for ad100: %d\n", port);
		return -EINVAL;
	}

	flags = ingenic_pinctrl_lock(port);

	hal_gpio_set_func1(port, pin, func);

	ingenic_pinctrl_unlock(port, flags);

	return 0;
}

static int jzgpio_port_set_func(int port, enum gpio_function func, unsigned long pins)
{
	unsigned long flags;

	if (port < 0 || port > 4) {
		printk(KERN_ERR "gpio: invalid gpio port for ad100: %d\n", port);
		return -EINVAL;
	}

	flags = ingenic_pinctrl_lock(port);

	hal_gpio_port_set_func(port, pins, func);

	ingenic_pinctrl_unlock(port, flags);

	return 0;
}

static struct gpio_set set_func = {
	.single = jzgpio_set_func1,
	.multi = jzgpio_port_set_func,
};

static struct platform_device gpio_dev = {
	.name           = "ingenic-utils-gpiodev",
	.id             = -1,
	.dev = {
		.platform_data = &set_func,
	},
};

static int __init init_gpio(void)
{
	int ret;

	ret = platform_device_register(&gpio_dev);
	if (ret) {
		printk(KERN_ERR "gpio: Failed to register aic dev: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(init_gpio);
