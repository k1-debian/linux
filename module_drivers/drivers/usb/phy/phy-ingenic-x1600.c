#include <linux/delay.h>
#include <linux/syscore_ops.h>

#include "phy-ingenic.h"

#define CPM_USBPCR                      (0x3C)
#define CPM_USBRDT                      (0x40)
#define CPM_USBVBFIL                    (0x44)
#define CPM_USBPCR1                     (0x48)

#define CPM_SRBC                        (0xC4)
#define CPM_OPCR                        (0x24)

#define OPCR_SPENDN0_BIT            7
#define OPCR_GATE_USBPHY_CLK_BIT    23
#define SRBC_USB_SR                 12

#define USBRDT_RESUME_IRQ_ENABLE            31
#define USBRDT_RESUME_CLEAR_IRQ             30
#define USBRDT_RESUME_SPEED             28
#define USBRDT_RESUME_STATUS                27

static struct usb_phy_data *x1600_usb_phy;
static unsigned int x1600_set_wakeup;

int x1600_phy_suspend(void)
{
	unsigned int value;

	if (!x1600_usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(x1600_usb_phy, 0x108);
		value |= 1 << 3;
		usb_phy_writel(x1600_usb_phy, value, 0x108);
	}

	/* disable full/low speed driver at the receiver */
	value = usb_phy_readl(x1600_usb_phy, 0x100);
	value &= ~(1 << 6);
	usb_phy_writel(x1600_usb_phy, value, 0x100);

	usb_cpm_clear_bit(x1600_usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (x1600_usb_phy->usb_wakeup && x1600_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(x1600_usb_phy, CPM_USBRDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((x1600_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(x1600_usb_phy, value, CPM_USBRDT);

		usb_cpm_clear_bit(x1600_usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
		usb_cpm_set_bit(x1600_usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	return 0;
}

void x1600_phy_resume(void)
{
	unsigned int value;

	if (x1600_usb_phy->usb_wakeup && x1600_set_wakeup) {
		usb_cpm_clear_bit(x1600_usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	usb_cpm_set_bit(x1600_usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	/* enable full/low speed driver at the receiver */
	value = usb_phy_readl(x1600_usb_phy, 0x100);
	value |= 1 << 6;
	usb_phy_writel(x1600_usb_phy, value, 0x100);

	if (!x1600_usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power on. */
		value = usb_phy_readl(x1600_usb_phy, 0x108);
		value &= ~(1 << 3);
		usb_phy_writel(x1600_usb_phy, value, 0x108);
	}
}

static struct syscore_ops x1600_phy_syscore_ops = {
	.suspend = x1600_phy_suspend,
	.resume = x1600_phy_resume,
};

static int x1600_priv_data_init(struct usb_phy_data *usb_phy)
{
	/* reset usb */
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	x1600_usb_phy = usb_phy;
	register_syscore_ops(&x1600_phy_syscore_ops);

	return 0;
}

static int x1600_phy_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0x00200000, CPM_USBPCR1);
	usb_cpm_writel(usb_phy, 0x80400000, CPM_USBPCR);
	udelay(500);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x70000000, CPM_USBPCR1);
	udelay(500);

	/* always enable pre-emphasis */
	value = usb_phy_readl(usb_phy, 0x30);
	// value &= ~(0x7 << 0);
	value |= 0x7 << 0;
	usb_phy_writel(usb_phy, value, 0x30);

	/* Tx HS pre_emphasize strength configure */
	value = usb_phy_readl(usb_phy, 0x40);
	// value &= ~(0x7 << 3);
	value |= 0x7 << 3;
	usb_phy_writel(usb_phy, value, 0x40);

	/* Vbus 5V mode */
	value = usb_phy_readl(usb_phy, 0x10C);
	value &= ~((0x7 << 0) | (0x7 << 3));
	value |= ((0x5 << 0) | (0x5 << 3));
	usb_phy_writel(usb_phy, value, 0x10C);

	/* Vbus 5V mode */
	value = usb_phy_readl(usb_phy, 0x110);
	value &= ~((0x7 << 0) | (0x7 << 3));
	value |= ((0x5 << 0) | (0x5 << 3));
	usb_phy_writel(usb_phy, value, 0x110);

	if (usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(usb_phy, 0x108);
		value |= 0x1 << 3;
		usb_phy_writel(usb_phy, value, 0x108);
	}

	return 0;
}

static int x1600_phy_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	x1600_set_wakeup = enabled;

	return 0;
}

static int x1600_phy_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

static enum usb_device_power x1600_phy_get_power(struct usb_phy_data *usb_phy)
{
	enum usb_device_power type = USB_POWER_UNKNOWN;

	usb_phy_writel(usb_phy, 0x60, 0x104);
	usleep_range(20000, 20000);
	if (usb_phy_readl(usb_phy, 0x110) & (1 << 5)) {
		usb_phy_writel(usb_phy, 0x90, 0x104);
		usleep_range(20000, 20000);
		if (usb_phy_readl(usb_phy, 0x10C) & (1 << 1)) {
			usb_phy_writel(usb_phy, 0x0c, 0x104);
			usleep_range(20000, 20000);
			if (usb_phy_readl(usb_phy, 0x10C) & (1 << 0)) {
				type = USB_POWER_DCP;
			} else {
				type = USB_POWER_CDP;
			}
		} else {
			type = USB_POWER_SDP;
		}
	}

	usb_phy_writel(usb_phy, 0, 0x104);

	printk("%s: %s\n", __func__, usb_power_string(type));
	return type;
}

struct usb_phy_priv usb_phy_x1600_priv = {
	.priv_data_init = x1600_priv_data_init,

	.phy_init = x1600_phy_init,
	.phy_set_wakeup = x1600_phy_set_wakeup,
	.phy_get_wakeup = x1600_phy_get_wakeup,
	.phy_get_power = x1600_phy_get_power,
};
