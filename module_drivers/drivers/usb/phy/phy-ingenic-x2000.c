#include <linux/delay.h>
#include <linux/syscore_ops.h>

#include "phy-ingenic.h"

#define CPM_USBPCR     0x3C
#define CPM_USBRDT     0x40
#define CPM_USBVBFIL   0x44
#define CPM_USBPCR1    0x48

#define CPM_SRBC    0xC4
#define CPM_OPCR    0x24

#define OPCR_SPENDN0_BIT            7
#define OPCR_GATE_USBPHY_CLK_BIT    23
#define SRBC_USB_SR                 12

#define USBRDT_RESUME_IRQ_ENABLE            31
#define USBRDT_RESUME_CLEAR_IRQ             30
#define USBRDT_RESUME_SPEED             28
#define USBRDT_RESUME_STATUS                27

static struct usb_phy_data *x2000_usb_phy;
static unsigned int x2000_set_wakeup;

int x2000_phy_suspend(void)
{
	unsigned int value;

	if (!x2000_usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(x2000_usb_phy, 0x108);
		value |= 1 << 3;
		usb_phy_writel(x2000_usb_phy, value, 0x108);
	}

	/* disable full/low speed driver at the receiver */
	value = usb_phy_readl(x2000_usb_phy, 0x100);
	value &= ~(1 << 6);
	usb_phy_writel(x2000_usb_phy, value, 0x100);

	usb_cpm_clear_bit(x2000_usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (x2000_usb_phy->usb_wakeup && x2000_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(x2000_usb_phy, CPM_USBRDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((x2000_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(x2000_usb_phy, value, CPM_USBRDT);

		usb_cpm_clear_bit(x2000_usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
		usb_cpm_set_bit(x2000_usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	return 0;
}

void x2000_phy_resume(void)
{
	unsigned int value;

	if (x2000_usb_phy->usb_wakeup && x2000_set_wakeup) {
		usb_cpm_clear_bit(x2000_usb_phy, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	usb_cpm_set_bit(x2000_usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	/* enable full/low speed driver at the receiver */
	value = usb_phy_readl(x2000_usb_phy, 0x100);
	value |= 1 << 6;
	usb_phy_writel(x2000_usb_phy, value, 0x100);

	if (!x2000_usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power on. */
		value = usb_phy_readl(x2000_usb_phy, 0x108);
		value &= ~(1 << 3);
		usb_phy_writel(x2000_usb_phy, value, 0x108);
	}
}

static struct syscore_ops x2000_phy_syscore_ops = {
	.suspend = x2000_phy_suspend,
	.resume = x2000_phy_resume,
};

static int x2000_priv_data_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	x2000_usb_phy = usb_phy;
	register_syscore_ops(&x2000_phy_syscore_ops);
	return 0;
}

static int x2000_phy_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0x00200000, CPM_USBPCR1);
	usb_cpm_writel(usb_phy, 0x80400000, CPM_USBPCR);
	udelay(800);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x70000000, CPM_USBPCR1);
	udelay(800);

	/* TX HS driver strength configure */
	value = usb_phy_readl(usb_phy, 0x40);
	value |= 0x7 << 3;
	usb_phy_writel(usb_phy, value, 0x40);

	if (usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(usb_phy, 0x108);
		value |= 0x1 << 3;
		usb_phy_writel(usb_phy, value, 0x108);
	}
	return 0;
}

static int x2000_phy_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	x2000_set_wakeup = enabled;

	return 0;
}

static int x2000_phy_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

struct usb_phy_priv usb_phy_x2000_priv = {
	.priv_data_init = x2000_priv_data_init,

	.phy_init = x2000_phy_init,
	.phy_set_wakeup = x2000_phy_set_wakeup,
	.phy_get_wakeup = x2000_phy_get_wakeup,
};
