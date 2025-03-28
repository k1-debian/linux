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
#define SRBC_USB_SR                 14

#define USBRDT_RESUME_IRQ_ENABLE            31
#define USBRDT_RESUME_CLEAR_IRQ             30
#define USBRDT_RESUME_SPEED             28
#define USBRDT_RESUME_STATUS                27

static struct usb_phy_data *ad100_usb_phy0;
static unsigned int ad100_port0_set_wakeup;

int ad100_phy_port0_suspend(void)
{
	unsigned int value;

	if (!ad100_usb_phy0->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(ad100_usb_phy0, 0x3c);
		value |= 0x1 << 7;
		usb_phy_writel(ad100_usb_phy0, value, 0x3c);
	}

	/*In suspend mode, turnning off differential receiver to save power.*/
	value = usb_phy_readl(ad100_usb_phy0, 0x30);
	value &= ~(0x1 << 2);
	usb_phy_writel(ad100_usb_phy0, value, 0x30);

	usb_cpm_clear_bit(ad100_usb_phy0, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (ad100_usb_phy0->usb_wakeup && ad100_port0_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(ad100_usb_phy0, CPM_USBRDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((ad100_port0_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(ad100_usb_phy0, value, CPM_USBRDT);

		usb_cpm_clear_bit(ad100_usb_phy0, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
		usb_cpm_set_bit(ad100_usb_phy0, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	return 0;
}

void ad100_phy_port0_resume(void)
{
	unsigned int value;

	if (ad100_usb_phy0->usb_wakeup && ad100_port0_set_wakeup) {
		usb_cpm_clear_bit(ad100_usb_phy0, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}
	usb_cpm_set_bit(ad100_usb_phy0, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (!ad100_usb_phy0->external_vbus_detect) {
		/* VBUS voltage level detection power on. */
		value = usb_phy_readl(ad100_usb_phy0, 0x3c);
		value &= ~(1 << 7);
		usb_phy_writel(ad100_usb_phy0, value, 0x3c);
	}

	/*turnning on differential receiver.*/
	value = usb_phy_readl(ad100_usb_phy0, 0x30);
	value |= (0x1 << 2);
	usb_phy_writel(ad100_usb_phy0, value, 0x30);
}

static struct syscore_ops ad100_phy_port0_syscore_ops = {
	.suspend = ad100_phy_port0_suspend,
	.resume = ad100_phy_port0_resume,
};

static int ad100_priv_data_port0_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	ad100_usb_phy0 = usb_phy;
	register_syscore_ops(&ad100_phy_port0_syscore_ops);
	return 0;
}

static int ad100_phy_port0_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	/* vbus signal always valid, id pin always pullup */
	usb_cpm_writel(usb_phy, 0x00000000, CPM_USBPCR1);
	usb_cpm_writel(usb_phy, 0x80100000, CPM_USBPCR);
	usleep_range(800, 800);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USBPCR);
	usb_cpm_writel(usb_phy, 0x30000000, CPM_USBPCR1);
	usleep_range(800, 800);

	/* Chirp K or SE0 resume enable */
	value = usb_cpm_readl(usb_phy, CPM_USBRDT);
	value |= 0x1 << 26;
	usb_cpm_writel(usb_phy, value, CPM_USBRDT);

	/* In fact, when the high-speed eye height is set to the highest,
	   the register value should be 3'b110. The default value of 3'b111 in PM
	   is the lowest. */
	value = usb_phy_readl(usb_phy, 0x30);
	value &= ~(0x1 << 4);
	usb_phy_writel(usb_phy, value, 0x30);

	if (usb_phy->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(usb_phy, 0x3c);
		value |= 0x1 << 7;
		usb_phy_writel(usb_phy, value, 0x3c);
	}

	return 0;
}

static int ad100_phy_port0_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	ad100_port0_set_wakeup = enabled;

	return 0;
}

static int ad100_phy_port0_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

struct usb_phy_priv usb0_phy_ad100_priv = {
	.priv_data_init = ad100_priv_data_port0_init,

	.phy_init = ad100_phy_port0_init,
	.phy_set_wakeup = ad100_phy_port0_set_wakeup,
	.phy_get_wakeup = ad100_phy_port0_get_wakeup,
};

// smaple

#define CPM_USB1PCR     0x4C
#define CPM_USB1RDT     0x50
#define CPM_USB1VBFIL   0x58
#define CPM_USB1PCR1    0xe8
#define SRBC_USB1_SR                    15
#define OPCR_SPENDN0_BIT            7
#define OPCR_SPENDN1_BIT            6

static struct usb_phy_data *ad100_usb_phy1;
static unsigned int ad100_port1_set_wakeup;

int ad100_phy_port1_suspend(void)
{
	unsigned int value;

	/*In suspend mode, turnning off differential receiver to save power.*/
	value = usb_phy_readl(ad100_usb_phy1, 0x30);
	value &= ~(0x1 << 2);
	usb_phy_writel(ad100_usb_phy1, value, 0x30);

	usb_cpm_clear_bit(ad100_usb_phy1, OPCR_SPENDN1_BIT, CPM_OPCR);

	if (ad100_usb_phy1->usb_wakeup && ad100_port1_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(ad100_usb_phy1, CPM_USB1RDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((ad100_port1_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(ad100_usb_phy1, value, CPM_USB1RDT);

		usb_cpm_clear_bit(ad100_usb_phy1, USBRDT_RESUME_CLEAR_IRQ, CPM_USB1RDT);
		usb_cpm_set_bit(ad100_usb_phy1, USBRDT_RESUME_IRQ_ENABLE, CPM_USB1RDT);
	}

	return 0;
}

void ad100_phy_port1_resume(void)
{
	unsigned int value;

	if (ad100_usb_phy1->usb_wakeup && ad100_port1_set_wakeup) {
		usb_cpm_clear_bit(ad100_usb_phy1, USBRDT_RESUME_IRQ_ENABLE, CPM_USB1RDT);
	}
	usb_cpm_set_bit(ad100_usb_phy1, OPCR_SPENDN1_BIT, CPM_OPCR);

	/*turnning on differential receiver.*/
	value = usb_phy_readl(ad100_usb_phy1, 0x30);
	value |= (0x1 << 2);
	usb_phy_writel(ad100_usb_phy1, value, 0x30);
}

static struct syscore_ops ad100_phy_port1_syscore_ops = {
	.suspend = ad100_phy_port1_suspend,
	.resume = ad100_phy_port1_resume,
};

static int ad100_priv_data_port1_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB1_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB1_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN1_BIT, CPM_OPCR);

	ad100_usb_phy1 = usb_phy;
	register_syscore_ops(&ad100_phy_port1_syscore_ops);
	return 0;
}

static int ad100_phy_port1_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_writel(usb_phy, 0x00000000, CPM_USB1PCR1);
	usb_cpm_writel(usb_phy, 0x80100000, CPM_USB1PCR);
	usleep_range(800, 800);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USB1PCR);
	usb_cpm_writel(usb_phy, 0x70000000, CPM_USB1PCR1);
	usleep_range(800, 800);

	return 0;
}

static int ad100_phy_port1_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	ad100_port1_set_wakeup = enabled;

	return 0;
}

static int ad100_phy_port1_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USB1RDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USB1RDT);
			return 1;
		}
	}

	return 0;
}

struct usb_phy_priv usb1_phy_ad100_priv = {
	.priv_data_init = ad100_priv_data_port1_init,

	.phy_init = ad100_phy_port1_init,
	.phy_set_wakeup = ad100_phy_port1_set_wakeup,
	.phy_get_wakeup = ad100_phy_port1_get_wakeup,
};
