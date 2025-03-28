#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include "phy-ingenic.h"

#define CPM_USBPCR     0x3C
#define CPM_USBRDT     0x40
#define CPM_USBVBFIL   0x44
#define CPM_USBPCR1    0x48

#define CPM_USB1PCR     0x4C
#define CPM_USB1RDT     0x50
#define CPM_USB1VBFIL   0x58
#define CPM_USB1PCR1    0xe8

#define CPM_SRBC    0xC4
#define CPM_OPCR    0x24

#define OPCR_SPENDN0_BIT            7
#define OPCR_SPENDN1_BIT            6

#define OPCR_GATE_USBPHY_CLK_BIT    23
#define SRBC_USB_SR                    14
#define SRBC_USB1_SR                    15

#define USBRDT_RESUME_IRQ_ENABLE            31
#define USBRDT_RESUME_CLEAR_IRQ                30
#define USBRDT_RESUME_SPEED                28
#define USBRDT_RESUME_STATUS                27

static struct usb_phy_data *x2600_usb_phy0;
static unsigned int x2600_port0_set_wakeup;

static struct usb_phy_data *x2600_usb_phy1;
static unsigned int x2600_port1_set_wakeup;

int x2600_phy_port0_suspend(void)
{
	unsigned int value;

	if (!x2600_usb_phy0->external_vbus_detect) {
		/* VBUS voltage level detection power down. */
		value = usb_phy_readl(x2600_usb_phy0, 0x3c);
		value |= 0x1 << 7;
		usb_phy_writel(x2600_usb_phy0, value, 0x3c);
	}

	/*In suspend mode, turnning off differential receiver to save power.*/
	value = usb_phy_readl(x2600_usb_phy0, 0x30);
	value &= ~(0x1 << 2);
	usb_phy_writel(x2600_usb_phy0, value, 0x30);

	usb_cpm_clear_bit(x2600_usb_phy0, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (x2600_usb_phy0->usb_wakeup && x2600_port0_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(x2600_usb_phy0, CPM_USBRDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((x2600_port0_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(x2600_usb_phy0, value, CPM_USBRDT);

		usb_cpm_clear_bit(x2600_usb_phy0, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
		usb_cpm_set_bit(x2600_usb_phy0, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}

	return 0;
}

void x2600_phy_port0_resume(void)
{
	unsigned int value;

	if (x2600_usb_phy0->usb_wakeup && x2600_port0_set_wakeup) {
		usb_cpm_clear_bit(x2600_usb_phy0, USBRDT_RESUME_IRQ_ENABLE, CPM_USBRDT);
	}
	usb_cpm_set_bit(x2600_usb_phy0, OPCR_SPENDN0_BIT, CPM_OPCR);

	if (!x2600_usb_phy0->external_vbus_detect) {
		/* VBUS voltage level detection power on. */
		value = usb_phy_readl(x2600_usb_phy0, 0x3c);
		value &= ~(1 << 7);
		usb_phy_writel(x2600_usb_phy0, value, 0x3c);
	}

	/*turnning on differential receiver.*/
	value = usb_phy_readl(x2600_usb_phy0, 0x30);
	value |= (0x1 << 2);
	usb_phy_writel(x2600_usb_phy0, value, 0x30);
}

static struct syscore_ops x2600_phy_port0_syscore_ops = {
	.suspend = x2600_phy_port0_suspend,
	.resume = x2600_phy_port0_resume,
};

static int x2600_priv_data_port0_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN0_BIT, CPM_OPCR);

	x2600_usb_phy0 = usb_phy;
	register_syscore_ops(&x2600_phy_port0_syscore_ops);
	return 0;
}

static int x2600_phy_port0_init(struct usb_phy_data *usb_phy)
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

static int x2600_phy_port0_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	x2600_port0_set_wakeup = enabled;

	return 0;
}

static int x2600_phy_port0_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USBRDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USBRDT);
			return 1;
		}
	}

	return 0;
}

int x2600_phy_port1_suspend(void)
{
	unsigned int value;

	/*In suspend mode, turnning off differential receiver to save power.*/
	value = usb_phy_readl(x2600_usb_phy1, 0x30);
	value &= ~(0x1 << 2);
	usb_phy_writel(x2600_usb_phy1, value, 0x30);

	usb_cpm_clear_bit(x2600_usb_phy1, OPCR_SPENDN1_BIT, CPM_OPCR);

	if (x2600_usb_phy1->usb_wakeup && x2600_port1_set_wakeup) {
		/* set wakeup speed */
		value = usb_cpm_readl(x2600_usb_phy1, CPM_USB1RDT);
		value &= ~(0x3 << USBRDT_RESUME_SPEED);
		value |= ((x2600_port1_set_wakeup - 1) & 0x3) << USBRDT_RESUME_SPEED;
		usb_cpm_writel(x2600_usb_phy1, value, CPM_USB1RDT);

		usb_cpm_clear_bit(x2600_usb_phy1, USBRDT_RESUME_CLEAR_IRQ, CPM_USB1RDT);
		usb_cpm_set_bit(x2600_usb_phy1, USBRDT_RESUME_IRQ_ENABLE, CPM_USB1RDT);
	}

	return 0;
}

void x2600_phy_port1_resume(void)
{
	unsigned int value;

	if (x2600_usb_phy1->usb_wakeup && x2600_port1_set_wakeup) {
		usb_cpm_clear_bit(x2600_usb_phy1, USBRDT_RESUME_IRQ_ENABLE, CPM_USB1RDT);
	}
	usb_cpm_set_bit(x2600_usb_phy1, OPCR_SPENDN1_BIT, CPM_OPCR);

	/*turnning on differential receiver.*/
	value = usb_phy_readl(x2600_usb_phy1, 0x30);
	value |= (0x1 << 2);
	usb_phy_writel(x2600_usb_phy1, value, 0x30);
}

static struct syscore_ops x2600_phy_port1_syscore_ops = {
	.suspend = x2600_phy_port1_suspend,
	.resume = x2600_phy_port1_resume,
};

static int x2600_priv_data_port1_init(struct usb_phy_data *usb_phy)
{
	usb_cpm_set_bit(usb_phy, SRBC_USB1_SR, CPM_SRBC);
	udelay(10);
	usb_cpm_clear_bit(usb_phy, SRBC_USB1_SR, CPM_SRBC);

	usb_cpm_set_bit(usb_phy, OPCR_SPENDN1_BIT, CPM_OPCR);

	x2600_usb_phy1 = usb_phy;
	register_syscore_ops(&x2600_phy_port1_syscore_ops);
	return 0;
}

static int x2600_phy_port1_init(struct usb_phy_data *usb_phy)
{
	unsigned int value;

	usb_cpm_writel(usb_phy, 0x00000000, CPM_USB1PCR1);
	usb_cpm_writel(usb_phy, 0x80100000, CPM_USB1PCR);
	usleep_range(800, 800);
	usb_cpm_writel(usb_phy, 0x80000000, CPM_USB1PCR);
	usb_cpm_writel(usb_phy, 0x30000000, CPM_USB1PCR1);
	usleep_range(800, 800);

	/* Chirp K or SE0 resume enable */
	value = usb_cpm_readl(usb_phy, CPM_USB1RDT);
	value |= 0x1 << 26;
	usb_cpm_writel(usb_phy, value, CPM_USB1RDT);

	/* In fact, when the high-speed eye height is set to the highest,
	   the register value should be 3'b110. The default value of 3'b111 in PM
	   is the lowest. */
	value = usb_phy_readl(usb_phy, 0x30);
	value &= ~(0x1 << 4);
	usb_phy_writel(usb_phy, value, 0x30);

	return 0;
}

static int x2600_phy_port1_set_wakeup(struct usb_phy_data *usb_phy, int enabled)
{
	x2600_port1_set_wakeup = enabled;

	return 0;
}

static int x2600_phy_port1_get_wakeup(struct usb_phy_data *usb_phy)
{
	if (usb_phy->usb_wakeup) {
		if (usb_cpm_test_bit(usb_phy, USBRDT_RESUME_STATUS, CPM_USB1RDT)) {
			usb_cpm_set_bit(usb_phy, USBRDT_RESUME_CLEAR_IRQ, CPM_USB1RDT);
			return 1;
		}
	}

	return 0;
}

/***
 *
* 1, DCTL.SftDiscon = 1( for device soft-disconnect)
*
* 2. Data pin contact detect.
*     - Set BCCR.IdpSrcEn and BCCR.RdmPdwnEn to 1.
*     - Check BCCR.DpAttached. If BCCR.DpAttached = 1, continue next step.
*
* 3. SDP primary detection.
*     - Clear BCCR.IdpSrcEn and BCCR.RdmPdwnEn to 0.
*     - Set BCCR.IdmSinkEn and BCCR.VdpSrcEn to 1.
*     - Check BCCR.CpDetected. If BCCR.CpDetected = 1, port is DCP or CDP, continue next step.
*     - Else BCCR.CpDetected = 0, port is SDP, follow the last step.
*
* 4. DCP/CDP secondary detection.
*     - Clear BCCR.IdmSinkEn and BCCR.VdpSrcEn to 0.
*     - Set BCCR.IdpSinkEn and BCCR.VdmSrcEn to 1.
*     - Check PHY BCCR.DcpDetected. If BCCR.DcpDetected = 1, port is DCP.
*     - Else BCCR.DcpDetected=0, port is CDP.
*
*     5. Clear BCCR[31:26] to 0, quit BC1.2 detection.
*
***/

static enum usb_device_power x2600_phy_get_power(struct usb_phy_data *usb_phy)
{

	enum usb_device_power type = USB_POWER_UNKNOWN;
	unsigned int bccr_value;
	unsigned int reg_offset = 0x038;

	/* 1. Device soft-disconnect */
	// has been soft-disconnect

	/* 2. Data pin contact detect */
	bccr_value = usb_controller_readl(usb_phy, reg_offset);
	bccr_value |= (1U << 28) | (1U << 29);
	usb_controller_writel(usb_phy, bccr_value, reg_offset);
	usleep_range(20000, 20000);

	bccr_value = usb_controller_readl(usb_phy, reg_offset);
	if (bccr_value & (1U << 4)) { /* 3. SDP primary detection */

		bccr_value &= ~(1U << 28) & ~(1U << 29);
		usb_controller_writel(usb_phy, bccr_value, reg_offset);

		bccr_value |= (1U << 30) | (1U << 27);
		usb_controller_writel(usb_phy, bccr_value, reg_offset);

		usleep_range(20000, 20000);

		bccr_value = usb_controller_readl(usb_phy, reg_offset);
		if (bccr_value & (1U << 3)) {  /* 4. DCP/CDP secondary detection */

			bccr_value &= ~((1U << 30) | (1U << 27));
			usb_controller_writel(usb_phy, bccr_value, reg_offset);

			bccr_value |= (1U << 31) | (1U << 26);
			usb_controller_writel(usb_phy, bccr_value, reg_offset);

			usleep_range(20000, 20000);

			bccr_value = usb_controller_readl(usb_phy, reg_offset);
			if (bccr_value & (1U << 2)) {
				type = USB_POWER_DCP;
			} else {
				type = USB_POWER_CDP;
			}
		} else {
			type = USB_POWER_SDP;
		}
	} else {
		printk("No device attached.\n");
	}

	/* 5. Clear BCCR[31:26] to 0 */
	bccr_value = usb_controller_readl(usb_phy, reg_offset);
	usb_controller_writel(usb_phy, bccr_value & ~0xfc000000, reg_offset);

	return type;
}

struct usb_phy_priv usb0_phy_x2600_priv = {
	.priv_data_init = x2600_priv_data_port0_init,

	.phy_init = x2600_phy_port0_init,
	.phy_set_wakeup = x2600_phy_port0_set_wakeup,
	.phy_get_wakeup = x2600_phy_port0_get_wakeup,
	.phy_get_power = x2600_phy_get_power,

};

struct usb_phy_priv usb1_phy_x2600_priv = {
	.priv_data_init = x2600_priv_data_port1_init,

	.phy_init = x2600_phy_port1_init,
	.phy_set_wakeup = x2600_phy_port1_set_wakeup,
	.phy_get_wakeup = x2600_phy_port1_get_wakeup,
};
