/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _PHY_INGENIC_H_
#define _PHY_INGENIC_H_

#include <asm/io.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

/* If gpio wake-up mode, OTG cannot detect wake-up interrupt, enable this macro definition */
//#define TEST_USB_WAKEUP_FLAG

struct usb_phy_priv;

struct usb_phy_data {
	struct usb_phy        phy;
	struct device          *dev;
	void __iomem *cpm_base;
	void __iomem *phy_base;
	void __iomem *controller_base;
	char hsotg_sw_switch[8];
	unsigned char sw_switch_mode;
	struct clk *gate_clk;
	struct gpio_desc    *vbus_gpiod;
	struct gpio_desc    *id_gpiod;
	struct gpio_desc    *drvvbus_gpiod;
	struct delayed_work    vbus_work;
	struct delayed_work    id_work;
	int            vbus;
	int            vbus_irq;
	int            id;
	int            id_irq;
	bool    usb_wakeup;
	bool    vbus_wakeup;
	bool    id_wakeup;
	/* parsing from the phy used by the device tree */
	bool    external_vbus_detect;
	bool     disable_sw_switch_id;
	spinlock_t sw_switch_id_lock;

#ifdef TEST_USB_WAKEUP_FLAG
	unsigned int wakeup_flag;
#endif
	struct gpio_desc    *switch_gpiod;
	struct gpio_desc    *wakeup_gpiod;
	int            wakeup_irq;

	struct usb_phy_priv *phy_priv;
	void *priv_data;
};

struct usb_phy_priv {
	int (*priv_data_init)(struct usb_phy_data *usb_phy);
	void (*priv_data_exit)(struct usb_phy_data *usb_phy);

	int (*phy_init)(struct usb_phy_data *usb_phy);
	void (*phy_shutdown)(struct usb_phy_data *usb_phy);

	int (*phy_set_suspend)(struct usb_phy_data *usb_phy, int suspend);

	int (*phy_set_wakeup)(struct usb_phy_data *usb_phy, int enabled);
	int (*phy_get_wakeup)(struct usb_phy_data *usb_phy);

	enum usb_device_power(*phy_get_power)(struct usb_phy_data *usb_phy);
};

static inline unsigned int usb_cpm_readl(struct usb_phy_data *usb_phy, unsigned int reg_offset)
{
	return readl(usb_phy->cpm_base + reg_offset);
}

static inline void usb_cpm_writel(struct usb_phy_data *usb_phy, unsigned int reg_data, unsigned int reg_offset)
{
	writel(reg_data, (usb_phy->cpm_base + reg_offset));
}

#define usb_cpm_clear_bit(_data,_val,_off) \
	do{usb_cpm_writel(_data, (usb_cpm_readl(_data, _off) & ~(1 << (_val))),(_off));}while(0)
#define usb_cpm_set_bit(_data,_val,_off) \
	do{usb_cpm_writel(_data, (usb_cpm_readl(_data, _off) | (1 << (_val))),(_off));}while(0)
#define usb_cpm_test_bit(_data,_val,_off) \
	(usb_cpm_readl(_data, _off) & (0x1 << (_val)))

static inline unsigned int usb_phy_readl(struct usb_phy_data *usb_phy, unsigned int reg_offset)
{
	unsigned int data = 0;

	if (!IS_ERR_OR_NULL(usb_phy->phy_base)) {
		data = readl(usb_phy->phy_base + reg_offset);
	}

	return data;
}

static inline void usb_phy_writel(struct usb_phy_data *usb_phy, unsigned int reg_data, unsigned int reg_offset)
{
	if (!IS_ERR_OR_NULL(usb_phy->phy_base)) {
		writel(reg_data, (usb_phy->phy_base + reg_offset));
	}
}

#define usb_phy_clear_bit(_data,_val,_off) \
	do{usb_phy_writel(_data, (usb_phy_readl(_data, _off) & ~(1 << (_val))),(_off));}while(0)
#define usb_phy_set_bit(_data,_val,_off) \
	do{usb_phy_writel(_data, (usb_phy_readl(_data, _off) | (1 << (_val))),(_off));}while(0)
#define usb_phy_test_bit(_data,_val,_off) \
	(usb_phy_readl(_data, _off) & (0x1 << (_val)))

//controller
static inline unsigned int usb_controller_readl(struct usb_phy_data *usb, unsigned int reg_offset)
{
	unsigned int data = 0;

	if (!IS_ERR_OR_NULL(usb->controller_base)) {
		data = readl(usb->controller_base + reg_offset);
	}

	return data;
}

static inline void usb_controller_writel(struct usb_phy_data *usb, unsigned int reg_data, unsigned int reg_offset)
{
	if (!IS_ERR_OR_NULL(usb->controller_base)) {
		writel(reg_data, (usb->controller_base + reg_offset));
	}
}

#define usb_controller_clear_bit(_data,_val,_off) \
	do{usb_controller_writel(_data, (usb_controller_readl(_data, _off) & ~(1 << (_val))),(_off));}while(0)
#define usb_controller_set_bit(_data,_val,_off) \
	do{usb_controller_writel(_data, (usb_controller_readl(_data, _off) | (1 << (_val))),(_off));}while(0)
#define usb_controller_test_bit(_data,_val,_off) \
	(usb_controller_readl(_data, _off) & (0x1 << (_val)))

extern struct usb_phy_priv usb_phy_x1000_priv;
extern struct usb_phy_priv usb_phy_x1600_priv;
extern struct usb_phy_priv usb_phy_x2000_priv;
extern struct usb_phy_priv usb_phy_x2500_priv;
extern struct usb_phy_priv usb0_phy_x2600_priv;
extern struct usb_phy_priv usb1_phy_x2600_priv;
extern struct usb_phy_priv usb0_phy_ad100_priv;
extern struct usb_phy_priv usb1_phy_ad100_priv; //sample
#endif /* _PHY_INGENIC_H_ */
