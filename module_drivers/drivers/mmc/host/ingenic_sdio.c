#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/rtc.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>

#if (defined(CONFIG_INGENIC_MMC_MMC0) || defined(CONFIG_INGENIC_MMC_MMC1))
	#include "ingenic_mmc.h"
#else
	#include "sdhci-ingenic.h"
#endif

struct wifi_data {
	int sdio_index;
	struct gpio_desc *wifi_reset;
	struct gpio_desc *wifi_irq;
	struct pinctrl *pctrl;
	atomic_t rtc32k_ref;
	struct clk *clk;
};

#define MANUALLY_INSERT 2

struct wifi_data wifi_data;
static int rtc32k_init(struct device *dev, struct wifi_data *wdata)
{
	atomic_set(&wdata->rtc32k_ref, 0);
	wdata->pctrl = NULL;
	wdata->clk = NULL;
#ifdef CONFIG_RTC_DRV_PCF8563
	struct rtc_device *rtc_pdev = NULL;

	rtc_pdev = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc_pdev) {
		printk("%s is loaded!!!\n", CONFIG_RTC_HCTOSYS_DEVICE);
	} else {
		printk("%s is not loaded!!!\n", CONFIG_RTC_HCTOSYS_DEVICE);
		return -1;
	}

	{
		struct of_phandle_args phandle;
		phandle.np = of_find_compatible_node(NULL, NULL, "nxp,pcf8563");
		if (phandle.np) {
			wdata->clk = of_clk_get_from_provider(&phandle);
		}
	}
#else
	wdata->pctrl = devm_pinctrl_get(dev);
#endif
	return 0;
}

void rtc32k_enable(void)
{

	if (atomic_inc_return(&wifi_data.rtc32k_ref) == 1) {
		if (wifi_data.clk) {
			printk("@@ rtc.pcf8563 %s @@\n", __func__);
			clk_prepare_enable(wifi_data.clk);
		}
		if (wifi_data.pctrl) {
			struct pinctrl_state *state = NULL;
			struct pinctrl *p = wifi_data.pctrl;
			state = pinctrl_lookup_state(p, "enable");
			if (!IS_ERR_OR_NULL(state)) {
				pinctrl_select_state(p, state);
			}
		}
	}
}
EXPORT_SYMBOL(rtc32k_enable);

void rtc32k_disable(void)
{
	if (atomic_dec_return(&wifi_data.rtc32k_ref) == 0) {
		if (wifi_data.clk) {
			printk("@@ rtc.pcf8563 %s @@\n", __func__);
			clk_disable_unprepare(wifi_data.clk);
		}
		if (wifi_data.pctrl) {
			struct pinctrl_state *state = NULL;
			struct pinctrl *p = wifi_data.pctrl;
			state = pinctrl_lookup_state(p, "disable");
			if (!IS_ERR_OR_NULL(state)) {
				pinctrl_select_state(p, state);
			}
		}
	}
}
EXPORT_SYMBOL(rtc32k_disable);

static const struct of_device_id wlan_ingenic_of_match[] = {
	{.compatible = "rtk,rtl8723ds_wlan"},
	{},
};

int ingenic_sdio_wlan_init(struct device *dev, int index)
{
	struct device_node *np = dev->of_node, *cnp;
	int ret = 0;

	for_each_child_of_node(np, cnp) {
		if (of_device_is_compatible(cnp, "rtk,rtl8723ds_wlan")) {
			printk("----rtk,rtl8723ds_wlan\n");
			wifi_data.wifi_reset = devm_gpiod_get_optional(dev, "ingenic,sdio-reset", GPIOD_OUT_HIGH);
			wifi_data.wifi_irq = devm_gpiod_get_optional(dev, "ingenic,sdio-irq", GPIOD_IN);
		}
	}

	ret = rtc32k_init(dev, &wifi_data);
	if (ret) {
		dev_err(dev, "Failed to init rtc32k clock!\n");
		return ret;
	}
	wifi_data.sdio_index = index;

	return 0;
}
EXPORT_SYMBOL(ingenic_sdio_wlan_init);

int ingenic_bcmdhd_wlan_power_onoff(int flag)
{
	if (flag) {
		printk("wlan power on:%d\n", flag);
		rtc32k_enable();
		ingenic_mmc_clk_ctrl(wifi_data.sdio_index, 1);
		if (flag == MANUALLY_INSERT) {
			ingenic_mmc_manual_detect(wifi_data.sdio_index, 1);
		}
	} else {
		printk("wlan power off:%d\n", flag);
		rtc32k_disable();
	}
	return 0;

}
EXPORT_SYMBOL(ingenic_bcmdhd_wlan_power_onoff);
