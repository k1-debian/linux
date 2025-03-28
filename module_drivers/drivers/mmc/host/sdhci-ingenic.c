#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include <linux/mmc/host.h>
#include <soc/cpm.h>
#include <linux/mmc/core.h>
#include <linux/mmc/slot-gpio.h>
#include "sdhci.h"
#include "sdhci-ingenic.h"

#define CLK_CTRL
/* Software redefinition caps */
#define CAPABILITIES1_SW    0x276dc898
#define CAPABILITIES2_SW    0

static LIST_HEAD(manual_list);

#ifdef CONFIG_SOC_X2500
	#define CPM_MSC0_CLK_R      (0xB0000068)
	#define CPM_MSC1_CLK_R      (0xB000006c)
#else
	#define CPM_MSC0_CLK_R      (0xB0000068)
	#define CPM_MSC1_CLK_R      (0xB00000a4)
	#define CPM_MSC2_CLK_R      (0xB00000a8)
#endif

#ifdef CONFIG_FPGA_TEST
#define MSC_CLK_H_FREQ      (0x1 << 20)

static void sdhci_ingenic_fpga_clk(unsigned int clock)
{
#define CPM_MSC_CLK_R CPM_MSC0_CLK_R
	//#define CPM_MSC_CLK_R CPM_MSC1_CLK_R
	unsigned int val;

	if (500000 <= clock) {
		val = readl((const volatile void *)CPM_MSC_CLK_R);
		val |= MSC_CLK_H_FREQ;
		writel(val, (void *)CPM_MSC_CLK_R);
	} else {
		val = readl((const volatile void *)CPM_MSC_CLK_R);
		val &= ~MSC_CLK_H_FREQ;
		writel(val, (void *)CPM_MSC_CLK_R);
	}
	printk("\tclk=%d, CPM_MSC0_CLK_R: %08x\n\n", clock, readl((const volatile void *)CPM_MSC0_CLK_R));
}
#endif

static unsigned int sdhci_ingenic_get_cpm_msc(struct sdhci_host *host)
{
	char msc_ioaddr[16];
	unsigned int cpm_msc;
	sprintf(msc_ioaddr, "0x%x", (unsigned int)host->ioaddr);
#ifdef CONFIG_SOC_X2500
	if (!strcmp(msc_ioaddr, "0xb3450000")) {
		cpm_msc = CPM_MSC0_CLK_R;
	}
	if (!strcmp(msc_ioaddr, "0xb3070000")) {
		cpm_msc = CPM_MSC1_CLK_R;
	}
#else
	if (!strcmp(msc_ioaddr, "0xb3450000")) {
		cpm_msc = CPM_MSC0_CLK_R;
	}
	if (!strcmp(msc_ioaddr, "0xb3460000")) {
		cpm_msc = CPM_MSC1_CLK_R;
	}
	if (!strcmp(msc_ioaddr, "0xb3490000")) {
		cpm_msc = CPM_MSC2_CLK_R;
	}
#endif
	return cpm_msc;

}

/**
 * sdhci_ingenic_msc_tuning  Enable msc controller tuning
 *
 * Tuning rx phase
 * */
static void sdhci_ingenic_en_msc_tuning(struct sdhci_host *host, unsigned int cpm_msc)
{
	if (host->mmc->ios.timing & MMC_TIMING_UHS_SDR50 ||
	    host->mmc->ios.timing & MMC_TIMING_UHS_SDR104 ||
	    host->mmc->ios.timing & MMC_TIMING_MMC_HS400) {
		*(volatile unsigned int *)cpm_msc &= ~(0x1 << 20);
	}
}

static void sdhci_ingenic_sel_rx_phase(unsigned int cpm_msc)
{
	*(volatile unsigned int *)cpm_msc |= (0x1 << 20); // default

	*(volatile unsigned int *)cpm_msc &= ~(0x7 << 17);
	*(volatile unsigned int *)cpm_msc |= (0x7 << 17);
}

static void sdhci_ingenic_sel_tx_phase(unsigned int cpm_msc)
{
	*(volatile unsigned int *)cpm_msc &= ~(0x3 << 15);
	/*  *(volatile unsigned int*)cpm_msc |= (0x2 << 15); // 180  100M OK*/
	*(volatile unsigned int *)cpm_msc |= (0x3 << 15);
}

/**
 * sdhci_ingenic_set_clock - callback on clock change
 * @host: The SDHCI host being changed
 * @clock: The clock rate being requested.
 *
 * When the card's clock is going to be changed, look at the new frequency
 * and find the best clock source to go with it.
*/
static void sdhci_ingenic_set_clock(struct sdhci_host *host, unsigned int clock)
{
#ifndef CONFIG_FPGA_TEST
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	unsigned int cpm_msc = sdhci_ingenic_get_cpm_msc(host);
	char clkname[16];

	if (clock == 0) {
		return ;
	}

	sdhci_set_clock(host, clock);
	sprintf(clkname, "mux_msc%d", sdhci_ing->pdev->id);
	sdhci_ing->clk_mux = clk_get(NULL, clkname);

	if (clock > 400000) {
		clk_set_parent(sdhci_ing->clk_mux, sdhci_ing->clk_mpll);
	} else {
		clk_set_parent(sdhci_ing->clk_mux, sdhci_ing->clk_ext);
		*(volatile unsigned int *)0xB0000068 |= 1 << 21;
	}

	clk_set_rate(sdhci_ing->clk_cgu, clock);

	//  printk("%s, set clk: %d, get_clk_rate=%ld\n", __func__, clock, clk_get_rate(sdhci_ing->clk_cgu));

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    host->mmc->ios.timing == MMC_TIMING_UHS_SDR104) {

		/* RX phase selecte */
		if (sdhci_ing->pdata->enable_cpm_rx_tuning == 1) {
			sdhci_ingenic_sel_rx_phase(cpm_msc);
		} else {
			sdhci_ingenic_en_msc_tuning(host, cpm_msc);
		}
		/* TX phase selecte */
		if (sdhci_ing->pdata->enable_cpm_tx_tuning == 1) {
			sdhci_ingenic_sel_tx_phase(cpm_msc);
		}
	}
#else //CONFIG_FPGA_TEST
	sdhci_ingenic_fpga_clk(clock);
#endif
}

/* I/O Driver Strength Types */
#define INGENIC_TYPE_0  0x0     //30
#define INGENIC_TYPE_1  0x1     //50
#define INGENIC_TYPE_2  0x2     //66
#define INGENIC_TYPE_3  0x3     //100

#if (!defined(CONFIG_SOC_X2500))
void sdhci_ingenic_voltage_switch(struct sdhci_host *host)
{
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	struct sdhci_ingenic_pdata *pdata = sdhci_ing->pdata;
	unsigned int val;

	if (IS_ERR(pdata->sdr_v18)) {
		return;
	}

	if (host->flags & SDHCI_SIGNALING_180) {
		val = cpm_inl(CPM_EXCLK_DS) | (1 << 31);
		cpm_outl(val, CPM_EXCLK_DS);
		/*Set up hardware circuit 1.8V*/
		gpiod_set_value(pdata->sdr_v18, 1);
	} else {
		val = cpm_inl(CPM_EXCLK_DS) & ~(1 << 31);
		cpm_outl(val, CPM_EXCLK_DS);
		/*Set up hardware circuit 3V*/
		gpiod_set_value(pdata->sdr_v18, 0);
	}

}
#endif

void sdhci_ingenic_power_set(struct sdhci_host *host, int onoff)
{
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	struct sdhci_ingenic_pdata *pdata = sdhci_ing->pdata;

	if (!IS_ERR(pdata->gpio->pwr)) {
		if (onoff) {
			gpiod_set_value(pdata->gpio->pwr, 1);
		} else {
			gpiod_set_value(pdata->gpio->pwr, 0);
		}
	} else {
		return ;
	}
}

void ingenic_sdhci_hwreset(struct sdhci_host *host)
{
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	struct sdhci_ingenic_pdata *pdata = sdhci_ing->pdata;

	if (!IS_ERR(pdata->gpio->rst)) {
		gpiod_set_value(pdata->gpio->rst, 1);
		udelay(10);
		gpiod_set_value(pdata->gpio->rst, 0);
		usleep_range(300, 500);
	} else {
		return ;
	}

}

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void ingenic_sdhci_adma_write_desc(struct sdhci_host *host, void **desc,
        dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static struct sdhci_ops sdhci_ingenic_ops = {
	.set_clock          = sdhci_ingenic_set_clock,
	.set_bus_width          = sdhci_set_bus_width,
	.reset              = sdhci_reset,
	.set_uhs_signaling      = sdhci_set_uhs_signaling,
	//  .power_set          = sdhci_ingenic_power_set,
	.hw_reset           = ingenic_sdhci_hwreset,
	.adma_write_desc        = ingenic_sdhci_adma_write_desc,
#if (!defined(CONFIG_SOC_X2500) && !defined(CONFIG_SOC_X2600) && !defined(CONFIG_SOC_AD100))
	.voltage_switch         = sdhci_ingenic_voltage_switch,
#endif
};

#ifdef CONFIG_OF
static inline void ingenic_mmc_clk_onoff(struct sdhci_ingenic *ingenic_ing, unsigned int on)
{
	if (on) {
		clk_prepare_enable(ingenic_ing->clk_cgu);
		clk_prepare_enable(ingenic_ing->clk_gate);
	} else {
		clk_disable_unprepare(ingenic_ing->clk_cgu);
		clk_disable_unprepare(ingenic_ing->clk_gate);
	}
}

/**
 *  ingenic_mmc_manual_detect - insert or remove card manually
 *  @index: host->index, namely the index of the controller.
 *  @on: 1 means insert card, 0 means remove card.
 *
 *  This functions will be called by manually card-detect driver such as
 *  wifi. To enable this mode you can set value pdata.removal = MANUAL.
 */
int ingenic_mmc_manual_detect(int index, int on)
{
	struct sdhci_ingenic *sdhci_ing;
	struct sdhci_host *host;
	struct list_head *pos;

	list_for_each(pos, &manual_list) {
		sdhci_ing = list_entry(pos, struct sdhci_ingenic, list);
		if (sdhci_ing->pdev->id == index) {
			break;
		} else {
			sdhci_ing = NULL;
		}
	}

	if (!sdhci_ing) {
		printk("no manual card detect\n");
		return -1;
	}

	host = sdhci_ing->host;

	if (on) {
		dev_err(&sdhci_ing->pdev->dev, "card insert manually\n");
		set_bit(INGENIC_MMC_CARD_PRESENT, &sdhci_ing->flags);
#ifdef CLK_CTRL
		ingenic_mmc_clk_onoff(sdhci_ing, 1);
#endif
		host->flags &= ~SDHCI_DEVICE_DEAD;
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
		mmc_detect_change(sdhci_ing->host->mmc, 0);
	} else {
		dev_err(&sdhci_ing->pdev->dev, "card remove manually\n");
		clear_bit(INGENIC_MMC_CARD_PRESENT, &sdhci_ing->flags);

		host->flags |= SDHCI_DEVICE_DEAD;
		host->quirks &= ~SDHCI_QUIRK_BROKEN_CARD_DETECTION;
		mmc_detect_change(sdhci_ing->host->mmc, 0);
#ifdef CLK_CTRL
		ingenic_mmc_clk_onoff(sdhci_ing, 0);
#endif
	}

	return 0;
}
EXPORT_SYMBOL(ingenic_mmc_manual_detect);

/**
 *  ingenic_mmc_clk_ctrl - enable or disable msc clock gate
 *  @index: host->index, namely the index of the controller.
 *  @on: 1-enable msc clock gate, 0-disable msc clock gate.
 */
int ingenic_mmc_clk_ctrl(int index, int on)
{
	struct sdhci_ingenic *sdhci_ing;
	struct list_head *pos;

#ifdef CLK_CTRL
	list_for_each(pos, &manual_list) {
		sdhci_ing = list_entry(pos, struct sdhci_ingenic, list);
		if (sdhci_ing->pdev->id == index) {
			break;
		} else {
			sdhci_ing = NULL;
		}
	}

	if (!sdhci_ing) {
		printk("no manual card detect\n");
		return -1;
	}
	ingenic_mmc_clk_onoff(sdhci_ing, on);
#endif
	return 0;
}
EXPORT_SYMBOL(ingenic_mmc_clk_ctrl);

static int sdhci_ingenic_parse_dt(struct device *dev,
                                  struct sdhci_host *host,
                                  struct sdhci_ingenic_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	struct card_gpio *card_gpio;
	unsigned int val;

	card_gpio = devm_kzalloc(dev, sizeof(struct card_gpio), GFP_KERNEL);
	if (!card_gpio) {
		return 0;
	}
	pdata->gpio = card_gpio;

	pdata->sdr_v18 = devm_gpiod_get_optional(dev, "ingenic,sdr", GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->sdr_v18)) {
		dev_err(dev, "get ingenic,sdr failed with error %ld\n",
		        PTR_ERR(pdata->sdr_v18));
	}

	pdata->gpio->pwr = devm_gpiod_get_optional(dev, "ingenic,pwr", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio->pwr)) {
		dev_err(dev, "get ingenic,pwr failed with error %ld\n",
		        PTR_ERR(pdata->gpio->pwr));
	}

	pdata->gpio->rst = devm_gpiod_get_optional(dev, "ingenic,rst", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio->rst)) {
		dev_err(dev, "get ingenic,rst failed with error %ld\n",
		        PTR_ERR(pdata->gpio->rst));
	}

	/* assuming internal card detect that will be configured by pinctrl */
	pdata->cd_type = SDHCI_INGENIC_CD_INTERNAL;

	if (of_property_read_bool(np, "pio-mode")) {
		pdata->pio_mode = 1;
	}
	if (of_property_read_bool(np, "enable_autocmd12")) {
		pdata->enable_autocmd12 = 1;
	}
	if (of_property_read_bool(np, "enable_cpm_rx_tuning")) {
		pdata->enable_cpm_rx_tuning = 1;
	}
	if (of_property_read_bool(np, "enable_cpm_tx_tuning")) {
		pdata->enable_cpm_tx_tuning = 1;
	}

	/* get the card detection method */
	if (of_get_property(np, "broken-cd", NULL)) {
		pdata->cd_type = SDHCI_INGENIC_CD_NONE;
	}

	if (of_get_property(np, "non-removable", NULL)) {
		pdata->cd_type = SDHCI_INGENIC_CD_PERMANENT;
	}

	if (of_get_property(np, "cd-inverted", NULL)) {
		pdata->cd_type = SDHCI_INGENIC_CD_GPIO;
	}

	if (!(of_property_read_u32(np, "ingenic,sdio_clk", &val))) {
		pdata->sdio_clk = val;
	}

	/* if(of_property_read_bool(np, "ingenic,removal-dontcare")) { */
	/*  pdata->removal = DONTCARE; */
	/* } else if(of_property_read_bool(np, "ingenic,removal-nonremovable")) { */
	/*  pdata->removal = NONREMOVABLE; */
	/* } else if(of_property_read_bool(np, "ingenic,removal-removable")) { */
	/*  pdata->removal = REMOVABLE; */
	/* } else if(of_property_read_bool(np, "ingenic,removal-manual")) { */
	/*  pdata->removal = MANUAL; */
	/* }; */

	/* mmc_of_parse_voltage(np, &pdata->ocr_avail); */

	return 0;
}
#else
static int sdhci_ingenic_parse_dt(struct device *dev,
                                  struct sdhci_host *host,
                                  struct sdhci_ingenic_pdata *pdata)
{
	return -EINVAL;
}
#endif

static int sdhci_ingenic_probe(struct platform_device *pdev)
{
	struct sdhci_ingenic_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_ingenic *sdhci_ing;
	char clkname[16];
	int ret, irq;

	if (!pdev->dev.platform_data && !pdev->dev.of_node) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_ingenic));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}
	sdhci_ing = sdhci_priv(host);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		return ret;
	}

	if (pdev->dev.of_node) {
		ret = sdhci_ingenic_parse_dt(dev, host, pdata);
		if (ret) {
			return ret;
		}
	} else {
		memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));
	}

	pdev->id = of_alias_get_id(pdev->dev.of_node, "mmc");

	sprintf(clkname, "div_msc%d", pdev->id);
	sdhci_ing->clk_cgu = devm_clk_get(&pdev->dev, clkname);
	if (!sdhci_ing->clk_cgu) {
		dev_err(&pdev->dev, "Failed to Get MSC clk!\n");
		return PTR_ERR(sdhci_ing->clk_cgu);
	}
	sprintf(clkname, "gate_msc%d", pdev->id);
	sdhci_ing->clk_gate = devm_clk_get(&pdev->dev, clkname);
	if (!sdhci_ing->clk_gate) {
		dev_err(&pdev->dev, "Failed to Get PWC MSC clk!\n");
		return PTR_ERR(sdhci_ing->clk_gate);
	}
	sdhci_ing->clk_ext = clk_get(NULL, "ext");
	sdhci_ing->clk_mpll = clk_get(NULL, "mpll");

	ingenic_mmc_clk_onoff(sdhci_ing, 1);

	sdhci_ing->host = host;
	sdhci_ing->dev  = &pdev->dev;
	sdhci_ing->pdev = pdev;
	sdhci_ing->pdata = pdata;

	host->ioaddr = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(host->ioaddr)) {
		return PTR_ERR(host->ioaddr);
	}

	platform_set_drvdata(pdev, host);

	/* sdio for WIFI init*/
	if (pdata->sdio_clk) {
		ingenic_sdio_wlan_init(&pdev->dev, pdev->id);
		list_add(&(sdhci_ing->list), &manual_list);
	}

	host->hw_name = "ingenic-sdhci";
	host->ops = &sdhci_ingenic_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Software redefinition caps */
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
	host->caps  = CAPABILITIES1_SW;
	host->caps1 = CAPABILITIES2_SW;

	/* not check wp */
	host->quirks |= SDHCI_QUIRK_INVERTED_WRITE_PROTECT;

	/* Setup quirks for the controller */
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks |= SDHCI_QUIRK_NO_HISPD_BIT;

	/* Data Timeout Counter Value */
	//host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
	host->timeout_clk = 24000; //TMCLK = 24MHz

	/* This host supports the Auto CMD12 */
	if (pdata->enable_autocmd12) {
		host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;
	}

	/* PIO transfer mode */
	if (pdata->pio_mode) {
		host->quirks |= SDHCI_QUIRK_BROKEN_DMA;
		host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
	}
	/* TODO:SoCs need BROKEN_ADMA_ZEROLEN_DESC */
	/*  host->quirks |= SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC;*/

	if (pdata->cd_type == SDHCI_INGENIC_CD_NONE ||
	    pdata->cd_type == SDHCI_INGENIC_CD_PERMANENT) {
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	}

	if (pdata->cd_type == SDHCI_INGENIC_CD_PERMANENT) {
		host->mmc->caps = MMC_CAP_NONREMOVABLE;
	}

	if (pdata->pm_caps) {
		host->mmc->pm_caps |= pdata->pm_caps;
	}

	host->quirks |= (SDHCI_QUIRK_32BIT_DMA_ADDR |
	                 SDHCI_QUIRK_32BIT_DMA_SIZE);

	/* It supports additional host capabilities if needed */
	if (pdata->host_caps) {
		host->mmc->caps |= pdata->host_caps;
	}

	if (pdata->host_caps2) {
		host->mmc->caps2 |= pdata->host_caps2;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);
#endif

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(dev, "mmc_of_parse() failed\n");
		pm_runtime_forbid(&pdev->dev);
		pm_runtime_get_noresume(&pdev->dev);
		return ret;
	}

	sdhci_enable_v4_mode(host);
	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		pm_runtime_forbid(&pdev->dev);
		pm_runtime_get_noresume(&pdev->dev);
		return ret;
	}

	/*   enable card  inserted and unplug wake-up system */
	if (host->mmc->slot.cd_irq > 0) {
		enable_irq_wake(host->mmc->slot.cd_irq);
	}
#ifdef CONFIG_PM_RUNTIME
	if (pdata->cd_type != SDHCI_INGENIC_CD_INTERNAL) {
		clk_disable_unprepare(sdhci_ing->clk_cgu);
		/* clk_disable_unprepare(sdhci_ing->clk_gate); */
	}
#endif

	return 0;
}

static int sdhci_ingenic_remove(struct platform_device *pdev)
{
	struct sdhci_host *host =  platform_get_drvdata(pdev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);

	if (sdhci_ing->ext_cd_irq) {
		free_irq(sdhci_ing->ext_cd_irq, sdhci_ing);
	}

#ifdef CONFIG_PM_RUNTIME
	if (pdata->cd_type != SDHCI_INGENIC_CD_INTERNAL) {
		/* clk_prepare_enable(sdhci_ing->clk_gate); */
		clk_prepare_enable(sdhci_ing->clk_cgu);
	}
#endif
	sdhci_remove_host(host, 1);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(sdhci_ing->clk_cgu);
	/* clk_disable_unprepare(sdhci_ing->clk_gate); */

	sdhci_free_host(host);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int sdhci_ingenic_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host);
	int ret = 0;

	ret = sdhci_suspend_host(host);

	clk_disable_unprepare(sdhci_ing->clk_gate);
	return ret;
}

static int sdhci_ingenic_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_ingenic *sdhci_ing = sdhci_priv(host) ;
	clk_prepare_enable(sdhci_ing->clk_gate);
	return sdhci_resume_host(host);
}
#endif

#ifdef CONFIG_PM
static int sdhci_ingenic_runtime_suspend(struct device *dev)
{
	return 0;
}

static int sdhci_ingenic_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops sdhci_ingenic_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_ingenic_suspend, sdhci_ingenic_resume)
	SET_RUNTIME_PM_OPS(sdhci_ingenic_runtime_suspend, sdhci_ingenic_runtime_resume,
	                   NULL)
};

#define SDHCI_INGENIC_PMOPS (&sdhci_ingenic_pmops)

#else
#define SDHCI_INGENIC_PMOPS NULL
#endif

static struct platform_device_id sdhci_ingenic_driver_ids[] = {
	{
		.name       = "ingenic,sdhci",
		.driver_data    = (kernel_ulong_t)NULL,
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, sdhci_ingenic_driver_ids);

#ifdef CONFIG_OF
static const struct of_device_id sdhci_ingenic_dt_match[] = {
	{.compatible = "ingenic,sdhci",},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_ingenic_dt_match);
#endif

static struct platform_driver sdhci_ingenic_driver = {
	.probe      = sdhci_ingenic_probe,
	.remove     = sdhci_ingenic_remove,
	.id_table   = sdhci_ingenic_driver_ids,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "ingenic,sdhci",
		.pm = SDHCI_INGENIC_PMOPS,
		.of_match_table = of_match_ptr(sdhci_ingenic_dt_match),
	},
};

module_platform_driver(sdhci_ingenic_driver);

MODULE_DESCRIPTION("Ingenic SDHCI (MSC) driver");
MODULE_AUTHOR("Large Dipper <wangquan.shao@ingenic.cn>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20160808");
