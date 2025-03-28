#ifndef __INGENIC_MMC_H__
#define __INGENIC_MMC_H__

#include "ingenic_mmc_reg.h"
#include <linux/interrupt.h>
enum {
	DONTCARE = 0,
	NONREMOVABLE,
	REMOVABLE,
	MANUAL,
};

enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_TRANS_COMPLETE,
	EVENT_DMA_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_STOP_COMPLETE,
	EVENT_ERROR,
};

enum ingenic_mmc_state {
	STATE_IDLE = 0,
	STATE_WAITING_RESP,
	STATE_WAITING_DATA,
	STATE_SENDING_STOP,
	STATE_ERROR,
};

struct sdma_desc {
	volatile u32 nda;
	volatile u32 da;
	volatile u32 len;
	volatile u32 dcmd;
};

struct desc_hd {
	struct sdma_desc *dma_desc;
	dma_addr_t dma_desc_phys_addr;
	struct desc_hd *next;
};

#define LOW_ENABLE          0
#define HIGH_ENABLE         1

struct card_gpio {
	struct gpio_desc    *wp;
	struct gpio_desc    *cd;
	struct gpio_desc    *pwr;
	struct gpio_desc    *rst;
};

/**
 * struct ingenic_mmc_platform_data is a struct which defines board MSC informations
 * @removal: This shows the card slot's type:
 *  REMOVABLE/IRREMOVABLE/MANUAL (Tablet card/Phone card/build-in SDIO).
 * @sdio_clk: SDIO device's clock can't use Low-Power-Mode.
 * @ocr_mask: This one shows the voltage that host provide.
 * @capacity: Shows the host's speed capacity and bus width.
 * @max_freq: The max freqency of mmc host.
 *
 * @recovery_info: Informations that Android recovery mode uses.
 * @gpio: Slot's gpio information including pins of write-protect, card-detect and power.
 * @pio_mode: Indicate that whether the MSC host use PIO mode.
 * @private_init: Board private initial function, mostly for SDIO devices.
 */
struct ingenic_mmc_pdata {
	unsigned short          removal;
	unsigned short          sdio_clk;
	unsigned int            ocr_avail;
	unsigned int            capacity;
	unsigned int            pm_flags;
	struct card_gpio        *gpio;
	unsigned int            pio_mode;
	int (*private_init)(void);
};

/**
 * struct ingenic_mmc_host - Ingenic MMC/SD Controller host structure
 * @pdata: The platform data.
 * @dev: The mmc device pointer.
 * @irq: Interrupt of MSC.
 * @clk: Main Clk of MSC, including cgu and clk gate.
 * @pwc_clk: Power of MSC module, MSC register can be access when pwc_clk on.
 * @power: Power regulator of SD/MMC attached to SD slot.
 * @mrq: mmc_request pointer which includes all the information
 *  of the current request, or NULL when the host is idle.
 * @cmd: Command information of mmc_request.
 * @data: Data information of mmc_request, or NULL when mrq without
 *  data request.
 * @mmc: The mmc_host representing this slot.
 * @pending_events: Bitmask of events flagged by the interrupt handler
 *  to be processed by the state machine.
 * @iomem: Pointer to MSC registers.
 * @detect_timer: Timer used for debouncing card insert interrupts.
 * @request_timer: Timer used for preventing request time out.
 * @flags: Random state bits associated with the slot.
 * @cmdat: Variable for MSC_CMDAT register.
 * @cmdat_def: Defalt CMDAT register value for every request.
 * @gpio: Information of gpio including cd, wp and pwr.
 * @index: Number of each MSC host.
 * @decshds[]: Descriptor DMA information structure.
 * @state: It's the state for request.
 * @list: List head for manually detect card such as wifi.
 * @lock: Lock the registers operation.
 * @double_enter: Prevent state machine reenter.
 * @timeout_cnt: The count of timeout second.
 */
#define MAX_SEGS        128 /* max count of sg */
#define INGENIC_MMC_CARD_PRESENT    0
#define INGENIC_MMC_CARD_NEED_INIT  1
#define INGENIC_MMC_USE_PIO     2

struct ingenic_mmc_host {
	void __iomem                *iomem;
	struct device               *dev;
	struct clk                  *clk_cgu;
	struct clk                  *clk_gate;
	struct regulator            *power;
	struct mmc_request          *mrq;
	struct mmc_command          *cmd;
	struct mmc_data             *data;
	struct mmc_host             *mmc;
	struct timer_list            detect_timer;
	struct timer_list            request_timer;
	struct tasklet_struct        tasklet;
	struct list_head             list;
	struct ingenic_mmc_pdata    *pdata;
	struct desc_hd               decshds[MAX_SEGS];
	enum ingenic_mmc_state       state;
	spinlock_t                   lock;
	unsigned long                pending_events;
	unsigned long                flags;
	unsigned int                 cmdat;
	unsigned int                 cmdat_def;
	unsigned int                 index;
	unsigned int                 double_enter;
	int                          timeout_cnt;
	int                          irq;
};

struct ingenic_mmc_priv {
	void (*get_clk_name)(int id, char *cgu_name, char *gate_name);
};

/* Register access macros */
#define msc_readl(port,reg)                     \
	__raw_readl((port)->iomem + MSC_##reg)
#define msc_writel(port,reg,value)              \
	__raw_writel((value), (port)->iomem + MSC_##reg)
int ingenic_sdio_wlan_init(struct device *dev, int index);
int ingenic_mmc_clk_ctrl(int index, int on);
int ingenic_mmc_manual_detect(int index, int on);
#endif /* __INGENIC_MMC_H__ */
