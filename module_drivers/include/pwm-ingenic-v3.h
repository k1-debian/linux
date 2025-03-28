#ifndef __LINUX_PWM_INGENIC_V3_H
#define __LINUX_PWM_INGENIC_V3_H

//#define PWM_DBUG
#ifndef PWM_DBUG
	#define print_dbg(fmt, argv...)
#else
	#define print_dbg(fmt, argv...) printk(fmt, ##argv)
#endif

#define BUFFER_SIZE     (PAGE_SIZE * 16)
#define FRAME_SIZE      (PAGE_SIZE * 4)
#define SG_PWM_NUM      (100)
#define PWM_SIGDMA      44
#define NS_IN_HZ        (1000000000UL)
#define DEFAULT_PWM_CLK_RATE    (400000000)

#define PWM_ENS     (0x00)
#define PWM_ENC     (0x04)
#define PWM_EN      (0x08)
#define PWM_UPT     (0x10)
#define PWM_BUSY    (0x14)
#define PWM_MS      (0x20)
#define PWM_INL     (0x24)
#define PWM_IDL     (0x28)
#define PWM_CCFG0   (0x40)
#define PWM_WCFG0   (0x80)
#define PWM_DR0     (0xc0)
#define PWM_DFN0    (0x100)
#define PWM_DRTN0   (0x140)
#define PWM_DRE     (0x180)
#define PWM_DRS     (0x184)
#define PWM_DFIE    (0x188)
#define PWM_DFS     (0x18c)
#define PWM_DAFF    (0x190)
#define PWM_DCFF    (0x194)
#define PWM_SS      (0x200)
#define PWM_SIE     (0x204)
#define PWM_SC0     (0x210)
#define PWM_SN0     (0x250)
#define PWM_ON0     (0x290)
#define PWM_CCFG(n) (PWM_CCFG0 + 4*n)
#define PWM_WCFG(n) (PWM_WCFG0 + 4*n)
#define PWM_DR(n)   (PWM_DR0 + 4*n)
#define PWM_DFN(n)  (PWM_DFN0 + 4*n)
#define PWM_DRTN(n) (PWM_DRTN0 + 4*n)
#define PWM_SC(n)   (PWM_SC0 + 4*n)
#define PWM_SN(n)   (PWM_SN0 + 4*n)
#define PWM_ON(n)   (PWM_ON0 + 4*n)

#define PWM_WCFG_HIGH       (16)
#define PWM_WCFG_LOW        (0)

#define PWM_STORE_SE    BIT(18)
#define PWM_STORE_SPE   BIT(17)
#define PWM_STORE_SNE   BIT(16)
#define PWM_STORE_SFN_LBIT  0
#define PWM_STORE_SFN_HBIT  9
#define PWM_STORE_SFN_MASK  \
	GENMASK(PWM_STORE_SFN_HBIT, PWM_STORE_SFN_LBIT)

#define PWM_ENABLE_FLAG 1

struct ingenic_pwm_channel {
	u32 period_ns;
	u32 duty_ns;
};

struct ingenic_pwm_chan {
	int id;
	struct ingenic_pwm_chip *chip;
	struct dma_chan     *dma_chan;
	struct scatterlist      *sg;    /* I/O scatter list */
	void            *buffer;
	dma_addr_t      buffer_dma;
	unsigned int        total_buffer_size;  /* 用于dma cyclic 传输的总大小.*/
	unsigned int        frame_size;     /* 用于控制每帧数据的大小. total_buffer_size = frame_size * n */
	int mode;
	int dma_pos;
	int user_pos;
	unsigned int duty_ns;
	unsigned int period_ns;
	bool init_level;
	bool finish_level;
	bool trigger_en;
	bool trigger_posedge;
	bool trigger_negedge;
	int trigger_filter;
	unsigned int store_irq_num;
	unsigned int sg_pwm_num;
	unsigned int full_duty_status; /*Record last output level*/
	unsigned int en_status;
	unsigned int flags;        /*Used for some judgment needs*/
	unsigned int sys_prescale;

	/*pwm audio defined*/
	bool pwm_ext_en;
	unsigned int clk_in;
	struct mutex *mutex;
	struct completion *done_tx_dma;
	void (* callback)(void *);
	void *callback_param;
	/*pwm audio defined*/

};

enum pwm_mode_sel {
	COMMON_MODE,
	DMA_MODE_SG,
	DMA_MODE_CYCLIC,
	DMA_MODE_SMC,
};

struct ingenic_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk_pwm;
	struct clk *clk_gate;
	void __iomem    *iomem;
	int irq;
	struct mutex mutex;
	unsigned int output_mask;

	struct pwm_device **debug_pwm;
	int debug_current_id;

	unsigned long       phys;
	struct ingenic_pwm_chan *chan;
	const struct of_device_id *priv;
	struct pwm_chip_priv *chip_priv;
};

struct pwm_chip_priv {
	unsigned int prescale;
	int version_num;
	int npwm;   /* number of pwm chan. */
};

static inline struct ingenic_pwm_chip *to_ingenic_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct ingenic_pwm_chip, chip);
};

#endif /* __LINUX_PWM_INGENIC_V3_H*/
