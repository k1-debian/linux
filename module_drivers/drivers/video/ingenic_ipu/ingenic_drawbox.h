#ifndef __DBOX__H__
#define __DBOX__H__

#define DBOX_BASE   0xB30D0000

#define DBOX_CTRL       0x00
#define DBOX_YB         0x04
#define DBOX_CB         0x08
#define DBOX_STRIDE     0x0c
#define DBOX_IMG_WH     0x10
#define DBOX_COLOR_Y    0x14
#define DBOX_COLOR_U    0x18
#define DBOX_COLOR_V    0x1c
#define DBOX_RAM        0x20
#define DBOX_TIMEOUT    0x24

#define u32 unsigned int

#define DBOX_RAM_SIZE 24

struct dbox_reg_struct {
	char *name;
	unsigned int addr;
};

struct dbox_buf_info {
	unsigned int vaddr_alloc;
	unsigned int paddr;
	unsigned int paddr_align;
	unsigned int size;
};

struct jz_dbox {

	int irq;
	char name[16];

	struct clk *clk;
	struct clk *ahb0_gate; /* T40 drawbox mount at AHB0*/
	void __iomem *iomem;
	struct device *dev;
	struct resource *res;
	struct miscdevice misc_dev;

	struct mutex mutex;
	struct completion done_dbox;
	struct completion done_buf;
	struct dbox_buf_info pbuf;
};

struct dbox_flush_cache_para {
	void *addr;
	unsigned int size;
};

struct dbox_ram {
	unsigned int        box_x;
	unsigned int        box_y;
	unsigned int        box_w;
	unsigned int        box_h;
	unsigned int        line_w;
	unsigned int        line_l;
	unsigned int        box_mode;
	unsigned int        color_mode;
};

struct dbox_param {
	unsigned int        box_pbuf;
	unsigned int        img_w;
	unsigned int        img_h;

	unsigned int        boxs_num;
	unsigned int        is_rgba;
	struct dbox_ram ram_para[DBOX_RAM_SIZE];  //kernel stack overflow
};

#define JZDBOX_IOC_MAGIC  'X'
#define IOCTL_DBOX_START            _IO(JZDBOX_IOC_MAGIC, 106)
#define IOCTL_DBOX_RES_PBUFF        _IO(JZDBOX_IOC_MAGIC, 114)
#define IOCTL_DBOX_GET_PBUFF        _IO(JZDBOX_IOC_MAGIC, 115)
#define IOCTL_DBOX_BUF_LOCK         _IO(JZDBOX_IOC_MAGIC, 116)
#define IOCTL_DBOX_BUF_UNLOCK       _IO(JZDBOX_IOC_MAGIC, 117)
#define IOCTL_DBOX_BUF_FLUSH_CACHE  _IO(JZDBOX_IOC_MAGIC, 118)
#define IOCTL_DBOX_GET_DMA_PHY  _IO(JZDBOX_IOC_MAGIC, 119)

/* DBOX_CTRL*/
#define DBOX_RAM_RP        (17)
#define DBOX_RAM_WP        (9)
#define DBOX_TIMEOUT_MASK  (1 << 8)
#define DBOX_TIMEOUT_IRQ   (1 << 7)
#define DBOX_IS_BGRA       (1 << 6)
#define DBOX_CLOCK_GATE_MASK (1 << 5)
#define DBOX_IRQ (4)
#define DBOX_IRQ_OPEN  (1 << DBOX_IRQ)
#define DBOX_IRQ_MASK (1 << 3)
#define DBOX_COLOR_MASK (1 << 2)
#define DBOX_RESET (1 << 1)
#define DBOX_START (1 << 0)

/* DBOX_YB luma base register */
#define DBOX_LUMA_BASE (0)

/* DBOX_CB */
#define DBOX_CONCENTRATION_BASE (0)

/* DBOX_STRIDE */
#define DBOX_C_STRIDE       (16)
#define DBOX_Y_STRIDE       (0)

/* DBOX_IMG_WH */
#define DBOX_WIDTH       (0)
#define DBOX_HEIGHT      (16)

/* DBOX_COLOR_Y */
#define DBOX_COLOR_ADDR_Y            (0)

/* DBOX_COLOR_U */
#define DBOX_COLOR_ADDR_U       (0)

/* DBOX_COLOR_V */
#define DBOX_COLOR_ADDR_V      (0)

/* DBOX_RAM */
#define DBOX_BOX_LINE_WIDTH     (55)
#define DBOX_BOX_LINE_LENGTH    (52)
#define DBOX_BOX_MODE      (50)
#define DBOX_MODE_RECT     (0 << DBOX_BOX_MODE)
#define DBOX_MODE_HRECT    (1 << DBOX_BOX_MODE)
#define DBOX_MODE_HLINE    (2 << DBOX_BOX_MODE)
#define DBOX_MODE_VLINE    (3 << DBOX_BOX_MODE)
#define DBOX_COLOR_MODE    (48)
#define DBOX_BOX_HEIGHT    (36)
#define DBOX_BOX_WIDTH     (24)
#define DBOX_BOX_Y         (12)
#define DBOX_BOX_X         (0)

//extern void dboxdev_init(void);
//extern void dboxdev_exit(void);

static inline unsigned int dbox_reg_read(struct jz_dbox *jzdbox, int offset)
{
	return readl(jzdbox->iomem + offset);
}

static inline void dbox_reg_write(struct jz_dbox *jzdbox, int offset, unsigned int val)
{
	writel(val, jzdbox->iomem + offset);
}

#define __dbox_mask_irq()       reg_bit_clr(dbox, DBOX_CTRL, DBOX_IRQ_MASK)
#define __dbox_irq_clear(value)     reg_bit_clr(dbox, DBOX_CTRL, DBOX_IRQ_OPEN)

#define __start_dbox()          reg_bit_set(dbox, DBOX_CTRL, DBOX_START)
#define __reset_dbox()          reg_bit_set(dbox, DBOX_CTRL, DBOX_RESET)
#define __uv_mask_dbox()            reg_bit_set(dbox, DBOX_CTRL, DBOX_COLOR_MASK)

#endif
