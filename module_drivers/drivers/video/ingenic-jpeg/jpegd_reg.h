#ifndef __JPEGD_REG_H__
#define __JPEGD_REG_H__

#define JPEGDEC_BASE    0xb3200000

#define JPEGD_WRITE_REG(offset,val) writel((val),(void __iomem *)JPEGDEC_BASE + (offset))
#define JPEGD_READ_REG(offset)  readl((void __iomem *)JPEGDEC_BASE + (offset))

/**/
#define JPEGD_CTL_STA   0x00
#define JPEGD_ROW_COL   0x04
#define JPEGD_RD_ADDR   0x08
#define JPEGD_WR_ADDR   0x0C
#define JPEGD_FILE_SIZE 0x10
#define JPEGD_FRM_CNT   0x14
#define JPEGD_WORD_CNT  0x18
#define JPEGD_ERR_CNT   0x1C
#define JPEGD_PIX_DEC_CNT   0x20
#define JPEGD_PIX_OUT_CNT   0x24
#define JPEGD_DEC_SIZE  0X28
#define JPEGD_RESVER_STA    0x30
#define JPEGD_FB_ROW_COL    0x34
#define JPEGD_WDT_PERIOD    0x78
#define JPEGD_ERR_HW_FLG    0x7C

/* CTR_STA  */
#define PIX_OUT_ORDER   (1 << 21)
#define DEFAULT_OUT_PIX     (1 << 20)
#define IMG_PIX_TYPE    (7 << 16)
#define IMG_PRASED_FLAG (1 << 15)
#define ERR_IRQ_MASK    (1 << 9)
#define FINISH_IRQ_MASK (1 << 8)
#define DEC_FINISH      (1 << 7)
#define INT_STA         (1 << 4)
#define SOFT_RST        (1 << 2)
#define ERR_DETECT      (1 << 1)
#define DEC_START       (1 << 0)

#define JPEGD_OUT_DEFAULT   0
#define JPEGD_OUT_YUV422P   1
#define JPEGD_OUT_NV12      2
#define JPEGD_OUT_ARGB32    3
#define JPEGD_OUT_NV21      4

#define _jpegd_finish_irq_disable() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,FINISH_IRQ_MASK)

#define _jpegd_finish_irq_enable()  \
	reg_bit_clr(jpegd,JPEGD_CTL_STA,FINISH_IRQ_MASK)

#define _jpegd_err_irq_disable()    \
	reg_bit_set(jpegd,JPEGD_CTL_STA,ERR_IRQ_MASK)

#define _jpegd_err_irq_enable() \
	reg_bit_clr(jpegd,JPEGD_CTL_STA,ERR_IRQ_MASK)

#define _jpegd_start()      \
	reg_bit_set(jpegd,JPEGD_CTL_STA,DEC_START)

#define _jpegd_out_normal_order() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,PIX_OUT_ORDER)

#define _jpegd_out_yuv() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,DEFAULT_OUT_PIX)

#define _jpegd_out_rgb() \
	reg_bit_clr(jpegd,JPEGD_CTL_STA,DEFAULT_OUT_PIX)

#define _jpegd_clear_err_hw_flag()\
	reg_bit_set(jpegd,JPEGD_ERR_HW_FLG,0xffffffff)

#define _jpegd_soft_reset() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,SOFT_RST)

#define _jpegd_clear_int() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,INT_STA)

#define _jpegd_clear_err() \
	reg_bit_set(jpegd,JPEGD_CTL_STA,ERR_DETECT)

#endif
