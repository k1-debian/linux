#ifndef __JPEGE_REG_H__
#define __JPEGE_REG_H__

#define JPEGENC_BASE    0xb3210000

#define JPEGE_WRITE_REG(offset,val) writel((val),(unsigned int *)(JPEGENC_BASE + (offset)))
#define JPEGE_READ_REG(offset)  readl((unsigned int *)(JPEGENC_BASE + (offset)))

#define JPEGE_VERSION           0x00
#define JPEGE_CTL_STA           0x10
#define JPEGE_FRM_SIZE          0x14
#define JPEGE_CAP_CTL_STA       0x18
#define JPEGE_PIX_IN_STA        0x20
#define JPEGE_PIX_IN_ADDR       0x24
#define JPEGE_UV_OFFSET         0x28
#define JPEGE_OUT_STA           0x30
#define JPEGE_OUT_ADDR          0x34
#define JPEGE_OUT_BUFFER_SIZE   0x38
#define JPEGE_IMGE_SIZE         0x3C
#define JPEGE_NEXT_OUT_ADDR     0x40    // for read? this is 32bit addr
#define JPEGE_STA_FLAG1         0x78
#define JPEGE_STA_FLAG2         0x7C

/* control / status  */
#define ERR_FLAG        (1 << 15)
#define YCBCR_MASK      (1 << 14)
#define DATAENC_IDLE    (1 << 13)
#define IPCORE_IDLE     (1 << 12)
#define SUBSAMP_MASK    (2 << 10)
#define CLR_FLAGS_MASK  (1 << 9)
#define GRAYSCALE_MASK  (1 << 8)
#define QUALITY_MASK    (0x7f << 1)
#define START_MASK      (1 << 0)

#define _start_jpege() do{\
		reg_bit_set(jpege,JPEGE_CTL_STA,START_MASK);    \
	}while(0)

#define _set_quality(x) do{\
		reg_bit_clr(jpege,JPEGE_CTL_STA,QUALITY_MASK);      \
		reg_bit_set(jpege,JPEGE_CTL_STA,(x & 0x7f) << 1);   \
	}while(0)

#define _clear_all_status() do{\
		reg_bit_set(jpege,JPEGE_CTL_STA,CLR_FLAGS_MASK);    \
	}while(0)

#define _input_is_yuv() do{\
		reg_bit_clr(jpege,JPEGE_CTL_STA,GRAYSCALE_MASK); \
		reg_bit_set(jpege,JPEGE_CTL_STA,YCBCR_MASK);  \
	}while(0)

#define _input_is_grayscale() do{\
		reg_bit_set(jpege,JPEGE_CTL_STA,YCBCR_MASK); \
		reg_bit_set(jpege,JPEGE_CTL_STA,GRAYSCALE_MASK); \
	}while(0)

#define _input_is_rgb() do{\
		reg_bit_clr(jpege,JPEGE_CTL_STA,GRAYSCALE_MASK); \
		reg_bit_clr(jpege,JPEGE_CTL_STA,YCBCR_MASK); \
	}while(0)

#define _set_input_subsamp(x) \
	reg_bit_set(jpege,JPEGE_CTL_STA,(x & 3) << 10)

#define _jpege_clr_int() \
	reg_bit_set(jpege,JPEGE_CTL_STA,CLR_FLAGS_MASK)

#endif
