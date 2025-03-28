#ifndef __ROT_REG_H__
#define __ROT_REG_H__

#include "dpu_ctrl.h"

#define ROT_BASE    0xb3070000

#define ROT_WRITE_REG(offset,val)     writel((val),(void*)(ROT_BASE + (offset)))
#define ROT_READ_REG(offset)  readl((void*)(ROT_BASE + (offset)))

#define BUFF_CFG_ADDR   0x2C
#define WDMA_CFG_ADDR   0x3C
#define ROT_FRM_SIZE    0x04
#define ROT_GLB_CFG 0x08
#define ROT_CTRL    0x0C
#define ROT_STATUS  0x10
#define ROT_CLR_STATUS  0x14
#define ROT_INT_MASK    0x18
#define ROT_RDMA_SITE   0x20
#define ROT_QOS_CTRL    0x30
#define ROT_COUNT       0x40

#define ROTBUF_ALIGN    512
/* frame's size(ROT_FRM_SIZE) bit field define */

/* Frame's width */
#define ROT_FRM_WIDTH_LBIT                      (0)
#define ROT_FRM_WIDTH_HBIT                      (10)
#define ROT_FRM_WIDTH_MASK            \
	GENMASK(ROT_FRM_WIDTH_HBIT, ROT_FRM_WIDTH_LBIT)
/* Frame's height */
#define ROT_FRM_HEIGHT_LBIT                     (16)
#define ROT_FRM_HEIGHT_HBIT                     (26)
#define ROT_FRM_HEIGHT_MASK           \
	GENMASK(ROT_FRM_HEIGHT_HBIT, ROT_FRM_HEIGHT_LBIT)

/* global config(ROT_GLB_CFG) bit field define */

/* RDMA max length of the block DMA's burst. */
#define ROT_RDMA_BURST_LEN_LBIT                 (0)
#define ROT_RDMA_BURST_LEN_HBIT                 (1)
#define ROT_RDMA_BURST_LEN_MASK       \
	GENMASK(ROT_RDMA_BURST_LEN_HBIT, ROT_RDMA_BURST_LEN_LBIT)

#define ROT_RDMA_BURST_4                (0) << ROT_RDMA_BURST_LEN_LBIT
/* Rotater angle. */
#define ROT_ANGLE_LBIT                  (4)
#define ROT_ANGLE_HBIT                  (5)
#define ROT_ANGLE_MASK                \
	GENMASK(ROT_ANGLE_HBIT, ROT_ANGLE_LBIT)
#define ROT_ANGLE_90                    (1) << ROT_ANGLE_LBIT
#define ROT_ANGLE_180                   (2) << ROT_ANGLE_LBIT
#define ROT_ANGLE_270                   (3) << ROT_ANGLE_LBIT
/* RDMA ORDER. */
#define ROT_RDMA_ORDER_LBIT                     (16)
#define ROT_RDMA_ORDER_HBIT                     (18)
#define ROT_RDMA_ORDER_MASK           \
	GENMASK(ROT_RDMA_ORDER_HBIT, ROT_RDMA_ORDER_LBIT)

#define ROT_RDMA_ORDER_RGB              (0) << ROT_RDMA_ORDER_LBIT
/* MANUAL_AUTO. */
#define ROT_MANUAL_AUTO                 BIT(19)
/* FIFO GATE. */
#define ROT_FIFO_GATE                   BIT(20)
/* RDMA format. */
#define ROT_RDMA_FMT_LBIT                       (24)
#define ROT_RDMA_FMT_HBIT                       (27)
#define ROT_RDMA_FMT_MASK             \
	GENMASK(ROT_RDMA_FMT_HBIT, ROT_RDMA_FMT_LBIT)

#define ROT_RDMA_FMT_CUSTOM             (3) << ROT_RDMA_FMT_LBIT

/* rotate control(ROT_CTRL) bit field define */

/* QCK_STOP */
#define ROT_QCK_STP                     BIT(1)
/* GEN_STOP */
#define ROT_GEN_STP                     BIT(2)
/* flush command fifo. */
#define ROT_FLUSH_FIFO          BIT(4)

/* rotate status(ROT_SATUS) bit field define */

/* Rotater is working */
#define ROT_WORKING                     BIT(0)
/* Rotater is general stop */
#define ROT_GEN_STOP_ACK                BIT(1)
/* One frame read end */
#define ROT_EOF                         BIT(2)
/* One frme read start */
#define ROT_SOF                         BIT(3)

#define ROT_BUF_WORKING                 BIT(7)
/* One address was taken */
#define ROT_FETCG_ADDR                  BIT(4)

/* rotate clear status(ROT_CLR_ST) bit field define */

/* Clear general stop acknowledge */
#define ROT_CLR_GEN_STOP_ACK            BIT(1)
/* Clear FRM_END */
#define ROT_CLR_EOF                     BIT(2)
/* Clear FRM_START */
#define ROT_CLR_SOF                     BIT(3)
/* clear FETCH_ADDR. */
#define ROT_CLR_FETCG_ADDR              BIT(4)

/* rotator interrupt mask(ROT_INT_MASK) bit field define */

/* Mask general stop acknowledge */
#define ROT_GSA_MASK                    BIT(1)
/* Mask FRM_END */
#define ROT_EOF_MASK                    BIT(2)
/* Mask FRM_START */
#define ROT_SOF_MASK                    BIT(3)
/* Mask FA_MASK */
#define ROT_FA_MASK         BIT(4)
/* rotate qos control(ROT_QOS_CTRL) bit field define */
#define QOS_FIX_EN          BIT(0)
#define QOS_FIX_0           (0 << 1)
#define QOS_FIX_1           (1 << 1)
#define QOS_FIX_2           (2 << 1)
#define QOS_FIX_3           (3 << 1)

enum rot_mode {
	ROT_MODE_WDMA_AUTO = 0,   // 分配BUF用于rotate,使用 rot auto 模式
	ROT_MODE_RDMA_MANUAL,       // 共享RDMA BUF用于rotate,使用 rot_manual 模式
};
void dump_rot_registers(void);
void rotator_cfg_init(struct rot_cfg *rot_cfg);
void rot_manual_start(struct dpu_ctrl *dctrl);
void rot_quick_stop(void);
int rot_ctrl_init(struct dpu_ctrl *dctrl, int angle, enum rot_mode mode);
int rot_ctrl_exit(struct dpu_ctrl *dctrl);
void rot_normal_stop(void);

static inline int rotator_aligned_size(int sz)
{
	//#define CEIL_ALIGN(size,aligned)  ((size + aligned - 1) / aligned * aligned)
#define CEIL_ALIGN(size,aligned)  (((size) / aligned + 1) * aligned)
#ifdef CONFIG_ROT_INGENIC
	return CEIL_ALIGN(sz, 512);
#else
	return sz;
#endif
#undef CEIL_ALIGN
}

#endif
