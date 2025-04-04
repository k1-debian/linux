#ifndef __INGENIC_ISP_VIC_REG_H__
#define __INGENIC_ISP_VIC_REG_H__

#define VIC_CONTROL         0x00
#define VIC_RESOLUTION          0x04
#define VIC_FRM_ECC                     0x08
#define VIC_INTF_TYPE                   0x0C
#define VIC_IN_DVP          0x10
#define VIC_IN_CSI_FMT                  0x14
#define VIC_IN_HOR_PARA0                0x18
#define VIC_IN_HOR_PARA1                0x1C
#define VIC_BK_CB_CTRL                  0x28
#define VIC_BK_CB_BLK                   0x2C
#define VIC_IN_VER_PARA0                0x30
#define VIC_IN_VER_PARA1                0x34
#define VIC_IN_VER_PARA2                0x38
#define VIC_IN_VER_PARA3                0x3C
#define VIC_VLD_LINE_SAV        0x60
#define VIC_VLD_LINE_EAV        0x64
#define VIC_VLD_FRM_SAV         0x70
#define VIC_VLD_FRM_EAV         0x74
#define VIC_VC_CONTROL          0x8C
#define VIC_VC_CONTROL_CH0_PIX      0x90
#define VIC_VC_CONTROL_CH1_PIX      0x94
#define VIC_VC_CONTROL_CH2_PIX      0x98
#define VIC_VC_CONTROL_CH3_PIX      0x9C
#define VIC_VC_CONTROL_CH0_LINE     0xA0
#define VIC_VC_CONTROL_CH1_LINE     0xA4
#define VIC_VC_CONTROL_CH2_LINE     0xA8
#define VIC_VC_CONTROL_CH3_LINE     0xAC
#define VIC_VC_CONTROL_FIFO_USE     0xB0
#define VIC_CB_1ST          0xC0
#define VIC_CB_2ND          0xC4
#define VIC_CB_3RD          0xC8
#define VIC_CB_4TH          0xCC
#define VIC_CB_5TH          0xD0
#define VIC_CB_6TH          0xD4
#define VIC_CB_7TH          0xD8
#define VIC_CB_8TH          0xDC
#define VIC_CB2_1ST         0xE0
#define VIC_CB2_2ND         0xE4
#define VIC_CB2_3RD         0xE8
#define VIC_CB2_4TH         0xEC
#define VIC_CB2_5TH         0xF0
#define VIC_CB2_6TH         0xF4
#define VIC_CB2_7TH         0xF8
#define VIC_CB2_8TH         0xFC
#define MIPI_ALL_WIDTH_4BYTE        0x100
#define MIPI_VCROP_DEL01        0x104
#define MIPI_SENSOR_CONTROL     0x10C
#define MIPI_HCROP_CH0          0x110
#define MIPI_VCROP_SHADOW_CFG       0x120
#define VIC_CONTROL_LIMIT       0x1A0
#define VIC_CONTROL_DELAY       0x1A4
#define VIC_CONTROL_TIZIANO_ROUTE   0x1A8
#define VIC_CONTROL_DMA_ROUTE       0x1B0
#define VIC_INT_STA         0X1E0
#define VIC_INT_MASK            0x1E8
#define VIC_INT_CLR         0x1F0
/*DMA register*/
#define VIC_DMA_OUTPUT_MAX_WIDTH 2688
#define VIC_DMA_CONFIG          0x300
#define VIC_DMA_RESOLUTION      0x304
#define VIC_DMA_RESET           0x308
#define VIC_DMA_Y_STRID         0x314
#define VIC_DMA_Y_BUF0          0x318
#define VIC_DMA_Y_BUF1          0x31c
#define VIC_DMA_Y_BUF2          0x320
#define VIC_DMA_Y_BUF3          0x324
#define VIC_DMA_Y_BUF4          0x328
#define VIC_DMA_UV_STRID        0x334
#define VIC_DMA_UV_BUF0         0x338
#define VIC_DMA_UV_BUF1         0x33c
#define VIC_DMA_UV_BUF2         0x340
#define VIC_DMA_UV_BUF3         0x344
#define VIC_DMA_UV_BUF4         0x348

/*TIMESTAMP registers*/
#define VIC_TS_ENABLE           0x360   /* 控制使能*/
#define TS_COUNTER_EN           (1 << 0)/*使能计数器*/
#define TS_VIC_DMA_EN           (1 << 4)/*使能VIC DMA的timstsamp*/
#define TS_MS_CH0_EN            (1 << 5)/*使能MSCALER CH0 ts*/
#define TS_MS_CH1_EN            (1 << 6)/*使能MSCALER CH1 ts*/
#define TS_MS_CH2_EN            (1 << 7)/*使能MSCALER CH2 ts*/
#define VIC_TS_COUNTER          0x368   /* 多少个ISP时钟周期计数一次 */
/* timestamp 写入到帧数据中的偏移位置.*/
#define VIC_TS_DMA_OFFSET       0x370
#define VIC_TS_MS_CH0_OFFSET        0x374
#define VIC_TS_MS_CH1_OFFSET        0x378
#define VIC_TS_MS_CH2_OFFSET        0x37c

#define    LINE_SYNC_MODE               22

/*VIC_CONTROL*/
//#define VIC_RESET         (1<<4)
#define GLB_RST             (1<<2)
#define REG_ENABLE          (1<<1)
#define VIC_START           (1<<0)

/*VIC_RESOLUTION*/
#define H_RESOLUTION            (1<<16)
#define V_RESOLUTION            (1)

/*VIC_FRM_ECC*/
#define FRAME_ECC_EN               (1<<0)
#define FRAME_ECC_MODE             (1<<1)

/*VIC_INTF_TYPE*/
#define INTF_TYPE_BT656         0x0
#define INTF_TYPE_BT601         0x1
#define INTF_TYPE_MIPI          0x2
#define INTF_TYPE_DVP           0x3
#define INTF_TYPE_BT1120        0x4

/*VIC_IN_DVP*/
#define DVP_DATA_POS            (1<<24)
#define DVP_RGB_ORDER           (1<<21)
#define DVP_RAW_ALIG            (1<<20)
#define DVP_DATA_TYPE           (17)
#define DVP_RAW8            (0<<DVP_DATA_TYPE)
#define DVP_RAW10           (1<<DVP_DATA_TYPE)
#define DVP_RAW12           (2<<DVP_DATA_TYPE)
#define DVP_YUV422_16BIT        (3<<DVP_DATA_TYPE)
//#define DVP_RGB565_16BIT      (4<<DVP_DATA_TYPE)
//#define DVP_BRG565_16BIT      (5<<DVP_DATA_TYPE)
#define DVP_YUV422_8BIT         (6<<DVP_DATA_TYPE)
//#define DVP_RGB565_8BIT           (7<<DVP_DATA_TYPE)
#define DVP_TIMEING_MODE        (1<<15)
#define DVP_HSYNC_MODE          (1 << 15)
#define DVP_SONY_MODE           (2 << 15)
#define BT_INTF_WIDE            (1<<11)
#define BT_LINE_MODE            (1<<10)
#define BT_601_TIMING_MODE      (1<<9)
#define YUV_DATA_ORDER          (4)
#define YVYU                (0<<YUV_DATA_ORDER)
#define YUYV                (1<<YUV_DATA_ORDER)
#define VYUY                (2<<YUV_DATA_ORDER)
#define UYVY                (3<<YUV_DATA_ORDER)
#define FIRST_FIELD_TYPE        (1<<3)
#define INTERLACE_EN            (1<<2)
#define HSYN_POLAR          (1<<1)
#define VSYN_POLAR          (1<<0)

/*VIC_IN_CSI_FMT*/
#define MIPI_RAW8           0x0
#define MIPI_RAW10          0x1
#define MIPI_RAW12          0x2
//#define MIPI_RGB555           0x3
//#define MIPI_RGB565           0x4
//#define MIPI_RGB666           0x5
//#define MIPI_RGB888           0x6
#define MIPI_YUV422         0x7
//#define MIPI_YUV422_10BIT     0x8

#endif/* __VIC_REG_H__ */
