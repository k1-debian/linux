#ifndef _VIC_SENSOR_INFO_H_
#define _VIC_SENSOR_INFO_H_

typedef enum {
	SENSOR_DATA_BUS_MIPI = 0,
	SENSOR_DATA_BUS_DVP,
	SENSOR_DATA_BUS_BT601,      /* 暂不支持 */
	SENSOR_DATA_BUS_BT656,      /* 暂不支持 */
	SENSOR_DATA_BUS_BT1120,     /* 暂不支持 */
	SENSOR_DATA_BUS_BUTT,
} sensor_data_bus_type;

typedef enum {
	DVP_RAW8            = 0,    /* 该顺序不可调整 */
	DVP_RAW10,
	DVP_RAW12,
	DVP_YUV422,
	DVP_RESERVED1,
	DVP_RESERVED2,
	DVP_YUV422_8BIT,
} dvp_data_fmt;

typedef enum {
	DVP_PA_LOW_10BIT,
	DVP_PA_HIGH_10BIT,
	DVP_PA_12BIT,
	DVP_PA_LOW_8BIT,
	DVP_PA_HIGH_8BIT,
} dvp_gpio_mode;

typedef enum {
	DVP_HREF_MODE,
	DVP_HSYNC_MODE,
	DVP_SONY_MODE,
} dvp_timing_mode;

/*                         clk1,clk2,clk3,clk4
 * 初始yuv 4字节顺序_1_2_3_4   1    2    3    4
 * 可转变成以下顺序
 */
typedef enum {
	order_2_1_4_3,
	order_2_3_4_1,
	order_1_2_3_4,
	order_1_4_3_2,
} yuv_data_order;

typedef enum {
	POLARITY_HIGH_ACTIVE,
	POLARITY_LOW_ACTIVE,
} dvp_sync_polarity;

typedef enum {
	POLARITY_SAMPLE_RISING,
	POLARITY_SAMPLE_FALLING,
} dvp_sample_polarity;

typedef enum {
	DVP_IMG_SCAN_PROGRESS,
	DVP_IMG_SCAN_INTERLACE,
} dvp_img_scan_mode;

struct dvp_bus {
	dvp_data_fmt data_fmt;
	dvp_gpio_mode gpio_mode;
	dvp_timing_mode timing_mode;
	yuv_data_order yuv_data_order;
	dvp_sample_polarity pclk_polarity;
	dvp_sync_polarity hsync_polarity;
	dvp_sync_polarity vsync_polarity;
	dvp_img_scan_mode img_scan_mode;
};

/*
 * MIPI information
 */
typedef enum {
	MIPI_RAW8           = 0,    /* 该顺序不可调整 */
	MIPI_RAW10,
	MIPI_RAW12,
	MIPI_RESERVED1,
	MIPI_RESERVED2,
	MIPI_RESERVED3,
	MIPI_RESERVED4,
	MIPI_YUV422         = 7,
	MIPI_RESERVED5,
} mipi_data_fmt;

struct mipi_bus {
	int data_lanes;
	mipi_data_fmt data_fmt;
	unsigned short clk_settle_time;  /* unit: ns, range: 95 ~ 300ns */
	unsigned short data_settle_time; /* unit: ns, range: 85 ~ 145ns + 10*UI */
};

struct vic_hal_info {
	sensor_data_bus_type bus_type;

	struct mipi_bus mipi;
	struct dvp_bus dvp;

	int width;
	int height;
	int is_y8;
	int index;
	unsigned int isp_clk_rate;
	void *iobase;
};

#endif /* _VIC_SENSOR_INFO_H_ */
