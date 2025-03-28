#ifndef __DPU_CTRL_H__
#define __DPU_CTRL_H__

#include <libdmmu.h>
#include "../include/uapi_ingenicfb.h"
#include "../include/jz_dsim.h"
#include "../include/lcd_panel.h"

#include "dpu_dma_desc.h"
#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"
#include "../jz_mipi_dsi/jz_mipi_dsi_regs.h"
#include "../jz_mipi_dsi/jz_mipi_dsi_lowlevel.h"
extern struct dsi_device *jzdsi_init(struct jzdsi_data *pdata);
extern struct dsi_device *jz_lvds_init(struct jzdsi_data *pdata, int panel_type);
extern void jzdsi_remove(struct dsi_device *dsi);
extern void dump_dsi_reg(struct dsi_device *dsi);
extern int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power);

#define DPU_COMP_MAX_FRAMEDESC 4 //(CONFIG_FB_INGENIC_NR_FRAMES)
#define DPU_MAX_REQUESTS_PER_LAYER 16

#define DESC_ALIGN 8
typedef enum stop_mode {
	QCK_STOP,
	GEN_STOP,
} stop_mode_t;

typedef enum csc_mode {
	CSC_MODE_0,
	CSC_MODE_1,
	CSC_MODE_2,
	CSC_MODE_3,
} csc_mode_t;

typedef enum data_ch {
	DATA_CH_COMP,
	DATA_CH_RDMA,
	DATA_CH_ROT,
} data_ch_t;

#define MAX_RDMA_FRAMES 5

struct rdma_setup_info {
	unsigned int nframes;
	unsigned int format;
	unsigned int color;
	unsigned int stride;
	int continuous;     /*连续帧还是单帧模式.*/
	unsigned char *vidmem[MAX_RDMA_FRAMES];
	unsigned int vidmem_phys[MAX_RDMA_FRAMES];
};

typedef enum comp_mode {
	COMP_DIRECT_OUT,        /* Composer Direct to Display */
	COMP_WRITE_BACK,        /* Composer write back to ddr */
	COMP_WRITE_BACK_2_ROTATOR,  /* Compoer write back to ddr share with rotator direct to display. */
	COMP_WRITE_BACK_2_RDMA,     /* Composer write back to ddr shared with rdma buffer to display*/
} comp_mode_t;

struct comp_setup_info {
	unsigned int nframes;
	comp_mode_t out_mode;   /*Direct Display or Writeback to Memory.*/

	struct ingenicfb_frm_cfg frm_cfg;
	unsigned int layer_to_update[DPU_SUPPORT_MAX_LAYERS];
	unsigned int layer_update_frame[DPU_SUPPORT_MAX_LAYERS]; // updated buffer index per layer.
	unsigned int layer_frames[DPU_SUPPORT_MAX_LAYERS]; // max frame buffers per layer.
	unsigned int layer_framesize[DPU_SUPPORT_MAX_LAYERS]; // framesize per layer.
};

/*
    compose requet for Layer.
*/
struct comp_layer_request {
	struct list_head list;
	struct ingenicfb_lay_cfg cfg;
};

struct buffer_addr_area {
	unsigned int start[3];
	unsigned int end[3];
	int cnt;
};

struct dbg_irqcnt {
	unsigned long long irqcnt;
	unsigned long long cmp_start;
	unsigned long long stop_disp_ack;
	unsigned long long disp_end;
	unsigned long long tft_under;
	unsigned long long wdma_over;
	unsigned long long wdma_end;
	unsigned long long layer3_end;
	unsigned long long layer2_end;
	unsigned long long layer1_end;
	unsigned long long layer0_end;
	unsigned long long clr_cmp_end;
	unsigned long long stop_wrbk_ack;
	unsigned long long srd_start;
	unsigned long long srd_end;
	unsigned long long cmp_w_slow;
};
#define dbg_irqcnt_inc(dbg,x)   dbg.x++

struct rot_cfg {
	unsigned int next_wbbuf;
	unsigned int rot_angle;
	unsigned int rot_manual_auto;
	unsigned int rot_stop;
	unsigned int rot_angle_set;
	u32 xres;
	u32 yres;
};
struct dpu_ctrl {

	int irq;        /* lcdc interrupt num */
	int roirq;
	int dsi_irq;
	void __iomem *base;
	struct resource *mem;
	struct device *dev;
	struct platform_device *pdev;

	struct lcd_panel *panel;
	struct fb_videomode *active_video_mode;

	char clk_name[16];
	char pclk_name[16];
	char irq_name[16];
	char roirq_name[16];
	char dsi_irq_name[16];
	struct clk *clk;
	struct clk *pclk;
	struct clk *rot_clk;

	spinlock_t  irq_lock;

	/* wait 事件.*/
	int srd_stopped;
	int comp_stopped;
	int wback_end;
	wait_queue_head_t wq;
	struct dsi_device *dsi;

	void (*set_vsync_value)(void *data);
	void *vsync_data;

	unsigned int user_addr;
	unsigned int tlba;
	unsigned int tlb_err_cnt;

	struct ingenicfb_sreadesc *sreadesc[CONFIG_FB_INGENIC_NR_FRAMES * 2];
	dma_addr_t sreadesc_phys[CONFIG_FB_INGENIC_NR_FRAMES * 2];
	unsigned int sreadesc_size;

	struct ingenicfb_framedesc *framedesc[DPU_COMP_MAX_FRAMEDESC];
	dma_addr_t framedesc_phys[DPU_COMP_MAX_FRAMEDESC];
	unsigned int framedesc_size;

	struct ingenicfb_layerdesc *layerdesc[DPU_COMP_MAX_FRAMEDESC][DPU_SUPPORT_MAX_LAYERS];
	dma_addr_t layerdesc_phys[DPU_COMP_MAX_FRAMEDESC][DPU_SUPPORT_MAX_LAYERS];
	unsigned int layerdesc_size;

	csc_mode_t csc_mode;

	unsigned int comp_last_framedesc;   /*记录软件需要更新的描述符物理地址.*/
	int comp_last_frame;            /*记录软件上一次刷新的帧.*/
	unsigned int comp_hw_framedesc;     /*记录硬件当前使用的描述符物理地址.*/

	int comp_next_frame;        /*记录comp下一帧需要合成的帧number.*/
	int comp_waitupdate;

	int rdma_changing_frame;    /*记录正在changing 的 frame.*/
	int rdma_desc_changing;     /*记录当前正处于desc 切换状态.*/
	int rdma_last_frame;        /*记录正在刷新的rdma frame index.*/
	int rdma_changed;       /*记录有新的rdma修改请求，需要硬件进行更新.*/

	int rdma_next_frame;        /*记录rdma将会使用的frame，软件可以修改frame内容.*/
	struct rdma_setup_info rdma_info;   /*记录当前硬件会使用的rdma_info*/
	struct rdma_setup_info wback_rdma_info; /*记录用于wback rdma 直通功能的setup info.*/
	struct comp_setup_info comp_info;      /*记录硬件修正后的comp_info信息.*/
	struct comp_setup_info saved_comp_info; /*记录软件填充的comp_info.*/
	int comp_info_changed;      /* 判断当前传入的comp_info是否和上一次发生变化.*/
	int layer_info_changed[DPU_SUPPORT_MAX_LAYERS];

	data_ch_t chan; /*当前显示使用的数据通道， RDMA or COMPOSER.*/

	struct dbg_irqcnt dbg_irqcnt;
	unsigned long long comp_fps;    /*记录软件更新的帧率.*/

	bool tlb_disable;
	int tlb_disable_ch;

	bool support_comp;  /*This value is false for some controllers that do not support comp mode*/
	bool work_in_rdma;
#ifdef CONFIG_SOC_X2600
	int32_t *rot_wbbuf[CONFIG_ROT_INGENIC_BUFS];
	dma_addr_t rot_wbbuf_phy[CONFIG_ROT_INGENIC_BUFS];
#endif
	int rot_init;
	struct rot_cfg rot_cfg;
	struct dmmu_mm_notifier dmn;
	int blank;
	int vblank_count;
	int vblank_layer[DPU_SUPPORT_MAX_LAYERS];

	struct list_head comp_desc_done_list;   //硬件不可用，软件可用
	struct list_head comp_desc_queued_list; //硬件使用，软件不可用列表。
	unsigned int rot_wbbuf_size;

	struct comp_layer_request layer_requests[DPU_SUPPORT_MAX_LAYERS][DPU_MAX_REQUESTS_PER_LAYER];
	struct list_head comp_layers_req_free_list[DPU_SUPPORT_MAX_LAYERS];
	struct list_head comp_layers_req_queued_list[DPU_SUPPORT_MAX_LAYERS];
	unsigned int refresh_time;
	struct work_struct dsi_work;
};

int dpu_ctrl_init(struct dpu_ctrl *dctrl, struct lcd_panel *panel);
void dpu_ctrl_exit(struct dpu_ctrl *dctrl);

int dpu_ctrl_suspend(struct dpu_ctrl *dctrl);
int dpu_ctrl_resume(struct dpu_ctrl *dctrl);

int dpu_ctrl_setup(struct dpu_ctrl *dctrl);

int dpu_ctrl_rdma_setup(struct dpu_ctrl *dctrl, struct rdma_setup_info *rdma_info);
int dpu_ctrl_rdma_start(struct dpu_ctrl *dctrl);
int dpu_ctrl_rdma_stop(struct dpu_ctrl *dctrl, enum stop_mode mode);
int dpu_ctrl_rdma_change(struct dpu_ctrl *dctrl, int frame);

int dpu_ctrl_comp_setup(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info);
int dpu_ctrl_comp_start(struct dpu_ctrl *dctrl, int block);
int dpu_ctrl_comp_wait_done(struct dpu_ctrl *dctrl);
int dpu_ctrl_comp_stop(struct dpu_ctrl *dctrl, enum stop_mode mode);
int dpu_ctrl_try_rdma_release(struct dpu_ctrl *dctrl);
int dpu_ctrl_comp_dmmu_map(struct dpu_ctrl *dctrl, struct comp_setup_info *comp_info);

int dpu_ctrl_comp_wait_for_vsync(struct dpu_ctrl *dctrl, int layer, struct buffer_addr_area *next_area);
int dpu_ctrl_rdma_wait_for_vsync(struct dpu_ctrl *dctrl);

int ingenic_set_lvds_display_clk(struct dpu_ctrl *dctrl);
void ingenic_set_pixclk(struct dpu_ctrl *dctrl, unsigned int pixclock_khz);

void dump_lcdc_registers(struct dpu_ctrl *dctrl);
void dump_comp_desc(struct dpu_ctrl *dctrl);
void dump_rdma_desc(struct dpu_ctrl *dctrl);

static inline unsigned long reg_read(struct dpu_ctrl *dctrl, int offset)
{
	return readl(dctrl->base + offset);
}

static inline void reg_write(struct dpu_ctrl *dctrl, int offset, unsigned long val)
{
	writel(val, dctrl->base + offset);
}

#endif
