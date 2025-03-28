#ifndef __INGENIC_ADC_H
#define __INGENIC_ADC_H

#define ADC_ADDR_BASE           (0x13650000 | 0xa0000000)

#define ADC_SR                  (0x0000)
#define ADC_IE                  (0x0004)
#define ADC_IE_FORRISCV         (0x0008)
#define ADC_IR                  (0x000C)
#define ADC_IR_FORRISCV         (0x0010)
#define ADC_CR                  (0x0014)
#define ADC_CFR                 (0x0018)
#define ADC_CLK0                (0x001C)
#define ADC_CLK1                (0x0020)
#define ADC_CLK2                (0x0024)
#define ADC_EXT_GPIO_CR         (0x0028)
#define ADC_AWD_CRN             (0x002c)
#define ADC_AWD_CR(n)           (ADC_AWD_CRN + (4 * n))
#define ADC_AWD_SR              (0x006c)
#define ADC_AWD_IM              (0x0070)
#define ADC_AWD_IM_FORRISCV     (0x0074)
#define ADC_AWD_IF              (0x0078)
#define ADC_AWD_IF_FORRISCV     (0x007c)
#define ADC_DBG_CR              (0x0080)
#define ADC_DBG_FSM             (0x0084)
#define ADC_SEQ0_CR             (0x0100)
#define ADC_SEQ0_CNR0           (0x0104)
#define ADC_SEQ0_DCR            (0x0108)
#define ADC_SEQ0_DR0            (0x010c)
#define ADC_SEQ0_DR1            (0x0110)
#define ADC_SEQ1_CR             (0x0200)
#define ADC_SEQ1_CNR0           (0x0204)
#define ADC_SEQ1_CNR1           (0x0208)
#define ADC_SEQ1_DCR            (0x020C)
#define ADC_SEQ1_DR             (0x0210)
#define ADC_SEQ1_DLYN           (0x0214)
#define ADC_SEQ1_DLY(n)         (ADC_SEQ1_DLYN + (4 * n))
#define ADC_SEQ1_CONT_CRN       (0x0234)
#define ADC_SEQ1_CONT_CR(n)     (ADC_SEQ1_CONT_CRN + (4 * n))
#define ADC_SEQ1_RCNT           (0x0254)
#define ADC_SEQ1_DMA_RCNT       (0x0258)
#define ADC_SEQ2_CR             (0x0300)
#define ADC_SEQ2_CNR0           (0x0304)
#define ADC_SEQ2_DCR            (0x030c)
#define ADC_SEQ2_DR             (0x0310)
#define ADC_SEQ2_DLYN           (0x0314)
#define ADC_SEQ2_DLY(n)         (ADC_SEQ2_DLYN + (4 * n))
#define ADC_SEQ2_RCNT           (0x0354)
#define ADC_SEQ2_DMA_RCNT       (0x0358)

/*SADC_SR*/
#define SR_AWD                       BIT(31)
#define SR_PHY_EOC_ERR               BIT(30)
#define SR_SEQ2_DATA_OVR             BIT(23)
#define SR_SEQ2_DMA_FIN              BIT(22)
#define SR_SEQ2_DR                   BIT(21)
#define SR_SEQ2_EVT_OVR              BIT(20)
#define SR_SEQ2_FSM_SHIFT            16
#define SR_SEQ2_FSM_MASK         (0xf<<SR_SEQ2_FSM_SHIFT)
#define SR_SEQ1_DATA_OVR             BIT(15)
#define SR_SEQ1_DMA_FIN              BIT(14)
#define SR_SEQ1_DR                   BIT(13)
#define SR_SEQ1_EVT_OVR              BIT(12)
#define SR_SEQ1_FSM_SHIFT            8
#define SR_SEQ1_FSM_MASK             (0xf << SR_SEQ1_FSM_SHIFT)
#define SR_SEQ0_DR                   BIT(5)
#define SR_SEQ0_EVT_OVR              BIT(4)
#define SR_SEQ0_FSM_SHIFT            0
#define SR_SEQ0_FSM_MASK             (0xf << SR_SEQ0_FSM_SHIFT)

/*ADC_IE*/
#define IE_PHY_EOC_ERR               BIT(30)
#define IE_SEQ2_DATA_OVR             BIT(21)
#define IE_SEQ2_DMA_FIN              BIT(20)
#define IE_SEQ2_DR                   BIT(19)
#define IE_SEQ2_EVT_OVR              BIT(18)
#define IE_SEQ2_EOQ                  BIT(17)
#define IE_SEQ2_SOQ              BIT(16)
#define IE_SEQ1_CONT_EOG             BIT(15)
#define IE_SEQ1_CONT_SOG             BIT(14)
#define IE_SEQ1_DATA_OVR             BIT(13)
#define IE_SEQ1_DMA_FIN              BIT(12)
#define IE_SEQ1_DR                   BIT(11)
#define IE_SEQ1_EVT_OVR              BIT(10)
#define IE_SEQ1_EOQ                  BIT(9)
#define IE_SEQ1_SOQ                  BIT(8)
#define IE_SEQ0_DR                   BIT(3)
#define IE_SEQ0_EVT_OVR              BIT(2)
#define IE_SEQ0_EOQ                  BIT(1)
#define IE_SEQ0_SOQ                  BIT(0)

/*ADC_IR*/
#define IR_AWD                       BIT(31)
#define IR_PHY_EOC_ERR               BIT(30)
#define IR_SEQ2_DATA_OVR             BIT(21)
#define IR_SEQ2_DMA_FIN              BIT(20)
#define IR_SEQ2_DR                   BIT(19)
#define IR_SEQ2_EVT_OVR              BIT(18)
#define IR_SEQ2_EOQ                  BIT(17)
#define IR_SEQ2_SOQ              BIT(16)
#define IR_SEQ1_CONT_EOG             BIT(15)
#define IR_SEQ1_CONT_SOG             BIT(14)
#define IR_SEQ1_DATA_OVR             BIT(13)
#define IR_SEQ1_DMA_FIN              BIT(12)
#define IR_SEQ1_DR                   BIT(11)
#define IR_SEQ1_EVT_OVR              BIT(10)
#define IR_SEQ1_EOQ                  BIT(9)
#define IR_SEQ1_SOQ                  BIT(8)
#define IR_SEQ0_DR                   BIT(3)
#define IR_SEQ0_EVT_OVR              BIT(2)
#define IR_SEQ0_EOQ                  BIT(1)
#define IR_SEQ0_SOQ                  BIT(0)

/*ADC_CR*/
#define SEQ2_START                   BIT(10)
#define SEQ1_START                   BIT(9)
#define SEQ0_START                   BIT(8)
#define SEQ2_RESET                   BIT(2)
#define SEQ1_RESET                   BIT(1)
#define SEQ0_RESET                   BIT(0)

/*ADC_CFR*/
#define PHY_SEL_EOC                  BIT(27)
#define PHY_SEL_EN                   BIT(26)
#define PHY_RESET                    BIT(25)
#define PHY_PD                       BIT(24)
#define BREAK_MD                     BIT(16)
#define SEQ2_PRI_SHIFT               4
#define SEQ2_PRI_MASK            (0x3 << SEQ2_PRI_SHIFT)
#define SEQ1_PRI_SHIFT               2
#define SEQ1_PRI_MASK                (0x3 << SEQ1_PRI_SHIFT)
#define SEQ0_PRI_SHIFT               0
#define SEQ0_PRI_MASK                (0x3 << SEQ0_PRI_SHIFT)

/*ADC_CLKR0*/
#define ADCCLK_DIV_SHIFT             24
#define ADCCLK_DIV_MASK          (0xff << ADCCLK_DIV_SHIFT)
#define SEQ1_CONTCLK_DIV_SHIFT       0
#define SEQ1_CONTCLK_DIV_MASK        (0xffffff << SEQ1_CONTCLK_DIV_SHIFT)

/*ADC_CLKR1*/
#define SEQ1_DLYCLK_DIV_SHIFT        0
#define SEQ1_DLYCLK_DIV_MASK         (0xffffff << SEQ1_DLYCLK_DIV_SHIFT)

/*ADC_CLKR2*/
#define SEQ2_DLYCLK_DIV_SHIFT        0
#define SEQ2_DLYCLK_DIV_MASK         (0xffffff << SEQ2_DLYCLK_DIV_SHIFT)

/*ADC_AWD_CR*/
#define HTR_EN                       BIT(31)
#define HTR_SHIFT                    16
#define HTR_MASK                     (0xfff << HTR_SHIFT)
#define LTR_EN                       BIT(15)
#define LTR_SHIFT                    0
#define LTR_MASK             (0xfff << LTR_SHIFT)

/*ADC_SEQ0_CR*/
#define SEQ0_CR_LEN_SHIFT            12
#define SEQ0_CR_LEN_MASK             (0x3 << SEQ0_CR_LEN_SHIFT)
#define SEQ0_CR_CH_NUM_EN            BIT(0)

/*ADC_SEQ0_DCR*/
#define SEQ0_DCR_DRT_CUS3            BIT(19)
#define SEQ0_DCR_DRT_CUS2            BIT(18)
#define SEQ0_DCR_DRT_CUS1            BIT(17)
#define SEQ0_DCR_DRT_CUS0            BIT(16)
#define SEQ0_DCR_DRT                 BIT(13)
#define SEQ0_DCR_CNT_SHIFT           0
#define SEQ0_DCR_CNT_MASK            (0x7 << SEQ0_DCR_CNT_SHIFT)

/*ADC_SEQ0_DR0*/
#define SEQ0_DR0_CH_NUM1_SHIFT       28
#define SEQ0_DR0_CH_NUM1_MASK        (0xf << SEQ0_DR0_CH_NUM1_SHIFT)
#define SEQ0_DR0_DATA1_SHIFT         16
#define SEQ0_DR0_DATA1_MASK          (0xfff << SEQ0_DR0_DATA1_SHIFT)
#define SEQ0_DR0_CH_NUM0_SHIFT       12
#define SEQ0_DR0_CH_NUM0_MASK        (0xf << SEQ0_DR0_CH_NUM0_SHIFT)
#define SEQ0_DR0_DATA0_SHIFT         0
#define SEQ0_DR0_DATA0_MASK          (0xfff << SEQ0_DR0_DATA0_SHIFT)

/*ADC_SEQ1_CR*/
#define STORAGE15                    BIT(31)
#define STORAGE14                    BIT(30)
#define STORAGE13                    BIT(29)
#define STORAGE12                    BIT(28)
#define STORAGE11                    BIT(27)
#define STORAGE10                    BIT(26)
#define STORAGE9                     BIT(25)
#define STORAGE8                     BIT(24)
#define STORAGE7                     BIT(23)
#define STORAGE6                     BIT(22)
#define STORAGE5                     BIT(21)
#define STORAGE4                     BIT(20)
#define STORAGE3                     BIT(19)
#define STORAGE2                     BIT(18)
#define STORAGE1                     BIT(17)
#define STORAGE0                     BIT(16)
#define SEQ1_CR_LEN_SHIFT            12
#define SEQ1_CR_LEN_MASK             (0xf << SEQ1_CR_LEN_SHIFT)
#define EXT_TCU_CH_SEL_SHIFT         8
#define EXT_TCU_CH_SEL_MASK          (0xf << EXT_TCU_CH_SEL_SHIFT)
#define EXTSEL_SHIFT                 6
#define EXTSEL_MASK                  (0x3 << EXTSEL_SHIFT)
#define CONT_GLEN_SHIFT              2
#define CONT_GLEN_MASK               (0x7 << CONT_GLEN_SHIFT)
#define CONT_EN                      BIT(1)
#define CONT_EN_MASK                 BIT(1)
#define SEQ1_CR_CH_NUM_EN            BIT(0)

/*ADC_SEQ1_CNR0*/
#define CNR_CH_NUM7_SHIFT       28
#define CNR_CH_NUM7_MASK        (0xf << CNR_CH_NUM7_SHIFT)
#define CNR_CH_NUM6_SHIFT       24
#define CNR_CH_NUM6_MASK        (0xf << CNR_CH_NUM6_SHIFT)
#define CNR_CH_NUM5_SHIFT       20
#define CNR_CH_NUM5_MASK        (0xf << CNR_CH_NUM5_SHIFT)
#define CNR_CH_NUM4_SHIFT           16
#define CNR_CH_NUM4_MASK            (0xf << CNR_CH_NUM4_SHIFT)
#define CNR_CH_NUM3_SHIFT       12
#define CNR_CH_NUM3_MASK            (0xf << CNR_CH_NUM3_SHIFT)
#define CNR_CH_NUM2_SHIFT           8
#define CNR_CH_NUM2_MASK            (0xf << CNR_CH_NUM2_SHIFT)
#define CNR_CH_NUM1_SHIFT           4
#define CNR_CH_NUM1_MASK            (0xf << CNR_CH_NUM1_SHIFT)
#define CNR_CH_NUM0_SHIFT           0
#define CNR_CH_NUM0_MASK            (0xf << CNR_CH_NUM0_SHIFT)

/*ADC_SEQ1_CNR1*/
#define CNR_CH_NUM15_SHIFT      28
#define CNR_CH_NUM15_MASK           (0xf << CNR_CH_NUM15_SHIFT)
#define CNR_CH_NUM14_SHIFT          24
#define CNR_CH_NUM14_MASK           (0xf << CNR_CH_NUM14_SHIFT)
#define CNR_CH_NUM13_SHIFT          20
#define CNR_CH_NUM13_MASK           (0xf << CNR_CH_NUM13_SHIFT)
#define CNR_CH_NUM12_SHIFT          16
#define CNR_CH_NUM12_MASK           (0xf << CNR_CH_NUM12_SHIFT)
#define CNR_CH_NUM11_SHIFT      12
#define CNR_CH_NUM11_MASK           (0xf << CNR_CH_NUM11_SHIFT)
#define CNR_CH_NUM10_SHIFT          8
#define CNR_CH_NUM10_MASK           (0xf << CNR_CH_NUM10_SHIFT)
#define CNR_CH_NUM9_SHIFT           4
#define CNR_CH_NUM9_MASK            (0xf << CNR_CH_NUM9_SHIFT)
#define CNR_CH_NUM8_SHIFT           0
#define CNR_CH_NUM8_MASK            (0xf << CNR_CH_NUM8_SHIFT)

/*ADC_SEQ1_DCR*/
#define DCR_DRT_CUS15             BIT(31)
#define DCR_DRT_CUS14                 BIT(30)
#define DCR_DRT_CUS13                 BIT(29)
#define DCR_DRT_CUS12                 BIT(28)
#define DCR_DRT_CUS11                 BIT(27)
#define DCR_DRT_CUS10                 BIT(26)
#define DCR_DRT_CUS9                  BIT(25)
#define DCR_DRT_CUS8                  BIT(24)
#define DCR_DRT_CUS7                  BIT(23)
#define DCR_DRT_CUS6                  BIT(22)
#define DCR_DRT_CUS5                  BIT(21)
#define DCR_DRT_CUS4                  BIT(20)
#define DCR_DRT_CUS3                  BIT(19)
#define DCR_DRT_CUS2                  BIT(18)
#define DCR_DRT_CUS1                  BIT(17)
#define DCR_DRT_CUS0                  BIT(16)
#define DCR_DRT_SHIFT                 13
#define DCR_DRT_MASK                  (0x7 << DCR_DRT_SHIFT)
#define DMA_MD                        BIT(8)
#define SEQ1_DCR_CNT_SHIFT        0
#define SEQ1_DCR_CNT_MASK             (0x3f << SEQ1_DCR_CNT_SHIFT)

/*ADC_SEQ1_DR*/
#define SEQ1_DR_CH_NUM_SHIFT         12
#define SEQ1_DR_CH_NUM_MASK          (0xf << SEQ1_DR_CH_NUM_SHIFT)
#define SEQ1_DR_DATA_SHIFT           0
#define SEQ1_DR_DATA_MASK            (0xfff << SEQ1_DR_DATA_SHIFT)

/*ADC_SEQ1_DLYn*/
#define SEQ1_DLY_HIGH_SHIFT          16
#define SEQ1_DLY_HIGH_MASK           (0xffff << SEQ1_DLY_HIGH_SHIFT)
#define SEQ1_DLY_LOW_SHIFT       0
#define SEQ1_DLY_LOW_MASK            (0xffff << SEQ1_DLY_LOW_SHIFT)

/*ADC_SEQ1_CONT_CRn*/
#define CONT_CNT_SHIFT               16
#define CONT_CNT_MASK                (0xffff << CONT_CNT_SHIFT)
#define SEQ1_CONT_LEN_SHIFT          0
#define SEQ1_CONT_LEN_MASK           (0xf << SEQ1_CONT_LEN_SHIFT)

/*ADC_SEQ1_RCNT*/
#define MAX_SHIFT                    16
#define MAX_MASK                     (0xffff << MAX_SHIFT)

/*ADC_SEQ2_CR*/
#define SEQ2_CR_LEN_SHIFT            12
#define SEQ2_CR_LEN_MASK             (0xf << SEQ2_CR_LEN_SHIFT)
#define SEQ2_CR_CH_NUM_EN            BIT(0)

/*ADC_SEQ2_DCR*/
#define SEQ2_DCR_CNT_SHIFT        0
#define SEQ2_DCR_CNT_MASK             (0x1f << SEQ2_DCR_CNT_SHIFT)

/*ADC_SEQ2_DR*/
#define SEQ2_DR_CH_NUM_SHIFT         12
#define SEQ2_DR_CH_NUM_MASK          (0xf << SEQ2_DR_CH_NUM_SHIFT)
#define SEQ2_DR_DATA_SHIFT           0
#define SEQ2_DR_DATA_MASK            (0x3f << SEQ2_DR_DATA_SHIFT)

/*ADC_SEQ2_DLYn*/
#define SEQ2_DLY_04_SHIFT            0
#define SEQ2_DLY_04_MASK             (0xff << SEQ2_DLY_04_SHIFT)
#define SEQ2_DLY_15_SHIFT            8
#define SEQ2_DLY_15_MASK             (0xff << SEQ2_DLY_15_SHIFT)
#define SEQ2_DLY_26_SHIFT            16
#define SEQ2_DLY_26_MASK             (0xff << SEQ2_DLY_26_SHIFT)
#define SEQ2_DLY_37_SHIFT            24
#define SEQ2_DLY_37_MASK             (0xff << SEQ2_DLY_37_SHIFT)

#define ADC_REG_MIN               (ADC_ADDR_BASE + ADC_SR)
#define ADC_REG_MAX               (ADC_ADDR_BASE + ADC_SEQ2_DMA_RCNT)

#define DMA_BUFFER_SIZE           PAGE_SIZE

struct ingenic_adc {
	void __iomem            *base;
	void __iomem        *base_phy;
	struct device       *dev ;
	struct resource     *mem;
	struct completion       completion;
	struct clk              *clk;
	struct clk              *gate_clk;
	int                     irq;
	spinlock_t              lock;   /* interrupt lock */
	struct mutex        mutex;
	u32                     res;
	struct dma_chan         *seq1rxchan;
	u32                      *seq1rx_buf;
	dma_addr_t              seq1rx_dma_buf;
	struct scatterlist      *seq1_sg_rx;
	u8                      use_dma;
	int         record;
	u32         seq0_len;
	u32         seq1_len;
	u32         seq2_len;
	u32         seq0_buf[4];
	u32         seq1_buf[32];
	u32         seq2_buf[8];
	int conversion_en;
	int seq1_cont_interval;
};

#endif

enum adc_scan {
	SEQ0,
	SEQ1,
	SEQ2,
};
