#ifndef __LINUX_INGENIC_ADC_H__
#define __LINUX_INGENIC_ADC_H__

#include <linux/device.h>

/*
 * SAR A/D Controller(SADC) address definition
 */
#define	SADC_BASE		0xb0070000

/*************************************************************************
 * SADC (Smart A/D Controller)
 *************************************************************************/
#define BIT0            (1 << 0)
#define BIT1            (1 << 1)
#define BIT2            (1 << 2)
#define BIT3            (1 << 3)
#define BIT4            (1 << 4)
#define BIT5            (1 << 5)
#define BIT6            (1 << 6)
#define BIT7            (1 << 7)
#define BIT8            (1 << 8)
#define BIT9            (1 << 9)
#define BIT10           (1 << 10)
#define BIT11           (1 << 11)
#define BIT12 	        (1 << 12)
#define BIT13 	        (1 << 13)
#define BIT14 	        (1 << 14)
#define BIT15 	        (1 << 15)
#define BIT16 	        (1 << 16)
#define BIT17 	        (1 << 17)
#define BIT18 	        (1 << 18)
#define BIT19 	        (1 << 19)
#define BIT20 	        (1 << 20)
#define BIT21 	        (1 << 21)
#define BIT22 	        (1 << 22)
#define BIT23 	        (1 << 23)
#define BIT24 	        (1 << 24)
#define BIT25 	        (1 << 25)
#define BIT26 	        (1 << 26)
#define BIT27 	        (1 << 27)
#define BIT28 	        (1 << 28)
#define BIT29 	        (1 << 29)
#define BIT30 	        (1 << 30)
#define BIT31 	        (1 << 31)

#define SADC_ENA	(SADC_BASE + 0x00)  /* ADC Enable Register */
#define SADC_CFG	(SADC_BASE + 0x04)  /* ADC Configure Register */
#define SADC_CTRL	(SADC_BASE + 0x08)  /* ADC Control Register */
#define SADC_STATE	(SADC_BASE + 0x0C)  /* ADC Status Register*/
#define SADC_SAMETIME	(SADC_BASE + 0x10)  /* ADC Same Point Time Register */
#define SADC_WAITTIME	(SADC_BASE + 0x14)  /* ADC Wait Time Register */
#define SADC_TSDAT	(SADC_BASE + 0x18)  /* ADC Touch Screen Data Register */
#define SADC_BATDAT	(SADC_BASE + 0x1C)  /* ADC VBAT Data Register */
#define SADC_SADDAT	(SADC_BASE + 0x20)  /* ADC AUX Data Register */
#define SADC_FLT	(SADC_BASE + 0x24)  /* ADC Filter Register */

#define REG_SADC_ENA		REG8(SADC_ENA)
#define REG_SADC_CFG		REG32(SADC_CFG)
#define REG_SADC_CTRL		REG8(SADC_CTRL)
#define REG_SADC_STATE		REG8(SADC_STATE)
#define REG_SADC_SAMETIME	REG16(SADC_SAMETIME)
#define REG_SADC_WAITTIME	REG16(SADC_WAITTIME)
#define REG_SADC_TSDAT		REG32(SADC_TSDAT)
#define REG_SADC_BATDAT		REG16(SADC_BATDAT)
#define REG_SADC_SADDAT		REG16(SADC_SADDAT)
#define REG_SADC_ADCLK		REG32(SADC_ADCLK)
#define REG_SADC_FLT		REG16(SADC_FLT)
   #define SADC_FLT_ENA		(1 << 15)

/* ADENA: ADC Enable Register */
#define SADC_ENA_POWER		(1 << 7)  /* SADC Power control bit */
#define SADC_ENA_SLP_MD		(1 << 6)  /* SLEEP mode control */
#define SADC_ENA_TSEN		(1 << 2)  /* Touch Screen Enable */
#define SADC_ENA_PBATEN		(1 << 1)  /* PBAT Enable */
#define SADC_ENA_SADCINEN	(1 << 0)  /* AUX n Enable */

/* ADC Configure Register */
#define SADC_CFG_SPZZ           (1 << 31)
#define SADC_CFG_TS_DMA		(1 << 15)  /* Touch Screen DMA Enable */
#define SADC_CFG_XYZ_BIT	13  /* XYZ selection */
#define SADC_CFG_XYZ_MASK	(0x3 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XY		(0 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XYZ		(1 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XYZ1Z2	(2 << SADC_CFG_XYZ_BIT)
#define SADC_CFG_SNUM_BIT	10  /* Sample Number */
#define SADC_CFG_SNUM(x)	(((x) - 1) << SADC_CFG_SNUM_BIT)
#define SADC_CFG_SNUM_MASK	(0x7 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_1	(0x0 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_2	(0x1 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_3	(0x2 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_4	(0x3 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_5	(0x4 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_6	(0x5 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_8	(0x6 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_9	(0x7 << SADC_CFG_SNUM_BIT)
#define SADC_CFG_CMD_BIT	0  /* ADC Command */
#define SADC_CFG_CMD_MASK	(0x3 << SADC_CFG_CMD_BIT)
  #define SADC_CFG_CMD_AUX0	(0x0 << SADC_CFG_CMD_BIT) /* AUX voltage */
  #define SADC_CFG_CMD_AUX1	(0x1 << SADC_CFG_CMD_BIT) /* AUX1 voltage */
  #define SADC_CFG_CMD_AUX2	(0x2 << SADC_CFG_CMD_BIT) /* AUX2 voltage */
  #define SADC_CFG_CMD_RESERVED	(0x3 << SADC_CFG_CMD_BIT) /* Reserved */

/* ADCCTRL: ADC Control Register */
#define SADC_CTRL_SLPENDM	(1 << 5)  /* Sleep Interrupt Mask */
#define SADC_CTRL_PENDM		(1 << 4)  /* Pen Down Interrupt Mask */
#define SADC_CTRL_PENUM		(1 << 3)  /* Pen Up Interrupt Mask */
#define SADC_CTRL_TSRDYM	(1 << 2)  /* Touch Screen Data Ready Interrupt Mask */
#define SADC_CTRL_PBATRDYM	(1 << 1)  /* VBAT Data Ready Interrupt Mask */
#define SADC_CTRL_SRDYM		(1 << 0)  /* AUX Data Ready Interrupt Mask */

/* ADSTATE: ADC Status Register */
#define SADC_STATE_SLP_RDY	(1 << 7)  /* Sleep state bit */
#define SADC_STATE_SLEEPND	(1 << 5)  /* Pen Down Interrupt Flag */
#define SADC_STATE_PEND		(1 << 4)  /* Pen Down Interrupt Flag */
#define SADC_STATE_PENU		(1 << 3)  /* Pen Up Interrupt Flag */
#define SADC_STATE_TSRDY	(1 << 2)  /* Touch Screen Data Ready Interrupt Flag */
#define SADC_STATE_PBATRDY		(1 << 1)  /* VBAT Data Ready Interrupt Flag */
#define SADC_STATE_SRDY		(1 << 0)  /* AUX Data Ready Interrupt Flag */

/* ADTCH: ADC Touch Screen Data Register */
#define SADC_TSDAT_TYPE1	(1 << 31)
#define SADC_TSDAT_DATA1_BIT	16
#define SADC_TSDAT_DATA1_MASK	(0xfff << SADC_TSDAT_DATA1_BIT)
#define SADC_TSDAT_TYPE0	(1 << 15)
#define SADC_TSDAT_DATA0_BIT	0
#define SADC_TSDAT_DATA0_MASK	(0xfff << SADC_TSDAT_DATA0_BIT)

/* ADCLK: ADC Clock Divide Register */
#define SADC_ADCLK_CLKDIV_MS	16
#define SADC_ADCLK_CLKDIV_MS_MASK	(0xffff << SADC_ADCLK_CLKDIV_MS)
#define SADC_ADCLK_CLKDIV_US	8
#define SADC_ADCLK_CLKDIV_US_MASK	(0xff << SADC_ADCLK_CLKDIV_US)
#define SADC_ADCLK_CLKDIV_BIT		0
#define SADC_ADCLK_CLKDIV_MASK		(0xff << SADC_ADCLK_CLKDIV_BIT)

/*
 * SADC registers offset definition
 */
#define SADC_ADENA_OFFSET	(0x00)	/* rw,  8, 0x00 */
#define SADC_ADCFG_OFFSET       (0x04)  /* rw, 32, 0x0002000c */
#define SADC_ADCTRL_OFFSET      (0x08)  /* rw,  8, 0x3f */
#define SADC_ADSTATE_OFFSET     (0x0c)  /* rw,  8, 0x00 */
#define SADC_ADSAME_OFFSET    	(0x10)  /* rw, 16, 0x0000 */
#define SADC_ADWAIT_OFFSET    	(0x14)  /* rw, 16, 0x0000 */
#define SADC_ADTCH_OFFSET       (0x18)  /* rw, 32, 0x00000000 */
#define SADC_ADVDAT_OFFSET      (0x1c)  /* rw, 16, 0x0000 */
#define SADC_ADADAT_OFFSET      (0x20)  /* rw, 16, 0x0000 */
#define SADC_ADCMD_OFFSET       (0x24)  /* rw, 32, 0x00000000 */
#define SADC_ADCLK_OFFSET       (0x28)  /* rw, 32, 0x00000000 */

/*
 * SADC registers address definition
 */
#define SADC_ADENA		(SADC_BASE + SADC_ADENA_OFFSET)	 /* ADC Enable Register */
#define SADC_ADCFG		(SADC_BASE + SADC_ADCFG_OFFSET)	 /* ADC Configure Register */
#define SADC_ADCTRL		(SADC_BASE + SADC_ADCTRL_OFFSET) /* ADC Control Register */
#define SADC_ADSTATE		(SADC_BASE + SADC_ADSTATE_OFFSET)/* ADC Status Register*/
#define SADC_ADSAME		(SADC_BASE + SADC_ADSAME_OFFSET) /* ADC Same Point Time Register */
#define SADC_ADWAIT		(SADC_BASE + SADC_ADWAIT_OFFSET) /* ADC Wait Time Register */
#define SADC_ADTCH		(SADC_BASE + SADC_ADTCH_OFFSET)  /* ADC Touch Screen Data Register */
#define SADC_ADVDAT		(SADC_BASE + SADC_ADVDAT_OFFSET) /* ADC VBAT Data Register */
#define SADC_ADADAT		(SADC_BASE + SADC_ADADAT_OFFSET) /* ADC AUX Data Register */
#define SADC_ADCMD		(SADC_BASE + SADC_ADCMD_OFFSET)  /* ADC COMMAND Register */
#define SADC_ADCLK		(SADC_BASE + SADC_ADCLK_OFFSET)  /* ADC Clock Divide Register */

/*
 * SADC registers common define
 */

/* ADC Enable Register (ADENA) */
#define ADENA_POWER		BIT7
#define ADENA_SLP_MD		BIT6
#define ADENA_TCHEN		BIT2
#define ADENA_PENDEN		BIT3
#define ADENA_VBATEN		BIT1
#define ADENA_AUXEN		BIT0

/* ADC Configure Register (ADCFG) */
#define ADCFG_SPZZ           	BIT31

#define ADCFG_CMD_SEL		BIT22

#define ADCFG_DMA_EN		BIT15

#define ADCFG_XYZ_LSB		13
#define ADCFG_XYZ_MASK		BITS_H2L(14, ADCFG_XYZ_LSB)
#define ADCFG_XYZ_XYS		(0x0 << ADCFG_XYZ_LSB)
#define ADCFG_XYZ_XYD		(0x1 << ADCFG_XYZ_LSB)
#define ADCFG_XYZ_XYZ1Z2	(0x2 << ADCFG_XYZ_LSB)
#define ADCFG_XYZ_XYZ1Z2X2Y2	(0x3 << ADCFG_XYZ_LSB)

#define ADCFG_SNUM_LSB		10
#define ADCFG_SNUM_MASK		BITS_H2L(12, ADCFG_SNUM_LSB)
#define ADCFG_SNUM(n)          (((n) <= 6 ? ((n)-1) : ((n)-2)) << ADCFG_SNUM_LSB)

#define ADCFG_CMD_LSB		0
#define ADCFG_CMD_MASK		BITS_H2L(1, ADCFG_CMD_LSB)
#define ADCFG_CMD_AUX(n)	((n) << ADCFG_CMD_LSB)

/* ADC Control Register (ADCCTRL) */
#define ADCTRL_SLPENDM		BIT5
#define ADCTRL_PENDM		BIT4
#define ADCTRL_PENUM		BIT3
#define ADCTRL_DTCHM		BIT2
#define ADCTRL_VRDYM		BIT1
#define ADCTRL_ARDYM		BIT0
#define ADCTRL_MASK_ALL         (ADCTRL_SLPENDM | ADCTRL_PENDM | ADCTRL_PENUM \
                                | ADCTRL_DTCHM | ADCTRL_VRDYM | ADCTRL_ARDYM)

/*  ADC Status Register  (ADSTATE) */
#define ADSTATE_SLP_RDY		BIT7
#define ADSTATE_SLPEND		BIT5
#define ADSTATE_PEND		BIT4
#define ADSTATE_PENU		BIT3
#define ADSTATE_DTCH		BIT2
#define ADSTATE_VRDY		BIT1
#define ADSTATE_ARDY		BIT0

/* ADC Same Point Time Register (ADSAME) */
#define ADSAME_SCNT_LSB		0
#define ADSAME_SCNT_MASK	BITS_H2L(15, ADSAME_SCNT_LSB)

/* ADC Wait Pen Down Time Register (ADWAIT) */
#define ADWAIT_WCNT_LSB		0
#define ADWAIT_WCNT_MASK	BITS_H2L(15, ADWAIT_WCNT_LSB)

/* ADC Touch Screen Data Register (ADTCH) */
#define ADTCH_TYPE1		BIT31
#define ADTCH_TYPE0		BIT15

#define ADTCH_DATA1_LSB		16
#define ADTCH_DATA1_MASK	BITS_H2L(27, ADTCH_DATA1_LSB)

#define ADTCH_DATA0_LSB		0
#define ADTCH_DATA0_MASK	BITS_H2L(11, ADTCH_DATA0_LSB)

/* ADC VBAT Date Register (ADVDAT) */
#define ADVDAT_VDATA_LSB	0
#define ADVDAT_VDATA_MASK	BITS_H2L(11, ADVDAT_VDATA_LSB)

/* ADC AUX Data Register (ADADAT) */
#define ADADAT_ADATA_LSB	0
#define ADADAT_ADATA_MASK	BITS_H2L(11, ADADAT_ADATA_LSB)

/*  ADC Clock Divide Register (ADCLK) */
#define ADCLK_CLKDIV_MS_LSB	16
#define ADCLK_CLKDIV_MS_MASK	BITS_H2L(31, ADCLK_CLKDIV_MS_LSB)

#define ADCLK_CLKDIV_US_LSB	8
#define ADCLK_CLKDIV_US_MASK	BITS_H2L(15, ADCLK_CLKDIV_US_LSB)

#define ADCLK_CLKDIV_LSB	0
#define ADCLK_CLKDIV_MASK	BITS_H2L(7, ADCLK_CLKDIV_LSB)

/* ADC Filter Register (ADFLT) */
#define ADFLT_FLT_EN		BIT15

#define ADFLT_FLT_D_LSB		0
#define ADFLT_FLT_D_MASK	BITS_H2L(11, ADFLT_FLT_D_LSB)

/* ADC Command Register (ADCMD) */
#define ADCMD_PIL		BIT31
#define ADCMD_RPU(n)		((n) << 26)
#define ADCMD_XPSUP		BIT25
#define ADCMD_YPSUP		BIT23
#define	ADCMD_XNGRU		BIT21
#define	ADCMD_YNGRU		BIT20
#define	ADCMD_VREFNXN		BIT18
#define	ADCMD_VREFNYN		BIT16
#define ADCMD_VREFPXP		BIT12
#define ADCMD_VREFPYP		BIT11
#define ADCMD_XPADC		BIT10
#define ADCMD_YPADC		BIT8

struct ingenic_ts_info{
	unsigned int x_max;
	unsigned int x_min;
	unsigned int y_max;
	unsigned int y_min;
	unsigned int z_max;
	unsigned int z_min;
	unsigned short x_resolution;
	unsigned short y_resolution;
	unsigned short	y_r_plate;
	unsigned short	x_r_plate;
	unsigned short  pressure_max;
	unsigned int	use_5_wire;
	unsigned int	support_keypad;
	unsigned int	support_mt_touch;
	unsigned int	support_sleepmode;
	void		*private_data;
};

struct ingenic_adc_priv {
	unsigned int aux_channels;
};

struct ingenic_adc {
	struct resource *mem;
	void __iomem *base;

	int irq;
	int irq_base;
	struct irq_domain *irq_domain;
	struct device_node *np;
	struct resource *aux_res_ptr;
	struct mfd_cell *adc_cells;

	struct clk *clk;
	atomic_t clk_ref;

	spinlock_t lock;
	const struct ingenic_adc_priv *priv;/*soc data*/
	int	(*enable)(struct platform_device *dev);
	int     (*disable)(struct platform_device *dev);
};

int adc_write_reg(struct device *dev,uint8_t addr_offset,uint32_t mask,uint32_t val);
uint32_t adc_read_reg(struct device *dev,uint8_t addr_offset);

#endif /*ingenic_adc.h*/
