#ifndef __INGENIC_BAIC_H__
#define __INGENIC_BAIC_H__

#undef BIT
#define BIT(n)          (1 << (n))
#define BAIC_DAIFMT_NO_GRP  (0x0 << 0)
#define BAIC_DAIFMT_I2S_GRP (0x1 << 0)
#define BAIC_DAIFMT_DSP_GRP (0x2 << 0)
#define BAIC_DAIFMT_GRP_MSK (0x3 << 0)
#define BAIC_DAIFMT_MODEA   (BIT(0) << 2)   /*dsp grp*/
#define BAIC_DAIFMT_MODEB   (BIT(1) << 2)
#define BAIC_DAIFMT_PCM     (BIT(0) << 4)
#define BAIC_DAIFMT_DSP     (BIT(1) << 4)
#define BAIC_DAIFMT_TDM2    (BIT(2) << 4)
#define BAIC_DAIFMT_TDM1    (BIT(3) << 4)
#define BAIC_DAIFMT_I2S     (BIT(4) << 4)   /*i2s grp*/
#define BAIC_DAIFMT_RIGHTJ  (BIT(5) << 4)
#define BAIC_DAIFMT_LEFTJ   (BIT(6) << 4)

#define BAIC_NO_REPLAY      (BIT(7) << 4)   /*just support replay or record*/
#define BAIC_NO_RECORD      (BIT(8) << 4)

#define BAIC_PCM_MODE   (BAIC_DAIFMT_PCM|BAIC_DAIFMT_DSP_GRP|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_MODEB)
#define BAIC_DSP_MODE   (BAIC_DAIFMT_DSP|BAIC_DAIFMT_DSP_GRP|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_MODEB)
#define BAIC_TDM1_MODE  (BAIC_DAIFMT_TDM1|BAIC_DAIFMT_DSP_GRP|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_MODEB)
#define BAIC_TDM2_MODE  (BAIC_DAIFMT_TDM2|BAIC_DAIFMT_DSP_GRP|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_MODEB)
#define BAIC_I2S_MODE   (BAIC_DAIFMT_I2S|BAIC_DAIFMT_RIGHTJ|BAIC_DAIFMT_LEFTJ|BAIC_DAIFMT_I2S_GRP)

#define BAIC_2AND(x,y)      ((x) | (y))
#define BAIC_3AND(x,y,z)    ((x) | (y) | (z))
#define BAIC_4AND(x,y,z,m)  ((x) | (y) | (z) | (m))
#define BAIC_5AND(x,y,z,m,n)    ((x) | (y) | (z) | (m) | (n))

#endif /*__INGENIC_BAIC_H__*/
