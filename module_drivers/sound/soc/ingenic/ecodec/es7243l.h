/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es7243l.h -- es7243l ALSA SoC adc driver
 *
 * Author:      David Yang, <yangxiaohua@everest-semi.com>
 * Copyright:   (C) 2019 Everest Semiconductor Co Ltd.,
 *
 * Based on sound/soc/codecs/es7243.c by DavidYang
 *
 * Notes:
 *  es7243l.h is a user interface file which is convenient for setting
 *  digital format, clock ratio, etc.
 *
 */

#define ES7243l_RESET                   0x00    /* RESET circuit    */
#define ES7243l_CLK1                    0x01    /* CLOCK MANAGER    */
#define ES7243l_CLK2                    0x02    /* CLOCK MANAGER    */
#define ES7243l_ADC_OSR                 0x03    /* ADC Over Sample Rate control */
#define ES7243l_PREDIV                  0x04    /* MCLK Pre-multiplication and divider  */
#define ES7243l_CLK_DIV                 0x05    /* CF&DSP clock divider */
#define ES7243l_BCLK_DIV                0x06    /* BCLK divider at master mode  */
#define ES7243l_CLK_TRI                 0x07    /* Tri-state control and Master LRCK divider */
#define ES7243l_LRCK_DIV                0x08    /* Master LRCK divider  */
#define ES7243l_S1_SEL                  0x09    /* state machine 1 select   */
#define ES7243l_S3_SEL                  0x0A    /* state machine 3 select   */
#define ES7243l_SDP_FORMAT              0x0B    /* SDP formate and word length  */
#define ES7243l_TDM                     0x0C    /* TDM_MODE TDM_FLAG    */
#define ES7243l_ADCCTL1                 0x0D    /* ADC gain scale up ADC data mux   */
#define ES7243l_ADC_VOL                 0x0E    /* ADC volume control   */
#define ES7243l_ADCCTL2                 0x0F    /* ADC VC ramp rate */
#define ES7243l_ADCCTL3                 0x10
#define ES7243l_ADCCTL4                 0x11
#define ES7243l_ADCCTL5                 0x12
#define ES7243l_ADCCTL6                 0x13    /* ALC rate selection, ALC target level */
#define ES7243l_ADC_HPF1                0x14    /* ADCHPF stage1 coeff  */
#define ES7243l_ADC_HPF2                0x15    /* ADCHPF stage2 coeff  */
#define ES7243l_ANALOG_PDN              0x16    /* powerdown analog & PGA   */
#define ES7243l_VMIDSEL                 0x17    /* select VMID  */
#define ES7243l_ADC_BIAS_0x18           0x18    /* ADC BIAS_SW  */
#define ES7243l_PGA_BIAS                0x19    /* PGA BIAS_SW  */
#define ES7243l_ADC_BIAS_0x1A           0x1A    /* ADC I1BIAS_SW    */
#define ES7243l_ADC_MICBIAS             0x1B    /* ADC MBIAS_SW */
#define ES7243l_ADC_VRPBIAS             0x1C    /* ADC VRPBIAS_SW   */
#define ES7243l_ADC_LP                  0x1D    /* ADC low power select */
#define ES7243l_ADC_PGA_LP              0x1E    /* PGA low power select */
#define ES7243l_ADC_VMID                0x1F    /* VMIDLVL select   */
#define ES7243l_PGA1                    0x20    /* PGA1 gain, input */
#define ES7243l_PGA2                    0x21    /* PGA2 gain, input */
#define ES7243l_TESTMOD_0xF7            0xF7 /* */
#define ES7243l_TESTMOD_0xF8            0xF8 /* */
#define ES7243l_DLL_PWN                 0xF9 /* DLL_PWD */
#define ES7243l_I2C_CONFIG              0xFC /* I2C signals retime */
#define ES7243l_FLAG                    0xFA /* CSM & ADC automute flag */
#define ES7243l_CHIPID1                 0xFD /* chip ID1 */
#define ES7243l_CHIPID2                 0xFE /* chip ID2 */
#define ES7243l_CHIP_VER                0xFF /* chip versoin */

#define ENABLE     1
#define DISABLE    0
/*
 * Here is the definition of ES7243l ADC I2S DAI Format
 *
 * ES7243l_WORK_MODE is used to select I2S format, and user must update it for their system
 * In ES7243l codec driver (es7243l.c), ES7243l_WORK_MODE will be used for digital format setting.
 *
 * In normal mode, ES7243l supports four digital formats including I2S, LJ, DSP-A and DSP-B, with
 * bit resolution from 16bits to 32bits.
 * In NFS mode, ES7243l only supports NFS_I2S/NFS_DSPA mode, doesn't support DSPB or LJ mode.
 * DAI format also modified in es7243l_set_dai_fmt
 */
#define ES7243l_NORMAL_I2S  0
#define ES7243l_NORMAL_LJ  1
#define ES7243l_NORMAL_DSPA  2
#define ES7243l_NORMAL_DSPB  3
#define ES7243l_TDM_A  4
#define ES7243l_NFS  5
#define ES7243l_WORK_MODE ES7243l_NORMAL_I2S
/*
 * Here is the definition of MCLK/LRCK rato.
 * ES7243l will have different register configuration for each MCLK/LRCK ratio.
 * Please check the MCLK/LRCK ratio in your system before you update ES7243l_MCLK_LRCK_RATIO.
 * ES7243l codec driver will configure the clock registers according to the value of ES7243l_MCLK_LRCK_RATO.
 */
#define RATIO_3072 3072
#define RATIO_2048 2048
#define RATIO_1536 1536
#define RATIO_1024 1024
#define RATIO_768  768
#define RATIO_512  512
#define RATIO_384  384
#define RATIO_256  256
#define RATIO_192  192
#define RATIO_128  128
#define RATIO_64  64
#define ES7243l_MCLK_LRCK_RATIO   64
/*
 * To select the total analog input channel for microphone array
 */
#define AIN_2_CH   2
#define AIN_4_CH   4
#define AIN_6_CH   6
#define AIN_8_CH   8
#define AIN_10_CH  10
#define AIN_12_CH  12
#define AIN_14_CH  14
#define AIN_16_CH  16
#define ES7243l_CHANNELS_MAX    AIN_2_CH
/*
 * to select the clock soure for internal MCLK clock
 */
#define FROM_MCLK_PIN   0
#define FROM_INTERNAL_BCLK  1
//#define ES7243l_MCLK_SOURCE  FROM_MCLK_PIN
#define ES7243l_MCLK_SOURCE  FROM_INTERNAL_BCLK
/*
 * to select the data length or resolution
 * data length also modified in es7243l_set_dai_fmt
 */
#define DATA_16BITS    0
#define DATA_24BITS    1
#define DATA_32BITS    2
#define ES7243l_DATA_LENGTH   DATA_32BITS
//#define ES7243l_DATA_LENGTH   DATA_24BITS
/*
 * to select the pdm digital microphone interface
 */
#define DMIC_INTERFACE_ON   true
#define DMIC_INTERFACE_OFF  false
#define DMIC_INTERFACE      DMIC_INTERFACE_ON
/*
 * to select bclk inverted or not
 */
#define BCLK_NORMAL       false
#define BCLK_INVERTED     true
#define BCLK_INVERTED_OR_NOT    BCLK_NORMAL
/*
 * to select mclk inverted or not
 */
#define MCLK_NORMAL       false
#define MCLK_INVERTED     true
#define MCLK_INVERTED_OR_NOT    MCLK_NORMAL
/*
 * to select PGA gain for different analog input channel
 * user must allocate the PGA gain for each analog input channel
 * ES7243l_MIC_ARRAY_AIN1_PGA to ES7243l_MIC_ARRAY_AIN16_PGA is used for PGA gain
 */
#define PGA_0DB           0
#define PGA_3DB           1
#define PGA_6DB           2
#define PGA_9DB           3
#define PGA_12DB          4
#define PGA_15DB          5
#define PGA_18DB          6
#define PGA_21DB          7
#define PGA_24DB          8
#define PGA_27DB          9
#define PGA_30DB          10
#define PGA_33DB          11
#define PGA_34DB          12
#define PGA_36DB          13
#define PGA_37DB          14

#if ES7243l_CHANNELS_MAX > 0
	#define ES7243l_MIC_ARRAY_AIN1_PGA     PGA_27DB
	#define ES7243l_MIC_ARRAY_AIN2_PGA     PGA_27DB
#endif

#if ES7243l_CHANNELS_MAX > 2
	#define ES7243l_MIC_ARRAY_AIN3_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN4_PGA     PGA_33DB
#endif

#if ES7243l_CHANNELS_MAX > 4
	#define ES7243l_MIC_ARRAY_AIN5_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN6_PGA     PGA_33DB
#endif

#if ES7243l_CHANNELS_MAX > 6
	#define ES7243l_MIC_ARRAY_AIN7_PGA     PGA_0DB
	#define ES7243l_MIC_ARRAY_AIN8_PGA     PGA_0DB
#endif

#if ES7243l_CHANNELS_MAX > 8
	#define ES7243l_MIC_ARRAY_AIN9_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN10_PGA     PGA_33DB
#endif

#if ES7243l_CHANNELS_MAX > 10
	#define ES7243l_MIC_ARRAY_AIN11_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN12_PGA     PGA_33DB
#endif

#if ES7243l_CHANNELS_MAX > 12
	#define ES7243l_MIC_ARRAY_AIN13_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN14_PGA     PGA_33DB
#endif

#if ES7243l_CHANNELS_MAX > 14
	#define ES7243l_MIC_ARRAY_AIN15_PGA     PGA_33DB
	#define ES7243l_MIC_ARRAY_AIN16_PGA     PGA_33DB
#endif

/*
 * Here is the definition of digital volume.
 * the digital volume is 0dB by default.
 * ES7243l_DIGITAL_VOLUME_1 to ES7243l_DIGITAL_VOLUME_16 is used for digital volume
 */

#if ES7243l_CHANNELS_MAX > 0
	#define DIG_VOL_1     0     // DB
	#define ES7243l_DIGITAL_VOLUME_1        0xbf + (DIG_VOL_1 * 2)
	#define DIG_VOL_2     0     // DB
	#define ES7243l_DIGITAL_VOLUME_2        0xbf + (DIG_VOL_2 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 2
	#define DIG_VOL_3     0     // DB
	#define ES7243l_DIGITAL_VOLUME_3        0xbf + (DIG_VOL_3 * 2)
	#define DIG_VOL_4     0     // DB
	#define ES7243l_DIGITAL_VOLUME_4        0xbf + (DIG_VOL_4 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 4
	#define DIG_VOL_5     0     // DB
	#define ES7243l_DIGITAL_VOLUME_5        0xbf + (DIG_VOL_5 * 2)
	#define DIG_VOL_6     0     // DB
	#define ES7243l_DIGITAL_VOLUME_6        0xbf + (DIG_VOL_6 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 6
	#define DIG_VOL_7     0     // DB
	#define ES7243l_DIGITAL_VOLUME_7        0xbf + (DIG_VOL_7 * 2)
	#define DIG_VOL_8     0     // DB
	#define ES7243l_DIGITAL_VOLUME_8        0xbf + (DIG_VOL_8 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 8
	#define DIG_VOL_9     0     // DB
	#define ES7243l_DIGITAL_VOLUME_9        0xbf + (DIG_VOL_9 * 2)
	#define DIG_VOL_10     0    // DB
	#define ES7243l_DIGITAL_VOLUME_10   0xbf + (DIG_VOL_10 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 10
	#define DIG_VOL_11     0    // DB
	#define ES7243l_DIGITAL_VOLUME_11   0xbf + (DIG_VOL_11 * 2)
	#define DIG_VOL_12     0    // DB
	#define ES7243l_DIGITAL_VOLUME_12   0xbf + (DIG_VOL_12 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 12
	#define DIG_VOL_13     0    // DB
	#define ES7243l_DIGITAL_VOLUME_13   0xbf + (DIG_VOL_13 * 2)
	#define DIG_VOL_14     0    // DB
	#define ES7243l_DIGITAL_VOLUME_14   0xbf + (DIG_VOL_14 * 2)
#endif

#if ES7243l_CHANNELS_MAX > 14
	#define DIG_VOL_15     0    // DB
	#define ES7243l_DIGITAL_VOLUME_15   0xbf + (DIG_VOL_15 * 2)
	#define DIG_VOL_16     0    // DB
	#define ES7243l_DIGITAL_VOLUME_16   0xbf + (DIG_VOL_16 * 2)
#endif

/*
 * set the I2C chip address for each es7243l device in TDM linkloop
 * user can update the chip address according their system circuit
 */
#define I2C_CHIP_ADDR_10H   0x10    // AD0 and AD1 pulled down, ASDOUT pulled down or float
#define I2C_CHIP_ADDR_11H       0x11    // AD0 pulled up, AD1 pulled down, ASDOUT pulled down or float
#define I2C_CHIP_ADDR_12H       0x12    // AD0 pulled down, AD1 pulled up, ASDOUT pulled down or float
#define I2C_CHIP_ADDR_13H       0x13    // AD0 and AD1 pulled up, ASDOUT pulled down or float
#define I2C_CHIP_ADDR_14H       0x14    // AD0 and AD1 pulled down, ASDOUT pulled up
#define I2C_CHIP_ADDR_15H       0x15    // AD0 pulled up, AD1 pulled down, ASDOUT pulled up
#define I2C_CHIP_ADDR_16H       0x16    // AD0 pulled down, AD1 pulled up, ASDOUT pulled up
#define I2C_CHIP_ADDR_17H       0x17    // AD0 and AD1 pulled up, ASDOUT pulled up
#if ES7243l_CHANNELS_MAX > 0
	#define ES7243l_I2C_CHIP_ADDRESS_0       I2C_CHIP_ADDR_10H
#endif
#if ES7243l_CHANNELS_MAX > 2
	#define ES7243l_I2C_CHIP_ADDRESS_1       I2C_CHIP_ADDR_13H
#endif
#if ES7243l_CHANNELS_MAX > 4
	#define ES7243l_I2C_CHIP_ADDRESS_2       I2C_CHIP_ADDR_12H
#endif
#if ES7243l_CHANNELS_MAX > 6
	#define ES7243l_I2C_CHIP_ADDRESS_3       I2C_CHIP_ADDR_11H
#endif
#if ES7243l_CHANNELS_MAX > 8
	#define ES7243l_I2C_CHIP_ADDRESS_4       I2C_CHIP_ADDR_14H
#endif
#if ES7243l_CHANNELS_MAX > 10
	#define ES7243l_I2C_CHIP_ADDRESS_5       I2C_CHIP_ADDR_15H
#endif
#if ES7243l_CHANNELS_MAX > 12
	#define ES7243l_I2C_CHIP_ADDRESS_6       I2C_CHIP_ADDR_16H
#endif
#if ES7243l_CHANNELS_MAX > 14
	#define ES7243l_I2C_CHIP_ADDRESS_7       I2C_CHIP_ADDR_17H
#endif

#define ES7243l_I2C_BUS_NUM         1
#define ES7243l_CODEC_RW_TEST_EN        0
#define ES7243l_IDLE_RESET_EN           1   //reset ES7243 when in idle time
#define ES7243l_MATCH_DTS_EN            1   //ES7243 match method select: 0: i2c_detect, 1:of_device_id

#define VDDA_1V8    1

#define VDDA_VOLTAGE    VDDA_1V8 /* es7243l only supports 1.8V AVDD */
