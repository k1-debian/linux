/*
 *  sound/soc/ingenic/ecodec/wm8594.c
 *  ALSA SoC driver for wm8594 ADC
 *
 *  Copyright 2017 Ingenic Semiconductor Co.,Ltd
 *  wqshao <wangquan.shao@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Only for x2000 FPGA SoCs test.
 */

#ifndef _WM8594_H_
#define _WM8594_H_

#define WM8594_RESET        0x00
#define WM8594_DEVICE_ID    0x00
#define WM8594_REVISION     0x01
#define WM8594_DAC1_CTRL1   0x02
#define WM8594_DAC1_CTRL2   0x03
#define WM8594_DAC1_CTRL3   0x04
#define WM8594_DAC1L_VOL    0x05
#define WM8594_DAC1R_VOL    0x06
#define WM8594_DAC2_CTRL1   0x07
#define WM8594_DAC2_CTRL2   0x08
#define WM8594_DAC2_CTRL3   0x09
#define WM8594_DAC2L_VOL    0x0a
#define WM8594_DAC2R_VOL    0x0b
#define WM8594_ENABLE       0x0c
#define WM8594_ADC_CTRL1    0x0d
#define WM8594_ADC_CTRL2    0x0e
#define WM8594_ADC_CTRL3    0x0f
#define WM8594_ADCL_VOL     0x10
#define WM8594_ADCR_VOL     0x11
/* register 0x12 reserved */
#define WM8594_PGA1L_VOL    0x13
#define WM8594_PGA1R_VOL    0x14
#define WM8594_PGA2L_VOL    0x15
#define WM8594_PGA2R_VOL    0x16
#define WM8594_PGA3L_VOL    0x17
#define WM8594_PGA3R_VOL    0x18
#define WM8594_PGA_CTRL1    0x19
#define WM8594_PGA_CTRL2    0x1a
#define WM8594_ADD_CTRL1    0x1b
#define WM8594_INPUT_CTRL1  0x1c
#define WM8594_INPUT_CTRL2  0x1d
#define WM8594_INPUT_CTRL3  0x1e
#define WM8594_INPUT_CTRL4  0x1f
#define WM8594_OUTPUT_CTRL1 0x20
#define WM8594_OUTPUT_CTRL2 0x21
#define WM8594_OUTPUT_CTRL3 0x22
#define WM8594_BIAS     0x23
#define WM8594_PGA_CTRL3    0x24

#define WM8594_CACHEREG_NUM 36

#define  CTRL2_SR       0
#define  CTRL2_BCLKDIV  3

#endif
