/*
 * SOC CPM register definition.
 *
 * CPM (Clock reset and Power control Management)
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __SOC_CPM_H__
#define __SOC_CPM_H__

#include <soc/base.h>

#define CPM_CPCCR       (0x00)
#define CPM_CPCSR       (0xD4)

#define CPM_DDRCDR      (0x2c)
#define CPM_MACCDR      (0x54)
#define CPM_MACTXCDR        (0x58)
#define CPM_MACTXCDR1       (0xdc)
#define CPM_MACPTP      (0x4c)
#define CPM_I2S0CDR     (0x60)
#define CPM_I2S1CDR     (0x7c)
#define CPM_I2S2CDR     (0x84)
#define CPM_I2S3CDR     (0x8c)
#define CPM_I2S0CDR1        (0x70)
#define CPM_I2S1CDR1        (0x80)
#define CPM_I2S2CDR1        (0x88)
#define CPM_I2S3CDR1        (0xa0)
#define CPM_AUDIOCR     (0xac)
#define CPM_LPCDR       (0x64)
#define CPM_MSC0CDR     (0x68)
#define CPM_MSC1CDR     (0xa4)
#define CPM_MSC2CDR     (0xa8)
#define CPM_SFCCDR      (0x74)
#define CPM_SSICDR      (0x5c)
#define CPM_CIMCDR      (0x78)
#define CPM_PWMCDR      (0x6c)
#define CPM_ISPCDR      (0x30)
#define CPM_RSACDR      (0x50)

#define CPM_INTR        (0xB0)
#define CPM_INTRE       (0xB4)
#define CPM_SFTINT      (0xBC)
#define CPM_DRCG        (0xD0)
#define CPM_CPPSR       (0x34)
#define CPM_CPSPPR      (0x38)
#define CPM_USBPCR      (0x3C)
#define CPM_USBRDT      (0x40)
#define CPM_USBVBFIL        (0x44)
#define CPM_USBPCR1     (0x48)

#define CPM_CPPCR       (0x0C)
#define CPM_CPAPCR      (0x10)
#define CPM_CPMPCR      (0x14)
#define CPM_CPEPCR      (0x18)

#define CPM_LCR         (0x04)
#define CPM_PSWC0ST     (0x90)
#define CPM_PSWC1ST     (0x94)
#define CPM_PSWC2ST     (0x98)
#define CPM_PSWC3ST     (0x9C)
#define CPM_CLKGR       (0x20)
#define CPM_CLKGR1      (0x28)
#define CPM_MESTSEL     (0xEC)
#define CPM_SRBC        (0xC4)
#define CPM_EXCLK_DS        (0xE0)
#define CPM_MPDCR       (0xF8)
#define CPM_MPDCR1      (0xFC)
#define CPM_SLBC        (0xC8)
#define CPM_SLPC        (0xCC)
#define CPM_OPCR        (0x24)

#define CPM_RSR         (0x08)

#ifndef BIT
	#define BIT(nr)  (1UL << nr)
#endif

/*USB Parameter Control Register*/
#define USBPCR_USB_MODE                 BIT(31)
#define USBPCR_AVLD_REG                 BIT(30)
#define USBPCR_IDPULLUP_MASK_BIT        28
#define USBPCR_IDPULLUP_MASK        (0x3 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_OTG             (0x0 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS_SUSPEND  (0x1 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS          (0x2 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_INCR_MASK                BIT(27)
#define USBPCR_POR_BIT                  22
#define USBPCR_POR                      BIT(USBPCR_POR_BIT)

/*USB Reset Detect Timer Register*/
#define USBRDT_RESUME_INTEEN        BIT(31) /*RW*/
#define USBRDT_RESUME_INTERCLR      BIT(30) /*W0*/
#define USBRDT_RESUME_SPEED_BIT     28  /*RW*/
#define USBRDT_RESUME_SPEED_MSK     (0x3 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_HIGH    (0x0 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_FULL    (0x1 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_LOW     (0x2 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_STATUS        27  /*RO*/
#define USBRDT_HB_MASK                  BIT(26)
#define USBRDT_VBFIL_LD_EN              BIT(25)
#define USBRDT_IDDIG_EN                 BIT(24)
#define USBRDT_IDDIG_REG                BIT(23)
#define USBRDT_USBRDT_MSK               (0x7fffff)
#define USBRDT_USBRDT(x)                ((x) & USBRDT_USBRDT_MSK)

/*USB VBUS Jitter Filter Register*/
#define USBVBFIL_USBVBFIL(x)        ((x) & 0xffff)
#define USBVBFIL_IDDIGFIL(x)        ((x) & (0xffff << 16))

/*USB Parameter Control Register1*/
#define USBPCR1_BVLD_REG        BIT(31)
#define USBPCR1_DPPULLDOWN  BIT(29)
#define USBPCR1_DMPULLDOWN  BIT(28)
#define USBPCR1_PORT_RST    BIT(21)

/*Oscillator and Power Control Register*/
#define OPCR_USB_SPENDN     BIT(7)


#define LCR_LPM_MASK        (0x3)
#define LCR_LPM_SLEEP       (0x1)

#define OPCR_ERCS       (0x1<<2)
#define OPCR_PD         (0x1<<3)
#define OPCR_IDLE       (0x1<<31)


#define cpm_inl(off)        inl(CPM_IOBASE + (off))
#define cpm_outl(val,off)   outl(val,CPM_IOBASE + (off))
#define cpm_clear_bit(val,off)  do{cpm_outl((cpm_inl(off) & ~(1<<(val))),off);}while(0)
#define cpm_set_bit(val,off)    do{cpm_outl((cpm_inl(off) |  (1<<val)),off);}while(0)
#define cpm_test_bit(val,off)   (cpm_inl(off) & (0x1<<val))

#endif
