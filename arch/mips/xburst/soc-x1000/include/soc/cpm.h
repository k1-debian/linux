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

#define CPM_CPCCR	(0x00)
#define CPM_CPCSR	(0xd4)

#define CPM_DDRCDR	(0x2c)
#define CPM_I2SCDR	(0x60)
#define CPM_I2SCDR1	(0x70)
#define CPM_LPCDR	(0x64)
#define CPM_MSC0CDR	(0x68)
#define CPM_MSC1CDR	(0xa4)
#define CPM_USBCDR	(0x50)
#define CPM_MACCDR	(0x54)
#define CPM_SFCCDR	(0x74)
#define CPM_CIMCDR	(0x7c)
#define CPM_PCMCDR	(0x84)
#define CPM_PCMCDR1	(0xe0)
#define CPM_MPHYC	(0xe8)

#define CPM_INTR	(0xb0)
#define CPM_INTRE	(0xb4)
#define CPM_DRCG	(0xd0)
#define CPM_CPSPPR	(0x38)
#define CPM_CPPSR	(0x34)

#define CPM_USBPCR	(0x3c)
#define CPM_USBRDT	(0x40)
#define CPM_USBVBFIL	(0x44)
#define CPM_USBPCR1	(0x48)

#define CPM_CPAPCR	(0x10)
#define CPM_CPMPCR	(0x14)

#define CPM_LCR		(0x04)
#define CPM_PSWC0ST     (0x90)
#define CPM_PSWC1ST     (0x94)
#define CPM_PSWC2ST     (0x98)
#define CPM_PSWC3ST     (0x9c)
#define CPM_CLKGR	(0x20)
#define CPM_MESTSEL	(0xec)
#define CPM_SRBC	(0xc4)
#define CPM_ERNG	(0xd8)
#define CPM_RNG	        (0xdc)
#define CPM_SLBC	(0xc8)
#define CPM_SLPC	(0xcc)
#define CPM_OPCR	(0x24)
#define CPM_RSR		(0x08)

#define LCR_LPM_MASK		(0x3)
#define LCR_LPM_SLEEP		(0x1)

#ifndef BIT
#define BIT(nr)  (1UL << nr)
#endif

/*USB Parameter Control Register*/
#define USBPCR_USB_MODE                 BIT(31)
#define USBPCR_AVLD_REG                 BIT(30)
#define USBPCR_IDPULLUP_MASK_BIT        28
#define USBPCR_IDPULLUP_MASK_MSK                (0x3 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_OTG                             (0x0 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS_SUSPEND  (0x1 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS                  (0x2 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_INCR_MASK                BIT(27)
#define USBPCR_TXRISETUNE               BIT(26)         /*0*/
#define USBPCR_COMMONONN                BIT(25)
#define USBPCR_VBUSVLDEXT               BIT(24)
#define USBPCR_VBUSVLDEXTSEL    BIT(23)
#define USBPCR_POR_BIT                  22
#define USBPCR_POR                              BIT(USBPCR_POR_BIT)
#define USBPCR_SIDDQ                    BIT(21)
#define USBPCR_OTG_DISABLE              BIT(20)
#define USBPCR_COMPDISTUNE_BIT  17
#define USBPCR_COMPDISTUNE_MSK  (0x7 << USBPCR_COMPDISTUNE_BIT)
#define USBPCR_COMPDISTUNE(x)   (((x) << USBPCR_COMPDISTUNE_BIT) & USBPCR_COMPDISTUNE_MSK)  /*4*/
#define USBPCR_OTGTUNE_BIT              14
#define USBPCR_OTGTUNE_MSK              (0x7 << USBPCR_OTGTUNE_BIT)
#define USBPCR_OTGTUNE(x)               (((x) << USBPCR_OTGTUNE_BIT) & USBPCR_OTGTUNE_MSK)                      /*4*/
#define USBPCR_SQRXTUNE_BIT             11
#define USBPCR_SQRXTUNE_MSK             (0x7 << USBPCR_SQRXTUNE_BIT)
#define USBPCR_SQRXTUNE(x)              (((x) << USBPCR_SQRXTUNE_BIT) & USBPCR_SQRXTUNE_MSK)            /*3*/
#define USBPCR_TXFSLSTUNE_BIT   7
#define USBPCR_TXFSLSTUNE_MSK   (0xf << USBPCR_TXFSLSTUNE_BIT)
#define USBPCR_TXFSLSTUNE(x)    (((x) << USBPCR_TXFSLSTUNE_BIT) & USBPCR_TXFSLSTUNE_MSK)        /*2*/
#define USBPCR_TXPREEMPHTUNE    BIT(6)                                          /*0*/
#define USBPCR_TXHSXVTUNE_BIT   4
#define USBPCR_TXHSXVTUNE_MSK   (0x3 << USBPCR_TXHSXVTUNE_BIT)
#define USBPCR_TXHSXVTUNE(x)    (((x) << USBPCR_TXHSXVTUNE_BIT) & USBPCR_TXHSXVTUNE_MSK)        /*3*/
#define USBPCR_TXVREFTUNE_BIT   0
#define USBPCR_TXVREFTUNE_MSK   (0xf << USBPCR_TXVREFTUNE_BIT)
#define USBPCR_TXVREFTUNE(x)    (((x) << USBPCR_TXVREFTUNE_BIT) & USBPCR_TXVREFTUNE_MSK)    /*4*/

/*USB Reset Detect Timer Register*/
#define USBRDT_HB_MASK                  BIT(26)
#define USBRDT_VBFIL_LD_EN              BIT(25)
#define USBRDT_IDDIG_EN                 24
#define USBRDT_IDDIG_REG                23
#define USBRDT_USBRDT_MSK               (0x7fffff)
#define USBRDT_USBRDT(x)                ((x) & USBRDT_USBRDT_MSK)

/*USB VBUS Jitter Filter Register*/
#define USBVBFIL_USBVBFIL(x)    ((x) & 0xffff)
#define USBVBFIL_IDDIGFIL(x)    ((x) & (0xffff << 16))

/*USB Parameter Control Register1*/
#define USBPCR1_BVLD_REG                BIT(31)
#define USBPCR1_REFCLKSEL               (0x3 << 26)
#define USBPCR1_REFCLKDIV_MSK   (0x3 << 24)
#define USBPCR1_REFCLKDIV(x)    (((x) & 0x3) << 24)
#define USBPCR1_REFCLKDIV_48M   (0x2)
#define USBPCR1_REFCLKDIV_24M   (0x1)
#define USBPCR1_REFCLKDIV_12M   (0x0)
#define USBPCR1_PORT_RST		BIT(21)
#define USBPCR1_PORT1_RST		BIT(20)
#define USBPCR1_WORD_IF_16BIT		BIT(19)
#define USBPCR1_WORD_IF1_16BIT		BIT(18)
#define USBPCR1_COMPDISTUNE1_BIT	15
#define USBPCR1_COMPDISTUNE1_MSK	(0x7 << USBPCR1_COMPDISTUNE1_BIT)
#define USBPCR1_COMPDISTUNE1(x)		(((x) << USBPCR1_COMPDISTUNE1_BIT) & USBPCR1_COMPDISTUNE1_MSK)
#define USBPCR1_SQRXTUNE1_BIT	12
#define USBPCR1_SQRXTUNE1_MSK	(0x7 << USBPCR1_SQRXTUNE1_BIT)
#define USBPCR1_SQRXTUNE1(x)	(((x) << USBPCR1_SQRXTUNE1_BIT) & USBPCR1_SQRXTUNE1_MSK)            /*3*/
#define USBPCR1_TXFSLSTUNE1_BIT	8
#define USBPCR1_TXFSLSTUNE1_MSK	(0xf << USBPCR1_TXFSLSTUNE1_BIT)
#define USBPCR1_TXFSLSTUNE1(x)	(((x) << USBPCR1_TXFSLSTUNE1_BIT) & USBPCR1_TXFSLSTUNE1_MSK)        /*2*/
#define USBPCR1_TXPREEMPHTUNE1	BIT(6)                                          /*0*/
#define USBPCR1_TXHSXVTUNE1_BIT	5
#define USBPCR1_TXHSXVTUNE1_MSK	(0x3 << USBPCR1_TXHSXVTUNE1_BIT)
#define USBPCR1_TXHSXVTUNE1(x)	(((x) << USBPCR1_TXHSXVTUNE1_BIT) & USBPCR1_TXHSXVTUNE1_MSK)        /*3*/
#define USBPCR1_TXVREFTUNE1_BIT 1
#define USBPCR1_TXVREFTUNE1_MSK	(0xf << USBPCR1_TXVREFTUNE1_BIT)
#define USBPCR1_TXVREFTUNE1(x)	(((x) << USBPCR1_TXVREFTUNE1_BIT) & USBPCR1_TXVREFTUNE1_MSK)    /*4*/
#define USBPCR1_TXRISETUNE1	BIT(0)

/*Oscillator and Power Control Register*/
#define OPCR_IDLE		BIT(31)
#define OPCR_USB_SPENDN		BIT(7)
#define OPCR_USB_SPENDN1	BIT(6)
#define OPCR_PD			BIT(3)
#define OPCR_ERCS		BIT(2)

/*Soft Reset and Bus Control Register*/
#define SRBC_OTG_SR		BIT(12)

#define cpm_inl(off)		inl(CPM_IOBASE + (off))
#define cpm_outl(val,off)	outl(val,CPM_IOBASE + (off))
#define cpm_clear_bit(val,off)	do{cpm_outl((cpm_inl(off) & ~(1<<(val))),off);}while(0)
#define cpm_set_bit(val,off)	do{cpm_outl((cpm_inl(off) |  (1<<val)),off);}while(0)
#define cpm_test_bit(val,off)	(cpm_inl(off) & (0x1<<val))
#endif
