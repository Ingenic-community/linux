#pragma once

#define	CPM_IOBASE			0xb0000000

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
#define CPM_UHCCDR	(0x6c)
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

#define OPCR_ERCS		(0x1<<2)
#define OPCR_PD			(0x1<<3)
#define OPCR_IDLE		(0x1<<31)


#define TCU_IOBASE			0xb0002000

#define TCU_TSSR    0x2c
#define TCU_TSCR    0x3c


#define	WDT_IOBASE			0xb0002000

#define WDT_TCSR    0xc
#define WDT_TCER    0x4
#define WDT_TDR     0x0
#define WDT_TCNT    0x8

#define TSSR_WDTSS  (1 << 16)
#define TSCR_WDTSC  (1 << 16)

#define TCSR_PRESCALE_1     (0 << 3)
#define TCSR_PRESCALE_4     (1 << 3)
#define TCSR_PRESCALE_16    (2 << 3)
#define TCSR_PRESCALE_64    (3 << 3)
#define TCSR_PRESCALE_256   (4 << 3)
#define TCSR_PRESCALE_1024  (5 << 3)

#define TCSR_EXT_EN (1 << 2)
#define TCSR_RTC_EN (1 << 1)
#define TCSR_PCK_EN (1 << 0)

#define TCER_TCEN   (1 << 0)


#define	RTC_IOBASE			0xb0003000

#define RTC_RTCCR		0x0
#define RTCCR_WRDY		BIT(7)

#define RTC_HCR			0x20
#define HCR_PD			BIT(0)

#define RTC_WENR		0x3c
#define WENR_WEN		BIT(31)

#define reg_writel(val, iobase, off)	writel(val, (void *)(iobase) + (off))
#define reg_readl(iobase, off)		readl((void *)(iobase) + (off))

