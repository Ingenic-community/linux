// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-3-Clause)
/*
 * Ingenic X1000 series power management routines
 *
 * Copyright (C) 2006-2016, Ingenic Semiconductor Inc.
 * Copyright (C) 2022-2023, Reimu NotMoe <reimu@sudomaker.com>
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/pm.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/types.h>

#include <asm/bootinfo.h>
#include <asm/machine.h>
#include <asm/reboot.h>

#include "regs.h"

#ifdef CONFIG_PM_SLEEP

#define SLEEP_TCSM_SPACE           0xb3422000
#define VOICE_TCSM_DATA_BUF	   0xb3423000
#define TCSM_BANK_LEN              4096

#define cpm_inl(off)		readl((void *)(CPM_IOBASE) + (off))
#define cpm_outl(val,off)	writel(val, (void *)(CPM_IOBASE) + (off))
#define cpm_clear_bit(val,off)	do{cpm_outl((cpm_inl(off) & ~(1<<(val))),off);}while(0)
#define cpm_set_bit(val,off)	do{cpm_outl((cpm_inl(off) |  (1<<val)),off);}while(0)
#define cpm_test_bit(val,off)	(cpm_inl(off) & (0x1<<val))

struct sleep_param
{
	unsigned int  pm_core_enter;
	unsigned char pmu_i2c_scl;		//default 0xff
	unsigned char pmu_i2c_sda;		//default 0xff
	unsigned char pmu_addr;			//default 0xff
	unsigned char pmu_reg;			//default 0xff
	unsigned char pmu_register_val;

	unsigned char pmu_pin;			//default 0xff
	unsigned char pmu_pin_func;		//default 0xff
	unsigned char uart_id;			//default 0xff

	unsigned int  prev_resume_pc;		//ddr is self-reflash default 0xffffffff
	unsigned int  post_resume_pc;		//ddr is ok. default 0xffffffff
	unsigned int  prev_sleep_pc;		//after flush cache. default 0xffffffff
	unsigned int  post_sleep_pc;		//before wait. default 0xffffffff

};

struct sleep_save_register
{
	unsigned int lcr;
	unsigned int opcr;
	unsigned int ddr_training_space[20];
};

static struct sleep_param *sleep_param;
static struct sleep_save_register s_reg;

static unsigned int pm_firmware_new[] = {
#include "x1000_sleep.hex"
};

static void load_pm_firmware_new(unsigned int addr) {
	void (*func)(unsigned int addr,unsigned int to);
	unsigned int firmware_size = sizeof(pm_firmware_new);

	if (firmware_size > TCSM_BANK_LEN * 1024)
		printk(KERN_WARNING "WARN: firmware_size %d bigger than" \
		       "TCSM_BANK_LEN %d\n", firmware_size, TCSM_BANK_LEN * 1024);
	func = (void (*)(unsigned int,unsigned int))addr;
	memcpy((void *)addr,pm_firmware_new,firmware_size);
	func(addr,0);
}

extern long long save_goto(unsigned int);
extern int restore_goto(void);

extern int ingenic_pm_clk_mode;

void ingenic_pm_init_x1000(void) {
	unsigned int lcr, opcr;

	/* init opcr and lcr for idle */
	lcr = cpm_inl(CPM_LCR);
	lcr &= ~(0x3);		/* LCR.SLEEP.DS=1'b0,LCR.LPM=2'b00*/
	lcr |= 0xff << 8;	/* power stable time */
	cpm_outl(lcr,CPM_LCR);

	opcr = cpm_inl(CPM_OPCR);
	opcr |= 0xff << 8;	/* EXCLK stable time */
	opcr &= ~(1 << 4);	/* EXCLK oscillator is disabled in Sleep mode */
	cpm_outl(opcr,CPM_OPCR);
}

void ingenic_pm_poweroff_x1000(void) {
	int timeout = 0x2000;

	while (!(reg_readl(RTC_IOBASE, RTC_RTCCR) & RTCCR_WRDY) && timeout--);

	if (!timeout) {
		printk("ingenic_pm: WARNING: RTC does not tick, either the 32k crystal is not connected, "
			"or there is a hardware failure. Poweroff will not succeed.\n");

		panic("RTC est mort");
	}

	reg_writel(0x0000a55a, RTC_IOBASE, RTC_WENR);
	while(!(reg_readl(RTC_IOBASE, RTC_RTCCR) & RTCCR_WRDY));
	while(!(reg_readl(RTC_IOBASE, RTC_WENR) & WENR_WEN));
	reg_writel(0x1, RTC_IOBASE, RTC_HCR);
	while(!(reg_readl(RTC_IOBASE, RTC_RTCCR) & RTCCR_WRDY));

	panic("Poweroff failed\n");
}

void ingenic_pm_sleep_x1000(void) {
	unsigned int lcr, opcr, clkgr;

	printk("ingenic_pm_x1000: begin sleep\n");

	memcpy(&s_reg.ddr_training_space, (void*)0x80000000, sizeof(s_reg.ddr_training_space));
	s_reg.opcr = cpm_inl(CPM_OPCR);
	s_reg.lcr = cpm_inl(CPM_LCR);

	clkgr = cpm_inl(CPM_CLKGR);

	if (clkgr & (1 << 21)) {
		clkgr &= ~(1 << 21);
		cpm_outl(clkgr, CPM_CLKGR);

		while (cpm_inl(CPM_CLKGR) & (1 << 21));
	}

	load_pm_firmware_new(SLEEP_TCSM_SPACE);
	sleep_param = (struct sleep_param *)SLEEP_TCSM_SPACE;

	sleep_param->post_resume_pc = (unsigned int)restore_goto;
	sleep_param->uart_id = -1;

	opcr = s_reg.opcr;
	lcr = s_reg.lcr;

	opcr &= ~((1 << 25) | (1 << 22) | (0xfff << 8) | (1 << 4) | (1 << 3) | (1 << 2));
	opcr |= (1 << 31) | (1 << 30) |  (1 << 25) | (0xfff << 8) | (1 << 4) | (1 << 3);
	lcr &= ~3;


	if (ingenic_pm_clk_mode) {
		opcr &= ~((1 << 4) | (1 << 2));
		opcr |= (1 << 2);
	}

	lcr |= LCR_LPM_SLEEP;

	cpm_outl(opcr,CPM_OPCR);
	cpm_outl(lcr,CPM_LCR);

	// printk("#####lcr:%08x\n", cpm_inl(CPM_LCR));
	// printk("#####gate:%08x\n", cpm_inl(CPM_CLKGR));
	// printk("#####opcr:%08x\n", cpm_inl(CPM_OPCR));
	// printk("#####INT_MASK0:%08x\n", *(volatile unsigned int*)(0xB0001004));
	// printk("#####INT_MASK1:%08x\n", *(volatile unsigned int*)(0xB0001024));

	mb();
	save_goto((unsigned int)sleep_param->pm_core_enter);
	mb();

	memcpy((void*)0x80000000,&s_reg.ddr_training_space,sizeof(s_reg.ddr_training_space));
	dma_cache_wback_inv(0x80000000,sizeof(s_reg.ddr_training_space));
	cpm_outl(s_reg.lcr,CPM_LCR);
	cpm_outl(s_reg.opcr,CPM_OPCR);

	printk("ingenic_pm_x1000: end sleep\n");

}

#endif