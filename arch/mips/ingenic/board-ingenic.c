// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Support for Ingenic SoCs
 *
 * Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2020 Paul Cercueil <paul@crapouillou.net>
 * Copyright (C) 2022-2023 Reimu NotMoe <reimu@sudomaker.com>
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
#include <asm/delay.h>

#include "regs.h"

int ingenic_pm_clk_mode = 0;

static __init char *ingenic_get_system_type(unsigned long machtype)
{
	switch (machtype) {
	case MACH_INGENIC_X2100:
		return "X2100";
	case MACH_INGENIC_X2000H:
		return "X2000H";
	case MACH_INGENIC_X2000E:
		return "X2000E";
	case MACH_INGENIC_X2000:
		return "X2000";
	case MACH_INGENIC_X1830:
		return "X1830";
	case MACH_INGENIC_X1500:
		return "X1500";
	case MACH_INGENIC_X1501:
		return "X1501";
	case MACH_INGENIC_X1000E:
		return "X1000E";
	case MACH_INGENIC_X1000:
		return "X1000";
	case MACH_INGENIC_JZ4780:
		return "JZ4780";
	case MACH_INGENIC_JZ4775:
		return "JZ4775";
	case MACH_INGENIC_JZ4770:
		return "JZ4770";
	case MACH_INGENIC_JZ4760B:
		return "JZ4760B";
	case MACH_INGENIC_JZ4760:
		return "JZ4760";
	case MACH_INGENIC_JZ4755:
		return "JZ4755";
	case MACH_INGENIC_JZ4750:
		return "JZ4750";
	case MACH_INGENIC_JZ4725B:
		return "JZ4725B";
	case MACH_INGENIC_JZ4730:
		return "JZ4730";
	default:
		return "JZ4740";
	}
}

static __init const void *ingenic_fixup_fdt(const void *fdt, const void *match_data)
{
	/*
	 * Old devicetree files for the qi,lb60 board did not have a /memory
	 * node. Hardcode the memory info here.
	 */
	if (!fdt_node_check_compatible(fdt, 0, "qi,lb60") &&
		fdt_path_offset(fdt, "/memory") < 0)
		early_init_dt_add_memory_arch(0, SZ_32M);

	mips_machtype = (unsigned long)match_data;
	system_type = ingenic_get_system_type(mips_machtype);

	return fdt;
}

static const struct of_device_id ingenic_of_match[] __initconst = {
	{ .compatible = "ingenic,jz4730", .data = (void *)MACH_INGENIC_JZ4730 },
	{ .compatible = "ingenic,jz4740", .data = (void *)MACH_INGENIC_JZ4740 },
	{ .compatible = "ingenic,jz4725b", .data = (void *)MACH_INGENIC_JZ4725B },
	{ .compatible = "ingenic,jz4750", .data = (void *)MACH_INGENIC_JZ4750 },
	{ .compatible = "ingenic,jz4755", .data = (void *)MACH_INGENIC_JZ4755 },
	{ .compatible = "ingenic,jz4760", .data = (void *)MACH_INGENIC_JZ4760 },
	{ .compatible = "ingenic,jz4760b", .data = (void *)MACH_INGENIC_JZ4760B },
	{ .compatible = "ingenic,jz4770", .data = (void *)MACH_INGENIC_JZ4770 },
	{ .compatible = "ingenic,jz4775", .data = (void *)MACH_INGENIC_JZ4775 },
	{ .compatible = "ingenic,jz4780", .data = (void *)MACH_INGENIC_JZ4780 },
	{ .compatible = "ingenic,x1000", .data = (void *)MACH_INGENIC_X1000 },
	{ .compatible = "ingenic,x1000e", .data = (void *)MACH_INGENIC_X1000E },
	{ .compatible = "ingenic,x1500", .data = (void *)MACH_INGENIC_X1500 },
	{ .compatible = "ingenic,x1501", .data = (void *)MACH_INGENIC_X1501 },
	{ .compatible = "ingenic,x1830", .data = (void *)MACH_INGENIC_X1830 },
	{ .compatible = "ingenic,x2000", .data = (void *)MACH_INGENIC_X2000 },
	{ .compatible = "ingenic,x2000e", .data = (void *)MACH_INGENIC_X2000E },
	{ .compatible = "ingenic,x2000h", .data = (void *)MACH_INGENIC_X2000H },
	{ .compatible = "ingenic,x2100", .data = (void *)MACH_INGENIC_X2100 },
	{}
};

MIPS_MACHINE(ingenic) = {
	.matches = ingenic_of_match,
	.fixup_fdt = ingenic_fixup_fdt,
};

#ifdef CONFIG_MACH_X1000
extern void ingenic_pm_init_x1000(void);
extern void ingenic_pm_poweroff_x1000(void);
extern void ingenic_pm_sleep_x1000(void);
#endif

static void ingenic_wait_instr(void)
{
	__asm__(".set push;\n"
		".set mips3;\n"
		"wait;\n"
		".set pop;\n"
	);
}

#define WDT_DIV			64
#define RTCLK_FREQ		32768
#define EXCLK_FREQ		24000000
#define RESET_DELAY_MS		4
#define TCSR_PRESCALE		TCSR_PRESCALE_64

static void ingenic_halt(void)
{
	for (;;)
		ingenic_wait_instr();
}

static void ingenic_restart(char *command)
{
		uint32_t src_freq, time;

	if (ingenic_pm_clk_mode)
			src_freq = RTCLK_FREQ;
	else
			src_freq = EXCLK_FREQ / 512;

	time = src_freq / WDT_DIV * RESET_DELAY_MS / 1000;

	if (time > 65535)
		time = 65535;

	reg_writel(TSCR_WDTSC, TCU_IOBASE, TCU_TSCR);

	reg_writel(0, WDT_IOBASE, WDT_TCNT);
	reg_writel(time, WDT_IOBASE, WDT_TDR);
	reg_writel(TCSR_PRESCALE | TCSR_RTC_EN, WDT_IOBASE, WDT_TCSR);
	reg_writel(0, WDT_IOBASE, WDT_TCER);

	printk("ingenic_pm: reset in %dms\n", RESET_DELAY_MS);

	reg_writel(TCER_TCEN, WDT_IOBASE, WDT_TCER);
	ingenic_halt();
}

static int __maybe_unused ingenic_pm_enter(suspend_state_t state)
{

	#ifdef CONFIG_PM_SLEEP
		#ifdef CONFIG_MACH_X1000
			ingenic_pm_sleep_x1000();
		#else
			ingenic_wait_instr();
		#endif
	#else
		ingenic_wait_instr();
	#endif

	return 0;
}

static const struct platform_suspend_ops ingenic_pm_ops __maybe_unused = {
	.valid = suspend_valid_only_mem,
	.enter = ingenic_pm_enter,
};

static int __init ingenic_pm_init(void)
{
	if (boot_cpu_type() == CPU_XBURST || boot_cpu_type() == CPU_XBURST2) {

		if (IS_ENABLED(CONFIG_PM_SLEEP))
			suspend_set_ops(&ingenic_pm_ops);

		_machine_halt = ingenic_halt;
		_machine_restart = ingenic_restart;

		#ifdef CONFIG_PM_SLEEP
			#ifdef CONFIG_MACH_X1000
				ingenic_pm_init_x1000();
				pm_power_off = ingenic_pm_poweroff_x1000;
			#endif
		#endif

		pr_info("ingenic_pm: PM retention clock selection: %s\n", ingenic_pm_clk_mode ? "RTCLK" : "EXCLK");
	}

	return 0;

}

static int __init ingenic_pm_clk_setup(char *s) {
	int clk_mode = -1;

	if (s) {
		clk_mode = s[0] - 0x30;
	}

	if (clk_mode == 0 || clk_mode == 1) {
		ingenic_pm_clk_mode = clk_mode;
	} else {
		pr_info("ingenic_pm: BAD PM retention clock selection: %s\n", s);
		pr_info("ingenic_pm: valid selections: 0 - EXCLK, 1 - RTCLK\n");
	}

	return 0;
}

early_param("ingenic_pm_clk", ingenic_pm_clk_setup);

late_initcall(ingenic_pm_init);
