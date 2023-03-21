// SPDX-License-Identifier: GPL-2.0
/*
 * X1000 SoC CGU driver
 * Copyright (C) 2019-2023 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 * Copyright (C) 2023 Reimu NotMoe <reimu@sudomaker.com>
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>

#include <dt-bindings/clock/ingenic,x1000-cgu.h>

#include "cgu.h"
#include "pm.h"

/* CGU register offsets */
#define CGU_REG_CPCCR		0x00
#define CGU_REG_APLL		0x10
#define CGU_REG_MPLL		0x14
#define CGU_REG_CLKGR		0x20
#define CGU_REG_OPCR		0x24
#define CGU_REG_DDRCDR		0x2c
#define CGU_REG_USBPCR		0x3c
#define CGU_REG_USBPCR1		0x48
#define CGU_REG_USBCDR		0x50
#define CGU_REG_MACCDR		0x54
#define CGU_REG_I2SCDR		0x60
#define CGU_REG_LPCDR		0x64
#define CGU_REG_MSC0CDR		0x68
#define CGU_REG_I2SCDR1		0x70
#define CGU_REG_SSICDR		0x74
#define CGU_REG_CIMCDR		0x7c
#define CGU_REG_PCMCDR		0x84
#define CGU_REG_MSC1CDR		0xa4
#define CGU_REG_CMP_INTR	0xb0
#define CGU_REG_CMP_INTRE	0xb4
#define CGU_REG_DRCG		0xd0
#define CGU_REG_CPCSR		0xd4
#define CGU_REG_PCMCDR1		0xe0
#define CGU_REG_MACPHYC		0xe8

/* bits within the OPCR register */
#define OPCR_SPENDN0		BIT(7)
#define OPCR_SPENDN1		BIT(6)

/* bits within the USBPCR register */
#define USBPCR_SIDDQ		BIT(21)
#define USBPCR_OTG_DISABLE	BIT(20)

/* bits within the USBPCR1 register */
#define USBPCR1_REFCLKSEL_SHIFT	26
#define USBPCR1_REFCLKSEL_MASK	(0x3 << USBPCR1_REFCLKSEL_SHIFT)
#define USBPCR1_REFCLKSEL_CORE	(0x2 << USBPCR1_REFCLKSEL_SHIFT)
#define USBPCR1_REFCLKDIV_SHIFT	24
#define USBPCR1_REFCLKDIV_MASK	(0x3 << USBPCR1_REFCLKDIV_SHIFT)
#define USBPCR1_REFCLKDIV_48	(0x2 << USBPCR1_REFCLKDIV_SHIFT)
#define USBPCR1_REFCLKDIV_24	(0x1 << USBPCR1_REFCLKDIV_SHIFT)
#define USBPCR1_REFCLKDIV_12	(0x0 << USBPCR1_REFCLKDIV_SHIFT)

/* bits within the I2SCDR register */
#define I2SCDR_I2CS_SHIFT		30
#define I2SCDR_I2CS_MASK		(0x1 << I2SCDR_I2CS_SHIFT)

static struct ingenic_cgu *cgu;

#define X1000_CGU_PLL_CACHE_SIZE	8

struct x1000_cgu_pll_cache {
	unsigned long rate;
	unsigned long parent_rate;
	unsigned int m;
	unsigned int n;
};

static struct x1000_cgu_pll_cache pll_cache[X1000_CGU_PLL_CACHE_SIZE];
static unsigned int pll_cache_usage = 0;

static void x1000_cgu_pll_cache_fill(unsigned long rate, unsigned long parent_rate,
		       unsigned int m, unsigned int n)
{
	int i = pll_cache_usage;

	pll_cache[i].rate = rate;
	pll_cache[i].parent_rate = parent_rate;
	pll_cache[i].m = m;
	pll_cache[i].n = n;

	pll_cache_usage = (pll_cache_usage + 1) % X1000_CGU_PLL_CACHE_SIZE;
}

static struct x1000_cgu_pll_cache *x1000_cgu_pll_cache_find(unsigned long rate,
			unsigned long parent_rate)
{
	for (int i=0; i<X1000_CGU_PLL_CACHE_SIZE; i++) {
		if (pll_cache[i].rate == rate && pll_cache[i].parent_rate == parent_rate) {
			// printk("pll_cache: found at %d\n", i);
			return &pll_cache[i];
		}
	}

	return NULL;
}

// This is a stupid way of calculating the PLL, but it doesn't have any edge cases.

// Since the I2S of X1000 supports arbitrary sample rates and it would be a pity to
// only support standard audio sample rates. For example, supporting a 3.579545 MHz
// MCLK would be very useful if you want to run synth software with a R2R DAC.

// With a simple 8-entries cache, it shouldn't be slow on average, and we can take full
// advantage of it.
static void x1000_i2s_calc_m_n(const struct ingenic_cgu_pll_info *pll_info,
		       unsigned long rate, unsigned long parent_rate,
		       unsigned int *pm, unsigned int *pn, unsigned int *pod)
{
	unsigned long curr_m, curr_n;
	u64 freq_real, freq_diff, freq_diff_min = U64_MAX;
	struct x1000_cgu_pll_cache *cached_value;

	pr_info("x1000_i2s_pll: parent_rate: %lu, rate: %lu\n", parent_rate, rate);

	if (parent_rate == 0 || rate == 0) {
		curr_m = curr_n = 0;
		goto out_nofill;
	}

	cached_value = x1000_cgu_pll_cache_find(rate, parent_rate);

	if (cached_value) {
		curr_m = cached_value->m;
		curr_n = cached_value->n;
		goto out_nofill;
	}

	if ((parent_rate % rate == 0) && ((parent_rate / rate) > 1)) {
		curr_m = 1;
		curr_n = parent_rate / rate;
		goto out;
	}

	/*
	 * The length of M is 9 bits, its value must be between 1 and 511.
	 * The length of N is 13 bits, its value must be between 2 and 8191,
	 * and must not be less than 2 times of the value of M.
	 */
	for (curr_m = 511; curr_m >= 1; curr_m--) {
		for (curr_n = 8191; curr_n >= (curr_m * 2); curr_n--) {
			freq_real = (u64)parent_rate * (u64)curr_m;
			__div64_32(&freq_real, curr_n);

			if (freq_real == rate) {
				goto out;
			} else {
				freq_diff = freq_real > rate ? freq_real - rate : rate - freq_real;

				if (freq_diff < freq_diff_min) {
					freq_diff_min = freq_diff;
				}
			}
		}
	}

	for (curr_m = 511; curr_m >= 1; curr_m--) {
		for (curr_n = 8191; curr_n >= (curr_m * 2); curr_n--) {
			freq_real = (u64)parent_rate * (u64)curr_m;
			__div64_32(&freq_real, curr_n);

			freq_diff = freq_real > rate ? freq_real - rate : rate - freq_real;

			if (freq_diff == freq_diff_min) {
				goto out;
			}
		}
	}

out:
	x1000_cgu_pll_cache_fill(rate, parent_rate, curr_m, curr_n);

out_nofill:
	*pm = curr_m;
	*pn = curr_n;

	/*
	 * The I2S PLL does not have OD bits, so set the *pod to 1 to ensure
	 * that the ingenic_pll_calc() in cgu.c can run properly.
	 */
	*pod = 1;
}

static unsigned long x1000_otg_phy_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	u32 usbpcr1;
	unsigned refclk_div;

	usbpcr1 = readl(cgu->base + CGU_REG_USBPCR1);
	refclk_div = usbpcr1 & USBPCR1_REFCLKDIV_MASK;

	switch (refclk_div) {
	case USBPCR1_REFCLKDIV_12:
		return 12000000;

	case USBPCR1_REFCLKDIV_24:
		return 24000000;

	case USBPCR1_REFCLKDIV_48:
		return 48000000;
	}

	return parent_rate;
}

static long x1000_otg_phy_round_rate(struct clk_hw *hw, unsigned long req_rate,
				      unsigned long *parent_rate)
{
	if (req_rate < 18000000)
		return 12000000;

	if (req_rate < 36000000)
		return 24000000;

	return 48000000;
}

static int x1000_otg_phy_set_rate(struct clk_hw *hw, unsigned long req_rate,
				   unsigned long parent_rate)
{
	unsigned long flags;
	u32 usbpcr1, div_bits;

	switch (req_rate) {
	case 12000000:
		div_bits = USBPCR1_REFCLKDIV_12;
		break;

	case 24000000:
		div_bits = USBPCR1_REFCLKDIV_24;
		break;

	case 48000000:
		div_bits = USBPCR1_REFCLKDIV_48;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&cgu->lock, flags);

	usbpcr1 = readl(cgu->base + CGU_REG_USBPCR1);
	usbpcr1 &= ~USBPCR1_REFCLKDIV_MASK;
	usbpcr1 |= div_bits;
	writel(usbpcr1, cgu->base + CGU_REG_USBPCR1);

	spin_unlock_irqrestore(&cgu->lock, flags);
	return 0;
}

static int x1000_usb_phy_enable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	writel(readl(reg_opcr) | OPCR_SPENDN0, reg_opcr);
	writel(readl(reg_usbpcr) & ~USBPCR_OTG_DISABLE & ~USBPCR_SIDDQ, reg_usbpcr);
	return 0;
}

static void x1000_usb_phy_disable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	writel(readl(reg_opcr) & ~OPCR_SPENDN0, reg_opcr);
	writel(readl(reg_usbpcr) | USBPCR_OTG_DISABLE | USBPCR_SIDDQ, reg_usbpcr);
}

static int x1000_usb_phy_is_enabled(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;
	void __iomem *reg_usbpcr	= cgu->base + CGU_REG_USBPCR;

	return (readl(reg_opcr) & OPCR_SPENDN0) &&
		!(readl(reg_usbpcr) & USBPCR_SIDDQ) &&
		!(readl(reg_usbpcr) & USBPCR_OTG_DISABLE);
}

static const struct clk_ops x1000_otg_phy_ops = {
	.recalc_rate = x1000_otg_phy_recalc_rate,
	.round_rate = x1000_otg_phy_round_rate,
	.set_rate = x1000_otg_phy_set_rate,

	.enable		= x1000_usb_phy_enable,
	.disable	= x1000_usb_phy_disable,
	.is_enabled	= x1000_usb_phy_is_enabled,
};

static const s8 pll_od_encoding[8] = {
	0x0, 0x1, -1, 0x2, -1, -1, -1, 0x3,
};

static u8 x1000_i2s_get_parent(struct clk_hw *hw)
{
	u32 i2scdr;

	i2scdr = readl(cgu->base + CGU_REG_I2SCDR);

	return (i2scdr & I2SCDR_I2CS_MASK) >> I2SCDR_I2CS_SHIFT;
}

static int x1000_i2s_set_parent(struct clk_hw *hw, u8 idx)
{
	unsigned long flags;
	u32 i2scdr;

	if (idx > 1)
		return -EINVAL;

	spin_lock_irqsave(&cgu->lock, flags);

	i2scdr = readl(cgu->base + CGU_REG_I2SCDR);
	i2scdr &= ~I2SCDR_I2CS_MASK;
	i2scdr |= idx << I2SCDR_I2CS_SHIFT;
	writel(i2scdr, cgu->base + CGU_REG_I2SCDR);

	spin_unlock_irqrestore(&cgu->lock, flags);

	return 0;
}

static int x1000_i2s_enable(struct clk_hw *hw)
{
	u32 i2scdr;

	i2scdr = readl(cgu->base + CGU_REG_I2SCDR);

	if (i2scdr & I2SCDR_I2CS_MASK)
		writel(readl(cgu->base + CGU_REG_I2SCDR1), cgu->base + CGU_REG_I2SCDR1);

	return 0;
}

static const struct clk_ops x1000_i2s_ops = {
	.get_parent = x1000_i2s_get_parent,
	.set_parent = x1000_i2s_set_parent,

	.enable = x1000_i2s_enable,
};

static const struct ingenic_cgu_clk_info x1000_cgu_clocks[] = {

	/* External clocks */

	[X1000_CLK_EXCLK] = { "ext", CGU_CLK_EXT },
	[X1000_CLK_RTCLK] = { "rtc", CGU_CLK_EXT },

	/* PLLs */

	[X1000_CLK_APLL] = {
		"apll", CGU_CLK_PLL,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_APLL,
			.rate_multiplier = 1,
			.m_shift = 24,
			.m_bits = 7,
			.m_offset = 1,
			.n_shift = 18,
			.n_bits = 5,
			.n_offset = 1,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 8,
			.od_encoding = pll_od_encoding,
			.bypass_reg = CGU_REG_APLL,
			.bypass_bit = 9,
			.enable_bit = 8,
			.stable_bit = 10,
		},
	},

	[X1000_CLK_MPLL] = {
		"mpll", CGU_CLK_PLL,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_MPLL,
			.rate_multiplier = 1,
			.m_shift = 24,
			.m_bits = 7,
			.m_offset = 1,
			.n_shift = 18,
			.n_bits = 5,
			.n_offset = 1,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 8,
			.od_encoding = pll_od_encoding,
			.bypass_reg = CGU_REG_MPLL,
			.bypass_bit = 6,
			.enable_bit = 7,
			.stable_bit = 0,
		},
	},

	[X1000_CLK_I2SPLL] = {
		"i2s_pll", CGU_CLK_PLL | CGU_CLK_MUX,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL },
		.mux = { CGU_REG_I2SCDR, 31, 1 },
		.pll = {
			.reg = CGU_REG_I2SCDR,
			.rate_multiplier = 1,
			.m_shift = 13,
			.m_bits = 9,
			.m_offset = 0,
			.n_shift = 0,
			.n_bits = 13,
			.n_offset = 0,
			.bypass_bit = -1,
			.enable_bit = 29,
			.stable_bit = -1,
			.calc_m_n_od = x1000_i2s_calc_m_n,
		},
	},

	/* Custom (SoC-specific) OTG PHY */

	[X1000_CLK_OTGPHY] = {
		"otg_phy", CGU_CLK_CUSTOM,
		.parents = { -1, -1, X1000_CLK_EXCLK, -1 },
		.custom = { &x1000_otg_phy_ops },
	},

	/* Muxes & dividers */

	[X1000_CLK_SCLKA] = {
		"sclk_a", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_EXCLK, X1000_CLK_APLL, -1 },
		.mux = { CGU_REG_CPCCR, 30, 2 },
	},

	[X1000_CLK_CPUMUX] = {
		"cpu_mux", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 28, 2 },
	},

	[X1000_CLK_CPU] = {
		"cpu", CGU_CLK_DIV | CGU_CLK_GATE,
		/*
		 * Disabling the CPU clock or any parent clocks will hang the
		 * system; mark it critical.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { X1000_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CPCCR, 0, 1, 4, 22, -1, -1 },
		.gate = { CGU_REG_CLKGR, 30 },
	},

	[X1000_CLK_L2CACHE] = {
		"l2cache", CGU_CLK_DIV,
		/*
		 * The L2 cache clock is critical if caches are enabled and
		 * disabling it or any parent clocks will hang the system.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { X1000_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CPCCR, 4, 1, 4, 22, -1, -1 },
	},

	[X1000_CLK_AHB0] = {
		"ahb0", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 26, 2 },
		.div = { CGU_REG_CPCCR, 8, 1, 4, 21, -1, -1 },
	},

	[X1000_CLK_AHB2PMUX] = {
		"ahb2_apb_mux", CGU_CLK_MUX,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 24, 2 },
	},

	[X1000_CLK_AHB2] = {
		"ahb2", CGU_CLK_DIV,
		.parents = { X1000_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CPCCR, 12, 1, 4, 20, -1, -1 },
	},

	[X1000_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X1000_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CPCCR, 16, 1, 4, 20, -1, -1 },
		.gate = { CGU_REG_CLKGR, 28 },
	},

	[X1000_CLK_DDR] = {
		"ddr", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		/*
		 * Disabling DDR clock or its parents will render DRAM
		 * inaccessible; mark it critical.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { -1, X1000_CLK_SCLKA, X1000_CLK_MPLL, -1 },
		.mux = { CGU_REG_DDRCDR, 30, 2 },
		.div = { CGU_REG_DDRCDR, 0, 1, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 31 },
	},

	[X1000_CLK_MACPHY] = {
		"mac_phy", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL },
		.mux = { CGU_REG_MACCDR, 31, 1 },
		.div = { CGU_REG_MACCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X1000_CLK_MAC] = {
		"mac", CGU_CLK_GATE,
		.parents = { X1000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR, 25 },
	},

	[X1000_CLK_LCD] = {
		"lcd", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL },
		.mux = { CGU_REG_LPCDR, 31, 1 },
		.div = { CGU_REG_LPCDR, 0, 1, 8, 28, 27, 26 },
		.gate = { CGU_REG_CLKGR, 23 },
	},

	[X1000_CLK_MSCMUX] = {
		"msc_mux", CGU_CLK_MUX,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL},
		.mux = { CGU_REG_MSC0CDR, 31, 1 },
	},

	[X1000_CLK_MSC0] = {
		"msc0", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X1000_CLK_MSCMUX, -1, -1, -1 },
		.div = { CGU_REG_MSC0CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 4 },
	},

	[X1000_CLK_MSC1] = {
		"msc1", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X1000_CLK_MSCMUX, -1, -1, -1 },
		.div = { CGU_REG_MSC1CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 5 },
	},

	[X1000_CLK_OTG] = {
		"otg", CGU_CLK_DIV | CGU_CLK_GATE | CGU_CLK_MUX,
		.parents = { X1000_CLK_EXCLK, -1,
					 X1000_CLK_APLL, X1000_CLK_MPLL },
		.mux = { CGU_REG_USBCDR, 30, 2 },
		.div = { CGU_REG_USBCDR, 0, 1, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR, 3 },
	},

	[X1000_CLK_SSIPLL] = {
		"ssi_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL, -1, -1 },
		.mux = { CGU_REG_SSICDR, 31, 1 },
		.div = { CGU_REG_SSICDR, 0, 1, 8, 29, 28, 27 },
	},

	[X1000_CLK_SSIPLL_DIV2] = {
		"ssi_pll_div2", CGU_CLK_FIXDIV,
		.parents = { X1000_CLK_SSIPLL },
		.fixdiv = { 2 },
	},

	[X1000_CLK_SSIMUX] = {
		"ssi_mux", CGU_CLK_MUX,
		.parents = { X1000_CLK_EXCLK, X1000_CLK_SSIPLL_DIV2, -1, -1 },
		.mux = { CGU_REG_SSICDR, 30, 1 },
	},

	[X1000_CLK_CIM] = {
		"cim", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X1000_CLK_SCLKA, X1000_CLK_MPLL, -1, -1 },
		.mux = { CGU_REG_CIMCDR, 31, 1 },
		.div = { CGU_REG_CIMCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X1000_CLK_EXCLK_DIV512] = {
		"exclk_div512", CGU_CLK_FIXDIV,
		.parents = { X1000_CLK_EXCLK },
		.fixdiv = { 512 },
	},

	[X1000_CLK_RTC] = {
		"rtc_ercs", CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK_DIV512, X1000_CLK_RTCLK },
		.mux = { CGU_REG_OPCR, 2, 1},
		.gate = { CGU_REG_CLKGR, 27 },
	},

	[X1000_CLK_I2S] = {
		"i2s", CGU_CLK_CUSTOM,
		.parents = { X1000_CLK_EXCLK, X1000_CLK_I2SPLL, -1, -1 },
		.custom = { &x1000_i2s_ops },
	},

	/* Gate-only clocks */

	[X1000_CLK_EMC] = {
		"emc", CGU_CLK_GATE,
		.parents = { X1000_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 0 },
	},

	[X1000_CLK_EFUSE] = {
		"efuse", CGU_CLK_GATE,
		.parents = { X1000_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 1 },
	},

	[X1000_CLK_SFC] = {
		"sfc", CGU_CLK_GATE,
		.parents = { X1000_CLK_SSIPLL, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 2 },
	},

	[X1000_CLK_I2C0] = {
		"i2c0", CGU_CLK_GATE,
		.parents = { X1000_CLK_PCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 7 },
	},

	[X1000_CLK_I2C1] = {
		"i2c1", CGU_CLK_GATE,
		.parents = { X1000_CLK_PCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 8 },
	},

	[X1000_CLK_I2C2] = {
		"i2c2", CGU_CLK_GATE,
		.parents = { X1000_CLK_PCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 9 },
	},

	[X1000_CLK_AIC] = {
		"aic", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 11 },
	},

	[X1000_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 14 },
	},

	[X1000_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 15 },
	},

	[X1000_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 16 },
	},

	[X1000_CLK_DMIC] = {
		"dmic", CGU_CLK_GATE,
		.parents = { X1000_CLK_PCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 17 },
	},

	[X1000_CLK_TCU] = {
		"tcu", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 18 },
	},

	[X1000_CLK_SSI] = {
		"ssi", CGU_CLK_GATE,
		.parents = { X1000_CLK_SSIMUX, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 19 },
	},

	[X1000_CLK_OST] = {
		"ost", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 20 },
	},

	[X1000_CLK_PDMA] = {
		"pdma", CGU_CLK_GATE,
		.parents = { X1000_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR, 21 },
	},
};

static void __init x1000_cgu_init(struct device_node *np)
{
	int retval;

	memset(pll_cache, 0, sizeof(pll_cache));

	cgu = ingenic_cgu_new(x1000_cgu_clocks,
			      ARRAY_SIZE(x1000_cgu_clocks), np);
	if (!cgu) {
		pr_err("%s: failed to initialise CGU\n", __func__);
		return;
	}

	// The POR value of I2SCDR does NOT comfort to datasheet
	// specs and WILL confuse clk_get_parent() calls.
	writel(0x0, cgu->base + CGU_REG_I2SCDR);

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval) {
		pr_err("%s: failed to register CGU Clocks\n", __func__);
		return;
	}

	ingenic_cgu_register_syscore_ops(cgu);
}
/*
 * CGU has some children devices, this is useful for probing children devices
 * in the case where the device node is compatible with "simple-mfd".
 */
CLK_OF_DECLARE_DRIVER(x1000_cgu, "ingenic,x1000-cgu", x1000_cgu_init);
