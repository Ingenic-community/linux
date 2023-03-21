// SPDX-License-Identifier: GPL-2.0
/*
 * X2000 SoC CGU driver
 * Copyright (C) 2020 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>

#include <dt-bindings/clock/x2000-cgu.h>

#include "cgu.h"
#include "pm.h"

#define MHZ (1000 * 1000)

/* CGU register offsets */
#define CGU_REG_CPCCR			0x00
#define CGU_REG_LCR				0x04
#define CGU_REG_CPPCR			0x0c
#define CGU_REG_CPAPCR			0x10
#define CGU_REG_CPMPCR			0x14
#define CGU_REG_CPEPCR			0x18
#define CGU_REG_CLKGR0			0x20
#define CGU_REG_OPCR			0x24
#define CGU_REG_CLKGR1			0x28
#define CGU_REG_DDRCDR			0x2c
#define CGU_REG_ISPCDR			0x30
#define CGU_REG_CPSPR			0x34
#define CGU_REG_CPSPPR			0x38
#define CGU_REG_USBPCR			0x3c
#define CGU_REG_USBRDT			0x40
#define CGU_REG_USBVBFIL		0x44
#define CGU_REG_USBPCR1			0x48
#define CGU_REG_MACPTPCDR		0x4c
#define CGU_REG_RSACDR			0x50
#define CGU_REG_MACCDR			0x54
#define CGU_REG_MAC0TXCDR		0x58
#define CGU_REG_SSICDR			0x5c
#define CGU_REG_I2S0CDR			0x60
#define CGU_REG_LPCDR			0x64
#define CGU_REG_MSC0CDR			0x68
#define CGU_REG_PWMCDR			0x6c
#define CGU_REG_I2S0CDR1		0x70
#define CGU_REG_SFCCDR			0x74
#define CGU_REG_CIMCDR			0x78
#define CGU_REG_I2S1CDR			0x7c
#define CGU_REG_I2S1CDR1		0x80
#define CGU_REG_I2S2CDR			0x84
#define CGU_REG_I2S2CDR1		0x88
#define CGU_REG_I2S3CDR			0x8c
#define CGU_REG_PSWC0ST			0x90
#define CGU_REG_PSWC1ST			0x94
#define CGU_REG_PSWC2ST			0x98
#define CGU_REG_PSWC3ST			0x9c
#define CGU_REG_I2S3CDR1		0xa0
#define CGU_REG_MSC1CDR			0xa4
#define CGU_REG_MSC2CDR			0xa8
#define CGU_REG_AUDIOCR			0xac
#define CGU_REG_CMP_INTR		0xb0
#define CGU_REG_CMP_INTRE		0xb4
#define CGU_REG_CMP_SFTINT		0xbc
#define CGU_REG_SRBC			0xc4
#define CGU_REG_SLBC			0xc8
#define CGU_REG_SLPC			0xcc
#define CGU_REG_DRCG			0xd0
#define CGU_REG_CPCSR			0xd4
#define CGU_REG_MAC1TXCDR		0xdc
#define CGU_REG_MAC0PHYC		0xe4
#define CGU_REG_MAC1PHYC		0xe8
#define CGU_REG_MESTSEL			0xec
#define CGU_REG_MEMPD0			0xf8
#define CGU_REG_MEMPD1			0xfc

/* bits within the CPCCR register */
#define CPCCR_L2CDIV_MASK		GENMASK(7, 4)
#define CPCCR_CDIV_MASK			GENMASK(3, 0)

/* bits within the OPCR register */
#define OPCR_GATE_USBPHYCLK		BIT(23)
#define OPCR_SPENDN				BIT(7)

/* bits within the CLKGR1 register */
#define CLKGR1_I2S0				BIT(8)
#define CLKGR1_I2S1				BIT(9)
#define CLKGR1_I2S2				BIT(10)
#define CLKGR1_I2S3				BIT(11)

static struct ingenic_cgu *cgu;

static void x2000_cgu_calc_m_n_od(const struct ingenic_cgu_pll_info *pll_info,
		       unsigned long rate, unsigned long parent_rate,
		       unsigned int *pm, unsigned int *pn, unsigned int *pod)
{
	unsigned int m, n, od;
	unsigned int m_max = 1 << pll_info->m_bits;
	unsigned int n_max = 1 << pll_info->n_bits;
	unsigned int multiplier = pll_info->rate_multiplier;

	/*
	 * The frequency after the input divider must be between 5 and 200 MHz.
	 * The highest divider yields the best resolution.
	 */
	n = parent_rate / (5 * MHZ);

	/* The value of N must be at least 1. */
	n = clamp_val(n, 1, n_max);

	/*
	 * The frequency of the VCO must be not higher than 4800 MHz.
	 * The highest divider yields the best resolution.
	 */
	od = 4800ul / (rate / MHZ);
	od = min_t(unsigned int, 1 << __fls(od), pll_info->od_max);

	do {
		m = (rate / MHZ) * od * n / (multiplier * (parent_rate / MHZ));

		if (m >= 1 && m <= m_max)
			break;
	} while (--n);

	*pm = m;
	*pn = n;
	*pod = od;
}

static void x2000_i2s_calc_m_n(const struct ingenic_cgu_pll_info *pll_info,
		       unsigned long rate, unsigned long parent_rate,
		       unsigned int *pm, unsigned int *pn, unsigned int *pod)
{
	unsigned int m, n, rem;
	u64 curr_m, curr_n, curr_rem;

	if ((parent_rate % rate == 0) && ((parent_rate / rate) > 1)) {
		m = 1;
		n = parent_rate / rate;
		goto out;
	}

	rem = 0xffffffff;

	/*
	 * The length of M is 9 bits, its maximum value is 511.
	 * The length of N is 20 bits, its maximum value is 1048575.
	 */
	curr_m = min_t(u64, rate * 1048575 / parent_rate + 1, 511);

	/* The value of M must be no less than 1 */
	for (; curr_m >= 1; curr_m--) {
		curr_n = parent_rate * curr_m;
		curr_rem = do_div(curr_n, rate);

		/*
		 * When the remainer value is greater than the half
		 * of the divisor value, use the quotient value + 1
		 * as the N value to obtain the smallest deviation.
		 */
		if (curr_rem * 2 > rate) {
			curr_rem = rate - curr_rem;
			curr_n += 1;
		}

		/*
		 * The value of N must be no less than 2 times of the
		 * value of M, and no greater than 1048575.
		 */
		if (curr_n >= 2 * curr_m && curr_n <= 1048575) {

			/*
			 * To determine whether a better set of parameters
			 * has been found by comparing the quotient values
			 * of curr_rem ÷ curr_n and rem ÷ n.
			 */
			if (curr_rem * n < rem * curr_n) {
				m = curr_m;
				n = curr_n;

				if (curr_rem)
					rem = curr_rem;
				else
					break;
			}
		}
	}

out:
	*pm = m;
	*pn = n;

	/*
	 * The I2S PLL does not have any OD bits, so set the *pod to 1 to
	 * ensure that the ingenic_pll_calc() in cgu.c can run properly.
	 */
	*pod = 1;
}

static unsigned long x2000_cpu_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return parent_rate / ((readl(cgu->base + CGU_REG_CPCCR) & 0xf) + 1);
}

static long x2000_cpu_round_rate(struct clk_hw *hw, unsigned long req_rate,
				      unsigned long *parent_rate)
{
	return *parent_rate / ((*parent_rate - 1) / req_rate + 1);
}

static int x2000_cpu_set_rate(struct clk_hw *hw, unsigned long req_rate, unsigned long parent_rate)
{
	unsigned int div;
	u32 cpccr;

	div = parent_rate / req_rate;
	div = min_t(unsigned int, div, 16);

	cpccr = readl(cgu->base + CGU_REG_CPCCR);

	if (req_rate <= 750 * MHZ) {
		u32p_replace_bits(&cpccr, div - 1, CPCCR_L2CDIV_MASK);
		u32p_replace_bits(&cpccr, div - 1, CPCCR_CDIV_MASK);
		writel(cpccr | (div - 1) | (div - 1) << 4, cgu->base + CGU_REG_CPCCR);
	} else {
		u32p_replace_bits(&cpccr, div - 1, CPCCR_L2CDIV_MASK);
		u32p_replace_bits(&cpccr, div - 1, CPCCR_CDIV_MASK);
		writel(cpccr | (div - 1) | (div * 2 - 1) << 4, cgu->base + CGU_REG_CPCCR);
	}

	return 0;
}

static const struct clk_ops x2000_cpu_ops = {
	.recalc_rate = x2000_cpu_recalc_rate,
	.round_rate = x2000_cpu_round_rate,
	.set_rate = x2000_cpu_set_rate,
};

static int x2000_usb_phy_enable(struct clk_hw *hw)
{
	void __iomem *reg_opcr	= cgu->base + CGU_REG_OPCR;

	writel((readl(reg_opcr) | OPCR_SPENDN) & ~OPCR_GATE_USBPHYCLK, reg_opcr);

	return 0;
}

static void x2000_usb_phy_disable(struct clk_hw *hw)
{
	void __iomem *reg_opcr	= cgu->base + CGU_REG_OPCR;

	writel((readl(reg_opcr) & ~OPCR_SPENDN) | OPCR_GATE_USBPHYCLK, reg_opcr);
}

static int x2000_usb_phy_is_enabled(struct clk_hw *hw)
{
	void __iomem *reg_opcr	= cgu->base + CGU_REG_OPCR;

	return !!(readl(reg_opcr) & OPCR_SPENDN);
}

static const struct clk_ops x2000_otg_phy_ops = {
	.enable = x2000_usb_phy_enable,
	.disable = x2000_usb_phy_disable,
	.is_enabled	= x2000_usb_phy_is_enabled,
};

static const s8 pll_od_encoding[64] = {
	 -1, 0x1,  -1, 0x2,  -1,  -1,  -1, 0x3,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1, 0x4,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1, 0x5,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
	 -1,  -1,  -1,  -1,  -1,  -1,  -1, 0x6,
};

static const struct ingenic_cgu_clk_info x2000_cgu_clocks[] = {

	/* External clocks */

	[X2000_CLK_EXCLK] = { "ext", CGU_CLK_EXT },
	[X2000_CLK_RTCLK] = { "rtc", CGU_CLK_EXT },

	/* PLLs */

	[X2000_CLK_APLL] = {
		"apll", CGU_CLK_PLL,
		.parents = { X2000_CLK_EXCLK, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_CPAPCR,
			.rate_multiplier = 2,
			.m_shift = 20,
			.m_bits = 9,
			.m_offset = 1,
			.n_shift = 14,
			.n_bits = 6,
			.n_offset = 1,
			.od_shift = 11,
			.od_bits = 3,
			.od_max = 64,
			.od_encoding = pll_od_encoding,
			.bypass_reg = CGU_REG_CPPCR,
			.bypass_bit = 30,
			.enable_bit = 0,
			.stable_bit = 3,
			.calc_m_n_od = x2000_cgu_calc_m_n_od,
		},
	},

	[X2000_CLK_MPLL] = {
		"mpll", CGU_CLK_PLL,
		.parents = { X2000_CLK_EXCLK, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_CPMPCR,
			.rate_multiplier = 2,
			.m_shift = 20,
			.m_bits = 10,
			.m_offset = 1,
			.n_shift = 14,
			.n_bits = 6,
			.n_offset = 1,
			.od_shift = 11,
			.od_bits = 3,
			.od_max = 64,
			.od_encoding = pll_od_encoding,
			.bypass_reg = CGU_REG_CPPCR,
			.bypass_bit = 28,
			.enable_bit = 0,
			.stable_bit = 3,
			.calc_m_n_od = x2000_cgu_calc_m_n_od,
		},
	},

	[X2000_CLK_EPLL] = {
		"epll", CGU_CLK_PLL,
		.parents = { X2000_CLK_EXCLK, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_CPEPCR,
			.rate_multiplier = 2,
			.m_shift = 20,
			.m_bits = 10,
			.m_offset = 1,
			.n_shift = 14,
			.n_bits = 6,
			.n_offset = 1,
			.od_shift = 11,
			.od_bits = 3,
			.od_max = 64,
			.od_encoding = pll_od_encoding,
			.bypass_reg = CGU_REG_CPPCR,
			.bypass_bit = 26,
			.enable_bit = 0,
			.stable_bit = 3,
			.calc_m_n_od = x2000_cgu_calc_m_n_od,
		},
	},

	[X2000_CLK_I2S0] = {
		"i2s0", CGU_CLK_PLL,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_EPLL, -1, -1 },
		.pll = {
			.reg = CGU_REG_I2S0CDR,
			.rate_multiplier = 1,
			.m_shift = 20,
			.m_bits = 9,
			.m_offset = 0,
			.n_shift = 0,
			.n_bits = 20,
			.n_offset = 0,
			.bypass_bit = -1,
			.enable_bit = 29,
			.stable_bit = -1,
			.calc_m_n_od = x2000_i2s_calc_m_n,
		},
	},

	[X2000_CLK_I2S1] = {
		"i2s1", CGU_CLK_PLL,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_EPLL, -1, -1 },
		.pll = {
			.reg = CGU_REG_I2S1CDR,
			.rate_multiplier = 1,
			.m_shift = 20,
			.m_bits = 9,
			.m_offset = 0,
			.n_shift = 0,
			.n_bits = 20,
			.n_offset = 0,
			.bypass_bit = -1,
			.enable_bit = 29,
			.stable_bit = -1,
			.calc_m_n_od = x2000_i2s_calc_m_n,
		},
	},

	[X2000_CLK_I2S2] = {
		"i2s2", CGU_CLK_PLL,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_EPLL, -1, -1 },
		.pll = {
			.reg = CGU_REG_I2S2CDR,
			.rate_multiplier = 1,
			.m_shift = 20,
			.m_bits = 9,
			.m_offset = 0,
			.n_shift = 0,
			.n_bits = 20,
			.n_offset = 0,
			.bypass_bit = -1,
			.enable_bit = 29,
			.stable_bit = -1,
			.calc_m_n_od = x2000_i2s_calc_m_n,
		},
	},

	[X2000_CLK_I2S3] = {
		"i2s3", CGU_CLK_PLL,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_EPLL, -1, -1 },
		.pll = {
			.reg = CGU_REG_I2S3CDR,
			.rate_multiplier = 1,
			.m_shift = 20,
			.m_bits = 9,
			.m_offset = 0,
			.n_shift = 0,
			.n_bits = 20,
			.n_offset = 0,
			.bypass_bit = -1,
			.enable_bit = 29,
			.stable_bit = -1,
			.calc_m_n_od = x2000_i2s_calc_m_n,
		},
	},

	/* Custom (SoC-specific) OTG PHY */

	[X2000_CLK_CPU] = {
		"cpu", CGU_CLK_CUSTOM,
		.flags = CLK_IS_CRITICAL,
		.parents = { X2000_CLK_CPUMUX, -1, -1, -1 },
		.custom = { &x2000_cpu_ops },
	},

	[X2000_CLK_OTGPHY] = {
		"otg_phy", CGU_CLK_CUSTOM,
		.parents = { X2000_CLK_EXCLK, -1, -1, -1 },
		.custom = { &x2000_otg_phy_ops },
	},

	/* Muxes & dividers */

	[X2000_CLK_SCLKA] = {
		"sclk_a", CGU_CLK_MUX,
		.parents = { -1, X2000_CLK_EXCLK, X2000_CLK_APLL, -1 },
		.mux = { CGU_REG_CPCCR, 30, 2 },
	},

	[X2000_CLK_CPUMUX] = {
		"cpu_mux", CGU_CLK_MUX,
		.parents = { -1, X2000_CLK_SCLKA, X2000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 28, 2 },
	},

	[X2000_CLK_L2CACHE] = {
		"l2cache", CGU_CLK_DIV,
		.flags = CLK_IS_CRITICAL,
		.parents = { X2000_CLK_CPUMUX },
		.div = { CGU_REG_CPCCR, 4, 1, 4, 22, -1, -1 },
	},

	[X2000_CLK_AHB0] = {
		"ahb0", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { -1, X2000_CLK_SCLKA, X2000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 26, 2 },
		.div = { CGU_REG_CPCCR, 8, 1, 4, 21, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 29 },
	},

	[X2000_CLK_AHB2PMUX] = {
		"ahb2_apb_mux", CGU_CLK_MUX,
		.parents = { -1, X2000_CLK_SCLKA, X2000_CLK_MPLL, -1 },
		.mux = { CGU_REG_CPCCR, 24, 2 },
	},

	[X2000_CLK_AHB2] = {
		"ahb2", CGU_CLK_DIV,
		.parents = { X2000_CLK_AHB2PMUX },
		.div = { CGU_REG_CPCCR, 12, 1, 4, 20, -1, -1 },
	},

	[X2000_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2PMUX },
		.div = { CGU_REG_CPCCR, 16, 1, 4, 20, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 28 },
	},

	[X2000_CLK_DDR] = {
		"ddr", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.flags = CLK_IS_CRITICAL,
		.parents = { -1, X2000_CLK_SCLKA, X2000_CLK_MPLL, -1 },
		.mux = { CGU_REG_DDRCDR, 30, 2 },
		.div = { CGU_REG_DDRCDR, 0, 1, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 31 },
	},

	[X2000_CLK_ISP] = {
		"isp", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_ISPCDR, 30, 2 },
		.div = { CGU_REG_ISPCDR, 0, 1, 4, 29, 28, 27 },
	},

	[X2000_CLK_MACPTP] = {
		"mac_ptp", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_MACPTPCDR, 30, 2 },
		.div = { CGU_REG_MACPTPCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X2000_CLK_MACPHY] = {
		"mac_phy", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_MACCDR, 30, 2 },
		.div = { CGU_REG_MACCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X2000_CLK_MAC0TX] = {
		"mac0_tx", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_MAC0TXCDR, 30, 2 },
		.div = { CGU_REG_MAC0TXCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X2000_CLK_MAC1TX] = {
		"mac1_tx", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_MAC1TXCDR, 30, 2 },
		.div = { CGU_REG_MAC1TXCDR, 0, 1, 8, 29, 28, 27 },
	},

	[X2000_CLK_RSA] = {
		"rsa", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EXCLK, -1 },
		.mux = { CGU_REG_LPCDR, 30, 2 },
		.div = { CGU_REG_LPCDR, 0, 1, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 25 },
	},

	[X2000_CLK_SSIPLL] = {
		"ssi_pll", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_SSICDR, 30, 2 },
		.div = { CGU_REG_SSICDR, 0, 1, 8, 29, 28, 27 },
	},

	[X2000_CLK_LCD] = {
		"lcd", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EXCLK, -1 },
		.mux = { CGU_REG_LPCDR, 30, 2 },
		.div = { CGU_REG_LPCDR, 0, 1, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 23 },
	},

	[X2000_CLK_MSC_EXCLK] = {
		"msc_exclk", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_MSC0CDR, 21, true },
	},

	[X2000_CLK_MSC0] = {
		"msc0", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_MSC_EXCLK, -1 },
		.mux = { CGU_REG_MSC0CDR, 30, 2 },
		.div = { CGU_REG_MSC0CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 4 },
	},

	[X2000_CLK_MSC1] = {
		"msc1", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_MSC_EXCLK, -1 },
		.mux = { CGU_REG_MSC1CDR, 30, 2 },
		.div = { CGU_REG_MSC1CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 5 },
	},

	[X2000_CLK_MSC2] = {
		"msc2", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_MSC_EXCLK, -1 },
		.mux = { CGU_REG_MSC2CDR, 30, 2 },
		.div = { CGU_REG_MSC2CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR1, 25 },
	},

	[X2000_CLK_PWM] = {
		"pwm", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_PWMCDR, 30, 2 },
		.div = { CGU_REG_PWMCDR, 0, 1, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR1, 5 },
	},

	[X2000_CLK_SFC] = {
		"sfc", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_SFCCDR, 30, 2 },
		.div = { CGU_REG_SFCCDR, 0, 1, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 2 },
	},

	[X2000_CLK_CIM] = {
		"cim", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2000_CLK_SCLKA, X2000_CLK_MPLL, X2000_CLK_EPLL, -1 },
		.mux = { CGU_REG_CIMCDR, 30, 2 },
		.div = { CGU_REG_CIMCDR, 0, 1, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 22 },
	},

	[X2000_CLK_DMIC_EXCLK] = {
		"dmic_exclk", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_AUDIOCR, 31 },
	},

	[X2000_CLK_DMIC] = {
		"dmic", CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { X2000_CLK_DMIC_EXCLK, X2000_CLK_I2S3 },
		.mux = { CGU_REG_AUDIOCR, 0, 1},
		.gate = { CGU_REG_CLKGR1, 13 },
	},

	[X2000_CLK_EXCLK_DIV512] = {
		"exclk_div512", CGU_CLK_FIXDIV,
		.parents = { X2000_CLK_EXCLK },
		.fixdiv = { 512 },
	},

	[X2000_CLK_RTC] = {
		"rtc_ercs", CGU_CLK_MUX | CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK_DIV512, X2000_CLK_RTCLK },
		.mux = { CGU_REG_OPCR, 2, 1},
		.gate = { CGU_REG_CLKGR0, 27 },
	},

	/* Gate-only clocks */

	[X2000_CLK_EMC] = {
		"emc", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR0, 0 },
	},

	[X2000_CLK_EFUSE] = {
		"efuse", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR0, 1 },
	},

	[X2000_CLK_OTG] = {
		"otg", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 3 },
	},

	[X2000_CLK_SCC] = {
		"scc", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 6 },
	},

	[X2000_CLK_I2C0] = {
		"i2c0", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR0, 7 },
	},

	[X2000_CLK_I2C1] = {
		"i2c1", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR0, 8 },
	},

	[X2000_CLK_I2C2] = {
		"i2c2", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR0, 9 },
	},

	[X2000_CLK_I2C3] = {
		"i2c3", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR0, 10 },
	},

	[X2000_CLK_SADC] = {
		"sadc", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 13 },
	},

	[X2000_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 14 },
	},

	[X2000_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 15 },
	},

	[X2000_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 16 },
	},

	[X2000_CLK_DTRNG] = {
		"dtrng", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR0, 17 },
	},

	[X2000_CLK_TCU] = {
		"tcu", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 18 },
	},

	[X2000_CLK_SSI0] = {
		"ssi0", CGU_CLK_GATE,
		.parents = { X2000_CLK_SSIPLL },
		.gate = { CGU_REG_CLKGR0, 19 },
	},

	[X2000_CLK_OST] = {
		"ost", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 20 },
	},

	[X2000_CLK_PDMA] = {
		"pdma", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR0, 21 },
	},

	[X2000_CLK_SSI1] = {
		"ssi1", CGU_CLK_GATE,
		.parents = { X2000_CLK_SSIPLL },
		.gate = { CGU_REG_CLKGR0, 26 },
	},

	[X2000_CLK_I2C4] = {
		"i2c4", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR1, 0 },
	},

	[X2000_CLK_I2C5] = {
		"i2c5", CGU_CLK_GATE,
		.parents = { X2000_CLK_PCLK },
		.gate = { CGU_REG_CLKGR1, 1 },
	},

	[X2000_CLK_ISP0] = {
		"isp0", CGU_CLK_GATE,
		.parents = { X2000_CLK_ISP },
		.gate = { CGU_REG_CLKGR1, 2 },
	},

	[X2000_CLK_ISP1] = {
		"isp1", CGU_CLK_GATE,
		.parents = { X2000_CLK_ISP },
		.gate = { CGU_REG_CLKGR1, 3 },
	},

	[X2000_CLK_HASH] = {
		"hash", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR1, 6 },
	},

	[X2000_CLK_UART3] = {
		"uart3", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 16 },
	},

	[X2000_CLK_UART4] = {
		"uart4", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 17 },
	},

	[X2000_CLK_UART5] = {
		"uart5", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 18 },
	},

	[X2000_CLK_UART6] = {
		"uart6", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 19 },
	},

	[X2000_CLK_UART7] = {
		"uart7", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 20 },
	},

	[X2000_CLK_UART8] = {
		"uart8", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 21 },
	},

	[X2000_CLK_UART9] = {
		"uart9", CGU_CLK_GATE,
		.parents = { X2000_CLK_EXCLK },
		.gate = { CGU_REG_CLKGR1, 22 },
	},

	[X2000_CLK_MAC0] = {
		"mac0", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR1, 23 },
	},

	[X2000_CLK_MAC1] = {
		"mac1", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR1, 24 },
	},

	[X2000_CLK_INTC] = {
		"intc", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB2 },
		.gate = { CGU_REG_CLKGR1, 26 },
	},

	[X2000_CLK_CSI] = {
		"csi", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB0 },
		.gate = { CGU_REG_CLKGR1, 28 },
	},

	[X2000_CLK_DSI] = {
		"dsi", CGU_CLK_GATE,
		.parents = { X2000_CLK_AHB0 },
		.gate = { CGU_REG_CLKGR1, 29 },
	},
};

static void __init x2000_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(x2000_cgu_clocks,
			      ARRAY_SIZE(x2000_cgu_clocks), np);
	if (!cgu) {
		pr_err("%s: failed to initialise CGU\n", __func__);
		return;
	}

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
CLK_OF_DECLARE_DRIVER(x2000_cgu, "ingenic,x2000-cgu", x2000_cgu_init);
