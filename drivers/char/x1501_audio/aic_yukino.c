// SPDX-License-Identifier: GPL-2.0 or BSD-3-Clause
//
// Lightweight OSS audio driver for X1501
//
// Copyright (C) 2023, Yukino Song <yukino@sudomaker.com>

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/soundcard.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/iopoll.h>

#include <linux/dma-mapping.h>

#include "aic_regs.h"
#include "codec_regs.h"

#define NR_PERIODS		4

struct period {
	unsigned buf_pos;
	struct period* next;
};
typedef struct period period;

struct audio_buffer {
	unsigned len;
	unsigned len_period;

	uint8_t *buf;
	unsigned cursor;

	uint8_t user_written;

	period periods[NR_PERIODS];
	period* period_in_use_by_dma;

	dma_addr_t buf_phys;
	struct dma_chan *dma_ch;
	struct dma_async_tx_descriptor *dma_desc;

	struct completion compl_buf_period;
};

struct x1501_audio {
	struct resource *mem;
	void __iomem *base;
	dma_addr_t phys_base;

	struct regmap *codec_regmap;

	struct clk *clk_aic;
	struct clk *clk_i2s;

	struct audio_buffer abuf_tx, abuf_rx;

	bool opened;
	bool playing;
	bool recording;

	u8 channels;
	u32 format;
	u32 rate;
};

static struct x1501_audio priv;


static inline uint32_t aic_readl(unsigned int reg)
{
	return readl(priv.base + reg);
}

static inline void aic_writel(unsigned int reg, uint32_t value)
{
	writel(value, priv.base + reg);

	if (reg == JZ_REG_AIC_RGADW || reg == JZ_REG_AIC_RGDATA)
		printk("------ aregW RG: 0x%02x 0x%08x\n", reg, value);
	else
		printk("-- aregW: 0x%02x 0x%08x\n", reg, value);
}

static int codec_reg_read(void *context, unsigned int reg, unsigned int *val);
static int codec_reg_write(void *context, unsigned int reg, unsigned int val);

static bool codec_volatile(struct device *dev, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return true;

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_IFR:
	case SCODA_REG_IFR2:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return true;
	default:
		return false;
	}
}

static bool codec_readable(struct device *dev, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return false;
	else
		return true;
}

static bool codec_writeable(struct device *dev, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return false;

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return false;
	default:
		return true;
	}
}

static int codec_io_wait(void)
{
	u32 reg;

	return readl_poll_timeout(priv.base + JZ_REG_AIC_RGADW, reg,
				  !(reg & ICDC_RGADW_RGWR),
				  1000, 1 * USEC_PER_SEC);
}

static int icdc_d3_hw_read_extend(unsigned int sreg, unsigned int *val)
{
	int creg, cdata, dreg;
	switch (sreg) {
		case SCODA_MIX_0 ... SCODA_MIX_4:
			creg = SCODA_REG_CR_MIX;
			dreg = SCODA_REG_DR_MIX;
			sreg -= SCODA_MIX_0;
			break;
		case SCODA_DAC_AGC0 ... SCODA_DAC_AGC3:
			creg = SCODA_REG_CR_DAC_AGC;
			dreg = SCODA_REG_DR_DAC_AGC;
			sreg -= SCODA_DAC_AGC0;
			break;
		case SCODA_DAC2_AGC0 ... SCODA_DAC2_AGC3:
			creg = SCODA_REG_CR_DAC2_AGC;
			dreg = SCODA_REG_DR_DAC2_AGC;
			sreg -= SCODA_DAC2_AGC0;
			break;
		case SCODA_ADC_AGC0 ... SCODA_ADC_AGC4:
			creg = SCODA_REG_CR_ADC_AGC;
			dreg = SCODA_REG_DR_ADC_AGC;
			sreg -= SCODA_ADC_AGC0;
			break;
		default:
			return 0;
	}

	codec_reg_read(NULL, creg, &cdata);
	cdata = (cdata & (~0x7f)) | (sreg & 0x3f);

	codec_reg_write(NULL, creg, cdata);
	codec_reg_read(NULL, dreg, val);

	return 0;
}

static int codec_reg_read(void *context, unsigned int reg,
				 unsigned int *val)
{
	unsigned int i;
	u32 tmp;
	int ret;

	if (reg > SCODA_REG_SR_TR_SRCDAC)
		return icdc_d3_hw_read_extend(reg, val);

	ret = codec_io_wait();
	if (ret)
		return ret;

	tmp = readl(priv.base + JZ_REG_AIC_RGADW);
	tmp = (tmp & ~ICDC_RGADW_RGADDR_MASK)
		| (reg << ICDC_RGADW_RGADDR_OFFSET);
	writel(tmp, priv.base + JZ_REG_AIC_RGADW);

	/* wait 6+ cycles */
	for (i = 0; i < 6; i++)
		*val = readl(priv.base + JZ_REG_AIC_RGDATA) &
			ICDC_RGDATA_RGDOUT_MASK;

	return 0;
}

static int icdc_d3_hw_write_extend(unsigned int sreg, unsigned int val)
{
	int creg, cdata, dreg, val_rb;
	switch (sreg) {
		case SCODA_MIX_0 ... SCODA_MIX_4:
			creg = SCODA_REG_CR_MIX;
			dreg = SCODA_REG_DR_MIX;
			sreg -= SCODA_MIX_0;
			break;
		case SCODA_DAC_AGC0 ... SCODA_DAC_AGC3:
			creg = SCODA_REG_CR_DAC_AGC;
			dreg = SCODA_REG_DR_DAC_AGC;
			sreg -= SCODA_DAC_AGC0;
			break;
		case SCODA_DAC2_AGC0 ... SCODA_DAC2_AGC3:
			creg = SCODA_REG_CR_DAC2_AGC;
			dreg = SCODA_REG_DR_DAC2_AGC;
			sreg -= SCODA_DAC2_AGC0;
			break;
		case SCODA_ADC_AGC0 ... SCODA_ADC_AGC4:
			creg = SCODA_REG_CR_ADC_AGC;
			dreg = SCODA_REG_DR_ADC_AGC;
			sreg -= SCODA_ADC_AGC0;
			break;
		default:
			return 0;
	}

	printk("write extend : sreg: %d [0 - 4], creg: %x sdata: %d\n", sreg, creg, val);

	codec_reg_read(NULL, creg, &cdata);
	cdata = (cdata & (~0x3f)) | ((sreg & 0x3f) | 0x40);

	codec_reg_write(NULL, creg, cdata);
	codec_reg_write(NULL, dreg, val);
	codec_reg_read(NULL, dreg, &val_rb);

	if (val != val_rb)
		return -1;

	return 0;
}

static int codec_reg_write(void *context, unsigned int reg,
				  unsigned int val)
{
	int ret;

	if (reg > SCODA_REG_SR_TR_SRCDAC)
		return icdc_d3_hw_write_extend(reg, val);

	ret = codec_io_wait();
	if (ret)
		return ret;

	writel(ICDC_RGADW_RGWR | (reg << ICDC_RGADW_RGADDR_OFFSET) | val,
		   priv.base + JZ_REG_AIC_RGADW);

	ret = codec_io_wait();
	if (ret)
		return ret;

	printk("== cregW: 0x%02x 0x%02x\n", reg, val);

	return 0;
}

static const u8 codec_reg_defaults[] = {
	/* reg 0x0 ... 0x9 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xd3,0xd3,
	/* reg 0xa ... 0x13 */
	0x00,0x30,0x30,0xb0,0xb1,0xb0,0x00,0x00,0x0f,0x40,
	/* reg 0x14 ... 0x1d */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,
	/* reg 0x1e ... 0x27 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	/* reg 0x28 ... 0x31 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	/* reg 0x32 ... 0x39 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	/* extern reg */
	0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x34,0x07,0x44,0x1f,0x00,
};

static struct regmap_config codec_regmap_config = {
	.reg_bits = 7,
	.val_bits = 8,

	.max_register = SCODA_MAX_REG_NUM,
	.volatile_reg = codec_volatile,
	.readable_reg = codec_readable,
	.writeable_reg = codec_writeable,

	.reg_read = codec_reg_read,
	.reg_write = codec_reg_write,

	.reg_defaults_raw = codec_reg_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(codec_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static void codec_startup(void)
{
	/*power on codec*/
	regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0);
	msleep(250);

	regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0);
	msleep(400);

	
}

static void codec_shutdown(void)
{
	/*power off codec*/
	regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);

	regmap_update_bits(priv.codec_regmap, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SLAVE_MASK, 0);
}

static void codec_power_dac(bool on)
{
	if (on) {
		regmap_update_bits(priv.codec_regmap, SCODA_REG_AICR_DAC,
				SCODA_AICR_DAC_SLAVE_MASK | SCODA_AICR_DAC_SB_MASK,
				0 | 0);
		regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_DAC, SCODA_CR_DAC_SB_MASK, 0 << SCODA_CR_DAC_SB_SHIFT);
	} else {
		regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_DAC, SCODA_CR_DAC_SB_MASK, 1 << SCODA_CR_DAC_SB_SHIFT);

		regmap_update_bits(priv.codec_regmap, SCODA_REG_AICR_DAC,
				SCODA_AICR_DAC_SLAVE_MASK | SCODA_AICR_DAC_SB_MASK,
				SCODA_AICR_DAC_SLAVE_MASK | SCODA_AICR_DAC_SB_MASK);
	}

}

static void codec_mute_stream(int playback, int recording)
{
	
	if (playback != -1)
		regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_DAC, SCODA_CR_DAC_SMUTE_MASK, playback << SCODA_CR_DAC_SMUTE_SHIFT);
	
	if (recording != -1)
		regmap_update_bits(priv.codec_regmap, SCODA_REG_CR_ADC, SCODA_CR_ADC_SMUTE_MASK, recording << SCODA_CR_ADC_SMUTE_SHIFT);

}

static const unsigned int codec_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100,
	48000, 88200, 96000, 176400, 192000
};

static int codec_apply_hw_params(bool playback)
{
	unsigned int rate, bit_width;

	int aicr_reg = playback ? SCODA_REG_AICR_DAC : SCODA_REG_AICR_ADC;
	int fcr_reg = playback ? SCODA_REG_FCR_DAC : SCODA_REG_FCR_ADC;

	printk("codec_hw_params enter\n");

	switch (priv.format) {
	case AFMT_S16_LE:
		bit_width = 0;
		break;
	// case SNDRV_PCM_FORMAT_S18_3LE:
	// 	bit_width = 1;
	// 	break;
	// case SNDRV_PCM_FORMAT_S20_3LE:
	// 	bit_width = 2;
	// 	break;
	// case SNDRV_PCM_FORMAT_S24_3LE:
	// 	bit_width = 3;
	// 	break;
	default:
		printk("codec_hw_params bad fmt: %d\n", priv.format);
		return -EINVAL;
	}

	for (rate = 0; rate < ARRAY_SIZE(codec_sample_rates); rate++) {
		if (codec_sample_rates[rate] == priv.rate)
			break;
	}

	if (rate == ARRAY_SIZE(codec_sample_rates)) {
		printk("bad rate\n");
		return -EINVAL;
	}

	regmap_update_bits(priv.codec_regmap, aicr_reg, SCODA_AICR_DAC_ADWL_MASK,
				   bit_width << SCODA_AICR_DAC_ADWL_SHIFT);

	regmap_update_bits(priv.codec_regmap, fcr_reg, SCODA_FCR_FREQ_MASK,
				   rate << SCODA_FCR_FREQ_SHIFT);

	return 0;
}

static void codec_init(void)
{
	struct regmap *regmap = priv.codec_regmap;

	/* Collect updates for later sending. */
	regcache_cache_only(regmap, true);

	printk("codec icdc-d3 probe enter\n");

	/* power off codec */
	regmap_update_bits(regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	regmap_update_bits(regmap, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);

	/* codec select enable 24M clock*/
	regmap_update_bits(regmap, SCODA_REG_CR_CK , SCODA_CR_CK_MCLK_DIV_MASK, 1 << SCODA_CR_CK_MCLK_DIV_SHIFT);
	regmap_update_bits(regmap, SCODA_REG_CR_CK , SCODA_CR_CK_SDCLK_MASK, 0 << SCODA_CR_CK_SDCLK_SHIFT);
	regmap_update_bits(regmap, SCODA_REG_CR_CK , SCODA_CR_CRYSTAL_MASK, 0 << SCODA_CR_CRYSTAL_SHIFT);

	/*codec select Dac/Adc i2s interface*/
	regmap_update_bits(regmap, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SLAVE_MASK, 0);
	regmap_update_bits(regmap, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_AUDIO_MASK, SCODA_AICR_DAC_AUDIOIF_I2S);

	/*codec generated IRQ is a high level */
	regmap_update_bits(regmap, SCODA_REG_ICR, SCODA_ICR_INT_FORM_MASK, SCODA_ICR_INT_FORM_LOW);

	/*codec irq mask*/
	regmap_write(regmap, SCODA_REG_IMR, SCODA_IMR_COMMON_MASK);
	regmap_write(regmap, SCODA_REG_IMR2, SCODA_IMR2_COMMON_MASK);

	/*codec clear all irq*/
	regmap_write(regmap, SCODA_REG_IFR, SCODA_IMR_COMMON_MASK);
	regmap_write(regmap, SCODA_REG_IFR2, SCODA_IMR2_COMMON_MASK);

	/* Send collected updates. */
	regcache_cache_only(regmap, false);
	regcache_sync(regmap);

	codec_reg_write(NULL, SCODA_MIX_3, 0x5 << 4);
	codec_reg_write(NULL, SCODA_MIX_2, 0x1 << 4);

}

static int x1501_audio_startup(void)
{
	uint32_t conf, ctrl;
	int ret;

	ctrl = aic_readl(JZ_REG_AIC_CTRL);
	ctrl |= JZ_AIC_CTRL_FLUSH;
	aic_writel(JZ_REG_AIC_CTRL, ctrl);

	ret = clk_prepare_enable(priv.clk_i2s);
	if (ret) {
		printk("jz4740_i2s_startup: failed to enable clk\n");
		return ret;
	}

	printk("~~ i2s clk activé\n");

	conf = aic_readl(JZ_REG_AIC_CONF);
	conf |= JZ_AIC_CONF_ENABLE;
	aic_writel(JZ_REG_AIC_CONF, conf);

	return 0;
}

static void x1501_audio_shutdown(void)
{
	uint32_t conf;

	conf = aic_readl(JZ_REG_AIC_CONF);
	conf &= ~JZ_AIC_CONF_ENABLE;
	aic_writel(JZ_REG_AIC_CONF, conf);

	clk_disable_unprepare(priv.clk_i2s);
	printk("~~ i2s clk desactivé\n");
}

static int x1501_audio_stream_control(int playback, int recording)
{

	uint32_t ctrl;
	uint32_t mask = 0;

	ctrl = aic_readl(JZ_REG_AIC_CTRL);

	if (playback != -1) {
		mask = JZ_AIC_CTRL_ENABLE_PLAYBACK | JZ_AIC_CTRL_ENABLE_TX_DMA;
		if (playback)
			ctrl |= mask;
		else
			ctrl &= ~mask;
	}
	
	if (recording != -1) {
		mask = JZ_AIC_CTRL_ENABLE_CAPTURE | JZ_AIC_CTRL_ENABLE_RX_DMA;
		if (recording)
			ctrl |= mask;
		else
			ctrl &= ~mask;
	}

	aic_writel(JZ_REG_AIC_CTRL, ctrl);

	return 0;
}

static int x1501_audio_apply_hw_params(bool is_playback)
{
	unsigned int sample_size;
	uint32_t ctrl, div_reg;
	unsigned long mclk_rate, bclk_rate;
	int div, R;

	ctrl = aic_readl(JZ_REG_AIC_CTRL);
	div_reg = aic_readl(JZ_REG_AIC_CLK_DIV);

	mclk_rate = clk_get_rate(priv.clk_i2s);
	bclk_rate = 32 * priv.channels * priv.rate;
	div = mclk_rate / bclk_rate;
	R = mclk_rate % bclk_rate;

	if (bclk_rate - R < R) {
		div += 1;
	}

	printk("mclk_rate: %lu, bclk_rate: %lu, div: %u\n", mclk_rate, bclk_rate, div);

	switch (priv.format) {
	// case AFMT_U8:
	// case AFMT_S8:
	// 	sample_size = 0;
	// 	break;
	case AFMT_S16_LE:
	// case AFMT_S16_BE:
		sample_size = 1;
		break;
	// case 24:
	// 	sample_size = 4;
	// 	break;
	default:
		return -EINVAL;
	}

	if (is_playback) {
		ctrl &= ~JZ_AIC_CTRL_OUTPUT_SAMPLE_SIZE_MASK;
		ctrl |= sample_size << JZ_AIC_CTRL_OUTPUT_SAMPLE_SIZE_OFFSET;
		if (priv.channels == 1)
			ctrl |= JZ_AIC_CTRL_MONO_TO_STEREO;
		else
			ctrl &= ~JZ_AIC_CTRL_MONO_TO_STEREO;

		div_reg &= ~I2SDIV_DV_MASK;
		div_reg |= (div - 1) << I2SDIV_DV_SHIFT;
	} else {
		ctrl &= ~JZ_AIC_CTRL_INPUT_SAMPLE_SIZE_MASK;
		ctrl |= sample_size << JZ_AIC_CTRL_INPUT_SAMPLE_SIZE_OFFSET;

		div_reg &= ~I2SDIV_IDV_MASK;
		div_reg |= (div - 1) << I2SDIV_IDV_SHIFT;
	}

	aic_writel(JZ_REG_AIC_CTRL, ctrl);
	aic_writel(JZ_REG_AIC_CLK_DIV, div_reg);

	return 0;
}

static int x1501_audio_init(void)
{
	uint32_t conf, format;
	int ret;

	conf = (7 << JZ4760_AIC_CONF_FIFO_RX_THRESHOLD_OFFSET) |
		(12 << JZ4760_AIC_CONF_FIFO_TX_THRESHOLD_OFFSET) |
		JZ_AIC_CONF_I2S | JZ_AIC_CONF_INTERNAL_CODEC;

	format = 0;

	aic_writel(JZ_REG_AIC_CONF, JZ_AIC_CONF_RESET);
	aic_writel(JZ_REG_AIC_CONF, conf);

	ret = clk_set_rate(priv.clk_i2s, 24000000);
	if (ret)
		return ret;

	aic_writel(JZ_REG_AIC_I2S_FMT, format);

	priv.format = AFMT_S16_LE;
	priv.channels = 2;
	priv.rate = 48000;

	return 0;
}

static int audio_buffer_init(struct device *dev, struct audio_buffer *abuf, 
	const char *dma_chan_name, unsigned len)
{
	unsigned len_period = len / NR_PERIODS;
	unsigned period_pos = 0;
	period* last_period = abuf->periods;

	abuf->len = len;
	abuf->len_period = len_period;
	abuf->period_in_use_by_dma = last_period;

	abuf->dma_ch = dma_request_chan(dev, dma_chan_name);
	if (IS_ERR(abuf->dma_ch))
		return PTR_ERR(abuf->dma_ch);

	dev_info(dev, "Allocating %u bytes for audio buffer '%s'\n", len, dma_chan_name);
	abuf->buf = dmam_alloc_coherent(dev, len, &abuf->buf_phys, GFP_KERNEL);
	if (!abuf->buf) {
		dev_err(dev, "Failed to allocate DMA descriptors\n");
		return -ENOMEM;
	}

	while (period_pos < len) {
		last_period->buf_pos = period_pos;
		last_period->next = last_period + 1;
		period_pos += len_period;
		last_period = last_period->next;
	}

	abuf->periods[NR_PERIODS - 1].next = abuf->periods;

	init_completion(&abuf->compl_buf_period);

	return 0;
}

static void audio_buffer_zero(struct audio_buffer *abuf) {
	memset(abuf->buf, 0, abuf->len);
}

static void audio_buffer_dma_cb(void *userp)
{
	struct audio_buffer *abuf = userp;
	if (abuf->user_written)
		abuf->user_written = 0;
	else
		memset(abuf->buf + abuf->period_in_use_by_dma->buf_pos, 0, abuf->len_period);
	abuf->period_in_use_by_dma = abuf->period_in_use_by_dma->next;
	// printk("Period pos: %u\n", abuf->period_in_use_by_dma->buf_pos);
	complete(&abuf->compl_buf_period);
}

static int audio_buffer_dma_start(struct audio_buffer *abuf,
				enum dma_transfer_direction dir)
{
	// 1. Prepare

	struct dma_slave_config cfg = {
		.direction = dir,
		.src_addr = priv.phys_base + JZ_REG_AIC_FIFO,
		.dst_addr = priv.phys_base + JZ_REG_AIC_FIFO,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES,
		.src_maxburst = 8,
		.dst_maxburst = 8,
	};

	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret;

	ret = dmaengine_slave_config(abuf->dma_ch, &cfg);
	if (ret)
		return ret;

	desc = dmaengine_prep_dma_cyclic(abuf->dma_ch,
		abuf->buf_phys, abuf->len, abuf->len_period, dir, DMA_PREP_INTERRUPT);

	if (!desc)
		return -ENOMEM;

	desc->callback = audio_buffer_dma_cb;

	// if (dir == DMA_DEV_TO_MEM) {
	// 	desc->callback = spi_ingenic_dma_finished_rx;
	// } else {
	// 	desc->callback = spi_ingenic_dma_finished_tx;
	// }

	desc->callback_param = abuf;

	cookie = dmaengine_submit(desc);

	ret = dma_submit_error(cookie);
	if (ret) {
		dmaengine_desc_free(desc);
		return ret;
	}

	abuf->dma_desc = desc;

	// 2. Start DMA
	audio_buffer_zero(abuf);
	dma_async_issue_pending(abuf->dma_ch);

	// 3. Instruct user to use the next period by 
	// marking the current one as full.
	// abuf->pos_period[abuf->period] = abuf->len_period;

	return 0;
}

static void audio_buffer_dma_stop(struct audio_buffer *abuf)
{
	dmaengine_terminate_async(abuf->dma_ch);
	dmaengine_synchronize(abuf->dma_ch);
	dmaengine_desc_free(abuf->dma_desc);

	abuf->period_in_use_by_dma = abuf->periods;
	abuf->cursor = 0;
	memset(abuf->buf, 0, abuf->len);

	// abuf->period = 0;
	// memset(abuf->pos_period, 0, sizeof(abuf->pos_period));
	reinit_completion(&abuf->compl_buf_period);
}

static int playback_start(void) {
	int ret;

	codec_apply_hw_params(true);
	x1501_audio_apply_hw_params(true);

	codec_power_dac(true);
	codec_mute_stream(0, 0);

	x1501_audio_stream_control(1, -1);

	ret = audio_buffer_dma_start(&priv.abuf_tx, DMA_MEM_TO_DEV);
	if (ret) {
		return ret;
	}

	return 0;
}

static void playback_stop(void) {
	codec_power_dac(false);
	x1501_audio_stream_control(0, -1);
	audio_buffer_dma_stop(&priv.abuf_tx);
}

static int dsp_open(struct inode *inode, struct file *file)
{
	int ret;

	if (priv.opened)
		return -EBUSY;

	ret = x1501_audio_startup();
	if (ret)
		return ret;

	codec_startup();

	priv.opened = 1;

	printk("opened\n");

	return 0;
}

static int dsp_release(struct inode *inode, struct file *file)
{
	if (priv.playing)
		playback_stop();

	codec_shutdown();
	x1501_audio_shutdown();

	printk("closed\n");

	priv.opened = 0;
	priv.playing = 0;
	priv.recording = 0;

	return 0;
}

static ssize_t dsp_read(struct file *file, char __user *buf,
			size_t count, loff_t *offp)
{
	return 0;
}

static ssize_t dsp_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *offp)
{
	struct audio_buffer *abuf = &priv.abuf_tx;
	int ret;
	unsigned bytes_available;
	unsigned bytes_write;

	if (!priv.playing) {
		ret = playback_start();
		if (ret)
			return ret;

		priv.playing = 1;
	}

	bytes_available = abuf->period_in_use_by_dma->buf_pos + abuf->len - abuf->cursor;

	if (bytes_available > abuf->len) {
		bytes_available -= abuf->len;
	} else if (bytes_available > (abuf->len - abuf->len_period)) {
		if (wait_for_completion_interruptible(&abuf->compl_buf_period) < 0) {
			printk("DMA operation cancelled\n");
		}
		return 0;
	}

	bytes_write = bytes_available;

	if (count < bytes_write)
		bytes_write = count;

	if (abuf->cursor + bytes_write > abuf->len) {
		unsigned tail_len = abuf->len - abuf->cursor;
		unsigned head_len = bytes_write - tail_len;
		copy_from_user(abuf->buf + abuf->cursor, buf, tail_len);
		copy_from_user(abuf->buf, buf + tail_len, head_len);
		abuf->cursor = head_len;
	} else {
		copy_from_user(abuf->buf + abuf->cursor, buf, bytes_write);
		abuf->cursor += bytes_write;
	}

	if (abuf->cursor >= abuf->len)
		abuf->cursor = abuf->cursor - abuf->len;

	if (bytes_write >= bytes_available) {
		if (wait_for_completion_interruptible(&abuf->compl_buf_period) < 0) {
			printk("DMA operation cancelled\n");
		}
	}

	abuf->user_written = 1;

	return bytes_write;
}

static long dsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int value;
	bool ok;

	switch(cmd) {
	case SNDCTL_DSP_SETFMT:
		if (copy_from_user(&value, (int *)arg, sizeof(value)))
			return -EFAULT;

		switch (value) {
		case AFMT_S16_LE:
		// case AFMT_S16_BE:
		// case AFMT_S8:
		// case AFMT_U8:
			priv.format = value;
			break;
		default:
			return -ENOSYS;
		}
		break;
	case SNDCTL_DSP_SPEED:
		if (copy_from_user(&value, (int *)arg, sizeof(value)))
			return -EFAULT;

		ok = false;

		for (int i=0; i<ARRAY_SIZE(codec_sample_rates); i++) {
			if (codec_sample_rates[i] == value) {
				ok = true;
				priv.rate = value;
				break;
			}
		}

		if (!ok)
			return -ENOSYS;

		break;
	case SNDCTL_DSP_CHANNELS:
		if (copy_from_user(&value, (int *)arg, sizeof(value)))
			return -EFAULT;

		switch (value) {
		case 1:
		case 2:
			priv.channels = value;
			break;
		default:
			return -ENOSYS;
		}
		break;
	case SNDCTL_DSP_STEREO:
		if (copy_from_user(&value, (int *)arg, sizeof(value)))
			return -EFAULT;

		switch (value) {
		case 0:
		case 1:
			priv.channels = value + 1;
			break;
		default:
			return -ENOSYS;
		}
		break;
	case SNDCTL_DSP_GETFMTS:
		value = AFMT_S16_LE /* | AFMT_S16_BE | AFMT_S8 | AFMT_U8 */;

		if (copy_to_user((int *)arg, &value, sizeof(value)))
			return -EFAULT;

		break;
	case SNDCTL_DSP_RESET:
		if (priv.playing)
			playback_stop();

		break;
	default:
		pr_info("ignoring OSS ioctl 0x%x\n", cmd);
		break;
	}
	return 0;
}

static const struct file_operations x1501_audio_fops = {
	.owner		= THIS_MODULE,
	.open		= dsp_open,
	.release	= dsp_release,
	.read		= dsp_read,
	.write		= dsp_write,
	.unlocked_ioctl = dsp_ioctl
};

static struct miscdevice x1501_audio_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KBUILD_MODNAME,
	.fops = &x1501_audio_fops,
};

static int x1501_audio_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	int ret;

	memset(&priv, 0, sizeof(struct x1501_audio));

	priv.base = devm_platform_get_and_ioremap_resource(pdev, 0, &mem);
	if (IS_ERR(priv.base))
		return PTR_ERR(priv.base);

	priv.phys_base = mem->start;

	priv.clk_aic = devm_clk_get(dev, "aic");
	if (IS_ERR(priv.clk_aic))
		return PTR_ERR(priv.clk_aic);

	priv.clk_i2s = devm_clk_get(dev, "i2s");
	if (IS_ERR(priv.clk_i2s))
		return PTR_ERR(priv.clk_i2s);

	ret = audio_buffer_init(dev, &priv.abuf_tx, "tx", 12288);
	if (ret)
		return ret;

	ret = audio_buffer_init(dev, &priv.abuf_rx, "rx", 8192);
	if (ret)
		return ret;

	priv.codec_regmap = devm_regmap_init(dev, NULL, NULL,
					&codec_regmap_config);
	if (IS_ERR(priv.codec_regmap))
		return PTR_ERR(priv.codec_regmap);

	ret = clk_prepare_enable(priv.clk_aic);
	if (ret)
		return ret;

	codec_init();

	ret = x1501_audio_init();
	if (ret)
		return ret;

	return misc_register(&x1501_audio_miscdev);
}

static const struct of_device_id x1501_audio_of_matches[] = {
	{ .compatible = "ingenic,x1501-audio", .data = NULL },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x1501_audio_of_matches);

static struct platform_driver x1501_audio_driver = {
	.probe = x1501_audio_dev_probe,
	.driver = {
		.name = "x1501-audio",
		.of_match_table = x1501_audio_of_matches,
	},
};

module_platform_driver(x1501_audio_driver);

MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_DESCRIPTION("Ingenic X1501 Lightweight Audio Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:x1501-audio");
