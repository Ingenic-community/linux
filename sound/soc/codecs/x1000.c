// SPDX-License-Identifier: GPL-2.0
//
// Ingenic X1000 CODEC driver
//
// Copyright (C) 2015, Ingenic Semiconductor Co.,Ltd. - sccheng <shicheng.cheng@ingenic.com>
// Copyright (C) 2023, Reimu NotMoe <reimu@sudomaker.com>
//
// Based on jz4770.c

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/time64.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#define ICDC_RGADW_OFFSET		0x00
#define ICDC_RGDATA_OFFSET		0x04

/* ICDC internal register access control register(RGADW) */
#define ICDC_RGADW_RGWR			BIT(16)

#define ICDC_RGADW_RGADDR_OFFSET	8
#define	ICDC_RGADW_RGADDR_MASK		GENMASK(14, ICDC_RGADW_RGADDR_OFFSET)

#define ICDC_RGADW_RGDIN_OFFSET		0
#define	ICDC_RGADW_RGDIN_MASK		GENMASK(7, ICDC_RGADW_RGDIN_OFFSET)

/* ICDC internal register data output register (RGDATA)*/
#define ICDC_RGDATA_IRQ			BIT(8)

#define ICDC_RGDATA_RGDOUT_OFFSET	0
#define ICDC_RGDATA_RGDOUT_MASK		GENMASK(7, ICDC_RGDATA_RGDOUT_OFFSET)

/* Internal register space, accessed through regmap */
enum {
	SCODA_REG_SR = 0x0,
	SCODA_REG_SR2,
	SCODA_REG_SIGR,
	SCODA_REG_SIGR2,
	SCODA_REG_SIGR3,
	SCODA_REG_SIGR5,
	SCODA_REG_SIGR7,
	SCODA_REG_MR,
	SCODA_REG_AICR_DAC,
	SCODA_REG_AICR_ADC,
	SCODA_REG_CR_DMIC,
	SCODA_REG_CR_MIC1,
	SCODA_REG_CR_MIC2,
	SCODA_REG_CR_DAC,
	SCODA_REG_CR_DAC2,
	SCODA_REG_CR_ADC,
	SCODA_REG_CR_MIX,
	SCODA_REG_DR_MIX,
	SCODA_REG_CR_VIC,
	SCODA_REG_CR_CK,
	SCODA_REG_FCR_DAC,
	SCODA_REG_SFCCR_DAC,
	SCODA_REG_SFFCR_DAC,
	SCODA_REG_FCR_ADC,
	SCODA_REG_CR_TIMER_MSB,
	SCODA_REG_CR_TIMER_LSB,
	SCODA_REG_ICR,
	SCODA_REG_IMR,
	SCODA_REG_IFR,
	SCODA_REG_IMR2,
	SCODA_REG_IFR2,
	SCODA_REG_GCR_DACL,
	SCODA_REG_GCR_DACR,
	SCODA_REG_GCR_DACL2,
	SCODA_REG_GCR_DACR2,
	SCODA_REG_GCR_MIC1,
	SCODA_REG_GCR_MIC2,
	SCODA_REG_GCR_ADCL,
	SCODA_REG_GCR_ADCR,
	SCODA_REG_GCR_MIXDACL,
	SCODA_REG_GCR_MIXDACR,
	SCODA_REG_GCR_MIXADCL,
	SCODA_REG_GCR_MIXADCR,
	SCODA_REG_CR_DAC_AGC,
	SCODA_REG_DR_DAC_AGC,
	SCODA_REG_CR_DAC2_AGC,
	SCODA_REG_DR_DAC2_AGC,
	SCODA_REG_CR_ADC_AGC,
	SCODA_REG_DR_ADC_AGC,
	SCODA_REG_SR_ADC_AGCDGL,
	SCODA_REG_SR_ADC_AGCDGR,
	SCODA_REG_SR_ADC_AGCAGL,
	SCODA_REG_SR_ADC_AGCAGR,
	SCODA_REG_CR_TR,
	SCODA_REG_DR_TR,
	SCODA_REG_SR_TR1,
	SCODA_REG_SR_TR2,
	SCODA_REG_SR_TR_SRCDAC,

/*  icdc_d3 internal register extend space */
	SCODA_MIX_0,
	SCODA_MIX_1,
	SCODA_MIX_2,
	SCODA_MIX_3,
	SCODA_MIX_4,

	SCODA_DAC_AGC0,
	SCODA_DAC_AGC1,
	SCODA_DAC_AGC2,
	SCODA_DAC_AGC3,

	SCODA_DAC2_AGC0,
	SCODA_DAC2_AGC1,
	SCODA_DAC2_AGC2,
	SCODA_DAC2_AGC3,

	SCODA_ADC_AGC0,
	SCODA_ADC_AGC1,
	SCODA_ADC_AGC2,
	SCODA_ADC_AGC3,
	SCODA_ADC_AGC4,
	SCODA_MAX_REG_NUM,
};

/*aicr dac*/
#define SCODA_AICR_DAC_ADWL_SHIFT (6)
#define SCODA_AICR_DAC_ADWL_MASK (0x3 << SCODA_AICR_DAC_ADWL_SHIFT)
#define SCODA_AICR_DAC_SLAVE_SHIFT (5)
#define SCODA_AICR_DAC_SLAVE_MASK (0x1 << SCODA_AICR_DAC_SLAVE_SHIFT)
#define SCODA_AICR_DAC_SLAVE (1 << 5)
#define SCODA_AICR_DAC_SB_SHIFT (4)
#define SCODA_AICR_DAC_SB_MASK (0x1 << SCODA_AICR_DAC_SB_SHIFT)
#define SCODA_AICR_DAC_AUDIOIF_SHIFT (0)
#define SCODA_AICR_DAC_AUDIO_MASK (0x3 << SCODA_AICR_DAC_AUDIOIF_SHIFT)
#define SCODA_AICR_DAC_AUDIOIF_I2S (0x3)

/* aicr adc */
#define SCODA_AICR_ADC_ADWL_SHIFT (6)
#define SCODA_AICR_ADC_ADWL_MASK (0x3 << SCODA_AICR_ADC_ADWL_SHIFT)
#define SCODA_AICR_ADC_SB_SHIFT (4)
#define SCODA_AICR_ADC_SB_MASK (0x1 << SCODA_AICR_ADC_SB_SHIFT)
#define SCODA_AICR_ADC_AUDIOIF_SHIFT (0)
#define SCODA_AICR_ADC_AUDIO_MASK (0x3 << SCODA_AICR_ADC_AUDIOIF_SHIFT)
#define SCODA_AICR_ADC_AUDIOIF_I2S (0x3)

/* cr vic */
#define SCODA_CR_VIC_SB_SHIFT (0)
#define SCODA_CR_VIC_SB_MASK (1 << SCODA_CR_VIC_SB_SHIFT)
#define SCODA_CR_VIC_SB_SLEEP_SHIFT (1)
#define SCODA_CR_VIC_SB_SLEEP_MASK (1 << SCODA_CR_VIC_SB_SLEEP_SHIFT)

/* fcr adc/dac */
#define SCODA_FCR_FREQ_SHIFT (0)
#define SCODA_FCR_FREQ_MASK (0xf << SCODA_FCR_FREQ_SHIFT)

/* cr dac */
#define SCODA_CR_DAC_SMUTE_SHIFT (7)
#define SCODA_CR_DAC_SMUTE_MASK (0x1 << SCODA_CR_DAC_SMUTE_SHIFT)
#define SCODA_CR_DAC_SB_SHIFT (4)
#define SCODA_CR_DAC_SB_MASK (0x1 << SCODA_CR_DAC_SB_SHIFT)
#define SCODA_CR_DAC_ZERO_SHIFT (0)
#define SCODA_CR_DAC_ZERO_MASK (0x1 << SCODA_CR_DAC_ZERO_SHIFT)

/* cr dac */
#define SCODA_CR_ADC_SMUTE_SHIFT (7)
#define SCODA_CR_ADC_SMUTE_MASK (0x1 << SCODA_CR_ADC_SMUTE_SHIFT)
#define SCODA_CR_ADC_MIC_SEL_SHIFT (6)
#define SCODA_CR_ADC_MIC_SEL_MASK (0x1 << SCODA_CR_ADC_MIC_SEL_SHIFT)
#define SCODA_CR_ADC_SB_SHIFT (4)
#define SCODA_CR_ADC_SB_MASK (0x1 << SCODA_CR_ADC_SB_SHIFT)
#define SCODA_CR_ADC_ZERO_SHIFT (0)
#define SCODA_CR_ADC_ZERO_MASK (0x1 << SCODA_CR_ADC_ZERO_SHIFT)

/* ifr */
#define SCODA_IFR_DAC_MUTE_SHIFT (0)
#define SCODA_IFR_DAC_MUTE_MASK (0x1 << SCODA_IFR_DAC_MUTE_SHIFT)
#define SCODA_IFR_ADC_MUTE_SHIFT (2)
#define SCODA_IFR_ADC_MUTE_MASK (0x1 << SCODA_IFR_ADC_MUTE_SHIFT)
#define SCODA_IFR_ADAS_LOCK_SHIFT (7)
#define SCODA_IFR_ADAS_LOCK_MASK (0x1 << SCODA_IFR_ADAS_LOCK_SHIFT)

/* cr ck */
#define SCODA_CR_CK_MCLK_DIV_SHIFT (6)
#define SCODA_CR_CK_MCLK_DIV_MASK (0x1 << SCODA_CR_CK_MCLK_DIV_SHIFT)
#define SCODA_CR_CK_SDCLK_SHIFT (4)
#define SCODA_CR_CK_SDCLK_MASK (0x1 << SCODA_CR_CK_SDCLK_SHIFT)
#define SCODA_CR_CRYSTAL_SHIFT (0)
#define SCODA_CR_CRYSTAL_MASK (0xf << SCODA_CR_CRYSTAL_SHIFT)

/* icr */
#define SCODA_ICR_INT_FORM_SHIFT (6)
#define SCODA_ICR_INT_FORM_MASK (0x3 << SCODA_ICR_INT_FORM_SHIFT)
#define SCODA_ICR_INT_FORM_HIGH (0)
#define SCODA_ICR_INT_FORM_LOW  (1)

/* imr */
#define SCODA_IMR_COMMON_MASK (0xff)
#define SCODA_IMR2_COMMON_MASK (0xff)

/*For Codec*/
#define RGADW		(0xA4)
#define RGDATA		(0xA8)

/* codec private data */
struct jz_codec {
	struct device *dev;
	struct regmap *regmap;
	void __iomem *base;
	struct clk *clk;
};

static int x1000_codec_reg_read(void *context, unsigned int reg, unsigned int *val);
static int x1000_codec_reg_write(void *context, unsigned int reg, unsigned int val);

static int x1000_codec_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;

	/*power on codec*/
	if (snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0))
		msleep(250);

	if (snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0))
		msleep(400);

	return 0;
}

static void x1000_codec_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;

	/*power off codec*/
	snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);
}


static int x1000_codec_pcm_trigger(struct snd_pcm_substream *substream,
				    int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* do nothing */
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int x1000_codec_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *codec = dai->component;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_component_update_bits(codec, SCODA_REG_CR_DAC, SCODA_CR_DAC_SMUTE_MASK, mute << SCODA_CR_DAC_SMUTE_SHIFT);
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		snd_soc_component_update_bits(codec, SCODA_REG_CR_ADC, SCODA_CR_ADC_SMUTE_MASK, mute << SCODA_CR_ADC_SMUTE_SHIFT);
	}

	return 0;
}

/* unit: 0.01dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(mix_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(mic_tlv, 0, 100, 0);

static const unsigned int icdc_d3_adc_mic_sel_value[] = {0x0, 0x1,};
static const unsigned int icdc_d3_mixer_input_sel_value[] = {0x0, 0x5, 0xa, 0xf,};
static const unsigned int icdc_d3_mixer_input_sel_value_double[] = {0x0, 0x1, 0x2, 0x3,};
static const unsigned int icdc_d3_mixer_mode_sel_value[] = {0x0, 0x1,};

static const char *icdc_d3_mixer_input_sel[] = { "Normal Inputs", "Cross Inputs", "Mixed Inputs", "Zero Inputs"};
static const char *icdc_d3_adc_mic_sel[] = { "AMIC ON", "DMIC ON"};
static const char *icdc_d3_mercury_vir_sel[] = { "MERCURY ON", "MERCURY OFF"};
static const char *icdc_d3_titanium_vir_sel[] = { "TITANIUM ON", "TITANIUM OFF"};
static const char *icdc_d3_dac_mixer_mode_sel[] = { "PLAYBACK DAC", "PLAYBACK DAC + ADC"};
static const char *icdc_d3_adc_mixer_mode_sel[] = { "RECORD INPUT", "RECORD INPUT + DAC"};

static const struct soc_enum icdc_d3_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SCODA_REG_CR_ADC, 6, 0x1,  ARRAY_SIZE(icdc_d3_adc_mic_sel),icdc_d3_adc_mic_sel, icdc_d3_adc_mic_sel_value), /*0*/
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(icdc_d3_mercury_vir_sel), icdc_d3_mercury_vir_sel), /*1*/
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(icdc_d3_titanium_vir_sel), icdc_d3_titanium_vir_sel), /*2*/

	/*select input method*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_0, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*3*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_1, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*4*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_2, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double),/*5*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_3, 6,4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double),/*6*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_4, 4, 0xf,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value),/*7*/

	/*select mix mode*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_0, 0, 1,  ARRAY_SIZE(icdc_d3_dac_mixer_mode_sel),icdc_d3_dac_mixer_mode_sel, icdc_d3_mixer_mode_sel_value),/*8*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_4, 0, 1,  ARRAY_SIZE(icdc_d3_dac_mixer_mode_sel),icdc_d3_dac_mixer_mode_sel, icdc_d3_mixer_mode_sel_value),/*9*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 0, 1,  ARRAY_SIZE(icdc_d3_adc_mixer_mode_sel),icdc_d3_adc_mixer_mode_sel, icdc_d3_mixer_mode_sel_value),/*10*/

	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 6, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value),/*11*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel),icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value),/*12*/
};

/* Unconditional controls. */
static const struct snd_kcontrol_new x1000_codec_snd_controls[] = {
	/* Volume controls */
	SOC_DOUBLE_R_TLV("MERCURY Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 63, 0, dac_tlv),
	// SOC_DOUBLE_R_TLV("TITANIUM Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 63, 0, dac_tlv),
	SOC_DOUBLE_R_TLV("Playback Mixer Volume", SCODA_REG_GCR_MIXDACL, SCODA_REG_GCR_MIXDACR, 0, 31, 1, mix_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Volume", SCODA_REG_GCR_ADCL, SCODA_REG_GCR_ADCR, 0, 43, 0, adc_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Mixer Volume", SCODA_REG_GCR_MIXADCL, SCODA_REG_GCR_MIXADCR, 0, 31, 1, mix_tlv),
	SOC_SINGLE_TLV("Mic Volume", SCODA_REG_GCR_MIC1, 0, 4, 0, mic_tlv),

	/* ADC private controls */
	SOC_SINGLE("ADC High Pass Filter Switch", SCODA_REG_FCR_ADC, 6, 1, 0),

	/* mic private controls */
	SOC_SINGLE("Digital Playback mute", SCODA_REG_CR_DAC, 7, 1, 0),
	/* mixer enable controls */
	SOC_SINGLE("Mixer Enable", SCODA_REG_CR_MIX, 7, 1, 0),
};

static const struct snd_kcontrol_new icdc_d3_adc_controls =
	SOC_DAPM_ENUM("Route",  icdc_d3_enum[0]);

static const struct snd_kcontrol_new icdc_d3_mercury_vmux_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[1]);

static const struct snd_kcontrol_new icdc_d3_titanium_vmux_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[2]);

static const struct snd_kcontrol_new icdc_d3_mercury_aidac_input_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[3]);

static const struct snd_kcontrol_new icdc_d3_dac_input_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[4]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[5]);

static const struct snd_kcontrol_new icdc_d3_adc_input_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[6]);

static const struct snd_kcontrol_new icdc_d3_titanium_aidac_input_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[7]);

static const struct snd_kcontrol_new icdc_d3_mercury_mixer_mode_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[8]);

static const struct snd_kcontrol_new icdc_d3_titanium_mixer_mode_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[9]);

static const struct snd_kcontrol_new icdc_d3_aiadc_mixer_mode_sel_controls =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[10]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls_l =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[11]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls_r =
	SOC_DAPM_ENUM("Route", icdc_d3_enum[12]);

static const struct snd_soc_dapm_widget x1000_codec_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", "Capture" , SCODA_REG_AICR_ADC, 4, 1),
	SND_SOC_DAPM_MUX("ADC Mux", SCODA_REG_CR_ADC, 4, 1, &icdc_d3_adc_controls),
	SND_SOC_DAPM_MICBIAS("MICBIAS", SCODA_REG_CR_MIC1, 5, 1),
	SND_SOC_DAPM_PGA("AMIC", SCODA_REG_CR_MIC1, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("DMIC", SCODA_REG_CR_DMIC, 7, 0, NULL, 0),

/* DAC */
	SND_SOC_DAPM_DAC("DAC", "Playback", SCODA_REG_AICR_DAC, 4, 1),

	SND_SOC_DAPM_MUX("DAC_MERCURY VMux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_vmux_controls),
	SND_SOC_DAPM_PGA("DAC_MERCURY", SCODA_REG_CR_DAC, 4, 1, NULL, 0),

/*	SND_SOC_DAPM_MUX("DAC_TITANIUM VMux", SND_SOC_NOPM, 0, 0, &icdc_d3_titanium_vmux_controls),*/
/*	SND_SOC_DAPM_PGA("DAC_TITANIUM", SCODA_REG_CR_DAC2, 4, 1, NULL, 0),*/

/* MIXER */
	SND_SOC_DAPM_MUX("MERCURY AIDAC MIXER Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_dac_input_sel_controls),
	SND_SOC_DAPM_MUX("DAC Mode Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_mixer_mode_sel_controls),
	SND_SOC_DAPM_MUX("MERCURY AIDAC Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_aidac_input_sel_controls),

/* ADC */
	SND_SOC_DAPM_MUX("ADC Mode Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_mixer_mode_sel_controls),
	SND_SOC_DAPM_MUX("AIADC Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls),
	SND_SOC_DAPM_MUX("AIADC Mux L", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls_l),
	SND_SOC_DAPM_MUX("AIADC Mux R", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls_r),
	SND_SOC_DAPM_MUX("ADC MIXER Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_adc_input_sel_controls),

/* PINS */
	SND_SOC_DAPM_INPUT("AIP"),
	SND_SOC_DAPM_INPUT("AIN"),
	SND_SOC_DAPM_INPUT("DMIC IN"),
	SND_SOC_DAPM_OUTPUT("DO_LO_PWM"),
	SND_SOC_DAPM_OUTPUT("DO_BO_PWM"),
};

/* Unconditional routes. */
static const struct snd_soc_dapm_route x1000_codec_dapm_routes[] = {
	{ "MICBIAS",  NULL,  "AIP" },
	{ "MICBIAS",  NULL,  "AIN" },
	{ "AMIC",  NULL,  "MICBIAS" },
	{ "AMIC",  NULL,  "MICBIAS" },

	/*input*/
	{ "ADC Mux", "AMIC ON", "AMIC" },
	{ "ADC Mux", "DMIC ON", "DMIC IN" },

	{ "ADC Mode Mux" , "RECORD INPUT","ADC Mux"},
	{ "ADC Mode Mux" , "RECORD INPUT + DAC","ADC Mux"},

	{ "AIADC Mux" , "Normal Inputs","ADC Mode Mux"},
	{ "AIADC Mux" , "Cross Inputs","ADC Mode Mux"},
	{ "AIADC Mux" , "Mixed Inputs","ADC Mode Mux"},
	{ "AIADC Mux" , "Zero Inputs","ADC Mode Mux"},
	{ "AIADC Mux L" , "Normal Inputs","ADC Mode Mux"},
	{ "AIADC Mux L" , "Cross Inputs","ADC Mode Mux"},
	{ "AIADC Mux L" , "Mixed Inputs","ADC Mode Mux"},
	{ "AIADC Mux L" , "Zero Inputs","ADC Mode Mux"},
	{ "AIADC Mux R" , "Normal Inputs","ADC Mode Mux"},
	{ "AIADC Mux R" , "Cross Inputs","ADC Mode Mux"},
	{ "AIADC Mux R" , "Mixed Inputs","ADC Mode Mux"},
	{ "AIADC Mux R" , "Zero Inputs","ADC Mode Mux"},

	{ "ADC", NULL, "AIADC Mux" },
	{ "ADC", NULL, "AIADC Mux L" },
	{ "ADC", NULL, "AIADC Mux R" },

	{ "ADC MIXER Mux" , "Normal Inputs","ADC Mux"},
	{ "ADC MIXER Mux" , "Cross Inputs","ADC Mux"},
	{ "ADC MIXER Mux" , "Mixed Inputs","ADC Mux"},
	{ "ADC MIXER Mux" , "Zero Inputs","ADC Mux"},

	{"DAC Mode Mux", NULL, "ADC MIXER Mux"},
	/*output*/
	{ "DAC_MERCURY"  , NULL, "DAC" },
	{ "DAC_MERCURY VMux"  , "MERCURY ON"  , "DAC_MERCURY" },

	/* select mixer inputs*/
	{"MERCURY AIDAC MIXER Mux", "Normal Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Cross Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Mixed Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Zero Inputs", "DAC_MERCURY VMux"},

	/*select mixer mode*/
	{"DAC Mode Mux", "PLAYBACK DAC", "DAC_MERCURY VMux"},
	{"DAC Mode Mux", "PLAYBACK DAC + ADC", "MERCURY AIDAC MIXER Mux"},

	/*	DAC_MERCURY Vmux->DAC Mux*/
	/*select mixer output channels*/
	{ "MERCURY AIDAC Mux"  , "Normal Inputs"  , "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux"  , "Cross Inputs"  , "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux"  , "Mixed Inputs"  , "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux"  , "Zero Inputs"  , "DAC Mode Mux" },

	{ "DO_LO_PWM", NULL, "MERCURY AIDAC Mux" },
};

static void x1000_codec_codec_init_regs(struct snd_soc_component *codec)
{
	struct jz_codec *jz_codec = snd_soc_component_get_drvdata(codec);
	struct regmap *regmap = jz_codec->regmap;

	/* Collect updates for later sending. */
	regcache_cache_only(regmap, true);

	dev_info(codec->dev, "codec icdc-d3 probe enter\n");

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

	x1000_codec_reg_write(jz_codec, SCODA_MIX_3, 0x5 << 4);
	x1000_codec_reg_write(jz_codec, SCODA_MIX_2, 0x1 << 4);
}

#ifdef CONFIG_PM
static int x1000_codec_suspend(struct snd_soc_component *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(codec);

	snd_soc_component_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 1);
	snd_soc_component_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 1);
	snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);
	snd_soc_component_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 1);
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_OFF);
	return 0;
}

static int x1000_codec_resume(struct snd_soc_component *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(codec);

	snd_soc_component_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 0);

	if (snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0))
		msleep(250);
	if (snd_soc_component_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0)) {
		msleep(10);
	}
	snd_soc_component_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 0);
	snd_soc_component_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 0);
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_STANDBY);
	return 0;
}
#endif

static int x1000_codec_codec_probe(struct snd_soc_component *codec)
{
	struct jz_codec *jz_codec = snd_soc_component_get_drvdata(codec);

	clk_prepare_enable(jz_codec->clk);

	x1000_codec_codec_init_regs(codec);

	return 0;
}

static void x1000_codec_codec_remove(struct snd_soc_component *codec)
{
	struct jz_codec *jz_codec = snd_soc_component_get_drvdata(codec);

	clk_disable_unprepare(jz_codec->clk);
}

static const struct snd_soc_component_driver x1000_codec_soc_codec_dev = {
	.probe			= x1000_codec_codec_probe,
	.remove			= x1000_codec_codec_remove,
#ifdef CONFIG_PM
	.suspend		= x1000_codec_suspend,
	.resume			= x1000_codec_resume,
#endif
	.controls		= x1000_codec_snd_controls,
	.num_controls		= ARRAY_SIZE(x1000_codec_snd_controls),
	.dapm_widgets		= x1000_codec_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(x1000_codec_dapm_widgets),
	.dapm_routes		= x1000_codec_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(x1000_codec_dapm_routes),
	.suspend_bias_off	= 1,
	.use_pmdown_time	= 1,
};

static const unsigned int x1000_codec_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100,
	48000, 88200, 96000, 176400, 192000
};

static int x1000_codec_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct jz_codec *codec = snd_soc_component_get_drvdata(dai->component);
	unsigned int rate, bit_width;
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	int aicr_reg = playback ? SCODA_REG_AICR_DAC : SCODA_REG_AICR_ADC;
	int fcr_reg = playback ? SCODA_REG_FCR_DAC : SCODA_REG_FCR_ADC;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width = 3;
		break;
	default:
		dev_err(codec->dev, "bad format: %d\n", params_format(params));
		return -EINVAL;
	}

	for (rate = 0; rate < ARRAY_SIZE(x1000_codec_sample_rates); rate++) {
		if (x1000_codec_sample_rates[rate] == params_rate(params))
			break;
	}

	if (rate == ARRAY_SIZE(x1000_codec_sample_rates)) {
		dev_err(codec->dev, "bad format: %d\n", params_rate(params));
		return -EINVAL;
	}

	regmap_update_bits(codec->regmap, aicr_reg, SCODA_AICR_DAC_ADWL_MASK,
				   bit_width << SCODA_AICR_DAC_ADWL_SHIFT);

	regmap_update_bits(codec->regmap, fcr_reg, SCODA_FCR_FREQ_MASK,
				   rate << SCODA_FCR_FREQ_SHIFT);

	return 0;
}

static const struct snd_soc_dai_ops x1000_codec_dai_ops = {
	.startup	= x1000_codec_startup,
	.shutdown	= x1000_codec_shutdown,
	.hw_params	= x1000_codec_hw_params,
	.trigger	= x1000_codec_pcm_trigger,
	.mute_stream	= x1000_codec_mute_stream,
	.no_capture_mute = 0,
};

#define JZ_CODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  | \
			  SNDRV_PCM_FMTBIT_S18_3LE | \
			  SNDRV_PCM_FMTBIT_S20_3LE | \
			  SNDRV_PCM_FMTBIT_S24_3LE)

static struct snd_soc_dai_driver x1000_codec_dai = {
	.name = "x1000-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = JZ_CODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = JZ_CODEC_FORMATS,
	},
	.ops = &x1000_codec_dai_ops,
};

static bool x1000_codec_volatile(struct device *dev, unsigned int reg)
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

static bool x1000_codec_readable(struct device *dev, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return false;
	else
		return true;
}

static bool x1000_codec_writeable(struct device *dev, unsigned int reg)
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

static int x1000_codec_io_wait(struct jz_codec *codec)
{
	u32 reg;

	return readl_poll_timeout(codec->base + ICDC_RGADW_OFFSET, reg,
				  !(reg & ICDC_RGADW_RGWR),
				  1000, 1 * USEC_PER_SEC);
}

static int icdc_d3_hw_read_extend(void *context, unsigned int sreg,
				 unsigned int *val)
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

	x1000_codec_reg_read(context, creg, &cdata);
	cdata = (cdata & (~0x7f)) | (sreg & 0x3f);

	x1000_codec_reg_write(context, creg, cdata);
	x1000_codec_reg_read(context, dreg, val);

	return 0;
}

static int x1000_codec_reg_read(void *context, unsigned int reg,
				 unsigned int *val)
{
	struct jz_codec *codec = context;
	unsigned int i;
	u32 tmp;
	int ret;

	if (reg > SCODA_REG_SR_TR_SRCDAC)
		return icdc_d3_hw_read_extend(context, reg, val);

	ret = x1000_codec_io_wait(codec);
	if (ret)
		return ret;

	tmp = readl(codec->base + ICDC_RGADW_OFFSET);
	tmp = (tmp & ~ICDC_RGADW_RGADDR_MASK)
	    | (reg << ICDC_RGADW_RGADDR_OFFSET);
	writel(tmp, codec->base + ICDC_RGADW_OFFSET);

	/* wait 6+ cycles */
	for (i = 0; i < 6; i++)
		*val = readl(codec->base + ICDC_RGDATA_OFFSET) &
			ICDC_RGDATA_RGDOUT_MASK;

	return 0;
}

static int icdc_d3_hw_write_extend(void *context, unsigned int sreg,
				  unsigned int val)
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

	// printk("write extend : sreg: %d [0 - 4], creg: %x sdata: %d\n", sreg, creg, val);

	x1000_codec_reg_read(context, creg, &cdata);
	cdata = (cdata & (~0x3f)) | ((sreg & 0x3f) | 0x40);

	x1000_codec_reg_write(context, creg, cdata);
	x1000_codec_reg_write(context, dreg, val);
	x1000_codec_reg_read(context, dreg, &val_rb);

	if (val != val_rb)
		return -1;

	return 0;
}

static int x1000_codec_reg_write(void *context, unsigned int reg,
				  unsigned int val)
{
	struct jz_codec *codec = context;
	int ret;

	if (reg > SCODA_REG_SR_TR_SRCDAC)
		return icdc_d3_hw_write_extend(context, reg, val);

	ret = x1000_codec_io_wait(codec);
	if (ret)
		return ret;

	writel(ICDC_RGADW_RGWR | (reg << ICDC_RGADW_RGADDR_OFFSET) | val,
	       codec->base + ICDC_RGADW_OFFSET);

	ret = x1000_codec_io_wait(codec);
	if (ret)
		return ret;

	return 0;
}

static const u8 x1000_codec_reg_defaults[] = {
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

static struct regmap_config x1000_codec_regmap_config = {
	.reg_bits = 7,
	.val_bits = 8,

	.max_register = SCODA_MAX_REG_NUM,
	.volatile_reg = x1000_codec_volatile,
	.readable_reg = x1000_codec_readable,
	.writeable_reg = x1000_codec_writeable,

	.reg_read = x1000_codec_reg_read,
	.reg_write = x1000_codec_reg_write,

	.reg_defaults_raw = x1000_codec_reg_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(x1000_codec_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static int x1000_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz_codec *codec;
	int ret;

	codec = devm_kzalloc(dev, sizeof(*codec), GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	codec->dev = dev;

	codec->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(codec->base))
		return PTR_ERR(codec->base);

	codec->regmap = devm_regmap_init(dev, NULL, codec,
					&x1000_codec_regmap_config);
	if (IS_ERR(codec->regmap))
		return PTR_ERR(codec->regmap);

	codec->clk = devm_clk_get(dev, "aic");
	if (IS_ERR(codec->clk))
		return PTR_ERR(codec->clk);

	platform_set_drvdata(pdev, codec);

	ret = devm_snd_soc_register_component(dev, &x1000_codec_soc_codec_dev,
					      &x1000_codec_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id x1000_codec_of_matches[] = {
	{ .compatible = "ingenic,x1000-codec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x1000_codec_of_matches);

static struct platform_driver x1000_codec_driver = {
	.probe			= x1000_codec_probe,
	.driver			= {
		.name		= "x1000-codec",
		.of_match_table = x1000_codec_of_matches,
	},
};
module_platform_driver(x1000_codec_driver);

MODULE_DESCRIPTION("X1000 SoC internal codec driver");
MODULE_AUTHOR("sccheng <shicheng.cheng@ingenic.com>");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL v2");
