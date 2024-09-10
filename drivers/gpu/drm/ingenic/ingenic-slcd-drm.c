// SPDX-License-Identifier: GPL-2.0-or-later
//
// KMS driver for SLCD controller of Ingenic X1000 series SoCs

// Copyright (C) 2023-2024 SudoMaker, Ltd.
// Author: Reimu NotMoe <reimu@sudomaker.com>

// The SLCDC of X1000 series SoCs does not work in the same way as the LCD
// controller found in previous JZ series SoCs. It only supports "smart LCDs"
// with 8080/6800 style parallel bus. So creating a new driver is more logical.

// The SLCDC do contain a stripped down LCDC from previous JZ series SoCs, but it's
// only used for DMA management and most original LCDC registers are now useless,
// **despite their existence in the reference manual** (X1000_PM.pdf).
// In the terms of LCDC, it only contains one usable foreground and DMA channel.

// PIO operations are usable as long as the clock is provided and SLCD related
// registers are properly configured. There isn't a separate enable register for
// the SLCD part. LCDCTRL.ENA only controls DMA operations, as mentioned above.

// Pitfalls not mentioned in datasheets:

// - LCDCMD.FRM_EN is only effective in data descriptors (LCDCMD.CMD=0).
// For command descriptors (LCDCMD.CMD=1) it's always enabled.

// - LCDCMD<31:30> are only effective in data descriptors (LCDCMD.CMD=0).
// For command descriptors (LCDCMD.CMD=1) no interrupts will fire.

// - Descriptors must appear in pairs of command and data. e.g. cmd0->data0->
// cmd1->data1->cmd0->data0. Otherwise it will not work.

// - LCDCNUM (aka LCDCPOS<7:0>) actually denotes how many bytes (or maybe bus
// transfers, I don't have hardware to test in 16bit bus mode) that a command
// descriptor contains. So there is no need to "set it to 4 and send 0x2c2c2c2c".

// - MCFG_NEW.DTIMES will affect PIO operations. It must be 0 for PIO to work
// correctly (overflowed bits got stripped). Otherwise all bits in MDATA<23:0>
// will be sent each in the size of bus width, which is often undesired when
// sending commands.

// - 24bit compressed mode (LCDCPOS.BPP0=6) does not work.

// - TE waiting is still activated in PIO mode


// Notes:

// 1. On some controllers, changing the rotation (0x36) & window setting
// (0x2a & 0x2b) immediately after turning on the display would result in a
// blank screen and/or incorrect rotation being set. As a workaround, you can
// surround the rotation set commands with turn off display (0x28) and turn on
// display (0x29) commands in "rotate-sequence-x" properties.

// 2. If the image is completely broken in Xorg, check if your current display
// resolution is smaller than 320x200. This is a weird man-made restriction and
// you need to patch Xorg to remove it.


// ChangeLog:

// 2024-01-06: Fix RGB565 byte swap
// 2024-04-11: Simplify colorspace selection & fix TE PIO bug
// 2024-09-10: Examined Ingenic's old 3.10 kernel source and confirmed the absence of FG1


#include "ingenic-slcd-drm.h"

#include <linux/bitfield.h>
#include <linux/component.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_format_helper.h>

/*
 * Assume a monitor resolution of 96 dpi to
 * get a somewhat reasonable screen size.
 */
#define RES_MM(d)	\
	(((d) * 254ul) / (96ul * 10ul))

#define SIMPLEDRM_MODE(hd, vd)	\
	DRM_SIMPLE_MODE(hd, vd, RES_MM(hd), RES_MM(vd))

struct ingenic_dma_hwdesc {
	u32 next;
	u32 addr;
	u32 id;
	u32 cmd;
	u32 offsize;
	u32 pagewidth;
	u32 cpos;
	u32 descsize;
};

// DMA descriptors allocation:
// desc[0]: cmd[0]: command[0]<7:0> = 0x2c (memory write)
// desc[1]: dat[0]: pixel data, updated in display pipe functions

struct ingenic_dma_hwdescs {
	struct ingenic_dma_hwdesc hwdesc[2];	// size: 0x20
	uint32_t command[4];			// size: 0x10
};

struct ingenic_slcd_runtime_data {
	bool in_dma_mode;
	bool vsync_enabled;
	bool dma_running;
	int rotation;
	u32 dtimes_dma;
	u32 pitch;
	struct timer_list idle_timer;
	struct drm_rect old_rect;
};

struct ingenic_slcd_display {
	u16 width;
	u16 height;
	u8 buswidth;
	u8 bpp_vram;
	u8 bpp_panel;
	u16 rotation_default;
	u16 reported_fps;

	u32 bus_max_speed;
	u8 sequence_format;
	u8 debug;

	bool use_te;
	bool te_active_low;
	bool bus_speed_advantage;
	bool cs_active_high;
	bool wr_active_high;
	bool dc_command_high;

	u16 *init_sequence;
	u16 *sleep_sequence;
	u16 *resume_sequence;
	u16 *enable_sequence;
	u16 *disable_sequence;
	u16 *write_sequence;
	u16 *window_sequence;
	unsigned window_sequence_len;

	u16 *rotate_sequence_0;
	u16 *rotate_sequence_90;
	u16 *rotate_sequence_180;
	u16 *rotate_sequence_270;

	struct gpio_desc *gpio_reset, *gpio_rd;
};

struct ingenic_slcd_drm {
	// Driver internals
	struct ingenic_slcd_runtime_data rt_data;
	struct ingenic_slcd_display disp;

	// DRM stuff
	struct drm_device drm;
	struct drm_display_mode mode[4];
	const struct drm_format_info *format;
	struct drm_connector connector;
	struct drm_simple_display_pipe pipe;

	// Hardware stuff
	struct device *dev;
	struct regmap *map;
	struct clk *clk_lcd;

	struct ingenic_dma_hwdescs *dma_hwdescs;
	dma_addr_t dma_hwdescs_phys;

};

struct ingenic_slcd_drm_bridge {
	struct drm_encoder encoder;
	struct drm_bridge bridge, *next_bridge;

	struct drm_bus_cfg bus_cfg;
};

#if 0
#define x1000_slcdc_debug(dev, str, ...)		dev_info(dev, str, ##__VA_ARGS__)
#else
#define x1000_slcdc_debug(dev, str, ...)		if (0) dev_info(dev, str, ##__VA_ARGS__)
#endif

static inline struct ingenic_slcd_drm_bridge *
to_ingenic_slcdc_bridge(struct drm_encoder *encoder)
{
	return container_of(encoder, struct ingenic_slcd_drm_bridge, encoder);
}

static const struct regmap_config ingenic_slcdc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static inline struct ingenic_slcd_drm *drm_device_get_priv(struct drm_device *drm)
{
	return container_of(drm, struct ingenic_slcd_drm, drm);
}

static inline dma_addr_t dma_hwdesc_addr(const struct ingenic_slcd_drm *priv,
					 unsigned int idx)
{
	u32 offset = offsetof(struct ingenic_dma_hwdescs, hwdesc[idx]);

	return priv->dma_hwdescs_phys + offset;
}

static inline dma_addr_t dma_command_addr(const struct ingenic_slcd_drm *priv,
					 unsigned int idx)
{
	u32 offset = offsetof(struct ingenic_dma_hwdescs, command[idx]);

	return priv->dma_hwdescs_phys + offset;
}

static inline bool is_valid_degree(unsigned val) {
	val %= 360;

	return (val % 90) == 0;
}

static void ingenic_slcdc_execute_panel_sequence(struct ingenic_slcd_drm *priv, u16 *seq);

static inline void ingenic_slcdc_quick_enable(struct ingenic_slcd_drm *priv)
{
	regmap_update_bits(priv->map, JZ_REG_LCD_CTRL, JZ_LCD_CTRL_ENABLE, JZ_LCD_CTRL_ENABLE);
}

static inline void ingenic_slcdc_quick_disable(struct ingenic_slcd_drm *priv)
{
	regmap_update_bits(priv->map, JZ_REG_LCD_CTRL, JZ_LCD_CTRL_ENABLE, 0);
}

static inline void ingenic_slcdc_poll_busy(struct ingenic_slcd_drm *priv)
{
	u32 var;
	regmap_read_poll_timeout(priv->map, JZ_REG_SLCD_MSTATE, var,
				 (var & JZ_SLCD_MSTATE_BUSY) == 0, 200, 1 * 1000 * 1000);
}

static inline void ingenic_slcdc_poll_disable(struct ingenic_slcd_drm *priv)
{
	u32 var;
	regmap_read_poll_timeout(priv->map, JZ_REG_LCD_STATE, var,
				 var & JZ_LCD_STATE_QD, 200, 1 * 1000 * 1000);

	regmap_update_bits(priv->map, JZ_REG_LCD_STATE, JZ_LCD_STATE_QD, 0);
}

static inline void ingenic_slcdc_pio_write_command(struct ingenic_slcd_drm *priv, u16 cmd)
{
	u8 sf = priv->disp.sequence_format;
	u32 reg;

	// cmd = cpu_to_be16(cmd);

	if (sf == 1) {
		ingenic_slcdc_poll_busy(priv);
		reg = (u32)((cmd >> 8) & 0xff) | (1 << 30);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, reg);
		ingenic_slcdc_poll_busy(priv);
		reg = (u32)((cmd >> 0) & 0xff) | (1 << 30);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, reg);
	} else if (sf == 2) {
		ingenic_slcdc_poll_busy(priv);
		reg = (u32)((cmd >> 0) & 0xff) | (1 << 30);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, reg);
		ingenic_slcdc_poll_busy(priv);
		reg = (u32)((cmd >> 8) & 0xff) | (1 << 30);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, reg);
	} else {
		ingenic_slcdc_poll_busy(priv);
		reg = (u32)cmd | (1 << 30);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, reg);
	}
}

static inline void ingenic_slcdc_pio_write_data(struct ingenic_slcd_drm *priv, u16 dat)
{
	u8 sf = priv->disp.sequence_format;

	if (sf == 1) {
		ingenic_slcdc_poll_busy(priv);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, (u32)((dat >> 8) & 0xff));
		ingenic_slcdc_poll_busy(priv);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, (u32)((dat >> 0) & 0xff));
	} else if (sf == 2) {
		ingenic_slcdc_poll_busy(priv);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, (u32)((dat >> 0) & 0xff));
		ingenic_slcdc_poll_busy(priv);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, (u32)((dat >> 8) & 0xff));
	} else {
		ingenic_slcdc_poll_busy(priv);
		regmap_write(priv->map, JZ_REG_SLCD_MDATA, (u32)dat);
	}
}

static inline void ingenic_slcdc_set_pio_mode(struct ingenic_slcd_drm *priv)
{
	if (!priv->rt_data.in_dma_mode)
		return;

	// Reduce clock speed when PIO mode
	// See the comments in ingenic_slcdc_configure_hardware() for reason
	clk_set_rate(priv->clk_lcd, 4687500);

	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_NOT_USE_TE|JZ_SLCD_MCTRL_GATE_MASK, JZ_SLCD_MCTRL_NOT_USE_TE);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_DTIMES_MASK | JZ_SLCD_MCFGNEW_FMT_CONV, JZ_SLCD_MCFGNEW_DTIMES_1);

	priv->rt_data.in_dma_mode = false;
}

static inline void ingenic_slcdc_set_dma_mode(struct ingenic_slcd_drm *priv)
{
	struct ingenic_slcd_runtime_data *rt_data = &priv->rt_data;

	if (priv->rt_data.in_dma_mode)
		return;

	ingenic_slcdc_poll_busy(priv);

	// Peripheral clock speed is double of bus speed
	// See X1000_PM.pdf p.87
	clk_set_rate(priv->clk_lcd, priv->disp.bus_max_speed * 2);

	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_NOT_USE_TE|JZ_SLCD_MCTRL_GATE_MASK,
		(priv->disp.use_te ? 0 : JZ_SLCD_MCTRL_NOT_USE_TE)|JZ_SLCD_MCTRL_GATE_MASK);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_DTIMES_MASK | JZ_SLCD_MCFGNEW_FMT_CONV, rt_data->dtimes_dma | JZ_SLCD_MCFGNEW_FMT_CONV);

	priv->rt_data.in_dma_mode = true;
}

DEFINE_DRM_GEM_DMA_FOPS(ingenic_slcdc_fops);

static const struct drm_driver ingenic_slcdc_driver_data = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name			= "ingenic-slcd-drm",
	.desc			= "DRM module for Ingenic X1000 SLCD controller",
	.date			= "20240411",
	.major			= 1,
	.minor			= 1,
	.patchlevel		= 0,
	.fops			= &ingenic_slcdc_fops,
	DRM_GEM_DMA_DRIVER_OPS,
};

static void __maybe_unused ingenic_slcdc_release_rmem(void *d)
{
	of_reserved_mem_device_release(d);
}

static void ingenic_slcdc_configure_hwdesc(struct ingenic_slcd_drm *priv)
{
	struct ingenic_dma_hwdescs *dma_hwdescs = priv->dma_hwdescs;
	struct ingenic_dma_hwdesc *hd = &dma_hwdescs->hwdesc[0];
	struct ingenic_slcd_display *disp = &priv->disp;
	u32 write_cmd = 0x2c;
	u16 sf = disp->sequence_format;

	u32 cpos_dd = disp->bpp_vram == 32 ? (5 << 27) : (4 << 27);
	u32 descsize_dd = (disp->width - 1) | ((disp->height - 1) << 12);
	u32 len = disp->width * disp->height;

	// This magic comes from Ingenic folks; NEEDS REVISIT
	if (priv->disp.bpp_vram == 16)
		len = (len + 1) / 2;

	if (disp->write_sequence)
		write_cmd = disp->write_sequence[1];

	if (sf == 1) {
		write_cmd = be16_to_cpu(write_cmd);
		hd[0].cpos = 2;
	} else if (sf == 2) {
		hd[0].cpos = 2;
	} else {
		hd[0].cpos = 1;
	}

	dma_hwdescs->command[0] = write_cmd;

	hd[0].next = dma_hwdesc_addr(priv, 1);
	hd[0].addr = dma_command_addr(priv, 0);
	hd[0].id = 0xcafebabe;
	hd[0].cmd = 0x24000001;
	hd[0].offsize = 0;
	hd[0].pagewidth = 0;

	hd[1].next = dma_hwdesc_addr(priv, 0);
	hd[1].addr = 0;
	hd[1].id = 0xdeadbeef;
	hd[1].cmd = (1 << 26) | (1 << 30) | len;
	hd[1].offsize = 0;
	hd[1].pagewidth = 0;
	hd[1].cpos = cpos_dd;
	hd[1].descsize = descsize_dd;
}

static inline void ingenic_slcdc_update_hwdesc_fb_phys(struct ingenic_slcd_drm *priv, dma_addr_t fb_phys) {
	priv->dma_hwdescs->hwdesc[1].addr = fb_phys;
}


static const u16 ingenic_slcdc_default_window_sequence[] = {
	1, 0x2a, 4, 0, 4, 1, 4, 2, 4, 3,
	1, 0x2b, 4, 4, 4, 5, 4, 6, 4, 7,
};

static void ingenic_slcdc_update_panel_window(struct ingenic_slcd_drm *priv, bool rotated) {
	struct ingenic_slcd_display *disp = &priv->disp;
	unsigned stpl_len = disp->window_sequence_len;
	const u16 *stpl = disp->window_sequence;
	u16 *seq;
	u16 type, value;

	unsigned x1 = 0, x2 = disp->width - 1, y1 = 0, y2 = disp->height - 1;

	if (rotated) {
		x2 = disp->height - 1;
		y2 = disp->width - 1;
	} else {
		x2 = disp->width - 1;
		y2 = disp->height - 1;
	}

	if (!stpl) {
		stpl = ingenic_slcdc_default_window_sequence;
		stpl_len = ARRAY_SIZE(ingenic_slcdc_default_window_sequence);
	}

	// I hear ya, no VLAs!
	// Let's kmalloc() for this less-than-50-bytes array! 🤡
	seq = kmalloc((stpl_len + 2) * sizeof(u16), GFP_KERNEL);
	BUG_ON(!seq);

	seq[stpl_len] = 0;
	seq[stpl_len+1] = 0;

	for (unsigned i=0; i<stpl_len; i++) {
		if (i % 2) { // Odd: Value
			value = stpl[i];

			if (type == 4) { // Type is template
				seq[i-1] = 2; // Filled type is data
				switch (value) {
				case 0:
					// x1<15:8>
					seq[i] = x1 >> 8;
					break;
				case 1:
					// x1<7:0>
					seq[i] = x1 & 0xff;
					break;
				case 2:
					// x2<15:8>
					seq[i] = x2 >> 8;
					break;
				case 3:
					// x2<7:0>
					seq[i] = x2 & 0xff;
					break;
				case 4:
					// y1<15:8>
					seq[i] = y1 >> 8;
					break;
				case 5:
					// y1<7:0>
					seq[i] = y1 & 0xff;
					break;
				case 6:
					// y2<15:8>
					seq[i] = y2 >> 8;
					break;
				case 7:
					// y2<7:0>
					seq[i] = y2 & 0xff;
					break;
				case 16:
					// x1<15:0>
					seq[i] = x1 & 0xffff;
					break;
				case 17:
					// x2<15:0>
					seq[i] = x2 & 0xffff;
					break;
				case 18:
					// y1<15:0>
					seq[i] = y1 & 0xffff;
					break;
				case 19:
					// y2<15:0>
					seq[i] = y2 & 0xffff;
					break;
				default:
					break;
				}
			} else { // Type is not template, just copy
				seq[i-1] = type;
				seq[i] = value;
			}
		} else { // Even: Type
			type = stpl[i];
		}
	}

	ingenic_slcdc_execute_panel_sequence(priv, seq);

	kfree(seq);

	x1000_slcdc_debug(priv->dev, "panel window: [%u %u] [%u %u]\n", x1, x2, y1, y2);
}

static inline void ingenic_slcdc_dma_cont(struct ingenic_slcd_drm *priv) {
	if (priv->rt_data.dma_running)
		return;
	// Set DMAMODE to 0 (continually transfer data)
	// Set DMASTART to 1
	// Set DMATXEN to 1
	u32 mask = JZ_SLCD_MCTRL_DMAMODE|JZ_SLCD_MCTRL_DMATXEN|JZ_SLCD_MCTRL_DMASTART;
	u32 val = JZ_SLCD_MCTRL_DMATXEN|JZ_SLCD_MCTRL_DMASTART;
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, mask, val);
	priv->rt_data.dma_running = true;
}

static inline void ingenic_slcdc_dma_pause(struct ingenic_slcd_drm *priv) {
	// Set DMAMODE to 1 (stop when one descriptor finished)
	// Set DMASTART to 0
	// Set DMATXEN to 0
	u32 mask = JZ_SLCD_MCTRL_DMAMODE|JZ_SLCD_MCTRL_DMASTART|JZ_SLCD_MCTRL_DMATXEN;
	u32 val = JZ_SLCD_MCTRL_DMAMODE;
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, mask, val);
	ingenic_slcdc_poll_busy(priv);
	priv->rt_data.dma_running = false;
}

static inline void ingenic_slcdc_dma_once(struct ingenic_slcd_drm *priv) {
	// Set DMAMODE to 1 (stop when one descriptor finished)
	// Set DMASTART to 1
	// Set DMATXEN to 1
	u32 mask = JZ_SLCD_MCTRL_DMAMODE|JZ_SLCD_MCTRL_DMATXEN|JZ_SLCD_MCTRL_DMASTART;
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, mask, mask);
}

static void ingenic_slcdc_controller_enable(struct ingenic_slcd_drm *priv) {
	ingenic_slcdc_poll_busy(priv);
	regmap_write(priv->map, JZ_REG_LCD_DA0, priv->dma_hwdescs_phys);
	ingenic_slcdc_quick_enable(priv);
}

static void ingenic_slcdc_controller_disable(struct ingenic_slcd_drm *priv) {
	ingenic_slcdc_dma_pause(priv);
	ingenic_slcdc_quick_disable(priv);
	ingenic_slcdc_poll_disable(priv);
}

static irqreturn_t ingenic_slcdc_irq_handler(int irq, void *arg)
{
	struct ingenic_slcd_drm *priv = drm_device_get_priv(arg);
	static uint64_t last_time;
	uint64_t now_time;
	unsigned int state;

	regmap_read(priv->map, JZ_REG_LCD_STATE, &state);

	regmap_update_bits(priv->map, JZ_REG_LCD_STATE,
			   JZ_LCD_STATE_EOF_IRQ, 0);

	if (state & JZ_LCD_STATE_EOF_IRQ) {
		now_time = ktime_get_ns();
		x1000_slcdc_debug(priv->dev, "EOF, %llu ns\n", now_time - last_time);
		last_time = now_time;
		drm_crtc_handle_vblank(&priv->pipe.crtc);
	}

	return IRQ_HANDLED;
}

/* returns 0 if the property is not present */
static u32 ingenic_slcdc_property_value(struct device *dev, const char *propname)
{
	int ret;
	u32 val = 0;

	ret = device_property_read_u32(dev, propname, &val);
	if (ret == 0)
		dev_info(dev, "%s: %s = %u\n", __func__, propname, val);

	return val;
}

// @return		0 is success
static int ingenic_slcdc_property_array_value(struct device *dev, const char *propname, u16 **out_values,
	unsigned *out_count)
{
	int count, ret;
	u16 *values;

	count = device_property_count_u16(dev, propname);
	if (count < 0)
		return count;
	if (count == 0 || count % 2)
		return -EINVAL;

	values = kmalloc_array(count + 2, sizeof(*values), GFP_KERNEL);
	if (!values)
		return -ENOMEM;

	values[count] = 0;
	values[count+1] = 0;

	ret = device_property_read_u16_array(dev, propname, values, count);
	if (ret) {
		dev_err(dev, "unable to read props\n");
		kfree(values);
	} else {
		if (out_values)
			*out_values = values;
		if (out_count)
			*out_count = count;
	}

	return ret;
}

static struct drm_display_mode ingenic_slcdc_display_mode(u16 width,
					      u16 height, u16 fps, const char *tag)
{
	struct drm_display_mode mode = { SIMPLEDRM_MODE(width, height) };

	mode.clock = mode.hdisplay * mode.vdisplay * fps / 1000 /* kHz */;

	snprintf(mode.name, DRM_DISPLAY_MODE_LEN, "%dx%d%s",
			 mode.hdisplay, mode.vdisplay, tag);

	return mode;
}

static struct gpio_desc *ingenic_slcdc_request_gpio(struct ingenic_slcd_drm *priv, const char *id) {
	struct gpio_desc *ret;

	ret = devm_gpiod_get_optional(priv->dev, id, GPIOD_OUT_HIGH);
	if (IS_ERR(ret)) {
		dev_err_probe(priv->dev, PTR_ERR(ret), "Failed to get GPIO '%s'\n", id);
		return NULL;
	}
	if (!ret)
		dev_info(priv->dev, "GPIO '%s' is unspecified\n", id);

	return ret;
}

static int ingenic_slcdc_properties_load_examine(struct ingenic_slcd_drm *priv)
{
	struct device *dev = priv->dev;
	struct ingenic_slcd_runtime_data *rt_data = &priv->rt_data;
	struct ingenic_slcd_display *disp = &priv->disp;

	unsigned int vals[2];

	if (!dev_fwnode(dev)) {
		dev_err(dev, "Missing platform data or properties\n");
		return -EINVAL;
	}

	disp->width = ingenic_slcdc_property_value(dev, "width");
	disp->height = ingenic_slcdc_property_value(dev, "height");

	vals[0] = disp->width;
	vals[1] = disp->height;

	for (int i=0; i<(sizeof(vals)/sizeof(unsigned int)); i++) {
		unsigned int val = vals[i];

		if (val < 32 || val > 1024 || val % 4) {
			dev_err(dev, "width and height must between 32..1024 and is multiple of 4\n");
			return -EINVAL;
		}
	}

	disp->buswidth = ingenic_slcdc_property_value(dev, "buswidth");

	switch (disp->buswidth) {
	case 8:
	case 16:
		break;
	default:
		dev_err(dev, "buswidth must be 8 or 16\n");
		return -EINVAL;
	}

	disp->bpp_vram = ingenic_slcdc_property_value(dev, "bpp-vram");

	switch (disp->bpp_vram) {
	case 16:
		priv->format = drm_format_info(DRM_FORMAT_RGB565);
		rt_data->pitch = disp->width * 2;
		break;
	case 32:
		priv->format = drm_format_info(DRM_FORMAT_XRGB8888);
		rt_data->pitch = disp->width * 4;
		break;
	default:
		dev_err(dev, "bpp-vram must be 16 (RGB565) or 32 (XRGB8888)\n");
		return -EINVAL;
	}

	disp->bpp_panel = ingenic_slcdc_property_value(dev, "bpp-panel");

	switch (disp->bpp_panel) {
	case 8:
		rt_data->dtimes_dma = JZ_SLCD_MCFGNEW_DTIMES_1;
		break;
	case 16:
		if (disp->buswidth == 16)
			rt_data->dtimes_dma = JZ_SLCD_MCFGNEW_DTIMES_1;
		else
			rt_data->dtimes_dma = JZ_SLCD_MCFGNEW_DTIMES_2;
		break;
	case 24:
		if (disp->buswidth == 16)
			rt_data->dtimes_dma = JZ_SLCD_MCFGNEW_DTIMES_2;
		else
			rt_data->dtimes_dma = JZ_SLCD_MCFGNEW_DTIMES_3;
		break;
	default:
		dev_err(dev, "bpp-panel must be 8, 16 or 24\n");
		return -EINVAL;
	}

	disp->debug = ingenic_slcdc_property_value(dev, "debug");
	disp->rotation_default = ingenic_slcdc_property_value(dev, "rotation-default");
	disp->reported_fps = ingenic_slcdc_property_value(dev, "reported-fps");

	if (!disp->reported_fps)
		disp->reported_fps = 60;

	if (!is_valid_degree(disp->rotation_default)) {
		dev_err(dev, "rotation-default must be 0, 90, 180, 270\n");
		return -EINVAL;
	}

	rt_data->rotation = -1;

	disp->use_te = ingenic_slcdc_property_value(dev, "use-te");
	disp->te_active_low = ingenic_slcdc_property_value(dev, "te-active-low");
	disp->bus_speed_advantage = ingenic_slcdc_property_value(dev, "bus-speed-advantage");

	disp->bus_max_speed = ingenic_slcdc_property_value(dev, "bus-max-speed");

	// Sequence format:
	// 0 - (Default) No conversion, clipped to bus width. e.g. 0x2c -> 0x2c
	// 1 - 16bit to 2x8bit, big endian. e.g. 0x2c00 -> 0x2c 0x00 (such as NT35510 and R61509V)
	// 2 - 16bit to 2x8bit, little endian. e.g. 0x2c00 -> 0x00 0x2c
	disp->sequence_format = ingenic_slcdc_property_value(dev, "sequence-format");

	ingenic_slcdc_property_array_value(dev, "init-sequence", &disp->init_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "sleep-sequence", &disp->sleep_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "resume-sequence", &disp->resume_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "enable-sequence", &disp->enable_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "disable-sequence", &disp->disable_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "write-sequence", &disp->write_sequence, NULL);
	ingenic_slcdc_property_array_value(dev, "window-sequence", &disp->window_sequence, &disp->window_sequence_len);
	ingenic_slcdc_property_array_value(dev, "rotate-sequence-0", &disp->rotate_sequence_0, NULL);
	ingenic_slcdc_property_array_value(dev, "rotate-sequence-90", &disp->rotate_sequence_90, NULL);
	ingenic_slcdc_property_array_value(dev, "rotate-sequence-180", &disp->rotate_sequence_180, NULL);
	ingenic_slcdc_property_array_value(dev, "rotate-sequence-270", &disp->rotate_sequence_270, NULL);

	// Change FPS a little bit to prevent 180 and 270 get discarded by the arrogant drm internals
	priv->mode[0] = ingenic_slcdc_display_mode(disp->width, disp->height, disp->reported_fps - 1, "_0");
	priv->mode[1] = ingenic_slcdc_display_mode(disp->height, disp->width, disp->reported_fps - 1, "_90");
	priv->mode[2] = ingenic_slcdc_display_mode(disp->width, disp->height, disp->reported_fps + 1, "_180");
	priv->mode[3] = ingenic_slcdc_display_mode(disp->height, disp->width, disp->reported_fps + 1, "_270");

	disp->gpio_reset = ingenic_slcdc_request_gpio(priv, "reset");
	disp->gpio_rd = ingenic_slcdc_request_gpio(priv, "rd");

	return 0;
}

static void ingenic_slcdc_execute_panel_sequence(struct ingenic_slcd_drm *priv, u16 *seq) {
	u16 type, value;

	if (!seq)
		return;

	ingenic_slcdc_set_pio_mode(priv);

	for (unsigned i=0; ; i++) {
		if (i % 2) { // Odd: Value
			value = seq[i];

			switch (type) {
			case 1: // Command
				ingenic_slcdc_pio_write_command(priv, value);
				x1000_slcdc_debug(priv->dev, "command: %x\n", value);
				break;
			case 2: // Data
				ingenic_slcdc_pio_write_data(priv, value);
				x1000_slcdc_debug(priv->dev, "data: %x\n", value);
				break;
			case 3: // Sleep (ms)
				x1000_slcdc_debug(priv->dev, "sleep: %x\n", value);
				msleep(value);
				break;
			default:
				x1000_slcdc_debug(priv->dev, "unknown type %x: %x\n", type, value);
				break;
			}
		} else { // Even: Type
			type = seq[i];
			if (type == 0) {
				x1000_slcdc_debug(priv->dev, "done, cnt=%u\n", i);
				break;
			}
		}
	}

}

static void ingenic_slcdc_reset_panel(struct ingenic_slcd_drm *priv)
{
	struct ingenic_slcd_display *disp = &priv->disp;

	if (disp->gpio_rd)
		gpiod_set_value_cansleep(disp->gpio_rd, 0);

	if (!disp->gpio_reset)
		return;

	x1000_slcdc_debug(priv->dev, "resetting panel...\n");

	gpiod_set_value_cansleep(disp->gpio_reset, 1);
	msleep(50);
	gpiod_set_value_cansleep(disp->gpio_reset, 0);
	msleep(120);
}

static int ingenic_slcdc_configure_hardware(struct ingenic_slcd_drm *priv)
{
	struct ingenic_slcd_display *disp = &priv->disp;
	u32 cwidth, dwidth;
	int ret;

	// At the init stage we reduce the bus speed as much as we can to
	// accommodate controllers that need PLL configuration (such as the SSD1963).
	// 600MHz / 128 = 4.6875MHz. The MPLL is usually 600MHz and the divider has
	// 8bit precision so we can always get a reasonably low frequency even you
	// have doubled or halved MPLL's speed.
	clk_set_rate(priv->clk_lcd, 4687500);

	ret = clk_prepare_enable(priv->clk_lcd);
	if (ret) {
		dev_err(priv->dev, "Unable to start SLCDC clock\n");
		return ret;
	}

	ingenic_slcdc_quick_disable(priv);
	ingenic_slcdc_poll_busy(priv);

	regmap_write(priv->map, JZ_REG_LCD_STATE, 0);

	regmap_write(priv->map, JZ_REG_SLCD_WTIME, 0);
	regmap_write(priv->map, JZ_REG_SLCD_TASH, 0);
	regmap_write(priv->map, JZ_REG_SLCD_SMWT, 0);

	regmap_write(priv->map, JZ_REG_LCD_DAH, disp->width);
	regmap_write(priv->map, JZ_REG_LCD_DAV, disp->height);

	switch (disp->buswidth) {
	case 8:
		cwidth = JZ_SLCD_MCFG_CWIDTH_8;
		dwidth = JZ_SLCD_MCFGNEW_DWIDTH_8;
		break;
	case 16:
		cwidth = JZ_SLCD_MCFG_CWIDTH_16_9;
		dwidth = JZ_SLCD_MCFGNEW_DWIDTH_16;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG, JZ_SLCD_MCFG_CWIDTH_MASK, cwidth);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_DWIDTH_MASK, dwidth);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_CMD_9BIT, 0);

	// Configure TE but keep it disabled in PIO stages
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_NOT_USE_TE | JZ_SLCD_MCTRL_NARROW_TE,
		JZ_SLCD_MCTRL_NOT_USE_TE | JZ_SLCD_MCTRL_NARROW_TE);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_TE_INV,
		disp->te_active_low ? JZ_SLCD_MCTRL_TE_INV : 0);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_FAST_MODE,
		disp->bus_speed_advantage ? JZ_SLCD_MCTRL_FAST_MODE : 0);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_CSPOL,
		disp->cs_active_high ? JZ_SLCD_MCFGNEW_CSPOL : 0);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_6800_MD,
		disp->wr_active_high ? JZ_SLCD_MCFGNEW_6800_MD : 0);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_RSPOL,
		disp->dc_command_high ? JZ_SLCD_MCFGNEW_RSPOL : 0);
	regmap_update_bits(priv->map, JZ_REG_SLCD_MCFG_NEW, JZ_SLCD_MCFGNEW_FMT_CONV, 0);

	ingenic_slcdc_reset_panel(priv);
	priv->rt_data.in_dma_mode = true; // Stupid hack
	ingenic_slcdc_execute_panel_sequence(priv, priv->disp.init_sequence);

	regmap_update_bits(priv->map, JZ_REG_LCD_CTRL, JZ_LCD_CTRL_BURST_MASK,
		JZ_LCD_CTRL_BURST_64);

	ingenic_slcdc_controller_enable(priv);

	return 0;
}

static ssize_t ingenic_slcdc_sysfs_show_rotation_default(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ingenic_slcd_drm *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", priv->disp.rotation_default);
}

static ssize_t ingenic_slcdc_sysfs_set_rotation_default(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct ingenic_slcd_drm *priv = dev_get_drvdata(dev);

	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err) {
		return err;
	}

	if (!is_valid_degree(val))
		return -EINVAL;

	priv->disp.rotation_default = val;

	return count;
}

static struct device_attribute ingenic_slcdc_device_attrs_real[] = {
	__ATTR(rotation_default, S_IRUGO | S_IWUSR,
		ingenic_slcdc_sysfs_show_rotation_default,
		ingenic_slcdc_sysfs_set_rotation_default),
};

static struct attribute *ingenic_slcdc_device_attrs[] = {
	&ingenic_slcdc_device_attrs_real[0].attr,
	NULL,
};

static const struct attribute_group ingenic_slcdc_device_attr_group = {
	.attrs = ingenic_slcdc_device_attrs,
};

static const uint64_t ingenic_slcdc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int ingenic_slcdc_rotation_from_mode(struct ingenic_slcd_drm *priv,
		struct drm_display_mode *mode)
{
	struct drm_display_mode *pmode;

	for (int i=0; i<4; i++) {
		pmode = &priv->mode[i];

		if (mode->hdisplay == pmode->hdisplay &&
	    	mode->vdisplay == pmode->vdisplay &&
	    	mode->clock == pmode->clock)
			return i * 90;
	}

	return -1;

}

static void ingenic_slcdc_set_rotation(struct ingenic_slcd_drm *priv,
		       int deg)
{
	struct ingenic_slcd_display *disp = &priv->disp;

	u16 *seq = NULL;
	bool r = false;

	switch (deg) {
	case 0:
		seq = disp->rotate_sequence_0;
		break;
	case 90:
		seq = disp->rotate_sequence_90;
		r = true;
		break;
	case 180:
		seq = disp->rotate_sequence_180;
		break;
	case 270:
		seq = disp->rotate_sequence_270;
		r = true;
		break;
	default:
		break;
	}

	x1000_slcdc_debug(priv->dev, "panel rotation changed to %d\n", deg);

	ingenic_slcdc_execute_panel_sequence(priv, seq);
	ingenic_slcdc_update_panel_window(priv, r);
}

static int ingenic_slcdc_connector_helper_get_modes(struct drm_connector *connector)
{
	struct ingenic_slcd_drm *priv = drm_device_get_priv(connector->dev);
	struct drm_display_mode *mode;

	int i;
	int deg2idx = priv->disp.rotation_default / 90;

	for (i=0; i<4; i++) {
		mode = drm_mode_duplicate(connector->dev, &priv->mode[i]);
		if (!mode)
			break;

		if (!mode->name[0])
			drm_mode_set_name(mode);

		if (i == deg2idx)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static const struct drm_connector_helper_funcs ingenic_slcdc_connector_helper_funcs = {
	.get_modes = ingenic_slcdc_connector_helper_get_modes,
};

static const struct drm_connector_funcs ingenic_slcdc_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static enum drm_mode_status
ingenic_slcdc_simple_display_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
				    const struct drm_display_mode *mode)
{
	struct ingenic_slcd_drm *priv = drm_device_get_priv(pipe->crtc.dev);
	struct drm_display_mode *pmode;

	for (int i=0; i<4; i++) {
		pmode = &priv->mode[i];

		if (mode->hdisplay == pmode->hdisplay &&
	   	mode->vdisplay == pmode->vdisplay &&
	   	mode->clock == pmode->clock)
			return MODE_OK;
	}

	return MODE_PANEL;
}

static void
ingenic_slcdc_process_crtc_vblank_event(struct drm_crtc *crtc) {
	struct drm_crtc_state *crtc_state = crtc->state;
	struct drm_pending_vblank_event *vb_event = crtc_state->event;

	if (vb_event) {
		crtc_state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, vb_event);
		else
			drm_crtc_send_vblank_event(crtc, vb_event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static void
ingenic_slcdc_idle_timer_callback(struct timer_list *t)
{
	struct ingenic_slcd_runtime_data *rt_data = from_timer(rt_data, t, idle_timer);
	struct ingenic_slcd_drm *priv = container_of(rt_data, struct ingenic_slcd_drm, rt_data);

	if (rt_data->dma_running) {
		regmap_update_bits(priv->map, JZ_REG_SLCD_MCTRL, JZ_SLCD_MCTRL_DMAMODE, JZ_SLCD_MCTRL_DMAMODE);
		rt_data->dma_running = false;
		// printk("idle\n");
	}
}

static void
ingenic_slcdc_simple_display_pipe_enable(struct drm_simple_display_pipe *pipe,
				     struct drm_crtc_state *crtc_state,
				     struct drm_plane_state *plane_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *dev = crtc->dev;
	struct ingenic_slcd_drm *priv = drm_device_get_priv(dev);
	struct ingenic_slcd_runtime_data *rt_data = &priv->rt_data;
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_display_mode *mode = &crtc_state->mode;

	dma_addr_t dst = 0;
	int rotation;

	timer_setup(&rt_data->idle_timer, ingenic_slcdc_idle_timer_callback, 0);

	ingenic_slcdc_dma_pause(priv);
	rotation = ingenic_slcdc_rotation_from_mode(priv, mode);
	if (rt_data->rotation != rotation) {
		ingenic_slcdc_set_rotation(priv, rotation);
		rt_data->rotation = rotation;
	}
	ingenic_slcdc_execute_panel_sequence(priv, priv->disp.enable_sequence);

	drm_crtc_vblank_on(crtc);

	x1000_slcdc_debug(priv->dev, "display enabled, fb addr: 0x%08x, mode name: %s\n", dst, mode->name);
}

static void
ingenic_slcdc_simple_display_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *dev = crtc->dev;
	struct ingenic_slcd_drm *priv = drm_device_get_priv(dev);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct drm_pending_vblank_event *vb_event = crtc_state->event;

	del_timer(&priv->rt_data.idle_timer);

	drm_crtc_vblank_off(crtc);

	if (vb_event) {
		crtc_state->event = NULL;
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, vb_event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}

	ingenic_slcdc_dma_pause(priv);
	ingenic_slcdc_execute_panel_sequence(priv, priv->disp.disable_sequence);

	x1000_slcdc_debug(priv->dev, "display disabled\n");
}

static void
ingenic_slcdc_simple_display_pipe_update(struct drm_simple_display_pipe *pipe,
				     struct drm_plane_state *old_plane_state)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *dev = crtc->dev;
	struct ingenic_slcd_drm *priv = drm_device_get_priv(dev);
	struct ingenic_slcd_runtime_data *rt_data = &priv->rt_data;
	struct drm_plane_state *plane_state = pipe->plane.state;
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t dst;
	int rotation;

	ingenic_slcdc_process_crtc_vblank_event(crtc);

	if (fb) {
		crtc_state = plane_state->crtc->state;
		dst = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
		ingenic_slcdc_update_hwdesc_fb_phys(priv, dst);

		if (drm_atomic_crtc_needs_modeset(crtc_state)) {
			rotation = ingenic_slcdc_rotation_from_mode(priv, &crtc_state->mode);

			ingenic_slcdc_dma_pause(priv);
			ingenic_slcdc_set_rotation(priv, rotation);
			ingenic_slcdc_execute_panel_sequence(priv, priv->disp.enable_sequence);

			rt_data->rotation = rotation;
			x1000_slcdc_debug(priv->dev, "modeset done\n");
		}

		x1000_slcdc_debug(priv->dev, "display updated, fb addr: 0x%08x\n", dst);
	} else {
		x1000_slcdc_debug(priv->dev, "display updated\n");
	}

	ingenic_slcdc_set_dma_mode(priv);
	ingenic_slcdc_dma_cont(priv);

	mod_timer(&rt_data->idle_timer, jiffies + msecs_to_jiffies(100));
}

static int
ingenic_slcdc_simple_display_pipe_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct ingenic_slcd_drm *priv = drm_device_get_priv(pipe->crtc.dev);

	regmap_update_bits(priv->map, JZ_REG_LCD_CTRL,
			   JZ_LCD_CTRL_EOF_IRQ, JZ_LCD_CTRL_EOF_IRQ);

	priv->rt_data.vsync_enabled = true;
	x1000_slcdc_debug(priv->dev, "HW VSYNC enabled\n");

	return 0;
}

static void
ingenic_slcdc_simple_display_pipe_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct ingenic_slcd_drm *priv = drm_device_get_priv(pipe->crtc.dev);

	regmap_update_bits(priv->map, JZ_REG_LCD_CTRL,
			   JZ_LCD_CTRL_EOF_IRQ, 0);
	regmap_update_bits(priv->map, JZ_REG_LCD_STATE,
			   JZ_LCD_STATE_EOF_IRQ, 0);

	priv->rt_data.vsync_enabled = false;
	x1000_slcdc_debug(priv->dev, "HW VSYNC disabled\n");
}

static const struct drm_simple_display_pipe_funcs
ingenic_slcdc_simple_display_pipe_funcs = {
	.mode_valid = ingenic_slcdc_simple_display_pipe_mode_valid,
	.enable = ingenic_slcdc_simple_display_pipe_enable,
	.disable = ingenic_slcdc_simple_display_pipe_disable,
	.update = ingenic_slcdc_simple_display_pipe_update,
	.enable_vblank = ingenic_slcdc_simple_display_pipe_enable_vblank,
	.disable_vblank = ingenic_slcdc_simple_display_pipe_disable_vblank,
};

static const struct drm_mode_config_funcs ingenic_slcdc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs ingenic_slcdc_mode_config_helpers = {
	/*
	 * Using this function is necessary to commit atomic updates
	 * that need the CRTC to be enabled before a commit, as is
	 * the case with e.g. DSI displays.
	 */
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static int ingenic_slcdc_init_modeset(struct ingenic_slcd_drm *priv)
{
	struct device *dev = priv->dev;
	struct ingenic_slcd_display *disp = &priv->disp;
	struct drm_device *drm = &priv->drm;
	struct drm_connector *connector = &priv->connector;
	struct drm_simple_display_pipe *pipe = &priv->pipe;

	uint32_t format = priv->format->format;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret) {
		dev_err(dev, "drmm_mode_config_init() failed: %d\n", ret);
		return ret;
	}

	drm->mode_config.min_width = 16;
	drm->mode_config.max_width = 1024;
	drm->mode_config.min_height = 16;
	drm->mode_config.max_height = 1024;
	drm->mode_config.preferred_depth = priv->format->cpp[0] * 8;
	drm->mode_config.funcs = &ingenic_slcdc_mode_config_funcs;
	drm->mode_config.helper_private = &ingenic_slcdc_mode_config_helpers;

	ret = drm_connector_init(drm, connector, &ingenic_slcdc_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret) {
		dev_err(dev, "drm_connector_init() failed: %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(connector, &ingenic_slcdc_connector_helper_funcs);
	drm_connector_set_panel_orientation_with_quirk(connector,
						       DRM_MODE_PANEL_ORIENTATION_UNKNOWN,
						       disp->width, disp->height);

	ret = drm_simple_display_pipe_init(drm, pipe, &ingenic_slcdc_simple_display_pipe_funcs,
					   &format, 1, ingenic_slcdc_format_modifiers,
					   connector);

	if (ret) {
		dev_err(dev, "drm_simple_display_pipe_init() failed: %d\n", ret);
		return ret;
	}

	ret = drm_vblank_init(drm, 1);
	if (ret) {
		dev_err(dev, "drm_vblank_init() failed: %d\n", ret);
		return ret;
	}

	drm_mode_config_reset(drm);

	return 0;
}

static int ingenic_slcdc_bind(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenic_slcd_drm *priv;
	struct ingenic_slcd_display *disp;
	struct drm_device *drm;
	void __iomem *base;
	struct resource *res;
	struct regmap_config regmap_config;
	unsigned int sz_dma_hwdescs;
	int ret, irq;

	if (IS_ENABLED(CONFIG_OF_RESERVED_MEM)) {
		ret = of_reserved_mem_device_init(dev);

		if (ret && ret != -ENODEV)
			dev_warn(dev, "Failed to get reserved memory: %d\n", ret);

		if (!ret) {
			ret = devm_add_action_or_reset(dev, ingenic_slcdc_release_rmem, dev);
			if (ret)
				return ret;
		}
	}

	priv = devm_drm_dev_alloc(dev, &ingenic_slcdc_driver_data,
				  struct ingenic_slcd_drm, drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->dev = dev;
	disp = &priv->disp;
	drm = &priv->drm;

	platform_set_drvdata(pdev, priv);

	priv->clk_lcd = devm_clk_get(dev, "lcd");
	if (IS_ERR(priv->clk_lcd)) {
		dev_err(dev, "Failed to get lcd clock\n");
		return PTR_ERR(priv->clk_lcd);
	}

	ret = ingenic_slcdc_properties_load_examine(priv);
	if (ret)
		return ret;

	ret = ingenic_slcdc_init_modeset(priv);
	if (ret)
		return ret;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base)) {
		dev_err(dev, "Failed to get memory resource\n");
		return PTR_ERR(base);
	}

	regmap_config = ingenic_slcdc_regmap_config;
	regmap_config.max_register = res->end - res->start;
	priv->map = devm_regmap_init_mmio(dev, base, &regmap_config);
	if (IS_ERR(priv->map)) {
		dev_err(dev, "Failed to create regmap\n");
		return PTR_ERR(priv->map);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	sz_dma_hwdescs = sizeof(*priv->dma_hwdescs);
	dev_info(dev, "Allocating %u bytes for DMA descriptors\n", sz_dma_hwdescs);
	priv->dma_hwdescs = dmam_alloc_coherent(dev, sz_dma_hwdescs,
						&priv->dma_hwdescs_phys,
						GFP_KERNEL);
	if (!priv->dma_hwdescs) {
		dev_err(dev, "Failed to allocate DMA descriptors\n");
		return -ENOMEM;
	}
	dev_info(dev, "DMA virt: %p\n", priv->dma_hwdescs);

	ingenic_slcdc_configure_hwdesc(priv);

	ret = devm_request_irq(dev, irq, ingenic_slcdc_irq_handler, 0, drm->driver->name, drm);
	if (ret) {
		dev_err(dev, "Unable to install IRQ handler\n");
		return ret;
	}

	ret = ingenic_slcdc_configure_hardware(priv);
	if (ret) {
		dev_err(dev, "Unable to configure SLCDC hardware\n");
		goto err_devclk_disable;
	}

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err(dev, "Failed to register DRM driver\n");
		goto err_devclk_disable;
	}

	ret = sysfs_create_group(&dev->kobj, &ingenic_slcdc_device_attr_group);
	if (ret) {
		dev_err(dev, "Failed to register sysfs files\n");
		goto err_devclk_disable;
	}

	drm_fbdev_generic_setup(drm, 0);

	return 0;

err_devclk_disable:
	clk_disable_unprepare(priv->clk_lcd);

	return ret;
}

static inline void kfree_if_not_null(void *p) {
	if (p)
		kfree(p);
}

static void ingenic_slcdc_unbind(struct device *dev)
{
	struct ingenic_slcd_drm *priv = dev_get_drvdata(dev);
	struct ingenic_slcd_display *disp = &priv->disp;

	ingenic_slcdc_controller_disable(priv);
	drm_dev_unplug(&priv->drm);

	kfree_if_not_null(&disp->init_sequence);
	kfree_if_not_null(&disp->sleep_sequence);
	kfree_if_not_null(&disp->resume_sequence);
	kfree_if_not_null(&disp->enable_sequence);
	kfree_if_not_null(&disp->disable_sequence);
	kfree_if_not_null(&disp->write_sequence);
	kfree_if_not_null(&disp->window_sequence);
	kfree_if_not_null(&disp->rotate_sequence_0);
	kfree_if_not_null(&disp->rotate_sequence_90);
	kfree_if_not_null(&disp->rotate_sequence_180);
	kfree_if_not_null(&disp->rotate_sequence_270);

	dmam_free_coherent(dev, sizeof(*priv->dma_hwdescs), priv->dma_hwdescs, priv->dma_hwdescs_phys);

	clk_disable_unprepare(priv->clk_lcd);
}

static int ingenic_slcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	return ingenic_slcdc_bind(dev);
}

static int ingenic_slcdc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	ingenic_slcdc_unbind(dev);
	return 0;
}

static int __maybe_unused ingenic_slcdc_suspend(struct device *dev)
{
	struct ingenic_slcd_drm *priv = dev_get_drvdata(dev);

	int ret = drm_mode_config_helper_suspend(&priv->drm);
	if (ret)
		return ret;

	ingenic_slcdc_controller_disable(priv);
	ingenic_slcdc_execute_panel_sequence(priv, priv->disp.sleep_sequence);
	ingenic_slcdc_poll_busy(priv);

	clk_disable_unprepare(priv->clk_lcd);

	dev_info(dev, "suspend\n");

	return 0;
}

static int __maybe_unused ingenic_slcdc_resume(struct device *dev)
{
	struct ingenic_slcd_drm *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(priv->clk_lcd);
	if (ret) {
		dev_err(dev, "Unable to start SLCDC clock again\n");
		return ret;
	}

	ingenic_slcdc_execute_panel_sequence(priv, priv->disp.resume_sequence);
	ingenic_slcdc_poll_busy(priv);
	ingenic_slcdc_controller_enable(priv);

	dev_info(dev, "resume\n");

	return drm_mode_config_helper_resume(&priv->drm);
}

static SIMPLE_DEV_PM_OPS(ingenic_slcdc_pm_ops, ingenic_slcdc_suspend, ingenic_slcdc_resume);

static const struct of_device_id ingenic_slcdc_of_match[] = {
	{ .compatible = "ingenic,x1000-slcd", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ingenic_slcdc_of_match);

static struct platform_driver ingenic_slcdc_driver = {
	.driver = {
		.name = "ingenic-slcd-drm",
		.pm = pm_ptr(&ingenic_slcdc_pm_ops),
		.of_match_table = of_match_ptr(ingenic_slcdc_of_match),
	},
	.probe = ingenic_slcdc_probe,
	.remove = ingenic_slcdc_remove,
};

static int ingenic_slcdc_init(void)
{
	if (drm_firmware_drivers_only())
		return -ENODEV;

	return platform_driver_register(&ingenic_slcdc_driver);
}
module_init(ingenic_slcdc_init);

static void ingenic_slcdc_exit(void)
{
	platform_driver_unregister(&ingenic_slcdc_driver);
}
module_exit(ingenic_slcdc_exit);

MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_DESCRIPTION("DRM driver for Ingenic X1000 series SoCs\n");
MODULE_LICENSE("GPL v2");
