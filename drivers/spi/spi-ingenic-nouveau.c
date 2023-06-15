// SPDX-License-Identifier: GPL-2.0
/*
 * SPI bus driver for the Ingenic X series SoCs
 *
 * Copyright (c) 2022-2023 ReimuNotMoe <reimu@sudomaker.com>
 *
 * Based on spi-ingenic.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#define REG_SSIDR	0x0
#define REG_SSICR0	0x4
#define REG_SSICR1	0x8
#define REG_SSISR	0xc
#define REG_SSIGR	0x18
#define REG_SSIRCNT	0x1c

#define REG_SSICR0_TENDIAN_MASK		(BIT(19) | BIT(18))
#define REG_SSICR0_TENDIAN_POS		18
#define REG_SSICR0_RENDIAN_MASK		(BIT(17) | BIT(16))
#define REG_SSICR0_RENDIAN_POS		16
#define REG_SSICR0_TENDIAN_LSB		BIT(19)
#define REG_SSICR0_RENDIAN_LSB		BIT(17)
#define REG_SSICR0_SSIE			BIT(15)
#define REG_SSICR0_TIE			BIT(14)
#define REG_SSICR0_RIE			BIT(13)
#define REG_SSICR0_TEIE			BIT(12)
#define REG_SSICR0_REIE			BIT(11)
#define REG_SSICR0_LOOP			BIT(10)
#define REG_SSICR0_RFINC		BIT(8)
#define REG_SSICR0_EACLRUN		BIT(7)
#define REG_SSICR0_FSEL			BIT(6)
#define REG_SSICR0_TFLUSH		BIT(2)
#define REG_SSICR0_RFLUSH		BIT(1)
#define REG_SSICR0_DISREV		BIT(0)

#define REG_SSICR1_FRMHL_MASK		(BIT(31) | BIT(30))
#define REG_SSICR1_FRMHL		BIT(30)
#define REG_SSICR1_LFST			BIT(25)
#define REG_SSICR1_UNFIN		BIT(23)
#define REG_SSICR1_TTRG_POS		16
#define REG_SSICR1_TTRG_MASK		(0xf << REG_SSICR1_TTRG_POS)
#define REG_SSICR1_RTRG_POS		8
#define REG_SSICR1_RTRG_MASK		(0xf << REG_SSICR1_RTRG_POS)
#define REG_SSICR1_FLEN_POS		3
#define REG_SSICR1_FLEN_MASK		(0x1f << REG_SSICR1_FLEN_POS)
#define REG_SSICR1_PHA			BIT(1)
#define REG_SSICR1_POL			BIT(0)

#define REG_SSISR_TFIFONUM_POS		16
#define REG_SSISR_TFIFONUM_MASK		(0xff << REG_SSISR_TFIFONUM_POS)
#define REG_SSISR_RFIFONUM_POS		8
#define REG_SSISR_RFIFONUM_MASK		(0xff << REG_SSISR_RFIFONUM_POS)
#define REG_SSISR_END			BIT(7)
#define REG_SSISR_BUSY			BIT(6)
#define REG_SSISR_TFF			BIT(5)
#define REG_SSISR_RFE			BIT(4)
#define REG_SSISR_TFHE			BIT(3)
#define REG_SSISR_RFHF			BIT(2)
#define REG_SSISR_UNDR			BIT(1)
#define REG_SSISR_OVER			BIT(0)

#define SPI_INGENIC_FIFO_SIZE		128u
#define SPI_INGENIC_FIFO_SIZE_HALF	(SPI_INGENIC_FIFO_SIZE / 2)

struct jz_soc_info {
	u32 bits_per_word_mask;
	struct reg_field flen_field;
	bool has_trendian;

	unsigned int max_speed_hz;
	unsigned int max_native_cs;
};

// The regmap API has very high overhead, and like LWN said, it's designed
// for SLOW things. In order to be FAST, we need this cache.
struct ingenic_spi_hw_params_cache {
	unsigned int speed_hz, bits;
	u32 cr0_msg, cr1_msg;
};

// Mainly used by PIO logic. Also used for DMA setup.
struct ingenic_spi_cur_xfer {
	const void *txbuf;
	void *rxbuf;
	unsigned int element_size, len, last_xfer_len;
	unsigned int txpos, rxpos;
	bool wait_with_udelay;
	unsigned long udelay;
};

struct ingenic_spi {
	struct device *dev;
	const struct jz_soc_info *soc_info;
	struct clk *clk;
	struct resource *mem_res;

	struct regmap *map;
	struct regmap_field *flen_field;

	int irq;
	struct completion compl_fifo_empty;
	struct completion compl_xfer_done;

	struct ingenic_spi_cur_xfer cur_xfer;
	struct ingenic_spi_hw_params_cache hw_params_cache;
};

static void spi_ingenic_set_cs(struct spi_device *spi, bool disable)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(spi->controller);

	if (spi->mode & SPI_CS_HIGH)
		disable = !disable;

	// Note: FIFO empty detection is moved to spi_ingenic_finish_transfer()
	// In order to support GPIO CS correctly.

	if (disable) {
		regmap_clear_bits(priv->map, REG_SSICR1, REG_SSICR1_UNFIN);
		regmap_clear_bits(priv->map, REG_SSISR, REG_SSISR_UNDR | REG_SSISR_OVER);
	} else {
		regmap_set_bits(priv->map, REG_SSICR1, REG_SSICR1_UNFIN);
	}
}

static void spi_ingenic_finish_transfer(struct ingenic_spi *priv,
					 struct spi_device *spi,
					 struct spi_transfer *xfer) {

	// Here's the tricky part:
	// Only the hardware CS is controlled by set_cs(). For the GPIO CS,
	// they're changed by gpiod_*() right after transfer_one() returns.

	// Since the UNFIN only controls the hardware CS, we don't even need
	// to bother to clear it here.

	// So now here becomes a central point to ensure the FIFOs are flushed.
	// It's no longer in set_cs().

	// It's less efficient, but what else can we do?

	// TODO: Do we need to wait 1 extra FIFO time to eliminate the possible
	// delay from FIFO to the shift register connected to hardware pins?
	// If so, it's only needed when using GPIO CS.

	if (priv->cur_xfer.wait_with_udelay) {
		udelay(priv->cur_xfer.udelay);
		priv->cur_xfer.wait_with_udelay = false;
	} else {
		regmap_set_bits(priv->map, REG_SSICR0, REG_SSICR0_TEIE);

		if (wait_for_completion_interruptible(&priv->compl_fifo_empty) < 0) {
			dev_err(priv->dev, "SPI operation cancelled");
			regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_TEIE);
		}
	}

	priv->cur_xfer.txbuf = NULL;
	priv->cur_xfer.rxbuf = NULL;

}

static void spi_ingenic_dma_finished_tx(void *controller)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(controller);

	if (priv->cur_xfer.txbuf) {
		if (!priv->cur_xfer.rxbuf) {
			complete(&priv->compl_xfer_done);
		}
	}
}

static void spi_ingenic_dma_finished_rx(void *controller)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(controller);

	if (priv->cur_xfer.rxbuf) {
		complete(&priv->compl_xfer_done);
	}
}

static struct dma_async_tx_descriptor *
spi_ingenic_prepare_dma(struct spi_controller *ctlr, struct dma_chan *chan,
			struct sg_table *sg, enum dma_transfer_direction dir,
			unsigned int bits)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(ctlr);
	struct dma_slave_config cfg = {
		.direction = dir,
		.src_addr = priv->mem_res->start + REG_SSIDR,
		.dst_addr = priv->mem_res->start + REG_SSIDR,
	};
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret;

	if (bits > 16) {
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		cfg.src_maxburst = 4;
	} else if (bits > 8) {
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		cfg.src_maxburst = 2;
	} else {
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		cfg.src_maxburst = 1;
	}

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ERR_PTR(ret);

	desc = dmaengine_prep_slave_sg(chan, sg->sgl, sg->nents, dir,
				       DMA_PREP_INTERRUPT);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	if (dir == DMA_DEV_TO_MEM) {
		desc->callback = spi_ingenic_dma_finished_rx;
	} else {
		desc->callback = spi_ingenic_dma_finished_tx;
	}

	desc->callback_param = ctlr;

	cookie = dmaengine_submit(desc);

	ret = dma_submit_error(cookie);
	if (ret) {
		dmaengine_desc_free(desc);
		return ERR_PTR(ret);
	}

	return desc;
}

static inline void spi_ingenic_determine_wait(struct ingenic_spi *priv,
			      struct spi_transfer *xfer, unsigned int bits,
			      unsigned int target) {

	priv->cur_xfer.udelay = 1000000 * bits * target / xfer->speed_hz;

	if (priv->cur_xfer.udelay <= 50)
		priv->cur_xfer.wait_with_udelay = true;
}

static int spi_ingenic_dma_transfer(struct spi_controller *ctlr,
			      struct spi_transfer *xfer, unsigned int bits)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(ctlr);
	struct dma_async_tx_descriptor *rx_desc = NULL, *tx_desc = NULL;
	u32 val, val2;

	if (xfer->rx_buf) {
		rx_desc = spi_ingenic_prepare_dma(ctlr, ctlr->dma_rx,
						  &xfer->rx_sg, DMA_DEV_TO_MEM, bits);
		if (IS_ERR(rx_desc)) {
			return PTR_ERR(rx_desc);
		}

		val = (0x0 << REG_SSICR1_TTRG_POS) | (0x0 << REG_SSICR1_RTRG_POS);
	} else {
		val = (0xf << REG_SSICR1_TTRG_POS) | (0x0 << REG_SSICR1_RTRG_POS);
	}

	if (xfer->tx_buf) {
		tx_desc = spi_ingenic_prepare_dma(ctlr, ctlr->dma_tx,
						  &xfer->tx_sg, DMA_MEM_TO_DEV, bits);
		if (IS_ERR(tx_desc)) {
			if (rx_desc) {
				dmaengine_terminate_async(ctlr->dma_rx);
				dmaengine_desc_free(rx_desc);
			}
			return PTR_ERR(tx_desc);
		}
	}

	val2 = REG_SSICR1_TTRG_MASK | REG_SSICR1_RTRG_MASK;
	regmap_update_bits(priv->map, REG_SSICR1, val2, val);

	if (rx_desc)
		dma_async_issue_pending(ctlr->dma_rx);

	if (tx_desc)
		dma_async_issue_pending(ctlr->dma_tx);

	if (wait_for_completion_timeout(&priv->compl_xfer_done, 5000) == 0) {
		dev_err(priv->dev, "DMA operation timed out, DMA driver is buggy or HW errata");
		return -ETIMEDOUT;
	}

	regmap_update_bits(priv->map, REG_SSICR1, val2, 0);

	spi_ingenic_determine_wait(priv, xfer, bits, SPI_INGENIC_FIFO_SIZE);

	return 0;
}

static int spi_ingenic_pio_transfer(struct ingenic_spi *priv,
			      struct spi_transfer *xfer, unsigned int bits)
{
	unsigned int element_size, count, tx_prefill_len;
	unsigned int tx_remaining_len, rx_remaining_len;
	unsigned int i, val, val2;

	element_size = bits / 8;
	count = xfer->len / element_size;
	tx_prefill_len = xfer->tx_buf ? min(count, SPI_INGENIC_FIFO_SIZE) : 0;
	tx_remaining_len = xfer->tx_buf ? count - tx_prefill_len : 0;
	rx_remaining_len = xfer->rx_buf ? count : 0;

	switch (bits) {
		case 8: {
			const u8 *buf = xfer->tx_buf;
			for (i = 0; i < tx_prefill_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		case 16: {
			const u16 *buf = xfer->tx_buf;
			for (i = 0; i < tx_prefill_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		case 32: {
			const u32 *buf = xfer->tx_buf;
			for (i = 0; i < tx_prefill_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		default:
			return -EINVAL;
	}

	// We're finished here if it's TX only and the data fits
	// in the entire FIFO
	if ((tx_remaining_len == 0) && (rx_remaining_len == 0)) {
		spi_ingenic_determine_wait(priv, xfer, bits, tx_prefill_len);
		return 0;
	}

	// The journey continues...
	priv->cur_xfer.element_size = element_size;
	priv->cur_xfer.len = count;
	priv->cur_xfer.txpos = count - tx_remaining_len;
	priv->cur_xfer.rxpos = count - rx_remaining_len;

	// Trigger at FIFO half empty
	val = (0x8 << REG_SSICR1_TTRG_POS) | (0x0 << REG_SSICR1_RTRG_POS);
	val2 = REG_SSICR1_TTRG_MASK | REG_SSICR1_RTRG_MASK;
	regmap_update_bits(priv->map, REG_SSICR1, val2, val);


	// Enable interrupts respectively
	val = 0;

	if (tx_remaining_len)
		val |= REG_SSICR0_TIE;

	if (rx_remaining_len)
		val |= REG_SSICR0_RIE;

	// Letsgooooooo
	regmap_set_bits(priv->map, REG_SSICR0, val);

	wait_for_completion(&priv->compl_xfer_done);

	// if (wait_for_completion_interruptible(&priv->compl_xfer_done) < 0) {
	// 	regmap_clear_bits(priv->map, REG_SSICR0, val);
	// 	dev_err(priv->dev, "PIO operation cancelled");
	// }

	// At this point, TIE & RIE should be disabled
	// Revert FIFO settings to 0/0 ensure correct detection of FIFO empty
	regmap_update_bits(priv->map, REG_SSICR1, val2, 0);

	spi_ingenic_determine_wait(priv, xfer, bits, priv->cur_xfer.last_xfer_len);

	return 0;
}


static void spi_ingenic_pio_do_tx(struct ingenic_spi *priv, unsigned int fifo_avail_len) {
	unsigned int i, val;

	unsigned int element_size = priv->cur_xfer.element_size;
	unsigned int pos = priv->cur_xfer.txpos;
	unsigned int remaining = priv->cur_xfer.len - priv->cur_xfer.txpos;
	unsigned int transfer_len = min(fifo_avail_len, remaining);

	switch (element_size) {
		case 1: {
			const u8 *buf = priv->cur_xfer.txbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		case 2: {
			const u16 *buf = priv->cur_xfer.txbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		case 4: {
			const u32 *buf = priv->cur_xfer.txbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				val = buf[i];
				regmap_write(priv->map, REG_SSIDR, val);
			}
			break;
		}
		default:
			BUG();
			break;
	}

	priv->cur_xfer.txpos += transfer_len;
	priv->cur_xfer.last_xfer_len = transfer_len;
}

static void spi_ingenic_pio_do_rx(struct ingenic_spi *priv, unsigned int fifo_avail_len) {
	unsigned int i, val;

	unsigned int element_size = priv->cur_xfer.element_size;
	unsigned int pos = priv->cur_xfer.rxpos;
	unsigned int remaining = priv->cur_xfer.len - priv->cur_xfer.rxpos;
	unsigned int transfer_len = min(fifo_avail_len, remaining);

	switch (element_size) {
		case 1: {
			u8 *buf = priv->cur_xfer.rxbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				regmap_read(priv->map, REG_SSIDR, &val);
				buf[i] = val;
			}
			break;
		}
		case 2: {
			u16 *buf = priv->cur_xfer.rxbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				regmap_read(priv->map, REG_SSIDR, &val);
				buf[i] = val;
			}
			break;
		}
		case 4: {
			u32 *buf = priv->cur_xfer.rxbuf;
			buf += pos;

			for (i = 0; i < transfer_len; i++) {
				regmap_read(priv->map, REG_SSIDR, &val);
				buf[i] = val;
			}
			break;
		}
		default:
			BUG();
			break;
	}

	priv->cur_xfer.rxpos += transfer_len;
}

static irqreturn_t ingenic_spi_irq_handler(int irq, void *data)
{
	struct ingenic_spi *priv = data;
	u32 ssisr, ssicr0, fifo_avail_len;

	int stuff_done = 0;

	// Get interrupt flags
	regmap_read(priv->map, REG_SSISR, &ssisr);
	regmap_read(priv->map, REG_SSICR0, &ssicr0);

	// FIFO empty
	if (ssisr & REG_SSISR_UNDR) {
		if (ssicr0 & REG_SSICR0_TEIE) {
			regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_TEIE);
			regmap_clear_bits(priv->map, REG_SSISR, REG_SSISR_UNDR);
			complete(&priv->compl_fifo_empty);
			stuff_done++;
		} else {
			regmap_clear_bits(priv->map, REG_SSISR, REG_SSISR_UNDR);
		}
	}

	// PIO RX
	if ((ssicr0 & REG_SSICR0_RIE) && (ssisr & REG_SSISR_RFHF)) {
		while (1) {
			fifo_avail_len = (ssisr & REG_SSISR_RFIFONUM_MASK) >> REG_SSISR_RFIFONUM_POS;
			if ((fifo_avail_len == 0) || (priv->cur_xfer.rxpos >= priv->cur_xfer.len))
				break;
			spi_ingenic_pio_do_rx(priv, fifo_avail_len);
			regmap_read(priv->map, REG_SSISR, &ssisr);
		}

		if (priv->cur_xfer.rxpos >= priv->cur_xfer.len) {
			regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_RIE);

			if (priv->cur_xfer.txpos >= priv->cur_xfer.len) {
				complete(&priv->compl_xfer_done);
			}

			if (priv->cur_xfer.rxpos > priv->cur_xfer.len) {
				printk("wtf: %u %u\n", priv->cur_xfer.txpos, priv->cur_xfer.len);
			}
		}

		stuff_done++;

	}

	// PIO TX
	if ((ssicr0 & REG_SSICR0_TIE) && (ssisr & REG_SSISR_TFHE)) {
		fifo_avail_len = SPI_INGENIC_FIFO_SIZE - ((ssisr & REG_SSISR_TFIFONUM_MASK) >> REG_SSISR_TFIFONUM_POS);

		spi_ingenic_pio_do_tx(priv, fifo_avail_len);

		if (priv->cur_xfer.txpos >= priv->cur_xfer.len) {
			regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_TIE);

			if (priv->cur_xfer.rxpos >= priv->cur_xfer.len) {
				complete(&priv->compl_xfer_done);
			}

			if (priv->cur_xfer.txpos > priv->cur_xfer.len) {
				printk("wTf: %u %u\n", priv->cur_xfer.txpos, priv->cur_xfer.len);
			}
		}

		stuff_done++;
	}

	if (stuff_done == 0) {
		printk("I did nothing!\n");
	}

	return IRQ_HANDLED;
}

static unsigned int spi_ingenic_prepare_transfer(struct ingenic_spi *priv,
					 struct spi_device *spi,
					 struct spi_transfer *xfer)
{
	unsigned int speed_hz = xfer->speed_hz ?: spi->max_speed_hz;
	unsigned int bits = xfer->bits_per_word ?: spi->bits_per_word;
	unsigned int bits_rounded_up;

	// bits: bits with vaild data, should be put to FLEN
	// bits_rounded_up: bits rounded to nearest byte

	if (bits > 16)
		bits_rounded_up = 32;
	else if (bits > 8)
		bits_rounded_up = 16;
	else
		bits_rounded_up = 8;

	if (priv->hw_params_cache.bits != bits) {
		regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_SSIE);
		regmap_field_write(priv->flen_field, bits - 2);
		regmap_set_bits(priv->map, REG_SSICR0, REG_SSICR0_SSIE);
		priv->hw_params_cache.bits = bits;
	}

	if (priv->hw_params_cache.speed_hz != speed_hz) {
		unsigned long clk_hz = clk_get_rate(priv->clk);
		u32 cdiv;

		cdiv = clk_hz / (speed_hz * 2);
		cdiv = clamp(cdiv, 1u, 0x100u) - 1;

		regmap_write(priv->map, REG_SSIGR, cdiv);
		priv->hw_params_cache.speed_hz = speed_hz;
	}

	priv->cur_xfer.txbuf = xfer->tx_buf;
	priv->cur_xfer.rxbuf = xfer->rx_buf;

	if (xfer->rx_buf) {
		regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_DISREV);
	} else {
		regmap_set_bits(priv->map, REG_SSICR0, REG_SSICR0_DISREV);
	}

	// FYI: In order to the job, the hardware CS is low now
	regmap_set_bits(priv->map, REG_SSICR0, REG_SSICR0_RFLUSH | REG_SSICR0_TFLUSH);
	regmap_set_bits(priv->map, REG_SSICR1, REG_SSICR1_UNFIN);

	return bits_rounded_up;
}

static int spi_ingenic_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *xfer)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(ctlr);
	unsigned int bits = spi_ingenic_prepare_transfer(priv, spi, xfer);
	int ret = -1;

	if (ctlr->cur_msg_mapped && ctlr->can_dma && ctlr->can_dma(ctlr, spi, xfer)) {
		ret = spi_ingenic_dma_transfer(ctlr, xfer, bits);
		if (ret != 0) {
			dev_warn(&ctlr->dev, "SPI DMA rejected: bits: %u, len: %zu, txbuf=%p, rxbuf=%p, txdma=%08x, rxdma=%08x\n",
				bits, xfer->len, xfer->tx_buf, xfer->rx_buf, xfer->tx_dma, xfer->rx_dma);
		}
	}

	if (ret == -1) {
		ret = spi_ingenic_pio_transfer(priv, xfer, bits);
	}

	spi_ingenic_finish_transfer(priv, spi, xfer);

	return ret;
}

static int spi_ingenic_prepare_message(struct spi_controller *ctlr,
				       struct spi_message *message)
{
	struct ingenic_spi *priv = spi_controller_get_devdata(ctlr);
	struct spi_device *spi = message->spi;

	unsigned int cs = REG_SSICR1_FRMHL << spi->chip_select;
	unsigned int ssicr0_mask = REG_SSICR0_LOOP | REG_SSICR0_FSEL;
	unsigned int ssicr1_mask = REG_SSICR1_PHA | REG_SSICR1_POL | cs;
	unsigned int ssicr0 = 0, ssicr1 = 0;

	if (priv->soc_info->has_trendian) {
		ssicr0_mask |= REG_SSICR0_TENDIAN_MASK | REG_SSICR0_RENDIAN_MASK;

		if (spi->mode & SPI_LSB_FIRST)
			ssicr0 |= REG_SSICR0_RENDIAN_LSB | REG_SSICR0_TENDIAN_LSB;
	} else {
		ssicr1_mask |= REG_SSICR1_LFST;

		if (spi->mode & SPI_LSB_FIRST)
			ssicr1 |= REG_SSICR1_LFST;
	}

	if (spi->mode & SPI_LOOP)
		ssicr0 |= REG_SSICR0_LOOP;

	if (spi->chip_select)
		ssicr0 |= REG_SSICR0_FSEL;

	if (spi->mode & SPI_CPHA)
		ssicr1 |= REG_SSICR1_PHA;
	if (spi->mode & SPI_CPOL)
		ssicr1 |= REG_SSICR1_POL;
	if (spi->mode & SPI_CS_HIGH)
		ssicr1 |= cs;

	if (priv->hw_params_cache.cr0_msg != ssicr0) {
		regmap_update_bits(priv->map, REG_SSICR0, ssicr0_mask, ssicr0);
		priv->hw_params_cache.cr0_msg = ssicr0;
	}

	if (priv->hw_params_cache.cr1_msg != ssicr1) {
		regmap_update_bits(priv->map, REG_SSICR1, ssicr1_mask, ssicr1);
		priv->hw_params_cache.cr1_msg = ssicr1;
		priv->hw_params_cache.bits = 0;
	}

	return 0;
}

static bool spi_ingenic_can_dma(struct spi_controller *ctlr,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct dma_slave_caps caps;
	int ret;

	ret = dma_get_slave_caps(ctlr->dma_tx, &caps);
	if (ret) {
		dev_err(&spi->dev, "Unable to get slave caps: %d\n", ret);
		return false;
	}

	if (xfer->len > 128) {
		return !caps.max_sg_burst ||
			xfer->len <= caps.max_sg_burst * SPI_INGENIC_FIFO_SIZE;
	} else {
		return false;
	}
}

static int spi_ingenic_request_dma(struct spi_controller *ctlr,
				   struct device *dev)
{
	ctlr->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!ctlr->dma_tx)
		return -ENODEV;

	ctlr->dma_rx = dma_request_slave_channel(dev, "rx");

	if (!ctlr->dma_rx)
		return -ENODEV;

	ctlr->can_dma = spi_ingenic_can_dma;

	return 0;
}

static void spi_ingenic_release_dma(void *data)
{
	struct spi_controller *ctlr = data;

	if (ctlr->dma_tx)
		dma_release_channel(ctlr->dma_tx);
	if (ctlr->dma_rx)
		dma_release_channel(ctlr->dma_rx);
}

static const struct regmap_config spi_ingenic_regmap_config = {
	.disable_locking = true,
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = REG_SSIGR,
};

static int spi_ingenic_probe(struct platform_device *pdev)
{
	const struct jz_soc_info *pdata;
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ingenic_spi *priv;
	void __iomem *base;
	int num_cs, ret;

	pdata = of_device_get_match_data(dev);
	if (!pdata) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

	ctlr = devm_spi_alloc_master(dev, sizeof(*priv));
	if (!ctlr) {
		dev_err(dev, "Unable to allocate SPI controller.\n");
		return -ENOMEM;
	}

	priv = spi_controller_get_devdata(ctlr);
	priv->dev = dev;
	priv->soc_info = pdata;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "Unable to get clock.\n");
	}

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &priv->mem_res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->map = devm_regmap_init_mmio(dev, base, &spi_ingenic_regmap_config);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	priv->flen_field = devm_regmap_field_alloc(dev, priv->map,
						   pdata->flen_field);
	if (IS_ERR(priv->flen_field))
		return PTR_ERR(priv->flen_field);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	if (device_property_read_u32(dev, "num-cs", &num_cs))
		num_cs = pdata->max_native_cs;

	platform_set_drvdata(pdev, ctlr);

	ret = devm_request_irq(dev, priv->irq, ingenic_spi_irq_handler, 0, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "Failed to request irq %d, ret = %d\n", priv->irq, ret);
		return ret;
	}

	init_completion(&priv->compl_fifo_empty);
	init_completion(&priv->compl_xfer_done);

	ctlr->prepare_message = spi_ingenic_prepare_message;
	ctlr->set_cs = spi_ingenic_set_cs;
	ctlr->transfer_one = spi_ingenic_transfer_one;
	ctlr->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST | SPI_LOOP | SPI_CS_HIGH;
	ctlr->flags = SPI_CONTROLLER_MUST_TX;
	ctlr->bits_per_word_mask = pdata->bits_per_word_mask;
	ctlr->min_speed_hz = 32000; // Ensure one FIFO entry takes 1ms or less to finish for now
	ctlr->max_speed_hz = pdata->max_speed_hz;
	ctlr->use_gpio_descriptors = true;
	ctlr->max_native_cs = pdata->max_native_cs;
	ctlr->num_chipselect = num_cs;
	ctlr->dev.of_node = pdev->dev.of_node;

	if (spi_ingenic_request_dma(ctlr, dev))
		dev_warn(dev, "DMA not available.\n");

	ret = devm_add_action_or_reset(dev, spi_ingenic_release_dma, ctlr);
	if (ret) {
		dev_err(dev, "Unable to add action.\n");
		return ret;
	}

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		dev_err(dev, "Unable to register SPI controller.\n");


	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "Unable to enable clock.\n");

	regmap_write(priv->map, REG_SSICR0, REG_SSICR0_EACLRUN);
	regmap_write(priv->map, REG_SSICR1, 0);
	regmap_write(priv->map, REG_SSISR, 0);
	regmap_set_bits(priv->map, REG_SSICR0, REG_SSICR0_SSIE);

	return ret;
}

static int spi_ingenic_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct ingenic_spi *priv;

	ctlr = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(ctlr);

	regmap_clear_bits(priv->map, REG_SSICR0, REG_SSICR0_SSIE);
	clk_disable_unprepare(priv->clk);

	return 0;
}


static const struct jz_soc_info x1000_soc_info = {
	.bits_per_word_mask = SPI_BPW_RANGE_MASK(2, 32),
	.flen_field = REG_FIELD(REG_SSICR1, 3, 7),
	.has_trendian = true,

	.max_speed_hz = 150000000,
	.max_native_cs = 2,
};

static const struct jz_soc_info x2000_soc_info = {
	.bits_per_word_mask = SPI_BPW_RANGE_MASK(2, 32),
	.flen_field = REG_FIELD(REG_SSICR1, 3, 7),
	.has_trendian = true,

	.max_speed_hz = 150000000,
	.max_native_cs = 1,
};

static const struct of_device_id spi_ingenic_of_match[] = {
	{ .compatible = "ingenic,x1000-spi", .data = &x1000_soc_info },
	{ .compatible = "ingenic,x2000-spi", .data = &x2000_soc_info },
	{}
};
MODULE_DEVICE_TABLE(of, spi_ingenic_of_match);

static struct platform_driver spi_ingenic_driver = {
	.driver = {
		.name = "spi-ingenic-nouveau",
		.of_match_table = spi_ingenic_of_match,
	},
	.probe = spi_ingenic_probe,
	.remove = spi_ingenic_remove,
};

module_platform_driver(spi_ingenic_driver);
MODULE_DESCRIPTION("SPI bus driver for the Ingenic X series SoCs");
MODULE_AUTHOR("ReimuNotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL");
