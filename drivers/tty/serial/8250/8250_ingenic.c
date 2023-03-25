// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2010 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2015 Imagination Technologies
 * Copyright (C) 2016 Ingenic Semiconductor Co.,Ltd
 *   Author: bliu
 * Copyright (C) 2023 SudoMaker, Ltd.
 *   Author: Reimu NotMoe <reimu@sudomaker.com>
 *
 * Ingenic SoC UART support
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/libfdt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>

#include "8250.h"

#define REG_UDLLR		0x00
#define REG_UDLHR		0x04
#define REG_UMR			0x24
#define REG_UACR		0x28

/** ingenic_uart_config: SOC specific config data. */
struct ingenic_uart_config {
	int tx_loadsz;
	int fifosize;
};

struct ingenic_uart_data {
	struct clk	*clk_module;
	struct clk	*clk_baud;
	int		line;
};

static const struct of_device_id of_match[];

#define UART_FCR_UME	BIT(4)

#define UART_MCR_MDCE	BIT(7)
#define UART_MCR_FCM	BIT(6)

static struct earlycon_device *early_device;

static uint8_t early_in(struct uart_port *port, int offset)
{
	return readl(port->membase + (offset << 2));
}

static void early_out(struct uart_port *port, int offset, uint8_t value)
{
	writel(value, port->membase + (offset << 2));
}

static void ingenic_early_console_putc(struct uart_port *port, unsigned char c)
{
	u16 lsr;

	do {
		lsr = early_in(port, UART_LSR);
	} while ((lsr & UART_LSR_TEMT) == 0);

	early_out(port, UART_TX, c);
}

static void ingenic_early_console_write(struct console *console,
					      const char *s, unsigned int count)
{
	uart_console_write(&early_device->port, s, count,
			   ingenic_early_console_putc);
}

static void __init ingenic_early_console_setup_clock(struct earlycon_device *dev)
{
	void *fdt = initial_boot_params;
	const __be32 *prop;
	int offset;

	offset = fdt_path_offset(fdt, "/ext");
	if (offset < 0)
		return;

	prop = fdt_getprop(fdt, offset, "clock-frequency", NULL);
	if (!prop)
		return;

	dev->port.uartclk = be32_to_cpup(prop);
}

static int __init ingenic_early_console_setup(struct earlycon_device *dev,
					      const char *opt)
{
	struct uart_port *port = &dev->port;
	unsigned int divisor;
	int baud = 115200;

	if (!dev->port.membase)
		return -ENODEV;

	if (opt) {
		unsigned int parity, bits, flow; /* unused for now */

		uart_parse_options(opt, &baud, &parity, &bits, &flow);
	}

	ingenic_early_console_setup_clock(dev);

	if (dev->baud)
		baud = dev->baud;
	divisor = DIV_ROUND_CLOSEST(port->uartclk, 16 * baud);

	early_out(port, UART_IER, 0);
	early_out(port, UART_LCR, UART_LCR_DLAB | UART_LCR_WLEN8);
	early_out(port, UART_DLL, 0);
	early_out(port, UART_DLM, 0);
	early_out(port, UART_LCR, UART_LCR_WLEN8);
	early_out(port, UART_FCR, UART_FCR_UME | UART_FCR_CLEAR_XMIT |
			UART_FCR_CLEAR_RCVR | UART_FCR_ENABLE_FIFO);
	early_out(port, UART_MCR, UART_MCR_RTS | UART_MCR_DTR);

	early_out(port, UART_LCR, UART_LCR_DLAB | UART_LCR_WLEN8);
	early_out(port, UART_DLL, divisor & 0xff);
	early_out(port, UART_DLM, (divisor >> 8) & 0xff);
	early_out(port, UART_LCR, UART_LCR_WLEN8);

	early_device = dev;
	dev->con->write = ingenic_early_console_write;

	return 0;
}

OF_EARLYCON_DECLARE(jz4740_uart, "ingenic,jz4740-uart",
		    ingenic_early_console_setup);

OF_EARLYCON_DECLARE(jz4770_uart, "ingenic,jz4770-uart",
		    ingenic_early_console_setup);

OF_EARLYCON_DECLARE(jz4775_uart, "ingenic,jz4775-uart",
		    ingenic_early_console_setup);

OF_EARLYCON_DECLARE(jz4780_uart, "ingenic,jz4780-uart",
		    ingenic_early_console_setup);

OF_EARLYCON_DECLARE(x1000_uart, "ingenic,x1000-uart",
		    ingenic_early_console_setup);

static void ingenic_uart_serial_out(struct uart_port *p, int offset, int value)
{
	int ier;

	switch (offset) {
	case UART_FCR:
		/* UART module enable */
		value |= UART_FCR_UME;
		break;

	case UART_IER:
		/*
		 * Enable receive timeout interrupt with the receive line
		 * status interrupt.
		 */
		value |= (value & 0x4) << 2;
		break;

	case UART_MCR:
		/*
		 * If we have enabled modem status IRQs we should enable
		 * modem mode.
		 */
		ier = p->serial_in(p, UART_IER);

		if (ier & UART_IER_MSI)
			value |= UART_MCR_MDCE | UART_MCR_FCM;
		else
			value &= ~(UART_MCR_MDCE | UART_MCR_FCM);
		break;

	default:
		break;
	}

	writeb(value, p->membase + (offset << p->regshift));
}

static unsigned int ingenic_uart_serial_in(struct uart_port *p, int offset)
{
	unsigned int value;

	value = readb(p->membase + (offset << p->regshift));

	/* Hide non-16550 compliant bits from higher levels */
	switch (offset) {
	case UART_FCR:
		value &= ~UART_FCR_UME;
		break;

	case UART_MCR:
		value &= ~(UART_MCR_MDCE | UART_MCR_FCM);
		break;

	default:
		break;
	}
	return value;
}

static void calc_div(unsigned long exclk_freq, unsigned long baud_rate,
	      unsigned *out_div, unsigned *out_umr, unsigned *out_uacr)
{
	unsigned long umr_best = 0, div_best = 0, uacr_best = 0, uacr_count_best = 0;
	unsigned long umr = 0, div = 1, sum = 0;
	unsigned long uacr, uacr_count;
	unsigned long tmp0[12], tmp1[12];
	unsigned baud_mul16 = 16 * baud_rate;
	u64 t0, t1, t2, t3;
	s64 err;

	memset(tmp0, 0, sizeof(tmp0));
	memset(tmp1, 0, sizeof(tmp1));

	if ((exclk_freq % baud_mul16) == 0) {
		div_best = exclk_freq / baud_mul16;
		umr_best = 16;
		uacr_best = 0;
	} else {
		while (1) {
			umr = exclk_freq / (baud_rate * div);
			if (umr > 32) {
				div++;
				continue;
			}
			if (umr < 4) {
				break;
			}
			for (unsigned i=0; i<12; i++) {
				tmp0[i] = umr;
				tmp1[i] = 0;
				sum = 0;
				for (unsigned j=0; j<(i+1); j++) {
					sum += tmp0[j];
				}

				t0 = 0x1000000000;
				t1 = (i + 1) * t0;
				t2 = (sum * div) * t0;
				t3 = div * t0;
				do_div(t1, baud_rate);
				do_div(t2, exclk_freq);
				do_div(t3, (2 * exclk_freq));

				err = t1 - t2 - t3;
				if (err > 0) {
					tmp0[i] += 1;
					tmp1[i] = 1;
				}
			}

			uacr = 0;
			uacr_count = 0;

			for (unsigned i=0; i<12; i++) {
				if (tmp1[i] == 1) {
					uacr |= 1 << i;
					uacr_count += 1;
				}
			}

			if (div_best == 0) {
				div_best = div;
				umr_best = umr;
				uacr_best = uacr;
				uacr_count_best = uacr_count;
			};

			// the best value of umr should be near 16,
			// and the value of uacr should better be smaller

			if (((umr - 16) < (umr_best - 16)) ||
			    ((umr - 16) == (umr_best - 16) && uacr_best > uacr)) {
				div_best = div;
				umr_best = umr;
				uacr_best = uacr;
				uacr_count_best = uacr_count;
			}

			div++;
		}
	}

	if (uacr_count_best != 0) {
		uacr_best = 0;
		for (unsigned i=uacr_count_best; i<11; i+=(11/uacr_count_best)) {
			uacr_best |= 1 << i;
		}
	};

	*out_div = div_best;
	*out_umr = umr_best;
	*out_uacr = uacr_best;
}

static void ingenic_uart_do_set_divisor(struct uart_port *port, unsigned int baud,
			       unsigned int quot, unsigned int quot_frac)
{
	unsigned div, umr, uacr;
	unsigned tmp;

	calc_div(port->uartclk, baud, &div, &umr, &uacr);

	tmp = ingenic_uart_serial_in(port, UART_LCR);
	tmp |= UART_LCR_DLAB;
	ingenic_uart_serial_out(port, UART_LCR, tmp);

	writeb((div >> 8) & 0xff, port->membase + REG_UDLHR);
	writeb(div & 0xff, port->membase + REG_UDLLR);

	writel(umr, port->membase + REG_UMR);
	writel(uacr, port->membase + REG_UACR);
}

static int ingenic_uart_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct ingenic_uart_data *data;
	const struct ingenic_uart_config *cdata;
	struct resource *regs;
	int irq, err, line;

	cdata = of_device_get_match_data(&pdev->dev);
	if (!cdata) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "no registers defined\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&uart.port.lock);
	uart.port.type = PORT_16550A;
	uart.port.flags = UPF_SKIP_TEST | UPF_IOREMAP | UPF_FIXED_TYPE;
	uart.port.iotype = UPIO_MEM;
	uart.port.mapbase = regs->start;
	uart.port.regshift = 2;
	uart.port.set_divisor = ingenic_uart_do_set_divisor;
	uart.port.serial_out = ingenic_uart_serial_out;
	uart.port.serial_in = ingenic_uart_serial_in;
	uart.port.irq = irq;
	uart.port.dev = &pdev->dev;
	uart.port.fifosize = cdata->fifosize;
	uart.tx_loadsz = cdata->tx_loadsz;
	uart.capabilities = UART_CAP_FIFO | UART_CAP_RTOIE;

	/* Check for a fixed line number */
	line = of_alias_get_id(pdev->dev.of_node, "serial");
	if (line >= 0)
		uart.port.line = line;

	uart.port.membase = devm_ioremap(&pdev->dev, regs->start,
					 resource_size(regs));
	if (!uart.port.membase)
		return -ENOMEM;

	data->clk_module = devm_clk_get(&pdev->dev, "module");
	if (IS_ERR(data->clk_module))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->clk_module),
				     "unable to get module clock\n");

	data->clk_baud = devm_clk_get(&pdev->dev, "baud");
	if (IS_ERR(data->clk_baud))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->clk_baud),
				     "unable to get baud clock\n");

	err = clk_prepare_enable(data->clk_module);
	if (err) {
		dev_err(&pdev->dev, "could not enable module clock: %d\n", err);
		goto out;
	}

	err = clk_prepare_enable(data->clk_baud);
	if (err) {
		dev_err(&pdev->dev, "could not enable baud clock: %d\n", err);
		goto out_disable_moduleclk;
	}
	uart.port.uartclk = clk_get_rate(data->clk_baud);

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0) {
		err = data->line;
		goto out_disable_baudclk;
	}

	platform_set_drvdata(pdev, data);
	return 0;

out_disable_baudclk:
	clk_disable_unprepare(data->clk_baud);
out_disable_moduleclk:
	clk_disable_unprepare(data->clk_module);
out:
	return err;
}

static int ingenic_uart_remove(struct platform_device *pdev)
{
	struct ingenic_uart_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	clk_disable_unprepare(data->clk_module);
	clk_disable_unprepare(data->clk_baud);
	return 0;
}

static const struct ingenic_uart_config jz4740_uart_config = {
	.tx_loadsz = 8,
	.fifosize = 16,
};

static const struct ingenic_uart_config jz4760_uart_config = {
	.tx_loadsz = 16,
	.fifosize = 32,
};

static const struct ingenic_uart_config jz4780_uart_config = {
	.tx_loadsz = 32,
	.fifosize = 64,
};

static const struct ingenic_uart_config x1000_uart_config = {
	.tx_loadsz = 32,
	.fifosize = 64,
};

static const struct of_device_id of_match[] = {
	{ .compatible = "ingenic,jz4740-uart", .data = &jz4740_uart_config },
	{ .compatible = "ingenic,jz4760-uart", .data = &jz4760_uart_config },
	{ .compatible = "ingenic,jz4770-uart", .data = &jz4760_uart_config },
	{ .compatible = "ingenic,jz4775-uart", .data = &jz4760_uart_config },
	{ .compatible = "ingenic,jz4780-uart", .data = &jz4780_uart_config },
	{ .compatible = "ingenic,x1000-uart", .data = &x1000_uart_config },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver ingenic_uart_platform_driver = {
	.driver = {
		.name		= "ingenic-uart",
		.of_match_table	= of_match,
	},
	.probe			= ingenic_uart_probe,
	.remove			= ingenic_uart_remove,
};

module_platform_driver(ingenic_uart_platform_driver);

MODULE_AUTHOR("Paul Burton");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic SoC UART driver");
