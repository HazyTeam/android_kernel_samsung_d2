/*
 * Synopsys DesignWare 8250 driver.
 *
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include "8250.h"

/* Offsets for the DesignWare specific registers */
#define DW_UART_USR	0x1f /* UART Status Register */
#define DW_UART_CPR	0xf4 /* Component Parameter Register */
#define DW_UART_UCV	0xf8 /* UART Component Version */

/* Component Parameter Register bits */
#define DW_UART_CPR_ABP_DATA_WIDTH	(3 << 0)
#define DW_UART_CPR_AFCE_MODE		(1 << 4)
#define DW_UART_CPR_THRE_MODE		(1 << 5)
#define DW_UART_CPR_SIR_MODE		(1 << 6)
#define DW_UART_CPR_SIR_LP_MODE		(1 << 7)
#define DW_UART_CPR_ADDITIONAL_FEATURES	(1 << 8)
#define DW_UART_CPR_FIFO_ACCESS		(1 << 9)
#define DW_UART_CPR_FIFO_STAT		(1 << 10)
#define DW_UART_CPR_SHADOW		(1 << 11)
#define DW_UART_CPR_ENCODED_PARMS	(1 << 12)
#define DW_UART_CPR_DMA_EXTRA		(1 << 13)
#define DW_UART_CPR_FIFO_MODE		(0xff << 16)
/* Helper for fifo size calculation */
#define DW_UART_CPR_FIFO_SIZE(a)	(((a >> 16) & 0xff) * 16)


struct dw8250_data {
	int		last_lcr;
	int		line;
	struct clk	*clk;
};

static void dw8250_serial_out(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	if (offset == UART_LCR)
		d->last_lcr = value;

	offset <<= p->regshift;
	writeb(value, p->membase + offset);
}

static unsigned int dw8250_serial_in(struct uart_port *p, int offset)
{
	offset <<= p->regshift;

	return readb(p->membase + offset);
}

static void dw8250_serial_out32(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	if (offset == UART_LCR)
		d->last_lcr = value;

	offset <<= p->regshift;
	writel(value, p->membase + offset);
}

static unsigned int dw8250_serial_in32(struct uart_port *p, int offset)
{
	offset <<= p->regshift;

	return readl(p->membase + offset);
}

static int dw8250_handle_irq(struct uart_port *p)
{
	struct dw8250_data *d = p->private_data;
	unsigned int iir = p->serial_in(p, UART_IIR);

	if (serial8250_handle_irq(p, iir)) {
		return 1;
	} else if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
		/* Clear the USR and write the LCR again. */
		(void)p->serial_in(p, DW_UART_USR);
		p->serial_out(p, UART_LCR, d->last_lcr);

		return 1;
	}

	return 0;
}

static int __devinit dw8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct device_node *np = pdev->dev.of_node;
	u32 val;
	struct dw8250_data *data;

	if (!regs || !irq) {
		dev_err(&pdev->dev, "no registers/irq defined\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	uart.port.private_data = data;

	spin_lock_init(&uart.port.lock);
	uart.port.mapbase = regs->start;
	uart.port.irq = irq->start;
	uart.port.handle_irq = dw8250_handle_irq;
	uart.port.type = PORT_8250;
	uart.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF | UPF_IOREMAP |
		UPF_FIXED_PORT | UPF_FIXED_TYPE;
	uart.port.dev = &pdev->dev;

	uart.port.iotype = UPIO_MEM;
	uart.port.serial_in = dw8250_serial_in;
	uart.port.serial_out = dw8250_serial_out;
	if (!of_property_read_u32(np, "reg-io-width", &val)) {
		switch (val) {
		case 1:
			break;
		case 4:
			uart.port.iotype = UPIO_MEM32;
			uart.port.serial_in = dw8250_serial_in32;
			uart.port.serial_out = dw8250_serial_out32;
			break;
		default:
			dev_err(&pdev->dev, "unsupported reg-io-width (%u)\n",
				val);
			return -EINVAL;
		}
	}

	if (!of_property_read_u32(np, "reg-shift", &val))
		uart.port.regshift = val;

	if (of_property_read_u32(np, "clock-frequency", &val)) {
		dev_err(&pdev->dev, "no clock-frequency property set\n");
		return -EINVAL;
	}
	uart.port.uartclk = val;

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0)
		return data->line;

	platform_set_drvdata(pdev, data);

	return 0;
}

static int __devexit dw8250_remove(struct platform_device *pdev)
{
	struct dw8250_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);

	return 0;
}

static const struct of_device_id dw8250_of_match[] = {
	{ .compatible = "snps,dw-apb-uart" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw8250_of_match);

static const struct acpi_device_id dw8250_acpi_match[] = {
	{ "INT33C4", 0 },
	{ "INT33C5", 0 },
	{ "80860F0A", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, dw8250_acpi_match);

static struct platform_driver dw8250_platform_driver = {
	.driver = {
		.name		= "dw-apb-uart",
		.owner		= THIS_MODULE,
		.of_match_table	= dw8250_of_match,
		.acpi_match_table = ACPI_PTR(dw8250_acpi_match),
	},
	.probe			= dw8250_probe,
	.remove			= __devexit_p(dw8250_remove),
};

module_platform_driver(dw8250_platform_driver);

MODULE_AUTHOR("Jamie Iles");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Synopsys DesignWare 8250 serial port driver");
