// SPDX-License-Identifier: GPL-2.0
/*
 * NitroUART serial controller Driver
 *
 * Copyright (C) 2019-2020 Antmicro <www.antmicro.com>
 * Copyright (C) 2021 Sylvain Munaut <tnt@246tNt.com>
 */

#include <linux/console.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>
#include <linux/xarray.h>


#define URF_EMPTY		(1 << 31)
#define URF_OVERFLOW	(1 << 30)
#define UTF_EMPTY		(1 << 29)
#define UTF_FULL		(1 << 28)
#define DIV_MASK		((1 << 12)-1)

struct no2uart_regs {
	uint32_t data;
	uint32_t csr;
} __attribute__((packed,aligned(4)));

#define to_no2uart_regs(port)	(volatile struct no2uart_regs __iomem *)(port->membase)

struct no2uart_port {
	struct uart_port port;
	struct timer_list timer;
	u32 id;
};

#define to_no2uart_port(port)	container_of(port, struct no2uart_port, port)

static DEFINE_XARRAY_FLAGS(no2uart_array, XA_FLAGS_ALLOC);

#ifdef CONFIG_SERIAL_NO2UART_CONSOLE
static struct console no2uart_console;
#endif

static struct uart_driver no2uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "no2uart",
	.dev_name = "ttyNO2U",
	.major = 0,
	.minor = 0,
	.nr = CONFIG_SERIAL_NO2UART_MAX_PORTS,
#ifdef CONFIG_SERIAL_NO2UART_CONSOLE
	.cons = &no2uart_console,
#endif
};

static void no2uart_timer(struct timer_list *t)
{
	struct no2uart_port *uart = from_timer(uart, t, timer);
	struct uart_port *port = &uart->port;
	volatile struct no2uart_regs __iomem *regs = to_no2uart_regs(port);
	unsigned int flg = TTY_NORMAL;
	int32_t ch;
	unsigned long status;

	while (1) {
		ch = regs->data;
		if (ch < 0)
			break;

		port->icount.rx++;

		/* no overflow bits in status */
		if (!(uart_handle_sysrq_char(port, ch)))
			uart_insert_char(port, status, 0, ch, flg);

		tty_flip_buffer_push(&port->state->port);
	}

	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
}

static void no2uart_putchar(struct uart_port *port, int ch)
{
	volatile struct no2uart_regs __iomem *regs = to_no2uart_regs(port);

	while (regs->csr & UTF_FULL)
		cpu_relax();

	regs->data = ch;
}

static unsigned int no2uart_tx_empty(struct uart_port *port)
{
	volatile struct no2uart_regs __iomem *regs = to_no2uart_regs(port);

	if (regs->csr & UTF_EMPTY)
		return TIOCSER_TEMT;

	return 0;
}

static void no2uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* modem control register is not present in NitroUART */
}

static unsigned int no2uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void no2uart_stop_tx(struct uart_port *port)
{
}

static void no2uart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned char ch;

	if (unlikely(port->x_char)) {
		no2uart_putchar(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
	} else if (!uart_circ_empty(xmit)) {
		while (xmit->head != xmit->tail) {
			ch = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			no2uart_putchar(port, ch);
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void no2uart_stop_rx(struct uart_port *port)
{
	struct no2uart_port *uart = to_no2uart_port(port);

	/* just delete timer */
	del_timer(&uart->timer);
}

static void no2uart_break_ctl(struct uart_port *port, int break_state)
{
	/* NitroUART doesn't support sending break signal */
}

static int no2uart_startup(struct uart_port *port)
{
	struct no2uart_port *uart = to_no2uart_port(port);

	/* prepare timer for polling */
	timer_setup(&uart->timer, no2uart_timer, 0);
	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));

	return 0;
}

static void no2uart_shutdown(struct uart_port *port)
{
}

static void no2uart_set_termios(struct uart_port *port, struct ktermios *new,
				 struct ktermios *old)
{
	unsigned int baud;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* update baudrate */
	baud = uart_get_baud_rate(port, new, old, 0, 2000000);
	uart_update_timeout(port, new->c_cflag, baud);
	/* FIXME ??? */

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *no2uart_type(struct uart_port *port)
{
	return "nitroart";
}

static void no2uart_release_port(struct uart_port *port)
{
}

static int no2uart_request_port(struct uart_port *port)
{
	return 0;
}

static void no2uart_config_port(struct uart_port *port, int flags)
{
	/*
	 * Driver core for serial ports forces a non-zero value for port type.
	 * Write an arbitrary value here to accommodate the serial core driver,
	 * as ID part of UAPI is redundant.
	 */
	port->type = 1;
}

static int no2uart_verify_port(struct uart_port *port,
				struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != 1)
		return -EINVAL;

	return 0;
}

static const struct uart_ops no2uart_ops = {
	.tx_empty     = no2uart_tx_empty,
	.set_mctrl    = no2uart_set_mctrl,
	.get_mctrl    = no2uart_get_mctrl,
	.stop_tx      = no2uart_stop_tx,
	.start_tx     = no2uart_start_tx,
	.stop_rx      = no2uart_stop_rx,
	.break_ctl    = no2uart_break_ctl,
	.startup      = no2uart_startup,
	.shutdown     = no2uart_shutdown,
	.set_termios  = no2uart_set_termios,
	.type         = no2uart_type,
	.release_port = no2uart_release_port,
	.request_port = no2uart_request_port,
	.config_port  = no2uart_config_port,
	.verify_port  = no2uart_verify_port,
};

static int no2uart_probe(struct platform_device *pdev)
{
	struct no2uart_port *uart;
	struct uart_port *port;
	struct resource *res;
	struct xa_limit limit;
	int dev_id, ret;

	/* look for aliases; auto-enumerate for free index if not found */
	dev_id = of_alias_get_id(pdev->dev.of_node, "serial");
	if (dev_id < 0)
		limit = XA_LIMIT(0, CONFIG_SERIAL_NO2UART_MAX_PORTS);
	else
		limit = XA_LIMIT(dev_id, dev_id);

	uart = devm_kzalloc(&pdev->dev, sizeof(struct no2uart_port), GFP_KERNEL);
	if (!uart)
		return -ENOMEM;

	ret = xa_alloc(&no2uart_array, &dev_id, uart, limit, GFP_KERNEL);
	if (ret)
		return ret;

	uart->id = dev_id;
	port = &uart->port;

	/* get membase */
	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (!port->membase)
		return -ENXIO;
	port->mapbase = res->start;

	/* values not from device tree */
	port->dev = &pdev->dev;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &no2uart_ops;
	port->regshift = 2;
	port->fifosize = 384;		// FIXME specify in device tree ?
	port->iobase = 1;
	port->type = PORT_UNKNOWN;
	port->line = dev_id;
	spin_lock_init(&port->lock);

	return uart_add_one_port(&no2uart_driver, &uart->port);
}

static int no2uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct no2uart_port *uart = to_no2uart_port(port);

	xa_erase(&no2uart_array, uart->id);

	return 0;
}

static const struct of_device_id no2uart_of_match[] = {
	{ .compatible = "no2fpga,no2uart" },
	{}
};
MODULE_DEVICE_TABLE(of, no2uart_of_match);

static struct platform_driver no2uart_platform_driver = {
	.probe = no2uart_probe,
	.remove = no2uart_remove,
	.driver = {
		.name = "no2uart",
		.of_match_table = no2uart_of_match,
	},
};

#ifdef CONFIG_SERIAL_NO2UART_CONSOLE

static void no2uart_console_write(struct console *co, const char *s,
	unsigned int count)
{
	struct no2uart_port *uart;
	struct uart_port *port;
	unsigned long flags;

	uart = (struct no2uart_port *)xa_load(&no2uart_array, co->index);
	port = &uart->port;

	spin_lock_irqsave(&port->lock, flags);
	uart_console_write(port, s, count, no2uart_putchar);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int no2uart_console_setup(struct console *co, char *options)
{
	struct no2uart_port *uart;
	struct uart_port *port;
	int baud = 1000000;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	uart = (struct no2uart_port *)xa_load(&no2uart_array, co->index);
	if (!uart)
		return -ENODEV;

	port = &uart->port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console no2uart_console = {
	.name = "nitrouart",	// console names can't contain numbers !!!
	.write = no2uart_console_write,
	.device = uart_console_device,
	.setup = no2uart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &no2uart_driver,
};

static int __init no2uart_console_init(void)
{
	register_console(&no2uart_console);

	return 0;
}
console_initcall(no2uart_console_init);
#endif /* CONFIG_SERIAL_NO2UART_CONSOLE */

static int __init no2uart_init(void)
{
	int res;

	res = uart_register_driver(&no2uart_driver);
	if (res)
		return res;

	res = platform_driver_register(&no2uart_platform_driver);
	if (res) {
		uart_unregister_driver(&no2uart_driver);
		return res;
	}

	return 0;
}

static void __exit no2uart_exit(void)
{
	platform_driver_unregister(&no2uart_platform_driver);
	uart_unregister_driver(&no2uart_driver);
}

module_init(no2uart_init);
module_exit(no2uart_exit);

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_DESCRIPTION("NitroUART serial driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform: no2uart");
