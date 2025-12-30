// SPDX-License-Identifier: GPL-2.0
/*
 * xilinx_uartlite.c - Xilinx UARTLite Driver for PCIe Bridge
 *
 * This driver provides standard TTY interfaces (/dev/ttyUL0-11) for
 * Xilinx UARTLite IP cores accessed via PCIe.
 *
 * Features:
 * - Works with mcu_pcie.c bridge driver
 * - Supports 12 UART ports
 * - Supports both interrupt and polling modes
 * - Compatible with minicom and other terminal applications
 * - Compatible with RHEL 9 and RT-patched kernels
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>

#define DRIVER_NAME      "xilinx_uartlite"
#define UARTLITE_NAME    "ttyUL"
#define UARTLITE_MAJOR   204
#define UARTLITE_MINOR   187
#define UARTLITE_NR      12       /* Support 12 UARTs (ttyUL0-ttyUL11) */

/*
 * UARTLite Register Offsets
 * Note: On PCIe/x86 systems, use little-endian access
 */
#define ULITE_RX         0x00     /* Receive Data Register */
#define ULITE_TX         0x04     /* Transmit Data Register */
#define ULITE_STATUS     0x08     /* Status Register */
#define ULITE_CONTROL    0x0C     /* Control Register */

/*
 * Status Register Bits
 */
#define ULITE_STATUS_RXVALID    BIT(0)   /* RX FIFO has data */
#define ULITE_STATUS_RXFULL     BIT(1)   /* RX FIFO is full */
#define ULITE_STATUS_TXEMPTY    BIT(2)   /* TX FIFO is empty */
#define ULITE_STATUS_TXFULL     BIT(3)   /* TX FIFO is full */
#define ULITE_STATUS_IE         BIT(4)   /* Interrupt enabled */
#define ULITE_STATUS_OVERRUN    BIT(5)   /* RX overrun error */
#define ULITE_STATUS_FRAME      BIT(6)   /* Framing error */
#define ULITE_STATUS_PARITY     BIT(7)   /* Parity error (if enabled) */

/*
 * Control Register Bits
 */
#define ULITE_CONTROL_RST_TX    BIT(0)   /* Reset TX FIFO */
#define ULITE_CONTROL_RST_RX    BIT(1)   /* Reset RX FIFO */
#define ULITE_CONTROL_IE        BIT(4)   /* Enable interrupts */

/* FIFO depth */
#define ULITE_FIFO_SIZE         16

/* Polling interval in ms */
#define POLL_INTERVAL_MS        1

/* Forward declarations */
extern int mcu_pcie_register_irq_handler(struct device *dev, struct notifier_block *nb);
extern void mcu_pcie_unregister_irq_handler(struct device *dev, struct notifier_block *nb);
extern u32 mcu_pcie_get_pending(struct device *dev);

/*
 * Per-port data structure
 */
struct uartlite_port {
    struct uart_port port;
    struct platform_device *pdev;
    void __iomem *regs;
    
    /* IRQ handling */
    int irq_idx;                      /* User IRQ index in bridge */
    struct notifier_block irq_nb;     /* For bridge notifications */
    bool use_irq;                     /* IRQ mode active */
    
    /* Polling fallback */
    struct timer_list poll_timer;
    bool polling_active;
    
    /* TX state */
    bool tx_busy;
    
    /* Statistics */
    unsigned long rx_count;
    unsigned long tx_count;
    unsigned long err_count;
};

static struct uart_driver uartlite_uart_driver;
static struct uartlite_port *uartlite_ports[UARTLITE_NR];
static DEFINE_MUTEX(uartlite_mutex);
static int uartlite_port_count;

/*
 * Register access
 * 
 * UARTLite on PCIe requires:
 * - BYTE access for RX (0x00) and TX (0x04) data registers
 * - WORD access for Status (0x08) and Control (0x0C) registers
 */
static inline u32 ulite_read(struct uartlite_port *up, int offset)
{
    if (offset == ULITE_RX) {
        /* RX data - use byte access */
        return ioread8(up->regs + offset);
    } else {
        /* Status/Control - use word access */
        return ioread32(up->regs + offset);
    }
}

static inline void ulite_write(struct uartlite_port *up, u32 val, int offset)
{
    if (offset == ULITE_TX) {
        /* TX data - use byte access */
        iowrite8((u8)val, up->regs + offset);
    } else {
        /* Status/Control - use word access */
        iowrite32(val, up->regs + offset);
    }
}

/*
 * Receive characters from FIFO
 */
static void ulite_receive(struct uartlite_port *up)
{
    struct uart_port *port = &up->port;
    struct tty_port *tport = &port->state->port;
    u32 status;
    u8 ch;
    char flag;
    int max_count = 256;  /* Limit iterations */
    int received = 0;
    
    while (max_count-- > 0) {
        status = ulite_read(up, ULITE_STATUS);
        
        if (!(status & ULITE_STATUS_RXVALID))
            break;
        
        ch = ulite_read(up, ULITE_RX) & 0xFF;
        flag = TTY_NORMAL;
        port->icount.rx++;
        up->rx_count++;
        received++;
        
        /* Check for errors */
        if (status & ULITE_STATUS_PARITY) {
            port->icount.parity++;
            flag = TTY_PARITY;
            up->err_count++;
        }
        if (status & ULITE_STATUS_FRAME) {
            port->icount.frame++;
            flag = TTY_FRAME;
            up->err_count++;
        }
        if (status & ULITE_STATUS_OVERRUN) {
            port->icount.overrun++;
            up->err_count++;
            /*
             * Overrun is special - insert the good char first,
             * then report the overrun as a separate event
             */
            tty_insert_flip_char(tport, ch, flag);
            ch = 0;
            flag = TTY_OVERRUN;
        }
        
        /* Handle sysrq (magic keys) */
        if (uart_handle_sysrq_char(port, ch))
            continue;
        
        /* Insert into TTY buffer */
        tty_insert_flip_char(tport, ch, flag);
    }
    
    if (received > 0) {
        dev_dbg(&up->pdev->dev, "RX: received %d bytes (total=%lu)\n", 
                received, up->rx_count);
    }
    
    /* Push buffer to line discipline */
    if (received > 0)
        tty_flip_buffer_push(tport);
}

/*
 * Transmit characters from circular buffer
 */
static void ulite_transmit(struct uartlite_port *up)
{
    struct uart_port *port = &up->port;
    struct circ_buf *xmit = &port->state->xmit;
    u32 status;
    int count;
    int sent = 0;
    
    /* Handle x_char (high priority character like XON/XOFF) */
    if (port->x_char) {
        status = ulite_read(up, ULITE_STATUS);
        if (!(status & ULITE_STATUS_TXFULL)) {
            ulite_write(up, port->x_char, ULITE_TX);
            port->x_char = 0;
            port->icount.tx++;
            up->tx_count++;
        }
        return;
    }
    
    /* Check if TX is stopped or buffer empty */
    if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
        up->tx_busy = false;
        return;
    }
    
    /* Transmit up to FIFO size characters */
    count = ULITE_FIFO_SIZE;
    while (count-- > 0) {
        status = ulite_read(up, ULITE_STATUS);
        if (status & ULITE_STATUS_TXFULL)
            break;
        
        if (uart_circ_empty(xmit))
            break;
        
        ulite_write(up, xmit->buf[xmit->tail], ULITE_TX);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
        up->tx_count++;
        sent++;
    }
    
    if (sent > 0) {
        dev_dbg(&up->pdev->dev, "TX: sent %d bytes (total=%lu)\n",
                sent, up->tx_count);
    }
    
    /* Wake up writers if buffer space available */
    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
    
    /* Check if more to send */
    up->tx_busy = !uart_circ_empty(xmit);
}

/*
 * Main interrupt/poll handler
 */
static void ulite_handle_port(struct uartlite_port *up)
{
    unsigned long flags;
    
    spin_lock_irqsave(&up->port.lock, flags);
    
    /* Handle receive */
    ulite_receive(up);
    
    /* Handle transmit */
    if (up->tx_busy)
        ulite_transmit(up);
    
    spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * IRQ notification callback from PCIe bridge
 */
static int ulite_irq_notify(struct notifier_block *nb, unsigned long pending, void *data)
{
    struct uartlite_port *up = container_of(nb, struct uartlite_port, irq_nb);
    u32 status;
    
    /* Check if our IRQ bit is set */
    if (!(pending & BIT(up->irq_idx)))
        return NOTIFY_DONE;
    
    /* Debug: read status to see what triggered the interrupt */
    status = ulite_read(up, ULITE_STATUS);
    
    dev_dbg(&up->pdev->dev, "IRQ notify: pending=0x%lx status=0x%x\n", 
            pending, status);
    
    ulite_handle_port(up);
    
    return NOTIFY_OK;
}

/*
 * Direct IRQ handler (used when bridge doesn't use notifier)
 */
static irqreturn_t ulite_isr(int irq, void *dev_id)
{
    struct uartlite_port *up = dev_id;
    u32 status;
    
    status = ulite_read(up, ULITE_STATUS);
    
    /* Check if we have anything to do */
    if (!(status & (ULITE_STATUS_RXVALID | ULITE_STATUS_TXEMPTY)))
        return IRQ_NONE;
    
    ulite_handle_port(up);
    
    return IRQ_HANDLED;
}

/*
 * Polling timer callback
 */
static void ulite_poll_timer(struct timer_list *t)
{
    struct uartlite_port *up = from_timer(up, t, poll_timer);
    u32 status;
    static int poll_count = 0;
    
    if (!up->polling_active)
        return;
    
    /* Check status and log if there's RX data */
    status = ulite_read(up, ULITE_STATUS);
    if (status & ULITE_STATUS_RXVALID) {
        dev_dbg(&up->pdev->dev, "Poll: RX data available, status=0x%x\n", status);
    }
    
    ulite_handle_port(up);
    
    /* Log every 1000 polls (~1 second) to show polling is active */
    if (++poll_count >= 1000) {
        poll_count = 0;
        dev_dbg(&up->pdev->dev, "Poll: status=0x%x rx=%lu tx=%lu\n",
                status, up->rx_count, up->tx_count);
    }
    
    /* Re-arm timer */
    mod_timer(&up->poll_timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

/*
 * Start polling
 */
static void ulite_start_poll(struct uartlite_port *up)
{
    if (up->polling_active)
        return;
    
    up->polling_active = true;
    timer_setup(&up->poll_timer, ulite_poll_timer, 0);
    mod_timer(&up->poll_timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
    
    dev_info(&up->pdev->dev, "Started polling mode\n");
}

/*
 * Stop polling
 */
static void ulite_stop_poll(struct uartlite_port *up)
{
    if (!up->polling_active)
        return;
    
    up->polling_active = false;
    del_timer_sync(&up->poll_timer);
}

/*
 * UART operations
 */

static unsigned int ulite_tx_empty(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    u32 status = ulite_read(up, ULITE_STATUS);
    
    return (status & ULITE_STATUS_TXEMPTY) ? TIOCSER_TEMT : 0;
}

static void ulite_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    /* UARTLite has no modem control signals */
}

static unsigned int ulite_get_mctrl(struct uart_port *port)
{
    /* Report all modem signals as active (for compatibility) */
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ulite_stop_tx(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    up->tx_busy = false;
}

static void ulite_start_tx(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    
    up->tx_busy = true;
    ulite_transmit(up);
}

static void ulite_stop_rx(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    u32 control;
    
    /* Disable RX interrupt */
    control = ulite_read(up, ULITE_CONTROL);
    control &= ~ULITE_CONTROL_IE;
    ulite_write(up, control, ULITE_CONTROL);
}

static void ulite_break_ctl(struct uart_port *port, int ctl)
{
    /* UARTLite doesn't support break */
}

static int ulite_startup(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    int ret = 0;
    u32 status;
    
    dev_info(&up->pdev->dev, "UART startup\n");
    
    /* Reset FIFOs */
    ulite_write(up, ULITE_CONTROL_RST_TX | ULITE_CONTROL_RST_RX, ULITE_CONTROL);
    
    /* Small delay for reset to complete */
    udelay(100);
    
    /* Verify reset worked */
    status = ulite_read(up, ULITE_STATUS);
    dev_info(&up->pdev->dev, "After reset: status=0x%08x (expect 0x04)\n", status);
    
    /* Try to set up interrupts */
    up->use_irq = false;
    
    if (port->irq > 0) {
        /* Register with bridge's notifier system */
        up->irq_nb.notifier_call = ulite_irq_notify;
        up->irq_nb.priority = 0;
        
        ret = mcu_pcie_register_irq_handler(&up->pdev->dev, &up->irq_nb);
        if (ret == 0) {
            up->use_irq = true;
            dev_info(&up->pdev->dev, "Using bridge IRQ notifications (idx=%d)\n", 
                     up->irq_idx);
        } else {
            dev_warn(&up->pdev->dev, 
                     "IRQ registration failed (%d), using polling\n", ret);
        }
    }
    
    /* ALWAYS start polling as backup - this ensures RX works even if IRQs don't */
    ulite_start_poll(up);
    dev_info(&up->pdev->dev, "Polling enabled as backup\n");
    
    /* Enable interrupts in UARTLite */
    ulite_write(up, ULITE_CONTROL_IE, ULITE_CONTROL);
    
    /* Verify interrupt enable */
    status = ulite_read(up, ULITE_STATUS);
    dev_info(&up->pdev->dev, "After IE: status=0x%08x (bit4=IE should be set)\n", status);
    
    up->tx_busy = false;
    
    return 0;
}

static void ulite_shutdown(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    
    dev_info(&up->pdev->dev, "UART shutdown (rx=%lu tx=%lu err=%lu)\n",
             up->rx_count, up->tx_count, up->err_count);
    
    /* Stop polling */
    ulite_stop_poll(up);
    
    /* Disable interrupts in UARTLite */
    ulite_write(up, 0, ULITE_CONTROL);
    
    /* Unregister IRQ handler */
    if (up->use_irq) {
        mcu_pcie_unregister_irq_handler(&up->pdev->dev, &up->irq_nb);
    }
    
    up->use_irq = false;
}

static void ulite_set_termios(struct uart_port *port,
                              struct ktermios *termios,
                              const struct ktermios *old)
{
    unsigned long flags;
    unsigned int baud;
    
    spin_lock_irqsave(&port->lock, flags);
    
    /* UARTLite has fixed format: 8 data bits, 1 stop bit, no parity */
    termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);
    termios->c_cflag |= CS8;
    
    /* UARTLite doesn't support HW flow control */
    termios->c_cflag &= ~CRTSCTS;
    
    /* Calculate baud rate (for timeout calculation only - HW is fixed) */
    baud = uart_get_baud_rate(port, termios, old, 9600, 921600);
    uart_update_timeout(port, termios->c_cflag, baud);
    
    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ulite_type(struct uart_port *port)
{
    return port->type == PORT_UARTLITE ? DRIVER_NAME : NULL;
}

static void ulite_release_port(struct uart_port *port)
{
    /* Nothing to release - managed by platform device */
}

static int ulite_request_port(struct uart_port *port)
{
    return 0;
}

static void ulite_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_UARTLITE;
}

static int ulite_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    if (ser->type != PORT_UNKNOWN && ser->type != PORT_UARTLITE)
        return -EINVAL;
    return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int ulite_poll_get_char(struct uart_port *port)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    u32 status = ulite_read(up, ULITE_STATUS);
    
    if (!(status & ULITE_STATUS_RXVALID))
        return NO_POLL_CHAR;
    
    return ulite_read(up, ULITE_RX) & 0xFF;
}

static void ulite_poll_put_char(struct uart_port *port, unsigned char ch)
{
    struct uartlite_port *up = container_of(port, struct uartlite_port, port);
    int timeout = 10000;
    
    /* Wait for TX FIFO to have space */
    while (timeout-- > 0) {
        if (!(ulite_read(up, ULITE_STATUS) & ULITE_STATUS_TXFULL))
            break;
        udelay(1);
    }
    
    ulite_write(up, ch, ULITE_TX);
}
#endif

static const struct uart_ops uartlite_ops = {
    .tx_empty       = ulite_tx_empty,
    .set_mctrl      = ulite_set_mctrl,
    .get_mctrl      = ulite_get_mctrl,
    .stop_tx        = ulite_stop_tx,
    .start_tx       = ulite_start_tx,
    .stop_rx        = ulite_stop_rx,
    .break_ctl      = ulite_break_ctl,
    .startup        = ulite_startup,
    .shutdown       = ulite_shutdown,
    .set_termios    = ulite_set_termios,
    .type           = ulite_type,
    .release_port   = ulite_release_port,
    .request_port   = ulite_request_port,
    .config_port    = ulite_config_port,
    .verify_port    = ulite_verify_port,
#ifdef CONFIG_CONSOLE_POLL
    .poll_get_char  = ulite_poll_get_char,
    .poll_put_char  = ulite_poll_put_char,
#endif
};

/*
 * Platform driver probe
 */
static int uartlite_probe(struct platform_device *pdev)
{
    struct uartlite_port *up;
    struct resource *res;
    int ret;
    int port_num;
    int *irq_idx_ptr;
    
    dev_info(&pdev->dev, "UARTLite probe\n");
    
    /* Find an available port number */
    mutex_lock(&uartlite_mutex);
    for (port_num = 0; port_num < UARTLITE_NR; port_num++) {
        if (!uartlite_ports[port_num])
            break;
    }
    if (port_num >= UARTLITE_NR) {
        mutex_unlock(&uartlite_mutex);
        dev_err(&pdev->dev, "No available port slots\n");
        return -ENODEV;
    }
    
    /* Allocate port structure */
    up = devm_kzalloc(&pdev->dev, sizeof(*up), GFP_KERNEL);
    if (!up) {
        mutex_unlock(&uartlite_mutex);
        return -ENOMEM;
    }
    
    uartlite_ports[port_num] = up;
    uartlite_port_count++;
    mutex_unlock(&uartlite_mutex);
    
    up->pdev = pdev;
    
    /* Get IRQ index from platform data */
    irq_idx_ptr = dev_get_platdata(&pdev->dev);
    up->irq_idx = irq_idx_ptr ? *irq_idx_ptr : 0;
    
    /* Get memory resource */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No memory resource\n");
        ret = -ENODEV;
        goto err_free_slot;
    }
    
    /* Map registers */
    up->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
    if (!up->regs) {
        dev_err(&pdev->dev, "Failed to map registers at 0x%llx\n",
                (unsigned long long)res->start);
        ret = -ENOMEM;
        goto err_free_slot;
    }
    
    /* Initialize uart_port structure */
    up->port.dev = &pdev->dev;
    up->port.type = PORT_UARTLITE;
    up->port.iotype = UPIO_MEM;
    up->port.mapbase = res->start;
    up->port.membase = up->regs;
    up->port.ops = &uartlite_ops;
    up->port.flags = UPF_BOOT_AUTOCONF;
    up->port.fifosize = ULITE_FIFO_SIZE;
    up->port.line = port_num;
    
    /* Get IRQ (optional - will use polling if not available) */
    up->port.irq = platform_get_irq(pdev, 0);
    if (up->port.irq < 0) {
        dev_info(&pdev->dev, "No IRQ, will use polling mode\n");
        up->port.irq = 0;
    }
    
    /* Fixed baud rate - UARTLite is typically configured at synthesis time */
    up->port.uartclk = 115200 * 16;  /* Assume 115200 baud */
    
    spin_lock_init(&up->port.lock);
    
    platform_set_drvdata(pdev, up);
    
    /* Verify UART is accessible by reading status register */
    {
        u32 status = ulite_read(up, ULITE_STATUS);
        u8 test_byte;
        
        dev_info(&pdev->dev, "Initial status: 0x%08x\n", status);
        
        /* Reset the UART */
        ulite_write(up, ULITE_CONTROL_RST_TX | ULITE_CONTROL_RST_RX, ULITE_CONTROL);
        udelay(100);
        
        status = ulite_read(up, ULITE_STATUS);
        dev_info(&pdev->dev, "After reset status: 0x%08x (expect TXEMPTY=0x04)\n", status);
        
        /* Test byte access to TX - write and check status changes */
        dev_info(&pdev->dev, "Testing byte access to TX register...\n");
        iowrite8(0x00, up->regs + ULITE_TX);  /* Send NULL byte as test */
        udelay(100);
        status = ulite_read(up, ULITE_STATUS);
        dev_info(&pdev->dev, "After TX test: status=0x%08x\n", status);
    }
    
    /* Register with uart subsystem */
    ret = uart_add_one_port(&uartlite_uart_driver, &up->port);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add UART port: %d\n", ret);
        goto err_free_slot;
    }
    
    dev_info(&pdev->dev, "Xilinx UARTLite registered as /dev/ttyUL%d\n", port_num);
    dev_info(&pdev->dev, "  Memory: 0x%llx, IRQ: %d, IRQ idx: %d\n",
             (unsigned long long)res->start, up->port.irq, up->irq_idx);
    
    return 0;

err_free_slot:
    mutex_lock(&uartlite_mutex);
    uartlite_ports[port_num] = NULL;
    uartlite_port_count--;
    mutex_unlock(&uartlite_mutex);
    return ret;
}

static int uartlite_remove(struct platform_device *pdev)
{
    struct uartlite_port *up = platform_get_drvdata(pdev);
    int port_num = up->port.line;
    
    dev_info(&pdev->dev, "UARTLite remove\n");
    
    uart_remove_one_port(&uartlite_uart_driver, &up->port);
    
    mutex_lock(&uartlite_mutex);
    uartlite_ports[port_num] = NULL;
    uartlite_port_count--;
    mutex_unlock(&uartlite_mutex);
    
    return 0;
}

static struct platform_driver uartlite_platform_driver = {
    .probe  = uartlite_probe,
    .remove = uartlite_remove,
    .driver = {
        .name = DRIVER_NAME,
    },
};

static struct uart_driver uartlite_uart_driver = {
    .owner          = THIS_MODULE,
    .driver_name    = DRIVER_NAME,
    .dev_name       = UARTLITE_NAME,
    .major          = UARTLITE_MAJOR,
    .minor          = UARTLITE_MINOR,
    .nr             = UARTLITE_NR,
};

static int __init uartlite_init(void)
{
    int ret;
    
    pr_info("Xilinx UARTLite driver for PCIe bridge (12 ports)\n");
    
    /* Register UART driver first */
    ret = uart_register_driver(&uartlite_uart_driver);
    if (ret) {
        pr_err("Failed to register UART driver: %d\n", ret);
        return ret;
    }
    
    /* Then register platform driver */
    ret = platform_driver_register(&uartlite_platform_driver);
    if (ret) {
        pr_err("Failed to register platform driver: %d\n", ret);
        uart_unregister_driver(&uartlite_uart_driver);
        return ret;
    }
    
    return 0;
}

static void __exit uartlite_exit(void)
{
    platform_driver_unregister(&uartlite_platform_driver);
    uart_unregister_driver(&uartlite_uart_driver);
    pr_info("Xilinx UARTLite driver unloaded\n");
}

module_init(uartlite_init);
module_exit(uartlite_exit);

MODULE_AUTHOR("MCU Team");
MODULE_DESCRIPTION("Xilinx UARTLite driver for PCIe bridge");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
