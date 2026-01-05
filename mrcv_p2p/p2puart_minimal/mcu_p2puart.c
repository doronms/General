/*
 * Minimal P2PUART Driver
 * 
 * Written from scratch based on:
 * - mcu_pcie platform device interface
 * - P2PUART ComCtrl register specification
 * 
 * Philosophy: Simple, correct, minimal. Add complexity only when proven needed.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#define DRIVER_NAME     "p2puart"
#define DRIVER_VERSION  "2.0-minimal"
#define TTY_NAME        "ttyP2P"
#define MAX_PORTS       16

/* ============================================================================
 * Register Map (from datasheet)
 * ============================================================================ */

/* ComCtrl Common Registers */
#define REG_MVR             0x04    /* Module Version */
#define REG_MPR             0x08    /* Module Properties */
#define REG_CPR             0x0C    /* Channel Properties */
#define REG_SOFT_RESET      0x10    /* Soft Reset */
#define REG_RXBUF0_CTRL     0x14    /* RX Buffer 0 Control */
#define REG_RXBUF1_CTRL     0x18    /* RX Buffer 1 Control */
#define REG_TXBUF0_CTRL     0x1C    /* TX Buffer 0 Control */
#define REG_TXBUF1_CTRL     0x20    /* TX Buffer 1 Control */
#define REG_ISR             0x24    /* Interrupt Source */
#define REG_IPR             0x28    /* Interrupt Pending */
#define REG_IMR             0x2C    /* Interrupt Mask */
#define REG_TX_BUF_ADDR     0x38    /* TX Buffer Address */
#define REG_RX_BUF_ADDR     0x3C    /* RX Buffer Address */

/* P2P UART Specific Registers */
#define REG_SCP             0x60    /* Specific Channel Properties */
#define REG_BAUD            0x64    /* Baud Rate */
#define REG_SCAP            0x80    /* Specific Capabilities */

/* MPR Fields */
#define MPR_CLK_FREQ_MASK       GENMASK(17, 0)
#define MPR_RX_DPR_SIZE_SHIFT   20
#define MPR_RX_DPR_SIZE_MASK    GENMASK(25, 20)
#define MPR_TX_DPR_SIZE_SHIFT   26
#define MPR_TX_DPR_SIZE_MASK    GENMASK(31, 26)

/* Reset Values */
#define RESET_RX_ASSERT         0xDEAD0000
#define RESET_RX_RELEASE        0xBA5E0000
#define RESET_TX_ASSERT         0x0000DEAD
#define RESET_TX_RELEASE        0x0000BA5E

/* TxBuf_Ctrl Fields */
#define TXBUF_SIZE_SHIFT        16
#define TXBUF_SIZE_MASK         GENMASK(30, 16)
#define TXBUF_TX_START          BIT(0)
#define TXBUF_BUSY              BIT(1)

/* RxBuf_Ctrl Fields */
#define RXBUF_LAST_ADDR_SHIFT   16
#define RXBUF_LAST_ADDR_MASK    GENMASK(30, 16)
#define RXBUF_DONE              BIT(9)
#define RXBUF_OVERRUN           BIT(0)
#define RXBUF_FRAME_ERR         BIT(1)
#define RXBUF_BREAK_ERR         BIT(2)
#define RXBUF_PARITY_ERR        BIT(3)
#define RXBUF_ALL_ERRORS        (RXBUF_OVERRUN | RXBUF_FRAME_ERR | RXBUF_BREAK_ERR | RXBUF_PARITY_ERR)

/* IPR/IMR Bits */
#define IPR_RX_SIZE_DONE0       BIT(0)
#define IPR_RX_SIZE_DONE1       BIT(1)
#define IPR_RX_LINK_DONE0       BIT(2)
#define IPR_RX_LINK_DONE1       BIT(3)
#define IPR_OVERRUN0            BIT(4)
#define IPR_OVERRUN1            BIT(5)
#define IPR_TIMEOUT0            BIT(6)
#define IPR_TIMEOUT1            BIT(7)
#define IPR_LAST_BUF            BIT(15)
#define IPR_TX_DONE0            BIT(16)
#define IPR_TX_DONE1            BIT(17)

#define IPR_RX0_ALL     (IPR_RX_SIZE_DONE0 | IPR_RX_LINK_DONE0 | IPR_OVERRUN0 | IPR_TIMEOUT0)
#define IPR_RX1_ALL     (IPR_RX_SIZE_DONE1 | IPR_RX_LINK_DONE1 | IPR_OVERRUN1 | IPR_TIMEOUT1)
#define IPR_RX_ALL      (IPR_RX0_ALL | IPR_RX1_ALL)
#define IPR_TX_ALL      (IPR_TX_DONE0 | IPR_TX_DONE1)
#define IPR_ALL         (IPR_RX_ALL | IPR_TX_ALL)

/* SCP Fields */
#define SCP_TX_PARITY_MASK      GENMASK(2, 0)
#define SCP_RX_PARITY_SHIFT     4
#define SCP_STOP_BITS_SHIFT     8
#define SCP_STOP_BITS_MASK      GENMASK(9, 8)
#define SCP_CHAR_TIMEOUT_SHIFT  16
#define SCP_CHAR_TIMEOUT_MASK   GENMASK(23, 16)

#define PARITY_NONE             0x2
#define STOP_BITS_1             0x0
#define STOP_BITS_2             0x2

/* CPR Fields */
#define CPR_EOP_TRIGGER_SHIFT   4
#define CPR_EOP_TRIGGER_MASK    GENMASK(5, 4)
#define CPR_RX_PKT_SIZE_SHIFT   16
#define CPR_RX_PKT_SIZE_MASK    GENMASK(30, 16)

#define EOP_LINK_DONE           0x0
#define EOP_PACKET_SIZE         0x1
#define EOP_BOTH                0x2

/* ============================================================================
 * Driver State
 * ============================================================================ */

struct p2p_port {
    struct uart_port        port;
    
    /* Buffer geometry (from hardware) */
    u32                     rx_buf_size;
    u32                     tx_buf_size;
    u32                     rx_buf_offset[2];
    u32                     tx_buf_offset[2];
    
    /* Runtime state */
    u8                      next_tx_buf;
    u8                      next_rx_buf;
    bool                    tx_busy[2];
    
    /* Hardware info */
    u32                     clk_freq_khz;
    
    /* Temp buffer for RX with errors */
    u8                      *rx_temp;
};

#define to_p2p_port(p)  container_of(p, struct p2p_port, port)

/* Static driver data */
static struct uart_driver p2p_uart_driver;
static struct p2p_port *p2p_ports[MAX_PORTS];
static DEFINE_MUTEX(port_mutex);
static int port_count;

/* ============================================================================
 * Register Access
 * ============================================================================ */

static inline u32 p2p_read(struct uart_port *port, u32 reg)
{
    u32 val = readl(port->membase + reg);
    return val;
}

static inline void p2p_write(struct uart_port *port, u32 reg, u32 val)
{
    writel(val, port->membase + reg);
}

/* Force write completion on PCIe */
static inline void p2p_write_flush(struct uart_port *port, u32 reg, u32 val)
{
    writel(val, port->membase + reg);
    wmb();
    (void)readl(port->membase + reg);
}

/* ============================================================================
 * Hardware Operations
 * ============================================================================ */

static void p2p_reset_rx(struct uart_port *port)
{
    p2p_write(port, REG_SOFT_RESET, RESET_RX_ASSERT);
    udelay(10);
    p2p_write(port, REG_SOFT_RESET, RESET_RX_RELEASE);
}

static void p2p_reset_tx(struct uart_port *port)
{
    p2p_write(port, REG_SOFT_RESET, RESET_TX_ASSERT);
    udelay(10);
    p2p_write(port, REG_SOFT_RESET, RESET_TX_RELEASE);
}

static void p2p_set_baud(struct uart_port *port, unsigned int baud)
{
    struct p2p_port *pp = to_p2p_port(port);
    u64 clk_hz = (u64)pp->clk_freq_khz * 1000;
    u32 divisor, n, d;
    
    /* Baud = clk_hz * N / (D * 16) => D = clk_hz * N / (Baud * 16) */
    /* Use N = 1 for simplicity */
    n = 1;
    divisor = div64_u64(clk_hz * n, (u64)baud * 16);
    d = clamp_val(divisor, 1, 0xFFFF);
    
    p2p_write(port, REG_BAUD, (n << 16) | d);
}

static void p2p_set_termios_hw(struct uart_port *port, unsigned int baud,
                                unsigned int parity, unsigned int stop_bits)
{
    u32 scp;
    
    scp = p2p_read(port, REG_SCP);
    
    /* Clear parity and stop bits fields */
    scp &= ~(SCP_TX_PARITY_MASK | (SCP_TX_PARITY_MASK << SCP_RX_PARITY_SHIFT) | SCP_STOP_BITS_MASK);
    
    /* Set parity (TX and RX same) */
    scp |= parity | (parity << SCP_RX_PARITY_SHIFT);
    
    /* Set stop bits */
    scp |= (stop_bits << SCP_STOP_BITS_SHIFT);
    
    /* Character timeout = 3 (default) */
    scp &= ~SCP_CHAR_TIMEOUT_MASK;
    scp |= (3 << SCP_CHAR_TIMEOUT_SHIFT);
    
    p2p_write(port, REG_SCP, scp);
    p2p_set_baud(port, baud);
}

/* ============================================================================
 * TX Path - Simple and Direct
 * ============================================================================ */

static bool p2p_tx_buffer_busy(struct uart_port *port, int buf)
{
    u32 reg = (buf == 0) ? REG_TXBUF0_CTRL : REG_TXBUF1_CTRL;
    return !!(p2p_read(port, reg) & TXBUF_BUSY);
}

static bool p2p_start_tx_buffer(struct uart_port *port, int buf, u32 size)
{
    struct p2p_port *pp = to_p2p_port(port);
    u32 reg = (buf == 0) ? REG_TXBUF0_CTRL : REG_TXBUF1_CTRL;
    u32 ctrl;
    int retry;
    
    /* Build control word: size in upper bits */
    ctrl = (size << TXBUF_SIZE_SHIFT) & TXBUF_SIZE_MASK;
    
    /* Issue TX_START and verify handshake */
    for (retry = 0; retry < 16; retry++) {
        p2p_write(port, reg, ctrl | TXBUF_TX_START);
        wmb();
        
        if (p2p_read(port, reg) & TXBUF_BUSY) {
            pp->tx_busy[buf] = true;
            return true;
        }
        cpu_relax();
    }
    
    dev_err(port->dev, "TX_START handshake failed buf=%d size=%u\n", buf, size);
    return false;
}

static void p2p_do_tx(struct uart_port *port)
{
    struct p2p_port *pp = to_p2p_port(port);
    struct circ_buf *xmit = &port->state->xmit;
    int buf = pp->next_tx_buf;
    u32 offset, count, chunk;
    
    if (uart_tx_stopped(port) || uart_circ_empty(xmit))
        return;
    
    /* Check if current buffer is available */
    if (pp->tx_busy[buf] && p2p_tx_buffer_busy(port, buf))
        return;
    
    pp->tx_busy[buf] = false;
    offset = pp->tx_buf_offset[buf];
    count = 0;
    
    /* Copy data to TX buffer */
    while (!uart_circ_empty(xmit) && count < pp->tx_buf_size) {
        chunk = min_t(u32, pp->tx_buf_size - count,
                      CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));
        
        memcpy_toio(port->membase + offset + count, &xmit->buf[xmit->tail], chunk);
        xmit->tail = (xmit->tail + chunk) & (UART_XMIT_SIZE - 1);
        count += chunk;
    }
    
    if (count == 0)
        return;
    
    wmb();
    port->icount.tx += count;
    
    /* Start transmission */
    if (p2p_start_tx_buffer(port, buf, count))
        pp->next_tx_buf = !buf;
    
    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

/* ============================================================================
 * RX Path - Simple and Direct
 * ============================================================================ */

static u32 p2p_get_rx_last_addr(u32 ctrl)
{
    return (ctrl & RXBUF_LAST_ADDR_MASK) >> RXBUF_LAST_ADDR_SHIFT;
}

static void p2p_clear_rx_buffer(struct uart_port *port, int buf, u32 ctrl)
{
    u32 reg = (buf == 0) ? REG_RXBUF0_CTRL : REG_RXBUF1_CTRL;
    u32 clear = 0;
    
    /* Clear any error bits that are set */
    clear |= (ctrl & 0xFF);
    
    /* Clear Buffer_Done if set */
    if (ctrl & RXBUF_DONE)
        clear |= RXBUF_DONE;
    
    if (clear)
        p2p_write_flush(port, reg, clear);
}

static void p2p_rx_buffer(struct uart_port *port, int buf, u32 ipr)
{
    struct p2p_port *pp = to_p2p_port(port);
    u32 reg = (buf == 0) ? REG_RXBUF0_CTRL : REG_RXBUF1_CTRL;
    u32 ctrl, last_addr, count, errors;
    char flag = TTY_NORMAL;
    
    /* Snapshot control register */
    ctrl = p2p_read(port, reg);
    last_addr = p2p_get_rx_last_addr(ctrl);
    count = last_addr + 1;
    
    if (count == 0 || count > pp->rx_buf_size) {
        p2p_clear_rx_buffer(port, buf, ctrl);
        return;
    }
    
    /* Check for errors */
    errors = ctrl & RXBUF_ALL_ERRORS;
    if (errors) {
        if (errors & RXBUF_OVERRUN) {
            flag = TTY_OVERRUN;
            port->icount.overrun++;
        } else if (errors & RXBUF_PARITY_ERR) {
            flag = TTY_PARITY;
            port->icount.parity++;
        } else if (errors & RXBUF_FRAME_ERR) {
            flag = TTY_FRAME;
            port->icount.frame++;
        }
    }
    
    /* Copy data to TTY */
    if (flag == TTY_NORMAL) {
        /* Fast path */
        unsigned char *dest;
        int space = tty_prepare_flip_string(&port->state->port, &dest, count);
        if (space > 0) {
            memcpy_fromio(dest, port->membase + pp->rx_buf_offset[buf], 
                         min_t(u32, space, count));
        }
    } else {
        /* Slow path with error flag */
        memcpy_fromio(pp->rx_temp, port->membase + pp->rx_buf_offset[buf], count);
        tty_insert_flip_string_fixed_flag(&port->state->port, pp->rx_temp, flag, count);
    }
    
    port->icount.rx += count;
    tty_flip_buffer_push(&port->state->port);
    
    /* Release buffer back to hardware */
    p2p_clear_rx_buffer(port, buf, ctrl);
}

static void p2p_do_rx(struct uart_port *port, u32 ipr)
{
    struct p2p_port *pp = to_p2p_port(port);
    bool rx0 = !!(ipr & IPR_RX0_ALL);
    bool rx1 = !!(ipr & IPR_RX1_ALL);
    int first, second;
    
    if (!rx0 && !rx1)
        return;
    
    if (rx0 && rx1) {
        /* Both buffers ready - process in order based on LAST_BUF */
        if (ipr & IPR_LAST_BUF) {
            first = 0; second = 1;
        } else {
            first = 1; second = 0;
        }
        p2p_rx_buffer(port, first, ipr);
        p2p_rx_buffer(port, second, ipr);
        pp->next_rx_buf = second;
    } else {
        /* Single buffer */
        first = rx0 ? 0 : 1;
        p2p_rx_buffer(port, first, ipr);
        pp->next_rx_buf = !first;
    }
}

/* ============================================================================
 * Interrupt Handler
 * ============================================================================ */

static irqreturn_t p2p_irq_handler(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    struct p2p_port *pp = to_p2p_port(port);
    unsigned long flags;
    u32 ipr, ack = 0;
    
    spin_lock_irqsave(&port->lock, flags);
    
    ipr = p2p_read(port, REG_IPR);
    
    if (!(ipr & IPR_ALL)) {
        spin_unlock_irqrestore(&port->lock, flags);
        return IRQ_NONE;
    }
    
    /* Handle TX completion first (simple) */
    if (ipr & IPR_TX_ALL) {
        if (ipr & IPR_TX_DONE0)
            pp->tx_busy[0] = false;
        if (ipr & IPR_TX_DONE1)
            pp->tx_busy[1] = false;
        
        ack |= (ipr & IPR_TX_ALL);
        p2p_do_tx(port);
    }
    
    /* Handle RX */
    if (ipr & IPR_RX_ALL) {
        p2p_do_rx(port, ipr);
        ack |= (ipr & IPR_RX_ALL);
    }
    
    /* Acknowledge processed interrupts */
    if (ack)
        p2p_write_flush(port, REG_IPR, ack);
    
    spin_unlock_irqrestore(&port->lock, flags);
    
    return IRQ_HANDLED;
}

/* ============================================================================
 * UART Operations
 * ============================================================================ */

static unsigned int p2p_tx_empty(struct uart_port *port)
{
    struct p2p_port *pp = to_p2p_port(port);
    
    if (pp->tx_busy[0] || pp->tx_busy[1])
        return 0;
    
    return TIOCSER_TEMT;
}

static void p2p_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    /* No modem control lines */
}

static unsigned int p2p_get_mctrl(struct uart_port *port)
{
    return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void p2p_stop_tx(struct uart_port *port)
{
    /* Nothing to do - TX completes naturally */
}

static void p2p_start_tx(struct uart_port *port)
{
    p2p_do_tx(port);
}

static void p2p_stop_rx(struct uart_port *port)
{
    u32 imr = p2p_read(port, REG_IMR);
    imr |= IPR_RX_ALL;
    p2p_write(port, REG_IMR, imr);
}

static void p2p_break_ctl(struct uart_port *port, int break_state)
{
    /* Not supported */
}

static int p2p_startup(struct uart_port *port)
{
    struct p2p_port *pp = to_p2p_port(port);
    u32 cpr;
    int ret;
    
    /* Reset both RX and TX */
    p2p_reset_rx(port);
    p2p_reset_tx(port);
    
    /* Initialize state */
    pp->next_tx_buf = 0;
    pp->next_rx_buf = 0;
    pp->tx_busy[0] = false;
    pp->tx_busy[1] = false;
    
    /* Configure CPR: EOP on both packet size and link done */
    cpr = p2p_read(port, REG_CPR);
    cpr &= ~(CPR_EOP_TRIGGER_MASK | CPR_RX_PKT_SIZE_MASK);
    cpr |= (EOP_BOTH << CPR_EOP_TRIGGER_SHIFT);
    cpr |= ((pp->rx_buf_size - 1) << CPR_RX_PKT_SIZE_SHIFT);
    p2p_write(port, REG_CPR, cpr);
    
    /* Request IRQ */
    ret = request_irq(port->irq, p2p_irq_handler, IRQF_SHARED, DRIVER_NAME, port);
    if (ret) {
        dev_err(port->dev, "Failed to request IRQ %d: %d\n", port->irq, ret);
        return ret;
    }
    
    /* Unmask all interrupts */
    p2p_write(port, REG_IMR, 0);
    
    /* Clear any pending interrupts */
    p2p_write_flush(port, REG_IPR, IPR_ALL);
    
    return 0;
}

static void p2p_shutdown(struct uart_port *port)
{
    /* Mask all interrupts */
    p2p_write(port, REG_IMR, IPR_ALL);
    
    /* Free IRQ */
    free_irq(port->irq, port);
    
    /* Reset hardware */
    p2p_reset_rx(port);
    p2p_reset_tx(port);
}

static void p2p_set_termios(struct uart_port *port, struct ktermios *termios,
                            const struct ktermios *old)
{
    unsigned int baud, parity, stop_bits;
    unsigned long flags;
    
    /* Get baud rate */
    baud = uart_get_baud_rate(port, termios, old, 9600, 4000000);
    
    /* Get parity */
    if (termios->c_cflag & PARENB) {
        if (termios->c_cflag & PARODD)
            parity = 0x1;  /* Odd */
        else
            parity = 0x0;  /* Even */
    } else {
        parity = PARITY_NONE;
    }
    
    /* Get stop bits */
    if (termios->c_cflag & CSTOPB)
        stop_bits = STOP_BITS_2;
    else
        stop_bits = STOP_BITS_1;
    
    spin_lock_irqsave(&port->lock, flags);
    
    /* Update hardware */
    p2p_set_termios_hw(port, baud, parity, stop_bits);
    
    /* Update port->read_status_mask and ignore_status_mask */
    port->read_status_mask = RXBUF_OVERRUN;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= RXBUF_FRAME_ERR | RXBUF_PARITY_ERR;
    
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= RXBUF_PARITY_ERR | RXBUF_FRAME_ERR;
    if (termios->c_iflag & IGNBRK)
        port->ignore_status_mask |= RXBUF_BREAK_ERR;
    
    /* Update timeout */
    uart_update_timeout(port, termios->c_cflag, baud);
    
    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *p2p_type(struct uart_port *port)
{
    return "P2PUART";
}

static void p2p_release_port(struct uart_port *port)
{
    /* Nothing to release */
}

static int p2p_request_port(struct uart_port *port)
{
    return 0;
}

static void p2p_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_16550A;
}

static int p2p_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    return 0;
}

static const struct uart_ops p2p_uart_ops = {
    .tx_empty       = p2p_tx_empty,
    .set_mctrl      = p2p_set_mctrl,
    .get_mctrl      = p2p_get_mctrl,
    .stop_tx        = p2p_stop_tx,
    .start_tx       = p2p_start_tx,
    .stop_rx        = p2p_stop_rx,
    .break_ctl      = p2p_break_ctl,
    .startup        = p2p_startup,
    .shutdown       = p2p_shutdown,
    .set_termios    = p2p_set_termios,
    .type           = p2p_type,
    .release_port   = p2p_release_port,
    .request_port   = p2p_request_port,
    .config_port    = p2p_config_port,
    .verify_port    = p2p_verify_port,
};

/* ============================================================================
 * Platform Driver
 * ============================================================================ */

static int p2p_probe(struct platform_device *pdev)
{
    struct p2p_port *pp;
    struct resource *mem, *irq_res;
    u32 mpr, tx_addr, rx_addr;
    int ret, line;
    
    /* Get resources from mcu_pcie parent */
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    
    if (!mem || !irq_res) {
        dev_err(&pdev->dev, "Missing resources\n");
        return -EINVAL;
    }
    
    /* Allocate port structure */
    pp = devm_kzalloc(&pdev->dev, sizeof(*pp), GFP_KERNEL);
    if (!pp)
        return -ENOMEM;
    
    /* Map registers */
    pp->port.membase = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
    if (!pp->port.membase) {
        dev_err(&pdev->dev, "Failed to map registers\n");
        return -ENOMEM;
    }
    
    /* Read hardware properties */
    mpr = p2p_read(&pp->port, REG_MPR);
    pp->clk_freq_khz = mpr & MPR_CLK_FREQ_MASK;
    pp->rx_buf_size = 1 << ((mpr & MPR_RX_DPR_SIZE_MASK) >> MPR_RX_DPR_SIZE_SHIFT);
    pp->tx_buf_size = 1 << ((mpr & MPR_TX_DPR_SIZE_MASK) >> MPR_TX_DPR_SIZE_SHIFT);
    
    /* Get buffer addresses */
    tx_addr = p2p_read(&pp->port, REG_TX_BUF_ADDR);
    rx_addr = p2p_read(&pp->port, REG_RX_BUF_ADDR);
    
    pp->tx_buf_offset[0] = tx_addr;
    pp->tx_buf_offset[1] = tx_addr + pp->tx_buf_size;
    pp->rx_buf_offset[0] = rx_addr;
    pp->rx_buf_offset[1] = rx_addr + pp->rx_buf_size;
    
    /* Allocate temp RX buffer */
    pp->rx_temp = devm_kzalloc(&pdev->dev, pp->rx_buf_size, GFP_KERNEL);
    if (!pp->rx_temp)
        return -ENOMEM;
    
    /* Find available line number */
    mutex_lock(&port_mutex);
    for (line = 0; line < MAX_PORTS; line++) {
        if (!p2p_ports[line])
            break;
    }
    if (line >= MAX_PORTS) {
        mutex_unlock(&port_mutex);
        dev_err(&pdev->dev, "No available port slots\n");
        return -ENOSPC;
    }
    p2p_ports[line] = pp;
    port_count++;
    mutex_unlock(&port_mutex);
    
    /* Initialize uart_port */
    pp->port.dev = &pdev->dev;
    pp->port.mapbase = mem->start;
    pp->port.irq = irq_res->start;
    pp->port.type = PORT_16550A;
    pp->port.iotype = UPIO_MEM32;
    pp->port.regshift = 0;
    pp->port.fifosize = pp->rx_buf_size;
    pp->port.ops = &p2p_uart_ops;
    pp->port.flags = UPF_BOOT_AUTOCONF;
    pp->port.line = line;
    pp->port.uartclk = pp->clk_freq_khz * 1000;
    spin_lock_init(&pp->port.lock);
    
    platform_set_drvdata(pdev, pp);
    
    /* Register with serial core */
    ret = uart_add_one_port(&p2p_uart_driver, &pp->port);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add uart port: %d\n", ret);
        mutex_lock(&port_mutex);
        p2p_ports[line] = NULL;
        port_count--;
        mutex_unlock(&port_mutex);
        return ret;
    }
    
    dev_info(&pdev->dev, "P2PUART %d: clk=%u kHz, rx_buf=%u, tx_buf=%u, irq=%d\n",
             line, pp->clk_freq_khz, pp->rx_buf_size, pp->tx_buf_size, pp->port.irq);
    
    return 0;
}

static int p2p_remove(struct platform_device *pdev)
{
    struct p2p_port *pp = platform_get_drvdata(pdev);
    int line = pp->port.line;
    
    uart_remove_one_port(&p2p_uart_driver, &pp->port);
    
    mutex_lock(&port_mutex);
    p2p_ports[line] = NULL;
    port_count--;
    mutex_unlock(&port_mutex);
    
    return 0;
}

static struct platform_driver p2p_platform_driver = {
    .probe  = p2p_probe,
    .remove = p2p_remove,
    .driver = {
        .name = "p2puart",
    },
};

/* ============================================================================
 * Module Init/Exit
 * ============================================================================ */

static struct uart_driver p2p_uart_driver = {
    .owner          = THIS_MODULE,
    .driver_name    = DRIVER_NAME,
    .dev_name       = TTY_NAME,
    .major          = 0,  /* Dynamic */
    .minor          = 0,
    .nr             = MAX_PORTS,
};

static int __init p2p_init(void)
{
    int ret;
    
    pr_info("P2PUART: %s version %s\n", DRIVER_NAME, DRIVER_VERSION);
    
    ret = uart_register_driver(&p2p_uart_driver);
    if (ret) {
        pr_err("P2PUART: Failed to register uart driver: %d\n", ret);
        return ret;
    }
    
    ret = platform_driver_register(&p2p_platform_driver);
    if (ret) {
        pr_err("P2PUART: Failed to register platform driver: %d\n", ret);
        uart_unregister_driver(&p2p_uart_driver);
        return ret;
    }
    
    return 0;
}

static void __exit p2p_exit(void)
{
    platform_driver_unregister(&p2p_platform_driver);
    uart_unregister_driver(&p2p_uart_driver);
    pr_info("P2PUART: Driver unloaded\n");
}

module_init(p2p_init);
module_exit(p2p_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Minimal Rewrite");
MODULE_DESCRIPTION("Minimal P2PUART Serial Driver");
MODULE_VERSION(DRIVER_VERSION);
