/*
 * P2PUART serial driver for Linux - CORRECTED VERSION
 * The P2PUART IP block contains a single uart port.
 * The driver supports IRQ sharing, so that multiple ports can share
 * the same IRQ signal.
 * 
 * CRITICAL FIXES:
 * 1. Fixed RX throttle/unthrottle mechanism
 * 2. Corrected IPR acknowledgment timing
 * 3. Fixed buffer clearing race conditions
 * 4. Added proper memory barriers
 * 5. Improved TX_START handshake reliability
 */

#if defined(CONFIG_SERIAL_P2PUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/timekeeping.h>
#include <linux/version.h>
#include <linux/printk.h>

#include "p2puart_ioctls.h"
#include "mcu_pcie.h"

#define P2PUART_DESC "P2PUART Linux Serial Driver Over MCU PCIe"
#define P2PUART_VER "1.14-FIXED"

/* Debug options - configure via EXTRA_CFLAGS */
//#define DEBUG_PRINT_REGISTERS_ACCESS
//#define DEBUG_PRINT_DEVICE_MAPPINGS
//#define DEBUG_PRINT_FUNCTION_ENTRIES
//#define DEBUG_PRINT_FUNCTION_RETURNS
//#define DEBUG_PRINT_BAUD_RATE_CALCS
//#define DEBUG_PRINT_TX_COMMANDS
//#define DEBUG_PRINT_RX_INTERRUPT
//#define DEBUG_PRINT_PARITY
//#define DEBUG_WITH_UNITTEST_IOCTLS
//#define DEBUG_PRINT_RX_BUFFERS
//#define DEBUG_PRINT_TX_BUFFERS

#ifdef DEBUG_PRINT_FUNCTION_ENTRIES
#define PRINT_FUNCTION_ENTRY(funcname)                                         \
	printk("%s: entered %s()\n", to_p2puart_port(port)->name, funcname);
#else
#define PRINT_FUNCTION_ENTRY(funcname)
#endif

#ifdef DEBUG_PRINT_FUNCTION_RETURNS
#define PRINT_FUNCTION_RETURN(funcname)                                        \
	printk("%s: returning from %s()\n", to_p2puart_port(port)->name,       \
	       funcname);
#else
#define PRINT_FUNCTION_RETURN(funcname)
#endif

#define PRINT_RX_BUFFERS_ALL_PORTS -1
#define PRINT_TX_BUFFERS_ALL_PORTS -1

#ifndef DEBUG_PRINT_RX_BUFFERS_PORT_NUM
#define DEBUG_PRINT_RX_BUFFERS_PORT_NUM PRINT_RX_BUFFERS_ALL_PORTS
#endif

#ifndef DEBUG_PRINT_TX_BUFFERS_PORT_NUM
#define DEBUG_PRINT_TX_BUFFERS_PORT_NUM PRINT_RX_BUFFERS_ALL_PORTS
#endif

#ifdef DEBUG_PRINT_RX_BUFFERS
#define PRINT_RX_BUFFER(port, p, size)                                         \
	if (DEBUG_PRINT_RX_BUFFERS_PORT_NUM == PRINT_RX_BUFFERS_ALL_PORTS ||   \
	    DEBUG_PRINT_RX_BUFFERS_PORT_NUM == port->line) {                   \
		char msg[128];                                                 \
		printk("%s: <-- (len: %d bytes). Data: ", port->name, size);   \
		sprintf(msg, "%s: <-- ", port->name);                          \
		print_hex_dump(KERN_WARNING, msg, DUMP_PREFIX_OFFSET, 32, 1,   \
			       p, size, false);                                \
	}
#else
#define PRINT_RX_BUFFER(port, p, size)
#endif

#ifdef DEBUG_PRINT_TX_BUFFERS
#define PRINT_TX_BUFFER(port, p, size)                                         \
	if (DEBUG_PRINT_TX_BUFFERS_PORT_NUM == PRINT_TX_BUFFERS_ALL_PORTS ||   \
	    DEBUG_PRINT_TX_BUFFERS_PORT_NUM == port->line) {                   \
		char msg[128];                                                 \
		printk("%s: --> (len: %d bytes). Data: ", port->name, size);   \
		sprintf(msg, "%s: --> ", port->name);                          \
		print_hex_dump(KERN_WARNING, msg, DUMP_PREFIX_OFFSET, 32, 1,   \
			       p, size, false);                                \
	}
#else
#define PRINT_TX_BUFFER(port, p, size)
#endif

/* Maximum number of P2PUART ports the driver will manage. */
#define P2PUART_NR_MAX 32

/* device name */
#define P2PUART_TTY_NAME "ttyP2P"

/* ======================================================================== */
/* ComCtrl (common) registers                                               */
/* ======================================================================== */

/* Module Version Register */
#define COMCTRL_MVR 0x04

/* Module Properties Register */
#define COMCTRL_MPR 0x08

/* Channel Properties Register */
#define COMCTRL_CPR 0x0c

/* Soft Reset register */
#define SOFT_RESET 0x10

#define RXBUF0_CTRL 0x14
#define RXBUF1_CTRL 0x18

#define TXBUF0_CTRL 0x1c
#define TXBUF1_CTRL 0x20

/* Interrupt Source Register */
#define COMCTRL_ISR 0x24

/* Interrupt Pending Register */
#define COMCTRL_IPR 0x28

/* Interrupt Mask Register */
#define COMCTRL_IMR 0x2c

/* Register that contains the address of the Tx Buffer */
#define COMCTRL_TX_BUF_ADDR_OFFSET 0x38

/* Register that contains the address of the Rx Buffer */
#define COMCTRL_RX_BUF_ADDR_OFFSET 0x3c

/* MPR bit definitions */
#define COMCTRL_CLK_FREQ_KHZ_MASK GENMASK(17, 0)
#define COMCTRL_RX_DPR_SIZE_SHIFT 20
#define COMCTRL_RX_DPR_SIZE_MASK GENMASK(25, 20)
#define COMCTRL_TX_DPR_SIZE_SHIFT 26
#define COMCTRL_TX_DPR_SIZE_MASK GENMASK(31, 26)

/* Magic values for the SOFT_RESET register */
#define RESET_RX 0xdead0000
#define RESET_RX_NORMAL_OPERATION 0xba5e0000
#define RESET_TX 0xdead
#define RESET_TX_NORMAL_OPERATION 0xba5e

/* TxBuf_Ctrl Registers bits */
#define TXBUF_PACKET_SIZE_SHIFT 16
#define TXBUF_PACKET_SIZE_MASK GENMASK(30, 16)
#define TX_START BIT(0)
#define BUFF_TX_BUSY BIT(1)
#define MUTE_ENABLE BIT(2)

/* RxBuf_Ctrl Registers bits */
#define RXBUF_LAST_ADDRESS_SHIFT 16
#define RXBUF_LAST_ADDRESS_MASK GENMASK(30, 16)
#define RXBUF_BUFFER_DONE_MASK BIT(9)

/* Error bits in RxBuf_Ctrl */
#define COMCTRL_RXBUF_OVERRUN_ERROR BIT(0)
#define P2PUART_FRAME_ERROR BIT(1)
#define P2PUART_BREAK_ERROR BIT(2)
#define P2PUART_PARITY_ERROR BIT(3)

#define P2PUART_RXBUF_ALL_ERRORS_MASK                                          \
	(P2PUART_FRAME_ERROR | P2PUART_BREAK_ERROR | P2PUART_PARITY_ERROR |    \
	 COMCTRL_RXBUF_OVERRUN_ERROR)

/* IPR bits */
#define IPR_RX_PACKET_SIZE_DONE0 BIT(0)
#define IPR_RX_PACKET_SIZE_DONE1 BIT(1)
#define IPR_RX_LINK_DONE0 BIT(2)
#define IPR_RX_LINK_DONE1 BIT(3)
#define IPR_OVERRUN_ERROR0 BIT(4)
#define IPR_OVERRUN_ERROR1 BIT(5)
#define IPR_WINDOW_TIMEOUT_ERROR0 BIT(6)
#define IPR_WINDOW_TIMEOUT_ERROR1 BIT(7)
#define IPR_LAST_BUFFER BIT(15)
#define IPR_TX_DONE0 BIT(16)
#define IPR_TX_DONE1 BIT(17)

#define IPR_RX0_MASK                                                           \
	(IPR_RX_PACKET_SIZE_DONE0 | IPR_RX_LINK_DONE0 | IPR_OVERRUN_ERROR0 |   \
	 IPR_WINDOW_TIMEOUT_ERROR0)

#define IPR_RX1_MASK                                                           \
	(IPR_RX_PACKET_SIZE_DONE1 | IPR_RX_LINK_DONE1 | IPR_OVERRUN_ERROR1 |   \
	 IPR_WINDOW_TIMEOUT_ERROR1)

#define IPR_RX_MASK (IPR_RX0_MASK | IPR_RX1_MASK)
#define IPR_TX_MASK (IPR_TX_DONE0 | IPR_TX_DONE1)
#define IPR_WINDOW_TIMEOUT_MASK GENMASK(7, 6)
#define IPR_PHY_MASK GENMASK(31, 24)

#define IPR_ALL_MASK                                                           \
	(IPR_RX_MASK | IPR_TX_MASK | IPR_PHY_MASK | IPR_WINDOW_TIMEOUT_MASK)

#define IMR_RX_MASK IPR_RX_MASK
#define IMR_TX_MASK IPR_TX_MASK
#define IMR_WINDOW_TIMEOUT_MASK IPR_WINDOW_TIMEOUT_MASK
#define IMR_PHY_MASK IPR_PHY_MASK
#define IMR_ALL_MASK IPR_ALL_MASK

/* CPR bits */
#define CPR_STREAMING_MODE_ENABLE_MASK BIT(6)
#define CPR_END_OF_PACKET_CONDITION_TRIGGER_SHIFT 4
#define CPR_END_OF_PACKET_CONDITION_TRIGGER_MASK GENMASK(5, 4)
#define CPR_NO_RECEIVE_WHILE_TRANSMIT_MASK BIT(2)
#define CPR_LOOPBACK_MASK GENMASK(1, 0)
#define CPR_RX_PACKET_SIZE_SHIFT 16
#define CPR_RX_PACKET_SIZE_MASK GENMASK(30, 16)

/* Loopback modes */
#define LOOPBACK_NONE 0x0
#define LOOPBACK_COMCTRL 0x1
#define LOOPBACK_ENDPOINT 0x2
#define LOOPBACK_EXTERNAL 0x3

/* End of Packet conditions */
#define CPR_EOP_LINK_DONE 0x0
#define CPR_EOP_PACKET_SIZE 0x1
#define CPR_EOP_BOTH 0x2

/* ======================================================================== */
/* P2P Uart registers */
/* ======================================================================== */

/* Baud Rate Register */
#define P2P_BAUD_RATE 0x64

/* Specific Channel Properties Register */
#define P2P_SCP 0x60

/* Specific Capabilities Register */
#define P2P_SCAP 0x80

#define BAUD_RATE_D_MASK GENMASK(15, 0)
#define BAUD_RATE_N_SHIFT 16
#define BAUD_RATE_N_MASK GENMASK(31, 16)

#define SCP_TX_PARITY_MASK GENMASK(2, 0)
#define SCP_RX_PARITY_SHIFT 4
#define SCP_RX_PARITY_MASK GENMASK(6, 4)
#define SCP_TX_NUMBER_OF_STOP_BITS_SHIFT 8
#define SCP_TX_NUMBER_OF_STOP_BITS_MASK GENMASK(9, 8)

#define SCP_1_STOP_BIT ((0x0 << SCP_TX_NUMBER_OF_STOP_BITS_SHIFT) & SCP_TX_NUMBER_OF_STOP_BITS_MASK)
#define SCP_2_STOP_BITS ((0x2 << SCP_TX_NUMBER_OF_STOP_BITS_SHIFT) & SCP_TX_NUMBER_OF_STOP_BITS_MASK)

#define SCP_PARITY_EVEN 0x0
#define SCP_PARITY_ODD 0x1
#define SCP_PARITY_NONE 0x2
#define SCP_PARITY_MARK 0x3
#define SCP_PARITY_SPACE 0x4
#define SCP_PARITY_WAKEUP 0x5

#define SCP_TRANSCEIVER_TERMINATION_MASK BIT(12)
#define SCP_CHARACTER_TIMEOUT_PERIOD_SHIFT 16
#define SCP_CHARACTER_TIMEOUT_PERIOD_MASK GENMASK(23, 16)

#define SCAP_MAXIMUM_BAUD_RATE_KBPS_MASK GENMASK(15, 0)
#define SCAP_FULL_HALF_DUPLEX_SHIFT 20
#define SCAP_FULL_HALF_DUPLEX_MASK GENMASK(21, 20)
#define SCAP_ALWAYS_HALF_DUPLEX 0x0
#define SCAP_TRANSCEIVER_TERMINATION_SHIFT 24
#define SCAP_TRANSCEIVER_TERMINATION_MASK GENMASK(25, 24)
#define SCAP_RX_CHANNEL_EXISTS BIT(28)
#define SCAP_TX_CHANNEL_EXISTS BIT(29)

/* ======================================================================== */
/* General definitions                                                      */
/* ======================================================================== */

#define RX_BUFFER_0 0
#define RX_BUFFER_1 1
#define RX_BUFFERS_0_AND_1 2

#define MAX_BAUD_RATE_ERROR_PERCENTS 3

#define P2PUART_LINUX_PORT_TYPE PORT_16550A
#define P2PUART_PORT_TYPE_STRING "16550A"

#define P2PUART_DEFAULT_CHARACTER_TIMEOUT 3

/* Transceiver termination */
#define TRANSCEIVER_TERMINATION_NONE 0x0
#define TRANSCEIVER_TERMINATION_CONSTANT 0x1
#define TRANSCEIVER_TERMINATION_CONTROLLABLE 0x2

#define P2PUART_TX_START_MAX_RETRIES 32

/* ======================================================================== */
/* Types                                                                    */
/* ======================================================================== */

struct p2puart_port {
	struct uart_port port;
	char name[16];
	bool rx_channel_exists;
	bool tx_channel_exists;
	unsigned int maximum_baud_rate_khz;
	unsigned int full_half_duplex_capability;
	unsigned int transceiver_termination_capability;
	unsigned int next_tx_buffer;
	unsigned int next_rx_buffer;
	unsigned int rx_buffer_size;
	unsigned int rx_buffer_offset[2];
	unsigned int tx_buffer_size;
	unsigned int tx_buffer_offset[2];
	bool mute_enabled;
	unsigned int loopback;
	bool no_receive_while_transmit;
	bool streaming;
	unsigned int parity;
	unsigned int wakeup_parity_mode;
	unsigned int number_of_stop_bits;
	unsigned int termination_enable;
	unsigned int character_timeout_period;
	bool rx_enabled;
	unsigned int previous_packet_size_last_address[2];
	bool throttle_state;
	unsigned int rx_packet_size;
	unsigned long requested_baud_rate;
	unsigned long calculated_baud_rate;
	bool tx_buffer_busy[2];
	unsigned char *temp_rx_buffer;
};

/* ======================================================================== */
/* Static variables                                                         */
/* ======================================================================== */

static struct p2puart_port *p2puart_ports[P2PUART_NR_MAX];
static int p2puart_ports_num;

/* ======================================================================== */
/* Forward declarations                                                     */
/* ======================================================================== */

static void p2puart_update_scp(struct uart_port *port);
static struct p2puart_port *to_p2puart_port(struct uart_port *port);

/* ======================================================================== */
/* Helper Functions                                                         */
/* ======================================================================== */

static struct p2puart_port *to_p2puart_port(struct uart_port *port)
{
	return container_of(port, struct p2puart_port, port);
}

static uint32_t p2puart_serial_in(struct uart_port *port, int offset)
{
	uint32_t regval;

	PRINT_FUNCTION_ENTRY("p2puart_serial_in");

	regval = ioread32(port->membase + offset);

#ifdef DEBUG_PRINT_REGISTERS_ACCESS
	printk("%s: read 0x%08llx (value:0x%08llx)\n",
	       to_p2puart_port(port)->name,
	       (unsigned long long)(port->mapbase + offset),
	       (unsigned long long)regval);
#endif

	return regval;
}

static void p2puart_serial_out(struct uart_port *port, int offset, uint32_t value)
{
	PRINT_FUNCTION_ENTRY("p2puart_serial_out");

#ifdef DEBUG_PRINT_REGISTERS_ACCESS
	printk("%s: write 0x%08llx 0x%08llx\n", to_p2puart_port(port)->name,
	       (unsigned long long)(port->mapbase + offset),
	       (unsigned long long)value);
#endif
	iowrite32(value, port->membase + offset);
	/* Read back to ensure write is flushed */
	ioread32(port->membase + offset);
}

static void p2puart_unmask_all_interrupts(struct uart_port *port)
{
	p2puart_serial_out(port, COMCTRL_IMR, 0x00000000);
}

static unsigned int p2puart_get_tx_ctrl_register_offset(unsigned int buffer_number)
{
	return buffer_number ? TXBUF1_CTRL : TXBUF0_CTRL;
}

static void p2puart_raise_reset_tx(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_raise_reset_tx");
	p2puart_serial_out(port, SOFT_RESET, RESET_TX);
	wmb(); /* Ensure write completes */
	udelay(1);
}

static void p2puart_raise_reset_rx_and_tx(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_raise_reset_rx_and_tx");
	p2puart_serial_out(port, SOFT_RESET, RESET_RX | RESET_TX);
	wmb();
	udelay(1);
	to_p2puart_port(port)->next_rx_buffer = 0;
}

static void p2puart_lower_reset_tx(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_lower_reset_tx");
	p2puart_serial_out(port, SOFT_RESET, RESET_TX_NORMAL_OPERATION);
	wmb();
	udelay(1);
}

static void p2puart_lower_reset_rx_and_tx(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_lower_reset_rx_and_tx");
	p2puart_serial_out(port, SOFT_RESET,
			   RESET_RX_NORMAL_OPERATION | RESET_TX_NORMAL_OPERATION);
	wmb();
	udelay(1);
}

static unsigned int p2puart_get_clock_frequency(struct uart_port *port)
{
	uint32_t mpr;
	unsigned int freq_khz;

	PRINT_FUNCTION_ENTRY("p2puart_get_clock_frequency");

	mpr = p2puart_serial_in(port, COMCTRL_MPR);
	freq_khz = mpr & COMCTRL_CLK_FREQ_KHZ_MASK;
	return freq_khz * 1000;
}

static uint32_t p2puart_cto_raw_to_physical(uint8_t character_time_out)
{
	return (0 == character_time_out) ? 256 : (uint32_t)character_time_out;
}

static uint8_t p2puart_cto_physical_to_raw(uint32_t character_time_out)
{
	return (256 == character_time_out) ? 0 : (uint8_t)character_time_out;
}

static unsigned int p2puart_get_character_timeout_period(struct uart_port *port)
{
	uint32_t scp;
	unsigned int timeout_period;

	PRINT_FUNCTION_ENTRY("p2puart_get_character_timeout_period");

	scp = p2puart_serial_in(port, P2P_SCP);
	timeout_period = ((scp & SCP_CHARACTER_TIMEOUT_PERIOD_MASK) >>
			  SCP_CHARACTER_TIMEOUT_PERIOD_SHIFT);
	timeout_period = p2puart_cto_raw_to_physical(timeout_period);
	return timeout_period;
}

static unsigned int p2puart_get_tx_buffer_size(struct uart_port *port)
{
	uint32_t mpr;
	unsigned int tx_dpr_size_kb;

	PRINT_FUNCTION_ENTRY("p2puart_get_tx_buffer_size");

	mpr = p2puart_serial_in(port, COMCTRL_MPR);
	tx_dpr_size_kb = ((mpr & COMCTRL_TX_DPR_SIZE_MASK) >> COMCTRL_TX_DPR_SIZE_SHIFT);

	if (!tx_dpr_size_kb)
		tx_dpr_size_kb = 64;

	return (tx_dpr_size_kb * 1024) / 2;
}

static unsigned int p2puart_get_rx_buffer_size(struct uart_port *port)
{
	uint32_t mpr;
	unsigned int rx_dpr_size_kb;

	PRINT_FUNCTION_ENTRY("p2puart_get_rx_buffer_size");

	mpr = p2puart_serial_in(port, COMCTRL_MPR);
	rx_dpr_size_kb = ((mpr & COMCTRL_RX_DPR_SIZE_MASK) >> COMCTRL_RX_DPR_SIZE_SHIFT);
	
	if (!rx_dpr_size_kb)
		rx_dpr_size_kb = 64;

	return (rx_dpr_size_kb * 1024) / 2;
}

static bool p2puart_is_tx_buffer_busy(struct uart_port *port, unsigned int buffer_number)
{
	unsigned int tx_ctrl_register_offset;
	uint32_t regval;

	PRINT_FUNCTION_ENTRY("p2puart_is_tx_buffer_busy");

	tx_ctrl_register_offset = p2puart_get_tx_ctrl_register_offset(buffer_number);
	regval = p2puart_serial_in(port, tx_ctrl_register_offset);
	return !!(regval & BUFF_TX_BUSY);
}

/* ======================================================================== */
/* Baud Rate Calculation                                                    */
/* ======================================================================== */

static unsigned long p2puart_calc_baud_divs(unsigned long clock_freq_hz,
					    unsigned long baud, uint16_t *n,
					    uint16_t *d)
{
	unsigned long baud_n;
	unsigned long baud_d;
	unsigned long largest;
	unsigned long cur_baud;
	unsigned long calc_baud;
	unsigned long baud_error;
	unsigned long min_baud_error;
	unsigned long best_n;
	unsigned long best_d;

	min_baud_error = baud;
	best_n = 0;
	best_d = 0;

	if ((8 * baud) > clock_freq_hz) {
		*n = 0;
		*d = 0;
		return 0;
	}

	largest = (1 << 16);

	for (baud_n = 1; baud_n <= largest; ++baud_n) {
		baud_d = DIV_ROUND_CLOSEST_ULL(
			(unsigned long long)baud * 8 * baud_n, clock_freq_hz);
		if (0 == baud_d)
			continue;
		cur_baud = div_u64(((unsigned long long)clock_freq_hz * baud_d),
				   (8 * baud_n));

		baud_error = (baud > cur_baud ? baud - cur_baud : cur_baud - baud);
		if (baud_error < min_baud_error) {
			min_baud_error = baud_error;
			best_n = baud_n;
			best_d = baud_d;
		}
	}

	baud_n = best_n;
	baud_d = best_d;

	if (baud_n == largest)
		baud_n = 0;
	if (baud_d == largest)
		baud_d = 0;

	*n = baud_n;
	*d = baud_d;

	if (0 == baud_n)
		baud_n = largest;
	if (0 == baud_d)
		baud_d = largest;
	calc_baud = (unsigned long)div_u64(
		((unsigned long long)clock_freq_hz * baud_d), (8 * baud_n));

	return calc_baud;
}

/* ======================================================================== */
/* UART Operations                                                          */
/* ======================================================================== */

static unsigned int p2puart_tx_empty(struct uart_port *port)
{
	struct p2puart_port *pp = to_p2puart_port(port);
	unsigned long flags;
	bool busy;

	PRINT_FUNCTION_ENTRY("p2puart_tx_empty");

	spin_lock_irqsave(&port->lock, flags);
	busy = pp->tx_buffer_busy[0] || pp->tx_buffer_busy[1];
	spin_unlock_irqrestore(&port->lock, flags);

	return busy ? 0 : TIOCSER_TEMT;
}

static unsigned int p2puart_get_mctrl(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_get_mctrl");
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void p2puart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	PRINT_FUNCTION_ENTRY("p2puart_set_mctrl");
	/* Nothing to do */
}

static void p2puart_stop_tx(struct uart_port *port)
{
	uint32_t imr;
	uint32_t ipr;
	struct p2puart_port *pp = to_p2puart_port(port);

	PRINT_FUNCTION_ENTRY("p2puart_stop_tx");

	/* Save IMR value */
	imr = p2puart_serial_in(port, COMCTRL_IMR);

	/* Reset tx to stop transmission immediately */
	wmb();
	p2puart_raise_reset_tx(port);
	wmb();
	p2puart_lower_reset_tx(port);
	wmb();

	/* Read and clear TX done bits in IPR */
	ipr = p2puart_serial_in(port, COMCTRL_IPR);
	if (ipr & (IPR_TX_DONE0 | IPR_TX_DONE1)) {
		ipr &= (IPR_TX_DONE0 | IPR_TX_DONE1);
		p2puart_serial_out(port, COMCTRL_IPR, ipr);
	}

	/* Restore IMR */
	p2puart_serial_out(port, COMCTRL_IMR, imr);

	pp->next_tx_buffer = 0;
	pp->tx_buffer_busy[0] = false;
	pp->tx_buffer_busy[1] = false;
}

/**
 * FIXED: TX_START handshake implementation
 * Follows ComCtrl spec section 6.1.1:
 * 1. Poll Buff_Tx_Busy until cleared
 * 2. Assert Tx_Start and set TxBuf_Packet_Size
 * 3. Poll Buff_Tx_Busy: if asserted â€" done, else retry from step 2
 */
static bool p2puart_issue_tx_start_command(struct uart_port *port,
                                           int buffer_number,
                                           u32 packet_size)
{
	struct p2puart_port *pp = to_p2puart_port(port);
	unsigned int tx_ctrl_off = p2puart_get_tx_ctrl_register_offset(buffer_number);
	u32 ctrl = 0;
	int attempt;

	/* Build control value */
	if (pp->mute_enabled)
		ctrl |= MUTE_ENABLE;

	ctrl |= (packet_size << TXBUF_PACKET_SIZE_SHIFT) & TXBUF_PACKET_SIZE_MASK;

	/* Step 1: Wait for buffer to be idle */
	for (attempt = 0; attempt < P2PUART_TX_START_MAX_RETRIES; ++attempt) {
		u32 reg = p2puart_serial_in(port, tx_ctrl_off);
		if (!(reg & BUFF_TX_BUSY))
			break;
		cpu_relax();
	}

	/* Still busy - abort */
	if (p2puart_serial_in(port, tx_ctrl_off) & BUFF_TX_BUSY) {
		dev_warn(pp->port.dev,
		         "TX buf %d still busy, cannot start packet (size=%u)\n",
		         buffer_number, packet_size);
		return false;
	}

	/* Steps 2 + 3: Issue TX_START and verify BUFF_TX_BUSY asserts */
	for (attempt = 0; attempt < P2PUART_TX_START_MAX_RETRIES; ++attempt) {
		u32 reg;

		/* Step 2: Write TX_START command */
		p2puart_serial_out(port, tx_ctrl_off, ctrl | TX_START);
		wmb(); /* Ensure write completes */

		/* Step 3: Check if BUFF_TX_BUSY went high */
		reg = p2puart_serial_in(port, tx_ctrl_off);
		
		if (reg & BUFF_TX_BUSY) {
			pp->tx_buffer_busy[buffer_number] = true;
			return true;
		}

		cpu_relax();
	}

	/* TX_START handshake failed */
	dev_err(pp->port.dev,
	        "TX_START handshake failed on buf %d (size=%u)\n",
	        buffer_number, packet_size);
	return false;
}

static void p2puart_transmit_xchar(struct uart_port *port, char x_char)
{
	struct p2puart_port *pp = to_p2puart_port(port);
	unsigned int current_tx_buffer;
	unsigned int buffer_offset;
	uint32_t x;

	current_tx_buffer = pp->next_tx_buffer;
	buffer_offset = pp->tx_buffer_offset[current_tx_buffer];

	x = 0;
	*(uint8_t *)(&x) = x_char;

	/* Write character to buffer */
	p2puart_serial_out(port, buffer_offset, x);
	port->icount.tx++;
	port->x_char = 0;

	wmb();

	/* Issue TX_START */
	if (p2puart_issue_tx_start_command(port, current_tx_buffer, 1)) {
		pp->next_tx_buffer = !current_tx_buffer;
	}
}

static unsigned int p2puart_copy_chars_from_tty_to_tx_buffer(struct uart_port *port)
{
	struct p2puart_port *pp = to_p2puart_port(port);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int current_tx_buffer;
	unsigned int buffer_offset;
	unsigned int write_offset;
	unsigned int tx_buffer_size;

	current_tx_buffer = pp->next_tx_buffer;
	buffer_offset = pp->tx_buffer_offset[current_tx_buffer];
	tx_buffer_size = pp->tx_buffer_size;

	write_offset = 0;
	while (uart_circ_chars_pending(xmit) && (write_offset < tx_buffer_size)) {
		uint32_t i;
		uint32_t remaining = tx_buffer_size - write_offset;
		i = min(remaining, (uint32_t)uart_circ_chars_pending(xmit));
		i = min(i, (uint32_t)CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));
		
		PRINT_TX_BUFFER(port, &xmit->buf[xmit->tail], i);
		
		memcpy_toio(port->membase + buffer_offset + write_offset,
		            &xmit->buf[xmit->tail], i);
		xmit->tail = (xmit->tail + i) & (UART_XMIT_SIZE - 1);
		write_offset += i;
	}

	port->icount.tx += write_offset;
	return write_offset;
}

static void p2puart_handle_tx(struct uart_port *port)
{
    struct p2puart_port *pp = to_p2puart_port(port);
    struct circ_buf *xmit = &port->state->xmit;
    unsigned int current_tx_buffer;
    unsigned int chars_to_copy;

    if (uart_tx_stopped(port) || uart_circ_empty(xmit))
        return;

    current_tx_buffer = pp->next_tx_buffer;

    /* Check current buffer */
    if (p2puart_is_tx_buffer_busy(port, current_tx_buffer)) {
        return;
    }

    /* ADDED: Check if OTHER buffer also busy - we're overloaded! */
    if (p2puart_is_tx_buffer_busy(port, !current_tx_buffer)) {
        dev_warn_ratelimited(pp->port.dev,
            "TX overload: both buffers busy, data rate too high for baud rate\n");
        /* This means your baud rate is too low for the data rate! */
    }

    chars_to_copy = p2puart_copy_chars_from_tty_to_tx_buffer(port);
    if (!chars_to_copy)
        return;

    wmb();

    if (p2puart_issue_tx_start_command(port, current_tx_buffer, chars_to_copy)) {
        pp->next_tx_buffer = !current_tx_buffer;
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

static void p2puart_start_tx(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_start_tx");

	if (uart_tx_stopped(port))
		return;

	if (uart_circ_empty(&port->state->xmit))
		return;

	p2puart_handle_tx(port);
}

static void p2puart_stop_rx(struct uart_port *port)
{
	struct p2puart_port *pp = to_p2puart_port(port);

	PRINT_FUNCTION_ENTRY("p2puart_stop_rx");

	pp->rx_enabled = false;
}

static void p2puart_break_ctl(struct uart_port *port, int break_state)
{
	PRINT_FUNCTION_ENTRY("p2puart_break_ctl");
	/* Not supported */
}

/* ======================================================================== */
/* FIXED: RX Processing Functions                                           */
/* ======================================================================== */

static unsigned int p2puart_get_active_rx_buffer(uint32_t ipr)
{
	bool buf0 = (ipr & IPR_RX0_MASK) != 0;
	bool buf1 = (ipr & IPR_RX1_MASK) != 0;

	if (buf0 && buf1)
		return RX_BUFFERS_0_AND_1;
	if (buf0)
		return RX_BUFFER_0;
	if (buf1)
		return RX_BUFFER_1;

	return RX_BUFFER_0;
}

static unsigned int p2puart_get_last_address(uint32_t rxbuf_ctrl)
{
	return ((rxbuf_ctrl & RXBUF_LAST_ADDRESS_MASK) >> RXBUF_LAST_ADDRESS_SHIFT);
}

static uint32_t p2puart_get_rxbuf_ctrl(struct uart_port *port, unsigned int buffer_number)
{
	unsigned int rxbuf_ctrl_address;

	PRINT_FUNCTION_ENTRY("p2puart_get_rxbuf_ctrl");

	rxbuf_ctrl_address = (buffer_number == 0) ? RXBUF0_CTRL : RXBUF1_CTRL;
	return p2puart_serial_in(port, rxbuf_ctrl_address);
}

static char p2puart_get_rx_flag_and_update_error_counters(
	struct uart_port *port, uint32_t rxbuf_ctrl, uint32_t status)
{
	char flag = TTY_NORMAL;

	if (unlikely(status)) {
		if (status & COMCTRL_RXBUF_OVERRUN_ERROR) {
			flag = TTY_OVERRUN;
			port->icount.overrun++;
		} else if (status & P2PUART_PARITY_ERROR) {
			flag = TTY_PARITY;
			port->icount.parity += (p2puart_get_last_address(rxbuf_ctrl) + 1);
		} else if (status & P2PUART_FRAME_ERROR) {
			flag = TTY_FRAME;
			port->icount.frame += (p2puart_get_last_address(rxbuf_ctrl) + 1);
		}
	}

	return flag;
}

static void p2puart_copy_rx_data_to_tty_buffer(struct uart_port *port,
					       unsigned int buffer_number,
					       unsigned int last_address,
					       char flag)
{
	struct p2puart_port *pp = to_p2puart_port(port);
	unsigned int bytes_to_copy;
	uint32_t src;

	bytes_to_copy = last_address + 1;
	src = pp->rx_buffer_offset[buffer_number];

	if (likely(flag == TTY_NORMAL)) {
		/* Fast path - direct copy */
		unsigned char *chars;
		int space;
		
		space = tty_prepare_flip_string(&port->state->port, &chars, bytes_to_copy);
		if (space < bytes_to_copy) {
			dev_warn(pp->port.dev,
			         "Not enough TTY buffer space: %d of %u bytes lost\n",
			         (int)bytes_to_copy - space, bytes_to_copy);
			bytes_to_copy = space;
		}
		memcpy_fromio(chars, port->membase + src, bytes_to_copy);
		PRINT_RX_BUFFER(port, chars, bytes_to_copy);
	} else {
		/* Slow path - with error flag */
		memcpy_fromio(pp->temp_rx_buffer, port->membase + src, bytes_to_copy);
		PRINT_RX_BUFFER(port, pp->temp_rx_buffer, bytes_to_copy);
		tty_insert_flip_string_fixed_flag(&port->state->port,
		                                  pp->temp_rx_buffer, flag,
		                                  bytes_to_copy);
	}

	port->icount.rx += bytes_to_copy;
	tty_flip_buffer_push(&port->state->port);
}

static void p2puart_read_from_buffer(struct uart_port *port,
                                     unsigned int buffer_number, 
                                     uint32_t ipr,
                                     uint32_t rxbuf_ctrl)
{
	struct p2puart_port *pp;
	uint32_t status;
	char flag;
	unsigned int last_address;
	unsigned int bytes_to_read;
	bool is_packet_size_done;
	bool is_link_done;

	pp = to_p2puart_port(port);

	/* ================================================================
	 * STEP 1: Determine which interrupt type triggered this call
	 * ================================================================ */
	
	is_packet_size_done = !!(ipr & ((buffer_number == 0) ? 
	                         IPR_RX_PACKET_SIZE_DONE0 : 
	                         IPR_RX_PACKET_SIZE_DONE1));
	
	is_link_done = !!(ipr & ((buffer_number == 0) ? 
	                  IPR_RX_LINK_DONE0 : 
	                  IPR_RX_LINK_DONE1));

	/* ================================================================
	 * STEP 2: Extract Last_Address from RxBuf_Ctrl
	 * ================================================================ */
	
	last_address = p2puart_get_last_address(rxbuf_ctrl);

	/* ================================================================
	 * STEP 3: Calculate bytes to read based on interrupt type
	 * 
	 * PACKET_SIZE_DONE:
	 *   - Buffer filled to configured packet size
	 *   - Last_Address points to the last byte written (0-based)
	 *   - bytes_to_read = Last_Address + 1 (entire packet)
	 *   - Update tracking for next Link_Done
	 * 
	 * LINK_DONE:
	 *   - End-of-packet signaled by endpoint or timeout
	 *   - Last_Address might be incremental from previous Packet_Size
	 *   - Calculate delta from previous_packet_size_last_address
	 *   - If Last_Address < previous, assume buffer wrapped or first packet
	 * 
	 * ================================================================ */
	
	if (is_packet_size_done) {
		/* 
		 * Packet Size Done: 
		 * Buffer filled to configured Rx_Packet_Size.
		 * Last_Address is valid and shows the complete packet.
		 */
		bytes_to_read = last_address + 1;
		
		/* Update tracking for next Link_Done interrupt */
		pp->previous_packet_size_last_address[buffer_number] = last_address;
		
#ifdef DEBUG_PRINT_RX_INTERRUPT
		dev_dbg(pp->port.dev,
		        "PACKET_SIZE_DONE buf=%u last_addr=%u bytes=%u\n",
		        buffer_number, last_address, bytes_to_read);
#endif
	} 
	else if (is_link_done) {
		/* 
		 * Link Done:
		 * End-of-packet condition from endpoint or character timeout.
		 * Need to calculate bytes since last Packet_Size event.
		 */
		
		if (last_address >= pp->previous_packet_size_last_address[buffer_number]) {
			/*
			 * Normal case: Last_Address advanced since last Packet_Size.
			 * Read the delta (incremental bytes since last read).
			 * 
			 * Example:
			 *   Previous Packet_Size: Last_Address = 1023
			 *   Current Link_Done:    Last_Address = 1050
			 *   Bytes to read: 1050 - 1023 = 27 bytes
			 */
			bytes_to_read = last_address - pp->previous_packet_size_last_address[buffer_number];
		} else {
			/*
			 * Wrapped or first packet:
			 * Last_Address is less than previous, could be:
			 * 1. First packet after reset (previous = buffer_size)
			 * 2. Buffer wrapped (unlikely with double buffering)
			 * 3. Short packet at start of buffer
			 * 
			 * Safe approach: Read from 0 to Last_Address
			 */
			bytes_to_read = last_address + 1;
		}
		
		/* Update tracking for next interrupt */
		pp->previous_packet_size_last_address[buffer_number] = last_address;
		
#ifdef DEBUG_PRINT_RX_INTERRUPT
		dev_dbg(pp->port.dev,
		        "LINK_DONE buf=%u last_addr=%u bytes=%u\n",
		        buffer_number, last_address, bytes_to_read);
#endif
	}
	else {
		/*
		 * Neither PACKET_SIZE_DONE nor LINK_DONE is set.
		 * This should not happen if interrupts are configured correctly.
		 * Log warning and abort.
		 */
		dev_warn_ratelimited(pp->port.dev,
		    "RX interrupt without PACKET_SIZE_DONE or LINK_DONE "
		    "(buf=%u, ipr=0x%08x, rxbuf_ctrl=0x%08x)\n",
		    buffer_number, ipr, rxbuf_ctrl);
		return;
	}

	/* ================================================================
	 * STEP 4: Sanity checks on calculated byte count
	 * ================================================================ */
	
	if (bytes_to_read > pp->rx_buffer_size) {
		dev_err(pp->port.dev,
		    "Invalid byte count: %u exceeds buffer size %u "
		    "(buf=%u, last_addr=%u, ipr=0x%08x)\n",
		    bytes_to_read, pp->rx_buffer_size, 
		    buffer_number, last_address, ipr);
		return;
	}

	if (bytes_to_read == 0) {
		/* 
		 * Zero bytes to read - can happen with Link_Done if 
		 * Last_Address hasn't advanced since last Packet_Size.
		 * This is valid (empty packet or duplicate interrupt).
		 */
#ifdef DEBUG_PRINT_RX_INTERRUPT
		dev_dbg(pp->port.dev,
		        "Zero bytes to read (buf=%u, last_addr=%u)\n",
		        buffer_number, last_address);
#endif
		return;
	}

	/* ================================================================
	 * STEP 5: Check for errors in received data
	 * ================================================================ */
	
	status = rxbuf_ctrl & P2PUART_RXBUF_ALL_ERRORS_MASK;
	status &= port->read_status_mask;

	flag = p2puart_get_rx_flag_and_update_error_counters(port, rxbuf_ctrl, status);

	/* ================================================================
	 * STEP 6: Handle overrun error specially
	 * 
	 * Overrun means both RX buffers were full when new data arrived.
	 * Data was lost. Insert overrun character to TTY and return.
	 * ================================================================ */
	
	if (unlikely(TTY_OVERRUN == flag)) {
		tty_insert_flip_char(&port->state->port, 0, TTY_OVERRUN);
		tty_flip_buffer_push(&port->state->port);
		
		dev_warn_ratelimited(pp->port.dev,
		    "RX buffer overrun detected (buf=%u) - data lost\n",
		    buffer_number);
		return;
	}

	/* ================================================================
	 * STEP 7: Copy data to TTY if not filtered by ignore_status_mask
	 * ================================================================ */
	
	if (likely(!(status & port->ignore_status_mask)) && likely(pp->rx_enabled)) {
		p2puart_copy_rx_data_to_tty_buffer(port, buffer_number, 
		                                   bytes_to_read, flag);
	} else {
		/*
		 * Data filtered out by ignore_status_mask or RX disabled.
		 * Count the bytes but don't pass them to TTY.
		 */
		port->icount.rx += bytes_to_read;
		
#ifdef DEBUG_PRINT_RX_INTERRUPT
		dev_dbg(pp->port.dev,
		        "RX data filtered: %u bytes (status=0x%x, rx_enabled=%d)\n",
		        bytes_to_read, status, pp->rx_enabled);
#endif
	}
}


/**
 * p2puart_copy_rx_data_to_tty_buffer - Copy data from DPR to TTY layer
 * 
 * @port: Pointer to uart_port structure
 * @buffer_number: Which buffer to read from (0 or 1)
 * @bytes_to_read: Number of bytes to copy
 * @flag: TTY flag for error indication (TTY_NORMAL, TTY_PARITY, etc.)
 * 
 * Fast path (TTY_NORMAL): Direct memcpy_fromio to TTY buffer
 * Slow path (errors): Copy via temp buffer with error flag
 */



/**
 * p2puart_get_rx_flag_and_update_error_counters - Determine TTY flag from errors
 * 
 * @port: Pointer to uart_port structure
 * @rxbuf_ctrl: Snapshot of RxBuf_Ctrl register
 * @status: Filtered error status (after read_status_mask applied)
 * 
 * Returns: TTY flag for the received data
 *   TTY_NORMAL  - No errors
 *   TTY_OVERRUN - Buffer overrun (data lost)
 *   TTY_PARITY  - Parity error
 *   TTY_FRAME   - Framing error
 * 
 * Updates port->icount error counters.
 */



/**
 * FIXED: Buffer clearing function
 * Properly clears Buffer_Done and error bits in correct order
 */
static void p2puart_clear_buffer_done_bit_and_errors(struct uart_port *port,
                                                     unsigned int buffer_number,
                                                     uint32_t rxbuf_ctrl_snapshot)  // ← ADD THIS
{
	u32 reg = (buffer_number == 0) ? RXBUF0_CTRL : RXBUF1_CTRL;
	u32 clear_mask = 0;

	/* CRITICAL: Only clear error bits that are ACTUALLY SET */
	clear_mask |= (rxbuf_ctrl_snapshot & 0x000000FF);

	/* Clear Buffer_Done if it's set */
	if (rxbuf_ctrl_snapshot & RXBUF_BUFFER_DONE_MASK)
		clear_mask |= RXBUF_BUFFER_DONE_MASK;

	if (clear_mask) {
		p2puart_serial_out(port, reg, clear_mask);
		wmb();
		(void)p2puart_serial_in(port, reg);
	}
}

/**
 * FIXED: Single buffer RX processing
 * Returns the IPR bits that should be acknowledged
 */
static uint32_t p2puart_rx_single_buffer(struct uart_port *port,  // ← uint32_t, not void
                                         unsigned int buffer_number, 
                                         uint32_t ipr) 
{
    struct p2puart_port *pp = to_p2puart_port(port);
    uint32_t rxbuf_ctrl_snapshot;  // ← Renamed for clarity
    uint32_t this_buf_mask;
    uint32_t ack_mask = 0;

    this_buf_mask = (buffer_number == 0) ? IPR_RX0_MASK : IPR_RX1_MASK;

    /* CRITICAL: Snapshot RxBuf_Ctrl ONCE at start */
    rxbuf_ctrl_snapshot = p2puart_get_rxbuf_ctrl(port, buffer_number);

    /* Read data into TTY */
    p2puart_read_from_buffer(port, buffer_number, ipr, rxbuf_ctrl_snapshot);

    /* If throttled, HOLD the buffer - don't clear Buffer_Done */
    if (pp->throttle_state) {
        return 0; /* Don't acknowledge IPR */
    }

    /* Clear Buffer_Done and errors - releases buffer to HW */
    p2puart_clear_buffer_done_bit_and_errors(port, buffer_number, rxbuf_ctrl_snapshot);

    /* Return which IPR bits to acknowledge */
    ack_mask = ipr & this_buf_mask;
    return ack_mask;  // ← Must return the mask
}

/**
 * FIXED: Main RX handler
 * Processes RX interrupts in correct order and returns IPR bits to acknowledge
 */
static uint32_t p2puart_handle_rx(struct uart_port *port, uint32_t ipr)  // ← Returns uint32_t
{
    unsigned int active_rx_buffer;
    unsigned int first_buf, second_buf;
    struct p2puart_port *pp;
    uint32_t processed_mask = 0;  // ← Accumulate masks

    pp = to_p2puart_port(port);

    if (unlikely(!pp->rx_channel_exists))
        return 0;

    active_rx_buffer = p2puart_get_active_rx_buffer(ipr);

    if (active_rx_buffer != RX_BUFFERS_0_AND_1) {
        first_buf = active_rx_buffer;
        pp->next_rx_buffer = 1 - first_buf;
        processed_mask |= p2puart_rx_single_buffer(port, first_buf, ipr);  // ← Collect mask
    } else {
        bool last_is_buf1 = !!(ipr & IPR_LAST_BUFFER);

        if (last_is_buf1) {
            first_buf = 0;
            second_buf = 1;
        } else {
            first_buf = 1;
            second_buf = 0;
        }

        processed_mask |= p2puart_rx_single_buffer(port, first_buf, ipr);   // ← Collect mask
        processed_mask |= p2puart_rx_single_buffer(port, second_buf, ipr);  // ← Collect mask

        pp->next_rx_buffer = second_buf;
    }

    return processed_mask;  // ← Return to ISR what to acknowledge
}

/**
 * FIXED: ISR implementation
 * Properly sequences RX and TX interrupt handling
 */
#ifdef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
static
#endif
irqreturn_t p2puart_handle_irq(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    struct p2puart_port *pp = to_p2puart_port(port);
    uint32_t ipr;
    uint32_t tx_ack_mask = 0;
    uint32_t rx_ack_mask = 0; 
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    /* Step 1: Snapshot IPR once at IRQ entry */
    ipr = p2puart_serial_in(port, COMCTRL_IPR);

    if (!(ipr & IPR_ALL_MASK)) {
        spin_unlock_irqrestore(&port->lock, flags);
        return IRQ_NONE;
    }
    
    /* ↓↓↓ ADD THIS ENTIRE BLOCK ↓↓↓ */
    /* Handle window timeout errors FIRST (before RX/TX processing) */
    if (ipr & IPR_WINDOW_TIMEOUT_MASK) {
        /* Window timeout indicates firmware expected data within a time
         * window but didn't receive it. This could indicate:
         * - Physical layer issue (cable disconnected, bad connection)
         * - Remote side stopped transmitting mid-packet
         * - Electrical noise causing signal loss
         */
        dev_warn_ratelimited(pp->port.dev,
            "Window timeout error detected (ipr=0x%08x) - possible physical layer issue\n",
            ipr);
        
        /* Count as framing error (closest standard UART error type) */
        port->icount.frame++;
        
        /* Clear the window timeout error bits immediately */
        p2puart_serial_out(port, COMCTRL_IPR, ipr & IPR_WINDOW_TIMEOUT_MASK);
        wmb();
        
        /* Re-read IPR to get updated state without timeout bits */
        ipr = p2puart_serial_in(port, COMCTRL_IPR);
    }
    /* ↑↑↑ ADD THIS ENTIRE BLOCK ↑↑↑ */
    
    /* 2. TX side: Process and save ACK mask (TX can be Ack'd early) */
    if (ipr & IPR_TX_MASK) {
        if (ipr & IPR_TX_DONE0)
            pp->tx_buffer_busy[0] = false;

        if (ipr & IPR_TX_DONE1)
            pp->tx_buffer_busy[1] = false;

        tx_ack_mask = ipr & IPR_TX_MASK;
        
        p2puart_handle_tx(port);
    }
    
    /* 3. RX side: Get back mask of successfully processed buffers */
    if (ipr & IPR_RX_MASK) {
        rx_ack_mask = p2puart_handle_rx(port, ipr); 
    }
    
    /* 4. Acknowledge Handled Interrupts */
    if (tx_ack_mask || rx_ack_mask) {
        p2puart_serial_out(port, COMCTRL_IPR, tx_ack_mask | rx_ack_mask);
        wmb();
        (void)p2puart_serial_in(port, COMCTRL_IPR); 
    }

    spin_unlock_irqrestore(&port->lock, flags);

    return IRQ_HANDLED;
}

#ifndef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
EXPORT_SYMBOL(p2puart_handle_irq);
#endif

/**
 * FIXED: Throttle implementation
 * Properly masks RX interrupts without clearing held buffers
 */
#define IMR_RX_ALL_MASK ( \
	IPR_RX_PACKET_SIZE_DONE0 | IPR_RX_LINK_DONE0 | IPR_OVERRUN_ERROR0 | IPR_WINDOW_TIMEOUT_ERROR0 | \
	IPR_RX_PACKET_SIZE_DONE1 | IPR_RX_LINK_DONE1 | IPR_OVERRUN_ERROR1 | IPR_WINDOW_TIMEOUT_ERROR1 )

static void p2puart_throttle(struct uart_port *port)
{
	unsigned long flags;
	u32 imr;
	struct p2puart_port *pp = to_p2puart_port(port);

	spin_lock_irqsave(&port->lock, flags);

	/* Set throttle state */
	pp->throttle_state = true;

	/* Mask RX interrupts */
	imr = p2puart_serial_in(port, COMCTRL_IMR);
	imr |= IMR_RX_ALL_MASK;
	p2puart_serial_out(port, COMCTRL_IMR, imr);

	spin_unlock_irqrestore(&port->lock, flags);
}

/**
 * FIXED: Unthrottle implementation  
 * Properly releases held buffers and re-enables RX interrupts
 */
static void p2puart_unthrottle(struct uart_port *port)
{
    unsigned long flags;
    u32 imr;
    struct p2puart_port *pp = to_p2puart_port(port);

    spin_lock_irqsave(&port->lock, flags);

    pp->throttle_state = false;

    /* CRITICAL: Read snapshots before clearing */
    uint32_t rxbuf0 = p2puart_serial_in(port, RXBUF0_CTRL);
    uint32_t rxbuf1 = p2puart_serial_in(port, RXBUF1_CTRL);

    /* Release held buffers with snapshots */
    p2puart_clear_buffer_done_bit_and_errors(port, 0, rxbuf0);
    p2puart_clear_buffer_done_bit_and_errors(port, 1, rxbuf1);

    /* Unmask RX interrupts */
    imr = p2puart_serial_in(port, COMCTRL_IMR);
    imr &= ~IMR_RX_ALL_MASK;
    p2puart_serial_out(port, COMCTRL_IMR, imr);

    spin_unlock_irqrestore(&port->lock, flags);
    
    /* NO manual IRQ trigger - let hardware interrupt naturally */
}

/* ======================================================================== */
/* Configuration Functions                                                  */
/* ======================================================================== */

void p2puart_update_no_receive_while_transmit(struct uart_port *port)
{
	uint32_t cpr;
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_update_no_receive_while_transmit");

	pp = to_p2puart_port(port);
	cpr = p2puart_serial_in(port, COMCTRL_CPR);

	if (pp->no_receive_while_transmit)
		cpr |= CPR_NO_RECEIVE_WHILE_TRANSMIT_MASK;
	else
		cpr &= ~CPR_NO_RECEIVE_WHILE_TRANSMIT_MASK;

	p2puart_serial_out(port, COMCTRL_CPR, cpr);
}

#ifndef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
static int p2puart_ioctl_set_no_receive_while_transmit(
	struct uart_port *port, unsigned long no_receive_while_transmit);

void p2puart_platform_update_no_receive_while_transmit(
	struct platform_device *pdev, unsigned long no_receive_while_transmit)
{
	struct uart_port *up = platform_get_drvdata(pdev);
	p2puart_ioctl_set_no_receive_while_transmit(up, no_receive_while_transmit);
}

EXPORT_SYMBOL(p2puart_platform_update_no_receive_while_transmit);
#endif

bool p2puart_get_no_receive_while_transmit(struct uart_port *port)
{
	uint32_t cpr;

	PRINT_FUNCTION_ENTRY("p2puart_get_no_receive_while_transmit");

	cpr = p2puart_serial_in(port, COMCTRL_CPR);
	return !!(cpr & CPR_NO_RECEIVE_WHILE_TRANSMIT_MASK);
}

static void p2puart_update_streaming_mode_bit(struct uart_port *port)
{
	uint32_t cpr;
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_update_streaming_mode_bit");

	pp = to_p2puart_port(port);
	cpr = p2puart_serial_in(port, COMCTRL_CPR);

	if (pp->streaming)
		cpr |= CPR_STREAMING_MODE_ENABLE_MASK;
	else
		cpr &= ~CPR_STREAMING_MODE_ENABLE_MASK;

	p2puart_serial_out(port, COMCTRL_CPR, cpr);
}

static void p2puart_set_end_of_packet_condition_trigger(struct uart_port *port,
							unsigned int trigger)
{
	uint32_t cpr;

	PRINT_FUNCTION_ENTRY("p2puart_set_end_of_packet_condition_trigger");

	cpr = p2puart_serial_in(port, COMCTRL_CPR);
	cpr &= ~CPR_END_OF_PACKET_CONDITION_TRIGGER_MASK;
	cpr |= ((trigger << CPR_END_OF_PACKET_CONDITION_TRIGGER_SHIFT) &
		CPR_END_OF_PACKET_CONDITION_TRIGGER_MASK);
	p2puart_serial_out(port, COMCTRL_CPR, cpr);
}

static void p2puart_set_loopback_mode(struct uart_port *port, unsigned int loopback)
{
	uint32_t cpr;

	PRINT_FUNCTION_ENTRY("p2puart_set_loopback_mode");

	cpr = p2puart_serial_in(port, COMCTRL_CPR);
	cpr &= ~CPR_LOOPBACK_MASK;
	cpr |= (loopback & CPR_LOOPBACK_MASK);
	p2puart_serial_out(port, COMCTRL_CPR, cpr);
}

static void p2puart_set_rx_packet_size(struct uart_port *port, unsigned int rx_packet_size)
{
	uint32_t cpr;
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_set_rx_packet_size");

	pp = to_p2puart_port(port);
	cpr = p2puart_serial_in(port, COMCTRL_CPR);
	cpr &= ~CPR_RX_PACKET_SIZE_MASK;
	cpr |= ((rx_packet_size << CPR_RX_PACKET_SIZE_SHIFT) & CPR_RX_PACKET_SIZE_MASK);
	p2puart_serial_out(port, COMCTRL_CPR, cpr);

	pp->rx_packet_size = rx_packet_size ? rx_packet_size : pp->rx_buffer_size;
}

static void p2puart_mask_all_interrupts(struct uart_port *port)
{
	uint32_t ipr;

	/* Mask all interrupts */
	p2puart_serial_out(port, COMCTRL_IMR, IMR_ALL_MASK);

	/* Clear TX done bits in IPR if set */
	ipr = p2puart_serial_in(port, COMCTRL_IPR);
	ipr &= (IPR_TX_DONE0 | IPR_TX_DONE1);
	if (ipr)
		p2puart_serial_out(port, COMCTRL_IPR, ipr);
}

static int p2puart_startup(struct uart_port *port)
{
    struct p2puart_port *pp;
    unsigned long flags;
    uint32_t imr;

    pp = to_p2puart_port(port);

    PRINT_FUNCTION_ENTRY("p2puart_startup");

    pp->wakeup_parity_mode = 0;

    /* Request the firmware to reset the Rx and Tx channels */
    p2puart_raise_reset_rx_and_tx(port);

    p2puart_mask_all_interrupts(port);

    p2puart_update_streaming_mode_bit(port);

    p2puart_set_loopback_mode(port, pp->loopback);

    p2puart_set_end_of_packet_condition_trigger(port, CPR_EOP_BOTH);

    p2puart_update_scp(port);

    p2puart_lower_reset_rx_and_tx(port);
    
    p2puart_unmask_all_interrupts(port); //added doron
    
    pp->tx_buffer_busy[0] = false;
    pp->tx_buffer_busy[1] = false;

    /* enable Tx and Rx interrupts */
    spin_lock_irqsave(&port->lock, flags);
    imr = IMR_ALL_MASK;

    /* Enable link done interrupts */
    imr &= ~(IPR_RX_LINK_DONE0 | IPR_RX_LINK_DONE1);

    /* Enable Packet Size interrupts */
    imr &= ~(IPR_RX_PACKET_SIZE_DONE0 | IPR_RX_PACKET_SIZE_DONE1);
    
    imr &= ~(IPR_OVERRUN_ERROR0 | IPR_OVERRUN_ERROR1); //doron addition
    
    /* ↓↓↓ ADD THIS LINE ↓↓↓ */
    /* Enable window timeout error interrupts */
    imr &= ~(IPR_WINDOW_TIMEOUT_ERROR0 | IPR_WINDOW_TIMEOUT_ERROR1);
    /* ↑↑↑ ADD THIS LINE ↑↑↑ */
    
    imr &= ~IMR_TX_MASK;
    p2puart_serial_out(port, COMCTRL_IMR, imr);

    pp->previous_packet_size_last_address[0] = pp->rx_buffer_size;
    pp->previous_packet_size_last_address[1] = pp->rx_buffer_size;

    spin_unlock_irqrestore(&port->lock, flags);

    PRINT_FUNCTION_RETURN("p2puart_startup");

    return 0;
}

static void p2puart_shutdown(struct uart_port *port)
{
	unsigned long flags;
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_shutdown");

	pp = to_p2puart_port(port);

	spin_lock_irqsave(&port->lock, flags);

	p2puart_raise_reset_rx_and_tx(port);
	p2puart_mask_all_interrupts(port);

	spin_unlock_irqrestore(&port->lock, flags);

	/* Wait for any pending ISR to complete */
	synchronize_irq(port->irq);

	if (pp->tx_buffer_busy[0] || pp->tx_buffer_busy[1])
		dev_warn(pp->port.dev, "TX buffer still busy in shutdown\n");

	PRINT_FUNCTION_RETURN("p2puart_shutdown");
}

static void p2puart_update_scp(struct uart_port *port)
{
	uint32_t scp;
	struct p2puart_port *pp;
	uint32_t cto;
	unsigned int parity;

	PRINT_FUNCTION_ENTRY("p2puart_update_scp");

	pp = to_p2puart_port(port);
	parity = pp->parity;
	
	if (pp->wakeup_parity_mode && parity == SCP_PARITY_MARK)
		parity = SCP_PARITY_WAKEUP;

#ifdef DEBUG_PRINT_PARITY
	printk("%s: setting SCP parity to %u\n", pp->name, parity);
#endif

	scp = 0;

	/* Stop bits */
	if (1 == pp->number_of_stop_bits)
		scp |= SCP_1_STOP_BIT;
	else if (2 == pp->number_of_stop_bits)
		scp |= SCP_2_STOP_BITS;

	/* RX parity */
	scp |= ((parity << SCP_RX_PARITY_SHIFT) & SCP_RX_PARITY_MASK);
	
	/* TX parity */
	scp |= (parity & SCP_TX_PARITY_MASK);

	/* Transceiver termination */
	scp &= ~SCP_TRANSCEIVER_TERMINATION_MASK;
	if (pp->termination_enable)
		scp |= SCP_TRANSCEIVER_TERMINATION_MASK;

	/* Character timeout period */
	cto = p2puart_cto_physical_to_raw(pp->character_timeout_period);
	scp |= ((cto << SCP_CHARACTER_TIMEOUT_PERIOD_SHIFT) &
		SCP_CHARACTER_TIMEOUT_PERIOD_MASK);

	p2puart_serial_out(port, P2P_SCP, scp);
}

static void p2puart_warn_if_baud_rate_error_is_too_high(struct uart_port *port,
					    unsigned long calculated_baud,
					    unsigned long baud)
{
	struct p2puart_port *pp;
	unsigned long baud_error;
	unsigned long max_error;

	PRINT_FUNCTION_ENTRY("p2puart_warn_if_baud_rate_error_is_too_high");

	max_error = (baud * MAX_BAUD_RATE_ERROR_PERCENTS) / 100;
	pp = to_p2puart_port(port);
	
	baud_error = (baud < calculated_baud) ? 
	             (calculated_baud - baud) : (baud - calculated_baud);

	if (baud_error > max_error) {
		dev_warn(pp->port.dev,
		         "Baud rate error exceeds %d%%: desired=%lu, actual=%lu, error=%lu\n",
		         MAX_BAUD_RATE_ERROR_PERCENTS, baud, calculated_baud, baud_error);
	}
}

#if defined(RHEL_RELEASE_CODE) && defined(RHEL_RELEASE_VERSION)
#if RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(9,2)
static void p2puart_set_termios(struct uart_port *port,
				 struct ktermios *termios,
				 const struct ktermios *old)
#endif
#else
static void p2puart_set_termios(struct uart_port *port,
				 struct ktermios *termios,
				 struct ktermios *old)
#endif
{
	unsigned long baud;
	unsigned long min_baud;
	unsigned long max_baud;
	unsigned long uartclk;
	unsigned long calculated_baud;
	uint16_t baud_n;
	uint16_t baud_d;
	uint32_t brr;
	uint32_t imr;
	unsigned long flags;
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_set_termios");

	pp = to_p2puart_port(port);
	uartclk = port->uartclk;
	min_baud = 0;
	max_baud = pp->maximum_baud_rate_khz * 1000;

	/* Get baud rate */
	baud = uart_get_baud_rate(port, termios, old, min_baud, max_baud);

	/* Calculate dividers */
	calculated_baud = p2puart_calc_baud_divs(uartclk, baud, &baud_n, &baud_d);
	
	if (0 == calculated_baud) {
		dev_err(pp->port.dev, "Invalid baud rate %lu for clock %u Hz\n",
		        baud, uartclk);
		calculated_baud = 9600; /* Fallback */
	}

#ifdef DEBUG_PRINT_BAUD_RATE_CALCS
	printk("%s: clock=%u, requested_baud=%lu, calculated_baud=%lu, n=%d, d=%d, max=%lu\n",
	       pp->name, port->uartclk, baud, calculated_baud, 
	       (int)baud_n, (int)baud_d, max_baud);
#endif

	if (calculated_baud != baud)
		p2puart_warn_if_baud_rate_error_is_too_high(port, calculated_baud, baud);

	/* Construct Baud Rate Register value */
	brr = 0;
	brr |= (((uint32_t)baud_n << BAUD_RATE_N_SHIFT) & BAUD_RATE_N_MASK);
	brr |= ((uint32_t)baud_d & BAUD_RATE_D_MASK);

	spin_lock_irqsave(&port->lock, flags);

	pp->requested_baud_rate = baud;
	pp->calculated_baud_rate = calculated_baud;
	uart_update_timeout(port, termios->c_cflag, calculated_baud);

	/* Save IMR */
	imr = p2puart_serial_in(port, COMCTRL_IMR);

	/* Reset to change baud rate */
	p2puart_raise_reset_rx_and_tx(port);

	/* Write baud rate */
	p2puart_serial_out(port, P2P_BAUD_RATE, brr);

	/* Warn if data bits != 8 */
	if ((termios->c_cflag & CSIZE) != CS8)
		dev_warn(pp->port.dev, "Non-8-bit data size requested but not supported\n");

	/* Set stop bits */
	pp->number_of_stop_bits = (termios->c_cflag & CSTOPB) ? 2 : 1;

	/* Set parity */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & CMSPAR) {
			pp->parity = (termios->c_cflag & PARODD) ? 
			             SCP_PARITY_MARK : SCP_PARITY_SPACE;
		} else {
			pp->parity = (termios->c_cflag & PARODD) ? 
			             SCP_PARITY_ODD : SCP_PARITY_EVEN;
		}
	} else {
		pp->parity = SCP_PARITY_NONE;
	}

	/* Configure status masks */
	port->ignore_status_mask = 0;
	port->read_status_mask = COMCTRL_RXBUF_OVERRUN_ERROR;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |= P2PUART_PARITY_ERROR | P2PUART_FRAME_ERROR;

	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= P2PUART_PARITY_ERROR | P2PUART_FRAME_ERROR;

	pp->rx_enabled = !!(termios->c_cflag & CREAD);

	p2puart_update_scp(port);

	pp->tx_buffer_busy[0] = false;
	pp->tx_buffer_busy[1] = false;

	/* Exit reset */
	p2puart_lower_reset_rx_and_tx(port);

	/* Restore IMR */
	p2puart_serial_out(port, COMCTRL_IMR, imr);

	spin_unlock_irqrestore(&port->lock, flags);

	/* Update termios with actual baud rate */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, calculated_baud, calculated_baud);

	PRINT_FUNCTION_RETURN("p2puart_set_termios");
}

static const char *p2puart_type(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_type");
	return P2PUART_PORT_TYPE_STRING;
}

static void p2puart_release_port(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_release_port");
	/* Nothing to do */
}

static int p2puart_request_port(struct uart_port *port)
{
	PRINT_FUNCTION_ENTRY("p2puart_request_port");
	return 0;
}

static void p2puart_config_port(struct uart_port *port, int flags)
{
	PRINT_FUNCTION_ENTRY("p2puart_config_port");

	if (flags & UART_CONFIG_TYPE)
		port->type = P2PUART_LINUX_PORT_TYPE;
}

static int p2puart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_verify_port");

	pp = to_p2puart_port(port);
	
	if (ser->type != P2PUART_LINUX_PORT_TYPE) {
		dev_err(pp->port.dev, "Port type incorrect\n");
		return -EINVAL;
	}
	if (port->irq != ser->irq) {
		dev_err(pp->port.dev, "IRQ incorrect\n");
		return -EINVAL;
	}
	if (port->iotype != ser->io_type) {
		dev_err(pp->port.dev, "IO type incorrect\n");
		return -EINVAL;
	}
	return 0;
}

/* ======================================================================== */
/* IOCTL Handlers                                                           */
/* ======================================================================== */

static int p2puart_ioctl_set_loopback_mode(struct uart_port *port, unsigned long loopback)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_loopback_mode");

	pp = to_p2puart_port(port);
	
	if ((loopback != LOOPBACK_NONE) && (loopback != LOOPBACK_COMCTRL) &&
	    (loopback != LOOPBACK_ENDPOINT) && (loopback != LOOPBACK_EXTERNAL))
		return -EINVAL;

	spin_lock_irqsave(&port->lock, flags);
	pp->loopback = loopback;

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_set_loopback_mode(port, loopback);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static int p2puart_ioctl_get_loopback_mode(struct uart_port *port)
{
	struct p2puart_port *pp;
	int loopback;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_loopback_mode");

	pp = to_p2puart_port(port);
	spin_lock_irqsave(&port->lock, flags);
	loopback = pp->loopback;
	spin_unlock_irqrestore(&port->lock, flags);
	return loopback;
}

static int p2puart_ioctl_set_streaming_mode(struct uart_port *port, unsigned long streaming)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_streaming_mode");

	pp = to_p2puart_port(port);
	if (streaming > 1)
		return -EINVAL;

	spin_lock_irqsave(&port->lock, flags);
	pp->streaming = !!streaming;

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_update_streaming_mode_bit(port);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static int p2puart_ioctl_get_streaming_mode(struct uart_port *port)
{
	struct p2puart_port *pp;
	int streaming;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_streaming_mode");

	pp = to_p2puart_port(port);
	spin_lock_irqsave(&port->lock, flags);
	streaming = pp->streaming ? 1 : 0;
	spin_unlock_irqrestore(&port->lock, flags);
	return streaming;
}

static int p2puart_ioctl_get_no_receive_while_transmit(struct uart_port *port)
{
	struct p2puart_port *pp;
	int no_receive_while_transmit;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_no_receive_while_transmit");

	pp = to_p2puart_port(port);
	spin_lock_irqsave(&port->lock, flags);
	no_receive_while_transmit = pp->no_receive_while_transmit ? 1 : 0;
	spin_unlock_irqrestore(&port->lock, flags);
	return no_receive_while_transmit;
}

static int p2puart_ioctl_set_no_receive_while_transmit(
	struct uart_port *port, unsigned long no_receive_while_transmit)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_no_receive_while_transmit");

	pp = to_p2puart_port(port);
	if (no_receive_while_transmit > 1)
		return -EINVAL;

	spin_lock_irqsave(&port->lock, flags);
	pp->no_receive_while_transmit = !!no_receive_while_transmit;

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_update_no_receive_while_transmit(port);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static int p2puart_ioctl_get_termination_capability(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_termination_capability");

	pp = to_p2puart_port(port);
	return pp->transceiver_termination_capability;
}

static bool p2puart_is_termination_controllable(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_is_termination_controllable");

	pp = to_p2puart_port(port);
	return (2 == pp->transceiver_termination_capability);
}

static int p2puart_ioctl_set_termination(struct uart_port *port, unsigned long termination)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_termination");

	pp = to_p2puart_port(port);
	if (termination > 1)
		return -EINVAL;

	if (!p2puart_is_termination_controllable(port))
		return -ENOTSUPP;

	spin_lock_irqsave(&port->lock, flags);
	pp->termination_enable = termination;

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_update_scp(port);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static int p2puart_ioctl_get_termination(struct uart_port *port)
{
	uint32_t scp;
	int termination_enable;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_termination");

	spin_lock_irqsave(&port->lock, flags);
	scp = p2puart_serial_in(port, P2P_SCP);
	termination_enable = !!(scp & SCP_TRANSCEIVER_TERMINATION_MASK);
	spin_unlock_irqrestore(&port->lock, flags);

	return termination_enable;
}

static int p2puart_ioctl_get_duplex_capability(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_duplex_capability");

	pp = to_p2puart_port(port);
	return pp->full_half_duplex_capability;
}

static int p2puart_ioctl_get_transmit_capability(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_transmit_capability");

	pp = to_p2puart_port(port);
	return pp->tx_channel_exists;
}

static int p2puart_ioctl_get_receive_capability(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_receive_capability");

	pp = to_p2puart_port(port);
	return pp->rx_channel_exists;
}

static int p2puart_ioctl_set_character_timeout_period(struct uart_port *port,
						      unsigned long cto)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_character_timeout_period");

	pp = to_p2puart_port(port);
	if ((0 == cto) || (cto > 256))
		return -EINVAL;

	spin_lock_irqsave(&port->lock, flags);
	pp->character_timeout_period = cto;

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_update_scp(port);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static int p2puart_ioctl_get_character_timeout_period(struct uart_port *port)
{
	unsigned int character_timeout;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_character_timeout_period");

	spin_lock_irqsave(&port->lock, flags);
	character_timeout = p2puart_get_character_timeout_period(port);
	spin_unlock_irqrestore(&port->lock, flags);
	return character_timeout;
}

static int p2puart_ioctl_set_wakeup_parity_mode(struct uart_port *port, unsigned long mode)
{
	struct p2puart_port *pp;
	uint32_t imr;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_set_wakeup_parity_mode");

	pp = to_p2puart_port(port);
	if (mode > 1)
		return -EINVAL;

	if (mode == pp->wakeup_parity_mode)
		return 0;

	pp->wakeup_parity_mode = mode;

	spin_lock_irqsave(&port->lock, flags);

	imr = p2puart_serial_in(port, COMCTRL_IMR);
	p2puart_raise_reset_rx_and_tx(port);
	p2puart_update_scp(port);
	p2puart_lower_reset_rx_and_tx(port);
	p2puart_serial_out(port, COMCTRL_IMR, imr);
	
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int p2puart_ioctl_get_wakeup_parity_mode(struct uart_port *port)
{
	struct p2puart_port *pp;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl_get_wakeup_parity_mode");

	pp = to_p2puart_port(port);
	return pp->wakeup_parity_mode;
}

static int p2puart_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	int err = -ENOIOCTLCMD;
	uint32_t param_value;

	PRINT_FUNCTION_ENTRY("p2puart_ioctl");

	switch (cmd) {
	case P2PUART_IOCTL_SET_LOOPBACK_MODE:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_loopback_mode(port, param_value);
		break;
	case P2PUART_IOCTL_GET_LOOPBACK_MODE:
		err = p2puart_ioctl_get_loopback_mode(port);
		break;
	case P2PUART_IOCTL_SET_STREAMING_MODE:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_streaming_mode(port, param_value);
		break;
	case P2PUART_IOCTL_GET_STREAMING_MODE:
		err = p2puart_ioctl_get_streaming_mode(port);
		break;
	case P2PUART_IOCTL_SET_NO_RECEIVE_WHILE_TRANSMIT:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_no_receive_while_transmit(port, param_value);
		break;
	case P2PUART_IOCTL_GET_NO_RECEIVE_WHILE_TRANSMIT:
		err = p2puart_ioctl_get_no_receive_while_transmit(port);
		break;
	case P2PUART_IOCTL_GET_TERMINATION_CAPABILITY:
		err = p2puart_ioctl_get_termination_capability(port);
		break;
	case P2PUART_IOCTL_SET_TERMINATION:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_termination(port, param_value);
		break;
	case P2PUART_IOCTL_GET_TERMINATION:
		err = p2puart_ioctl_get_termination(port);
		break;
	case P2PUART_IOCTL_GET_DUPLEX_CAPABILITY:
		err = p2puart_ioctl_get_duplex_capability(port);
		break;
	case P2PUART_IOCTL_GET_TRANSMIT_CAPABILITY:
		err = p2puart_ioctl_get_transmit_capability(port);
		break;
	case P2PUART_IOCTL_GET_RECEIVE_CAPABILITY:
		err = p2puart_ioctl_get_receive_capability(port);
		break;
	case P2PUART_IOCTL_SET_CHARACTER_TIMEOUT_PERIOD:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_character_timeout_period(port, param_value);
		break;
	case P2PUART_IOCTL_GET_CHARACTER_TIMEOUT_PERIOD:
		err = p2puart_ioctl_get_character_timeout_period(port);
		break;
	case P2PUART_IOCTL_SET_WAKEUP_PARITY_MODE:
		if (get_user(param_value, (uint32_t __user *)arg))
			return -EFAULT;
		err = p2puart_ioctl_set_wakeup_parity_mode(port, param_value);
		break;
	case P2PUART_IOCTL_GET_WAKEUP_PARITY_MODE:
		err = p2puart_ioctl_get_wakeup_parity_mode(port);
		break;
	default:
		break;
	}

	return err;
}

static const struct uart_ops serial_p2puart_ops = {
	.tx_empty = p2puart_tx_empty,
	.get_mctrl = p2puart_get_mctrl,
	.set_mctrl = p2puart_set_mctrl,
	.stop_tx = p2puart_stop_tx,
	.start_tx = p2puart_start_tx,
	.stop_rx = p2puart_stop_rx,
	.break_ctl = p2puart_break_ctl,
	.startup = p2puart_startup,
	.shutdown = p2puart_shutdown,
	.set_termios = p2puart_set_termios,
	.throttle = p2puart_throttle,
	.unthrottle = p2puart_unthrottle,
	.type = p2puart_type,
	.release_port = p2puart_release_port,
	.request_port = p2puart_request_port,
	.config_port = p2puart_config_port,
	.verify_port = p2puart_verify_port,
	.ioctl = p2puart_ioctl
};

/* ======================================================================== */
/* Console Support (Minimal)                                                */
/* ======================================================================== */

#ifdef CONFIG_SERIAL_P2PUART_CONSOLE
#define P2PUART_CONSOLE (&p2puart_console)

static void p2puart_console_putchar(struct uart_port *port, int ch)
{
	PRINT_FUNCTION_ENTRY("p2puart_console_putchar");
	/* Not implemented */
}

static void p2puart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &p2puart_ports[co->index]->port;
	int locked = 1;
	unsigned long flags;

	PRINT_FUNCTION_ENTRY("p2puart_console_write");

	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&port->lock, flags);
	else
		spin_lock_irqsave(&port->lock, flags);

	uart_console_write(port, s, count, p2puart_console_putchar);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int __init p2puart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	PRINT_FUNCTION_ENTRY("p2puart_console_setup");

	if (co->index >= P2PUART_NR_MAX || co->index < 0)
		co->index = 0;

	port = &p2puart_ports[co->index]->port;
	if (port == NULL) {
		pr_info("serial port %d not yet initialized\n", co->index);
		return -ENODEV;
	}
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver p2puart_uart_driver;
static struct console p2puart_console = {
	.name = P2PUART_TTY_NAME,
	.write = p2puart_console_write,
	.device = uart_console_device,
	.setup = p2puart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &p2puart_uart_driver,
};

#else
#define P2PUART_CONSOLE NULL
#endif

static struct uart_driver p2puart_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "p2puart",
	.dev_name = P2PUART_TTY_NAME,
	.major = 0,
	.minor = 0,
	.nr = P2PUART_NR_MAX,
	.cons = P2PUART_CONSOLE,
};

/* ======================================================================== */
/* Platform Driver Functions                                                */
/* ======================================================================== */

static int p2puart_probe_dt_alias(int index, struct device *dev)
{
	struct device_node *np;
	int ret = index;

	if (!IS_ENABLED(CONFIG_OF))
		return ret;

	np = dev->of_node;
	if (!np)
		return ret;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0)
		ret = index;
	else if (ret >= ARRAY_SIZE(p2puart_ports) || p2puart_ports[ret] != NULL) {
		dev_warn(dev, "Requested serial port %d not available\n", ret);
		ret = index;
	}

	return ret;
}

#ifdef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
static
#endif
int p2puart_remove(struct platform_device *pdev)
{
	struct p2puart_port *pp = platform_get_drvdata(pdev);

	if (pp) {
		uart_remove_one_port(&p2puart_uart_driver, &pp->port);
		p2puart_ports[pp->port.line] = NULL;
		p2puart_ports_num--;
	}

	if (!p2puart_ports_num)
		uart_unregister_driver(&p2puart_uart_driver);

	return 0;
}

#ifndef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
EXPORT_SYMBOL(p2puart_remove);
#endif

static void p2puart_set_buffer_offsets_and_sizes(struct uart_port *port)
{
	struct p2puart_port *pp;

	pp = to_p2puart_port(port);
	pp->rx_buffer_size = p2puart_get_rx_buffer_size(port);
	pp->rx_buffer_offset[0] = p2puart_serial_in(port, COMCTRL_RX_BUF_ADDR_OFFSET);
	pp->rx_buffer_offset[1] = pp->rx_buffer_offset[0] + pp->rx_buffer_size;
	pp->tx_buffer_size = p2puart_get_tx_buffer_size(port);
	pp->tx_buffer_offset[0] = p2puart_serial_in(port, COMCTRL_TX_BUF_ADDR_OFFSET);
	pp->tx_buffer_offset[1] = pp->tx_buffer_offset[0] + pp->tx_buffer_size;
	port->fifosize = pp->tx_buffer_size;
}

static void p2puart_set_port_capabilities(struct uart_port *port)
{
	struct p2puart_port *pp;
	uint32_t scap;

	pp = to_p2puart_port(port);
	scap = p2puart_serial_in(port, P2P_SCAP);
	
	pp->maximum_baud_rate_khz = scap & SCAP_MAXIMUM_BAUD_RATE_KBPS_MASK;
	pp->full_half_duplex_capability = 
		(scap & SCAP_FULL_HALF_DUPLEX_MASK) >> SCAP_FULL_HALF_DUPLEX_SHIFT;
	pp->transceiver_termination_capability =
		(scap & SCAP_TRANSCEIVER_TERMINATION_MASK) >> SCAP_TRANSCEIVER_TERMINATION_SHIFT;
	pp->rx_channel_exists = !!(scap & SCAP_RX_CHANNEL_EXISTS);
	pp->tx_channel_exists = !!(scap & SCAP_TX_CHANNEL_EXISTS);
}

static int p2puart_map_memory(struct platform_device *pdev, struct uart_port *port)
{
	struct resource *res;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		return -ENODEV;
	}
	
	port->mapbase = res->start;
	port->membase = devm_ioremap(&pdev->dev, res->start, resource_size(res));

	if (!port->membase) {
		dev_err(&pdev->dev, "Unable to map registers\n");
		return -ENOMEM;
	}

#ifdef DEBUG_PRINT_DEVICE_MAPPINGS
	printk("Virtual: 0x%p, Physical: 0x%llx\n",
	       port->membase, (unsigned long long)port->mapbase);
#endif

	return 0;
}

static int p2puart_get_irq(struct platform_device *pdev, struct uart_port *port)
{
	int irq = platform_get_irq(pdev, 0);
	
	if (irq < 0) {
		dev_err(&pdev->dev, "Unable to get IRQ resource: %d\n", irq);
		return irq;
	}
	
	dev_info(&pdev->dev, "IRQ %d assigned\n", irq);
	port->irq = irq;
	return 0;
}

static void p2puart_print_driver_version(void)
{
	pr_info("%s - version %s\n", P2PUART_DESC, P2PUART_VER);
}

#ifndef DISABLE_MEMORY_BUFFERS_QUICK_TEST
static bool p2puart_quick_test_memory_buffer(struct uart_port *port,
					     unsigned int buffer_offset,
					     unsigned int buffer_size)
{
	int i;
	static const char *test_patterns[] = { "0", "12", "345", "6789" };
	char temp[4];

	if (buffer_size < sizeof(temp))
		return false;

	for (i = 0; i < ARRAY_SIZE(test_patterns); ++i) {
		size_t len = strlen(test_patterns[i]);

		if (len > sizeof(temp))
			continue;

		memcpy_toio(port->membase + buffer_offset, test_patterns[i], len);
		memcpy_fromio(temp, port->membase + buffer_offset, len);
		
		if (memcmp(test_patterns[i], temp, len) != 0) {
			printk(KERN_ERR "P2PUART: Memory buffer test failed at offset 0x%x\n",
			       buffer_offset);
			printk(KERN_ERR "Written: %s, Read: ", test_patterns[i]);
			print_hex_dump_bytes("", DUMP_PREFIX_NONE, temp, len);
			return false;
		}
	}

	return true;
}

static bool p2puart_quick_test_all_memory_buffers(struct uart_port *port)
{
	struct p2puart_port *pp;
	int i;
	
	pp = to_p2puart_port(port);

	for (i = 0; i < 2; ++i)
		if (!p2puart_quick_test_memory_buffer(
			    port, pp->tx_buffer_offset[i], pp->tx_buffer_size))
			return false;

	for (i = 0; i < 2; ++i)
		if (!p2puart_quick_test_memory_buffer(
			    port, pp->rx_buffer_offset[i], pp->rx_buffer_size))
			return false;

	return true;
}
#endif

static bool p2puart_check_module_version(struct platform_device *pdev, struct uart_port *port)
{
	uint32_t mvr;
	uint8_t cc_major;
	uint8_t cc_minor;
	uint8_t ep_major;
	uint8_t ep_minor;
	bool supported_cc = false;
	bool supported_ep = false;

	mvr = p2puart_serial_in(port, COMCTRL_MVR);

	cc_major = (uint8_t)((mvr >> 24) & 0xFF);
	cc_minor = (uint8_t)((mvr >> 16) & 0xFF);
	ep_major = (uint8_t)((mvr >> 8) & 0xFF);
	ep_minor = (uint8_t)(mvr & 0xFF);

	if ((8 == cc_major) && (cc_minor >= 2))
		supported_cc = true;

	if ((5 == ep_major) && (ep_minor >= 1))
		supported_ep = true;

	if (!supported_cc)
		dev_err(&pdev->dev, "Unsupported ComCtrl version: %d.%d\n",
			cc_major, cc_minor);

	if (!supported_ep)
		dev_err(&pdev->dev, "Unsupported Endpoint version: %d.%d\n",
			ep_major, ep_minor);

	return (supported_cc && supported_ep);
}

static int p2puart_probe(struct platform_device *pdev)
{
	struct uart_port *up;
	struct p2puart_port *pp;
	int index;
	int ret;
	uint32_t scp;

	/* Find free slot */
	for (index = 0; index < P2PUART_NR_MAX; index++)
		if (!p2puart_ports[index])
			break;
			
	if (index == P2PUART_NR_MAX)
		return -EBUSY;

	index = p2puart_probe_dt_alias(index, &pdev->dev);

	/* Allocate port structure */
	p2puart_ports[index] = devm_kzalloc(&pdev->dev,
	                                    sizeof(*p2puart_ports[index]),
	                                    GFP_KERNEL);
	if (!p2puart_ports[index])
		return -ENOMEM;

	up = &p2puart_ports[index]->port;
	up->dev = &pdev->dev;
	up->line = index;
	up->type = PORT_16550A;
	up->iotype = UPIO_MEM;
	up->ops = &serial_p2puart_ops;
	up->flags = UPF_BOOT_AUTOCONF;

	/* Map memory */
	ret = p2puart_map_memory(pdev, up);
	if (ret < 0)
		return ret;

	/* Check module version */
	if (!p2puart_check_module_version(pdev, up))
		return -ENODEV;

	/* Get IRQ */
	ret = p2puart_get_irq(pdev, up);
	if (ret < 0)
		return ret;

	/* Get clock frequency */
	up->uartclk = p2puart_get_clock_frequency(up);

	pp = to_p2puart_port(up);

	/* Set buffer offsets and capabilities */
	p2puart_set_buffer_offsets_and_sizes(up);
	p2puart_set_port_capabilities(up);

	/* Allocate temp RX buffer */
	pp->temp_rx_buffer = devm_kmalloc(&pdev->dev,
	                                  p2puart_get_rx_buffer_size(up),
	                                  GFP_KERNEL);
	if (!pp->temp_rx_buffer)
		return -ENOMEM;

	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, up->irq, p2puart_handle_irq, 
	                       IRQF_SHARED, "p2puart_irq", up);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n", up->irq, ret);
		return ret;
	}
	dev_info(&pdev->dev, "IRQ %d registered\n", up->irq);

	/* Initialize port settings */
	pp->mute_enabled = false;
	pp->character_timeout_period = P2PUART_DEFAULT_CHARACTER_TIMEOUT;

	scp = p2puart_serial_in(up, P2P_SCP);
	pp->termination_enable = !!(scp & SCP_TRANSCEIVER_TERMINATION_MASK);

	pp->number_of_stop_bits = 0;
	pp->parity = 0;
	pp->rx_enabled = true;
	pp->loopback = LOOPBACK_NONE;
	pp->no_receive_while_transmit = false;
	pp->streaming = false;
	pp->throttle_state = false;

	/* Initialize with reset state */
	p2puart_raise_reset_rx_and_tx(up);
	p2puart_set_rx_packet_size(up, 0);
	p2puart_update_no_receive_while_transmit(up);

	pp->tx_buffer_busy[0] = false;
	pp->tx_buffer_busy[1] = false;
	pp->next_tx_buffer = 0;
	pp->next_rx_buffer = 0;

#ifndef DISABLE_MEMORY_BUFFERS_QUICK_TEST
	if (!p2puart_quick_test_all_memory_buffers(up)) {
		dev_err(&pdev->dev, "Memory buffer test failed\n");
		return -EIO;
	}
#endif

	snprintf(pp->name, sizeof(pp->name), "p2puart%d", up->line);

	p2puart_mask_all_interrupts(up);

	/* Register driver if first port */
	if (!p2puart_ports_num) {
		p2puart_print_driver_version();
		ret = uart_register_driver(&p2puart_uart_driver);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register UART driver: %d\n", ret);
			return ret;
		}
	}
	p2puart_ports_num++;

	/* Add port */
	ret = uart_add_one_port(&p2puart_uart_driver, up);
	if (ret) {
		p2puart_ports[index] = NULL;
		p2puart_remove(pdev);
		return ret;
	}

	platform_set_drvdata(pdev, up);

	dev_info(&pdev->dev, "P2PUART probed successfully: %s\n", pp->name);
	return 0;
}

#ifndef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
EXPORT_SYMBOL(p2puart_probe);
#endif

#ifdef CONFIG_PM_SLEEP
static int p2puart_suspend(struct device *dev)
{
	struct p2puart_port *pp = dev_get_drvdata(dev);
	uart_suspend_port(&p2puart_uart_driver, &pp->port);
	return 0;
}

static int p2puart_resume(struct device *dev)
{
	struct p2puart_port *pp = dev_get_drvdata(dev);
	uart_resume_port(&p2puart_uart_driver, &pp->port);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(p2puart_pm_ops, p2puart_suspend, p2puart_resume);

static const struct of_device_id serial_ids[] = {
	{ .compatible = "xlnx,axi4-lite-p2p-uart-1.0", },
	{ .compatible = "xlnx,axi4-p2p-uart-1.0", },
	{ }
};
MODULE_DEVICE_TABLE(of, serial_ids);

#ifdef P2PUART_IS_EXTERNAL_PLATFORM_DEVICE
static struct platform_driver p2puart_platform_driver = {
	.probe  = p2puart_probe,
	.remove = p2puart_remove,
	.driver = {
		.name = "p2puart",
		.of_match_table = of_match_ptr(serial_ids),
		.pm = &p2puart_pm_ops,
	},
};

module_platform_driver(p2puart_platform_driver);
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rafael BSP Team");
MODULE_DESCRIPTION(P2PUART_DESC);
MODULE_VERSION(P2PUART_VER);
