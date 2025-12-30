#ifndef _P2PUART_IOCTLS_H_
#define _P2PUART_IOCTLS_H_

#define P2PUART_IOC_MAGIC 'P'

/*
 * 0=No loopback
 * 1=ComCtrl loopback
 * 2=EndPoint (P2PUART) loopback
 * 3=External loopback
 */
#define P2PUART_IOCTL_SET_LOOPBACK_MODE _IO(P2PUART_IOC_MAGIC, 0xc0)
#define P2PUART_IOCTL_GET_LOOPBACK_MODE _IO(P2PUART_IOC_MAGIC, 0xc1)

/*
 * 0=Streaming mode disabled
 * 1=Streaming mode enabled
 */
#define P2PUART_IOCTL_SET_STREAMING_MODE _IO(P2PUART_IOC_MAGIC, 0xc2)
#define P2PUART_IOCTL_GET_STREAMING_MODE _IO(P2PUART_IOC_MAGIC, 0xc3)

/* Set/get No Receive While Transmit mode.
 * 0 = Receive enabled while transmitting.
 * 1 = Receive disabled while transmitting.
 */
#define P2PUART_IOCTL_SET_NO_RECEIVE_WHILE_TRANSMIT _IO(P2PUART_IOC_MAGIC, 0xc4)
#define P2PUART_IOCTL_GET_NO_RECEIVE_WHILE_TRANSMIT _IO(P2PUART_IOC_MAGIC, 0xc5)

/* Get transceiver termination capability.
 * 0=No termination available
 * 1=Constant termination
 * 2=Controllable termination
 */
#define P2PUART_IOCTL_GET_TERMINATION_CAPABILITY _IO(P2PUART_IOC_MAGIC, 0xc7)

/* Set/get transceiver termination.
 * 0=Disabled
 * 1=Enabled
 * Termination can be changed with P2PUART_IOCTL_SET_TERMINATION only if
 * P2PUART_IOCTL_GET_TERMINATION_CAP returns 2 (Controllable termination).
 */
#define P2PUART_IOCTL_SET_TERMINATION _IO(P2PUART_IOC_MAGIC, 0xc8)
#define P2PUART_IOCTL_GET_TERMINATION _IO(P2PUART_IOC_MAGIC, 0xc9)

/* Get Full/Half duplex capability.
 * 0=Half
 * 1=Full
 * 2=Controllable
 */
#define P2PUART_IOCTL_GET_DUPLEX_CAPABILITY _IO(P2PUART_IOC_MAGIC, 0xcd)

/* Set/get duplex mode.
 * 0=?
 * 1=?
 */

/*
Not implemented because this feature is missing in HSID document.
#define P2PUART_IOCTL_SET_DUPLEX		_IO(P2PUART_IOC_MAGIC, 0xce)
#define P2PUART_IOCTL_GET_DUPLEX		_IO(P2PUART_IOC_MAGIC, 0xcf)
*/

/* Get transmit/receive capability
 * 0=The capability is not available
 * 1=The capability is available
 */
#define P2PUART_IOCTL_GET_TRANSMIT_CAPABILITY _IO(P2PUART_IOC_MAGIC, 0xd0)
#define P2PUART_IOCTL_GET_RECEIVE_CAPABILITY _IO(P2PUART_IOC_MAGIC, 0xd1)

/* Set/get character timeout period. The values are physical values,
between 1-256 (inclusive). In character time units. */
#define P2PUART_IOCTL_SET_CHARACTER_TIMEOUT_PERIOD _IO(P2PUART_IOC_MAGIC, 0xd4)
#define P2PUART_IOCTL_GET_CHARACTER_TIMEOUT_PERIOD _IO(P2PUART_IOC_MAGIC, 0xd5)

/* Wakeup parity mode.
To enable wakeup parity, use the set wakeup parity mode ioctl
(with value 1) in conjunction with the stty Mark parity setting
(cmspar parodd).
The wakeup parity mode will be reset to 0 upon port open (startup).

0=Wakeup parity will not be enabled.
1=Wakeup parity setting will be used whenever port parity is set to
Mark (CMSPAR).
*/
#define P2PUART_IOCTL_SET_WAKEUP_PARITY_MODE _IO(P2PUART_IOC_MAGIC, 0xd8)
#define P2PUART_IOCTL_GET_WAKEUP_PARITY_MODE _IO(P2PUART_IOC_MAGIC, 0xd9)

/* ****************************************************************** */
/*  Unit test IO controls. Enabled using DEBUG_WITH_UNIT_TEST_IOCTLS  */
/* ****************************************************************** */

/* Transmits a single Tx buffer (half of the double buffer) */
#define P2PUART_IOCTL_UNITTEST_TRANSMIT_FULL_TX_BUFFER                         \
	_IO(P2PUART_IOC_MAGIC, 0xe1)

/* Transmits two Tx buffers (A single double buffer).
   Altough this ioctl tries to transmits the two buffers continuosly,
   there might be short delay between the two buffers in some cases
   (high baud rates / loaded system) */
#define P2PUART_IOCTL_UNITTEST_TRANSMIT_TWO_FULL_TX_BUFFERS                    \
	_IO(P2PUART_IOC_MAGIC, 0xe2)

/*
 * perform DPR read/write benchmarks, results are printed to kernel log.
 */
#define P2PUART_IOCTL_UNITTEST_BENCHMARK_TXBUF_WRITE                           \
	_IO(P2PUART_IOC_MAGIC, 0xf4)

#define P2PUART_IOCTL_UNITTEST_BENCHMARK_RXBUF_READ _IO(P2PUART_IOC_MAGIC, 0xf5)

#endif /* _P2PUART_IOCTLS_H_ */
