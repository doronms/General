#ifndef _MCU_PCIE_H_
#define _MCU_PCIE_H_

/*
 * mcu_pcie.h - Shared declarations for MCU PCIe bridge driver
 *
 * Include this header in child drivers that need to interact with
 * the PCIe bridge interrupt system.
 */

#include <linux/notifier.h>
#include <linux/device.h>

/*
 * PCIe Device IDs
 */
#define FPGA_VENDOR_ID          0x10EE
#define FPGA_DEVICE_ID          0x9021

/*
 * FPGA register alignment
 */
#define FPGA_REG_ALIGN          4
#define MAX10_REG_ALIGN         4

/*
 * BAR assignments
 */
#define PCIE_BAR_FPGA_PERIPHERALS       0   /* BAR0: FPGA peripheral registers */
#define PCIE_BAR_XDMA_CONFIG            1   /* BAR1: XDMA/IRQ configuration */

/*
 * Child device offsets in BAR0
 * Adjust these according to your FPGA address map
 */
#define PCIE_OFFSET_FPGA_REGS           0x20000
#define PCIE_OFFSET_MAX10_REGS          0x30000
#define PCIE_OFFSET_UARTLITE            0x70000

/*
 * XDMA IRQ register offsets in BAR1
 */
#define XDMA_IRQ_USER_INT_EN            0x2004
#define XDMA_IRQ_USER_INT_DIS           0x2008
#define XDMA_IRQ_USER_INT_PENDING       0x2048
#define XDMA_IRQ_USER_VEC_BASE          0x2080
#define XDMA_IRQ_USER_VEC_STRIDE        0x4

/*
 * Number of user interrupt lines
 */
#define XDMA_IRQ_LINES                  16

/*
 * Functions exported by mcu_pcie driver
 */

/**
 * mcu_pcie_register_irq_handler - Register for IRQ notifications
 * @dev: Your platform device
 * @nb: Notifier block (callback will receive pending IRQ bitmap as 'action')
 *
 * Returns 0 on success, negative error code on failure.
 *
 * Your callback should check if your IRQ bit is set:
 *   if (pending & BIT(your_irq_idx)) { handle_irq(); }
 */
extern int mcu_pcie_register_irq_handler(struct device *dev, 
                                         struct notifier_block *nb);

/**
 * mcu_pcie_unregister_irq_handler - Unregister IRQ notifications
 * @dev: Your platform device
 * @nb: Previously registered notifier block
 */
extern void mcu_pcie_unregister_irq_handler(struct device *dev,
                                            struct notifier_block *nb);

/**
 * mcu_pcie_get_pending - Read pending IRQ bitmap (doesn't clear)
 * @dev: Your platform device
 *
 * Returns current pending interrupt bitmap from hardware.
 */
extern u32 mcu_pcie_get_pending(struct device *dev);

/**
 * xdma_get_and_clear_pending - Read and clear pending IRQ bitmap
 * @dev: Your platform device
 *
 * Returns pending interrupt bitmap and clears it in hardware.
 * Legacy function for compatibility with older drivers.
 */
extern u32 xdma_get_and_clear_pending(struct device *dev);

#endif /* _MCU_PCIE_H_ */
