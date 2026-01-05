#ifndef _FPGA_REGS_H_
#define _FPGA_REGS_H_

#include "mcu_pcie.h"

#define SET_MAX10_REGS 0xD
#define GET_MAX10_REGS 0xE
#define MAX10_MAX_REGS 1000

#define MAX10_UART_INT_MASK_REG_ADDRESS_OFFSET			(0x3A * MAX10_REG_ALIGN)
#define MAX10_UART_INT_STATUS_REG_ADDRESS_OFFSET		(0x3B * MAX10_REG_ALIGN)
#define MAX10_QUART_GLOBAL_INT_MASK_REG_ADDRESS_OFFSET		(0x3C * MAX10_REG_ALIGN)
#define MAX10_QUART_GLOBAL_INT_STATUS_REG_ADDRESS_OFFSET	(0x3D * MAX10_REG_ALIGN)

struct max10regs {
    unsigned int offset;
    unsigned int length;
    unsigned int __user *pregs; // User-space pointer
};


#endif

