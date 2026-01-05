#ifndef _FPGA_REGS_H_
#define _FPGA_REGS_H_

#include "mcu_pcie.h"

#define SET_FPGA_REGS 0xA
#define GET_FPGA_REGS 0xB
#define FPGA_MAX_REGS 1000

#define FPGA_UART_INT_MASK_REG_ADDRESS_OFFSET			(0x3A * FPGA_REG_ALIGN)
#define FPGA_UART_INT_STATUS_REG_ADDRESS_OFFSET			(0x3B * FPGA_REG_ALIGN)
#define FPGA_QUART_GLOBAL_INT_MASK_REG_ADDRESS_OFFSET		(0x3C * FPGA_REG_ALIGN)
#define FPGA_QUART_GLOBAL_INT_STATUS_REG_ADDRESS_OFFSET		(0x3D * FPGA_REG_ALIGN)

struct fpgaregs {
    unsigned int offset;
    unsigned int length;
    unsigned int __user *pregs; // User-space pointer
};


extern unsigned long get_video_matrix_base_address(void);
extern unsigned int get_uart_int_mask_register(void);
extern void set_uart_int_mask_register(unsigned int reg);
extern unsigned int get_uart_int_status_register(void);
extern unsigned int get_quart_global_int_mask_register(void);
extern void set_quart_global_int_mask_register(unsigned int reg);
unsigned int get_quart_global_int_status_register(void);
extern unsigned int get_fpga_register(unsigned char reg_ofs);
extern void set_fpga_register(unsigned int reg_ofs, unsigned int value);

#endif

