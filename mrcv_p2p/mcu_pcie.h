#ifndef _MCU_PCIE_H_
#define _MCU_PCIE_H_

//#define FPGA_VENDOR_ID	0x1172
//#define FPGA_DEVICE_ID	0xe001

#define FPGA_VENDOR_ID	0x10ee
#define FPGA_DEVICE_ID	0x9014
#define MSI_DEVICE_ID	0x0007


#define FPGA_REG_ALIGN  4
#define MAX10_REG_ALIGN  4

#define PCIe_FPGA_WORD_OFFSET			4
#define UARTS_NUM									1
#define QUARTS_NUM								4

#define PCIE_CONTROL_REGISTER_BAR						2
#define PCIE_CARRIER_BOARD_UART_BAR						0
#define PCIE_CARRIER_BOARD_VIDEO_MATRIX_BAR					0
#define PCIE_CARRIER_BOARD_DIRECT_IO_BAR					0
#define PCIE_CARRIER_BOARD_CPLD_BAR						0  
#define PCIE_CARRIER_I2C_BAR                					0
#define PCIE_CARRIER_BOARD_CAN1_BAR						0
#define PCIE_CARRIER_BOARD_CAN2_BAR						0
#define PCIE_CARRIER_VIDEO_MATRIX_BAR					0
#define PCIE_CARRIER_BOARD_QUART_BAR					0

//#define PCIE_FIRMWARE_VERSION_OFFSET_IN_BAR		0x08000400
#define PCIE_CONTROL_REGISTER_OFFSET_IN_BAR		0x0
#define PCIE_DIRECT_IO_OFFSET_IN_BAR					0x00000000 
#define PCIE_P2PUART_IO_OFFSET_IN_BAR					0x00010000
//#define PCIE_UART2_OFFSET_IN_BAR							0x09000000
//#define PCIE_I2C_OFFSET_IN_BAR               	0x07000000
//#define PCIE_CPLD_OFFSET_IN_BAR								0x02000000
//#define PCIE_QUART_OFFSET_IN_BAR							0x02000000
//#define PCIE_CAN1_OFFSET_IN_BAR								0x08002000
//#define PCIE_CAN2_OFFSET_IN_BAR								0x08004000
#define PCIE_DEFAULT_ADDRESS_RANGE						0x1000
#define PCIE_CONTROL_ADDRESS_RANGE						0x60
#define PCIE_INT_STATUS_REGISTER_OFFSET			0x0040
#define PCIE_INT_ENABLE_REGISTER_OFFSET			0x0050
#define PCIE_MSI_ENABLE_MASK								0x80

#define PCIE_DEVICE_BASE_ADDRESS_OFFSET_VIDEO_MATRIX	0x15 * FPGA_REG_ALIGN // VM SPI ready in FPGA custom logic

extern unsigned int get_irq(void);
extern unsigned long get_uart_base_address(unsigned int uartno);
extern unsigned long get_quart_base_address(unsigned int uartno);
extern unsigned long get_fpga_regs_base_address(void);
extern unsigned long get_cpld_regs_base_address(void);
extern unsigned long get_firmware_version_base_address(void);

#endif

