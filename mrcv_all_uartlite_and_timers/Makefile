# Makefile for MCU PCIe Bridge and Child Drivers
#
# Usage:
#   make              - Build all modules
#   make clean        - Clean build artifacts
#   make install      - Install modules to /lib/modules
#   make load         - Load all modules in correct order
#   make unload       - Unload all modules in correct order
#   make reload       - Unload then load all modules
#   make status       - Show loaded modules and devices
#
# Individual module targets:
#   make mcu_pcie     - Build PCIe bridge only
#   make xilinx_uartlite - Build UARTLite driver only
#   make mcu_fpga     - Build FPGA registers driver only
#   make mcu_max10    - Build MAX10 registers driver only
#   make mcu_timer1pps - Build 1PPS timer driver only
#   make mcu_timer1ppm - Build 1PPM timer driver only

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# All module object files
obj-m += mcu_pcie.o
obj-m += xilinx_uartlite.o
obj-m += mcu_fpga.o
obj-m += mcu_max10.o
obj-m += mcu_timer1pps.o
obj-m += mcu_timer1ppm.o

# Extra CFLAGS
ccflags-y := -DDEBUG

.PHONY: all clean install load unload reload status help

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	@echo ""
	@echo "Build complete. Modules:"
	@ls -la *.ko 2>/dev/null || echo "  (no .ko files found)"

# Individual module builds
mcu_pcie:
	$(MAKE) -C $(KDIR) M=$(PWD) mcu_pcie.ko

xilinx_uartlite:
	$(MAKE) -C $(KDIR) M=$(PWD) xilinx_uartlite.ko

mcu_fpga:
	$(MAKE) -C $(KDIR) M=$(PWD) mcu_fpga.ko

mcu_max10:
	$(MAKE) -C $(KDIR) M=$(PWD) mcu_max10.ko

mcu_timer1pps:
	$(MAKE) -C $(KDIR) M=$(PWD) mcu_timer1pps.ko

mcu_timer1ppm:
	$(MAKE) -C $(KDIR) M=$(PWD) mcu_timer1ppm.ko

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f Module.symvers modules.order
	@echo "Cleaned."

install: all
	sudo $(MAKE) -C $(KDIR) M=$(PWD) modules_install
	sudo depmod -a
	@echo "Modules installed. Run 'make load' to load them."

# Load modules in dependency order
# mcu_pcie must be loaded first (provides IRQ infrastructure)
# Then child drivers can be loaded in any order
load:
	@echo "Loading MCU PCIe modules..."
	@sudo modprobe -r mcu_timer1ppm 2>/dev/null || true
	@sudo modprobe -r mcu_timer1pps 2>/dev/null || true
	@sudo modprobe -r mcu_max10 2>/dev/null || true
	@sudo modprobe -r mcu_fpga 2>/dev/null || true
	@sudo modprobe -r xilinx_uartlite 2>/dev/null || true
	@sudo modprobe -r mcu_pcie 2>/dev/null || true
	@sleep 0.5
	sudo insmod ./mcu_pcie.ko
	@sleep 0.5
	sudo insmod ./xilinx_uartlite.ko
	sudo insmod ./mcu_fpga.ko
	sudo insmod ./mcu_max10.ko
	sudo insmod ./mcu_timer1pps.ko
	sudo insmod ./mcu_timer1ppm.ko
	@echo ""
	@echo "Modules loaded. Checking status..."
	@$(MAKE) --no-print-directory status

# Unload modules in reverse order
unload:
	@echo "Unloading MCU PCIe modules..."
	-sudo rmmod mcu_timer1ppm 2>/dev/null
	-sudo rmmod mcu_timer1pps 2>/dev/null
	-sudo rmmod mcu_max10 2>/dev/null
	-sudo rmmod mcu_fpga 2>/dev/null
	-sudo rmmod xilinx_uartlite 2>/dev/null
	-sudo rmmod mcu_pcie 2>/dev/null
	@echo "Modules unloaded."

reload: unload
	@sleep 1
	@$(MAKE) --no-print-directory load

# Show status of modules and devices
status:
	@echo ""
	@echo "=== Loaded Modules ==="
	@lsmod | grep -E "mcu_|xilinx_uart" || echo "  (none loaded)"
	@echo ""
	@echo "=== TTY Devices ==="
	@ls -la /dev/ttyUL* 2>/dev/null || echo "  (no ttyUL devices)"
	@echo ""
	@echo "=== Misc Devices ==="
	@ls -la /dev/fpga_regs /dev/max10_regs /dev/timer1pps /dev/timer1ppm 2>/dev/null || echo "  (no misc devices)"
	@echo ""
	@echo "=== PCIe Device ==="
	@lspci -d 10ee:9021 2>/dev/null || echo "  (device not found)"
	@echo ""
	@echo "=== Recent dmesg ==="
	@dmesg | grep -E "mcu_pci|uartlite|fpga|max10|timer1pp" | tail -20

# Show help
help:
	@echo "MCU PCIe Driver Build System"
	@echo ""
	@echo "Targets:"
	@echo "  make              - Build all modules"
	@echo "  make clean        - Clean build artifacts"
	@echo "  make install      - Install to /lib/modules"
	@echo "  make load         - Load all modules"
	@echo "  make unload       - Unload all modules"
	@echo "  make reload       - Unload and reload all modules"
	@echo "  make status       - Show module and device status"
	@echo ""
	@echo "Individual modules:"
	@echo "  make mcu_pcie"
	@echo "  make xilinx_uartlite"
	@echo "  make mcu_fpga"
	@echo "  make mcu_max10"
	@echo "  make mcu_timer1pps"
	@echo "  make mcu_timer1ppm"
	@echo ""
	@echo "Devices created:"
	@echo "  /dev/ttyUL0-11    - UARTLite serial ports"
	@echo "  /dev/fpga_regs    - FPGA register access"
	@echo "  /dev/max10_regs   - MAX10 register access"
	@echo "  /dev/timer1pps    - 1PPS timer"
	@echo "  /dev/timer1ppm    - 1PPM timer"
