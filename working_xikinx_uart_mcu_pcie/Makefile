# Makefile for MCU PCIe Bridge and UARTLite drivers
#
# Usage:
#   make              - Build modules for running kernel
#   make clean        - Clean build artifacts
#   make install      - Install modules (requires root)
#   make load         - Load modules in correct order
#   make unload       - Unload modules
#   make reload       - Unload and reload modules

# Kernel build directory (adjust if needed)
KDIR ?= /lib/modules/$(shell uname -r)/build

# Module names
obj-m += mcu_pcie.o
obj-m += xilinx_uartlite.o

# Additional compiler flags
ccflags-y += -DDEBUG

# Default target
all: modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f *.o *.ko *.mod.c *.mod *.order *.symvers

install: modules
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# Load modules in the correct order
load:
	@echo "Loading mcu_pcie module..."
	sudo insmod ./mcu_pcie.ko
	@sleep 1
	@echo "Loading xilinx_uartlite module..."
	sudo insmod ./xilinx_uartlite.ko
	@echo "Modules loaded. Check dmesg for details."
	@echo "Device should appear at /dev/ttyUL0"

# Unload modules in reverse order
unload:
	@echo "Unloading modules..."
	-sudo rmmod xilinx_uartlite 2>/dev/null || true
	-sudo rmmod mcu_pcie 2>/dev/null || true
	@echo "Modules unloaded."

# Reload modules
reload: unload load

# Show status
status:
	@echo "=== Loaded modules ==="
	@lsmod | grep -E "(mcu_pcie|xilinx_uartlite)" || echo "No modules loaded"
	@echo ""
	@echo "=== PCIe devices ==="
	@lspci -d 10ee: 2>/dev/null || echo "No Xilinx PCIe devices found"
	@echo ""
	@echo "=== TTY devices ==="
	@ls -la /dev/ttyUL* 2>/dev/null || echo "No ttyUL devices"
	@echo ""
	@echo "=== Recent kernel messages ==="
	@dmesg | tail -30 | grep -E "(mcu_pcie|uartlite|UARTLite)" || echo "No relevant messages"

# Debug info
debug:
	@echo "=== IRQ Statistics ==="
	@cat /sys/kernel/debug/mcu_pcie/total_irqs 2>/dev/null || echo "N/A"
	@for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do \
		printf "IRQ %2d: " $$i; \
		cat /sys/kernel/debug/mcu_pcie/irq_0$$i 2>/dev/null || echo "N/A"; \
	done
	@echo ""
	@echo "=== Interrupts ==="
	@cat /proc/interrupts | grep -E "(mcu_pcie|uartlite)" || echo "No interrupts registered"

# Test the UART
test:
	@echo "=== Testing UART ==="
	@if [ -c /dev/ttyUL0 ]; then \
		echo "Setting up /dev/ttyUL0 at 115200 baud..."; \
		stty -F /dev/ttyUL0 115200 cs8 -cstopb -parenb raw -echo; \
		echo "Sending test message..."; \
		echo "Hello from ttyUL0" > /dev/ttyUL0; \
		echo "Test message sent. Use minicom or similar to receive."; \
	else \
		echo "ERROR: /dev/ttyUL0 not found!"; \
		echo "Make sure modules are loaded with 'make load'"; \
	fi

.PHONY: all modules clean install load unload reload status debug test
