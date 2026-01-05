# Minimal P2PUART Driver Makefile

obj-m += mcu_p2puart_minimal.o

# Kernel build directory (adjust as needed)
KDIR ?= /lib/modules/$(shell uname -r)/build

# Build flags
ccflags-y := -DDEBUG

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Install
install: all
	sudo cp mcu_p2puart_minimal.ko /lib/modules/$(shell uname -r)/extra/
	sudo depmod -a

# Load/Unload helpers
load:
	sudo insmod mcu_p2puart_minimal.ko

unload:
	sudo rmmod mcu_p2puart_minimal || true

reload: unload load

# Test helpers
test:
	@echo "=== Driver Info ===" 
	@lsmod | grep p2p || echo "Driver not loaded"
	@echo ""
	@echo "=== TTY Devices ==="
	@ls -la /dev/ttyP2P* 2>/dev/null || echo "No ttyP2P devices"
	@echo ""
	@echo "=== Interrupts ==="
	@cat /proc/interrupts | grep -i p2p || echo "No p2p interrupts"
	@echo ""
	@echo "=== Recent dmesg ==="
	@dmesg | grep -i p2p | tail -20

.PHONY: all clean install load unload reload test
