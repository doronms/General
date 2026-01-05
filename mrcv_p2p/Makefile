obj-m += mcu_pcie.o
obj-m += mcu_fpga.o
obj-m += mcu_max10.o
obj-m += mcu_p2puart.o
obj-m += mcu_timer1ppm.o
obj-m += mcu_timer1pps.o

# These .o files all get linked into p2puart.ko
#p2puart-objs := mcu_p2puart.o

ccflags-y += -I$(src)
ccflags-y += -DP2PUART_IS_EXTERNAL_PLATFORM_DEVICE
ccflags-y += -DDISABLE_MEMORY_BUFFERS_QUICK_TEST
#ccflags-y += -DDEBUG_PRINT_FUNCTION_ENTRIES
#ccflags-y += -DDDEBUG_PRINT_TX_BUFFERS
#ccflags-y+= -DEBUG_PRINT_REGISTERS_ACCESS
#ccflags-y+= -DEBUG_PRINT_DEVICE_MAPPINGS
#ccflags-y+= -DEBUG_PRINT_FUNCTION_RETURNS
#ccflags-y+= -DEBUG_PRINT_BAUD_RATE_CALCS
#ccflags-y+= -DEBUG_PRINT_TX_COMMANDS
3ccflags-y+= -DEBUG_PRINT_RX_INTERRUPT -DDEBUG_PRINT_RX_BUFFERS_PORT_NUM=0
#ccflags-y+= -DEBUG_WITH_UNITTEST_IOCTLS
#ccflags-y+= -DEBUG_PRINT_RX_BUFFERS


KERNEL_BUILD := /lib/modules/$(shell uname -r)/build
KBUILD_CFLAGS += -g -Wall

.PHONY: all modules user clean

all: modules user

modules:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) clean
	rm -f $(USER_TARGET)
