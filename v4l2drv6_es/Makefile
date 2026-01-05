obj-m += v4l_xdma.o

v4l_xdma-objs := \
	v4l_xdma_main.o libxdma.o xdma_thread.o kcu116_hw.o si5324drv.o video_out.o xvtc.o video_in.o video_dma.o switch.o\
	fzetta_fmc/fzetta_fmc_ctlr.o \
	fzetta_fmc/fzetta_fmc_gpio.o \
	fzetta_fmc/fzetta_fmc_iic.o \
	fzetta_fmc/fzetta_fmc_init_table.o \
	fzetta_fmc/fzetta_fmc_spi.o \
	xil/spi_v4_10/src/xspi.o \
	xil/spi_v4_10/src/xspi_g.o \
	xil/spi_v4_10/src/xspi_options.o \
	xil/spi_v4_10/src/xspi_selftest.o \
	xil/spi_v4_10/src/xspi_sinit.o \
	xil/spi_v4_10/src/xspi_stats.o \
	xil/iic_v3_9/src/xiic.o \
	xil/iic_v3_9/src/xiic_dyn_master.o \
	xil/iic_v3_9/src/xiic_g.o \
	xil/iic_v3_9/src/xiic_intr.o \
	xil/iic_v3_9/src/xiic_l.o \
	xil/iic_v3_9/src/xiic_master.o \
	xil/iic_v3_9/src/xiic_multi_master.o \
	xil/iic_v3_9/src/xiic_options.o \
	xil/iic_v3_9/src/xiic_selftest.o \
	xil/iic_v3_9/src/xiic_sinit.o \
	xil/iic_v3_9/src/xiic_slave.o \
	xil/iic_v3_9/src/xiic_stats.o \
	xil/sdi_common_v1_3/src/xv_sdivid.o \
	xil/video_common_v4_13/src/xvidc.o \
	xil/video_common_v4_13/src/xvidc_timings_table.o \
	xil/gpio_v4_9/src/xgpio.o \
	xil/gpio_v4_9/src/xgpio_extra.o \
	xil/gpio_v4_9/src/xgpio_g.o \
	xil/gpio_v4_9/src/xgpio_intr.o \
	xil/gpio_v4_9/src/xgpio_selftest.o \
	xil/gpio_v4_9/src/xgpio_sinit.o

KDIR := /lib/modules/$(shell uname -r)/build
EXTRA_MOD_DIR := /lib/modules/$(shell uname -r)/extra
CONF_DIR := /etc/modules-load.d
CONF_FILE := v4l_xdma.conf

EXTRA_CFLAGS := -I$(PWD) \
                -I$(PWD)/xil \
                -I$(PWD)/xil/gpio_v4_9/src \
                -I$(PWD)/xil/iic_v3_9/src \
                -I$(PWD)/xil/spi_v4_10/src \
                -I$(PWD)/xil/video_common_v4_13/src \
                -I$(PWD)/xil/sdi_common_v1_3/src \
                -I$(PWD)/fzetta_fmc

.PHONY: all build install load_man unload_man uninstall clean

all: build

build:
	make -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

install:
	sudo mkdir -p $(EXTRA_MOD_DIR)
	sudo cp v4l_xdma.ko $(EXTRA_MOD_DIR)/
	echo v4l_xdma | sudo tee $(CONF_FILE) > /dev/null
	sudo mv $(CONF_FILE) $(CONF_DIR)/
	sudo depmod

load_man:
	sudo mkdir -p $(EXTRA_MOD_DIR)
	sudo cp v4l_xdma.ko $(EXTRA_MOD_DIR)/
	sudo depmod
	sudo modprobe -r v4l_xdma || true
	sudo modprobe v4l_xdma

unload_man:
	sudo modprobe -r v4l_xdma || true

uninstall:
	sudo rm -f $(EXTRA_MOD_DIR)/v4l_xdma.ko
	sudo rm -f $(CONF_DIR)/$(CONF_FILE)
	sudo depmod
	sudo modprobe -r v4l_xdma || true
	@echo "v4l_xdma module uninstalled."

clean:
	make -C $(KDIR) M=$(PWD) clean
