CC = gcc
INCLUDE =-I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib64/glib-2.0/include -I/usr/include/gdk-pixbuf-2.0 -I../xdma_drv/ -I./ -I./xil -I./xil/spi_v4_10/src
CFLAGS =  -O2 --std=c99 -Wall $(shell pkg-config --cflags gstreamer-1.0) $(INCLUDE)
LDFLAGS = $(shell pkg-config --libs gstreamer-1.0 gstreamer-video-1.0) -lX11 -lgstapp-1.0 -laio -lpthread -lcurses
OBJFILES = main.o si5324drv.o ./xil/xiic.o ./xil/xiic_l.o ./xil/xiic_stats.o \
./fzetta_fmc/fzetta_fmc_ctlr.o ./fzetta_fmc/fzetta_fmc_gpio.o ./fzetta_fmc/fzetta_fmc_iic.o ./fzetta_fmc/fzetta_fmc_init_table.o ./fzetta_fmc/fzetta_fmc_spi.o \
./xil/spi_v4_10/src/xspi.o ./xil/spi_v4_10/src/xspi_g.o ./xil/spi_v4_10/src/xspi_options.o ./xil/spi_v4_10/src/xspi_selftest.o ./xil/spi_v4_10/src/xspi_sinit.o ./xil/spi_v4_10/src/xspi_stats.o   

TARGET = fmc


all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)
clean:
	rm -f $(OBJFILES) $(TARGET) *~
