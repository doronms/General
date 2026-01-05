# Makefile for user-space code

CC      = gcc
CFLAGS  = -Wall -Wextra -O2

# Our target (executable) name
TARGET  = fpga_regs_test

# The source code for the user-space program
SRC     = fpga_regs_test.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

