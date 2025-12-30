# Makefile for Serial Link Tester
#
# Usage:
#   make          - Build the serial tester
#   make clean    - Remove build artifacts
#   make install  - Install to /usr/local/bin (requires sudo)
#   make uninstall - Remove from /usr/local/bin

CC = gcc
CFLAGS = -O2 -Wall -Wextra
LDFLAGS = -pthread

TARGET = serial_tester
SRC = serial_tester.c

PREFIX = /usr/local/bin

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)
	@echo ""
	@echo "Build complete: ./$(TARGET)"
	@echo "Run with: ./$(TARGET) /dev/ttyUL0"

clean:
	rm -f $(TARGET)
	@echo "Cleaned."

install: $(TARGET)
	sudo cp $(TARGET) $(PREFIX)/
	sudo chmod 755 $(PREFIX)/$(TARGET)
	@echo "Installed to $(PREFIX)/$(TARGET)"

uninstall:
	sudo rm -f $(PREFIX)/$(TARGET)
	@echo "Uninstalled from $(PREFIX)/$(TARGET)"
