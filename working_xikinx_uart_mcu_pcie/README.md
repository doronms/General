# MCU PCIe Bridge and Xilinx UARTLite Drivers

This package provides Linux kernel drivers for accessing Xilinx UARTLite IP cores
over PCIe. The drivers create a standard TTY device (`/dev/ttyUL0`) that can be
used with minicom, screen, or any other serial terminal application.

## Files

- `mcu_pcie.c` - PCIe bridge driver that handles MSI interrupts and creates
  platform devices for FPGA peripherals
- `mcu_pcie.h` - Header file with shared definitions
- `xilinx_uartlite.c` - UARTLite driver providing the TTY interface
- `Makefile` - Build and utility commands

## Building

```bash
# Build the modules
make

# Clean build artifacts
make clean
```

## Loading Modules

```bash
# Load in correct order (mcu_pcie first, then xilinx_uartlite)
make load

# Or manually:
sudo insmod mcu_pcie.ko
sudo insmod xilinx_uartlite.ko

# Verify the device appeared
ls -la /dev/ttyUL*
```

## Using minicom

```bash
# Install minicom if needed
sudo dnf install minicom   # RHEL/Fedora
sudo apt install minicom   # Debian/Ubuntu

# Configure minicom
sudo minicom -s

# In the configuration menu:
# 1. Select "Serial port setup"
# 2. Press A and set serial device to /dev/ttyUL0
# 3. Press E and set baud rate to 115200 8N1
# 4. Press F to disable Hardware Flow Control (set to No)
# 5. Press G to disable Software Flow Control (set to No)
# 6. Press Enter to exit serial setup
# 7. Select "Save setup as dfl" to save as default
# 8. Select "Exit" to start minicom

# Or run directly with settings:
minicom -D /dev/ttyUL0 -b 115200
```

## Troubleshooting

### Check module loading status
```bash
make status
# Shows loaded modules, PCIe devices, tty devices, and recent kernel messages
```

### Check interrupt statistics
```bash
make debug
# Shows IRQ counts and registered interrupts
```

### View kernel messages
```bash
dmesg | grep -E "(mcu_pcie|uartlite|UARTLite)"
```

### Common Issues

1. **No /dev/ttyUL0 device**
   - Check if modules are loaded: `lsmod | grep -E "(mcu_pcie|xilinx_uartlite)"`
   - Check dmesg for errors: `dmesg | tail -50`
   - Verify PCIe device is detected: `lspci -d 10ee:`

2. **"Permission denied" when accessing /dev/ttyUL0**
   - Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Log out and back in
   - Or use sudo: `sudo minicom -D /dev/ttyUL0`

3. **No data received (but transmit works)**
   - Check FPGA UARTLite configuration (baud rate, FIFO depth)
   - Verify IRQ is firing: `make debug`
   - The driver falls back to polling if IRQs don't work

4. **"Cannot open /dev/ttyUL0: No such file or directory"**
   - Reload modules: `make reload`
   - Check for errors in dmesg

5. **Garbled data**
   - Verify baud rate matches FPGA configuration
   - UARTLite is fixed at synthesis time - adjust FPGA if needed
   - Ensure both ends use 8N1 (8 data bits, no parity, 1 stop bit)

6. **Module won't load - unknown symbol**
   - Make sure mcu_pcie.ko is loaded before xilinx_uartlite.ko
   - Use `make load` which handles the order automatically

### Manual UART Testing

```bash
# Set up the port
stty -F /dev/ttyUL0 115200 cs8 -cstopb -parenb raw -echo

# Send data
echo "Hello" > /dev/ttyUL0

# Receive data (in another terminal)
cat /dev/ttyUL0
```

### Checking FPGA Address Map

The FPGA peripheral addresses are defined in `mcu_pcie.c`:

```c
static const struct child_info children[] = {
    { .name = "fpga_regs",       .offset = 0x20000, ... },
    { .name = "max10_regs",      .offset = 0x30000, ... },
    { .name = "xilinx_uartlite", .offset = 0x70000, ... },
    ...
};
```

Adjust these offsets to match your FPGA design!

## Configuration

### Changing PCIe Device ID

Edit `mcu_pcie.c`:
```c
#define P2PUART_VENDOR_ID 0x10EE
#define P2PUART_DEVICE_ID 0x9021  // Change to your device ID
```

### Changing UARTLite Address

Edit the `children[]` array in `mcu_pcie.c`:
```c
{
    .name = "xilinx_uartlite",
    .offset = 0x70000,    // Change to your UARTLite address offset
    .size   = 0x1000,
    .needs_irq = true,
    .irq_idx   = 0,       // Which user interrupt line (0-15)
},
```

### Adding More UARTs

Add additional entries to `children[]` with unique offsets and IRQ indices.

## FPGA Requirements

The FPGA design must:
1. Implement Xilinx UARTLite IP at the configured address
2. Connect the UARTLite interrupt to an XDMA user interrupt line
3. Use XDMA IP for PCIe interface with MSI support enabled

UARTLite register map (offsets from base):
- 0x00: RX Data (read)
- 0x04: TX Data (write)
- 0x08: Status (read)
- 0x0C: Control (read/write)

## License

GPL v2
