#!/bin/sh

insmod mcu_pcie.ko
insmod mcu_fpga.ko
insmod mcu_max10.ko
insmod xilinx_uartlite.ko
insmod mcu_timer1ppm.ko
insmod mcu_timer1pps.ko
sleep 1
chmod 777 /dev/ttyUL*
sync
sleep 1
chmod 777 /dev/fpga_regs
chmod 777 /dev/max10_regs
chmod 777 /dev/timer1pps
chmod 777 /dev/timer1ppm

echo "Done loading Drv"


