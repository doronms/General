#!/bin/sh

rmmod mcu_timer1ppm.ko
rmmod mcu_timer1pps.ko
rmmod xilinx_uartlite.ko
rmmod mcu_fpga.ko
rmmod mcu_max10.ko
rmmod mcu_pcie.ko

echo "Done unloading Drv"


