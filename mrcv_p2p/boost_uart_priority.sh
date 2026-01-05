#!/bin/bash

# 1. Target RT Priority (85 is higher than default kworkers/interrupts)
TARGET_PRIO=85
# 2. UART Device Prefix
DEV_PREFIX="/dev/ttyP2P"

echo "--- Starting RHEL 9 RT Latency Optimization ---"

# --- STEP 1: Threaded IRQ Priority Boosting ---
# Find all PIDs for the p2puart IRQ threads (currently at priority 50)
PIDS=$(ps -eo pid,comm | grep 'irq/.*-p2puart' | awk '{print $1}')

if [ -z "$PIDS" ]; then
    echo "Error: No p2puart IRQ threads found. Is the driver loaded?"
else
    for PID in $PIDS; do
        # Elevate priority to 85 using FIFO scheduling class
        chrt -f -p $TARGET_PRIO $PID
        echo "Elevated PID $PID ($(cat /proc/$PID/comm)) to RT Priority $TARGET_PRIO"
    done
fi

# --- STEP 2: TTY Low Latency Configuration ---
# This forces the kernel to push data to the user application immediately
# rather than waiting for internal TTY buffer thresholds.
for PORT in {0..11}; do
    DEVICE="${DEV_PREFIX}${PORT}"
    if [ -e "$DEVICE" ]; then
        setserial "$DEVICE" low_latency
        echo "Set $DEVICE to Low Latency mode."
    fi
done

# --- STEP 3: Power Management / CPU DMA Latency ---
# Prevents CPUs from entering deep sleep states (C-states) which 
# adds microseconds of wake-up delay during the 5ms window.
if [ -e "/dev/cpu_dma_latency" ]; then
    echo "Note: To lock CPU DMA latency, keep this script running or use tuned-adm."
fi

echo "--- Optimization Complete ---"
