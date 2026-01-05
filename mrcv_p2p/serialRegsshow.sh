#!/bin/bash

# Define the base address and number of registers to read
BASE_ADDR=0x80470000
NUM_REGS=12
# Assuming 32-bit registers (4 bytes each)
OFFSET_STEP=4

echo "Reading $NUM_REGS 32-bit registers starting from $BASE_ADDR..."

# Loop 10 times (from 0 to 9)
for i in $(seq 0 $((NUM_REGS - 1)))
do
  # Calculate the address offset (i * OFFSET_STEP)
  # Bash arithmetic is handled within (( ... ))
  offset=$((i * OFFSET_STEP))
  
  # Calculate the current physical address by adding the offset to the base address
  # We convert hex strings to decimal for calculation and back to hex for printing/devmem2
  current_addr=$(printf "0x%x" $(($BASE_ADDR + $offset)))
  
  echo "--- Reading register $i at address: $current_addr ---"
  
  # Execute devmem with "w" for word (32-bit) access
  # Note: devmem often requires sudo to access /dev/mem
  sudo ./devmem $current_addr w
  
  echo "" # Add a newline for better formatting
done

echo "Finished reading registers."

