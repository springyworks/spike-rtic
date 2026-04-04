#!/bin/bash
# Flash RTIC firmware to LEGO SPIKE Prime Hub via DFU.
#
# Prerequisites:
#   - dfu-util installed (apt install dfu-util)
#   - Hub in DFU mode (hold center button during power-on, or type 'dfu' in shell)
#   - Hub connected via USB
#
# Usage:
#   ./flash.sh              # build release + flash
#   ./flash.sh --debug      # build debug + flash

set -e

PROFILE="release"
if [ "$1" = "--debug" ]; then
    PROFILE="debug"
    cargo build
else
    cargo build --release
fi

BIN_NAME="spike-rtic"
ELF="target/thumbv7em-none-eabihf/${PROFILE}/${BIN_NAME}"

# Convert ELF to raw binary
arm-none-eabi-objcopy -O binary "$ELF" /tmp/spike-rtic.bin
SIZE=$(wc -c < /tmp/spike-rtic.bin)
echo "Firmware size: ${SIZE} bytes"

# Flash via DFU (LEGO DFU bootloader)
# Address 0x08008000 = after 32K LEGO DFU bootloader
# VID:PID 0694:0011 = LEGO Powered Up Hub in DFU mode
echo "Flashing to 0x08008000 via DFU..."
dfu-util -d 0694:0011 -a 0 -s 0x08008000:leave -D /tmp/spike-rtic.bin

echo "Done! Hub will reset and run the firmware."
