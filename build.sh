#!/bin/sh
# Run from the workspace root: sh build.sh
set -eu
cargo build --locked --release -p bootloader --target thumbv7m-none-eabi
elf=target/thumbv7m-none-eabi/release/bootloader
arm-none-eabi-size "$elf"
arm-none-eabi-objcopy -O binary "$elf" target/bootloader.bin
bytes=$(wc -c < target/bootloader.bin)
if [ "$bytes" -gt 8192 ]; then
    echo "bootloader.bin exceeds the 8192-byte boot region ($bytes bytes)" >&2
    exit 1
fi
printf 'Firmware: target/bootloader.bin (%s / 8192 bytes)\n' "$bytes"
