#!/bin/sh
# Build MATEKF405SE (sim-demo variant: ICM-42688P on SPI1/PA4, I2C1_SDA typo fixed,
# DCD_DevConnect after USBD_Init) with a newer GCC than the repo pins.
set -e
cd "$(dirname "$0")/.."
make -j6 TARGET=MATEKF405SE GCC_REQUIRED_VERSION="$(arm-none-eabi-gcc -dumpversion)" ARM_SDK_DIR="${ARM_SDK_DIR:-$HOME/opt/arm-gnu-toolchain}"
cp obj/main/EmuFlight_MATEKF405SE.elf sim-demo/ef.elf
