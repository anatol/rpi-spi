#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

if [[ ! -d pico-sdk ]]; then
  git clone https://github.com/raspberrypi/pico-sdk.git
fi

if [[ ! -d tinyusb ]]; then
  git clone https://github.com/hathach/tinyusb.git
fi

# Keep SDK submodules available (CMSIS, hardware headers, etc.).
git -C pico-sdk submodule update --init

mkdir -p build
SPI_DEBUG_CONSOLE="${SPI_DEBUG_CONSOLE:-0}"
cmake -S . -B build \
  -DPICO_SDK_PATH="$ROOT_DIR/pico-sdk" \
  -DPICO_TINYUSB_PATH="$ROOT_DIR/tinyusb" \
  -DSP_ENABLE_DIAG_CONSOLE="$SPI_DEBUG_CONSOLE"
cmake --build build -j"$(nproc)"
