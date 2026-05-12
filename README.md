# rpi-spi serprog firmware

USB CDC `serprog` firmware for RP2040/RP2350 boards (for example Raspberry Pi Pico), optimized for SPI flash read/write with `flashrom`.

This project turns your board into a USB SPI programmer that `flashrom` can talk to through the `serprog` protocol.

## What you get

- USB CDC `serprog` interface (works with `flashrom -p serprog:...`)
- Hardware SPI backend (`spi0`) for good throughput
- Configurable SPI speed from host (`S_SPI_FREQ`)
- Optional safe-disconnect control pins:
  - `ISOLATE_EN` for external bus switch/buffer
  - `TARGET_PWR_EN` for target power switch
- Build helper script that fetches required dependencies

## Quick start

### 1. Build

Requirements:

- `git`
- `cmake`
- C/C++ toolchain for Pico SDK builds

Run:

```bash
./build.sh
```

This script:

- clones `pico-sdk` into `./pico-sdk` (if missing)
- clones `tinyusb` into `./tinyusb` (if missing)
- initializes Pico SDK submodules
- builds firmware into `./build`

Output files:

- `build/rpi-spi.uf2`
- `build/rpi-spi.elf`

### 2. Flash firmware to board

1. Hold `BOOTSEL` while plugging in the board.
2. A USB mass storage drive appears (usually `RPI-RP2`).
3. Copy UF2:

```bash
cp build/rpi-spi.uf2 /media/$USER/RPI-RP2/
```

After reboot, the board appears as a USB CDC serial device.

### 3. Wire SPI

Default pinout (`spi0`):

- `GPIO17` -> flash `CS#`
- `GPIO18` -> flash `CLK`
- `GPIO19` -> flash `DI / MOSI`
- `GPIO16` -> flash `DO / MISO`
- `3V3` (or external regulator output) -> flash `VCC`
- `GND` -> target `GND`

Optional control pins:

- `GPIO20` -> `ISOLATE_EN` (external switch/buffer enable)
- `GPIO21` -> `TARGET_PWR_EN` (external target VCC switch enable)

Targeted use case for these pins:

- In-circuit programming of a flash chip that is still connected to its normal host SoC.
- Without isolation, the host SoC can keep driving `CS/CLK/MOSI/MISO` (or back-power parts of the board), which causes bus contention, unstable reads/writes, or possible damage.
- `ISOLATE_EN` is intended to control external analog switches/buffers so the programmer can cleanly connect/disconnect SPI lines.
- `TARGET_PWR_EN` is intended to control an external load switch so the target flash domain can be power-cycled or held off while isolating.
- These pins serve different electrical roles: `ISOLATE_EN` controls SPI signal path isolation, while `TARGET_PWR_EN` controls target VCC power gating.
- Current firmware behavior ties both to `S_PIN_STATE` (enabled together when `S_PIN_STATE=1`, disabled together when `S_PIN_STATE=0`).
- Together, they let you keep a clip attached while switching between "safe disconnected" and "active programming" states.

Important:

- The flash must be powered (`VCC` + `GND`) to respond on SPI.
- Share ground between programmer and target.
- Confirm voltage compatibility before connecting (many SPI flashes are 3.3V-only).
- Do not drive `CS/CLK/MOSI/MISO` into an unpowered target flash (can back-power through IO protection paths).
- For in-circuit programming, isolation hardware is strongly recommended.

### 4. Use with flashrom

Find your serial device:

- Linux: usually `/dev/ttyACM0` (or `/dev/ttyACM1`, ...)
- macOS: usually `/dev/cu.usbmodem*`
- Windows: `COMx`

Read:

```bash
flashrom -p serprog:dev=/dev/ttyACM0,spispeed=12M -r backup.bin
```

Write:

```bash
flashrom -p serprog:dev=/dev/ttyACM0,spispeed=12M -w image.bin
```

Verify:

```bash
flashrom -p serprog:dev=/dev/ttyACM0,spispeed=12M -v image.bin
```

## Speed tuning

Start conservative, then increase:

- try `spispeed=12M` first
- then `24M`, `32M`, etc. if stable

If you see read/verify errors:

- shorten wires
- improve grounding
- reduce SPI speed
- add proper bus isolation for in-circuit use

Note: firmware returns the nearest supported SPI clock to host requests.

## Safer in-circuit workflow

If your hardware includes isolation and/or target power control tied to `GPIO20/21`:

- `S_PIN_STATE = 1`: SPI drivers active, optional isolate/power enabled
- `S_PIN_STATE = 0`: SPI pins tri-stated, CS released, optional isolate/power disabled

This allows keeping clips connected while electrically disconnecting the programmer side.

## Custom pin mapping (compile-time)

You can override defaults at build time:

```bash
cmake -S . -B build \
  -DPICO_SDK_PATH=$PWD/pico-sdk \
  -DPICO_TINYUSB_PATH=$PWD/tinyusb \
  -DCMAKE_C_FLAGS='-DSP_PIN_CS=5 -DSP_PIN_SCK=6 -DSP_PIN_MOSI=7 -DSP_PIN_MISO=4 -DSP_PIN_ISOLATE_EN=10 -DSP_PIN_TARGET_PWR=11'
cmake --build build -j"$(nproc)"
```

Useful flags:

- `SP_DEFAULT_SPI_HZ` (default startup SPI speed)
- `SP_PIN_ISOLATE_EN=-1` to disable isolate pin feature
- `SP_PIN_TARGET_PWR=-1` to disable target power pin feature
- `SP_PIN_ISOLATE_EN_ACTIVE_HIGH=0|1`
- `SP_PIN_TARGET_PWR_ACTIVE_HIGH=0|1`

## USB enumeration compatibility option

In `tusb_config.h`:

```c
#define TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX 0
```

What this workaround targets:

- RP2040 B0/B1 can fail to enumerate on some host/hub combinations after USB bus reset.
- In Pico SDK this is implemented as a "brute force workaround for USB device enumeration issue" (`pico_fix/rp2040_usb_device_enumeration`).
- The fix waits for reset (`SE0`) to end, then forces a valid `LS_J` state for ~1 ms so the controller can enter the connected state reliably.

When to enable:

- You see intermittent "device not recognized" / missing CDC port after reconnect, reboot, or flashing.
- Enumeration is less reliable when connected through certain USB hubs/docks.

Tradeoff:

- Current default (`0`) favors max throughput/lowest overhead in stable setups.
- Set to `1` if you need better plug/replug enumeration robustness, then rebuild.

## Troubleshooting

- Device not found by `flashrom`:
  - confirm board enumerates as CDC serial device
  - confirm `dev=` path or `COM` port is correct
  - verify no other process has the serial port open
- `flashrom` can connect but read/write fails:
  - check wiring order (`CS/SCK/MOSI/MISO`)
  - check target voltage and ground
  - lower `spispeed`
- In-circuit target behaves strangely:
  - add/verify bus isolation hardware
  - ensure target host SoC is not actively driving the SPI bus

## Credits / lineage

Related projects reviewed during development:

- https://github.com/stacksmashing/pico-serprog
- https://github.com/kukrimate/pico-serprog
