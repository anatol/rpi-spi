# rpi-spi serprog firmware

USB CDC `serprog` firmware for RP2040/RP2350 boards (for example Raspberry Pi Pico), optimized for SPI flash read/write with `flashrom`.

This project turns your board into a USB SPI programmer that `flashrom` can talk to through the `serprog` protocol.

## What you get

- USB CDC `serprog` interface (works with `flashrom -p serprog:...`)
- Hardware SPI backend (`spi0`) for good throughput
- Configurable SPI speed from host (`S_SPI_FREQ`)
- Optional safe-disconnect control pins:
  - `FLASH_ACTIVE_EN` for external gating (isolation and/or target power switch)
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

After reboot, the board appears as USB CDC serial devices.

### 3. Wire SPI

Default pinout (`spi0`):

- `GPIO17` -> flash `CS#`
- `GPIO18` -> flash `CLK`
- `GPIO19` -> flash `DI / MOSI`
- `GPIO16` -> flash `DO / MISO`
- `3V3` (or external regulator output) -> flash `VCC`
- `GND` -> target `GND`

Optional control pins:

- `GPIO20` -> `FLASH_ACTIVE_EN` (external gating enable)

Targeted use case for these pins:

- In-circuit programming of a flash chip that is still connected to its normal host SoC.
- Without isolation, the host SoC can keep driving `CS/CLK/MOSI/MISO` (or back-power parts of the board), which causes bus contention, unstable reads/writes, or possible damage.
- `FLASH_ACTIVE_EN` can drive one external control path that enables your isolation and/or target-power switch chain.
- Current firmware behavior ties `FLASH_ACTIVE_EN` to `S_PIN_STATE`:
  - `S_PIN_STATE=1` -> `FLASH_ACTIVE_EN` asserted
  - `S_PIN_STATE=0` -> `FLASH_ACTIVE_EN` deasserted
- Default behavior is inactive at boot (use a pull-down so it stays low until firmware explicitly asserts it).

How to use `FLASH_ACTIVE_EN` in hardware:

- Target-power gating:
  - Connect `FLASH_ACTIVE_EN` to the `EN` pin of a load switch/regulator feeding flash `VCC`.
  - `FLASH_ACTIVE_EN=0`: target flash power path off (safe idle/disconnected state).
  - `FLASH_ACTIVE_EN=1`: target flash power path on for read/write/verify operations.
- SPI-line isolation:
  - Connect `FLASH_ACTIVE_EN` to the `OE/EN` of analog switches or bus buffers inserted in `CS/SCK/MOSI/MISO`.
  - `FLASH_ACTIVE_EN=0`: SPI path open/high-Z between programmer and target host domain.
  - `FLASH_ACTIVE_EN=1`: SPI path connected so programmer can access the flash.
- Combined use:
  - You can fan out `FLASH_ACTIVE_EN` to both power-gating and SPI-isolation control inputs (if voltage domains are compatible).
  - This gives one control state for "programming active" and one for "electrically isolated/safe idle".

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

### 5. Use diagnostic console (`screen`)

Firmware exposes two USB serial interfaces:

- `serprog` port: for `flashrom`
- `diag-console` port: interactive diagnostics

#### Linux: enable non-root USB access (`udev`)

This repo includes [`99-rpi-spi.rules`](99-rpi-spi.rules) for USB VID:PID `2e8a:000a`.

Install it:

```bash
sudo cp 99-rpi-spi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug/replug the board after installing the rule.

The rule sets device mode to `0660`, assigns group `dialout`, and adds `uaccess`.
If needed, add your user to `dialout` and re-login:

```bash
sudo usermod -aG dialout "$USER"
```

On Linux you can identify ports by symlink name:

```bash
ls -l /dev/serial/by-id/
```

Look for entries containing `serprog` and `diag-console`.

Connect to the diagnostic console:

```bash
screen /dev/ttyACM1 115200
```

If `ttyACM1` is not the console, try the other enumerated port.

Exit screen:

- `Ctrl-a`, then `k`, then `y`

#### Console commands

- `help`: show command list
- `status`: show current SPI state and optional control-pin state
- `check`: run default diagnostic suite
- `check force`: run extended diagnostics including CS-effect check

#### What `check` verifies

- contention: detects if another domain appears to drive `CS/SCK/MOSI` while this firmware tri-states drivers
- control pins: validates that `FLASH_ACTIVE_EN` can toggle (if configured)
- flash detect: JEDEC-ID probe at conservative speed
- stability: repeated JEDEC reads for flaky connections/noise
- speed margin: low/mid/high speed probe comparison
- optional CS-effect (force mode): checks if CS changes target response as expected
- likely disconnected pin hints with confidence:
  - `likely_cs_disconnected` (`high`): CS-effect test shows identical behavior regardless of CS state
  - `likely_miso_disconnected` (`medium`): JEDEC bytes stay stuck at `0xFF`/`0x00`
  - `likely_clk_mosi_path_issue` (`low`): no JEDEC response plus no observed bus activity

`FLASH_ACTIVE_EN` is optional. If not connected or disabled at build time, diagnostics will skip related checks and continue.

#### Actionable report output

`check` prints a final report with:

- `Overall: PASS | WARN | FAIL`
- per-check evidence and likely cause
- confidence tags in likely-cause lines for pin-disconnect inference (`high`/`medium`/`low`)
- `Recommended Actions` in priority order
- `Next Validation` steps

Typical recommended actions include:

- lower `flashrom` speed (`spispeed=1M`, then `4M`, then `12M`)
- reseat clip, shorten wires, improve ground
- verify target flash power and voltage
- hold target SoC in reset or add SPI isolation hardware
- verify CS/SCK/MOSI/MISO mapping
- continuity-check suspected lines from programmer header to flash pins when a `likely_*_disconnected` finding appears

#### Concurrency behavior

- `flashrom` always owns the `serprog` port.
- diagnostics run on `diag-console`.
- diagnostics are intended to run when `flashrom` is not actively performing SPI operations.

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

If your hardware includes isolation and/or target power control tied to `GPIO20`:

- `S_PIN_STATE = 1`: SPI drivers active, `FLASH_ACTIVE_EN` asserted
- `S_PIN_STATE = 0`: SPI pins tri-stated, CS released, `FLASH_ACTIVE_EN` deasserted

This allows keeping clips connected while electrically disconnecting the programmer side.

## Custom pin mapping (compile-time)

You can override defaults at build time:

```bash
cmake -S . -B build \
  -DPICO_SDK_PATH=$PWD/pico-sdk \
  -DPICO_TINYUSB_PATH=$PWD/tinyusb \
  -DCMAKE_C_FLAGS='-DSP_PIN_CS=5 -DSP_PIN_SCK=6 -DSP_PIN_MOSI=7 -DSP_PIN_MISO=4 -DSP_PIN_FLASH_ACTIVE_EN=10'
cmake --build build -j"$(nproc)"
```

Useful flags:

- `SP_DEFAULT_SPI_HZ` (default startup SPI speed)
- `SP_PIN_FLASH_ACTIVE_EN=-1` to disable flash-active pin feature
- `SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH=0|1`

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
