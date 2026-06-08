# rpi-spi serprog firmware

USB CDC `serprog` firmware for RP2040/RP2350 boards (for example Raspberry Pi Pico), optimized for SPI flash read/write with `flashrom`.

This project turns your board into a USB SPI programmer that `flashrom` can talk to through the `serprog` protocol, and also exposes a simultaneous UART console bridge.

## What you get

- USB CDC `serprog` interface (works with `flashrom -p serprog:...`)
- USB CDC `uart-console` bridge (USB <-> target UART TX/RX)
- Hardware SPI backend (`spi0`) for good throughput
- Configurable SPI speed from host (`S_SPI_FREQ`)
- Non-blocking RGB status LED protocol (onboard WS2812 on `GPIO16` by default)
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

The default build target is `waveshare_rp2040_zero`, including its onboard
WS2812 on `GPIO16`. Override it for another Pico SDK board when needed:

```bash
PICO_BOARD=pico ./build.sh
```

Diagnostic console is disabled by default. To enable it for a build:

```bash
SPI_DEBUG_CONSOLE=1 ./build.sh
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

### Status LED protocol

The default LED is the onboard WS2812 on RP2040 Zero-compatible boards (`GPIO16`).
Animations run from the main polling path and do not intentionally delay SPI or UART traffic.

| Color / pattern | Meaning |
| --- | --- |
| White, 3 blinks | Firmware booted and initialized the status LED |
| Blue, 2 blinks | Host opened the `serprog` USB interface |
| Yellow, 2 blinks | SPI drivers and optional flash gate were enabled |
| Magenta, 2 blinks | SPI pins were isolated / tri-stated |
| Green, 1 blink | SPI transaction traffic |
| Cyan, 2 blinks | UART traffic in either direction |
| Red, 3 blinks | Unsupported or rejected serprog operation |

The dim steady idle color summarizes current state: purple means USB is not mounted,
blue means USB is mounted, cyan means only the UART console is open, green means
serprog is open with SPI enabled, and magenta means serprog is open with SPI isolated.
Traffic indications are rate-limited so sustained transfers remain visible without
continually restarting an animation.

Other useful indications to add later are diagnostic-test progress/result, flash
erase/write/verify phases (if exposed by a higher-level protocol), target power-good,
and detected SPI bus contention. Those states are not reliably distinguishable from
the current low-level serprog byte stream.

### 3. Wire SPI

Default pinout (`spi0`):

- `GPIO1` -> flash `CS#`
- `GPIO2` -> flash `CLK`
- `GPIO3` -> flash `DI / MOSI`
- `GPIO0` -> flash `DO / MISO`
- `3V3` (or external regulator output) -> flash `VCC`
- `GND` -> target `GND`

UART bridge pinout (default, RP2040 Zero friendly):

- `GPIO8` -> target UART `RX` (this board drives TX on GPIO8)
- `GPIO9` -> target UART `TX` (this board reads RX on GPIO9)

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

With default build settings, firmware exposes two USB serial interfaces:

- `serprog` port: for `flashrom`
- `uart-console` port: raw UART bridge to `GPIO8/GPIO9`

When built with `SPI_DEBUG_CONSOLE=1`, a third interface is added:

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

Look for entries containing `serprog` and `uart-console`.
If built with `SPI_DEBUG_CONSOLE=1`, you will also see `diag-console`.

If built with `SPI_DEBUG_CONSOLE=1`, connect to the diagnostic console:

```bash
screen /dev/ttyACM1 115200
```

If `ttyACM2` is not the diagnostic console, try the other enumerated ports by name.

Connect to UART bridge console:

```bash
screen /dev/ttyACM1 115200
```

Use the `uart-console` by-id symlink for stable naming when possible.
This is a transparent bridge to the target UART on `GPIO8/GPIO9`. The
programmer firmware does not print a startup banner on `uart-console`, so it
stays silent until the target sends data. Messages such as "connected to
/dev/ttyACM1" come from the host terminal tool, not from this firmware.

Use a build with `SPI_DEBUG_CONSOLE=1` and open `diag-console` for
firmware-generated status output.

Exit screen:

- `Ctrl-a`, then `k`, then `y`

#### Console commands

- `help`: show command list
- `info`: show firmware/build/board identifiers and configured pin map
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
- UART bridge runs on `uart-console` continuously, including during SPI flashing.
- diagnostics run on `diag-console` when built with `SPI_DEBUG_CONSOLE=1`.
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
- `SP_PIN_UART_TX` / `SP_PIN_UART_RX` (UART bridge pins)
- `SP_DEFAULT_UART_BAUD` (default UART bridge speed; host can change by serial port settings)
- `SP_PIN_FLASH_ACTIVE_EN=-1` to disable flash-active pin feature
- `SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH=0|1`
- `SP_STATUS_LED_PIN` (default `16`, onboard RP2040 Zero WS2812)
- `SP_STATUS_LED_ENABLED=0|1` (disable for boards without a WS2812)

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
- UART bridge is silent:
  - `uart-console` carries target UART traffic only; it does not contain
    programmer logs
  - cross TX/RX: programmer `GPIO8` to target RX and target TX to programmer
    `GPIO9`, with a shared ground
  - confirm the target uses the selected baud rate and 8-N-1 framing
  - temporarily connect programmer `GPIO8` directly to `GPIO9`; typed data
    should echo back through `uart-console`
- UART output becomes corrupt when the target resets or switches power:
  - confirm the programmer and target retain a solid shared ground
  - avoid routing UART beside relay coils, contacts, or switched power wiring
  - the firmware biases RX to the idle-high state and drops bytes carrying
    hardware framing, parity, break, or overrun errors
- No status LED at boot:
  - the default `waveshare_rp2040_zero` build blinks its GPIO16 WS2812 white
    three times immediately after reset, before a terminal is opened
  - confirm `build/rpi-spi.uf2` was flashed and `PICO_BOARD` matches the board
  - boards without a GPIO16 WS2812 need the correct `SP_STATUS_LED_PIN`, or
    `SP_STATUS_LED_ENABLED=0`
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
