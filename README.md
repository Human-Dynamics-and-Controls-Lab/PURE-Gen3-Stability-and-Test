# PURE Gen3 Stability and Test

Embedded control firmware for the PURE Gen3 stability mechanism and the
balancing prototype. The code runs on the RoboMaster Development Board Type C
and uses ChibiOS/RT with C and C++.

This repository is a development copy of
[`tyh4n/Meta-Embedded`](https://github.com/tyh4n/Meta-Embedded), focused on the
following applications:

| Target | Purpose | Primary hardware interfaces |
| --- | --- | --- |
| `STABILITY_SYSTEM` | Four-leg positioning, homing, inverse kinematics, and TI host communication | 8 M2006 motors, CAN1/CAN2, SPI2, Board C user button |
| `BALANCING_PROTOTYPE` | Single-axis balancing research platform | GM6020 motor, onboard IMU/AHRS, CAN1 |

The complete Meta-Embedded source tree is retained so the two applications can
continue to share ChibiOS, CAN, AHRS, shell, and build infrastructure. Other
vehicle applications are currently out of scope and should remain unchanged
unless shared infrastructure requires a compatible update.

## Applications

### Stability System

`STABILITY_SYSTEM` controls four independent linkage legs. Each leg uses two
M2006 motors and has an independent Cartesian `(r, h)` position in millimeters.
The application converts each target through inverse kinematics and commands
the associated motors through a CAN position loop.

The configured Cartesian workspace is:

```text
r: 272.61 to 385.14 mm
h: 0.00 to 50.00 mm
```

SPI coordinates use 0.1 mm resolution. The STM32 sends and receives each value
as the coordinate multiplied by 10.

Homing behavior:

1. After power-up, motors remain inactive and the status LED is red.
2. Press the Board C user button (`PA0`) once to start the initial homing cycle.
3. The mechanism drives toward its mechanical limits, detects stall, backs off
   using the existing position-loop sequence, and establishes the new zero point.
4. When homing succeeds, the LED turns green and SPI communication becomes ready.
5. After initialization, a TI `HOME` command runs the same homing cycle. Repeated
   HOME frames during a cycle do not start additional cycles.
6. Position targets are rejected while the system is unhomed.

The key implementation is in
[`main_stability_system.cpp`](dev/application/vehicles/stability_system/main_stability_system.cpp),
with motor and PID configuration in
[`can_motor_config.cpp`](dev/application/vehicles/stability_system/can_motor_config.cpp).

### TI to STM32 SPI Protocol

The TI LaunchPad is the SPI master and the STM32 is the SPI slave. The current
STM32 configuration uses 16-bit transfers, SPI mode 0, and MSB-first word order.
Both sides must use the same mode, clock polarity/phase, and common ground. The
intended host update rate is 20 Hz (50 ms per cycle).

Each transfer contains 12 16-bit words:

| Word | Field | Description |
| ---: | --- | --- |
| 0 | `magic` | Frame marker, `0xA55A` |
| 1 | `seq` | Host transaction sequence number |
| 2 | `command/status` | Command in the high byte, response status in the low byte |
| 3 | `r1x10` | Leg 1 `r` coordinate multiplied by 10 |
| 4 | `h1x10` | Leg 1 `h` coordinate multiplied by 10 |
| 5 | `r2x10` | Leg 2 `r` coordinate multiplied by 10 |
| 6 | `h2x10` | Leg 2 `h` coordinate multiplied by 10 |
| 7 | `r3x10` | Leg 3 `r` coordinate multiplied by 10 |
| 8 | `h3x10` | Leg 3 `h` coordinate multiplied by 10 |
| 9 | `r4x10` | Leg 4 `r` coordinate multiplied by 10 |
| 10 | `h4x10` | Leg 4 `h` coordinate multiplied by 10 |
| 11 | `crc16` | CRC over words 0 through 10 |

The CRC uses initial value `0xFFFF`, polynomial `0xA001`, and processes each
16-bit word least-significant bit first.

Commands:

| Value | Command | Meaning |
| ---: | --- | --- |
| 0 | `NONE` | No operation |
| 1 | `SET_TARGET` | Set all four independent `r/h` targets |
| 2 | `QUERY` | Request readiness and current leg positions |
| 3 | `HOME` | Start one homing cycle |

Response status:

| Value | Status | Meaning |
| ---: | --- | --- |
| 0 | `UNHOMED` | Homing has not completed; position targets must not be applied |
| 1 | `READY` | Position and readiness data are valid; targets may be applied |
| 2 | `INVALID` | The request failed validation |

TI should send `QUERY` until it observes `READY`, then send `SET_TARGET` at the
configured rate. If a HOME cycle is requested, the STM32 reports `UNHOMED` while
the mechanism is moving and returns to `READY` only after all four homing
threads complete.

### Balancing Prototype

`BALANCING_PROTOTYPE` reads the onboard BMI088/IST8310 attitude estimate and
controls one GM6020 through direct current commands. The prototype is intended
for bench testing and controller tuning.

The control loop starts immediately after boot, so the mechanism must be
supported and the motor direction checked before power is applied. Controller
gains and startup assumptions are defined in
[`main_balancing_prototype.cpp`](dev/application/vehicles/balancing_prototype/main_balancing_prototype.cpp);
the CAN motor profile is in
[`can_motor_config.cpp`](dev/application/vehicles/balancing_prototype/can_motor_config.cpp).

## Prerequisites

- RoboMaster Development Board Type C
- ST-Link probe and OpenOCD
- Arm GNU Toolchain with `arm-none-eabi-gcc`, `arm-none-eabi-g++`, and GDB
- CMake 3.25 or newer
- Ninja
- macOS Apple Silicon for the provided quick-start scripts and presets

The repository also contains ChibiOS, CMSIS, and shared device drivers, so a
separate ChibiOS installation is not required.

## Quick Start on macOS

Install Homebrew first, then run:

```bash
./scripts/bootstrap-macos.sh
./scripts/doctor.sh
```

The bootstrap script installs the required Homebrew packages, installs or links
the Arm GNU Toolchain, and checks OpenOCD. The `Brewfile` records the Homebrew
dependencies.

Build without flashing:

```bash
./scripts/build-flash.sh --no-flash STABILITY_SYSTEM c release
./scripts/build-flash.sh --no-flash BALANCING_PROTOTYPE c release
```

Build and flash with ST-Link:

```bash
./scripts/build-flash.sh STABILITY_SYSTEM c release
./scripts/build-flash.sh BALANCING_PROTOTYPE c release
```

The script arguments are `<target> [board] [debug|release]`. For the focused
applications, use board `c` and an explicit build type.

Generated ELF files are written to:

```text
build/mac-arm64-c-debug/bin/<target>.elf
build/mac-arm64-c-release/bin/<target>.elf
```

## Build and Flash on Other Hosts

Set `ARM_NONE_EABI_TOOLCHAIN_DIR` to the directory containing the Arm toolchain
executables, then configure the project directly:

```bash
export ARM_NONE_EABI_TOOLCHAIN_DIR="/path/to/arm-none-eabi/bin"

cmake -S . -B build/board-c-release \
  -G Ninja \
  -DBOARD_NAME=rm_board_c \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build/board-c-release --target STABILITY_SYSTEM --parallel
cmake --build build/board-c-release --target BALANCING_PROTOTYPE --parallel
```

Flash an ELF manually:

```bash
openocd -f config/openocd_stm32f4.cfg \
  -c 'program "build/board-c-release/bin/STABILITY_SYSTEM.elf" verify reset exit'
```

## CLion

1. Open this repository as a CMake project.
2. Select the `mac-arm64-c-debug` or `mac-arm64-c-release` preset.
3. Select `STABILITY_SYSTEM` or `BALANCING_PROTOTYPE` as the run target.
4. Set the global OpenOCD executable in
   `Settings -> Build, Execution, Deployment -> Embedded Development`.
5. Use the CLion flash/debug controls for ST-Link debugging.

The CMake presets keep debug and release artifacts in separate directories.

## Validation Targets

Useful hardware tests include:

```bash
./scripts/build-flash.sh ut_button c release
./scripts/build-flash.sh ut_spi c release
```

`ut_button` starts with a red LED and toggles between red and green on each
Board C user-button press. `ut_spi` exercises the SPI interface used by the
stability system.

## Repository Layout

```text
dev/application/vehicles/stability_system/    Stability-system application
dev/application/vehicles/balancing_prototype/ Balancing-prototype application
dev/interface/                                CAN, AHRS, SPI, USB, and shell drivers
dev/scheduler/                                Motor and balancing control scheduling
dev/board_pin/rm_board_c/                     Board C pin and MCU configuration
config/                                       Toolchain and OpenOCD configuration
scripts/                                      Bootstrap, build, flash, and diagnostics
CMakePresets.json                             macOS build presets
```

## Upstream

This repository is derived from the Meta-Embedded project and its shared
RoboMaster infrastructure. Preserve the existing upstream notices and review
the upstream repository for project history and third-party component details:

- <https://github.com/tyh4n/Meta-Embedded>
- <https://github.com/Meta-Team/Meta-Embedded>
