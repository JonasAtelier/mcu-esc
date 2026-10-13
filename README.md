# mcu-esc

<p align="center">
  <img alt="platform: ESP-IDF | STM32 HAL" src="https://img.shields.io/badge/platform-ESP--IDF%20%7C%20STM32%20HAL-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A small C library for **brushless motor drivers** on **ESP-IDF 5.2+** and
the **STM32 HAL**: drivers that take a command and return feedback where
they have it, including motors with the driver built in (GM6020, HT-03).
Each model keeps its own API and hands out a `struct esc_dev` per motor: a
command in amperes, newton metres or throttle, and feedback decoded the same
way whichever model sent it.

**Scope.** Steppers are [mcu-stepper](https://github.com/JonasAtelier/mcu-stepper)'s.
Hobby servos and the PCA9685 are out of scope.

The CAN models run on [mcu-can](https://github.com/JonasAtelier/mcu-can)'s
`struct can_dev` -- ESP32 TWAI, MCP2515, STM32 bxCAN -- and never own or
configure the bus. This library replaces `esp-HT03`, whose `ht03.h` it keeps
call for call on errno. `esp-c620-control` stays as it is, and links next to
this library: no header, type, macro, file or global symbol is shared.

The repository is named `mcu-esc`; the C API, `CONFIG_ESC_*` options and
forge package key are `esc`.

> **Hardware unverified.** Host tests use a fake CAN bus, fake LEDC and TIM
> drivers and platform stubs. Nothing here has turned a motor.

[Supported hardware](#supported-hardware) · [Install](#install) ·
[Quick start](#quick-start) · [Examples](#examples) ·
[Documentation](#documentation) · [Tests](#tests) · [License](#license)

## Supported hardware

| Model | Motor | Link | Command | Feedback |
|---|---|---|---|---|
| **DJI C610** | M2006 | CAN, 1 Mbit/s | current, +-10 A | angle, position, velocity |
| **DJI C620** | M3508 | CAN, 1 Mbit/s | current, +-20 A | angle, position, velocity, temperature |
| **DJI GM6020** | built in | CAN, 1 Mbit/s | current +-3 A, or voltage in raw counts | angle, position, velocity |
| **DJI C615** | Snail 2305 and others | PWM, 400-2200 us, up to 500 Hz | throttle 0..1 | none |
| **Haitai HT-03** | HT8108-J6, built in | CAN, 1 Mbit/s, MIT protocol | torque; the full MIT command | position, velocity, torque |

One Kconfig option per model: `ESC_C610`, `ESC_C620`, `ESC_GM6020`,
`ESC_C615`, `ESC_HT03`. Every number is cited to its manual in the
[model reference](docs/chips.md); wiring is in the
[hardware guide](docs/hardware.md).

## Install

### ESP-IDF

```sh
git clone https://github.com/JonasAtelier/mcu-can components/can
git clone https://github.com/JonasAtelier/mcu-esc components/esc
```

```ini
CONFIG_ESC=y
CONFIG_ESC_C620=y
CONFIG_CAN_TWAI=y
```

Other components declare `REQUIRES esc can driver`. mcu-esc requires mcu-can
even for a C615-only build: see the [installation guide](docs/installation.md).

### esp-forge / stm-forge

```make
can_GIT=https://github.com/JonasAtelier/mcu-can@stable
esc_GIT=https://github.com/JonasAtelier/mcu-esc@stable
```

```make
PKG_can             = y
CAN_TWAI            = y
PKG_esc             = y
ESC_C620            = y
```

**No model is enabled by default.**

## Quick start

A C620 on the TWAI:

```c
#include "dji.h"
#include "twai_can.h"

static struct dji_bus bus;
struct dji_motor_config cfg = { .model = DJI_MODEL_C620, .id = 1 };
struct can_dev *can = twaican_can(twai);    /* the app's, at 1 Mbit/s */
struct esc_dev *wheel;
struct can_frame frame;

dji_init(&bus, can);
dji_add(&bus, &cfg, &wheel);

/* every control period */
while (can_recv(can, &frame, 0) == 0)
    dji_handle_frame(&bus, &frame);     /* -ENOENT: not for this bus */
esc_set_command(wheel, ESC_COMMAND_CURRENT, 2.0f);
dji_update(&bus, dt_s);                 /* the only call that sends */
```

and the same `esc_dev` calls for any model:

```c
struct esc_feedback fb;

if (esc_get_feedback(wheel, &fb) == 0 && fb.online)
    esc_log_feedback(wheel, &fb);
```

- **0 or a negative errno** from every call, on both platforms. A command
  kind a model does not take is `-EOPNOTSUPP`.
- **No clock is read here.** The app passes `dt_s` to `dji_update()` and
  `ht03_update()`, which also age the feedback: a motor silent for its
  timeout (0.1 s) is offline and is sent zero, or limp on the HT-03.
- **No speed or position loop.** The PID loop of esp-c620-control stays
  there; close loops in the app (upid works) on `esc_get_feedback()`.

## Examples

| Model | ESP-IDF | STM32 HAL |
|---|---|---|
| C620 + GM6020 | [dji](examples/esp/dji/), on the TWAI | -- |
| C610 + C620 | -- | [dji_bxcan](examples/stm/dji_bxcan/main.c), on bxCAN |
| HT-03 | [ht03](examples/esp/ht03/), esp-HT03's position hold | -- |
| C615 | [c615](examples/esp/c615/), a throttle ramp on LEDC | [c615](examples/stm/c615/main.c), four on TIM8 as on Hawk |

ESP-IDF examples find this library and mcu-can through
`EXTRA_COMPONENT_DIRS`, which assumes they are checked out side by side as
`esc` and `can`:

```sh
cd examples/esp/dji
idf.py set-target esp32s3 # Replace with your board's target.
idf.py build
```

STM32 examples are snippets to integrate into your application.

## Scope and limitations

- **No feedback current in amperes from the DJI ESCs.** None of the three
  manuals gives the scale of the torque current they report; it is in
  `dji_get_raw()` as counts only.
- **GM6020 voltage mode is raw counts** (+-25000): the manual gives no
  unit. Current mode needs motor firmware v1.0.11.2+ and is chosen per motor
  at `dji_add()`.
- **C615: no stop or idle pulse invented.** The app gives the endpoints it
  calibrated the ESC to; the manual does not say which pulse stops it.
- **HT-03 checked against the STM32F446 rewrite** of the MIT firmware; the
  HT-03 ships the older mbed one. See the [model reference](docs/chips.md#haitai-ht-03).
- **No locks.** One task per bus, or your own mutex.

## Documentation

| Guide | Contents |
|---|---|
| [Installation](docs/installation.md) | Forge, ESP-IDF, STM32 HAL, build options, logging, next to esp-c620-control |
| [Hardware](docs/hardware.md) | CAN wiring and IDs, the GM6020's DIP switch, the C615's PWM cable and calibration |
| [API usage](docs/usage.md) | The `esc_dev`, the DJI bus, the HT-03, the C615, errors, changes from esp-HT03 |
| [Model reference](docs/chips.md) | Every frame, identifier, range and pulse, each cited to its manual |
| [Development](docs/development.md) | Layout, ports, bus abstraction, adding a model and test coverage |

## Tests

```sh
make -C test test
```

Host-only; needs mcu-can checked out next to this repository (or
`MCU_CAN=/path/to/mcu-can`). See the
[test reference](docs/development.md#tests). Log lines during the suite are
expected.

## License

[MIT](LICENSE).
