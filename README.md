# mcu-esc

Control brushless motors with an electronic speed controller (ESC). DJI C610, C620, GM6020 and Haitai HT-03 use CAN, the DJI C615 uses PWM, and compatible ESCs accept DShot.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

## Contents

- [Quick start](#quick-start)
- [Controllers and examples](#controllers-and-examples)
- [Using the library in your project](#using-the-library-in-your-project)
- [License](#license)

## Quick start

The example uses an ESP32-S3 and four DShot600 controllers. Remove propellers, secure the motors and check signal pins and common ground before powering them. ESP-IDF requires the sibling `can` component even for DShot.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32-S3.

```sh
git clone https://github.com/JonasAtelier/mcu-can.git can
git clone https://github.com/JonasAtelier/mcu-esc.git esc
cd esc/examples/esp/dshot
idf.py set-target esp32s3
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/dshot/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

The example arms the controllers, beeps motor 1 and ramps all four motors to 10% throttle and back to zero. For DJI CAN models, reported feedback current is in raw counts.

## Controllers and examples

The command units depend on the controller. Check the supported command kinds before calling `esc_set_command()`. For a CAN controller, its driver header also describes feedback and bus handling. [Shared API](include/esc.h)

| Controller | Driver header | ESP-IDF project | STM32 HAL example |
|---|---|---|---|
| [DJI C610, C620, GM6020](docs/chips.md#dji-can-frames) | [dji.h](include/dji.h) | [C620 and GM6020 on TWAI](examples/esp/dji/) | [C610 and C620 on bxCAN](examples/stm/dji_bxcan/main.c) |
| [Haitai HT-03](docs/chips.md#haitai-ht-03) | [ht03.h](include/ht03.h) | [Position hold on TWAI](examples/esp/ht03/) | — |
| [DJI C615](docs/chips.md#c615) | [c615.h](include/c615.h) | [LEDC pulse output](examples/esp/c615/) | [Timer pulse output](examples/stm/c615/main.c) |
| [DShot](docs/chips.md#dshot) | [dshot.h](include/dshot.h) | [Throttle](examples/esp/dshot/) · [Bidirectional telemetry](examples/esp/dshot_bidir/) | [Throttle](examples/stm/dshot/main.c) · [Bidirectional telemetry](examples/stm/dshot_bidir/main.c) |

A dash marks a hardware and platform combination without a dedicated example.

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Open the ESP-IDF project for your hardware, or copy the STM32 example into a project with board setup already in place. [All ESP-IDF examples](examples/esp/) · [All STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test MCU_CAN=../../can` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE).

[Browse the workshop catalog · ESP32 and STM32 drivers](https://github.com/JonasAtelier/jonas-atelier-libraries#esp32-and-stm32-drivers)
