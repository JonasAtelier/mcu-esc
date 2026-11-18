# mcu-esc

Control brushless motors with an electronic speed controller (ESC). DJI C610, C620, GM6020 and Haitai HT-03 use CAN, the DJI C615 uses PWM, and compatible ESCs accept DShot.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

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

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Start from an example for your chip. The STM32 examples fit into a project with board setup already in place. [ESP-IDF examples](examples/esp/) · [STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test MCU_CAN=../../can` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE).
