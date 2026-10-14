# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-esc`; the forge package key, C API and configuration
options are `esc`. The clone commands below name the ESP-IDF component
directories `esc` and `can`. It replaces `esp-HT03` (forge key `ht03`): a
project that `REQUIRES esp-HT03` now `REQUIRES esc can`; `ht03.h` keeps its
calls, on `struct can_dev` and errno (see
[changes from esp-HT03](usage.md#changes-from-esp-ht03)).

## Dependency: mcu-can

`dji.h` and `ht03.h` include mcu-can's `can.h`, and the drivers send through
`can_send()`. On ESP-IDF the component always `REQUIRES can`: IDF resolves
`REQUIRES` before it reads sdkconfig, so the requirement cannot follow the
model options. A C615-only build therefore still needs mcu-can checked out;
nothing of it is linked in. On stm-forge, `PKG_can = y` is needed only for
the CAN models: with only `ESC_C615 = y` the CAN drivers compile to nothing
and never include `can.h`.

mcu-esc never configures the bus. Pick and start the controller in mcu-can
(`CAN_TWAI`, `CAN_MCP2515`, `CAN_BXCAN`) at 1 Mbit/s, the only rate every
model here runs at ([model reference](chips.md)).

## With forge

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog lines in
`profiles/pkgversion` are the same for both:

```make
can_GIT=https://github.com/JonasAtelier/mcu-can@stable
esc_GIT=https://github.com/JonasAtelier/mcu-esc@stable
```

and a target turns the library and its models on in
`configs/<target>_defconfig`:

```make
PKG_esc             = y
ESC_C620            = y
# ESC_C610 is not set
# ESC_GM6020 is not set
# ESC_C615 is not set
# ESC_DSHOT is not set
# ESC_HT03 is not set
# ESC_DEBUG is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_ESC_<MODEL>=y`
in `sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_ESC_<MODEL>=1`.
`ESC_DEBUG = y` turns on `esc_dbg()` lines on the HAL; on ESP-IDF the log
level decides. stm-forge must be at `ee921c3` or later: older ones compile
only `src/*.c`, so the drivers in `src/<model>/` are never built.

Neither forge's catalog has these lines yet: they go in once the repository
is on GitHub.

## ESP-IDF

Clone both into your project's `components/` directory. A local component is
named after its directory, and that name is what you `REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-can components/can
git clone https://github.com/JonasAtelier/mcu-esc components/esc
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES esc can driver)
```

Enable the models under `idf.py menuconfig` → **Component config →
Brushless motor driver (ESC) subsystem**, or put `CONFIG_ESC_<MODEL>=y` in
your `sdkconfig.defaults` as each example in `examples/esp/` does. With the
subsystem on and no model picked, the build warns.

### Next to esp-c620-control

Both libraries link into one app: nothing in this one is named like
anything in that one -- no header (`rm.h`, `internal.h`), no source file
(`core.c`, `loop.c`, `status.c`), no type or macro (`rm_t`, `rm_sta`,
`RM_*`, `C620_*`, `M3508_*`), and no global symbol (`rm_*`,
`get_motor_by_id`, `pos_to_degree`, ...). Nor with esp-sn65hvd230-can, which
esp-c620-control sits on. `test/test_names.c` includes `rm.h`,
`hvd230can.h` and every header here in one file; a scratch firmware with all
four libraries and every model on links with no global symbol defined twice
(`docs/development.md`, [Tests](development.md#tests)).

**One owner per bus.** esp-c620-control's `rm_t` sends 0x200 and 0x1FF on
its bus; a `struct dji_bus` on the same bus sends them too. Keep the two on
different buses.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, model folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"`),
`USE_HAL_DRIVER`, and `CONFIG_ESC_<MODEL>=1`. The CAN models need mcu-can
the same way.

**No model is built by default.** The core is always built and each driver
is opt-in: a model whose option is absent or 0 compiles to nothing. With
the library on and no model picked, `src/esc_port_stm.c` emits a
`#warning`.

**The HAL modules are yours to enable:**

| Model | Needs in hal_conf | HAL source | Gives |
|---|---|---|---|
| C615 | `HAL_TIM_MODULE_ENABLED` | `stm32<family>xx_hal_tim.c` | `c615_init()` |
| DShot | `HAL_TIM_MODULE_ENABLED`, `HAL_DMA_MODULE_ENABLED` | `stm32<family>xx_hal_tim.c`, `_dma.c` | `dshot_init()` |
| C610, C620, GM6020, HT-03 | what mcu-can's controller needs | -- | `dji_*`, `ht03_*` |

Without the TIM module, a C615 build stops with an `#error` in
`src/esc_priv.h` (and in `c615.h`); without TIM and DMA, a DShot build does
the same (and in `dshot.h`). stm-forge's `mcu/stm32f4.cmake` lists both.

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART.

## Logging

`esc_err()`, `esc_warn()`, `esc_info()` and `esc_dbg()` prefix every line
with the motor's model name. On ESP-IDF they are `ESP_LOGx` under the one
tag `"esc"`, so `esp_log_level_set("esc", ...)` controls every motor. On the
HAL they are `printf("esc: <name>: ...")`, and `esc_dbg()` is compiled in
only with `CONFIG_ESC_DEBUG`. `esc_log_feedback()` prints the fields a
model has:

```
esc: c620: online angle 1.571 position -12.250 velocity 3.00 temperature 40
```
