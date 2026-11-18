# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-esc`; the forge package key, C API and configuration
options are `esc`. The clone commands below name the ESP-IDF component
directories `esc` and `can`. It replaces `esp-HT03` (forge key `ht03`): a
project that `REQUIRES esp-HT03` now `REQUIRES esc can`; `ht03.h` keeps its
calls, on `struct can_dev` and errno (see
[changes from esp-HT03](usage.md#changes-from-esp-ht03)).

Three step-by-step guides come first: [ESP-IDF](#esp-idf-step-by-step),
[STM32 with CMake](#with-cmake-a-cubemx-cmake-project) and
[STM32 without CMake](#without-cmake-stm32cubeide-a-makefile). None
needs a forge; if you build with esp-forge or stm-forge, see
[With forge](#with-forge). The steps drive a DJI C620 (M3508 motor) over
CAN, so they bring in two libraries: this one, and
[mcu-can](https://github.com/JonasAtelier/mcu-can) for the CAN controller
([why](#dependency-mcu-can)). The C615 and DShot models need no CAN and no
mcu-can on the STM32. The reference follows the guides.

**Arduino is not supported yet.** TODO: make the library usable from
Arduino (Arduino IDE and PlatformIO).

## ESP-IDF, step by step

**You need** ESP-IDF 5.2 or newer, with `idf.py` working in your terminal, and
an ESP-IDF project: a folder with a `CMakeLists.txt` and a `main/` folder.
`idf.py create-project my-project` makes one. On the wire: a CAN transceiver
such as the SN65HVD230 on the ESP32's TWAI pins, 120 ohm at both ends of the
bus, and the motors. The example of step 4 drives a C620 at ID 1 and a
GM6020 at ID 2.

ESP-IDF calls a library a **component**: a folder with its own
`CMakeLists.txt` that the build finds and compiles. These libraries are the
components `esc` and `can`, and `esc` asks for `can` by that name.

1. **Get both libraries.** Pick one way.

   - **Component manager**, the simplest: the first build downloads them
     into `managed_components/esc` and `managed_components/can`. Create
     `main/idf_component.yml`:

     ```yaml
     dependencies:
       can:
         git: https://github.com/JonasAtelier/mcu-can.git
         version: stable
       esc:
         git: https://github.com/JonasAtelier/mcu-esc.git
         version: stable
     ```

     Keep the key `can`: it becomes the component's name.

   - **Clone**, if you want to edit the libraries or build offline. In the
     project folder:

     ```sh
     git clone https://github.com/JonasAtelier/mcu-can components/can
     git clone https://github.com/JonasAtelier/mcu-esc components/esc
     ```

     Keep the folder names: ESP-IDF names a component after its folder.

   ```
   my-project/
   ├── CMakeLists.txt
   ├── sdkconfig.defaults       step 2
   ├── main/
   │   ├── CMakeLists.txt       step 3
   │   ├── idf_component.yml    step 1, component manager
   │   └── my-project.c         step 4
   └── components/
       ├── can/                 step 1, clone
       └── esc/                 step 1, clone
   ```

2. **Turn the models and the CAN controller on.** No driver is built until
   you do. Create `sdkconfig.defaults` in the project folder, a text file of
   settings you choose:

   ```ini
   CONFIG_CAN_TWAI=y
   CONFIG_ESC_C620=y
   CONFIG_ESC_GM6020=y
   CONFIG_FREERTOS_HZ=1000
   ```

   `CAN_TWAI` is mcu-can's driver for the ESP32's own CAN controller; the
   `ESC_` lines pick the models, one per line (the others are
   `CONFIG_ESC_C610`, `_C615`, `_DSHOT` and `_HT03`); the 1 kHz tick lets
   the example's loop run once a millisecond. ESP-IDF copies these lines
   into `sdkconfig`, the project's full settings file, only when it creates
   that file: **if `sdkconfig` already exists, delete it** so your lines are
   read. `idf.py menuconfig` → **Component config → Brushless motor driver
   (ESC) subsystem** and **CAN subsystem (Classic CAN; CAN FD on the
   FDCAN)** does the same from a menu.

3. **Let your code see the libraries.** A new project's
   `main/CMakeLists.txt` has no `REQUIRES` and already sees every component.
   If yours has a `REQUIRES` list, add `esc`, `can` and `driver` to it:

   ```cmake
   idf_component_register(SRCS "main.c" REQUIRES esc can driver)
   ```

4. **Write the smallest test.** Copy
   [examples/esp/dji/main/main.c](../examples/esp/dji/main/main.c) over your
   main source file and set `PIN_TX`, `PIN_RX` and `PIN_RS` to your wiring
   (`PIN_RS` to `TWAICAN_NO_GPIO` if the transceiver's Rs pin is tied low).
   Its opening comment says how to set the motors' IDs. **It spins both
   motors**: wheels off the ground. [API usage](usage.md) explains the
   calls.

5. **Build.**

   ```sh
   idf.py set-target esp32    # your chip: esp32s3, esp32c3, ...
   idf.py build
   ```

   A good build ends with `Project build complete.` Flash it as any ESP-IDF
   app and open the serial monitor: every half second each motor prints a
   line ending in, for the C620,
   `esc: c620: online angle 1.571 position -12.250 velocity 3.00 temperature 40`.
   `offline` instead of `online` means no feedback frame has arrived from
   that motor: check its ID, the wiring and the termination.

**If it fails**

| The build prints | Why | Fix |
|---|---|---|
| `Failed to resolve component 'can' required by component 'esc': unknown name.` | mcu-can is not in a folder named `can` (or not under the key `can`) | Step 1 |
| `fatal error: dji.h: No such file or directory` | `main/CMakeLists.txt` has a `REQUIRES` list without `esc` | Step 3 |
| `esc: CONFIG_ESC is on but no model is selected; set CONFIG_ESC_<MODEL>=y (esp-forge: ESC_<MODEL> = y)` (a CMake warning) | No model turned on, often because `sdkconfig` existed before you edited `sdkconfig.defaults` | Step 2: delete `sdkconfig`, build again |

## STM32, step by step

**You need** a project for an STM32F4 that uses ST's **HAL**, the vendor
library behind `HAL_Init()`, `HAL_CAN_Start()` and friends, with CAN1 set up
at 1 Mbit/s, the only rate the DJI ESCs run at. STM32CubeMX makes such a
project; with **Toolchain / IDE: CMake** it makes a CMake one, the main case
below. On the wire: a transceiver on CAN1's pins, 120 ohm at both ends of
the bus, and a C620 at ID 1.

Two terms. A **HAL module** is the part of the HAL for one peripheral: CAN,
TIM, DMA. It is compiled only when it is switched on in **hal_conf**, the
file `Core/Inc/stm32f4xx_hal_conf.h`. CubeMX switches a module on when you
enable that peripheral.

### With CMake (a CubeMX CMake project)

1. **Get both libraries.** In the project folder, next to the top-level
   `CMakeLists.txt`:

   ```sh
   git clone https://github.com/JonasAtelier/mcu-can
   git clone https://github.com/JonasAtelier/mcu-esc
   ```

   ```
   my-board/
   ├── CMakeLists.txt                  step 2
   ├── cmake/stm32cubemx/              made by CubeMX
   ├── Core/
   │   ├── Inc/stm32f4xx_hal_conf.h    hal_conf, step 3
   │   └── Src/main.c                  step 4
   ├── Drivers/                        the HAL, copied in by CubeMX
   ├── mcu-can/                        step 1
   └── mcu-esc/                        step 1
   ```

2. **Add them to the build.** In the top `CMakeLists.txt`, after the line
   `add_subdirectory(cmake/stm32cubemx)`, add:

   ```cmake
   set(CONFIG_CAN_BXCAN ON)            # mcu-can: the STM32's CAN controller
   set(CONFIG_ESC_C620 ON)             # your model; one line per model
   set(MCU_HAL_TARGET stm32cubemx)
   set(MCU_DEVICE_HEADER stm32f4xx.h)
   add_subdirectory(mcu-can)           # first: mcu-esc links to it
   add_subdirectory(mcu-esc)
   ```

   and add `can` and `esc` to the link line CubeMX already wrote, so it
   reads:

   ```cmake
   target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx can esc)
   ```

   Do not add `PRIVATE`: CubeMX's line has no such keyword, and CMake refuses
   to mix the two forms on one target.

   `stm32cubemx` is a CMake *target*, a named bundle of settings, that CubeMX
   makes to carry the HAL's include folders and defines; `MCU_HAL_TARGET`
   lends those to both libraries, which works with any CubeMX version.
   `MCU_DEVICE_HEADER` is your chip family's main header. Both libraries read
   these two, so they are set once. The CAN models (C610, C620, GM6020,
   HT-03) link to mcu-can's target `can`, which is why mcu-can is added
   first; a C615- or DShot-only build needs neither mcu-can line.

3. **Check the HAL module.** The bxCAN needs `#define HAL_CAN_MODULE_ENABLED`
   in hal_conf; enabling CAN1 in CubeMX does that. In CAN1's settings there,
   set the bit timing for 1 Mbit/s (the prescaler and the two bit segments)
   and turn on automatic bus-off management; under its NVIC settings, enable
   the CAN1 RX0 interrupt, so CubeMX calls `HAL_CAN_IRQHandler()` from it.
   The [HAL module table](#stm32-hal) lists what the other models need.

4. **Write the smallest test.** It is
   [examples/stm/dji_bxcan/main.c](../examples/stm/dji_bxcan/main.c) fitted
   into CubeMX's `Core/Src/main.c`, with one C620 at ID 1 commanded 0 A, so
   the motor stays still while its feedback comes in. Paste each part
   between the `USER CODE BEGIN` and `END` comments named, so a regenerate
   keeps it.

   After `/* USER CODE BEGIN Includes */`:

   ```c
   #include <stdio.h>

   #include "bxcan.h"
   #include "can.h"
   #include "dji.h"
   #include "esc.h"
   #include "esc_log.h"
   ```

   After `/* USER CODE BEGIN 0 */`, just above `int main(void)`:

   ```c
   static bxcan_handle_t bx;
   static struct dji_bus bus;
   static struct esc_dev *wheel;

   /*
    * All IDs into FIFO 0, as in mcu-can's examples/stm/bxcan. The bxCAN
    * receives nothing until a filter lets frames in.
    */
   static void accept_all(void)
   {
     CAN_FilterTypeDef f = {
       .FilterBank = 0,
       .FilterMode = CAN_FILTERMODE_IDMASK,
       .FilterScale = CAN_FILTERSCALE_32BIT,
       .FilterFIFOAssignment = CAN_RX_FIFO0,
       .FilterActivation = CAN_FILTER_ENABLE,
       .SlaveStartFilterBank = 14,
     };

     HAL_CAN_ConfigFilter(&hcan1, &f);
   }
   ```

   After `/* USER CODE BEGIN 2 */`, which comes after `MX_CAN1_Init()`:

   ```c
     bxcan_config_t cfg = { .hcan = &hcan1, .use_irq = true };
     struct dji_motor_config wheel_cfg = {
       .model = DJI_MODEL_C620, .id = 1,
       .gear_ratio = 3591.0f / 187.0f,   /* M3508 P19 */
     };

     accept_all();
     if (bxcan_init(&cfg, &bx) || dji_init(&bus, bxcan_can(bx)) ||
         dji_add(&bus, &wheel_cfg, &wheel))
       Error_Handler();
     esc_set_command(wheel, ESC_COMMAND_CURRENT, 0.0f);
   ```

   After `/* USER CODE BEGIN 3 */`, inside the `while (1)` loop:

   ```c
       static uint32_t last_ms, ticks;
       uint32_t now_ms = HAL_GetTick();
       struct can_frame frame;
       struct esc_feedback fb;

       if (now_ms == last_ms)
         continue; /* once a millisecond */
       while (can_recv(bxcan_can(bx), &frame, 0) == 0)
         dji_handle_frame(&bus, &frame);
       dji_update(&bus, (float)(now_ms - last_ms) / 1000.0f);
       last_ms = now_ms;
       if (++ticks % 500 == 0 && esc_get_feedback(wheel, &fb) == 0)
         esc_log_feedback(wheel, &fb);
   ```

   After `/* USER CODE BEGIN 4 */`:

   ```c
   /* The HAL calls it from HAL_CAN_IRQHandler(), on the CAN1 RX0 interrupt. */
   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
   {
     if (hcan == &hcan1)
       bxcan_irq(bx);
   }
   ```

   The lines go out through `printf()`: send that to a UART by writing
   newlib's `_write()`, and add this to `CMakeLists.txt` so numbers are
   printed:

   ```cmake
   target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
   ```

   To make the motor turn, command a current instead of 0 A, with the wheel
   off the ground. [API usage](usage.md) explains the calls.

5. **Build** as before, for example `cmake --build` with your build folder.
   A good build shows a `Linking C executable` line and no error. On the
   UART, every half second:
   `esc: c620: online angle 1.571 position -12.250 velocity 3.00 temperature 40`.
   `offline` instead of `online` means no feedback frame has arrived: check
   the motor's ID, the bit rate and the wiring.

**If it fails**

| You see | Why | Fix |
|---|---|---|
| `esc: the CAN models (C610, C620, GM6020, HT-03) need mcu-can; add_subdirectory(mcu-can) before add_subdirectory(mcu-esc)` (a CMake error) | mcu-can is missing, or added after mcu-esc | Steps 1 and 2 |
| `esc: no model is selected; set(CONFIG_ESC_<MODEL> ON) before add_subdirectory()` (a CMake warning), then ``undefined reference to `dji_init'`` | No model turned on, or turned on after `add_subdirectory(mcu-esc)` | Step 2 |
| `#error "CAN_BXCAN needs the HAL CAN module: ...` | The HAL module is off | Enable CAN1 in CubeMX and regenerate |
| `#error "can: CMSIS_device_header is not defined; ...` (mcu-esc's sources say the same, with `esc:`) | `MCU_DEVICE_HEADER` is missing | Step 2 |
| `fatal error: stm32f4xx.h: No such file or directory` | `MCU_HAL_TARGET` is missing | Step 2 |
| `Target "stm32cubmx" not found.` (a CMake error) | `MCU_HAL_TARGET` is misspelt | Step 2 |
| `The plain signature for target_link_libraries has already been used with the target` | `PRIVATE` added to the link line | Step 2 |

### Without CMake (STM32CubeIDE, a Makefile)

1. Copy `include/` and `src/` of both repositories into your project, for
   example to `Middlewares/mcu-can/` and `Middlewares/mcu-esc/`, keeping the
   driver folders inside each `src/`.
2. Add every `.c` file under both `src/` folders to the build, driver
   folders included.
3. Add four include paths, `Middlewares/mcu-can/include`,
   `Middlewares/mcu-can/src`, `Middlewares/mcu-esc/include` and
   `Middlewares/mcu-esc/src`, and three defines:
   `CMSIS_device_header="stm32f4xx.h"`, quotes included,
   `CONFIG_CAN_BXCAN=1` and `CONFIG_ESC_C620=1`, one per model. CubeMX
   already defines `USE_HAL_DRIVER` and your device, such as `STM32F405xx`.
4. Steps 3 to 5 of the CMake guide apply as they are; `-u _printf_float`
   goes in the linker flags.

Where these settings usually live: in STM32CubeIDE, **Project → Properties →
C/C++ Build → Settings → Tool Settings → MCU GCC Compiler** (Include paths,
Preprocessor); in a CubeMX Makefile, `C_SOURCES`, `C_INCLUDES` and `C_DEFS`.
The build errors are the ones in the table above; the fix is the matching
define or include path instead of a CMake line. One more:
`#include expects "FILENAME" or <FILENAME>` means `CMSIS_device_header` lost
its quotes.

## Dependency: mcu-can

`dji.h` and `ht03.h` include mcu-can's `can.h`, and the drivers send through
`can_send()`. On ESP-IDF the component always `REQUIRES can`: IDF resolves
`REQUIRES` before it reads sdkconfig, so the requirement cannot follow the
model options. A C615-only build therefore still needs mcu-can checked out;
nothing of it is linked in. On stm-forge, `PKG_can = y` is needed only for
the CAN models: with only `ESC_C615 = y` the CAN drivers compile to nothing
and never include `can.h`. Through `add_subdirectory()` it is the same: the
static library `esc` links to mcu-can's target `can` only when a CAN model
is on, and stops at configure time if that target does not exist yet.

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

## ESP-IDF

Setup: [ESP-IDF, step by step](#esp-idf-step-by-step). For reference:

- In `main/idf_component.yml`, `version` is a git ref; `stable` is the
  released one.
- A clone in `components/` is a local component, named after its folder;
  that name is what other components `REQUIRES`. This component
  `REQUIRES can`, so mcu-can's folder must be `can`. An
  `idf_component.yml` dependency named `can` is fetched into a folder of
  that name; a `path:` dependency keeps its own folder's name, so a local
  checkout named `mcu-can` does not work that way either.
- The examples in `examples/esp/` take this repository from any checkout
  name (their `main` lists no `REQUIRES`, and ESP-IDF gives `main` every
  component), and mcu-can from a folder named `can` next to it:

  ```sh
  git clone https://github.com/JonasAtelier/mcu-can can
  git clone https://github.com/JonasAtelier/mcu-esc
  cd mcu-esc/examples/esp/dji
  idf.py set-target esp32s3
  idf.py build
  ```

  Without that folder the build stops with
  `Directory specified in EXTRA_COMPONENT_DIRS doesn't exist:`.
- With the subsystem on and no model picked, the build warns.

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

Setup: [With CMake](#with-cmake-a-cubemx-cmake-project) or
[Without CMake](#without-cmake-stm32cubeide-a-makefile). The
`add_subdirectory()` route builds a static library `esc`. For reference:

- The family is the F4: `CMSIS_device_header` is `"stm32f4xx.h"`. DShot in
  particular is written for the STM32F4's TIM1 or TIM8 (`dshot.h`).
- Each `CONFIG_ESC_<MODEL>` (and `CONFIG_ESC_DEBUG`) that is true reaches the
  compiler as `CONFIG_ESC_<MODEL>=1`, for the library and your app.
- The CAN models need mcu-can: through `add_subdirectory()`, mcu-can first
  (see [Dependency: mcu-can](#dependency-mcu-can)); without CMake, its
  `include/` and `src/` the same way as this library's.
- `MCU_HAL_TARGET` is a target that carries your `USE_HAL_DRIVER`, device
  define (`STM32F405xx`, ...) and the include dirs of the HAL, CMSIS and your
  `stm32f4xx_hal_conf.h`. The library copies only those include dirs and
  defines, never the target's sources, so this works with any CubeMX
  version. `MCU_DEVICE_HEADER` becomes `CMSIS_device_header`, which CubeMX
  does not define. Every `mcu-*` library reads these two, so set them once.
  Leave them unset if your project already provides both.
- The CPU flags (`-mcpu`, `-mfloat-abi`, ...) must reach the library too;
  CubeMX's toolchain file sets them for every target.
- Without CMake, `src/` only needs to be on the include path of this
  library's own sources; the defines are what stm-forge passes.

**No model is built by default.** The core is always built and each driver
is opt-in: a model whose option is absent or 0 compiles to nothing. With
the library on and no model picked, the CMakeLists warns (ESP-IDF and
`add_subdirectory()`) and `src/esc_port_stm.c` emits a `#warning`.

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
