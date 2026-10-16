# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  esc.h                     struct esc_dev, commands, feedback
  esc_log.h                 esc_err/warn/info/dbg, esc_log_feedback()
  dji.h                     C610, C620, GM6020: struct dji_bus, dji_*()
  ht03.h                    esp-HT03's API on struct can_dev, ht03_esc()
  c615.h                    struct c615 and its config, struct esc_pwm, c615_*()
  dshot.h                   struct dshot_bus and its config, struct dshot_port, dshot_*()
src/                        private to the library
  esc.c                     the core: the command and feedback checks
  esc_log.c                 esc_log_feedback()
  esc_priv.h                bus ops, attach and probe, errno map, the PWM and DShot ports
  esc_port_esp.c            all of that on ESP-IDF (LEDC, RMT); empty elsewhere
  esc_port_stm.c            all of that on the STM32 HAL (TIM, TIM DMA burst); empty under ESP-IDF
  dji/                      dji.c  dji_proto.h  dji_math.h
  ht03/                     ht03.c  ht03_math.h
  c615/                     c615.c
  dshot/                    dshot.c  dshot_proto.h  dshot_math.h
test/                       host tests, `make -C test test`
  test_dji_math.c           the DJI conversions against the manuals' numbers
  test_ht03_math.c          esp-HT03's test_pack.c: layout, scaling, clamping
  test_dshot_math.c         DShot frames, CRCs, timing, throttle and the reply against the sources' examples
  test_names.c              rm.h, hvd230can.h, upid.h and every header here in one file
  test_dji.c                dji.c on a fake can_send()
  test_ht03.c               esp-HT03's test_core.c on a fake can_send(), and the esc_dev
  core_cases.h              core cases, shared by esp/ and stm/
  c615_cases.h              C615 cases, shared by esp/ and stm/
  dshot_cases.h             DShot cases, shared by esp/ and stm/
  esp/                      suites on idf_stubs/, through esc_port_esp.c, a fake LEDC and RMT
                            (test_dshot_nosync, test_dshot_c3: the ESP32's and C3's RMT)
  stm/                      suites on hal_stubs/, through esc_port_stm.c, a fake TIM, DMA and GPIO
                            (test_dshot_nof4: an STM32 other than the F4)
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
  fixtures/                 verbatim copies of rm.h, hvd230can.h, upid.h for test_names
examples/
  esp/<name>/               ESP-IDF projects: dji, ht03, c615, dshot, dshot_bidir
  stm/<name>/               STM32 snippets: dji_bxcan, c615, dshot, dshot_bidir
docs/                       the five guides; the manuals here are gitignored
CMakeLists.txt              ESP-IDF only; stm-forge globs src/**/*.c instead
Kconfig                     ESC and ESC_<MODEL>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
LICENSE                     MIT
```

Naming inside `src/<model>/`, the same in every category library:

| File | Holds |
|---|---|
| `<model>.c` | the driver: init, the ops, the model's calls |
| `<model>_regs.h` | register map, each value with its source citation |
| `<model>_proto.h` | serial protocol, for parts without registers |
| `<model>_math.h` | pure conversions, compiled and asserted on the host |
| `<model>_priv.h` | driver state, only if needed |

The three DJI CAN models share one frame family, so they share one folder,
`src/dji/`, with one Kconfig option each; `dji.c` is built when any of them
is on, and a model built out is `-EOPNOTSUPP` from `dji_add()`. The HT-03
keeps its protocol constants in `ht03.h`, where esp-HT03 had them, and its
packing in `ht03_math.h`; `ht03.c` and `ht03_math.h` keep the original's
four-space style. The library's own files use tabs.

**Names next to esp-c620-control and esp-sn65hvd230-can.** Apps link them
together, so nothing here may share a name with them: not a header, not a
source file, not a type, tag, enumerator, macro or global symbol.
Everything public is `esc_*`, `ESC_*`, `dji_*`, `DJI_*`, `ht03_*`,
`HT03_*`, `c615_*` or `C615_*`; `rm_*`, `RM_*`, `C620_*`, `M3508_*`,
`hvd230_*`, `can_frame_t` and the like are taken. `test_names` and the
`nm` check below hold this.

## Handles first

There are no init/del handles on the heap: the app places a
`struct dji_bus`, an `ht03_t` or a `struct c615`, as it placed esp-c620-
control's `rm_t` and esp-HT03's `ht03_t`. Each holds a `struct esc_dev` per
motor, with `priv` pointing back at the motor's state. `set_command` is
required; `get_feedback` is optional and `-EOPNOTSUPP` when absent, which
is how the C615 says it has none.

The core checks what every model would: the kind, the value finite, the
kind in `commands`. It hands `get_feedback` a zeroed struct and copies it
out only on success.

**No clock is read.** Feedback ages by the `dt_s` the app passes to the
update call, as in esp-c620-control's `rm_update()`. Once the time since
the last frame passes the motor's timeout, it is offline.

## Platform ports

The drivers in `src/` are the same C on both platforms; what differs is in
`src/esc_port_esp.c` and `src/esc_port_stm.c`: the bus ops, attach,
`esc_i2c_probe()`, and the PWM port. The CAN models touch no platform code
at all: everything goes through mcu-can's `can_send()`.

**The PWM port** is `struct esc_pwm` and `esc_pwm_attach/check/start/
write_us/stop()`. On ESP-IDF it is one LEDC channel in low-speed mode, which
every ESP32 has, on a timer it configures at the rate with 14-bit duty --
the widest the S2, S3 and C3 take; at 400 Hz one step is 0.15 us. On the HAL
it is a TIM channel the app set up in PWM mode; the port reads ARR, works
out the rate from the app's `tick_hz`, and writes the compare. Stopping
writes compare 0 before the channel goes off, so the pin is low whatever
the stopped channel's idle state. The struct lives in `c615.h`, its only
user, so the app can place a `struct c615`.

**The DShot port** is `struct dshot_port` and `esc_dshot_attach/busy/send/
reply/detach()`; `dshot.c` above it is the same C on both platforms and knows
frames, commands and waits, nothing of pulses. On ESP-IDF it is one RMT TX
channel per motor at 8 MHz, a copy encoder each, and a sync manager over
them for more than one motor, reset before every frame as IDF asks; if a
send fails part-way, the channels that already took a frame are disabled
and enabled again and the sync reset, or they would wait for the rest
forever. The port exists only where `SOC_RMT_SUPPORTED`; the 16
symbols of a frame stay in the port while the encoder may still read them,
and `rmt_tx_wait_all_done(..., 0)` is the busy test. On the HAL it is the
app's timer, one bit per period, and its update DMA burst-writing CCR1-CCR4
from a 17-row buffer, the last row zeros so the line ends low; the HAL
leaves its burst state busy after a transfer, so every send stops the last
burst first, and the DMA handle's state, which the app's IRQ handler
returns to ready, is the busy test. `struct dshot_port` lives in `dshot.h`
so the app can place a `struct dshot_bus`.

**Bidirectional**, the port also hands `dshot.c` each motor's reply to
the last frame as 21 line bits (`esc_dshot_reply()`), and `dshot.c`
decodes them with `dshot_math.h`: GCR, CRC, eRPM or EDT. Both ports turn
what they saw into levels and lengths and build the bits with the same
`dshot_reply_append()`. On ESP-IDF an RX channel per motor on the same GPIO
hears the frame and the reply, and the TX-done callback turns the pin
open-drain for the ESC; the RX-done callback stores the symbol count, and
a count of -1 is the busy test. On the HAL (STM32F4 only) the frame goes
out on the update DMA started directly -- the HAL's burst call would take
the transfer-complete callback -- with two rows of zeros; that callback
turns the pins to inputs and the same stream into a sampler of the port's
IDR at three samples a reply bit, and the next one turns everything back.
The callbacks find the port through the DMA handle's `Parent`, which the
port borrows while the bus lives and gives back at del; the port's phase
is the busy test.

## Bus abstraction

The bus layer is mcu-gamecontroller's: `struct esc_bus_ops` with
`transfer()`, `exchange()`, `receive()` and `detach()`. Every category
library offers I2C, SPI and UART, used or not, and
`test/{esp,stm}/test_port_*` check all three. No model here uses it: the
CAN models are on mcu-can's `struct can_dev`, the C615 on the PWM port,
both outside the bus layer, the way mcu-can's TWAI and bxCAN sit outside
its own.

## Adding a model

1. `include/foo.h`: the model's API, platform types behind
   `#ifdef ESP_PLATFORM`, and a call that hands out the `esc_dev`.
2. `src/foo/foo.c`: include `esc_priv.h` first, then wrap in
   `#if CONFIG_ESC_FOO`.
   - Embed a `struct esc_dev` per motor; set `ops`, `priv`, `name` and
     `commands` once the motor is ready.
   - Give it `set_command`, and `get_feedback` if the model reports
     anything, setting `has` for each field it fills.
   - On CAN: frame in through a `foo_handle_frame()` returning `-ENOENT`
     for frames not its own; frames out from an update call taking `dt_s`.
3. Cite every identifier, range and byte to its manual in
   `src/foo/foo_proto.h`, and hold them in a `test_foo_math.c`.
4. Add `CONFIG_ESC_FOO` to:
   - the no-model checks in `CMakeLists.txt` and `src/esc_port_stm.c`;
   - a HAL module `#error` in `src/esc_priv.h`, if it needs one;
   - its sources in `CMakeLists.txt`;
   - `Kconfig`;
   - both forges' defconfigs;
   - `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL, ESP-IDF or motor needed, but mcu-can's
`include/can.h` from the checkout next to this one (`MCU_CAN=` to point
elsewhere). Nothing is checked against a real ESC.

Each change is checked by a test that fails without it: 91 mutations of the
source were run, each on a scratch copy; 89 fail the suite. The two that
survive drop the cap on the time since the last frame, in `dji.c` and in
esp-HT03's `ht03.c` alike, which only keeps a float from growing while a
motor stays unplugged -- not something a host test can see. DShot had its
own pass: 69 mutations of `dshot.c`, `dshot_math.h`, `dshot_proto.h` and
both ports' DShot halves, all 69 failing the suite.

| Binary | Checks |
|---|---|
| `test_dji_math` | **every command scale at its ends, half and rounding, against C610 p.8, C620 p.14, GM6020 pp.7-8**; the clamp, v1.2's 30000 to 25000; the identifiers; byte order; **the angle wrap at exactly half a turn and one past**; rpm and counts to radians through a gear ratio |
| `test_ht03_math` | esp-HT03's test_pack.c: every field at both ends, hand-worked mid values, clamping, NaN as 0, the limp frame, no overflow at the top, round trips, the special frames, the reply |
| `test_names` | hvd230can.h, rm.h, upid.h, can.h and every header here in one file under `-Werror` |
| `test_dji` | refused configs; names and command kinds per model; **the feedback clash, GM6020 n against C6x0 n + 4 in both GM6020 modes, and a full bus**; **the frames byte by byte: which identifiers, in order, which slot, what scale**; GM6020 voltage through `dji_set_raw()`; a failed send not stopping the rest; feedback decoded, offline before the first frame, `-ENOENT` for extended, remote, short and unknown frames; **multi-turn across the wrap both ways, and past +-2^31 counts**; temperature on the C620 only; **the timeout per motor, 0 on the bus, and the command back on the next frame**; the DJI-only calls refusing another model's `esc_dev` |
| `test_ht03` | esp-HT03's test_core.c on errno and `can_dev`: slots, special commands and their wait, set zero not sent when the exit fails, reply matching, DLC 6 and the F446 rewrite's 7 heard, 5 and 8 refused, offline and limp, a send error before offline; the `esc_dev`: torque only, the whole command replaced, feedback bits |
| `esp/test_core`, `stm/test_core` | the kind, value and `commands` checks before the op; the op's error passed through; feedback zeroed for the op and untouched on failure; `-EOPNOTSUPP` without `get_feedback`; the log line |
| `esp/test_c615`, `stm/test_c615` | refused endpoints, rates and a `max_us` that fills the period; nothing running after a failed init; throttle 0, 0.5, 1, clamped, rounded to the microsecond; the raw pulse inside the endpoints; del and after. ESP-IDF: the LEDC timer and channel settings, **the duty to the step at 400 and 50 Hz**, the stop level. HAL: **the rate from ARR**, the compare at 1, 84 and 1.5 MHz, the compare 0 before stop, refused handles and channels, a 32-bit ARR that wraps |
| `test_dshot_math` | **[BW]'s worked frame 1046 both ways, `0x82C6` and `0x82C9`**; frames for 0, 48, 2047 and a command, with and without the bit; **bit and high ticks at 40 and 168 MHz against [BW]'s table**, rounding at 100 MHz; throttle 0, the bottom, half, the top, past it; **the reply: [BW]'s GCR example and its worst case, any leading level, quintets outside the table, a wrong or inverted CRC**; every table entry; the period m << e, [EDT]'s 512, 0x0FFF as stopped, period 0 refused, eRPM rounding; EDT against eRPM by prefix; **levels to reply bits at 8 and 168 MHz, a glitch, a low past bit 20, a long rest; all 65536 frames through levels and back, and their longest level 3 bits**; EDT types and values; RPM and rad/s from eRPM |
| `esp/test_dshot`, `stm/test_dshot` | refused counts, speeds and bidirectional; **every motor's frame read back from its pulses**: throttle, raw values, the telemetry bit held and one-shot; **every command with a meaning: its frame count with the bit, the wait after it counted in dt_s, refused while running for 0-36 and taken for 42-47**, refused numbers, another model's `esc_dev`; **raw values 1-47 refused**; a busy line consuming nothing; **a failed send consuming no repeat, no wait, no one-shot bit**; del. ESP-IDF: **each channel at 8 MHz, 48 symbols, queue 1, every symbol one bit**, the sync manager over all channels and reset each frame, none for one motor, a pin below 0, **every setup call failing in turn leaving nothing allocated**, **a transmit failing on the third motor: the first two stopped and re-enabled, the sync reset, the next update going**; `esp/test_dshot_nosync`, the original ESP32's RMT: one motor works, more is `-EOPNOTSUPP`. HAL: **the burst into CCR1, 4 transfers, 68 words, stop before start**, the zero row, motors on channels out of order and an unused slot left 0, compares 0 at init and del, **every wrong DMA setting, channel and ARR refused**, ticks that do not divide, a channel that will not start stopping the others. **Bidirectional, both platforms: DShot150 and odd poles refused; the inverted CRC on every frame and command; eRPM read one update late, RPM and velocity from it, a silent or garbled motor counted and its last value kept, 0x0FFF stopped, offline after `DSHOT_REPLY_TIMEOUT_S`; EDT only after command 13, its version, every type, eRPM still eRPM, off after 0xEFF**; nothing back from a bus that is not bidirectional. ESP-IDF: **TX inverted, an RX channel on each GPIO at 8 MHz, receives armed with 200 ns and 45 us, the pin open-drain after each frame and push-pull before the next**, a busy receiver, replies at both speeds, **29 setup calls each failing in turn leaving nothing**, a receive or transmit failing part-way stopping every receiver; the ESP32 one motor, the C3 two. HAL: **CCxP and pull-ups set, 18 rows started on the DMA with DCR as the burst's, the pins inputs and ARR a third of a reply bit while 167 or 117 half-words come from the IDR, all back after**, motors on pins out of order, a sampling DMA that will not start or fails, **a FIFO or direct-mode error changing nothing and a transfer error mid-frame zeroing the compares**, a frame DMA that will not start, refused pins, **an update DMA on DMA1 refused**, del giving the handle back; another family `-EOPNOTSUPP` |
| `esp/test_port_esp`, `stm/test_port_stm` | the bus layer, as in every category library |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic` with UBSan, and
`-Wl,--wrap` (GNU ld) to capture `printf()` on the HAL side.

**Builds.** The ESP-IDF examples build for esp32s3 on ESP-IDF v5.5.5 with
no warning; since DShot, so does `examples/esp/dshot`, and a copy of it with
every model and the TWAI on, which compiles every source of the library. A scratch app with esp-c620-control, esp-sn65hvd230-can,
mcu-can and mcu-esc, every model on, links; `nm -g --defined-only` over the
four component archives finds no symbol defined by two of them. On
stm-forge, a scratch STM32F405 target with mcu-can (`CAN_BXCAN`), every
model on and Hawk's HAL conf plus the CAN module, links each of
`examples/stm/dji_bxcan` and `examples/stm/c615` against STM32CubeF4 HAL
v1.8.5, with no warning from either library; `examples/stm/dshot` links
the same way with DShot alone, with the C615, and with every model on.
Bidirectional DShot: `examples/esp/dshot_bidir` builds for esp32s3 and
esp32c3, and `examples/esp/dshot` still for esp32s3, on ESP-IDF v5.5.5
with no warning; `examples/stm/dshot_bidir` and `examples/stm/dshot` link
for an STM32F405 against STM32CubeF4 HAL with no warning from the library.
Neither has been run against an ESC.
