# Hardware

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

None of this has been checked on a bench with this library. Page numbers
are the PDF pages of the manuals in `docs/` (gitignored); the
[model reference](chips.md#sources) lists them.

## The CAN models

The C610, C620, GM6020 and HT-03 all hang on one Classic CAN bus at
1 Mbit/s ([C620] p.20, [C610] p.9, [GM6020] p.3, [BK]). The controller and
transceiver are mcu-can's business: a TWAI with an SN65HVD230, an MCP2515
module with a 16 MHz crystal (8 MHz cannot make 1 Mbit/s), or bxCAN with its
transceiver; see mcu-can's `docs/hardware.md`.

Terminate the bus at both ends with 120 ohm. The ESCs carry their own:

| Model | Termination | Source |
|---|---|---|
| C620 | the Termination Resistance Switch, item [3] | [C620] p.5 |
| GM6020 | DIP switch bit 4, ON enables it | [GM6020] p.6 |

### IDs

Each motor's ID decides its identifiers ([model reference](chips.md#dji-can-frames)).
Two motors with one feedback identifier cannot share a bus, and `dji_add()`
refuses the second: a GM6020 at ID n answers where a C610/C620 at ID n + 4
does.

| Model | Range | Set by | Source |
|---|---|---|---|
| C620 | 1-8 | the SET button, "Using the SET Button"; the status LED blinks green the ID's number of times | [C620] pp.10, 12 |
| C610 | 1-8 | the SET button, the same way | [C610] pp.6-7 |
| GM6020 | 1-7 | DIP switch Bit[2:0]; 000 is invalid | [GM6020] p.6 |
| HT-03 | 1-255 here | the motor's own firmware setting | [BK] |

The C620's LED blinks orange twice a second when two controllers on the bus
share an ID ([C620] p.11).

### GM6020: current mode

Torque current control needs motor firmware v1.0.11.2 or later and the
**Current Ring** switch on in RoboMaster Assistant v2.7 or later
([GM6020] p.7). Without it the motor is in voltage mode, and
`gm6020_current_mode` must be false.

### HT-03

The reply comes from the motor's master ID, 0 in stock firmware: mcu-esc
matches replies on their length and first byte, so any master ID works
([model reference](chips.md#haitai-ht-03)).

## C615 (PWM)

| Wire | To | Source |
|---|---|---|
| C, black | GND | [C615] p.3 |
| D, grey | TX: DJI Assistant 2 only; leave open | [C615] p.3 |
| E, white | PWM in: the MCU's timer pin | [C615] p.3 |

The C615 takes 3S-6S LiPo, rated 24 V ([C615] p.2, p.5). The control
signal is a pulse of 400-2200 us, at most 500 Hz ([C615] p.5). The ESC's
own endpoints are set in DJI Assistant 2, or by the **throttle range
calibration** of "PWM Configuration" ([C615] p.4): power up with the pulse at
the maximum, then within two seconds of a double beep drop it to the
minimum; a single beep confirms. Give `c615_init()` the same two endpoints
as `min_us` and `max_us`.

What the beeps say ([C615] p.5): a chime is "Ready to work"; rapid beeping
is "Minimum PWM signal pulse width not reached"; slow beeping is "No PWM
signal" -- which is what `c615_del()` leaves on the pin.

**Which pulse stops the motor is not in the guide.** Check on the bench,
props off, that `min_us` stops it before the first arm.

### On the Hawk flight controller

`hardware/hawk/control/GPIO.md`: TIM8 CH1-CH4 on PC6-PC9 drive the four
C615 headers through 47 ohm, each with a 10 k pull-down, so the pins are low
through reset. `examples/stm/c615/main.c` sets TIM8 up the way Hawk does:
a 1 MHz counter, ARR 2499, 400 Hz, with output-compare preload on each
channel so a new pulse width starts at the next period.

On ESP-IDF the C615 runs on any LEDC-capable pin, through one LEDC timer
and channel per ESC; ESCs at one rate may share the timer.

## DShot

Each ESC's signal wire to one output, grounds common. The line rests low;
every frame is 16 pulses of 3/4 or 3/8 of the bit ([model
reference](chips.md#dshot)). Bidirectional, the line rests high, every
pulse is low, and the ESC answers on the same wire; nothing may pull the
line down while it rests.

**ESP-IDF (the S3 is the target).** One RMT TX channel per motor, any
output-capable GPIO. The S3 has four TX channels with 48 symbols each and can
start several at once (IDF v5.5.5 `soc/esp32s3/include/soc/soc_caps.h:282-291`),
so four motors take all its TX channels; a frame is 16 symbols.

**STM32 HAL.** One timer, up to its four channels, and the timer's update
DMA: every update burst-writes CCR1-CCR4 (`TIM_DMABASE_CCR1`, 4 transfers),
so four motors cost one DMA stream. The timer is the app's: prescaler and
ARR so that one counter period is one bit, repetition counter 0,
output-compare preload on the channels, the update DMA word-to-word in
normal mode with its IRQ handled. A timer runs DShot or C615 PWM, not both.

### On the Hawk flight controller

| Motor | Pin | Timer | AF | Board | Source |
|---|---|---|---|---|---|
| M1 | PC6 | TIM8_CH1 | 3 | 47 R in series, 10 k pull-down | `hardware/hawk/control/GPIO.md` |
| M2 | PC7 | TIM8_CH2 | 3 | the same | the same |
| M3 | PC8 | TIM8_CH3 | 3 | the same | the same |
| M4 | PC9 | TIM8_CH4 | 3 | the same | the same |

TIM8 runs at 168 MHz on Hawk's clock tree (APB2 84 MHz, timers at twice
that; `project/hawk/src/board.c:22-23, 50`), so with prescaler 0 a DShot600
bit is ARR 279, DShot300 559, DShot150 1119. **TIM8_UP is DMA2 stream 1,
channel 7**, its only stream (RM0090 Rev 3, Table 34 "DMA2 request mapping",
p.217; `docs/rm0090.pdf`, gitignored); the library takes the app's DMA
handle and never names it.

**DMA streams on Hawk.** Table 34 gives the requests TIM8 and the IMU's SPI1
could use:

| Request | DMA2 stream / channel | Needed by |
|---|---|---|
| TIM8_UP | 1 / 7 | DShot, now |
| TIM8_CH1 | 2 / 7, or 2 / 0 | nothing: bidirectional DShot samples through TIM8_UP |
| TIM8_CH2 | 3 / 7, or 2 / 0 | the same |
| TIM8_CH3 | 4 / 7, or 2 / 0 | the same |
| TIM8_CH4 | 7 / 7 (shared with TIM8_TRIG, TIM8_COM) | the same |
| SPI1_RX | 0 / 3, or 2 / 3 | the IMU, if its SPI1 ever moves to DMA |
| SPI1_TX | 3 / 3, or 5 / 3 | the same |

A stream serves one channel at a time (9.3.3, p.216). Hawk's firmware uses
no DMA today -- SPI1 is polled (`project/hawk/src`, no DMA call) -- so
nothing competes with stream 1 now. **No collision with SPI1**: DShot, plain or
bidirectional, takes stream 1 only, and SPI1 can have any of its streams.

**Hawk cannot run DShot as built.** Its four ESCs are DJI C615s, which take
PWM only: the C615 guide lists 400-2200 us and at most 500 Hz and never
mentions DShot, and `GPIO.md` marks TIM8 "DJI C615 PWM (no DShot)". DShot on
Hawk means other ESCs on the same headers.

**Bidirectional DShot needs a board change too.** Its line rests high and
the ESC drives the reply ([BW] "Bidirectional DSHOT"); the 10 k pull-downs
on PC6-PC9 pull the other way while nobody drives it. They have to become
pull-ups, or go, before bidirectional DShot can work there.

### Bidirectional DShot

- **ESP-IDF:** an RMT RX channel per motor on the motor's own GPIO, besides
  its TX channel -- all four of the S3's RX channels for four motors, both
  of the C3's for two ([model reference](chips.md#bidirectional-dshot)).
  The pin is push-pull while a frame goes out and open-drain while the ESC
  answers; the RX channel's internal pull-up holds the line high in
  between, and the ESC drives its reply. Per frame and motor: a receive
  armed before sending, two interrupts (TX done, RX done), and the decoding
  in the next `dshot_update()`. The TX-done interrupt must come within the
  30 us before the reply, and a flash write holds off a non-cache-safe
  interrupt for longer: set `CONFIG_RMT_TX_ISR_CACHE_SAFE` and
  `CONFIG_RMT_RX_ISR_CACHE_SAFE` (IDF v5.5.5 `esp_driver_rmt/Kconfig`; the
  library's callbacks are in IRAM and touch only the bus, which must then
  be in internal RAM), as `examples/esp/dshot_bidir` does.
- **STM32F4:** TIM1 or TIM8, whose update DMA is on DMA2 -- DMA1, which
  TIM2-5 use, cannot reach the GPIO (RM0090 2.1.5, p.52), so their bus is
  `-EOPNOTSUPP`. Every motor's pin on one GPIO port -- PC6-PC9 on Hawk --
  in alternate-function mode; the library sets their pull-ups and the
  channels' polarity. Per frame: two DMA interrupts, in which the pins
  turn to inputs and back and the stream turns round. The first must run
  within about 15 us of the frame's end, half the 30 us before the reply:
  give the stream's IRQ (DMA2_Stream1 on Hawk) a high priority, above
  anything that runs long. Nothing else may use
  the update DMA handle or write the port's MODER from an interrupt that
  can preempt the DMA's.
- **Either:** a frame and its reply take up to about 130 us at DShot600
  and 185 at DShot300, so the loop runs no faster than that. Not yet run
  against an ESC.
