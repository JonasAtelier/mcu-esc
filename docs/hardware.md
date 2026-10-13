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
