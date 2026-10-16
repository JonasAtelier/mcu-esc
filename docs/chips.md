# Model reference

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

**Scope.** mcu-esc covers brushless motor drivers that take a command and
return feedback where they have it, including motors with the driver built
in (the GM6020, the HT-03). Steppers belong to mcu-stepper; hobby servos and
the PCA9685 are out of scope.

## Sources

Page numbers are the PDF's own pages, counted from 1.

| Short | Document | Where |
|---|---|---|
| C620 | RoboMaster C620 Brushless DC Motor Speed Controller User Manual V1.0, 2017.09 | `docs/C620_user_manual_v1.0.pdf` (gitignored); the same file as esp-c620-control's `doc/C620_datasheet.pdf` |
| C610 | RoboMaster C610 Brushless DC Motor Speed Controller User Guide v1.0, 2019.03 | `docs/C610_user_guide_v1.0.pdf` (gitignored) |
| GM6020 | RoboMaster GM6020 Brushless DC Motor User Guide v1.4, 2023.10, from `rm-static.djicdn.com/tem/17348/` | `docs/GM6020_user_guide_v1.4.pdf` (gitignored) |
| GM6020 v1.2 | the same, v1.2, 2020.05, for comparison | `docs/GM6020_user_guide_v1.2.pdf` (gitignored) |
| C615 | RoboMaster C615 Brushless DC Motor Speed Controller User Guide V1.0, 2019.10 | `docs/C615_user_guide_v1.0.pdf` (gitignored) |
| BK | Ben Katz, `motorcontrol`, the MIT Mini Cheetah actuator firmware: the STM32F446 rewrite, master `93bbc606` | read, not built; files and lines below |
| SCUT | SCUT-RobotIC/RC2024-THE-DOG, `CANFORF407ZG/HT03/MDK-ARM/HT03.h` | esp-HT03's source for the J6 ranges, not re-checked |

## DJI CAN frames

All three: standard identifiers, DATA frames, DLC 8, every 16-bit field high
byte first, 1 Mbit/s.

| What | C610 | C620 | GM6020 |
|---|---|---|---|
| Bit rate | 1 Mbps, C610 p.9 | 1 Mbps, C620 p.20 | 1 Mbps, GM6020 p.3 |
| IDs | 1-8 (C610 p.8) | 1-8 (C620 p.12) | 1-7, DIP Bit[2:0], 000 invalid (GM6020 p.6) |
| Command, IDs 1-4 | 0x200 (p.8) | 0x200 (p.14) | voltage 0x1FF (p.7), current 0x1FE (p.8) |
| Command, IDs 5-8 | 0x1FF (p.8) | 0x1FF (p.15) | voltage 0x2FF (p.7), current 0x2FE (p.8); IDs 5-7, DATA[6..7] Null |
| Command slot | DATA[0..1] ID 1/5, [2..3] 2/6, [4..5] 3/7, [6..7] 4/8 | same | same |
| Command range | -10000..10000 = -10..10 A (p.8) | -16384..16384 = -20..20 A (p.14) | voltage -25000..25000, no unit (p.7); current -16384..16384 = -3..3 A (p.8) |
| Feedback identifier | 0x200 + ID (p.8) | 0x200 + ID (p.16) | 0x204 + ID, 0x205-0x20B (pp.6, 8) |
| DATA[0..1] | rotor angle 0..8191 = 0..360 deg (p.9) | same (p.17) | same (p.9) |
| DATA[2..3] | rotor speed, rpm (p.9) | same (p.17) | same (p.9) |
| DATA[4..5] | actual torque current, **no scale given** | same | same |
| DATA[6] | Null (p.9) | motor temperature, deg C (p.17) | motor temperature, **no unit given** (p.9) |
| DATA[7] | Null | Null | Null |
| Feedback rate | 1 kHz default, changeable (p.9) | 1 kHz default (p.17) | 1 kHz (p.9) |

In `src/dji/dji_proto.h`, each value with its citation; held by
`test/test_dji_math.c`.

### What the manuals do not give, and what mcu-esc does about it

- **The feedback torque current's scale**, on all three. esp-c620-control
  assumed the command's +-16384 = +-20 A; nothing in the C620 manual says
  so. mcu-esc reports it only as raw counts, `dji_raw.current`, and leaves
  `ESC_HAS_TORQUE` clear.
- **The GM6020's temperature unit.** Raw only, `dji_raw.temperature`;
  `ESC_HAS_TEMPERATURE` is set for the C620 alone.
- **The GM6020's voltage unit.** Voltage mode is `dji_set_raw()` only; there
  is no `ESC_COMMAND_VOLTAGE`.

### GM6020 v1.2 and v1.4

v1.2 gives the voltage range as -30000..30000 (p.8) and has no current mode.
v1.4 gives -25000..25000 (p.7) and adds current mode for firmware
v1.0.11.2+ with Assistant v2.7+ (p.7). mcu-esc clamps to 25000.

### Identifier clashes

A GM6020 at ID n has the feedback identifier of a C610/C620 at ID n + 4, and
in voltage mode the same command slot (0x1FF slot n - 1 is C6x0 ID n + 4's).
Any two motors that would share a command slot also share a feedback
identifier, so `dji_add()` checks the feedback identifier alone. Eleven
motors fill every identifier from 0x201 to 0x20B.

### Multi-turn position

The feedback angle covers one rotor turn. `dji_handle_frame()` adds the step
between two frames, taking any step over half a turn (4096 counts) as the
wrap: at the default 1 kHz that follows up to 30000 rotor rpm. Position
counts from the first frame after `dji_add()`. Gear ratios are the app's:
`gear_ratio` divides position and velocity, and esp-c620-control's M3508
figure is 3591/187.

## C615

| What | Value | Source |
|---|---|---|
| Control throttle range | 400-2200 us, settable in DJI Assistant 2 | C615 p.5, Specifications |
| Max control signal frequency | 500 Hz | C615 p.5 |
| Throttle range calibration | power up at the maximum pulse, drop to the minimum within 2 s of the double beep; one beep confirms | C615 p.4, PWM Configuration |
| Rapid beeping | minimum PWM pulse width not reached | C615 p.5 |
| Slow beeping | no PWM signal | C615 p.5 |
| PWM cable | C black GND, D grey TX, E white PWM/RX | C615 p.3 |
| Supply | 3S-6S LiPo; rated DC 24 V, max 26 V | C615 p.2, p.5 |

**Not in the guide:** the pulse that stops the motor, the idle pulse, the
direction of the throttle, a lowest rate. mcu-esc maps throttle 0..1 linearly
onto the app's calibrated `min_us..max_us`, starts at `min_us`, and refuses a
rate over 500 Hz or a `max_us` that leaves no low time in one period.
Hawk's 1000/2000 us at 400 Hz are its own choice and still "bench-confirm on
the C615" in `project/hawk/README.md`.

## Haitai HT-03

The HT-03 (HT8108-J6, 1:6) runs Ben Katz's MIT Mini Cheetah firmware: its
own controller computes the torque and closes the loop. The layout in
`include/ht03.h` is esp-HT03's; it was re-checked for this library against
the firmware's **STM32F446 rewrite** at `93bbc606`. **The HT-03 ships the
older mbed firmware**, which was not available, so every agreement below is
with the rewrite.

| What | Rewrite says | mcu-esc |
|---|---|---|
| Control command, 8 bytes | `Core/Src/can.c:189-204` `unpack_cmd()`: position 16 bit in bytes 0-1, velocity 12 bit in 2 and 3's high nibble, kp 12 bit in 3's low nibble and 4, kd 12 bit in 5 and 6's high nibble, torque 12 bit in 6's low nibble and 7 | the same, `ht03_pack_command()` |
| Scaling | `Core/Src/math_ops.c:44-57` `float_to_uint()` / `uint_to_float()`: `(x - min) * (2^bits - 1) / span`, cast truncating | the same, plus a clamp and NaN as 0 |
| Special commands | `Core/Src/stm32f4xx_it.c:298-317`: an 8-byte frame of `FF x7` then `FC` motor mode, `FD` menu (rest) mode, `FE` zero; every 8-byte frame is answered | the same, `ht03_enable/disable/set_zero()` |
| Torque law | `Core/Src/foc.c:289-291` `torque_control()`: `kp * (p_des - theta) + t_ff_filt + kd * (v_des - dtheta)`, with `t_ff_filt = 0.9 t_ff_filt + 0.1 t_ff` per loop | `ESC_COMMAND_TORQUE` is kp = kd = 0 with `t_ff`: torque only, through the rewrite's low-pass |
| Reply torque | `stm32f4xx_it.c:301`: `i_q_filt * KT * GR`, newton metres, over `+-(I_MAX + SENSE_BUFFER) * KT * GR`; `SENSE_BUFFER` is 0 (`Core/Inc/can.h:49`) | decoded as Nm over `+-torque_max` |
| Reply length | `can.c:157-171` `pack_reply()`: DLC 7, byte 6 the bus voltage | DLC 6, as esp-HT03 has it from the mbed firmware (not re-checked here), or 7 from the rewrite; byte 6 is ignored. Never 8, which a DJI feedback frame could alias |
| **Motor ID** | `Core/Inc/can.h:56, 64, 77-78`: the identifier carries an opcode in bits 7 and up; the legacy protocol is opcode 0, IDs 0-127 | IDs 1-255, as esp-HT03; on the rewrite only 1-127 reach the legacy protocol |
| Ranges | read from the motor's flash (`CAN_VAR_P_MAX` and on) | `HT03_J6`: 95.5 rad, 45 rad/s, 500 Nm/rad, 5 Nm s/rad, 18 Nm, from SCUT, not re-checked |

Replies are matched on DLC 6 or 7 and byte 0, not on the identifier, because the
master ID is a firmware setting. A C620 feedback frame is DLC 8 and never
matches.

Some HT-03 documents call the reply's torque field a current, +-40 A; the
firmware's line above makes it newton metres, and so does mcu-esc.

## DShot

There is no official DShot specification. These are the sources, none of
them in the repository; read on 2026-10-14:

| Short | Document | Where |
|---|---|---|
| BW | Chris Landa, "DSHOT - the missing Handbook", 2021-02-27 | https://brushlesswhoop.com/dshot-and-bidirectional-dshot/ |
| BLH | BLHeli_32 `Digital_Cmd_Spec.txt` | https://raw.githubusercontent.com/bitdump/BLHeli/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt |
| EDT | Extended DShot Telemetry specification, README | github.com/bird-sanctuary/extended-dshot-telemetry @ `438b87983b58369e3e46068b0b975634f4ea5416` |
| BF | Betaflight `src/main/drivers/dshot.c` and `dshot_bitbang_decode.c` -- GPL, read for facts only, nothing copied | github.com/betaflight/betaflight @ `4b7fa8e40a562a20b328fc50bd8829d6bded08eb` |
| IDF | ESP-IDF v5.5.5 `examples/peripherals/rmt/dshot_esc` (Apache-2.0), a cross-check | `~/esp-idf` |
| RM0090 | ST RM0090 Rev 3, STM32F40x/41x/42x/43x reference manual (Doc ID 018909): the timer DMA burst, the DMA2 request mapping and bus, the GPIO registers | `docs/rm0090.pdf` (gitignored) |
| Bluejay | github.com/bird-sanctuary/bluejay @ `0368d114b72f3939ea49c671b199bf0d13d9079d`: only that it supports DShot 300 and 600 and bidirectional DShot (README) | |

The constants are in `src/dshot/dshot_proto.h`, the arithmetic in
`src/dshot/dshot_math.h`; `test/test_dshot_math.c` holds both to the
sources' own worked examples.

### The frame

| What | Value | Source |
|---|---|---|
| Frame | 16 bits, most significant first: 11-bit value, telemetry bit, 4-bit CRC (`SSSSSSSSSSSTCCCC`) | BW "Frame Structure" |
| Values | 0 disarmed (motor stop), 1-47 special commands, 48-2047 throttle | BW "Frame Structure" |
| CRC | `(d ^ (d >> 4) ^ (d >> 8)) & 0x0F`, d the 12 bits before it; 1046 without telemetry is `1000001011000110` | BW "Calculating the CRC"; IDF `dshot_esc_encoder.c:41` |
| CRC, bidirectional | the same, inverted before the mask: `1000001011001001` | BW "Bidirectional DSHOT" |
| Bit timing | a 1 high for 3/4 of the bit, a 0 for 3/8: DShot150 bit 6.67 us, T1H 5.00, T0H 2.50; DShot300 3.33, 2.50, 1.25; DShot600 1.67, 1.25, 0.625 | BW "Frame Structure" table; IDF uses 0.7485 and 0.37425 (`:132-134`) |
| Frame time | 16 bit periods: 106.72, 53.28, 26.72 us | BW table |
| Arming | most firmware wants value 0 for a while first; 300 ms on Bluejay | BW "Arming sequence" |

**Frame gap.** No source gives a minimum gap between frames: [BW] has the
flight controller send one frame per control loop. The library sends one
per `dshot_update()` and refuses (`-EBUSY`) while the last one is still on
the wire; on the STM32 a row of zeros after bit 15 returns the line low.

**The STM32's burst** [RM0090]. The update DMA writes the timer's DMAR,
which lands on CR1 + (DCR.DBA + index) x 4 (14.4.20, p.415). DBA is the
first register as a word offset from CR1 and DBL the count less one
(14.4.19, p.414); CCR1-CCR4 are at 0x34-0x40 (Table 72, p.417), so DBA 13
and DBL 3 write all four compares per update. The DMA's count is every word
of every update -- 17 rows of 4, 68 -- in normal mode, as the manual's own
example has it (p.415). Word writes are allowed (14.4, p.390). With OCxPE
set, a compare takes effect at the next update (14.4.7, p.402), so each
row is one bit, one period later. On the F405, TIM8_UP is DMA2 stream 1,
channel 7 (Table 34, p.217).

**What mcu-esc picks.** On ESP-IDF the RMT runs at 8 MHz, which divides
every chip's default RMT clock -- 80 MHz on the ESP32, S2, S3, C3, C5, C6
and P4, the H2's 32 MHz crystal (`RMT_CLK_SRC_DEFAULT`,
`soc/<chip>/include/soc/clk_tree_defs.h`, IDF v5.5.5) -- so no chip gets
another resolution than asked:

| Rate | Bit | vs [BW] | 1 high | 0 high |
|---|---|---|---|---|
| DShot150 | 53 ticks, 6.625 us | 6.67 us, -0.7 % | 40, 5.000 us | 20, 2.500 us |
| DShot300 | 27 ticks, 3.375 us | 3.33 us, +1.3 % | 20, 2.500 us | 10, 1.250 us |
| DShot600 | 13 ticks, 1.625 us | 1.67 us, -2.5 % | 10, 1.250 us | 5, 0.625 us |

The highs, which tell a 1 from a 0, are [BW]'s exactly; the bit period is
off by the rounding shown. No source gives a tolerance for either. On the
STM32 one counter period is one bit, the app's ARR; Hawk's 168 MHz TIM8
gives 1120, 560 and 280 ticks, exact.

**Which ESP chips.** DShot needs an RMT (`SOC_RMT_SUPPORTED`): the ESP32,
S2, S3, C3, C5, C6, H2 and P4 have one, the C2 and C61 do not -- there
`ESC_DSHOT` stops the build with an `#error`. Several motors need the RMT
to start channels together (`SOC_RMT_SUPPORT_TX_SYNCHRO`), which the
original ESP32 lacks: on it `count` above 1 is `-EOPNOTSUPP`, one motor
works. (`soc/<chip>/include/soc/soc_caps.h`, IDF v5.5.5.)

### Special commands

Every command frame has the telemetry bit set, with throttle zero [BLH line
30]; 0-36 are executed only with the motor stopped [BW], so `dshot_command()`
refuses them while the motor's value is not 0. After a command, the motor is
held at value 0 for the wait below, counted in `dshot_update()`'s `dt_s`.

| Number | Command | Frames | Then wait | Source |
|---|---|---|---|---|
| 1, 2 | BEEP1, BEEP2 | 1 | 260 ms | BLH 2-3 |
| 3, 4 | BEEP3, BEEP4 | 1 | 280 ms | BLH 4-5 ([BW] says 260) |
| 5 | BEEP5 | 1 | 1020 ms | BLH 6 ([BW] says 260) |
| 6 | ESC_INFO | 1 | 12 ms | BLH 7 |
| 7, 8 | SPIN_DIRECTION_1, _2 | 6 | -- | BLH 8-9 |
| 9, 10 | 3D_MODE_OFF, _ON | 6 | -- | BLH 10-11 |
| 11 | SETTINGS_REQUEST ("not implemented") | 1 | -- | BLH 12 |
| 12 | SAVE_SETTINGS | 6 | 35 ms | BLH 13 |
| 13, 14 | EXTENDED_TELEMETRY_ENABLE, _DISABLE | 6 | 35 ms | BLH 14-15; EDT "Enabling EDT" |
| 20, 21 | SPIN_DIRECTION_NORMAL, _REVERSED | 6 | -- | BLH 16-17 |
| 22-29 | LED0-3 on, off | 1 | -- | BLH 18-25 |
| 30, 31 | audio stream, silent mode ("not implemented") | 1 | -- | BLH 26-27 |
| 32-35 | signal-line telemetry off, on, continuous eRPM, continuous eRPM period | 6 | -- | BW |
| 42-47 | signal-line temperature, voltage, current, consumption, eRPM, eRPM period | 1 | -- | BW |

15-19, 36-41 and anything past 47 have no meaning: `-EINVAL`. [BLH] adds
that "a significant delay (several hundred ms) may be needed between
commands" (line 31); that is the app's to add. Betaflight sends 10 repeats
rather than 6, 1 ms apart [BF `dshot_command.c:39, 191-205`]; mcu-esc sends the
6 the specification asks for, one per `dshot_update()`.

### Bidirectional DShot

| What | Value | Source |
|---|---|---|
| Line | inverted: idle high, every bit starts low; the frame's CRC inverted | BW "Bidirectional DSHOT" |
| Rates | DShot300 and up only; below is `-EINVAL` | BW |
| Reply | 21 bits at 5/4 of the bit rate (375 / 750 kbit/s), not inverted: a 0, so the line falls, then 20 GCR bits sent as changes of level (a 1 toggles) | BW "eRPM Transmission" |
| When | 30 us after the frame, whatever the speed | EDT "Timing"; BW "Bidirectional DSHOT" |
| GCR | nibble 0-F to 19 1B 12 13 1D 15 16 17 1A 09 0A 0B 1E 0D 0E 0F; decode `gcr = raw ^ (raw >> 1)` | BW "eRPM Transmission", "Decoding" |
| Longest level | 3 bits: no quintet has more than two 0s in a row, none starts or ends with more than one | the table above; `test_dshot_math` checks all 65536 frames |
| Frame | `eeemmmmmmmmmcccc`: period in us = m << e; CRC the plain one | BW "eRPM Telemetry Frame"; EDT "Checksum"; BF `dshot.c:214` |
| Stopped | data `0x0FFF` | BF `dshot.c:209-211` |
| eRPM | 60 000 000 / period, rounded (one electrical turn per period) | arithmetic |
| RPM | eRPM / (poles / 2), rounded | BF `dshot.c:393-395` |
| Sampling | Betaflight's bit-banged decoder reads the line at 3 samples a bit; mcu-esc's STM32 port does the same | BF `dshot_bitbang_decode.c:181-183` |

**Extended DShot Telemetry.** Once command 13 has gone out six times, an
EDT firmware answers with a version frame and from then on sends, now and
then instead of eRPM, frames whose prefix (bits 11-8) is not 0 and has bit 8
clear: type in bits 11-9, value in 7-0 [EDT "eRPM Frames", "EDT Frames",
"Enabling EDT"]. eRPM frames are normalised so they never look like that;
before command 13 every reply is eRPM, which mcu-esc keeps to.

| Type | Bits 11-8 | Value | Source |
|---|---|---|---|
| Temperature | 001 0 | deg C, 0-255 | EDT "EDT Frames" |
| Voltage | 010 0 | 0.25 V a step, 0-63.75 V | EDT |
| Current | 011 0 | A, 0-255 | EDT |
| Debug 1, 2 | 100 0, 101 0 | the firmware's own | EDT |
| Stress level | 110 0 | 0-255 (since EDT v2.0.0; [BW] still calls it debug 3) | EDT; BW "Extended DShot Telemetry (EDT)" |
| Status | 111 0 | bit 7 alert, 6 warning, 5 error, 3-0 highest stress (since v2.0.0) | EDT |
| Version | 111 0 | the status-type frame right after command 13 | EDT "Enabling EDT" |
| Off | `0xEFF` | the answer to command 14; mcu-esc reads eRPM again after it | EDT "Disabling EDT" |

**What mcu-esc picks, and why.**

- **When a reply is read.** The answer to a frame is read by the next
  `dshot_update()`, before it sends: one update of latency, no interrupt
  work in the library beyond turning the line round.
- **How long it listens.** 45 us after the frame for the reply to start --
  [EDT]'s 30 us and half again (`DSHOT_REPLY_WAIT_US`), a margin no source
  gives -- then 22 reply bits. A reply that starts later is lost.
- **ESP-IDF.** The TX channel's output is inverted (`invert_out`), so the
  frame's symbols are plain DShot's and the line rests high. An RMT RX channel
  on the same GPIO, as IDF's own 1-wire bus does it (IDF v5.5.5
  `docs/en/api-reference/peripherals/rmt.rst:628`), is armed before each
  frame and hears the frame, the rest, the reply, and stops once the line
  has rested `DSHOT_REPLY_WAIT_US` (`signal_range_max_ns`); pulses under
  200 ns are glitches (`signal_range_min_ns`; the shortest real one is
  375 ns). The reply is what follows the first level longer than any a
  reply holds. While the frame goes out the pin is push-pull; the TX-done
  interrupt turns it open-drain (`gpio_ll_od_enable()`, inline, so the
  callback needs no flash) and the next send turns it back, so the ESC can
  drive the reply against nothing but the RX channel's pull-up. A flash
  write would hold that interrupt off past the reply unless the RMT's ISRs
  are cache-safe (`CONFIG_RMT_TX_ISR_CACHE_SAFE`,
  `CONFIG_RMT_RX_ISR_CACHE_SAFE`).
- **Which ESP chips.** Every chip with an RMT: the RX channel is a second
  channel per motor. The S3 and P4 have four RX channels of their own; the
  C3, C5, C6 and H2 two; the ESP32 and S2 have channels that each do
  either, eight and four (`SOC_RMT_RX_CANDIDATES_PER_GROUP`,
  `SOC_RMT_CHANNELS_PER_GROUP`, `soc/<chip>/include/soc/soc_caps.h`, IDF
  v5.5.5). So four motors on the S3 and P4, two on the S2, C3, C5, C6 and
  H2, one on the ESP32 (no TX sync); more is `-EOPNOTSUPP`.
- **STM32.** The same timer and the same update DMA stream: no capture
  channels, no second stream. The burst gets a second row of zeros, so its
  transfer-complete interrupt comes with the line already at rest. That
  interrupt turns the motors' pins to inputs (pull-ups set at init), sets
  the timer to a third of a reply bit -- 75 ticks at 168 MHz for DShot600,
  149 for DShot300 -- and points the stream at the port's input register,
  half-word by half-word, for 45 us and 22 reply bits: 167 and 117 samples.
  The next interrupt puts everything back. One port read samples all four
  motors at once, which is why their pins must share a port. It needs
  DMA2, whose peripheral port reaches the AHB1 GPIO through the bus matrix
  (RM0090 2.1.5, p.52; Figure 25 note 1, p.214) -- TIM8_UP is DMA2 stream
  1, TIM1_UP DMA2 stream 5 (Table 34, p.217); TIM2-5's update requests
  are on DMA1 (Table 33, p.216), whose streams sit below DMA2's (0x4002
  6000 against 0x4002 6400, Table 2, p.53), so an update DMA below
  `DMA2_Stream0` is `-EOPNOTSUPP` -- and it is built for the STM32F4 only,
  the family RM0090 covers; elsewhere `bidirectional` is `-EOPNOTSUPP`.
  Only a transfer error stops the stream (`HAL_DMA_IRQHandler()`): then
  the compares go 0 and the line rests; FIFO and direct-mode errors are
  left alone.

| RM0090 register | What the STM32 port writes | Where |
|---|---|---|
| TIMx_CCER CCxP | 1: output active low, so compare 0 holds the line high | 14.4.9, p.405; PWM mode 1, 14.4.7, p.402 |
| TIMx_EGR UG | reload ARR and the counter at once when the period changes | 14.4.6, p.399 |
| TIMx_DIER UDE | update DMA off while switching, on to run | 14.4.4, p.396 |
| TIMx_DCR | CCR1, 4 transfers, as the HAL's burst sets it | 14.4.19, p.414 |
| GPIOx_MODER | 00 input between frames, 10 alternate function for them | 7.4.1, p.198 |
| GPIOx_PUPDR | 01 pull-up, at init; left so at del | 7.4.4, p.200 |
| GPIOx_IDR | read by the DMA as a half-word, which GPIO registers allow | 7.4, p.198; 7.4.5, p.200 |
