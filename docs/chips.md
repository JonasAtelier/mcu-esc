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
