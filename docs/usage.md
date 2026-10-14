# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

Every call returns 0 or a negative errno, on both platforms. Nothing here
locks: call everything for one bus, or one C615, from one task, or hold your
own mutex around it.

## The esc_dev

Each model hands out one `struct esc_dev *` per motor:

| Call | Does |
|---|---|
| `esc_set_command(esc, kind, value)` | stores a command, clamped to the model's range; `-EOPNOTSUPP` for a kind the model does not take, `-EINVAL` for a value that is not finite |
| `esc_get_feedback(esc, &fb)` | the last feedback, decoded; `-EOPNOTSUPP` on the C615 |
| `esc_log_feedback(esc, &fb)` | one info line of the fields it has |

| Kind | Unit | Taken by |
|---|---|---|
| `ESC_COMMAND_CURRENT` | amperes of torque current | C610, C620, GM6020 in current mode |
| `ESC_COMMAND_TORQUE` | newton metres at the output | HT-03 |
| `ESC_COMMAND_THROTTLE` | 0..1 of the calibrated range; DShot: 0 stop, then 48..2047 | C615, DShot |

`esc->commands` holds the kinds a motor takes, as `ESC_COMMAND_BIT(kind)`.

`struct esc_feedback` has `online` and, for each field a model fills, a bit
in `has`; the others read 0:

| Field | Bit | Unit | From |
|---|---|---|---|
| `angle_rad` | `ESC_HAS_ANGLE` | 0..2pi, one turn of the shaft the driver reads | DJI: the rotor |
| `position_rad` | `ESC_HAS_POSITION` | output shaft, multi-turn | DJI: counted since `dji_add()`, over `gear_ratio`; HT-03: from its zero |
| `velocity_rad_s` | `ESC_HAS_VELOCITY` | output shaft | DJI: rotor rpm over `gear_ratio`; HT-03 |
| `torque_nm` | `ESC_HAS_TORQUE` | output shaft | HT-03 |
| `temperature_c` | `ESC_HAS_TEMPERATURE` | deg C | C620 |

A command reaches the motor through the model: at the next `dji_update()`
or `ht03_update()` on CAN, at once on the C615.

## The DJI bus

```c
static struct dji_bus bus;              /* one per CAN bus, placed by the app */

dji_init(&bus, can);
dji_add(&bus, &(struct dji_motor_config){
            .model = DJI_MODEL_GM6020, .id = 3,
            .gm6020_current_mode = true }, &yaw);
```

| Call | Does |
|---|---|
| `dji_init(&bus, can)` | empties the bus; `can` is started by the app and outlives it |
| `dji_add(&bus, &cfg, &esc)` | places one motor and hands out its `esc_dev`; `-EINVAL` for an ID outside the model's range or a bad ratio or timeout, `-EOPNOTSUPP` for a model built out, `-EADDRINUSE` when its feedback identifier is taken |
| `dji_handle_frame(&bus, &frame)` | feeds one received frame; `-ENOENT` when it is not a feedback frame of a motor on this bus |
| `dji_update(&bus, dt_s)` | ages the feedback by `dt_s`, then sends every command frame that has a motor in it, without waiting for room |
| `dji_set_raw(esc, counts)` | a command in the model's own counts, clamped: GM6020 voltage mode's only way in |
| `dji_get_raw(esc, &raw)` | the last feedback as it came, and the command as it goes out |

`struct dji_motor_config`: `model`, `id` (as set on the ESC), `gear_ratio`
(rotor turns per output turn, 0 for 1), `timeout_s` (0 for 0.1 s),
`gm6020_current_mode`.

**Offline.** A motor whose feedback is older than its timeout is offline:
`fb.online` false, and `dji_update()` sends it 0 while keeping its stored
command, which goes out again with the first feedback frame. Every motor
starts offline. `dji_update()` returns `-ENOTCONN` while any motor is
offline, and a `can_send()` error before that.

**Frames sent.** Only the command identifiers that have a motor:
0x1FE, 0x1FF, 0x200, 0x2FE, 0x2FF in that order, each once per call.

**Sharing the bus** with other devices: run your own receive loop and pass
each frame to `dji_handle_frame()`, then to the next handler when it
returns `-ENOENT`.

## The HT-03

esp-HT03's calls, unchanged in name and meaning:

| Call | Does |
|---|---|
| `ht03_init(&ht, can)` | empties every slot |
| `ht03_cfg(&ht, id, &range)` | takes a motor ID with its range (`HT03_J6`); offline and limp until it replies |
| `ht03_set_timeout(&ht, id, s)` | reply-loss threshold, 0.1 s by default |
| `ht03_enable/disable(&ht, id)` | motor mode in and out, sent at once; the stored command is reset to limp |
| `ht03_set_zero(&ht, id)` | exits motor mode, then zeroes; leaves the motor disabled |
| `ht03_command(&ht, id, p, v, kp, kd, t)` | the MIT command, stored; `_torque`, `_position`, `_velocity` zero the rest |
| `ht03_handle_frame(&ht, &frame)` | feeds one received frame; `-ENOENT` when it is not a configured motor's reply |
| `ht03_update(&ht, dt_s)` | sends every motor its command -- limp while it is offline -- which is also what makes it reply |
| `ht03_get_feedback(&ht, id, &fb)` | position, velocity, torque, online |
| `ht03_esc(&ht, id)` | the motor's `esc_dev` |

`ESC_COMMAND_TORQUE` through the `esc_dev` replaces the whole stored command
with kp = kd = 0 and the torque as feed-forward.

### Changes from esp-HT03

| esp-HT03 | mcu-esc |
|---|---|
| `hvd230_t *` | `struct can_dev *`, any of mcu-can's controllers |
| `ht03_sta`: `HT03_OK`, `EINVAL`, `ENOTCFG`, `ENOSLOT`, `EOFFLINE`, `ECAN` | `int`: 0, `-EINVAL`, `-ENOENT`, `-ENOSPC`, `-ENOTCONN`, `can_send()`'s error |
| `ht03_handle_frame()` returns `EINVAL` for a frame not its own | `-ENOENT`, so a dispatch loop can tell "not mine" from a bad call |
| `ht03_poll()` drained the bus | gone: `while (can_recv(can, &f, 0) == 0) ht03_handle_frame(&ht, &f);` |
| a FreeRTOS recursive mutex inside | no lock; one task, or your own |
| -- | `ht03_esc()` |

## The C615

```c
static struct c615 esc;
struct c615_config cfg = {
    .gpio = 7, .timer = LEDC_TIMER_0, .channel = LEDC_CHANNEL_0,
    .rate_hz = 400,                       /* STM32: .tim, .channel, .tick_hz */
    .min_us = 1000, .max_us = 2000,       /* the calibrated endpoints */
};

c615_init(&esc, &cfg);                    /* the pin starts at min_us */
esc_set_command(c615_esc(&esc), ESC_COMMAND_THROTTLE, 0.3f);
```

| Call | Does |
|---|---|
| `c615_init(&c615, &cfg)` | checks the config, sets the pin up and starts it at `min_us`; nothing left running on failure |
| `c615_set_us(&c615, us)` | the pulse itself, inside `min_us..max_us` |
| `c615_esc(&c615)` | the `esc_dev`; NULL before init and after del |
| `c615_del(&c615)` | stops the pulses: the pin stays low |

Throttle 0..1 maps linearly onto `min_us..max_us`, rounded to the
microsecond. `c615_init()` refuses endpoints outside 400..2200 us or out of
order, a rate of 0 or over 500 Hz, and a `max_us` that leaves no low time in
a period. On ESP-IDF the library sets up the LEDC timer and channel; on the
HAL the timer is the app's, and the library reads the rate back from ARR
and `tick_hz`. **On the HAL, turn on output-compare preload** for the
channel (CCMRx OCxPE): the library writes the compare whenever a command
comes, and without preload a write in the middle of a period takes effect
at once, so the pulse in flight can come out short or doubled.

## DShot

```c
static struct dshot_bus bus;
struct dshot_config cfg = {
    .speed = DSHOT_600, .count = 4,
    .gpio = { 4, 5, 6, 7 },                 /* STM32: .tim, .tick_hz, .channel */
};

dshot_init(&bus, &cfg);                     /* lines low, nothing sent yet */
/* every control period, value 0 for a while first so the ESCs arm */
esc_set_command(dshot_esc(&bus, 0), ESC_COMMAND_THROTTLE, t);
dshot_update(&bus, dt_s);                   /* one frame to every motor */
```

| Call | Does |
|---|---|
| `dshot_init(&bus, &cfg)` | checks the config and sets the outputs up; `-EOPNOTSUPP` for `bidirectional` |
| `dshot_esc(&bus, i)` | motor i's `esc_dev`; NULL past `count` |
| `dshot_update(&bus, dt_s)` | one frame to every motor, together; `dt_s` counts down a command's wait. `-EBUSY`, sending nothing, while the last frame is still on the wire |
| `dshot_command(esc, cmd)` | a special command: its frames with the telemetry bit, then value 0 for its wait ([model reference](chips.md#special-commands)); `-EBUSY` while one is still going, or for 0-36 while the motor runs |
| `dshot_set_value(esc, v, telemetry)` | the raw value, 0 or 48-2047 -- 3D mode's halves, say; the bit stays on until the next throttle. 1-47 are `-EINVAL`: commands go through `dshot_command()` |
| `dshot_request_telemetry(esc)` | the bit in the next frame only |
| `dshot_del(&bus)` | outputs stopped, lines low |

Throttle 0 is value 0, motor stop; any throttle above 0 is 48..2047,
linear, rounded. The library sends nothing between two updates: call
`dshot_update()` at the loop rate, no faster than a frame lasts (106.72,
53.28 or 26.72 us). ESP-IDF needs a chip with an RMT -- not the C2 -- and
for more than one motor one that starts channels together -- not the
original ESP32 ([model reference](chips.md#dshot)); the STM32 a
timer and its update DMA set up as `dshot.h` describes, the DMA's IRQ
calling `HAL_DMA_IRQHandler()` -- the library sees a frame finished only
through the HAL's DMA state.

## Errors

| errno | Means |
|---|---|
| `-EINVAL` | a bad argument: NULL, an ID out of range, a value not finite, a bad config |
| `-EOPNOTSUPP` | a command kind the model does not take, feedback from a C615 or DShot, a model built out, bidirectional DShot |
| `-EADDRINUSE` | `dji_add()`: that feedback identifier is taken on this bus |
| `-ENOENT` | a frame not for this bus; an HT-03 ID never configured |
| `-ENOSPC` | `ht03_cfg()`: every slot is taken |
| `-EBUSY` | `dshot_update()`: the last frame is still going out; `dshot_command()`: a command still running, or the motor not stopped |
| `-ENOTCONN` | `dji_update()`, `ht03_update()`: a motor is offline; its frames were still sent |
| other | `can_send()`'s, the LEDC's or the HAL's, passed through |

## No loops here

esp-c620-control closes speed and position loops with upid inside the
library. mcu-esc does not: it is the protocol and nothing else. Close loops
in the app on `esc_get_feedback()` and `esc_set_command()`, with upid or
anything else; the dt you pass to the update call is the same one the loop
wants.
