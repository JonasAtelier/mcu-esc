/*
 * Haitai (海泰機電) HT-03 actuator, HT8108-J6 with a 1:6 reduction, on CAN
 * through mcu-can's struct can_dev. Ported from esp-HT03: the same calls,
 * with negative errno for ht03_sta and struct can_dev for hvd230_t.
 *
 *  1. The HT-03 runs the MIT Mini Cheetah motor firmware (Ben Katz,
 *     "motorcontrol"). Its own controller closes the loop:
 *
 *         torque = kp * (position_target - position)
 *                + kd * (velocity_target - velocity)
 *                + torque_feedforward
 *
 *     This library only packs those five numbers and unpacks the reply.
 *     ht03_command() is that command whole, the model's own extension.
 *     ESC_COMMAND_TORQUE through esc_set_command() sends kp = kd = 0 and
 *     torque_feedforward, which by the line above is a torque command and
 *     nothing else.
 *  2. The motor answers ONLY when it receives a frame. There is no periodic
 *     broadcast, so ht03_update() sends the stored command to every motor
 *     on every call, and that is also what keeps feedback coming.
 *  3. No clock is read here. You pass dt to ht03_update(), in seconds.
 *  4. A motor that stops replying for its timeout is declared offline and
 *     sent a limp command instead of yours: kp = kd = 0, and torque raw
 *     0x7FF, which decodes to about -0.004 Nm (0 Nm is not on the grid).
 *  5. Ranges are per motor (ht03_range_t). HT03_J6 below is the only set
 *     this library knows. The J9 (HT-03-9) and J36 (HT-03-D) variants need
 *     their own numbers, which are not known here.
 *  6. ht03_init() before anything else, and ht03_disable() every motor
 *     before you stop calling ht03_update().
 *  7. Not thread-safe: call everything for one ht03_t from one task, or
 *     hold your own lock around it. (esp-HT03 took a FreeRTOS mutex; the
 *     STM32 HAL has none to take.)
 *
 *  Sources
 *  -------
 *  [BK]   Ben Katz, motorcontrol, the firmware the HT-03 runs. What the
 *         HT-03 ships is the older mbed version; the layout below was
 *         re-checked against the STM32F446 rewrite, master 93bbc606:
 *         Core/Src/can.c:157-171 pack_reply(), :189-204 unpack_cmd();
 *         Core/Src/stm32f4xx_it.c:298-317 the special commands;
 *         Core/Src/foc.c:289-291 torque_control(), the line in note 1;
 *         Core/Src/math_ops.c:44-57 float_to_uint() / uint_to_float().
 *         docs/chips.md lists where the rewrite differs.
 *  [SCUT] SCUT-RobotIC/RC2024-THE-DOG, CANFORF407ZG/HT03/MDK-ARM/HT03.h and
 *         HT03.c: the HT-03 J6 ranges and the same frame layout.
 *
 *  HT-03 CAN Data Frame Structure. CAN 2.0A standard identifier, 1 Mbps,
 *  bit fields big-endian (most significant bits in the lower byte) [BK].
 *
 *  TX special command: id = motor ID, DLC 8 [BK stm32f4xx_it.c]
 *  FF FF FF FF FF FF FF FC   enter motor mode
 *  FF FF FF FF FF FF FF FD   exit motor mode (rest, the motor goes limp)
 *  FF FF FF FF FF FF FF FE   current position becomes zero. Send it while
 *                            the motor is in rest mode.
 *
 *  TX control command: id = motor ID, DLC 8 [BK unpack_cmd]
 *  Byte 0: Position [15:8]                    16 bit, -P_MAX to +P_MAX rad
 *  Byte 1: Position [7:0]
 *  Byte 2: Velocity [11:4]                    12 bit, -V_MAX to +V_MAX rad/s
 *  Byte 3: Velocity [3:0] << 4 | Kp [11:8]    Kp 12 bit, 0 to KP_MAX Nm/rad
 *  Byte 4: Kp [7:0]
 *  Byte 5: Kd [11:4]                          12 bit, 0 to KD_MAX Nm*s/rad
 *  Byte 6: Kd [3:0] << 4 | Torque [11:8]      Torque 12 bit, -T_MAX to +T_MAX Nm
 *  Byte 7: Torque [7:0]
 *  In rest mode the motor ignores this frame, but still replies to it.
 *
 *  RX reply: id = master ID (0 in the stock firmware, but configurable),
 *            DLC 6, one per frame the motor receives; DLC 7 from the F446
 *            rewrite, byte 6 the bus voltage, ignored here [BK pack_reply]
 *  Byte 0: Motor ID
 *  Byte 1: Position [15:8]                    16 bit, -P_MAX to +P_MAX rad
 *  Byte 2: Position [7:0]
 *  Byte 3: Velocity [11:4]                    12 bit, -V_MAX to +V_MAX rad/s
 *  Byte 4: Velocity [3:0] << 4 | Torque [11:8]
 *  Byte 5: Torque [7:0]                       12 bit, -T_MAX to +T_MAX Nm
 *
 *  Replies are matched on DLC 6 or 7 and byte 0, NOT on the frame
 *  identifier, because the master ID is a firmware setting. A C620
 *  feedback frame (0x201 to 0x208) is DLC 8 and never matches.
 *
 *  Reply torque is i_q * KT * GR scaled over +-T_MAX [BK stm32f4xx_it.c:301],
 *  i.e. newton metres, while some HT-03 documents call the same field
 *  current, +-40 A. This library decodes it over +-torque_max as Nm.
 *
 *  Encoding: x is clamped to [min, max], then
 *      raw = (int)((x - min) * ((1 << bits) - 1) / (max - min))
 *  truncated, as [BK] float_to_uint() does. Decoding is the inverse.
 *
 *  Feedback through esc_get_feedback(): position, velocity and torque, all
 *  of the output shaft.
 */
#ifndef HT03_H
#define HT03_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"
#include "esc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Library limits. One slot per motor on this bus. */
#define HT03_MAX_MOTORS         8
#define HT03_MIN_ID             1       /* to 255: the reply has 1 ID byte */

/* Reply older than this declares the motor offline */
#define HT03_DEFAULT_TIMEOUT_S  0.1f

/* Protocol constants [BK] */
#define HT03_BITRATE            1000000u
#define HT03_COMMAND_DLC        8
#define HT03_REPLY_DLC          6
#define HT03_REPLY_DLC_VBUS     7       /* the F446 rewrite adds Vbus */

#define HT03_POSITION_BITS      16
#define HT03_VELOCITY_BITS      12
#define HT03_KP_BITS            12
#define HT03_KD_BITS            12
#define HT03_TORQUE_BITS        12

/* Last byte of a special command; bytes 0 to 6 are 0xFF [BK] */
#define HT03_ENTER_MOTOR_MODE   0xFCu
#define HT03_EXIT_MOTOR_MODE    0xFDu
#define HT03_SET_ZERO           0xFEu

/** Encoding range of one motor. Position, velocity and torque run from
 *  -max to +max; kp and kd from 0 to max. Must match the firmware's own
 *  P_MAX, V_MAX, KP_MAX, KD_MAX and T_MAX, or every value is scaled wrong.
 */
typedef struct {
    float position_max;   /* rad, output shaft */
    float velocity_max;   /* rad/s */
    float kp_max;         /* Nm/rad */
    float kd_max;         /* Nm*s/rad */
    float torque_max;     /* Nm */
} ht03_range_t;

/** HT-03, HT8108-J6, 1:6 [SCUT HT03.h]. An initializer:
 *
 *      static const ht03_range_t j6 = HT03_J6;
 */
#define HT03_J6 {                \
    .position_max = 95.5f,       \
    .velocity_max = 45.0f,       \
    .kp_max       = 500.0f,      \
    .kd_max       = 5.0f,        \
    .torque_max   = 18.0f,       \
}

/* What a motor is told to do. Clamped to its range when packed. */
typedef struct {
    float position;   /* rad */
    float velocity;   /* rad/s */
    float kp;         /* Nm/rad */
    float kd;         /* Nm*s/rad */
    float torque;     /* Nm, feed-forward */
} ht03_command_t;

/* Decoded status of one motor */
typedef struct {
    float position;   /* rad, from the last zero */
    float velocity;   /* rad/s */
    float torque;     /* Nm, see the ambiguity note above */
    bool  is_online;  /* replied within its timeout */
} ht03_feedback_t;

/* Private. Declared here only so the caller can place an ht03_t; never read
 * or write a field directly.
 */
typedef struct {
    struct esc_dev  esc;
    bool            configured;
    uint8_t         id;
    ht03_range_t    range;
    ht03_command_t  command;
    float           timeout_s;
    float           since_reply_s;
    bool            is_online;
    float           position;
    float           velocity;
    float           torque;
} ht03_motor_t;

typedef struct {
    struct can_dev *can;
    ht03_motor_t motor[HT03_MAX_MOTORS];
} ht03_t;

/* Every call returns 0 or a negative errno: -EINVAL for a bad handle, ID,
 * range or a value that is not finite, -ENOENT for a motor ID never
 * configured, -ENOSPC when every slot is taken, -ENOTCONN when a motor
 * has not replied within its timeout, or can_send()'s error.
 */

/** Bind an already-started CAN bus and empty every motor slot.
 *  can must outlive the ht03_t.
 */
int ht03_init(ht03_t *ht, struct can_dev *can);

/** Take a motor ID into a slot, with its encoding range. Configuring an ID
 *  again resets its slot. The command starts limp and the motor offline.
 */
int ht03_cfg(ht03_t *ht, uint8_t id, const ht03_range_t *range);
/** Reply-loss threshold, default HT03_DEFAULT_TIMEOUT_S. Your control period
 *  must be shorter than timeout_s, or the motor reads offline between two
 *  calls of ht03_update().
 */
int ht03_set_timeout(ht03_t *ht, uint8_t id, float timeout_s);

/* The motor's esc_dev, or NULL for an ID never configured. */
struct esc_dev *ht03_esc(ht03_t *ht, uint8_t id);

/** Special commands, sent at once, each waiting up to 10 ms for a free
 *  transmit slot. All three reset the stored command to limp, so nothing old
 *  is applied on the next ht03_update().
 *
 *  ht03_set_zero() sends exit motor mode and then set zero: the firmware
 *  zeroes in any mode, and in motor mode the stored position target would be
 *  re-read against the new zero and the motor would jump. The motor is
 *  DISABLED after the call; ht03_enable() it again.
 */
int ht03_enable(ht03_t *ht, uint8_t id);
int ht03_disable(ht03_t *ht, uint8_t id);
int ht03_set_zero(ht03_t *ht, uint8_t id);

/** Store a command; ht03_update() sends it. Values outside the motor's
 *  range are clamped when packed; non-finite ones are -EINVAL.
 */
int ht03_command(ht03_t *ht, uint8_t id, float position, float velocity,
                 float kp, float kd, float torque);

/* The same, with the unused fields zero */
int ht03_command_torque(ht03_t *ht, uint8_t id, float torque);
int ht03_command_position(ht03_t *ht, uint8_t id, float position, float kp,
                          float kd);
int ht03_command_velocity(ht03_t *ht, uint8_t id, float velocity, float kd);

/** Feed one received frame. 0 when it was a configured motor's reply,
 *  -ENOENT when it was not for this ht03_t -- pass it on.
 */
int ht03_handle_frame(ht03_t *ht, const struct can_frame *frame);

/** Send every motor its command, and age its feedback
 *  @param  dt_s  seconds since the previous call, finite and > 0
 *  @retval 0          every configured motor was sent its command
 *          -ENOTCONN  at least one motor has not replied within its
 *                     timeout; it was sent a limp command instead
 *          other      can_send()'s error: a frame could not be queued
 *  @note   The motor only replies to frames it receives, so this call is
 *          both the command path and the feedback request. Call it every
 *          control period even when nothing changed.
 */
int ht03_update(ht03_t *ht, float dt_s);

/* Checking information */
int ht03_get_feedback(const ht03_t *ht, uint8_t id, ht03_feedback_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HT03_H */
