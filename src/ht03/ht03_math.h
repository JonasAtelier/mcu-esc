/*
 * HT-03 frame packing, pure C: esp-HT03's pack.c, unchanged but for being
 * inline. No bus, no state, so the host tests cover it whole. Layout in
 * ht03.h.
 */
#ifndef HT03_MATH_H
#define HT03_MATH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ht03.h"

/* Clamp to [min, max], then scale onto 0 .. 2^bits - 1, truncated [BK] */
static inline uint16_t ht03_float_to_uint(float value, float min, float max,
                                          int bits)
{
    const float steps = (float)((1u << bits) - 1u);

    /* NaN never reaches the cast: it becomes 0, then clamps like any value,
     * so a NaN torque packs as about 0 Nm rather than -torque_max.
     */
    if (value != value)
        value = 0.0f;
    if (value < min)
        value = min;
    if (value > max)
        value = max;

    return (uint16_t)((value - min) * steps / (max - min));
}

/* The inverse of ht03_float_to_uint() */
static inline float ht03_uint_to_float(uint16_t value, float min, float max,
                                       int bits)
{
    const float steps = (float)((1u << bits) - 1u);

    return (float)value * (max - min) / steps + min;
}

/* Control command, layout in ht03.h */
static inline void ht03_pack_command(uint8_t data[static HT03_COMMAND_DLC],
                                     const ht03_command_t *command,
                                     const ht03_range_t *range)
{
    const uint16_t position = ht03_float_to_uint(command->position,
        -range->position_max, range->position_max, HT03_POSITION_BITS);
    const uint16_t velocity = ht03_float_to_uint(command->velocity,
        -range->velocity_max, range->velocity_max, HT03_VELOCITY_BITS);
    const uint16_t kp = ht03_float_to_uint(command->kp,
        0.0f, range->kp_max, HT03_KP_BITS);
    const uint16_t kd = ht03_float_to_uint(command->kd,
        0.0f, range->kd_max, HT03_KD_BITS);
    const uint16_t torque = ht03_float_to_uint(command->torque,
        -range->torque_max, range->torque_max, HT03_TORQUE_BITS);

    data[0] = (uint8_t)(position >> 8);
    data[1] = (uint8_t)(position & 0xFF);
    data[2] = (uint8_t)(velocity >> 4);
    data[3] = (uint8_t)(((velocity & 0xF) << 4) | (kp >> 8));
    data[4] = (uint8_t)(kp & 0xFF);
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0xF) << 4) | (torque >> 8));
    data[7] = (uint8_t)(torque & 0xFF);
}

/* FF FF FF FF FF FF FF <last> */
static inline void ht03_pack_special(uint8_t data[static HT03_COMMAND_DLC],
                                     uint8_t last)
{
    memset(data, 0xFF, HT03_COMMAND_DLC - 1);
    data[HT03_COMMAND_DLC - 1] = last;
}

/* Position, velocity and torque out of a reply. The caller has already
 * checked DLC 6 and byte 0, so this cannot fail.
 */
static inline void ht03_unpack_reply(const uint8_t data[static HT03_REPLY_DLC],
                                     const ht03_range_t *range,
                                     ht03_feedback_t *out)
{
    const uint16_t position = (uint16_t)((data[1] << 8) | data[2]);
    const uint16_t velocity = (uint16_t)((data[3] << 4) | (data[4] >> 4));
    const uint16_t torque = (uint16_t)(((data[4] & 0xF) << 8) | data[5]);

    out->position = ht03_uint_to_float(position, -range->position_max,
                                       range->position_max,
                                       HT03_POSITION_BITS);
    out->velocity = ht03_uint_to_float(velocity, -range->velocity_max,
                                       range->velocity_max,
                                       HT03_VELOCITY_BITS);
    out->torque = ht03_uint_to_float(torque, -range->torque_max,
                                     range->torque_max, HT03_TORQUE_BITS);
}

#endif /* HT03_MATH_H */
