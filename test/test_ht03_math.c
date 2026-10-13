/* Byte layout, scaling, clamping. ht03_math.h only, no bus. esp-HT03's
 * tests/test_pack.c, unchanged but for the header it includes.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ht03_math.h"

static const ht03_range_t j6 = HT03_J6;

static void bytes_are(const uint8_t *actual, const uint8_t *expected, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (actual[i] != expected[i]) {
            fprintf(stderr, "byte %d: got %02X, want %02X\n", i, actual[i],
                    expected[i]);
            assert(0);
        }
    }
}

static void pack(uint8_t data[8], float position, float velocity, float kp,
                 float kd, float torque)
{
    const ht03_command_t command = {position, velocity, kp, kd, torque};

    ht03_pack_command(data, &command, &j6);
}

/* The bit layout, with every field at an end of its range so no scaling
 * question can hide a shifted nibble.
 */
static void test_layout(void)
{
    static const uint8_t all_max[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                       0xFF, 0xFF, 0xFF, 0xFF};
    static const uint8_t all_min[8] = {0};
    static const uint8_t mixed_a[8] = {0xFF, 0xFF, 0x00, 0x0F,
                                       0xFF, 0x00, 0x0F, 0xFF};
    static const uint8_t mixed_b[8] = {0x00, 0x00, 0xFF, 0xF0,
                                       0x00, 0xFF, 0xF0, 0x00};
    uint8_t data[8];

    pack(data, 95.5f, 45.0f, 500.0f, 5.0f, 18.0f);
    bytes_are(data, all_max, 8);

    pack(data, -95.5f, -45.0f, 0.0f, 0.0f, -18.0f);
    bytes_are(data, all_min, 8);

    /* position max, velocity min, kp max, kd min, torque max */
    pack(data, 95.5f, -45.0f, 500.0f, 0.0f, 18.0f);
    bytes_are(data, mixed_a, 8);

    /* and the opposite */
    pack(data, -95.5f, 45.0f, 0.0f, 5.0f, -18.0f);
    bytes_are(data, mixed_b, 8);
}

/* Mid-range values against the formula in the brief, worked by hand:
 *   kp 250 of 0..500     -> 250 * 4095 / 500     = 2047.5 -> 0x7FF
 *   kd 1 of 0..5         -> 1 * 4095 / 5         = 819    -> 0x333
 *   position 0 of +-95.5 -> 95.5 * 65535 / 191   = 32767.5 -> 0x7FFF
 *   velocity 0 of +-45   -> 45 * 4095 / 90       = 2047.5 -> 0x7FF
 *   torque 9 of +-18     -> 27 * 4095 / 36       = 3071.25 -> 0xBFF
 */
static void test_scaling(void)
{
    static const uint8_t expected[8] = {0x7F, 0xFF, 0x7F, 0xF7,
                                        0xFF, 0x33, 0x3B, 0xFF};
    uint8_t data[8];

    pack(data, 0.0f, 0.0f, 250.0f, 1.0f, 9.0f);
    bytes_are(data, expected, 8);
}

/* At the limit, past it, and not a number */
static void test_clamping(void)
{
    uint8_t at_limit[8];
    uint8_t over[8];
    uint8_t data[8];

    pack(at_limit, 95.5f, 45.0f, 500.0f, 5.0f, 18.0f);
    pack(over, 1000.0f, 1e9f, 1e6f, 50.0f, INFINITY);
    bytes_are(over, at_limit, 8);

    pack(at_limit, -95.5f, -45.0f, 0.0f, 0.0f, -18.0f);
    pack(over, -1000.0f, -1e9f, -3.0f, -1.0f, -INFINITY);
    bytes_are(over, at_limit, 8);

    /* NaN must not reach the integer cast; it is treated as 0 */
    pack(at_limit, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pack(data, NAN, NAN, NAN, NAN, NAN);
    bytes_are(data, at_limit, 8);
}

/* The limp frame ht03_update() sends offline must apply no gain and next
 * to no torque.
 */
static void test_limp(void)
{
    uint8_t data[8];

    pack(data, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    assert((data[3] & 0xF) == 0 && data[4] == 0);          /* kp 0 */
    assert(data[5] == 0 && (data[6] >> 4) == 0);           /* kd 0 */
    assert((((data[6] & 0xF) << 8) | data[7]) == 0x7FF);   /* -0.0044 Nm */
    assert(fabsf(ht03_uint_to_float(0x7FF, -18.0f, 18.0f, 12)) < 0.005f);
}

/* The top of the range must be exactly 2^bits - 1, never 2^bits */
static void test_no_overflow(void)
{
    static const float maxes[] = {95.5f, 45.0f, 500.0f, 5.0f, 18.0f,
                                  12.5f, 0.1f, 1000.0f, 3.3f, 65.0f};
    size_t i;

    for (i = 0; i < sizeof(maxes) / sizeof(maxes[0]); i++) {
        assert(ht03_float_to_uint(maxes[i], -maxes[i], maxes[i], 16) ==
               0xFFFF);
        assert(ht03_float_to_uint(maxes[i], -maxes[i], maxes[i], 12) ==
               0xFFF);
        assert(ht03_float_to_uint(maxes[i], 0.0f, maxes[i], 12) == 0xFFF);
        assert(ht03_float_to_uint(nextafterf(maxes[i], INFINITY), 0.0f,
                                  maxes[i], 12) == 0xFFF);
    }
}

/* Encode then decode lands within one step of the input */
static void round_trip(float min, float max, int bits)
{
    const float step = (max - min) / (float)((1u << bits) - 1u);
    int i;

    for (i = 0; i <= 1000; i++) {
        const float value = min + (max - min) * (float)i / 1000.0f;
        const float back = ht03_uint_to_float(
            ht03_float_to_uint(value, min, max, bits), min, max, bits);

        assert(fabsf(back - value) <= step * 1.001f);
    }
}

static void test_round_trip(void)
{
    round_trip(-95.5f, 95.5f, HT03_POSITION_BITS);
    round_trip(-45.0f, 45.0f, HT03_VELOCITY_BITS);
    round_trip(0.0f, 500.0f, HT03_KP_BITS);
    round_trip(0.0f, 5.0f, HT03_KD_BITS);
    round_trip(-18.0f, 18.0f, HT03_TORQUE_BITS);
}

static void test_special(void)
{
    static const uint8_t enter[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                     0xFF, 0xFF, 0xFF, 0xFC};
    static const uint8_t leave[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                     0xFF, 0xFF, 0xFF, 0xFD};
    static const uint8_t zero[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFE};
    uint8_t data[8];

    ht03_pack_special(data, HT03_ENTER_MOTOR_MODE);
    bytes_are(data, enter, 8);
    ht03_pack_special(data, HT03_EXIT_MOTOR_MODE);
    bytes_are(data, leave, 8);
    ht03_pack_special(data, HT03_SET_ZERO);
    bytes_are(data, zero, 8);
}

/* Reply: b0 id, b1..b2 position, b3 velocity[11:4],
 * b4 velocity[3:0] << 4 | torque[11:8], b5 torque[7:0]
 */
static void test_reply(void)
{
    const uint8_t top[6] = {1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t bottom[6] = {1, 0x00, 0x00, 0x00, 0x00, 0x00};
    /* position 0x8000, velocity 0x123, torque 0xABC */
    const uint8_t mixed[6] = {1, 0x80, 0x00, 0x12, 0x3A, 0xBC};
    ht03_feedback_t out;

    ht03_unpack_reply(top, &j6, &out);
    assert(fabsf(out.position - 95.5f) < 1e-4f);
    assert(fabsf(out.velocity - 45.0f) < 1e-4f);
    assert(fabsf(out.torque - 18.0f) < 1e-4f);

    ht03_unpack_reply(bottom, &j6, &out);
    assert(fabsf(out.position + 95.5f) < 1e-4f);
    assert(fabsf(out.velocity + 45.0f) < 1e-4f);
    assert(fabsf(out.torque + 18.0f) < 1e-4f);

    ht03_unpack_reply(mixed, &j6, &out);
    assert(fabsf(out.position -
                 (0x8000 * 191.0f / 65535.0f - 95.5f)) < 1e-4f);
    assert(fabsf(out.velocity - (0x123 * 90.0f / 4095.0f - 45.0f)) < 1e-4f);
    assert(fabsf(out.torque - (0xABC * 36.0f / 4095.0f - 18.0f)) < 1e-4f);
}

int main(void)
{
    test_layout();
    test_scaling();
    test_clamping();
    test_limp();
    test_no_overflow();
    test_round_trip();
    test_special();
    test_reply();

    puts("test_ht03_math: all passed");
    return 0;
}
