/* Slots, special commands, reply matching, timeout, and the esc_dev.
 * esp-HT03's tests/test_core.c ported to struct can_dev and errno, against
 * a fake can_send() defined here; ht03_poll() is gone, so replies go in
 * through ht03_handle_frame().
 */

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esc.h"
#include "ht03.h"
#include "ht03_math.h"

/* ------------------------------------------------------------- fake bus */

static struct can_frame sent[32];
static uint32_t sent_timeout[32];
static int sent_count;
static int send_error;     /* what a failing can_send() returns */
static int send_passes;    /* how many sends go before the failures */
static int send_failures;  /* how many sends then fail */

int can_send(struct can_dev *can, const struct can_frame *frame,
             uint32_t timeout_ms)
{
    (void)can;
    if (send_passes)
        send_passes--;
    else if (send_failures) {
        send_failures--;
        return send_error;
    }
    assert(sent_count < 32);
    sent_timeout[sent_count] = timeout_ms;
    sent[sent_count++] = *frame;
    return 0;
}

/* ---------------------------------------------------------------- helpers */

static const ht03_range_t j6 = HT03_J6;
static struct can_dev can;

static struct can_frame reply(uint32_t frame_id, uint8_t motor_id)
{
    /* position 0xC000, velocity 0x800, torque 0x800 */
    struct can_frame frame = {
        .id = frame_id, .dlc = HT03_REPLY_DLC,
        .data = {motor_id, 0xC0, 0x00, 0x80, 0x08, 0x00},
    };
    return frame;
}

static bool is_special(const struct can_frame *frame, uint32_t id,
                       uint8_t last)
{
    static const uint8_t ff[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    return frame->id == id && frame->dlc == 8 && !frame->ext &&
           !frame->rtr && memcmp(frame->data, ff, 7) == 0 &&
           frame->data[7] == last;
}

static bool is_limp(const struct can_frame *frame)
{
    const ht03_command_t limp = {0};
    uint8_t expected[8];

    ht03_pack_command(expected, &limp, &j6);
    return frame->dlc == 8 && memcmp(frame->data, expected, 8) == 0;
}

static void feed(ht03_t *ht, uint8_t motor_id)
{
    struct can_frame frame = reply(0x000, motor_id);

    assert(ht03_handle_frame(ht, &frame) == 0);
}

/* ------------------------------------------------------------------ tests */

static void test_slots(void)
{
    ht03_t ht;
    const ht03_range_t bad = {95.5f, 45.0f, 500.0f, 0.0f, 18.0f};
    const ht03_range_t nan_range = {NAN, 45.0f, 500.0f, 5.0f, 18.0f};
    uint8_t id;

    assert(ht03_init(NULL, &can) == -EINVAL);
    assert(ht03_init(&ht, NULL) == -EINVAL);
    assert(ht03_init(&ht, &can) == 0);

    assert(ht03_cfg(&ht, 0, &j6) == -EINVAL);
    assert(ht03_cfg(&ht, 1, NULL) == -EINVAL);
    assert(ht03_cfg(&ht, 1, &bad) == -EINVAL);
    assert(ht03_cfg(&ht, 1, &nan_range) == -EINVAL);
    assert(ht03_esc(&ht, 1) == NULL);

    for (id = 1; id <= HT03_MAX_MOTORS; id++)
        assert(ht03_cfg(&ht, id, &j6) == 0);

    assert(ht03_cfg(&ht, 3, &j6) == 0);   /* again: same slot */
    assert(ht03_cfg(&ht, 200, &j6) == -ENOSPC);
    assert(ht03_enable(&ht, 200) == -ENOENT);
    assert(ht03_set_timeout(&ht, 200, 1.0f) == -ENOENT);
    assert(ht03_set_timeout(&ht, 3, 0.0f) == -EINVAL);
    assert(ht03_set_timeout(&ht, 3, INFINITY) == -EINVAL);
    assert(ht03_set_timeout(&ht, 3, 0.5f) == 0);
}

static void test_special_commands(void)
{
    ht03_t ht;

    ht03_init(&ht, &can);
    ht03_cfg(&ht, 7, &j6);
    sent_count = 0;

    assert(ht03_enable(&ht, 7) == 0);
    assert(ht03_disable(&ht, 7) == 0);
    assert(sent_count == 2);
    assert(is_special(&sent[0], 7, 0xFC));
    assert(is_special(&sent[1], 7, 0xFD));
    /* Specials wait for room; a lost disable would leave the motor on */
    assert(sent_timeout[0] == 10 && sent_timeout[1] == 10);

    /* Set zero from motor mode with a live position command: exit motor
     * mode first, then zero, and the stored command is gone.
     */
    ht03_enable(&ht, 7);
    assert(ht03_command_position(&ht, 7, 50.0f, 100.0f, 1.0f) == 0);
    feed(&ht, 7);

    sent_count = 0;
    assert(ht03_set_zero(&ht, 7) == 0);
    assert(ht03_update(&ht, 0.001f) == 0);
    assert(sent_count == 3);
    assert(is_special(&sent[0], 7, 0xFD));
    assert(is_special(&sent[1], 7, 0xFE));
    assert(sent[2].id == 7 && is_limp(&sent[2]));
    assert(sent_timeout[2] == 0);

    /* The exit cannot be queued: the zero is not sent */
    sent_count = 0;
    send_error = -ETIMEDOUT;
    send_failures = 1;
    assert(ht03_set_zero(&ht, 7) == -ETIMEDOUT);
    assert(sent_count == 0);

    /* The exit goes, the zero does not: the zero's error comes back */
    send_passes = 1;
    send_failures = 1;
    assert(ht03_set_zero(&ht, 7) == -ETIMEDOUT);
    assert(sent_count == 1 && is_special(&sent[0], 7, 0xFD));
}

/* Replies match on DLC 6 or 7 and byte 0, from any identifier, and nothing
 * else */
static void test_reply_matching(void)
{
    ht03_t ht;
    ht03_feedback_t feedback;
    struct can_frame frame;

    ht03_init(&ht, &can);
    ht03_cfg(&ht, 1, &j6);

    assert(ht03_handle_frame(&ht, NULL) == -EINVAL);

    /* A C620 feedback frame: 0x201, DLC 8, byte 0 happens to be 1 */
    frame = reply(0x201, 1);
    frame.dlc = 8;
    assert(ht03_handle_frame(&ht, &frame) == -ENOENT);

    /* Shorter than 6 is not a reply */
    frame = reply(0x000, 1);
    frame.dlc = 5;
    assert(ht03_handle_frame(&ht, &frame) == -ENOENT);

    frame = reply(0x000, 1);
    frame.ext = true;
    assert(ht03_handle_frame(&ht, &frame) == -ENOENT);

    frame = reply(0x000, 1);
    frame.rtr = true;
    assert(ht03_handle_frame(&ht, &frame) == -ENOENT);

    frame = reply(0x000, 2);
    assert(ht03_handle_frame(&ht, &frame) == -ENOENT);

    assert(ht03_get_feedback(&ht, 1, &feedback) == 0);
    assert(!feedback.is_online);

    /* Stock master ID 0, and any other identifier */
    frame = reply(0x000, 1);
    assert(ht03_handle_frame(&ht, &frame) == 0);
    frame = reply(0x7FF, 1);
    assert(ht03_handle_frame(&ht, &frame) == 0);

    /* The F446 rewrite's 7-byte reply, Vbus in byte 6: heard, Vbus ignored */
    frame = reply(0x000, 1);
    frame.dlc = 7;
    frame.data[6] = 0xC8;
    assert(ht03_handle_frame(&ht, &frame) == 0);

    assert(ht03_get_feedback(&ht, 1, &feedback) == 0);
    assert(feedback.is_online);
    assert(fabsf(feedback.position -
                 (0xC000 * 191.0f / 65535.0f - 95.5f)) < 1e-4f);
    assert(fabsf(feedback.velocity - (0x800 * 90.0f / 4095.0f - 45.0f)) <
           1e-4f);
    assert(fabsf(feedback.torque - (0x800 * 36.0f / 4095.0f - 18.0f)) <
           1e-4f);
    assert(ht03_get_feedback(&ht, 2, &feedback) == -ENOENT);
    assert(ht03_get_feedback(&ht, 1, NULL) == -EINVAL);
}

/* Offline until the first reply, then the stored command, then limp again
 * once replies stop.
 */
static void test_update_and_timeout(void)
{
    ht03_t ht;
    ht03_feedback_t feedback;
    const ht03_command_t command = {1.0f, 2.0f, 30.0f, 0.5f, -1.0f};
    uint8_t expected[8];

    ht03_init(&ht, &can);
    ht03_cfg(&ht, 4, &j6);

    assert(ht03_update(&ht, 0.0f) == -EINVAL);
    assert(ht03_update(&ht, NAN) == -EINVAL);
    assert(ht03_command(&ht, 4, INFINITY, 0, 0, 0, 0) == -EINVAL);
    assert(ht03_command(&ht, 4, 1.0f, 2.0f, 30.0f, 0.5f, -1.0f) == 0);

    /* Never replied: a limp frame still goes out, to ask for one */
    sent_count = 0;
    assert(ht03_update(&ht, 0.001f) == -ENOTCONN);
    assert(sent_count == 1 && sent[0].id == 4 && is_limp(&sent[0]));

    feed(&ht, 4);

    sent_count = 0;
    assert(ht03_update(&ht, 0.001f) == 0);
    ht03_pack_command(expected, &command, &j6);
    assert(sent_count == 1 && sent[0].id == 4 && !sent[0].ext &&
           memcmp(sent[0].data, expected, 8) == 0);

    /* Just under the 100 ms default, still online */
    assert(ht03_update(&ht, 0.098f) == 0);

    /* Past it: offline, and the command is replaced by limp */
    sent_count = 0;
    assert(ht03_update(&ht, 0.002f) == -ENOTCONN);
    assert(ht03_get_feedback(&ht, 4, &feedback) == 0);
    assert(!feedback.is_online);
    assert(sent_count == 1 && is_limp(&sent[0]));

    /* A send error outranks offline */
    send_error = -EIO;
    send_failures = 1;
    assert(ht03_update(&ht, 0.001f) == -EIO);

    /* Disable resets the stored command, so a reply brings back limp */
    assert(ht03_disable(&ht, 4) == 0);
    feed(&ht, 4);
    sent_count = 0;
    assert(ht03_update(&ht, 0.001f) == 0);
    assert(sent_count == 1 && is_limp(&sent[0]));

    /* The helpers zero what they leave out */
    assert(ht03_command_position(&ht, 4, 1.0f, 30.0f, 0.5f) == 0);
    sent_count = 0;
    ht03_update(&ht, 0.001f);
    {
        const ht03_command_t position = {1.0f, 0.0f, 30.0f, 0.5f, 0.0f};

        ht03_pack_command(expected, &position, &j6);
        assert(memcmp(sent[0].data, expected, 8) == 0);
    }
    assert(ht03_command_velocity(&ht, 4, 3.0f, 0.2f) == 0);
    sent_count = 0;
    ht03_update(&ht, 0.001f);
    {
        const ht03_command_t velocity = {0.0f, 3.0f, 0.0f, 0.2f, 0.0f};

        ht03_pack_command(expected, &velocity, &j6);
        assert(memcmp(sent[0].data, expected, 8) == 0);
    }
}

/* The common API: torque only, as kp = kd = 0 plus feed-forward */
static void test_esc(void)
{
    ht03_t ht;
    struct esc_dev *esc;
    struct esc_feedback feedback;
    uint8_t expected[8];

    ht03_init(&ht, &can);
    ht03_cfg(&ht, 5, &j6);
    esc = ht03_esc(&ht, 5);
    assert(esc && strcmp(esc->name, "ht03") == 0);

    assert(esc_set_command(esc, ESC_COMMAND_CURRENT, 1.0f) == -EOPNOTSUPP);
    assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.5f) == -EOPNOTSUPP);
    assert(esc_set_command(esc, ESC_COMMAND_TORQUE, NAN) == -EINVAL);

    /* A live MIT command is replaced whole, not just its torque */
    assert(ht03_command(&ht, 5, 1.0f, 2.0f, 30.0f, 0.5f, 4.0f) == 0);
    assert(esc_set_command(esc, ESC_COMMAND_TORQUE, -2.5f) == 0);
    feed(&ht, 5);
    sent_count = 0;
    assert(ht03_update(&ht, 0.001f) == 0);
    {
        const ht03_command_t torque = {0.0f, 0.0f, 0.0f, 0.0f, -2.5f};

        ht03_pack_command(expected, &torque, &j6);
        assert(memcmp(sent[0].data, expected, 8) == 0);
    }

    assert(esc_get_feedback(esc, &feedback) == 0);
    assert(feedback.has ==
           (ESC_HAS_POSITION | ESC_HAS_VELOCITY | ESC_HAS_TORQUE));
    assert(feedback.online);
    assert(fabsf(feedback.position_rad -
                 (0xC000 * 191.0f / 65535.0f - 95.5f)) < 1e-4f);
    assert(fabsf(feedback.velocity_rad_s -
                 (0x800 * 90.0f / 4095.0f - 45.0f)) < 1e-4f);
    assert(fabsf(feedback.torque_nm - (0x800 * 36.0f / 4095.0f - 18.0f)) <
           1e-4f);
    assert(feedback.angle_rad == 0.0f && feedback.temperature_c == 0.0f);
}

int main(void)
{
    test_slots();
    test_special_commands();
    test_reply_matching();
    test_update_and_timeout();
    test_esc();

    puts("test_ht03: all passed");
    return 0;
}
