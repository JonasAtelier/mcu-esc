/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "esc_priv.h"

#if CONFIG_ESC_HT03

/* Motor slots, and every call that touches the bus. esp-HT03's core.c in
 * its own four-space style, ported to struct can_dev and errno, without the
 * FreeRTOS lock; the byte layout lives in ht03_math.h.
 */

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "ht03.h"
#include "ht03_math.h"

/* How long a special command may wait for a free transmit slot */
#define SPECIAL_TIMEOUT_MS 10

static int ht03_set_esc_command(struct esc_dev *esc, enum esc_command kind,
                                float value);
static int ht03_get_esc_feedback(struct esc_dev *esc,
                                 struct esc_feedback *feedback);

static const struct esc_ops ht03_ops = {
    .set_command = ht03_set_esc_command,
    .get_feedback = ht03_get_esc_feedback,
};

/* The configured slot holding this ID, or NULL */
static ht03_motor_t *find_motor(const ht03_t *ht, uint8_t id)
{
    uint8_t i;

    if (!ht || !ht->can)
        return NULL;

    for (i = 0; i < HT03_MAX_MOTORS; i++) {
        if (ht->motor[i].configured && ht->motor[i].id == id)
            return (ht03_motor_t *)&ht->motor[i];
    }

    return NULL;
}

/* -EINVAL without a bound handle, else -ENOENT: no such motor */
static int missing(const ht03_t *ht)
{
    return ht && ht->can ? -ENOENT : -EINVAL;
}

static bool is_range_valid(const ht03_range_t *range)
{
    return isfinite(range->position_max) && range->position_max > 0.0f &&
           isfinite(range->velocity_max) && range->velocity_max > 0.0f &&
           isfinite(range->kp_max) && range->kp_max > 0.0f &&
           isfinite(range->kd_max) && range->kd_max > 0.0f &&
           isfinite(range->torque_max) && range->torque_max > 0.0f;
}

/* Standard frame. Control commands pass 0: a late command is worse than a
 * dropped one. Special commands wait SPECIAL_TIMEOUT_MS, so a disable is not
 * lost behind a full transmit queue.
 */
static int send(ht03_t *ht, uint8_t id, const uint8_t *data,
                uint32_t timeout_ms)
{
    struct can_frame frame = { .id = id, .dlc = HT03_COMMAND_DLC };

    memcpy(frame.data, data, HT03_COMMAND_DLC);
    return can_send(ht->can, &frame, timeout_ms);
}

int ht03_init(ht03_t *ht, struct can_dev *can)
{
    if (!ht || !can)
        return -EINVAL;

    memset(ht, 0, sizeof(*ht));
    ht->can = can;

    return 0;
}

int ht03_cfg(ht03_t *ht, uint8_t id, const ht03_range_t *range)
{
    ht03_motor_t *m;
    uint8_t i;

    if (!ht || !ht->can || !range || id < HT03_MIN_ID ||
        !is_range_valid(range))
        return -EINVAL;

    m = find_motor(ht, id);

    for (i = 0; !m && i < HT03_MAX_MOTORS; i++) {
        if (!ht->motor[i].configured)
            m = &ht->motor[i];
    }

    if (!m)
        return -ENOSPC;

    memset(m, 0, sizeof(*m));
    m->configured = true;
    m->id = id;
    m->range = *range;
    m->timeout_s = HT03_DEFAULT_TIMEOUT_S;
    m->esc.name = "ht03";
    m->esc.ops = &ht03_ops;
    m->esc.priv = m;
    m->esc.commands = ESC_COMMAND_BIT(ESC_COMMAND_TORQUE);

    return 0;
}

int ht03_set_timeout(ht03_t *ht, uint8_t id, float timeout_s)
{
    ht03_motor_t *m;

    if (!isfinite(timeout_s) || timeout_s <= 0.0f)
        return -EINVAL;

    m = find_motor(ht, id);

    if (!m)
        return missing(ht);

    m->timeout_s = timeout_s;
    return 0;
}

struct esc_dev *ht03_esc(ht03_t *ht, uint8_t id)
{
    ht03_motor_t *m = find_motor(ht, id);

    return m ? &m->esc : NULL;
}

/* Shared body of the three special commands */
static int special(ht03_t *ht, uint8_t id, uint8_t last)
{
    uint8_t data[HT03_COMMAND_DLC];
    ht03_motor_t *m;

    m = find_motor(ht, id);

    if (!m)
        return missing(ht);

    memset(&m->command, 0, sizeof(m->command));

    ht03_pack_special(data, last);
    return send(ht, id, data, SPECIAL_TIMEOUT_MS);
}

/** Enter motor mode
 *  @note   The stored command is reset to limp first, so the motor wakes up
 *          applying nothing until you ht03_command() it.
 */
int ht03_enable(ht03_t *ht, uint8_t id)
{
    return special(ht, id, HT03_ENTER_MOTOR_MODE);
}

/* Exit motor mode: the motor goes limp and ignores control commands */
int ht03_disable(ht03_t *ht, uint8_t id)
{
    return special(ht, id, HT03_EXIT_MOTOR_MODE);
}

/** The current position becomes zero
 *  @note   Exits motor mode first. The firmware zeroes in any mode, and in
 *          motor mode the stored position target would be re-read against
 *          the new zero: up to the full range away, at up to full torque.
 *          The motor is disabled afterwards; ht03_enable() it again.
 *          If the exit cannot be queued, the zero is not sent.
 */
int ht03_set_zero(ht03_t *ht, uint8_t id)
{
    const int ret = special(ht, id, HT03_EXIT_MOTOR_MODE);

    return ret ? ret : special(ht, id, HT03_SET_ZERO);
}

int ht03_command(ht03_t *ht, uint8_t id, float position, float velocity,
                 float kp, float kd, float torque)
{
    ht03_motor_t *m;

    if (!isfinite(position) || !isfinite(velocity) || !isfinite(kp) ||
        !isfinite(kd) || !isfinite(torque))
        return -EINVAL;

    m = find_motor(ht, id);

    if (!m)
        return missing(ht);

    m->command.position = position;
    m->command.velocity = velocity;
    m->command.kp = kp;
    m->command.kd = kd;
    m->command.torque = torque;

    return 0;
}

int ht03_command_torque(ht03_t *ht, uint8_t id, float torque)
{
    return ht03_command(ht, id, 0.0f, 0.0f, 0.0f, 0.0f, torque);
}

int ht03_command_position(ht03_t *ht, uint8_t id, float position, float kp,
                          float kd)
{
    return ht03_command(ht, id, position, 0.0f, kp, kd, 0.0f);
}

int ht03_command_velocity(ht03_t *ht, uint8_t id, float velocity, float kd)
{
    return ht03_command(ht, id, 0.0f, velocity, 0.0f, kd, 0.0f);
}

/** Feed one received frame to the library
 *  @note   Matches on DLC 6 and byte 0, any identifier: see ht03.h.
 */
int ht03_handle_frame(ht03_t *ht, const struct can_frame *frame)
{
    ht03_feedback_t reply;
    ht03_motor_t *m;

    if (!ht || !ht->can || !frame)
        return -EINVAL;
    /* 6, or 7 from the F446 rewrite with Vbus in byte 6, which is ignored.
     * Never more: a DJI feedback frame is DLC 8 and could alias.
     */
    if (frame->ext || frame->rtr || (frame->dlc != HT03_REPLY_DLC &&
                                     frame->dlc != HT03_REPLY_DLC_VBUS))
        return -ENOENT;

    m = find_motor(ht, frame->data[0]);

    if (!m)
        return -ENOENT;

    ht03_unpack_reply(frame->data, &m->range, &reply);
    m->position = reply.position;
    m->velocity = reply.velocity;
    m->torque = reply.torque;
    m->since_reply_s = 0.0f;
    m->is_online = true;

    return 0;
}

int ht03_update(ht03_t *ht, float dt_s)
{
    static const ht03_command_t limp = {0};
    int result = 0;
    uint8_t i;

    if (!ht || !ht->can || !isfinite(dt_s) || dt_s <= 0.0f)
        return -EINVAL;

    for (i = 0; i < HT03_MAX_MOTORS; i++) {
        ht03_motor_t *m = &ht->motor[i];
        uint8_t data[HT03_COMMAND_DLC];
        int ret;

        if (!m->configured)
            continue;

        /* Saturate rather than grow forever while a motor is unplugged */
        if (m->since_reply_s <= m->timeout_s)
            m->since_reply_s += dt_s;

        if (m->since_reply_s > m->timeout_s)
            m->is_online = false;

        /* Offline still sends: a limp frame is what asks it to reply */
        ht03_pack_command(data, m->is_online ? &m->command : &limp,
                          &m->range);

        /* A send error outranks offline, as esp-HT03's ECAN did */
        ret = send(ht, m->id, data, 0);
        if (ret)
            result = ret;
        else if (!m->is_online && !result)
            result = -ENOTCONN;
    }

    return result;
}

int ht03_get_feedback(const ht03_t *ht, uint8_t id, ht03_feedback_t *out)
{
    const ht03_motor_t *m;

    if (!out)
        return -EINVAL;

    m = find_motor(ht, id);

    if (!m)
        return missing(ht);

    out->position = m->position;
    out->velocity = m->velocity;
    out->torque = m->torque;
    out->is_online = m->is_online;

    return 0;
}

/* ESC_COMMAND_TORQUE: kp = kd = 0, so the firmware's law is t_ff alone */
static int ht03_set_esc_command(struct esc_dev *esc, enum esc_command kind,
                                float value)
{
    ht03_motor_t *m = esc->priv;

    (void)kind;    /* torque: the only kind in commands */
    memset(&m->command, 0, sizeof(m->command));
    m->command.torque = value;
    return 0;
}

static int ht03_get_esc_feedback(struct esc_dev *esc,
                                 struct esc_feedback *feedback)
{
    const ht03_motor_t *m = esc->priv;

    feedback->has = ESC_HAS_POSITION | ESC_HAS_VELOCITY | ESC_HAS_TORQUE;
    feedback->position_rad = m->position;
    feedback->velocity_rad_s = m->velocity;
    feedback->torque_nm = m->torque;
    feedback->online = m->is_online;
    return 0;
}

#endif /* CONFIG_ESC_HT03 */
