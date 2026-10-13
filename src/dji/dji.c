/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "esc_priv.h"

#if CONFIG_ESC_C610 || CONFIG_ESC_C620 || CONFIG_ESC_GM6020

#include <math.h>
#include <string.h>

#include "dji.h"
#include "dji_math.h"
#include "dji_proto.h"

/* The five command identifiers, in the order dji_update() sends them. */
static const uint16_t dji_command_ids[] = {
	DJI_GM6020_CURRENT_LOW,		/* 0x1FE */
	DJI_C6X0_COMMAND_HIGH,		/* 0x1FF, also GM6020 voltage 1-4 */
	DJI_C6X0_COMMAND_LOW,		/* 0x200 */
	DJI_GM6020_CURRENT_HIGH,	/* 0x2FE */
	DJI_GM6020_VOLTAGE_HIGH,	/* 0x2FF */
};

static int dji_set_command(struct esc_dev *esc, enum esc_command kind,
			   float value);
static int dji_get_feedback(struct esc_dev *esc,
			    struct esc_feedback *feedback);

static const struct esc_ops dji_ops = {
	.set_command = dji_set_command,
	.get_feedback = dji_get_feedback,
};

static struct dji_motor *dji_motor_of(struct esc_dev *esc)
{
	if (!esc || esc->ops != &dji_ops)
		return NULL;
	return esc->priv;
}

/*
 * Where the model puts this ID: frames, slot and command range. -EINVAL for
 * an ID outside the model's range, -EOPNOTSUPP for a model built out.
 */
static int dji_place(struct dji_motor *motor,
		     const struct dji_motor_config *config)
{
	uint8_t id = config->id;
	bool low = id <= DJI_MOTORS_PER_FRAME;

	/* ESP-IDF leaves an option that is off undefined: #if only. */
	switch (config->model) {
#if CONFIG_ESC_C610
	case DJI_MODEL_C610:
#endif
#if CONFIG_ESC_C620
	case DJI_MODEL_C620:
#endif
#if CONFIG_ESC_C610 || CONFIG_ESC_C620
		if (id < 1 || id > DJI_C6X0_ID_MAX)
			return -EINVAL;
		motor->command_id = low ? DJI_C6X0_COMMAND_LOW :
					  DJI_C6X0_COMMAND_HIGH;
		motor->feedback_id = (uint16_t)(DJI_C6X0_FEEDBACK_BASE + id);
		if (config->model == DJI_MODEL_C610) {
			motor->esc.name = "c610";
			motor->raw_max = DJI_C610_RAW_MAX;
			motor->amps_max = DJI_C610_AMPS_MAX;
		} else {
			motor->esc.name = "c620";
			motor->raw_max = DJI_C620_RAW_MAX;
			motor->amps_max = DJI_C620_AMPS_MAX;
		}
		break;
#endif
#if CONFIG_ESC_GM6020
	case DJI_MODEL_GM6020:
		if (id < 1 || id > DJI_GM6020_ID_MAX)
			return -EINVAL;
		motor->esc.name = "gm6020";
		motor->feedback_id = (uint16_t)(DJI_GM6020_FEEDBACK_BASE + id);
		if (config->gm6020_current_mode) {
			motor->command_id = low ? DJI_GM6020_CURRENT_LOW :
						  DJI_GM6020_CURRENT_HIGH;
			motor->raw_max = DJI_GM6020_CURRENT_MAX;
			motor->amps_max = DJI_GM6020_AMPS_MAX;
		} else {
			motor->command_id = low ? DJI_GM6020_VOLTAGE_LOW :
						  DJI_GM6020_VOLTAGE_HIGH;
			motor->raw_max = DJI_GM6020_VOLTAGE_MAX;
			motor->amps_max = 0.0f;
		}
		break;
#endif
	default:
		return (unsigned)config->model <= DJI_MODEL_GM6020 ?
		       -EOPNOTSUPP : -EINVAL;
	}
	motor->slot = (uint8_t)((id - 1) % DJI_MOTORS_PER_FRAME);
	return 0;
}

int dji_init(struct dji_bus *bus, struct can_dev *can)
{
	if (!bus || !can)
		return -EINVAL;
	memset(bus, 0, sizeof(*bus));
	bus->can = can;
	return 0;
}

int dji_add(struct dji_bus *bus, const struct dji_motor_config *config,
	    struct esc_dev **esc)
{
	struct dji_motor motor = { 0 };
	int ret;

	if (!bus || !bus->can || !config || !esc)
		return -EINVAL;
	if (!isfinite(config->gear_ratio) || config->gear_ratio < 0.0f ||
	    !isfinite(config->timeout_s) || config->timeout_s < 0.0f)
		return -EINVAL;
	ret = dji_place(&motor, config);
	if (ret)
		return ret;

	/*
	 * A unique feedback identifier is enough: two motors that would share
	 * a command slot -- the same model and ID, or a GM6020 at n in voltage
	 * mode and a C6x0 at n + 4 -- share a feedback identifier too.
	 */
	for (uint8_t i = 0; i < bus->count; i++)
		if (bus->motors[i].feedback_id == motor.feedback_id)
			return -EADDRINUSE;
	/*
	 * No room check: DJI_MOTORS_MAX is the number of feedback identifiers
	 * there are, so a twelfth motor has already clashed above.
	 */

	motor.model = config->model;
	motor.id = config->id;
	motor.gear_ratio = config->gear_ratio > 0.0f ? config->gear_ratio :
						       1.0f;
	motor.timeout_s = config->timeout_s > 0.0f ? config->timeout_s :
						     DJI_DEFAULT_TIMEOUT_S;
	motor.bus = bus;

	bus->motors[bus->count] = motor;
	*esc = &bus->motors[bus->count].esc;
	(*esc)->ops = &dji_ops;
	(*esc)->priv = &bus->motors[bus->count];
	(*esc)->commands = motor.amps_max > 0.0f ?
			   ESC_COMMAND_BIT(ESC_COMMAND_CURRENT) : 0;
	bus->count++;
	return 0;
}

int dji_handle_frame(struct dji_bus *bus, const struct can_frame *frame)
{
	struct dji_motor *motor = NULL;
	uint16_t angle;

	if (!bus || !frame)
		return -EINVAL;
	if (frame->ext || frame->rtr || frame->dlc != DJI_FRAME_DLC)
		return -ENOENT;
	for (uint8_t i = 0; i < bus->count; i++) {
		if (bus->motors[i].feedback_id == frame->id) {
			motor = &bus->motors[i];
			break;
		}
	}
	if (!motor)
		return -ENOENT;

	/* The top bits are not part of 0..8191; keep one turn. */
	angle = dji_read_be16(&frame->data[0]) % DJI_ANGLE_COUNTS;
	if (motor->seen)
		motor->position += dji_angle_step(motor->raw.angle, angle);
	motor->seen = true;
	motor->raw.angle = angle;
	motor->raw.rpm = (int16_t)dji_read_be16(&frame->data[2]);
	motor->raw.current = (int16_t)dji_read_be16(&frame->data[4]);
	motor->raw.temperature = frame->data[6];
	motor->since_feedback_s = 0.0f;
	motor->online = true;
	return 0;
}

int dji_update(struct dji_bus *bus, float dt_s)
{
	int ret = 0;
	bool offline = false;

	if (!bus || !bus->can || !isfinite(dt_s) || dt_s <= 0.0f)
		return -EINVAL;

	for (uint8_t i = 0; i < bus->count; i++) {
		struct dji_motor *motor = &bus->motors[i];

		/* Saturate rather than grow while a motor is unplugged. */
		if (motor->since_feedback_s <= motor->timeout_s)
			motor->since_feedback_s += dt_s;
		if (motor->since_feedback_s > motor->timeout_s)
			motor->online = false;
		offline |= !motor->online;
	}

	for (size_t f = 0; f < sizeof(dji_command_ids) / sizeof(dji_command_ids[0]);
	     f++) {
		struct can_frame frame = {
			.id = dji_command_ids[f],
			.dlc = DJI_FRAME_DLC,
		};
		bool used = false;
		int err;

		for (uint8_t i = 0; i < bus->count; i++) {
			const struct dji_motor *motor = &bus->motors[i];

			if (motor->command_id != frame.id)
				continue;
			used = true;
			dji_write_be16(&frame.data[motor->slot * 2],
				       motor->online ? motor->raw.command : 0);
		}
		/* Never spend bus time on a frame nobody listens to. */
		if (!used)
			continue;
		/*
		 * Without waiting: a full queue means the loop is behind, and
		 * a late command is worse than a dropped one.
		 */
		err = can_send(bus->can, &frame, 0);
		if (err && !ret)
			ret = err;
	}
	if (ret)
		return ret;
	return offline ? -ENOTCONN : 0;
}

static int dji_set_command(struct esc_dev *esc, enum esc_command kind,
			   float value)
{
	struct dji_motor *motor = esc->priv;

	(void)kind;	/* current: the only kind in commands */
	motor->raw.command = dji_amps_to_raw(value, motor->amps_max,
					     motor->raw_max);
	return 0;
}

static int dji_get_feedback(struct esc_dev *esc, struct esc_feedback *feedback)
{
	const struct dji_motor *motor = esc->priv;

	feedback->has = ESC_HAS_ANGLE | ESC_HAS_POSITION | ESC_HAS_VELOCITY;
	feedback->online = motor->online;
	feedback->angle_rad = dji_counts_to_rad(motor->raw.angle, 1.0f);
	feedback->position_rad = dji_counts_to_rad(motor->position,
						   motor->gear_ratio);
	feedback->velocity_rad_s = dji_rpm_to_rad_s(motor->raw.rpm,
						    motor->gear_ratio);
	if (motor->model == DJI_MODEL_C620) {
		feedback->has |= ESC_HAS_TEMPERATURE;
		feedback->temperature_c = motor->raw.temperature;
	}
	return 0;
}

int dji_set_raw(struct esc_dev *esc, int16_t command)
{
	struct dji_motor *motor = dji_motor_of(esc);

	if (!motor)
		return -EINVAL;
	motor->raw.command = dji_clamp_raw(command, motor->raw_max);
	return 0;
}

int dji_get_raw(struct esc_dev *esc, struct dji_raw *raw)
{
	struct dji_motor *motor = dji_motor_of(esc);

	if (!motor || !raw)
		return -EINVAL;
	*raw = motor->raw;
	return 0;
}

#endif /* CONFIG_ESC_C610 || CONFIG_ESC_C620 || CONFIG_ESC_GM6020 */
