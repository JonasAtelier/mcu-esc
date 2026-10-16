/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "esc_priv.h"

#if CONFIG_ESC_DSHOT

#if defined(ESP_PLATFORM) && !defined(ESC_HAVE_DSHOT)
#error "ESC_DSHOT needs the RMT peripheral, which this chip does not have (SOC_RMT_SUPPORTED): turn ESC_DSHOT off for it"
#endif

#include <math.h>
#include <string.h>

#include "dshot.h"
#include "dshot_math.h"
#include "dshot_proto.h"

static int dshot_set_esc_command(struct esc_dev *esc, enum esc_command kind,
				 float value);
static int dshot_get_esc_feedback(struct esc_dev *esc,
				  struct esc_feedback *feedback);

static const struct esc_ops dshot_ops = {
	.set_command = dshot_set_esc_command,
	.get_feedback = dshot_get_esc_feedback,
};

static struct dshot_motor *dshot_motor_of(struct esc_dev *esc)
{
	if (!esc || esc->ops != &dshot_ops)
		return NULL;
	return esc->priv;
}

/*
 * How many frames a command needs and how long to hold 0 after it, by
 * dshot_proto.h's table. false for a number without a meaning.
 */
static bool dshot_command_rule(uint8_t command, uint8_t *repeats,
			       float *hold_s)
{
	*repeats = 1;
	*hold_s = 0.0f;
	switch (command) {
	case DSHOT_CMD_BEEP1:
	case DSHOT_CMD_BEEP2:
		*hold_s = DSHOT_BEEP12_WAIT_S;
		return true;
	case DSHOT_CMD_BEEP3:
	case DSHOT_CMD_BEEP4:
		*hold_s = DSHOT_BEEP34_WAIT_S;
		return true;
	case DSHOT_CMD_BEEP5:
		*hold_s = DSHOT_BEEP5_WAIT_S;
		return true;
	case DSHOT_CMD_ESC_INFO:
		*hold_s = DSHOT_ESC_INFO_WAIT_S;
		return true;
	case DSHOT_CMD_SAVE_SETTINGS:
	case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
	case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
		*repeats = DSHOT_COMMAND_REPEATS;
		*hold_s = DSHOT_SAVE_WAIT_S;
		return true;
	case DSHOT_CMD_SPIN_DIRECTION_1:
	case DSHOT_CMD_SPIN_DIRECTION_2:
	case DSHOT_CMD_3D_MODE_OFF:
	case DSHOT_CMD_3D_MODE_ON:
	case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
	case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
	case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE:
	case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE:
	case DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM:
	case DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD:
		*repeats = DSHOT_COMMAND_REPEATS;
		return true;
	case DSHOT_CMD_SETTINGS_REQUEST:
	case DSHOT_CMD_LED0_ON:
	case DSHOT_CMD_LED1_ON:
	case DSHOT_CMD_LED2_ON:
	case DSHOT_CMD_LED3_ON:
	case DSHOT_CMD_LED0_OFF:
	case DSHOT_CMD_LED1_OFF:
	case DSHOT_CMD_LED2_OFF:
	case DSHOT_CMD_LED3_OFF:
	case DSHOT_CMD_AUDIO_STREAM:
	case DSHOT_CMD_SILENT_MODE:
	case DSHOT_CMD_SIGNAL_LINE_TEMPERATURE:
	case DSHOT_CMD_SIGNAL_LINE_VOLTAGE:
	case DSHOT_CMD_SIGNAL_LINE_CURRENT:
	case DSHOT_CMD_SIGNAL_LINE_CONSUMPTION:
	case DSHOT_CMD_SIGNAL_LINE_ERPM:
	case DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD:
		return true;
	default:
		return false;
	}
}

int dshot_init(struct dshot_bus *bus, const struct dshot_config *config)
{
	int ret;

	if (!bus || !config)
		return -EINVAL;
	if (config->count < 1 || config->count > DSHOT_MOTORS_MAX)
		return -EINVAL;
	if (config->speed != DSHOT_150 && config->speed != DSHOT_300 &&
	    config->speed != DSHOT_600)
		return -EINVAL;
	if (config->bidirectional && config->speed < DSHOT_BIDIR_SPEED_MIN)
		return -EINVAL;
	if (config->poles % 2)
		return -EINVAL;
	memset(bus, 0, sizeof(*bus));

	ret = esc_dshot_attach(&bus->port, config);
	if (ret)
		return ret;

	for (uint8_t i = 0; i < config->count; i++) {
		struct dshot_motor *motor = &bus->motors[i];

		motor->bus = bus;
		motor->esc.name = "dshot";
		motor->esc.ops = &dshot_ops;
		motor->esc.priv = motor;
		motor->esc.commands = ESC_COMMAND_BIT(ESC_COMMAND_THROTTLE);
	}
	bus->count = config->count;
	bus->poles = config->poles;
	bus->bidirectional = config->bidirectional;
	return 0;
}

struct esc_dev *dshot_esc(struct dshot_bus *bus, unsigned int index)
{
	if (!bus || index >= bus->count)
		return NULL;
	return &bus->motors[index].esc;
}

int dshot_command(struct esc_dev *esc, enum dshot_command command)
{
	struct dshot_motor *motor = dshot_motor_of(esc);
	uint8_t repeats;
	float hold_s;

	if (!motor || (unsigned)command > DSHOT_THROTTLE_MIN - 1 ||
	    !dshot_command_rule((uint8_t)command, &repeats, &hold_s))
		return -EINVAL;
	if (motor->repeats_left || motor->hold_s > 0.0f)
		return -EBUSY;
	if ((unsigned)command <= DSHOT_STOPPED_ONLY_MAX && motor->value)
		return -EBUSY;
	motor->command = (uint8_t)command;
	motor->repeats_left = repeats;
	motor->hold_s = hold_s;
	return 0;
}

/*
 * 1-47 are special commands: sent here they would skip the motor-stopped
 * rule, the repeat count and the wait, and go out every frame -- 12, save
 * settings, at loop rate. They go through dshot_command() only.
 */
int dshot_set_value(struct esc_dev *esc, uint16_t value, bool telemetry)
{
	struct dshot_motor *motor = dshot_motor_of(esc);

	if (!motor || value > DSHOT_VALUE_MAX ||
	    (value && value < DSHOT_THROTTLE_MIN))
		return -EINVAL;
	motor->value = value;
	motor->telemetry = telemetry;
	return 0;
}

int dshot_request_telemetry(struct esc_dev *esc)
{
	struct dshot_motor *motor = dshot_motor_of(esc);

	if (!motor)
		return -EINVAL;
	motor->telemetry_once = true;
	return 0;
}

/* What this motor sends now. Changes nothing: see dshot_frame_sent(). */
static uint16_t dshot_frame_now(const struct dshot_motor *motor)
{
	/* Bidirectional DShot inverts the CRC [BW "Bidirectional DSHOT"]. */
	bool inverted = motor->bus->bidirectional;

	/* The command frames: telemetry bit set [BLH line 30]. */
	if (motor->repeats_left)
		return dshot_frame(motor->command, true, inverted);
	if (motor->hold_s > 0.0f)
		return dshot_frame(0, false, inverted);
	return dshot_frame(motor->value,
			   motor->telemetry || motor->telemetry_once, inverted);
}

/*
 * The frame went out: count the repeat or the wait, or spend the one-shot
 * telemetry bit. Only then -- a frame that failed to go counts for nothing.
 */
static void dshot_frame_sent(struct dshot_motor *motor, float dt_s)
{
	if (motor->repeats_left) {
		/* EDT is on once its command has gone out in full [EDT
		 * "Enabling EDT"]; the ESC answers with its version. */
		if (--motor->repeats_left == 0 &&
		    motor->command == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE) {
			motor->edt = true;
			motor->edt_version_next = true;
		}
	} else if (motor->hold_s > 0.0f)
		motor->hold_s -= dt_s;
	else
		motor->telemetry_once = false;
}

/*
 * One EDT frame [EDT "EDT Frames"] into the telemetry. The status frame
 * after command 13 is the version; 0xEFF says EDT is off again [EDT
 * "Enabling EDT", "Disabling EDT"].
 */
static void dshot_take_edt(struct dshot_motor *motor, uint16_t data)
{
	struct dshot_telemetry *telemetry = &motor->reported;
	uint8_t value = dshot_edt_value(data);

	switch (dshot_edt_type(data)) {
	case DSHOT_EDT_TEMPERATURE:
		telemetry->temperature_c = value;
		telemetry->has |= DSHOT_HAS_TEMPERATURE;
		break;
	case DSHOT_EDT_VOLTAGE:
		telemetry->voltage_v = value * 0.25f;
		telemetry->has |= DSHOT_HAS_VOLTAGE;
		break;
	case DSHOT_EDT_CURRENT:
		telemetry->current_a = value;
		telemetry->has |= DSHOT_HAS_CURRENT;
		break;
	case DSHOT_EDT_DEBUG1:
		telemetry->debug1 = value;
		telemetry->has |= DSHOT_HAS_DEBUG1;
		break;
	case DSHOT_EDT_DEBUG2:
		telemetry->debug2 = value;
		telemetry->has |= DSHOT_HAS_DEBUG2;
		break;
	case DSHOT_EDT_STRESS:
		telemetry->stress = value;
		telemetry->has |= DSHOT_HAS_STRESS;
		break;
	default:	/* DSHOT_EDT_STATUS */
		if (data == DSHOT_EDT_OFF_REPLY) {
			motor->edt = false;
			motor->edt_version_next = false;
		} else if (motor->edt_version_next) {
			telemetry->edt_version = value;
			telemetry->has |= DSHOT_HAS_EDT_VERSION;
			motor->edt_version_next = false;
		} else {
			telemetry->status = value;
			telemetry->has |= DSHOT_HAS_STATUS;
		}
		break;
	}
}

/*
 * The answer to the last frame, from the port: eRPM, or with EDT on maybe
 * an EDT frame. Nothing sent since the last read is no reply at all; one
 * missing or garbled counts as an error, and the age keeps growing.
 */
static void dshot_take_reply(struct dshot_motor *motor, int ret, uint32_t raw,
			     float dt_s)
{
	struct dshot_telemetry *telemetry = &motor->reported;
	uint16_t data;
	uint32_t erpm;

	motor->reply_age_s += dt_s;
	if (ret == -ENOENT)
		return;
	if (!ret)
		ret = dshot_reply_decode(raw, &data);
	if (!ret && motor->edt && dshot_reply_is_edt(data)) {
		dshot_take_edt(motor, data);
	} else if (!ret) {
		ret = dshot_reply_erpm(data, &erpm);
		if (!ret) {
			telemetry->erpm = erpm;
			telemetry->has |= DSHOT_HAS_ERPM;
		}
	}
	if (ret) {
		telemetry->errors++;
		return;
	}
	telemetry->replies++;
	motor->reply_age_s = 0.0f;
}

int dshot_update(struct dshot_bus *bus, float dt_s)
{
	uint16_t frames[DSHOT_MOTORS_MAX];
	uint32_t raw;
	int ret;

	if (!bus || !bus->count || !isfinite(dt_s) || dt_s < 0.0f)
		return -EINVAL;
	/* Nothing changes state until the frame can go. */
	ret = esc_dshot_busy(&bus->port);
	if (ret)
		return ret;
	if (bus->bidirectional) {
		for (uint8_t i = 0; i < bus->count; i++) {
			raw = 0;
			ret = esc_dshot_reply(&bus->port, i, &raw);
			dshot_take_reply(&bus->motors[i], ret, raw, dt_s);
		}
	}
	for (uint8_t i = 0; i < bus->count; i++)
		frames[i] = dshot_frame_now(&bus->motors[i]);
	ret = esc_dshot_send(&bus->port, frames);
	if (ret)
		return ret;
	for (uint8_t i = 0; i < bus->count; i++)
		dshot_frame_sent(&bus->motors[i], dt_s);
	return 0;
}

/* The motor of a bidirectional bus, or NULL with *ret the reason. */
static struct dshot_motor *dshot_listener_of(struct esc_dev *esc, int *ret)
{
	struct dshot_motor *motor = dshot_motor_of(esc);

	*ret = -EINVAL;
	if (!motor)
		return NULL;
	*ret = -EOPNOTSUPP;
	if (!motor->bus->bidirectional)
		return NULL;
	*ret = 0;
	return motor;
}

int dshot_get_erpm(struct esc_dev *esc, uint32_t *erpm)
{
	int ret;
	struct dshot_motor *motor = dshot_listener_of(esc, &ret);

	if (!motor)
		return ret;
	if (!erpm)
		return -EINVAL;
	if (!(motor->reported.has & DSHOT_HAS_ERPM))
		return -ENODATA;
	*erpm = motor->reported.erpm;
	return 0;
}

int dshot_get_rpm(struct esc_dev *esc, unsigned int poles, uint32_t *rpm)
{
	uint32_t erpm;
	int ret;

	if (!poles || poles % 2 || !rpm)
		return -EINVAL;
	ret = dshot_get_erpm(esc, &erpm);
	if (ret)
		return ret;
	*rpm = dshot_rpm(erpm, poles);
	return 0;
}

int dshot_get_telemetry(struct esc_dev *esc,
			struct dshot_telemetry *telemetry)
{
	int ret;
	struct dshot_motor *motor = dshot_listener_of(esc, &ret);

	if (!motor)
		return ret;
	if (!telemetry)
		return -EINVAL;
	*telemetry = motor->reported;
	return 0;
}

void dshot_del(struct dshot_bus *bus)
{
	if (!bus || !bus->count)
		return;
	esc_dshot_detach(&bus->port);
	memset(bus, 0, sizeof(*bus));
}

/* Throttle: 0 is motor stop, above it 48..2047; clears the telemetry bit. */
static int dshot_set_esc_command(struct esc_dev *esc, enum esc_command kind,
				 float value)
{
	struct dshot_motor *motor = esc->priv;

	(void)kind;	/* throttle: the only kind in commands */
	motor->value = dshot_throttle_value(value);
	motor->telemetry = false;
	return 0;
}

/*
 * Bidirectional only: the shaft's velocity from eRPM, given config.poles,
 * and the EDT temperature, each once it has arrived; online while good
 * replies keep coming (DSHOT_REPLY_TIMEOUT_S).
 */
static int dshot_get_esc_feedback(struct esc_dev *esc,
				  struct esc_feedback *feedback)
{
	struct dshot_motor *motor = esc->priv;
	const struct dshot_telemetry *telemetry = &motor->reported;
	uint8_t poles = motor->bus->poles;

	if (!motor->bus->bidirectional)
		return -EOPNOTSUPP;
	if (poles && (telemetry->has & DSHOT_HAS_ERPM)) {
		feedback->velocity_rad_s = dshot_erpm_to_rad_s(telemetry->erpm,
							       poles);
		feedback->has |= ESC_HAS_VELOCITY;
	}
	if (telemetry->has & DSHOT_HAS_TEMPERATURE) {
		feedback->temperature_c = telemetry->temperature_c;
		feedback->has |= ESC_HAS_TEMPERATURE;
	}
	feedback->online = telemetry->replies &&
			   motor->reply_age_s <= DSHOT_REPLY_TIMEOUT_S;
	return 0;
}

#endif /* CONFIG_ESC_DSHOT */
