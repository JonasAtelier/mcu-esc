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

static const struct esc_ops dshot_ops = {
	.set_command = dshot_set_esc_command,
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
	if (config->bidirectional)
		return -EOPNOTSUPP;
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
	/* The command frames: telemetry bit set [BLH line 30]. */
	if (motor->repeats_left)
		return dshot_frame(motor->command, true, false);
	if (motor->hold_s > 0.0f)
		return dshot_frame(0, false, false);
	return dshot_frame(motor->value,
			   motor->telemetry || motor->telemetry_once, false);
}

/*
 * The frame went out: count the repeat or the wait, or spend the one-shot
 * telemetry bit. Only then -- a frame that failed to go counts for nothing.
 */
static void dshot_frame_sent(struct dshot_motor *motor, float dt_s)
{
	if (motor->repeats_left)
		motor->repeats_left--;
	else if (motor->hold_s > 0.0f)
		motor->hold_s -= dt_s;
	else
		motor->telemetry_once = false;
}

int dshot_update(struct dshot_bus *bus, float dt_s)
{
	uint16_t frames[DSHOT_MOTORS_MAX];
	int ret;

	if (!bus || !bus->count || !isfinite(dt_s) || dt_s < 0.0f)
		return -EINVAL;
	/* Nothing changes state until the frame can go. */
	ret = esc_dshot_busy(&bus->port);
	if (ret)
		return ret;
	for (uint8_t i = 0; i < bus->count; i++)
		frames[i] = dshot_frame_now(&bus->motors[i]);
	ret = esc_dshot_send(&bus->port, frames);
	if (ret)
		return ret;
	for (uint8_t i = 0; i < bus->count; i++)
		dshot_frame_sent(&bus->motors[i], dt_s);
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

#endif /* CONFIG_ESC_DSHOT */
