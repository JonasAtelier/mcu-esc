/*
 * esc -- a small Linux-IIO-style subsystem for brushless motor drivers on
 * ESP-IDF and the STM32 HAL: drivers that take a command and, where they
 * have it, return feedback. Motors with a driver built in count too.
 *
 * One struct esc_dev per motor. It holds what every driver here shares: a
 * command in physical units, and the feedback decoded the same way. How a
 * command reaches the motor stays with each model, deliberately. One DJI
 * frame carries four motors' commands, an HT-03 answers only the frame it is
 * sent, and a C615 is a pulse on a timer pin; a shared "send now" call would
 * be wrong for two of them.
 *
 * Each model keeps its own API (dji.h, ht03.h, c615.h). The model's handle
 * holds a struct esc_dev per motor and hands it out, for code that treats
 * every motor the same way.
 *
 * No clock is read anywhere in this library: the app passes the elapsed time
 * to each model's update call, which is also where feedback ages.
 *
 * Every int-returning function returns 0 or a negative errno. A command kind
 * a model does not take is -EOPNOTSUPP.
 */
#ifndef ESC_H
#define ESC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum esc_command {
	ESC_COMMAND_CURRENT,	/* torque current, amperes */
	ESC_COMMAND_TORQUE,	/* newton metres at the output shaft */
	ESC_COMMAND_THROTTLE,	/* 0..1 of the throttle range */
};

/* The bit of one kind in esc_dev.commands. */
#define ESC_COMMAND_BIT(kind)	(1u << (kind))

/* Which fields of struct esc_feedback a model fills. */
#define ESC_HAS_ANGLE		(1u << 0)
#define ESC_HAS_POSITION	(1u << 1)
#define ESC_HAS_VELOCITY	(1u << 2)
#define ESC_HAS_TORQUE		(1u << 3)
#define ESC_HAS_TEMPERATURE	(1u << 4)

/*
 * The last feedback, decoded. A field whose ESC_HAS_* bit is clear reads 0;
 * each model's header says which it has.
 */
struct esc_feedback {
	uint32_t has;		/* ESC_HAS_* */
	float angle_rad;	/* single turn, 0..2pi, of the shaft the
				 * driver's own sensor reads */
	float position_rad;	/* output shaft, multi-turn */
	float velocity_rad_s;	/* output shaft */
	float torque_nm;	/* output shaft */
	float temperature_c;
	bool online;		/* feedback arrived within its timeout */
};

struct esc_dev;

struct esc_ops {
	/* The kind is one of commands and value is finite: checked already. */
	int (*set_command)(struct esc_dev *esc, enum esc_command kind,
			   float value);
	/* optional; *feedback arrives zeroed */
	int (*get_feedback)(struct esc_dev *esc,
			    struct esc_feedback *feedback);
};

struct esc_dev {
	const char *name;		/* "c620"; log prefix */
	const struct esc_ops *ops;
	void *priv;			/* the model's per-motor state */
	uint32_t commands;		/* ESC_COMMAND_BIT()s it takes */
};

/*
 * Store a command, clamped to the model's range. It reaches the motor
 * through the model: at its next update call on CAN, at once on PWM.
 * -EINVAL for a value that is not finite or an unknown kind, -EOPNOTSUPP
 * for a kind the model does not take.
 */
int esc_set_command(struct esc_dev *esc, enum esc_command kind, float value);

/*
 * The last feedback, written only on success. -EOPNOTSUPP for a model with
 * none.
 */
int esc_get_feedback(struct esc_dev *esc, struct esc_feedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* ESC_H */
