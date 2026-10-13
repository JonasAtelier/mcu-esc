/*
 * DJI RoboMaster C610, C620 and GM6020 on CAN, through mcu-can's struct
 * can_dev. One frame family: the app commands up to four motors per frame,
 * each motor answers with its own feedback frame, standard identifiers,
 * DLC 8, every 16-bit field high byte first, 1 Mbit/s.
 *
 *   model   motor          ID    command (current)       feedback
 *   C610    M2006          1-8   0x200 (1-4), 0x1FF (5-8) 0x200 + ID
 *   C620    M3508          1-8   0x200 (1-4), 0x1FF (5-8) 0x200 + ID
 *   GM6020  built in       1-7   0x1FE (1-4), 0x2FE (5-7) 0x204 + ID
 *                                voltage: 0x1FF, 0x2FF
 *
 * Every value, with its page, is in src/dji/dji_proto.h. A GM6020 at ID n
 * and a C610/C620 at ID n + 4 share a feedback identifier (and, in voltage
 * mode, a command slot), so they cannot share a bus; dji_add() refuses the
 * second one.
 *
 * One struct dji_bus per CAN bus, placed by the app. The library never
 * configures or owns the bus: start it at 1 Mbit/s yourself, then
 *
 *   dji_init(&bus, can);
 *   dji_add(&bus, &(struct dji_motor_config){ .model = DJI_MODEL_C620,
 *                                            .id = 1 }, &wheel);
 *   loop:
 *     while (can_recv(can, &frame, 0) == 0)
 *         dji_handle_frame(&bus, &frame);
 *     esc_set_command(wheel, ESC_COMMAND_CURRENT, amps);
 *     dji_update(&bus, dt_s);
 *
 * dji_update() is the only call that sends. A motor whose feedback stops for
 * its timeout is offline, and is sent 0 until feedback returns; its stored
 * command is kept and applied again then.
 *
 * Not thread-safe: call everything for one bus from one task, or hold your
 * own lock around it.
 *
 * Feedback through esc_get_feedback(): angle (the rotor, single turn),
 * position (multi-turn, divided by gear_ratio), velocity (rotor rpm divided
 * by gear_ratio) on all three; temperature on the C620 only, the one manual
 * that gives its unit. The torque current the ESCs report has no scale in
 * any of the three manuals: it is in dji_get_raw() only, as counts.
 */
#ifndef DJI_H
#define DJI_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"
#include "esc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Feedback identifiers 0x201 to 0x20B, one motor each. */
#define DJI_MOTORS_MAX		11

/* Feedback older than this marks a motor offline, when the config says 0. */
#define DJI_DEFAULT_TIMEOUT_S	0.1f

enum dji_model {
	DJI_MODEL_C610,
	DJI_MODEL_C620,
	DJI_MODEL_GM6020,
};

struct dji_motor_config {
	enum dji_model model;
	uint8_t id;		/* as set on the ESC: LED blinks or DIP switch */
	float gear_ratio;	/* rotor turns per output turn; 0 for 1 */
	float timeout_s;	/* 0 for DJI_DEFAULT_TIMEOUT_S */
	/*
	 * GM6020 only. false: voltage mode, the motor's default, commanded
	 * with dji_set_raw(). true: current mode, which needs motor firmware
	 * v1.0.11.2 or later with "Current Ring" switched on in RoboMaster
	 * Assistant; then ESC_COMMAND_CURRENT works.
	 */
	bool gm6020_current_mode;
};

/* The last feedback frame as it came, and the command as it goes out. */
struct dji_raw {
	uint16_t angle;		/* 0..8191 over one rotor turn */
	int16_t rpm;		/* rotor */
	int16_t current;	/* actual torque current; scale not given */
	uint8_t temperature;	/* C620: deg C. GM6020: unit not given.
				 * C610: the byte is Null */
	int16_t command;	/* the counts the next update sends */
};

struct dji_bus;

/* One motor. Private: read it through the calls below. */
struct dji_motor {
	struct esc_dev esc;
	struct dji_bus *bus;
	enum dji_model model;
	uint8_t id;
	uint16_t command_id;	/* the frame that carries its command */
	uint8_t slot;		/* 0..3 in that frame */
	uint16_t feedback_id;
	int16_t raw_max;	/* command range is +-raw_max */
	float amps_max;		/* raw_max in amperes; 0: not current */
	float gear_ratio;
	float timeout_s;
	float since_feedback_s;
	bool online;
	bool seen;		/* a feedback frame has arrived */
	int64_t position;	/* rotor counts, multi-turn; int32 overflows
				 * after 262144 turns, minutes at full speed */
	struct dji_raw raw;
};

struct dji_bus {
	struct can_dev *can;
	struct dji_motor motors[DJI_MOTORS_MAX];
	uint8_t count;
};

/* Empty the bus; can must be started by the app and outlive bus. */
int dji_init(struct dji_bus *bus, struct can_dev *can);

/*
 * Add one motor and hand out its esc_dev. -EINVAL for an ID outside the
 * model's range or a bad ratio or timeout, -EOPNOTSUPP for a model whose
 * CONFIG_ESC_<MODEL> is off, -EADDRINUSE when the motor's feedback
 * identifier is already taken on this bus -- which is also how a bus is
 * full, and how a shared command slot shows. Offline until its first
 * feedback frame.
 */
int dji_add(struct dji_bus *bus, const struct dji_motor_config *config,
	    struct esc_dev **esc);

/*
 * Feed one received frame. 0 when it was a motor's feedback, -ENOENT when
 * it was not for this bus -- pass it on to whatever else is on the bus.
 */
int dji_handle_frame(struct dji_bus *bus, const struct can_frame *frame);

/*
 * Age the feedback by dt_s seconds, then send every command frame that has a
 * motor in it, without waiting for room. -EINVAL for a dt_s that is not
 * finite and positive; can_send()'s error when a frame could not be queued
 * (the others are still sent); else -ENOTCONN when a motor is offline; else
 * 0.
 */
int dji_update(struct dji_bus *bus, float dt_s);

/*
 * Command in the model's raw counts, clamped to its range: the only way to
 * command a GM6020 in voltage mode (+-25000, GM6020 User Guide v1.4; v1.2
 * said +-30000). On a current-mode model the same as ESC_COMMAND_CURRENT
 * without the scaling. -EINVAL for an esc_dev that is not a DJI motor.
 */
int dji_set_raw(struct esc_dev *esc, int16_t command);

/* The raw counts. -EINVAL for an esc_dev that is not a DJI motor. */
int dji_get_raw(struct esc_dev *esc, struct dji_raw *raw);

#ifdef __cplusplus
}
#endif

#endif /* DJI_H */
