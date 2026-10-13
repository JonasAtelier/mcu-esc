/*
 * src/dji/dji.c against a fake can_send() defined here: placing motors and
 * refusing clashes, decoding feedback into the esc_dev, the multi-turn
 * count, the timeout and its zero command, and the frames dji_update()
 * puts on the bus, byte by byte.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "dji.h"
#include "dji_math.h"
#include "esc.h"

/* ------------------------------------------------------------ fake bus -- */

static struct can_frame sent[16];
static uint32_t sent_timeout[16];
static int sent_count;
static uint32_t fail_id;	/* can_send() of this identifier fails */

int can_send(struct can_dev *can, const struct can_frame *frame,
	     uint32_t timeout_ms)
{
	(void)can;
	if (fail_id && frame->id == fail_id)
		return -EAGAIN;
	assert(sent_count < 16);
	sent_timeout[sent_count] = timeout_ms;
	sent[sent_count++] = *frame;
	return 0;
}

static struct can_dev can;

/* ------------------------------------------------------------- helpers -- */

static const struct can_frame *sent_with(uint32_t id)
{
	for (int i = 0; i < sent_count; i++)
		if (sent[i].id == id)
			return &sent[i];
	return NULL;
}

static int16_t slot_of(const struct can_frame *frame, int slot)
{
	return (int16_t)dji_read_be16(&frame->data[slot * 2]);
}

static struct can_frame feedback(uint32_t id, uint16_t angle, int16_t rpm,
				 int16_t current, uint8_t temperature)
{
	struct can_frame frame = { .id = id, .dlc = 8 };

	dji_write_be16(&frame.data[0], (int16_t)angle);
	dji_write_be16(&frame.data[2], rpm);
	dji_write_be16(&frame.data[4], current);
	frame.data[6] = temperature;
	return frame;
}

static struct esc_dev *add(struct dji_bus *bus, enum dji_model model,
			   uint8_t id)
{
	struct dji_motor_config config = { .model = model, .id = id };
	struct esc_dev *esc = NULL;

	assert(dji_add(bus, &config, &esc) == 0 && esc);
	return esc;
}

static int try_add(struct dji_bus *bus, enum dji_model model, uint8_t id,
		   bool current_mode)
{
	struct dji_motor_config config = {
		.model = model, .id = id, .gm6020_current_mode = current_mode,
	};
	struct esc_dev *esc;

	return dji_add(bus, &config, &esc);
}

/* --------------------------------------------------------------- tests -- */

static void test_add(void)
{
	struct dji_bus bus;
	struct dji_motor_config config = { .model = DJI_MODEL_C620, .id = 1 };
	struct esc_dev *esc;

	assert(dji_init(NULL, &can) == -EINVAL);
	assert(dji_init(&bus, NULL) == -EINVAL);
	assert(dji_init(&bus, &can) == 0);
	assert(dji_add(&bus, NULL, &esc) == -EINVAL);
	assert(dji_add(&bus, &config, NULL) == -EINVAL);

	/* IDs: 1-8 on the C6x0, 1-7 on the GM6020 */
	assert(try_add(&bus, DJI_MODEL_C620, 0, false) == -EINVAL);
	assert(try_add(&bus, DJI_MODEL_C620, 9, false) == -EINVAL);
	assert(try_add(&bus, DJI_MODEL_C610, 9, false) == -EINVAL);
	assert(try_add(&bus, DJI_MODEL_GM6020, 0, false) == -EINVAL);
	assert(try_add(&bus, DJI_MODEL_GM6020, 8, false) == -EINVAL);
	assert(try_add(&bus, (enum dji_model)7, 1, false) == -EINVAL);

	config.gear_ratio = -1.0f;
	assert(dji_add(&bus, &config, &esc) == -EINVAL);
	config.gear_ratio = NAN;
	assert(dji_add(&bus, &config, &esc) == -EINVAL);
	config.gear_ratio = 0.0f;
	config.timeout_s = -0.1f;
	assert(dji_add(&bus, &config, &esc) == -EINVAL);
	config.timeout_s = INFINITY;
	assert(dji_add(&bus, &config, &esc) == -EINVAL);
	assert(bus.count == 0);

	/* The names and commands each model takes */
	esc = add(&bus, DJI_MODEL_C620, 1);
	assert(strcmp(esc->name, "c620") == 0 &&
	       esc->commands == ESC_COMMAND_BIT(ESC_COMMAND_CURRENT));
	esc = add(&bus, DJI_MODEL_C610, 2);
	assert(strcmp(esc->name, "c610") == 0 &&
	       esc->commands == ESC_COMMAND_BIT(ESC_COMMAND_CURRENT));
	esc = add(&bus, DJI_MODEL_GM6020, 7);
	assert(strcmp(esc->name, "gm6020") == 0 && esc->commands == 0);
	assert(bus.count == 3);
}

/*
 * One feedback identifier, one command slot, per motor: a GM6020 at ID n
 * is a C6x0 at ID n + 4 to the bus.
 */
static void test_clashes(void)
{
	struct dji_bus bus;

	dji_init(&bus, &can);
	assert(try_add(&bus, DJI_MODEL_C620, 5, false) == 0);
	/* 0x205 both, and 0x1FF slot 0 both in voltage mode */
	assert(try_add(&bus, DJI_MODEL_GM6020, 1, false) == -EADDRINUSE);
	/* in current mode the command moves to 0x1FE; 0x205 still clashes */
	assert(try_add(&bus, DJI_MODEL_GM6020, 1, true) == -EADDRINUSE);
	/* a C610 at the same ID is the same frames */
	assert(try_add(&bus, DJI_MODEL_C610, 5, false) == -EADDRINUSE);
	/* the same motor twice */
	assert(try_add(&bus, DJI_MODEL_C620, 5, false) == -EADDRINUSE);
	assert(bus.count == 1);

	/* Next to each other, without a clash */
	assert(try_add(&bus, DJI_MODEL_C620, 1, false) == 0);	/* 0x201 */
	assert(try_add(&bus, DJI_MODEL_GM6020, 2, false) == 0);	/* 0x206 */
	assert(try_add(&bus, DJI_MODEL_C610, 6, false) == -EADDRINUSE);

	/* Full: C6x0 1-4 and GM6020 1-7 are all 11 feedback identifiers */
	dji_init(&bus, &can);
	for (uint8_t id = 1; id <= 4; id++)
		assert(try_add(&bus, DJI_MODEL_C620, id, false) == 0);
	for (uint8_t id = 1; id <= 7; id++)
		assert(try_add(&bus, DJI_MODEL_GM6020, id, id & 1) == 0);
	assert(bus.count == DJI_MOTORS_MAX);
	for (uint8_t id = 1; id <= 8; id++)
		assert(try_add(&bus, DJI_MODEL_C610, id, false) ==
		       -EADDRINUSE);
}

static void test_frames(void)
{
	struct dji_bus bus;
	struct esc_dev *c620_2, *c620_4, *c620_6, *c610_8, *gm_v3, *gm_c5;
	const struct can_frame *frame;
	struct dji_motor_config config = {
		.model = DJI_MODEL_GM6020, .id = 5,
		.gm6020_current_mode = true,
	};
	struct can_frame in;

	dji_init(&bus, &can);
	c620_2 = add(&bus, DJI_MODEL_C620, 2);	/* 0x200 slot 1 */
	c620_4 = add(&bus, DJI_MODEL_C620, 4);	/* 0x200 slot 3 */
	c620_6 = add(&bus, DJI_MODEL_C620, 6);	/* 0x1FF slot 1 */
	c610_8 = add(&bus, DJI_MODEL_C610, 8);	/* 0x1FF slot 3 */
	gm_v3 = add(&bus, DJI_MODEL_GM6020, 3);	/* 0x1FF slot 2, voltage */
	assert(dji_add(&bus, &config, &gm_c5) == 0);	/* 0x2FE slot 0 */

	assert(esc_set_command(c620_2, ESC_COMMAND_CURRENT, 10.0f) == 0);
	assert(esc_set_command(c620_4, ESC_COMMAND_CURRENT, 3.0f) == 0);
	assert(esc_set_command(c620_6, ESC_COMMAND_CURRENT, -20.0f) == 0);
	assert(esc_set_command(c610_8, ESC_COMMAND_CURRENT, 2.5f) == 0);
	assert(esc_set_command(gm_v3, ESC_COMMAND_CURRENT, 1.0f) ==
	       -EOPNOTSUPP);
	assert(dji_set_raw(gm_v3, -30000) == 0);
	assert(esc_set_command(gm_c5, ESC_COMMAND_CURRENT, 1.5f) == 0);
	assert(esc_set_command(c620_2, ESC_COMMAND_TORQUE, 1.0f) ==
	       -EOPNOTSUPP);

	/* Nobody has answered yet: every slot goes out 0 */
	sent_count = 0;
	assert(dji_update(&bus, 0.001f) == -ENOTCONN);
	assert(sent_count == 3);
	for (int i = 0; i < sent_count; i++)
		for (int b = 0; b < 8; b++)
			assert(sent[i].data[b] == 0);

	/* Feedback from everyone, then the commands go out */
	in = feedback(0x202, 0, 0, 0, 0);
	assert(dji_handle_frame(&bus, &in) == 0);
	in.id = 0x204;
	assert(dji_handle_frame(&bus, &in) == 0);
	in.id = 0x206;
	assert(dji_handle_frame(&bus, &in) == 0);
	in.id = 0x208;
	assert(dji_handle_frame(&bus, &in) == 0);
	in.id = 0x207;		/* GM6020 ID 3 */
	assert(dji_handle_frame(&bus, &in) == 0);
	in.id = 0x209;		/* GM6020 ID 5 */
	assert(dji_handle_frame(&bus, &in) == 0);

	sent_count = 0;
	assert(dji_update(&bus, 0.001f) == 0);
	/* 0x1FE (none), 0x1FF, 0x200, 0x2FE, 0x2FF (none): in that order */
	assert(sent_count == 3);
	assert(sent[0].id == 0x1FF && sent[1].id == 0x200 &&
	       sent[2].id == 0x2FE);
	for (int i = 0; i < sent_count; i++) {
		assert(sent[i].dlc == 8 && !sent[i].ext && !sent[i].rtr);
		assert(sent_timeout[i] == 0);
	}
	frame = sent_with(0x200);
	/* 3 A of 20 is 2457.6 counts -> 2458, ID 4 the last slot of 0x200 */
	assert(slot_of(frame, 0) == 0 && slot_of(frame, 1) == 8192 &&
	       slot_of(frame, 2) == 0 && slot_of(frame, 3) == 2458);
	assert(frame->data[2] == 0x20 && frame->data[3] == 0x00);
	frame = sent_with(0x1FF);
	assert(slot_of(frame, 0) == 0 && slot_of(frame, 1) == -16384 &&
	       slot_of(frame, 2) == -25000 && slot_of(frame, 3) == 2500);
	assert(frame->data[2] == 0xC0 && frame->data[3] == 0x00);
	frame = sent_with(0x2FE);
	assert(slot_of(frame, 0) == 8192 && frame->data[6] == 0 &&
	       frame->data[7] == 0);

	/* One frame that cannot be queued: the others still go */
	fail_id = 0x200;
	sent_count = 0;
	assert(dji_update(&bus, 0.001f) == -EAGAIN);
	assert(sent_count == 2 && !sent_with(0x200));
	fail_id = 0;

	assert(dji_update(&bus, 0.0f) == -EINVAL);
	assert(dji_update(&bus, -1.0f) == -EINVAL);
	assert(dji_update(&bus, NAN) == -EINVAL);
	assert(dji_update(NULL, 0.001f) == -EINVAL);
}

static void test_feedback(void)
{
	struct dji_bus bus;
	struct dji_motor_config config = {
		.model = DJI_MODEL_C620, .id = 3, .gear_ratio = 2.0f,
	};
	struct esc_dev *c620, *c610, *gm;
	struct esc_feedback fb;
	struct dji_raw raw;
	struct can_frame in;

	dji_init(&bus, &can);
	assert(dji_add(&bus, &config, &c620) == 0);
	c610 = add(&bus, DJI_MODEL_C610, 1);
	gm = add(&bus, DJI_MODEL_GM6020, 2);	/* 0x206 */

	/* Before any frame: offline, the fields it has read 0 */
	assert(esc_get_feedback(c620, &fb) == 0);
	assert(!fb.online && fb.position_rad == 0.0f);

	/* Not for this bus */
	in = feedback(0x203, 100, 0, 0, 0);
	in.ext = true;
	assert(dji_handle_frame(&bus, &in) == -ENOENT);
	in.ext = false;
	in.rtr = true;
	assert(dji_handle_frame(&bus, &in) == -ENOENT);
	in.rtr = false;
	in.dlc = 7;
	assert(dji_handle_frame(&bus, &in) == -ENOENT);
	in.dlc = 8;
	in.id = 0x204;		/* nobody at C6x0 ID 4 */
	assert(dji_handle_frame(&bus, &in) == -ENOENT);
	in.id = 0x202;		/* nobody at C6x0 ID 2: GM6020 2 is 0x206 */
	assert(dji_handle_frame(&bus, &in) == -ENOENT);
	assert(dji_handle_frame(&bus, NULL) == -EINVAL);
	assert(dji_handle_frame(NULL, &in) == -EINVAL);

	/* C620 ID 3: angle 2048, -600 rpm, current -1234, 45 deg C */
	in = feedback(0x203, 2048, -600, -1234, 45);
	assert(dji_handle_frame(&bus, &in) == 0);
	assert(esc_get_feedback(c620, &fb) == 0);
	assert(fb.online);
	assert(fb.has == (ESC_HAS_ANGLE | ESC_HAS_POSITION | ESC_HAS_VELOCITY |
			  ESC_HAS_TEMPERATURE));
	assert(fabsf(fb.angle_rad - DJI_TWO_PI / 4.0f) < 1e-5f);
	/* the first frame sets the start: position counts from here */
	assert(fb.position_rad == 0.0f);
	/* -600 rpm is -10 rev/s at the rotor, -5 at the output */
	assert(fabsf(fb.velocity_rad_s + 5.0f * DJI_TWO_PI) < 1e-3f);
	assert(fb.temperature_c == 45.0f);
	assert(fb.torque_nm == 0.0f);
	assert(dji_get_raw(c620, &raw) == 0);
	assert(raw.angle == 2048 && raw.rpm == -600 && raw.current == -1234 &&
	       raw.temperature == 45);

	/* Forward across the wrap, twice round: 2048 -> 8000 -> 100 */
	in = feedback(0x203, 8000, 0, 0, 45);
	dji_handle_frame(&bus, &in);	/* 2048 -> 8000: -2240 */
	in = feedback(0x203, 100, 0, 0, 45);
	dji_handle_frame(&bus, &in);	/* 8000 -> 100: +292 */
	assert(esc_get_feedback(c620, &fb) == 0);
	assert(fabsf(fb.position_rad - dji_counts_to_rad(-2240 + 292, 2.0f)) <
	       1e-5f);
	/* the angle keeps one turn: bits above 8191 are not angle */
	in = feedback(0x203, 0x2000 | 100, 0, 0, 45);
	dji_handle_frame(&bus, &in);
	assert(dji_get_raw(c620, &raw) == 0 && raw.angle == 100);

	/* C610 and GM6020: no temperature through the esc_dev */
	in = feedback(0x201, 0, 60, 0, 99);
	assert(dji_handle_frame(&bus, &in) == 0);
	assert(esc_get_feedback(c610, &fb) == 0);
	assert(!(fb.has & ESC_HAS_TEMPERATURE) && fb.temperature_c == 0.0f);
	assert(fabsf(fb.velocity_rad_s - DJI_TWO_PI) < 1e-4f);
	in = feedback(0x206, 0, 0, 0, 40);
	assert(dji_handle_frame(&bus, &in) == 0);
	assert(esc_get_feedback(gm, &fb) == 0 && fb.online);
	assert(!(fb.has & ESC_HAS_TEMPERATURE));
	assert(dji_get_raw(gm, &raw) == 0 && raw.temperature == 40);
}

/*
 * Past 2^31 rotor counts: 262144 turns, about a quarter of an hour of an
 * M2006 at full speed. 600000 frames of +4000 counts each is 2.4e9.
 */
static void test_long_run(void)
{
	struct dji_bus bus;
	struct esc_dev *esc;
	struct esc_feedback fb;
	struct can_frame in;
	const int frames = 600000;
	double expected;

	dji_init(&bus, &can);
	esc = add(&bus, DJI_MODEL_C610, 1);
	for (int k = 0; k <= frames; k++) {
		in = feedback(0x201, (uint16_t)((4000LL * k) % 8192), 0, 0, 0);
		assert(dji_handle_frame(&bus, &in) == 0);
	}
	assert(bus.motors[0].position == 4000LL * frames);
	assert(bus.motors[0].position > INT32_MAX);
	assert(esc_get_feedback(esc, &fb) == 0);
	expected = 4000.0 * frames * 6.283185307179586 / 8192.0;
	assert(fabs(fb.position_rad - expected) < 1e-5 * expected);

	/* and back down past zero to below -2^31 */
	for (int k = frames - 1; k >= -frames; k--) {
		in = feedback(0x201,
			      (uint16_t)(((4000LL * k) % 8192 + 8192) % 8192),
			      0, 0, 0);
		assert(dji_handle_frame(&bus, &in) == 0);
	}
	assert(bus.motors[0].position == -4000LL * frames);
}

/* Offline after the timeout, 0 on the bus, the command kept for later. */
static void test_timeout(void)
{
	struct dji_bus bus;
	struct dji_motor_config config = {
		.model = DJI_MODEL_C620, .id = 1, .timeout_s = 0.05f,
	};
	struct esc_dev *esc, *other;
	struct esc_feedback fb;
	struct can_frame in = feedback(0x201, 0, 0, 0, 0);

	dji_init(&bus, &can);
	assert(dji_add(&bus, &config, &esc) == 0);
	other = add(&bus, DJI_MODEL_C620, 2);	/* the 0.1 s default */
	esc_set_command(esc, ESC_COMMAND_CURRENT, 5.0f);
	esc_set_command(other, ESC_COMMAND_CURRENT, 5.0f);
	dji_handle_frame(&bus, &in);
	in.id = 0x202;
	dji_handle_frame(&bus, &in);

	sent_count = 0;
	assert(dji_update(&bus, 0.049f) == 0);
	assert(slot_of(&sent[0], 0) == 4096 && slot_of(&sent[0], 1) == 4096);

	sent_count = 0;
	assert(dji_update(&bus, 0.002f) == -ENOTCONN);
	assert(esc_get_feedback(esc, &fb) == 0 && !fb.online);
	assert(esc_get_feedback(other, &fb) == 0 && fb.online);
	assert(slot_of(&sent[0], 0) == 0 && slot_of(&sent[0], 1) == 4096);

	/* 0.1 s for the other one */
	sent_count = 0;
	assert(dji_update(&bus, 0.05f) == -ENOTCONN);
	assert(esc_get_feedback(other, &fb) == 0 && !fb.online);
	assert(slot_of(&sent[0], 1) == 0);

	/* Silent for a long time, then back: the stored command resumes */
	for (int i = 0; i < 1000; i++) {
		sent_count = 0;
		dji_update(&bus, 1000.0f);
	}
	in.id = 0x201;
	dji_handle_frame(&bus, &in);
	sent_count = 0;
	assert(dji_update(&bus, 0.001f) == -ENOTCONN);
	assert(slot_of(&sent[0], 0) == 4096 && slot_of(&sent[0], 1) == 0);
}

/* The DJI-only calls refuse an esc_dev that is not a DJI motor. */
static int other_set(struct esc_dev *esc, enum esc_command kind, float value)
{
	(void)esc;
	(void)kind;
	(void)value;
	return 0;
}

static void test_raw(void)
{
	static const struct esc_ops other_ops = { .set_command = other_set };
	/* priv points somewhere real: the check must be on ops, not priv */
	static struct dji_motor decoy;
	struct esc_dev other = {
		.name = "other", .ops = &other_ops, .priv = &decoy,
	};
	struct dji_bus bus;
	struct dji_raw raw;
	struct esc_dev *c610;

	assert(dji_set_raw(&other, 1) == -EINVAL);
	assert(dji_get_raw(&other, &raw) == -EINVAL);
	assert(dji_set_raw(NULL, 1) == -EINVAL);
	assert(decoy.raw.command == 0);

	dji_init(&bus, &can);
	c610 = add(&bus, DJI_MODEL_C610, 4);
	assert(dji_get_raw(c610, NULL) == -EINVAL);
	assert(dji_set_raw(c610, 12000) == 0);
	assert(dji_get_raw(c610, &raw) == 0 && raw.command == 10000);
	assert(dji_set_raw(c610, -32768) == 0);
	assert(dji_get_raw(c610, &raw) == 0 && raw.command == -10000);
	assert(esc_set_command(c610, ESC_COMMAND_CURRENT, -0.5f) == 0);
	assert(dji_get_raw(c610, &raw) == 0 && raw.command == -500);
}

int main(void)
{
	test_add();
	test_clashes();
	test_frames();
	test_feedback();
	test_timeout();
	test_long_run();
	test_raw();
	printf("test_dji: all passed\n");
	return 0;
}
