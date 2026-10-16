/*
 * The DShot cases, shared by esp/test_dshot.c and stm/test_dshot.c. Each
 * includes this after providing, over its own fake output:
 *
 *   config_for(config, speed, count)  a valid config
 *   sent_frame(motor)    the 16 bits the fake last put out for a motor,
 *                        read back from its pulse widths
 *   sends                how many frames went out (one per update)
 *   line_busy            the last frame is still going out
 *   fail_send            the next sends fail, before any frame goes out
 *   detached             the port has been released
 *
 * and for bidirectional DShot:
 *
 *   bidir_config_for(config, speed, count)  a valid bidirectional config
 *   settle()             the last frame and the reply after it are over
 *   esc_answer[motor]    the 21 line bits the ESC answers every frame
 *                        with, from the next on; 0 for silence
 *
 * Frames are checked against dshot_frame(), which test_dshot_math holds to
 * the sources' worked examples, and the replies are built here from [BW]'s
 * GCR table, which test_dshot_math holds to [BW]'s example.
 */

static int other_set(struct esc_dev *esc, enum esc_command kind, float value)
{
	(void)esc;
	(void)kind;
	(void)value;
	return 0;
}

static const struct esc_ops other_ops = { .set_command = other_set };

static uint16_t frame_of(uint16_t value, bool telemetry)
{
	return dshot_frame(value, telemetry, false);
}

/* The line bits of the reply carrying 12 data bits, CRC as [EDT
 * "Checksum"], GCR and level changes as [BW "eRPM Transmission"]. */
static uint32_t answer_of(uint16_t data)
{
	static const uint8_t to_quintet[16] = {
		0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
		0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
	};
	uint16_t frame = (uint16_t)((data << 4) | dshot_crc(data, false));
	uint32_t gcr = 0, line = 0, level = 0;

	for (int shift = 12; shift >= 0; shift -= 4)
		gcr = (gcr << 5) | to_quintet[(frame >> shift) & 0xF];
	for (int bit = 19; bit >= 0; bit--) {
		if (gcr & (1u << bit))
			level ^= 1;
		line = (line << 1) | level;
	}
	return line;
}

/* One update with the hardware done first, as the loop would see it. */
static int bidir_update(struct dshot_bus *bus, float dt_s)
{
	settle();
	return dshot_update(bus, dt_s);
}

static void test_init_refused(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	config_for(&config, DSHOT_600, 0);
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, DSHOT_MOTORS_MAX + 1);
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, (enum dshot_speed)1200, 1);
	assert(dshot_init(&bus, &config) == -EINVAL);
	/* Bidirectional only from DShot300 [BW]; poles even */
	bidir_config_for(&config, DSHOT_150, 1);
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_300, 1);
	config.poles = 13;
	assert(dshot_init(&bus, &config) == -EINVAL);
	assert(dshot_init(NULL, &config) == -EINVAL);
	assert(dshot_init(&bus, NULL) == -EINVAL);
	assert(sends == 0);
}

static void test_throttle(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct esc_feedback fb;

	config_for(&config, DSHOT_600, 4);
	assert(dshot_init(&bus, &config) == 0);
	assert(dshot_esc(&bus, 4) == NULL && dshot_esc(NULL, 0) == NULL);
	assert(strcmp(dshot_esc(&bus, 0)->name, "dshot") == 0);
	assert(dshot_esc(&bus, 0)->commands ==
	       ESC_COMMAND_BIT(ESC_COMMAND_THROTTLE));

	/* Nothing set yet: every motor at 0, motor stop */
	assert(dshot_update(&bus, 0.001f) == 0);
	for (int i = 0; i < 4; i++)
		assert(sent_frame(i) == 0x0000);

	assert(esc_set_command(dshot_esc(&bus, 1), ESC_COMMAND_THROTTLE,
			       0.5f) == 0);
	assert(esc_set_command(dshot_esc(&bus, 2), ESC_COMMAND_THROTTLE,
			       1.0f) == 0);
	assert(dshot_set_value(dshot_esc(&bus, 3), 1046, true) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x0000);
	assert(sent_frame(1) == frame_of(1048, false));
	assert(sent_frame(2) == frame_of(2047, false));
	/* [BW]'s worked example, telemetry bit on: 1046 -> 0x82C, crc */
	assert(sent_frame(3) == frame_of(1046, true));
	/* the raw value's bit stays on, frame after frame */
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(3) == frame_of(1046, true));
	/* until the throttle mapping takes the motor back */
	assert(esc_set_command(dshot_esc(&bus, 3), ESC_COMMAND_THROTTLE,
			       0.0f) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(3) == 0x0000);

	/* A one-shot telemetry request: this frame only */
	assert(dshot_request_telemetry(dshot_esc(&bus, 2)) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(2) == frame_of(2047, true));
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(2) == frame_of(2047, false));

	/* Refused: past 11 bits, and 1-47, the special commands, which only
	 * dshot_command() sends -- 12 would be save settings every frame */
	assert(dshot_set_value(dshot_esc(&bus, 0), 2048, false) == -EINVAL);
	assert(dshot_set_value(dshot_esc(&bus, 0), 1, false) == -EINVAL);
	assert(dshot_set_value(dshot_esc(&bus, 0), 12, true) == -EINVAL);
	assert(dshot_set_value(dshot_esc(&bus, 0), 47, false) == -EINVAL);
	assert(dshot_set_value(dshot_esc(&bus, 0), 0, false) == 0);
	assert(dshot_set_value(dshot_esc(&bus, 0), 48, false) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == frame_of(48, false));
	assert(dshot_set_value(dshot_esc(&bus, 0), 0, false) == 0);
	assert(esc_set_command(dshot_esc(&bus, 0), ESC_COMMAND_CURRENT,
			       1.0f) == -EOPNOTSUPP);
	assert(esc_set_command(dshot_esc(&bus, 0), ESC_COMMAND_THROTTLE,
			       NAN) == -EINVAL);
	assert(esc_get_feedback(dshot_esc(&bus, 0), &fb) == -EOPNOTSUPP);
	{
		/* nothing comes back on a bus that is not bidirectional */
		struct dshot_telemetry telemetry;
		uint32_t erpm;

		assert(dshot_get_erpm(dshot_esc(&bus, 0), &erpm) ==
		       -EOPNOTSUPP);
		assert(dshot_get_rpm(dshot_esc(&bus, 0), 14, &erpm) ==
		       -EOPNOTSUPP);
		assert(dshot_get_telemetry(dshot_esc(&bus, 0), &telemetry) ==
		       -EOPNOTSUPP);
	}
	assert(dshot_update(&bus, -0.001f) == -EINVAL);
	assert(dshot_update(&bus, INFINITY) == -EINVAL);
	assert(dshot_update(NULL, 0.001f) == -EINVAL);
	dshot_del(&bus);
}

/* Commands: their repeats with the telemetry bit, the motor-stopped rule,
 * the hold of 0 after, and the other motors untouched. */
static void test_commands(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct esc_dev *a, *b;
	int start;

	config_for(&config, DSHOT_300, 2);
	assert(dshot_init(&bus, &config) == 0);
	a = dshot_esc(&bus, 0);
	b = dshot_esc(&bus, 1);
	esc_set_command(b, ESC_COMMAND_THROTTLE, 1.0f);

	/* Unknown numbers, and an esc_dev that is not a DShot motor */
	assert(dshot_command(a, (enum dshot_command)0) == -EINVAL);
	assert(dshot_command(a, (enum dshot_command)15) == -EINVAL);
	assert(dshot_command(a, (enum dshot_command)19) == -EINVAL);
	assert(dshot_command(a, (enum dshot_command)36) == -EINVAL);
	assert(dshot_command(a, (enum dshot_command)41) == -EINVAL);
	assert(dshot_command(a, (enum dshot_command)48) == -EINVAL);
	assert(dshot_command(NULL, DSHOT_CMD_BEEP1) == -EINVAL);
	{
		/* priv points somewhere real: the check is on ops */
		static struct dshot_motor decoy;
		struct esc_dev other = {
			.name = "other", .ops = &other_ops,
			.priv = &decoy,
		};

		assert(dshot_command(&other, DSHOT_CMD_BEEP1) == -EINVAL);
		assert(dshot_set_value(&other, 100, false) == -EINVAL);
		assert(dshot_request_telemetry(&other) == -EINVAL);
		assert(decoy.repeats_left == 0 && decoy.value == 0 &&
		       !decoy.telemetry_once);
	}

	/* 0-36 only with the motor stopped; 42-47 any time */
	assert(dshot_command(b, DSHOT_CMD_SPIN_DIRECTION_NORMAL) == -EBUSY);
	assert(dshot_command(b, DSHOT_CMD_LED0_ON) == -EBUSY);
	assert(dshot_command(b, DSHOT_CMD_SIGNAL_LINE_ERPM) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(1) == frame_of(46, true));
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(1) == frame_of(2047, false));

	/* Six frames of 21 with the bit set, then back to the value */
	start = sends;
	assert(dshot_command(a, DSHOT_CMD_SPIN_DIRECTION_REVERSED) == 0);
	assert(dshot_command(a, DSHOT_CMD_BEEP1) == -EBUSY);
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(sent_frame(0) == frame_of(21, true));
		assert(sent_frame(1) == frame_of(2047, false));
	}
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x0000);
	assert(sends - start == 7);
	/* no wait after it: a next command at once */
	assert(dshot_command(a, DSHOT_CMD_LED1_OFF) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == frame_of(27, true));

	/* BEEP1: one frame, then 0 held 260 ms, counted in dt_s */
	assert(dshot_command(a, DSHOT_CMD_BEEP1) == 0);
	assert(dshot_update(&bus, 0.1f) == 0);
	assert(sent_frame(0) == frame_of(1, true));
	esc_set_command(a, ESC_COMMAND_THROTTLE, 0.5f);	/* waits */
	assert(dshot_command(a, DSHOT_CMD_BEEP2) == -EBUSY);
	for (int i = 0; i < 3; i++) {			/* 0.26, .16, .06 */
		assert(dshot_update(&bus, 0.1f) == 0);
		assert(sent_frame(0) == 0x0000);
	}
	assert(dshot_update(&bus, 0.1f) == 0);
	assert(sent_frame(0) == frame_of(1048, false));
	esc_set_command(a, ESC_COMMAND_THROTTLE, 0.0f);

	/* BEEP3 holds 280 ms: still 0 at 270 */
	assert(dshot_command(a, DSHOT_CMD_BEEP3) == 0);
	dshot_update(&bus, 0.0f);
	esc_set_command(a, ESC_COMMAND_THROTTLE, 0.5f);
	assert(dshot_update(&bus, 0.27f) == 0);
	assert(sent_frame(0) == 0x0000);
	assert(dshot_update(&bus, 0.02f) == 0);
	assert(sent_frame(0) == 0x0000);
	assert(dshot_update(&bus, 0.0f) == 0);
	assert(sent_frame(0) == frame_of(1048, false));
	esc_set_command(a, ESC_COMMAND_THROTTLE, 0.0f);

	/* BEEP5 holds 1020 ms */
	assert(dshot_command(a, DSHOT_CMD_BEEP5) == 0);
	dshot_update(&bus, 0.0f);
	assert(dshot_update(&bus, 1.0f) == 0);
	assert(dshot_command(a, DSHOT_CMD_BEEP1) == -EBUSY);
	assert(dshot_update(&bus, 0.03f) == 0);
	assert(dshot_command(a, DSHOT_CMD_BEEP1) == 0);
	for (int i = 0; i < 4; i++)
		dshot_update(&bus, 0.1f);

	/* ESC_INFO: one frame, 12 ms */
	assert(dshot_command(a, DSHOT_CMD_ESC_INFO) == 0);
	assert(dshot_update(&bus, 0.0f) == 0);
	assert(sent_frame(0) == frame_of(6, true));
	assert(dshot_update(&bus, 0.011f) == 0);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == -EBUSY);
	assert(dshot_update(&bus, 0.002f) == 0);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == 0);
	dshot_update(&bus, 0.0f);

	/* SAVE_SETTINGS: six frames, then 35 ms */
	assert(dshot_command(a, DSHOT_CMD_SAVE_SETTINGS) == 0);
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.0f) == 0);
		assert(sent_frame(0) == frame_of(12, true));
	}
	assert(dshot_update(&bus, 0.034f) == 0);
	assert(sent_frame(0) == 0x0000);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == -EBUSY);
	assert(dshot_update(&bus, 0.002f) == 0);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == 0);
	dshot_update(&bus, 0.0f);

	/* EDT on: six frames of 13, 35 ms */
	assert(dshot_command(a, DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE) == 0);
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.0f) == 0);
		assert(sent_frame(0) == frame_of(13, true));
	}
	assert(dshot_update(&bus, 0.03f) == 0);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == -EBUSY);
	dshot_update(&bus, 0.01f);

	/* Every command with a meaning is taken; its frame goes out */
	{
		static const uint8_t six[] = { 7, 8, 9, 10, 12, 13, 14, 20,
					       21, 32, 33, 34, 35 };
		static const uint8_t once[] = { 1, 2, 3, 4, 5, 6, 11, 22,
						23, 24, 25, 26, 27, 28, 29,
						30, 31, 42, 43, 44, 45, 46,
						47 };

		for (size_t i = 0; i < sizeof(six); i++) {
			assert(dshot_command(a, six[i]) == 0);
			for (int n = 0; n < 6; n++) {
				dshot_update(&bus, 0.0f);
				assert(sent_frame(0) == frame_of(six[i], true));
			}
			dshot_update(&bus, 2.0f);
			assert(sent_frame(0) != frame_of(six[i], true));
			dshot_update(&bus, 2.0f);
		}
		for (size_t i = 0; i < sizeof(once); i++) {
			assert(dshot_command(a, once[i]) == 0);
			dshot_update(&bus, 0.0f);
			assert(sent_frame(0) == frame_of(once[i], true));
			dshot_update(&bus, 2.0f);
			assert(sent_frame(0) == 0x0000);
			dshot_update(&bus, 2.0f);
		}
	}
	dshot_del(&bus);
}

/* A frame still on the wire: nothing sent, nothing consumed. */
static void test_busy(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct esc_dev *a;
	int start;

	config_for(&config, DSHOT_150, 1);
	assert(dshot_init(&bus, &config) == 0);
	a = dshot_esc(&bus, 0);
	assert(dshot_command(a, DSHOT_CMD_3D_MODE_ON) == 0);
	start = sends;
	line_busy = true;
	assert(dshot_update(&bus, 0.001f) == -EBUSY);
	assert(dshot_update(&bus, 0.001f) == -EBUSY);
	line_busy = false;
	assert(sends == start);
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(sent_frame(0) == frame_of(10, true));
	}
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x0000);
	dshot_del(&bus);
}

/* A frame that did not go out counts no repeat, no wait, no one-shot bit. */
static void test_failed_send(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct esc_dev *a;

	config_for(&config, DSHOT_300, 1);
	assert(dshot_init(&bus, &config) == 0);
	a = dshot_esc(&bus, 0);

	assert(dshot_command(a, DSHOT_CMD_SPIN_DIRECTION_NORMAL) == 0);
	fail_send = true;
	assert(dshot_update(&bus, 0.001f) < 0);
	assert(dshot_update(&bus, 0.001f) < 0);
	fail_send = false;
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(sent_frame(0) == frame_of(20, true));
	}
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x0000);

	/* the wait: failed updates do not count their dt_s */
	assert(dshot_command(a, DSHOT_CMD_ESC_INFO) == 0);
	assert(dshot_update(&bus, 0.0f) == 0);		/* the command */
	fail_send = true;
	assert(dshot_update(&bus, 1.0f) < 0);
	fail_send = false;
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == -EBUSY);
	assert(dshot_update(&bus, 0.013f) == 0);
	assert(dshot_command(a, DSHOT_CMD_LED0_ON) == 0);
	dshot_update(&bus, 0.0f);

	/* the one-shot telemetry bit waits for a frame that goes */
	esc_set_command(a, ESC_COMMAND_THROTTLE, 1.0f);
	assert(dshot_request_telemetry(a) == 0);
	fail_send = true;
	assert(dshot_update(&bus, 0.001f) < 0);
	fail_send = false;
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == frame_of(2047, true));
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == frame_of(2047, false));
	dshot_del(&bus);
}

static void test_del(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	config_for(&config, DSHOT_600, 2);
	assert(dshot_init(&bus, &config) == 0);
	detached = false;
	dshot_del(&bus);
	assert(detached);
	assert(dshot_esc(&bus, 0) == NULL);
	assert(dshot_update(&bus, 0.001f) == -EINVAL);
	detached = false;
	dshot_del(&bus);		/* twice is harmless */
	dshot_del(NULL);
	assert(!detached);
}

/* ------------------------------------------------- bidirectional -- */

/*
 * Inverted CRCs out, eRPM back: read one update late, per motor, a
 * silent or garbled motor counted and its last value kept; RPM and the
 * esc_dev's velocity from it; online while replies keep coming.
 */
static void test_bidir_erpm(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct dshot_telemetry telemetry;
	struct esc_feedback fb;
	struct esc_dev *a, *b;
	uint32_t erpm, rpm;
	/* e 2, m 256: 1024 us, 58593.75 eRPM [BW "eRPM Telemetry Frame"] */
	const uint16_t turning = (2 << 9) | 256;

	bidir_config_for(&config, DSHOT_600, 2);
	config.poles = 14;
	assert(dshot_init(&bus, &config) == 0);
	a = dshot_esc(&bus, 0);
	b = dshot_esc(&bus, 1);

	/* [BW "Bidirectional DSHOT"]: 1046 with its CRC inverted */
	assert(dshot_set_value(a, 1046, false) == 0);
	assert(bidir_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x82C9);
	assert(sent_frame(1) == dshot_frame(0, false, true));
	/* no frame before this one: nothing to read, nothing counted */
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.has == 0 && !telemetry.replies && !telemetry.errors);
	assert(dshot_get_erpm(a, &erpm) == -ENODATA);
	assert(esc_get_feedback(a, &fb) == 0);
	assert(fb.has == 0 && !fb.online);

	/* motor 0 answers from the next frame, motor 1 stays silent */
	esc_answer[0] = answer_of(turning);
	assert(bidir_update(&bus, 0.001f) == 0);	/* sent, answered */
	assert(dshot_get_erpm(a, &erpm) == -ENODATA);
	assert(bidir_update(&bus, 0.001f) == 0);	/* read */
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 58594);
	assert(dshot_get_rpm(a, 14, &rpm) == 0 && rpm == 8371);
	assert(dshot_get_rpm(a, 12, &rpm) == 0 && rpm == 9766);
	assert(dshot_get_rpm(a, 0, &rpm) == -EINVAL);
	assert(dshot_get_rpm(a, 7, &rpm) == -EINVAL);
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.has == DSHOT_HAS_ERPM);
	assert(telemetry.replies == 1 && telemetry.errors == 1);
	assert(esc_get_feedback(a, &fb) == 0);
	assert(fb.has == ESC_HAS_VELOCITY && fb.online);
	assert(fabsf(fb.velocity_rad_s - 58594.0f / 7.0f * 6.2831853f /
		     60.0f) < 0.01f);
	assert(dshot_get_erpm(b, &erpm) == -ENODATA);
	assert(dshot_get_telemetry(b, &telemetry) == 0);
	assert(telemetry.errors == 2 && !telemetry.replies);
	assert(esc_get_feedback(b, &fb) == 0 && !fb.online);

	/* a CRC that does not match: counted, the last eRPM kept */
	esc_answer[0] = answer_of(turning) ^ 0x3;
	bidir_update(&bus, 0.001f);
	assert(bidir_update(&bus, 0.001f) == 0);
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 58594);
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.errors >= 2);

	/* 0x0FFF is a stopped motor [BF dshot.c:209-211] */
	esc_answer[0] = answer_of(DSHOT_REPLY_STOPPED);
	bidir_update(&bus, 0.001f);
	assert(bidir_update(&bus, 0.001f) == 0);
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 0);

	/* silent again: offline once DSHOT_REPLY_TIMEOUT_S has passed */
	esc_answer[0] = 0;
	assert(bidir_update(&bus, 0.06f) == 0);	/* the last good one */
	assert(esc_get_feedback(a, &fb) == 0 && fb.online);
	assert(bidir_update(&bus, 0.06f) == 0);
	assert(esc_get_feedback(a, &fb) == 0 && fb.online);
	assert(bidir_update(&bus, 0.06f) == 0);
	assert(esc_get_feedback(a, &fb) == 0 && !fb.online);
	assert(fb.has == ESC_HAS_VELOCITY);		/* the last value */

	/* no poles: no velocity */
	dshot_del(&bus);
	bidir_config_for(&config, DSHOT_300, 1);
	assert(dshot_init(&bus, &config) == 0);
	esc_answer[0] = answer_of(turning);
	bidir_update(&bus, 0.001f);
	bidir_update(&bus, 0.001f);
	assert(dshot_get_erpm(dshot_esc(&bus, 0), &erpm) == 0);
	assert(esc_get_feedback(dshot_esc(&bus, 0), &fb) == 0);
	assert(fb.has == 0 && fb.online);
	assert(dshot_get_erpm(NULL, &erpm) == -EINVAL);
	assert(dshot_get_erpm(dshot_esc(&bus, 0), NULL) == -EINVAL);
	assert(dshot_get_telemetry(dshot_esc(&bus, 0), NULL) == -EINVAL);
	dshot_del(&bus);
	esc_answer[0] = 0;
}

/* The answer the next update reads, after one frame to carry it. */
static void bidir_hear(struct dshot_bus *bus, uint16_t data)
{
	esc_answer[0] = answer_of(data);
	assert(bidir_update(bus, 0.001f) == 0);
	assert(bidir_update(bus, 0.001f) == 0);
	esc_answer[0] = 0;
}

/*
 * EDT [EDT "EDT Frames"]: only once command 13 has gone out; the version
 * answering it, then every type; eRPM frames still eRPM; 0xEFF turns it
 * off again.
 */
static void test_bidir_edt(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct dshot_telemetry telemetry;
	struct esc_feedback fb;
	struct esc_dev *a;
	uint32_t erpm;

	bidir_config_for(&config, DSHOT_600, 1);
	assert(dshot_init(&bus, &config) == 0);
	a = dshot_esc(&bus, 0);

	/* EDT off: 001 0 0001 1001 is eRPM, period 25 << 1 = 50 us */
	bidir_hear(&bus, 0x219);
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 1200000);
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.has == DSHOT_HAS_ERPM);

	/* Six frames of 13, inverted CRC, and EDT is on */
	assert(dshot_command(a, DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE) == 0);
	for (int i = 0; i < 6; i++) {
		assert(bidir_update(&bus, 0.0f) == 0);
		assert(sent_frame(0) == dshot_frame(13, true, true));
	}
	bidir_update(&bus, 0.05f);			/* the 35 ms wait */
	bidir_hear(&bus, 0xE05);			/* version 5 */
	bidir_hear(&bus, 0x219);			/* 25 deg C */
	bidir_hear(&bus, 0x430);			/* 48 * 0.25 V */
	bidir_hear(&bus, 0x60A);			/* 10 A */
	bidir_hear(&bus, 0x8FE);
	bidir_hear(&bus, 0xA01);
	bidir_hear(&bus, 0xC80);
	bidir_hear(&bus, 0xEA3);	/* alert, error, stress 3 */
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.has == (DSHOT_HAS_ERPM | DSHOT_HAS_TEMPERATURE |
				 DSHOT_HAS_VOLTAGE | DSHOT_HAS_CURRENT |
				 DSHOT_HAS_DEBUG1 | DSHOT_HAS_DEBUG2 |
				 DSHOT_HAS_STRESS | DSHOT_HAS_STATUS |
				 DSHOT_HAS_EDT_VERSION));
	assert(telemetry.edt_version == 5);
	assert(telemetry.temperature_c == 25);
	assert(telemetry.voltage_v == 12.0f);
	assert(telemetry.current_a == 10);
	assert(telemetry.debug1 == 0xFE && telemetry.debug2 == 0x01);
	assert(telemetry.stress == 0x80);
	assert(telemetry.status == (DSHOT_STATUS_ALERT | DSHOT_STATUS_ERROR |
				    3));
	assert(telemetry.erpm == 1200000);	/* untouched by EDT frames */
	assert(esc_get_feedback(a, &fb) == 0);
	assert(fb.has == ESC_HAS_TEMPERATURE && fb.temperature_c == 25.0f);

	/* eRPM frames are still eRPM: 001 1 0000 0000 is 512 us */
	bidir_hear(&bus, 0x300);
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 117188);
	bidir_hear(&bus, 0x0FF);			/* prefix 0: 255 us */
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 235294);

	/* 14 out, 0xEFF back: off; 001 0 is eRPM again */
	assert(dshot_command(a, DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE) == 0);
	for (int i = 0; i < 7; i++)
		bidir_update(&bus, 0.05f);
	bidir_hear(&bus, DSHOT_EDT_OFF_REPLY);
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.status == 0xA3);	/* not a status */
	bidir_hear(&bus, 0x219);
	assert(dshot_get_erpm(a, &erpm) == 0 && erpm == 1200000);
	assert(dshot_get_telemetry(a, &telemetry) == 0);
	assert(telemetry.temperature_c == 25);
	dshot_del(&bus);
}

static void run_dshot_cases(void)
{
	test_init_refused();
	test_throttle();
	test_commands();
	test_busy();
	test_failed_send();
	test_del();
	test_bidir_erpm();
	test_bidir_edt();
}
