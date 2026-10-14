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
 * Frames are checked against dshot_frame(), which test_dshot_math holds to
 * the sources' worked examples.
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
	config_for(&config, DSHOT_300, 1);
	config.bidirectional = true;
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);
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

static void run_dshot_cases(void)
{
	test_init_refused();
	test_throttle();
	test_commands();
	test_busy();
	test_failed_send();
	test_del();
}
