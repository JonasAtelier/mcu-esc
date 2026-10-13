/*
 * The core's cases -- the command and feedback checks, the log line --
 * shared by esp/test_core.c and stm/test_core.c. Each includes this after
 * providing:
 *
 *   LOG_PREFIX             what the platform's log puts before "esc: "
 *   stub_log_last          the last log line
 */

static int fake_calls;
static int fake_ret;			/* fail the next op with this */
static enum esc_command fake_kind;
static float fake_value;

static int fake_set_command(struct esc_dev *esc, enum esc_command kind,
			    float value)
{
	(void)esc;
	fake_calls++;
	fake_kind = kind;
	fake_value = value;
	return fake_ret;
}

static int fake_get_feedback(struct esc_dev *esc,
			     struct esc_feedback *feedback)
{
	(void)esc;
	fake_calls++;
	/* The core hands the op a zeroed struct. */
	assert(feedback->has == 0 && feedback->position_rad == 0.0f &&
	       !feedback->online);
	feedback->has = ESC_HAS_POSITION | ESC_HAS_TEMPERATURE;
	feedback->position_rad = 1.5f;
	feedback->temperature_c = 41.0f;
	feedback->online = true;
	return fake_ret;
}

static const struct esc_ops fake_ops = {
	.set_command = fake_set_command,
	.get_feedback = fake_get_feedback,
};

static const struct esc_ops command_only_ops = {
	.set_command = fake_set_command,
};

static void test_set_command(void)
{
	struct esc_dev esc = {
		.name = "fake", .ops = &fake_ops,
		.commands = ESC_COMMAND_BIT(ESC_COMMAND_CURRENT) |
			    ESC_COMMAND_BIT(ESC_COMMAND_THROTTLE),
	};
	struct esc_dev unbound = { .name = "unbound" };

	fake_calls = 0;
	assert(esc_set_command(&esc, ESC_COMMAND_CURRENT, -2.5f) == 0);
	assert(fake_calls == 1 && fake_kind == ESC_COMMAND_CURRENT &&
	       fake_value == -2.5f);
	assert(esc_set_command(&esc, ESC_COMMAND_THROTTLE, 0.25f) == 0);
	assert(fake_calls == 2 && fake_kind == ESC_COMMAND_THROTTLE);

	/* Refused before the op: a kind it lacks, an unknown kind, no number */
	assert(esc_set_command(&esc, ESC_COMMAND_TORQUE, 1.0f) == -EOPNOTSUPP);
	assert(esc_set_command(&esc, (enum esc_command)3, 1.0f) == -EINVAL);
	assert(esc_set_command(&esc, (enum esc_command)-1, 1.0f) == -EINVAL);
	assert(esc_set_command(&esc, ESC_COMMAND_CURRENT, NAN) == -EINVAL);
	assert(esc_set_command(&esc, ESC_COMMAND_CURRENT, INFINITY) == -EINVAL);
	assert(esc_set_command(&esc, ESC_COMMAND_CURRENT, -INFINITY) ==
	       -EINVAL);
	assert(esc_set_command(NULL, ESC_COMMAND_CURRENT, 1.0f) == -EINVAL);
	assert(esc_set_command(&unbound, ESC_COMMAND_CURRENT, 1.0f) ==
	       -EINVAL);
	assert(fake_calls == 2);

	/* The op's error comes back as it was */
	fake_ret = -EIO;
	assert(esc_set_command(&esc, ESC_COMMAND_CURRENT, 1.0f) == -EIO);
	fake_ret = 0;
}

static void test_get_feedback(void)
{
	struct esc_dev esc = { .name = "fake", .ops = &fake_ops };
	struct esc_dev pwm = { .name = "pwm", .ops = &command_only_ops };
	struct esc_feedback fb;

	fake_calls = 0;
	memset(&fb, 0xA5, sizeof(fb));
	assert(esc_get_feedback(&esc, &fb) == 0);
	assert(fake_calls == 1);
	assert(fb.has == (ESC_HAS_POSITION | ESC_HAS_TEMPERATURE));
	assert(fb.position_rad == 1.5f && fb.temperature_c == 41.0f);
	/* what the op did not set reads 0, not what *fb held */
	assert(fb.angle_rad == 0.0f && fb.velocity_rad_s == 0.0f &&
	       fb.torque_nm == 0.0f && fb.online);

	/* Untouched on failure */
	fake_ret = -ETIMEDOUT;
	memset(&fb, 0, sizeof(fb));
	assert(esc_get_feedback(&esc, &fb) == -ETIMEDOUT);
	assert(fb.has == 0 && fb.position_rad == 0.0f && !fb.online);
	fake_ret = 0;

	assert(esc_get_feedback(&pwm, &fb) == -EOPNOTSUPP);
	assert(esc_get_feedback(&esc, NULL) == -EINVAL);
	assert(esc_get_feedback(NULL, &fb) == -EINVAL);
}

static void test_log(void)
{
	struct esc_dev esc = { .name = "fake", .ops = &fake_ops };
	struct esc_feedback fb = {
		.has = ESC_HAS_ANGLE | ESC_HAS_POSITION | ESC_HAS_VELOCITY |
		       ESC_HAS_TORQUE | ESC_HAS_TEMPERATURE,
		.angle_rad = 0.5f, .position_rad = -12.25f,
		.velocity_rad_s = 3.0f, .torque_nm = -0.5f,
		.temperature_c = 40.0f, .online = true,
	};

	esc_log_feedback(&esc, &fb);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "esc: fake: online angle 0.500 position -12.250 "
		       "velocity 3.00 torque -0.50 temperature 40"));
	fb.has = 0;
	fb.online = false;
	esc_log_feedback(&esc, &fb);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I") "esc: fake: offline"));
	esc_err(&esc, "no answer (%d)", -ETIMEDOUT);
	assert(!strcmp(stub_log_last,
		       LOG_PREFIX("E") "esc: fake: no answer (-110)"));
}

static void run_core_cases(void)
{
	test_set_command();
	test_get_feedback();
	test_log();
}
