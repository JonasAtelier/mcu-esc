/*
 * The C615's cases, shared by esp/test_c615.c and stm/test_c615.c. Each
 * includes this after providing, over its own fake PWM:
 *
 *   config_at(config, rate_hz, min_us, max_us)   a valid config
 *   out_us()        the pulse the pin puts out now, to the nearest us
 *   out_running()   whether the pin is putting out pulses at all
 *   fail_next       the platform call that starts the output fails
 *
 * Page numbers are the C615 User Guide V1.0's: 400-2200 us and at most
 * 500 Hz on page 5.
 */

static void test_refused(void)
{
	struct c615 c615;
	struct c615_config config;

	/* Endpoints inside 400..2200 us and in order */
	config_at(&config, 400, 399, 2000);
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1000, 2201);
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1500, 1500);
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1600, 1500);
	assert(c615_init(&c615, &config) == -EINVAL);
	assert(c615_init(NULL, &config) == -EINVAL);
	assert(c615_init(&c615, NULL) == -EINVAL);

	/* At most 500 Hz */
	config_at(&config, 501, 1000, 1900);
	assert(c615_init(&c615, &config) == -EINVAL);

	/* At 500 Hz the period is 2000 us: 2200 does not fit, nor does 2000,
	 * which leaves the pin no low time; 1999 does */
	config_at(&config, 500, 1000, 2200);
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 500, 1000, 2000);
	assert(c615_init(&c615, &config) == -EINVAL);
	assert(!out_running());
	config_at(&config, 500, 1000, 1999);
	assert(c615_init(&c615, &config) == 0);
	assert(c615_set_us(&c615, 1999) == 0 && out_us() == 1999);
	c615_del(&c615);

	/* The platform refuses: nothing started */
	config_at(&config, 400, 1000, 2000);
	fail_next = true;
	assert(c615_init(&c615, &config) != 0);
	fail_next = false;
	assert(!out_running());
	assert(c615_esc(&c615) == NULL);

	/* The edges themselves are fine: 400..2200 at 50 Hz */
	config_at(&config, 50, 400, 2200);
	assert(c615_init(&c615, &config) == 0);
	assert(out_running() && out_us() == 400);
	assert(c615_set_us(&c615, 2200) == 0 && out_us() == 2200);
	c615_del(&c615);
	assert(!out_running());
}

static void test_throttle(void)
{
	struct c615 c615;
	struct c615_config config;
	struct esc_dev *esc;
	struct esc_feedback fb;

	config_at(&config, 400, 1000, 2000);
	assert(c615_init(&c615, &config) == 0);
	assert(out_running() && out_us() == 1000);
	esc = c615_esc(&c615);
	assert(esc && strcmp(esc->name, "c615") == 0);
	assert(esc->commands == ESC_COMMAND_BIT(ESC_COMMAND_THROTTLE));

	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.5f) == 0);
	assert(out_us() == 1500);
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 1.0f) == 0);
	assert(out_us() == 2000);
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.0f) == 0);
	assert(out_us() == 1000);
	/* clamped to 0..1 */
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 7.0f) == 0);
	assert(out_us() == 2000);
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, -0.5f) == 0);
	assert(out_us() == 1000);
	/* to the nearest microsecond: 0.4 us down, 0.6 us up */
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.0004f) == 0);
	assert(c615.pulse_us == 1000);
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.0006f) == 0);
	assert(c615.pulse_us == 1001);
	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, 0.2346f) == 0);
	assert(c615.pulse_us == 1235 && out_us() == 1235);

	assert(esc_set_command(esc, ESC_COMMAND_THROTTLE, NAN) == -EINVAL);
	assert(esc_set_command(esc, ESC_COMMAND_CURRENT, 1.0f) == -EOPNOTSUPP);
	assert(esc_set_command(esc, ESC_COMMAND_TORQUE, 1.0f) == -EOPNOTSUPP);
	assert(esc_get_feedback(esc, &fb) == -EOPNOTSUPP);

	/* The raw pulse, inside the endpoints only */
	assert(c615_set_us(&c615, 1234) == 0 && out_us() == 1234);
	assert(c615_set_us(&c615, 999) == -EINVAL);
	assert(c615_set_us(&c615, 2001) == -EINVAL);
	assert(out_us() == 1234 && c615.pulse_us == 1234);

	c615_del(&c615);
	assert(!out_running());
	assert(c615_esc(&c615) == NULL);
	assert(c615_set_us(&c615, 1500) == -EINVAL);
	c615_del(&c615);	/* twice is harmless */
	c615_del(NULL);
}

static void run_c615_cases(void)
{
	test_refused();
	test_throttle();
}
