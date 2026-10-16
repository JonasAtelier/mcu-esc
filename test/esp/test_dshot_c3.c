/*
 * Bidirectional DShot on the ESP32-C3's RMT (built with STUB_RMT_C3): two
 * TX and two RX channels, so two motors; a third is -EOPNOTSUPP before
 * anything is set up. The fakes are the S3 suite's.
 */
#define main s3_suite_main
#include "test_dshot.c"
#undef main

int main(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	uint32_t erpm;

	bidir_config_for(&config, DSHOT_300, 3);
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);
	assert(live_channels == 0);
	bidir_config_for(&config, DSHOT_300, 2);
	assert(dshot_init(&bus, &config) == 0);
	assert(live_channels == 4);
	esc_answer[1] = answer_of(0x300);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(dshot_get_erpm(dshot_esc(&bus, 1), &erpm) == 0 &&
	       erpm == 117188);
	dshot_del(&bus);
	assert(live_channels == 0);
	printf("test_dshot_c3 (esp): all passed\n");
	return 0;
}
