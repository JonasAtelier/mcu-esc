/*
 * DShot on the original ESP32, whose RMT has no TX sync (built with
 * STUB_RMT_NO_TX_SYNCHRO): one motor works, more is -EOPNOTSUPP before
 * anything is set up. The fakes are the S3 suite's.
 */
#define main s3_suite_main
#include "test_dshot.c"
#undef main

int main(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	config_for(&config, DSHOT_600, 2);
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);
	assert(live_channels == 0);
	config_for(&config, DSHOT_600, 4);
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);

	config_for(&config, DSHOT_600, 1);
	assert(dshot_init(&bus, &config) == 0);
	assert(!sync_manager.live && live_channels == 1);
	assert(dshot_set_value(dshot_esc(&bus, 0), 1046, false) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x82C6);
	dshot_del(&bus);
	assert(live_channels == 0);
	printf("test_dshot_nosync (esp): all passed\n");
	return 0;
}
