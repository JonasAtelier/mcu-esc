/*
 * DShot on an STM32 other than the F4 (built with STUB_NOT_F4): plain
 * DShot as on the F4, bidirectional -EOPNOTSUPP with nothing touched. The
 * fakes are the F4 suite's.
 */
#define main f4_suite_main
#include "test_dshot.c"
#undef main

int main(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	bidir_config_for(&config, DSHOT_600, 4);
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);
	assert(started == 0 && regs.CCER == 0 && port_c.PUPDR == 0);
	assert(update_dma.Parent == &tim);

	config_for(&config, DSHOT_600, 4);
	assert(dshot_init(&bus, &config) == 0);
	assert(dshot_set_value(dshot_esc(&bus, 0), 1046, false) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x82C6);
	dshot_del(&bus);
	assert(started == 0);
	printf("test_dshot_nof4 (stm): all passed\n");
	return 0;
}
