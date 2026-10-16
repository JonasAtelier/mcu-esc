/*
 * Four bidirectional DShot600 ESCs on TIM8 CH1-CH4 of an STM32F405, the
 * Hawk flight controller's motor pins (PC6-PC9, AF3; docs/hardware.md),
 * with each motor's RPM and EDT temperature read back. Hawk's 10 k
 * pull-downs on those pins have to go first (docs/hardware.md, "DShot").
 *
 * A snippet, not a project: htim8 and its DMA are the app's, set up as for
 * examples/stm/dshot -- prescaler 0 on 168 MHz, ARR 279, PWM mode 1 with
 * preload on CH1-4, TIM8_UP on DMA2 stream 1 channel 7, memory to
 * peripheral, word to word, normal mode, its IRQ calling
 * HAL_DMA_IRQHandler() -- at a high priority, above anything that runs
 * long: its first interrupt after a frame has about 15 us to turn the
 * line round before the reply (docs/hardware.md). TIM8 or TIM1 only:
 * their update DMA is on DMA2, the one that reaches the GPIO. PC6-PC9 in
 * alternate function 3. The library turns the outputs active-low, the
 * pins' pulls up, and between frames borrows the stream to sample GPIOC;
 * nothing else may use that handle.
 *
 * Needs HAL_TIM_MODULE_ENABLED, HAL_DMA_MODULE_ENABLED and ESC_DSHOT = y.
 * Props off.
 */
#include <errno.h>
#include <stdio.h>

#include CMSIS_device_header

#include "dshot.h"
#include "esc.h"

#define MOTOR_POLES	14		/* count your motor's magnets */

extern TIM_HandleTypeDef htim8;		/* from MX_TIM8_Init() */

static struct dshot_bus bus;

int esc_example_init(void)
{
	const struct dshot_config cfg = {
		.speed = DSHOT_600,
		.bidirectional = true,
		.count = 4,
		.poles = MOTOR_POLES,
		.tim = &htim8,
		.tick_hz = 168000000u,
		.channel = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
			     TIM_CHANNEL_4 },
		.gpio = GPIOC,
		.pin = { GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9 },
	};
	int err = dshot_init(&bus, &cfg);

	if (err)
		printf("dshot_init: %d\n", err);
	return err;
}

/*
 * From the control loop, every period, no faster than a frame and its
 * reply take (about 115 us at DShot600): the four throttles, 0..1, and
 * the seconds since the last call.
 */
void esc_example_step(const float throttle[4], float dt_s)
{
	for (unsigned i = 0; i < 4; i++)
		esc_set_command(dshot_esc(&bus, i), ESC_COMMAND_THROTTLE,
				throttle[i]);
	if (dshot_update(&bus, dt_s) == -EBUSY)
		printf("dshot: loop faster than a frame and its reply\n");
}

/* Once, motors stopped: Extended DShot Telemetry on, for temperatures. */
void esc_example_edt_on(void)
{
	for (unsigned i = 0; i < 4; i++)
		dshot_command(dshot_esc(&bus, i),
			      DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE);
}

/* Each motor's speed and temperature, as the last replies said. */
void esc_example_report(void)
{
	for (unsigned i = 0; i < 4; i++) {
		struct dshot_telemetry telemetry;
		uint32_t rpm = 0;

		dshot_get_rpm(dshot_esc(&bus, i), MOTOR_POLES, &rpm);
		if (dshot_get_telemetry(dshot_esc(&bus, i), &telemetry))
			continue;
		printf("motor %u: %lu rpm, %u C, %lu/%lu bad\n", i + 1,
		       (unsigned long)rpm, telemetry.temperature_c,
		       (unsigned long)telemetry.errors,
		       (unsigned long)(telemetry.errors + telemetry.replies));
	}
}

void esc_example_stop(void)
{
	dshot_del(&bus);
}
