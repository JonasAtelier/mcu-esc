/*
 * Four C615s on TIM8 channels 1-4, as on the Hawk flight controller
 * (hardware/hawk/control: PC6-PC9, 400 Hz).
 *
 * A snippet, not a project: htim8 is the app's, as CubeMX generates it in
 * PWM mode 1 on all four channels, with the prescaler set for a 1 MHz
 * counter and ARR 2499 -- 400 Hz, 1 us per count -- and output-compare
 * preload on each channel (CCMRx OCxPE), so a new width waits for the next
 * period instead of cutting the pulse in flight. Needs
 * HAL_TIM_MODULE_ENABLED, stm32f4xx_hal_tim.c in the build and
 * ESC_C615 = y. Props off: MIN_US is the calibrated bottom endpoint, and
 * the C615 User Guide does not say that it stops the motor.
 */
#include <stdio.h>

#include CMSIS_device_header

#include "c615.h"
#include "esc.h"

#define MOTORS		4
#define MIN_US		1000
#define MAX_US		2000

extern TIM_HandleTypeDef htim8;		/* from MX_TIM8_Init() */

static struct c615 motor[MOTORS];

void esc_example_init(void)
{
	static const uint32_t channel[MOTORS] = {
		TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
	};

	for (int i = 0; i < MOTORS; i++) {
		struct c615_config cfg = {
			.tim = &htim8,
			.channel = channel[i],
			.tick_hz = 1000000,
			.min_us = MIN_US,
			.max_us = MAX_US,
		};
		int err = c615_init(&motor[i], &cfg);

		if (err)
			printf("c615 %d: %d\n", i + 1, err);
	}
}

/* Each motor's throttle, 0..1, from the mixer. */
void esc_example_mix(const float throttle[MOTORS])
{
	for (int i = 0; i < MOTORS; i++)
		esc_set_command(c615_esc(&motor[i]), ESC_COMMAND_THROTTLE,
				throttle[i]);
}

/* Shut down: no pulses at all, which the ESC reads as no signal. */
void esc_example_stop(void)
{
	for (int i = 0; i < MOTORS; i++)
		c615_del(&motor[i]);
}
