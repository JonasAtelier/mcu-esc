/*
 * Four DShot600 ESCs on TIM8 CH1-CH4 of an STM32F405, the Hawk flight
 * controller's motor pins (PC6-PC9, AF3; docs/hardware.md). Hawk ships with
 * DJI C615s, which take PWM only: this is for a DShot ESC swap.
 *
 * A snippet, not a project: htim8 and its DMA are the app's, as CubeMX
 * generates them --
 *
 *   TIM8: prescaler 0 on a 168 MHz timer clock (Hawk's: APB2 84 MHz, x2),
 *         ARR 279 -- 168 MHz / 600 kHz = 280 ticks, one bit; repetition
 *         counter 0, so every period raises an update; PWM mode 1 on CH1-4
 *         with output-compare preload; auto-reload preload on.
 *   DMA:  TIM8_UP is DMA2 stream 1, channel 7 -- the only stream it has
 *         (RM0090 Rev 3, Table 34 "DMA2 request mapping", p.217). Memory to
 *         peripheral,
 *         word to word, memory increment, normal mode, linked as
 *         __HAL_LINKDMA(&htim8, hdma[TIM_DMA_ID_UPDATE], hdma_tim8_up),
 *         its stream IRQ calling HAL_DMA_IRQHandler(&hdma_tim8_up).
 *
 * Needs HAL_TIM_MODULE_ENABLED, HAL_DMA_MODULE_ENABLED and ESC_DSHOT = y.
 * Props off.
 */
#include <errno.h>
#include <stdio.h>

#include CMSIS_device_header

#include "dshot.h"
#include "esc.h"

extern TIM_HandleTypeDef htim8;		/* from MX_TIM8_Init() */

static struct dshot_bus bus;

int esc_example_init(void)
{
	const struct dshot_config cfg = {
		.speed = DSHOT_600,
		.count = 4,
		.tim = &htim8,
		.tick_hz = 168000000u,
		.channel = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
			     TIM_CHANNEL_4 },
	};
	int err = dshot_init(&bus, &cfg);

	if (err)
		printf("dshot_init: %d\n", err);
	return err;
}

/*
 * From the control loop, every period: the mixer's four throttles, 0..1,
 * and the seconds since the last call. Send 0 for a while first so the
 * ESCs arm.
 */
void esc_example_step(const float throttle[4], float dt_s)
{
	for (unsigned i = 0; i < 4; i++)
		esc_set_command(dshot_esc(&bus, i), ESC_COMMAND_THROTTLE,
				throttle[i]);
	if (dshot_update(&bus, dt_s) == -EBUSY)
		printf("dshot: loop faster than a frame\n");
}

/* Once, motors stopped: beep motor 1 so you can tell which one it is. */
int esc_example_identify(void)
{
	return dshot_command(dshot_esc(&bus, 0), DSHOT_CMD_BEEP1);
}

void esc_example_stop(void)
{
	dshot_del(&bus);
}
