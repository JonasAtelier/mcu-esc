/*
 * The C615 on the STM32 HAL: c615.c through src/esc_port_stm.c against a
 * fake TIM whose compare registers are the pin. The cases are
 * ../c615_cases.h; the rate read back from ARR and the compares are checked
 * here.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "c615.h"
#include "esc.h"
#include "esc_priv.h"

#define TICK_HZ		1000000u	/* 1 us per count */

static TIM_TypeDef regs;
static TIM_HandleTypeDef tim = { .Instance = &regs };
static bool running;
static uint32_t started_channel;
static bool fail_next;

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t ch)
{
	assert(htim == &tim);
	if (fail_next) {
		fail_next = false;
		return HAL_ERROR;
	}
	started_channel = ch;
	running = true;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t ch)
{
	assert(htim == &tim && ch == started_channel);
	/* The port has pulled the compare to 0 before stopping. */
	assert(regs.CCR3 == 0);
	running = false;
	return HAL_OK;
}

static void config_at(struct c615_config *config, uint32_t rate_hz,
		      uint16_t min_us, uint16_t max_us)
{
	regs.ARR = TICK_HZ / rate_hz - 1;
	*config = (struct c615_config){
		.tim = &tim, .channel = TIM_CHANNEL_3, .tick_hz = TICK_HZ,
		.min_us = min_us, .max_us = max_us,
	};
}

static uint32_t out_us(void)
{
	return (uint32_t)((uint64_t)regs.CCR3 * 1000000u / TICK_HZ);
}

static bool out_running(void)
{
	return running;
}

#include "../c615_cases.h"

static void test_tim(void)
{
	struct c615 c615;
	struct c615_config config;

	/* 1 MHz, ARR 2499: 400 Hz, compare = us */
	config_at(&config, 400, 1000, 2000);
	assert(regs.ARR == 2499);
	assert(c615_init(&c615, &config) == 0);
	assert(c615.pwm.rate_hz == 400);
	assert(started_channel == TIM_CHANNEL_3 && regs.CCR3 == 1000);
	assert(regs.CCR1 == 0 && regs.CCR2 == 0 && regs.CCR4 == 0);
	assert(c615_set_us(&c615, 1777) == 0 && regs.CCR3 == 1777);
	c615_del(&c615);

	/* 84 MHz with no prescaler, ARR 209999: 400 Hz, 84 counts per us */
	regs.ARR = 209999;
	config.tick_hz = 84000000u;
	assert(c615_init(&c615, &config) == 0);
	assert(c615.pwm.rate_hz == 400 && regs.CCR3 == 84000);
	assert(c615_set_us(&c615, 1500) == 0 && regs.CCR3 == 126000);
	c615_del(&c615);

	/* A tick that does not divide: 1.5 MHz, ARR 3749, 400 Hz;
	 * 1001 us is 1501.5 counts -> 1502 */
	regs.ARR = 3749;
	config.tick_hz = 1500000u;
	assert(c615_init(&c615, &config) == 0);
	assert(c615_set_us(&c615, 1001) == 0 && regs.CCR3 == 1502);
	c615_del(&c615);

	/* Refused before the HAL: no handle, no tick, no such channel, a
	 * 32-bit ARR whose period wraps to 0 */
	config_at(&config, 400, 1000, 2000);
	config.tim = NULL;
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1000, 2000);
	config.tick_hz = 0;
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1000, 2000);
	config.channel = 0x10;
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1000, 2000);
	regs.ARR = 0xFFFFFFFFu;
	assert(c615_init(&c615, &config) == -EINVAL);
	assert(!running);
	{
		/* the port itself, at a tick where a wrapped period of 1
		 * would pass c615's 500 Hz check */
		struct esc_pwm pwm;

		assert(esc_pwm_attach(&pwm, &tim, TIM_CHANNEL_1, 400) ==
		       -EINVAL);
		regs.ARR = 0;
		assert(esc_pwm_attach(&pwm, &tim, TIM_CHANNEL_1, 400) == 0 &&
		       pwm.rate_hz == 400);
	}
}

int main(void)
{
	run_c615_cases();
	test_tim();
	printf("test_c615 (stm): all passed\n");
	return 0;
}
