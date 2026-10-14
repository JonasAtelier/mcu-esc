/*
 * DShot on the STM32 HAL: dshot.c through src/esc_port_stm.c against a
 * fake TIM and DMA that keep the burst buffer the port hands the HAL. The
 * cases are ../dshot_cases.h; the timer and DMA side is checked here.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "dshot.h"
#include "dshot_math.h"
#include "esc.h"

#define TICK_HZ		168000000u	/* TIM8 on a 168 MHz F405 */

static TIM_TypeDef regs;
static DMA_HandleTypeDef update_dma;
static TIM_HandleTypeDef tim = { .Instance = &regs };

static uint32_t burst[(16 + 1) * 4];	/* what the HAL was handed */
static uint32_t burst_base, burst_source, burst_length, data_length;
static int sends;
static bool line_busy;
static bool fail_send;			/* the burst start fails */
static bool detached;
static bool burst_running;		/* started and not stopped since */
static char calls[64];			/* S start, P write stop, B burst */
static unsigned started;		/* channels running, bit per CCR */
static int fail_start_at;		/* the n-th PWM start fails */
static int pwm_starts;
static uint8_t slot_of_motor[4];

static void called(char c)
{
	size_t n = strlen(calls);

	if (n + 1 < sizeof(calls)) {
		calls[n] = c;
		calls[n + 1] = '\0';
	}
}

static unsigned bit_of(uint32_t channel)
{
	return 1u << (channel / 4);
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t ch)
{
	assert(htim == &tim);
	if (++pwm_starts == fail_start_at)
		return HAL_ERROR;
	called('S');
	assert(!(started & bit_of(ch)));
	started |= bit_of(ch);
	return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t ch)
{
	assert(htim == &tim && (started & bit_of(ch)));
	started &= ~bit_of(ch);
	detached = true;
	return HAL_OK;
}

HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
	assert(hdma == &update_dma);
	return line_busy ? HAL_DMA_STATE_BUSY : HAL_DMA_STATE_READY;
}

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim,
					     uint32_t source)
{
	assert(htim == &tim && source == TIM_DMA_UPDATE);
	called('P');
	burst_running = false;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim,
						   uint32_t base,
						   uint32_t source,
						   const uint32_t *buffer,
						   uint32_t length,
						   uint32_t count)
{
	assert(htim == &tim);
	/* The HAL refuses a second start without a stop in between. */
	if (burst_running)
		return HAL_BUSY;
	if (fail_send)
		return HAL_ERROR;
	called('B');
	burst_running = true;
	burst_base = base;
	burst_source = source;
	burst_length = length;
	data_length = count;
	assert(count <= sizeof(burst) / sizeof(burst[0]));
	memcpy(burst, buffer, count * sizeof(burst[0]));
	/* The first update event has moved row 0 into the compares. */
	regs.CCR1 = burst[0];
	regs.CCR2 = burst[1];
	regs.CCR3 = burst[2];
	regs.CCR4 = burst[3];
	sends++;
	return HAL_OK;
}

/* ----------------------------------------------------- the cases' side -- */

static void dma_ready(void)
{
	update_dma = (DMA_HandleTypeDef){
		.Init = {
			.Direction = DMA_MEMORY_TO_PERIPH,
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_ENABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_WORD,
			.MemDataAlignment = DMA_MDATAALIGN_WORD,
			.Mode = DMA_NORMAL,
		},
		.State = HAL_DMA_STATE_READY,
	};
	tim.hdma[TIM_DMA_ID_UPDATE] = &update_dma;
}

/* Motors 0-3 on CH1-CH4; ARR for one bit of speed at 168 MHz. */
static void config_for(struct dshot_config *config, enum dshot_speed speed,
		       uint8_t count)
{
	static const uint32_t channel[DSHOT_MOTORS_MAX] = {
		TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
	};

	memset(config, 0, sizeof(*config));
	config->speed = speed;
	config->count = count;
	config->tim = &tim;
	config->tick_hz = TICK_HZ;
	memcpy(config->channel, channel, sizeof(channel));
	for (int i = 0; i < 4; i++)
		slot_of_motor[i] = (uint8_t)i;
	dma_ready();
	if (speed == DSHOT_150 || speed == DSHOT_300 || speed == DSHOT_600)
		regs.ARR = TICK_HZ / ((uint32_t)speed * 1000u) - 1u;
}

/* Bit b of a motor: its CCR in row b, high past half the bit a 1. */
static uint16_t sent_frame(int motor)
{
	uint32_t bit_ticks = regs.ARR + 1u;
	uint16_t frame = 0;

	assert(data_length == 17 * 4);
	for (int bit = 0; bit < 16; bit++) {
		uint32_t high = burst[bit * 4 + slot_of_motor[motor]];

		frame = (uint16_t)((frame << 1) | (high * 2 > bit_ticks));
	}
	return frame;
}

#include "../dshot_cases.h"

/* ------------------------------------------------ the timer and DMA side -- */

static void test_burst(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	/* 168 MHz: bit ticks and the 1 and 0 high times, [BW] table */
	static const struct { enum dshot_speed speed; uint32_t bit, one, zero; }
	rates[] = {
		{ DSHOT_150, 1120, 840, 420 },
		{ DSHOT_300, 560, 420, 210 },
		{ DSHOT_600, 280, 210, 105 },
	};

	for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
		config_for(&config, rates[r].speed, 4);
		assert(regs.ARR + 1 == rates[r].bit);
		regs.CCR1 = regs.CCR2 = regs.CCR3 = regs.CCR4 = 99;
		calls[0] = '\0';
		assert(dshot_init(&bus, &config) == 0);
		/* every channel started, compares 0 first: the lines low */
		assert(started == 0xF && !strcmp(calls, "SSSS"));
		assert(!regs.CCR1 && !regs.CCR2 && !regs.CCR3 && !regs.CCR4);

		assert(dshot_set_value(dshot_esc(&bus, 2), 2047, true) == 0);
		calls[0] = '\0';
		assert(dshot_update(&bus, 0.0f) == 0);
		/* the last burst's state cleared before the next starts */
		assert(!strcmp(calls, "PB"));
		assert(burst_base == TIM_DMABASE_CCR1);
		assert(burst_source == TIM_DMA_UPDATE);
		assert(burst_length == TIM_DMABURSTLENGTH_4TRANSFERS);
		assert(data_length == 17 * 4);
		for (int bit = 0; bit < 16; bit++)
			for (int s = 0; s < 4; s++)
				assert(burst[bit * 4 + s] == rates[r].one ||
				       burst[bit * 4 + s] == rates[r].zero);
		for (int s = 0; s < 4; s++)
			assert(burst[16 * 4 + s] == 0);	/* ends low */
		assert(burst[0 * 4 + 2] == rates[r].one);	/* 2047: 1s */
		assert(burst[0 * 4 + 0] == rates[r].zero);	/* 0: 0s */
		assert(sent_frame(2) == dshot_frame(2047, true, false));

		calls[0] = '\0';
		assert(dshot_update(&bus, 0.0f) == 0);
		assert(!strcmp(calls, "PB"));
		dshot_del(&bus);
		assert(started == 0 && !burst_running);
		assert(!regs.CCR1 && !regs.CCR2 && !regs.CCR3 && !regs.CCR4);
	}

	/* Motors on channels out of order, CH2 unused: each in its slot */
	config_for(&config, DSHOT_600, 3);
	config.channel[0] = TIM_CHANNEL_3;
	config.channel[1] = TIM_CHANNEL_1;
	config.channel[2] = TIM_CHANNEL_4;
	slot_of_motor[0] = 2;
	slot_of_motor[1] = 0;
	slot_of_motor[2] = 3;
	assert(dshot_init(&bus, &config) == 0);
	assert(started == 0xD);
	dshot_set_value(dshot_esc(&bus, 0), 1046, false);
	dshot_set_value(dshot_esc(&bus, 1), 48, true);
	dshot_set_value(dshot_esc(&bus, 2), 2047, false);
	assert(dshot_update(&bus, 0.0f) == 0);
	assert(sent_frame(0) == 0x82C6);	/* [BW]'s worked frame */
	assert(sent_frame(1) == dshot_frame(48, true, false));
	assert(sent_frame(2) == dshot_frame(2047, false, false));
	for (int bit = 0; bit < 17; bit++)
		assert(burst[bit * 4 + 1] == 0);	/* CH2 untouched */
	dshot_del(&bus);
}

static void test_refused(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	sends = 0;
	/* The timer and its DMA as the burst needs them */
	config_for(&config, DSHOT_600, 2);
	config.tim = NULL;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	config.tick_hz = 0;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	tim.hdma[TIM_DMA_ID_UPDATE] = NULL;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.Direction = 0;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.PeriphInc = DMA_PINC_ENABLE;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.MemInc = 0;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.MemDataAlignment = 0;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	update_dma.Init.Mode = DMA_CIRCULAR;
	assert(dshot_init(&bus, &config) == -EINVAL);

	/* Channels: no such one, or one twice */
	config_for(&config, DSHOT_600, 2);
	config.channel[1] = 0x10;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	config.channel[1] = TIM_CHANNEL_1;
	assert(dshot_init(&bus, &config) == -EINVAL);

	/* ARR one off the bit, either way, or for another speed */
	config_for(&config, DSHOT_600, 2);
	regs.ARR = 280;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	regs.ARR = 278;
	assert(dshot_init(&bus, &config) == -EINVAL);
	config_for(&config, DSHOT_600, 2);
	config.speed = DSHOT_300;
	assert(dshot_init(&bus, &config) == -EINVAL);
	assert(started == 0 && sends == 0);

	/* A tick that does not divide: 84 MHz at DShot300 is 280 ticks */
	config_for(&config, DSHOT_300, 1);
	config.tick_hz = 84000000u;
	regs.ARR = 279;
	assert(dshot_init(&bus, &config) == 0);
	dshot_del(&bus);
	/* 100 MHz at 600 kbit/s is 166.67: 167 */
	config_for(&config, DSHOT_600, 1);
	config.tick_hz = 100000000u;
	regs.ARR = 166;
	assert(dshot_init(&bus, &config) == 0);
	dshot_del(&bus);

	/* The third channel will not start: the first two stop again */
	config_for(&config, DSHOT_600, 4);
	pwm_starts = 0;
	fail_start_at = 3;
	assert(dshot_init(&bus, &config) == -EIO);
	assert(started == 0);
	fail_start_at = 0;
}

int main(void)
{
	run_dshot_cases();
	test_burst();
	test_refused();
	printf("test_dshot (stm): all passed\n");
	return 0;
}
