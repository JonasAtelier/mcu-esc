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
#include "esc_priv.h"	/* esc_dshot_reply(), read directly below */

#define TICK_HZ		168000000u	/* TIM8 on a 168 MHz F405 */

static TIM_TypeDef regs;
static DMA_HandleTypeDef update_dma;
static TIM_HandleTypeDef tim = { .Instance = &regs };
static GPIO_TypeDef port_c;		/* PC6-PC9, Hawk's motor pins */

static uint32_t burst[(16 + 2) * 4];	/* what the DMA was handed */
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

/* bidirectional: the DMA started directly, the ESC's answers */
static uint32_t esc_answer[DSHOT_MOTORS_MAX];
static uint32_t answering[DSHOT_MOTORS_MAX];	/* those of the frame out */
static bool frame_pending, samples_pending;
static uint32_t dma_source, dma_destination, dma_length;
static int dma_inits, dma_aborts;
static bool fail_dma_start, fail_dma_init;

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
	assert(hdma == &update_dma && hdma->State != HAL_DMA_STATE_BUSY);
	dma_inits++;
	if (fail_dma_init)
		return HAL_ERROR;
	hdma->State = HAL_DMA_STATE_READY;
	return HAL_OK;
}

/*
 * A frame (memory to the timer's DMAR) or the samples (the port's IDR to
 * memory). Neither finishes here: settle() is the hardware.
 */
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma,
				   uint32_t SrcAddress, uint32_t DstAddress,
				   uint32_t DataLength)
{
	struct dshot_port *port = hdma->Parent;

	assert(hdma == &update_dma && port);
	if (fail_dma_start)
		return HAL_ERROR;
	if (hdma->State != HAL_DMA_STATE_READY)
		return HAL_BUSY;
	hdma->State = HAL_DMA_STATE_BUSY;
	dma_source = SrcAddress;
	dma_destination = DstAddress;
	dma_length = DataLength;
	if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH) {
		assert(SrcAddress == (uint32_t)(uintptr_t)port->burst);
		assert(DstAddress == (uint32_t)(uintptr_t)&regs.DMAR);
		assert(DataLength <= sizeof(burst) / sizeof(burst[0]));
		memcpy(burst, port->burst, DataLength * sizeof(burst[0]));
		data_length = DataLength;
		memcpy(answering, esc_answer, sizeof(answering));
		frame_pending = true;
		sends++;
	} else {
		assert(SrcAddress == (uint32_t)(uintptr_t)&port_c.IDR);
		assert(DstAddress == (uint32_t)(uintptr_t)port->samples);
		samples_pending = true;
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
	assert(hdma == &update_dma);
	dma_aborts++;
	frame_pending = samples_pending = false;
	hdma->State = HAL_DMA_STATE_READY;
	return HAL_OK;
}

/* The last row moved: the DMA's transfer-complete interrupt. */
static void frame_out(void)
{
	assert(frame_pending && (regs.DIER & TIM_DMA_UPDATE));
	frame_pending = false;
	update_dma.State = HAL_DMA_STATE_READY;
	update_dma.XferCpltCallback(&update_dma);
}

/*
 * The samples, each one sample period after the last from the frame's
 * end: every pin high but where its ESC answers -- 30 us after the frame
 * [EDT "Timing"], 21 bits at 5/4 of the bit rate [BW] -- then the
 * interrupt.
 */
static void samples_in(void)
{
	struct dshot_port *port = update_dma.Parent;
	uint32_t sample = regs.ARR + 1u;
	uint32_t start = 30u * (TICK_HZ / 1000000u);
	uint32_t bit = port->bit_ticks * 4u / 5u;

	assert(samples_pending && (regs.DIER & TIM_DMA_UPDATE));
	for (uint32_t n = 0; n < dma_length; n++) {
		uint32_t t = (n + 1u) * sample;
		uint16_t idr = 0xFFFF;

		for (int i = 0; i < 4; i++) {
			uint32_t b = t >= start ? (t - start) / bit : 0;

			if (answering[i] && t >= start && b < 21 &&
			    !(answering[i] & (1u << (20 - b))))
				idr &= (uint16_t)~(1u << (6 + i));
		}
		port->samples[n] = idr;
	}
	samples_pending = false;
	update_dma.State = HAL_DMA_STATE_READY;
	update_dma.XferCpltCallback(&update_dma);
}

static void settle(void)
{
	if (frame_pending)
		frame_out();
	if (samples_pending)
		samples_in();
}

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
		.Instance = DMA2_Stream1,	/* TIM8_UP */
		.Init = {
			.Direction = DMA_MEMORY_TO_PERIPH,
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_ENABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_WORD,
			.MemDataAlignment = DMA_MDATAALIGN_WORD,
			.Mode = DMA_NORMAL,
		},
		.State = HAL_DMA_STATE_READY,
		.Parent = &tim,		/* __HAL_LINKDMA() */
	};
	tim.hdma[TIM_DMA_ID_UPDATE] = &update_dma;
	frame_pending = samples_pending = false;
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
	regs.CCER = regs.DIER = regs.DCR = 0;
}

/* The same on PC6-PC9, alternate function, no pull. */
static void bidir_config_for(struct dshot_config *config,
			     enum dshot_speed speed, uint8_t count)
{
	static const uint16_t pins[DSHOT_MOTORS_MAX] = {
		GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9,
	};

	config_for(config, speed, count);
	config->bidirectional = true;
	config->gpio = &port_c;
	memcpy(config->pin, pins, sizeof(pins));
	port_c.MODER = 0xAAu << 12;
	port_c.PUPDR = 0;
}

/* Bit b of a motor: its CCR in row b, high past half the bit a 1. */
static uint16_t sent_frame(int motor)
{
	uint32_t bit_ticks = regs.ARR + 1u;
	uint16_t frame = 0;

	assert(data_length == 17 * 4 || data_length == 18 * 4);
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

/*
 * Bidirectional on TIM8: outputs active-low, pins pulled up; a frame of 18
 * rows started on the DMA itself, DCR as the HAL's burst; once out, pins
 * to inputs, the timer to 1/3 of a reply bit, the DMA reading PC's IDR
 * half-word by half-word; then everything back for the next frame.
 */
static void test_bidir_timer(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	/* 168 MHz: sample period round(bit * 4/5 / 3), and the samples
	 * covering 45 us and 22 reply bits */
	static const struct { enum dshot_speed speed; uint32_t sample, count; }
	rates[] = {
		{ DSHOT_300, 149, 117 },	/* (7560 + 9856) / 149 */
		{ DSHOT_600, 75, 167 },		/* (7560 + 4928) / 75 */
	};

	for (size_t r = 0; r < 2; r++) {
		uint32_t bit_arr;

		bidir_config_for(&config, rates[r].speed, 4);
		bit_arr = regs.ARR;
		calls[0] = '\0';
		dma_inits = 0;
		assert(dshot_init(&bus, &config) == 0);
		assert(regs.CCER == 0x2222);	/* CC1P-CC4P */
		assert(port_c.PUPDR == 0x55u << 12);
		assert(update_dma.Parent == &bus.port);
		assert(started == 0xF && !strcmp(calls, "SSSS"));
		assert(bus.port.sample_ticks == rates[r].sample);
		assert(bus.port.sample_count == rates[r].count);

		assert(dshot_set_value(dshot_esc(&bus, 1), 1046, false) == 0);
		calls[0] = '\0';
		assert(dshot_update(&bus, 0.0f) == 0);
		assert(!strcmp(calls, ""));	/* not the HAL's burst */
		assert(regs.DCR == (TIM_DMABASE_CCR1 |
				    TIM_DMABURSTLENGTH_4TRANSFERS));
		assert(dma_length == 18 * 4 && (regs.DIER & TIM_DMA_UPDATE));
		assert(!update_dma.XferHalfCpltCallback);
		for (int s = 0; s < 8; s++)
			assert(burst[16 * 4 + s] == 0);	/* rests twice */
		assert(sent_frame(1) == 0x82C9);
		/* the frame is still out: busy */
		assert(dshot_update(&bus, 0.0f) == -EBUSY);

		frame_out();
		assert((port_c.MODER >> 12 & 0xFF) == 0);	/* inputs */
		assert(regs.ARR == rates[r].sample - 1 && regs.EGR == TIM_EGR_UG);
		assert(update_dma.Init.Direction == DMA_PERIPH_TO_MEMORY);
		assert(update_dma.Init.PeriphDataAlignment ==
		       DMA_PDATAALIGN_HALFWORD);
		assert(update_dma.Init.MemDataAlignment ==
		       DMA_MDATAALIGN_HALFWORD);
		assert(dma_length == rates[r].count);
		assert(regs.DIER & TIM_DMA_UPDATE);
		assert(dshot_update(&bus, 0.0f) == -EBUSY);

		samples_in();
		assert((port_c.MODER >> 12 & 0xFF) == 0xAA);	/* AF again */
		assert(regs.ARR == bit_arr && !(regs.DIER & TIM_DMA_UPDATE));
		assert(update_dma.Init.Direction == DMA_MEMORY_TO_PERIPH);
		assert(update_dma.Init.PeriphDataAlignment ==
		       DMA_PDATAALIGN_WORD);
		assert(update_dma.Init.MemDataAlignment == DMA_MDATAALIGN_WORD);
		assert(dma_inits == 2);
		assert(dshot_update(&bus, 0.0f) == 0);

		/* del: the timer's settings and the handle as the app had */
		settle();
		dshot_del(&bus);
		assert(regs.CCER == 0 && update_dma.Parent == &tim);
		assert(!update_dma.XferCpltCallback && started == 0);
		assert((port_c.MODER >> 12 & 0xFF) == 0xAA);
	}
}

/* The samples as line bits; a DMA that fails: no reply, counted. */
static void test_bidir_samples(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	struct dshot_telemetry telemetry;
	uint32_t raw;

	bidir_config_for(&config, DSHOT_600, 3);
	config.pin[0] = GPIO_PIN_8;	/* motors on PC8, PC6, PC7 */
	config.pin[1] = GPIO_PIN_6;
	config.pin[2] = GPIO_PIN_7;
	assert(dshot_init(&bus, &config) == 0);
	assert(esc_dshot_reply(&bus.port, 0, &raw) == -ENOENT);
	esc_answer[1] = answer_of(0x82C);	/* answers on PC7 */
	assert(dshot_update(&bus, 0.0f) == 0);
	settle();
	assert(esc_dshot_reply(&bus.port, 0, &raw) == -ENODATA);
	/* the fake's ESC i answers on PC(6 + i): PC7, motor 2's pin */
	assert(esc_dshot_reply(&bus.port, 2, &raw) == 0);
	assert(raw == answer_of(0x82C));
	assert(esc_dshot_reply(&bus.port, 1, &raw) == -ENODATA);
	assert(esc_dshot_reply(&bus.port, 2, &raw) == -ENOENT);
	esc_answer[1] = 0;

	/* the sampling DMA will not start: back at rest, no reply */
	assert(dshot_update(&bus, 0.0f) == 0);
	fail_dma_start = true;
	frame_out();
	fail_dma_start = false;
	assert((port_c.MODER >> 12 & 0x3F) == 0x2A && !samples_pending);
	assert(dshot_update(&bus, 0.0f) == 0);	/* reads -ENODATA */
	assert(dshot_get_telemetry(dshot_esc(&bus, 0), &telemetry) == 0);
	assert(telemetry.errors == 1);

	/* a transfer error while sampling: the same */
	frame_out();
	update_dma.State = HAL_DMA_STATE_READY;
	samples_pending = false;
	update_dma.ErrorCode = HAL_DMA_ERROR_TE;
	update_dma.XferErrorCallback(&update_dma);
	update_dma.ErrorCode = HAL_DMA_ERROR_NONE;
	assert(regs.ARR == 279 && !(regs.DIER & TIM_DMA_UPDATE));
	assert(dshot_update(&bus, 0.0f) == 0);
	assert(dshot_get_telemetry(dshot_esc(&bus, 0), &telemetry) == 0);
	assert(telemetry.errors == 2);

	/*
	 * Mid-frame: a direct-mode or FIFO error leaves the stream running
	 * (HAL_DMA_IRQHandler()), so nothing changes; a transfer error stops
	 * it with a bit's compare loaded: the compares go 0, the line rests.
	 */
	settle();
	assert(dshot_update(&bus, 0.0f) == 0);
	regs.CCR1 = 210;
	regs.CCR2 = 105;
	dma_inits = 0;
	update_dma.ErrorCode = HAL_DMA_ERROR_DME | HAL_DMA_ERROR_FE;
	update_dma.XferErrorCallback(&update_dma);
	assert(dma_inits == 0 && regs.CCR1 == 210 && regs.CCR2 == 105);
	assert(dshot_update(&bus, 0.0f) == -EBUSY);
	update_dma.ErrorCode = HAL_DMA_ERROR_TE | HAL_DMA_ERROR_DME;
	frame_pending = false;
	update_dma.State = HAL_DMA_STATE_READY;
	update_dma.XferErrorCallback(&update_dma);
	update_dma.ErrorCode = HAL_DMA_ERROR_NONE;
	assert(dma_inits == 1);
	assert(!regs.CCR1 && !regs.CCR2 && !regs.CCR3 && !regs.CCR4);
	assert(regs.ARR == 279 && (port_c.MODER >> 12 & 0x3F) == 0x2A);
	assert(dshot_update(&bus, 0.0f) == 0);	/* reads -ENODATA */

	/* the frame's DMA will not start: an error, and not busy after */
	settle();
	fail_dma_start = true;
	assert(dshot_update(&bus, 0.0f) == -EIO);
	fail_dma_start = false;
	assert(dshot_update(&bus, 0.0f) == 0);
	settle();
	dshot_del(&bus);
}

static void test_bidir_refused(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	bidir_config_for(&config, DSHOT_600, 2);
	config.gpio = NULL;
	assert(dshot_init(&bus, &config) == -EINVAL);
	bidir_config_for(&config, DSHOT_600, 2);
	config.pin[1] = 0;
	assert(dshot_init(&bus, &config) == -EINVAL);
	bidir_config_for(&config, DSHOT_600, 2);
	config.pin[1] = GPIO_PIN_6 | GPIO_PIN_7;
	assert(dshot_init(&bus, &config) == -EINVAL);
	bidir_config_for(&config, DSHOT_600, 2);
	config.pin[1] = GPIO_PIN_6;
	assert(dshot_init(&bus, &config) == -EINVAL);
	assert(started == 0 && regs.CCER == 0 && port_c.PUPDR == 0);
	assert(update_dma.Parent == &tim);

	/* an update DMA on DMA1, as TIM2-5 have: it cannot read the GPIO */
	bidir_config_for(&config, DSHOT_600, 2);
	update_dma.Instance = DMA1_Stream5;
	assert(dshot_init(&bus, &config) == -EOPNOTSUPP);
	update_dma.Instance = DMA2_Stream0;
	assert(dshot_init(&bus, &config) == 0);	/* the lowest DMA2 stream */
	dshot_del(&bus);
	config_for(&config, DSHOT_600, 2);	/* plain DShot: any DMA */
	update_dma.Instance = DMA1_Stream5;
	assert(dshot_init(&bus, &config) == 0);
	dshot_del(&bus);

	/* the third channel will not start: all undone */
	bidir_config_for(&config, DSHOT_600, 4);
	pwm_starts = 0;
	fail_start_at = 3;
	assert(dshot_init(&bus, &config) == -EIO);
	fail_start_at = 0;
	assert(started == 0 && regs.CCER == 0 && update_dma.Parent == &tim);
}

int main(void)
{
	run_dshot_cases();
	test_burst();
	test_refused();
	test_bidir_timer();
	test_bidir_samples();
	test_bidir_refused();
	printf("test_dshot (stm): all passed\n");
	return 0;
}
