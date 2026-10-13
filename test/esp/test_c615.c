/*
 * The C615 on ESP-IDF: c615.c through src/esc_port_esp.c against a fake
 * LEDC that keeps what the real one would put on the pin. The cases are
 * ../c615_cases.h; the LEDC settings themselves are checked here.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/ledc.h"

#include "c615.h"
#include "esc.h"

#define PIN		17

static ledc_timer_config_t timer;	/* the last timer config */
static ledc_channel_config_t channel;	/* the last channel config */
static bool running;
static uint32_t duty;			/* on the pin */
static uint32_t pending;		/* set, not yet updated */
static uint32_t idle_level = 9;
static bool fail_next;

static esp_err_t fail(void)
{
	fail_next = false;
	return ESP_FAIL;
}

esp_err_t ledc_timer_config(const ledc_timer_config_t *timer_conf)
{
	if (fail_next)
		return fail();
	timer = *timer_conf;
	return ESP_OK;
}

esp_err_t ledc_channel_config(const ledc_channel_config_t *ledc_conf)
{
	if (fail_next)
		return fail();
	channel = *ledc_conf;
	assert(channel.timer_sel == timer.timer_num);
	duty = pending = channel.duty;
	running = true;
	return ESP_OK;
}

esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t ch, uint32_t value)
{
	assert(mode == LEDC_LOW_SPEED_MODE && ch == channel.channel);
	if (fail_next)
		return fail();
	pending = value;
	return ESP_OK;
}

esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t ch)
{
	assert(mode == LEDC_LOW_SPEED_MODE && ch == channel.channel);
	duty = pending;
	return ESP_OK;
}

esp_err_t ledc_stop(ledc_mode_t mode, ledc_channel_t ch, uint32_t level)
{
	assert(mode == LEDC_LOW_SPEED_MODE && ch == channel.channel);
	running = false;
	idle_level = level;
	return ESP_OK;
}

static void config_at(struct c615_config *config, uint32_t rate_hz,
		      uint16_t min_us, uint16_t max_us)
{
	*config = (struct c615_config){
		.gpio = PIN, .timer = LEDC_TIMER_1,
		.channel = LEDC_CHANNEL_2, .rate_hz = rate_hz,
		.min_us = min_us, .max_us = max_us,
	};
}

static uint32_t out_us(void)
{
	return (uint32_t)lround((double)duty * 1e6 /
				((double)timer.freq_hz * 16384.0));
}

static bool out_running(void)
{
	return running;
}

#include "../c615_cases.h"

/* What the LEDC is told: low-speed mode, 14 bits, the rate, the pin. */
static void test_ledc(void)
{
	struct c615 c615;
	struct c615_config config;

	config_at(&config, 400, 1000, 2000);
	assert(c615_init(&c615, &config) == 0);
	assert(timer.speed_mode == LEDC_LOW_SPEED_MODE);
	assert(timer.duty_resolution == LEDC_TIMER_14_BIT);
	assert(timer.timer_num == LEDC_TIMER_1 && timer.freq_hz == 400);
	assert(timer.clk_cfg == LEDC_AUTO_CLK && !timer.deconfigure);
	assert(channel.gpio_num == PIN && channel.channel == LEDC_CHANNEL_2);
	assert(channel.speed_mode == LEDC_LOW_SPEED_MODE);
	assert(channel.intr_type == LEDC_INTR_DISABLE);
	assert(channel.hpoint == 0 && !channel.flags.output_invert);
	/* 1000 us of 2500: 0.4 * 16384 = 6553.6 -> 6554 */
	assert(duty == 6554);
	assert(c615_set_us(&c615, 2000) == 0 && duty == 13107);
	c615_del(&c615);
	assert(idle_level == 0);

	/* 50 Hz: 1500 us is 0.075 * 16384 = 1228.8 -> 1229 */
	config_at(&config, 50, 1000, 2000);
	assert(c615_init(&c615, &config) == 0);
	assert(c615_set_us(&c615, 1500) == 0 && duty == 1229);
	c615_del(&c615);

	/* A failed write keeps the pulse it had */
	config_at(&config, 400, 1000, 2000);
	assert(c615_init(&c615, &config) == 0);
	assert(c615_set_us(&c615, 1234) == 0);
	fail_next = true;
	assert(c615_set_us(&c615, 1800) == -EIO);
	assert(c615.pulse_us == 1234 && out_us() == 1234);
	c615_del(&c615);

	/* No rate at all, or no pin */
	config_at(&config, 0, 1000, 2000);
	assert(c615_init(&c615, &config) == -EINVAL);
	config_at(&config, 400, 1000, 2000);
	config.gpio = -1;
	assert(c615_init(&c615, &config) == -EINVAL);
}

int main(void)
{
	run_c615_cases();
	test_ledc();
	printf("test_c615 (esp): all passed\n");
	return 0;
}
