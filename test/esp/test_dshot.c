/*
 * DShot on ESP-IDF: dshot.c through src/esc_port_esp.c against a fake RMT
 * TX driver that keeps every channel's settings and the symbols it was
 * last asked to send. The cases are ../dshot_cases.h; the RMT side is
 * checked here.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/rmt_tx.h"

#include "dshot.h"
#include "dshot_math.h"
#include "esc.h"

static struct rmt_channel_t {
	bool live, enabled;
	bool queued;		/* a frame waits for the sync manager */
	int disables;
	rmt_tx_channel_config_t config;
	rmt_symbol_word_t symbols[16];
	size_t bytes;
	rmt_transmit_config_t transmit;
} channels[8];

static struct rmt_encoder_t {
	bool live;
} encoders[8];

static struct rmt_sync_manager_t {
	bool live;
	size_t size;
	rmt_channel_handle_t members[4];
	int resets;
} sync_manager;

static int sends;		/* updates that sent, all channels */
static int transmits;		/* rmt_transmit() calls */
static bool line_busy;
static bool fail_send;		/* rmt_transmit() on motor 0 fails */
static int fail_transmit_at = -1;	/* ... or on this motor */
static bool detached;		/* a channel was deleted */
static int fail_at;		/* the fail_at-th setup call fails; 0 never */
static int setup_calls;
static int live_channels, live_encoders;
static rmt_channel_handle_t order[4];	/* channels in creation order */
static int created;

static esp_err_t setup_call(void)
{
	return ++setup_calls == fail_at ? ESP_FAIL : ESP_OK;
}

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
			     rmt_channel_handle_t *ret_chan)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	for (int i = 0; i < 8; i++) {
		if (!channels[i].live) {
			memset(&channels[i], 0, sizeof(channels[i]));
			channels[i].live = true;
			channels[i].config = *config;
			*ret_chan = &channels[i];
			live_channels++;
			if (created < 4)
				order[created] = &channels[i];
			created++;
			return ESP_OK;
		}
	}
	return ESP_ERR_NOT_FOUND;
}

esp_err_t rmt_del_channel(rmt_channel_handle_t channel)
{
	assert(channel->live && !channel->enabled);
	channel->live = false;
	live_channels--;
	detached = true;
	return ESP_OK;
}

esp_err_t rmt_enable(rmt_channel_handle_t channel)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	assert(channel->live && !channel->enabled);
	channel->enabled = true;
	return ESP_OK;
}

/* Disabling stops the channel and drops its frame (rmt_tx_disable()). */
esp_err_t rmt_disable(rmt_channel_handle_t channel)
{
	assert(channel->live && channel->enabled);
	channel->enabled = false;
	channel->queued = false;
	channel->disables++;
	return ESP_OK;
}

esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
			       rmt_encoder_handle_t *ret_encoder)
{
	(void)config;
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	for (int i = 0; i < 8; i++) {
		if (!encoders[i].live) {
			encoders[i].live = true;
			*ret_encoder = &encoders[i];
			live_encoders++;
			return ESP_OK;
		}
	}
	return ESP_ERR_NO_MEM;
}

esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder)
{
	assert(encoder->live);
	encoder->live = false;
	live_encoders--;
	return ESP_OK;
}

esp_err_t rmt_new_sync_manager(const rmt_sync_manager_config_t *config,
			       rmt_sync_manager_handle_t *ret_synchro)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	assert(!sync_manager.live && config->array_size <= 4);
	sync_manager.live = true;
	sync_manager.size = config->array_size;
	for (size_t i = 0; i < config->array_size; i++) {
		assert(config->tx_channel_array[i]->enabled);
		sync_manager.members[i] = config->tx_channel_array[i];
	}
	sync_manager.resets = 0;
	*ret_synchro = &sync_manager;
	return ESP_OK;
}

esp_err_t rmt_del_sync_manager(rmt_sync_manager_handle_t synchro)
{
	assert(synchro == &sync_manager && sync_manager.live);
	sync_manager.live = false;
	return ESP_OK;
}

esp_err_t rmt_sync_reset(rmt_sync_manager_handle_t synchro)
{
	assert(synchro == &sync_manager && sync_manager.live);
	sync_manager.resets++;
	return ESP_OK;
}

esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t tx_channel,
			       int timeout_ms)
{
	assert(tx_channel->live && timeout_ms == 0);
	return line_busy || tx_channel->queued ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t rmt_transmit(rmt_channel_handle_t tx_channel,
		       rmt_encoder_handle_t encoder, const void *payload,
		       size_t payload_bytes,
		       const rmt_transmit_config_t *config)
{
	assert(tx_channel->live && tx_channel->enabled && encoder->live);
	assert(!tx_channel->queued);
	if ((fail_send && tx_channel == order[0]) ||
	    (fail_transmit_at >= 0 && tx_channel == order[fail_transmit_at]))
		return ESP_FAIL;
	assert(payload_bytes <= sizeof(tx_channel->symbols));
	memcpy(tx_channel->symbols, payload, payload_bytes);
	tx_channel->bytes = payload_bytes;
	tx_channel->transmit = *config;
	transmits++;
	if (tx_channel == order[0])
		sends++;
	/*
	 * Under a sync manager a frame waits until every member has one, then
	 * they all go -- and here, finish at once.
	 */
	tx_channel->queued = true;
	if (sync_manager.live) {
		for (size_t i = 0; i < sync_manager.size; i++)
			if (!sync_manager.members[i]->queued)
				return ESP_OK;
		for (size_t i = 0; i < sync_manager.size; i++)
			sync_manager.members[i]->queued = false;
	} else {
		tx_channel->queued = false;
	}
	return ESP_OK;
}

/* ----------------------------------------------------- the cases' side -- */

static void config_for(struct dshot_config *config, enum dshot_speed speed,
		       uint8_t count)
{
	static const int pins[DSHOT_MOTORS_MAX] = { 4, 5, 6, 7 };

	memset(config, 0, sizeof(*config));
	config->speed = speed;
	config->count = count;
	memcpy(config->gpio, pins, sizeof(pins));
	created = 0;
}

/* High longer than half the bit is a 1. */
static uint16_t sent_frame(int motor)
{
	const struct rmt_channel_t *ch = order[motor];
	uint16_t frame = 0;

	assert(ch->bytes == 16 * sizeof(rmt_symbol_word_t));
	for (int bit = 0; bit < 16; bit++) {
		const rmt_symbol_word_t *s = &ch->symbols[bit];
		uint32_t total = s->duration0 + s->duration1;

		assert(s->level0 == 1 && s->level1 == 0);
		frame = (uint16_t)((frame << 1) | (s->duration0 * 2 > total));
	}
	return frame;
}

#include "../dshot_cases.h"

/* ------------------------------------------------------- the RMT side -- */

static void test_rmt_setup(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	/* bit ticks at 8 MHz and the 1 and 0 high times, [BW] table */
	static const struct { enum dshot_speed speed; uint32_t bit, one, zero; }
	rates[] = {
		{ DSHOT_150, 53, 40, 20 },	/* 6.625 us, 5.000, 2.500 */
		{ DSHOT_300, 27, 20, 10 },	/* 3.375 us, 2.500, 1.250 */
		{ DSHOT_600, 13, 10, 5 },	/* 1.625 us, 1.250, 0.625 */
	};

	for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
		config_for(&config, rates[r].speed, 3);
		assert(dshot_init(&bus, &config) == 0);
		for (int i = 0; i < 3; i++) {
			const rmt_tx_channel_config_t *c = &order[i]->config;

			assert(c->gpio_num == 4 + i);
			assert(c->resolution_hz == 8000000u);
			assert(c->clk_src == RMT_CLK_SRC_DEFAULT);
			assert(c->mem_block_symbols == 48);
			assert(c->trans_queue_depth == 1);
			assert(!c->flags.invert_out && !c->flags.io_od_mode &&
			       !c->flags.io_loop_back && !c->flags.with_dma);
			assert(order[i]->enabled);
		}
		assert(sync_manager.live && sync_manager.size == 3);
		for (int i = 0; i < 3; i++)
			assert(sync_manager.members[i] == order[i]);

		/* 0x8000 | ... : a 1 then 0s; every symbol is one bit */
		assert(dshot_set_value(dshot_esc(&bus, 0), 2047, true) == 0);
		assert(dshot_update(&bus, 0.0f) == 0);
		assert(sync_manager.resets == 1);
		for (int i = 0; i < 3; i++) {
			assert(order[i]->transmit.loop_count == 0);
			assert(order[i]->transmit.flags.eot_level == 0);
			for (int bit = 0; bit < 16; bit++) {
				const rmt_symbol_word_t *s =
					&order[i]->symbols[bit];

				assert(s->duration0 + s->duration1 ==
				       rates[r].bit);
				assert(s->duration0 == rates[r].one ||
				       s->duration0 == rates[r].zero);
			}
		}
		/* 2047 << 1 | 1 = 0xFFF, crc 0xF ^ 0xFF ^ 0xFFF... */
		assert(sent_frame(0) == dshot_frame(2047, true, false));
		assert(order[0]->symbols[0].duration0 == rates[r].one);
		assert(order[1]->symbols[0].duration0 == rates[r].zero);
		dshot_update(&bus, 0.0f);
		assert(sync_manager.resets == 2);
		dshot_del(&bus);
		assert(!sync_manager.live && live_channels == 0 &&
		       live_encoders == 0);
	}

	/* One motor: no sync manager, nothing to reset */
	config_for(&config, DSHOT_300, 1);
	assert(dshot_init(&bus, &config) == 0);
	assert(!sync_manager.live);
	assert(dshot_update(&bus, 0.0f) == 0);
	dshot_del(&bus);

	/* A pin below 0 */
	config_for(&config, DSHOT_300, 2);
	config.gpio[1] = -1;
	assert(dshot_init(&bus, &config) == -EINVAL);
	assert(live_channels == 0);
}

/* Every setup call failing in turn leaves nothing behind. */
static void test_rmt_unwind(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	int calls;

	/* 4 motors: 3 calls each (channel, encoder, enable), then sync */
	config_for(&config, DSHOT_600, 4);
	setup_calls = 0;
	fail_at = 0;
	assert(dshot_init(&bus, &config) == 0);
	calls = setup_calls;
	assert(calls == 13);
	dshot_del(&bus);

	for (fail_at = 1; fail_at <= calls; fail_at++) {
		config_for(&config, DSHOT_600, 4);
		setup_calls = 0;
		assert(dshot_init(&bus, &config) == -EIO);
		assert(live_channels == 0 && live_encoders == 0 &&
		       !sync_manager.live);
		for (int i = 0; i < 8; i++)
			assert(!channels[i].enabled);
	}
	fail_at = 0;
}

/*
 * rmt_transmit() failing on motor 2 of 3: motors 0 and 1 hold frames the
 * sync manager would keep waiting forever. They are stopped and re-enabled
 * and the sync reset, so the next update goes.
 */
static void test_rmt_partial(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	int resets;

	config_for(&config, DSHOT_600, 3);
	assert(dshot_init(&bus, &config) == 0);
	assert(dshot_command(dshot_esc(&bus, 0), DSHOT_CMD_3D_MODE_ON) == 0);
	resets = sync_manager.resets;
	fail_transmit_at = 2;
	assert(dshot_update(&bus, 0.001f) == -EIO);
	fail_transmit_at = -1;
	assert(order[0]->disables == 1 && order[1]->disables == 1);
	assert(order[2]->disables == 0);
	for (int i = 0; i < 3; i++)
		assert(order[i]->enabled && !order[i]->queued);
	/* one reset before the send, one after the failure */
	assert(sync_manager.resets == resets + 2);
	/* not busy, and the command still has all six frames to go */
	for (int i = 0; i < 6; i++) {
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(sent_frame(0) == dshot_frame(10, true, false));
	}
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(sent_frame(0) == 0x0000);
	dshot_del(&bus);
}

int main(void)
{
	run_dshot_cases();
	test_rmt_setup();
	test_rmt_unwind();
	test_rmt_partial();
	printf("test_dshot (esp): all passed\n");
	return 0;
}
