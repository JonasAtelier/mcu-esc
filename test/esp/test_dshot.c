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

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "hal/gpio_ll.h"

#include "dshot.h"
#include "dshot_math.h"
#include "esc.h"
#include "esc_priv.h"	/* esc_dshot_reply(), read directly below */

static struct rmt_channel_t {
	bool live, enabled;
	bool queued;		/* a frame waits for the sync manager */
	int disables;
	rmt_tx_channel_config_t config;
	rmt_symbol_word_t symbols[16];
	size_t bytes;
	rmt_transmit_config_t transmit;
	/* bidirectional */
	bool rx;
	rmt_rx_channel_config_t rx_config;
	rmt_tx_done_callback_t on_sent;
	rmt_rx_done_callback_t on_heard;
	void *context;
	bool listening;
	rmt_symbol_word_t *buffer;
	size_t buffer_size;
	rmt_receive_config_t receive;
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
static rmt_channel_handle_t order[4];	/* TX channels in creation order */
static int created;
static rmt_channel_handle_t receivers[4];	/* RX channels, the same */
static int receivers_created;
static int fail_receive_at = -1;	/* rmt_receive() on this motor fails */
static bool open_drain[64];		/* per GPIO */
static uint32_t esc_answer[DSHOT_MOTORS_MAX];

void gpio_ll_od_enable(gpio_dev_t *hw, uint32_t gpio_num)
{
	assert(hw == &GPIO && gpio_num < 64);
	open_drain[gpio_num] = true;
}

void gpio_ll_od_disable(gpio_dev_t *hw, uint32_t gpio_num)
{
	assert(hw == &GPIO && gpio_num < 64);
	open_drain[gpio_num] = false;
}

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

esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config,
			     rmt_channel_handle_t *ret_chan)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	for (int i = 0; i < 8; i++) {
		if (!channels[i].live) {
			memset(&channels[i], 0, sizeof(channels[i]));
			channels[i].live = true;
			channels[i].rx = true;
			channels[i].rx_config = *config;
			*ret_chan = &channels[i];
			live_channels++;
			if (receivers_created < 4)
				receivers[receivers_created] = &channels[i];
			receivers_created++;
			return ESP_OK;
		}
	}
	return ESP_ERR_NOT_FOUND;
}

esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t tx_channel,
					  const rmt_tx_event_callbacks_t *cbs,
					  void *user_data)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	assert(tx_channel->live && !tx_channel->rx && !tx_channel->enabled);
	tx_channel->on_sent = cbs->on_trans_done;
	tx_channel->context = user_data;
	return ESP_OK;
}

esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t rx_channel,
					  const rmt_rx_event_callbacks_t *cbs,
					  void *user_data)
{
	if (setup_call() != ESP_OK)
		return ESP_FAIL;
	assert(rx_channel->live && rx_channel->rx && !rx_channel->enabled);
	rx_channel->on_heard = cbs->on_recv_done;
	rx_channel->context = user_data;
	return ESP_OK;
}

esp_err_t rmt_receive(rmt_channel_handle_t rx_channel, void *buffer,
		      size_t buffer_size, const rmt_receive_config_t *config)
{
	assert(rx_channel->live && rx_channel->rx && rx_channel->enabled);
	assert(!rx_channel->listening);
	if (fail_receive_at >= 0 && rx_channel == receivers[fail_receive_at])
		return ESP_FAIL;
	rx_channel->listening = true;
	rx_channel->buffer = buffer;
	rx_channel->buffer_size = buffer_size;
	rx_channel->receive = *config;
	return ESP_OK;
}

/*
 * The line as the receiver on tx's GPIO hears it, then its done call: the
 * frame, inverted; the line at rest 30 us [EDT "Timing"]; the motor's
 * answer at 5/4 of the bit rate [BW], edges rounded to the 8 MHz tick;
 * then at rest until the receiver stops. Symbols alternate levels, a zero
 * duration ends them.
 */
static void hear(rmt_channel_handle_t tx, uint32_t answer)
{
	rmt_channel_handle_t rx = NULL;
	struct { bool high; uint32_t ticks; } runs[64];
	int count = 0;
	uint32_t bit = tx->symbols[0].duration0 + tx->symbols[0].duration1;
	uint32_t speed = bit == 13 ? 600 : bit == 27 ? 300 : 150;
	rmt_rx_done_event_data_t event = { 0 };
	size_t symbols;

	for (int i = 0; i < 4; i++)
		if (receivers[i] && receivers[i]->live &&
		    receivers[i]->rx_config.gpio_num == tx->config.gpio_num)
			rx = receivers[i];
	if (!rx || !rx->listening)
		return;
#define RUN(level, length) do { \
		if (count && runs[count - 1].high == (level)) \
			runs[count - 1].ticks += (length); \
		else { \
			runs[count].high = (level); \
			runs[count].ticks = (length); \
			count++; \
		} \
	} while (0)
	for (int b = 0; b < 16; b++) {
		RUN(!tx->symbols[b].level0, tx->symbols[b].duration0);
		RUN(!tx->symbols[b].level1, tx->symbols[b].duration1);
	}
	RUN(true, 240);
	if (answer) {
		uint32_t done = 0;

		for (int b = 0; b < 21; b++) {
			uint32_t end = (uint32_t)(((uint64_t)(b + 1) *
						   8000000u * 4 +
						   speed * 2500u) /
						  (speed * 5000u));

			RUN((answer >> (20 - b)) & 1, end - done);
			done = end;
		}
	}
	RUN(true, rx->receive.signal_range_max_ns / 125);
#undef RUN
	symbols = (size_t)(count + 2) / 2;
	assert(symbols * sizeof(rmt_symbol_word_t) <= rx->buffer_size);
	memset(rx->buffer, 0, rx->buffer_size);
	for (int r = 0; r < count; r++) {
		rmt_symbol_word_t *symbol =
			&((rmt_symbol_word_t *)rx->buffer)[r / 2];

		if (r % 2) {
			symbol->level1 = runs[r].high;
			symbol->duration1 = runs[r].ticks;
		} else {
			symbol->level0 = runs[r].high;
			symbol->duration0 = runs[r].ticks;
		}
	}
	rx->listening = false;
	event.received_symbols = rx->buffer;
	event.num_symbols = symbols;
	rx->on_heard(rx, &event, rx->context);
}

/* A frame went out on tx: its done call, then the receiver's. */
static void frame_done(rmt_channel_handle_t tx)
{
	const rmt_tx_done_event_data_t event = { .num_symbols = 17 };
	uint32_t answer = 0;

	for (int i = 0; i < 4; i++)
		if (order[i] == tx)
			answer = esc_answer[i];
	if (tx->on_sent)
		tx->on_sent(tx, &event, tx->context);
	hear(tx, answer);
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

/* Disabling stops the channel and drops its frame (rmt_tx_disable()),
 * or its receive (rmt_rx_disable()). */
esp_err_t rmt_disable(rmt_channel_handle_t channel)
{
	assert(channel->live && channel->enabled);
	channel->enabled = false;
	channel->queued = false;
	channel->listening = false;
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
	assert(!tx_channel->queued && !tx_channel->rx);
	/* bidirectional: push-pull again before the frame; never touched
	 * otherwise (config_for() clears it) */
	assert(!open_drain[tx_channel->config.gpio_num]);
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
		for (size_t i = 0; i < sync_manager.size; i++) {
			sync_manager.members[i]->queued = false;
			frame_done(sync_manager.members[i]);
		}
	} else {
		tx_channel->queued = false;
		frame_done(tx_channel);
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
	receivers_created = 0;
	memset(receivers, 0, sizeof(receivers));
	memset(open_drain, 0, sizeof(open_drain));
}

static void bidir_config_for(struct dshot_config *config,
			     enum dshot_speed speed, uint8_t count)
{
	config_for(config, speed, count);
	config->bidirectional = true;
}

/* The fake finishes every frame and its reply inside rmt_transmit(). */
static void settle(void)
{
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

/*
 * Bidirectional on the RMT: TX inverted, push-pull; an RX channel on the
 * same GPIO at 8 MHz; a receive armed before every frame, glitches under
 * 200 ns, stopping after 45 us at rest; the pin open-drain once the frame
 * is out and push-pull before the next (asserted in rmt_transmit()).
 */
static void test_rmt_bidir_setup(void)
{
	struct dshot_bus bus;
	struct dshot_config config;

	bidir_config_for(&config, DSHOT_600, 4);
	assert(dshot_init(&bus, &config) == 0);
	assert(live_channels == 8 && sync_manager.live &&
	       sync_manager.size == 4);
	for (int i = 0; i < 4; i++) {
		const rmt_tx_channel_config_t *c = &order[i]->config;
		const rmt_rx_channel_config_t *r = &receivers[i]->rx_config;

		assert(c->flags.invert_out && !c->flags.io_od_mode &&
		       !c->flags.io_loop_back);
		assert(c->resolution_hz == 8000000u &&
		       c->mem_block_symbols == 48);
		assert(order[i]->on_sent &&
		       (uintptr_t)order[i]->context == (uintptr_t)(4 + i));
		assert(r->gpio_num == 4 + i);
		assert(r->clk_src == RMT_CLK_SRC_DEFAULT);
		assert(r->resolution_hz == 8000000u);
		assert(r->mem_block_symbols == 48);
		assert(!r->flags.invert_in && !r->flags.io_loop_back &&
		       !r->flags.with_dma);
		assert(receivers[i]->enabled && receivers[i]->on_heard);
		assert(!receivers[i]->listening);
	}
	assert(dshot_update(&bus, 0.001f) == 0);
	for (int i = 0; i < 4; i++) {
		const rmt_receive_config_t *r = &receivers[i]->receive;

		assert(r->signal_range_min_ns == 200);
		assert(r->signal_range_max_ns == 45000);
		assert(!r->flags.en_partial_rx);
		assert(receivers[i]->buffer_size == 48 * 4);
		assert(open_drain[4 + i]);
		assert(order[i]->transmit.flags.eot_level == 0);
	}
	/* a receiver still listening is a busy line */
	receivers[2]->listening = true;
	bus.port.heard_symbols[2] = -1;
	assert(dshot_update(&bus, 0.001f) == -EBUSY);
	bus.port.heard_symbols[2] = 0;
	receivers[2]->listening = false;
	assert(dshot_update(&bus, 0.001f) == 0);
	dshot_del(&bus);
	assert(live_channels == 0 && live_encoders == 0 && !sync_manager.live);
}

/*
 * What the receiver heard, as line bits: the reply after the frame's rest
 * at both speeds, nothing heard but the frame, levels that are no reply.
 */
static void test_rmt_bidir_heard(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	uint32_t raw;
	static const enum dshot_speed speeds[] = { DSHOT_300, DSHOT_600 };

	for (size_t k = 0; k < 2; k++) {
		bidir_config_for(&config, speeds[k], 1);
		assert(dshot_init(&bus, &config) == 0);
		assert(esc_dshot_reply(&bus.port, 0, &raw) == -ENOENT);
		esc_answer[0] = answer_of(0x82C);	/* [BW]'s example */
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(esc_dshot_reply(&bus.port, 0, &raw) == 0);
		assert(raw == answer_of(0x82C));
		assert(esc_dshot_reply(&bus.port, 0, &raw) == -ENOENT);
		esc_answer[0] = 0;
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(esc_dshot_reply(&bus.port, 0, &raw) == -ENODATA);
		/* levels that are no reply come over as they are; the GCR
		 * refuses them (17 bits low) */
		esc_answer[0] = 0x00000F;
		assert(dshot_update(&bus, 0.001f) == 0);
		assert(esc_dshot_reply(&bus.port, 0, &raw) == 0);
		assert(raw == 0x00000F);
		esc_answer[0] = 0;
		dshot_del(&bus);
	}
}

/* Every bidirectional setup call failing in turn leaves nothing behind:
 * per motor TX channel, encoder, TX callback, enable, RX channel, RX
 * callback, RX enable; then the sync manager. */
static void test_rmt_bidir_unwind(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	int calls;

	bidir_config_for(&config, DSHOT_600, 4);
	setup_calls = 0;
	fail_at = 0;
	assert(dshot_init(&bus, &config) == 0);
	calls = setup_calls;
	assert(calls == 29);
	dshot_del(&bus);

	for (fail_at = 1; fail_at <= calls; fail_at++) {
		bidir_config_for(&config, DSHOT_600, 4);
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
 * A receive or a transmit failing part-way: every armed receiver stopped
 * and ready again, nothing busy, the next update going and heard.
 */
static void test_rmt_bidir_partial(void)
{
	struct dshot_bus bus;
	struct dshot_config config;
	uint32_t erpm;

	bidir_config_for(&config, DSHOT_600, 3);
	assert(dshot_init(&bus, &config) == 0);
	fail_receive_at = 1;
	assert(dshot_update(&bus, 0.001f) == -EIO);
	fail_receive_at = -1;
	assert(receivers[0]->disables == 1 && receivers[1]->disables == 0);
	for (int i = 0; i < 3; i++)
		assert(!receivers[i]->listening && receivers[i]->enabled &&
		       bus.port.heard_symbols[i] == 0);
	assert(transmits == 0 || !order[0]->queued);

	fail_transmit_at = 2;
	assert(dshot_update(&bus, 0.001f) == -EIO);
	fail_transmit_at = -1;
	for (int i = 0; i < 3; i++)
		assert(!receivers[i]->listening && receivers[i]->enabled &&
		       bus.port.heard_symbols[i] == 0);
	assert(bus.port.unread == 0);
	esc_answer[1] = answer_of(0x300);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(dshot_update(&bus, 0.001f) == 0);
	assert(dshot_get_erpm(dshot_esc(&bus, 1), &erpm) == 0 &&
	       erpm == 117188);
	esc_answer[1] = 0;
	dshot_del(&bus);
}

int main(void)
{
	run_dshot_cases();
	test_rmt_setup();
	test_rmt_unwind();
	test_rmt_partial();
	test_rmt_bidir_setup();
	test_rmt_bidir_heard();
	test_rmt_bidir_unwind();
	test_rmt_bidir_partial();
	printf("test_dshot (esp): all passed\n");
	return 0;
}
