#ifndef RMT_RX_H
#define RMT_RX_H

#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_encoder.h"
#include "esp_err.h"

/* Host stub for the ESP-IDF RMT RX driver, as in IDF v5.5.5's
 * driver/rmt_rx.h: only what the DShot port uses. */

typedef struct {
	rmt_rx_done_callback_t on_recv_done;
} rmt_rx_event_callbacks_t;

typedef struct {
	gpio_num_t gpio_num;
	rmt_clock_source_t clk_src;
	uint32_t resolution_hz;
	size_t mem_block_symbols;
	int intr_priority;
	struct {
		uint32_t invert_in : 1;
		uint32_t with_dma : 1;
		uint32_t io_loop_back : 1;
		uint32_t allow_pd : 1;
	} flags;
} rmt_rx_channel_config_t;

typedef struct {
	uint32_t signal_range_min_ns;
	uint32_t signal_range_max_ns;
	struct extra_rmt_receive_flags {
		uint32_t en_partial_rx : 1;
	} flags;
} rmt_receive_config_t;

esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config,
			     rmt_channel_handle_t *ret_chan);
esp_err_t rmt_receive(rmt_channel_handle_t rx_channel, void *buffer,
		      size_t buffer_size, const rmt_receive_config_t *config);
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t rx_channel,
					  const rmt_rx_event_callbacks_t *cbs,
					  void *user_data);

#endif
