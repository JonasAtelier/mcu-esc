#ifndef RMT_TX_H
#define RMT_TX_H

#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_encoder.h"
#include "esp_err.h"

/* Host stub for the ESP-IDF RMT TX driver, as in IDF v5.5.5's
 * driver/rmt_tx.h and driver/rmt_common.h: only what the DShot port uses. */

typedef struct {
	gpio_num_t gpio_num;
	rmt_clock_source_t clk_src;
	uint32_t resolution_hz;
	size_t mem_block_symbols;
	size_t trans_queue_depth;
	int intr_priority;
	struct {
		uint32_t invert_out : 1;
		uint32_t with_dma : 1;
		uint32_t io_loop_back : 1;
		uint32_t io_od_mode : 1;
		uint32_t allow_pd : 1;
		uint32_t init_level : 1;
	} flags;
} rmt_tx_channel_config_t;

typedef struct {
	int loop_count;
	struct {
		uint32_t eot_level : 1;
		uint32_t queue_nonblocking : 1;
	} flags;
} rmt_transmit_config_t;

typedef struct {
	const rmt_channel_handle_t *tx_channel_array;
	size_t array_size;
} rmt_sync_manager_config_t;

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
			     rmt_channel_handle_t *ret_chan);
esp_err_t rmt_del_channel(rmt_channel_handle_t channel);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_disable(rmt_channel_handle_t channel);
esp_err_t rmt_transmit(rmt_channel_handle_t tx_channel,
		       rmt_encoder_handle_t encoder, const void *payload,
		       size_t payload_bytes,
		       const rmt_transmit_config_t *config);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t tx_channel,
			       int timeout_ms);
esp_err_t rmt_new_sync_manager(const rmt_sync_manager_config_t *config,
			       rmt_sync_manager_handle_t *ret_synchro);
esp_err_t rmt_del_sync_manager(rmt_sync_manager_handle_t synchro);
esp_err_t rmt_sync_reset(rmt_sync_manager_handle_t synchro);

#endif
