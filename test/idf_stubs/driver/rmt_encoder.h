#ifndef RMT_ENCODER_H
#define RMT_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Host stub for the ESP-IDF RMT driver's types and encoders: only what the
 * DShot port uses. Names, fields and layout as in IDF v5.5.5's
 * hal/rmt_types.h, driver/rmt_types.h and driver/rmt_encoder.h.
 */
typedef int gpio_num_t;
typedef struct rmt_channel_t *rmt_channel_handle_t;
typedef struct rmt_encoder_t *rmt_encoder_handle_t;
typedef struct rmt_sync_manager_t *rmt_sync_manager_handle_t;

typedef enum {
	RMT_CLK_SRC_DEFAULT = 4,
} rmt_clock_source_t;

typedef union {
	struct {
		uint16_t duration0 : 15;
		uint16_t level0 : 1;
		uint16_t duration1 : 15;
		uint16_t level1 : 1;
	};
	uint32_t val;
} rmt_symbol_word_t;

typedef struct {
	int unused;	/* empty in IDF; C forbids that without GNU */
} rmt_copy_encoder_config_t;

/* The event data and callbacks, driver/rmt_types.h. */
typedef struct {
	size_t num_symbols;
} rmt_tx_done_event_data_t;

typedef bool (*rmt_tx_done_callback_t)(rmt_channel_handle_t tx_chan,
				       const rmt_tx_done_event_data_t *edata,
				       void *user_ctx);

typedef struct {
	rmt_symbol_word_t *received_symbols;
	size_t num_symbols;
	struct {
		uint32_t is_last : 1;
	} flags;
} rmt_rx_done_event_data_t;

typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t rx_chan,
				       const rmt_rx_done_event_data_t *edata,
				       void *user_ctx);

esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
			       rmt_encoder_handle_t *ret_encoder);
esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder);

#endif
