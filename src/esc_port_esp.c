/*
 * esc_priv.h on ESP-IDF: the bus layer and the PWM port. The CMakeLists
 * lists this file; stm-forge compiles every .c in src/, so outside ESP-IDF
 * it is empty.
 */
#ifdef ESP_PLATFORM

#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esc.h"
#include "esc_priv.h"

#ifdef ESC_HAVE_DSHOT
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "hal/gpio_ll.h"	/* gpio_ll_od_*(): inline, for the ISR */

#include "dshot/dshot_math.h"
#endif

/* 400 kHz when a caller says 0: fast mode. */
#define DEFAULT_SCL_HZ		400000

/*
 * Ticks for at least ms milliseconds, for the bus timeouts. Two things make
 * a plain pdMS_TO_TICKS(ms) wait less than asked:
 *
 *   - pdMS_TO_TICKS() truncates. At IDF's default 100 Hz tick, 2 ms is 0
 *     ticks and 15 ms is 1, so round up instead.
 *   - A wait of n ticks counts tick interrupts, and the first one may be
 *     about to fire, so n ticks can be as little as n - 1 full periods.
 *     Add one.
 *
 * So 20 ms at 100 Hz is 3 ticks, 20-30 ms: long, never short.
 */
static TickType_t esc_ticks(uint32_t ms)
{
	return (TickType_t)(((uint64_t)ms * configTICK_RATE_HZ + 999) / 1000) +
	       1;
}

/* --------------------------------------------------------------- I2C -- */

int esc_i2c_probe(esc_i2c_bus handle, uint8_t address,
		  uint32_t timeout_ms)
{
	return esc_errno(i2c_master_probe(handle, address,
					  (int)timeout_ms));
}

static int esc_i2c_transfer(struct esc_bus *bus,
			    uint8_t cmd,
			    const uint8_t *tx, size_t tx_len, uint8_t *rx,
			    size_t rx_len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN];

	if ((tx_len && rx_len) || tx_len > ESC_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (rx_len)
		return esc_errno(i2c_master_transmit_receive(
			bus->i2c, out, 1, rx, rx_len, (int)bus->timeout_ms));
	if (tx_len)
		memcpy(out + 1, tx, tx_len);
	return esc_errno(i2c_master_transmit(bus->i2c, out,
					     1 + tx_len,
					     (int)bus->timeout_ms));
}

static void esc_i2c_detach(struct esc_bus *bus)
{
	i2c_master_bus_rm_device(bus->i2c);
}

static const struct esc_bus_ops esc_i2c_ops = {
	.kind = ESC_BUS_I2C,
	.transfer = esc_i2c_transfer,
	.detach = esc_i2c_detach,
};

int esc_i2c_attach(struct esc_bus *bus,
		   esc_i2c_bus handle,
		   uint8_t address, uint32_t scl_hz, uint32_t timeout_ms)
{
	i2c_device_config_t devcfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address,
		.scl_speed_hz = scl_hz ? scl_hz : DEFAULT_SCL_HZ,
	};

	int ret;

	ret = esc_errno(i2c_master_bus_add_device(handle, &devcfg,
						  &bus->i2c));
	if (ret)
		return ret;
	/* Set only now: a bus with ops is one that has something to detach. */
	bus->ops = &esc_i2c_ops;
	bus->timeout_ms = timeout_ms;
	return 0;
}

/* --------------------------------------------------------------- SPI -- */

/*
 * One full-duplex transaction, so CS stays low across cmd, tx and rx: clock
 * out cmd and tx followed by zeros, and keep what came back after them. The
 * buffers live on the stack, word-aligned: IDF bounces non-DMA-capable or
 * unaligned buffers through a copy of its own.
 */
static int esc_spi_transfer(struct esc_bus *bus,
			    uint8_t cmd,
			    const uint8_t *tx, size_t tx_len, uint8_t *rx,
			    size_t rx_len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN] __attribute__((aligned(4))) = {
		0
	};
	uint8_t in[1 + ESC_BUS_MAX_LEN] __attribute__((aligned(4)));
	spi_transaction_t t = { 0 };
	int ret;

	if (tx_len + rx_len > ESC_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	t.length = (1 + tx_len + rx_len) * 8;
	t.tx_buffer = out;
	t.rx_buffer = in;
	ret = esc_errno(spi_device_polling_transmit(bus->spi, &t));
	if (!ret && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return ret;
}

/*
 * One frame, full duplex: rx[i] is what came back while tx[i] went out.
 * Polled, not queued: for a few bytes the interrupt and task switch of a
 * queued transfer cost more than the transfer itself. A frame of up to four
 * bytes travels in the transaction itself (tx_data, rx_data); a longer one,
 * through word-aligned buffers on the stack.
 */
static int esc_spi_exchange(struct esc_bus *bus,
			    const uint8_t *tx,
			    uint8_t *rx, size_t len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN] __attribute__((aligned(4)));
	uint8_t in[1 + ESC_BUS_MAX_LEN] __attribute__((aligned(4)));
	spi_transaction_t t = { 0 };
	bool inline_data = len <= sizeof(t.tx_data);
	int ret;

	if (!len || len > sizeof(out))
		return -EINVAL;

	t.length = len * 8;
	if (inline_data) {
		t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
		memcpy(t.tx_data, tx, len);
	} else {
		memcpy(out, tx, len);
		t.tx_buffer = out;
		t.rx_buffer = in;
	}
	ret = esc_errno(spi_device_polling_transmit(bus->spi, &t));
	if (!ret)
		memcpy(rx, inline_data ? t.rx_data : in, len);
	return ret;
}

static void esc_spi_detach(struct esc_bus *bus)
{
	spi_bus_remove_device(bus->spi);
}

static const struct esc_bus_ops esc_spi_ops = {
	.kind = ESC_BUS_SPI,
	.transfer = esc_spi_transfer,
	.exchange = esc_spi_exchange,
	.detach = esc_spi_detach,
};

int esc_spi_attach(struct esc_bus *bus,
		   spi_host_device_t host,
		   int cs_pin, uint32_t clock_hz, uint8_t mode)
{
	spi_device_interface_config_t devcfg = {
		.mode = mode,
		.clock_speed_hz = (int)clock_hz,
		.spics_io_num = cs_pin,
		.queue_size = 1,
	};

	int ret;

	ret = esc_errno(spi_bus_add_device(host, &devcfg,
					   &bus->spi));
	if (ret)
		return ret;
	bus->ops = &esc_spi_ops;
	bus->timeout_ms = 0;	/* spi_master transfers take none */
	return 0;
}

/* -------------------------------------------------------------- UART -- */

/*
 * Drop whatever is waiting -- a late reply to an earlier command would read
 * as the start of this one -- then send, then wait for the whole reply.
 * uart_read_bytes() returns early only with everything; a short count means
 * the timeout ran out.
 */
static int esc_uart_transfer(struct esc_bus *bus,
			     uint8_t cmd,
			     const uint8_t *tx, size_t tx_len,
			     uint8_t *rx, size_t rx_len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN];
	int n, ret;

	if (tx_len > ESC_BUS_MAX_LEN || rx_len > INT32_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	ret = esc_errno(uart_flush_input(bus->uart));
	if (ret)
		return ret;
	n = uart_write_bytes(bus->uart, out, 1 + tx_len);
	if (n != (int)(1 + tx_len))
		return -EIO;
	if (!rx_len)
		return 0;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    esc_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/*
 * The UART driver keeps everything the line delivers in its RX ring buffer,
 * so the echo of a whole datagram waits there however fast it came: flush,
 * send, wait for the last bit to leave, then read back as many bytes as
 * went out.
 */
static int esc_uart_exchange(struct esc_bus *bus,
			     const uint8_t *tx,
			     uint8_t *rx, size_t len)
{
	int n, ret;

	if (!len || len > 1 + ESC_BUS_MAX_LEN)
		return -EINVAL;

	ret = esc_errno(uart_flush_input(bus->uart));
	if (ret)
		return ret;
	n = uart_write_bytes(bus->uart, tx, len);
	if (n != (int)len)
		return -EIO;
	ret = esc_errno(uart_wait_tx_done(bus->uart,
					  esc_ticks(bus->timeout_ms)));
	if (ret)
		return ret;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)len,
			    esc_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < len ? -ETIMEDOUT : 0;
}

static int esc_uart_receive(struct esc_bus *bus,
			    uint8_t *rx,
			    size_t rx_len)
{
	int n;

	if (rx_len > INT32_MAX)
		return -EINVAL;
	n = uart_read_bytes(bus->uart, rx, (uint32_t)rx_len,
			    esc_ticks(bus->timeout_ms));
	if (n < 0)
		return -EIO;
	return (size_t)n < rx_len ? -ETIMEDOUT : 0;
}

/* The driver is the app's: nothing was added, nothing to remove. */
static void esc_uart_detach(struct esc_bus *bus)
{
	(void)bus;
}

static const struct esc_bus_ops esc_uart_ops = {
	.kind = ESC_BUS_UART,
	.transfer = esc_uart_transfer,
	.exchange = esc_uart_exchange,
	.receive = esc_uart_receive,
	.detach = esc_uart_detach,
};

int esc_uart_attach(struct esc_bus *bus, uart_port_t port,
		    uint32_t timeout_ms)
{
	if (!uart_is_driver_installed(port))
		return -EINVAL;
	bus->ops = &esc_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = port;
	return 0;
}

/* --------------------------------------------------------------- PWM -- */

#define ESC_PWM_MODE		LEDC_LOW_SPEED_MODE
#define ESC_PWM_DUTY_FULL	(1u << ESC_PWM_DUTY_BITS)

/*
 * pulse_us as LEDC duty: the share of one period, rounded to the nearest of
 * 2^14 steps. At 400 Hz one step is 0.15 us. A pulse as long as the period
 * or longer leaves no low time: -EINVAL.
 */
static int esc_pwm_duty(const struct esc_pwm *pwm, uint32_t pulse_us,
			uint32_t *duty)
{
	uint64_t scaled = (uint64_t)pulse_us * pwm->rate_hz;

	if (scaled >= 1000000u)
		return -EINVAL;
	*duty = (uint32_t)((scaled * ESC_PWM_DUTY_FULL + 500000u) / 1000000u);
	return 0;
}

int esc_pwm_check(const struct esc_pwm *pwm, uint32_t pulse_us)
{
	uint32_t duty;

	return esc_pwm_duty(pwm, pulse_us, &duty);
}

int esc_pwm_attach(struct esc_pwm *pwm, int gpio, ledc_timer_t timer,
		   ledc_channel_t channel, uint32_t rate_hz)
{
	ledc_timer_config_t timer_cfg = {
		.speed_mode = ESC_PWM_MODE,
		.duty_resolution = LEDC_TIMER_14_BIT,	/* ESC_PWM_DUTY_BITS */
		.timer_num = timer,
		.freq_hz = rate_hz,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	int ret;

	if (!rate_hz || gpio < 0)
		return -EINVAL;
	ret = esc_errno(ledc_timer_config(&timer_cfg));
	if (ret)
		return ret;
	pwm->gpio = gpio;
	pwm->timer = timer;
	pwm->channel = channel;
	pwm->rate_hz = rate_hz;
	return 0;
}

int esc_pwm_start(struct esc_pwm *pwm, uint32_t pulse_us)
{
	ledc_channel_config_t channel_cfg = {
		.gpio_num = pwm->gpio,
		.speed_mode = ESC_PWM_MODE,
		.channel = pwm->channel,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = pwm->timer,
	};
	uint32_t duty;
	int ret;

	ret = esc_pwm_duty(pwm, pulse_us, &duty);
	if (ret)
		return ret;
	channel_cfg.duty = duty;
	return esc_errno(ledc_channel_config(&channel_cfg));
}

int esc_pwm_write_us(struct esc_pwm *pwm, uint32_t pulse_us)
{
	uint32_t duty;
	int ret;

	ret = esc_pwm_duty(pwm, pulse_us, &duty);
	if (ret)
		return ret;
	ret = esc_errno(ledc_set_duty(ESC_PWM_MODE, pwm->channel, duty));
	if (ret)
		return ret;
	return esc_errno(ledc_update_duty(ESC_PWM_MODE, pwm->channel));
}

void esc_pwm_stop(struct esc_pwm *pwm)
{
	ledc_stop(ESC_PWM_MODE, pwm->channel, 0);
}

#ifdef ESC_HAVE_DSHOT
/* ------------------------------------------------------------- DShot -- */

/*
 * Bidirectional: a pulse shorter than this is a glitch to the receiver.
 * The shortest real one is 375 ns, a 1's rest at DShot600 (3 ticks of
 * 125 ns, esc_priv.h).
 */
#define DSHOT_GLITCH_NS		200

/*
 * Bidirectional: the frame is out, so let the line go for the ESC's reply
 * [BW "Bidirectional DSHOT"]. Open-drain, the channel's output at rest --
 * high, the inverted line's rest -- drives nothing, and the RX channel's
 * pull-up holds the line until the ESC drives it. In the RMT's interrupt,
 * so nothing from flash: the LL call is inline. context is the GPIO.
 */
static IRAM_ATTR bool esc_dshot_sent(rmt_channel_handle_t channel,
				     const rmt_tx_done_event_data_t *event,
				     void *context)
{
	(void)channel;
	(void)event;
	gpio_ll_od_enable(&GPIO, (uint32_t)(uintptr_t)context);
	return false;
}

/*
 * The RX channel has stopped: the line held one level for
 * DSHOT_REPLY_WAIT_US. context is the motor's heard_symbols.
 */
static IRAM_ATTR bool esc_dshot_heard(rmt_channel_handle_t channel,
				      const rmt_rx_done_event_data_t *event,
				      void *context)
{
	(void)channel;
	*(volatile int32_t *)context = (int32_t)event->num_symbols;
	return false;
}

/* Motor i's RX channel on its GPIO, ready to receive; nothing on failure. */
static int esc_dshot_add_receiver(struct dshot_port *port, uint8_t i)
{
	rmt_rx_channel_config_t receiver_cfg = {
		.gpio_num = port->gpio[i],
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = DSHOT_RMT_HZ,
		.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
	};
	const rmt_rx_event_callbacks_t heard = {
		.on_recv_done = esc_dshot_heard,
	};
	int ret;

	ret = esc_errno(rmt_new_rx_channel(&receiver_cfg, &port->receiver[i]));
	if (ret)
		return ret;
	ret = esc_errno(rmt_rx_register_event_callbacks(
		port->receiver[i], &heard, (void *)&port->heard_symbols[i]));
	if (!ret)
		ret = esc_errno(rmt_enable(port->receiver[i]));
	if (ret) {
		rmt_del_channel(port->receiver[i]);
		port->receiver[i] = NULL;
	}
	return ret;
}

/*
 * Motor i: its TX channel, encoder and, bidirectional, the TX-done call
 * and the receiver; nothing on failure. Bidirectional, the TX output is
 * inverted: the same symbols, the line resting high.
 */
static int esc_dshot_add(struct dshot_port *port, uint8_t i)
{
	rmt_tx_channel_config_t channel_cfg = {
		.gpio_num = port->gpio[i],
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = DSHOT_RMT_HZ,
		.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
		.trans_queue_depth = 1,
	};
	const rmt_tx_event_callbacks_t sent = {
		.on_trans_done = esc_dshot_sent,
	};
	/* Empty in IDF: no initialiser fits both it and -pedantic. */
	rmt_copy_encoder_config_t encoder_cfg;
	int ret;

	channel_cfg.flags.invert_out = port->bidirectional;
	memset(&encoder_cfg, 0, sizeof(encoder_cfg));
	ret = esc_errno(rmt_new_tx_channel(&channel_cfg, &port->channel[i]));
	if (ret)
		return ret;
	ret = esc_errno(rmt_new_copy_encoder(&encoder_cfg, &port->encoder[i]));
	if (ret) {
		rmt_del_channel(port->channel[i]);
		return ret;
	}
	if (port->bidirectional)
		ret = esc_errno(rmt_tx_register_event_callbacks(
			port->channel[i], &sent,
			(void *)(uintptr_t)port->gpio[i]));
	if (!ret)
		ret = esc_errno(rmt_enable(port->channel[i]));
	if (ret) {
		rmt_del_encoder(port->encoder[i]);
		rmt_del_channel(port->channel[i]);
		return ret;
	}
	if (port->bidirectional) {
		ret = esc_dshot_add_receiver(port, i);
		if (ret) {
			rmt_disable(port->channel[i]);
			rmt_del_encoder(port->encoder[i]);
			rmt_del_channel(port->channel[i]);
		}
	}
	return ret;
}

/* Undo esc_dshot_add() for motor i. */
static void esc_dshot_remove(struct dshot_port *port, uint8_t i)
{
	if (port->receiver[i]) {
		rmt_disable(port->receiver[i]);
		rmt_del_channel(port->receiver[i]);
		port->receiver[i] = NULL;
	}
	rmt_disable(port->channel[i]);
	rmt_del_encoder(port->encoder[i]);
	rmt_del_channel(port->channel[i]);
}

/* The bit and high ticks at DSHOT_RMT_HZ are worked out in esc_priv.h. */
int esc_dshot_attach(struct dshot_port *port,
		     const struct dshot_config *config)
{
	int ret = 0;
	uint8_t i;

	for (i = 0; i < config->count; i++)
		if (config->gpio[i] < 0)
			return -EINVAL;
#if !SOC_RMT_SUPPORT_TX_SYNCHRO
	/* Without a sync manager the frames would start one after another. */
	if (config->count > 1)
		return -EOPNOTSUPP;
#endif
	/*
	 * Bidirectional: an RX channel per motor besides its TX channel. The
	 * ESP32's and S2's channels each do either, so that is two of them a
	 * motor; elsewhere RX channels are their own (soc_caps.h).
	 */
	if (config->bidirectional &&
	    (config->count > SOC_RMT_RX_CANDIDATES_PER_GROUP ||
	     2 * config->count > SOC_RMT_CHANNELS_PER_GROUP))
		return -EOPNOTSUPP;
	port->bit_ticks = dshot_bit_ticks(DSHOT_RMT_HZ, config->speed);
	port->speed = config->speed;
	port->bidirectional = config->bidirectional;
	memcpy(port->gpio, config->gpio, sizeof(port->gpio));

	for (i = 0; i < config->count; i++) {
		ret = esc_dshot_add(port, i);
		if (ret)
			break;
	}
#if SOC_RMT_SUPPORT_TX_SYNCHRO
	if (!ret && config->count > 1) {
		rmt_sync_manager_config_t sync_cfg = {
			.tx_channel_array = port->channel,
			.array_size = config->count,
		};

		ret = esc_errno(rmt_new_sync_manager(&sync_cfg, &port->sync));
	}
#endif
	if (ret) {
		/* Undo motors 0..i-1, all set up in full. */
		while (i--)
			esc_dshot_remove(port, i);
		return ret;
	}
	port->count = config->count;
	return 0;
}

int esc_dshot_busy(struct dshot_port *port)
{
	for (uint8_t i = 0; i < port->count; i++) {
		if (rmt_tx_wait_all_done(port->channel[i], 0) != ESP_OK)
			return -EBUSY;
		if (port->bidirectional && port->heard_symbols[i] < 0)
			return -EBUSY;
	}
	return 0;
}

/*
 * Stop a receive that will hear nothing: disabling an RX channel stops it
 * (IDF v5.5.5 rmt_rx.c, rmt_rx_disable()); enabling it again leaves it
 * ready for the next.
 */
static void esc_dshot_stop_listening(struct dshot_port *port, uint8_t count)
{
	for (uint8_t i = 0; i < count; i++) {
		rmt_disable(port->receiver[i]);
		rmt_enable(port->receiver[i]);
		port->heard_symbols[i] = 0;
	}
}

/*
 * A send that failed part-way: channels 0..sent-1 already hold a frame
 * that, under the sync manager, waits for the rest forever -- and keeps
 * them busy. Disabling a channel stops it and drops its transaction (IDF
 * v5.5.5 rmt_tx.c, rmt_tx_disable()); enabling it again and resetting the
 * sync manager leaves all of them as before the send. Bidirectional,
 * every receiver was armed and hears nothing now.
 */
static void esc_dshot_cancel(struct dshot_port *port, uint8_t sent)
{
	for (uint8_t i = 0; i < sent; i++) {
		rmt_disable(port->channel[i]);
		rmt_enable(port->channel[i]);
	}
#if SOC_RMT_SUPPORT_TX_SYNCHRO
	if (port->sync)
		rmt_sync_reset(port->sync);
#endif
	if (port->bidirectional)
		esc_dshot_stop_listening(port, port->count);
}

/*
 * Bidirectional, before the frame: each pin push-pull again for it, and
 * its receiver armed, to hear the frame and the reply after it and stop
 * once the line has rested DSHOT_REPLY_WAIT_US.
 */
static int esc_dshot_listen(struct dshot_port *port)
{
	const rmt_receive_config_t receive_cfg = {
		.signal_range_min_ns = DSHOT_GLITCH_NS,
		.signal_range_max_ns = DSHOT_REPLY_WAIT_US * 1000u,
	};
	int ret;

	for (uint8_t i = 0; i < port->count; i++) {
		gpio_ll_od_disable(&GPIO, (uint32_t)port->gpio[i]);
		port->heard_symbols[i] = -1;
		ret = esc_errno(rmt_receive(port->receiver[i], port->heard[i],
					    sizeof(port->heard[i]),
					    &receive_cfg));
		if (ret) {
			port->heard_symbols[i] = 0;
			esc_dshot_stop_listening(port, i);
			return ret;
		}
	}
	return 0;
}

/*
 * Sixteen symbols a motor, high for a 1's or a 0's time, then low for the
 * rest of the bit; the channel rests low after the last one (eot_level 0),
 * and bidirectional the output inverts all of it. The symbols stay in the
 * port until the next send: the copy encoder may read them after
 * rmt_transmit() has returned.
 */
int esc_dshot_send(struct dshot_port *port, const uint16_t *frames)
{
	const rmt_transmit_config_t transmit_cfg = { .loop_count = 0 };
	uint32_t one = dshot_high_ticks(port->bit_ticks, true);
	uint32_t zero = dshot_high_ticks(port->bit_ticks, false);
	int ret;

#if SOC_RMT_SUPPORT_TX_SYNCHRO
	if (port->sync) {
		ret = esc_errno(rmt_sync_reset(port->sync));
		if (ret)
			return ret;
	}
#endif
	port->unread = 0;
	if (port->bidirectional) {
		ret = esc_dshot_listen(port);
		if (ret)
			return ret;
	}
	for (uint8_t i = 0; i < port->count; i++) {
		for (int bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
			bool set = frames[i] & (0x8000u >> bit);
			uint32_t high = set ? one : zero;
			rmt_symbol_word_t *symbol = &port->symbols[i][bit];

			symbol->level0 = 1;
			symbol->duration0 = high;
			symbol->level1 = 0;
			symbol->duration1 = port->bit_ticks - high;
		}
		ret = esc_errno(rmt_transmit(port->channel[i], port->encoder[i],
					     port->symbols[i],
					     sizeof(port->symbols[i]),
					     &transmit_cfg));
		if (ret) {
			esc_dshot_cancel(port, i);
			return ret;
		}
	}
	if (port->bidirectional)
		port->unread = (uint8_t)((1u << port->count) - 1);
	return 0;
}

/*
 * What motor's receiver heard, as alternating levels in RMT symbols: the
 * frame, the line at rest longer than any level inside a reply, then the
 * reply, falling first, and the rest that stopped the receiver. A
 * duration of 0 ends the symbols.
 */
int esc_dshot_reply(struct dshot_port *port, uint8_t motor, uint32_t *raw)
{
	const rmt_symbol_word_t *heard = port->heard[motor];
	uint32_t symbols = (uint32_t)port->heard_symbols[motor];
	uint32_t value = 0, bits = 0;
	bool in_reply = false;
	int ret;

	if (!(port->unread & (1u << motor)))
		return -ENOENT;
	port->unread &= (uint8_t)~(1u << motor);
	if (symbols > sizeof(port->heard[0]) / sizeof(port->heard[0][0]))
		symbols = sizeof(port->heard[0]) / sizeof(port->heard[0][0]);
	for (uint32_t half = 0; half < 2 * symbols; half++) {
		const rmt_symbol_word_t *symbol = &heard[half / 2];
		uint32_t ticks = half % 2 ? symbol->duration1 :
					    symbol->duration0;
		bool high = half % 2 ? symbol->level1 : symbol->level0;
		uint32_t count;

		if (!ticks)
			break;
		count = dshot_reply_run_bits(ticks, DSHOT_RMT_HZ, port->speed);
		if (!in_reply) {
			in_reply = high && count > DSHOT_REPLY_RUN_MAX + 1;
			continue;
		}
		ret = dshot_reply_append(&value, &bits, high, count);
		if (ret)
			return ret;
	}
	if (!bits)
		return -ENODATA;
	*raw = dshot_reply_finish(value, bits);
	return 0;
}

void esc_dshot_detach(struct dshot_port *port)
{
#if SOC_RMT_SUPPORT_TX_SYNCHRO
	if (port->sync)
		rmt_del_sync_manager(port->sync);
#endif
	for (uint8_t i = 0; i < port->count; i++)
		esc_dshot_remove(port, i);
	port->sync = NULL;
	port->count = 0;
}
#endif /* ESC_HAVE_DSHOT */

#endif /* ESP_PLATFORM */
