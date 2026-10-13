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

#endif /* ESP_PLATFORM */
