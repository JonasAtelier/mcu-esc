/*
 * Shared by the model drivers inside this library. Not public API.
 *
 * Two layers sit under the models, the way regmap sits under a kernel
 * driver:
 *
 *   - The bus kind is chosen at run time. A driver holds a struct esc_bus
 *     and reaches it only through bus->ops, so it can sit on I2C, SPI or a
 *     UART. Every category library offers all three kinds, used or not --
 *     and no model here is on any of them: the DJI ESCs and the HT-03 are
 *     on CAN, through mcu-can's struct can_dev, and the C615 is a PWM pulse,
 *     through the PWM port at the end of this file.
 *   - The platform is chosen at link time. esc_port_esp.c and
 *     esc_port_stm.c each define the ops tables, the attach functions and
 *     the PWM port below for their platform; the other one compiles to
 *     nothing.
 *
 * ESP_PLATFORM, which ESP-IDF defines for every component, and the HAL's own
 * module switches are defined or not, never 0 or 1, so they are the one place
 * #ifdef is right; this library's CONFIG_ESC_* options are always tested
 * with #if.
 */
#ifndef ESC_PRIV_H
#define ESC_PRIV_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The most bytes after cmd: tx on I2C and UART, tx and rx together on SPI. */
#define ESC_BUS_MAX_LEN	31

struct esc_bus;

enum esc_bus_kind {
	ESC_BUS_I2C,
	ESC_BUS_SPI,
	ESC_BUS_UART,
};

struct esc_bus_ops {
	enum esc_bus_kind kind;
	/*
	 * One transaction: cmd, then tx_len bytes of tx, then rx_len bytes
	 * into rx. cmd is the register address, with any SPI read bit already
	 * set by the chip, and the first byte of a datagram on a UART.
	 *
	 *   I2C:  rx_len ? cmd, repeated start, read : cmd and tx, one write;
	 *         a bare cmd is a one-byte write. Sending tx and then reading
	 *         is -EINVAL.
	 *   SPI:  CS low, cmd, tx, rx, CS high -- high again even when the
	 *         transfer failed.
	 *   UART: drop stale input, send cmd and tx, then wait for rx_len
	 *         bytes; fewer within the timeout is -ETIMEDOUT. For a line
	 *         where the MCU does not hear itself; see exchange().
	 *
	 * On SPI more than ESC_BUS_MAX_LEN bytes after cmd is -EINVAL; on
	 * I2C and UART only tx is bounded so, and rx on the HAL by its 16-bit
	 * count. 0 or a negative errno.
	 */
	int (*transfer)(struct esc_bus *bus, uint8_t cmd,
			const uint8_t *tx, size_t tx_len, uint8_t *rx,
			size_t rx_len);
	/*
	 * len bytes of tx out while len bytes come back into rx, rx[i] the
	 * one that answered tx[i]. More than 1 + ESC_BUS_MAX_LEN bytes, or
	 * none, is -EINVAL. NULL on I2C.
	 *
	 *   SPI:  one frame, full duplex: CS low, the bytes at the same clocks,
	 *         CS high -- high again even when the transfer failed. For a
	 *         part whose answer to one frame comes back in the next.
	 *   UART: drop stale input, send tx, and collect what the line gave
	 *         back meanwhile. On a single wire the MCU hears its own
	 *         transmission; this is how a chip takes that echo off the
	 *         line, so that the answer which follows starts clean. Fewer
	 *         than len bytes back within the timeout is -ETIMEDOUT.
	 */
	int (*exchange)(struct esc_bus *bus, const uint8_t *tx,
			uint8_t *rx, size_t len);
	/*
	 * rx_len bytes, with nothing sent and nothing dropped first: the
	 * answer that follows an exchange(). Fewer within the timeout is
	 * -ETIMEDOUT; rx is then undefined. NULL on I2C and SPI.
	 */
	int (*receive)(struct esc_bus *bus, uint8_t *rx,
		       size_t rx_len);
	/* Undo the attach. The bus itself is the app's and is left alone. */
	void (*detach)(struct esc_bus *bus);
};

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"

/* The I2C argument of every I2C attach on this platform. */
typedef i2c_master_bus_handle_t esc_i2c_bus;

/*
 * One device on one bus, as its driver holds it. ESC_HAVE_BUS
 * says the platform has at least one bus kind, so the struct exists.
 */
#define ESC_HAVE_BUS
struct esc_bus {
	const struct esc_bus_ops *ops;
	uint32_t timeout_ms;
	union {
		i2c_master_dev_handle_t i2c;
		spi_device_handle_t spi;
		uart_port_t uart;
	};
};

/*
 * esp_err_t -> negative errno, at the bus boundary and nowhere else.
 * ESP_ERR_NOT_FOUND is -ENXIO (nothing at that address), not -ENODEV
 * (something answered but it is the wrong chip).
 */
static inline int esc_errno(esp_err_t err)
{
	switch (err) {
	case ESP_OK:
		return 0;
	case ESP_ERR_INVALID_ARG:
		return -EINVAL;
	case ESP_ERR_TIMEOUT:
		return -ETIMEDOUT;
	case ESP_ERR_NOT_FOUND:
		return -ENXIO;
	case ESP_ERR_INVALID_SIZE:
		return -EMSGSIZE;
	case ESP_ERR_NO_MEM:
		return -ENOMEM;
	case ESP_ERR_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	default:
		return -EIO;
	}
}

#else /* STM32 HAL */
#include CMSIS_device_header

#if !defined(HAL_TIM_MODULE_ENABLED) && CONFIG_ESC_C615
#error "the C615 driver needs the HAL TIM module: #define HAL_TIM_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_tim.c in the build"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef *esc_i2c_bus;
#endif

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
#define ESC_HAVE_BUS
struct esc_bus {
	const struct esc_bus_ops *ops;
	uint32_t timeout_ms;
	union {
#ifdef HAL_I2C_MODULE_ENABLED
		struct {
			I2C_HandleTypeDef *handle;
			uint8_t address;	/* 7-bit */
		} i2c;
#endif
#ifdef HAL_SPI_MODULE_ENABLED
		struct {
			SPI_HandleTypeDef *handle;
			GPIO_TypeDef *cs_port;	/* app's output, idle high */
			uint16_t cs_pin;
		} spi;
#endif
#ifdef HAL_UART_MODULE_ENABLED
		UART_HandleTypeDef *uart;
#endif
	};
};
#endif

/*
 * HAL_StatusTypeDef -> negative errno, at the bus boundary and nowhere else.
 * HAL_BUSY -- the bus or the handle in use, or a handle never initialised --
 * is -EAGAIN. A NACK in the middle of a transfer is HAL_ERROR like any other
 * fault, so -EIO.
 */
static inline int esc_hal_errno(HAL_StatusTypeDef status)
{
	switch (status) {
	case HAL_OK:
		return 0;
	case HAL_TIMEOUT:
		return -ETIMEDOUT;
	case HAL_BUSY:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

#endif /* STM32 HAL */

#ifdef ESC_HAVE_BUS
/* bus->ops->exchange(), or -EOPNOTSUPP on a kind that has none. */
static inline int esc_bus_exchange(struct esc_bus *bus,
				   const uint8_t *tx, uint8_t *rx,
				   size_t len)
{
	if (!bus->ops->exchange)
		return -EOPNOTSUPP;
	return bus->ops->exchange(bus, tx, rx, len);
}

/* bus->ops->receive(), or -EOPNOTSUPP on a kind that has none. */
static inline int esc_bus_receive(struct esc_bus *bus,
				  uint8_t *rx,
				  size_t rx_len)
{
	if (!bus->ops->receive)
		return -EOPNOTSUPP;
	return bus->ops->receive(bus, rx, rx_len);
}

static inline void esc_bus_detach(struct esc_bus *bus)
{
	bus->ops->detach(bus);
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
/*
 * Is anything at this 7-bit address? -ENXIO when nothing ACKs -- nothing at
 * that address -- not -ENODEV, which means something answered but it is the
 * wrong chip. A NACK in the middle of a transfer is a plain -EIO on both
 * platforms, which is why a driver decides presence here, never by its
 * first transfer.
 */
int esc_i2c_probe(esc_i2c_bus handle, uint8_t address,
		  uint32_t timeout_ms);

/*
 * Bind bus to the I2C device at address. On ESP-IDF the driver adds the
 * device at scl_hz, 0 for 400 kHz; on the HAL the clock is the handle's and
 * scl_hz is ignored. Undo with esc_bus_detach(). Every attach leaves
 * bus->ops NULL when it fails, so a bus with ops is one with something to
 * detach.
 */
int esc_i2c_attach(struct esc_bus *bus,
		   esc_i2c_bus handle,
		   uint8_t address, uint32_t scl_hz, uint32_t timeout_ms);
#endif

#ifdef ESP_PLATFORM
/*
 * Add the SPI device on host's cs_pin, in SPI mode 0-3 at clock_hz. The
 * host is the app's, already spi_bus_initialize()d.
 */
int esc_spi_attach(struct esc_bus *bus,
		   spi_host_device_t host,
		   int cs_pin, uint32_t clock_hz, uint8_t mode);

/* port's driver is the app's, already installed and configured. */
int esc_uart_attach(struct esc_bus *bus, uart_port_t port,
		    uint32_t timeout_ms);
#else
#ifdef HAL_SPI_MODULE_ENABLED
/*
 * Bind bus to the SPI device behind cs_port/cs_pin, and deselect it, so the
 * first transfer starts on a falling edge. Mode, clock and frame are the
 * handle's; the chip select is an output the app has set up.
 */
int esc_spi_attach(struct esc_bus *bus,
		   SPI_HandleTypeDef *handle,
		   GPIO_TypeDef *cs_port, uint16_t cs_pin,
		   uint32_t timeout_ms);
#endif
#ifdef HAL_UART_MODULE_ENABLED
/* handle is the app's, already HAL_UART_Init()ed. */
int esc_uart_attach(struct esc_bus *bus,
		    UART_HandleTypeDef *handle,
		    uint32_t timeout_ms);
#endif
#endif

/*
 * The PWM port: one output pin that repeats a pulse of pulse_us at rate_hz,
 * for a driver commanded by pulse width. Outside the bus layer: no bytes
 * cross it.
 *
 *   ESP-IDF: an LEDC channel in low-speed mode, which every ESP32 has, on a
 *            timer the attach configures at rate_hz with 14-bit duty, the
 *            widest the ESP32-S2, -S3 and -C3 take. A timer may be shared
 *            by channels at one rate.
 *   HAL:     a TIM channel the app has set up in PWM mode. The rate is the
 *            app's: tick_hz, the counter clock after the prescaler, over
 *            ARR + 1. The attach reads it back into rate_hz.
 *
 * esc_pwm_attach() binds without driving the pin; esc_pwm_start() then puts
 * out pulse_us; esc_pwm_write_us() changes it, from the next period on;
 * esc_pwm_stop() holds the pin low, no pulses at all. A pulse that does not
 * fit inside one period is -EINVAL.
 */
#if defined(ESP_PLATFORM) || defined(HAL_TIM_MODULE_ENABLED)
#define ESC_HAVE_PWM
#include "c615.h"		/* struct esc_pwm, placed in struct c615 */

#ifdef ESP_PLATFORM
#define ESC_PWM_DUTY_BITS	14

int esc_pwm_attach(struct esc_pwm *pwm, int gpio, ledc_timer_t timer,
		   ledc_channel_t channel, uint32_t rate_hz);
#else
int esc_pwm_attach(struct esc_pwm *pwm, TIM_HandleTypeDef *tim,
		   uint32_t channel, uint32_t tick_hz);
#endif
#endif

#ifdef ESC_HAVE_PWM
/* 0 when pulse_us fits in one period, else -EINVAL; touches nothing. */
int esc_pwm_check(const struct esc_pwm *pwm, uint32_t pulse_us);
int esc_pwm_start(struct esc_pwm *pwm, uint32_t pulse_us);
int esc_pwm_write_us(struct esc_pwm *pwm, uint32_t pulse_us);
void esc_pwm_stop(struct esc_pwm *pwm);
#endif

#endif /* ESC_PRIV_H */
