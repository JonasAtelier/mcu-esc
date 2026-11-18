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
#include "soc/soc_caps.h"	/* SOC_RMT_*: which chips have an RMT */
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
/*
 * Undefined, the include below fails with only
 * `#include expects "FILENAME" or <FILENAME>`. Every library source that
 * needs the HAL includes this header before any model header, so one check
 * here names the cause on every build route. The HAL module checks sit in
 * its #else: without the family header no module looks enabled, and each of
 * them would blame a module instead.
 */
#ifndef CMSIS_device_header
#error "esc: CMSIS_device_header is not defined; define it as the STM32 family header, \"stm32f4xx.h\" (CMake: set(MCU_DEVICE_HEADER stm32f4xx.h))"
#else
#include CMSIS_device_header

#if (!defined(HAL_TIM_MODULE_ENABLED) || !defined(HAL_DMA_MODULE_ENABLED)) && \
    CONFIG_ESC_DSHOT
#error "the DShot driver needs the HAL TIM and DMA modules: #define HAL_TIM_MODULE_ENABLED and HAL_DMA_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_tim.c and _dma.c in the build"
#endif
#if !defined(HAL_TIM_MODULE_ENABLED) && CONFIG_ESC_C615
#error "the C615 driver needs the HAL TIM module: #define HAL_TIM_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_tim.c in the build"
#endif
#endif /* CMSIS_device_header */

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

/*
 * The DShot port: up to four outputs that each send one 16-bit frame per
 * esc_dshot_send(), together. Outside the bus layer too.
 *
 *   ESP-IDF: one RMT TX channel per motor at DSHOT_RMT_HZ, a copy encoder
 *            each, and for more than one motor a sync manager, so all start
 *            at once. Only on a chip with an RMT (SOC_RMT_SUPPORTED: not
 *            the C2), and more than one motor only where the RMT can start
 *            channels together (SOC_RMT_SUPPORT_TX_SYNCHRO: not the
 *            original ESP32). The line rests low. Bidirectional: the TX
 *            output inverted, so the line rests high, and an RMT RX channel
 *            on the same GPIO, armed before each frame; the pin turns
 *            open-drain when the frame is out, so the ESC can answer, and
 *            push-pull again before the next.
 *   HAL:     the app's timer, one bit per counter period; its update DMA
 *            burst-writes CCR1-CCR4 (DCR base CCR1, 4 transfers) from a
 *            buffer of 16 rows of four compares and one row of zeros, so
 *            the line ends low. One stream for every motor. Bidirectional
 *            (STM32F4): outputs active-low, two rows of zeros, and once
 *            they are out the same stream samples the pins' input register
 *            at three times the reply's bit rate, the pins inputs with
 *            pull-up meanwhile.
 *
 * esc_dshot_attach() checks and sets up, lines at rest; esc_dshot_busy() is
 * -EBUSY while the last frame is still going out or its reply still due;
 * esc_dshot_send() puts frames[0..count-1] out; esc_dshot_reply() hands
 * over a motor's answer to the last frame as its 21 line bits -- -ENOENT
 * when no frame has gone out since the last read, -ENODATA when the ESC
 * said nothing, -EBADMSG for levels that are no reply; esc_dshot_detach()
 * stops and releases.
 */
#if (defined(ESP_PLATFORM) && SOC_RMT_SUPPORTED) || \
    (!defined(ESP_PLATFORM) && defined(HAL_TIM_MODULE_ENABLED) && \
     defined(HAL_DMA_MODULE_ENABLED))
#define ESC_HAVE_DSHOT
#include "dshot.h"		/* struct dshot_port, placed in struct dshot_bus */

/*
 * RMT resolution, 125 ns ticks: 8 MHz divides every chip's default RMT
 * clock (RMT_CLK_SRC_DEFAULT, soc/<chip>/include/soc/clk_tree_defs.h in IDF
 * v5.5.5): the 80 MHz APB of the ESP32, S2, S3 and C3, the 80 MHz PLL of
 * the C5, C6 and P4, the H2's 32 MHz crystal -- so every channel gets
 * exactly this. The bit and its highs, against [BW]'s table
 * (docs/chips.md, "DShot"):
 *
 *   DShot150: 53 ticks = 6.625 us (6.67, -0.7 %); 1 high 40 = 5.000 us,
 *             0 high 20 = 2.500 us -- both exact
 *   DShot300: 27 ticks = 3.375 us (3.33, +1.3 %); 20 = 2.500, 10 = 1.250
 *   DShot600: 13 ticks = 1.625 us (1.67, -2.5 %); 10 = 1.250, 5 = 0.625
 *
 * The highs, which tell a 1 from a 0, come out exact; only the bit period
 * is off by the rounding above. No source gives a tolerance for it.
 */
#define DSHOT_RMT_HZ	8000000u

int esc_dshot_attach(struct dshot_port *port,
		     const struct dshot_config *config);
int esc_dshot_busy(struct dshot_port *port);
int esc_dshot_send(struct dshot_port *port, const uint16_t *frames);
int esc_dshot_reply(struct dshot_port *port, uint8_t motor, uint32_t *raw);
void esc_dshot_detach(struct dshot_port *port);
#endif

#ifdef ESC_HAVE_PWM
/* 0 when pulse_us fits in one period, else -EINVAL; touches nothing. */
int esc_pwm_check(const struct esc_pwm *pwm, uint32_t pulse_us);
int esc_pwm_start(struct esc_pwm *pwm, uint32_t pulse_us);
int esc_pwm_write_us(struct esc_pwm *pwm, uint32_t pulse_us);
void esc_pwm_stop(struct esc_pwm *pwm);
#endif

#endif /* ESC_PRIV_H */
