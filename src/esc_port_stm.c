/*
 * esc_priv.h on the STM32 HAL: the bus layer and the PWM port. stm-forge
 * compiles every .c in src/, so under ESP-IDF this file is empty; the
 * ESP-IDF CMakeLists never lists it.
 */
#ifndef ESP_PLATFORM

#include <string.h>

#include "esc.h"
#include "esc_priv.h"

#ifdef ESC_HAVE_DSHOT
#include "dshot/dshot_math.h"
#endif

/*
 * The core is always built; each model's driver only under its own option.
 * With the library built and no model picked, say so: this file is compiled
 * only when the library is, so this is the CMakeLists' "no model" warning
 * for every build that never reads that file (stm-forge, a hand-made
 * project).
 */
#if !(CONFIG_ESC_C610 || CONFIG_ESC_C620 || CONFIG_ESC_GM6020 || \
      CONFIG_ESC_C615 || CONFIG_ESC_HT03 || CONFIG_ESC_DSHOT)
#warning "esc: no model is selected; define CONFIG_ESC_<MODEL>=1 (stm-forge: ESC_<MODEL> = y)"
#endif

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
/* Nothing to undo: the handle is the app's, and nothing was added to it. */
static void esc_hal_detach(struct esc_bus *bus)
{
	(void)bus;
}
#endif

#ifdef HAL_I2C_MODULE_ENABLED
/*
 * HAL_I2C_IsDeviceReady() returns HAL_ERROR when no trial was ACKed, which is
 * -ENXIO. A busy or stuck bus keeps its own errno. On the F4, no pull-ups or
 * SDA held low sets the peripheral's BUSY flag, so the HAL gives up after
 * 25 ms with HAL_BUSY, -EAGAIN; HAL_TIMEOUT, -ETIMEDOUT, means a START never
 * completed.
 */
int esc_i2c_probe(esc_i2c_bus handle, uint8_t address,
		  uint32_t timeout_ms)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(handle, (uint16_t)(address << 1), 1,
				       timeout_ms);
	return status == HAL_ERROR ? -ENXIO : esc_hal_errno(status);
}

/* HAL_I2C_Mem_Read(): cmd, repeated start, burst. Mem_Write(): cmd, data. */
static int esc_i2c_transfer(struct esc_bus *bus,
			    uint8_t cmd,
			    const uint8_t *tx, size_t tx_len, uint8_t *rx,
			    size_t rx_len)
{
	uint16_t address = (uint16_t)(bus->i2c.address << 1);

	if ((tx_len && rx_len) || tx_len > ESC_BUS_MAX_LEN ||
	    rx_len > UINT16_MAX)
		return -EINVAL;
	if (rx_len)
		return esc_hal_errno(HAL_I2C_Mem_Read(bus->i2c.handle,
						      address, cmd,
						      I2C_MEMADD_SIZE_8BIT,
						      rx, (uint16_t)rx_len,
						      bus->timeout_ms));
	if (tx_len)
		return esc_hal_errno(HAL_I2C_Mem_Write(bus->i2c.handle,
						       address, cmd,
						       I2C_MEMADD_SIZE_8BIT,
						       (uint8_t *)tx,
						       (uint16_t)tx_len,
						       bus->timeout_ms));
	/* A bare command, for a part with no registers: its own START, STOP. */
	return esc_hal_errno(HAL_I2C_Master_Transmit(bus->i2c.handle,
						     address, &cmd, 1,
						     bus->timeout_ms));
}

static const struct esc_bus_ops esc_i2c_ops = {
	.kind = ESC_BUS_I2C,
	.transfer = esc_i2c_transfer,
	.detach = esc_hal_detach,
};

int esc_i2c_attach(struct esc_bus *bus,
		   esc_i2c_bus handle,
		   uint8_t address, uint32_t scl_hz, uint32_t timeout_ms)
{
	(void)scl_hz;
	bus->ops = &esc_i2c_ops;
	bus->timeout_ms = timeout_ms;
	bus->i2c.handle = handle;
	bus->i2c.address = address;
	return 0;
}
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
/*
 * One transaction under a software chip select: CS low, cmd and tx out --
 * full duplex with zeros after them when there is something to read -- CS
 * high. CS goes high even when the transfer failed; see esc_bus_ops.
 */
static int esc_spi_transfer(struct esc_bus *bus,
			    uint8_t cmd,
			    const uint8_t *tx, size_t tx_len, uint8_t *rx,
			    size_t rx_len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN] = { 0 };
	uint8_t in[1 + ESC_BUS_MAX_LEN];
	uint16_t n = (uint16_t)(1 + tx_len + rx_len);
	HAL_StatusTypeDef status;

	if (tx_len + rx_len > ESC_BUS_MAX_LEN)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_RESET);
	if (rx_len)
		status = HAL_SPI_TransmitReceive(bus->spi.handle, out, in, n,
						 bus->timeout_ms);
	else
		status = HAL_SPI_Transmit(bus->spi.handle, out, n,
					  bus->timeout_ms);
	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_SET);
	if (status == HAL_OK && rx_len)
		memcpy(rx, in + 1 + tx_len, rx_len);
	return esc_hal_errno(status);
}

/* One frame, full duplex: rx[i] is what came back while tx[i] went out. */
static int esc_spi_exchange(struct esc_bus *bus,
			    const uint8_t *tx,
			    uint8_t *rx, size_t len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN];
	uint8_t in[1 + ESC_BUS_MAX_LEN];
	HAL_StatusTypeDef status;

	if (!len || len > sizeof(out))
		return -EINVAL;
	memcpy(out, tx, len);

	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(bus->spi.handle, out, in,
					 (uint16_t)len, bus->timeout_ms);
	HAL_GPIO_WritePin(bus->spi.cs_port, bus->spi.cs_pin, GPIO_PIN_SET);
	if (status == HAL_OK)
		memcpy(rx, in, len);
	return esc_hal_errno(status);
}

static const struct esc_bus_ops esc_spi_ops = {
	.kind = ESC_BUS_SPI,
	.transfer = esc_spi_transfer,
	.exchange = esc_spi_exchange,
	.detach = esc_hal_detach,
};

int esc_spi_attach(struct esc_bus *bus,
		   SPI_HandleTypeDef *handle,
		   GPIO_TypeDef *cs_port, uint16_t cs_pin,
		   uint32_t timeout_ms)
{
	bus->ops = &esc_spi_ops;
	bus->timeout_ms = timeout_ms;
	bus->spi.handle = handle;
	bus->spi.cs_port = cs_port;
	bus->spi.cs_pin = cs_pin;
	/* Deselected first, so the first transfer starts on a falling edge. */
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	return 0;
}
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
/*
 * Blocking HAL_UART_Receive() hands back whatever already sits in the data
 * register, so a late byte from an earlier exchange would be read as the
 * start of this reply. Drop it first: clear an overrun, then read the data
 * register empty (on the F7 that macro is a flush request). Bounded by the
 * timeout, in case the line never goes quiet.
 */
static void esc_uart_drop_input(struct esc_bus *bus)
{
	UART_HandleTypeDef *handle = bus->uart;
	uint32_t start = HAL_GetTick();

	__HAL_UART_CLEAR_OREFLAG(handle);
	while (__HAL_UART_GET_FLAG(handle, UART_FLAG_RXNE) &&
	       (uint32_t)(HAL_GetTick() - start) <= bus->timeout_ms)
		__HAL_UART_FLUSH_DRREGISTER(handle);
}

static int esc_uart_transfer(struct esc_bus *bus,
			     uint8_t cmd,
			     const uint8_t *tx, size_t tx_len,
			     uint8_t *rx, size_t rx_len)
{
	uint8_t out[1 + ESC_BUS_MAX_LEN];
	int ret;

	if (tx_len > ESC_BUS_MAX_LEN || rx_len > UINT16_MAX)
		return -EINVAL;
	out[0] = cmd;
	if (tx_len)
		memcpy(out + 1, tx, tx_len);

	esc_uart_drop_input(bus);
	ret = esc_hal_errno(HAL_UART_Transmit(bus->uart, out,
					      (uint16_t)(1 + tx_len),
					      bus->timeout_ms));
	if (ret || !rx_len)
		return ret;
	/* A reply that stops short runs into the timeout: HAL_TIMEOUT. */
	return esc_hal_errno(HAL_UART_Receive(bus->uart, rx,
					      (uint16_t)rx_len,
					      bus->timeout_ms));
}

/*
 * The echo, a byte at a time. Blocking HAL_UART_Transmit() of a whole
 * datagram never reads the line while it sends, and the data register holds
 * one byte: on a single wire the second echoed byte overruns the first
 * (ORE), the rest are lost, and the reply that follows comes up misaligned.
 * So send one byte and take its echo before the next goes out; only one
 * byte is ever in flight, and nothing overruns.
 *
 * The other way, a half-duplex USART (HDSEL) that leaves the receiver deaf
 * while it sends, needs the app to set the handle up so and the driver to
 * switch direction around each datagram -- a different contract with the
 * app, and nothing this library can check from a handle.
 */
static int esc_uart_exchange(struct esc_bus *bus,
			     const uint8_t *tx,
			     uint8_t *rx, size_t len)
{
	size_t i;
	int ret;

	if (!len || len > 1 + ESC_BUS_MAX_LEN)
		return -EINVAL;

	esc_uart_drop_input(bus);
	for (i = 0; i < len; i++) {
		uint8_t byte = tx[i];

		ret = esc_hal_errno(HAL_UART_Transmit(bus->uart,
						      &byte, 1,
						      bus->timeout_ms));
		if (ret)
			return ret;
		ret = esc_hal_errno(HAL_UART_Receive(bus->uart,
						     &rx[i], 1,
						     bus->timeout_ms));
		if (ret)
			return ret;
	}
	return 0;
}

static int esc_uart_receive(struct esc_bus *bus,
			    uint8_t *rx,
			    size_t rx_len)
{
	if (rx_len > UINT16_MAX)
		return -EINVAL;
	return esc_hal_errno(HAL_UART_Receive(bus->uart, rx,
					      (uint16_t)rx_len,
					      bus->timeout_ms));
}

static const struct esc_bus_ops esc_uart_ops = {
	.kind = ESC_BUS_UART,
	.transfer = esc_uart_transfer,
	.exchange = esc_uart_exchange,
	.receive = esc_uart_receive,
	.detach = esc_hal_detach,
};

int esc_uart_attach(struct esc_bus *bus,
		    UART_HandleTypeDef *handle,
		    uint32_t timeout_ms)
{
	bus->ops = &esc_uart_ops;
	bus->timeout_ms = timeout_ms;
	bus->uart = handle;
	return 0;
}
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
/* --------------------------------------------------------------- PWM -- */

/*
 * pulse_us in timer counts, rounded to the nearest. A compare past ARR would
 * hold the pin high the whole period: -EINVAL, as is a pulse that leaves no
 * low time at all.
 */
static int esc_pwm_compare(const struct esc_pwm *pwm, uint32_t pulse_us,
			   uint32_t *compare)
{
	uint64_t counts = ((uint64_t)pulse_us * pwm->tick_hz + 500000u) /
			  1000000u;

	if (counts > __HAL_TIM_GET_AUTORELOAD(pwm->tim))
		return -EINVAL;
	*compare = (uint32_t)counts;
	return 0;
}

int esc_pwm_check(const struct esc_pwm *pwm, uint32_t pulse_us)
{
	uint32_t compare;

	return esc_pwm_compare(pwm, pulse_us, &compare);
}

int esc_pwm_attach(struct esc_pwm *pwm, TIM_HandleTypeDef *tim,
		   uint32_t channel, uint32_t tick_hz)
{
	uint32_t period;

	if (!tim || !tick_hz || (channel != TIM_CHANNEL_1 &&
	    channel != TIM_CHANNEL_2 && channel != TIM_CHANNEL_3 &&
				 channel != TIM_CHANNEL_4))
		return -EINVAL;
	period = __HAL_TIM_GET_AUTORELOAD(tim) + 1u;
	/* ARR 0xFFFFFFFF on a 32-bit timer wraps the period to 0. */
	if (!period)
		return -EINVAL;
	pwm->tim = tim;
	pwm->channel = channel;
	pwm->tick_hz = tick_hz;
	pwm->rate_hz = tick_hz / period;
	return 0;
}

int esc_pwm_start(struct esc_pwm *pwm, uint32_t pulse_us)
{
	uint32_t compare;
	int ret;

	ret = esc_pwm_compare(pwm, pulse_us, &compare);
	if (ret)
		return ret;
	__HAL_TIM_SET_COMPARE(pwm->tim, pwm->channel, compare);
	return esc_hal_errno(HAL_TIM_PWM_Start(pwm->tim, pwm->channel));
}

int esc_pwm_write_us(struct esc_pwm *pwm, uint32_t pulse_us)
{
	uint32_t compare;
	int ret;

	ret = esc_pwm_compare(pwm, pulse_us, &compare);
	if (ret)
		return ret;
	__HAL_TIM_SET_COMPARE(pwm->tim, pwm->channel, compare);
	return 0;
}

/*
 * Compare 0 first: in PWM mode 1 the pin is then low the whole period, so
 * the last pulse is not left on the line whatever the stopped channel's idle
 * state is.
 */
void esc_pwm_stop(struct esc_pwm *pwm)
{
	__HAL_TIM_SET_COMPARE(pwm->tim, pwm->channel, 0);
	HAL_TIM_PWM_Stop(pwm->tim, pwm->channel);
}
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef ESC_HAVE_DSHOT
/* ------------------------------------------------------------- DShot -- */

#define DSHOT_BURST_ROWS	(DSHOT_FRAME_BITS + 1)

/*
 * Bidirectional DShot on the STM32F4 only: the update DMA reads the pins'
 * port, which takes DMA2 -- only its peripheral port reaches the bus
 * matrix and so the AHB1 GPIO (RM0090 2.1.5, p.52; Figure 25 note 1,
 * p.214) -- and RM0090 is the manual this was checked against. Elsewhere
 * it is -EOPNOTSUPP.
 */
#if defined(STM32F4)
#define ESC_HAVE_DSHOT_BIDIR
#endif

#ifdef ESC_HAVE_DSHOT_BIDIR
/*
 * Two rows of zeros: the transfer-complete interrupt comes when the last
 * row has been moved, one period before it is put out (see
 * esc_dshot_send()), so with two the line is at rest by then.
 */
#define DSHOT_BIDIR_BURST_ROWS	(DSHOT_FRAME_BITS + 2)

enum {
	DSHOT_IDLE,
	DSHOT_SENDING,		/* the burst is going out */
	DSHOT_SAMPLING,		/* the DMA reads the port */
};

/* GPIOx_MODER and GPIOx_PUPDR, two bits a pin (RM0090 7.4.1, p.198;
 * 7.4.4, p.200). */
#define DSHOT_MODER_INPUT	0u
#define DSHOT_MODER_ALTERNATE	2u
#define DSHOT_PUPDR_PULL_UP	1u

/* GPIO_PIN_x to its number 0-15, or -1 for not one pin. */
static int esc_dshot_pin_number(uint16_t pin)
{
	for (int n = 0; n < 16; n++)
		if (pin == (1u << n))
			return n;
	return -1;
}

/* Every motor's pin to mode, DSHOT_MODER_*. In the DMA's interrupt too. */
static void esc_dshot_pins(struct dshot_port *port, uint32_t mode)
{
	uint32_t moder = port->gpio->MODER;

	for (uint8_t i = 0; i < port->count; i++)
		moder = (moder & ~(3u << (2 * port->pin[i]))) |
			(mode << (2 * port->pin[i]));
	port->gpio->MODER = moder;
}

/*
 * The timer to a counter period of ticks, at once: UG reloads the counter
 * and ARR (RM0090 14.4.6, p.399), and with UDE off asks the DMA nothing.
 */
static void esc_dshot_period(struct dshot_port *port, uint32_t ticks)
{
	__HAL_TIM_DISABLE_DMA(port->tim, TIM_DMA_UPDATE);
	port->tim->Instance->ARR = ticks - 1u;
	port->tim->Instance->EGR = TIM_EGR_UG;
}

/* The DMA's settings: word to the timer for a frame, half-word from the
 * port for samples (GPIO registers take either, RM0090 7.4, p.198). */
static HAL_StatusTypeDef esc_dshot_dma_mode(DMA_HandleTypeDef *dma,
					    bool sampling)
{
	dma->Init.Direction = sampling ? DMA_PERIPH_TO_MEMORY :
					 DMA_MEMORY_TO_PERIPH;
	dma->Init.PeriphDataAlignment = sampling ? DMA_PDATAALIGN_HALFWORD :
						   DMA_PDATAALIGN_WORD;
	dma->Init.MemDataAlignment = sampling ? DMA_MDATAALIGN_HALFWORD :
						DMA_MDATAALIGN_WORD;
	return HAL_DMA_Init(dma);
}

/*
 * Back to sending: compares 0, so the line rests even when a frame was cut
 * short (UG below makes them active at once), bit period, pins to the
 * timer, DMA to the timer. From the DMA's interrupt, or after a failure; a
 * DMA init that fails here makes the next send fail.
 */
static void esc_dshot_rest(struct dshot_port *port)
{
	for (uint8_t i = 0; i < port->count; i++)
		__HAL_TIM_SET_COMPARE(port->tim, port->channel[i], 0);
	esc_dshot_period(port, port->bit_ticks);
	esc_dshot_pins(port, DSHOT_MODER_ALTERNATE);
	esc_dshot_dma_mode(port->tim->hdma[TIM_DMA_ID_UPDATE], false);
	port->phase = DSHOT_IDLE;
}

/* The samples are in. */
static void esc_dshot_sampled(DMA_HandleTypeDef *dma)
{
	struct dshot_port *port = dma->Parent;

	port->sampled = true;
	esc_dshot_rest(port);
}

/*
 * A DMA error, sending or sampling. Only a transfer error stops the
 * stream (stm32f4xx_hal_dma.c, HAL_DMA_IRQHandler(): FIFO and direct-mode
 * errors leave it running); then no reply this time. Anything else is
 * left to finish as it was.
 */
static void esc_dshot_dma_failed(DMA_HandleTypeDef *dma)
{
	if (!(dma->ErrorCode & HAL_DMA_ERROR_TE))
		return;
	esc_dshot_rest(dma->Parent);
}

/*
 * The frame is out and the line at rest: let it go for the ESC's reply
 * [BW "Bidirectional DSHOT"] -- pins to inputs, held high by their
 * pull-ups -- and read the port every sample_ticks into samples[], by the
 * same update DMA. In the DMA's interrupt; the reply starts 30 us later.
 */
static void esc_dshot_frame_out(DMA_HandleTypeDef *dma)
{
	struct dshot_port *port = dma->Parent;

	esc_dshot_period(port, port->sample_ticks);
	esc_dshot_pins(port, DSHOT_MODER_INPUT);
	dma->XferCpltCallback = esc_dshot_sampled;
	port->phase = DSHOT_SAMPLING;
	if (esc_dshot_dma_mode(dma, true) != HAL_OK ||
	    HAL_DMA_Start_IT(dma, (uint32_t)(uintptr_t)&port->gpio->IDR,
			     (uint32_t)(uintptr_t)port->samples,
			     port->sample_count) != HAL_OK) {
		esc_dshot_rest(port);
		return;
	}
	__HAL_TIM_ENABLE_DMA(port->tim, TIM_DMA_UPDATE);
}

/*
 * Bidirectional setup: the pins, each once and on one port, with
 * pull-ups; the channels active-low, so compare 0 holds the line high
 * (CCxP, RM0090 14.4.9, p.405; PWM mode 1, 14.4.7, p.402); the sample
 * period, 1/3 of a reply bit, and how many samples cover
 * DSHOT_REPLY_WAIT_US and a reply and one bit more; and the DMA handle's
 * Parent, which its callbacks find the port by.
 */
static int esc_dshot_bidir_attach(struct dshot_port *port,
				  const struct dshot_config *config)
{
	const uint32_t per_sample = DSHOT_REPLY_RATE_NUM * DSHOT_REPLY_OVERSAMPLE;
	uint16_t used = 0;
	uint64_t window;

	/*
	 * The GPIO is on AHB1, which only DMA2's peripheral port reaches
	 * (RM0090 2.1.5, p.52; Figure 25 note 1, p.214): DMA1, at 0x4002 6000,
	 * lies below DMA2, at 0x4002 6400 (Table 2, p.53). TIM1 and TIM8 have
	 * their update request on DMA2 (Table 34, p.217), TIM2-5 on DMA1 (Table
	 * 33, p.216).
	 */
	if ((uintptr_t)port->tim->hdma[TIM_DMA_ID_UPDATE]->Instance <
	    (uintptr_t)DMA2_Stream0)
		return -EOPNOTSUPP;
	if (!config->gpio)
		return -EINVAL;
	for (uint8_t i = 0; i < config->count; i++) {
		int n = esc_dshot_pin_number(config->pin[i]);

		if (n < 0 || (used & config->pin[i]))
			return -EINVAL;
		used |= config->pin[i];
		port->pin[i] = (uint8_t)n;
	}
	port->sample_ticks = (port->bit_ticks * DSHOT_REPLY_RATE_DEN +
			      per_sample / 2) / per_sample;
	window = (uint64_t)config->tick_hz * DSHOT_REPLY_WAIT_US / 1000000u +
		 (uint64_t)(DSHOT_REPLY_BITS + 1) * port->bit_ticks *
		 DSHOT_REPLY_RATE_DEN / DSHOT_REPLY_RATE_NUM;
	if (!port->sample_ticks)
		return -EINVAL;
	window = (window + port->sample_ticks - 1) / port->sample_ticks;
	if (window > sizeof(port->samples) / sizeof(port->samples[0]))
		return -EINVAL;
	port->sample_count = (uint16_t)window;
	port->gpio = config->gpio;
	port->tick_hz = config->tick_hz;

	for (uint8_t i = 0; i < config->count; i++) {
		uint32_t shift = 2u * port->pin[i];

		port->gpio->PUPDR = (port->gpio->PUPDR & ~(3u << shift)) |
				    (DSHOT_PUPDR_PULL_UP << shift);
		port->tim->Instance->CCER |= TIM_CCER_CC1P << (4 * port->slot[i]);
	}
	port->tim->hdma[TIM_DMA_ID_UPDATE]->Parent = port;
	port->phase = DSHOT_IDLE;
	return 0;
}

/*
 * Undo the bidirectional setup, whatever is under way stopped first: the
 * timer, pins and DMA as the app set them -- but for the pull-ups, which
 * stay -- and the handle's Parent the timer again, as the HAL links it.
 */
static void esc_dshot_bidir_detach(struct dshot_port *port)
{
	DMA_HandleTypeDef *dma = port->tim->hdma[TIM_DMA_ID_UPDATE];

	if (!port->gpio)
		return;
	__HAL_TIM_DISABLE_DMA(port->tim, TIM_DMA_UPDATE);
	HAL_DMA_Abort(dma);
	esc_dshot_rest(port);
	for (uint8_t i = 0; i < port->count; i++)
		port->tim->Instance->CCER &=
			~(TIM_CCER_CC1P << (4 * port->slot[i]));
	dma->XferCpltCallback = NULL;
	dma->XferErrorCallback = NULL;
	dma->Parent = port->tim;
	port->gpio = NULL;
	port->bidirectional = false;
}
#endif /* ESC_HAVE_DSHOT_BIDIR */

/* TIM_CHANNEL_1..4 as CCR1..4's place in a burst row, or -1. */
static int esc_dshot_slot(uint32_t channel)
{
	switch (channel) {
	case TIM_CHANNEL_1:
		return 0;
	case TIM_CHANNEL_2:
		return 1;
	case TIM_CHANNEL_3:
		return 2;
	case TIM_CHANNEL_4:
		return 3;
	default:
		return -1;
	}
}

/*
 * The update DMA as the burst needs it: memory to the timer's DMAR, word
 * by word, the memory side stepping, once through the buffer.
 */
static bool esc_dshot_dma_fits(const DMA_HandleTypeDef *dma)
{
	return dma && dma->Init.Direction == DMA_MEMORY_TO_PERIPH &&
	       dma->Init.PeriphInc == DMA_PINC_DISABLE &&
	       dma->Init.MemInc == DMA_MINC_ENABLE &&
	       dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_WORD &&
	       dma->Init.MemDataAlignment == DMA_MDATAALIGN_WORD &&
	       dma->Init.Mode == DMA_NORMAL;
}

int esc_dshot_attach(struct dshot_port *port,
		     const struct dshot_config *config)
{
	TIM_HandleTypeDef *tim = config->tim;
	uint8_t used = 0;
	uint8_t i;
	int ret = 0;

	if (!tim || !config->tick_hz ||
	    !esc_dshot_dma_fits(tim->hdma[TIM_DMA_ID_UPDATE]))
		return -EINVAL;
	for (i = 0; i < config->count; i++) {
		int slot = esc_dshot_slot(config->channel[i]);

		if (slot < 0 || (used & (1u << slot)))
			return -EINVAL;
		used |= (uint8_t)(1u << slot);
		port->slot[i] = (uint8_t)slot;
		port->channel[i] = config->channel[i];
	}
	/* One counter period is one bit, exactly as the app was told. */
	port->bit_ticks = __HAL_TIM_GET_AUTORELOAD(tim) + 1u;
	if (port->bit_ticks != dshot_bit_ticks(config->tick_hz, config->speed))
		return -EINVAL;
	port->speed = config->speed;
	port->tim = tim;
	port->count = config->count;
	if (config->bidirectional) {
#ifdef ESC_HAVE_DSHOT_BIDIR
		ret = esc_dshot_bidir_attach(port, config);
#else
		ret = -EOPNOTSUPP;
#endif
		if (ret) {
			port->count = 0;
			return ret;
		}
		port->bidirectional = true;
	}

	for (i = 0; i < config->count; i++) {
		__HAL_TIM_SET_COMPARE(tim, config->channel[i], 0);
		ret = esc_hal_errno(HAL_TIM_PWM_Start(tim, config->channel[i]));
		if (ret)
			break;
	}
	if (ret) {
		while (i--)
			HAL_TIM_PWM_Stop(tim, config->channel[i]);
#ifdef ESC_HAVE_DSHOT_BIDIR
		esc_dshot_bidir_detach(port);
#endif
		port->count = 0;
		return ret;
	}
	return 0;
}

/*
 * The DMA is busy until its transfer-complete interrupt, which is why the
 * app's IRQ handler has to call HAL_DMA_IRQHandler().
 */
int esc_dshot_busy(struct dshot_port *port)
{
#ifdef ESC_HAVE_DSHOT_BIDIR
	if (port->bidirectional)
		return port->phase == DSHOT_IDLE ? 0 : -EBUSY;
#endif
	if (HAL_DMA_GetState(port->tim->hdma[TIM_DMA_ID_UPDATE]) ==
	    HAL_DMA_STATE_BUSY)
		return -EBUSY;
	return 0;
}

#ifdef ESC_HAVE_DSHOT_BIDIR
/*
 * The same burst as below, but started on the DMA directly: the HAL's
 * burst call would take the transfer-complete callback, and here that
 * callback turns the line round (esc_dshot_frame_out()). DCR as below.
 */
static int esc_dshot_send_bidir(struct dshot_port *port)
{
	DMA_HandleTypeDef *dma = port->tim->hdma[TIM_DMA_ID_UPDATE];
	int ret;

	port->unread = 0;
	port->sampled = false;
	dma->XferCpltCallback = esc_dshot_frame_out;
	dma->XferHalfCpltCallback = NULL;
	dma->XferErrorCallback = esc_dshot_dma_failed;
	port->tim->Instance->DCR = TIM_DMABASE_CCR1 |
				   TIM_DMABURSTLENGTH_4TRANSFERS;
	port->phase = DSHOT_SENDING;
	ret = esc_hal_errno(HAL_DMA_Start_IT(
		dma, (uint32_t)(uintptr_t)port->burst,
		(uint32_t)(uintptr_t)&port->tim->Instance->DMAR,
		DSHOT_BIDIR_BURST_ROWS * 4));
	if (ret) {
		port->phase = DSHOT_IDLE;
		return ret;
	}
	__HAL_TIM_ENABLE_DMA(port->tim, TIM_DMA_UPDATE);
	port->unread = (uint8_t)((1u << port->count) - 1);
	return 0;
}

/*
 * The samples of motor's pin: high while the line rests after the frame,
 * then the reply, falling first; each level's length in samples is
 * sample_ticks apiece.
 */
int esc_dshot_reply(struct dshot_port *port, uint8_t motor, uint32_t *raw)
{
	uint16_t mask = (uint16_t)(1u << port->pin[motor]);
	uint32_t value = 0, bits = 0, run = 0;
	bool in_reply = false, level = false;
	int ret;

	if (!(port->unread & (1u << motor)))
		return -ENOENT;
	port->unread &= (uint8_t)~(1u << motor);
	if (!port->sampled)
		return -ENODATA;
	for (uint16_t n = 0; n < port->sample_count; n++) {
		bool high = port->samples[n] & mask;

		if (!in_reply) {
			in_reply = !high;
			run = in_reply;
			continue;
		}
		if (high == level) {
			run++;
			continue;
		}
		ret = dshot_reply_append(&value, &bits, level,
					 dshot_reply_run_bits(
						 run * port->sample_ticks,
						 port->tick_hz, port->speed));
		if (ret)
			return ret;
		level = high;
		run = 1;
	}
	if (!in_reply)
		return -ENODATA;
	ret = dshot_reply_append(&value, &bits, level,
				 dshot_reply_run_bits(run * port->sample_ticks,
						      port->tick_hz,
						      port->speed));
	if (ret)
		return ret;
	*raw = dshot_reply_finish(value, bits);
	return 0;
}
#else
int esc_dshot_reply(struct dshot_port *port, uint8_t motor, uint32_t *raw)
{
	(void)port;
	(void)motor;
	(void)raw;
	return -ENOENT;
}
#endif

/*
 * Row b holds bit b's high time for every motor's CCR, the last row zeros:
 * dshot_init() zeroed the port, and nothing writes that row or an unused
 * channel's place.
 *
 * The timer's DMA burst, RM0090 Rev 3 (docs/rm0090.pdf, gitignored):
 *   - DCR.DBA is the first register as a word offset from CR1, DCR.DBL the
 *     number of registers less one (14.4.19, p.414). CCR1-CCR4 sit at
 *     0x34-0x40 (Table 72, p.417): DBA 0x34 / 4 = 13 = TIM_DMABASE_CCR1,
 *     4 registers = DBL 3 = TIM_DMABURSTLENGTH_4TRANSFERS.
 *   - Each update DMA request moves DBL + 1 words through DMAR into
 *     CCR1-CCR4 (14.4.20, p.415), and the DMA's count is all of them: rows
 *     times four, its example's "6" for two updates of three registers.
 *   - The registers take half-word or word writes (14.4, p.390): the DMA
 *     goes word to word.
 *   - With OCxPE set, a compare written now becomes active at the next
 *     update event (14.4.7, p.402): row b is put out in the period after
 *     the update that moved it, the zeros after bit 15, so the line ends
 *     low.
 *
 * The burst state of the last send is cleared first: the HAL leaves it busy
 * after completion (stm32f4xx_hal_tim.c, TIM_DMAPeriodElapsedCplt()).
 */
int esc_dshot_send(struct dshot_port *port, const uint16_t *frames)
{
	uint32_t one = dshot_high_ticks(port->bit_ticks, true);
	uint32_t zero = dshot_high_ticks(port->bit_ticks, false);

	for (int bit = 0; bit < DSHOT_FRAME_BITS; bit++)
		for (uint8_t i = 0; i < port->count; i++)
			port->burst[bit * 4 + port->slot[i]] =
				(frames[i] & (0x8000u >> bit)) ? one : zero;

#ifdef ESC_HAVE_DSHOT_BIDIR
	if (port->bidirectional)
		return esc_dshot_send_bidir(port);
#endif
	HAL_TIM_DMABurst_WriteStop(port->tim, TIM_DMA_UPDATE);
	return esc_hal_errno(HAL_TIM_DMABurst_MultiWriteStart(
		port->tim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, port->burst,
		TIM_DMABURSTLENGTH_4TRANSFERS, DSHOT_BURST_ROWS * 4));
}

void esc_dshot_detach(struct dshot_port *port)
{
#ifdef ESC_HAVE_DSHOT_BIDIR
	esc_dshot_bidir_detach(port);
#endif
	HAL_TIM_DMABurst_WriteStop(port->tim, TIM_DMA_UPDATE);
	for (uint8_t i = 0; i < port->count; i++) {
		__HAL_TIM_SET_COMPARE(port->tim, port->channel[i], 0);
		HAL_TIM_PWM_Stop(port->tim, port->channel[i]);
	}
	port->count = 0;
}
#endif /* ESC_HAVE_DSHOT */

#endif /* !ESP_PLATFORM */
