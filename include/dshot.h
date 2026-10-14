/*
 * DShot150/300/600 out to up to four ESCs at once: on ESP-IDF one RMT TX
 * channel per motor, started together -- on any chip with an RMT except the
 * original ESP32, which cannot start channels together and so takes one
 * motor (-EOPNOTSUPP for more), and not on the C2, which has no RMT; on
 * the STM32 HAL one timer whose
 * update DMA burst-writes CCR1-CCR4 every bit, so all four motors cost one
 * DMA stream.
 *
 * A frame is an 11-bit value, a telemetry request bit and a 4-bit CRC.
 * Value 0 stops the motor, 1-47 are special commands, 48-2047 throttle
 * (docs/chips.md, "DShot", with every source). Through the esc_dev:
 * ESC_COMMAND_THROTTLE 0 is value 0; above 0, 48..2047 linear.
 *
 * One struct dshot_bus per timer (or per group of RMT channels), placed by
 * the app:
 *
 *   dshot_init(&bus, &config);
 *   loop, at the app's rate:
 *     esc_set_command(dshot_esc(&bus, 0), ESC_COMMAND_THROTTLE, t);
 *     dshot_update(&bus, dt_s);       one frame to every motor
 *
 * Nothing is sent between two dshot_update() calls: an ESC that stops
 * getting frames disarms by its own rules. Most firmware wants value 0 for
 * a while before it arms -- 300 ms on Bluejay [BW "Arming sequence"] -- so
 * send throttle 0 for that long first. A call while the last frame is still
 * on the wire is -EBUSY and sends nothing.
 *
 * Special commands (dshot_command()) go out the number of times their
 * source asks, with the telemetry bit set, then hold the motor at 0 for the
 * wait it asks, before throttle resumes; dt_s counts that wait down. 0-36
 * need the motor stopped. The telemetry bit only asks: the reply comes on
 * the ESC's separate telemetry wire, which this library does not read.
 *
 * Bidirectional DShot (eRPM back on the signal wire) is phase 2:
 * config.bidirectional is -EOPNOTSUPP for now. No feedback in this phase:
 * esc_get_feedback() is -EOPNOTSUPP.
 *
 * Not thread-safe: one task per bus, or your own lock.
 */
#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "esc.h"

#ifdef ESP_PLATFORM
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#else
#include CMSIS_device_header
#if !defined(HAL_TIM_MODULE_ENABLED) || !defined(HAL_DMA_MODULE_ENABLED)
#error "dshot.h needs the HAL TIM and DMA modules: #define HAL_TIM_MODULE_ENABLED and HAL_DMA_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_tim.c and _dma.c in the build"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_MOTORS_MAX	4

/* Bit rate in kbit/s. */
enum dshot_speed {
	DSHOT_150 = 150,
	DSHOT_300 = 300,
	DSHOT_600 = 600,
};

/*
 * The special commands with a meaning [BLH lines 1-27; BW "Special
 * commands" for 32-35 and 42-47]. Others are -EINVAL.
 */
enum dshot_command {
	DSHOT_CMD_BEEP1 = 1,
	DSHOT_CMD_BEEP2 = 2,
	DSHOT_CMD_BEEP3 = 3,
	DSHOT_CMD_BEEP4 = 4,
	DSHOT_CMD_BEEP5 = 5,
	DSHOT_CMD_ESC_INFO = 6,
	DSHOT_CMD_SPIN_DIRECTION_1 = 7,
	DSHOT_CMD_SPIN_DIRECTION_2 = 8,
	DSHOT_CMD_3D_MODE_OFF = 9,
	DSHOT_CMD_3D_MODE_ON = 10,
	DSHOT_CMD_SETTINGS_REQUEST = 11,
	DSHOT_CMD_SAVE_SETTINGS = 12,
	DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE = 13,
	DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE = 14,
	DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
	DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
	DSHOT_CMD_LED0_ON = 22,
	DSHOT_CMD_LED1_ON = 23,
	DSHOT_CMD_LED2_ON = 24,
	DSHOT_CMD_LED3_ON = 25,
	DSHOT_CMD_LED0_OFF = 26,
	DSHOT_CMD_LED1_OFF = 27,
	DSHOT_CMD_LED2_OFF = 28,
	DSHOT_CMD_LED3_OFF = 29,
	DSHOT_CMD_AUDIO_STREAM = 30,
	DSHOT_CMD_SILENT_MODE = 31,
	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE = 33,
	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM = 34,
	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD = 35,
	DSHOT_CMD_SIGNAL_LINE_TEMPERATURE = 42,
	DSHOT_CMD_SIGNAL_LINE_VOLTAGE = 43,
	DSHOT_CMD_SIGNAL_LINE_CURRENT = 44,
	DSHOT_CMD_SIGNAL_LINE_CONSUMPTION = 45,
	DSHOT_CMD_SIGNAL_LINE_ERPM = 46,
	DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD = 47,
};

struct dshot_config {
	enum dshot_speed speed;
	bool bidirectional;		/* phase 2: -EOPNOTSUPP */
	uint8_t count;			/* motors, 1..DSHOT_MOTORS_MAX */
#ifdef ESP_PLATFORM
	int gpio[DSHOT_MOTORS_MAX];	/* motor i on gpio[i] */
#else
	/*
	 * Set up by the app: every used channel in PWM mode 1 with
	 * output-compare preload, the prescaler and ARR so that one counter
	 * period is one bit -- ARR + 1 = tick_hz / (speed * 1000), rounded --
	 * and the update DMA linked to hdma[TIM_DMA_ID_UPDATE]: memory to
	 * peripheral, word on both sides, memory increment, normal mode, its
	 * IRQ calling HAL_DMA_IRQHandler(). The library starts the channels.
	 */
	TIM_HandleTypeDef *tim;
	uint32_t tick_hz;		/* counter clock after the prescaler */
	uint32_t channel[DSHOT_MOTORS_MAX];	/* TIM_CHANNEL_1..4 */
#endif
};

/*
 * The platform's half. Private: here only so the app can place a
 * struct dshot_bus.
 */
struct dshot_port {
	uint32_t bit_ticks;		/* one bit, in the port's clock */
	uint8_t count;
#ifdef ESP_PLATFORM
	rmt_channel_handle_t channel[DSHOT_MOTORS_MAX];
	rmt_encoder_handle_t encoder[DSHOT_MOTORS_MAX];
	rmt_sync_manager_handle_t sync;	/* NULL for one motor */
	rmt_symbol_word_t symbols[DSHOT_MOTORS_MAX][16];
#else
	TIM_HandleTypeDef *tim;
	uint32_t channel[DSHOT_MOTORS_MAX];
	uint8_t slot[DSHOT_MOTORS_MAX];	/* CCR1..4 as 0..3 */
	uint32_t burst[(16 + 1) * 4];	/* 16 bits, then all low */
#endif
};

struct dshot_bus;

/* One motor. Private: read it through the calls below. */
struct dshot_motor {
	struct esc_dev esc;
	struct dshot_bus *bus;
	uint16_t value;			/* what goes out when no command runs */
	bool telemetry;			/* set the bit in every frame */
	bool telemetry_once;		/* set the bit in the next frame */
	uint8_t command;		/* the special command being sent */
	uint8_t repeats_left;		/* frames of it still to send */
	float hold_s;			/* then value 0 this much longer */
};

struct dshot_bus {
	struct dshot_port port;
	struct dshot_motor motors[DSHOT_MOTORS_MAX];
	uint8_t count;
};

/*
 * Check the config, set the outputs up and leave every line low. -EINVAL
 * for a count or speed out of range, a GPIO below 0, or on the HAL a
 * timer, channel, ARR or DMA set up otherwise than above;
 * -EOPNOTSUPP for bidirectional; the RMT's or HAL's error otherwise.
 * Nothing is left running on failure.
 */
int dshot_init(struct dshot_bus *bus, const struct dshot_config *config);

/* Motor index's esc_dev, NULL past count or before init. */
struct esc_dev *dshot_esc(struct dshot_bus *bus, unsigned int index);

/*
 * Queue a special command for one motor. -EINVAL for a command without a
 * meaning, -EBUSY while another command of this motor is still going out
 * or holding, or for 0-36 while its value is not 0 (motor running).
 */
int dshot_command(struct esc_dev *esc, enum dshot_command command);

/*
 * The raw 11-bit value, 0 or 48..2047, for what the throttle mapping does
 * not reach -- 3D mode's two halves, say. telemetry sets the request bit in
 * every frame until the next call. 1-47 are special commands: -EINVAL here,
 * use dshot_command().
 */
int dshot_set_value(struct esc_dev *esc, uint16_t value, bool telemetry);

/* Set the telemetry request bit in this motor's next frame only. */
int dshot_request_telemetry(struct esc_dev *esc);

/*
 * One frame to every motor, and dt_s off the wait of a command. -EINVAL
 * for a dt_s that is not finite or below 0; -EBUSY while the last frame
 * is still going out; the RMT's or HAL's error. A frame that did not go
 * out counts neither a command's repeat nor its wait.
 */
int dshot_update(struct dshot_bus *bus, float dt_s);

/* Stop the outputs, lines low, and release what init took. */
void dshot_del(struct dshot_bus *bus);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H */
