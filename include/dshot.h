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
 * Bidirectional DShot (config.bidirectional, DShot300 and up): the line is
 * inverted and the ESC answers every frame on the same wire, 30 us later,
 * with its eRPM -- or, once command 13 has turned Extended DShot
 * Telemetry on, now and then with a temperature, voltage, current, debug
 * or status value instead. dshot_update() reads the answer to the last
 * frame before it sends the next, so the values lag one update;
 * dshot_get_erpm(), dshot_get_rpm() and dshot_get_telemetry() return them,
 * and esc_get_feedback() the velocity (with config.poles) and the
 * temperature. ESP-IDF: an RMT RX channel per motor on the motor's GPIO,
 * on every chip with an RMT, best with CONFIG_RMT_TX_ISR_CACHE_SAFE and
 * CONFIG_RMT_RX_ISR_CACHE_SAFE so a flash write cannot delay turning the
 * line round; STM32: the STM32F4 only, TIM1 or TIM8, the timer's update
 * DMA sampling the motors' GPIO port between frames. Elsewhere, and on a
 * bus without it, the bidirectional calls are -EOPNOTSUPP.
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

/*
 * Bidirectional: esc_get_feedback() says offline once no good reply has
 * come for this long, counted in dshot_update()'s dt_s -- the DJI
 * driver's default, not a DShot source's.
 */
#define DSHOT_REPLY_TIMEOUT_S	0.1f

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
	bool bidirectional;		/* inverted line, eRPM back; 300 and up */
	uint8_t count;			/* motors, 1..DSHOT_MOTORS_MAX */
	uint8_t poles;			/* motor magnets, even; 0: no velocity
					 * in esc_get_feedback() */
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
	/*
	 * Bidirectional only (STM32F4): the timer TIM1 or TIM8, whose update
	 * DMA is on DMA2 -- the only one that reaches the GPIO; a stream of
	 * DMA1 is -EOPNOTSUPP -- and the motors' pins, all on one port, each
	 * in alternate-function mode for its channel. Between frames the
	 * library turns them into inputs with pull-up and back. The update
	 * DMA's IRQ needs a high priority: the pins must be inputs and the
	 * DMA sampling within about 15 us of the frame's end, half the 30 us
	 * before the reply.
	 */
	GPIO_TypeDef *gpio;
	uint16_t pin[DSHOT_MOTORS_MAX];	/* GPIO_PIN_x of motor i */
#endif
};

/* Which fields of struct dshot_telemetry have arrived at least once. */
#define DSHOT_HAS_ERPM		(1u << 0)
#define DSHOT_HAS_TEMPERATURE	(1u << 1)
#define DSHOT_HAS_VOLTAGE	(1u << 2)
#define DSHOT_HAS_CURRENT	(1u << 3)
#define DSHOT_HAS_DEBUG1	(1u << 4)
#define DSHOT_HAS_DEBUG2	(1u << 5)
#define DSHOT_HAS_STRESS	(1u << 6)
#define DSHOT_HAS_STATUS	(1u << 7)
#define DSHOT_HAS_EDT_VERSION	(1u << 8)

/* Status bits of EDT v2 [EDT "EDT Frames"]; bits 3-0 the highest stress. */
#define DSHOT_STATUS_ALERT	(1u << 7)
#define DSHOT_STATUS_WARNING	(1u << 6)
#define DSHOT_STATUS_ERROR	(1u << 5)
#define DSHOT_STATUS_STRESS_MAX	0x0Fu

/*
 * What one motor's ESC has said on the signal line, the latest of each.
 * EDT values ([EDT "EDT Frames"]) arrive only once command 13 has gone out
 * and only from firmware that has EDT; which, and how often, is the
 * firmware's choice.
 */
struct dshot_telemetry {
	uint32_t has;			/* DSHOT_HAS_* */
	uint32_t erpm;			/* electrical turns a minute; 0 stopped */
	uint8_t temperature_c;
	float voltage_v;		/* 0.25 V steps, up to 63.75 */
	uint8_t current_a;
	uint8_t debug1, debug2;		/* the firmware's own */
	uint8_t stress;			/* 0-255; "debug 3" before EDT v2 */
	uint8_t status;			/* DSHOT_STATUS_* */
	uint8_t edt_version;		/* the answer to command 13 */
	uint32_t replies;		/* replies decoded */
	uint32_t errors;		/* replies missing, or failing their
					 * GCR or CRC */
};

/*
 * The platform's half. Private: here only so the app can place a
 * struct dshot_bus.
 */
struct dshot_port {
	uint32_t bit_ticks;		/* one bit, in the port's clock */
	uint32_t speed;			/* kbit/s */
	uint8_t count;
	bool bidirectional;
	uint8_t unread;			/* bit i: motor i's reply to the last
					 * frame not read yet */
#ifdef ESP_PLATFORM
	rmt_channel_handle_t channel[DSHOT_MOTORS_MAX];
	rmt_encoder_handle_t encoder[DSHOT_MOTORS_MAX];
	rmt_sync_manager_handle_t sync;	/* NULL for one motor */
	rmt_symbol_word_t symbols[DSHOT_MOTORS_MAX][16];
	/* bidirectional: the receivers, and what each heard */
	int gpio[DSHOT_MOTORS_MAX];
	rmt_channel_handle_t receiver[DSHOT_MOTORS_MAX];
	rmt_symbol_word_t heard[DSHOT_MOTORS_MAX][48];	/* the frame and its
							 * reply need 28 */
	volatile int32_t heard_symbols[DSHOT_MOTORS_MAX];	/* -1 while
								 * listening */
#else
	TIM_HandleTypeDef *tim;
	uint32_t channel[DSHOT_MOTORS_MAX];
	uint8_t slot[DSHOT_MOTORS_MAX];	/* CCR1..4 as 0..3 */
	uint32_t burst[(16 + 2) * 4];	/* 16 bits, then all low (twice
					 * when bidirectional) */
	/* bidirectional: the pins and the samples of their port */
	uint32_t tick_hz;
	uint32_t sample_ticks;		/* one sample, in timer ticks */
	GPIO_TypeDef *gpio;
	uint8_t pin[DSHOT_MOTORS_MAX];	/* 0-15 */
	volatile uint8_t phase;		/* idle, sending or sampling */
	volatile bool sampled;		/* samples[] hold the last reply */
	uint16_t samples[192];		/* DShot600 at 168 MHz takes 167 */
	uint16_t sample_count;
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
	/* bidirectional */
	bool edt;			/* command 13 sent, the off reply not seen */
	bool edt_version_next;		/* the next status frame answers 13 */
	float reply_age_s;		/* since the last good reply */
	struct dshot_telemetry reported;	/* what the ESC said */
};

struct dshot_bus {
	struct dshot_port port;
	struct dshot_motor motors[DSHOT_MOTORS_MAX];
	uint8_t count;
	uint8_t poles;
	bool bidirectional;
};

/*
 * Check the config, set the outputs up and leave every line at rest: low,
 * high when bidirectional. -EINVAL for a count or speed out of range,
 * bidirectional below DShot300, odd poles, a GPIO below 0, or on the HAL
 * a timer, channel, ARR or DMA set up otherwise than above, or a
 * bidirectional pin missing or twice; -EOPNOTSUPP for bidirectional where
 * the port has none (the HAL off the STM32F4) or for more motors than its
 * RMT has RX channels for; the RMT's or HAL's error otherwise. Nothing is
 * left running on failure.
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
 * is still going out, or bidirectional, its reply still due; the RMT's
 * or HAL's error. A frame that did not go out counts neither a command's
 * repeat nor its wait. Bidirectional, the reply to the last frame is read
 * first, whether the new one goes or not.
 */
int dshot_update(struct dshot_bus *bus, float dt_s);

/*
 * The motor's last eRPM; -ENODATA until one has arrived, -EOPNOTSUPP on a
 * bus that is not bidirectional, -EINVAL for an esc_dev that is not a
 * DShot motor.
 */
int dshot_get_erpm(struct esc_dev *esc, uint32_t *erpm);

/* The same in shaft turns a minute, for a motor of poles magnets; poles
 * odd or 0 is -EINVAL. */
int dshot_get_rpm(struct esc_dev *esc, unsigned int poles, uint32_t *rpm);

/* Everything the ESC has said; -EOPNOTSUPP as above. */
int dshot_get_telemetry(struct esc_dev *esc,
			struct dshot_telemetry *telemetry);

/* Stop the outputs, lines low, and release what init took. */
void dshot_del(struct dshot_bus *bus);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H */
