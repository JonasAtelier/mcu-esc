/*
 * DJI RoboMaster C615 on PWM: one pulse per period, its width the throttle.
 * C615 User Guide V1.0 2019.10 (docs/, gitignored):
 *
 *   - "Control Throttle Range 400-2200 us", settable in DJI Assistant 2
 *     ("Specifications", page 5). Within that, the ESC's own endpoints are
 *     whatever it was calibrated to: "PWM Configuration", page 4, sets them
 *     from the maximum and minimum pulse the controller sends.
 *   - "Max Control Signal Frequency 500 Hz" (page 5).
 *   - "Rapid Beeping: Minimum PWM signal pulse width not reached";
 *     "Slow Beeping: No PWM signal" (page 5).
 *
 * The guide does not say which pulse stops the motor or which one idles it,
 * so this driver invents neither: min_us and max_us are the endpoints the
 * app calibrated the ESC to, throttle 0 is min_us and 1 is max_us, and the
 * app decides what 0 means on its own bench. The pin starts at min_us.
 *
 * Throttle only: ESC_COMMAND_THROTTLE through esc_set_command(), clamped to
 * 0..1, which writes the pulse at once. No feedback: the C615's TX wire is
 * for DJI Assistant 2 (page 3), so esc_get_feedback() is -EOPNOTSUPP.
 *
 * One struct c615 per ESC, placed by the app.
 */
#ifndef C615_H
#define C615_H

#include <stdint.h>

#include "esc.h"

#ifdef ESP_PLATFORM
#include "driver/ledc.h"
#else
#include CMSIS_device_header
#ifndef HAL_TIM_MODULE_ENABLED
#error "c615.h needs the HAL TIM module: #define HAL_TIM_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_tim.c in the build"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* "Control Throttle Range" and "Max Control Signal Frequency", page 5. */
#define C615_PULSE_MIN_US	400u
#define C615_PULSE_MAX_US	2200u
#define C615_RATE_MAX_HZ	500u

struct c615_config {
#ifdef ESP_PLATFORM
	int gpio;
	ledc_timer_t timer;	/* LEDC timer; set to rate_hz, 14-bit duty */
	ledc_channel_t channel;	/* LEDC channel, low-speed mode */
	uint32_t rate_hz;	/* 1..C615_RATE_MAX_HZ */
#else
	TIM_HandleTypeDef *tim;	/* set up in PWM mode 1 by the app, with
				 * output-compare preload on the channel
				 * (CCMRx OCxPE): without it a compare
				 * written mid-period takes effect at once
				 * and the pulse in flight can come out
				 * short or doubled */
	uint32_t channel;	/* TIM_CHANNEL_1..4 */
	uint32_t tick_hz;	/* counter clock after the prescaler; the
				 * rate is tick_hz / (ARR + 1), at most
				 * C615_RATE_MAX_HZ */
#endif
	uint16_t min_us;	/* calibrated endpoints, inside 400..2200, */
	uint16_t max_us;	/* min below max */
};

/*
 * The PWM port's state: one output pin, see src/esc_priv.h. Private, like
 * struct c615: here only so the app can place one.
 */
struct esc_pwm {
#ifdef ESP_PLATFORM
	int gpio;
	ledc_timer_t timer;
	ledc_channel_t channel;
#else
	TIM_HandleTypeDef *tim;
	uint32_t channel;
	uint32_t tick_hz;
#endif
	uint32_t rate_hz;
};

/* Private. Declared here only so the app can place a struct c615. */
struct c615 {
	struct esc_dev esc;
	struct esc_pwm pwm;
	uint16_t min_us;
	uint16_t max_us;
	uint16_t pulse_us;	/* what the pin puts out now */
};

/*
 * Check the config, set up the output and start it at min_us. -EINVAL for
 * endpoints outside 400..2200 us or not in order, a rate of 0 or over
 * 500 Hz, or a max_us that does not fit in one period; the LEDC's or the
 * HAL's error otherwise. Nothing is left running on failure.
 */
int c615_init(struct c615 *c615, const struct c615_config *config);

/*
 * The pulse itself, in microseconds, between min_us and max_us: for
 * calibration and bench checks. -EINVAL outside them.
 */
int c615_set_us(struct c615 *c615, uint16_t pulse_us);

struct esc_dev *c615_esc(struct c615 *c615);

/* Stop the pulses; the pin stays low, which the ESC reads as no signal. */
void c615_del(struct c615 *c615);

#ifdef __cplusplus
}
#endif

#endif /* C615_H */
