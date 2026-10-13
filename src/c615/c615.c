/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "esc_priv.h"

#if CONFIG_ESC_C615

#include <math.h>
#include <string.h>

#include "c615.h"

static int c615_set_command(struct esc_dev *esc, enum esc_command kind,
			    float value);

static const struct esc_ops c615_ops = {
	.set_command = c615_set_command,
};

int c615_init(struct c615 *c615, const struct c615_config *config)
{
	int ret;

	if (!c615 || !config || config->min_us < C615_PULSE_MIN_US ||
	    config->max_us > C615_PULSE_MAX_US ||
	    config->min_us >= config->max_us)
		return -EINVAL;
	memset(c615, 0, sizeof(*c615));

#ifdef ESP_PLATFORM
	if (!config->rate_hz || config->rate_hz > C615_RATE_MAX_HZ)
		return -EINVAL;
	ret = esc_pwm_attach(&c615->pwm, config->gpio, config->timer,
			     config->channel, config->rate_hz);
#else
	ret = esc_pwm_attach(&c615->pwm, config->tim, config->channel,
			     config->tick_hz);
	if (!ret && (!c615->pwm.rate_hz ||
		     c615->pwm.rate_hz > C615_RATE_MAX_HZ))
		ret = -EINVAL;
#endif
	if (ret)
		return ret;

	/* Start at min_us; a max_us that does not fit is refused first. */
	ret = esc_pwm_check(&c615->pwm, config->max_us);
	if (!ret)
		ret = esc_pwm_start(&c615->pwm, config->min_us);
	if (ret)
		return ret;

	c615->min_us = config->min_us;
	c615->max_us = config->max_us;
	c615->pulse_us = config->min_us;
	c615->esc.name = "c615";
	c615->esc.ops = &c615_ops;
	c615->esc.priv = c615;
	c615->esc.commands = ESC_COMMAND_BIT(ESC_COMMAND_THROTTLE);
	return 0;
}

int c615_set_us(struct c615 *c615, uint16_t pulse_us)
{
	int ret;

	if (!c615 || !c615->esc.ops || pulse_us < c615->min_us ||
	    pulse_us > c615->max_us)
		return -EINVAL;
	ret = esc_pwm_write_us(&c615->pwm, pulse_us);
	if (!ret)
		c615->pulse_us = pulse_us;
	return ret;
}

struct esc_dev *c615_esc(struct c615 *c615)
{
	return c615 && c615->esc.ops ? &c615->esc : NULL;
}

void c615_del(struct c615 *c615)
{
	if (!c615 || !c615->esc.ops)
		return;
	esc_pwm_stop(&c615->pwm);
	c615->esc.ops = NULL;
}

/* Throttle 0..1, clamped, onto min_us..max_us, rounded to whole us. */
static int c615_set_command(struct esc_dev *esc, enum esc_command kind,
			    float value)
{
	struct c615 *c615 = esc->priv;
	float span = (float)(c615->max_us - c615->min_us);

	(void)kind;	/* throttle: the only kind in commands */
	if (value < 0.0f)
		value = 0.0f;
	if (value > 1.0f)
		value = 1.0f;
	return c615_set_us(c615, (uint16_t)(c615->min_us +
					    lroundf(value * span)));
}

#endif /* CONFIG_ESC_C615 */
