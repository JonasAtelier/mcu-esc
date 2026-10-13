#include <errno.h>
#include <math.h>

#include "esc.h"

static bool esc_bound(const struct esc_dev *esc)
{
	return esc && esc->ops && esc->ops->set_command;
}

int esc_set_command(struct esc_dev *esc, enum esc_command kind, float value)
{
	if (!esc_bound(esc) || (unsigned)kind > ESC_COMMAND_THROTTLE ||
	    !isfinite(value))
		return -EINVAL;
	if (!(esc->commands & ESC_COMMAND_BIT(kind)))
		return -EOPNOTSUPP;
	return esc->ops->set_command(esc, kind, value);
}

int esc_get_feedback(struct esc_dev *esc, struct esc_feedback *feedback)
{
	struct esc_feedback tmp = { 0 };
	int ret;

	if (!esc_bound(esc) || !feedback)
		return -EINVAL;
	if (!esc->ops->get_feedback)
		return -EOPNOTSUPP;
	ret = esc->ops->get_feedback(esc, &tmp);
	if (!ret)
		*feedback = tmp;
	return ret;
}
