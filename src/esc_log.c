#include <stdio.h>

#include "esc_log.h"

void esc_log_feedback(const struct esc_dev *esc,
		      const struct esc_feedback *feedback)
{
	char line[128];
	int n = 0;

	if (feedback->has & ESC_HAS_ANGLE)
		n += snprintf(line + n, sizeof(line) - (size_t)n,
			      " angle %.3f", (double)feedback->angle_rad);
	if (feedback->has & ESC_HAS_POSITION)
		n += snprintf(line + n, sizeof(line) - (size_t)n,
			      " position %.3f", (double)feedback->position_rad);
	if (feedback->has & ESC_HAS_VELOCITY)
		n += snprintf(line + n, sizeof(line) - (size_t)n,
			      " velocity %.2f",
			      (double)feedback->velocity_rad_s);
	if (feedback->has & ESC_HAS_TORQUE)
		n += snprintf(line + n, sizeof(line) - (size_t)n,
			      " torque %.2f", (double)feedback->torque_nm);
	if (feedback->has & ESC_HAS_TEMPERATURE)
		n += snprintf(line + n, sizeof(line) - (size_t)n,
			      " temperature %.0f",
			      (double)feedback->temperature_c);
	if (!n)
		line[0] = '\0';
	esc_info(esc, "%s%s", feedback->online ? "online" : "offline", line);
}
