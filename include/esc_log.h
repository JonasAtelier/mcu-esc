/*
 * Kernel dev_err()-style logging for motor drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "esc",
 * prefixed with the motor's name, so one esp_log_level_set("esc", ...)
 * controls every motor and a line still says which model it came from.
 * There is deliberately no level machinery of our own; esc_dbg()
 * is also subject to CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_ESC_DEBUG
 * is not used.
 *
 * STM32 HAL: every line goes out through printf() as
 * "esc: <name>: ...". Where printf() ends up is the app's
 * business: with newlib, retarget _write() to a UART. esc_dbg()
 * alone is compiled out unless CONFIG_ESC_DEBUG is 1 (stm-forge:
 * ESC_DEBUG = y).
 */
#ifndef ESC_LOG_H
#define ESC_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "esc.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define ESC_LOG_TAG	"esc"

#define esc_err(esc, fmt, ...) \
	ESP_LOGE(ESC_LOG_TAG, "%s: " fmt, (esc)->name, ##__VA_ARGS__)
#define esc_warn(esc, fmt, ...) \
	ESP_LOGW(ESC_LOG_TAG, "%s: " fmt, (esc)->name, ##__VA_ARGS__)
#define esc_info(esc, fmt, ...) \
	ESP_LOGI(ESC_LOG_TAG, "%s: " fmt, (esc)->name, ##__VA_ARGS__)
#define esc_dbg(esc, fmt, ...) \
	ESP_LOGD(ESC_LOG_TAG, "%s: " fmt, (esc)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define esc_err(esc, fmt, ...) \
	printf("esc: %s: " fmt "\n", (esc)->name, ##__VA_ARGS__)
#define esc_warn(esc, fmt, ...) \
	printf("esc: %s: " fmt "\n", (esc)->name, ##__VA_ARGS__)
#define esc_info(esc, fmt, ...) \
	printf("esc: %s: " fmt "\n", (esc)->name, ##__VA_ARGS__)

#if CONFIG_ESC_DEBUG
#define esc_dbg(esc, fmt, ...) \
	printf("esc: %s: " fmt "\n", (esc)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define esc_dbg(esc, fmt, ...) do { \
	if (0) \
		printf("esc: %s: " fmt "\n", (esc)->name, \
		       ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/* One info line: the fields of feedback that the model has. */
void esc_log_feedback(const struct esc_dev *esc,
		      const struct esc_feedback *feedback);

#ifdef __cplusplus
}
#endif

#endif /* ESC_LOG_H */
