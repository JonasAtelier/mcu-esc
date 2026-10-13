/*
 * A C615 on an LEDC channel: start at the bottom endpoint, ramp to 20 %
 * throttle and back, then stop the pulses.
 *
 * Props off. MIN_US and MAX_US must be the endpoints the ESC was calibrated
 * to (C615 User Guide, "PWM Configuration"); the guide does not say that
 * MIN_US stops the motor, so check that on the bench first.
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "c615.h"
#include "esc.h"

#define PIN_PWM     7
#define MIN_US      1000
#define MAX_US      2000

static const char *TAG = "example";

void app_main(void)
{
    struct c615_config cfg = {
        .gpio = PIN_PWM,
        .timer = LEDC_TIMER_0,
        .channel = LEDC_CHANNEL_0,
        .rate_hz = 400,
        .min_us = MIN_US,
        .max_us = MAX_US,
    };
    static struct c615 c615;
    struct esc_dev *esc;
    int err;

    err = c615_init(&c615, &cfg);
    if (err) {
        ESP_LOGE(TAG, "c615_init: %d", err);
        return;
    }
    esc = c615_esc(&c615);

    /*
     * Hold the bottom endpoint until the ESC chimes "Ready to work" (beep
     * codes, page 5). How long that takes the guide does not say: 3 s is
     * this example's guess.
     */
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int step = 0; step <= 20; step++) {
        esc_set_command(esc, ESC_COMMAND_THROTTLE, step / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    for (int step = 20; step >= 0; step--) {
        esc_set_command(esc, ESC_COMMAND_THROTTLE, step / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "ramp done; stopping the pulses");
    c615_del(&c615);
}
