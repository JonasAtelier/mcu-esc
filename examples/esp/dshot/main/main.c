/*
 * Four DShot600 ESCs on an ESP32-S3: hold value 0 so they arm, beep motor
 * 1 once, then ramp all four to 10 % and back, one frame a millisecond.
 *
 * Props off. Each ESC's signal wire to a GPIO below, grounds common. Most
 * firmware wants value 0 for a while before it arms -- 300 ms on Bluejay
 * (docs/chips.md, "DShot"); ESP-IDF's own DShot example holds it 3 s
 * (examples/peripherals/rmt/dshot_esc), as this one does.
 */
#include <errno.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dshot.h"
#include "esc.h"

static const char *TAG = "example";

static struct dshot_bus bus;
static int64_t previous_us;

/* One frame period: wait a tick, then send. */
static void frame(void)
{
    int64_t now_us;
    int err;

    vTaskDelay(pdMS_TO_TICKS(1));
    now_us = esp_timer_get_time();
    err = dshot_update(&bus, (float)(now_us - previous_us) / 1e6f);
    if (err && err != -EBUSY)
        ESP_LOGW(TAG, "dshot_update: %d", err);
    previous_us = now_us;
}

void app_main(void)
{
    const struct dshot_config cfg = {
        .speed = DSHOT_600,
        .count = 4,
        .gpio = { 4, 5, 6, 7 },
    };
    int err;

    err = dshot_init(&bus, &cfg);
    if (err) {
        ESP_LOGE(TAG, "dshot_init: %d", err);
        return;
    }
    previous_us = esp_timer_get_time();

    for (int ms = 0; ms < 3000; ms++)
        frame();

    /* one frame of BEEP1, then 260 ms of 0, counted by dshot_update() */
    if (dshot_command(dshot_esc(&bus, 0), DSHOT_CMD_BEEP1) == 0)
        for (int ms = 0; ms < 400; ms++)
            frame();

    for (int step = 0; step <= 200; step++) {
        for (unsigned i = 0; i < 4; i++)
            esc_set_command(dshot_esc(&bus, i), ESC_COMMAND_THROTTLE,
                            (step <= 100 ? step : 200 - step) / 1000.0f);
        for (int ms = 0; ms < 20; ms++)
            frame();
    }
    ESP_LOGI(TAG, "ramp done; holding 0");
    for (;;)
        frame();
}
