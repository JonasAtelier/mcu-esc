/*
 * Two bidirectional DShot600 ESCs -- Bluejay, BLHeli_32 or AM32 with
 * bidirectional DShot -- on any ESP32 with an RMT: hold value 0 so they
 * arm, turn Extended DShot Telemetry on, spin both at 5 % and log each
 * motor's RPM and temperature twice a second.
 *
 * Two motors, so it runs on the C3, C6 and H2 too, whose RMT has two RX
 * channels (docs/hardware.md, "DShot"); the S3 and P4 take four. Props
 * off. Each ESC's signal wire to a GPIO below, grounds common; the line
 * rests high. MOTOR_POLES is the motor's magnet count, 14 on most 22xx
 * motors -- count yours.
 */
#include <errno.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dshot.h"
#include "esc.h"

#define MOTOR_POLES 14

static const char *TAG = "example";

static struct dshot_bus bus;
static int64_t previous_us;

/* One frame period: wait a tick, then send, and read the last answer. */
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

static void report(unsigned int motor)
{
    struct dshot_telemetry telemetry;
    uint32_t rpm;

    if (dshot_get_telemetry(dshot_esc(&bus, motor), &telemetry))
        return;
    if (dshot_get_rpm(dshot_esc(&bus, motor), MOTOR_POLES, &rpm))
        rpm = 0;
    ESP_LOGI(TAG, "motor %u: %lu rpm, %d C, %lu replies, %lu errors",
             motor, (unsigned long)rpm,
             (telemetry.has & DSHOT_HAS_TEMPERATURE) ?
             telemetry.temperature_c : -1,
             (unsigned long)telemetry.replies,
             (unsigned long)telemetry.errors);
}

void app_main(void)
{
    const struct dshot_config cfg = {
        .speed = DSHOT_600,
        .bidirectional = true,
        .count = 2,
        .poles = MOTOR_POLES,
        .gpio = { 4, 5 },
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

    /* six frames of 13 each, then 35 ms of 0 [EDT "Enabling EDT"] */
    for (unsigned i = 0; i < 2; i++)
        dshot_command(dshot_esc(&bus, i),
                      DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE);
    for (int ms = 0; ms < 100; ms++)
        frame();

    for (unsigned i = 0; i < 2; i++)
        esc_set_command(dshot_esc(&bus, i), ESC_COMMAND_THROTTLE, 0.05f);
    for (;;) {
        for (int ms = 0; ms < 500; ms++)
            frame();
        report(0);
        report(1);
    }
}
