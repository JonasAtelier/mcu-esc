/*
 * Hold one HT-03 at the angle it sat at on boot: esp-HT03's
 * examples/position_hold.c on mcu-can's TWAI.
 *
 * Three things that bite:
 *
 *   1. The motor replies only to frames it receives. ht03_update() every
 *      period is what keeps feedback coming, not just what sends commands.
 *   2. Zero is set while disabled, then enable. Enabling resets the stored
 *      command to limp, so nothing moves until the first ht03_command_*().
 *   3. kp and kd are the motor's own loop, in Nm/rad and Nm*s/rad. Start
 *      low; kp 500 is the encoding ceiling, not a tune.
 */
#include <errno.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can.h"
#include "ht03.h"
#include "twai_can.h"

#define PIN_TX     5
#define PIN_RX     4
#define MOTOR_ID   1
#define PERIOD_MS  2        /* esp-HT03's; sdkconfig.defaults sets a 1 kHz tick */

#define HOLD_KP 10.0f    /* Nm per rad of error */
#define HOLD_KD 0.3f     /* Nm per rad/s of error */

#define REPORT_EVERY_MS 500

static const char *TAG = "ht03-hold";

/* Read the bus, step the time, send. One period of the loop. */
static int tick(twaican_handle_t twai, ht03_t *ht, int64_t *prev_us)
{
    const int64_t now_us = esp_timer_get_time();
    const float dt_s = (float)(now_us - *prev_us) / 1000000.0f;
    struct can_dev *can = twaican_can(twai);
    struct can_frame frame;

    *prev_us = now_us;

    twaican_service(twai);       /* bus-off recovery */
    while (can_recv(can, &frame, 0) == 0)
        ht03_handle_frame(ht, &frame);  /* last period's replies */

    return ht03_update(ht, dt_s > 0.0f ? dt_s : 0.001f);
}

void app_main(void)
{
    static const ht03_range_t j6 = HT03_J6;
    twaican_config_t cfg = {
        .tx_gpio = PIN_TX,
        .rx_gpio = PIN_RX,
        .rs_gpio = TWAICAN_NO_GPIO,
        .bitrate = HT03_BITRATE,
        .tx_queue_len = 8,
        .rx_queue_len = 16,
        .restart_ms = 100,
    };
    uint32_t since_report_ms = 0;
    ht03_feedback_t feedback;
    twaican_handle_t twai;
    int64_t prev_us;
    static ht03_t ht;

    ESP_ERROR_CHECK(twaican_init(&cfg, &twai));

    if (ht03_init(&ht, twaican_can(twai)) != 0 ||
        ht03_cfg(&ht, MOTOR_ID, &j6) != 0) {
        ESP_LOGE(TAG, "ht03 setup failed");
        return;
    }

    /* ht03_set_zero() exits motor mode itself before zeroing, so the zero
     * is taken from a motor that is not pushing against anything. It leaves
     * the motor disabled.
     */
    if (ht03_set_zero(&ht, MOTOR_ID) != 0) {
        ESP_LOGE(TAG, "set zero not sent");
        return;
    }

    /* ht03_update() sends limp frames until the first reply arrives */
    prev_us = esp_timer_get_time();
    do {
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
        tick(twai, &ht, &prev_us);
    } while (ht03_get_feedback(&ht, MOTOR_ID, &feedback) != 0 ||
             !feedback.is_online);

    ESP_LOGI(TAG, "online at %.3f rad, enabling", feedback.position);
    if (ht03_enable(&ht, MOTOR_ID) != 0) {
        ESP_LOGE(TAG, "enable not sent");
        return;
    }

    while (true) {
        /* Recomputed every period: after a dropout the stored command is
         * applied again as soon as the motor replies.
         */
        ht03_command_position(&ht, MOTOR_ID, 0.0f, HOLD_KP, HOLD_KD);

        if (tick(twai, &ht, &prev_us) == -ENOTCONN)
            ESP_LOGW(TAG, "no reply, sending limp");

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));

        since_report_ms += PERIOD_MS;
        if (since_report_ms < REPORT_EVERY_MS)
            continue;
        since_report_ms = 0;

        if (ht03_get_feedback(&ht, MOTOR_ID, &feedback) == 0 &&
            feedback.is_online)
            ESP_LOGI(TAG, "%.3f rad, %.2f rad/s, %.2f Nm",
                     feedback.position, feedback.velocity, feedback.torque);
    }
}
