/*
 * A C620 (M3508) and a GM6020 on the ESP32's TWAI: feedback in, a current
 * command out, every millisecond; a line of feedback every half second.
 * sdkconfig.defaults sets the 1 kHz FreeRTOS tick that takes.
 *
 * Wiring: TX and RX to an SN65HVD230's D and R, its Rs pin to PIN_RS (or
 * TWAICAN_NO_GPIO and Rs tied low), 120 ohm at both ends of the bus. The
 * C620 at ID 1 (SET button, LED blinks once); the GM6020 at ID 2 (DIP
 * Bit[2:0] = 010), in current mode -- firmware v1.0.11.2 or later with
 * "Current Ring" on in RoboMaster Assistant. Wheels off the ground: this
 * spins both.
 */
#include <errno.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can.h"
#include "dji.h"
#include "esc.h"
#include "esc_log.h"
#include "twai_can.h"

#define PIN_TX      4
#define PIN_RX      5
#define PIN_RS      6

static const char *TAG = "example";

void app_main(void)
{
    twaican_config_t cfg = {
        .tx_gpio = PIN_TX,
        .rx_gpio = PIN_RX,
        .rs_gpio = PIN_RS,
        .bitrate = 1000000,     /* the DJI ESCs' fixed rate */
        .rx_queue_len = 32,     /* two motors at 1 kHz each */
        .restart_ms = 100,
    };
    struct dji_motor_config wheel_cfg = {
        .model = DJI_MODEL_C620,
        .id = 1,
        .gear_ratio = 3591.0f / 187.0f,     /* M3508 P19 */
    };
    struct dji_motor_config gimbal_cfg = {
        .model = DJI_MODEL_GM6020,
        .id = 2,
        .gm6020_current_mode = true,
    };
    static struct dji_bus bus;
    struct esc_dev *wheel, *gimbal;
    twaican_handle_t twai;
    struct can_dev *can;
    int64_t previous_us;
    unsigned tick = 0;

    ESP_ERROR_CHECK(twaican_init(&cfg, &twai));
    can = twaican_can(twai);
    if (dji_init(&bus, can) || dji_add(&bus, &wheel_cfg, &wheel) ||
        dji_add(&bus, &gimbal_cfg, &gimbal)) {
        ESP_LOGE(TAG, "dji setup failed");
        return;
    }
    esc_set_command(wheel, ESC_COMMAND_CURRENT, 1.0f);
    esc_set_command(gimbal, ESC_COMMAND_CURRENT, 0.3f);

    previous_us = esp_timer_get_time();
    while (1) {
        struct can_frame frame;
        struct esc_feedback fb;
        int64_t now_us;

        vTaskDelay(pdMS_TO_TICKS(1));
        while (can_recv(can, &frame, 0) == 0)
            dji_handle_frame(&bus, &frame);

        now_us = esp_timer_get_time();
        if (dji_update(&bus, (float)(now_us - previous_us) / 1e6f) ==
            -ENOTCONN && tick % 500 == 0)
            ESP_LOGW(TAG, "a motor is offline and held at 0");
        previous_us = now_us;
        twaican_service(twai);

        if (++tick % 500)
            continue;
        if (esc_get_feedback(wheel, &fb) == 0)
            esc_log_feedback(wheel, &fb);
        if (esc_get_feedback(gimbal, &fb) == 0)
            esc_log_feedback(gimbal, &fb);
    }
}
