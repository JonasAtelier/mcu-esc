/*
 * A C610 (M2006) and a C620 (M3508) on an STM32F4's bxCAN, through
 * mcu-can: feedback drained from the ring, a current command out, once a
 * millisecond.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * peripheral are the app's -- here hcan1 as CubeMX generates it at
 * 1 Mbit/s with AutoBusOff on, a filter that sends everything to FIFO 0,
 * and CAN1_RX0 in the NVIC with HAL_CAN_IRQHandler() in its handler, as in
 * mcu-can's examples/stm/bxcan. Needs CAN_BXCAN = y, ESC_C610 = y and
 * ESC_C620 = y.
 */
#include <stdio.h>

#include CMSIS_device_header

#include "bxcan.h"
#include "can.h"
#include "dji.h"
#include "esc.h"
#include "esc_log.h"

extern CAN_HandleTypeDef hcan1;		/* from MX_CAN1_Init() */

static bxcan_handle_t bx;
static struct dji_bus bus;
static struct esc_dev *arm, *wheel;

void esc_example_init(void)
{
	bxcan_config_t cfg = { .hcan = &hcan1, .use_irq = true };
	struct dji_motor_config arm_cfg = {
		.model = DJI_MODEL_C610, .id = 1, .gear_ratio = 36.0f,
	};
	struct dji_motor_config wheel_cfg = {
		.model = DJI_MODEL_C620, .id = 2,
		.gear_ratio = 3591.0f / 187.0f,
	};
	int err;

	err = bxcan_init(&cfg, &bx);
	if (!err)
		err = dji_init(&bus, bxcan_can(bx));
	if (!err)
		err = dji_add(&bus, &arm_cfg, &arm);
	if (!err)
		err = dji_add(&bus, &wheel_cfg, &wheel);
	if (err)
		printf("esc example: %d\n", err);
}

/* The HAL calls it from HAL_CAN_IRQHandler(). */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
		bxcan_irq(bx);
}

/* From a 1 kHz tick in thread context; dt_s is the time since the last. */
void esc_example_step(float dt_s, float arm_amps, float wheel_amps)
{
	struct can_dev *can = bxcan_can(bx);
	struct can_frame frame;

	if (!can)
		return;
	while (can_recv(can, &frame, 0) == 0)
		dji_handle_frame(&bus, &frame);
	esc_set_command(arm, ESC_COMMAND_CURRENT, arm_amps);
	esc_set_command(wheel, ESC_COMMAND_CURRENT, wheel_amps);
	dji_update(&bus, dt_s);
}

/* From the main loop, now and then. */
void esc_example_report(void)
{
	struct esc_feedback fb;

	if (arm && esc_get_feedback(arm, &fb) == 0)
		esc_log_feedback(arm, &fb);
	if (wheel && esc_get_feedback(wheel, &fb) == 0)
		esc_log_feedback(wheel, &fb);
}
