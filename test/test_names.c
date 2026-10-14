/*
 * The coexistence check, at compile time: one translation unit with every
 * header an ESP-IDF app holding esp-c620-control, esp-sn65hvd230-can,
 * mcu-can and this library may include together -- rm.h, hvd230can.h and
 * upid.h (fixtures/, verbatim copies of esp-c620-control a00d3e9,
 * esp-sn65hvd230-can 28593dc and upid 4d96e50), mcu-can's can.h, and every
 * header here. A type, tag, enumerator or prototype both define fails the
 * build, and so, under -Werror, does a macro defined twice differently. The
 * link-time half -- no global symbol defined by both -- is checked on a real
 * firmware build with nm; see docs/development.md.
 */
#include <stdio.h>

#include "hvd230can.h"
#include "rm.h"
#include "upid.h"

#include "can.h"
#include "c615.h"
#include "dji.h"
#include "dshot.h"
#include "esc.h"
#include "esc_log.h"
#include "ht03.h"

int main(void)
{
	/* One of each side's types, so each header is used, not skipped. */
	rm_t rm;
	rm_feedback_t old_feedback = { .angle = 1 };
	can_frame_t old_frame = { .id = 2 };
	struct can_frame frame = { .id = 3 };
	struct esc_feedback feedback = { .has = ESC_HAS_ANGLE };
	struct dji_bus bus;
	struct c615_config c615 = { .min_us = C615_PULSE_MIN_US };
	ht03_t ht;
	ht03_range_t j6 = HT03_J6;
	struct dshot_bus dshot;

	(void)rm;
	(void)bus;
	(void)ht;
	(void)dshot;
	printf("test_names: %u %u %u %u %u %u %u %.1f: no clash\n",
	       (unsigned)old_feedback.angle, (unsigned)old_frame.id,
	       (unsigned)frame.id, (unsigned)feedback.has,
	       (unsigned)C620_TXID_1_4, (unsigned)DJI_MOTORS_MAX,
	       (unsigned)c615.min_us, (double)j6.torque_max);
	return 0;
}
