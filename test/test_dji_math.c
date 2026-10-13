/*
 * The DJI conversions against the manuals' numbers, each vector worked by
 * hand. dji_math.h only, no bus.
 *
 *   [C620]   User Manual V1.0, p.14: -16384..16384 is -20..20 A.
 *   [C610]   User Guide v1.0, p.8:   -10000..10000 is -10..10 A.
 *   [GM6020] User Guide v1.4, p.7: voltage -25000..25000;
 *            p.8: current -16384..16384 is -3..3 A.
 *   All:     angle 0..8191 for one turn, speed in rpm, 16-bit fields high
 *            byte first ([C620] pp.16-17, [C610] pp.8-9, [GM6020] pp.8-9).
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "dji_math.h"
#include "dji_proto.h"

static void near(float actual, float expected)
{
	assert(fabsf(actual - expected) < 1e-4f * (1.0f + fabsf(expected)));
}

static void test_byte_order(void)
{
	uint8_t bytes[2];
	const uint8_t angle[2] = { 0x1F, 0xFF };	/* 8191 */
	const uint8_t speed[2] = { 0xFC, 0x18 };	/* -1000 rpm */

	dji_write_be16(bytes, 0x4000);
	assert(bytes[0] == 0x40 && bytes[1] == 0x00);
	dji_write_be16(bytes, -16384);
	assert(bytes[0] == 0xC0 && bytes[1] == 0x00);
	dji_write_be16(bytes, -1);
	assert(bytes[0] == 0xFF && bytes[1] == 0xFF);
	assert(dji_read_be16(angle) == 8191);
	assert((int16_t)dji_read_be16(speed) == -1000);
}

static void test_current_scales(void)
{
	/* [C620]: full scale both ways, half, and the rounding of 1 A */
	assert(dji_amps_to_raw(20.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       16384);
	assert(dji_amps_to_raw(-20.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       -16384);
	assert(dji_amps_to_raw(10.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       8192);
	/* 1 A = 819.2 counts -> 819; -1.5 A = -1228.8 -> -1229 */
	assert(dji_amps_to_raw(1.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       819);
	assert(dji_amps_to_raw(-1.5f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       -1229);
	/* past the range: clamped, never wrapped */
	assert(dji_amps_to_raw(25.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       16384);
	assert(dji_amps_to_raw(-1e30f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) ==
	       -16384);
	assert(dji_amps_to_raw(0.0f, DJI_C620_AMPS_MAX, DJI_C620_RAW_MAX) == 0);

	/* [C610]: 1 A = 1000 counts */
	assert(dji_amps_to_raw(10.0f, DJI_C610_AMPS_MAX, DJI_C610_RAW_MAX) ==
	       10000);
	assert(dji_amps_to_raw(-10.0f, DJI_C610_AMPS_MAX, DJI_C610_RAW_MAX) ==
	       -10000);
	assert(dji_amps_to_raw(2.5f, DJI_C610_AMPS_MAX, DJI_C610_RAW_MAX) ==
	       2500);
	assert(dji_amps_to_raw(11.0f, DJI_C610_AMPS_MAX, DJI_C610_RAW_MAX) ==
	       10000);

	/* [GM6020] current mode: 3 A full scale; 1.5 A = 8192 */
	assert(dji_amps_to_raw(3.0f, DJI_GM6020_AMPS_MAX,
			       DJI_GM6020_CURRENT_MAX) == 16384);
	assert(dji_amps_to_raw(-1.5f, DJI_GM6020_AMPS_MAX,
			       DJI_GM6020_CURRENT_MAX) == -8192);

	/* [GM6020] voltage mode, raw only: v1.2's 30000 clamps to 25000 */
	assert(dji_clamp_raw(30000, DJI_GM6020_VOLTAGE_MAX) == 25000);
	assert(dji_clamp_raw(-30000, DJI_GM6020_VOLTAGE_MAX) == -25000);
	assert(dji_clamp_raw(-24999, DJI_GM6020_VOLTAGE_MAX) == -24999);
	assert(dji_clamp_raw(-25001, DJI_GM6020_VOLTAGE_MAX) == -25000);
	assert(dji_clamp_raw(25001, DJI_GM6020_VOLTAGE_MAX) == 25000);
}

static void test_identifiers(void)
{
	/* [C620] pp.14-16, [C610] p.8 */
	assert(DJI_C6X0_COMMAND_LOW == 0x200 && DJI_C6X0_COMMAND_HIGH == 0x1FF);
	assert(DJI_C6X0_FEEDBACK_BASE + 1 == 0x201 &&
	       DJI_C6X0_FEEDBACK_BASE + DJI_C6X0_ID_MAX == 0x208);
	/* [GM6020] p.6: IDs 1-7, feedback 0x205-0x20B; pp.7-8 commands */
	assert(DJI_GM6020_FEEDBACK_BASE + 1 == 0x205 &&
	       DJI_GM6020_FEEDBACK_BASE + DJI_GM6020_ID_MAX == 0x20B);
	assert(DJI_GM6020_VOLTAGE_LOW == 0x1FF &&
	       DJI_GM6020_VOLTAGE_HIGH == 0x2FF);
	assert(DJI_GM6020_CURRENT_LOW == 0x1FE &&
	       DJI_GM6020_CURRENT_HIGH == 0x2FE);
	assert(DJI_BITRATE == 1000000u && DJI_FRAME_DLC == 8);
}

static void test_angle_step(void)
{
	assert(dji_angle_step(100, 150) == 50);
	assert(dji_angle_step(150, 100) == -50);
	/* forward across the wrap: 8190 -> 2 is +4 */
	assert(dji_angle_step(8190, 2) == 4);
	/* backward across it: 2 -> 8190 is -4 */
	assert(dji_angle_step(2, 8190) == -4);
	/* half a turn exactly is taken as motion; one more is the wrap */
	assert(dji_angle_step(0, 4096) == 4096);
	assert(dji_angle_step(0, 4097) == -4095);
	assert(dji_angle_step(4096, 0) == -4096);
	assert(dji_angle_step(4097, 0) == 4095);
}

static void test_units(void)
{
	const float m3508 = 3591.0f / 187.0f;

	near(dji_counts_to_rad(8192, 1.0f), DJI_TWO_PI);
	near(dji_counts_to_rad(2048, 1.0f), DJI_TWO_PI / 4.0f);
	near(dji_counts_to_rad(-8192 * 3, 1.0f), -3.0f * DJI_TWO_PI);
	/* past int32: 2^33 counts are 2^20 turns */
	near(dji_counts_to_rad(8589934592LL, 1.0f), 1048576.0f * DJI_TWO_PI);
	/* 3591/187 rotor turns are one output turn */
	near(dji_counts_to_rad((int32_t)(8192.0f * m3508), m3508), DJI_TWO_PI);
	near(dji_rpm_to_rad_s(60, 1.0f), DJI_TWO_PI);
	near(dji_rpm_to_rad_s(-600, 1.0f), -10.0f * DJI_TWO_PI);
	near(dji_rpm_to_rad_s(360, 36.0f), DJI_TWO_PI / 6.0f);
}

int main(void)
{
	test_byte_order();
	test_current_scales();
	test_identifiers();
	test_angle_step();
	test_units();
	printf("test_dji_math: all passed\n");
	return 0;
}
