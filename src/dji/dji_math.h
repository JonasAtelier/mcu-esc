/*
 * Pure conversions for the DJI frames, compiled and asserted on the host.
 */
#ifndef DJI_MATH_H
#define DJI_MATH_H

#include <math.h>
#include <stdint.h>

#include "dji_proto.h"

#define DJI_TWO_PI		6.28318530717958647692f

/* Every 16-bit field is high byte first. */
static inline uint16_t dji_read_be16(const uint8_t bytes[static 2])
{
	return (uint16_t)((bytes[0] << 8) | bytes[1]);
}

static inline void dji_write_be16(uint8_t bytes[static 2], int16_t value)
{
	bytes[0] = (uint8_t)((uint16_t)value >> 8);
	bytes[1] = (uint8_t)((uint16_t)value & 0xFF);
}

/*
 * Rotor counts moved from previous to now, both 0..8191. A step over half a
 * turn is the wrap, not motion: at the feedback's default 1 kHz that tracks
 * up to 30000 rotor rpm, beyond any of the three motors.
 */
static inline int32_t dji_angle_step(uint16_t previous, uint16_t now)
{
	int32_t step = (int32_t)now - (int32_t)previous;

	if (step > DJI_ANGLE_COUNTS / 2)
		step -= DJI_ANGLE_COUNTS;
	else if (step < -(DJI_ANGLE_COUNTS / 2))
		step += DJI_ANGLE_COUNTS;
	return step;
}

/* Clamp to +-raw_max. */
static inline int16_t dji_clamp_raw(int32_t value, int16_t raw_max)
{
	if (value > raw_max)
		return raw_max;
	if (value < -raw_max)
		return (int16_t)-raw_max;
	return (int16_t)value;
}

/*
 * Amperes to command counts, rounded to the nearest and clamped to the
 * range, so +-amps_max and beyond are exactly +-raw_max. value is finite.
 */
static inline int16_t dji_amps_to_raw(float amps, float amps_max,
				      int16_t raw_max)
{
	float scaled = amps / amps_max * (float)raw_max;

	if (scaled >= (float)raw_max)
		return raw_max;
	if (scaled <= -(float)raw_max)
		return (int16_t)-raw_max;
	return (int16_t)lroundf(scaled);
}

/* Rotor counts to radians of the shaft gear_ratio turns slower. */
static inline float dji_counts_to_rad(int64_t counts, float gear_ratio)
{
	return (float)counts * DJI_TWO_PI / (float)DJI_ANGLE_COUNTS /
	       gear_ratio;
}

/* Rotor rpm to rad/s of the shaft gear_ratio turns slower. */
static inline float dji_rpm_to_rad_s(int16_t rpm, float gear_ratio)
{
	return (float)rpm * DJI_TWO_PI / 60.0f / gear_ratio;
}

#endif /* DJI_MATH_H */
