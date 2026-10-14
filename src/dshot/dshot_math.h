/*
 * Pure DShot arithmetic, compiled and asserted on the host: the frame and
 * its CRC both ways, pulse widths, throttle, and the decoding of the
 * bidirectional reply (GCR, eRPM period, EDT) that phase 2's capture will
 * feed. Sources in dshot_proto.h.
 */
#ifndef DSHOT_MATH_H
#define DSHOT_MATH_H

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "dshot_proto.h"

/*
 * The 4-bit CRC over the twelve data bits (value << 1 | telemetry):
 * crc = (d ^ (d >> 4) ^ (d >> 8)) & 0x0F [BW "Calculating the CRC"; IDF
 * dshot_esc_encoder.c:41]. Bidirectional DShot inverts it before the mask
 * [BW "Bidirectional DSHOT"].
 */
static inline uint16_t dshot_crc(uint16_t data, bool inverted)
{
	uint16_t crc = (uint16_t)(data ^ (data >> 4) ^ (data >> 8));

	return (uint16_t)((inverted ? ~crc : crc) & 0x0F);
}

/* The 16-bit frame for an 11-bit value; value is 0..DSHOT_VALUE_MAX. */
static inline uint16_t dshot_frame(uint16_t value, bool telemetry,
				   bool inverted_crc)
{
	uint16_t data = (uint16_t)((value << 1) | (telemetry ? 1 : 0));

	return (uint16_t)((data << 4) | dshot_crc(data, inverted_crc));
}

/* High time of one bit, in the bit period's own ticks, rounded. */
static inline uint32_t dshot_high_ticks(uint32_t bit_ticks, bool one)
{
	if (one)
		return (bit_ticks * DSHOT_ONE_HIGH_NUM + DSHOT_ONE_HIGH_DEN / 2) /
		       DSHOT_ONE_HIGH_DEN;
	return (bit_ticks * DSHOT_ZERO_HIGH_NUM + DSHOT_ZERO_HIGH_DEN / 2) /
	       DSHOT_ZERO_HIGH_DEN;
}

/* Ticks of a clock at clock_hz in one bit at kbit_s, rounded. */
static inline uint32_t dshot_bit_ticks(uint32_t clock_hz, uint32_t kbit_s)
{
	uint32_t rate = kbit_s * 1000u;

	return (clock_hz + rate / 2) / rate;
}

/*
 * Throttle 0..1 to a frame value: 0 is motor stop (value 0); anything
 * above it is 48..2047, linear and rounded, clamped at 1. throttle is
 * finite.
 */
static inline uint16_t dshot_throttle_value(float throttle)
{
	const float span = (float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN);

	if (throttle <= 0.0f)
		return 0;
	if (throttle >= 1.0f)
		return DSHOT_THROTTLE_MAX;
	return (uint16_t)(DSHOT_THROTTLE_MIN + lroundf(throttle * span));
}

/* ---------------------------------------- the reply, for phase 2 -- */

/*
 * One GCR quintet back to its nibble, or -1. The table is [BW "eRPM
 * Transmission"]: nibble 0-F map to 19 1B 12 13 1D 15 16 17 1A 09 0A 0B 1E
 * 0D 0E 0F.
 */
static inline int dshot_gcr_nibble(uint32_t quintet)
{
	static const uint8_t to_quintet[16] = {
		0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
		0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
	};

	for (int nibble = 0; nibble < 16; nibble++)
		if (to_quintet[nibble] == quintet)
			return nibble;
	return -1;
}

/*
 * The 21 bits as they come off the line, the first in bit 20, to the
 * 12 data bits. A 1 in the GCR stream is a change of level, so
 * gcr = raw ^ (raw >> 1) [BW "Decoding eRPM frame"]; four quintets give the
 * 16-bit frame, whose CRC is the plain one, not inverted [BW "eRPM
 * Telemetry Frame"; EDT "Checksum"]. -EBADMSG for a quintet outside the
 * table or a CRC that does not match.
 */
static inline int dshot_reply_decode(uint32_t raw, uint16_t *data)
{
	uint32_t gcr = (raw ^ (raw >> 1)) & ((1u << DSHOT_GCR_BITS) - 1);
	uint16_t frame = 0;

	for (int shift = DSHOT_GCR_BITS - 5; shift >= 0; shift -= 5) {
		int nibble = dshot_gcr_nibble((gcr >> shift) & 0x1F);

		if (nibble < 0)
			return -EBADMSG;
		frame = (uint16_t)((frame << 4) | nibble);
	}
	if (dshot_crc((uint16_t)(frame >> 4), false) != (frame & 0x0F))
		return -EBADMSG;
	*data = (uint16_t)(frame >> 4);
	return 0;
}

/*
 * Would a decoded reply be an Extended DShot Telemetry frame? eRPM is a
 * prefix (bits 11-8) of 0 or bit 8 set [EDT "eRPM Frames"]; the rest is
 * EDT, type in bits 11-9, value in 7-0. Only ask once EDT is enabled
 * (command 13): before that, every reply is eRPM.
 */
static inline bool dshot_reply_is_edt(uint16_t data)
{
	return (data >> 8) != 0 && !(data & 0x100);
}

/*
 * The eRPM reply's period of one electrical turn, in microseconds:
 * eeem mmmm mmmm is m << e [BW "eRPM Telemetry Frame"; BF dshot.c:214].
 */
static inline uint32_t dshot_reply_period_us(uint16_t data)
{
	return (uint32_t)(data & 0x1FF) << ((data >> 9) & 0x7);
}

/*
 * Electrical turns per minute from a decoded eRPM reply, rounded; 0x0FFF,
 * the longest period, is a stopped motor [BF dshot.c:209-211]. A period of
 * 0 is not a speed: -EBADMSG.
 */
static inline int dshot_reply_erpm(uint16_t data, uint32_t *erpm)
{
	uint32_t period = dshot_reply_period_us(data);

	if (data == 0x0FFF) {
		*erpm = 0;
		return 0;
	}
	if (!period)
		return -EBADMSG;
	*erpm = (60000000u + period / 2) / period;
	return 0;
}

#endif /* DSHOT_MATH_H */
