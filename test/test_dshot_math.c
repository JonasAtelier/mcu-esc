/*
 * The DShot arithmetic against the sources' own worked examples, each
 * cited. dshot_math.h only, no hardware.
 *
 *   [BW]  brushlesswhoop.com "DSHOT - the missing Handbook": the timing
 *         table, the CRC of 1046 both ways, the GCR table and its two
 *         examples.
 *   [EDT] extended-dshot-telemetry README @438b8798: the eRPM ranges and
 *         EDT prefixes.
 *   [BF]  Betaflight dshot.c @4b7fa8e4 lines 206-221 (facts only): 0x0FFF
 *         is stopped, the shifted value is a period in microseconds.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>

#include "dshot_math.h"
#include "dshot_proto.h"

/* The 21 line bits for a 16-bit frame, by [BW "eRPM Transmission"]: the
 * GCR table, then a leading 0 and a change of level for every 1. */
static uint32_t gcr_of(uint16_t frame)
{
	static const uint8_t to_quintet[16] = {
		0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
		0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
	};
	uint32_t gcr = 0;

	for (int shift = 12; shift >= 0; shift -= 4)
		gcr = (gcr << 5) | to_quintet[(frame >> shift) & 0xF];
	return gcr;
}

static uint32_t line_of(uint32_t gcr)
{
	uint32_t line = 0, level = 0;

	for (int bit = 19; bit >= 0; bit--) {
		if (gcr & (1u << bit))
			level ^= 1;
		line = (line << 1) | level;
	}
	return line;		/* bit 20, the leading 0, is clear */
}

static void test_crc_and_frame(void)
{
	/* [BW "Calculating the CRC"]: 1046, no telemetry: 1000001011000110 */
	assert(dshot_crc(0x82C, false) == 0x6);
	assert(dshot_frame(1046, false, false) == 0x82C6);
	/* [BW "Bidirectional DSHOT"]: the same, inverted: 1000001011001001 */
	assert(dshot_crc(0x82C, true) == 0x9);
	assert(dshot_frame(1046, false, true) == 0x82C9);

	/* By the formula, worked by hand */
	assert(dshot_frame(0, false, false) == 0x0000);
	assert(dshot_frame(0, true, false) == 0x0011);
	assert(dshot_frame(0, false, true) == 0x000F);
	/* 0xFFE ^ 0x0FF ^ 0x00F = 0xF0E: crc E */
	assert(dshot_frame(2047, false, false) == 0xFFEE);
	/* 0xFFF ^ 0x0FF ^ 0x00F = 0xF0F: crc F, inverted 0 */
	assert(dshot_frame(2047, true, false) == 0xFFFF);
	assert(dshot_frame(2047, true, true) == 0xFFF0);
	/* command 21 with the bit: 0x02B ^ 0x002 = 0x029: crc 9 */
	assert(dshot_frame(21, true, false) == 0x02B9);
	/* 48: 0x060 ^ 0x006 = 0x066: crc 6 */
	assert(dshot_frame(48, false, false) == 0x0606);
}

static void test_timing(void)
{
	/* [BW] table at 40 MHz: 1 high 5.00/2.50/1.25 us, 0 2.50/1.25/0.625 */
	assert(dshot_bit_ticks(40000000u, 150) == 267);
	assert(dshot_bit_ticks(40000000u, 300) == 133);
	assert(dshot_bit_ticks(40000000u, 600) == 67);
	assert(dshot_high_ticks(267, true) == 200);
	assert(dshot_high_ticks(267, false) == 100);
	assert(dshot_high_ticks(133, true) == 100);
	assert(dshot_high_ticks(133, false) == 50);
	assert(dshot_high_ticks(67, true) == 50);
	assert(dshot_high_ticks(67, false) == 25);
	/* 8 MHz, the RMT's: bits 53.3/26.7/13.3 -> 53, 27, 13, and the highs
	 * exactly [BW]'s 5.000/2.500, 2.500/1.250, 1.250/0.625 us */
	assert(dshot_bit_ticks(8000000u, 150) == 53);
	assert(dshot_bit_ticks(8000000u, 300) == 27);
	assert(dshot_bit_ticks(8000000u, 600) == 13);
	assert(dshot_high_ticks(53, true) == 40);
	assert(dshot_high_ticks(53, false) == 20);
	assert(dshot_high_ticks(27, true) == 20);
	assert(dshot_high_ticks(27, false) == 10);
	assert(dshot_high_ticks(13, true) == 10);
	assert(dshot_high_ticks(13, false) == 5);
	/* 168 MHz, exact: 3/4 and 3/8 of 1120, 560, 280 */
	assert(dshot_bit_ticks(168000000u, 150) == 1120);
	assert(dshot_bit_ticks(168000000u, 300) == 560);
	assert(dshot_bit_ticks(168000000u, 600) == 280);
	assert(dshot_high_ticks(1120, true) == 840);
	assert(dshot_high_ticks(1120, false) == 420);
	assert(dshot_high_ticks(280, true) == 210);
	assert(dshot_high_ticks(280, false) == 105);
	/* rounding the bit: 100 MHz at 600k is 166.67 -> 167, 300k 333.3 */
	assert(dshot_bit_ticks(100000000u, 600) == 167);
	assert(dshot_bit_ticks(100000000u, 300) == 333);
	/* rounding the highs: 167 * 3/4 = 125.25, * 3/8 = 62.625 */
	assert(dshot_high_ticks(167, true) == 125);
	assert(dshot_high_ticks(167, false) == 63);
}

static void test_throttle(void)
{
	assert(dshot_throttle_value(0.0f) == 0);
	assert(dshot_throttle_value(-1.0f) == 0);
	assert(dshot_throttle_value(1e-6f) == DSHOT_THROTTLE_MIN);
	assert(dshot_throttle_value(1.0f) == DSHOT_THROTTLE_MAX);
	assert(dshot_throttle_value(5.0f) == DSHOT_THROTTLE_MAX);
	/* 48 + 0.5 * 1999 = 1047.5 -> 1048 */
	assert(dshot_throttle_value(0.5f) == 1048);
	/* 48 + 0.25 * 1999 = 547.75 -> 548 */
	assert(dshot_throttle_value(0.25f) == 548);
}

static void test_reply(void)
{
	uint16_t data;
	uint32_t erpm;

	/* [BW]'s example: 1000001011000110 -> GCR 11010 10010 11110 10110 */
	assert(gcr_of(0x82C6) == 0xD4BD6);
	assert(dshot_reply_decode(line_of(0xD4BD6), &data) == 0);
	assert(data == 0x82C);

	/* [BW]'s worst case: GCR 1010...10 is line 011001100110011001100,
	 * quintets 15 0A 15 0A, nibbles 5 A 5 A: data 0x5A5, crc A */
	assert(line_of(0xAAAAA) == 0x0CCCCC);
	assert(dshot_reply_decode(0x0CCCCC, &data) == 0);
	assert(data == 0x5A5);

	/* The leading bit does not matter, only the changes do */
	assert(dshot_reply_decode(line_of(0xD4BD6) ^ 0x1FFFFF, &data) == 0);
	assert(data == 0x82C);

	/* A quintet outside the table, a CRC that does not match */
	data = 0x123;
	assert(dshot_reply_decode(0, &data) == -EBADMSG);
	assert(dshot_reply_decode(line_of(gcr_of(0x82C7)), &data) ==
	       -EBADMSG);
	assert(data == 0x123);
	/* the reply's CRC is the plain one: an inverted CRC is refused */
	assert(dshot_reply_decode(line_of(gcr_of(0x82C9)), &data) ==
	       -EBADMSG);
	for (uint32_t quintet = 0; quintet < 32; quintet++) {
		int nibble = dshot_gcr_nibble(quintet);

		if (nibble >= 0)
			assert(gcr_of((uint16_t)(nibble << 12)) >> 15 ==
			       quintet);
	}

	/* Period m << e [BW; BF]: e 2, m 256 is 1024 us, 58593.75 eRPM */
	assert(dshot_reply_period_us((2 << 9) | 256) == 1024);
	assert(dshot_reply_erpm((2 << 9) | 256, &erpm) == 0 && erpm == 58594);
	/* [EDT "eRPM Frames"]: 001 1 0000 0000 is 512 */
	assert(dshot_reply_period_us(0x300) == 512);
	/* 0x0FFF is a stopped motor [BF dshot.c:209-211] */
	assert(dshot_reply_erpm(0x0FFF, &erpm) == 0 && erpm == 0);
	/* period 0 is not a speed */
	erpm = 7;
	assert(dshot_reply_erpm(0x0E00, &erpm) == -EBADMSG && erpm == 7);
	/* 1 us: 60000000 eRPM; 65280 us (e 7, m 510): 919.1 -> 919 */
	assert(dshot_reply_erpm(1, &erpm) == 0 && erpm == 60000000u);
	assert(dshot_reply_erpm((7 << 9) | 510, &erpm) == 0 && erpm == 919);

	/* [EDT]: prefix 0 or bit 8 set is eRPM; 001 0 is temperature */
	assert(!dshot_reply_is_edt(0x0FF));
	assert(!dshot_reply_is_edt(0x1FF));
	assert(!dshot_reply_is_edt(0x3FF));
	assert(dshot_reply_is_edt(0x219));	/* 001 0, 25 deg C */
	assert(dshot_reply_is_edt(0xE00));	/* 111 0, status */
	assert(!dshot_reply_is_edt(0xF00));
}

int main(void)
{
	test_crc_and_frame();
	test_timing();
	test_throttle();
	test_reply();
	printf("test_dshot_math: all passed\n");
	return 0;
}
