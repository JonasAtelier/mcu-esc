/*
 * The DShot protocol, each value with its source. There is no official
 * specification; these are the ones used, all kept outside the repository
 * (docs/chips.md, "DShot", lists them with their URLs and commits):
 *
 *   [BW]   Chris Landa, "DSHOT - the missing Handbook", brushlesswhoop.com,
 *          2021-02-27: sections "Frame Structure", "Special commands",
 *          "Calculating the CRC", "Bidirectional DSHOT", "eRPM Transmission".
 *   [BLH]  BLHeli_32 "Digital_Cmd_Spec.txt" (bitdump/BLHeli master),
 *          lines 1-31: commands, repeats and waits.
 *   [EDT]  bird-sanctuary/extended-dshot-telemetry README @438b8798,
 *          "eRPM Frames", "EDT Frames", "Checksum".
 *   [BF]   Betaflight src/main/drivers/dshot.c @4b7fa8e4, lines 206-221
 *          and 393-395, and dshot_bitbang_decode.c, read for facts only
 *          (GPL): the reply's period, 0x0FFF, RPM from eRPM, 3x sampling.
 *   [IDF]  ESP-IDF v5.5.5 examples/peripherals/rmt/dshot_esc (Apache-2.0),
 *          main/dshot_esc_encoder.c:41 and :132-135, as a cross-check.
 */
#ifndef DSHOT_PROTO_H
#define DSHOT_PROTO_H

/*
 * A frame is 16 bits, most significant first: an 11-bit value, a telemetry
 * request bit, a 4-bit CRC over the twelve before it [BW "Frame Structure"].
 * Value 0 is disarmed (motor stop), 1-47 special commands, 48-2047 throttle.
 */
#define DSHOT_FRAME_BITS	16
#define DSHOT_VALUE_MAX		2047
#define DSHOT_THROTTLE_MIN	48
#define DSHOT_THROTTLE_MAX	2047

/*
 * Every bit lasts one bit period; a 1 is high for 3/4 of it, a 0 for 3/8:
 * [BW "Frame Structure"] table, T1H/bit and T0H/bit at every rate --
 * 5.00/6.67 and 2.50/6.67 us at DShot150, 2.50/3.33 and 1.25/3.33 at 300,
 * 1.25/1.67 and 0.625/1.67 at 600. [IDF] uses 0.7485 and 0.37425.
 */
#define DSHOT_ONE_HIGH_NUM	3
#define DSHOT_ONE_HIGH_DEN	4
#define DSHOT_ZERO_HIGH_NUM	3
#define DSHOT_ZERO_HIGH_DEN	8

/*
 * Bidirectional DShot [BW "Bidirectional DSHOT"]: the line is inverted --
 * it rests high and every bit starts low -- and the frame's CRC too. Only
 * at DShot300 and up.
 */
#define DSHOT_BIDIR_SPEED_MIN	300

/*
 * The reply [BW "eRPM Transmission"]: 21 bits, a leading 0 and then 20 GCR
 * bits, at 5/4 of the frame's bit rate, not inverted, so the line falls
 * for the first. It starts 30 us after the frame, whatever the speed [EDT
 * "Timing"; BW "Bidirectional DSHOT"].
 */
#define DSHOT_REPLY_BITS	21	/* a leading 0, then 20 GCR bits */
#define DSHOT_GCR_BITS		20	/* four 5-bit quintets, one per nibble */
#define DSHOT_REPLY_RATE_NUM	5
#define DSHOT_REPLY_RATE_DEN	4
#define DSHOT_REPLY_DELAY_US	30

/*
 * The longest the line holds one level inside a reply: 3 bits. No quintet
 * of the GCR table has more than two 0s in a row, none starts or ends with
 * more than one, and a GCR 0 is no change of level [BW "eRPM
 * Transmission"]; test_dshot_math checks it over all 65536 frames. A level
 * held longer than DSHOT_REPLY_RUN_MAX + 1 bits is not part of a reply.
 */
#define DSHOT_REPLY_RUN_MAX	3

/*
 * How long after a frame the library waits for a reply to start: [EDT]'s
 * 30 us and half again, a margin the sources do not give.
 */
#define DSHOT_REPLY_WAIT_US	45

/*
 * Samples per reply bit where the line is read at a fixed rate (the STM32
 * port): Betaflight's bit-banged decoder reads 3 per bit [BF
 * dshot_bitbang_decode.c:181-183].
 */
#define DSHOT_REPLY_OVERSAMPLE	3

/*
 * Extended DShot Telemetry [EDT "EDT Frames"]: type in bits 11-9, value in
 * 7-0. Type 6 is the stress level since EDT v2.0.0; [BW] still calls it
 * debug 3. Status: bit 7 alert, 6 warning, 5 error, 3-0 the highest
 * stress level. After command 13 the ESC sends one status-type frame with
 * its EDT version, after command 14 one of 0xEFF [EDT "Enabling EDT",
 * "Disabling EDT"].
 */
#define DSHOT_EDT_TEMPERATURE	1	/* deg C */
#define DSHOT_EDT_VOLTAGE	2	/* 0.25 V a step */
#define DSHOT_EDT_CURRENT	3	/* A */
#define DSHOT_EDT_DEBUG1	4
#define DSHOT_EDT_DEBUG2	5
#define DSHOT_EDT_STRESS	6
#define DSHOT_EDT_STATUS	7
#define DSHOT_EDT_OFF_REPLY	0xEFF
#define DSHOT_REPLY_STOPPED	0xFFF	/* eRPM: motor stopped [BF dshot.c:209-211] */

/*
 * Repeats and waits per special command [BLH] lines 1-27; 32-35 and 42-47
 * from [BW "Special commands"], which [BLH] does not list. Commands 0-36
 * are only executed with the motor stopped [BW], and every command frame
 * carries the telemetry bit, with throttle zero [BLH] line 30.
 */
#define DSHOT_COMMAND_REPEATS		6	/* "Need 6x" */
#define DSHOT_BEEP12_WAIT_S		0.260f	/* BEEP1, BEEP2 [BLH] 2-3 */
#define DSHOT_BEEP34_WAIT_S		0.280f	/* BEEP3, BEEP4 [BLH] 4-5 */
#define DSHOT_BEEP5_WAIT_S		1.020f	/* BEEP5 [BLH] 6 */
#define DSHOT_ESC_INFO_WAIT_S		0.012f	/* [BLH] 7 */
#define DSHOT_SAVE_WAIT_S		0.035f	/* SAVE_SETTINGS, EDT on/off
						 * [BLH] 13-15 */
#define DSHOT_STOPPED_ONLY_MAX		36	/* [BW]: 0-36 need the motor stopped */

#endif /* DSHOT_PROTO_H */
