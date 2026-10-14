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
 *   [BF]   Betaflight src/main/drivers/dshot.c @4b7fa8e4, lines 206-221,
 *          read for facts only (GPL): the reply's period and 0x0FFF.
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

/* The reply of bidirectional DShot, phase 2 [BW "eRPM Transmission"]. */
#define DSHOT_REPLY_BITS	21	/* a leading 0, then 20 GCR bits */
#define DSHOT_GCR_BITS		20	/* four 5-bit quintets, one per nibble */

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
