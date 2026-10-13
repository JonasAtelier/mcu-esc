/*
 * The DJI RoboMaster CAN protocol, each value with its source. Page numbers
 * are the PDF's own pages, in docs/ (gitignored):
 *
 *   [C620]   C620 Brushless DC Motor Speed Controller User Manual V1.0
 *            2017.09, "CAN Communication Protocol", pages 14-17;
 *            "Specifications", page 20.
 *   [C610]   C610 Brushless DC Motor Speed Controller User Guide v1.0
 *            2019.03, "CAN Communication Protocol", pages 8-9;
 *            "Specifications", page 9.
 *   [GM6020] GM6020 Brushless DC Motor User Guide v1.4 2023.10:
 *            "Introduction", item 2 "CAN Port", page 3; "DIP Switch", page 6;
 *            "Usage" and "CAN Communication Protocol", pages 7-9.
 *            v1.2 2020.05 compared: it gives the voltage range as +-30000
 *            (page 8) and has no current mode.
 */
#ifndef DJI_PROTO_H
#define DJI_PROTO_H

/* Standard frames, DATA, DLC 8, on every message of all three. */
#define DJI_FRAME_DLC		8

/* [C620] p.20 "CAN Bus Bitrate 1 Mbps"; [C610] p.9; [GM6020] p.3. */
#define DJI_BITRATE		1000000u

/*
 * C610 and C620 command frames: 0x200 carries ESC IDs 1-4, 0x1FF IDs 5-8,
 * in DATA[0..1], [2..3], [4..5], [6..7], high byte first. [C620] pp.14-15
 * "Speed Controller Receiving Message Format"; [C610] p.8.
 */
#define DJI_C6X0_COMMAND_LOW	0x200u
#define DJI_C6X0_COMMAND_HIGH	0x1FFu
/* Feedback: 0x200 + ESC ID. [C620] p.16; [C610] p.8. */
#define DJI_C6X0_FEEDBACK_BASE	0x200u
#define DJI_C6X0_ID_MAX		8

/* [C620] p.14: -16384..16384 is -20..20 A of torque current. */
#define DJI_C620_RAW_MAX	16384
#define DJI_C620_AMPS_MAX	20.0f
/* [C610] p.8: -10000..10000 is -10..10 A. */
#define DJI_C610_RAW_MAX	10000
#define DJI_C610_AMPS_MAX	10.0f

/*
 * GM6020: DIP switch Bit[2:0] is the ID, 1-7 (000 invalid); feedback
 * 0x205-0x20B; control 0x1FF for IDs 1-4, 0x2FF for 5-7. [GM6020] p.6.
 * Voltage mode: 0x1FF / 0x2FF, -25000..25000, unit not given, DATA[6..7] of
 * 0x2FF Null. [GM6020] p.7. Current mode: 0x1FE / 0x2FE, -16384..16384 is
 * -3..3 A of torque current. [GM6020] p.8. Feedback 0x204 + driver ID.
 * [GM6020] p.8.
 */
#define DJI_GM6020_VOLTAGE_LOW	0x1FFu
#define DJI_GM6020_VOLTAGE_HIGH	0x2FFu
#define DJI_GM6020_CURRENT_LOW	0x1FEu
#define DJI_GM6020_CURRENT_HIGH	0x2FEu
#define DJI_GM6020_FEEDBACK_BASE 0x204u
#define DJI_GM6020_ID_MAX	7
#define DJI_GM6020_VOLTAGE_MAX	25000
#define DJI_GM6020_CURRENT_MAX	16384
#define DJI_GM6020_AMPS_MAX	3.0f

/* Four 16-bit commands per frame. */
#define DJI_MOTORS_PER_FRAME	4

/*
 * Feedback, all three: DATA[0..1] rotor mechanical angle, 0..8191 for
 * 0..360 deg; DATA[2..3] rotational speed, rpm; DATA[4..5] actual torque
 * current, no scale given; DATA[6] motor temperature -- deg C on the C620
 * (p.17), unit not given on the GM6020 (p.9), Null on the C610 (p.9);
 * DATA[7] Null. [C620] pp.16-17; [C610] pp.8-9; [GM6020] pp.8-9.
 */
#define DJI_ANGLE_COUNTS	8192

#endif /* DJI_PROTO_H */
