#ifndef RM_RM_H
#define RM_RM_H

#include <stdbool.h>
#include <stdint.h>

#include "hvd230can.h"
#include "upid.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Library for the RoboMaster M3508 P19 gear motor driven by a C620 speed
 *  controller, and nothing else. It carries no CAN driver and no PID maths of
 *  its own: every loop is a upid controller, and the bus is
 *  esp-sn65hvd230-can.
 *
 *  Please be reminded of these following points before use the library.
 *
 *  1. Speed controller IDs are 1 to 8, as blinked by the C620 status LED.
 *  2. One rm_t owns one whole CAN bus (8 motors max), because a single TX
 *     frame carries the current command of four controllers.
 *  3. Mode is current, speed or position, chosen per motor.
 *  4. No clock is read here. You pass dt to rm_update(), exactly like
 *     upid_spin().
 *  5. Feedback must keep arriving, or the motor is declared offline and
 *     commanded to zero current.
 *  6. There is no destructor in C. rm_init() before anything else, and
 *     rm_stop() before you stop calling rm_update().
 *  7. Please dont change the constant values.
 *
 *  M3508 P19 motor & C620 speed controller
 *  CAN Data Frame Structure (C620 User Guide V1.0, "CAN Communication
 *  Protocol"). All frames are standard format, DLC 8 bytes.
 *
 *  RX: id = 0x200 + speed controller ID  [0x201 to 0x208], 1 kHz by default
 *  Byte 0: Rotor mechanical angle [High byte]  0 to 8191 = 0 to 360 degrees
 *  Byte 1: Rotor mechanical angle [Low byte]
 *  Byte 2: Rotational speed [High byte]        int16, rotor rpm
 *  Byte 3: Rotational speed [Low byte]
 *  Byte 4: Actual torque current [High byte]   int16, +-16384 = +-20 A
 *  Byte 5: Actual torque current [Low byte]
 *  Byte 6: Motor temperature                   uint8, degrees C
 *  Byte 7: Null - Reserved
 *
 *  TX: id = 0x200 for speed controller ID 1 to 4
 *           0x1FF for speed controller ID 5 to 8
 *  Byte 0: Current for ID 1 or 5 [High byte]   int16, +-16384 = +-20 A
 *  Byte 1: Current for ID 1 or 5 [Low byte]
 *  Byte 2: Current for ID 2 or 6 [High byte]
 *  Byte 3: Current for ID 2 or 6 [Low byte]
 *  Byte 4: Current for ID 3 or 7 [High byte]
 *  Byte 5: Current for ID 3 or 7 [Low byte]
 *  Byte 6: Current for ID 4 or 8 [High byte]
 *  Byte 7: Current for ID 4 or 8 [Low byte]
 */

/* Library limits. Theoretical amount of motors on 1 CAN bus is 8. */
#define RM_MAX_MOTORS           8
#define RM_MIN_ID               1
#define RM_MAX_ID               8

/* Feedback older than this declares the motor offline */
#define RM_DEFAULT_TIMEOUT_S    0.1f

/* Protocol identifiers */
#define C620_TXID_1_4           0x200u  /* commands ID 1 to 4 */
#define C620_TXID_5_8           0x1FFu  /* commands ID 5 to 8 */
#define C620_RXID_BASE          0x200u  /* + speed controller ID */

/* Every C620 frame is 8 bytes, holding four big-endian 16-bit fields */
#define C620_FRAME_DLC          8
#define C620_MOTORS_PER_TX      4
#define C620_TX_FRAMES          2       /* 0x200 and 0x1FF */

/* The C620 CAN port runs at 1 Mbps and is not configurable */
#define C620_BITRATE            1000000u

/** Hardware acceptance filter covering every C620 feedback identifier.
 *
 *  Pass these to hvd230_set_filter() before hvd230_begin(), to drop
 *  everything else on the bus in the controller, before it costs an
 *  interrupt. Matching is bitwise, so this takes 0x200 to 0x20F, which is
 *  the smallest power-of-two range holding 0x201 to 0x208.
 */
#define C620_FILTER_ID          0x200u
#define C620_FILTER_MASK        0x7F0u

/* Protocol limitation: +-16384 counts maps to +-20 A of torque current */
#define C620_MAX_CURRENT_RAW    16384.0f
#define C620_FULL_SCALE_AMPS    20.0f
#define C620_RAW_PER_AMP        (C620_MAX_CURRENT_RAW / C620_FULL_SCALE_AMPS)

/* Protocol limitation: rotor mechanical angle is one revolution, 0 to 8191 */
#define M3508_COUNTS_PER_REV    8192

/** Nominal P19 reduction, "approximately 19:1" in the M3508 user guide.
 *  The real gearbox is 3591:187. It is a per-motor knob because a worn or
 *  swapped gearbox will not match the datasheet.
 */
#define M3508_GEAR_RATIO        (3591.0f / 187.0f)

/* Datasheet no-load output speed at rated voltage, used as the soft limit */
#define M3508_MAX_OUTPUT_RPM    482.0f

/* Rated current, well under the 20 A protocol ceiling. Default soft limit. */
#define M3508_RATED_AMPS        10.0f

#define RM_DEGREES_PER_REV      360.0f

/* Result of a controller operation */
typedef enum {
    RM_OK,
    RM_EINVAL,    /* bad id, or a value that was negative or not finite */
    RM_ENOTCFG,   /* that speed controller ID was never configured */
    RM_EOFFLINE,  /* feedback stopped; the motor is at zero current */
    RM_EPID,      /* the loop rejected its input, that motor is at zero */
    RM_ECAN,      /* the CAN driver reported a failure */
} rm_sta;

/* Three ways to control the motor, one per motor */
typedef enum {
    RM_MODE_CUR,   /* target is torque current in amps, no loop closed here */
    RM_MODE_RPM,   /* target is output shaft rpm */
    RM_MODE_POS,   /* target is output shaft degrees, multi-turn.
                    * Cascaded: position -> speed -> current. */
} rm_mode;

/* Whole decoded status of one motor */
typedef struct {
    uint16_t angle;      /* rotor mechanical angle, 0 to 8191 */
    int16_t  rpm;        /* rotor rpm, before the gearbox */
    int16_t  t_cur;      /* torque current, raw counts, +-16384 = +-20 A */
    uint8_t  temp;       /* degrees C */
    int32_t  pos;        /* accumulated multi-turn rotor counts */
    float    pos_deg;    /* accumulated output shaft angle, degrees */
    float    rpm_o;      /* output shaft rpm, after the gearbox */
    bool     is_online;  /* is the motor answering on CAN */
    int16_t  cmd;        /* raw current last sent to this motor */
} rm_feedback_t;

/* Private. Declared here only so the caller can place an rm_t; never read or
 * write a field directly.
 */
typedef struct {
    bool     configured;
    rm_mode  mode;
    float    gear_ratio;
    float    target;
    float    target_max;
    float    current_max_raw;
    float    timeout_s;
    float    since_rx_s;
    bool     is_online;

    /* Decoded feedback */
    uint16_t angle;
    int16_t  rpm;
    int16_t  t_cur;
    uint8_t  temp;

    /* Multi-turn tracking */
    int32_t  pos;
    uint16_t prev_angle;
    bool     angle_init;

    int16_t  cmd;
    struct upid pid;    /* the mode's loop: speed, or position's outer */
    struct upid vel;    /* position mode's inner speed loop */
} rm_motor_t;

typedef struct {
    hvd230_t *can;
    rm_motor_t motor[RM_MAX_MOTORS];
} rm_t;

/** Bind an already-started CAN driver and put every motor slot back to its
 *  declared defaults. Required before anything else.
 *
 *  A zeroed rm_t is not a valid one: upid has no default member values, so a
 *  controller inside would be indeterminate.
 *
 *  can must outlive the rm_t, and hvd230_begin() must already have succeeded
 *  on it.
 */
rm_sta rm_init(rm_t *rm, hvd230_t *can);

/* Config */
rm_sta rm_cfg(rm_t *rm, uint8_t id, rm_mode mode);

/** Read, edit, write. rm_cfg() already installed sane defaults, so start
 *  from those rather than from a bare upid_cfg:
 *
 *      struct upid_cfg pid;
 *      rm_get_pid(&rm, 1, &pid);
 *      pid.kp = 0.6f;
 *      rm_set_pid(&rm, 1, &pid);
 *
 *  rm_set_pid tunes the loop the mode's target speaks to -- the speed loop
 *  in RM_MODE_RPM, or the outer degrees-to-output-RPM loop in RM_MODE_POS.
 *  rm_get_vel_pid() and rm_set_vel_pid() access RM_MODE_POS's inner
 *  rotor-RPM-to-current loop. Output limits and direction are the motor's own
 *  and get overwritten.
 */

/** Gain units.
 *
 *  RM_MODE_RPM -- error is rotor rpm and the loop outputs raw C620 current
 *  counts, clamped to your current limit. 8192 counts = 10 A, the default
 *  ceiling.
 *
 *      kp   raw current counts per rotor rpm
 *      ki   raw current counts per rotor rpm-second
 *      kd   raw current count-seconds per rotor rpm
 *
 *  RM_MODE_POS -- rm_get_pid() and rm_set_pid() access the outer position
 *  loop. Its error is output-shaft degrees and its output is output RPM:
 *
 *      kp   output RPM per output-shaft degree
 *      ki   output RPM per output-shaft degree-second
 *      kd   output RPM-seconds per output-shaft degree
 *
 *  rm_set_limits(rm, id, target_max, current_max_amps) supplies this outer
 *  loop's output-RPM speed cap. The inner speed loop, accessed with
 *  rm_get_vel_pid() and rm_set_vel_pid(), has rotor-rpm error and raw-current
 *  output, so its gains use raw current counts per rotor RPM (and the
 *  corresponding integral and derivative units).
 *
 *  Never:
 *
 *      negative or non-finite      rejected as RM_EINVAL
 *      o_min / o_max               overwritten; use rm_set_limits()
 *      dir other than UPID_DIRECT  overwritten; sign of current carries it
 *
 *  Start low. A gain that saturates the output makes the loop on/off, and
 *  it will oscillate at whatever current you allowed.
 *
 *  Start with a low current limit. The defaults are a starting point, not a
 *  tune for every load.
 */

rm_sta rm_set_pid(rm_t *rm, uint8_t id, const struct upid_cfg *cfg);
rm_sta rm_get_pid(const rm_t *rm, uint8_t id, struct upid_cfg *cfg);

/* The inner speed loop of RM_MODE_POS. RM_EINVAL in any other mode. */
rm_sta rm_set_vel_pid(rm_t *rm, uint8_t id, const struct upid_cfg *cfg);
rm_sta rm_get_vel_pid(const rm_t *rm, uint8_t id, struct upid_cfg *cfg);
rm_sta rm_set_gear_ratio(rm_t *rm, uint8_t id, float ratio);
/** Soft limits.
 *
 *      RM_MODE_RPM   target_max caps the commanded rpm
 *      RM_MODE_POS   target_max is the SPEED CAP for the move, output rpm
 *      RM_MODE_CUR   target_max has no effect
 *
 *  current_max_amps always caps torque, clamped to the C620's 20 A.
 */
rm_sta rm_set_limits(rm_t *rm, uint8_t id, float target_max,
                     float current_max_amps);
rm_sta rm_set_timeout(rm_t *rm, uint8_t id, float timeout_s);

/* User input */
rm_sta rm_set_target(rm_t *rm, uint8_t id, float target);

/* CAN message handling */
rm_sta rm_handle_frame(rm_t *rm, const can_frame_t *frame);
rm_sta rm_poll(rm_t *rm, uint32_t timeout_ms);
rm_sta rm_update(rm_t *rm, float dt_s);
rm_sta rm_stop(rm_t *rm);

/* Checking information */
rm_sta rm_get_fb(const rm_t *rm, uint8_t id, rm_feedback_t *out);
rm_mode rm_get_mode(const rm_t *rm, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* RM_RM_H */
