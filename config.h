#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// COMPETITION PARAMETERS — Edit these and reflash at competition
// ============================================================
#define TARGET_DISTANCE_M   8.50f   // meters (7.00–10.00)
#define TARGET_TIME_S       15.0f   // seconds (10.0–20.0)
#define CAN_GAP_CM          15.0f   // cm, gap between cans (15–20 recommended)
#define BONUS_ENABLED       false   // true = sine curve, false = straight

// ============================================================
// PHYSICAL CONSTANTS — Measure with calipers
// ============================================================
#define WHEEL_DIAMETER_M    0.0520f // meters (52mm, measured)
#define TRACK_WIDTH_M       0.0400f // meters (40mm, measured)

// ============================================================
// MOTOR / ENCODER MAPPING — From calibration tests 1–2
// ============================================================
// Which motorWrite number is left/right wheel
#define MOTOR_LEFT          2       // 1 or 2
#define MOTOR_RIGHT         1       // 1 or 2
// Flip motor direction: 1.0 = default, -1.0 = reversed
#define MOTOR_LEFT_SIGN    -1.0f
#define MOTOR_RIGHT_SIGN   -1.0f

// Which encoder is left/right wheel
#define ENC_LEFT            2       // 1 or 2
#define ENC_RIGHT           1       // 1 or 2
// Flip encoder direction: 1.0 = increasing angle = forward, -1.0 = reversed
#define ENC_LEFT_SIGN      -1.0f
#define ENC_RIGHT_SIGN      1.0f

// ============================================================
// FEEDFORWARD — From calibration tests 3–4
// ============================================================
#define DEADBAND_LEFT       0.16f   // minimum duty that produces motion
#define DEADBAND_RIGHT      0.22f
#define KFF_LEFT            0.58f   // duty per (m/s) above deadband
#define KFF_RIGHT           0.73f

// ============================================================
// BRAKING — From calibration test 5
// ============================================================
#define BRAKE_DUTY          0.30f   // reverse duty cycle magnitude
#define BRAKE_DURATION_MS   80      // how long to apply reverse pulse

// ============================================================
// PID GAINS — Tune during practice
// ============================================================
// Wheel velocity PID (same gains for both wheels)
#define VEL_KP              2.0f
#define VEL_KI              0.5f
#define VEL_KD              0.01f
#define VEL_INTEGRAL_MAX    0.5f    // anti-windup clamp

// Heading correction PID
#define HEADING_KP          3.0f
#define HEADING_KI          0.1f
#define HEADING_KD          0.05f
#define HEADING_INTEGRAL_MAX 0.3f

// Cross-track error gain (feeds lateral offset into heading correction)
#define CROSS_TRACK_GAIN    2.0f

// ============================================================
// MOTION PROFILE
// ============================================================
#define MAX_ACCEL           0.40f   // m/s² (acceleration and deceleration)
#define MAX_VELOCITY        1.00f   // m/s safety cap

// ============================================================
// MOTOR LIMITS
// ============================================================
#define MAX_MOTOR_VOLTAGE   6.0f    // voltage cap to motors

// ============================================================
// TRIGGER
// ============================================================
#define TRIGGER_THRESHOLD   0.02f   // radians of encoder movement to detect pencil push

// ============================================================
// DEBUG
// ============================================================
#define DEBUG_SERIAL        true    // true = print debug, false for competition
#define DEBUG_INTERVAL_MS   50      // how often to print debug (ms)

#endif
