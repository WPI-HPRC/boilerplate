#include "controller.h"

#include <cmath>

// The gain schedule (kGainTable, kGainRows) is data in its own header. The
// simulation and the host test build against other tables by defining
// GNC_GAIN_TABLE_HEADER; flight builds use the default.
#ifndef GNC_GAIN_TABLE_HEADER
#define GNC_GAIN_TABLE_HEADER "lqr_gain_table.h"
#endif
#include GNC_GAIN_TABLE_HEADER

namespace gnc {

namespace {

const float kServoLag = 0.846482f;  // exp(-5 ms / 30 ms), A4

/**
 * Limits a value to the range [-limit, +limit].
 *
 * @param value  [in] The value to limit.
 * @param limit  [in] The largest allowed magnitude (positive).
 * @return value if it is inside the range, otherwise +limit or -limit.
 */
float clamp(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
}

/**
 * Looks up the LQR gains for an airspeed by linear interpolation between the
 * two nearest rows of kGainTable. Outside the table the end row is used.
 *
 * @param airspeed  [in] Airspeed [m/s].
 * @param k         [out] The six gains: [0] roll angle, [1] roll rate,
 *                  [2] roll servo, [3] pitch/yaw angle, [4] pitch/yaw rate,
 *                  [5] pitch/yaw servo.
 */
void lookupGains(float airspeed, float k[6]) {
    int hi = 1;
    while (hi < kGainRows - 1 && airspeed > kGainTable[hi][0]) {
        ++hi;
    }
    const float* a = kGainTable[hi - 1];
    const float* b = kGainTable[hi];
    float t = (airspeed - a[0]) / (b[0] - a[0]);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    for (int i = 0; i < 6; ++i) {
        k[i] = a[i + 1] + t * (b[i + 1] - a[i + 1]);
    }
}

/**
 * Solves the four-bar linkage: given the angle of one horn, finds the angle of
 * the other (MATH_DERIVATIONS B6). The same formula works in both directions
 * with the two horn lengths swapped.
 *
 * Horn angles are measured from the perpendicular to the line between the
 * shafts, positive when the horn tip swings away from the other shaft.
 *
 * @param theta_in  [in] Angle of the known horn [rad].
 * @param horn_in   [in] Length of the known horn.
 * @param horn_out  [in] Length of the horn whose angle is wanted.
 * @param b         [in] Push rod length.
 * @param d         [in] Distance between the two shafts.
 * @return Angle of the other horn [rad], or NaN if the links cannot close or
 *         theta_in is 90 deg or more from perpendicular (formula not valid).
 */
float hornAngle(float theta_in, float horn_in, float horn_out, float b, float d) {
    if (!(std::fabs(theta_in) < 1.5707963f)) {
        return NAN;
    }
    const float l_sq = horn_in * horn_in + d * d + 2.0f * horn_in * d * std::sin(theta_in);
    const float l = std::sqrt(l_sq);
    return std::acos((l_sq + d * d - horn_in * horn_in) / (2.0f * l * d)) +
           std::acos((l_sq + horn_out * horn_out - b * b) / (2.0f * l * horn_out)) -
           1.5707963f;
}

/**
 * Servo horn angle for a given canard horn angle on linkage g.
 *
 * @param g                  [in] Linkage dimensions.
 * @param canard_horn_angle  [in] Canard horn angle [rad].
 * @return Servo horn angle [rad], or NaN if the links cannot close.
 */
float servoHornAngle(const Linkage& g, float canard_horn_angle) {
    return hornAngle(canard_horn_angle, g.canard_horn, g.servo_horn, g.push_rod,
                     g.shaft_spacing);
}

}  // namespace

// The four public functions below are documented in controller.h.

void lqrController(const float e[3], const float w[3], const float sigma[4],
                   float airspeed, float command[4]) {
    float k[6];
    lookupGains(airspeed, k);

    // Canard angles in roll / pitch / yaw coordinates, then the three
    // single-axis laws v = -(k_e e + k_w w + k_s sigma) (A7).
    const float roll_error = clamp(e[2], kRollErrorLimit);
    const float v_roll = -(k[0] * roll_error + k[1] * w[2] +
                           k[2] * 0.25f * (sigma[0] + sigma[1] + sigma[2] + sigma[3]));
    const float v_pitch = -(k[3] * e[0] + k[4] * w[0] + k[5] * 0.5f * (sigma[2] - sigma[0]));
    const float v_yaw = -(k[3] * e[1] + k[4] * w[1] + k[5] * 0.5f * (sigma[3] - sigma[1]));

    command[0] = v_roll - v_pitch;
    command[1] = v_roll - v_yaw;
    command[2] = v_roll + v_pitch;
    command[3] = v_roll + v_yaw;

    // Failsafe: one common scale factor keeps pitch : yaw : roll proportions.
    float largest = 0.0f;
    bool finite = true;
    for (int i = 0; i < 4; ++i) {
        finite = finite && std::isfinite(command[i]);
        largest = std::fmax(largest, std::fabs(command[i]));
    }
    const float scale = largest > kLqrCanardLimit ? kLqrCanardLimit / largest : 1.0f;
    for (int i = 0; i < 4; ++i) {
        command[i] = finite ? clamp(command[i] * scale, kLqrCanardLimit) : 0.0f;
    }
}

void servoModelStep(float sigma[4], const float command[4]) {
    for (int i = 0; i < 4; ++i) {
        sigma[i] = kServoLag * sigma[i] + (1.0f - kServoLag) * command[i];
    }
}

float canardToServo(const Linkage& g, float canard) {
    canard = clamp(canard, kServoCanardLimit);
    const float servo =
        servoHornAngle(g, g.canard_horn_neutral + g.sign * canard) -
        servoHornAngle(g, g.canard_horn_neutral);
    return std::isfinite(servo) ? servo : 0.0f;
}

float servoToCanard(const Linkage& g, float servo) {
    const float servo_horn_angle = servoHornAngle(g, g.canard_horn_neutral) + servo;
    const float canard_horn_angle = hornAngle(servo_horn_angle, g.servo_horn,
                                              g.canard_horn, g.push_rod, g.shaft_spacing);
    const float canard = g.sign * (canard_horn_angle - g.canard_horn_neutral);
    return std::isfinite(canard) ? clamp(canard, kServoCanardLimit) : 0.0f;
}

}  // namespace gnc
