#include "arm_kinematics.h"

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kL2A = 236.82;
constexpr double kL2B = 30.00;
constexpr double kL3A = 144.49;
constexpr double kL3B = 0.0;
constexpr double kL4A = 171.67;
constexpr double kL4B = 13.69;
constexpr double kEoatA = 0.0;
constexpr double kEoatB = 0.0;
const double kL2 = std::hypot(kL2A, kL2B);
const double kL3 = std::hypot(kL3A, kL3B);
const double kLE = std::hypot(kL4A + kEoatA, kL4B + kEoatB);
// atan2 cannot be constexpr under C++17. Compute the fixed geometry at runtime.

bool IsFinite(const ArmPose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z) && std::isfinite(p.t) &&
           std::isfinite(p.r) && std::isfinite(p.g);
}

bool IsFinite(const JointAngles& q) {
    return std::isfinite(q.base) && std::isfinite(q.shoulder) &&
           std::isfinite(q.elbow) && std::isfinite(q.wrist) &&
           std::isfinite(q.roll) && std::isfinite(q.gripper);
}

// No silent reachability clamping; permit only floating-point roundoff near [-1, 1].
bool SafeAcos(double value, double& angle) {
    if (!std::isfinite(value) || value < -1.0 - 1e-12 || value > 1.0 + 1e-12)
        return false;
    if (value < -1.0) value = -1.0;
    if (value > 1.0) value = 1.0;
    angle = std::acos(value);
    return true;
}
}

bool ArmKinematics::Inverse(const ArmPose& target, JointAngles& out) {
    if (!IsFinite(target)) return false;

    const double t2 = std::atan2(kL2B, kL2A);
    const double t3 = std::atan2(kL3B, kL3A);
    const double tE = std::atan2(kL4B + kEoatB, kL4A + kEoatA);

    // Remove the terminal tool offset from the desired grasp position.
    // Matches rotatePoint(t - pi) in the manufacturer's IK code.
    const double tool_r = kLE * std::cos(target.t + tE);
    const double tool_z = kLE * std::sin(target.t + tE);
    const double radial_target = std::hypot(target.x, target.y);
    const double radial_wrist = radial_target - tool_r;
    const double vertical_wrist = target.z + tool_z;

    // The manufacturer uses the outward-facing IK branch. Do not reinterpret
    // a target behind the base as an equivalent solution with a flipped base.
    if (radial_wrist <= 1e-9) return false;
    const double distance = std::hypot(radial_wrist, vertical_wrist);
    if (distance <= 1e-9 || !std::isfinite(distance)) return false;

    double inner_1 = 0.0;
    double inner_2 = 0.0;
    const double d2 = distance * distance;
    if (!SafeAcos((kL2 * kL2 + d2 - kL3 * kL3) / (2.0 * kL2 * distance), inner_1) ||
        !SafeAcos((d2 + kL3 * kL3 - kL2 * kL2) / (2.0 * distance * kL3), inner_2))
        return false;

    const double psi = inner_1 + t2;
    const double alpha = kPi / 2.0 - std::atan2(vertical_wrist, radial_wrist) - psi;
    const double beta = psi + inner_2 - t3;
    const double wrist = target.t + (kPi / 2.0 - alpha - beta);

    JointAngles candidate{};
    candidate.base = static_cast<float>(std::atan2(target.y, target.x));
    candidate.shoulder = static_cast<float>(alpha);
    candidate.elbow = static_cast<float>(beta);
    candidate.wrist = static_cast<float>(wrist);
    candidate.roll = static_cast<float>(target.r);
    candidate.gripper = static_cast<float>(target.g);

    // Enforce the actual actuator/encoder representability policy from v0.15.
    ServoPositions positions{};
    if (!IsFinite(candidate) || !JointMapper::ToServoPositions(candidate, positions))
        return false;

    out = candidate;
    return true;
}

bool ArmKinematics::Forward(const JointAngles& joints, ArmPose& out) {
    if (!IsFinite(joints)) return false;
    ServoPositions positions{};
    if (!JointMapper::ToServoPositions(joints, positions)) return false;

    const double t2 = std::atan2(kL2B, kL2A);
    const double t3 = std::atan2(kL3B, kL3A);
    const double tE = std::atan2(kL4B + kEoatB, kL4A + kEoatA);
    const double s = joints.shoulder;
    const double e = joints.elbow;
    const double w = joints.wrist;
    const double a = kPi / 2.0 - (s + t2);
    const double b = kPi / 2.0 - (e + s + t3);
    const double c = kPi / 2.0 - (e + s + w + tE);

    const double radial = kL2 * std::cos(a) + kL3 * std::cos(b) + kLE * std::cos(c);
    const double height = kL2 * std::sin(a) + kL3 * std::sin(b) + kLE * std::sin(c);

    ArmPose candidate{};
    candidate.x = radial * std::cos(joints.base);
    candidate.y = radial * std::sin(joints.base);
    candidate.z = height;
    candidate.t = s + e + w - kPi / 2.0;
    candidate.r = joints.roll;
    candidate.g = joints.gripper;
    if (!IsFinite(candidate)) return false;
    out = candidate;
    return true;
}
