#include "joint_mapper.h"

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kCountsPerRad = 4096.0 / (2.0 * kPi);
constexpr double kRadPerCount = (2.0 * kPi) / 4096.0;

bool InRange(float value, double lo, double hi) {
    return std::isfinite(value) && value >= lo && value <= hi;
}

bool ValidCount(int32_t position) {
    return position >= 0 && position < JointMapper::kResolution;
}

int32_t Delta(float angle) {
    return static_cast<int32_t>(std::lround(static_cast<double>(angle) * kCountsPerRad));
}
}

bool JointMapper::ToServoPositions(const JointAngles& q, ServoPositions& out) {
    if (!InRange(q.base, -kPi, kPi) ||
        !InRange(q.shoulder, -kPi / 2.0, kPi / 2.0) ||
        !InRange(q.elbow, 0.0, kPi) ||
        !InRange(q.wrist, -kPi / 2.0, kPi / 2.0) ||
        !InRange(q.roll, -kPi, kPi) ||
        !InRange(q.gripper, 700.0 * kRadPerCount, 3396.0 * kRadPerCount))
        return false;

    const int32_t base = kCenter - Delta(q.base);
    const int32_t shoulder = Delta(q.shoulder);
    const int32_t elbow = 1024 + Delta(q.elbow);
    const int32_t wrist = kCenter + Delta(q.wrist);
    const int32_t roll = kCenter - Delta(q.roll);
    const int32_t gripper = Delta(q.gripper);

    const std::array<int32_t, kServoCount> raw = {
        base, kCenter + shoulder, kCenter - shoulder, elbow,
        wrist, roll, gripper
    };
    for (const int32_t p : raw) {
        if (!ValidCount(p)) return false;
    }
    ServoPositions result{};
    for (std::size_t i = 0; i < kServoCount; ++i)
        result.counts[i] = static_cast<int16_t>(raw[i]);
    out = result;
    return true;
}

bool JointMapper::ToJointAngles(const ServoPositions& in,
                                JointAngles& out,
                                int shoulder_tolerance_counts) {
    if (shoulder_tolerance_counts < 0) return false;
    for (int16_t c : in.counts) {
        if (!ValidCount(c)) return false;
    }
    const int32_t driving = in.counts[1];
    const int32_t driven = in.counts[2];
    if (std::abs(driving + driven - 2 * kCenter) > shoulder_tolerance_counts)
        return false;

    JointAngles q{};
    q.base     = static_cast<float>((kCenter - in.counts[0]) * kRadPerCount);
    q.shoulder = static_cast<float>((driving - kCenter) * kRadPerCount);
    q.elbow    = static_cast<float>((in.counts[3] - 1024) * kRadPerCount);
    q.wrist    = static_cast<float>((in.counts[4] - kCenter) * kRadPerCount);
    q.roll     = static_cast<float>((kCenter - in.counts[5]) * kRadPerCount);
    q.gripper  = static_cast<float>(in.counts[6] * kRadPerCount);

    if (!InRange(q.shoulder, -kPi/2.0 - kRadPerCount, kPi/2.0 + kRadPerCount) ||
        in.counts[3] < 1024 || in.counts[3] > 3071 ||
        in.counts[6] < 700 || in.counts[6] > 3396)
        return false;
    out = q;
    return true;
}
