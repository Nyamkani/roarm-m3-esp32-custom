#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// RoArm-M3 joint coordinates in radians (manufacturer conventions).
// Servo array order: IDs 11, 12, 13, 14, 15, 16, 17.
struct JointAngles {
    float base = 0.0f;
    float shoulder = 0.0f;
    float elbow = 0.0f;
    float wrist = 0.0f;
    float roll = 0.0f;
    float gripper = 0.0f;
};

struct ServoPositions {
    std::array<int16_t, 7> counts{};
};

class JointMapper final {
public:
    static constexpr std::size_t kServoCount = 7;
    static constexpr int kResolution = 4096;
    static constexpr int kCenter = 2047;

    // Rejects non-finite/out-of-range joint requests (no hidden clipping).
    static bool ToServoPositions(const JointAngles& joints, ServoPositions& positions);

    // Expects fresh, independently validated servo feedback from all 7 motors.
    // Checks coupled shoulder servo positions against the manufacturer relationship.
    // The tolerance is in encoder counts, and must be tuned from real hardware.
    static bool ToJointAngles(const ServoPositions& positions,
                              JointAngles& joints,
                              int shoulder_tolerance_counts = 32);
};
