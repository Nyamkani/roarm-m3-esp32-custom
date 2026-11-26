#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Compile-time-only HOMING joint limits (raw ST3215 encoder counts).
// These are NOT calibrated mechanical limits until confirmed on the robot.
// Development mode bypasses only unconfigured joint limits: raw-value,
// freshness, coupling and tracking error checks remain in force.
#ifndef MOTOR_HOME_DEVELOPMENT_MODE
#define MOTOR_HOME_DEVELOPMENT_MODE 1
#endif

// Actual Current Position Hold output is NOT implemented/validated.
// Keep an explicit compile-time safety gate for future work.
#ifndef MOTOR_FAULT_HOLD_OUTPUT_ENABLE
#define MOTOR_FAULT_HOLD_OUTPUT_ENABLE 0
#endif

#if MOTOR_FAULT_HOLD_OUTPUT_ENABLE
#error "Fault Current Position Hold output has not been implemented or physically validated"
#endif

namespace home_safety {

struct RawJointLimit {
    int16_t minimum_raw = 0;
    int16_t maximum_raw = 4095;
    bool configured = false;
    bool mechanically_verified = false;
};

// Ordered servo IDs: 11,12,13,14,15,16,17.
// Future verified per-servo mechanical limits can be added here.
// Do not populate from a single stationary encoder reading.
constexpr std::array<RawJointLimit, 7> kJointLimits{{
    {}, {}, {}, {}, {}, {}, {}
}};

inline bool CheckRawJointLimit(std::size_t index, int32_t raw) {
    if (index >= kJointLimits.size() || raw < 0 || raw > 4095)
        return false;
    const auto& limit = kJointLimits[index];
    if (!limit.configured || !limit.mechanically_verified)
        return MOTOR_HOME_DEVELOPMENT_MODE != 0;
    if (limit.minimum_raw < 0 || limit.maximum_raw > 4095 ||
        limit.minimum_raw >= limit.maximum_raw)
        return false;
    return raw >= limit.minimum_raw && raw <= limit.maximum_raw;
}

inline bool AllLimitsVerified() {
    for (const auto& limit : kJointLimits)
        if (!limit.configured || !limit.mechanically_verified ||
            limit.minimum_raw < 0 || limit.maximum_raw > 4095 ||
            limit.minimum_raw >= limit.maximum_raw)
            return false;
    return true;
}

} // namespace home_safety
