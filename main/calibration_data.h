#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Calibration DATA MODEL ONLY: no motor commands, NVS, EEPROM or
// automatic offset application. A structurally valid record is NOT
// evidence that its mechanical reference is physically verified.
namespace arm_calibration {

constexpr uint32_t kSchemaVersion = 1;
constexpr std::array<uint8_t, 7> kServoIds{{11, 12, 13, 14, 15, 16, 17}};

struct ServoReference {
    uint8_t servo_id = 0;
    int16_t zero_raw = -1; // raw encoder reading at verified mechanical zero
    int8_t direction = 0;  // +1 or -1, unknown is 0
    int16_t minimum_raw = -1;
    int16_t maximum_raw = -1;
};

struct CalibrationData {
    uint32_t schema_version = kSchemaVersion;
    std::array<ServoReference, 7> servos{};
    int32_t shoulder_pair_sum_raw = -1; // ID12 raw + ID13 raw at verified coupling
    bool mechanical_reference_verified = false; // requires independent reference
};

// A structural test only. It cannot establish mechanical correctness.
inline bool IsStructurallyValid(const CalibrationData& data) {
    if (data.schema_version != kSchemaVersion) return false;
    for (std::size_t i = 0; i < data.servos.size(); ++i) {
        const ServoReference& servo = data.servos[i];
        if (servo.servo_id != kServoIds[i] ||
            (servo.direction != 1 && servo.direction != -1) ||
            servo.minimum_raw < 0 || servo.maximum_raw > 4095 ||
            servo.minimum_raw >= servo.maximum_raw ||
            servo.zero_raw < servo.minimum_raw ||
            servo.zero_raw > servo.maximum_raw)
            return false;
    }
    if (data.shoulder_pair_sum_raw < 0 || data.shoulder_pair_sum_raw > 8190)
        return false;
    // The pairing reference must be consistent with the two stored
    // shoulder zero readings; actual mechanical verification is separate.
    return data.shoulder_pair_sum_raw ==
           static_cast<int32_t>(data.servos[1].zero_raw) +
           static_cast<int32_t>(data.servos[2].zero_raw);
}

}  // namespace arm_calibration
