#pragma once

#include <cstddef>
#include <cstdint>

#include "arm_kinematics.h"
#include "motor_interface.h"

// Application-level bridge between manufacturer-compatible JSON commands
// and the existing MotorInterface. Does not own UART or create a FreeRTOS task.
//
// Threading contract:
//   HandleJsonCommand(), Process(), and BuildFeedbackJson() must be called
//   sequentially from one application task (single command-snapshot writer).
class ArmCommandInterface final {
public:
    explicit ArmCommandInterface(MotorInterface& motor) noexcept;

    enum class CommandResult : uint8_t {
        ACCEPTED,
        INVALID_JSON,
        UNSUPPORTED_COMMAND,
        INVALID_TARGET,
        MOTOR_UNAVAILABLE
    };

    // First-stage commands:
    //   T=102  joint-space target in radians
    //   T=1041 Cartesian target (x/y/z mm, t/r/g rad)
    //   T=0    emergency stop request
    //
    // This only accepts/validates a target; it does not directly access UART
    // or implicitly enable servo torque.
    CommandResult HandleJsonCommand(const char* json);

    // Called periodically by the application's single owning task.
    // Re-publishes the accepted target to satisfy MotorInterface's
    // 100 ms command freshness requirement.
    //
    // A separate communications-loss / command-expiration safety policy
    // is required before enabling physical motor output.
    void Process();

    // Read JointAngles via MotorInterface and calculate FK on demand.
    // Writes a NUL-terminated JSON reply if capacity is sufficient.
    // Returns false when feedback is unavailable or buffer too small.
    bool BuildFeedbackJson(char* output, std::size_t capacity);

    // Clears retained targets and requests disarm; does not promise
    // immediate physical torque-off (MotorInterface FSM owns that).
    void RequestStop();

private:
    MotorInterface& motor_;

    JointAngles target_joints_{};
    bool has_target_ = false;
    int64_t last_publish_us_ = 0;

    static constexpr int64_t kPublishPeriodUs = 20000; // 50 Hz
};
