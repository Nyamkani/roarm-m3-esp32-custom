#include "arm_command_interface.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "cJSON.h"
#include "esp_timer.h"

namespace {

constexpr std::size_t kMaxJsonBytes = 512;

// Require explicit numeric fields. cJSON allows booleans, strings and nulls;
// these are rejected rather than silently interpreted as zero.
bool ReadFinite(const cJSON* root, const char* key, double& result) {
    const cJSON* field = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(field) || !std::isfinite(field->valuedouble)) {
        return false;
    }
    result = field->valuedouble;
    return true;
}

bool ReadFloat(const cJSON* root, const char* key, float& result) {
    double number = 0.0;
    if (!ReadFinite(root, key, number) ||
        std::abs(number) > std::numeric_limits<float>::max()) {
        return false;
    }
    result = static_cast<float>(number);
    return std::isfinite(result);
}

// The v0.16 execution layer has no trajectory speed/acceleration control.
// Do not silently accept an optional nonzero vendor speed/accel command.
bool UnsupportedMotionOptions(const cJSON* root) {
    for (const char* key : {"spd", "acc"}) {
        const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
        if (item != nullptr &&
            (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) ||
             item->valuedouble != 0.0)) {
            return true;
        }
    }
    return false;
}

bool ValidJointTarget(const JointAngles& candidate) {
    ServoPositions positions{};
    return JointMapper::ToServoPositions(candidate, positions);
}

}  // namespace

ArmCommandInterface::ArmCommandInterface(MotorInterface& motor) noexcept
    : motor_(motor) {}

ArmCommandInterface::CommandResult
ArmCommandInterface::HandleJsonCommand(const char* json) {
    if (json == nullptr) return CommandResult::INVALID_JSON;
    const std::size_t length = std::strlen(json);
    if (length == 0 || length > kMaxJsonBytes) {
        return CommandResult::INVALID_JSON;
    }

    const char* parse_end = nullptr;
    cJSON* root = cJSON_ParseWithOpts(json, &parse_end, 1);
    if (root == nullptr) return CommandResult::INVALID_JSON;

    CommandResult result = CommandResult::INVALID_JSON;

    do {
        if (!cJSON_IsObject(root)) break;

        double t = 0.0;
        if (!ReadFinite(root, "T", t) || std::floor(t) != t) break;

        if (t == 0.0) {
            RequestStop();
            result = CommandResult::ACCEPTED;
            break;
        }

// T=210: Servo Torque / ARM-DISARM control
if (t == 210.0)
{
    double cmd = 0.0;

    if (!ReadFinite(root, "cmd", cmd) ||
        (cmd != 0.0 && cmd != 1.0))
    {
        result = CommandResult::INVALID_TARGET;
        break;
    }

    // DISARM: always allow a stop request.
    if (cmd == 0.0)
    {
        RequestStop();
        result = CommandResult::ACCEPTED;
        break;
    }

    // ARM: disabled until physical safety validation.
    #if MOTOR_TORQUE_WRITE_ENABLE == 0
        result = CommandResult::MOTOR_UNAVAILABLE;
        break;
    #else
        if (motor_.GetState() != MotorInterfaceState::DISARMED ||
            !has_target_)
        {
            result = CommandResult::MOTOR_UNAVAILABLE;
            break;
        }

        const int64_t now_us = esp_timer_get_time();

        // Reject missing or expired upper-level command.
        if (last_command_us_ <= 0 ||
            now_us - last_command_us_ >= kCommandTimeoutUs)
        {
            RequestStop();
            result = CommandResult::MOTOR_UNAVAILABLE;
            break;
        }

        // Ensure the current command is available before ARM request.
        if (!motor_.SetJointTargets(target_joints_))
        {
            result = CommandResult::INVALID_TARGET;
            break;
        }

        last_publish_us_ = now_us;

        motor_.RequestServoOn();

        // Request accepted, not confirmation of physical ARM.
        result = CommandResult::ACCEPTED;
        break;
    #endif
    }



        if (t != 102.0 && t != 1041.0) {
            result = CommandResult::UNSUPPORTED_COMMAND;
            break;
        }

        if (UnsupportedMotionOptions(root)) {
            result = CommandResult::UNSUPPORTED_COMMAND;
            break;
        }

        const MotorInterfaceState state = motor_.GetState();
        if (state == MotorInterfaceState::INIT ||
            state == MotorInterfaceState::ERROR) {
            result = CommandResult::MOTOR_UNAVAILABLE;
            break;
        }

        JointAngles candidate{};

        if (t == 102.0) {
            // Vendor: T=102 {"base","shoulder","elbow","wrist","roll","hand"}.
            if (!ReadFloat(root, "base", candidate.base) ||
                !ReadFloat(root, "shoulder", candidate.shoulder) ||
                !ReadFloat(root, "elbow", candidate.elbow) ||
                !ReadFloat(root, "wrist", candidate.wrist) ||
                !ReadFloat(root, "roll", candidate.roll) ||
                !ReadFloat(root, "hand", candidate.gripper) ||
                !ValidJointTarget(candidate)) {
                result = CommandResult::INVALID_TARGET;
                break;
            }
        } else {
            ArmPose target{};
            if (!ReadFinite(root, "x", target.x) ||
                !ReadFinite(root, "y", target.y) ||
                !ReadFinite(root, "z", target.z) ||
                !ReadFinite(root, "t", target.t) ||
                !ReadFinite(root, "r", target.r) ||
                !ReadFinite(root, "g", target.g) ||
                !ArmKinematics::Inverse(target, candidate)) {
                result = CommandResult::INVALID_TARGET;
                break;
            }
        }

        // Commit only after the complete command passes validation.
        target_joints_ = candidate;
        has_target_ = true;

        last_command_us_ = esp_timer_get_time();
        last_publish_us_ = 0;// Publish at next Process().
        result = CommandResult::ACCEPTED;
    } while (false);

    cJSON_Delete(root);
    return result;
}

void ArmCommandInterface::Process()
{
    if (!has_target_)
        return;

    const int64_t now_us = esp_timer_get_time();

    // Upper command freshness check
    if (now_us - last_command_us_ >= kCommandTimeoutUs)
    {
        RequestStop();
        return;
    }

    const MotorInterfaceState state = motor_.GetState();

    if (state != MotorInterfaceState::DISARMED &&
        state != MotorInterfaceState::ARMED)
    {
        return;
    }

    if (last_publish_us_ != 0 &&
        now_us - last_publish_us_ < kPublishPeriodUs)
    {
        return;
    }

    if (!motor_.SetJointTargets(target_joints_))
    {
        RequestStop();
        return;
    }

    last_publish_us_ = now_us;
}

bool ArmCommandInterface::BuildFeedbackJson(char* output,
                                             std::size_t capacity) {
    if (output == nullptr || capacity == 0) return false;
    output[0] = '\0';

    JointAngles joints{};
    if (!motor_.GetJointFeedback(joints)) return false;

    ArmPose pose{};
    if (!ArmKinematics::Forward(joints, pose)) return false;

    // T=105 is used by the manufacturer to request position feedback.
    // This reply is a minimal custom subset, not a wire-compatible
    // reproduction of the vendor's full feedback payload.
    const int len = std::snprintf(
        output, capacity,
        "{\"T\":105,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,"
        "\"t\":%.6f,\"r\":%.6f,\"g\":%.6f}",
        pose.x, pose.y, pose.z, pose.t, pose.r, pose.g
    );

    if (len < 0 || static_cast<std::size_t>(len) >= capacity) {
        output[0] = '\0';
        return false;
    }
    return true;
}

void ArmCommandInterface::RequestStop()
{
    has_target_ = false;
    last_publish_us_ = 0;
    last_command_us_ = 0;

    motor_.RequestServoOff();
}