#include "gpio_config.h"
#include "motor_interface.h"
#include "arm_command_interface.h"
#include "pins.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <unistd.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "app_main";

namespace {

board_handles_t board{};
MotorInterface motor_interface{};
ArmCommandInterface command_interface{motor_interface};

constexpr std::size_t kLineCapacity = 513; // 512-byte JSON + '\0'
constexpr std::size_t kMaxCharsPerCycle = 128;
constexpr TickType_t kCommandTaskPeriod = pdMS_TO_TICKS(20);
constexpr uint32_t kCommandTaskStackWordsOrBytes = 6144;

const char* ResultToString(ArmCommandInterface::CommandResult result) {
    using R = ArmCommandInterface::CommandResult;
    switch (result) {
        case R::ACCEPTED:            return "accepted";
        case R::INVALID_JSON:        return "invalid_json";
        case R::UNSUPPORTED_COMMAND: return "unsupported_command";
        case R::INVALID_TARGET:      return "invalid_target";
        case R::MOTOR_UNAVAILABLE:   return "motor_unavailable";
        default:                     return "unknown";
    }
}

const char* MotorStateToString(MotorInterfaceState state)
{
    switch (state)
    {
        case MotorInterfaceState::INIT:       return "INIT";
        case MotorInterfaceState::READY:      return "READY";
        case MotorInterfaceState::DISARMED:   return "DISARMED";
        case MotorInterfaceState::ARMING:     return "ARMING";
        case MotorInterfaceState::ARMED:      return "ARMED";
        case MotorInterfaceState::PARKING:    return "PARKING";
        case MotorInterfaceState::HOMING:     return "HOMING";
        case MotorInterfaceState::FAULT_HOLD: return "FAULT_HOLD";
        case MotorInterfaceState::DISARMING:  return "DISARMING";
        case MotorInterfaceState::BOOT_RECOVERY: return "BOOT_RECOVERY";
        case MotorInterfaceState::ERROR:      return "ERROR";
        default:                              return "UNKNOWN";
    }
}

const char* OperationToString(MotorOperation operation) {
    switch (operation) {
        case MotorOperation::NONE: return "NONE";
        case MotorOperation::PARK: return "PARK";
        case MotorOperation::HOME: return "HOME";
        default: return "UNKNOWN";
    }
}

const char* HomePhaseToString(uint8_t phase, uint8_t stage) {
    switch (phase) {
        case 0: return "NOT_STARTED";
        case 1: return "PREPARE";
        case 2: return "MOVE_BASE";
        case 3: return "RELEASE_SHOULDER";
        case 4: return "MOVE_SHOULDER";
        case 5: return "SETTLE_SHOULDER";
        case 6: return "SET_SHOULDER_GOAL";
        case 7: return "VERIFY_SHOULDER_GOAL";
        case 8: return "VERIFY_SHOULDER_POSITION";
        case 9: return "RESTORE_SHOULDER";
        case 10:
            switch (stage) {
                case 0: return "MOVE_ELBOW";
                case 1: return "MOVE_WRIST";
                case 2: return "MOVE_ROLL";
                case 3: return "MOVE_GRIPPER";
                default: return "MOVE_REST_UNKNOWN";
            }
        case 11: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

const char* HomeId13VerifiedToString(MotorInterface::HomeId13Verified state) {
    switch (state) {
        case MotorInterface::HomeId13Verified::OFF: return "OFF";
        case MotorInterface::HomeId13Verified::ON: return "ON";
        default: return "UNKNOWN";
    }
}

const char* OperationStatusToString(MotorOperationStatus status) {
    switch (status) {
        case MotorOperationStatus::IDLE: return "IDLE";
        case MotorOperationStatus::RUNNING: return "RUNNING";
        case MotorOperationStatus::COMPLETED: return "COMPLETED";
        case MotorOperationStatus::INTERRUPTED: return "INTERRUPTED";
        case MotorOperationStatus::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

void HandleJsonLine(const char* json) {
    // Feedback request is a receive-side operation. Do not send it through
    // HandleJsonCommand(), which handles motion and stop commands only.
    cJSON* root = cJSON_Parse(json);
    if (root == nullptr) {
        std::puts("{\"ok\":false,\"error\":\"invalid_json\"}");
        return;
    }

    const cJSON* type = cJSON_GetObjectItemCaseSensitive(root, "T");


    // T=211: Custom diagnostic status request
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 211.0)
    {
        cJSON_Delete(root);

        const MotorInterfaceState state =
            motor_interface.GetState();

        const MotorOperationSnapshot operation = motor_interface.GetOperationSnapshot();
        const char* motion_state =
            state == MotorInterfaceState::PARKING ? "PARKING" :
            state == MotorInterfaceState::HOMING ? "HOMING" :
            state == MotorInterfaceState::FAULT_HOLD ? "FAULT_HOLD" :
            state == MotorInterfaceState::DISARMING ? "STOPPING" :
            (state == MotorInterfaceState::ARMED || state == MotorInterfaceState::DISARMED) ? "IDLE" : "UNAVAILABLE";

        const uint16_t home_progress = motor_interface.GetHomeProgressSnapshot();
        const uint8_t home_phase_code = static_cast<uint8_t>(home_progress & 0xffu);
        const uint8_t home_stage = static_cast<uint8_t>((home_progress >> 8) & 0xffu);
        JointAngles feedback{};
        const bool feedback_valid =
            motor_interface.GetJointFeedback(feedback);

        std::printf(
            "{\"T\":211,"
            "\"state\":\"%s\","
            "\"motion_state\":\"%s\","
            "\"operation\":\"%s\","
            "\"operation_status\":\"%s\","
            "\"home_id13_last_verified\":\"%s\","
            "\"home_phase\":\"%s\","
            "\"home_phase_code\":%u,"
            "\"feedback_valid\":%s,"
            "\"torque_write_enabled\":%s,"
            "\"pwm_output_enabled\":%s,"
            "\"position_output_enabled\":%s,"
            "\"physical_servo_enabled\":%s,"
            "\"torque_off_verified\":%s,"
            "\"initialization_complete\":%s,"
            "\"calibration_valid\":%s,"
            "\"move_allowed\":%s,"
            "\"vendor_park_enabled\":%s,"
            "\"control_mode\":\"%s\"}\n",
            MotorStateToString(state),
            motion_state,
            OperationToString(operation.operation),
            OperationStatusToString(operation.status),
            HomeId13VerifiedToString(motor_interface.GetHomeId13LastVerified()),
            HomePhaseToString(home_phase_code, home_stage),
            static_cast<unsigned>(home_phase_code),
            feedback_valid ? "true" : "false",
            MOTOR_TORQUE_WRITE_ENABLE ? "true" : "false",
            MOTOR_PWM_OUTPUT_ENABLE ? "true" : "false",
            MOTOR_POSITION_OUTPUT_ENABLE ? "true" : "false",
            MOTOR_PHYSICAL_SERVO_ENABLE ? "true" : "false",
            motor_interface.IsTorqueOffVerified() ? "true" : "false",
            motor_interface.IsInitializationComplete() ? "true" : "false",
            motor_interface.IsCalibrationValid() ? "true" : "false",
            motor_interface.CanExecuteMove() ? "true" : "false",
            MOTOR_VENDOR_SHOULDER_PARK_ENABLE ? "true" : "false",
            MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION ? "POSITION" : "PWM"
        );

        return;
    }


    // T=219: read-only initialization readiness status for SBC-KCF.
    // Phase 1 has no calibration or initialization execution; fail closed.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 219.0)
    {
        cJSON_Delete(root);
        std::printf(
            "{\"T\":219,\"initialization_status\":\"NOT_INITIALIZED\","
            "\"initialization_complete\":%s,"
            "\"calibration_valid\":%s,"
            "\"move_allowed\":%s,"
            "\"execution_enabled\":false}\n",
            motor_interface.IsInitializationComplete() ? "true" : "false",
            motor_interface.IsCalibrationValid() ? "true" : "false",
            motor_interface.CanExecuteMove() ? "true" : "false");
        return;
    }

    // T=220: read-only calibration metadata; no inferred encoder zeros.
    // A default-constructed record is unverified. No EEPROM/NVS/motion.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 220.0)
    {
        cJSON_Delete(root);
        std::printf(
            "{\"T\":220,\"schema_version\":%lu,"
            "\"calibration_valid\":%s,"
            "\"record_verified\":%s,"
            "\"persisted\":false,"
            "\"zero_raw\":[null,null,null,null,null,null,null],"
            "\"direction\":[null,null,null,null,null,null,null],"
            "\"raw_limits\":[null,null,null,null,null,null,null],"
            "\"shoulder_pair_sum_raw\":null}\n",
            static_cast<unsigned long>(motor_interface.GetCalibrationSchemaVersion()),
            motor_interface.IsCalibrationValid() ? "true" : "false",
            motor_interface.HasVerifiedCalibrationRecord() ? "true" : "false");
        return;
    }

    // T=222: read-only HOME safety policy and latched fault diagnostic.
    // A fault snapshot is historical, not permission to re-enable torque.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 222.0)
    {
        cJSON_Delete(root);
        const uint32_t fault = motor_interface.GetHomeFaultDiagnostic();
        const bool fault_recorded = (fault & 1u) != 0u;
        const bool feedback_was_fresh = (fault & 2u) != 0u;
        const bool id13_last_off = (fault & 4u) != 0u;
        const unsigned fault_phase = static_cast<unsigned>((fault >> 8) & 0xffu);
        const unsigned id13_state = static_cast<unsigned>((fault >> 16) & 0xffu);
        std::printf(
            "{\"T\":222,\"development_mode\":%s,"
            "\"joint_limits_verified\":%s,"
            "\"fault_hold_output_enabled\":%s,"
            "\"fault_hold_bench_enabled\":true,"
            "\"home_execution_enabled\":%s,"
            "\"full_home_execution_enabled\":%s,"
            "\"home_safe_fault_hold_enabled\":%s,"
            "\"fault_recorded\":%s,"
            "\"feedback_fresh_at_fault\":%s,"
            "\"id13_last_verified_off_at_fault\":%s,"
            "\"fault_home_phase_code\":%u,"
            "\"fault_id13_last_verified_code\":%u}\n",
            motor_interface.IsHomeDeveloperMode() ? "true" : "false",
            motor_interface.AreHomeLimitsVerified() ? "true" : "false",
            motor_interface.IsFaultHoldOutputEnabled() ? "true" : "false",
            motor_interface.IsHomeExecutionEnabled() ? "true" : "false",
            motor_interface.IsFullHomeExecutionEnabled() ? "true" : "false",
            motor_interface.IsHomeSafeFaultHoldEnabled() ? "true" : "false",
            fault_recorded ? "true" : "false",
            feedback_was_fresh ? "true" : "false",
            id13_last_off ? "true" : "false",
            fault_phase, id13_state);
        return;
    }

    // T225: supervised FAULT_HOLD bench entry; creates no fault in UART.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 225.0)
    {
        const cJSON* cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
        const bool valid = cJSON_IsNumber(cmd) && cmd->valuedouble == 1.0;
        cJSON_Delete(root);
        const bool accepted = valid && motor_interface.RequestFaultHoldBenchTest();
        std::printf("{\"T\":225,\"ok\":%s,\"status\":\"%s\"}\n",
            accepted ? "true" : "false",
            !valid ? "invalid_command" :
            accepted ? "accepted" : "motor_unavailable");
        return;
    }

    // T226: supervised explicit recovery from BENCH-only FAULT_HOLD.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 226.0)
    {
        const cJSON* cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
        const bool valid = cJSON_IsNumber(cmd) && cmd->valuedouble == 1.0;
        cJSON_Delete(root);
        const bool accepted = valid && motor_interface.RequestFaultHoldBenchResume();
        std::printf("{\"T\":226,\"ok\":%s,\"status\":\"%s\"}\n",
            accepted ? "true" : "false",
            !valid ? "invalid_command" :
            accepted ? "accepted" : "motor_unavailable");
        return;
    }

    // T227: diagnostics for supervised FAULT_HOLD bench lifecycle.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 227.0)
    {
        cJSON_Delete(root);
        const auto result = motor_interface.GetFaultHoldStatus();
        const char* status = "UNKNOWN";
        switch (result)
        {
            case MotorInterface::FaultHoldStatus::IDLE: status = "IDLE"; break;
            case MotorInterface::FaultHoldStatus::REQUESTED: status = "REQUESTED"; break;
            case MotorInterface::FaultHoldStatus::WRITE_SENT: status = "WRITE_SENT"; break;
            case MotorInterface::FaultHoldStatus::STABLE: status = "STABLE"; break;
            case MotorInterface::FaultHoldStatus::FAILED: status = "FAILED"; break;
        }
        MotorCommandFrame target{};
        const bool target_valid = motor_interface.GetFaultHoldTarget(target);
        std::printf(
            "{\"T\":227,\"status\":\"%s\",\"target_valid\":%s,"
            "\"targets\":[%d,%d,%d,%d,%d,%d,%d]}\n",
            status, target_valid ? "true" : "false",
            static_cast<int>(target.motors[0].target_position),
            static_cast<int>(target.motors[1].target_position),
            static_cast<int>(target.motors[2].target_position),
            static_cast<int>(target.motors[3].target_position),
            static_cast<int>(target.motors[4].target_position),
            static_cast<int>(target.motors[5].target_position),
            static_cast<int>(target.motors[6].target_position));
        return;
    }

    // T=223: explicitly re-capture CURRENT raw position as Hold target.
    // Available only in ARMED with fresh stationary feedback and paired
    // shoulder. It does not ARM, HOME, calibrate, or enable ordinary MOVE.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 223.0)
    {
        const cJSON* cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
        const bool valid_cmd = cJSON_IsNumber(cmd) && cmd->valuedouble == 1.0;
        cJSON_Delete(root);
        if (!valid_cmd)
        {
            std::puts("{\"T\":223,\"ok\":false,\"error\":\"invalid_command\"}");
            return;
        }
        const bool accepted = motor_interface.RequestCurrentHoldTest();
        std::printf("{\"T\":223,\"ok\":%s,\"status\":\"%s\"}\n",
            accepted ? "true" : "false",
            accepted ? "accepted" : "motor_unavailable");
        return;
    }

    // T=224: read-only result of the explicit in-place Hold test.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 224.0)
    {
        cJSON_Delete(root);
        const auto status = motor_interface.GetHoldTestStatus();
        const char* label = "UNKNOWN";
        switch (status)
        {
            case MotorInterface::HoldTestStatus::IDLE: label = "IDLE"; break;
            case MotorInterface::HoldTestStatus::PENDING: label = "PENDING"; break;
            case MotorInterface::HoldTestStatus::CAPTURED: label = "CAPTURED"; break;
            case MotorInterface::HoldTestStatus::WRITE_SENT: label = "WRITE_SENT"; break;
            case MotorInterface::HoldTestStatus::FEEDBACK_STABLE: label = "FEEDBACK_STABLE"; break;
            case MotorInterface::HoldTestStatus::REJECTED: label = "REJECTED"; break;
            case MotorInterface::HoldTestStatus::FAILED: label = "FAILED"; break;
        }
        MotorCommandFrame target{};
        const bool target_valid = motor_interface.GetHoldTestTarget(target);
        std::printf(
            "{\"T\":224,\"status\":\"%s\",\"target_valid\":%s,"
            "\"targets\":[%d,%d,%d,%d,%d,%d,%d]}\n",
            label, target_valid ? "true" : "false",
            static_cast<int>(target.motors[0].target_position),
            static_cast<int>(target.motors[1].target_position),
            static_cast<int>(target.motors[2].target_position),
            static_cast<int>(target.motors[3].target_position),
            static_cast<int>(target.motors[4].target_position),
            static_cast<int>(target.motors[5].target_position),
            static_cast<int>(target.motors[6].target_position));
        return;
    }

    // T=229: read-only HOME goal tracking and fault-hold status.
    if (cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 229.0)
    {
        cJSON_Delete(root);
        MotorCommandFrame home{};
        const bool target_valid = motor_interface.GetHomeTargets(home);
        const auto status = motor_interface.GetFaultHoldStatus();
        const char* fault_status = status == MotorInterface::FaultHoldStatus::IDLE ? "IDLE" :
            status == MotorInterface::FaultHoldStatus::REQUESTED ? "REQUESTED" :
            status == MotorInterface::FaultHoldStatus::WRITE_SENT ? "WRITE_SENT" :
            status == MotorInterface::FaultHoldStatus::STABLE ? "STABLE" : "FAILED";
        const auto operation = motor_interface.GetOperationSnapshot();
        const uint16_t progress = motor_interface.GetHomeProgressSnapshot();
        std::printf(
            "{\"T\":229,\"state\":\"%s\",\"operation\":\"%s\","
            "\"operation_status\":\"%s\",\"home_phase_code\":%u,"
            "\"home_stage\":%u,\"home_target_valid\":%s,"
            "\"home_targets\":[%d,%d,%d,%d,%d,%d,%d],"
            "\"fault_hold_status\":\"%s\"}\n",
            MotorStateToString(motor_interface.GetState()),
            OperationToString(operation.operation),
            OperationStatusToString(operation.status),
            static_cast<unsigned>(progress & 0xffu),
            static_cast<unsigned>((progress >> 8) & 0xffu),
            target_valid ? "true" : "false",
            static_cast<int>(home.motors[0].target_position),
            static_cast<int>(home.motors[1].target_position),
            static_cast<int>(home.motors[2].target_position),
            static_cast<int>(home.motors[3].target_position),
            static_cast<int>(home.motors[4].target_position),
            static_cast<int>(home.motors[5].target_position),
            static_cast<int>(home.motors[6].target_position),
            fault_status);
        return;
    }

    // T=221: last completed, freshness-validated raw encoder readings.
    // No new servo read request is made here. Only snapshot inspection.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 221.0)
    {
        cJSON_Delete(root);
        MotorFeedbackFrame raw{};
        int64_t oldest_age_us = -1;
        if (!motor_interface.GetRawEncoderSnapshot(raw, oldest_age_us))
        {
            std::puts("{\"T\":221,\"ok\":false,\"error\":\"raw_feedback_unavailable\"}");
            return;
        }
        std::printf(
            "{\"T\":221,\"ok\":true,\"servo_ids\":[11,12,13,14,15,16,17],"
            "\"positions\":[%d,%d,%d,%d,%d,%d,%d],"
            "\"velocities\":[%d,%d,%d,%d,%d,%d,%d],"
            "\"oldest_age_us\":%lld}\n",
            static_cast<int>(raw.motors[0].position),
            static_cast<int>(raw.motors[1].position),
            static_cast<int>(raw.motors[2].position),
            static_cast<int>(raw.motors[3].position),
            static_cast<int>(raw.motors[4].position),
            static_cast<int>(raw.motors[5].position),
            static_cast<int>(raw.motors[6].position),
            static_cast<int>(raw.motors[0].velocity),
            static_cast<int>(raw.motors[1].velocity),
            static_cast<int>(raw.motors[2].velocity),
            static_cast<int>(raw.motors[3].velocity),
            static_cast<int>(raw.motors[4].velocity),
            static_cast<int>(raw.motors[5].velocity),
            static_cast<int>(raw.motors[6].velocity),
            static_cast<long long>(oldest_age_us));
        return;
    }

    // T=212: read-only POSITION hold preflight; no target publication.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 212.0)
    {
        cJSON_Delete(root);
        MotorCommandFrame hold{};
        if (!motor_interface.PreparePositionHold(hold))
        {
            std::puts("{\"T\":212,\"ok\":false,\"error\":\"hold_preflight_failed\"}");
            return;
        }

        std::printf(
            "{\"T\":212,\"ok\":true,\"positions\":[%d,%d,%d,%d,%d,%d,%d]}\n",
            static_cast<int>(hold.motors[0].target_position),
            static_cast<int>(hold.motors[1].target_position),
            static_cast<int>(hold.motors[2].target_position),
            static_cast<int>(hold.motors[3].target_position),
            static_cast<int>(hold.motors[4].target_position),
            static_cast<int>(hold.motors[5].target_position),
            static_cast<int>(hold.motors[6].target_position)
        );
        return;
    }

    // T=213: read-only ARM preview with stillness and feedback checks.
    // It never requests ARM, changes command snapshots or transmits writes.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 213.0)
    {
        cJSON_Delete(root);
        MotorCommandFrame hold{};
        if (!motor_interface.PreviewPositionArm(hold))
        {
            std::puts("{\"T\":213,\"ok\":false,\"error\":\"arm_preview_failed\"}");
            return;
        }
        std::printf(
            "{\"T\":213,\"ok\":true,\"dry_run\":true,"
            "\"torque_write_enabled\":false,\"positions\":[%d,%d,%d,%d,%d,%d,%d]}\n",
            static_cast<int>(hold.motors[0].target_position),
            static_cast<int>(hold.motors[1].target_position),
            static_cast<int>(hold.motors[2].target_position),
            static_cast<int>(hold.motors[3].target_position),
            static_cast<int>(hold.motors[4].target_position),
            static_cast<int>(hold.motors[5].target_position),
            static_cast<int>(hold.motors[6].target_position)
        );
        return;
    }

    // T=214: vendor-derived Servo OFF parking plan (read-only).
    // The factory OFF routine parks before releasing all torque; ID 13
    // is temporarily released/re-engaged, not sent a position goal.
    // -1 means no position command for that servo in the vendor park step.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 214.0)
    {
        cJSON_Delete(root);
        MotorCommandFrame hold{};
        if (!motor_interface.PreparePositionHold(hold))
        {
            std::puts("{\"T\":214,\"ok\":false,\"error\":\"park_preview_failed\"}");
            return;
        }

        // RoArm-M3 vendor Move_to_location(): 2047 center, shoulder
        // driving 2047-1024, elbow 2047+1024. The driven shoulder
        // servo has no direct position goal in this sequence.
        constexpr int kParkGoal[MOTOR_COUNT] =
            {2047, 1023, -1, 3071, 2047, 2047, 2047};
        int max_delta = 0;
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (kParkGoal[i] < 0) continue;
            const int delta = kParkGoal[i] -
                static_cast<int>(hold.motors[i].target_position);
            const int magnitude = delta >= 0 ? delta : -delta;
            if (magnitude > max_delta) max_delta = magnitude;
        }
        std::printf(
            "{\"T\":214,\"ok\":true,\"dry_run\":true,"
            "\"torque_write_enabled\":false,"
            "\"park_goals\":[2047,1023,null,3071,2047,2047,2047],"
            "\"max_delta_counts\":%d}\n",
            max_delta
        );
        return;
    }

    // T=216: Home preview (read-only). Actual Homing FSM is not enabled.
    if (cJSON_IsObject(root) &&
        cJSON_IsNumber(type) &&
        type->valuedouble == 216.0)
    {
        cJSON_Delete(root);
        MotorCommandFrame home{};
        if (!motor_interface.PreviewPositionHome(home))
        {
            std::puts("{\"T\":216,\"ok\":false,\"error\":\"home_preflight_failed\"}");
            return;
        }
        std::printf(
            "{\"T\":216,\"ok\":true,\"dry_run\":true,"
            "\"servo_write_enabled\":false,"
            "\"calibration_enabled\":false,"
            "\"home_goals\":[%d,%d,null,%d,%d,%d,%d]}\n",
            static_cast<int>(home.motors[0].target_position),
            static_cast<int>(home.motors[1].target_position),
            static_cast<int>(home.motors[3].target_position),
            static_cast<int>(home.motors[4].target_position),
            static_cast<int>(home.motors[5].target_position),
            static_cast<int>(home.motors[6].target_position));
        return;
    }

    const bool feedback_requested =
        cJSON_IsObject(root) && cJSON_IsNumber(type) &&
        type->valuedouble == 105.0;
    cJSON_Delete(root);

    if (feedback_requested) {
        char reply[192]{};
        if (command_interface.BuildFeedbackJson(reply, sizeof(reply))) {
            // Minimal custom feedback format; not vendor wire-compatible.
            std::puts(reply);
        } else {
            std::puts("{\"ok\":false,\"error\":\"feedback_unavailable\"}");
        }
        return;
    }

    const auto result = command_interface.HandleJsonCommand(json);
    std::printf("{\"ok\":%s,\"status\":\"%s\"}\n",
                result == ArmCommandInterface::CommandResult::ACCEPTED
                    ? "true" : "false",
                ResultToString(result));
}

void CommandTask(void*) {
    // Requires ESP-IDF default (simple, non-blocking) console stdin VFS,
    // configured for UART0 at 115200. Do not enable blocking UART VFS reads.
    // UART1 is exclusively owned by MotorInterface / ServoTask.

    char line[kLineCapacity]{};
    std::size_t length = 0;
    bool discard_oversize = false;

    ESP_LOGI(TAG, "JSON line input ready (stdin console). Send one JSON per line");

    for (;;) {
        // Limit input work per iteration so Process() continues to run.
        for (std::size_t i = 0; i < kMaxCharsPerCycle; ++i) {
            char ch = '\0';
            const ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n <= 0) {
                break;
            }

            if (ch == '\r' || ch == '\n') {
                if (discard_oversize) {
                    std::puts("{\"ok\":false,\"error\":\"line_too_long\"}");
                } else if (length > 0) {
                    line[length] = '\0';
                    HandleJsonLine(line);
                }
                length = 0;
                discard_oversize = false;
                continue;
            }

            if (discard_oversize) continue;

            if (length + 1 >= sizeof(line)) {
                discard_oversize = true;
                length = 0;
                continue;
            }
            line[length++] = ch;
        }

        command_interface.Process();
        vTaskDelay(kCommandTaskPeriod);
    }
}

} // namespace

extern "C" void app_main(void) {
    if (BoardInitAll(&board) != ESP_OK) {
        ESP_LOGE(TAG, "Board initialization failed");
        return;
    }

    if (motor_interface.Initialize(board) != ESP_OK) {
        ESP_LOGE(TAG, "Motor interface initialization failed");
        return;
    }

    TaskHandle_t command_task = nullptr;
    const BaseType_t created = xTaskCreatePinnedToCore(
        CommandTask,
        "ArmCommandTask",
        kCommandTaskStackWordsOrBytes,
        nullptr,
        3,
        &command_task,
        1
    );

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ArmCommandTask");
        // Do not enable motor output or torque.
        motor_interface.RequestServoOff();
        return;
    }

    ESP_LOGI(TAG, "MotorInterface and ArmCommandTask started");
}
