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
        case MotorInterfaceState::DISARMING:  return "DISARMING";
        case MotorInterfaceState::BOOT_RECOVERY: return "BOOT_RECOVERY";
        case MotorInterfaceState::ERROR:      return "ERROR";
        default:                              return "UNKNOWN";
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

        JointAngles feedback{};
        const bool feedback_valid =
            motor_interface.GetJointFeedback(feedback);

        std::printf(
            "{\"T\":211,"
            "\"state\":\"%s\","
            "\"feedback_valid\":%s,"
            "\"torque_write_enabled\":%s,"
            "\"pwm_output_enabled\":%s,"
            "\"position_output_enabled\":%s,"
            "\"physical_servo_enabled\":%s,"
            "\"torque_off_verified\":%s,"
            "\"vendor_park_enabled\":%s,"
            "\"control_mode\":\"%s\"}\n",
            MotorStateToString(state),
            feedback_valid ? "true" : "false",
            MOTOR_TORQUE_WRITE_ENABLE ? "true" : "false",
            MOTOR_PWM_OUTPUT_ENABLE ? "true" : "false",
            MOTOR_POSITION_OUTPUT_ENABLE ? "true" : "false",
            MOTOR_PHYSICAL_SERVO_ENABLE ? "true" : "false",
            motor_interface.IsTorqueOffVerified() ? "true" : "false",
            MOTOR_VENDOR_SHOULDER_PARK_ENABLE ? "true" : "false",
            MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION ? "POSITION" : "PWM"
        );

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
