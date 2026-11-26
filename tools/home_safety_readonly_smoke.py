#!/usr/bin/env python3
"""Read-only HOME safety configuration smoke: T211 and T222 only.

No ARM/HOME/PARK/MOVE/TORQUE writes. UART opening can reset the ESP32.
Run only when the arm is stationary and supported.
"""
import argparse
import json
import sys
import time

def query(port, command, timeout):
    port.reset_input_buffer()
    port.write((json.dumps({"T": command}) + "\n").encode("ascii"))
    port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("utf8", "replace").strip()
        try:
            payload = json.loads(line)
        except ValueError:
            continue
        if isinstance(payload, dict) and payload.get("T") == command:
            return payload
    raise RuntimeError(f"T={command} timed out")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--expect-state", default="DISARMED")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: install pyserial", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            motor = query(port, 211, args.timeout)
            safety = query(port, 222, args.timeout)
        print(json.dumps(motor, ensure_ascii=False, separators=(",", ":")))
        print(json.dumps(safety, ensure_ascii=False, separators=(",", ":")))
        if motor.get("state") != args.expect_state:
            raise RuntimeError("unexpected motor state")
        if motor.get("initialization_complete") is not False or motor.get("move_allowed") is not False:
            raise RuntimeError("normal MOVE must stay blocked")
        expected = {
            "development_mode": True,
            "joint_limits_verified": False,
            "fault_hold_output_enabled": False,
            "fault_recorded": False,
            "fault_home_phase_code": 0,
        }
        for key, value in expected.items():
            if safety.get(key) is not value:
                raise RuntimeError(f"{key}: expected {value}, got {safety.get(key)}")
        print("PASS: limit policy configured; HOME disabled; active fault hold output disabled")
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())
