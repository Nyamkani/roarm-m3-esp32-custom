#!/usr/bin/env python3
"""Read-only RoArm-M3 calibration schema smoke test; sends T211/T219/T220 only.

No ARM, HOME, PARK, calibration write or MOVE is sent. Opening UART may reset
the ESP32 on some USB adapters: run only while stationary.
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
        data = port.readline().decode("utf-8", errors="replace").strip()
        try:
            result = json.loads(data)
        except ValueError:
            continue
        if isinstance(result, dict) and result.get("T") == command:
            return result
    raise RuntimeError("T={} timeout".format(command))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--expect-state", choices=("DISARMED", "ARMED"))
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: pyserial is required", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            status = query(port, 211, args.timeout)
            init = query(port, 219, args.timeout)
            calibration = query(port, 220, args.timeout)
        for result in (status, init, calibration):
            print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
        if args.expect_state and status.get("state") != args.expect_state:
            raise RuntimeError("wrong motor state: " + str(status.get("state")))
        for result in (status, init):
            if result.get("initialization_complete") is not False or result.get("calibration_valid") is not False or result.get("move_allowed") is not False:
                raise RuntimeError("readiness must remain false")
        if init.get("initialization_status") != "NOT_INITIALIZED" or init.get("execution_enabled") is not False:
            raise RuntimeError("T219 initialization execution must remain disabled")
        if (calibration.get("schema_version") != 1 or
            calibration.get("calibration_valid") is not False or
            calibration.get("record_verified") is not False or
            calibration.get("persisted") is not False or
            calibration.get("shoulder_pair_sum_raw") is not None):
            raise RuntimeError("T220 incorrectly reports valid calibration")
        for key in ("zero_raw", "direction", "raw_limits"):
            if calibration.get(key) != [None] * 7:
                raise RuntimeError("T220 expected 7 unknown entries for " + key)
        print("PASS: calibration schema readable; all references unknown; MOVE remains blocked")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print("FAIL: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
