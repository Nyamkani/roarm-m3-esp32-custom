#!/usr/bin/env python3
"""Read-only JSON-line bench smoke test for the RoArm-M3 console.

Requires: python3 -m pip install pyserial
Sends ONLY T=211 and T=216. Does NOT ARM, PARK, or execute HOME.
"""
import argparse
import json
import sys
import time

READ_ONLY = (211, 216)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 console serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--expect-state", choices=["DISARMED", "ARMED"],
                        help="Assert the expected state from T=211")
    parser.add_argument("--output", help="Optional JSONL transcript path")
    return parser.parse_args()


def read_reply(port, command, timeout):
    """Ignore ESP-IDF text logs; accept only a JSON object matching query T."""
    deadline = time.monotonic() + timeout
    port.write((json.dumps({"T": command}, separators=(",", ":")) + "\n").encode())
    port.flush()
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and obj.get("T") == command:
            return obj
    raise TimeoutError(f"No JSON T={command} reply within {timeout:.1f}s")


def check(status, preview, expected):
    if status.get("T") != 211:
        raise AssertionError("Status T=211 missing")
    if status.get("state") not in {
        "INIT", "READY", "DISARMED", "ARMING", "ARMED",
        "PARKING", "HOMING", "DISARMING", "BOOT_RECOVERY", "ERROR"
    }:
        raise AssertionError(f"Unexpected servo state: {status.get('state')}")
    if "operation_status" not in status or "home_id13_last_verified" not in status:
        raise AssertionError("Required status fields missing (wrong firmware branch?)")
    if status.get("home_phase") != "NOT_STARTED" or status.get("home_phase_code") != 0:
        if status.get("operation") != "HOME":
            raise AssertionError("Unexpected Home phase without Home operation")
    state = status["state"]
    if expected and state != expected:
        raise AssertionError(f"Expected {expected}, got {state}")
    if preview.get("T") != 216:
        raise AssertionError("Home preview reply missing")
    if state == "DISARMED":
        if preview.get("ok") is not False or preview.get("error") != "home_preflight_failed":
            raise AssertionError("T=216 should fail in DISARMED")
    elif state == "ARMED":
        if preview.get("ok") is True:
            if preview.get("dry_run") is not True or preview.get("servo_write_enabled") is not False:
                raise AssertionError("T=216 violated read-only protocol")
            if preview.get("calibration_enabled") is not False:
                raise AssertionError("T=216 calibration flag mismatch")
            if preview.get("home_goals") != [2047, 2047, None, 2047, 2047, 2047, 2047]:
                raise AssertionError("T=216 home goals mismatch")
        elif preview.get("error") != "home_preflight_failed":
            raise AssertionError(f"Unexpected ARMED preview failure: {preview}")
        # In ARMED, a failed preflight is a valid safety outcome.
    else:
        # For transitional and ERROR states, read-only preview normally fails.
        if preview.get("ok") is True:
            raise AssertionError(f"T=216 unexpectedly passed in state {state}")


def main():
    args = parse_args()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")
    try:
        import serial
    except ImportError:
        print("Install pyserial: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    # Opening the console port may reset some ESP32 USB/UART adapters.
    # Do not use during active motion. Never send a motion/torque command.
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2) as port:
            # Avoid consuming stale responses from earlier interactive commands.
            port.reset_input_buffer()
            results = [read_reply(port, command, args.timeout) for command in READ_ONLY]
        for item in results:
            print(json.dumps(item, ensure_ascii=False, separators=(",", ":")))
        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                for item in results:
                    f.write(json.dumps(item, ensure_ascii=False) + "\n")
        check(results[0], results[1], args.expect_state)
    except (OSError, TimeoutError, AssertionError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print("PASS: T211 status + T216 read-only home preview")
    return 0


if __name__ == "__main__":
    sys.exit(main())
