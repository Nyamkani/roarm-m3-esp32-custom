#!/usr/bin/env python3
"""Read-only raw ST3215 encoder snapshot smoke. Sends only T=211/T=221.

Opening a serial console can reset ESP32. Run with the arm safely stationary.
Does not ARM, HOME, PARK, MOVE, calibrate or write servo registers.
"""
import argparse
import json
import sys
import time


def query(port, t, timeout):
    port.reset_input_buffer()
    port.write((json.dumps({"T": t}) + "\n").encode("ascii"))
    port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if isinstance(obj, dict) and obj.get("T") == t:
            return obj
    raise RuntimeError(f"timeout waiting for T={t}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--expect-state", choices=["DISARMED", "ARMED"], default="DISARMED")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: pyserial required", file=sys.stderr)
        return 2

    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            status = query(port, 211, args.timeout)
            data = query(port, 221, args.timeout)
        print(json.dumps(status, ensure_ascii=False, separators=(",", ":")))
        print(json.dumps(data, ensure_ascii=False, separators=(",", ":")))
        if status.get("state") != args.expect_state:
            raise RuntimeError("unexpected state: " + str(status.get("state")))
        if status.get("initialization_complete") is not False or status.get("move_allowed") is not False:
            raise RuntimeError("initialization/MOVE unexpectedly enabled")
        if data.get("ok") is not True or data.get("servo_ids") != list(range(11, 18)):
            raise RuntimeError("missing or invalid seven-servo snapshot")
        pos, vel = data.get("positions"), data.get("velocities")
        if not isinstance(pos, list) or len(pos) != 7 or any(type(x) is not int or x < 0 or x > 4095 for x in pos):
            raise RuntimeError("invalid raw encoder positions")
        if not isinstance(vel, list) or len(vel) != 7 or any(type(x) is not int for x in vel):
            raise RuntimeError("invalid raw velocities")
        age = data.get("oldest_age_us")
        if type(age) is not int or age < 0:
            raise RuntimeError("invalid feedback age")
        print("PASS: 7 raw encoder positions available; no calibration or MOVE enabled")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print("FAIL: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
