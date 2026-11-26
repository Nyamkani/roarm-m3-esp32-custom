#!/usr/bin/env python3
"""Supervised hardware test: explicit in-place CURRENT POSITION Hold.

The arm must already be stationary and ARMED. This script does NOT ARM,
HOME, PARK, inject faults, or release torque. T223 DOES transmit a new
servo position goal via the ServoTask. Keep the robot supported.
Opening a serial port may reset ESP32 depending on USB adapter.
"""
import argparse
import json
import sys
import time


def query(port, t, timeout, cmd=None):
    port.reset_input_buffer()
    packet = {"T": t}
    if cmd is not None:
        packet["cmd"] = cmd
    port.write((json.dumps(packet) + "\n").encode("ascii"))
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if isinstance(obj, dict) and obj.get("T") == t:
            return obj
    raise RuntimeError(f"T={t} response timeout")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=5)
    p.add_argument("--verify-seconds", type=float, default=3)
    args = p.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: pyserial required", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            before = query(port, 211, args.timeout)
            print("before:", json.dumps(before, separators=(",", ":")))
            if before.get("state") != "ARMED":
                raise RuntimeError("requires already ARMED; no action transmitted")
            if before.get("move_allowed") is not False:
                raise RuntimeError("normal MOVE interlock unexpectedly enabled")
            raw = query(port, 221, args.timeout)
            if raw.get("ok") is not True or raw.get("servo_ids") != list(range(11, 18)):
                raise RuntimeError("fresh raw feedback missing")
            if any(abs(v) > 50 for v in raw.get("velocities", [])):
                raise RuntimeError("arm not stationary")
            print("pre-hold:", json.dumps(raw, separators=(",", ":")))
            result = query(port, 223, args.timeout, cmd=1)
            print("request:", json.dumps(result, separators=(",", ":")))
            if result.get("ok") is not True or result.get("status") != "accepted":
                raise RuntimeError("Hold test request rejected")
            end = time.monotonic() + args.verify_seconds
            reached = False
            last_status = None
            while time.monotonic() < end:
                status = query(port, 224, args.timeout)
                print("hold:", json.dumps(status, separators=(",", ":")))
                last_status = status
                if status.get("status") in ("FAILED", "REJECTED"):
                    raise RuntimeError("Hold test failed: " + str(status))
                if status.get("status") == "FEEDBACK_STABLE":
                    reached = True
                    break
                time.sleep(.15)
            if not reached:
                raise RuntimeError(f"Hold did not verify within window: {last_status}")
            after = query(port, 221, args.timeout)
            state = query(port, 211, args.timeout)
            print("after:", json.dumps(after, separators=(",", ":")))
            print("state:", json.dumps(state, separators=(",", ":")))
            if state.get("state") != "ARMED" or state.get("move_allowed") is not False:
                raise RuntimeError("unexpected motor state or normal MOVE enabled")
            targets = last_status.get("targets")
            pos = after.get("positions")
            if (last_status.get("target_valid") is not True or
                not isinstance(targets, list) or len(targets) != 7 or
                not isinstance(pos, list) or len(pos) != 7 or
                any(type(a) is not int or type(b) is not int or abs(a-b) > 10
                    for a,b in zip(targets,pos))):
                raise RuntimeError("post-Hold position mismatch")
            print("PASS: explicit current position Hold command sent and feedback stable")
            print("NOTE: this is NOT a validated Hold during communication failure or ID13 torque OFF")
            return 0
    except (OSError, ValueError, RuntimeError, TypeError) as exc:
        print("FAIL: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
