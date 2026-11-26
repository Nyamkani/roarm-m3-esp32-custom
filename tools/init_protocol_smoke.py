#!/usr/bin/env python3
"""Supervised-init protocol smoke: T=219 status, optional gated T=218 refusal.

Never sends ARM, HOME, PARK, calibration-write or MOVE commands. T=218 is
sent only after T=219 reports execution_enabled=false, and must be refused.
Opening serial may reset ESP32; run only while stationary.
"""
import argparse
import json
import sys
import time


def query(port, obj, expected_t=None, timeout=5):
    port.reset_input_buffer()
    port.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii"))
    port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = port.readline().decode("utf-8", "replace").strip()
        try:
            reply = json.loads(raw)
        except ValueError:
            continue
        if isinstance(reply, dict) and (expected_t is None or reply.get("T") == expected_t):
            if expected_t is None and "ok" not in reply and "status" not in reply:
                continue
            return reply
    raise RuntimeError(f"no response for {obj}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--expect-state", choices=("DISARMED", "ARMED"))
    parser.add_argument("--check-rejection", action="store_true",
                        help="Also send T=218,cmd=1 ONLY after confirming execution_enabled=false")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: pyserial required", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            status = query(port, {"T": 211}, expected_t=211, timeout=args.timeout)
            print(json.dumps(status, ensure_ascii=False))
            if args.expect_state and status.get("state") != args.expect_state:
                raise RuntimeError(f"expected state {args.expect_state}, got {status.get('state')}")
            if status.get("initialization_complete") is not False or status.get("calibration_valid") is not False or status.get("move_allowed") is not False:
                raise RuntimeError("T211 missing fail-closed initialization/calibration status")
            init = query(port, {"T": 219}, expected_t=219, timeout=args.timeout)
            print(json.dumps(init, ensure_ascii=False))
            if (init.get("initialization_status") != "NOT_INITIALIZED" or
                init.get("initialization_complete") is not False or
                init.get("calibration_valid") is not False or
                init.get("move_allowed") is not False or
                init.get("execution_enabled") is not False):
                raise RuntimeError("T219 did not confirm disabled initialization")
            if args.check_rejection:
                if status.get("state") != "DISARMED":
                    raise RuntimeError("T218 refusal check restricted to DISARMED")
                result = query(port, {"T": 218, "cmd": 1}, timeout=args.timeout)
                print(json.dumps(result, ensure_ascii=False))
                if result.get("ok") is not False or result.get("status") != "motor_unavailable":
                    raise RuntimeError("T218 not refused as expected")
            print("PASS: initialization and calibration remain invalid; MOVE blocked; initialization execution disabled")
            return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
