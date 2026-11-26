#!/usr/bin/env python3
"""Supervised active FAULT_HOLD bench test (no injected UART faults).

Prerequisites: physical support, stationary arm already ARMED, full
operator supervision. Opening host serial port can reset the controller.
The T225 command actively switches to FAULT_HOLD and transmits CURRENT
position targets over ST3215 UART1. No HOME, PARK, servo EEPROM, calibration.
By default leave the arm in FAULT_HOLD; --resume requests explicit ARMED.
"""
import argparse
import json
import sys
import time

def query(port, t, timeout, cmd=None):
    port.reset_input_buffer()
    payload = {"T": t}
    if cmd is not None:
        payload["cmd"] = cmd
    port.write((json.dumps(payload) + "\n").encode("ascii"))
    port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("utf8", "replace").strip()
        try:
            item = json.loads(line)
        except ValueError:
            continue
        if isinstance(item, dict) and item.get("T") == t:
            return item
    raise RuntimeError(f"T={t} response timeout")

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--verify-seconds", type=float, default=4.0)
    p.add_argument("--resume", action="store_true",
                   help="explicitly return to ARMED only after STABLE")
    args = p.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: pyserial required", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            status = query(port, 211, args.timeout)
            print("before:", json.dumps(status, separators=(",", ":")))
            if status.get("state") != "ARMED" or status.get("move_allowed") is not False:
                raise RuntimeError("requires stationary ARMED and normal MOVE blocked")
            raw = query(port, 221, args.timeout)
            print("pre-fault:", json.dumps(raw, separators=(",", ":")))
            pos = raw.get("positions")
            vel = raw.get("velocities")
            if (raw.get("ok") is not True or not isinstance(pos, list) or
                len(pos) != 7 or not isinstance(vel, list) or len(vel) != 7 or
                any(type(x) is not int or abs(x) > 50 for x in vel) or
                abs(pos[1] + pos[2] - 4094) > 64):
                raise RuntimeError("raw feedback invalid or non-stationary")
            result = query(port, 225, args.timeout, cmd=1)
            print("request:", json.dumps(result, separators=(",", ":")))
            if result.get("ok") is not True:
                raise RuntimeError("T225 request rejected")
            deadline = time.monotonic() + args.verify_seconds
            latest = None
            while time.monotonic() < deadline:
                latest = query(port, 227, args.timeout)
                print("fault hold:", json.dumps(latest, separators=(",", ":")))
                if latest.get("status") == "FAILED":
                    raise RuntimeError("fault hold failed")
                if latest.get("status") == "STABLE":
                    break
                time.sleep(.15)
            if latest is None or latest.get("status") != "STABLE":
                raise RuntimeError("fault hold did not reach STABLE")
            state = query(port, 211, args.timeout)
            feedback = query(port, 221, args.timeout)
            print("state:", json.dumps(state, separators=(",", ":")))
            print("feedback:", json.dumps(feedback, separators=(",", ":")))
            if state.get("state") != "FAULT_HOLD" or state.get("move_allowed") is not False:
                raise RuntimeError("fault state not latched or MOVE enabled")
            goals = latest.get("targets")
            if (latest.get("target_valid") is not True or
                not isinstance(goals, list) or len(goals) != 7 or
                feedback.get("ok") is not True or
                any(abs(a - b) > 10 for a, b in zip(goals, feedback["positions"]))):
                raise RuntimeError("fault hold feedback differs from target")
            print("PASS: FAULT_HOLD latched; position writes and stable feedback verified")
            if args.resume:
                resume = query(port, 226, args.timeout, cmd=1)
                print("resume:", json.dumps(resume, separators=(",", ":")))
                if resume.get("ok") is not True:
                    raise RuntimeError("resume was rejected")
                end = time.monotonic() + args.timeout
                while time.monotonic() < end:
                    state = query(port, 211, args.timeout)
                    if state.get("state") == "ARMED":
                        print("PASS: explicit bench recovery to ARMED")
                        break
                    time.sleep(.1)
                else:
                    raise RuntimeError("state did not return to ARMED")
            else:
                print("NOTE: arm remains FAULT_HOLD with torque ON; physical supervision required")
                print("To resume, use T226 cmd=1 after confirming current state is STABLE.")
            print("NOTE: this does NOT test real communication failures or ID13 torque-OFF recovery")
            return 0
    except (OSError, RuntimeError, ValueError, TypeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())
