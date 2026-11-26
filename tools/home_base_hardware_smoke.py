#!/usr/bin/env python3
"""Supervised BASE_ONLY physical HOME: no ID13 torque release, no full HOME.

Requires an already ARMED, stationary and mechanically supported robot.
T217 cmd=1 physically commands ID11 Base toward raw 2047. Other joints
retain their captured hold targets. Do not use on an unattended arm.

Only this script initiates HOME; a failed HOME stays in FAULT_HOLD or ERROR.
No automatic torque release, reset, or re-ARM. The operator must inspect
faults and support the arm. Opening UART0 may reset the MCU.
"""
import argparse
import json
import sys
import time


def query(port, t, timeout):
    port.reset_input_buffer()
    port.write((json.dumps({"T": t}) + "\n").encode("ascii"))
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        try:
            item = json.loads(line)
        except ValueError:
            continue
        if isinstance(item, dict) and item.get("T") == t:
            return item
    raise RuntimeError(f"T{t} timeout")


def command(port, t, cmd, timeout):
    port.reset_input_buffer()
    port.write((json.dumps({"T": t, "cmd": cmd}) + "\n").encode("ascii"))
    port.flush()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        try:
            item = json.loads(line)
        except ValueError:
            continue
        if isinstance(item, dict) and "status" in item and "ok" in item:
            return item
    raise RuntimeError(f"T{t} cmd={cmd} timeout")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--monitor-seconds", type=float, default=15)
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: install pyserial", file=sys.stderr)
        return 2
    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            before = query(port, 211, args.timeout)
            flags = query(port, 222, args.timeout)
            initial = query(port, 221, args.timeout)
            preview = query(port, 216, args.timeout)
            for title, item in (("before", before), ("flags", flags),
                                ("raw", initial), ("preview", preview)):
                print(title + ":", json.dumps(item, separators=(",", ":")))
            if before.get("state") != "ARMED" or before.get("move_allowed") is not False:
                raise RuntimeError("requires ARMED and normal MOVE interlock")
            if (flags.get("home_execution_enabled") is not True or
                flags.get("full_home_execution_enabled") is not False):
                raise RuntimeError("wrong firmware safety gates (BASE_ONLY required)")
            if (initial.get("ok") is not True or
                initial.get("servo_ids") != list(range(11, 18)) or
                any(abs(v) > 50 for v in initial.get("velocities", [])) or
                abs(initial["positions"][1] + initial["positions"][2] - 4094) > 64 or
                preview.get("ok") is not True):
                raise RuntimeError("HOME preflight/servo feedback failed")
            deny = command(port, 217, 2, args.timeout)
            print("full-home-denied:", json.dumps(deny, separators=(",", ":")))
            if deny.get("ok") is not False:
                raise RuntimeError("FULL HOME request not denied")
            start = command(port, 217, 1, args.timeout)
            print("base-home:", json.dumps(start, separators=(",", ":")))
            if start.get("ok") is not True:
                raise RuntimeError("BASE_ONLY HOME request rejected")

            deadline = time.monotonic() + args.monitor_seconds
            last = None
            while time.monotonic() < deadline:
                state = query(port, 211, args.timeout)
                progress = query(port, 229, args.timeout)
                last = state
                print("state:", json.dumps(state, separators=(",", ":")),
                      "home:", json.dumps(progress, separators=(",", ":")))
                if state.get("state") == "FAULT_HOLD":
                    fault = query(port, 227, args.timeout)
                    print("fault:", json.dumps(fault, separators=(",", ":")))
                    raise RuntimeError("BASE_HOME entered FAULT_HOLD; check T227/T222")
                if state.get("state") == "ERROR":
                    raise RuntimeError("BASE_HOME entered ERROR: operator intervention needed")
                if (state.get("state") == "ARMED" and
                    state.get("operation") == "HOME" and
                    state.get("operation_status") == "COMPLETED"):
                    after = query(port, 221, args.timeout)
                    print("after:", json.dumps(after, separators=(",", ":")))
                    if after.get("ok") is not True:
                        raise RuntimeError("feedback unavailable after completion")
                    if abs(after["positions"][0] - 2047) > 10:
                        raise RuntimeError("base did not settle at 2047 +/- 10")
                    if after.get("velocities") != [0]*7:
                        # Speed threshold is authoritative; exact zeros may jitter.
                        if any(abs(v) > 50 for v in after["velocities"]):
                            raise RuntimeError("arm not stationary after HOME")
                    print("PASS: BASE_ONLY physical HOME, no ID13 release, ARMED hold retained")
                    return 0
                time.sleep(.25)

            print("HOME monitor exceeded timeout; requesting non-torque-off hold abort")
            if last and last.get("state") == "HOMING":
                abort = command(port, 228, 1, args.timeout)
                print("hold-abort:", json.dumps(abort, separators=(",", ":")))
                if abort.get("ok") is True:
                    end = time.monotonic() + args.timeout
                    while time.monotonic() < end:
                        after_state = query(port, 211, args.timeout)
                        if after_state.get("state") in ("FAULT_HOLD", "ERROR"):
                            print("abort-state:", json.dumps(after_state, separators=(",", ":")))
                            break
                        time.sleep(.1)
            raise RuntimeError("BASE_ONLY HOME did not finish in time; inspect state and support arm")
    except (OSError, RuntimeError, ValueError, TypeError, KeyError) as exc:
        print("FAIL:", exc, file=sys.stderr)
        print("IMPORTANT: do not automatically torque-OFF an unsupported arm", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
