#!/usr/bin/env python3
"""Read-only initialization interlock smoke test for RoArm-M3.

Sends only T=211 (diagnostic). Does not ARM, HOME, PARK, MOVE or change torque.
Requires pyserial; no ESP-IDF environment is necessary.
"""
import argparse
import json
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--expect-state", choices=("ARMED", "DISARMED"))
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("FAIL: install pyserial", file=sys.stderr)
        return 2

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2) as port:
            # Opening a USB serial console can reset the ESP32 on some adapters.
            # Never run this test during physical motion.
            port.reset_input_buffer()
            port.write(b'{"T":211}\n')
            port.flush()
            deadline = time.monotonic() + args.timeout
            status = None
            while time.monotonic() < deadline:
                line = port.readline().decode("utf-8", errors="replace").strip()
                try:
                    data = json.loads(line)
                except (ValueError, TypeError):
                    continue
                if isinstance(data, dict) and data.get("T") == 211:
                    status = data
                    break
            if status is None:
                raise RuntimeError("No T=211 response")
            print(json.dumps(status, ensure_ascii=False, separators=(",", ":")))
            if args.expect_state and status.get("state") != args.expect_state:
                raise RuntimeError(
                    f"expected {args.expect_state}, got {status.get('state')}"
                )
            if status.get("initialization_complete") is not False:
                raise RuntimeError("initialization_complete must be false until calibration exists")
            if status.get("move_allowed") is not False:
                raise RuntimeError("MOVE must be blocked before valid initialization")
            print("PASS: initialization incomplete; MOVE interlock closed (diagnostic only)")
            return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
