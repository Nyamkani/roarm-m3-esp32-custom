#!/usr/bin/env python3
"""Read-only stability test for seven ST3215 raw encoder snapshots.

T=211 and repeated T=221 only; no ARM, HOME, PARK, MOVE or calibration write.
Run only with the arm stationary and mechanically supported as appropriate.
Opening the console serial port may reset ESP32 on some adapters.
"""
import argparse
import json
import statistics
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
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--max-span", type=int, default=10,
                        help="Maximum count range per servo across a stationary run")
    parser.add_argument("--max-abs-velocity", type=int, default=50)
    parser.add_argument("--shoulder-pair-reference", type=int, default=4094)
    parser.add_argument("--shoulder-tolerance", type=int, default=64)
    args = parser.parse_args()
    if args.samples < 2 or args.interval < 0 or args.max_span < 0 or args.max_abs_velocity < 0:
        parser.error("samples >= 2, interval >= 0, thresholds >= 0 required")

    try:
        import serial
    except ImportError:
        print("FAIL: pyserial required", file=sys.stderr)
        return 2

    try:
        with serial.Serial(args.port, args.baud, timeout=.2, write_timeout=2) as port:
            status = query(port, 211, args.timeout)
            if status.get("state") != "DISARMED":
                raise RuntimeError(f"expected DISARMED, got {status.get('state')}")
            if status.get("calibration_valid") is not False or status.get("move_allowed") is not False:
                raise RuntimeError("unexpected calibration or MOVE permission")
            samples = []
            for i in range(args.samples):
                snapshot = query(port, 221, args.timeout)
                if snapshot.get("ok") is not True or snapshot.get("servo_ids") != list(range(11, 18)):
                    raise RuntimeError(f"sample {i}: missing valid T221 reply")
                positions = snapshot.get("positions")
                velocities = snapshot.get("velocities")
                age = snapshot.get("oldest_age_us")
                if (not isinstance(positions, list) or len(positions) != 7 or
                    any(type(p) is not int or not 0 <= p <= 4095 for p in positions) or
                    not isinstance(velocities, list) or len(velocities) != 7 or
                    any(type(v) is not int for v in velocities) or
                    type(age) is not int or age < 0):
                    raise RuntimeError(f"sample {i}: invalid T221 data")
                samples.append((positions, velocities, age))
                print(f"sample {i+1}/{args.samples}: pos={positions} vel={velocities} oldest_age_us={age}")
                if i + 1 < args.samples:
                    time.sleep(args.interval)

        spans = []
        for idx, servo_id in enumerate(range(11, 18)):
            readings = [row[0][idx] for row in samples]
            span = max(readings) - min(readings)
            spans.append(span)
            print(f"servo {servo_id}: mean={statistics.mean(readings):.2f} span={span} counts")

        sums = [row[0][1] + row[0][2] for row in samples]
        print(f"shoulder pair sums: min={min(sums)} max={max(sums)} reference={args.shoulder_pair_reference}")
        errors = []
        if any(x > args.max_span for x in spans):
            errors.append("encoder span exceeds stationary threshold")
        if any(abs(v) > args.max_abs_velocity for row in samples for v in row[1]):
            errors.append("velocity exceeds stationary threshold")
        if any(abs(x - args.shoulder_pair_reference) > args.shoulder_tolerance for x in sums):
            errors.append("shoulder pair outside configured reference tolerance")
        if errors:
            raise RuntimeError("; ".join(errors))
        print("PASS: static encoder stability and shoulder pairing within configured thresholds")
        print("NOTE: does not verify mechanical zero, joint limits, or Homing safety")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
