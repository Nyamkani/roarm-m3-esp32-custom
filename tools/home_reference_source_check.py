#!/usr/bin/env python3
"""Host-only regression check: HOME raw pose differs from PARK and FULL HOME requires supervised PARK-only start.

No serial port, MCU, motors, or firmware output involved.
Run from repository root: python3 tools/home_reference_source_check.py
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
header = (root / "main/motor_interface.h").read_text(encoding="utf-8")
source = (root / "main/motor_interface.cpp").read_text(encoding="utf-8")

def raw_array(name):
    found = re.search(r"\b" + re.escape(name) + r"\s*\[\s*MOTOR_COUNT\s*\]\s*=\s*\{([^}]+)\}", header)
    if found is None:
        raise AssertionError(f"{name} not declared")
    return [int(v.strip()) for v in found.group(1).split(",")]

expected = [2043, 1703, 2386, 2927, 1549, 2048, 2046]
reference = raw_array("HOME_REFERENCE_RAW")
goals = raw_array("HOME_GOALS")
park = raw_array("PARK_GOALS")
assert reference == expected, (reference, expected)
assert goals == [2043, 1703, -1, 2927, 1549, 2048, 2046], goals
assert park != goals, "PARK and HOME targets must differ"
assert reference[1] + reference[2] == 4089
assert abs(reference[1] + reference[2] - 4094) <= 64
for macro in ("MOTOR_HOME_ALLOW_FULL_EXECUTION", "MOTOR_VENDOR_SHOULDER_PARK_ENABLE"):
    assert re.search(r"#define\s+" + macro + r"\s+1\b", header), f"{macro} must be ON for supervised test"
assert "HOME_PARK_START_TOLERANCE" in header
assert "HOME_SHOULDER_CORRIDOR_MARGIN" in header
assert "FULL HOME blocked: not at PARK" in source
assert "HOME shoulder raw corridor exceeded" in source
assert "HOME_GOALS[i]" in source, "preview and execution must use shared goals"
assert "std::abs(id12 - HOME_REFERENCE_RAW[1]) > 10" in source
assert "std::abs(id13 - HOME_REFERENCE_RAW[2]) > HOME_ID13_REFERENCE_TOLERANCE" in source
assert "std::abs(pos - HOME_REFERENCE_RAW[i]) > allowed_error" in source
print("PASS: seven-servo HOME reference matches user-confirmed Raw Encoder values")
print("PASS: ID13 is passive during shoulder motion; checked before re-enable and final completion")
print("PASS: supervised PARK and FULL HOME enabled with PARK-only pose and shoulder corridor interlocks")
print("NOTE: source regression check only, not compilation or physical safety validation")
