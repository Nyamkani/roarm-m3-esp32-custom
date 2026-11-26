#!/usr/bin/env bash
# Safe compile-only Home gate verification. Never flashes or sends servo commands.
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v idf.py >/dev/null 2>&1; then
  echo "ERROR: idf.py not found. Activate ESP-IDF v6.1 first." >&2
  exit 2
fi
if ! grep -Eq '^[[:space:]]*#[[:space:]]*define[[:space:]]+MOTOR_HOME_EXECUTION_ENABLE[[:space:]]+0([[:space:]]|$)' main/motor_interface.h; then
  echo "ERROR: HOME execution gate is not proven OFF. Refusing to build test configuration." >&2
  exit 3
fi
if ! grep -Eq '^[[:space:]]*#[[:space:]]*define[[:space:]]+MOTOR_EEPROM_WRITE_ENABLE[[:space:]]+0([[:space:]]|$)' main/motor_interface.h; then
  echo "ERROR: EEPROM writes are not proven disabled." >&2
  exit 3
fi
if ! grep -Eq '^[[:space:]]*#[[:space:]]*define[[:space:]]+MOTOR_HOME_COMPILE_CHECK[[:space:]]+1([[:space:]]|$)' main/motor_interface.h; then
  echo "ERROR: HOME internal compile-check is not enabled." >&2
  exit 3
fi
echo "PASS: HOME execution gate OFF, internal FSM compile-check ON, EEPROM writes OFF"
echo "Target: esp32 (RoArm-M3 ESP32-WROOM-32)"
idf.py set-target esp32
idf.py build
echo "PASS: ESP-IDF compile/link completed (NO flash performed)"
