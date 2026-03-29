#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"

if [[ ! -f "$SIM_VEHICLE" ]]; then
  echo "sim_vehicle.py not found at: $SIM_VEHICLE"
  echo "Set ARDUPILOT_DIR if your ArduPilot checkout lives somewhere else."
  exit 1
fi

cd "$ARDUPILOT_DIR/ArduCopter"
exec python3 "$SIM_VEHICLE" -v ArduCopter -I1 --out=udp:127.0.0.1:14560 --console --map