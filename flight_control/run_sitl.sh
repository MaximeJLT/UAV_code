#!/bin/bash

ARDUPILOT_DIR="$HOME/ardupilot"

echo "🛩️ Launching ArduPlane QuadPlane (VTOL+Plane) SITL..."

cd "$ARDUPILOT_DIR" || {
  echo "❌ ArduPilot directory not found: $ARDUPILOT_DIR"
  exit 1
}

# --out=udp:127.0.0.1:14550  → MAVProxy console/map (lecture seule pour toi)
# --out=udp:127.0.0.1:14551  → ton script Python (port dédié, sans MAVProxy entre les deux)
#
# IMPORTANT : dans connection.py, connecte-toi sur le port 14551 et non 14550
# pour éviter que MAVProxy duplique tes messages de mission upload.
# Le port 14550 reste disponible pour Mission Planner ou d'autres GCS.

Tools/autotest/sim_vehicle.py \
  -v ArduPlane \
  -f quadplane \
  --console \
  --map \
  -L maxime_home \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
