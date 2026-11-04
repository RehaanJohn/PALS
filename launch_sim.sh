#!/bin/bash
PALS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export QT_QPA_PLATFORM=xcb
# =============================================================
# Launch PX4 SITL + Gazebo Harmonic with X500 Quadcopter
# Terminal 1 — Run this first
# =============================================================

export PX4_SIMULATOR=gz
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=urban

# Required by px4-rc.simulator to know where to find the .sdf world file and spawn Gazebo automatically
export PX4_GZ_WORLDS=${PALS_ROOT}/px4/Tools/simulation/gz/worlds
export GZ_SIM_RESOURCE_PATH=${PALS_ROOT}/px4/Tools/simulation/gz/models:${PALS_ROOT}/models

PX4_BIN=${PALS_ROOT}/px4/build/px4_sitl_default/bin/px4
PX4_ROOTFS=${PALS_ROOT}/px4/build/px4_sitl_default

echo "Launching PX4 SITL + Gazebo Harmonic (x500 quadcopter)..."
echo "Press Ctrl+C to stop."
echo ""

# ── Background camera follow loop ────────────────────────────
# Runs in the same environment as PX4/Gazebo so gz topic works
(
  echo "[camera] Waiting for Gazebo world to load..."
  # Wait until the simulation clock topic appears
  for i in $(seq 1 120); do
    if gz topic -l 2>/dev/null | grep -q "/world/urban/clock"; then
      break
    fi
    sleep 1
  done
  echo "[camera] World running. Waiting 6s for drone to spawn (x500_0)..."
  sleep 6
  echo "[camera] Locking camera onto x500_0..."
  # Keep republishing to fight mouse-drag unlock
  while true; do
    gz topic -t /gui/follow \
      --msgtype gz.msgs.StringMsg \
      -p 'data: "x500_0"' 2>/dev/null
    sleep 0.3
    gz topic -t /gui/follow/offset \
      --msgtype gz.msgs.Vector3d \
      -p 'x: -2.5, y: 0, z: 1.0' 2>/dev/null
    sleep 0.7
  done
) &
FOLLOW_PID=$!

# Must cd into the rootfs dir — PX4 resolves startup script relative to CWD
cd "${PX4_ROOTFS}" || exit 1
"${PX4_BIN}" . -i 0

# Clean up camera loop when PX4 exits
kill $FOLLOW_PID 2>/dev/null
