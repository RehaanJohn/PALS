#!/bin/bash
PALS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# =============================================================
# arm.sh — Arm drone and enable OFFBOARD mode
# Run this AFTER teleop is running in Terminal 3
# =============================================================

source /opt/ros/humble/setup.bash
source "${PALS_ROOT}/ros2_ws/install/setup.bash"

echo ""
echo "Arming drone..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

sleep 1

echo ""
echo "Setting OFFBOARD mode..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'OFFBOARD'}"

echo ""
echo "Done! Switch to Terminal 3 (teleop) and press I to take off."
echo "Fly with WASD (forward/back/left/right) and IJKL (up/down/yaw)."
