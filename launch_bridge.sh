#!/bin/bash
PALS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# =============================================================
# launch_bridge.sh — MAVROS Bridge
# Terminal 2 — Run after Gazebo is loaded (Terminal 1)
# Camera follow is handled automatically by launch_sim.sh.
# =============================================================

source /opt/ros/humble/setup.bash
source "${PALS_ROOT}/ros2_ws/install/setup.bash"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  MAVROS Bridge                               ║"
echo "║  Wait for Gazebo to fully load first!        ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ── Start MAVROS in the background ───────────────────────────
echo "[1/2] Starting MAVROS bridge..."
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580" \
    tgt_system:=1 \
    tgt_component:=1 \
    log_output:=screen &
MAVROS_PID=$!

echo "      MAVROS PID: $MAVROS_PID"
echo ""

# ── Start Sensor Bridge (Gazebo → ROS2) ──────────────────────
echo "[2/2] Starting Sensor Bridge (Camera + LiDAR)..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
    /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked &
BRIDGE_PID=$!

echo "      Bridge PID: $BRIDGE_PID"
echo ""
echo "═══════════════════════════════════════════════"
echo "  MAVROS & Sensors running. Camera follow is automatic."
echo ""
echo "  Sensors available in ROS 2:"
echo "    - /camera (RGB)"
echo "    - /camera/depth_image"
echo "    - /lidar/points (3D PointCloud)"
echo ""
echo "  Next step: open Terminal 3 and run:"
echo "    ros2 run drone_ml teleop"
echo ""
echo "  Press Ctrl+C to stop MAVROS & Sensors."
echo "═══════════════════════════════════════════════"

trap "echo 'Stopping...'; kill $MAVROS_PID $BRIDGE_PID 2>/dev/null; wait; exit 0" SIGINT SIGTERM
wait $MAVROS_PID $BRIDGE_PID
