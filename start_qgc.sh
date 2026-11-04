#!/bin/bash
PALS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# =============================================================
# start_qgc.sh
# Starts the background video stream from ROS 2 to UDP,
# and launches the QGroundControl AppImage.
# =============================================================

# Add AppImage to path (sometimes needed by AppImages on Ubuntu 22.04)
export APPIMAGE_EXTRACT_AND_RUN=1

echo "Starting Video Streamer in background..."
source /opt/ros/humble/setup.bash
source "${PALS_ROOT}/ros2_ws/install/setup.bash"

# Run the python video bridging script we built
ros2 run drone_ml qgc_stream &
STREAM_PID=$!

echo "Video Bridge PID: $STREAM_PID"
echo "Launching QGroundControl..."

# Launch QGC (extracted to avoid FUSE issues on Ubuntu 22.04)
"${PALS_ROOT}/squashfs-root/AppRun"

# When QGC is closed, kill the video stream
echo "QGroundControl closed. Stopping video stream..."
kill -9 $STREAM_PID 2>/dev/null
wait $STREAM_PID 2>/dev/null
echo "Done."
