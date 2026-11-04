# PALS

# 🛸 Drone Simulation Project: Complete Guide
=============================================
Verified against: ROS2 Humble | PX4 v1.14.x | Gazebo Harmonic | Ubuntu 22.04

This project provides a high-fidelity simulation environment for autonomous drones, integrating professional flight stacks with modern robotics middleware and procedural world generation.

============================================================================================
 🏗️ SYSTEM ARCHITECTURE
============================================================================================

The simulator is built on a "Power Trio" of robotics software:

1.  **PX4 Autopilot (SITL):** Runs the actual flight control logic, handling stabilization, safety, and GPS mission execution.
2.  **Gazebo Harmonic:** A state-of-the-art physics engine that simulates the environment, drone dynamics, and advanced sensors.
3.  **ROS 2 Humble:** The "Intelligence Layer" where high-level vision, mapping, and AI scripts live.
4.  **MAVROS:** The bridge translating between PX4 (MAVLink protocol) and ROS 2 (DDS protocol).
5.  **QGroundControl:** The professional dashbord for mission planning, live telemetry, and H.264 video monitoring.

============================================================================================
 🚀 WHAT WE BUILT (KEY FEATURES)
============================================================================================

### 🌆 Procedural Urban World
We generated a complex city environment using `generate_city.py`.
- **Details:** 400x400m grid with randomly generated buildings, roads, and trees.
- **Purpose:** To provide a realistic "obstacle course" for testing avoidance algorithms.

### 🎥 Smart Follow-Camera
A custom "GTA-style" camera that locks onto the drone.
- **Implementation:** Uses a high-frequency tracking loop (`launch_sim.sh` & `follow_cam.py`) to stay 2.5m behind the drone.
- **Controls:** Press **'V'** in the teleop terminal to toggle the follow-cam on/off.

### 🤖 Sensory Superpowers
The simulated **x500** drone is equipped with:
- **Depth Camera:** See the world in 3D (Topic: `/camera/depth_image`).
- **3D LiDAR:** 16-beam laser for high-speed obstacle detection (Topic: `/lidar/points`).

### 📺 QGroundControl & Live Video
We integrated the industry-standard dashboard with a custom video feed.
- **H.264 Stream:** A ROS 2 node (`stream_video.py`) encodes the drone's view and broadcasts it via UDP to QGC.
- **FPV Experience:** You can see the live camera feed directly inside the QGC flight dashboard.

============================================================================================
 🎮 HOW TO RUN (THE WORKFLOW)
============================================================================================

### STEP 1: Start the World (Terminal 1)
```bash
bash ~/drone_sim/launch_sim.sh
```
Wait ~15s for the world to load and the camera to lock on.

### STEP 2: Start the Bridge (Terminal 2)
```bash
bash ~/drone_sim/launch_bridge.sh
```
This connects the drone's "brain" to ROS 2.

### STEP 3: Take Control (Terminal 3)
```bash
source ~/drone_sim/ros2_ws/install/setup.bash
ros2 run drone_ml teleop
```
**Controls:**
- **W / S**: Forward / Backward
- **A / D**: Slide Left / Right
- **I / K**: Up / Down (Takeoff: Press **I** after arming!)
- **J / L**: Rotate (Yaw)
- **V**: Toggle Follow-Camera
- **SPACE**: Immediate Hover

### STEP 4: Dashboard & Missions (Optional Terminal 4)
```bash
bash ~/drone_sim/start_qgc.sh
```
Use QGC for waypoint missions, health monitoring, and the live FPV video stream.

---
*Note: If you don't use QGC, you can manually arm the drone using `bash ~/drone_sim/arm.sh` before taking off in Terminal 3.*

============================================================================================
 🛠️ FOLDER STRUCTURE
============================================================================================

~/drone_sim/
├── launch_sim.sh     — Main entry point (PX4 + Gazebo + Camera)
├── launch_bridge.sh  — ROS 2 connection
├── start_qgc.sh      — QGroundControl + Video Streamer
├── generate_city.py  — Procedural world generator
├── px4/              — Source for PX4 Autopilot
└── ros2_ws/src/drone_ml/
    ├── drone_ml/
    │   ├── teleop_keyboard.py — Manual control logic
    │   ├── follow_cam.py      — Camera tracking logic
    │   └── stream_video.py    — H.264 Video encoder for QGC
    └── setup.py               — ROS 2 package config
