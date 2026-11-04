# Three-drone simulation: first implementation

This increment provides portable preparation/launch tooling and a per-drone autonomous
waypoint controller. It does not yet implement the central coordinator, battery-aware
allocation, incident roles, arbitrary urban path planning, or dynamic collision avoidance.
Flight behavior must be validated on Ubuntu; only source-level and dependency-free
checks have been run so far.

## Build on Ubuntu

Run from this repository's `PALS` directory. Keep your existing Gazebo installation;
first capture its versions and verify the selected PX4 build supports it.

```bash
source /opt/ros/humble/setup.bash
python3 scripts/check_environment.py > /tmp/pals-environment.json
cat /tmp/pals-environment.json

# Build the vendored firmware using its existing dependencies/toolchain.
make -C px4 px4_sitl_default

# Requires MAVROS, mavros_msgs, ros_gz_bridge and their dependencies in this ROS environment.
# Install missing dependencies through your normal ROS setup before building.
cd ros2_ws
colcon build --symlink-install --packages-select pals_drone_controller
source install/setup.bash
cd ..
```

The original `drone_ml` tools remain available but are not required for the demo.
Do not run teleop or `rl_node` against a vehicle while its autonomous controller owns
setpoints. The new launcher directly starts MAVROS using its installed PX4 configuration,
with per-drone namespace and frame parameters. The MAVROS namespace behavior follows
its [upstream node/plugin implementation](https://github.com/mavlink/mavros/tree/ros2/mavros/src).
Check the resulting topics against your installed release before the first flight.

## Prepare and inspect

This command works without ROS and does not start a simulator:

```bash
bash launch_swarm.sh --prepare-only
```

Each run creates a new ignored `runs/<timestamp>/` directory. It contains generated model
variants and `manifest.json` with vehicle IDs, PX4 instances, MAVLink system IDs,
Gazebo entity names, ROS namespaces, spawn poses and verified-source port formulas.
The vendored model is never changed by preparation. Use `--output /path/to/new-directory`
to choose the output directory; existing directories are rejected.

The fleet is configured in `config/fleet.json`. The first demo limits pads to the
generator's empty central area, zero spawn rotation, and at least 12 m spacing.
Each route is a 4 m square relative to the vehicle's initial MAVROS local pose;
the configured altitude is relative to its initial local Z. There is no shared-map
transform or global navigation claim yet. Keep the supplied urban world and routes
for initial validation.

## Spawn three vehicles without autonomous arming

```bash
bash launch_swarm.sh
```

The launcher runs one world, waits for its clock, starts each isolated PX4 instance,
waits for its IMU topic, then starts that vehicle's MAVROS process. Each vehicle has
its own working directory/parameters/log. `Ctrl+C` stops owned process groups.
Startup failures name the log to inspect. It checks for existing worlds in its Gazebo
partition and occupied UDP ports rather than attaching to an existing project.

The default isolated transport settings are `GZ_PARTITION=pals_swarm` and
`ROS_DOMAIN_ID=42`. Use matching settings for inspection terminals:

```bash
export GZ_PARTITION=pals_swarm ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /pals/drone_1/mavros/state
ros2 topic echo /pals/drone_2/mavros/local_position/pose
```

Confirm all three state topics connect, poses belong to different models, and the
manifest's model names appear in `gz topic -l`. Ports are derived from this checkout's
`px4-rc.mavlink`: MAVROS binds 14540–14542 and PX4 binds 14580–14582. If a differently
built firmware is used, inspect its MAVLink startup before relying on those endpoints.
The legacy single-drone bridge has also been aligned to the checked-in instance-0 port.

Use `--sensors` to bridge each generated model's RGB/depth/lidar topics beneath
`/pals/drone_N/sensors/`. Sensor topics are separated at the Gazebo source, not just
remapped after mixing. Sensor optical transforms/frame metadata still require live
validation before perception use. Sensors remain in the simulated model even when
bridges are disabled. `--headless` omits the GUI but still requires a functioning
Gazebo render backend for the RGB-D camera/GPU lidar.

## Autonomous takeoff, local rectangle, return and landing

After checking telemetry and the selected firmware's Offboard-loss/landing behavior,
stop the spawn-only run and start:

```bash
bash launch_swarm.sh --fly
```

`--fly` explicitly enables autonomous Offboard entry and arming for all configured
vehicles. No keyboard piloting or `arm.sh` is required. The controller:

1. Waits for fresh connected state and finite local position; rejects already armed vehicles.
2. Streams initial position setpoints at 20 Hz for two simulation seconds.
3. Requests Offboard and arming asynchronously, with bounded retries and state confirmation.
4. Takes off, traverses the local square with arrival dwell, and returns over its start.
5. Requests `AUTO.LAND` and waits for fresh landed state plus disarming.

Read `/pals/drone_N/mission/status` for JSON state/progress/reason. A stale connection,
lost position, unexpected mode change, or timeout stops Offboard setpoints and leaves
recovery to PX4's configured flight-side behavior. It does not blindly command a return
with invalid localization. Inspect and test that fallback in SITL; this increment does
not override the firmware's safety parameters.

Mission timing uses simulation time; the wrapper uses a steady timer to detect stale
data even if simulation time stops. A pause longer than the watchdog limit faults an
active mission instead of automatically resuming it. Clock rollback also faults the
controller. Start a new run to retry; it will not automatically rearm after a fault.
`Ctrl+C` terminates the simulation, rather than performing a graceful fleet landing.

## Local checks

```bash
python3 -m unittest discover -s tests -v
python3 scripts/swarm.py --prepare-only
python3 generate_city.py --seed 42 --output /tmp/pals-urban.sdf
```

Ubuntu acceptance checks still required: verify one vehicle's frame directions and
independent command routing, complete one autonomous rectangle, repeat for three,
check inter-vehicle separation, inject rejected commands/stale telemetry/controller
termination, and confirm the configured PX4 fallback. Record real-time factor and
review logs before increasing fleet size or adding perception load.

Next implementation: central registry and normalized telemetry, static-world route
validation/reservations, energy modeling and battery eligibility, then incident dispatch.
