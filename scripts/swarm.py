#!/usr/bin/env python3
"""Prepare or supervise isolated PX4/MAVROS instances without shell interpolation."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def load_fleet(path):
    fleet = json.loads(Path(path).read_text())
    if not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_]*", fleet.get("world", "")):
        raise ValueError("world must be a simple Gazebo name")
    drones = fleet.get("drones", [])
    if not 1 <= len(drones) <= 5:
        raise ValueError("configure between one and five drones")
    ids, instances = set(), set()
    for drone in drones:
        name, instance = drone["id"], drone["instance"]
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name) or name in ids:
            raise ValueError("drone IDs must be unique ROS names")
        if type(instance) is not int or not 0 <= instance < 5 or instance in instances:
            raise ValueError("instances must be unique integers from 0 to 4")
        pose = drone["spawn"]
        if len(pose) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in pose):
            raise ValueError("spawn requires six finite xyz/rpy numbers")
        if not 2 <= drone["altitude"] <= 20:
            raise ValueError("initial flight altitude must be between 2 and 20 m")
        # This first demo relies on parallel, disjoint 4 m rectangles in the
        # generator's open central area. Arbitrary urban routes need a planner.
        if pose[3:] != [0, 0, 0] or abs(pose[0]) > 20 or abs(pose[1]) > 20:
            raise ValueError("initial demo requires zero rotation and central pads within +/-20 m")
        if not 0.2 <= pose[2] <= 1:
            raise ValueError("initial demo pads must spawn 0.2 to 1 m above ground")
        for previous in drones[:len(ids)]:
            if math.dist(pose[:2], previous["spawn"][:2]) < 12:
                raise ValueError("initial demo pads must be at least 12 m apart")
        ids.add(name)
        instances.add(instance)
    return fleet


def prepare(fleet, output):
    """Create private model variants; never edit the vendored x500."""
    output.mkdir(parents=True, exist_ok=True)
    manifest = {"world": fleet["world"], "drones": []}
    source = ROOT / "px4/Tools/simulation/gz/models/x500"
    for drone in fleet["drones"]:
        name, instance = drone["id"], drone["instance"]
        model = "pals_" + name
        directory = output / "models" / model
        directory.mkdir(parents=True, exist_ok=True)
        tree = ET.parse(source / "model.sdf")
        tree.getroot().find("model").set("name", model)
        for sensor in tree.findall(".//sensor"):
            topic = sensor.find("topic")
            if topic is not None:
                topic.text = f"/pals/{name}/sensors/{sensor.get('name')}"
        tree.write(directory / "model.sdf", encoding="unicode", xml_declaration=True)
        config = ET.Element("model")
        ET.SubElement(config, "name").text = model
        ET.SubElement(config, "version").text = "1.0"
        ET.SubElement(config, "sdf", version="1.9").text = "model.sdf"
        ET.ElementTree(config).write(directory / "model.config", encoding="unicode")
        manifest["drones"].append(dict(
            drone, model=model, entity=f"{model}_{instance}", system_id=instance + 1,
            namespace=f"/pals/{name}",
            fcu_url=f"udp://:{14540 + instance}@127.0.0.1:{14580 + instance}"))
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


class Supervisor:
    def __init__(self, output, env):
        self.output, self.env, self.children = output, env, []

    def start(self, name, args, cwd=None, env=None):
        log = (self.output / f"{name}.log").open("w")
        try:
            process = subprocess.Popen(args, cwd=cwd, env=env or self.env,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        finally:
            log.close()
        self.children.append((name, process))
        print(f"Started {name}: PID {process.pid}", flush=True)

    def check(self):
        for name, process in self.children:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited ({process.returncode}); see {self.output / (name + '.log')}")

    def close(self):
        # Signal each owned process group, including descendants of launch tools.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for _, process in reversed(self.children):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                if all(p.poll() is not None for _, p in self.children):
                    break
                time.sleep(0.1)
        for _, process in self.children:
            process.wait()


def wait_topic(supervisor, topic, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        supervisor.check()
        try:
            result = subprocess.run(["gz", "topic", "-l"], env=supervisor.env,
                                    capture_output=True, text=True, timeout=5)
            if topic in result.stdout.splitlines():
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    raise RuntimeError(f"Timed out waiting for {topic}")


def run(args, manifest, output):
    for command in ("gz", "ros2"):
        if not shutil.which(command):
            raise RuntimeError(f"Missing {command}; source the Ubuntu ROS environment first")
    build = Path(args.px4_build).resolve()
    binary = build / "bin/px4"
    if not binary.is_file() or not (build / "etc").is_dir():
        raise RuntimeError(f"Build PX4 first: expected {binary} and {build / 'etc'}")
    world = ROOT / f"px4/Tools/simulation/gz/worlds/{manifest['world']}.sdf"
    if not world.is_file():
        raise RuntimeError(f"Missing world: {world}")
    env = os.environ.copy()
    env.update(GZ_PARTITION=args.partition, ROS_DOMAIN_ID=str(args.domain_id),
               GZ_SIM_RESOURCE_PATH=os.pathsep.join([
                   str(output / "models"), str(ROOT / "px4/Tools/simulation/gz/models"),
                   env.get("GZ_SIM_RESOURCE_PATH", "")]))
    # Fail instead of joining an unrelated world or sharing an active UDP bind.
    existing = subprocess.run(["gz", "topic", "-l"], env=env, capture_output=True,
                              text=True, timeout=5)
    if any(topic.startswith("/world/") for topic in existing.stdout.splitlines()):
        raise RuntimeError("Gazebo partition already has a world; use a different --partition")
    for drone in manifest["drones"]:
        for base in (14540, 14580, 18570):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(("0.0.0.0", base + drone["instance"]))
    mavros_prefix = subprocess.run(["ros2", "pkg", "prefix", "mavros"], env=env,
                                   capture_output=True, text=True, timeout=10, check=True).stdout.strip()
    mavros_share = Path(mavros_prefix) / "share/mavros/launch"
    for filename in ("px4_config.yaml", "px4_pluginlists.yaml"):
        if not (mavros_share / filename).is_file():
            raise RuntimeError(f"Missing MAVROS configuration: {mavros_share / filename}")
    (output / "environment.json").write_text(json.dumps({
        "GZ_PARTITION": args.partition, "ROS_DOMAIN_ID": args.domain_id,
        "px4_binary": str(binary), "world": str(world), "flight_enabled": args.fly,
    }, indent=2) + "\n")
    supervisor = Supervisor(output, env)
    try:
        supervisor.start("gazebo", ["gz", "sim", "-r", "-s", str(world)])
        wait_topic(supervisor, f"/world/{manifest['world']}/clock")
        if not args.headless:
            supervisor.start("gazebo_gui", ["gz", "sim", "-g"])
        clock_topic = f"/world/{manifest['world']}/clock"
        supervisor.start("clock", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                  f"{clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                                  "--ros-args", "-r", f"{clock_topic}:=/clock"])
        for drone in manifest["drones"]:
            name, ns = drone["id"], drone["namespace"]
            work = output / name
            work.mkdir(exist_ok=True)
            px4_env = env.copy()
            # Do not inherit single-drone spawn/binding overrides.
            for key in ("PX4_GZ_MODEL_NAME", "PX4_SIM_MODEL", "PX4_GZ_MODEL_POSE"):
                px4_env.pop(key, None)
            px4_env.update(PX4_SYS_AUTOSTART="4001", PX4_SIMULATOR="gz",
                           PX4_GZ_WORLD=manifest["world"], PX4_GZ_MODEL=drone["model"],
                           PX4_GZ_MODEL_POSE=",".join(map(str, drone["spawn"])), HEADLESS="1")
            supervisor.start(name + "_px4", [str(binary), str(build / "etc"), "-i", str(drone["instance"]), "-d"],
                             cwd=work, env=px4_env)
            wait_topic(supervisor, f"/world/{manifest['world']}/model/{drone['entity']}/link/base_link/sensor/imu_sensor/imu")
            supervisor.start(name + "_mavros", [
                "ros2", "run", "mavros", "mavros_node", "--ros-args",
                "--params-file", str(mavros_share / "px4_pluginlists.yaml"),
                "--params-file", str(mavros_share / "px4_config.yaml"),
                "-r", f"__ns:={ns}/mavros",
                "-p", f"fcu_url:={drone['fcu_url']}",
                "-p", f"tgt_system:={drone['system_id']}", "-p", "tgt_component:=1",
                "-p", f"base_link_frame:={name}/base_link",
                "-p", f"base_link_frame_id:={name}/base_link",
                "-p", f"map_frame:={name}/local_enu", "-p", f"map_frame_id:={name}/local_enu",
                "-p", f"odom_frame:={name}/odom", "-p", f"odom_frame_id:={name}/odom"])
            if args.sensors:
                base = f"{ns}/sensors"
                supervisor.start(name + "_sensors", [
                    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    f"{base}/camera@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{base}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{base}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    f"{base}/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"])
            if args.fly:
                supervisor.start(name + "_controller", [
                    "ros2", "run", "pals_drone_controller", "waypoint_demo", "--ros-args",
                    "-r", f"__ns:={ns}", "-p", "use_sim_time:=true",
                    "-p", f"altitude:={drone['altitude']}"])
        print(f"Fleet running. Logs: {output}. Ctrl+C stops owned processes.", flush=True)
        while True:
            supervisor.check()
            time.sleep(0.5)
    finally:
        supervisor.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/fleet.json")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--px4-build", type=Path, default=ROOT / "px4/build/px4_sitl_default")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--sensors", action="store_true", help="bridge RGB/depth/lidar (rendering still needed)")
    parser.add_argument("--fly", action="store_true", help="autonomously arm and fly disjoint local rectangles")
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--partition", default="pals_swarm")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("domain-id must be between 0 and 232")
    output = (args.output or ROOT / "runs" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    try:
        fleet = load_fleet(args.config)
        # Avoid concurrent launches and accidental replacement of prior evidence.
        output.mkdir(parents=True, exist_ok=False)
        manifest = prepare(fleet, output)
        print(f"Manifest: {output / 'manifest.json'}", flush=True)
        if not args.prepare_only:
            run(args, manifest, output)
    except KeyboardInterrupt:
        return 0
    except (ValueError, KeyError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
