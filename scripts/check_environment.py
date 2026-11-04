#!/usr/bin/env python3
"""Read-only environment inventory. Does not install or change packages."""
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def probe(command):
    if shutil.which(command[0]) is None:
        return {"available": False}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        return {"available": True, "exit_code": result.returncode,
                "output": (result.stdout + result.stderr).strip()}
    except subprocess.TimeoutExpired:
        return {"available": True, "error": "timeout"}


def main():
    print(json.dumps({
        "host": platform.platform(), "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros2": probe(["ros2", "doctor", "--report"]),
        "gazebo": probe(["gz", "sim", "--versions"]),
        "mavros": probe(["ros2", "pkg", "prefix", "mavros"]),
        "ros_gz_bridge": probe(["ros2", "pkg", "prefix", "ros_gz_bridge"]),
        "px4_binary_exists": (ROOT / "px4/build/px4_sitl_default/bin/px4").is_file(),
        "repository_revision": probe(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
    }, indent=2))


if __name__ == "__main__":
    main()
