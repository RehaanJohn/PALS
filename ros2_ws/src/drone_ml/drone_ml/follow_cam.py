#!/usr/bin/env python3
"""
follow_cam.py — Follow Camera for Gazebo Harmonic
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

The camera follow is configured in urban.sdf (GzScene3D > camera_follow)
with p_gain=20.0 (near-instant tracking) and starts automatically.

This node just:
  1. Publishes /gui/follow every 0.5s to keep the lock active (it can be
     released if the user drags the mouse in Gazebo)
  2. Publishes /gui/follow/offset (closer than the default)
  3. Listens for 'v' toggle from Teleop terminal

NOTE: Do NOT use the right-click "Follow" in the Gazebo entity tree —
      that uses Gazebo's slow default p_gain instead of the SDF one.
      Just let this node handle it automatically.
"""

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gz.transport13 import Node as GzNode
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2  import Vector3d

# ─── Camera position relative to drone ───────────────────────────────────────
MODEL_NAME = 'x500_0'   # PX4 SITL model name in Gazebo (do not change)
BACK_DIST  = -2.5       # metres behind drone  (must match urban.sdf offset x)
SIDE_DIST  =  0.0       # metres left/right
UP_OFFSET  =  1.0       # metres above drone   (must match urban.sdf offset z)
RETRY_SECS =  0.5       # re-publish every 0.5s to fight mouse-drag unlock
# ─────────────────────────────────────────────────────────────────────────────

# IMPORTANT: The p_gain controlling how snappy the camera moves is set in:
#   ~/drone_sim/px4/Tools/simulation/gz/worlds/urban.sdf
#   Look for <camera_follow><p_gain>20.0</p_gain>
#   Higher = snappier (20 = near-instant), lower = smoother but more lag


class FollowCamNode(Node):
    def __init__(self):
        super().__init__('follow_cam')

        self._gz = GzNode()
        self._follow_pub = self._gz.advertise('/gui/follow', StringMsg)
        self._offset_pub = self._gz.advertise('/gui/follow/offset', Vector3d)

        # ROS2: toggle from teleop 'v' key
        self.create_subscription(
            Bool, '/camera/follow_toggle', self._toggle_cb, 10)

        self._follow_on = True

        # Background thread republishes to snap camera back if mouse is dragged
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"FollowCam started — targeting '{MODEL_NAME}' "
            f"({abs(BACK_DIST):.1f}m behind, {UP_OFFSET:.1f}m above).\n"
            "  Camera follow uses p_gain=20.0 from urban.sdf (near-instant).\n"
            "  DO NOT use right-click 'Follow' in Gazebo — it resets p_gain!\n"
            "  Press 'v' in Teleop terminal to toggle follow on/off."
        )

    def _toggle_cb(self, msg: Bool):
        self._follow_on = msg.data
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"Follow camera toggled → {state}")
        self._do_publish()

    def _do_publish(self):
        sm = StringMsg()
        if self._follow_on:
            sm.data = MODEL_NAME
            self._follow_pub.publish(sm)

            v = Vector3d()
            v.x = BACK_DIST
            v.y = SIDE_DIST
            v.z = UP_OFFSET
            self._offset_pub.publish(v)
        else:
            sm.data = ''   # empty = release follow
            self._follow_pub.publish(sm)

    def _worker(self):
        time.sleep(4.0)   # wait for Gazebo GUI to be fully ready
        self.get_logger().info("Sending follow command to Gazebo GUI...")
        while rclpy.ok():
            self._do_publish()
            time.sleep(RETRY_SECS)


def main(args=None):
    rclpy.init(args=args)
    node = FollowCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
