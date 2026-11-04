"""Thin MAVROS wrapper for the restricted local rectangle demonstration."""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from .mission import Mission, Observation


class WaypointDemo(Node):
    def __init__(self):
        super().__init__("waypoint_demo")
        altitude = self.declare_parameter("altitude", 5.0).value
        self.mission = Mission(altitude=altitude)
        self.state, self.pose, self.extended = State(), None, ExtendedState()
        self.received = {"state": -math.inf, "pose": -math.inf, "extended": -math.inf}
        self.pending, self.pending_since, self.last_request = None, 0.0, -math.inf
        self.previous_state = None
        self.start_wall = time.monotonic()
        self.previous_sim, self.last_clock = None, time.monotonic()
        self.target_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)
        self.status_pub = self.create_publisher(String, "mission/status", 10)
        self.create_subscription(State, "mavros/state", self.state_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(ExtendedState, "mavros/extended_state", self.extended_cb, qos_profile_sensor_data)
        self.arm = self.create_client(CommandBool, "mavros/cmd/arming")
        self.mode = self.create_client(SetMode, "mavros/set_mode")
        # A steady timer catches stale data even if /clock or the world stops.
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def state_cb(self, msg):
        self.state = msg
        self.received["state"] = time.monotonic()

    def pose_cb(self, msg):
        p = msg.pose.position
        if all(math.isfinite(x) for x in (p.x, p.y, p.z)):
            self.pose = msg
            self.received["pose"] = time.monotonic()

    def extended_cb(self, msg):
        self.extended = msg
        self.received["extended"] = time.monotonic()

    def request(self, operation, wall):
        if self.pending is not None:
            if not self.pending.done():
                if wall - self.pending_since > 3:
                    self.pending.cancel()
                    self.pending = None
                    self.get_logger().warning("Command response timed out")
                return
            try:
                response = self.pending.result()
                if response is None or not (getattr(response, "success", False) or getattr(response, "mode_sent", False)):
                    self.get_logger().warning("Command not accepted; awaiting state or bounded retry")
            except Exception as exc:
                self.get_logger().warning(f"Command failed: {exc}")
            self.pending = None
        if wall - self.last_request < 2:
            return
        client = self.arm if operation == "ARM" else self.mode
        if not client.service_is_ready():
            return
        request = CommandBool.Request(value=True) if operation == "ARM" else SetMode.Request(custom_mode=operation)
        self.pending = client.call_async(request)
        self.pending_since = self.last_request = wall

    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds / 1e9
        if self.previous_sim is not None and sim < self.previous_sim:
            self.mission.fail(sim, "simulation clock moved backwards; restart demo")
        if sim != self.previous_sim:
            self.last_clock = wall
        self.previous_sim = sim
        position = None
        if self.pose is not None:
            p = self.pose.pose.position
            position = (p.x, p.y, p.z)
        fresh = (wall - self.received["pose"] < 2 and wall - self.received["state"] < 3
                 and wall - self.last_clock < 2)
        observation = Observation(
            self.state.connected, self.state.armed, self.state.mode, position, fresh,
            wall - self.received["extended"] < 3 and
            self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
        if self.mission.state == "WAITING" and wall - self.start_wall > 90:
            self.mission.fail(sim, "startup telemetry timeout")
        operation = self.mission.step(sim, observation)
        if self.mission.target is not None:
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = self.pose.header.frame_id
            target.pose.position.x, target.pose.position.y, target.pose.position.z = self.mission.target
            # Preserve starting heading; this demo never assumes a map yaw.
            target.pose.orientation = self.pose.pose.orientation
            self.target_pub.publish(target)
        if operation:
            self.request(operation, wall)
        if self.mission.state != self.previous_state:
            self.get_logger().info(f"{self.mission.state}: {self.mission.reason}")
            self.previous_state = self.mission.state
        self.status_pub.publish(String(data=json.dumps({
            "state": self.mission.state, "reason": self.mission.reason,
            "waypoint": self.mission.index, "simulation_time": sim})))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
