#!/usr/bin/env python3
"""
Minimal Python ROS 2 Node for PX4 RL/AI Integration
Subscribes to drone pose/state and publishes velocity commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class DroneRLNode(Node):
    def __init__(self):
        super().__init__('drone_rl_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State and Position subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)

        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Services for Arming and Mode switching
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = PoseStamped()

        # Timer to send high-frequency offboard setpoints (required by PX4)
        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz

        self.get_logger().info("Drone RL Node Initialized")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg
        # RL Observation update goes here
        # obs = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def timer_cb(self):
        # PX4 REQUIRES a stream of setpoints before it allows switching to OFFBOARD mode.
        cmd = TwistStamped()
        
        # ──────────────────────────────────────────────────────────
        # RL Action output goes here:
        # e.g., cmd.twist.linear.x = model.predict(obs)
        # ──────────────────────────────────────────────────────────

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.2  # Slight upward velocity to hover

        cmd.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        # Log status
        if self.current_state.connected and not self.current_state.armed:
            self.get_logger().info("Drone connected but not armed. Use standard ROS2 service calls to arm.", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = DroneRLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
