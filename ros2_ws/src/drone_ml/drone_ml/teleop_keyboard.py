#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool
import sys, select, termios, tty

msg = """
Control Your Drone!
---------------------------
Moving around:
   w
a  s  d

w/s : Forward/Backward
a/d : Left/Right

Up/Down:
   i
   k

i/k : Up/Down

Turn (Yaw):
   j
   l

j/l : Turn Left / Turn Right

v : Toggle Follow Camera

SPACE : Hover (Stop moving)
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'i': (0, 0, 1, 0),
    'k': (0, 0, -1, 0),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.camera_toggle_pub = self.create_publisher(Bool, '/camera/follow_toggle', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.current_state = State()
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.speed = 1.0  # m/s
        self.turn = 1.0   # rad/s
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        
        # High frequency timer for Offboard mode (required to be > 2Hz)
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.follow_on = True # Initial follow state
        self.get_logger().info("Teleop Node Initialized. Streaming setpoints...")

    def state_cb(self, msg):
        self.current_state = msg

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], None)  # block until key pressed
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_input(self):
        try:
            while rclpy.ok():
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key == ' ':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'v':
                    self.follow_on = not self.follow_on
                    bool_msg = Bool()
                    bool_msg.data = self.follow_on
                    self.camera_toggle_pub.publish(bool_msg)
                    self.get_logger().info(f"Keyboard Trigger: Follow Camera {'ON' if self.follow_on else 'OFF'}")
                else:
                    if (key == '\x03'): # CTRL-C
                        break
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def timer_cb(self):
        cmd = TwistStamped()
        cmd.twist.linear.x = self.x * self.speed
        cmd.twist.linear.y = self.y * self.speed
        cmd.twist.linear.z = self.z * self.speed
        cmd.twist.angular.z = self.th * self.turn
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    print(msg)
    print("Press SPACE to stop, CTRL-C to quit.")
    print("\n[NOTE] Open Terminal 4 to ARM and set OFFBOARD mode!")
    
    node.get_input()
    
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
