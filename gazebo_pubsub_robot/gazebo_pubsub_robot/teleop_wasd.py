import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys
import termios
import tty
import threading
import time
import math


class TeleopWASD(Node):

    def __init__(self):
        super().__init__('teleop_wasd')

        # Publisher: cmd_vel
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber: odom
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Motion state
        self.linear = 0.0
        self.angular = 0.0
        self.running = True

        # Timer for continuous publishing
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Keyboard thread
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()

        self.get_logger().info("WASD control ready")
        self.get_logger().info("W/S: forward/back | A/D: left/right | Q: quit")

    # -----------------------------
    # Keyboard handling
    # -----------------------------
    def keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running:
                key = sys.stdin.read(1)

                if key == 'w':
                    self.linear = 0.4
                elif key == 's':
                    self.linear = -0.4
                elif key == 'a':
                    self.angular = 1.2
                elif key == 'd':
                    self.angular = -1.2
                elif key == 'q':
                    self.running = False
                    break

                # short delay → simulates key hold
                time.sleep(0.05)

                # stop when key released
                self.linear = 0.0
                self.angular = 0.0

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # -----------------------------
    # Publish velocity commands
    # -----------------------------
    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    # -----------------------------
    # Odometry callback
    # -----------------------------
    def odom_callback(self, msg):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation (quaternion → yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f"ODOM | x: {x:.2f}  y: {y:.2f}  yaw: {math.degrees(yaw):.1f}°"
        )


def main():
    rclpy.init()
    node = TeleopWASD()
    rclpy.spin(node)

    node.running = False
    node.destroy_node()
    rclpy.shutdown()
