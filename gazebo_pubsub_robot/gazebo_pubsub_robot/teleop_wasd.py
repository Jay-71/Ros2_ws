import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time


class TeleopWASD(Node):

    def __init__(self):
        super().__init__('teleop_wasd')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0
        self.running = True

        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()

        self.get_logger().info("WASD control ready")
        self.get_logger().info("Hold key to move | Release to stop")

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

                # stop when key released (timeout behavior)
                time.sleep(0.05)
                self.linear = 0.0
                self.angular = 0.0

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopWASD()
    rclpy.spin(node)
    node.running = False
    rclpy.shutdown()

