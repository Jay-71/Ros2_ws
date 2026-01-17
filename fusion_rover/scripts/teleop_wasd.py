#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
       # Change this line in your script's __init__:
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.get_logger().info("WASD Teleop Started")
        self.get_logger().info("Use W/A/S/D to move, Space to stop, CTRL+C to quit")

        self.linear_speed = 0.4
        self.angular_speed = 1.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()

                # Reset velocities each loop
                twist.linear.x = 0.0
                twist.angular.z = 0.0

                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == ' ': # Added space for emergency stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':  # CTRL+C
                    break

                self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # Send stop command before exiting
            stop_twist = Twist()
            self.publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    
    # Run the movement loop
    node.run()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()