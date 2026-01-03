import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd)
        self.forward = True

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.3 if self.forward else -0.3
        self.forward = not self.forward
        self.get_logger().info('Publishing cmd_vel')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

