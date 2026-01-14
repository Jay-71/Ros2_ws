import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_distance)

    def publish_distance(self):
        distance = random.uniform(2.0, 400.0)  # simulate 2cm–400cm
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'📡 Published distance: {distance:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
