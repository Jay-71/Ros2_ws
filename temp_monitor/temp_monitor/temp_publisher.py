# temp_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TempPublisher(Node):
    def __init__(self):
        super().__init__('temp_publisher')
        self.pub = self.create_publisher(Float32, 'temp_data', 10)
        self.timer = self.create_timer(1.0, self.publish_temp)

    def publish_temp(self):
        temp = random.uniform(20.0, 80.0)   # simulate temp
        msg = Float32()
        msg.data = float(temp)
        self.pub.publish(msg)
        self.get_logger().info(f'Published temp: {temp:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = TempPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
