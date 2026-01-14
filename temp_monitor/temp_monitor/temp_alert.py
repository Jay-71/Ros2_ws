# temp_alert.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class TempAlert(Node):
    def __init__(self):
        super().__init__('temp_alert')
        self.threshold = 50.0
        self.alert_count = 0
        self.sub = self.create_subscription(Float32, 'temp_data', self.callback, 10)
        # optional alerts topic for other nodes (TTS / logger)
        self.alert_pub = self.create_publisher(String, 'alerts', 10)

    def callback(self, msg):
        temp = msg.data
        if temp > self.threshold:
            self.alert_count += 1
            alert_text = f'ALERT: High temperature {temp:.2f} °C'
            self.get_logger().warn(alert_text)
            self.alert_pub.publish(String(data=alert_text))
        else:
            self.get_logger().info(f'Temperature OK: {temp:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = TempAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
