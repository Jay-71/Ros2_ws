# alert_service.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class AlertService(Node):
    def __init__(self):
        super().__init__('alert_service')
        self.srv = self.create_service(Trigger, 'reset_alerts', self.reset_callback)

    def reset_callback(self, request, response):
        # For a simple model we just respond success. If you want to reset an external
        # node counter, that node must implement a service to reset itself.
        response.success = True
        response.message = 'Alert reset acknowledged'
        self.get_logger().info('Reset request handled.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AlertService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
