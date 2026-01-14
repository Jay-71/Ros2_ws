#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from countdown_action.action import Countdown


class CountdownClient(Node):

    def __init__(self):
        super().__init__('countdown_client')
        self.client = ActionClient(self, Countdown, 'countdown')

    def send_goal(self, seconds):
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        goal_msg = Countdown.Goal()
        goal_msg.seconds = seconds

        self.get_logger().info(f'Sending goal: {seconds}')

        self._send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Feedback: remaining = {feedback_msg.feedback.remaining}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success = {result.success}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CountdownClient()
    node.send_goal(5)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
