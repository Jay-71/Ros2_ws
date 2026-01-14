#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from countdown_action.action import Countdown


class CountdownServer(Node):

    def __init__(self):
        super().__init__('countdown_server')

        self._action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback)

        self.get_logger().info('Countdown Action Server started')

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Received goal: countdown from {goal_handle.request.seconds}')

        feedback = Countdown.Feedback()
        result = Countdown.Result()

        for i in range(goal_handle.request.seconds, 0, -1):

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                self.get_logger().info('Goal canceled')
                return result

            feedback.remaining = i
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Remaining: {i}')

            time.sleep(1)

        goal_handle.succeed()
        result.success = True
        self.get_logger().info('Countdown completed')

        return result


def main():
    rclpy.init()
    node = CountdownServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

