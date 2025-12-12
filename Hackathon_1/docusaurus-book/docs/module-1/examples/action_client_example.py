# Python Action Client Example
# This example demonstrates how to create an action client in Python using rclpy

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.action import Fibonacci
import time

class ActionClientExample(Node):
    def __init__(self):
        super().__init__('action_client_example')

        # Create an action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.get_logger().info('Action client initialized')

    def send_goal(self, order):
        """Send an action goal and wait for the result"""
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal and get a future
        self.get_logger().info(f'Sending goal: order = {order}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add a callback for when the goal is accepted
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Store the future so we can wait for the result later
        self._send_goal_future = send_goal_future

    def goal_response_callback(self, future):
        """Callback for when the goal response is received"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Request the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback for receiving action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        """Callback for when the action result is received"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        self.get_logger().info('Action completed successfully!')


def main(args=None):
    rclpy.init(args=args)
    action_client = ActionClientExample()

    # Send a goal to the action server
    action_client.send_goal(10)

    try:
        # Spin to process callbacks until the action completes
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client interrupted by user')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()