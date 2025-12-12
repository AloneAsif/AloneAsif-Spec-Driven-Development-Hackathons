# Basic rclpy Node Example
# This example demonstrates the fundamental structure of a Python node using rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time

class BasicRclpyNode(Node):
    def __init__(self):
        # Initialize the parent Node class with a node name
        super().__init__('basic_rclpy_node')

        # Create a publisher for string messages
        self.publisher = self.create_publisher(
            String,
            'basic_topic',
            QoSProfile(depth=10)
        )

        # Create a publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10)
        )

        # Create a subscriber to listen for messages
        self.subscription = self.create_subscription(
            String,
            'basic_topic',
            self.message_callback,
            QoSProfile(depth=10)
        )

        # Create a timer for periodic publishing
        self.timer = self.create_timer(2.0, self.timer_callback)  # Publish every 2 seconds
        self.counter = 0

        # Log that the node has been initialized
        self.get_logger().info('Basic rclpy Node initialized')

    def timer_callback(self):
        # Create and publish a message
        msg = String()
        msg.data = f'Hello from basic node! Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

    def message_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        # Respond with a velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.2  # Move forward slowly
        cmd_msg.angular.z = 0.1  # Gentle turn
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info('Sent velocity command in response')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = BasicRclpyNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown when Ctrl+C is pressed
        node.get_logger().info('Node interrupted by user')
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()