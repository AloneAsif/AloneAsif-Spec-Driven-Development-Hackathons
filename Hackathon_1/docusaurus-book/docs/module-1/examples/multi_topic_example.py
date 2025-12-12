# Multi-Topic Example
# This example demonstrates a node that publishes to and subscribes from multiple topics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist
import math
import random

class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers for different topics
        self.status_publisher = self.create_publisher(String, 'system_status', 10)
        self.sensor_publisher = self.create_publisher(Float64, 'sensor_value', 10)
        self.counter_publisher = self.create_publisher(Int32, 'counter', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'robot_cmd', 10)

        # Multiple subscribers for different topics
        self.status_subscriber = self.create_subscription(
            String,
            'system_status',
            self.status_callback,
            10
        )
        self.sensor_subscriber = self.create_subscription(
            Float64,
            'sensor_value',
            self.sensor_callback,
            10
        )
        self.counter_subscriber = self.create_subscription(
            Int32,
            'counter',
            self.counter_callback,
            10
        )

        # Timer for periodic publishing
        self.publish_timer = self.create_timer(1.0, self.publish_data)

        # Counter for the example
        self.counter = 0

        self.get_logger().info('Multi-Topic Node initialized')

    def publish_data(self):
        """Publish data to multiple topics"""
        # Publish system status
        status_msg = String()
        status_msg.data = f'System operational - cycle {self.counter}'
        self.status_publisher.publish(status_msg)

        # Publish sensor value (simulated)
        sensor_msg = Float64()
        sensor_msg.data = 20.0 + 5.0 * math.sin(self.counter * 0.1) + random.uniform(-0.5, 0.5)
        self.sensor_publisher.publish(sensor_msg)

        # Publish counter
        counter_msg = Int32()
        counter_msg.data = self.counter
        self.counter_publisher.publish(counter_msg)

        # Publish robot command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.2  # Move forward slowly
        cmd_msg.angular.z = math.sin(self.counter * 0.2) * 0.3  # Gentle turns
        self.cmd_publisher.publish(cmd_msg)

        self.get_logger().info(f'Published data - cycle {self.counter}')
        self.counter += 1

    def status_callback(self, msg):
        """Handle incoming status messages"""
        self.get_logger().info(f'Received status: {msg.data}')

    def sensor_callback(self, msg):
        """Handle incoming sensor messages"""
        self.get_logger().info(f'Received sensor value: {msg.data:.2f}')

    def counter_callback(self, msg):
        """Handle incoming counter messages"""
        self.get_logger().info(f'Received counter: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    multi_topic_node = MultiTopicNode()

    try:
        rclpy.spin(multi_topic_node)
    except KeyboardInterrupt:
        multi_topic_node.get_logger().info('Shutting down Multi-Topic Node')
    finally:
        multi_topic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()