# Advanced Publisher Example
# This example demonstrates a publisher that publishes multiple message types
# and includes advanced features like message validation and proper error handling

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import math
import random

class AdvancedPublisher(Node):
    def __init__(self):
        super().__init__('advanced_publisher')

        # Multiple publishers for different message types
        self.string_publisher = self.create_publisher(String, 'string_topic', 10)
        self.laser_publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.point_publisher = self.create_publisher(Point, 'robot_position', 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_callback)

        # Counter for message sequencing
        self.message_count = 0

        # Example data
        self.angle = 0.0
        self.position_x = 0.0
        self.position_y = 0.0

        self.get_logger().info('Advanced Publisher node initialized')

    def publish_callback(self):
        try:
            # Publish string message
            string_msg = String()
            string_msg.data = f'Advanced message #{self.message_count}'
            self.string_publisher.publish(string_msg)

            # Publish laser scan message
            laser_msg = self.generate_laser_scan()
            if laser_msg:  # Only publish if message is valid
                self.laser_publisher.publish(laser_msg)

            # Publish velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_msg.angular.z = math.sin(self.angle) * 0.2  # Oscillating turn
            self.cmd_publisher.publish(cmd_msg)

            # Publish robot position
            pos_msg = Point()
            pos_msg.x = self.position_x
            pos_msg.y = self.position_y
            pos_msg.z = 0.0  # Assume 2D movement
            self.point_publisher.publish(pos_msg)

            # Update position based on velocity
            dt = 0.1  # Timer period
            self.position_x += cmd_msg.linear.x * dt * math.cos(self.angle)
            self.position_y += cmd_msg.linear.x * dt * math.sin(self.angle)
            self.angle += cmd_msg.angular.z * dt

            # Reset position if it gets too far
            if abs(self.position_x) > 10.0 or abs(self.position_y) > 10.0:
                self.position_x = 0.0
                self.position_y = 0.0

            self.message_count += 1

            if self.message_count % 100 == 0:  # Log every 100 messages
                self.get_logger().info(f'Published {self.message_count} messages')

        except Exception as e:
            self.get_logger().error(f'Error in publish_callback: {e}')

    def generate_laser_scan(self):
        """Generate a simulated laser scan message with validation"""
        try:
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'laser_frame'

            msg.angle_min = -math.pi / 2  # -90 degrees
            msg.angle_max = math.pi / 2   # 90 degrees
            msg.angle_increment = math.pi / 180  # 1 degree
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.1
            msg.range_max = 10.0

            # Generate ranges (simulated with some obstacles)
            num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
            ranges = []
            for i in range(num_readings):
                angle = msg.angle_min + i * msg.angle_increment
                # Simulate some obstacles at different distances
                distance = 2.0 + 0.5 * math.sin(self.angle + i * 0.1)

                # Add some random obstacles
                if random.random() < 0.1:  # 10% chance of an obstacle
                    distance = 0.5 + random.random() * 1.5

                ranges.append(distance)

            msg.ranges = ranges
            msg.intensities = [100.0] * num_readings

            # Validate the message before returning
            if len(msg.ranges) != num_readings:
                self.get_logger().error('Generated laser scan has incorrect number of ranges')
                return None

            return msg
        except Exception as e:
            self.get_logger().error(f'Error generating laser scan: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    advanced_publisher = AdvancedPublisher()

    try:
        rclpy.spin(advanced_publisher)
    except KeyboardInterrupt:
        advanced_publisher.get_logger().info('Shutting down Advanced Publisher')
    finally:
        advanced_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()