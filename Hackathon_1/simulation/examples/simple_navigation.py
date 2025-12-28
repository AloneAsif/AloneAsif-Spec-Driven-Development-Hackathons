#!/usr/bin/env python3

"""
Example scenario: Simple robot navigation in the digital twin environment
This script demonstrates how to control the robot and receive sensor feedback
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
import time
import math


class SimpleNavigationExample(Node):
    def __init__(self):
        super().__init__('simple_navigation_example')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Navigation state
        self.scan_data = None
        self.joint_data = None
        self.nav_state = 'FORWARD'  # FORWARD, TURN, STOP
        self.forward_distance = 0.0
        self.target_distance = 2.0  # meters

        # Robot parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.5  # meters

        # Timer for navigation logic
        self.nav_timer = self.create_timer(0.1, self.navigation_logic)

        self.get_logger().info('Simple Navigation Example started')

    def scan_callback(self, msg):
        self.scan_data = msg

    def joint_callback(self, msg):
        self.joint_data = msg

    def navigation_logic(self):
        """Main navigation logic"""
        if self.scan_data is None:
            return

        # Check if there's an obstacle ahead
        # Get the front-facing range (middle of the scan)
        front_idx = len(self.scan_data.ranges) // 2
        front_distance = self.scan_data.ranges[front_idx]

        cmd = Twist()

        if self.nav_state == 'FORWARD':
            if front_distance > self.safe_distance:
                # Continue moving forward
                cmd.linear.x = self.linear_speed
                self.forward_distance += self.linear_speed * 0.1  # Approximate distance

                if self.forward_distance >= self.target_distance:
                    # Reached target distance, stop and turn
                    self.nav_state = 'TURN'
                    self.get_logger().info('Reached target distance, turning...')
            else:
                # Obstacle detected, stop and turn
                self.nav_state = 'TURN'
                self.get_logger().info('Obstacle detected, turning...')

        elif self.nav_state == 'TURN':
            # Turn for a set duration
            cmd.angular.z = self.angular_speed
            # For this example, we'll just turn for a bit then go back to forward
            self.nav_state = 'FORWARD'
            self.forward_distance = 0.0  # Reset distance counter
            self.get_logger().info('Turning completed, going forward...')

        # Publish command
        self.cmd_pub.publish(cmd)

        # Log current state
        self.get_logger().info(
            f'Nav State: {self.nav_state}, '
            f'Front Distance: {front_distance:.2f}m, '
            f'Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}'
        )


def main():
    rclpy.init()
    nav_example = SimpleNavigationExample()

    try:
        rclpy.spin(nav_example)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before exiting
        stop_cmd = Twist()
        nav_example.cmd_pub.publish(stop_cmd)
        nav_example.get_logger().info('Navigation example stopped')
        nav_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()