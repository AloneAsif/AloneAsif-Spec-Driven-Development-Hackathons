#!/usr/bin/env python3

"""
Test script for digital twin functionality
This script tests basic functionality of the digital twin system
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import threading


class DigitalTwinTester(Node):
    def __init__(self):
        super().__init__('digital_twin_tester')

        # Variables to track test results
        self.joint_states_received = False
        self.scan_received = False
        self.test_passed = True

        # Create subscribers to test data flow
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher for commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Digital Twin Tester initialized')

    def joint_state_callback(self, msg):
        if len(msg.name) > 0:
            self.joint_states_received = True
            self.get_logger().info(f'Received joint states: {msg.name}')

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            self.scan_received = True
            self.get_logger().info(f'Received scan data: {len(msg.ranges)} readings')

    def test_basic_functionality(self):
        """Test basic functionality of the digital twin"""
        self.get_logger().info('Starting basic functionality test...')

        # Wait a bit for data
        time.sleep(2.0)

        # Check if we received sensor data
        if not self.joint_states_received:
            self.get_logger().error('No joint states received!')
            self.test_passed = False
        else:
            self.get_logger().info('✓ Joint states received')

        if not self.scan_received:
            self.get_logger().error('No scan data received!')
            self.test_passed = False
        else:
            self.get_logger().info('✓ Scan data received')

        # Send a simple command to test actuation
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = 0.2  # Turn slightly
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Sent command: linear=0.5, angular=0.2')

        # Wait to see if command was processed
        time.sleep(1.0)

        if self.test_passed:
            self.get_logger().info('✓ All tests passed!')
            return True
        else:
            self.get_logger().error('✗ Some tests failed!')
            return False


def main():
    rclpy.init()

    tester = DigitalTwinTester()

    # Run test in a separate thread to allow ROS to spin
    def run_test():
        time.sleep(0.5)  # Give subscribers time to connect
        result = tester.test_basic_functionality()
        print(f"Test result: {'PASS' if result else 'FAIL'}")

    test_thread = threading.Thread(target=run_test)
    test_thread.start()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        test_thread.join(timeout=1.0)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()