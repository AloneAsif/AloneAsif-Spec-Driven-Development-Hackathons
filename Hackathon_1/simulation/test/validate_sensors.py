#!/usr/bin/env python3

"""
Sensor validation script for digital twin
Validates that all sensors publish correct ROS 2 topics with expected frequencies
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, JointState
from std_msgs.msg import Header
import time
from collections import deque


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Variables to track sensor data
        self.scan_data = deque(maxlen=10)  # Keep last 10 readings
        self.image_data = deque(maxlen=10)
        self.imu_data = deque(maxlen=10)
        self.joint_data = deque(maxlen=10)

        # Expected frequencies (Hz)
        self.expected_freq = {
            'scan': 10,
            'image': 30,
            'imu': 100,
            'joint_states': 50
        }

        # Create subscribers for all sensors
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Timer for validation checks
        self.timer = self.create_timer(5.0, self.validate_sensors)  # Check every 5 seconds

        self.get_logger().info('Sensor Validator initialized')

    def scan_callback(self, msg):
        self.scan_data.append({
            'timestamp': self.get_clock().now(),
            'range_count': len(msg.ranges),
            'min_range': min(msg.ranges) if msg.ranges else 0,
            'max_range': max(msg.ranges) if msg.ranges else 0
        })

    def image_callback(self, msg):
        self.image_data.append({
            'timestamp': self.get_clock().now(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding
        })

    def imu_callback(self, msg):
        self.imu_data.append({
            'timestamp': self.get_clock().now(),
            'orientation': msg.orientation,
            'angular_velocity': msg.angular_velocity,
            'linear_acceleration': msg.linear_acceleration
        })

    def joint_callback(self, msg):
        self.joint_data.append({
            'timestamp': self.get_clock().now(),
            'joint_count': len(msg.name),
            'joint_names': msg.name
        })

    def validate_sensors(self):
        """Validate that sensors are publishing at expected frequencies"""
        self.get_logger().info('Validating sensors...')

        # Check scan frequency
        if len(self.scan_data) >= 2:
            time_diff = (self.scan_data[-1]['timestamp'] - self.scan_data[0]['timestamp']).nanoseconds / 1e9
            freq = len(self.scan_data) / time_diff if time_diff > 0 else 0
            expected = self.expected_freq['scan']
            if abs(freq - expected) / expected < 0.2:  # Allow 20% tolerance
                self.get_logger().info(f'✓ LiDAR frequency: {freq:.2f} Hz (expected ~{expected} Hz)')
            else:
                self.get_logger().warn(f'⚠ LiDAR frequency: {freq:.2f} Hz (expected ~{expected} Hz)')

        # Check image frequency
        if len(self.image_data) >= 2:
            time_diff = (self.image_data[-1]['timestamp'] - self.image_data[0]['timestamp']).nanoseconds / 1e9
            freq = len(self.image_data) / time_diff if time_diff > 0 else 0
            expected = self.expected_freq['image']
            if abs(freq - expected) / expected < 0.2:  # Allow 20% tolerance
                self.get_logger().info(f'✓ Camera frequency: {freq:.2f} Hz (expected ~{expected} Hz)')
            else:
                self.get_logger().warn(f'⚠ Camera frequency: {freq:.2f} Hz (expected ~{expected} Hz)')

        # Check IMU frequency
        if len(self.imu_data) >= 2:
            time_diff = (self.imu_data[-1]['timestamp'] - self.imu_data[0]['timestamp']).nanoseconds / 1e9
            freq = len(self.imu_data) / time_diff if time_diff > 0 else 0
            expected = self.expected_freq['imu']
            if abs(freq - expected) / expected < 0.2:  # Allow 20% tolerance
                self.get_logger().info(f'✓ IMU frequency: {freq:.2f} Hz (expected ~{expected} Hz)')
            else:
                self.get_logger().warn(f'⚠ IMU frequency: {freq:.2f} Hz (expected ~{expected} Hz)')

        # Check joint states frequency
        if len(self.joint_data) >= 2:
            time_diff = (self.joint_data[-1]['timestamp'] - self.joint_data[0]['timestamp']).nanoseconds / 1e9
            freq = len(self.joint_data) / time_diff if time_diff > 0 else 0
            expected = self.expected_freq['joint_states']
            if abs(freq - expected) / expected < 0.2:  # Allow 20% tolerance
                self.get_logger().info(f'✓ Joint States frequency: {freq:.2f} Hz (expected ~{expected} Hz)')
            else:
                self.get_logger().warn(f'⚠ Joint States frequency: {freq:.2f} Hz (expected ~{expected} Hz)')

        # Validate data content
        self.validate_data_content()

    def validate_data_content(self):
        """Validate that sensor data has expected content"""
        if self.scan_data:
            scan = self.scan_data[-1]
            if scan['min_range'] >= 0 and scan['max_range'] <= 10.0:  # Based on URDF config
                self.get_logger().info('✓ LiDAR range values are valid')
            else:
                self.get_logger().error(f'✗ LiDAR range values invalid: min={scan["min_range"]}, max={scan["max_range"]}')

        if self.image_data:
            image = self.image_data[-1]
            if image['width'] == 640 and image['height'] == 480:  # Based on URDF config
                self.get_logger().info('✓ Camera resolution is correct')
            else:
                self.get_logger().error(f'✗ Camera resolution invalid: {image["width"]}x{image["height"]}')

        if self.joint_data:
            joint = self.joint_data[-1]
            expected_joints = {'left_arm_joint', 'right_arm_joint', 'left_leg_joint', 'right_leg_joint'}
            actual_joints = set(joint['joint_names'])
            if expected_joints.issubset(actual_joints):
                self.get_logger().info('✓ Joint names are correct')
            else:
                missing = expected_joints - actual_joints
                self.get_logger().error(f'✗ Missing joints: {missing}')


def main():
    rclpy.init()
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()