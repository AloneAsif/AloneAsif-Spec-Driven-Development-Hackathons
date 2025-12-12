#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class DigitalTwinBridge(Node):
    def __init__(self):
        super().__init__('digital_twin_bridge')

        self.get_logger().info('Initializing Digital Twin Bridge Node')

        # Create publishers for sensor data to Unity
        self.joint_state_pub = self.create_publisher(JointState, '/unity/joint_states', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/unity/scan', 10)
        self.image_pub = self.create_publisher(Image, '/unity/camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/unity/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/unity/odom', 10)

        # Create subscribers for commands from Unity
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/unity/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create timer for synchronization (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Digital Twin Bridge Node initialized')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel from Unity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

        # Forward command to Gazebo (in real implementation)
        cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        cmd_pub.publish(msg)

    def timer_callback(self):
        # Synchronize data between Gazebo and Unity
        self.sync_joint_states()
        self.sync_sensor_data()

    def sync_joint_states(self):
        # Create and publish joint state message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        # In a real implementation, these would come from Gazebo
        joint_msg.name = ['left_arm_joint', 'right_arm_joint', 'left_leg_joint', 'right_leg_joint']
        joint_msg.position = [0.0, 0.0, 0.0, 0.0]  # Placeholder values
        joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        joint_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self.joint_state_pub.publish(joint_msg)

    def sync_sensor_data(self):
        # Publish dummy sensor data (in real implementation, read from Gazebo)

        # Laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_link'
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = 2 * math.pi / 360  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Fill with dummy data
        scan_msg.ranges = [5.0] * 360  # 360 readings at 5m
        scan_msg.intensities = [1.0] * 360

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    digital_twin_bridge = DigitalTwinBridge()

    try:
        rclpy.spin(digital_twin_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        digital_twin_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()