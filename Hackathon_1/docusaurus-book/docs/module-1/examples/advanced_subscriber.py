# Advanced Subscriber Example
# This example demonstrates a subscriber that handles multiple message types
# and includes advanced features like data processing and statistics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import math
from collections import deque

class AdvancedSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_subscriber')

        # Create a custom QoS profile for more control
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Multiple subscribers for different message types
        self.string_subscriber = self.create_subscription(
            String,
            'string_topic',
            self.string_callback,
            qos_profile
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            qos_profile
        )

        self.cmd_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            qos_profile
        )

        self.position_subscriber = self.create_subscription(
            Point,
            'robot_position',
            self.position_callback,
            qos_profile
        )

        # Store recent data for analysis
        self.recent_laser_readings = deque(maxlen=100)
        self.cmd_history = deque(maxlen=50)
        self.position_history = deque(maxlen=20)

        # Statistics
        self.message_count = 0
        self.last_position = Point()
        self.velocity_estimate = Twist()

        # Timers for periodic tasks
        self.stats_timer = self.create_timer(5.0, self.print_statistics)  # Every 5 seconds
        self.obstacle_timer = self.create_timer(1.0, self.check_for_obstacles)  # Every second

        self.get_logger().info('Advanced Subscriber node initialized')

    def string_callback(self, msg):
        try:
            self.get_logger().info(f'String message received: "{msg.data}"')
            self.message_count += 1
        except Exception as e:
            self.get_logger().error(f'Error in string_callback: {e}')

    def laser_callback(self, msg):
        try:
            # Process laser scan data
            valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r != float('inf')]
            if valid_ranges:
                min_range = min(valid_ranges) if valid_ranges else float('inf')
                max_range = max(valid_ranges) if valid_ranges else 0.0
                avg_range = sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0.0

                self.get_logger().info(
                    f'Laser scan: min={min_range:.2f}m, max={max_range:.2f}m, avg={avg_range:.2f}m'
                )

                # Store recent readings for analysis
                reading_data = {
                    'timestamp': self.get_clock().now(),
                    'min_range': min_range,
                    'max_range': max_range,
                    'avg_range': avg_range,
                    'num_valid_ranges': len(valid_ranges)
                }
                self.recent_laser_readings.append(reading_data)

            else:
                self.get_logger().warn('No valid ranges in laser scan')

            self.message_count += 1
        except Exception as e:
            self.get_logger().error(f'Error in laser_callback: {e}')

    def cmd_callback(self, msg):
        try:
            # Process velocity command
            self.get_logger().info(
                f'Velocity command: linear.x={msg.linear.x:.2f}, '
                f'angular.z={msg.angular.z:.2f}'
            )

            # Store command in history
            cmd_data = {
                'timestamp': self.get_clock().now(),
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            }
            self.cmd_history.append(cmd_data)

            # Estimate velocity based on recent commands
            self.update_velocity_estimate()

            self.message_count += 1
        except Exception as e:
            self.get_logger().error(f'Error in cmd_callback: {e}')

    def position_callback(self, msg):
        try:
            # Process robot position
            self.get_logger().info(
                f'Robot position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
            )

            # Store position in history
            pos_data = {
                'timestamp': self.get_clock().now(),
                'x': msg.x,
                'y': msg.y,
                'z': msg.z
            }
            self.position_history.append(pos_data)

            # Update last position for velocity calculation
            self.last_position = msg

            self.message_count += 1
        except Exception as e:
            self.get_logger().error(f'Error in position_callback: {e}')

    def update_velocity_estimate(self):
        """Estimate velocity based on recent commands"""
        if len(self.cmd_history) > 1:
            latest_cmd = self.cmd_history[-1]
            prev_cmd = self.cmd_history[-2]
            dt = (latest_cmd['timestamp'] - prev_cmd['timestamp']).nanoseconds / 1e9

            if dt > 0:
                self.velocity_estimate.linear.x = latest_cmd['linear_x']
                self.velocity_estimate.angular.z = latest_cmd['angular_z']

    def print_statistics(self):
        """Print periodic statistics"""
        stats = self.get_statistics()
        self.get_logger().info(
            f'Statistics - Total: {stats["total_messages"]}, '
            f'Laser: {stats["laser_readings_count"]}, '
            f'Cmd: {stats["cmd_history_count"]}, '
            f'Pos: {stats["position_history_count"]}'
        )

    def check_for_obstacles(self):
        """Check if there are obstacles nearby based on laser data"""
        if self.recent_laser_readings:
            latest_reading = self.recent_laser_readings[-1]
            if latest_reading['min_range'] < 1.0:  # Obstacle within 1 meter
                self.get_logger().warn(f'Obstacle detected! Distance: {latest_reading["min_range"]:.2f}m')

    def get_statistics(self):
        """Get statistics about received messages"""
        return {
            'total_messages': self.message_count,
            'laser_readings_count': len(self.recent_laser_readings),
            'cmd_history_count': len(self.cmd_history),
            'position_history_count': len(self.position_history)
        }

def main(args=None):
    rclpy.init(args=args)
    advanced_subscriber = AdvancedSubscriber()

    try:
        rclpy.spin(advanced_subscriber)
    except KeyboardInterrupt:
        stats = advanced_subscriber.get_statistics()
        print(f"Final Statistics: {stats}")
        advanced_subscriber.get_logger().info('Shutting down Advanced Subscriber')
    finally:
        advanced_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()