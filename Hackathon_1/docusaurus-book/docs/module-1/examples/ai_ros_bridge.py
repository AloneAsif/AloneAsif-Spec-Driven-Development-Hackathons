# AI-ROS Bridge Example
# This example demonstrates a complete AI agent that interfaces with ROS 2 systems

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import random
import math

class AIROSBridge(Node):
    def __init__(self):
        super().__init__('ai_ros_bridge')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)  # 10 Hz

        # AI state variables
        self.current_position = Point()
        self.current_orientation = 0.0  # Yaw angle in radians
        self.laser_ranges = []
        self.obstacle_detected = False
        self.target_reached = False
        self.target_position = Point()
        self.target_position.x = 5.0  # Example target coordinates
        self.target_position.y = 5.0

        # Initialize AI model parameters (simplified for example)
        self.initialize_ai_model()

        self.get_logger().info('AI-ROS Bridge initialized')

    def initialize_ai_model(self):
        """Initialize the AI model parameters"""
        # For this example, we'll use a simple neural network structure
        # In a real application, you would load a pre-trained model
        self.input_size = 10  # Number of input features
        self.hidden_size = 8
        self.output_size = 2  # linear velocity, angular velocity

        # Initialize random weights (in practice, these would come from a trained model)
        self.weights_input_hidden = np.random.randn(self.input_size, self.hidden_size) * 0.5
        self.weights_hidden_output = np.random.randn(self.hidden_size, self.output_size) * 0.5

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_ranges = list(msg.ranges)

        # Check for obstacles in front of the robot (simplified)
        front_ranges = self.laser_ranges[len(self.laser_ranges)//2-10:len(self.laser_ranges)//2+10]
        min_range = min(front_ranges) if front_ranges else float('inf')

        self.obstacle_detected = min_range < 1.0  # Obstacle within 1 meter
        self.get_logger().debug(f'Obstacle detected: {self.obstacle_detected}, min range: {min_range:.2f}')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z

        # Extract yaw from quaternion (simplified - assumes no roll/pitch)
        quat = msg.pose.pose.orientation
        self.current_orientation = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                              1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

    def ai_decision_loop(self):
        """Main AI decision-making loop"""
        # Prepare input features for the AI model
        inputs = self.prepare_input_features()

        if inputs is not None:
            # Run the AI model to get movement commands
            outputs = self.run_ai_model(inputs)

            # Create and publish the movement command
            cmd_msg = Twist()
            cmd_msg.linear.x = float(outputs[0])
            cmd_msg.angular.z = float(outputs[1])

            self.cmd_publisher.publish(cmd_msg)

            # Publish status
            status_msg = String()
            status_msg.data = f'AI commanding: linear={cmd_msg.linear.x:.2f}, angular={cmd_msg.angular.z:.2f}'
            self.status_publisher.publish(status_msg)

    def prepare_input_features(self):
        """Prepare input features for the AI model"""
        if not self.laser_ranges:
            return None  # Wait for laser data

        # Feature 1-4: Distance to obstacles in different directions (front, left, right, back)
        front_idx = len(self.laser_ranges) // 2
        left_idx = len(self.laser_ranges) // 4
        right_idx = 3 * len(self.laser_ranges) // 4
        back_idx = 0

        features = [
            min(self.laser_ranges[front_idx-5:front_idx+5]) if front_idx-5 >= 0 and front_idx+5 < len(self.laser_ranges) else float('inf'),  # Front
            min(self.laser_ranges[left_idx-5:left_idx+5]) if left_idx-5 >= 0 and left_idx+5 < len(self.laser_ranges) else float('inf'),   # Left
            min(self.laser_ranges[right_idx-5:right_idx+5]) if right_idx-5 >= 0 and right_idx+5 < len(self.laser_ranges) else float('inf'), # Right
            min(self.laser_ranges[back_idx-5:back_idx+5]) if back_idx-5 >= 0 and back_idx+5 < len(self.laser_ranges) else float('inf'),   # Back
        ]

        # Feature 5-6: Current position relative to target
        dist_to_target = math.sqrt(
            (self.target_position.x - self.current_position.x)**2 +
            (self.target_position.y - self.current_position.y)**2
        )
        features.extend([dist_to_target, self.current_orientation])

        # Feature 7-10: Additional sensor features (simplified)
        features.extend([
            1.0 if self.obstacle_detected else 0.0,  # Obstacle flag
            random.random(),  # Simulated additional sensor
            random.random(),  # Simulated additional sensor
            random.random()   # Simulated additional sensor
        ])

        # Normalize features to [0, 1] range
        normalized_features = []
        for i, feature in enumerate(features):
            if i < 4:  # Distance features - normalize by max range (10m)
                normalized_features.append(min(feature, 10.0) / 10.0)
            elif i == 4:  # Distance to target - normalize by expected max (20m)
                normalized_features.append(min(feature, 20.0) / 20.0)
            elif i == 5:  # Orientation - normalize by 2*pi
                normalized_features.append((feature + math.pi) / (2 * math.pi))
            else:  # Binary or random features
                normalized_features.append(feature)

        return np.array(normalized_features)

    def run_ai_model(self, inputs):
        """Run the AI model with the given inputs"""
        # Simple feedforward neural network
        hidden = np.tanh(np.dot(inputs, self.weights_input_hidden))
        outputs = np.tanh(np.dot(hidden, self.weights_hidden_output))

        # Scale outputs to reasonable ranges
        # Linear velocity: -1.0 to 1.0 m/s
        # Angular velocity: -2.0 to 2.0 rad/s
        linear_vel = np.clip(outputs[0] * 1.0, -1.0, 1.0)
        angular_vel = np.clip(outputs[1] * 2.0, -2.0, 2.0)

        # Additional AI logic for obstacle avoidance
        if self.obstacle_detected:
            # If there's an obstacle, turn away from it
            linear_vel = 0.0  # Stop moving forward
            angular_vel = 0.5  # Turn in place to avoid obstacle

        # Additional AI logic for navigation
        if dist_to_target := math.sqrt(
            (self.target_position.x - self.current_position.x)**2 +
            (self.target_position.y - self.current_position.y)**2
        ) < 0.5:
            # Close to target, slow down
            linear_vel *= 0.3
            self.target_reached = True
        else:
            self.target_reached = False

        return [linear_vel, angular_vel]


def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIROSBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('AI-ROS Bridge interrupted by user')
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()