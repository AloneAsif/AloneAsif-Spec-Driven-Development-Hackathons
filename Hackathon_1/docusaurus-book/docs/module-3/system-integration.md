# System Integration: Perception → Localization → Planning → Control

## Overview

This chapter covers the complete integration of the Isaac ecosystem for humanoid robot navigation. Students will learn to connect Isaac Sim simulation with Isaac ROS perception and Nav2 navigation, creating a complete pipeline from sensor data to navigation commands.

## Integration Architecture

### Complete System Flow

The complete navigation pipeline connects all three components:

```
Isaac Sim Sensors (RGB, Depth, IMU) → Isaac ROS (VSLAM, Localization) → Nav2 (Path Planning) → Control Commands
```

### Data Flow Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Isaac Sim     │    │   Isaac ROS      │    │    Nav2         │    │   Robot         │
│   (Sensors)     │───▶│   (Localization) │───▶│   (Planning)    │───▶│   (Control)     │
│                 │    │                  │    │                 │    │                 │
│ • RGB Cameras   │    │ • Visual SLAM    │    │ • Global Plan   │    │ • Velocity      │
│ • Depth Sensors │    │ • IMU Fusion     │    │ • Local Plan    │    │ • Joint Commands│
│ • IMU           │    │ • Pose Estimation│    │ • Path Tracking │    │ • Balance Ctrl  │
└─────────────────┘    └──────────────────┘    └─────────────────┘    └─────────────────┘
```

## Isaac Sim ↔ ROS 2 Bridge Configuration

### Setting Up the Bridge

The Isaac Sim ROS 2 bridge enables communication between simulation and ROS 2:

```python
# isaac_sim_ros_bridge.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
import carb

def setup_isaac_sim_ros_bridge():
    """
    Set up Isaac Sim to ROS 2 bridge for simulation
    """
    # Enable ROS bridge extension
    enable_extension("omni.isaac.ros2_bridge")

    print("Isaac Sim ROS 2 Bridge enabled")

def configure_bridge_parameters():
    """
    Configure bridge parameters for optimal performance
    """
    # Get the bridge extension
    from omni.isaac.ros2_bridge import _ros2_bridge
    ros2_bridge = _ros2_bridge.acquire_ros2_bridge_interface()

    # Set bridge parameters
    bridge_config = {
        "namespace": "isaac_sim",
        "qos": {
            "sensor_qos": {
                "history": "keep_last",
                "depth": 1,
                "reliability": "best_effort",
                "durability": "volatile"
            },
            "default_qos": {
                "history": "keep_last",
                "depth": 10,
                "reliability": "reliable",
                "durability": "volatile"
            }
        },
        "synchronization": {
            "use_sim_time": True,
            "time_scale": 1.0,
            "publish_frequency": 60.0
        }
    }

    print("Bridge parameters configured")
    return bridge_config

def create_sensor_bridge_config():
    """
    Create specific bridge configuration for sensors
    """
    sensor_bridge_config = {
        # RGB camera bridge
        "rgb_camera": {
            "topic_name": "/camera/rgb/image_raw",
            "modality": "rgb",
            "qos_profile": "sensor_qos",
            "enabled": True
        },
        # Depth camera bridge
        "depth_camera": {
            "topic_name": "/camera/depth/image_raw",
            "modality": "depth",
            "qos_profile": "sensor_qos",
            "enabled": True
        },
        # IMU bridge
        "imu": {
            "topic_name": "/imu/data",
            "modality": "imu",
            "qos_profile": "sensor_qos",
            "enabled": True
        },
        # Ground truth pose (for validation)
        "ground_truth_pose": {
            "topic_name": "/ground_truth/pose",
            "modality": "pose",
            "qos_profile": "default_qos",
            "enabled": True
        }
    }

    print("Sensor bridge configuration created")
    return sensor_bridge_config

# Initialize bridge
setup_isaac_sim_ros_bridge()
bridge_params = configure_bridge_parameters()
sensor_config = create_sensor_bridge_config()
```

### Bridge Launch Configuration

```yaml
# bridge_config.yaml
/**:
  ros__parameters:
    # Isaac Sim Bridge parameters
    isaac_sim_bridge:
      namespace: "isaac_sim"
      use_sim_time: true
      time_scale: 1.0
      publish_frequency: 60.0

    # Sensor bridge configurations
    rgb_camera_bridge:
      topic_name: "/camera/rgb/image_raw"
      qos_history: "keep_last"
      qos_depth: 1
      qos_reliability: "best_effort"
      qos_durability: "volatile"

    depth_camera_bridge:
      topic_name: "/camera/depth/image_raw"
      qos_history: "keep_last"
      qos_depth: 1
      qos_reliability: "best_effort"
      qos_durability: "volatile"

    imu_bridge:
      topic_name: "/imu/data"
      qos_history: "keep_last"
      qos_depth: 10
      qos_reliability: "reliable"
      qos_durability: "volatile"
```

## Isaac ROS ↔ Nav2 Data Flow Integration

### Connecting Isaac ROS Localization to Nav2

The connection between Isaac ROS localization and Nav2 navigation is crucial for the complete pipeline:

```python
# isaac_ros_nav2_connector.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np

class IsaacRosNav2Connector(Node):
    def __init__(self):
        super().__init__('isaac_ros_nav2_connector')

        # Subscribe to Isaac ROS localization
        self.localization_sub = self.create_subscription(
            Odometry,  # Isaac ROS VSLAM typically publishes Odometry
            '/visual_slam/visual_odometry',
            self.localization_callback,
            10
        )

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for Nav2 initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # TF broadcaster for transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store latest pose and covariance
        self.latest_pose = None
        self.latest_covariance = None

        # Timer for broadcasting transforms
        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10 Hz

        self.get_logger().info('Isaac ROS to Nav2 connector initialized')

    def localization_callback(self, msg):
        """Process Isaac ROS localization data"""
        self.latest_pose = msg.pose.pose
        self.latest_covariance = msg.pose.covariance

        # Prepare initial pose for Nav2 if not set
        if self.should_set_initial_pose():
            self.publish_initial_pose(msg.pose)

    def imu_callback(self, msg):
        """Process IMU data for enhanced localization"""
        # Use IMU data to improve pose estimation
        self.process_imu_data(msg)

    def should_set_initial_pose(self):
        """Determine if initial pose should be set"""
        # Check if Nav2 has received initial pose
        # This is a simplified check - in practice, you'd check Nav2 state
        return self.latest_pose is not None

    def publish_initial_pose(self, pose_with_cov):
        """Publish initial pose to Nav2"""
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.pose = pose_with_cov

        self.initial_pose_pub.publish(initial_pose_msg)
        self.get_logger().info('Published initial pose to Nav2')

    def broadcast_transforms(self):
        """Broadcast TF transforms from Isaac ROS localization"""
        if self.latest_pose:
            # Create transform from map to odom
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = 'map'
            transform_msg.child_frame_id = 'odom'

            # Use the pose from Isaac ROS
            transform_msg.transform.translation.x = self.latest_pose.position.x
            transform_msg.transform.translation.y = self.latest_pose.position.y
            transform_msg.transform.translation.z = self.latest_pose.position.z

            transform_msg.transform.rotation.x = self.latest_pose.orientation.x
            transform_msg.transform.rotation.y = self.latest_pose.orientation.y
            transform_msg.transform.rotation.z = self.latest_pose.orientation.z
            transform_msg.transform.rotation.w = self.latest_pose.orientation.w

            self.tf_broadcaster.sendTransform(transform_msg)

    def process_imu_data(self, imu_msg):
        """Process IMU data to enhance localization"""
        # Use IMU data for better orientation estimation
        # This would integrate with the pose from Isaac ROS
        pass

def main(args=None):
    rclpy.init(args=args)
    connector = IsaacRosNav2Connector()
    rclpy.spin(connector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration Launch File

```xml
<!-- isaac_ros_nav2_integration.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.descriptions import ParameterFile
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file = LaunchConfiguration('params_file')

    # Isaac ROS Visual SLAM node
    visual_slam_node = LifecycleNode(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_occupancy_grid': True,
            'enable_slam_visualization': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }],
        remappings=[
            ('/visual_slam/corrected_odom', '/odom'),
            ('/visual_slam/tracking/feature_tracks', '/feature_tracks'),
            ('/visual_slam/tracking/gyroscope', '/imu/data'),
        ],
        output='screen'
    )

    # Isaac ROS to Nav2 connector
    connector_node = Node(
        package='isaac_ros_nav2_examples',
        executable='isaac_ros_nav2_connector',
        name='isaac_ros_nav2_connector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Nav2 Navigation Server
    navigation_server = Node(
        package='nav2_navigation',
        executable='navigation_server',
        name='navigation_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Behavior Tree Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': True},
                   {'node_names': ['controller_server',
                                 'planner_server',
                                 'bt_navigator',
                                 'velocity_smoother',
                                 'pose_smoother',
                                 'local_costmap',
                                 'global_costmap']}],
        output='screen'
    )

    # RViz for visualization
    rviz_config_file = os.path.join(
        get_package_share_directory('isaac_ros_nav2_examples'),
        'rviz',
        'isaac_ros_nav2_config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        visual_slam_node,
        connector_node,
        navigation_server,
        controller_server,
        planner_server,
        bt_navigator,
        lifecycle_manager,
        rviz_node
    ])
```

## Full Navigation Loop Implementation

### Complete Integration Pipeline

```python
# complete_navigation_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CompleteNavigationPipeline(Node):
    def __init__(self):
        super().__init__('complete_navigation_pipeline')

        # Subscriptions for Isaac Sim sensors
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/visual_slam/visual_odometry', self.odom_callback, 10)

        # Publishers for navigation commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Navigation state
        self.current_pose = None
        self.localization_quality = 0.0
        self.navigation_active = False

        # Timer for main navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz

        self.get_logger().info('Complete navigation pipeline initialized')

    def rgb_callback(self, msg):
        """Process RGB camera data"""
        # RGB data processing (for visualization or additional perception)
        pass

    def depth_callback(self, msg):
        """Process depth camera data"""
        # Depth data processing (for obstacle detection)
        pass

    def imu_callback(self, msg):
        """Process IMU data for stability"""
        # IMU data processing for balance and orientation
        pass

    def odom_callback(self, msg):
        """Update current pose from Isaac ROS localization"""
        self.current_pose = msg.pose.pose
        self.localization_quality = self.calculate_localization_quality(msg)

    def calculate_localization_quality(self, odom_msg):
        """Calculate localization quality from covariance"""
        covariance = odom_msg.pose.covariance
        # Calculate quality based on covariance diagonal elements
        position_uncertainty = np.sqrt(covariance[0] + covariance[7] + covariance[14])
        return 1.0 / (1.0 + position_uncertainty)  # Quality between 0 and 1

    def navigation_loop(self):
        """Main navigation loop: Perception → Localization → Planning → Control"""
        if not self.current_pose:
            self.get_logger().warn('No current pose available')
            return

        # Check localization quality
        if self.localization_quality < 0.5:
            self.get_logger().warn(f'Low localization quality: {self.localization_quality:.2f}')
            # Could trigger relocalization or pause navigation
            return

        # Step 1: Perception (process sensor data)
        obstacles_detected = self.perceive_environment()

        # Step 2: Localization (use Isaac ROS pose)
        current_location = self.get_current_location()

        # Step 3: Planning (if navigation is active)
        if self.navigation_active:
            navigation_cmd = self.plan_navigation(obstacles_detected, current_location)

            # Step 4: Control (send velocity commands)
            self.execute_navigation(navigation_cmd)

    def perceive_environment(self):
        """Perceive environment using available sensors"""
        # Process sensor data to detect obstacles
        # This would integrate RGB, depth, and other sensor data
        obstacles = {
            'front': False,
            'left': False,
            'right': False,
            'distance': 1.0  # meters
        }

        # Placeholder for actual perception logic
        # In real implementation, this would process depth images, laser scans, etc.
        return obstacles

    def get_current_location(self):
        """Get current location from Isaac ROS localization"""
        if self.current_pose:
            return {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z,
                'orientation': self.current_pose.orientation
            }
        return None

    def plan_navigation(self, obstacles, current_location):
        """Plan navigation based on current state and obstacles"""
        # Navigation planning logic
        # This would interface with Nav2 for path planning
        cmd_vel = Twist()

        if obstacles['front'] and obstacles['distance'] < 0.5:
            # Obstacle detected, turn away
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
        else:
            # Move forward
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def execute_navigation(self, cmd_vel):
        """Execute navigation command with humanoid-specific considerations"""
        # Apply humanoid-specific constraints
        cmd_vel_constrained = self.apply_humanoid_constraints(cmd_vel)

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel_constrained)

    def apply_humanoid_constraints(self, cmd_vel):
        """Apply humanoid-specific velocity constraints"""
        constrained_cmd = Twist()

        # Apply conservative limits for humanoid stability
        max_linear = 0.5  # m/s
        max_angular = 0.5  # rad/s

        constrained_cmd.linear.x = max(min(cmd_vel.linear.x, max_linear), -max_linear)
        constrained_cmd.angular.z = max(min(cmd_vel.angular.z, max_angular), -max_angular)

        # Keep other components zero for now
        constrained_cmd.linear.y = 0.0
        constrained_cmd.linear.z = 0.0
        constrained_cmd.angular.x = 0.0
        constrained_cmd.angular.y = 0.0

        return constrained_cmd

    def start_navigation(self, goal_pose):
        """Start navigation to specified goal"""
        self.navigation_active = True
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = goal_pose
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Navigation started')

    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Navigation stopped')

def main(args=None):
    rclpy.init(args=args)
    pipeline = CompleteNavigationPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Interrupted, shutting down')
    finally:
        pipeline.stop_navigation()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Full Pipeline

### Pipeline Validation

```python
# pipeline_validation.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

class PipelineValidator(Node):
    def __init__(self):
        super().__init__('pipeline_validator')

        # Subscriptions for monitoring pipeline
        self.perception_sub = self.create_subscription(
            Bool, '/perception/active', self.perception_callback, 10)
        self.localization_sub = self.create_subscription(
            Bool, '/localization/active', self.localization_callback, 10)
        self.planning_sub = self.create_subscription(
            Path, '/plan', self.planning_callback, 10)
        self.control_sub = self.create_subscription(
            Bool, '/control/active', self.control_callback, 10)

        # Publishers for validation results
        self.validation_pub = self.create_publisher(
            Float32, '/pipeline/validation_score', 10)

        # Validation state
        self.components_active = {
            'perception': False,
            'localization': False,
            'planning': False,
            'control': False
        }

        # Timer for periodic validation
        self.timer = self.create_timer(1.0, self.validate_pipeline)

        self.get_logger().info('Pipeline validator initialized')

    def perception_callback(self, msg):
        self.components_active['perception'] = msg.data

    def localization_callback(self, msg):
        self.components_active['localization'] = msg.data

    def planning_callback(self, msg):
        # Planning is active if a valid plan exists
        self.components_active['planning'] = len(msg.poses) > 0

    def control_callback(self, msg):
        self.components_active['control'] = msg.data

    def validate_pipeline(self):
        """Validate the complete pipeline"""
        active_components = sum(self.components_active.values())
        total_components = len(self.components_active)

        validation_score = Float32()
        validation_score.data = active_components / total_components

        self.validation_pub.publish(validation_score)

        self.get_logger().info(
            f'Pipeline validation: {active_components}/{total_components} components active '
            f'(score: {validation_score.data:.2f})'
        )

        if validation_score.data == 1.0:
            self.get_logger().info('✅ Pipeline fully operational')
        elif validation_score.data >= 0.75:
            self.get_logger().info('⚠️  Pipeline mostly operational')
        else:
            self.get_logger().warn('❌ Pipeline has issues')

def main(args=None):
    rclpy.init(args=args)
    validator = PipelineValidator()
    rclpy.spin(validator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization for Integration

### Optimizing the Complete Pipeline

```yaml
# integration_optimization.yaml
/**:
  ros__parameters:
    # Isaac ROS optimization
    visual_slam_node:
      ros__parameters:
        enable_slam_visualization: false  # Disable for production
        enable_observations_display: false
        enable_landmarks_view: false
        enable_observations_view: false
        max_num_landmarks: 500  # Reduce for performance
        map_frame: "map"
        odom_frame: "odom"
        base_frame: "base_link"
        publish_odom_tf: true

    # Nav2 optimization
    controller_server:
      ros__parameters:
        controller_frequency: 20.0  # Higher for better responsiveness
        min_x_velocity_threshold: 0.0005
        velocity_scaling:
          enabled: true
          scaling_threshold: 0.5
          scaling_factor: 0.2  # Conservative for humanoid stability

    planner_server:
      ros__parameters:
        expected_planner_frequency: 1.0  # Lower frequency for complex planning
        use_astar: true
        allow_unknown: false

    # Costmap optimization for integration
    local_costmap:
      ros__parameters:
        update_frequency: 10.0
        publish_frequency: 5.0
        resolution: 0.05  # Balance between accuracy and performance
        width: 10.0
        height: 10.0

    global_costmap:
      ros__parameters:
        update_frequency: 2.0
        publish_frequency: 1.0
        resolution: 0.1
        width: 100.0
        height: 100.0

    # Bridge optimization
    isaac_sim_bridge:
      ros__parameters:
        publish_frequency: 30.0  # Balance between accuracy and performance
        use_sim_time: true
        time_scale: 1.0