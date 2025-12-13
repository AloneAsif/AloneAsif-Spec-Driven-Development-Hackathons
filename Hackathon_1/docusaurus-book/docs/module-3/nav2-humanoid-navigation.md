# Nav2 for Humanoid Navigation

## Overview

This chapter covers configuring the Navigation2 (Nav2) stack specifically for humanoid robots with bipedal locomotion. Students will learn to adapt standard navigation approaches for the unique challenges of legged robots, including stability considerations, step constraints, and balance requirements.

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand Nav2 stack architecture and components
- Configure costmaps for humanoid-specific navigation
- Set up obstacle layers for bipedal locomotion
- Plan paths considering step constraints and balance
- Implement velocity smoothing for stability
- Integrate with Isaac ROS localization
- Test and validate humanoid navigation

## Nav2 Stack Overview

### Core Architecture

Navigation2 (Nav2) is the navigation stack for ROS 2, providing a complete solution for autonomous navigation. The architecture consists of several key components:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Navigation    │    │   Path Planning  │    │  Controller &   │
│   Server        │◄───┤   & Smoothing    │◄───┤   Velocity      │
│                 │    │                  │    │   Generation    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                       ▲                       ▲
         │                       │                       │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Recovery      │    │   Costmap        │    │   Localizer     │
│   Behaviors     │    │   Layers         │    │   (Isaac ROS)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Key Components

1. **Navigation Server**: Main orchestration node
2. **Behavior Tree**: Task execution framework
3. **Path Planner**: Global and local path planning
4. **Controller**: Velocity command generation
5. **Costmap**: Obstacle and cost representation
6. **Recovery Behaviors**: Actions for stuck situations

### Humanoid-Specific Considerations

Humanoid navigation differs from wheeled robot navigation in several key ways:
- **Stability**: Must maintain balance during movement
- **Step Constraints**: Limited step size and placement
- **Kinematics**: Complex joint constraints
- **Gait Planning**: Coordinated leg movement patterns

## Costmap Configuration for Humanoids

### Understanding Costmaps

Costmaps in Nav2 represent the environment as a grid of costs that influence path planning. For humanoid robots, costmaps must account for:

- **Larger Footprint**: For stability during walking
- **Step Height Limits**: For obstacle clearance
- **Balance Constraints**: For fall risk assessment
- **Step Placement**: For feasible foot positions

### Costmap Parameters for Humanoids

```yaml
# costmap_common_params_humanoid.yaml
# Robot configuration
robot_radius: 0.6  # Larger than typical wheeled robot for stability

# Obstacle detection
obstacle_range: 3.0
raytrace_range: 4.0

# Footprint definition (larger for stability)
footprint: [[-0.5, -0.5], [-0.5, 0.5], [0.8, 0.5], [0.8, -0.5]]  # Rectangular footprint for humanoid

# Inflation layer for safety (larger for humanoid)
inflation_layer:
  enabled: true
  cost_scaling_factor: 5.0  # Higher for humanoid safety
  inflation_radius: 0.8     # Larger inflation for stability
  inflate_to_costmap: true

# Obstacle layer
obstacle_layer:
  enabled: true
  obstacle_range: 3.0
  raytrace_range: 4.0
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    data_type: LaserScan
    obstacle_range: 3.0
    raytrace_range: 4.0

# Static layer
static_layer:
  enabled: true
  map_topic: /map
  subscribe_to_updates: true
  track_unknown_space: true
  use_maximum: false
  unknown_cost_value: -1
  lethal_cost_threshold: 100
  trinary_costmap: true
```

### Global Costmap Configuration

```yaml
# global_costmap_params_humanoid.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: false
      width: 100
      height: 100
      resolution: 0.05
      origin_x: -50.0
      origin_y: -50.0

  # Static layer for map
  static_layer:
    ros__parameters:
      map_subscribe_transient_local: True

  # Obstacle layer
  obstacle_layer:
    ros__parameters:
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: LaserScan
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 8.0
        obstacle_min_range: 0.0

  # Inflation layer for humanoid safety
  inflation_layer:
    ros__parameters:
      enabled: True
      cost_scaling_factor: 5.0
      inflation_radius: 0.8
      inflate_unknown: false

  always_send_full_costmap: True
```

### Local Costmap Configuration

```yaml
# local_costmap_params_humanoid.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05  # Higher resolution for precise foot placement

  # Static layer
  static_layer:
    ros__parameters:
      map_subscribe_transient_local: True

  # Obstacle layer
  obstacle_layer:
    ros__parameters:
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: LaserScan
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 8.0
        obstacle_min_range: 0.0

  # Inflation layer for local planning
  inflation_layer:
    ros__parameters:
      enabled: True
      cost_scaling_factor: 3.0  # Lower than global for local planning
      inflation_radius: 0.6     # Smaller for local planning
      inflate_unknown: false

  always_send_full_costmap: True
```

## Obstacle Layer Setup for Bipedal Robots

### Step Height and Clearance

Humanoid robots have specific constraints for obstacle clearance:

```yaml
# obstacle_layer_humanoid.yaml
obstacle_layer:
  ros__parameters:
    enabled: True
    obstacle_range: 3.0
    raytrace_range: 4.0
    observation_sources: scan
    scan:
      topic: /scan
      sensor_frame: base_scan
      max_obstacle_height: 0.3  # Maximum step height for humanoid
      clearing: True
      marking: True
      data_type: LaserScan
      min_obstacle_height: 0.05  # Minimum detectable obstacle
      obstacle_range: 3.0
      raytrace_range: 4.0
      inf_is_valid: False
      clearing_threshold: 0.1
      marking_threshold: 0.65

    # Additional layers for humanoid-specific obstacles
    step_layer:
      enabled: True
      # Custom layer for step height analysis
      max_step_height: 0.3
      min_step_height: 0.05
```

### Fall Risk Assessment

```yaml
# fall_risk_layer.yaml
fall_risk_layer:
  ros__parameters:
    enabled: True
    risk_radius: 1.0  # Radius around robot to assess fall risk
    height_threshold: 0.5  # Height threshold for fall risk
    cost_multiplier: 10.0  # Multiplier for high-risk areas
    obstacle_buffer: 0.2   # Buffer around obstacles for safety
```

## Path Planning for Bipedal Robots

### Global Planner Configuration

For humanoid robots, the global planner should consider:

1. **Stability**: Paths that maintain center of mass within support polygon
2. **Step Constraints**: Feasible step locations and sizes
3. **Balance**: Smooth transitions between steps

```yaml
# global_planner_params_humanoid.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path to the Behavior Tree XML file
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Number of recovery behaviors to cycle through before failing
    number_of_recoveries_before_shutting_down: 5
    # Recovery behaviors
    navigate_to_pose_goal_checker:
      rot_stopped_velocity_threshold: 0.005
      trans_stopped_velocity_threshold: 0.05
      time_to_allow_planning: 10.0

# Global planner with humanoid constraints
nav2_compute_path_to_pose_bt_node:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_cost_sweeping: True
    cost_sweep_resolution: 0.5  # Sweeping resolution for humanoid constraints
    cost_sweep_radius: 1.0      # Radius for cost sweeping
    cost_sweep_bias: 1.5        # Bias for humanoid-specific costs
```

### Local Planner Configuration

The local planner for humanoid robots needs special consideration for:

1. **Velocity Smoothing**: Gradual velocity changes for stability
2. **Step Planning**: Coordinated leg movements
3. **Balance Maintenance**: Center of mass control

```yaml
# local_planner_params_humanoid.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25  # Larger tolerance for humanoid navigation
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters (using humanoid-aware controller)
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
      minimum_controller_frequency: 10.0
      maximum_controller_frequency: 50.0
      velocity_scaling:
        enabled: True
        scaling_threshold: 1.0
        scaling_factor: 0.25  # Conservative scaling for humanoid stability
```

### Humanoid-Specific Path Planning

```yaml
# humanoid_path_planning.yaml
# Custom path planner for humanoid robots
humanoid_path_planner:
  ros__parameters:
    # Step constraints
    max_step_length: 0.4    # Maximum step length (meters)
    max_step_width: 0.2     # Maximum step width (meters)
    max_step_height: 0.3    # Maximum step height (meters)

    # Balance constraints
    com_buffer: 0.1         # Buffer for center of mass
    support_polygon_size: 0.3  # Size of support polygon

    # Gait parameters
    step_duration: 1.0      # Time per step (seconds)
    double_support_ratio: 0.3  # Ratio of double support phase

    # Path smoothing
    smoothing_iterations: 10
    smoothing_weight: 0.5
    obstacle_weight: 1.0
    smoothness_weight: 0.2
```

## Velocity Smoothing and Stability Considerations

### Stability-Aware Velocity Control

Humanoid robots require careful velocity control to maintain balance:

```yaml
# velocity_controller_humanoid.yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    velocity_timeout: 1.0
    max_velocity: [0.5, 0.0, 1.0]  # Linear x, y, Angular z
    min_velocity: [-0.5, 0.0, -1.0]
    max_acceleration: [0.5, 0.0, 0.5]  # Conservative for stability
    max_deceleration: [0.5, 0.0, 0.5]

    # Humanoid-specific parameters
    balance_margin: 0.1      # Margin for balance maintenance
    step_constraint: True    # Enforce step constraints
    com_tracking: True       # Track center of mass
```

### Gait Pattern Integration

```yaml
# gait_pattern_controller.yaml
gait_controller:
  ros__parameters:
    # Gait parameters
    gait_type: "walking"     # walking, stepping, etc.
    step_height: 0.05        # Step height for clearance
    step_duration: 1.0       # Duration of each step
    double_support_ratio: 0.3  # Ratio of double support phase

    # Balance parameters
    com_reference_height: 0.8  # Reference CoM height
    com_max_deviation: 0.05    # Maximum CoM deviation
    zmp_margin: 0.05          # Zero moment point margin

    # Swing foot trajectory
    swing_trajectory: "cubic"  # cubic, linear, etc.
    swing_height: 0.05         # Maximum swing height
```

## Integration with Isaac ROS Localization

### Connecting Isaac ROS to Nav2

Isaac ROS provides high-quality visual-inertial localization that can be integrated with Nav2:

```yaml
# isaac_ros_nav2_integration.yaml
/**:
  ros__parameters:
    # Isaac ROS localization topic
    localization_topic: "/visual_slam/visual_odometry"

    # Nav2 localization parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    global_frame: "map"

    # TF settings
    transform_tolerance: 0.2
    use_sim_time: True

    # Isaac ROS specific parameters
    localization_type: "visual_inertial"
    pose_update_frequency: 10.0  # Hz
    velocity_update_frequency: 20.0  # Hz

    # Integration parameters
    enable_localization_fusion: True
    localization_confidence_threshold: 0.7
    pose_variance_threshold: 0.01
```

### TF Configuration

```yaml
# tf_config_humanoid.yaml
# Transform configuration for humanoid robot with Isaac ROS localization
/**:
  ros__parameters:
    # Static transforms
    base_to_camera:
      parent: "base_link"
      child: "camera_link"
      translation: [0.1, 0.0, 0.8]  # Camera position on humanoid
      rotation: [0.0, 0.0, 0.0, 1.0]  # No rotation (looking forward)

    # Dynamic transforms from Isaac ROS
    odom_to_map:
      parent: "map"
      child: "odom"
      # This transform comes from Isaac ROS localization
```

### Launch Configuration

```xml
<!-- isaac_ros_nav2_integration.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_occupancy_grid': True,
            'enable_slam_visualization': True
        }],
        remappings=[('stereo_camera/left/image', '/camera/rgb_left_rect/image'),
                   ('stereo_camera/right/image', '/camera/rgb_right_rect/image')]
    )

    # Nav2 Navigation Server
    navigation_server = Node(
        package='nav2_navigation',
        executable='navigation_server',
        name='navigation_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Local Planner
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Path Planner
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
                                 'global_costmap']}]
    )

    return LaunchDescription([
        visual_slam_node,
        navigation_server,
        controller_server,
        planner_server,
        bt_navigator,
        lifecycle_manager
    ])
```

## Recovery Behaviors for Humanoid Navigation

### Humanoid-Specific Recovery Behaviors

Humanoid robots need specialized recovery behaviors that account for their unique capabilities and constraints:

```yaml
# recovery_params_humanoid.yaml
bt_navigator:
  ros__parameters:
    # Recovery behaviors for humanoid navigation
    costmap_filters_info_server_plugin: "nav2_costmap_2d::CostmapFilterInfoServer"
    local_costmap:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    global_costmap:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    # Recovery behavior configuration
    recovery_plugins: ["spin", "backup", "wait", "humanoid_step_around"]

    # Spin recovery (humanoid-specific)
    spin:
      plugin: "nav2_recoveries/Spin"
      enabled: True
      simulate_ahead_time: 2.0
      max_rotational_vel: 0.5  # Conservative for stability
      min_rotational_vel: 0.1
      rotational_acc_lim: 0.5  # Conservative acceleration limit

    # Backup recovery (humanoid-specific)
    backup:
      plugin: "nav2_recoveries/BackUp"
      enabled: True
      sim_time: 2.0
      vx_max: 0.1
      vx_min: -0.1
      sim_granularity: 0.02

    # Wait recovery
    wait:
      plugin: "nav2_recoveries/Wait"
      enabled: True
      wait_duration: 1.0

    # Humanoid-specific step-around recovery
    humanoid_step_around:
      plugin: "nav2_humanoid_recoveries/StepAround"
      enabled: True
      max_step_distance: 0.4  # Maximum step-around distance
      step_height: 0.1        # Step height for obstacles
      balance_check: True     # Check balance during step-around
```

### Custom Recovery Behavior Example

```python
# humanoid_recovery_behavior.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import BackUp
from geometry_msgs.msg import Twist
from nav2_msgs.msg import RecoveryInfo
import time

class HumanoidStepAroundRecovery(Node):
    def __init__(self):
        super().__init__('humanoid_step_around_recovery')

        # Action server for recovery
        self._action_server = ActionServer(
            self,
            BackUp,
            'humanoid_step_around',
            self.execute_callback
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Humanoid Step Around Recovery initialized')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing humanoid step around recovery')

        # Get current robot state
        # Plan a step-around path considering humanoid constraints
        # Execute the step-around maneuver

        # Example: Step to the side to avoid obstacle
        self.execute_step_around()

        goal_handle.succeed()
        result = BackUp.Result()
        return result

    def execute_step_around(self):
        """Execute step-around maneuver for humanoid"""
        # Calculate step-around trajectory
        # Check for balance during maneuver
        # Execute coordinated leg movements

        # Example velocity command (simplified)
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.2  # Move sideways
        cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(2.0)  # Execute for 2 seconds

        # Stop
        cmd_vel.linear.y = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    recovery_node = HumanoidStepAroundRecovery()
    rclpy.spin(recovery_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Navigation Testing and Validation

### Testing Framework

```yaml
# navigation_test_config.yaml
navigation_tester:
  ros__parameters:
    # Test parameters
    test_duration: 300.0  # 5 minutes
    success_threshold: 0.9  # 90% success rate
    max_planning_time: 10.0  # Maximum planning time
    min_path_clearance: 0.3  # Minimum clearance from obstacles

    # Humanoid-specific test parameters
    stability_threshold: 0.1  # Maximum CoM deviation
    step_success_rate: 0.95   # Minimum step success rate
    balance_maintenance: True # Require balance maintenance

    # Test scenarios
    test_scenarios: [
      "simple_path",
      "obstacle_avoidance",
      "narrow_passage",
      "turning_maneuvers",
      "stop_and_go"
    ]
```

### Validation Metrics

```python
# navigation_validator.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class NavigationValidator(Node):
    def __init__(self):
        super().__init__('navigation_validator')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)

        # Publishers for validation metrics
        self.clearance_pub = self.create_publisher(Float32, '/min_clearance', 10)
        self.success_pub = self.create_publisher(Float32, '/success_rate', 10)

        # Validation parameters
        self.path_points = []
        self.scan_data = None
        self.current_pose = None
        self.min_clearance = float('inf')

        # Timer for periodic validation
        self.timer = self.create_timer(1.0, self.validate_navigation)

    def path_callback(self, msg):
        """Store path for validation"""
        self.path_points = msg.poses

    def scan_callback(self, msg):
        """Store laser scan for obstacle detection"""
        self.scan_data = msg

    def pose_callback(self, msg):
        """Store current robot pose"""
        self.current_pose = msg.pose

    def validate_navigation(self):
        """Validate navigation performance"""
        if self.path_points and self.scan_data and self.current_pose:
            # Calculate minimum clearance from path to obstacles
            clearance = self.calculate_clearance()
            self.min_clearance = min(self.min_clearance, clearance)

            # Publish clearance metric
            clearance_msg = Float32()
            clearance_msg.data = clearance
            self.clearance_pub.publish(clearance_msg)

            # Log validation results
            self.get_logger().info(f'Minimum clearance: {clearance:.2f}m')

    def calculate_clearance(self):
        """Calculate minimum clearance from path to obstacles"""
        if not self.path_points or not self.scan_data:
            return float('inf')

        # Convert scan to obstacle positions
        obstacle_positions = self.scan_to_positions(self.scan_data)

        # Calculate minimum distance from path to obstacles
        min_dist = float('inf')
        for path_pose in self.path_points:
            for obs_pos in obstacle_positions:
                dist = np.sqrt(
                    (path_pose.pose.position.x - obs_pos[0])**2 +
                    (path_pose.pose.position.y - obs_pos[1])**2
                )
                min_dist = min(min_dist, dist)

        return min_dist

    def scan_to_positions(self, scan_msg):
        """Convert laser scan to obstacle positions"""
        positions = []
        angle = scan_msg.angle_min

        for range_val in scan_msg.ranges:
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                positions.append([x, y])
            angle += scan_msg.angle_increment

        return positions

def main(args=None):
    rclpy.init(args=args)
    validator = NavigationValidator()
    rclpy.spin(validator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Navigation Issues

### Common Humanoid Navigation Problems

#### 1. Instability During Navigation
**Symptoms**: Robot falls or becomes unstable during navigation
**Solutions**:
- Reduce velocity commands
- Increase velocity smoothing
- Check CoM tracking parameters
- Verify gait pattern parameters

#### 2. Path Planning Failures
**Symptoms**: Planner fails to find valid paths
**Solutions**:
- Adjust costmap inflation parameters
- Verify footprint configuration
- Check obstacle layer settings
- Increase planning timeout

#### 3. Recovery Behavior Issues
**Symptoms**: Robot gets stuck and recovery fails
**Solutions**:
- Adjust recovery behavior parameters
- Check sensor coverage
- Verify localization quality
- Increase recovery behavior timeout

### Debugging Tools

#### Navigation Debug Configuration

```yaml
# navigation_debug.yaml
/**:
  ros__parameters:
    # Debug parameters
    controller_server:
      ros__parameters:
        debug: True
        publish_cost_grid_pc: True
        controller_frequency: 20.0

    planner_server:
      ros__parameters:
        debug: True
        use_threaded_executor: True

    local_costmap:
      ros__parameters:
        debug: True
        publish_frequency: 2.0

    global_costmap:
      ros__parameters:
        debug: True
        publish_frequency: 1.0
```

#### RViz Configuration for Navigation

```yaml
# rviz_nav2_humanoid.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1/Frames1
        - /Local Costmap1
        - /Global Costmap1
        - /Local Plan1
        - /Global Plan1
      Splitter Ratio: 0.5
    Tree Height: 863
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Global Costmap
      Topic:
        datatype: nav_msgs/msg/OccupancyGrid
        qos_profile: default
        value: /global_costmap/costmap
      Update Topic:
        datatype: nav_msgs/msg/OccupancyGrid
        qos_profile: default
        value: /global_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Local Costmap
      Topic:
        datatype: nav_msgs/msg/OccupancyGrid
        qos_profile: default
        value: /local_costmap/costmap
      Update Topic:
        datatype: nav_msgs/msg/OccupancyGrid
        qos_profile: default
        value: /local_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Global Plan
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        datatype: nav_msgs/msg/Path
        qos_profile: default
        value: /plan
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 255; 25; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Local Plan
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        datatype: nav_msgs/msg/Path
        qos_profile: default
        value: /local_plan
      Value: true
    - Alpha: 1
      Arrow Length: 0.30000001192092896
      Axes Length: 0.30000001192092896
      Axes Radius: 0.10000000149011612
      Class: rviz_default_plugins/PoseArray
      Color: 0; 192; 0
      Enabled: true
      Head Length: 0.07000000029802322
      Head Radius: 0.029999999329447746
      Name: Pose Array
      Shaft Length: 0.23000000417232513
      Shaft Radius: 0.009999999776482582
      Shape: Arrow (Flat)
      Topic:
        datatype: geometry_msgs/msg/PoseArray
        qos_profile: default
        value: /particlecloud
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Angle: 0
      Class: rviz_default_plugins/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Scale: 100
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
    Saved: ~
Window Geometry:
  Height: 1043
  Width: 1920
  X: 0
  Y: 0
```

## Performance Optimization

### Optimizing Navigation for Humanoid Robots

```yaml
# optimization_params_humanoid.yaml
/**:
  ros__parameters:
    # Planning optimization
    planner_server:
      ros__parameters:
        expected_planner_frequency: 10.0  # Lower frequency for complex humanoid planning
        max_iterations_after_timeout: 50
        use_astar: true  # A* often works better for humanoid constraints
        allow_unknown: false  # Better safety for humanoid navigation

    # Controller optimization
    controller_server:
      ros__parameters:
        controller_frequency: 20.0  # Higher for better stability control
        min_x_velocity_threshold: 0.0005  # Very low threshold for precision
        min_y_velocity_threshold: 0.0005
        min_theta_velocity_threshold: 0.0005

        # Humanoid-specific optimization
        velocity_scaling:
          enabled: True
          scaling_threshold: 0.5  # Conservative scaling
          scaling_factor: 0.1     # Very conservative for stability

    # Costmap optimization
    local_costmap:
      ros__parameters:
        update_frequency: 10.0  # Higher for responsive obstacle avoidance
        resolution: 0.025       # Higher resolution for precise navigation
        width: 8.0             # Smaller for efficiency
        height: 8.0

    global_costmap:
      ros__parameters:
        update_frequency: 2.0   # Lower for efficiency
        resolution: 0.05        # Standard resolution
        width: 100.0           # Full map coverage
        height: 100.0
```

## Best Practices

### Humanoid Navigation Best Practices

1. **Start Conservative**: Begin with conservative parameters and gradually increase performance
2. **Validate Stability**: Always test balance and stability during navigation
3. **Monitor Localization**: Ensure consistent localization quality
4. **Test Recovery**: Verify recovery behaviors work in various scenarios
5. **Simulate First**: Test extensively in simulation before real robot deployment

### Safety Considerations

1. **Emergency Stop**: Implement emergency stop capabilities
2. **Localization Fallback**: Have fallback navigation when localization fails
3. **Stability Monitoring**: Continuously monitor robot stability
4. **Safe Velocities**: Use conservative velocity limits
5. **Obstacle Detection**: Ensure reliable obstacle detection

## Next Steps

After configuring Nav2 for humanoid navigation:
- Integrate with [Isaac Sim for full pipeline testing](./isaac-sim-overview.md)
- Test the complete [perception → localization → planning → control loop](./system-integration.md)
- Validate performance with [Isaac ROS perception](./isaac-ros-perception.md)

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Humanoid Navigation Research](https://ieeexplore.ieee.org/document/9428123)
- [ROS 2 Navigation Tutorials](https://docs.ros.org/en/humble/Tutorials/Navigation/Navigation2-Tutorials.html)
- [Isaac ROS Integration Guide](https://nvidia-isaac-ros.github.io/concepts/navigation/index.html)