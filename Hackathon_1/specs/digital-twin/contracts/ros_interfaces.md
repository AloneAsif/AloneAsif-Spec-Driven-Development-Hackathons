# ROS 2 Interface Contracts: Digital Twin

## Overview
This document defines the ROS 2 interface contracts (messages, services, and actions) for the physics-accurate and visually rich digital twin using Gazebo + Unity.

## Message Types

### 1. Sensor Data Messages

#### `sensor_msgs/LaserScan`
**Purpose**: Represents 2D laser scan data from LiDAR sensors
```yaml
# Time of message
header:
  stamp: time
  frame_id: string

# Measurement parameters
angle_min: float32    # start angle of the scan [rad]
angle_max: float32    # end angle of the scan [rad]
angle_increment: float32  # angular distance between measurements [rad]
time_increment: float32   # time between measurements [seconds]
scan_time: float32        # time between scans [seconds]
range_min: float32        # minimum range value [m]
range_max: float32        # maximum range value [m]

# Measurement data
ranges: float32[]         # range data [m]
intensities: float32[]    # intensity data [device dependent units]
```

#### `sensor_msgs/Image`
**Purpose**: Represents image data from camera sensors
```yaml
header:
  stamp: time
  frame_id: string

height: uint32
width: uint32
encoding: string      # pixel format (e.g., "rgb8", "bgr8", "mono8")
is_bigendian: uint8   # 0 for little endian, 1 for big endian
step: uint32          # full row length in bytes
data: uint8[]         # actual image data buffer
```

#### `sensor_msgs/Imu`
**Purpose**: Represents inertial measurement unit data
```yaml
header:
  stamp: time
  frame_id: string

orientation: geometry_msgs/Quaternion
orientation_covariance: float64[9]

angular_velocity: geometry_msgs/Vector3
angular_velocity_covariance: float64[9]

linear_acceleration: geometry_msgs/Vector3
linear_acceleration_covariance: float64[9]
```

### 2. Control Messages

#### `geometry_msgs/Twist`
**Purpose**: Represents velocity commands for differential drive robots
```yaml
linear: geometry_msgs/Vector3
angular: geometry_msgs/Vector3
```

#### `trajectory_msgs/JointTrajectory`
**Purpose**: Represents joint trajectory commands for humanoid robots
```yaml
header:
  stamp: time
  frame_id: string

joint_names: string[]

points: trajectory_msgs/JointTrajectoryPoint[]
```

#### `trajectory_msgs/JointTrajectoryPoint`
**Purpose**: Represents a single point in a joint trajectory
```yaml
positions: float64[]
velocities: float64[]
accelerations: float64[]
effort: float64[]
time_from_start: duration
```

### 3. State Messages

#### `sensor_msgs/JointState`
**Purpose**: Represents the current state of joints
```yaml
header:
  stamp: time
  frame_id: string

name: string[]
position: float64[]
velocity: float64[]
effort: float64[]
```

#### `nav_msgs/Odometry`
**Purpose**: Represents robot odometry data
```yaml
header:
  stamp: time
  frame_id: string

child_frame_id: string

pose:
  pose: geometry_msgs/PoseWithCovariance
  covariance: float64[36]

twist:
  twist: geometry_msgs/TwistWithCovariance
  covariance: float64[36]
```

## Service Types

### 1. Simulation Control Services

#### `std_srvs/Empty`
**Purpose**: Simple service for triggering actions without parameters
- **Request**: Empty
- **Response**: Empty

#### `gazebo_msgs/SetEntityState`
**Purpose**: Set the state of an entity in Gazebo simulation
```yaml
# Request
state: gazebo_msgs/ModelState

# Response
success: bool
status_message: string
```

#### `gazebo_msgs/GetEntityState`
**Purpose**: Get the state of an entity in Gazebo simulation
```yaml
# Request
name: string
reference_frame: string

# Response
state: gazebo_msgs/ModelState
success: bool
status_message: string
```

## Action Types

### 1. Navigation Actions

#### `nav2_msgs/NavigateToPose`
**Purpose**: Navigate robot to a specific pose
```yaml
# Goal
pose: geometry_msgs/PoseStamped

# Feedback
current_pose: geometry_msgs/PoseStamped
distance_remaining: float32
navigation_time: builtin_interfaces/Time
number_of_recoveries: int16

# Result
result_code: int8
final_pose: geometry_msgs/PoseStamped
canceled_goals: int32
```

## Topic Namespaces

### Sensor Topics
- `/scan` - Laser scan data (sensor_msgs/LaserScan)
- `/camera/image_raw` - Raw camera image (sensor_msgs/Image)
- `/camera/image_rect` - Rectified camera image (sensor_msgs/Image)
- `/imu/data` - IMU data (sensor_msgs/Imu)
- `/joint_states` - Joint positions (sensor_msgs/JointState)

### Control Topics
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/joint_group_position_controller/commands` - Joint position commands (std_msgs/Float64MultiArray)
- `/joint_trajectory_controller/joint_trajectory` - Joint trajectory commands (trajectory_msgs/JointTrajectory)

### State Topics
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/tf` - Transform data (tf2_msgs/TFMessage)
- `/tf_static` - Static transform data (tf2_msgs/TFMessage)

## Quality of Service (QoS) Settings

### Sensor Data Topics
- **Reliability**: Best effort (for real-time performance)
- **Durability**: Volatile
- **History**: Keep last N samples (typically N=1)
- **Depth**: 1

### Control Command Topics
- **Reliability**: Reliable (to ensure commands are received)
- **Durability**: Volatile
- **History**: Keep last N samples (typically N=1)
- **Depth**: 1

### State Topics
- **Reliability**: Reliable (for accurate state information)
- **Durability**: Volatile
- **History**: Keep last N samples (typically N=10)
- **Depth**: 10

## Communication Patterns

### Publisher-Subscriber (Pub/Sub)
- **Sensors**: Gazebo publishes sensor data, Unity subscribes
- **Commands**: Unity publishes control commands, Gazebo subscribes
- **State**: Gazebo publishes state data, Unity subscribes

### Client-Server (Service)
- **Simulation Control**: Unity sends service requests to control simulation state
- **Parameter Updates**: Configuration changes via services

### Publisher-Subscriber with Feedback (Action)
- **Navigation**: Complex tasks with progress feedback
- **Trajectory Execution**: Multi-step control sequences

## Error Handling

### Timeout Handling
- Default timeout for service calls: 5 seconds
- Retry mechanism: 3 attempts before failure
- Connection timeout: 10 seconds for initial connection

### Failure Modes
- **Network Disconnection**: Automatic reconnection attempts
- **Message Validation**: Reject malformed messages
- **Topic Unavailability**: Graceful degradation with warning logs

## Performance Requirements

### Latency Requirements
- Sensor data transmission: < 50ms
- Control command transmission: < 20ms
- Service calls: < 100ms

### Bandwidth Requirements
- High-resolution camera data: Up to 100 Mbps
- LiDAR data: 1-10 Mbps depending on resolution
- Joint state updates: < 1 Mbps
- Control commands: < 0.1 Mbps

### Frequency Requirements
- Joint state publishing: 50-100 Hz
- Sensor data publishing: 10-30 Hz (camera), 10-50 Hz (LiDAR)
- Control command publishing: 50-100 Hz
- Odometry publishing: 50-100 Hz