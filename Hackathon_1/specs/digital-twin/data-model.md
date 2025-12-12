# Data Model: Module 2: The Digital Twin

## Overview
This document defines the data models and entities for the physics-accurate and visually rich digital twin using Gazebo + Unity, integrated with ROS 2.

## Core Entities

### 1. RobotModel
**Description**: Represents the humanoid robot model with joints, links, and physical properties

**Fields**:
- id: string (unique identifier for the robot model)
- name: string (name of the robot, e.g., "Atlas", "Pepper", "Custom Humanoid")
- urdf_path: string (path to the URDF file)
- links: array[Link] (list of physical links in the robot)
- joints: array[Joint] (list of joints connecting the links)
- sensors: array[Sensor] (list of sensors attached to the robot)
- mass: float (total mass of the robot in kg)
- dimensions: object {width: float, height: float, depth: float} (physical dimensions)

### 2. Link
**Description**: Represents a rigid body part of the robot

**Fields**:
- id: string (unique identifier for the link)
- name: string (name of the link, e.g., "base_link", "left_arm", "head")
- visual_mesh: string (path to visual mesh file)
- collision_mesh: string (path to collision mesh file)
- mass: float (mass of the link in kg)
- inertia: object {ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float} (inertia tensor)
- position: object {x: float, y: float, z: float} (position in 3D space)
- orientation: object {x: float, y: float, z: float, w: float} (quaternion orientation)

### 3. Joint
**Description**: Represents a connection between two links with specific movement constraints

**Fields**:
- id: string (unique identifier for the joint)
- name: string (name of the joint, e.g., "left_elbow_joint", "right_knee_joint")
- type: string (joint type: "revolute", "prismatic", "fixed", "continuous", "floating")
- parent_link: string (name of the parent link)
- child_link: string (name of the child link)
- axis: object {x: float, y: float, z: float} (rotation/translational axis)
- limits: object {lower: float, upper: float, effort: float, velocity: float} (movement limits)
- position: float (current joint position)
- velocity: float (current joint velocity)
- effort: float (current joint effort/torque)

### 4. Sensor
**Description**: Represents a sensor attached to a robot link

**Fields**:
- id: string (unique identifier for the sensor)
- name: string (name of the sensor, e.g., "front_camera", "lidar_3d", "imu_sensor")
- type: string (sensor type: "camera", "lidar", "imu", "gps", "force_torque")
- parent_link: string (name of the link the sensor is attached to)
- position: object {x: float, y: float, z: float} (position relative to parent link)
- orientation: object {x: float, y: float, z: float, w: float} (orientation relative to parent link)
- parameters: object (sensor-specific parameters)
- ros_topic: string (ROS 2 topic name for sensor data)

### 5. SimulationEnvironment
**Description**: Represents the simulation environment with physics properties and world settings

**Fields**:
- id: string (unique identifier for the environment)
- name: string (name of the environment, e.g., "indoor_lab", "outdoor_park")
- gazebo_world_file: string (path to the Gazebo world file)
- gravity: object {x: float, y: float, z: float} (gravity vector)
- physics_engine: string (physics engine: "ode", "bullet", "dart")
- time_step: float (simulation time step in seconds)
- real_time_factor: float (real-time update rate multiplier)
- objects: array[EnvironmentObject] (static and dynamic objects in the environment)

### 6. EnvironmentObject
**Description**: Represents objects in the simulation environment

**Fields**:
- id: string (unique identifier for the object)
- name: string (name of the object)
- type: string (object type: "static", "dynamic", "kinematic")
- visual_mesh: string (path to visual mesh file)
- collision_mesh: string (path to collision mesh file)
- position: object {x: float, y: float, z: float} (position in world coordinates)
- orientation: object {x: float, y: float, z: float, w: float} (orientation in world coordinates)
- mass: float (mass of the object in kg)
- friction: float (friction coefficient)
- restitution: float (restitution coefficient)

### 7. SensorData
**Description**: Represents data published by sensors in the simulation

**Fields**:
- sensor_id: string (identifier of the source sensor)
- timestamp: float (ROS time stamp)
- frame_id: string (coordinate frame of the sensor)
- data: object (sensor-specific data payload)

**Subtypes**:
- **CameraData**: {image: base64_encoded_image, width: int, height: int, encoding: string}
- **LidarData**: {ranges: array[float], intensity: array[float], angle_min: float, angle_max: float, angle_increment: float}
- **ImuData**: {orientation: {x, y, z, w}, angular_velocity: {x, y, z}, linear_acceleration: {x, y, z}}

### 8. ControlCommand
**Description**: Represents commands sent to control the robot

**Fields**:
- command_id: string (unique identifier for the command)
- robot_id: string (identifier of the target robot)
- command_type: string (type: "joint_trajectory", "cmd_vel", "gripper_control")
- timestamp: float (time of command issuance)
- parameters: object (command-specific parameters)
- target_values: array[float] (target positions, velocities, or efforts)

## State Transitions

### Robot State Transitions
- **INITIALIZING** → **READY**: When robot model is loaded and all joints are calibrated
- **READY** → **ACTIVE**: When robot starts receiving commands
- **ACTIVE** → **PAUSED**: When simulation is paused
- **PAUSED** → **ACTIVE**: When simulation resumes
- **ACTIVE** → **ERROR**: When a critical error occurs
- **ERROR** → **READY**: After error recovery

### Simulation State Transitions
- **STOPPED** → **RUNNING**: When simulation starts
- **RUNNING** → **PAUSED**: When simulation is paused
- **PAUSED** → **RUNNING**: When simulation resumes
- **RUNNING** → **STOPPED**: When simulation stops
- **STOPPED** → **RESET**: When simulation is reset to initial state

## Relationships

1. **RobotModel** contains multiple **Link** and **Joint** entities
2. **RobotModel** has multiple **Sensor** entities attached
3. **Joint** connects exactly two **Link** entities (parent and child)
4. **Sensor** is attached to exactly one **Link**
5. **SimulationEnvironment** contains multiple **EnvironmentObject** entities
6. **Sensor** publishes multiple **SensorData** entities
7. **RobotModel** receives multiple **ControlCommand** entities

## Validation Rules

1. **RobotModel** must have at least one **Link** and one **Joint**
2. **Joint** parent_link and child_link must reference valid **Link** entities
3. **Sensor** parent_link must reference a valid **Link** entity
4. **Joint** limits must be physically realistic (e.g., velocity and effort limits)
5. **Link** mass must be positive
6. **SimulationEnvironment** must have valid gravity settings (typically z = -9.81)
7. **SensorData** timestamp must be within simulation time bounds
8. **ControlCommand** target_values must be within joint limits if applicable