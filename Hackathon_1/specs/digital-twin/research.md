# Research: Module 2: The Digital Twin

## Overview
This research document addresses the technical requirements for implementing a physics-accurate and visually rich digital twin using Gazebo + Unity, integrated with ROS 2.

## Decision: Gazebo and Unity Integration Approach
**Rationale**: Using Gazebo for physics simulation and Unity for high-fidelity visualization provides the best of both worlds - accurate physics simulation and high-quality rendering. The ROS 2 bridge enables seamless communication between both environments.

**Alternatives considered**:
- Using only Gazebo with custom rendering: Limited visualization quality
- Using only Unity with custom physics: Less accurate physics simulation
- Using NVIDIA Isaac Sim: More complex setup, potentially higher licensing costs

## Decision: Unity ROS-TCP Connector for Communication
**Rationale**: The Unity ROS-TCP Connector provides a straightforward way to connect Unity with ROS 2 systems, allowing bidirectional communication for sensor data and control commands.

**Alternatives considered**:
- Custom ROS 2 client for Unity: More development effort
- ROSBridge with WebSocket: Additional middleware layer complexity
- Direct TCP/IP communication: Less standardized approach

## Decision: Documentation Structure in Docusaurus
**Rationale**: Organizing documentation in Docusaurus with 3 chapters (Building the Digital Twin, Gazebo Simulation, Unity High-Fidelity Rendering) provides clear separation of concerns and logical progression for users.

**Alternatives considered**:
- Single comprehensive document: Harder to navigate and maintain
- Separate documentation systems: Less consistency and harder to maintain

## Technical Research Findings

### Gazebo Harmonic Setup
- Gazebo Harmonic is the latest stable version with good ROS 2 integration
- Supports physics simulation, collisions, and various sensors (LiDAR, camera, IMU)
- Can load URDF models directly from ROS 2 workspace
- Provides realistic humanoid joint simulation

### Unity Integration
- Unity 2022.3 LTS recommended for stability
- Unity ROS-TCP Connector available as Unity package
- Supports bidirectional communication with ROS 2
- Can receive sensor data and send control commands in real-time

### ROS 2 Bridge
- Uses standard ROS 2 topics and services for communication
- Supports /cmd_vel for movement commands
- Handles joint state publishing for humanoid models
- Enables sensor data publishing (LiDAR, camera, IMU)

### Sensor Integration
- LiDAR: Implemented as ray-based sensors in Gazebo, published as sensor_msgs/LaserScan
- Camera: RGB and depth cameras supported, published as sensor_msgs/Image
- IMU: Inertial measurement unit sensors, published as sensor_msgs/Imu

### Performance Considerations
- Real-time simulation requires careful optimization
- Network latency between Gazebo and Unity should be minimized
- Unity rendering performance depends on scene complexity
- ROS 2 communication should use appropriate QoS settings for real-time requirements

## Dependencies and Setup Requirements

### Gazebo Dependencies
- Gazebo Harmonic (gazebo)
- ROS 2 Humble Hawksbill or later
- gazebo_ros_pkgs for ROS integration
- URDF models from Module 1

### Unity Dependencies
- Unity 2022.3 LTS or later
- Unity ROS-TCP Connector package
- .NET Framework for ROS communication scripts

### ROS 2 Dependencies
- ROS 2 Humble Hawksbill or later
- Standard message types (sensor_msgs, geometry_msgs, etc.)
- rclpy or rclcpp for bridge implementation