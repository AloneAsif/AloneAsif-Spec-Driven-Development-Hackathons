---
title: URDF Basics for Humanoids
sidebar_position: 5
description: Learn the basics of URDF (Unified Robot Description Format) for humanoid structure and joints
---

# URDF Basics for Humanoids

## Overview

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. This chapter covers the fundamentals of URDF for creating humanoid robot models, including links, joints, and visualization aspects.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the structure and components of URDF files
- Create links and joints for humanoid robot models
- Define visual and collision properties for robot parts
- Visualize robot models in RViz
- Create complete humanoid robot descriptions

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based file format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial parameters and visual appearance.

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Links: The Building Blocks

Links represent rigid bodies in the robot model. Each link can have visual, collision, and inertial properties.

### Link Properties

- **visual**: Defines how the link appears in visualization tools
- **collision**: Defines the collision boundaries for physics simulation
- **inertial**: Defines the physical properties for dynamics simulation

### Visual Properties

The visual element defines how a link appears in visualization tools like RViz:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Choose one geometry type -->
    <box size="1 1 1"/>
    <!-- <cylinder radius="0.5" length="1"/> -->
    <!-- <sphere radius="0.5"/> -->
    <!-- <mesh filename="package://robot_description/meshes/link.dae"/> -->
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties

The collision element defines the collision boundaries:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial Properties

The inertial element defines physical properties for dynamics simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
</inertial>
```

## Joints: Connecting the Links

Joints define how links are connected and how they can move relative to each other.

### Joint Types

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement between links
- **floating**: 6 DOF without limits
- **planar**: Movement on a plane

### Joint Definition

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Creating a Simple Robot Model

Here's an example of a simple robot with a base and one rotating arm:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Humanoid Robot Considerations

When designing humanoid robots, special attention must be paid to:

### Kinematic Chains

Humanoid robots typically have multiple kinematic chains:
- Left arm: base → shoulder → elbow → wrist
- Right arm: base → shoulder → elbow → wrist
- Left leg: base → hip → knee → ankle
- Right leg: base → hip → knee → ankle

### Degrees of Freedom

Humanoid robots need sufficient DOF to mimic human-like movements:
- Head: 2-3 DOF (pitch, yaw, sometimes roll)
- Arms: 6-7 DOF each (shoulder: 3, elbow: 1, wrist: 2-3)
- Legs: 6 DOF each (hip: 3, knee: 1, ankle: 2)

## Visualization in RViz

RViz is ROS's 3D visualization tool that allows you to visualize your robot model along with sensor data, paths, and other information.

### Setting up URDF Visualization

To visualize your URDF model in RViz:

1. Create a launch file to publish the robot state
2. Use `robot_state_publisher` to publish the URDF
3. Use `joint_state_publisher` to publish joint values (for testing)

Example launch file:
```xml
<launch>
  <param name="robot_description" command="xacro $(find robot_description)/urdf/robot.urdf.xacro" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_description)/rviz/robot.rviz" />
</launch>
```

### Essential RViz Configuration

In RViz, you'll need to set up the following displays:

1. **RobotModel**: Shows the robot using the URDF
   - Set "Robot Description" to your parameter name (e.g., "robot_description")
   - Set "TF Prefix" if using multiple robots

2. **TF**: Shows the transform tree
   - Visualizes the relationships between different frames

3. **JointState**: If available, shows actual joint positions

### Launching and Viewing

To launch and view your robot:

```bash
# Launch your robot
ros2 launch your_package robot.launch.py

# In another terminal, run RViz
ros2 run rviz2 rviz2

# Or use the launch file that includes RViz
```

### Troubleshooting Visualization Issues

Common issues and solutions:

- **Robot not showing**: Check that the parameter name matches in launch file and RViz
- **Wrong colors**: Verify material definitions in URDF
- **Incorrect poses**: Check joint origins and limits
- **Missing links**: Ensure all links are connected through joints

## Gazebo Integration for URDF

Gazebo is a robotics simulation environment that can use URDF models for physics simulation.

### Adding Gazebo-Specific Elements

To make your URDF work in Gazebo, add Gazebo-specific elements:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- For adding plugins -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
  </plugin>
</gazebo>
```

### Physics Considerations

When preparing URDF for Gazebo simulation:

1. **Accurate inertial properties**: Mass, center of mass, and inertia tensors
2. **Appropriate friction values**: For realistic contact simulation
3. **Collision geometry**: Use simpler shapes for better performance
4. **Joint limits and dynamics**: Include damping and friction parameters

## Best Practices for URDF

1. **Use meaningful names**: Name links and joints descriptively
2. **Proper parenting**: Ensure all links are connected through joints
3. **Realistic inertias**: Calculate or estimate inertial properties accurately
4. **Collision vs visual**: Use simpler geometries for collision to improve performance
5. **Use xacro**: For complex robots, use Xacro (XML Macros) to simplify URDF
6. **Test incrementally**: Add links and joints one by one to debug issues
7. **Validate your URDF**: Use tools like `check_urdf` to verify structure

## Practical Exercises for URDF

### Exercise 1: Simple Mobile Robot
Create a URDF for a simple wheeled robot:
- Create a base with two drive wheels and one caster
- Define appropriate visual and collision properties
- Test visualization in RViz

### Exercise 2: Articulated Arm
Create a URDF for a simple robotic arm:
- Design 3-4 links connected by revolute joints
- Add proper inertial properties
- Test range of motion in RViz

### Exercise 3: Humanoid Torso
Create a URDF for a humanoid upper body:
- Model torso, head, and two arms
- Use appropriate joint types and limits
- Include realistic dimensions

## Summary

URDF is a fundamental component of ROS robot development, allowing you to describe robot models with links and joints. Understanding URDF is essential for creating humanoid robot models that can be simulated, visualized, and controlled in ROS environments.

## Next Steps

This completes the ROS 2 educational module. You now have a solid foundation in ROS 2 architecture, publishing/subscribing, Python AI agent integration, and URDF basics for humanoid robots.

## Navigation

- **Previous**: [Python AI Agent Integration](./ros2-python-agents.md)
- **Next**: None

## Related Chapters

- [Introduction to ROS 2 Architecture](./intro-to-ros2.md) - Fundamental concepts of ROS 2 architecture
- [ROS 2 Publishing and Subscribing](./ros2-pub-sub.md) - Learn how to implement publisher-subscriber communication
- [Python AI Agent Integration](./ros2-python-agents.md) - Connect Python AI agents to ROS controllers