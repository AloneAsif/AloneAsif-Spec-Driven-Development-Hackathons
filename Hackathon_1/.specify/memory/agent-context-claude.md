# Agent Context: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Project Context
- **Project**: Physical AI & Humanoid Robotics Book
- **Module**: 3 - The AI-Robot Brain (NVIDIA Isaac™)
- **Branch**: 1-isaac-robot-brain
- **Technology Stack**: Isaac Sim, Isaac ROS, Nav2, ROS 2 Humble, Ubuntu 22.04

## Key Technologies
- **Isaac Sim**: NVIDIA's photorealistic simulation environment based on Omniverse and USD
- **Isaac ROS**: GPU-accelerated ROS 2 packages for perception and navigation
- **Nav2**: Navigation stack for ROS 2, configured for humanoid (bipedal) robot navigation
- **VSLAM**: Visual Simultaneous Localization and Mapping using Isaac ROS
- **Synthetic Data Generation**: RGB, depth, and segmentation data from Isaac Sim

## Architecture Overview
- **Simulation Layer**: Isaac Sim with USD assets, sensors, synthetic data generation
- **Perception Layer**: Isaac ROS with GPU-accelerated VSLAM
- **Navigation Layer**: Nav2 with humanoid-specific configuration
- **Middleware**: ROS 2 Humble

## Hardware Requirements
- **Workstation**: RTX-capable GPU (RTX 3080 or higher recommended)
- **Edge**: Jetson Orin for deployment
- **Platform**: Ubuntu 22.04 with ROS 2 Humble

## Documentation Structure
- **Chapter 1**: Isaac Sim & Synthetic Worlds (isaac-sim-overview.md)
- **Chapter 2**: Isaac ROS: Perception & VSLAM (isaac-ros-perception.md)
- **Chapter 3**: Nav2 for Humanoid Navigation (nav2-humanoid-navigation.md)

## Humanoid-Specific Considerations
- Navigation configuration must account for bipedal locomotion rather than wheeled robot assumptions
- Larger robot footprint for stability
- Custom costmap inflation for step height and fall risk
- Velocity smoothing for stability during movement

## Integration Points
- Isaac Sim ↔ ROS 2 bridge for simulation-to-reality data flow
- Isaac ROS localization data feeding into Nav2 for navigation planning
- Full perception → localization → planning → control loop

## Performance Targets
- Isaac Sim running at 1x or higher speed
- Isaac ROS VSLAM without dropped frames
- Nav2 achieving 95% path execution success rate
- Synthetic data generation of 1000+ frames per session