# Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Welcome to the AI-Robot Brain

This module focuses on NVIDIA's Isaac ecosystem for developing the "brain" of humanoid robots. You'll learn to create photorealistic simulations, deploy GPU-accelerated perception systems, and configure navigation for bipedal robots.

## Learning Path

Follow this sequence to master the Isaac ecosystem:

### 1. Isaac Sim & Synthetic Worlds ([Isaac Sim Overview](./isaac-sim-overview.md))
- **Objective**: Master photorealistic humanoid simulation
- **Topics Covered**:
  - Isaac Sim installation and configuration
  - Omniverse and USD fundamentals
  - Humanoid model loading and physics
  - Sensor configuration (RGB, depth, segmentation)
  - Synthetic dataset generation
- **Outcome**: Students can create synthetic datasets for perception tasks

### 2. Isaac ROS: Perception & VSLAM ([Isaac ROS Perception](./isaac-ros-perception.md))
- **Objective**: Deploy GPU-accelerated perception and localization
- **Topics Covered**:
  - Isaac ROS architecture and components
  - GPU-accelerated VSLAM deployment
  - Camera and IMU integration
  - Mapping vs localization workflows
  - Jetson deployment procedures
- **Outcome**: Students can deploy Isaac ROS for accelerated VSLAM and navigation

### 3. Nav2 for Humanoid Navigation ([Nav2 for Humanoids](./nav2-humanoid-navigation.md))
- **Objective**: Configure navigation for bipedal locomotion
- **Topics Covered**:
  - Nav2 stack overview for humanoid navigation
  - Costmap configuration for bipeds
  - Path planning with step constraints
  - Velocity smoothing for stability
  - Integration with Isaac ROS localization
- **Outcome**: Students can configure Nav2 for humanoid-compatible path planning

### 4. Complete Integration ([System Integration](./system-integration.md))
- **Objective**: Connect perception → localization → planning → control
- **Topics Covered**:
  - Isaac Sim ↔ ROS 2 bridge configuration
  - Isaac ROS ↔ Nav2 data flow integration
  - Full navigation loop testing
  - Performance optimization
- **Outcome**: Students can integrate perception, localization, planning, and control

## Prerequisites

Before starting this module, you should have:
- Completed [Module 1: ROS 2 fundamentals](../module-1/intro-to-ros2.md)
- Completed [Module 2: Gazebo & Unity digital twins](../module-2/building-digital-twin.md)
- Access to Ubuntu 22.04 environment
- RTX-capable GPU for Isaac Sim
- ROS 2 Humble installation

## System Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA RTX (RTX 3080 or higher recommended)
- **RAM**: 16GB+ (32GB recommended)
- **Storage**: 50GB+ free space
- **ROS**: ROS 2 Humble Hawksbill
- **NVIDIA Drivers**: Compatible with your RTX GPU

## Getting Started

- **Start with Isaac Sim**: Begin with the [Isaac Sim Overview](./isaac-sim-overview.md) to set up your simulation environment
- **Move to Perception**: Continue with [Isaac ROS Perception](./isaac-ros-perception.md) to understand perception systems
- **Configure Navigation**: Learn navigation with [Nav2 for Humanoids](./nav2-humanoid-navigation.md)
- **Integrate Everything**: Complete the module with [System Integration](./system-integration.md)

## Learning Outcomes

By the end of this module, you will be able to:
- Explain the role of Isaac Sim in sim-to-real robotics pipelines
- Generate synthetic datasets for perception tasks using Isaac Sim
- Deploy Isaac ROS for accelerated VSLAM and navigation
- Configure Nav2 for humanoid-compatible path planning
- Integrate perception → localization → planning → control systems

## Additional Resources

- [NVIDIA Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac Sim User Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [ROS 2 Navigation (Nav2) Tutorials](https://navigation.ros.org/)
- [NVIDIA Developer Resources](https://developer.nvidia.com/)

## Troubleshooting

If you encounter issues:
- Verify your hardware meets the requirements
- Check that Isaac Sim is properly installed
- Ensure ROS 2 Humble is correctly configured
- Consult the troubleshooting sections in each chapter
- Review the [prerequisites](./prerequisites.md) and [configuration templates](./configuration-templates.md)

Start your journey into AI-powered humanoid robotics with the [Isaac Sim Overview](./isaac-sim-overview.md)!