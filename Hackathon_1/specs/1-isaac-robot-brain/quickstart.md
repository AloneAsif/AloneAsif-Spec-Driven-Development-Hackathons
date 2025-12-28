# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This quickstart guide provides a step-by-step introduction to setting up and running the NVIDIA Isaac ecosystem for humanoid robot AI development. This guide covers the essential steps to get you started with Isaac Sim, Isaac ROS, and Nav2.

## Prerequisites
- Ubuntu 22.04 LTS
- NVIDIA RTX GPU (RTX 3080 or higher recommended)
- ROS 2 Humble Hawksbill installed
- NVIDIA Omniverse account for Isaac Sim access

## Step 1: Environment Setup

### 1.1 Verify Hardware Requirements
```bash
# Check GPU availability
nvidia-smi

# Verify CUDA installation
nvcc --version

# Check available RAM
free -h
```

### 1.2 Install Isaac Sim
1. Download Isaac Sim from NVIDIA Developer website
2. Extract the package:
```bash
tar -xzf isaac_sim-2023.1.1.tar.gz
cd isaac_sim-2023.1.1
```
3. Run the setup script:
```bash
bash setup.sh --add-python-installation
```
4. Launch Isaac Sim:
```bash
./isaac-sim.sh
```

### 1.3 Install Isaac ROS Packages
```bash
# Add NVIDIA package repository
sudo apt update
sudo apt install -y software-properties-common
wget https://repo.download.nvidia.com/... # (follow NVIDIA ROS2 package instructions)

# Install Isaac ROS packages
sudo apt install -y ros-humble-isaac-ros-* ros-humble-nvblox-*
```

## Step 2: Chapter 1 - Isaac Sim & Synthetic Worlds

### 2.1 Launch Isaac Sim and Load Humanoid Model
1. Open Isaac Sim application
2. Create a new stage (Ctrl+N)
3. Import humanoid USD model:
   - Go to Window > Content Browser
   - Navigate to your humanoid robot USD file
   - Drag and drop into the viewport

### 2.2 Configure Sensors
1. Add RGB camera sensor:
   - Right-click on the humanoid model
   - Select "Add > Camera > RGB Camera"
   - Configure resolution (1920x1080) and frequency (30Hz)

2. Add depth sensor:
   - Right-click on the humanoid model
   - Select "Add > Camera > Depth Camera"
   - Configure depth range and precision

3. Add semantic segmentation:
   - Right-click on the humanoid model
   - Select "Add > Camera > Semantic Segmentation"
   - Assign semantic labels to objects in the scene

### 2.3 Generate Synthetic Dataset
1. Configure data collection:
   - Go to Isaac Sim Extension menu
   - Enable ROS 2 Bridge extension
   - Configure data export paths

2. Run simulation and collect data:
   - Start simulation (Spacebar)
   - Monitor data collection progress
   - Stop when desired dataset size is reached

## Step 3: Chapter 2 - Isaac ROS: Perception & VSLAM

### 3.1 Launch Isaac ROS VSLAM
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /usr/share/isaac_ros_common/setup.sh

# Launch stereo VSLAM
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_proc.launch.py
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py
```

### 3.2 Configure Perception Pipeline
1. Set up stereo camera calibration
2. Configure visual-inertial odometry parameters
3. Adjust mapping and localization settings

### 3.3 Validate Performance
```bash
# Monitor frame rates
ros2 topic hz /rgb/image_rect_color
ros2 topic hz /visual_slam/visual_odometry

# Check for dropped frames
# Ensure consistent 30Hz performance for RGB and 200Hz for odometry
```

## Step 4: Chapter 3 - Nav2 for Humanoid Navigation

### 4.1 Configure Nav2 for Humanoid
```bash
# Create custom Nav2 configuration for humanoid
mkdir -p ~/isaac_ws/src/my_robot_nav2_config
cd ~/isaac_ws/src/my_robot_nav2_config
```

Create custom costmap parameters for bipedal navigation:
```yaml
# costmap_common_params_humanoid.yaml
robot_radius: 0.6  # Larger than typical wheeled robot
footprint: [[-0.5, -0.5], [-0.5, 0.5], [0.8, 0.5], [0.8, -0.5]]  # Rectangular footprint

obstacle_range: 3.0
raytrace_range: 4.0

# Additional layers for humanoid-specific obstacles
inflation_layer:
  enabled: true
  cost_scaling_factor: 5.0
  inflation_radius: 0.8
```

### 4.2 Launch Navigation Stack
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Launch Nav2 with humanoid configuration
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=~/isaac_ws/src/my_robot_nav2_config/nav2_params_humanoid.yaml
```

### 4.3 Test Navigation
1. In RViz, set initial pose for the robot
2. Set navigation goal using the "Nav2 Goal" tool
3. Observe path planning and execution
4. Verify humanoid-appropriate path planning

## Step 5: Integration - Full Navigation Loop

### 5.1 Connect Perception to Navigation
```bash
# Terminal 1: Launch Isaac Sim with ROS bridge
./isaac-sim.sh

# Terminal 2: Launch Isaac ROS perception
source /opt/ros/humble/setup.bash
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py

# Terminal 3: Launch Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Terminal 4: Send navigation commands
ros2 run nav2_msgs send_goal.py
```

### 5.2 Validate Complete Pipeline
1. Verify perception → localization → planning → control loop
2. Check that robot can navigate to goals using visual SLAM
3. Ensure stable performance without dropped frames

## Troubleshooting

### Common Issues
- **Isaac Sim won't start**: Verify GPU drivers and CUDA installation
- **Dropped frames in VSLAM**: Reduce simulation complexity or upgrade GPU
- **Nav2 path planning fails**: Check costmap inflation parameters for humanoid
- **ROS bridge connection fails**: Verify Isaac Sim ROS bridge extension is enabled

### Performance Optimization
- Reduce simulation complexity for real-time performance
- Use lower resolution sensors if needed
- Optimize Nav2 parameters for faster planning

## Next Steps
- Complete hands-on exercises in each chapter
- Experiment with different humanoid models
- Try deployment on Jetson Orin platform
- Explore advanced Isaac Sim features