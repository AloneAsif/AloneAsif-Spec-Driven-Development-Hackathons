# Quickstart Guide: Module 2: The Digital Twin

## Overview
This guide provides a quick setup process to get the physics-accurate and visually rich digital twin running with Gazebo, Unity, and ROS 2 integration.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or Windows 10/11
- 8GB+ RAM (16GB recommended for optimal performance)
- Multi-core processor (4+ cores recommended)
- Dedicated GPU with OpenGL 4.1+ support (for Unity)
- 20GB+ free disk space

### Software Dependencies
- ROS 2 Humble Hawksbill (or later)
- Gazebo Harmonic
- Unity 2022.3 LTS (or later)
- Git
- Python 3.11+
- C# development tools (Visual Studio, Visual Studio Code, or Rider)

## Installation Steps

### 1. Install ROS 2 Humble Hawksbill

For Ubuntu:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-build

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo Harmonic

For Ubuntu:
```bash
sudo apt install -y gz-harmonic
# Or install specific packages:
sudo apt install -y libgz-sim8-dev libgz-common5-dev libgz-physics7-dev
```

### 3. Install Unity Hub and Unity Editor

1. Download Unity Hub from the [Unity website](https://unity.com/download)
2. Install Unity Hub and log in with your Unity account
3. Install Unity 2022.3 LTS or later
4. Install the following modules during installation:
   - Android Build Support (if targeting Android)
   - iOS Build Support (if targeting iOS)
   - Visual Studio Tools for Unity

### 4. Set up the Project Structure

```bash
# Create workspace directory
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Create simulation directories
mkdir -p simulation/gazebo/models
mkdir -p simulation/gazebo/worlds
mkdir -p simulation/gazebo/launch
mkdir -p simulation/unity/Assets
mkdir -p simulation/ros2/src

# Clone necessary repositories
cd ~/digital_twin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b humble
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

### 5. Install Unity ROS-TCP Connector

1. Open Unity Hub and create a new 3D project
2. In the Unity Editor, go to Window → Package Manager
3. Click the "+" button → Add package from git URL
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`
5. Install the package

### 6. Build the ROS-TCP Endpoint

```bash
cd ~/digital_twin_ws/src/ROS-TCP-Endpoint
pip3 install -r requirements.txt

# Run the endpoint server
python3 main.py
```

## Basic Configuration

### 1. Create a Basic Robot URDF

Create `~/digital_twin_ws/simulation/gazebo/models/my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Sensor Example -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>
</robot>
```

### 2. Create a Gazebo World

Create `~/digital_twin_ws/simulation/gazebo/worlds/simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your robot -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### 3. Unity Scene Setup

1. Create a new scene in Unity
2. Add the ROSConnectionManager prefab from the ROS-TCP Connector package
3. Create a GameObject to represent your robot
4. Add a script to handle ROS communication

Example C# script (`RobotController.cs`):

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string cmdVelTopic = "cmd_vel";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
    }

    // Update is called once per frame
    void Update()
    {
        // Send a command to move the robot forward
        if (Input.GetKeyDown(KeyCode.Space))
        {
            var twist = new TwistMsg();
            twist.linear = new Vector3Msg(1.0f, 0, 0); // Move forward
            twist.angular = new Vector3Msg(0, 0, 0);

            ros.Publish(cmdVelTopic, twist);
        }
    }
}
```

## Running the Digital Twin

### 1. Launch Gazebo Simulation

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/digital_twin_ws/install/setup.bash

# Launch Gazebo with your world
gz sim -r simple_world.world
```

### 2. Launch ROS Bridge

```bash
# Terminal 1: Start the ROS TCP endpoint
cd ~/digital_twin_ws/src/ROS-TCP-Endpoint
python3 main.py

# Terminal 2: Launch your robot controller
cd ~/digital_twin_ws
source install/setup.bash
ros2 run your_package robot_controller_node
```

### 3. Run Unity Scene

1. Open your Unity project
2. Load the scene with your robot
3. Press Play in the Unity Editor
4. The Unity scene should now receive data from Gazebo and send commands back

## Testing the Setup

### 1. Verify Sensor Data

Check if sensor data is being published:

```bash
# Listen to laser scan data
ros2 topic echo /scan sensor_msgs/msg/LaserScan

# Listen to camera data
ros2 topic echo /camera/image_raw sensor_msgs/msg/Image

# Listen to IMU data
ros2 topic echo /imu sensor_msgs/msg/Imu
```

### 2. Send Control Commands

Test sending commands to the robot:

```bash
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

### 3. Verify Unity Communication

In Unity, verify that:
- Sensor data is being received and visualized
- Control commands from Unity are affecting the Gazebo simulation
- The robot's state in Unity matches the state in Gazebo

## Troubleshooting

### Common Issues

1. **ROS Connection Issues**
   - Verify ROS TCP endpoint is running
   - Check that Unity and ROS are on the same network/IP
   - Confirm topic names match between systems

2. **Gazebo Not Loading URDF**
   - Check URDF syntax with `check_urdf`
   - Verify Gazebo can find model files in `GAZEBO_MODEL_PATH`

3. **Unity Performance Issues**
   - Reduce scene complexity
   - Check Unity rendering settings
   - Verify GPU drivers are up to date

### Useful Commands

```bash
# Check available ROS topics
ros2 topic list

# Check available ROS services
ros2 service list

# Visualize TF frames
rviz2

# Check URDF file
check_urdf path/to/your/robot.urdf
```

## Next Steps

1. Follow the detailed documentation in the Docusaurus site under `/docs/module-2/`
2. Explore advanced sensor configurations
3. Implement humanoid joint control
4. Add physics-based interactions
5. Optimize for real-time performance