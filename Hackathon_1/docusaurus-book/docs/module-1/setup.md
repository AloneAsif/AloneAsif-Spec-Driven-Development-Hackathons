---
title: ROS 2 Environment Setup
sidebar_position: 1
description: Instructions for setting up the ROS 2 development environment for this educational module
---

# ROS 2 Environment Setup

## Prerequisites

Before starting with the ROS 2 educational module, you need to set up your development environment. This guide will walk you through the process of installing ROS 2 and configuring your workspace.

## Supported Platforms

This educational module has been tested on:
- Ubuntu 22.04 (Jammy Jellyfish) with ROS 2 Humble Hawksbill (Recommended)
- Windows 10/11 with WSL2 and Ubuntu 22.04
- macOS (with Docker)

## Installing ROS 2 Humble Hawksbill

### On Ubuntu 22.04

1. **Set up your sources list:**
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 packages:**
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

3. **Install colcon build tool:**
   ```bash
   sudo apt install -y python3-colcon-common-extensions
   ```

4. **Install Python 3 packages:**
   ```bash
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

5. **Initialize rosdep:**
   ```bash
   sudo rosdep init
   rosdep update
   ```

6. **Source the ROS 2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

7. **Add sourcing to your bashrc to make it permanent:**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### On Windows with WSL2

1. Install WSL2 with Ubuntu 22.04 from Microsoft Store
2. Follow the Ubuntu installation instructions above within your WSL2 terminal

### On macOS with Docker

1. Install Docker Desktop for Mac
2. Use the official ROS 2 Docker images for development

## Setting up Your Workspace

1. **Create a workspace directory:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the workspace:**
   ```bash
   colcon build
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Verifying Your Installation

To verify that ROS 2 is properly installed, run:

```bash
ros2 --version
```

You should see the version of ROS 2 installed (e.g., `ros2 version 0.9.8`).

### Testing with a Simple Example

1. **Open a new terminal and source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Run a simple talker node:**
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

3. **In another terminal, source ROS 2 and run a listener:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp listener
   ```

If you see messages being passed between the talker and listener, your installation is working correctly!

## Installing Python Dependencies

For the Python examples in this module, you'll also need:

```bash
pip3 install rclpy
```

Most of the necessary Python packages are included with the desktop installation, but you may need to install additional packages depending on the specific examples you're working with.

## Setting up Your Development Environment

### Recommended IDEs

- **Visual Studio Code** with ROS extension
- **PyCharm** for Python-specific development
- **CLion** for C++ development

### Environment Variables

Some examples may require setting environment variables. You can add these to your `~/.bashrc`:

```bash
# ROS Domain ID (for network isolation)
export ROS_DOMAIN_ID=42

# ROS Localhost Only (for single machine testing)
export ROS_LOCALHOST_ONLY=1
```

## Troubleshooting Common Issues

If you encounter issues during setup, please refer to the troubleshooting section at the end of this module or check the [official ROS 2 documentation](https://docs.ros.org/en/humble/).

## Next Steps

Once you have successfully set up your ROS 2 environment, proceed to the first chapter of this module to begin learning about ROS 2 architecture fundamentals.