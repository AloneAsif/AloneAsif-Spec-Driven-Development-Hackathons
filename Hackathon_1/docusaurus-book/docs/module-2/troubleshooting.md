---
title: Troubleshooting Guide
sidebar_position: 4
---

# Troubleshooting Guide for Digital Twin

This guide provides solutions for common issues encountered when setting up and running the digital twin system with Gazebo, Unity, and ROS 2.

## Common Issues and Solutions

### 1. ROS Communication Issues

**Problem**: ROS nodes cannot communicate between each other.

**Solutions**:
- Verify that the ROS domain ID is the same for all nodes: `echo $ROS_DOMAIN_ID`
- Check that the network is properly configured and ports are not blocked
- Ensure all nodes are using the same RMW (ROS Middleware) implementation
- Verify ROS IP configuration: `echo $ROS_IP`

### 2. Gazebo Simulation Problems

**Problem**: Robot model doesn't appear in Gazebo or physics simulation is incorrect.

**Solutions**:
- Check URDF file for syntax errors: `check_urdf /path/to/robot.urdf`
- Verify Gazebo plugins are properly configured in the URDF
- Ensure required Gazebo models are installed: `gz model --list`
- Check that Gazebo can find your custom models by adding to GAZEBO_MODEL_PATH

### 3. Unity Connection Issues

**Problem**: Unity cannot connect to ROS or messages are not being received/sent.

**Solutions**:
- Verify Unity ROS-TCP Connector is properly installed and configured
- Check that the IP address and port match between Unity and ROS bridge
- Ensure the firewall is not blocking the connection
- Verify Unity is using the correct ROS TCP endpoint settings

### 4. Sensor Data Issues

**Problem**: Sensors are not publishing data or data format is incorrect.

**Solutions**:
- Check Gazebo sensor plugin configuration in URDF
- Verify sensor topics are being published: `ros2 topic list | grep sensor`
- Check sensor data format with: `ros2 topic echo /sensor_topic_name`
- Ensure sensor frame IDs are correctly set

### 5. Performance Problems

**Problem**: Low frame rates in Unity or high latency in simulation.

**Solutions**:
- Reduce simulation complexity (simpler meshes, fewer objects)
- Lower sensor update rates in URDF configuration
- Optimize Unity scene (reduce draw calls, use LODs)
- Check hardware requirements are met

### 6. Joint Control Issues

**Problem**: Robot joints are not responding to commands or moving incorrectly.

**Solutions**:
- Verify joint limits and types in URDF
- Check controller configuration files
- Ensure joint state publisher is running
- Verify command topics are correctly remapped

## Diagnostic Commands

### Check ROS 2 Status
```bash
# List all active topics
ros2 topic list

# Check specific topic
ros2 topic echo /joint_states

# List all nodes
ros2 node list

# Check node graph
rqt_graph
```

### Check Gazebo Status
```bash
# List Gazebo topics
gz topic -l

# Check model status
gz model -m digital_twin_robot -i
```

### Sensor Validation
```bash
# Validate sensor frequencies
python3 simulation/test/validate_sensors.py

# Check integration
python3 simulation/test/integration_test.py
```

## Network Configuration

### ROS Network Setup
If running across multiple machines:
- Set ROS_IP on each machine: `export ROS_IP=<machine_ip>` (Note: Replace `<machine_ip>` with the actual IP address)
- Ensure ROS_DOMAIN_ID is the same: `export ROS_DOMAIN_ID=<domain_number>` (Note: Replace `<domain_number>` with the actual domain ID)
- Check firewall settings allow ROS traffic (typically ports 11311 and up)

### Unity Network Setup
- Ensure Unity and ROS are on the same network or localhost
- Verify port 10000 (default) is available for ROS-TCP connection
- For remote connections, configure firewall to allow the connection

## Hardware Requirements

### Minimum Requirements
- CPU: Quad-core processor (Intel i5 or equivalent)
- RAM: 8 GB
- GPU: OpenGL 3.3 compatible graphics card
- OS: Ubuntu 22.04 or Windows 10/11

### Recommended Requirements
- CPU: Hexa-core processor (Intel i7 or equivalent)
- RAM: 16 GB or more
- GPU: Dedicated graphics card with 4GB+ VRAM
- OS: Ubuntu 22.04 LTS

## Common Error Messages

### "Failed to find match for field"
This usually indicates a message type mismatch. Check:
- Message definitions are correct
- All required packages are installed
- Message dependencies are properly declared

### "Connection refused"
This indicates a network connection issue:
- Check IP addresses and ports
- Verify firewall settings
- Ensure ROS bridge is running

### "Model not found"
For Gazebo model issues:
- Check GAZEBO_MODEL_PATH environment variable
- Verify model files exist in the correct location
- Ensure model.config file is properly configured

## Logging and Debugging

Enable detailed logging for troubleshooting:
```bash
# Enable ROS logging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Check ROS logs
ros2 topic echo /rosout
```

For Unity debugging, check the console for error messages and ensure the ROS connection status is displayed in the UI.