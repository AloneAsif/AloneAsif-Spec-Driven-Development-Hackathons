# Isaac Sim Installation Process for Ubuntu 22.04

## Prerequisites

Before installing Isaac Sim, ensure you have:
- Ubuntu 22.04 LTS
- NVIDIA RTX GPU (RTX 3080 or higher recommended)
- NVIDIA drivers (535 or higher) installed
- CUDA 11.8+ installed
- 50GB+ free disk space
- NVIDIA Omniverse account

## Step 1: System Preparation

### Verify Hardware and Software Requirements
```bash
# Check GPU availability
nvidia-smi

# Verify CUDA installation
nvcc --version

# Check available disk space
df -h

# Check Ubuntu version
lsb_release -a
```

### Install Additional Dependencies
```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep
pip3 install numpy transforms3d
```

## Step 2: Download Isaac Sim

### Option 1: Download from NVIDIA Developer Website
- Go to https://developer.nvidia.com/isaac-sim
- Sign in with your NVIDIA account
- Download the latest version for Linux
- Extract the downloaded archive:

```bash
tar -xzf isaac_sim-2023.1.1.tar.gz
cd isaac_sim-2023.1.1
```

### Option 2: Using Omniverse Launcher (Alternative)
If you prefer using the Omniverse Launcher:
- Download the Omniverse Launcher from https://www.nvidia.com/en-us/omniverse/download/
- Install and launch the Omniverse Launcher
- Sign in with your NVIDIA account
- Install Isaac Sim through the launcher

## Step 3: Install Isaac Sim

### Run the Setup Script
```bash
# Run the setup script to install Isaac Sim
bash setup.sh --add-python-installation
```

This script will:
- Install Isaac Sim to the current directory
- Set up Python environments
- Configure necessary paths and dependencies

### Verify Installation
```bash
# Test that Isaac Sim can be launched
./isaac-sim.sh --version
```

## Step 4: Configure Isaac Sim

### Set Up Environment Variables
Add the following to your `~/.bashrc` file:

```bash
# Isaac Sim Environment Variables
export ISAACSIM_PATH="/path/to/your/isaac_sim-2023.1.1"
export ISAACSIM_PYTHON_PATH="/path/to/your/isaac_sim-2023.1.1/python.sh"
export ISAACSIM_NUCLEUS_SERVERS="http://localhost:8120"
export ISAACSIM_TOKEN_SERVER="http://localhost:8121"
```

Replace `/path/to/your/isaac_sim-2023.1.1` with your actual installation path.

### Source the Environment
```bash
source ~/.bashrc
```

## Step 5: Launch Isaac Sim

### Basic Launch
```bash
# Launch Isaac Sim with default settings
./isaac-sim.sh
```

### Launch with Specific Parameters
```bash
# Launch without GUI (for headless operation)
./isaac-sim.sh --no-window --/renderer/enabled=False

# Launch with specific stage
./isaac-sim.sh --summary --/renderer/enabled=False --exec "omni.kit.quick_start"
```

## Step 6: Initial Configuration

### Enable Required Extensions
When Isaac Sim first launches:
1. Go to Window > Extensions
2. Enable the following extensions:
   - Isaac Sim ROS2 Bridge
   - Isaac Sim Sensors
   - Isaac Sim Navigation
   - Isaac Sim Manipulation

### Set Up User Preferences
1. Go to Window > Preferences
2. Under Isaac Sim, set:
   - Default stage: Choose appropriate template
   - Physics: Set gravity and solver parameters
   - Rendering: Adjust quality settings based on your GPU

## Troubleshooting Installation Issues

### GPU Not Detected
- Ensure NVIDIA drivers are properly installed: `nvidia-smi`
- Check if secure boot is disabled in BIOS
- Verify CUDA installation: `nvcc --version`

### Permission Issues
```bash
# Fix permissions if needed
chmod +x isaac-sim.sh
chmod +x python.sh
```

### Missing Dependencies
```bash
# Install additional dependencies if needed
sudo apt install -y libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1
```

### OpenGL Issues
If you encounter rendering issues:
```bash
# Check OpenGL support
glxinfo | grep -i opengl

# Install Mesa libraries if needed
sudo apt install -y mesa-utils libgl1-mesa-glx libgl1-mesa-dri
```

## Verification Steps

### Test Basic Functionality
1. Launch Isaac Sim: `./isaac-sim.sh`
2. Create a new stage (Ctrl+N)
3. Add a simple primitive (Cube, Sphere, etc.)
4. Verify rendering works properly
5. Test physics by enabling physics and dropping objects

### Test ROS 2 Bridge
1. Enable the Isaac Sim ROS2 Bridge extension
2. Check that ROS 2 topics can be published/subscribed
3. Verify that Isaac Sim can connect to ROS 2 network

## Next Steps

After successful installation:
1. Proceed to [Omniverse and USD Fundamentals](./omniverse-usd-fundamentals.md)
2. Learn about [Humanoid Model Loading](./humanoid-model-loading.md)
3. Configure [Physics and Lighting](./physics-lighting-config.md)

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac ROS Integration Guide](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_bridges/isaac_ros_cumotion/index.html)
- [Troubleshooting Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/programming_guide/troubleshooting.html)