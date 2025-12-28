# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview
This guide will help you get started with the Vision-Language-Action (VLA) system that unifies perception, language, and action for humanoid robots. The system converts voice commands into structured goals using OpenAI Whisper and LLMs for task planning, executes plans using ROS 2, and integrates vision feedback for validation.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended for ROS 2 Humble Hawksbill)
- Python 3.10 or 3.11
- At least 8GB RAM
- Modern CPU with support for hardware acceleration (for vision processing)

### Software Dependencies
1. **ROS 2 Humble Hawksbill** - Robot Operating System
2. **Gazebo** - Simulation environment
3. **OpenAI API key** (for Whisper and LLMs) or local models
4. **Docker** (optional, for containerized deployment)

## Installation

### 1. Setup ROS 2 Environment
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -gpg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 2. Clone the Repository
```bash
git clone https://github.com/your-org/vla-module.git
cd vla-module
```

### 3. Setup Python Environment
```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### 4. Install Additional Dependencies
```bash
# Install OpenAI library for Whisper and LLM integration
pip install openai

# Install computer vision libraries
pip install opencv-python torch torchvision

# Install ROS 2 Python interfaces
pip install ros2launch
```

## Configuration

### 1. Environment Variables
Create a `.env` file in the project root:
```bash
# OpenAI API key for Whisper and LLM services
OPENAI_API_KEY=your_openai_api_key_here

# ROS 2 configuration
ROS_DOMAIN_ID=42
ROS_LOG_DIR=/tmp/ros_logs

# Simulation parameters
SIMULATION_SPEED=1.0  # Real-time factor
```

### 2. VLA System Configuration
The system is configured through `config/vla_config.yaml`:
```yaml
voice:
  whisper_model: "base"  # Options: tiny, base, small, medium, large
  language: "en"
  sample_rate: 16000

llm:
  provider: "openai"  # Options: openai, anthropic, local
  model: "gpt-4-turbo"  # Model for task planning
  temperature: 0.3

vision:
  detection_model: "yolov8n.pt"  # Object detection model
  confidence_threshold: 0.5
  image_topic: "/camera/rgb/image_raw"

ros2:
  navigation_action: "/navigate_to_pose"
  manipulation_action: "/manipulation_controller"
  timeout: 30.0
```

## Running the System

### 1. Start the Simulation Environment
```bash
# In one terminal, start Gazebo simulation
source /opt/ros/humble/setup.bash
cd vla-module
ros2 launch simulation.launch.py
```

### 2. Start the VLA Pipeline
```bash
# In another terminal, start the VLA system
source vla_env/bin/activate
cd vla-module
python src/vla_pipeline.py
```

### 3. Send a Voice Command
The system provides multiple ways to input commands:

#### Option A: Audio File
```bash
curl -X POST http://localhost:8000/voice/commands \
  -H "Content-Type: multipart/form-data" \
  -F "audio=@command.wav"
```

#### Option B: Text Command (for testing)
```bash
curl -X POST http://localhost:8000/planning/generate \
  -H "Content-Type: application/json" \
  -d '{"command": "Navigate to the kitchen and pick up the red cup"}'
```

### 4. Monitor Execution
```bash
# Check the status of a task plan
curl http://localhost:8000/planning/plans/{plan_id}

# Monitor action execution
curl http://localhost:8000/execution/status/{sequence_id}
```

## Example Workflows

### Basic Navigation Command
1. User says: "Go to the kitchen"
2. Whisper transcribes to: "Go to the kitchen"
3. LLM generates navigation task
4. ROS 2 executes navigation action
5. Vision system confirms arrival at kitchen

### Complex Task Command
1. User says: "Clean the room by picking up the books and putting them on the shelf"
2. Whisper transcribes the command
3. LLM generates multi-step plan:
   - Navigate to room
   - Detect books
   - Pick up each book
   - Navigate to shelf
   - Place books on shelf
4. Each step is executed with vision feedback
5. System confirms task completion

## API Endpoints

### Voice Commands
- `POST /voice/commands` - Submit a voice command
- `GET /voice/commands/{commandId}` - Get command status

### Task Planning
- `POST /planning/generate` - Generate a task plan from text
- `GET /planning/plans/{planId}` - Get plan details

### Execution
- `POST /execution/execute` - Execute a task plan
- `GET /execution/status/{sequenceId}` - Get execution status

### Vision
- `POST /vision/detect` - Perform object detection
- `POST /feedback/validate` - Validate task completion with vision

## Troubleshooting

### Common Issues

1. **"No audio input detected"**
   - Check that microphone permissions are granted
   - Verify audio input device is selected correctly

2. **"LLM response timeout"**
   - Check internet connection
   - Verify API key is valid and has sufficient credits

3. **"ROS 2 nodes not connecting"**
   - Ensure ROS_DOMAIN_ID is consistent across all terminals
   - Check that ROS 2 network is properly configured

### Debugging
Enable verbose logging by setting:
```bash
export VLA_LOG_LEVEL=DEBUG
```

## Next Steps

1. Explore the detailed documentation in the `docusaurus-book/docs/module-4/` directory
2. Try the interactive tutorials in the `examples/` directory
3. Customize the system for your specific use case
4. Contribute to the project by reporting issues or submitting pull requests