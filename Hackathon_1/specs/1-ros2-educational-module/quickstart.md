# Quickstart Guide: The Robotic Nervous System (ROS 2)

## Overview
This quickstart guide provides a rapid introduction to ROS 2 concepts and how to follow along with the educational modules. It's designed to get students up and running quickly with the examples in the course.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic command-line knowledge
- Basic Python programming knowledge

## Environment Setup

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 3. Install Python Dependencies
```bash
pip3 install rclpy
```

## Basic ROS 2 Commands

### Check ROS 2 Installation
```bash
ros2 --version
```

### List Available Topics
```bash
ros2 topic list
```

### List Available Services
```bash
ros2 service list
```

## Creating Your First Node

### 1. Create a Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Simple Publisher Example
Create a file called `talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Simple Subscriber Example
Create a file called `listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Examples

### 1. Terminal 1 - Start the Publisher
```bash
cd ~/ros2_ws
source install/setup.bash
python3 talker.py
```

### 2. Terminal 2 - Start the Subscriber
```bash
cd ~/ros2_ws
source install/setup.bash
python3 listener.py
```

You should see the publisher sending messages and the subscriber receiving them.

## Key Concepts to Remember

1. **Nodes**: Independent processes that perform computation
2. **Topics**: Communication channels for publisher-subscriber pattern
3. **Messages**: Data structures passed between nodes
4. **rclpy**: Python client library for ROS 2
5. **Package**: Organized collection of ROS 2 functionality

## Troubleshooting

### Common Issues and Solutions

1. **"Command 'ros2' not found"**
   - Ensure ROS 2 is installed and sourced
   - Run: `source /opt/ros/humble/setup.bash`

2. **Nodes can't communicate**
   - Check that both terminals have sourced the ROS 2 environment
   - Verify both nodes are on the same ROS_DOMAIN_ID

3. **Python import errors**
   - Ensure rclpy is installed: `pip3 install rclpy`
   - Make sure the ROS 2 environment is sourced

## Next Steps

After completing this quickstart, you'll be ready to dive deeper into the three main chapters:
1. ROS 2 basics (Nodes, Topics, Services, Actions, DDS)
2. Python Agents with rclpy (agent → controller bridge)
3. URDF basics for humanoids (links, joints, simple example)