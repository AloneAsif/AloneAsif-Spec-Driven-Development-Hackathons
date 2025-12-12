---
title: ROS 2 Publishing and Subscribing
sidebar_position: 3
description: Learn how to implement publishing and subscribing between ROS 2 nodes to create communication pathways between different robot components
---

# ROS 2 Publishing and Subscribing

## Overview

This chapter focuses on the publisher-subscriber pattern in ROS 2, which is the most common communication mechanism in ROS 2 systems. You'll learn how to create publishers that send data and subscribers that receive data, enabling distributed communication between robot components.

## Learning Objectives

After completing this chapter, you will be able to:
- Create ROS 2 publishers to broadcast data to topics
- Create ROS 2 subscribers to receive data from topics
- Understand Quality of Service (QoS) profiles and their impact on communication
- Implement practical examples of publisher-subscriber communication
- Troubleshoot common issues with pub/sub patterns

## The Publisher-Subscriber Pattern

The publisher-subscriber (pub/sub) pattern is the backbone of ROS 2 communication. It enables asynchronous, decoupled communication between nodes:

- **Publishers** send messages to topics without knowing who (if anyone) is listening
- **Subscribers** receive messages from topics without knowing who (if anyone) is publishing
- Communication happens through **topics**, which act as named channels for data transmission

This pattern provides several advantages:
- **Decoupling**: Publishers and subscribers don't need to know about each other
- **Scalability**: Multiple publishers and subscribers can use the same topic
- **Asynchrony**: Communication doesn't block the execution of nodes
- **Flexibility**: Easy to add or remove nodes without affecting others

## Creating a Publisher

To create a publisher in Python, you'll use the `create_publisher()` method of a node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
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
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Parameters

When creating a publisher, you specify three key parameters:
1. **Message type**: The type of message to publish (e.g., `std_msgs.msg.String`)
2. **Topic name**: The name of the topic to publish to (e.g., `'topic'`)
3. **Queue size**: The size of the message queue for outgoing messages

### Advanced Publisher Implementation

For more complex scenarios, you might need to implement additional features in your publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class AdvancedPublisher(Node):
    def __init__(self):
        super().__init__('advanced_publisher')

        # Multiple publishers for different message types
        self.string_publisher = self.create_publisher(String, 'string_topic', 10)
        self.laser_publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_callback)

        # Counter for message sequencing
        self.message_count = 0

        # Example data
        self.angle = 0.0

    def publish_callback(self):
        # Publish string message
        string_msg = String()
        string_msg.data = f'Advanced message #{self.message_count}'
        self.string_publisher.publish(string_msg)

        # Publish laser scan message
        laser_msg = self.generate_laser_scan()
        self.laser_publisher.publish(laser_msg)

        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_msg.angular.z = math.sin(self.angle) * 0.2  # Oscillating turn
        self.cmd_publisher.publish(cmd_msg)

        self.message_count += 1
        self.angle += 0.1

        self.get_logger().info(f'Published advanced messages #{self.message_count}')

    def generate_laser_scan(self):
        """Generate a simulated laser scan message"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate ranges (simulated)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [2.0 + 0.5 * math.sin(self.angle + i * 0.1) for i in range(num_readings)]
        msg.intensities = [100.0] * num_readings

        return msg

def main(args=None):
    rclpy.init(args=args)
    advanced_publisher = AdvancedPublisher()

    try:
        rclpy.spin(advanced_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        advanced_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Best Practices

1. **Proper Error Handling**: Always include try-catch blocks around publish operations in complex scenarios
2. **Message Validation**: Validate message content before publishing to prevent downstream errors
3. **Resource Management**: Properly clean up publishers when the node shuts down
4. **Throttling**: Avoid publishing at rates that overwhelm the system
5. **Topic Naming**: Use descriptive, consistent topic names following ROS conventions

## Creating a Subscriber

Creating a subscriber is similarly straightforward using the `create_subscription()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Parameters

When creating a subscriber, you specify:
1. **Message type**: The type of message to expect
2. **Topic name**: The name of the topic to subscribe to
3. **Callback function**: The function to call when a message is received
4. **Queue size**: The size of the message queue for incoming messages

### Advanced Subscriber Implementation

For more complex scenarios, you might need to implement additional features in your subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import math

class AdvancedSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_subscriber')

        # Create a custom QoS profile for more control
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Multiple subscribers for different message types
        self.string_subscriber = self.create_subscription(
            String,
            'string_topic',
            self.string_callback,
            qos_profile
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            qos_profile
        )

        self.cmd_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            qos_profile
        )

        # Store recent data for analysis
        self.recent_laser_readings = []
        self.cmd_history = []

        # Statistics
        self.message_count = 0

    def string_callback(self, msg):
        self.get_logger().info(f'String message received: "{msg.data}"')
        self.message_count += 1

    def laser_callback(self, msg):
        # Process laser scan data
        min_range = min(msg.ranges) if msg.ranges else float('inf')
        max_range = max(msg.ranges) if msg.ranges else 0.0

        self.get_logger().info(f'Laser scan: min={min_range:.2f}m, max={max_range:.2f}m')

        # Store recent readings for analysis
        self.recent_laser_readings.append({
            'timestamp': self.get_clock().now(),
            'min_range': min_range,
            'max_range': max_range
        })

        # Keep only the last 100 readings
        if len(self.recent_laser_readings) > 100:
            self.recent_laser_readings = self.recent_laser_readings[-100:]

        self.message_count += 1

    def cmd_callback(self, msg):
        # Process velocity command
        self.get_logger().info(
            f'Velocity command: linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f}'
        )

        # Store command in history
        self.cmd_history.append({
            'timestamp': self.get_clock().now(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        })

        # Keep only the last 50 commands
        if len(self.cmd_history) > 50:
            self.cmd_history = self.cmd_history[-50:]

        self.message_count += 1

    def get_statistics(self):
        """Get statistics about received messages"""
        return {
            'total_messages': self.message_count,
            'laser_readings_count': len(self.recent_laser_readings),
            'cmd_history_count': len(self.cmd_history)
        }

def main(args=None):
    rclpy.init(args=args)
    advanced_subscriber = AdvancedSubscriber()

    try:
        rclpy.spin(advanced_subscriber)
    except KeyboardInterrupt:
        stats = advanced_subscriber.get_statistics()
        print(f"Statistics: {stats}")
    finally:
        advanced_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Best Practices

1. **Efficient Callbacks**: Keep callback functions lightweight to avoid blocking message processing
2. **Message Validation**: Always validate incoming message content before using it
3. **Resource Management**: Properly clean up subscribers when the node shuts down
4. **Threading Considerations**: Be aware of threading implications when accessing shared data
5. **QoS Matching**: Ensure subscriber QoS profiles are compatible with publishers

## Quality of Service (QoS) Profiles

QoS profiles allow you to fine-tune the communication behavior between publishers and subscribers. ROS 2 provides several built-in QoS profiles and policies that can be combined to achieve the desired communication characteristics:

### Reliability Policy
- **Reliable**: Ensures all messages are delivered (similar to TCP). The system will retry sending messages until they are acknowledged.
- **Best Effort**: Does not guarantee delivery (similar to UDP). Messages may be lost, but with lower latency.

### Durability Policy
- **Transient Local**: Late-joining subscribers receive the last known value(s) from publishers. Useful for configuration or state topics.
- **Volatile**: No messages are retained for late-joining subscribers. Only new messages sent after the subscriber joins are received.

### History Policy
- **Keep Last**: Store the most recent N messages (where N is specified by the queue depth). Older messages are discarded when the limit is reached.
- **Keep All**: Store all messages in the history queue. Use with caution as this can consume significant memory.

### Lifespan and Deadline Policies
- **Lifespan**: Specifies how long messages remain valid in the system
- **Deadline**: Defines the maximum time between consecutive messages

### Example with Custom QoS

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

# Create a custom QoS profile for critical data
critical_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Create a custom QoS profile for real-time sensor data
sensor_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Create a custom QoS profile for configuration data
config_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Use the custom profiles when creating publishers/subscribers
critical_publisher = node.create_publisher(String, 'critical_topic', critical_qos)
sensor_publisher = node.create_publisher(String, 'sensor_topic', sensor_qos)
config_publisher = node.create_publisher(String, 'config_topic', config_qos)
```

### Common QoS Combinations

1. **Sensor Data**: BEST_EFFORT, VOLATILE, KEEP_LAST (small depth)
2. **Actuator Commands**: RELIABLE, VOLATILE, KEEP_LAST (small depth)
3. **Configuration**: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST (depth=1)
4. **State Information**: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST (small depth)

## Message Types and Serialization

ROS 2 uses a standardized message format for communication between nodes. Messages are defined using `.msg` files and automatically converted to the appropriate programming language structures.

### Standard Message Types

ROS 2 comes with many built-in message types organized in various packages:

- **std_msgs**: Basic data types
  - `Bool`, `Int32`, `Float64`, `String`, `ColorRGBA`, etc.
- **geometry_msgs**: Geometric primitives
  - `Point`, `Pose`, `Twist`, `Vector3`, `Quaternion`, etc.
- **sensor_msgs**: Sensor data formats
  - `LaserScan`, `Image`, `PointCloud2`, `JointState`, etc.
- **nav_msgs**: Navigation-related messages
  - `Odometry`, `Path`, `OccupancyGrid`, etc.
- **action_msgs**: Action-related messages
  - `GoalStatus`, `GoalInfo`, etc.

### Message Structure

A typical message definition looks like this:

```
# Point.msg
float64 x
float64 y
float64 z
```

This generates appropriate structures in Python, C++, etc.:

```python
# Python
from geometry_msgs.msg import Point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.0
```

### Custom Message Types

To create custom messages:
1. Create a `.msg` file in your package's `msg/` directory
2. Define the message fields using standard types
3. Add the message to your package's CMakeLists.txt
4. Build your package to generate the language-specific code

### Serialization

ROS 2 uses a standardized serialization format (currently CDR - Common Data Representation) that ensures compatibility across different programming languages and platforms. The serialization process is handled automatically by the ROS 2 client libraries.

## Message Types

ROS 2 comes with many built-in message types in various packages:

- **std_msgs**: Basic data types (String, Int32, Float64, etc.)
- **sensor_msgs**: Sensor data (LaserScan, Image, PointCloud2, etc.)
- **geometry_msgs**: Geometric primitives (Point, Pose, Twist, etc.)
- **nav_msgs**: Navigation messages (Odometry, Path, OccupancyGrid, etc.)

You can also define custom message types using `.msg` files in your packages.

## Practical Exercise

1. Create a publisher that publishes sensor data (e.g., temperature readings)
2. Create a subscriber that receives and processes this data
3. Experiment with different QoS profiles to see how they affect communication
4. Run multiple publishers and subscribers to see how they interact

## Practical Exercises for Pub/Sub Patterns

### Exercise 1: Sensor Data Publisher
Create a publisher node that simulates a temperature sensor:
- Publish temperature readings every 2 seconds
- Use the `std_msgs.msg.Float64` message type
- Add some random variation to simulate real sensor noise
- Include error handling for invalid readings

### Exercise 2: Data Aggregator Subscriber
Create a subscriber that:
- Subscribes to multiple sensor topics
- Aggregates data from different sensors
- Calculates statistics (min, max, average)
- Publishes the aggregated data to a new topic

### Exercise 3: QoS Experimentation
- Implement the same publisher-subscriber pair with different QoS profiles
- Compare RELIABLE vs BEST_EFFORT delivery
- Compare TRANSIENT_LOCAL vs VOLATILE durability
- Document the differences in behavior

### Exercise 4: Multi-Node Communication
- Create 3 publisher nodes publishing to the same topic
- Create 2 subscriber nodes subscribing to that topic
- Observe how messages are distributed among subscribers
- Analyze the performance impact of multiple nodes

## Troubleshooting Common Pub/Sub Issues

### Issue: Nodes Cannot Communicate
**Symptoms**: Publishers and subscribers on the same topic don't see each other.

**Causes and Solutions**:
- **Different ROS_DOMAIN_ID**: Ensure all nodes are using the same domain ID
- **Network configuration**: Check if nodes are on the same network if running on different machines
- **Topic names**: Verify topic names match exactly (case-sensitive)
- **Message types**: Ensure publisher and subscriber use the same message type

### Issue: Late Joiners Don't Receive Messages
**Symptoms**: Subscribers that start after publishers miss initial messages.

**Solutions**:
- Use TRANSIENT_LOCAL durability for publishers if late joiners need initial messages
- Implement a latching mechanism for critical startup data

### Issue: Message Loss
**Symptoms**: Messages appear to be dropped or not received consistently.

**Causes and Solutions**:
- **Queue size too small**: Increase queue size in publisher/subscriber creation
- **System overload**: Reduce publishing frequency or optimize callback functions
- **QoS mismatch**: Ensure QoS profiles are compatible between publisher and subscriber

### Issue: High Memory Usage
**Symptoms**: Memory consumption increases over time.

**Causes and Solutions**:
- **Large queue sizes**: Reduce queue sizes or implement proper data cleanup
- **Data accumulation**: Implement circular buffers or time-based data expiration
- **Callback processing**: Ensure callbacks complete quickly to prevent queue buildup

## Summary

The publisher-subscriber pattern is fundamental to ROS 2 communication. It enables decoupled, asynchronous communication between nodes, making it ideal for distributed robotic systems. Understanding how to properly implement publishers and subscribers, along with QoS profiles, is crucial for building robust ROS 2 applications.

## Next Steps

In the next chapter, we'll explore how to integrate Python AI agents with ROS controllers using rclpy, bridging artificial intelligence with robotic control systems.

## Navigation

- **Previous**: [Introduction to ROS 2 Architecture](./intro-to-ros2.md)
- **Next**: [Python AI Agent Integration](./ros2-python-agents.md)

## Related Chapters

- [Introduction to ROS 2 Architecture](./intro-to-ros2.md) - Fundamental concepts of ROS 2 architecture
- [Python AI Agent Integration](./ros2-python-agents.md) - Connect Python AI agents to ROS controllers
- [URDF Basics for Humanoids](./urdf-basics.md) - Create robot models with URDF