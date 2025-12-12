---
title: Introduction to ROS 2 Architecture
sidebar_position: 2
description: Learn the fundamental concepts of ROS 2 architecture including nodes, topics, services, and actions
---

# Introduction to ROS 2 Architecture

## Overview

This chapter introduces the fundamental concepts of ROS 2 (Robot Operating System 2) architecture. ROS 2 is a flexible framework for writing robot software that provides a collection of tools, libraries, and conventions for creating robot applications. Understanding its architecture is crucial for developing effective communication systems for humanoid robots.

ROS 2 architecture is designed to support complex, distributed robotic systems. It enables multiple processes (potentially running on different machines) to communicate and coordinate with each other seamlessly. The architecture provides:

- **Process Isolation**: Individual robot functions run in separate processes, so a failure in one component doesn't bring down the entire system
- **Language Independence**: Nodes can be written in different programming languages (C++, Python, etc.) and still communicate effectively
- **Distributed Computing**: Components can run on different machines connected via a network
- **Modularity**: Robot functionality is broken down into reusable, independent components
- **Tool Ecosystem**: Rich set of tools for debugging, visualizing, and managing robotic systems

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the core middleware concepts of ROS 2
- Identify and describe the main architectural components (nodes, topics, services, actions)
- Understand how these components work together to create distributed robotic systems
- Create basic ROS 2 nodes for simple communication tasks

## What is ROS 2?

ROS 2 is the second generation of the Robot Operating System, a flexible framework for writing robot software. Unlike traditional operating systems, ROS 2 is a collection of tools, libraries, and conventions that aim to simplify the development of complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 was designed to address limitations of the original ROS, including:
- Improved real-time support
- Better security features
- Enhanced multi-robot systems support
- Production deployment capabilities
- Platform independence (Linux, Windows, macOS)

## Core Architecture Components

ROS 2 architecture is built around several core concepts that enable distributed computing for robotics applications. These include:

1. **Nodes**: The fundamental computing units that perform computation
2. **Topics**: Communication channels for asynchronous data exchange
3. **Services**: Synchronous request/response communication
4. **Actions**: Asynchronous communication for long-running tasks with feedback
5. **DDS**: Data Distribution Service that provides the underlying communication layer

## Nodes: The Building Blocks of ROS 2

Nodes are the fundamental building blocks of any ROS 2 system. A node is a process that performs computation and typically represents a single function within the robot system. For example, you might have nodes for:

- Sensor data processing
- Motion planning
- Path execution
- User interfaces
- Hardware interfaces

### Node Characteristics

Nodes in ROS 2 have several important characteristics:

- **Process Isolation**: Each node runs in its own process, providing fault tolerance
- **Identity**: Each node has a unique name within the ROS 2 domain
- **Communication Interface**: Nodes contain the interfaces (publishers, subscribers, services, etc.) to communicate with other nodes
- **Lifecycle**: Nodes can have different lifecycle states (unconfigured, inactive, active, finalized)

### Creating a Node

In Python, you create a node by inheriting from `rclpy.node.Node`:

```python
# Example of creating a basic ROS 2 node
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Initialization code here
        self.get_logger().info('Robot controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Management

ROS 2 provides tools to manage nodes:

- **ros2 node list**: Lists all active nodes in the system
- **ros2 node info [node_name]**: Provides detailed information about a specific node
- **Node composition**: Multiple nodes can be combined into a single process for efficiency
- **Lifecycle nodes**: Nodes with explicit state management for complex systems

## Topics: Asynchronous Communication Channels

Topics enable asynchronous communication between nodes using a publisher-subscriber pattern. This is the most common form of communication in ROS 2 systems and is ideal for streaming data like sensor readings, robot states, or command velocities.

### Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic
- **Data-driven**: Communication is based on the data being published rather than specific nodes
- **Type-safe**: Each topic has a specific message type that all publishers and subscribers must adhere to

### Publisher-Subscriber Pattern

In the publisher-subscriber pattern:
- **Publishers** send messages to a topic without knowing who (if anyone) is subscribed
- **Subscribers** receive messages from a topic without knowing who (if anyone) is publishing

### Creating Publishers and Subscribers

Here's an example of creating a publisher and subscriber in Python:

**Publisher Example:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
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
    node = TalkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber Example:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
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
    node = ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS)

ROS 2 provides Quality of Service settings that allow you to fine-tune communication behavior:

- **Reliability**: Choose between reliable (like TCP) or best-effort (like UDP) delivery
- **Durability**: Specify if late-joining subscribers should receive old messages
- **History**: Control how many messages to store
- **Deadline**: Maximum time between consecutive messages

## Services: Synchronous Request/Response Communication

Services provide synchronous request/response communication between nodes. Unlike topics which are asynchronous, services establish a direct connection between a client and a server for immediate communication. This pattern is ideal for operations that require a response, such as triggering an action, getting the current state, or performing a computation.

### Service Characteristics

- **Synchronous**: The client waits for a response from the service server
- **One-to-one**: One client connects to one server at a time
- **Request-Response**: Each request generates exactly one response
- **Type-safe**: Services have specific request and response message types

### Creating Services and Clients

Here's an example of creating a service server and client in Python:

**Service Server Example:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client Example:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    future = minimal_client.send_request(1, 2)

    try:
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### When to Use Services

Use services when you need:
- Immediate response to a request
- Confirmation that an operation was completed
- Synchronous coordination between nodes
- Simple queries or commands that don't require ongoing communication

## Actions: Asynchronous Communication with Feedback

Actions provide asynchronous communication for long-running tasks that may provide feedback and require goal management. They combine features of both topics and services, making them ideal for tasks like navigation, manipulation, or any process that takes time to complete but needs to provide status updates.

### Action Characteristics

- **Asynchronous**: Client doesn't block while the goal is being processed
- **Goal Management**: Supports sending goals, canceling goals, and receiving results
- **Feedback**: Provides ongoing feedback during goal execution
- **Status Updates**: Clients can monitor the status of their goals

### Creating Actions

Here's an example of creating an action server and client in Python:

**Action Server Example:**
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client Example:**
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()

if __name__ == '__main__':
    main()
```

### When to Use Actions

Use actions when you need:
- Long-running operations that provide feedback during execution
- Goal management with the ability to cancel operations
- Asynchronous operations that return a result when completed
- Operations that need to report progress to the client

## DDS (Data Distribution Service)

DDS (Data Distribution Service) is the middleware that underlies ROS 2. It provides a standardized API for machine-to-machine communication and is designed for real-time systems. DDS enables:

- **Data-centricity**: Communication is based on data rather than senders/receivers
- **Discovery**: Automatic discovery of participants in the system
- **Quality of Service (QoS)**: Configurable communication behavior (reliability, durability, etc.)
- **Platform independence**: Works across different operating systems and hardware

DDS implementations like Fast DDS, Cyclone DDS, and RTI Connext DDS provide the actual communication layer for ROS 2.

## Summary

This chapter introduced the fundamental architecture of ROS 2, focusing on its core components and the DDS middleware that enables distributed robotic systems. Understanding these concepts is essential for building effective robot applications that leverage the ROS 2 framework.

Key takeaways from this chapter:
- **Nodes** are the fundamental computing units that perform specific functions in a ROS 2 system
- **Topics** enable asynchronous, many-to-many communication using the publisher-subscriber pattern
- **Services** provide synchronous request/response communication for immediate operations
- **Actions** support long-running tasks with feedback and goal management
- **DDS** serves as the underlying middleware that handles all communication between nodes

These architectural components work together to create a flexible, distributed system for robot software development that supports real-time requirements, fault tolerance, and multi-platform deployment.

## Practical Exercise

1. Create a simple ROS 2 workspace and build it
2. Implement a basic publisher node that publishes messages to a custom topic
3. Create a subscriber node that listens to the same topic and logs the received messages
4. Try running both nodes simultaneously and observe the communication

## Further Reading

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Overview](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)

## Next Steps

In the next chapter, we'll dive deeper into practical implementation by exploring ROS 2 publishing and subscribing patterns, which form the backbone of most ROS 2 communication.

## Navigation

- **Previous**: None
- **Next**: [ROS 2 Publishing and Subscribing](./ros2-pub-sub.md)

## Related Chapters

- [ROS 2 Publishing and Subscribing](./ros2-pub-sub.md) - Learn how to implement publisher-subscriber communication
- [Python AI Agent Integration](./ros2-python-agents.md) - Connect Python AI agents to ROS controllers
- [URDF Basics for Humanoids](./urdf-basics.md) - Create robot models with URDF