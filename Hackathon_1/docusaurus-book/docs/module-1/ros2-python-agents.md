---
title: Python AI Agent Integration
sidebar_position: 4
description: Learn how to connect Python AI agents to ROS controllers using rclpy to integrate artificial intelligence with robotic control systems
---

# Python AI Agent Integration

## Overview

This chapter focuses on integrating Python-based AI agents with ROS 2 controllers using the `rclpy` library. You'll learn how to bridge the gap between artificial intelligence and robotic control systems, enabling intelligent decision-making and autonomous behavior in robotic applications.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the rclpy library and its role in Python-ROS integration
- Create Python nodes that communicate with ROS 2 systems
- Implement service clients and action clients in Python
- Design AI agents that interact with ROS controllers
- Build practical examples connecting AI agents to ROS systems

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2, providing Python bindings for the ROS 2 client library (rcl). It enables Python programs to interface with ROS 2 systems, allowing Python-based AI agents to communicate with robotic systems built using ROS 2.

### Key Features of rclpy

- **Node Creation**: Create ROS 2 nodes using Python classes
- **Communication Primitives**: Access to publishers, subscribers, services, and actions
- **Message Handling**: Automatic serialization and deserialization of ROS messages
- **Lifecycle Management**: Tools for managing node lifecycle and resources
- **Threading Support**: Concurrency models for handling multiple operations

### Installing rclpy

`rclpy` is included with the standard ROS 2 installation. If you followed the setup instructions in the first chapter, it should already be available:

```bash
# Verify rclpy is available
python3 -c "import rclpy; print('rclpy version:', rclpy.__version__)"
```

## Creating Python Nodes with rclpy

Python nodes in ROS 2 follow the same architectural principles as nodes in other languages. They inherit from the `Node` class and implement the necessary communication interfaces.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class BasicPythonNode(Node):
    def __init__(self):
        # Initialize the parent Node class with a node name
        super().__init__('basic_python_node')

        # Node-specific initialization code goes here
        self.get_logger().info('Basic Python Node initialized')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = BasicPythonNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown when Ctrl+C is pressed
        node.get_logger().info('Node interrupted by user')
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Node with Communication Interfaces

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile

class AdvancedPythonNode(Node):
    def __init__(self):
        super().__init__('advanced_python_node')

        # Create a publisher
        self.publisher = self.create_publisher(
            String,
            'ai_agent_output',
            QoSProfile(depth=10)
        )

        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.robot_status_callback,
            QoSProfile(depth=10)
        )

        # Create a service client
        self.service_client = self.create_client(SetBool, 'task_control')

        # Timer for periodic operations
        self.timer = self.create_timer(1.0, self.periodic_task)

        self.get_logger().info('Advanced Python Node initialized')

    def robot_status_callback(self, msg):
        self.get_logger().info(f'Received robot status: {msg.data}')
        # Process the status and make AI decisions here

    def periodic_task(self):
        # Example: Publish a message periodically
        msg = String()
        msg.data = f'AI Agent Status Update - {self.get_clock().now().nanoseconds}'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPythonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating AI Libraries with ROS

Python's rich ecosystem of AI libraries can be seamlessly integrated with ROS 2 through rclpy. Common AI libraries include:

- **TensorFlow/Keras**: For neural network models
- **PyTorch**: For deep learning applications
- **scikit-learn**: For traditional machine learning
- **OpenCV**: For computer vision tasks
- **NumPy/SciPy**: For numerical computations

### Example: AI Decision Node

```python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscriber for sensor data
        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Publisher for movement commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Simulated AI model parameters (in practice, these would be loaded from a trained model)
        self.model_weights = np.random.rand(4, 2)  # Example: 4 inputs to 2 outputs (linear, angular)

        self.get_logger().info('AI Decision Node initialized')

    def sensor_callback(self, msg):
        # Convert sensor data to numpy array
        sensor_data = np.array(msg.data)

        # Normalize sensor data if needed
        normalized_data = sensor_data / np.max(sensor_data) if np.max(sensor_data) != 0 else sensor_data

        # Make AI decision using simple linear model (in practice, use your trained model)
        outputs = np.dot(normalized_data[:4], self.model_weights)  # Use first 4 sensor values

        # Create movement command based on AI decision
        cmd_msg = Twist()
        cmd_msg.linear.x = float(outputs[0])  # Linear velocity
        cmd_msg.angular.z = float(outputs[1])  # Angular velocity

        # Publish the command
        self.cmd_pub.publish(cmd_msg)

        self.get_logger().info(f'AI Decision: linear={cmd_msg.linear.x:.2f}, angular={cmd_msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AIDecisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('AI Decision Node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services in Python

Services provide synchronous request/response communication between nodes. In Python, you can create both service servers and clients using rclpy.

### Creating a Service Client

A service client sends requests to a service server and waits for a response:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
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
    client = ServiceClientNode()

    # Send a request
    future = client.send_request(1, 2)

    # Wait for the response
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(f'Result: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Server

A service server responds to requests from clients:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = ServiceServerNode()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server interrupted')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Actions in Python

Actions provide asynchronous communication for long-running tasks with feedback. They're ideal for operations like navigation or manipulation.

### Creating an Action Client

An action client can send goals, receive feedback, and get results:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

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
        # Shutdown after receiving the result
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = ActionClientNode()

    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client interrupted')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Server

An action server handles goals from clients and provides feedback:

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
            self.execute_callback
        )

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

## Practical Examples of AI-ROS Integration

### Example 1: Path Planning with AI

AI agents can be used for path planning, obstacle avoidance, and navigation decision-making:

- Subscribe to sensor data (LIDAR, cameras, etc.)
- Process data through AI algorithms
- Publish movement commands to robot controllers

### Example 2: Object Recognition and Manipulation

AI agents can identify objects and coordinate with robotic manipulators:

- Process camera images using computer vision/AI
- Send object coordinates to manipulation controllers
- Receive feedback on manipulation success

### Example 3: Predictive Maintenance

AI agents can analyze sensor data to predict equipment failures:

- Subscribe to multiple sensor streams
- Apply machine learning models for anomaly detection
- Publish alerts or maintenance commands

## Best Practices for AI-ROS Integration

### 1. Efficient Data Handling
- Use appropriate QoS profiles for different types of data
- Implement data buffering and preprocessing
- Consider data compression for large messages

### 2. Error Handling and Robustness
- Implement proper exception handling in AI processing
- Design fallback behaviors when AI fails
- Monitor system resources and performance

### 3. Real-time Considerations
- Separate AI processing from real-time control when possible
- Use threading or separate processes for heavy computations
- Implement timeouts for service and action calls

### 4. Model Deployment
- Optimize AI models for embedded deployment
- Consider model quantization for resource-constrained systems
- Implement model versioning and updates

## Practical Exercises for Python AI Integration

### Exercise 1: Simple AI Controller
Create a Python node that:
- Subscribes to sensor data (e.g., laser scan)
- Implements a simple AI algorithm to determine movement
- Publishes velocity commands to control a robot
- Handles obstacles and navigation

### Exercise 2: Service-Based AI Decision Making
Create an AI node that:
- Provides a service for making decisions based on sensor data
- Implements a machine learning model for classification
- Clients can request decisions by sending sensor readings
- Returns appropriate control commands

### Exercise 3: Action-Based Task Execution
Create an AI node that:
- Implements an action server for high-level tasks
- Uses AI to plan and execute complex behaviors
- Provides feedback during task execution
- Reports results when tasks are completed

### Exercise 4: AI Model Integration
Integrate a pre-trained model with ROS:
- Load a TensorFlow or PyTorch model in your Python node
- Process sensor data through the model
- Use model outputs to control robot behavior
- Implement model updating mechanisms

## Summary

This chapter introduced the integration of Python AI agents with ROS 2 controllers using the rclpy library. We covered the basics of creating Python nodes, connecting them to ROS 2 communication primitives, and integrating AI libraries for intelligent robotic behavior. The examples provided demonstrate how to bridge the gap between artificial intelligence and robotic control systems.

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format) basics for humanoid robots, which is essential for modeling robot structures and joints in ROS 2.

## Navigation

- **Previous**: [ROS 2 Publishing and Subscribing](./ros2-pub-sub.md)
- **Next**: [URDF Basics for Humanoids](./urdf-basics.md)

## Related Chapters

- [Introduction to ROS 2 Architecture](./intro-to-ros2.md) - Fundamental concepts of ROS 2 architecture
- [ROS 2 Publishing and Subscribing](./ros2-pub-sub.md) - Learn how to implement publisher-subscriber communication
- [URDF Basics for Humanoids](./urdf-basics.md) - Create robot models with URDF