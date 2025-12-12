# Data Model: The Robotic Nervous System (ROS 2)

## Overview
This document defines the key entities and concepts that form the foundation of the ROS 2 educational content. These represent the core data structures and relationships that students will learn about in the module.

## Core Entities

### 1. ROS 2 Node
**Definition**: A process that performs computation in the ROS 2 system, representing individual robot components or functions.

**Attributes**:
- node_name: string (unique identifier for the node)
- namespace: string (optional namespace for the node)
- parameters: dictionary (configuration values for the node)
- lifecycle_state: enum (created, configuring, configured, activating, active, deactivating, inactive, cleaningup, shutdown, error)

**Relationships**:
- Publishes to zero or more Topics
- Subscribes to zero or more Topics
- Provides zero or more Services
- Uses zero or more Actions

### 2. Topic
**Definition**: A communication channel through which ROS 2 nodes exchange messages using the publisher/subscriber pattern.

**Attributes**:
- topic_name: string (unique identifier for the topic)
- message_type: string (type of message being published)
- qos_profile: object (quality of service settings)
- publishers_count: integer (number of publishers on this topic)
- subscribers_count: integer (number of subscribers to this topic)

**Relationships**:
- Connects one or more Publishers to one or more Subscribers

### 3. Message
**Definition**: Data structure used to exchange information between nodes through topics.

**Attributes**:
- message_type: string (e.g., std_msgs/String, sensor_msgs/LaserScan)
- fields: array of field objects (name, type, default_value)
- timestamp: datetime (when the message was created)

### 4. Service
**Definition**: A synchronous request/response communication pattern between ROS 2 nodes for specific tasks.

**Attributes**:
- service_name: string (unique identifier for the service)
- request_type: string (type of request message)
- response_type: string (type of response message)
- qos_profile: object (quality of service settings)

**Relationships**:
- Has one Service Server
- Used by one or more Service Clients

### 5. Action
**Definition**: An asynchronous communication pattern for long-running tasks with feedback capabilities.

**Attributes**:
- action_name: string (unique identifier for the action)
- goal_type: string (type of goal message)
- result_type: string (type of result message)
- feedback_type: string (type of feedback message)

**Relationships**:
- Has one Action Server
- Used by one or more Action Clients

### 6. URDF Model
**Definition**: An XML representation of a robot's physical structure, joints, and kinematic properties.

**Attributes**:
- robot_name: string (name of the robot)
- links: array of link objects (visual, collision, inertial properties)
- joints: array of joint objects (parent, child, type, limits)
- materials: array of material definitions
- gazebo_extensions: optional gazebo-specific extensions

### 7. rclpy Client Library
**Definition**: Python client library for ROS 2 that enables Python programs to interface with ROS 2 systems.

**Attributes**:
- library_version: string (version of rclpy)
- supported_message_types: array of string (message types supported)
- node_interface: object (methods for creating nodes, topics, services)

## Relationships

```
[Node] 1..* -- 0..* [Topic] (publishes/subscribes)
[Node] 1..* -- 0..* [Service] (provides/uses)
[Node] 1..* -- 0..* [Action] (provides/uses)
[Topic] -- [Message] (uses message type)
[Service] -- [Request/Response] (uses request and response types)
[Action] -- [Goal/Result/Feedback] (uses goal, result, and feedback types)
[URDF Model] -- [Link] (contains)
[URDF Model] -- [Joint] (contains)
```

## State Transitions

### Node Lifecycle States
```
created -> configuring -> configured -> activating -> active
   |           |             |            |           |
   v           v             v            v           v
cleaningup -> shutdown --------------------------------'
   |
   v
error
```

### Action Client States
- PENDING: Goal accepted by server, not yet processed
- EXECUTING: Goal is being processed
- SUCCEEDED: Goal completed successfully
- ABORTED: Goal processing failed
- CANCELED: Goal was canceled

## Validation Rules

1. **Node Naming**: Node names must be unique within a ROS domain
2. **Topic Naming**: Topic names must follow ROS naming conventions (start with / for global, ~ for private)
3. **Message Compatibility**: Publishers and subscribers on the same topic must use compatible message types
4. **Service Availability**: Service clients must verify service availability before making requests
5. **URDF Validity**: URDF models must be well-formed XML and follow URDF schema
6. **Parameter Validation**: Node parameters must be validated against expected types

## Key Behaviors

### Publisher-Subscriber Pattern
1. Publisher creates and publishes messages to a topic
2. Message is delivered to all subscribers of that topic
3. Communication is asynchronous and decoupled

### Service Request-Response Pattern
1. Client sends request to service server
2. Server processes request and sends response
3. Communication is synchronous

### Action Goal-Fulfillment Pattern
1. Client sends goal to action server
2. Server provides feedback during processing
3. Server returns result when goal is completed