# ROS 2 Concepts Glossary

This glossary defines key terms and concepts used throughout the ROS 2 educational module.

## Core Concepts

### Node
A process that performs computation in the ROS 2 system. Nodes are the fundamental building blocks of a ROS 2 application and represent individual robot components or functions.

### Topic
A communication channel through which ROS 2 nodes exchange messages using publisher/subscriber pattern. Topics enable asynchronous communication between nodes.

### Service
A synchronous request/response communication pattern between ROS 2 nodes for specific tasks. Services provide a way for nodes to request specific actions or information from other nodes.

### Action
An asynchronous communication pattern for long-running tasks with feedback capabilities. Actions are used for tasks that take time to complete and may need to provide status updates.

### DDS (Data Distribution Service)
A middleware protocol and API standard that defines how data is distributed between publishers and subscribers in ROS 2. DDS implementations provide the underlying communication layer for ROS 2.

### rclpy
Python client library for ROS 2 that enables Python programs to interface with ROS 2 systems. rclpy provides the Python API for creating nodes, publishers, subscribers, services, and actions.

### URDF (Unified Robot Description Format)
An XML representation of a robot's physical structure, joints, and kinematic properties. URDF is used to describe robot models in ROS 2.

### Package
A reusable software container in ROS 2 that contains libraries, nodes, and other resources. Packages are the basic building blocks of ROS 2 software organization.

### Workspace
A directory that contains multiple ROS 2 packages and their build artifacts. A workspace is the environment where ROS 2 packages are built and sourced.

### Message
A data structure used to communicate between ROS 2 nodes. Messages are defined in `.msg` files and are used for topic-based communication.

### Interface
The definition of communication types in ROS 2, including messages (`.msg`), services (`.srv`), and actions (`.action`).