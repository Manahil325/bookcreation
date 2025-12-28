---
sidebar_position: 1
title: "ROS 2 Fundamentals"
---

# ROS 2 Fundamentals: Building the Nervous System for Humanoid Robots

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. For humanoid robots, ROS 2 serves as the "nervous system" that connects sensors, actuators, and high-level AI systems.

## Core Concepts

### Nodes
Nodes are the fundamental building blocks of ROS 2. Each node is a process that performs a specific task, such as controlling a sensor, processing data, or executing a behavior. In a humanoid robot, you might have nodes for:

- Joint controllers
- Sensor data processing
- Perception systems
- Motion planning
- High-level decision making

### Topics, Services, and Actions

ROS 2 provides three main communication patterns:

#### Topics (Publish/Subscribe)
Topics enable asynchronous communication between nodes using a publish/subscribe model. Data flows from publishers to subscribers through named topics. This pattern is ideal for continuous data streams like:

- Sensor data (camera images, LIDAR scans, IMU readings)
- Joint states
- Robot poses
- Health monitoring data

#### Services (Request/Response)
Services provide synchronous communication using a request/response model. A client sends a request and waits for a response. This pattern is suitable for:

- Configuration changes
- Triggering specific actions
- Requesting computation results
- Calibration procedures

#### Actions
Actions combine the best of both worlds, providing asynchronous communication with feedback and goal management. They're perfect for long-running tasks like:

- Navigation goals
- Manipulation tasks
- Trajectory execution
- Complex behaviors

## The DDS Communication Model

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware. DDS provides:

- **Real-time performance**: Deterministic message delivery with configurable QoS (Quality of Service) settings
- **Distributed architecture**: Nodes can run on different machines and communicate seamlessly
- **Reliability**: Multiple QoS policies for different communication needs (reliable vs. best-effort, transient vs. volatile)

### Quality of Service (QoS) Settings

QoS settings allow you to fine-tune communication behavior:

- **Reliability**: Reliable (all messages delivered) or Best Effort (messages may be dropped)
- **Durability**: Transient Local (late-joining subscribers get initial data) or Volatile (only new data sent)
- **History**: Keep All (store all messages) or Keep Last (store only recent messages)

## ROS 2 in Humanoid Robot Systems

In humanoid robots, ROS 2 serves as the integration framework that connects:

### Perception Systems
- Computer vision nodes processing camera data
- SLAM (Simultaneous Localization and Mapping) systems
- Object detection and recognition
- Human-robot interaction interfaces

### Control Systems
- Joint position, velocity, and effort controllers
- Balance and stabilization algorithms
- Walking pattern generators
- Trajectory planners and executors

### AI and Decision Making
- High-level task planners
- Learning and adaptation systems
- Natural language processing
- Behavioral state machines

## Practical Example: Creating a Simple ROS 2 Node

Here's a basic example of a ROS 2 node that could be part of a humanoid robot's nervous system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)
        self.get_logger().info('Joint Monitor Node initialized')

    def joint_callback(self, msg):
        # Process joint state information
        for i, name in enumerate(msg.name):
            if 'leg' in name:  # Focus on leg joints for balance
                position = msg.position[i]
                # Implement balance logic based on joint positions
                self.get_logger().info(f'{name} position: {position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JointMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 provides the communication infrastructure that allows humanoid robots to function as integrated systems. By understanding nodes, topics, services, actions, and the DDS communication model, you can design effective robotic nervous systems that connect AI agents with physical robot controllers.

In the next chapter, we'll explore how to implement Python agents that connect to this ROS 2 infrastructure using the rclpy library.