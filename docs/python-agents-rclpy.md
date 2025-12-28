---
sidebar_position: 2
title: "Python Agents with rclpy"
---

# Python Agents with rclpy: Connecting AI to Robot Controllers

## Introduction

Python is one of the most popular languages for AI development, and rclpy is the Python client library for ROS 2. This chapter explains how to create Python agents that can connect to robot controllers, publish sensor data, and receive control commands within the ROS 2 ecosystem. This connection forms a crucial part of the robotic nervous system that links AI agents with physical robot controllers.

## Setting Up rclpy

To get started with rclpy, you'll need to install the ROS 2 Python client library. If you're using a standard ROS 2 installation, rclpy should already be available. Otherwise, you can install it using:

```bash
pip install rclpy
```

## Creating ROS 2 Nodes in Python

The foundation of any Python agent in ROS 2 is a node. Here's the basic structure:

```python
import rclpy
from rclpy.node import Node

class MyPythonAgent(Node):
    def __init__(self):
        super().__init__('python_agent')
        # Initialize publishers, subscribers, services, etc.
        self.get_logger().info('Python Agent initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishing Sensor Data

One of the most common tasks for AI agents is to publish sensor data that other nodes can use. Here's how to create a publisher:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers for different sensor types
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.laser_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.battery_publisher = self.create_publisher(Float32, 'battery_level', 10)

        # Create a timer to periodically publish data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz

        self.get_logger().info('Sensor Publisher initialized')

    def publish_sensor_data(self):
        # Simulate sensor data publishing
        battery_msg = Float32()
        battery_msg.data = 85.5  # Simulated battery level
        self.battery_publisher.publish(battery_msg)

        self.get_logger().info(f'Published battery level: {battery_msg.data}%')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscribing to Robot Data

Python agents often need to subscribe to data from other nodes to make intelligent decisions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)

        # Subscribe to velocity commands
        self.cmd_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)

        self.get_logger().info('Data Subscriber initialized')

    def joint_callback(self, msg):
        # Process joint state data
        for i, name in enumerate(msg.name):
            position = msg.position[i]
            velocity = msg.velocity[i]
            effort = msg.effort[i]

            # Implement AI logic based on joint data
            if 'head' in name and abs(position) > 1.5:
                self.get_logger().warn(f'{name} near position limit: {position:.2f}')

    def velocity_callback(self, msg):
        # Process velocity commands
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f'Received velocity command: linear={linear_x:.2f}, angular={angular_z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Services

Python agents can also provide services for other nodes to call:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')

        # Create a service to enable/disable the agent
        self.enable_service = self.create_service(
            SetBool,
            'enable_agent',
            self.enable_callback)

        # Create a service to perform calculations
        self.calc_service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

        self.agent_enabled = True
        self.get_logger().info('Service Server initialized')

    def enable_callback(self, request, response):
        self.agent_enabled = request.data
        response.success = True
        response.message = f'Agent {"enabled" if self.agent_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing AI Agents with ROS 2

Here's a more complete example showing how to implement an AI agent that processes sensor data and sends control commands:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class AINavigatorAgent(Node):
    def __init__(self):
        super().__init__('ai_navigator_agent')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_publisher = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.2, self.ai_decision_loop)  # 5Hz

        # Agent state
        self.laser_data = None
        self.obstacle_detected = False
        self.get_logger().info('AI Navigator Agent initialized')

    def scan_callback(self, msg):
        # Store laser scan data for AI processing
        self.laser_data = np.array(msg.ranges)
        # Filter out invalid ranges (inf or nan)
        self.laser_data = self.laser_data[np.isfinite(self.laser_data)]

        # Check for obstacles
        if len(self.laser_data) > 0:
            min_distance = np.min(self.laser_data)
            self.obstacle_detected = min_distance < 0.5  # 50cm threshold

    def ai_decision_loop(self):
        if self.laser_data is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected:
            # Stop and turn to avoid obstacle
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn right
            self.get_logger().warn('Obstacle detected! Turning...')
        else:
            # Move forward
            cmd_msg.linear.x = 0.3  # 0.3 m/s forward
            cmd_msg.angular.z = 0.0

        # Publish command
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AINavigatorAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting AI Libraries

Python's rich AI ecosystem can be easily integrated with ROS 2. Here's an example using TensorFlow for object detection:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import cv2
import tensorflow as tf

class AIVisionAgent(Node):
    def __init__(self):
        super().__init__('ai_vision_agent')

        # Publishers and subscribers
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            'object_detections',
            10)

        # CV bridge to convert ROS images to OpenCV format
        self.cv_bridge = CvBridge()

        # Load pre-trained model (simplified example)
        # self.model = tf.keras.models.load_model('path/to/model')

        self.get_logger().info('AI Vision Agent initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image with AI model (simplified)
            # detections = self.model.predict(cv_image)

            # For this example, we'll simulate detection
            detections = self.simulate_detection(cv_image)

            # Publish detections
            detection_msg = Detection2DArray()
            detection_msg.header = msg.header
            detection_msg.detections = detections
            self.detection_publisher.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def simulate_detection(self, image):
        # Simulated detection for example purposes
        # In real implementation, this would use actual AI model
        height, width = image.shape[:2]

        # Create a simulated detection
        detection = Detection2D()
        detection.bbox.center.x = width / 2
        detection.bbox.center.y = height / 2
        detection.bbox.size_x = 100
        detection.bbox.size_y = 100

        return [detection]

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python Agents

### Error Handling
Always implement proper error handling in your Python agents:

```python
try:
    # ROS operations
    result = some_ros_operation()
except Exception as e:
    self.get_logger().error(f'Operation failed: {e}')
    # Implement fallback behavior
```

### Resource Management
Properly manage resources and clean up when the node is destroyed:

```python
def destroy_node(self):
    # Clean up any resources before destroying the node
    if hasattr(self, 'camera'):
        self.camera.release()
    super().destroy_node()
```

### Threading Considerations
Be aware that rclpy is not thread-safe by default. Use the multi-threaded executor when needed:

```python
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

## Summary

Python agents with rclpy provide a powerful way to connect AI systems with robot controllers. By understanding how to create nodes, publishers, subscribers, and services, you can build sophisticated agents that process sensor data, make intelligent decisions, and control robot behavior. The integration with Python's rich AI ecosystem makes it an excellent choice for implementing the intelligent components of a robotic nervous system.

In the next chapter, we'll explore how to model humanoid robots using URDF (Unified Robot Description Format) and integrate these models with ROS 2.