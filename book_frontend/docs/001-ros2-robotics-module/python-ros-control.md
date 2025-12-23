---
sidebar_position: 3
title: 'Python Agents and ROS Control with rclpy'
description: 'Using Python to interface with ROS 2 systems through the rclpy client library'
---

# Python Agents and ROS Control with rclpy

## Introduction

Python is one of the most popular languages for robotics and AI development. The rclpy client library enables Python agents to interface with ROS 2 systems, allowing developers to create sophisticated control algorithms and intelligent behaviors for humanoid robots.

## Understanding rclpy

rclpy is the Python client library for ROS 2, providing a Pythonic interface to the ROS 2 ecosystem. It allows Python programs to:

- Create and manage ROS 2 nodes
- Publish and subscribe to topics
- Provide and call services
- Create action clients and servers
- Handle parameters and logging

## Creating a Basic ROS 2 Node with rclpy

Here's a simple example of a Python node using rclpy:

```python
import rclpy
from rclpy.node import Node

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        # Node initialization code here
        self.get_logger().info('Simple Robot Controller initialized')

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishing Messages

Python agents can publish messages to topics using rclpy publisher objects:

```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class MessagePublisher(Node):
    def __init__(self):
        super().__init__('message_publisher')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
```

## Subscribing to Topics

Python agents can subscribe to topics to receive data from other nodes:

```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class MessageSubscriber(Node):
    def __init__(self):
        super().__init__('message_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Implementing Services

Python agents can provide services for other nodes to call:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response
```

## Calling Services

Python agents can also call services provided by other nodes:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        return self.cli.call_async(request)
```

## Applications in Humanoid Robotics

Python and rclpy are particularly valuable for humanoid robotics in several areas:

### AI and Machine Learning Integration
Python's rich ecosystem of ML libraries (TensorFlow, PyTorch, scikit-learn) can be seamlessly integrated with ROS 2, enabling AI-powered robot behaviors.

### Rapid Prototyping
Python's interpreted nature and high-level abstractions make it ideal for quickly prototyping robot behaviors and algorithms.

### State Machines and Behavior Trees
Python can implement complex decision-making logic for humanoid robots, coordinating between different modules and behaviors.

### Simulation Integration
Python agents can easily interact with simulation environments, making it easier to test and develop robot behaviors before deployment on real hardware.

## Best Practices

When developing Python agents for ROS 2:

- Use appropriate QoS settings for different types of data
- Implement proper error handling for network interruptions
- Use parameters for configurable behaviors
- Follow ROS 2 naming conventions
- Consider threading requirements for complex algorithms
- Use lifecycle nodes for better resource management

## Summary

Python agents using rclpy provide a powerful way to implement intelligent behaviors for humanoid robots. The combination of Python's simplicity and ROS 2's communication infrastructure enables developers to create sophisticated robotic applications with minimal overhead.