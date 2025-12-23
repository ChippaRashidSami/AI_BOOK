---
sidebar_position: 2
title: 'ROS 2 Foundations: Nodes, Topics, and Services'
description: 'Understanding the core communication patterns in ROS 2 - nodes, topics, and services'
---

# ROS 2 Foundations: Nodes, Topics, and Services

## Introduction

ROS 2 (Robot Operating System 2) provides the foundational communication infrastructure for humanoid robots, functioning as the nervous system that connects different robot components. This chapter introduces the core communication patterns that make distributed robotic systems possible.

## Nodes: The Building Blocks of ROS 2

Nodes are independent processes that constitute the ROS 2 system and communicate with each other. In a humanoid robot system, nodes might include:

- Sensor nodes (IMU, cameras, LiDAR)
- Actuator nodes (motor controllers)
- Perception nodes (object detection, SLAM)
- Planning nodes (path planning, motion planning)
- Control nodes (walking, manipulation)

Each node is responsible for a specific function and communicates with other nodes through topics and services.

## Topics: Publish/Subscribe Communication

Topics provide a publish/subscribe communication pattern for streaming data between nodes. Key characteristics:

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to the same topic; multiple subscribers can listen to the same topic
- **Data streams**: Ideal for continuous data like sensor readings or motor commands

Example in a humanoid robot:
- IMU node publishes sensor data to `/sensors/imu_raw`
- Multiple nodes (estimation, logging, visualization) can subscribe to this topic simultaneously

## Services: Request/Response Communication

Services provide synchronous request/response communication patterns for discrete interactions. Key characteristics:

- **Synchronous**: The client waits for a response from the server
- **One-to-one**: One service server handles requests from clients
- **Discrete actions**: Ideal for actions with specific results

Example in a humanoid robot:
- A path planning service that accepts a goal location and returns a path
- A calibration service that configures sensor parameters

## Practical Example: Node Communication

Here's a simple example of how nodes might communicate in a humanoid robot system:

1. A vision node publishes detected objects to the `/vision/objects` topic
2. A planning node subscribes to this topic to update its understanding of the environment
3. When the robot needs to navigate, the high-level controller sends a request to the path planning service
4. The path planning service returns a computed path
5. The controller sends motor commands through topics to execute the path

## Benefits of the ROS 2 Architecture

The node-based architecture with topics and services provides several benefits for humanoid robots:

- **Modularity**: Each component can be developed, tested, and updated independently
- **Flexibility**: Components can be swapped or reconfigured without affecting the entire system
- **Scalability**: New components can be added without significant architectural changes
- **Robustness**: Failure of one node doesn't necessarily bring down the entire system

## Summary

ROS 2's node-based communication architecture provides a robust framework for connecting the various components of humanoid robots. Understanding nodes, topics, and services is essential for anyone working with robotic systems, as these concepts form the foundation for all robot communication.