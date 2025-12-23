---
sidebar_position: 1
---

# ROS 2 Foundations: Nodes, Topics, and Services

## Introduction

This chapter introduces the fundamental concepts of ROS 2 (Robot Operating System 2), which serves as the nervous system for humanoid robots. ROS 2 provides the middleware infrastructure that enables communication, control, and coordination between different components of a robotic system.

## Nodes: The Building Blocks of ROS 2

Nodes are the fundamental building blocks of any ROS 2 system. They represent individual processes that perform specific functions within the robotic system. In the context of humanoid robots, nodes might handle perception, control, planning, or actuation tasks.

### Key Characteristics of Nodes

- **Independence**: Each node operates independently and can be developed, tested, and maintained separately
- **Communication**: Nodes communicate with each other through topics, services, and actions
- **Specialization**: Each node typically specializes in a specific task or function

## Topics: Unidirectional Communication

Topics enable unidirectional, asynchronous communication between nodes. They use a publish-subscribe pattern where nodes publish messages to topics and other nodes subscribe to receive those messages.

### Topic Communication Pattern

- **Publishers**: Nodes that send data to topics
- **Subscribers**: Nodes that receive data from topics
- **Message Types**: Standardized formats for data exchange

## Services: Bidirectional Communication

Services provide synchronous, request-response communication between nodes. They are useful for operations that require a response, such as requesting specific robot actions or querying system status.

### Service Architecture

- **Service Server**: Provides specific functionality to other nodes
- **Service Client**: Requests functionality from a service server
- **Request-Response Cycle**: Synchronous communication pattern

## Actions: Advanced Communication Pattern

Actions extend the service pattern to support long-running operations with feedback and the ability to cancel requests. They are particularly useful for navigation and manipulation tasks in humanoid robots.

## Summary

ROS 2 provides the essential communication infrastructure for humanoid robots, enabling distributed and modular robot software development. Understanding nodes, topics, and services is crucial for developing complex robotic systems.