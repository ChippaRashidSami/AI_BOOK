---
sidebar_position: 3
---

# Humanoid Modeling with URDF

## Introduction

URDF (Unified Robot Description Format) is the standard format for describing robot models in ROS. For humanoid robots, URDF defines the physical structure, kinematic properties, and visual appearance of the robot. This chapter covers how to model humanoid robots using URDF.

## Understanding URDF

URDF is an XML-based format that describes robot models in terms of links and joints. For humanoid robots, this includes the body, head, arms, and legs with appropriate joints to enable human-like movement.

### Key Components of URDF

- **Links**: Rigid parts of the robot (e.g., torso, limbs)
- **Joints**: Connections between links that enable movement
- **Visual**: Visual representation for simulation and visualization
- **Collision**: Collision properties for physics simulation

## Link Elements

Links represent rigid bodies in the robot. For humanoid robots, these include the torso, head, arms, and legs.

### Link Structure

```xml
<link name="link_name">
  <inertial>
    <!-- Mass, center of mass, and inertia properties -->
  </inertial>
  <visual>
    <!-- Visual mesh or primitive shape -->
  </visual>
  <collision>
    <!-- Collision mesh or primitive shape -->
  </collision>
</link>
```

## Joint Elements

Joints connect links and define the degrees of freedom between them. Humanoid robots require various joint types to achieve human-like movement.

### Joint Types for Humanoids

- **Revolute**: Rotational joints with limited range (e.g., elbow, knee)
- **Continuous**: Rotational joints without limits (e.g., shoulder, hip)
- **Fixed**: Rigid connections (e.g., camera mounts)

## Humanoid Kinematic Chain

Humanoid robots have a specific kinematic structure that must be properly represented in URDF to enable correct movement and control.

### Kinematic Tree Structure

- **Base Link**: Root of the kinematic tree (typically torso)
- **Branches**: Arms and legs branching from the torso
- **End Effectors**: Hands and feet at the end of kinematic chains

## Materials and Colors

URDF supports material definitions for visualization purposes, allowing for realistic rendering of humanoid robot models.

## Xacro for Complex Models

Xacro is an XML macro language that extends URDF, allowing for more complex and reusable robot descriptions.

### Xacro Advantages

- **Macros**: Reusable components
- **Mathematical Expressions**: Calculations within the model
- **Inclusion**: Modular model definitions

## Example Humanoid URDF

A simplified example of a humanoid URDF structure includes the torso as the base link with arms and legs as branches.

## Summary

URDF provides the foundation for representing humanoid robots in ROS-based systems. Proper URDF models are essential for simulation, visualization, and control of humanoid robots.