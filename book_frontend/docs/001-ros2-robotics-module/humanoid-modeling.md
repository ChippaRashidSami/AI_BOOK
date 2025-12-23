---
sidebar_position: 4
title: 'Humanoid Modeling with URDF'
description: 'Understanding URDF for defining robot structure, joints, and kinematics'
---

# Humanoid Modeling with URDF

## Introduction

URDF (Unified Robot Description Format) is the standard way to represent robot models in ROS. For humanoid robots, URDF defines the physical structure, joints, and kinematics that enable simulation, control, and visualization. This chapter covers the fundamentals of URDF and how it applies to humanoid robot modeling.

## What is URDF?

URDF is an XML-based format that describes robot structure in terms of:

- Links: Rigid bodies with physical properties (mass, inertia, geometry)
- Joints: Constraints between links that allow relative motion
- Materials: Visual appearance definitions
- Transmissions: Information about actuators
- Gazebo plugins: Simulation-specific configurations

## Links: Defining Robot Parts

Links represent rigid bodies in the robot. For a humanoid robot, typical links include:

- Base link (torso/core)
- Limbs (arms, legs)
- End effectors (hands, feet)
- Sensors (cameras, LiDAR)

Each link has physical properties:

```xml
<link name="upper_arm">
  <inertial>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/upper_arm.dae"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.2" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

## Joints: Defining Motion Capabilities

Joints define how links connect and move relative to each other. Common joint types for humanoid robots:

- Revolute: Rotational joint with limited angle range (e.g., elbow, knee)
- Continuous: Rotational joint with unlimited rotation (e.g., shoulder, hip)
- Prismatic: Linear sliding joint with limited travel
- Fixed: No relative motion (e.g., sensor mounts)

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="30" velocity="1.0"/>
</joint>
```

## Special Considerations for Humanoid Robots

Humanoid robots have unique structural requirements:

### Kinematic Chains
- Multiple interconnected chains (arms, legs)
- Closed loops for stability (when feet contact ground)
- Complex branching structures from torso

### Degrees of Freedom
- High DOF for human-like movement
- Joint limits that reflect biological constraints
- Redundancy in some areas (shoulders, hips)

### Balance and Stability
- Center of mass considerations
- Foot placement for bipedal stability
- Compliance for safe interaction

## URDF Best Practices

When creating URDF models for humanoid robots:

1. **Start with a proper base**: Define a base_link as the reference frame
2. **Consider the kinematic structure**: Ensure joints form proper kinematic chains
3. **Include collision geometry**: Use simplified geometries for performance
4. **Accurate inertial properties**: Essential for simulation accuracy
5. **Use consistent naming**: Establish patterns for joint and link names
6. **Validate the model**: Check for singularities and proper kinematics

## Working with URDF in ROS

Several tools help work with URDF:

### Robot State Publisher
Publishes forward kinematics information based on joint states:
```xml
<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
  <param name="publish_frequency" value="50.0"/>
</node>
```

### TF (Transform) Frames
URDF automatically creates TF frames that describe spatial relationships between links, essential for navigation and manipulation.

### Visualization
RViz can visualize URDF models with proper joint angles, helping debug kinematic issues.

## Example: Simplified Humanoid URDF Structure

A basic humanoid structure would have a tree of links and joints like:

```
base_link
├── torso
    ├── head
    ├── left_upper_arm
    │   ├── left_lower_arm
    │       └── left_hand
    ├── right_upper_arm
    │   ├── right_lower_arm
    │       └── right_hand
    ├── left_thigh
    │   ├── left_shin
    │       └── left_foot
    └── right_thigh
        ├── right_shin
            └── right_foot
```

## Validation and Testing

Before deploying a URDF model:

- Check for self-collisions using collision checking tools
- Validate joint limits and ranges of motion
- Test kinematic solvers with the model
- Verify center of mass calculations
- Simulate the model in Gazebo

## Summary

URDF is fundamental to humanoid robotics, providing the representation needed for simulation, visualization, and control. Properly designed URDF models enable accurate simulation and facilitate development of robot behaviors before deployment to real hardware.