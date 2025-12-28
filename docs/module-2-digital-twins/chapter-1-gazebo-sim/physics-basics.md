---
title: "Physics Basics"
sidebar_position: 2
description: "Understanding physics simulation concepts and principles in Gazebo"
keywords: ["physics simulation", "gazebo", "robotics", "dynamics"]
learning_objectives:
  - "Understand fundamental physics concepts in simulation"
  - "Learn about mass, friction, and other physical parameters"
  - "Know how to configure physics properties for realistic simulation"
estimated_time: "2 hours"
difficulty: "Intermediate"
prerequisites:
  - "ROS 2 fundamentals"
  - "Basic understanding of physics"
---

# Physics Basics

Physics simulation is the foundation for creating realistic digital twins. In this section, we'll cover the fundamental concepts that govern how objects behave in simulated environments.

## Key Physics Concepts in Gazebo

### Mass and Inertia
- **Mass**: The amount of matter in an object, affecting how it responds to forces
- **Inertia**: The resistance of an object to changes in its state of motion
- Proper mass and inertia values are crucial for realistic robot behavior

### Friction
- **Static Friction**: The force that prevents objects from starting to move
- **Dynamic Friction**: The force that opposes motion when objects are already moving
- Friction parameters affect how robots interact with surfaces

### Collision Detection
- How Gazebo detects when objects come into contact
- Different collision algorithms and their trade-offs
- Performance considerations for complex environments

### Dynamics Simulation
- How forces, torques, and motion are calculated
- Integration methods (Euler, Runge-Kutta, etc.)
- Time step considerations for stability and accuracy

## Configuring Physics Properties

### URDF Integration
Gazebo reads physics properties from URDF files:

```xml
<link name="link_name">
  <inertial>
    <mass value="0.1" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
  </inertial>
  <collision name="collision_name">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </collision>
  <visual name="visual_name">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </visual>
</link>
```

### SDF Physics Parameters
You can also specify physics properties directly in SDF files:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

## Best Practices for Physics Simulation

1. **Start Simple**: Begin with basic shapes and simple physics properties
2. **Validate Against Reality**: Compare simulation behavior with real-world data
3. **Optimize Performance**: Balance accuracy with computational efficiency
4. **Iterative Refinement**: Adjust parameters based on observed behavior

## Common Physics Issues and Solutions

- **Unstable Simulations**: Reduce time step or adjust solver parameters
- **Penetrating Objects**: Improve collision geometry or adjust material properties
- **Excessive Bouncing**: Increase damping or adjust restitution coefficients

## Exercise

Create a simple simulation with a box and sphere, and experiment with different mass and friction values to observe their effects on motion.