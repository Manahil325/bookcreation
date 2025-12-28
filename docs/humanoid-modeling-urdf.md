---
sidebar_position: 3
title: "Humanoid Modeling with URDF"
---

# Humanoid Modeling with URDF: Building Robot Bodies

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF provides a way to define the physical structure, including links (rigid bodies), joints (connections between links), and other properties like visual appearance and collision properties. This chapter explains how to create URDF models for humanoid robots and integrate them with ROS 2.

## URDF Structure

A URDF file consists of several key elements:

- **Links**: Rigid bodies that represent parts of the robot (e.g., torso, head, arms, legs)
- **Joints**: Connections between links that define how they can move relative to each other
- **Visual**: How the link appears in simulation and visualization
- **Collision**: How the link interacts with other objects for collision detection
- **Inertial**: Physical properties like mass and moment of inertia

## Basic URDF Example

Here's a simple URDF structure for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Links and Their Properties

Links represent rigid bodies in the robot model. Each link should define:

### Visual Properties
How the link appears in simulation and visualization:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one: box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR -->
      <cylinder radius="0.05" length="0.2"/>
      <!-- OR -->
      <sphere radius="0.05"/>
      <!-- OR -->
      <mesh filename="package://robot_description/meshes/link_name.stl"/>
    </geometry>
    <material name="material_name">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>
</link>
```

### Collision Properties
How the link interacts with other objects:

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Usually simpler geometry than visual for performance -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties
Physical properties for simulation:

```xml
<link name="link_name">
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Joint Types and Properties

Joints define how links can move relative to each other. Common joint types for humanoid robots:

### Revolute Joints
Rotational joints with limited range (e.g., elbows, knees):

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Continuous Joints
Rotational joints without limits (e.g., head rotation):

```xml
<joint name="head_rotation" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1"/>
</joint>
```

### Fixed Joints
Non-moving connections (e.g., attaching sensors):

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

## Complete Humanoid URDF Example

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <!-- Right arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right shoulder joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <!-- Left leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left hip joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2"/>
  </joint>

  <!-- Right leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right hip joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2"/>
  </joint>
</robot>
```

## Integrating URDF with ROS 2

### Robot State Publisher

To visualize your URDF model in ROS 2, you need the robot_state_publisher package:

```bash
# Launch the robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
```

### Using xacro for Complex Models

For complex humanoid models, xacro (XML Macros) is often used to make URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.4" />
  <xacro:property name="torso_width" value="0.2" />
  <xacro:property name="torso_height" value="0.15" />

  <!-- Macro for creating limbs -->
  <xacro:macro name="limb" params="side parent_link">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.2" radius="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.2" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${side == 'left' and 0.15 or -0.15} 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
    </joint>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="${torso_length} ${torso_width} ${torso_height}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="${torso_length} ${torso_width} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Use the macro to create both arms -->
  <xacro:limb side="left" parent_link="torso"/>
  <xacro:limb side="right" parent_link="torso"/>

</robot>
```

## Visualization Tools

### RViz2
Use RViz2 to visualize your URDF model:

```bash
# Launch RViz2 and add RobotModel display
rviz2
```

### Joint State Publisher GUI
Use the joint state publisher to move joints interactively:

```bash
# Launch the GUI to control joint positions
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Best Practices for Humanoid URDF

### Naming Conventions
Use consistent naming conventions:

- Links: `torso`, `head`, `left_upper_arm`, `right_forearm`, `left_foot`, etc.
- Joints: `left_elbow_joint`, `right_knee_joint`, `neck_joint`, etc.

### Mass and Inertia
- Make sure mass values are realistic
- Calculate inertias properly for stable simulation
- Consider using CAD software to calculate exact inertial properties

### Joint Limits
- Set appropriate joint limits based on human anatomy or mechanical constraints
- Include safety margins in joint limits
- Consider the robot's intended use when setting limits

### Visual vs Collision Models
- Use detailed meshes for visual elements
- Use simpler geometries (boxes, cylinders, spheres) for collision detection for performance
- Ensure collision models are conservative (larger than visual models) for safety

## Troubleshooting Common Issues

### TF Tree Issues
- Ensure all links are connected through joints
- Check that there are no disconnected links
- Verify that the URDF forms a single connected tree

### Joint Direction
- Use the right-hand rule for joint axes
- Test joint directions in simulation before deployment
- Consider the impact of joint direction on control algorithms

### Units
- Always use consistent units (meters for distances, kilograms for mass)
- Check that all parameters are in the expected units

## Summary

URDF is a powerful tool for modeling humanoid robots in ROS 2. By understanding links, joints, and their properties, you can create accurate robot models that integrate well with the ROS 2 ecosystem. Proper modeling is crucial for simulation, visualization, and control of humanoid robots. The combination of URDF with ROS 2 tools like robot_state_publisher and RViz2 provides a complete solution for robot modeling and visualization.

With the knowledge from this chapter, you can now create detailed humanoid robot models that serve as the physical foundation for the ROS 2 nervous system we discussed in the previous chapters.