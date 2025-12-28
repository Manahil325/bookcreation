---
title: "Environment Setup"
sidebar_position: 3
description: "Setting up Gazebo environment for physics simulation"
keywords: ["gazebo", "environment setup", "simulation", "robotics"]
learning_objectives:
  - "Install and configure Gazebo"
  - "Set up simulation environments"
  - "Configure physics parameters"
estimated_time: "1.5 hours"
difficulty: "Intermediate"
prerequisites:
  - "ROS 2 installed"
  - "Basic command line skills"
---

# Environment Setup

This section covers how to set up your Gazebo environment for physics simulation. We'll walk through installation, basic configuration, and creating your first simulation environment.

## Installing Gazebo

### Prerequisites
Before installing Gazebo, ensure you have:
- ROS 2 (Humble Hawksbill or later recommended)
- Ubuntu 22.04 or compatible Linux distribution
- At least 4GB of RAM and 10GB of free disk space

### Installation Steps

1. **Add the ROS GPG key and repository**:
   ```bash
   sudo apt update && sudo apt install -y wget
   sudo wget -O /usr/share/keyrings/ros-archive-keyring.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install Gazebo and ROS 2 integration**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
   ```

3. **Install additional tools**:
   ```bash
   sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
   ```

## Basic Configuration

### Environment Variables
Add these to your `~/.bashrc` or `~/.zshrc`:
```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

### Verify Installation
Test that Gazebo is properly installed:
```bash
gazebo --version
```

## Creating Your First Environment

### Launching Gazebo
Start Gazebo with the default empty world:
```bash
gazebo
```

### Loading a Predefined World
Gazebo comes with several predefined worlds. Try:
```bash
gazebo --verbose worlds/empty.world
gazebo --verbose worlds/willowgarage.world
```

### Creating a Custom World
Create a simple world file (`my_world.sdf`):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Running Your Custom World
```bash
gazebo --verbose my_world.sdf
```

## Physics Configuration

### Adjusting Physics Parameters
You can adjust physics parameters through the Gazebo GUI or by modifying the world file. Key parameters include:
- **Max Step Size**: Controls simulation time step
- **Real Time Factor**: Controls how fast simulation runs relative to real time
- **Gravity**: Sets the gravitational acceleration

### Performance Considerations
- Smaller step sizes increase accuracy but decrease performance
- Higher real-time factors can cause instability
- Complex collision meshes impact performance significantly

## Common Setup Issues

1. **Graphics Issues**: Ensure you have proper graphics drivers installed
2. **Permission Issues**: Make sure you're not running Gazebo as root unnecessarily
3. **Library Conflicts**: Check for conflicting Gazebo installations

## Exercise

Set up Gazebo on your system and create a simple world with a ground plane and a few objects. Experiment with different physics parameters to see their effect on simulation behavior.