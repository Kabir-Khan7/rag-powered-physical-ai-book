---
id: chapter-3
title: "Chapter 3: Simulation Environments: Gazebo and Unity"
summary: >-
  ## Overview: The Virtual Proving Ground  Imagine testing a humanoid robot's ability to navigate a
  disaster zone, practicing delicate surgical procedures, or training an autonomous vehicle to
  handle rare road conditions—all without risking a single piece of hardware. This is the power of
  robotic simulation, the **virtual proving ground** where algorithms are tested, behaviors are
  refined, and systems are validated long before they encounter the messy unpredictability of the
  physical world. **Simu
difficulty: advanced
keywords:
  - joint
  - unity
  - geometry
  - visual
  - radius
  - collision
---
# Chapter 3: Simulation Environments: Gazebo and Unity

## Overview: The Virtual Proving Ground

Imagine testing a humanoid robot's ability to navigate a disaster zone, practicing delicate surgical procedures, or training an autonomous vehicle to handle rare road conditions—all without risking a single piece of hardware. This is the power of robotic simulation, the **virtual proving ground** where algorithms are tested, behaviors are refined, and systems are validated long before they encounter the messy unpredictability of the physical world.

**Simulation environments** provide the critical bridge between software development and hardware deployment. They allow roboticists to:

- **Test algorithms safely**: No risk of damaging expensive hardware
- **Accelerate development**: Run simulations faster than real-time, often 10-100x speedup
- **Generate synthetic data**: Create labeled training data for machine learning
- **Validate systems**: Test edge cases and failure modes that are difficult or dangerous to replicate physically
- **Collaborate globally**: Share virtual environments without shipping hardware

Two primary tools dominate this space: **Gazebo** for physics-based simulation and **Unity** for high-fidelity visualization and AI training. Together, they form a comprehensive simulation ecosystem that supports everything from basic kinematics to complex human-robot interaction scenarios.

### The Economics of Simulation

Consider these statistics:
- **Hardware costs**: A single humanoid robot like Boston Dynamics Atlas costs approximately $2 million, while simulation software licenses start at $0 (open-source) to $10,000/year
- **Development time**: Simulation allows parallel testing of multiple approaches simultaneously, reducing development cycles by 40-60%
- **Safety incidents**: NASA reports that simulation catches 95% of software errors before physical deployment
- **Data generation**: Unity can generate 100,000+ labeled training images in the time it takes to manually label 100 real images

## Key Topics

### Gazebo: The Physics Powerhouse

#### Architecture and Capabilities

**Gazebo** (now part of Open Robotics' **Ignition Gazebo** ecosystem) is an open-source, physics-based simulator that has become the de facto standard for robotics research and development. Its core strength lies in its **high-fidelity physics engine** and **modular plugin architecture**.

**Key Components**:
```
┌─────────────────────────────────────────────────────┐
│                   Gazebo Architecture                │
├────────────┬──────────────┬────────────┬────────────┤
│   Physics  │   Rendering  │   Sensors  │   GUI/API  │
│   Engine   │   Engine     │   (Noise,  │   (ROS 2,  │
│  (ODE,     │  (OGRE,      │   Models)  │   C++,     │
│   Bullet,  │   OptiX)     │            │   Python)  │
│   SimBody) │              │            │            │
└────────────┴──────────────┴────────────┴────────────┘
         │            │            │            │
         └────────────┼────────────┼────────────┘
                      ▼            ▼
           ┌──────────────────────────────────┐
           │        Simulation World          │
           │  (Robots, Objects, Environments) │
           └──────────────────────────────────┘
```

**Physics Engines Available**:
1. **ODE (Open Dynamics Engine)**: Default, good general-purpose performance
2. **Bullet**: Excellent for complex collisions and soft-body dynamics
3. **SimBody**: High precision for biomechanics and humanoid applications
4. **DART (Dynamic Animation and Robotics Toolkit)**: Specialized for articulated bodies

**Core Features**:
- **Multi-body dynamics** with realistic friction, damping, and constraints
- **Sensor simulation** with configurable noise models (cameras, LIDAR, IMU, force/torque)
- **Plugin system** for custom controllers, sensors, and world behaviors
- **Distributed simulation** for large-scale or complex scenarios
- **Cloud deployment** via Ignition Fuel for shared asset libraries

#### Gazebo in Production Pipelines

**Industrial Applications**:
- **NASA**: Simulates Mars rover operations with 2-second communication delays
- **Amazon Robotics**: Tests warehouse automation systems with thousands of simultaneous robots
- **Toyota Research**: Develops human-robot collaboration for manufacturing
- **Surgical Robotics**: Companies like Intuitive Surgical simulate new instrument designs

**Example: Autonomous Vehicle Testing**:
```bash
# Launch Gazebo with a city environment
ign gazebo -v 4 city.sdf

# Spawn a vehicle with sensors
ign service -s /world/city/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "vehicle_with_sensors.sdf"'
```

### Unity: The Visual and AI Training Platform

#### Beyond Gaming: Unity for Robotics

While initially developed for game development, **Unity** has emerged as a premier platform for robotic simulation due to its **photorealistic rendering**, **extensive asset library**, and **robust machine learning tools** through the Unity ML-Agents framework.

**Key Advantages for Robotics**:
1. **Visual Fidelity**: Ray-traced lighting, physically-based rendering, and post-processing effects
2. **Asset Ecosystem**: Thousands of free and paid 3D models, environments, and textures
3. **ML-Agents Framework**: Built-in reinforcement learning with PyTorch/TensorFlow integration
4. **AR/VR Support**: Native support for mixed reality applications and teleoperation
5. **Cross-Platform**: Deploy to Windows, Linux, macOS, Android, iOS, and WebGL

#### Unity's Robotics Ecosystem

**Core Packages**:
- **ROS-TCP-Connector**: Bidirectional communication between Unity and ROS
- **URDF Importer**: Direct import of robot description files
- **Perception Package**: Synthetic data generation with ground truth
- **Unity Robotics Hub**: Collection of tools, tutorials, and examples
- **NVIDIA Isaac Sim Integration**: High-end simulation for AI training

**Unity for Synthetic Data Generation**:
```csharp
using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;
using UnityEngine.Perception.Randomization.Samplers;

// Create randomized lighting for data augmentation
public class LightingRandomizer : Randomizer
{
    public Light lightSource;
    public FloatSampler intensitySampler = new UniformSampler(0.5f, 1.5f);
    public ColorSampler colorSampler = new ColorSampler();
    
    protected override void OnIterationStart()
    {
        // Randomize light properties each frame
        lightSource.intensity = intensitySampler.Sample();
        lightSource.color = colorSampler.Sample();
    }
}
```

#### Performance Considerations

**Rendering vs. Physics**:
- **Gazebo**: Physics-first, rendering-optimized (1000+ FPS for simple scenes)
- **Unity**: Rendering-first, physics-optimized (60-144 FPS for complex scenes)

**Compute Requirements**:
```python
# Typical resource usage comparison
requirements = {
    "Gazebo": {
        "CPU": "4+ cores (physics-heavy)",
        "GPU": "Integrated or basic discrete (OpenGL 3.3+)",
        "RAM": "8GB minimum, 16GB recommended",
        "Use Case": "Control validation, system dynamics"
    },
    "Unity": {
        "CPU": "4+ cores",
        "GPU": "GTX 1060 / RTX 2060 or better (for ML training)",
        "RAM": "16GB minimum, 32GB recommended",
        "Use Case": "Perception training, visualization, HRI"
    }
}
```

### URDF vs. SDFormat: Robot Modeling Languages

#### URDF (Unified Robot Description Format)

**URDF** is XML-based and designed specifically for describing robot kinematics and dynamics in ROS. It's ideal for **single robots** with **tree-like structures** (no closed loops).

**Basic URDF Structure**:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" 
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joints (connections between links) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**URDF Limitations**:
- No support for parallel mechanisms or closed loops
- Limited scene description (only robots)
- No versioning or modularity
- Static descriptions only

#### SDFormat (Simulation Description Format)

**SDFormat** is a more comprehensive XML format that can describe entire simulation worlds, including multiple robots, environments, lights, and physics properties. It's the native format for Gazebo.

**Key SDFormat Features**:
- **World descriptions**: Complete scenes with multiple entities
- **Modularity**: Include other SDF files via `<include>`
- **Versioning**: Explicit version numbers for compatibility
- **Plugins**: Attach custom behavior to any element
- **Nested models**: Create complex assemblies

**SDFormat Example**:
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
    
    <!-- Include a robot from external file -->
    <include>
      <uri>https://models.gazebosim.org/robots/drake</uri>
      <name>humanoid_robot</name>
      <pose>0 0 1 0 0 0</pose>
    </include>
    
    <!-- Add a simple box -->
    <model name="test_box">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial><mass>1.0</mass></inertial>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
      </link>
    </model>
    
    <!-- Sensor plugin -->
    <plugin filename="libgazebo_ros_camera.so" name="camera">
      <robotNamespace>/camera</robotNamespace>
      <cameraName>rgb_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
    </plugin>
  </world>
</sdf>
```

#### Conversion Between Formats

**URDF to SDF**:
```bash
# Use gz sdf command
gz sdf -p robot.urdf > robot.sdf

# Or use ROS 2 tools
ros2 run urdf2gazebo urdf2gazebo robot.urdf robot.sdf
```

**SDF to URDF** (limited):
```python
# Partial conversion possible for simple models
import sdformat13 as sdf

world = sdf.Root()
world.load('world.sdf')
# Extract first model and convert to URDF
# Note: Complex SDF features may not convert cleanly
```

**Best Practice**: Use URDF for ROS integration and SDF for Gazebo simulation. Maintain both files or convert as needed.

### The Simulation Pipeline

A complete simulation workflow typically involves:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Design Phase  │    │  Testing Phase  │    │  Training Phase │
│  (CAD → URDF)   │────│  (Gazebo)       │────│  (Unity)        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                      │                      │
         ▼                      ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Model Creation │    │ Physics Testing │    │ Perception      │
│  (Blender,      │    │ (Control,       │    │ Training        │
│   Fusion 360)   │    │  Dynamics)      │    │ (Synthetic Data)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                      │                      │
         └──────────────────────┼──────────────────────┘
                                ▼
                    ┌─────────────────────┐
                    │   Hardware-in-the-  │
                    │   Loop (HITL)       │
                    │  (Real components   │
                    │   in virtual env)   │
                    └─────────────────────┘
```

## Hands-On: Creating a Humanoid Model

### Part 1: Building a Simplified Humanoid in Gazebo

#### Step 1: Install Required Tools

```bash
# Install Gazebo (if not already installed)
sudo apt update
sudo apt install gazebo-fortress gazebo-fortress-common

# Install ROS 2 Gazebo integration
sudo apt install ros-humble-gazebo-ros-pkgs

# Install URDF tools
sudo apt install ros-humble-urdf-tutorial

# Install visualization tools
sudo apt install ros-humble-joint-state-publisher-gui
```

#### Step 2: Create the Humanoid URDF Model

Create a new ROS 2 package for our humanoid:
```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_model --dependencies rclpy urdf
cd humanoid_model
mkdir -p urdf meshes launch worlds
```

Create `urdf/humanoid.urdf.xacro` (using Xacro for modularity):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Define materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 0.8 0 1"/>
  </material>
  
  <!-- Define macros for repeated structures -->
  <xacro:macro name="cylinder_inertial" params="mass radius length">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${mass*(3*radius*radius + length*length)/12}" ixy="0" ixz="0"
               iyy="${mass*(3*radius*radius + length*length)/12}" iyz="0"
               izz="${mass*radius*radius/2}"/>
    </inertial>
  </xacro:macro>
  
  <xacro:macro name="sphere_inertial" params="mass radius">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${2*mass*radius*radius/5}" ixy="0" ixz="0"
               iyy="${2*mass*radius*radius/5}" iyz="0"
               izz="${2*mass*radius*radius/5}"/>
    </inertial>
  </xacro:macro>
  
  <xacro:macro name="box_inertial" params="mass x y z">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${mass*(y*y + z*z)/12}" ixy="0" ixz="0"
               iyy="${mass*(x*x + z*z)/12}" iyz="0"
               izz="${mass*(x*x + y*y)/12}"/>
    </inertial>
  </xacro:macro>
  
  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <xacro:box_inertial mass="5.0" x="0.3" y="0.2" z="0.4"/>
  </link>
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <xacro:sphere_inertial mass="1.0" radius="0.1"/>
  </link>
  
  <joint name="head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
  
  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="1.5" radius="0.05" length="0.3"/>
  </link>
  
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>
  
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="1.0" radius="0.04" length="0.25"/>
  </link>
  
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="15" velocity="1.5"/>
  </joint>
  
  <!-- Right Arm (mirror of left) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="1.5" radius="0.05" length="0.3"/>
  </link>
  
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>
  
  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="1.0" radius="0.04" length="0.25"/>
  </link>
  
  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="15" velocity="1.5"/>
  </joint>
  
  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="3.0" radius="0.06" length="0.4"/>
  </link>
  
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0.075 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2.0"/>
  </joint>
  
  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="2.5" radius="0.05" length="0.4"/>
  </link>
  
  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="40" velocity="2.5"/>
  </joint>
  
  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <xacro:box_inertial mass="0.5" x="0.15" y="0.08" z="0.05"/>
  </link>
  
  <joint name="left_ankle" type="fixed">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </joint>
  
  <!-- Right Leg (mirror of left) -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="3.0" radius="0.06" length="0.4"/>
  </link>
  
  <joint name="right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 -0.075 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2.0"/>
  </joint>
  
  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertial mass="2.5" radius="0.05" length="0.4"/>
  </link>
  
  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="40" velocity="2.5"/>
  </joint>
  
  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <xacro:box_inertial mass="0.5" x="0.15" y="0.08" z="0.05"/>
  </link>
  
  <joint name="right_ankle" type="fixed">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </joint>
  
  <!-- Transmission for each joint (for ROS control) -->
  <xacro:macro name="simple_transmission" params="joint_name">
    <transmission name="tran_${joint_name}">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${joint_name}">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="motor_${joint_name}">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>
  
  <xacro:simple_transmission joint_name="left_shoulder"/>
  <xacro:simple_transmission joint_name="left_elbow"/>
  <xacro:simple_transmission joint_name="right_shoulder"/>
  <xacro:simple_transmission joint_name="right_elbow"/>
  <xacro:simple_transmission joint_name="left_hip"/>
  <xacro:simple_transmission joint_name="left_knee"/>
  <xacro:simple_transmission joint_name="right_hip"/>
  <xacro:simple_transmission joint_name="right_knee"/>
  
  <!-- Gazebo plugins -->
  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo>
    <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
      <robotNamespace>/humanoid</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
  
</robot>
```

#### Step 3: Create a Gazebo World File

Create `worlds/simple_room.world`:
```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="simple_room">
    
    <!-- Physics settings -->
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <precon_iters>0</precon_iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
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
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Walls -->
    <model name="north_wall">
      <pose>0 5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="south_wall">
      <pose>0 -5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="east_wall">
      <pose>5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="west_wall">
      <pose>-5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Test objects -->
    <model name="test_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.166</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166</iyy>
            <iyz>0</iyz>
            <izz>0.166</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="test_sphere">
      <pose>-2 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.5</mass>
          <inertia>
            <ixx>0.09</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.09</iyy>
            <iyz>0</iyz>
            <izz>0.09</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

#### Step 4: Create Launch Files

Create `launch/humanoid_gazebo.launch.py`:
```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_humanoid = get_package_share_directory('humanoid_model')
    
    # Process URDF with xacro
    xacro_file = os.path.join(pkg_humanoid, 'urdf', 'humanoid.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': os.path.join(pkg_humanoid, 'worlds', 'simple_room.world'),
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # ROS control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo,
                on_exit=[spawn_robot]
            )
        ),
        ros2_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller]
            )
        ),
    ])
```

#### Step 5: Create Controller Configuration

Create `config/humanoid_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - left_shoulder
    - left_elbow
    - right_shoulder
    - right_elbow
    - left_hip
    - left_knee
    - right_hip
    - right_knee
    
  constraints:
    goal_time: 0.6
    stopped_velocity_tolerance: 0.05
    
  state_publish_rate:  50
  action_monitor_rate: 20
  
  gains:
    left_shoulder:
      p: 100.0
      d: 1.0
      i: 5.0
      i_clamp: 1.0
    left_elbow:
      p: 100.0
      d: 1.0
      i: 5.0
      i_clamp: 1.0
    right_shoulder:
      p: 100.0
      d: 1.0
      i: 5.0
      i_clamp: 1.0
    right_elbow:
      p: 100.0
      d: 1.0
      i: 5.0
      i_clamp: 1.0
    left_hip:
      p: 200.0
      d: 5.0
      i: 10.0
      i_clamp: 2.0
    left_knee:
      p: 200.0
      d: 5.0
      i: 10.0
      i_clamp: 2.0
    right_hip:
      p: 200.0
      d: 5.0
      i: 10.0
      i_clamp: 2.0
    right_knee:
      p: 200.0
      d: 5.0
      i: 10.0
      i_clamp: 2.0
```

#### Step 6: Build and Launch

```bash
# Build the package
cd ~/humanoid_ws
colcon build --packages-select humanoid_model

# Source the workspace
source install/setup.bash

# Launch the simulation
ros2 launch humanoid_model humanoid_gazebo.launch.py
```

#### Step 7: Basic Walking Controller

Create `scripts/walking_controller.py`:
```python
#!/usr/bin/env python3
"""
Simple walking controller for the humanoid model.
Implements a basic inverted pendulum walking pattern.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        
        # Create publisher for joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Walking parameters
        self.step_length = 0.2
        self.step_height = 0.05
        self.step_period = 2.0  # seconds per step
        self.hip_swing = 0.1
        self.arm_swing = 0.3
        
        # Phase tracking
        self.phase = 0.0
        self.support_leg = 'left'  # Start with left leg as support
        
        # Joint names
        self.leg_joints = ['left_hip', 'left_knee', 'right_hip', 'right_knee']
        self.arm_joints = ['left_shoulder', 'left_elbow', 'right_shoulder', 'right_elbow']
        
        # Control timer
        self.timer = self.create_timer(0.02, self.update_walking)  # 50Hz
        
        self.get_logger().info('Walking controller started')
    
    def calculate_leg_trajectory(self, phase, is_support_leg):
        """Calculate joint angles for walking."""
        angles = {}
        
        # Time within step cycle (0 to 1)
        t = phase % 1.0
        
        if is_support_leg:
            # Support leg: minimal movement, provides stability
            angles['hip'] = self.hip_swing * math.sin(2 * math.pi * t) * 0.2
            angles['knee'] = 0.1  # Slightly bent for stability
        else:
            # Swing leg: follows parabolic trajectory
            if t < 0.5:
                # First half: lifting
                lift_phase = t * 2
                angles['hip'] = self.step_length * lift_phase
                angles['knee'] = self.step_height * math.sin(math.pi * lift_phase)
            else:
                # Second half: lowering
                lower_phase = (t - 0.5) * 2
                angles['hip'] = self.step_length * (1 - lower_phase)
                angles['knee'] = self.step_height * math.sin(math.pi * (1 - lower_phase))
        
        return angles
    
    def calculate_arm_trajectory(self, phase):
        """Calculate arm swing for balance."""
        angles = {}
        t = phase % 1.0
        
        # Arms swing opposite to legs for balance
        arm_phase = (t + 0.5) % 1.0
        
        angles['shoulder'] = self.arm_swing * math.sin(2 * math.pi * arm_phase)
        angles['elbow'] = 0.5 + 0.3 * math.sin(2 * math.pi * arm_phase)
        
        return angles
    
    def update_walking(self):
        """Main walking update loop."""
        # Update phase
        dt = 0.02  # 50Hz update rate
        self.phase += dt / self.step_period
        
        # Switch support leg every half cycle
        if self.phase % 1.0 < 0.5:
            current_support = 'left'
        else:
            current_support = 'right'
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.leg_joints + self.arm_joints
        
        # Calculate trajectories
        left_leg_angles = self.calculate_leg_trajectory(
            self.phase, current_support == 'left'
        )
        right_leg_angles = self.calculate_leg_trajectory(
            self.phase + 0.5, current_support == 'right'
        )
        arm_angles = self.calculate_arm_trajectory(self.phase)
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Set leg positions
        point.positions = [
            left_leg_angles['hip'],    # left_hip
            left_leg_angles['knee'],   # left_knee
            -right_leg_angles['hip'],  # right_hip (mirrored)
            right_leg_angles['knee'],  # right_knee
            
            # Arm positions
            arm_angles['shoulder'],    # left_shoulder
            arm_angles['elbow'],       # left_elbow
            -arm_angles['shoulder'],   # right_shoulder (mirrored)
            arm_angles['elbow']        # right_elbow
        ]
        
        # Set velocities (derivative of positions)
        point.velocities = [0.0] * len(trajectory.joint_names)
        
        # Set time from start
        point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        trajectory.points = [point]
        
        # Publish trajectory
        self.publisher.publish(trajectory)
        
        # Logging
        if int(self.phase * 10) % 10 == 0:  # Log every 10th update
            self.get_logger().info(
                f'Phase: {self.phase:.2f}, Support: {current_support}, '
                f'Left Hip: {left_leg_angles["hip"]:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 8: Run the Walking Simulation

```bash
# In a new terminal, run the walking controller
cd ~/humanoid_ws
source install/setup.bash
ros2 run humanoid_model walking_controller.py

# You can also manually control joints using the GUI that appears
# Or send custom trajectories:
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['left_hip', 'left_knee', 'right_hip', 'right_knee'],
      points: [
        {positions: [0.0, 0.5, 0.0, 0.5], time_from_start: {sec: 1}},
        {positions: [0.2, 0.3, -0.2, 0.7], time_from_start: {sec: 2}},
        {positions: [0.0, 0.5, 0.0, 0.5], time_from_start: {sec: 3}}
      ]
    }
  }"
```

### Part 2: Exporting to Unity for Interactive Scenarios

#### Step 1: Install Unity and Required Packages

1. **Download Unity Hub** from [unity.com](https://unity.com)
2. **Install Unity Editor** (2022.3 LTS or newer)
3. **Create a new 3D project**
4. **Install required packages via Package Manager**:
   - **ROS-TCP-Connector** (v0.7.0+)
   - **URDF Importer** (v0.5.0+)
   - **ML-Agents** (v2.0.0+)
   - **Perception Package** (v1.0.0+)

#### Step 2: Convert Gazebo Model for Unity

Create a conversion script `convert_to_unity.py`:
```python
#!/usr/bin/env python3
"""
Convert Gazebo/ROS model for Unity import.
"""

import os
import xml.etree.ElementTree as ET
import json
import yaml

def convert_urdf_to_unity(urdf_path, output_dir):
    """Convert URDF to Unity-compatible format."""
    
    # Parse URDF
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    unity_data = {
        "name": root.get("name", "robot"),
        "links": [],
        "joints": [],
        "materials": {}
    }
    
    # Extract materials
    for material in root.findall("material"):
        name = material.get("name")
        color = material.find("color")
        if color is not None:
            rgba = list(map(float, color.get("rgba", "1 1 1 1").split()))
            unity_data["materials"][name] = {
                "color": rgba[:3],  # RGB
                "alpha": rgba[3]    # Alpha
            }
    
    # Extract links (rigid bodies)
    for link in root.findall("link"):
        link_data = {
            "name": link.get("name"),
            "visuals": [],
            "colliders": []
        }
        
        # Visual geometries
        for visual in link.findall("visual"):
            geom = visual.find("geometry")
            if geom is not None:
                visual_data = {"geometry": {}}
                
                # Check geometry type
                for child in geom:
                    if child.tag == "box":
                        size = list(map(float, child.get("size", "1 1 1").split()))
                        visual_data["geometry"]["type"] = "box"
                        visual_data["geometry"]["size"] = size
                    elif child.tag == "sphere":
                        radius = float(child.get("radius", "0.5"))
                        visual_data["geometry"]["type"] = "sphere"
                        visual_data["geometry"]["radius"] = radius
                    elif child.tag == "cylinder":
                        length = float(child.get("length", "1.0"))
                        radius = float(child.get("radius", "0.5"))
                        visual_data["geometry"]["type"] = "cylinder"
                        visual_data["geometry"]["length"] = length
                        visual_data["geometry"]["radius"] = radius
                
                # Material
                material = visual.find("material")
                if material is not None:
                    material_name = material.get("name")
                    if material_name in unity_data["materials"]:
                        visual_data["material"] = material_name
                
                link_data["visuals"].append(visual_data)
        
        # Collision geometries
        for collision in link.findall("collision"):
            geom = collision.find("geometry")
            if geom is not None:
                collider_data = {"geometry": {}}
                
                for child in geom:
                    if child.tag == "box":
                        size = list(map(float, child.get("size", "1 1 1").split()))
                        collider_data["geometry"]["type"] = "box"
                        collider_data["geometry"]["size"] = size
                    elif child.tag == "sphere":
                        radius = float(child.get("radius", "0.5"))
                        collider_data["geometry"]["type"] = "sphere"
                        collider_data["geometry"]["radius"] = radius
                    elif child.tag == "cylinder":
                        length = float(child.get("length", "1.0"))
                        radius = float(child.get("radius", "0.5"))
                        collider_data["geometry"]["type"] = "cylinder"
                        collider_data["geometry"]["length"] = length
                        collider_data["geometry"]["radius"] = radius
                
                link_data["colliders"].append(collider_data)
        
        # Inertial properties (for physics)
        inertial = link.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                link_data["mass"] = float(mass_elem.get("value", "1.0"))
        
        unity_data["links"].append(link_data)
    
    # Extract joints
    for joint in root.findall("joint"):
        joint_data = {
            "name": joint.get("name"),
            "type": joint.get("type", "fixed"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link")
        }
        
        # Origin transform
        origin = joint.find("origin")
        if origin is not None:
            xyz = list(map(float, origin.get("xyz", "0 0 0").split()))
            rpy = list(map(float, origin.get("rpy", "0 0 0").split()))
            joint_data["origin"] = {"xyz": xyz, "rpy": rpy}
        
        # Axis for revolute/prismatic joints
        axis = joint.find("axis")
        if axis is not None:
            joint_data["axis"] = list(map(float, axis.get("xyz", "0 0 1").split()))
        
        # Limits
        limit = joint.find("limit")
        if limit is not None:
            joint_data["limits"] = {
                "lower": float(limit.get("lower", "0")),
                "upper": float(limit.get("upper", "0")),
                "effort": float(limit.get("effort", "0")),
                "velocity": float(limit.get("velocity", "0"))
            }
        
        unity_data["joints"].append(joint_data)
    
    # Save as JSON for Unity import
    output_path = os.path.join(output_dir, "robot_unity.json")
    with open(output_path, 'w') as f:
        json.dump(unity_data, f, indent=2)
    
    print(f"Converted URDF to Unity format: {output_path}")
    return output_path

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python convert_to_unity.py <urdf_file> <output_dir>")
        sys.exit(1)
    
    convert_urdf_to_unity(sys.argv[1], sys.argv[2])
```

#### Step 3: Create Unity Scene with Humanoid

1. **Import the converted model** into Unity
2. **Create a new scene** and add the robot
3. **Add ROS connection** using ROS-TCP-Connector

Create a C# script `HumanoidController.cs`:
```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class HumanoidController : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;
    
    // Joint references
    public ArticulationBody[] joints;
    public string[] jointNames = {
        "left_shoulder", "left_elbow", "right_shoulder", "right_elbow",
        "left_hip", "left_knee", "right_hip", "right_knee"
    };
    
    // Camera for perception
    public Camera robotCamera;
    public RenderTexture cameraTexture;
    
    // Parameters
    public float publishRate = 30f; // Hz
    private float timer = 0f;
    
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register publishers
        ros.RegisterPublisher<JointStateMsg>("/humanoid/joint_states");
        ros.RegisterPublisher<ImageMsg>("/humanoid/camera/image_raw");
        ros.RegisterPublisher<CameraInfoMsg>("/humanoid/camera/camera_info");
        
        // Register subscribers
        ros.Subscribe<JointTrajectoryMsg>("/humanoid/joint_trajectory", TrajectoryCallback);
        
        // Get joint references
        FindJoints();
        
        Debug.Log("Humanoid ROS controller initialized");
    }
    
    void FindJoints()
    {
        joints = new ArticulationBody[jointNames.Length];
        
        for (int i = 0; i < jointNames.Length; i++)
        {
            GameObject jointObj = GameObject.Find(jointNames[i]);
            if (jointObj != null)
            {
                joints[i] = jointObj.GetComponent<ArticulationBody>();
                if (joints[i] == null)
                {
                    Debug.LogWarning($"Joint {jointNames[i]} missing ArticulationBody component");
                }
            }
            else
            {
                Debug.LogWarning($"Joint {jointNames[i]} not found in scene");
            }
        }
    }
    
    void Update()
    {
        // Publish at specified rate
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            PublishJointStates();
            PublishCameraImage();
            timer = 0f;
        }
    }
    
    void PublishJointStates()
    {
        var msg = new JointStateMsg();
        msg.header.stamp = new TimeStamp();
        msg.name = jointNames;
        msg.position = new double[joints.Length];
        msg.velocity = new double[joints.Length];
        msg.effort = new double[joints.Length];
        
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                msg.position[i] = joints[i].jointPosition[0];
                msg.velocity[i] = joints[i].jointVelocity[0];
                // Note: Effort not directly available in ArticulationBody
            }
        }
        
        ros.Publish("/humanoid/joint_states", msg);
    }
    
    void PublishCameraImage()
    {
        if (robotCamera == null || cameraTexture == null) return;
        
        // Render camera to texture
        robotCamera.targetTexture = cameraTexture;
        robotCamera.Render();
        RenderTexture.active = cameraTexture;
        
        // Read pixels
        Texture2D tex = new Texture2D(cameraTexture.width, cameraTexture.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, cameraTexture.width, cameraTexture.height), 0, 0);
        tex.Apply();
        
        // Convert to ROS image message
        var imageMsg = tex.ToImageMsg();
        imageMsg.header.frame_id = "camera_link";
        imageMsg.header.stamp = new TimeStamp();
        
        ros.Publish("/humanoid/camera/image_raw", imageMsg);
        
        // Cleanup
        Destroy(tex);
    }
    
    void TrajectoryCallback(JointTrajectoryMsg msg)
    {
        // Apply joint trajectory
        if (msg.points.Length > 0)
        {
            var point = msg.points[0]; // Use first point for simplicity
            
            for (int i = 0; i < Mathf.Min(jointNames.Length, point.positions.Length); i++)
            {
                int jointIndex = System.Array.IndexOf(jointNames, msg.joint_names[i]);
                if (jointIndex >= 0 && joints[jointIndex] != null)
                {
                    // Set joint target
                    var drive = joints[jointIndex].xDrive;
                    drive.target = (float)point.positions[i];
                    joints[jointIndex].xDrive = drive;
                }
            }
        }
    }
    
    // Helper method for walking control
    public void SetWalkingPattern(float speed, float stepHeight)
    {
        // Implement walking pattern here
        // This could use inverse kinematics or pre-recorded motions
        Debug.Log($"Setting walking pattern: speed={speed}, stepHeight={stepHeight}");
    }
    
    // Teleoperation methods
    public void MoveArm(string arm, Vector3 targetPosition)
    {
        // Implement inverse kinematics for arm movement
        Debug.Log($"Moving {arm} arm to {targetPosition}");
    }
    
    public void SetGazeDirection(Vector3 direction)
    {
        // Control head/eye direction
        Debug.Log($"Setting gaze direction to {direction}");
    }
}

// Extension method for Texture2D to ROS ImageMsg
public static class TextureExtensions
{
    public static ImageMsg ToImageMsg(this Texture2D texture)
    {
        var msg = new ImageMsg();
        msg.height = (uint)texture.height;
        msg.width = (uint)texture.width;
        msg.encoding = "rgb8";
        msg.step = (uint)(texture.width * 3); // 3 bytes per pixel for RGB
        msg.data = texture.GetRawTextureData();
        
        return msg;
    }
}
```

#### Step 4: Create Interactive Unity Environment

Create a scene with interactive elements:

1. **Add physics-based objects** for manipulation
2. **Create navigation waypoints** for autonomous movement
3. **Implement user interaction** via mouse/touch or VR controllers
4. **Add environmental effects** (lighting, weather, time of day)

Create `InteractiveEnvironment.cs`:
```csharp
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class InteractiveEnvironment : MonoBehaviour
{
    public GameObject humanoidRobot;
    public List<GameObject> interactiveObjects;
    public List<Transform> waypoints;
    
    private ROSConnection ros;
    private HumanoidController robotController;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        robotController = humanoidRobot.GetComponent<HumanoidController>();
        
        // Register services
        ros.RegisterRosService<EmptyRequest, EmptyResponse>("/environment/reset");
        ros.RegisterRosService<EmptyRequest, EmptyResponse>("/environment/get_state");
        
        // Initialize interactive objects
        foreach (var obj in interactiveObjects)
        {
            var interactable = obj.AddComponent<InteractableObject>();
            interactable.OnPickup += OnObjectPickedUp;
            interactable.OnRelease += OnObjectReleased;
        }
        
        Debug.Log("Interactive environment initialized");
    }
    
    void OnObjectPickedUp(GameObject obj)
    {
        Debug.Log($"Object picked up: {obj.name}");
        
        // Notify ROS
        var msg = new StringMsg();
        msg.data = $"object_picked_up:{obj.name}";
        ros.Publish("/environment/events", msg);
        
        // Optional: Highlight object
        var renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.green;
        }
    }
    
    void OnObjectReleased(GameObject obj)
    {
        Debug.Log($"Object released: {obj.name}");
        
        var msg = new StringMsg();
        msg.data = $"object_released:{obj.name}";
        ros.Publish("/environment/events", msg);
        
        // Reset object color
        var renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.white;
        }
    }
    
    // Public methods for scene control
    public void ResetScene()
    {
        foreach (var obj in interactiveObjects)
        {
            var rb = obj.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            
            // Reset to initial position (would need to store initial positions)
        }
        
        Debug.Log("Scene reset");
    }
    
    public void SpawnObject(GameObject prefab, Vector3 position)
    {
        var newObj = Instantiate(prefab, position, Quaternion.identity);
        interactiveObjects.Add(newObj);
        
        var interactable = newObj.AddComponent<InteractableObject>();
        interactable.OnPickup += OnObjectPickedUp;
        interactable.OnRelease += OnObjectReleased;
    }
    
    public Transform GetNextWaypoint()
    {
        if (waypoints.Count == 0) return null;
        // Simple round-robin waypoint selection
        // In practice, you might use path planning
        return waypoints[Random.Range(0, waypoints.Count)];
    }
}

public class InteractableObject : MonoBehaviour
{
    public System.Action<GameObject> OnPickup;
    public System.Action<GameObject> OnRelease;
    
    private bool isHeld = false;
    private Transform originalParent;
    
    void OnMouseDown()
    {
        Pickup();
    }
    
    void OnMouseUp()
    {
        Release();
    }
    
    public void Pickup()
    {
        if (isHeld) return;
        
        isHeld = true;
        originalParent = transform.parent;
        
        // For VR/AR, you would parent to controller
        // For mouse, we'll just flag as held
        
        OnPickup?.Invoke(gameObject);
    }
    
    public void Release()
    {
        if (!isHeld) return;
        
        isHeld = false;
        transform.parent = originalParent;
        
        OnRelease?.Invoke(gameObject);
    }
}
```

#### Step 5: Set Up ROS-Unity Communication

Create a ROS 2 node to bridge between ROS and Unity:

Create `ros2_unity_bridge.py`:
```python
#!/usr/bin/env python3
"""
ROS 2 - Unity bridge for humanoid simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import json
import socket
import threading
import numpy as np

class ROSUnityBridge(Node):
    def __init__(self):
        super().__init__('ros_unity_bridge')
        
        # TCP connection to Unity
        self.unity_host = '127.0.0.1'
        self.unity_port = 10000
        self.unity_socket = None
        self.connect_to_unity()
        
        # ROS subscribers (from ROS to Unity)
        self.joint_traj_sub = self.create_subscription(
            JointTrajectory,
            '/humanoid/joint_trajectory',
            self.joint_trajectory_callback,
            10
        )
        
        self.env_cmd_sub = self.create_subscription(
            String,
            '/environment/command',
            self.environment_command_callback,
            10
        )
        
        # ROS publishers (from Unity to ROS)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/unity/humanoid/joint_states',
            10
        )
        
        self.camera_pub = self.create_publisher(
            Image,
            '/unity/humanoid/camera/image',
            10
        )
        
        self.env_event_pub = self.create_publisher(
            String,
            '/unity/environment/events',
            10
        )
        
        # Start Unity message listener thread
        self.listener_thread = threading.Thread(target=self.listen_to_unity)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        self.get_logger().info('ROS-Unity bridge initialized')
    
    def connect_to_unity(self):
        """Establish TCP connection to Unity."""
        try:
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.connect((self.unity_host, self.unity_port))
            self.get_logger().info(f'Connected to Unity at {self.unity_host}:{self.unity_port}')
        except ConnectionRefusedError:
            self.get_logger().error('Unity not listening. Start Unity scene first.')
            self.unity_socket = None
    
    def send_to_unity(self, message_type, data):
        """Send JSON message to Unity."""
        if self.unity_socket is None:
            return
        
        message = {
            'type': message_type,
            'data': data
        }
        
        try:
            json_str = json.dumps(message) + '\n'
            self.unity_socket.sendall(json_str.encode('utf-8'))
        except BrokenPipeError:
            self.get_logger().error('Lost connection to Unity')
            self.unity_socket = None
    
    def joint_trajectory_callback(self, msg):
        """Forward joint trajectory to Unity."""
        trajectory_data = {
            'joint_names': msg.joint_names,
            'points': []
        }
        
        for point in msg.points:
            point_data = {
                'positions': point.positions,
                'velocities': point.velocities if point.velocities else [],
                'accelerations': point.accelerations if point.accelerations else [],
                'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            }
            trajectory_data['points'].append(point_data)
        
        self.send_to_unity('joint_trajectory', trajectory_data)
    
    def environment_command_callback(self, msg):
        """Forward environment commands to Unity."""
        try:
            command = json.loads(msg.data)
            self.send_to_unity('environment_command', command)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON command: {msg.data}')
    
    def listen_to_unity(self):
        """Listen for messages from Unity."""
        buffer = ""
        
        while rclpy.ok():
            if self.unity_socket is None:
                # Try to reconnect
                try:
                    self.connect_to_unity()
                except:
                    pass
                rclpy.spin_once(self, timeout_sec=1.0)
                continue
            
            try:
                # Receive data
                data = self.unity_socket.recv(4096)
                if not data:
                    self.get_logger().warn('Unity disconnected')
                    self.unity_socket.close()
                    self.unity_socket = None
                    continue
                
                buffer += data.decode('utf-8')
                
                # Process complete messages (delimited by newline)
                while '\n' in buffer:
                    message_str, buffer = buffer.split('\n', 1)
                    
                    try:
                        message = json.loads(message_str)
                        self.process_unity_message(message)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f'Invalid JSON from Unity: {message_str}')
                        
            except ConnectionResetError:
                self.get_logger().error('Unity connection reset')
                self.unity_socket = None
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().error(f'Error reading from Unity: {e}')
    
    def process_unity_message(self, message):
        """Process incoming message from Unity."""
        msg_type = message.get('type', '')
        data = message.get('data', {})
        
        if msg_type == 'joint_states':
            # Publish joint states to ROS
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = data.get('name', [])
            msg.position = data.get('position', [])
            msg.velocity = data.get('velocity', [])
            msg.effort = data.get('effort', [])
            
            self.joint_state_pub.publish(msg)
        
        elif msg_type == 'camera_image':
            # Publish camera image to ROS
            # Note: This is simplified. Real implementation would handle binary image data
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = data.get('frame_id', 'camera_link')
            msg.height = data.get('height', 480)
            msg.width = data.get('width', 640)
            msg.encoding = data.get('encoding', 'rgb8')
            msg.step = data.get('step', 1920)  # width * 3 for RGB
            # In real implementation, you'd convert base64 or binary data
            # msg.data = base64.b64decode(data['data'])
            
            self.camera_pub.publish(msg)
        
        elif msg_type == 'environment_event':
            # Forward environment events
            msg = String()
            msg.data = json.dumps(data)
            self.env_event_pub.publish(msg)
        
        else:
            self.get_logger().warn(f'Unknown message type from Unity: {msg_type}')
    
    def destroy_node(self):
        """Cleanup before destruction."""
        if self.unity_socket:
            self.unity_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = ROSUnityBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 6: Run the Complete System

```bash
# Terminal 1: Start ROS-Unity bridge
cd ~/humanoid_ws
source install/setup.bash
python3 ros2_unity_bridge.py

# Terminal 2: Start Unity scene
# Open Unity, load the humanoid scene, and press Play

# Terminal 3: Send commands to Unity via ROS
# Example: Send a walking trajectory
ros2 topic pub /humanoid/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['left_hip', 'left_knee', 'right_hip', 'right_knee'],
  points: [
    {
      positions: [0.0, 0.5, 0.0, 0.5],
      time_from_start: {sec: 1}
    },
    {
      positions: [0.3, 0.3, -0.3, 0.7],
      time_from_start: {sec: 2}
    }
  ]
}"

# Terminal 4: Monitor Unity output
ros2 topic echo /unity/humanoid/joint_states
ros2 topic echo /unity/environment/events
```

## Objective Achieved: Safe, Cost-Effective Prototyping

Through this hands-on chapter, you've learned to:

### 1. **Create Complex Robot Models**
- Designed a complete humanoid robot using URDF/Xacro
- Implemented realistic mass properties and joint constraints
- Added sensors and controllers for simulation

### 2. **Simulate Physics-Based Behaviors**
- Set up Gazebo environments with realistic physics
- Implemented basic walking controllers
- Tested stability and dynamic responses

### 3. **Build High-Fidelity Visual Simulations**
- Exported models to Unity for advanced rendering
- Created interactive environments with user input
- Implemented ROS-Unity communication bridges

### 4. **Develop Complete Simulation Pipelines**
- From CAD/URDF to functional simulation
- Cross-platform testing (Gazebo for physics, Unity for visuals)
- Integration with ROS 2 for control and monitoring

## Best Practices for Simulation Development

### 1. **Model Validation**
```python
def validate_robot_model(urdf_file):
    """Check robot model for common issues."""
    issues = []
    
    # Check mass properties
    total_mass = calculate_total_mass(urdf_file)
    if total_mass <= 0:
        issues.append("Total mass must be positive")
    
    # Check joint limits
    joints = parse_joints(urdf_file)
    for joint in joints:
        if joint.type == 'revolute':
            if joint.upper_limit <= joint.lower_limit:
                issues.append(f"Joint {joint.name}: invalid limits")
    
    # Check for floating base
    if not has_floating_base(urdf_file):
        issues.append("Humanoid should have floating base for walking")
    
    return issues
```

### 2. **Simulation Performance Optimization**

**Gazebo Optimization**:
```xml
<!-- Use simplified collision geometries -->
<link name="complex_link">
  <collision>
    <geometry>
      <!-- Simplified box instead of detailed mesh -->
      <box>
        <size>0.1 0.2 0.3</size>
      </box>
    </geometry>
  </collision>
  <visual>
    <geometry>
      <!-- Detailed mesh for visualization only -->
      <mesh filename="package://my_robot/meshes/detailed.stl"/>
    </geometry>
  </visual>
</link>

<!-- Adjust physics parameters -->
<physics name="optimized" type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Larger for faster simulation -->
  <real_time_factor>2.0</real_time_factor>  <!-- Faster than real-time -->
  <solver>
    <iters>20</iters>  <!-- Reduce iterations for speed -->
  </solver>
</physics>
```

**Unity Optimization**:
```csharp
// Use object pooling for frequently created/destroyed objects
public class ObjectPool : MonoBehaviour
{
    public GameObject prefab;
    public int poolSize = 100;
    private Queue<GameObject> pool = new Queue<GameObject>();
    
    void Start()
    {
        for (int i = 0; i < poolSize; i++)
        {
            GameObject obj = Instantiate(prefab);
            obj.SetActive(false);
            pool.Enqueue(obj);
        }
    }
    
    public GameObject GetObject()
    {
        if (pool.Count > 0)
        {
            GameObject obj = pool.Dequeue();
            obj.SetActive(true);
            return obj;
        }
        return Instantiate(prefab);
    }
    
    public void ReturnObject(GameObject obj)
    {
        obj.SetActive(false);
        pool.Enqueue(obj);
    }
}
```

### 3. **Debugging Simulation Issues**

**Common Issues and Solutions**:
1. **Robot falls through floor**: Check collision geometries and friction coefficients
2. **Joints behave erratically**: Verify mass properties and controller gains
3. **Poor simulation performance**: Use simplified collision geometries, reduce solver iterations
4. **ROS-Unity connection failures**: Check firewall settings, verify IP addresses and ports

**Debugging Tools**:
```bash
# Gazebo debugging
gz log -d  # Enable debug logging
gz stats   # Monitor simulation performance

# ROS 2 debugging
ros2 topic hz /joint_states  # Check message rates
ros2 node info /gazebo      # Check node connections

# Unity debugging
# Use Unity Profiler (Window → Analysis → Profiler)
# Enable Debug.Log in ROS-TCP-Connector
```

## Real-World Applications

### Case Study: Boston Dynamics Simulation Pipeline

Boston Dynamics uses a sophisticated simulation pipeline for developing Atlas:

1. **High-fidelity dynamics** in MuJoCo for controller development
2. **Visualization** in Unity for human-robot interaction studies
3. **Hardware-in-the-loop** for controller validation
4. **Cloud-based simulation** for large-scale testing

Their workflow reduces physical testing by 80% while catching 95% of software bugs before hardware deployment.

### Case Study: NASA Robonaut 2

NASA's Robonaut 2 development used extensive simulation:

- **Task verification** in simulated ISS environments
- **Human-robot collaboration** testing
- **Failure mode analysis** for space operations
- **Teleoperation training** for astronauts

Simulation allowed testing scenarios that would be impossible or dangerous in physical space.

## Key Takeaways

1. **Simulation is essential** for modern robotics development, enabling safe, fast, and cost-effective testing
2. **Gazebo excels at physics simulation** with multiple physics engine options and ROS integration
3. **Unity provides superior visualization** and AI training capabilities through ML-Agents
4. **URDF and SDFormat** are complementary formats for different stages of development
5. **ROS-Unity bridges** enable combining the strengths of both simulation platforms
6. **Performance optimization** is critical for efficient simulation workflows
7. **Simulation validation** against real-world data ensures meaningful results

## Discussion Questions

1. How would you validate that simulation results accurately predict real-world robot behavior?
2. What are the ethical considerations of training AI systems entirely in simulation?
3. How might simulation bias (differences from reality) affect deployed robotic systems?
4. What simulation capabilities will be needed for next-generation humanoid robots?

## Exercises

1. **Enhanced Humanoid Model**: Add hands with fingers to your humanoid model and implement grasping simulations in both Gazebo and Unity.

2. **Comparative Analysis**: Create the same walking controller in both Gazebo and Unity, then compare the results. What differences do you observe in stability, performance, and visual fidelity?

3. **Simulation-to-Real Transfer**: Train a reinforcement learning policy in Unity to balance the humanoid, then attempt to transfer it to Gazebo. What modifications are needed?

4. **Multi-Robot Simulation**: Extend your environment to include multiple humanoids working collaboratively on a task (e.g., moving a large object).

5. **Sensor Simulation**: Implement realistic sensor models (LIDAR with noise, camera distortion) and test perception algorithms in simulation before real deployment.

6. **Failure Mode Testing**: Design simulation scenarios that test robot recovery from falls, sensor failures, and unexpected obstacles.

7. **HITL (Hardware-in-the-Loop)**: If you have access to robotic hardware, implement a HITL simulation where real motors are controlled by a simulated environment.

---

*Next Chapter Preview: In Chapter 4, we'll dive into NVIDIA Isaac, exploring advanced AI perception and control systems. You'll learn to train object detection models, implement SLAM, and deploy reinforcement learning controllers for complex robotic tasks.*
