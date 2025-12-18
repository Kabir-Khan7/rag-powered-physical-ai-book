---
id: chapter-2
title: "Chapter 2: Fundamentals of Robot Operating Systems (ROS 2)"
summary: >-
  ## Overview: The Robotic Nervous System  Imagine trying to coordinate the complex symphony of
  processes in a humanoid robot—vision systems processing 60 frames per second, balance algorithms
  making millisecond adjustments, natural language processors parsing speech, and actuators
  executing precise movements. This coordination requires more than just good code; it requires a
  robust, standardized communication framework. That framework is **Robot Operating System (ROS)**,
  specifically ROS 2, which
difficulty: advanced
keywords:
  - counter
  - publisher
  - create
  - status
  - import
  - rclpy
---
# Chapter 2: Fundamentals of Robot Operating Systems (ROS 2)

## Overview: The Robotic Nervous System

Imagine trying to coordinate the complex symphony of processes in a humanoid robot—vision systems processing 60 frames per second, balance algorithms making millisecond adjustments, natural language processors parsing speech, and actuators executing precise movements. This coordination requires more than just good code; it requires a robust, standardized communication framework. That framework is **Robot Operating System (ROS)**, specifically ROS 2, which serves as the central nervous system for modern robotics.

**ROS 2** is not an operating system in the traditional sense but rather a middleware—a collection of software frameworks, tools, and libraries that enable developers to build complex robotic systems. Originally developed at Stanford and Willow Garage in 2007, ROS has evolved through several iterations, with ROS 2 representing a complete architectural redesign for production systems, featuring improved security, real-time capabilities, and cross-platform support.

### Why ROS 2 Matters

Consider these real-world applications enabled by ROS 2:
- **NASA's VIPER rover** uses ROS 2 for lunar surface exploration
- **Aurora's autonomous vehicles** leverage ROS 2 for perception and control
- **Surgical robots** like those from Intuitive Surgical employ ROS 2 for precise control loops
- **Industrial humanoids** from companies like Figure and Tesla use ROS 2 for their software stacks

ROS 2 provides the abstraction layer that allows roboticists to focus on application logic rather than reinventing communication protocols, hardware interfaces, and system orchestration for each new project.

## Key Topics

### ROS 2 Architecture: A Distributed Framework

#### The Core Philosophy: Everything is a Node
ROS 2 adopts a **distributed, node-based architecture** where each component of a robotic system is implemented as an independent executable (a node) that communicates with other nodes through well-defined interfaces. This modular approach offers several advantages:

1. **Fault Isolation**: A crash in one node doesn't necessarily bring down the entire system
2. **Language Agnosticism**: Nodes can be written in Python, C++, Java, or other supported languages
3. **Scalability**: New functionality can be added by simply creating new nodes
4. **Reusability**: Common nodes (sensor drivers, controllers) can be shared across projects

#### The ROS 2 Graph: Visualizing Communication
The collection of all nodes and their connections forms the **ROS graph**. Key components of this graph include:

- **Nodes**: Individual processes performing computations
- **Topics**: Named buses over which nodes exchange messages asynchronously
- **Services**: Request-reply interfaces for synchronous communication
- **Actions**: Long-running operations with feedback, cancellation, and result reporting
- **Parameters**: Configuration variables accessible across nodes

```
┌─────────────────────────────────────────────────────┐
│                   ROS 2 Graph                        │
├───────────┬───────────┬────────────┬───────────────┤
│   Node A  │   Node B  │   Node C   │    Node D     │
│  (Camera  │ (Object   │ (Motion    │  (Speech      │
│  Driver)  │ Detector) │ Planner)   │  Synthesizer) │
└─────┬─────┴─────┬─────┴──────┬─────┴───────┬───────┘
      │           │            │             │
      ▼           ▼            ▼             ▼
┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐
│ /camera/ │ │ /detect/ │ │ /cmd_vel │ │ /speech/ │
│  image   │ │  objects │ │          │ │   out    │
│ (Topic)  │ │ (Topic)  │ │ (Topic)  │ │ (Topic)  │
└──────────┘ └──────────┘ └──────────┘ └──────────┘
      │           │            │             │
      └───────────┼────────────┘             │
                  │                          │
            ┌─────▼─────┐              ┌─────▼─────┐
            │   Node E  │              │   Node F  │
            │ (Decision │              │   (GUI    │
            │  Maker)   │              │  Display) │
            └───────────┘              └───────────┘
```

### Core Communication Patterns

#### 1. Topics: The Publish-Subscribe Model

Topics implement a **many-to-many communication pattern** where multiple publishers can send messages to multiple subscribers, all without direct knowledge of each other.

**Key Characteristics**:
- **Asynchronous**: No waiting for receivers
- **Decoupled**: Publishers and subscribers operate independently
- **Typed**: Each topic carries messages of a specific data type
- **Buffered**: Messages can be queued if subscribers are temporarily unavailable

**Common Topic Examples**:
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed
- `/scan` (sensor_msgs/LaserScan) - LIDAR data
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/joint_states` (sensor_msgs/JointState) - Robot joint positions

#### 2. Services: The Request-Response Model

Services provide **synchronous communication** where a client node sends a request and waits for a response from a server node.

**Key Characteristics**:
- **Synchronous**: Client blocks until response received
- **One-to-one**: Typically one server handles multiple clients
- **Deterministic**: Guarantees response timing (with quality of service settings)

**Common Service Examples**:
- `/set_parameters` - Configure node parameters
- `/spawn_entity` (Gazebo) - Add objects to simulation
- `/compute_path` (Navigation) - Calculate optimal route

#### 3. Actions: Complex Operation Management

Actions extend the service model to support **long-running tasks with feedback**, cancellation, and progress tracking—perfect for operations like navigation, manipulation, or complex perception tasks.

**Action Structure**:
- **Goal**: Initial request defining the task
- **Feedback**: Periodic updates on progress
- **Result**: Final outcome upon completion

**Common Action Examples**:
- `/navigate_to_pose` - Move robot to specific location
- `/pick_object` - Grasp and lift an item
- `/recognize_speech` - Convert audio to text with confidence updates

### Quality of Service (QoS): Ensuring Reliable Communication

One of ROS 2's most powerful features is its **configurable Quality of Service policies** that determine how messages are delivered under various network conditions.

#### Key QoS Policies:
```python
# Example QoS profiles in Python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

# For sensor data (best effort, drop old samples)
sensor_qos = QoSProfile(
    depth=10,  # Queue size
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # May drop messages
    history=QoSHistoryPolicy.KEEP_LAST  # Keep only latest messages
)

# For control commands (reliable, keep all history)
control_qos = QoSProfile(
    depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Guarantee delivery
    history=QoSHistoryPolicy.KEEP_ALL  # Keep all messages
)

# For real-time systems (with deadlines)
realtime_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    deadline=Duration(seconds=0.1)  # Messages must arrive within 100ms
)
```

#### Common QoS Patterns:
- **Sensor Data**: `BEST_EFFORT` + `VOLATILE_DURABILITY` (accept occasional drops)
- **Control Commands**: `RELIABLE` + `TRANSIENT_LOCAL_DURABILITY` (guarantee delivery)
- **State Updates**: `RELIABLE` + `KEEP_ALL_HISTORY` (maintain complete record)

### The ROS 2 Computation Model

#### Executors: Managing Node Execution
ROS 2 uses **executors** to manage how nodes process callbacks (message handlers, service servers, etc.). The choice of executor significantly impacts system performance:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

# Single-threaded (simple, deterministic)
executor = SingleThreadedExecutor()

# Multi-threaded (parallel processing, better for complex systems)
executor = MultiThreadedExecutor(num_threads=4)

# Add nodes to executor
executor.add_node(camera_node)
executor.add_node(perception_node)

# Spin (start processing)
executor.spin()
```

#### Callback Groups: Organizing Execution
For complex nodes, **callback groups** allow fine-grained control over which callbacks can execute in parallel:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# Mutually exclusive (callbacks run one at a time)
exclusive_group = MutuallyExclusiveCallbackGroup()

# Reentrant (callbacks can run in parallel)
parallel_group = ReentrantCallbackGroup()

# Apply to publishers/subscribers
camera_sub = node.create_subscription(
    Image, '/camera/image', image_callback,
    qos_profile=sensor_qos,
    callback_group=exclusive_group  # Won't block other callbacks
)
```

### Integration with Sensors and Actuators

#### Standard Message Types
ROS 2 provides a rich set of **standard message types** that ensure interoperability between different hardware and software components:

**Sensor Messages** (`sensor_msgs`):
- `Image` - Camera frames with metadata
- `PointCloud2` - 3D point clouds from depth sensors
- `Imu` - Accelerometer, gyroscope, and magnetometer data
- `LaserScan` - 2D LIDAR scans
- `JointState` - Position, velocity, and effort of robot joints

**Geometry Messages** (`geometry_msgs`):
- `Pose` - Position and orientation in 3D space
- `Twist` - Linear and angular velocity
- `Transform` - Spatial relationship between coordinate frames

**Navigation Messages** (`nav_msgs`):
- `Odometry` - Estimated position and velocity
- `Path` - Sequence of poses defining a trajectory
- `OccupancyGrid` - 2D map for navigation

#### Hardware Interface Patterns
Connecting ROS 2 to physical hardware follows consistent patterns:

**Sensor Drivers** (Publishing Pattern):
```python
class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher = self.create_publisher(Image, '/camera/image', 10)
        self.timer = self.create_timer(0.033, self.capture_frame)  # 30Hz
    
    def capture_frame(self):
        # Hardware-specific capture code
        image_data = camera.capture()
        msg = Image()
        msg.data = image_data
        self.publisher.publish(msg)
```

**Actuator Controllers** (Subscription Pattern):
```python
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10)
    
    def velocity_callback(self, msg):
        # Convert ROS message to motor commands
        left_speed, right_speed = self.calculate_wheel_speeds(msg)
        motor_left.set_speed(left_speed)
        motor_right.set_speed(right_speed)
```

**Parameter Configuration**:
```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_range', 5.0)
        
        # Get parameter values
        max_speed = self.get_parameter('max_speed').value
        
        # Parameters can be changed at runtime via CLI or tools
        # ros2 param set /configurable_node max_speed 2.0
```

## Hands-On: Building Your First ROS 2 System

### Installation Guide

#### Option 1: Desktop Installation (Recommended)

**For Ubuntu 22.04 (ROS 2 Humble):**
```bash
# 1. Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 4. Source setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**For Windows 10/11:**
```powershell
# 1. Install Chocolatey (if not installed)
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# 2. Install ROS 2
choco feature enable -n=allowGlobalConfirmation
choco install ros-humble-desktop

# 3. Set up environment
refreshenv
```

**For macOS:**
```bash
# 1. Install Homebrew (if not installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 2. Install ROS 2
brew tap ros/deps
brew install ros-humble-desktop

# 3. Source setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

#### Option 2: Docker Container (Isolated Environment)
```bash
# 1. Pull ROS 2 image
docker pull osrf/ros:humble-desktop

# 2. Run container with GUI support
docker run -it --rm \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop
```

#### Option 3: ROS 2 on Raspberry Pi (for Hardware Projects)
```bash
# For Raspberry Pi OS (64-bit)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base
```

### Creating Your First ROS 2 Package

Let's build a complete publisher-subscriber system with multiple nodes communicating through topics.

#### Step 1: Create Workspace and Package
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python --license Apache-2.0 first_ros2_pkg --dependencies rclpy std_msgs

# Navigate to package
cd first_ros2_pkg/first_ros2_pkg
```

#### Step 2: Create Publisher Node
Create `publisher_node.py`:
```python
#!/usr/bin/env python3
"""
Simple publisher node that sends counter messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class CounterPublisher(Node):
    """Publishes incrementing counter values."""
    
    def __init__(self):
        super().__init__('counter_publisher')
        
        # Create publisher for Int32 messages on 'counter' topic
        # QoS: Keep last 10 messages, reliable delivery
        self.publisher_ = self.create_publisher(
            Int32, 
            'counter', 
            10
        )
        
        # Create timer to publish every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize counter
        self.counter = 0
        
        # Log initialization
        self.get_logger().info('Counter publisher node started')
    
    def timer_callback(self):
        """Timer callback: publish counter value."""
        # Create message
        msg = Int32()
        msg.data = self.counter
        
        # Publish message
        self.publisher_.publish(msg)
        
        # Log publication
        self.get_logger().info(f'Published: {self.counter}')
        
        # Increment counter
        self.counter += 1
    
    def destroy_node(self):
        """Cleanup before destruction."""
        self.get_logger().info('Shutting down publisher node')
        super().destroy_node()

def main(args=None):
    """Main function to run the publisher node."""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create publisher node
    publisher_node = CounterPublisher()
    
    try:
        # Keep node running
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        # Handle Ctrl-C gracefully
        publisher_node.get_logger().info('Keyboard interrupt received')
    finally:
        # Cleanup
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Create Subscriber Node
Create `subscriber_node.py`:
```python
#!/usr/bin/env python3
"""
Simple subscriber node that receives counter messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CounterSubscriber(Node):
    """Subscribes to counter values and processes them."""
    
    def __init__(self):
        super().__init__('counter_subscriber')
        
        # Configure QoS for reliable message delivery
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Create subscriber for Int32 messages on 'counter' topic
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            qos_profile
        )
        
        # Initialize statistics
        self.message_count = 0
        self.last_value = None
        self.start_time = self.get_clock().now()
        
        # Log initialization
        self.get_logger().info('Counter subscriber node started')
    
    def listener_callback(self, msg):
        """Callback for received messages."""
        # Process message
        self.message_count += 1
        self.last_value = msg.data
        
        # Calculate message rate
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9
        
        if elapsed_seconds > 0:
            rate = self.message_count / elapsed_seconds
        else:
            rate = 0.0
        
        # Log received message with statistics
        self.get_logger().info(
            f'Received: {msg.data} | '
            f'Total: {self.message_count} | '
            f'Rate: {rate:.2f} Hz'
        )
        
        # Example: Perform action based on value
        self.process_value(msg.data)
    
    def process_value(self, value):
        """Example processing function."""
        if value % 10 == 0:
            self.get_logger().info(f'Special: {value} is divisible by 10!')
        
        if value > 50:
            self.get_logger().warning(f'High value detected: {value}')
    
    def destroy_node(self):
        """Cleanup before destruction."""
        # Log final statistics
        elapsed_time = self.get_clock().now() - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9
        
        self.get_logger().info(
            f'Final statistics: {self.message_count} messages received '
            f'over {elapsed_seconds:.2f} seconds'
        )
        
        super().destroy_node()

def main(args=None):
    """Main function to run the subscriber node."""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create subscriber node
    subscriber_node = CounterSubscriber()
    
    try:
        # Keep node running
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        # Handle Ctrl-C gracefully
        subscriber_node.get_logger().info('Keyboard interrupt received')
    finally:
        # Cleanup
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 4: Create a Processing Node (Intermediate Node)
Create `processing_node.py` to demonstrate message transformation:
```python
#!/usr/bin/env python3
"""
Intermediate node that subscribes, processes, and republishes data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class ProcessingNode(Node):
    """Processes counter values and publishes status strings."""
    
    def __init__(self):
        super().__init__('processing_node')
        
        # Subscribe to counter topic
        self.counter_sub = self.create_subscription(
            Int32,
            'counter',
            self.counter_callback,
            10
        )
        
        # Create publisher for status messages
        self.status_pub = self.create_publisher(
            String,
            'status',
            10
        )
        
        # Log initialization
        self.get_logger().info('Processing node started')
    
    def counter_callback(self, msg):
        """Process counter values and generate status."""
        value = msg.data
        
        # Determine status based on value
        if value < 10:
            status = "Low"
        elif value < 30:
            status = "Medium"
        elif value < 50:
            status = "High"
        else:
            status = "Very High"
        
        # Add parity information
        parity = "even" if value % 2 == 0 else "odd"
        
        # Create status message
        status_msg = String()
        status_msg.data = f"Value {value} is {status} and {parity}"
        
        # Publish status
        self.status_pub.publish(status_msg)
        
        # Log transformation
        self.get_logger().debug(
            f'Transformed {value} -> "{status_msg.data}"'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 5: Create a Multi-Subscriber Node
Create `monitor_node.py` to demonstrate subscribing to multiple topics:
```python
#!/usr/bin/env python3
"""
Monitor node that subscribes to both counter and status topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from rclpy.executors import MultiThreadedExecutor

class MonitorNode(Node):
    """Monitors multiple topics and correlates information."""
    
    def __init__(self):
        super().__init__('monitor_node')
        
        # Buffer for recent messages
        self.last_counter = None
        self.last_status = None
        
        # Subscribe to counter topic
        self.counter_sub = self.create_subscription(
            Int32,
            'counter',
            self.counter_callback,
            10
        )
        
        # Subscribe to status topic
        self.status_sub = self.create_subscription(
            String,
            'status',
            self.status_callback,
            10
        )
        
        # Timer for periodic monitoring report
        self.timer = self.create_timer(2.0, self.monitor_report)
        
        # Log initialization
        self.get_logger().info('Monitor node started')
    
    def counter_callback(self, msg):
        """Handle counter messages."""
        self.last_counter = msg.data
        self.get_logger().debug(f'Counter update: {msg.data}')
    
    def status_callback(self, msg):
        """Handle status messages."""
        self.last_status = msg.data
        self.get_logger().debug(f'Status update: {msg.data}')
    
    def monitor_report(self):
        """Periodic report combining information from both topics."""
        if self.last_counter is not None and self.last_status is not None:
            self.get_logger().info(
                f'MONITOR: Counter={self.last_counter}, '
                f'Status="{self.last_status}"'
            )
        elif self.last_counter is not None:
            self.get_logger().warning(
                f'Counter={self.last_counter} but no status received'
            )
        elif self.last_status is not None:
            self.get_logger().warning(
                f'Status="{self.last_status}" but no counter received'
            )
        else:
            self.get_logger().warning('No messages received yet')

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    
    # Use multi-threaded executor for handling multiple callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Monitor node shutting down')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 6: Update Package Configuration
Update `setup.py`:
```python
from setuptools import setup

package_name = 'first_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='First ROS 2 package with publisher-subscriber examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = first_ros2_pkg.publisher_node:main',
            'subscriber_node = first_ros2_pkg.subscriber_node:main',
            'processing_node = first_ros2_pkg.processing_node:main',
            'monitor_node = first_ros2_pkg.monitor_node:main',
        ],
    },
)
```

Update `package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>first_ros2_pkg</name>
  <version>0.0.0</version>
  <description>First ROS 2 package with publisher-subscriber examples</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### Step 7: Build and Run the System
```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select first_ros2_pkg

# Source the workspace
source install/setup.bash

# Open terminal 1: Run publisher
ros2 run first_ros2_pkg publisher_node

# Open terminal 2: Run subscriber
ros2 run first_ros2_pkg subscriber_node

# Open terminal 3: Run processing node
ros2 run first_ros2_pkg processing_node

# Open terminal 4: Run monitor node
ros2 run first_ros2_pkg monitor_node

# Open terminal 5: View ROS graph
rqt_graph

# Open terminal 6: View topics
ros2 topic list
ros2 topic echo /counter
ros2 topic echo /status

# Open terminal 7: View node information
ros2 node list
ros2 node info /counter_publisher
```

#### Step 8: Advanced Testing with Launch Files
Create `launch/multi_node.launch.py`:
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Publisher node
        Node(
            package='first_ros2_pkg',
            executable='publisher_node',
            name='counter_publisher',
            output='screen',
            parameters=[{'publish_rate': 2.0}]  # Example parameter
        ),
        
        # Subscriber node
        Node(
            package='first_ros2_pkg',
            executable='subscriber_node',
            name='counter_subscriber',
            output='screen'
        ),
        
        # Processing node
        Node(
            package='first_ros2_pkg',
            executable='processing_node',
            name='data_processor',
            output='screen'
        ),
        
        # Monitor node
        Node(
            package='first_ros2_pkg',
            executable='monitor_node',
            name='system_monitor',
            output='screen'
        ),
    ])
```

Run the complete system:
```bash
ros2 launch first_ros2_pkg multi_node.launch.py
```

### Debugging and Monitoring Tools

ROS 2 provides powerful tools for debugging and monitoring your system:

#### Command Line Interface (CLI) Tools
```bash
# List all nodes in the system
ros2 node list

# List all topics
ros2 topic list
ros2 topic list -t  # Show topic types

# Echo topic messages
ros2 topic echo /counter
ros2 topic echo /status

# Get topic info
ros2 topic info /counter --verbose

# Monitor topic frequency
ros2 topic hz /counter

# List services
ros2 service list

# View parameters
ros2 param list
ros2 param get /counter_publisher publish_rate

# Set parameters dynamically
ros2 param set /counter_publisher publish_rate 1.0

# View node info
ros2 node info /counter_publisher

# Bag recording (data logging)
ros2 bag record -o my_recording /counter /status

# Bag playback
ros2 bag play my_recording
```

#### GUI Tools
```bash
# RQt - Modular GUI framework
rqt

# Launch specific plugins
rqt_graph  # Visualize node graph
rqt_console  # View and filter logs
rqt_plot  # Plot numerical topic data
rqt_reconfigure  # Dynamically reconfigure parameters

# RViz2 - 3D visualization tool
rviz2
```

## Objective Achieved: Programmatic Robot Control

Through this hands-on exercise, you've learned how to:

1. **Create independent nodes** that perform specific functions
2. **Establish communication channels** using topics
3. **Transform and process data** between nodes
4. **Monitor system behavior** using ROS 2 tools
5. **Package and deploy** complete ROS 2 systems

These skills form the foundation for controlling actual robot behaviors. The same patterns you used here—publishing sensor data, processing information, and sending commands—scale directly to controlling robot arms, mobile bases, or humanoid robots.

## Real-World Application: From Counter to Controller

Let's see how these patterns apply to actual robotics. Here's a simplified example of a robot controller using the same principles:

```python
#!/usr/bin/env python3
"""
Simplified robot controller demonstrating ROS 2 patterns.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np

class RobotController(Node):
    """Simple obstacle-avoiding robot controller."""
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.obstacle_distance = float('inf')
        self.robot_orientation = 0.0
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
    
    def scan_callback(self, msg):
        """Process LIDAR scan data."""
        # Find minimum distance in front sector
        num_ranges = len(msg.ranges)
        front_start = int(num_ranges * 0.4)  # 40% to 60% is front
        front_end = int(num_ranges * 0.6)
        front_ranges = msg.ranges[front_start:front_end]
        
        # Filter out invalid readings (0.0 or NaN)
        valid_ranges = [r for r in front_ranges if r > msg.range_min and r < msg.range_max]
        
        if valid_ranges:
            self.obstacle_distance = min(valid_ranges)
        else:
            self.obstacle_distance = float('inf')
    
    def imu_callback(self, msg):
        """Process IMU orientation data."""
        # Simplified: Extract yaw from quaternion
        from tf_transformations import euler_from_quaternion
        orientation_q = msg.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        ])
        self.robot_orientation = yaw
    
    def control_loop(self):
        """Main control loop."""
        cmd_msg = Twist()
        
        # Simple obstacle avoidance
        if self.obstacle_distance < 0.5:  # 0.5m threshold
            # Too close - turn right
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Radians per second
            self.get_logger().warning(f'Obstacle at {self.obstacle_distance:.2f}m - turning')
        else:
            # Safe - move forward
            cmd_msg.linear.x = 0.2  # m/s
            cmd_msg.angular.z = 0.0
        
        # Publish command
        self.cmd_pub.publish(cmd_msg)
    
    def destroy_node(self):
        """Send stop command before shutting down."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices and Production Considerations

### 1. Node Design Principles
- **Single Responsibility**: Each node should do one thing well
- **Stateless When Possible**: Design nodes to be restartable without persistent state
- **Graceful Degradation**: Handle missing data and partial failures
- **Configurable Parameters**: Use ROS parameters for tunable values

### 2. Performance Optimization
```python
# Use efficient data structures
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class EfficientImageProcessor(Node):
    def __init__(self):
        super().__init__('efficient_processor')
        self.bridge = CvBridge()
        
        # Pre-allocate arrays when possible
        self.kernel = np.ones((3, 3), np.float32) / 9
        
        # Use appropriate QoS
        qos = QoSProfile(
            depth=1,  # Minimal queue for real-time
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
    def process_image(self, msg):
        # Convert ROS image to numpy array efficiently
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Use vectorized operations
        processed = cv2.filter2D(cv_image, -1, self.kernel)
        
        return processed
```

### 3. Security Considerations
```bash
# Enable ROS 2 security
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ROOT_DIRECTORY=~/ros2_security

# Generate security keys
ros2 security generate_artifacts -k ~/ros2_security/ca.key.pem \
  -c ~/ros2_security/ca.cert.pem \
  -p my_robot_policy.xml \
  -e ~/ros2_security

# Run nodes with security
ros2 run first_ros2_pkg publisher_node \
  --ros-args --enclave ~/ros2_security/publisher
```

### 4. Testing and CI/CD
```python
# Example test for publisher node
import pytest
import rclpy
from std_msgs.msg import Int32
from first_ros2_pkg.publisher_node import CounterPublisher

class TestPublisherNode:
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = CounterPublisher()
    
    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_publisher_creation(self):
        # Test that publisher was created
        assert self.node.publisher_ is not None
        assert self.node.publisher_.topic_name == 'counter'
    
    def test_counter_increment(self):
        # Test counter increments correctly
        initial = self.node.counter
        self.node.timer_callback()
        assert self.node.counter == initial + 1
```

## Key Takeaways

1. **ROS 2 is middleware**, not an operating system, providing communication infrastructure for robotic systems
2. **Nodes communicate via topics** (async), **services** (sync), and **actions** (async with feedback)
3. **Quality of Service policies** allow tuning communication reliability and performance
4. **Modular design** enables scalable, maintainable robotic systems
5. **ROS 2 tools** provide comprehensive debugging and monitoring capabilities

## Common Pitfalls and Solutions

| Pitfall | Solution |
|---------|----------|
| Nodes not finding each other | Check namespace, ensure both are in same ROS domain (set ROS_DOMAIN_ID) |
| Messages delayed or lost | Adjust QoS policies, increase queue depth, use RELIABLE delivery |
| High CPU usage | Use appropriate executors, avoid blocking in callbacks |
| Memory leaks | Ensure proper cleanup in destroy_node(), use managed nodes |
| Timing issues | Use ROS 2 clocks for time synchronization |

## Further Reading

1. **Official Documentation**: [docs.ros.org](https://docs.ros.org)
2. **ROS 2 Design Articles**: [design.ros2.org](https://design.ros2.org)
3. **Book**: "Mastering ROS 2" by Lentin Joseph (Packt Publishing)
4. **Courses**: ROS 2 Fundamentals on [The Construct](https://www.theconstructsim.com)
5. **Community**: [ROS Answers](https://answers.ros.org), [ROS Discourse](https://discourse.ros.org)

## Discussion Questions

1. How does ROS 2's distributed architecture facilitate the development of complex robotic systems compared to monolithic approaches?
2. What are the trade-offs between using topics versus services for inter-node communication in a real-time control system?
3. How might QoS policies be configured differently for a surgical robot versus an autonomous vacuum cleaner?
4. What security considerations become important when ROS 2 systems are deployed in public or shared environments?

## Exercises

1. **Extended Publisher-Subscriber System**: Modify the example to include a service that resets the counter when called, and an action that counts to a specified number with progress feedback.

2. **Performance Analysis**: Create a system with multiple publishers at different rates and a subscriber that measures jitter and latency. Visualize the results using `rqt_plot`.

3. **Error Handling Implementation**: Enhance the nodes with comprehensive error handling for scenarios like lost connections, invalid data, and resource exhaustion.

4. **Real-world Interface Simulation**: Create a node that simulates sensor data (e.g., fake LIDAR scans) and another that processes this data to detect "obstacles," publishing avoidance commands.

5. **System Integration**: Combine the ROS 2 system from this chapter with the Gazebo simulation from Chapter 1, creating a complete simulated robot that can be controlled via ROS 2 topics.

---

*Next Chapter Preview: In Chapter 3, we'll explore simulation environments—Gazebo for physics-based simulation and Unity for high-fidelity visualization. You'll learn to create humanoid models, simulate walking gaits, and test robotic behaviors in virtual environments before deploying to physical hardware.*
