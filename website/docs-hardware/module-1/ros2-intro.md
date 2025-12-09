---
sidebar_position: 1
---

# Introduction to ROS 2

**ROS 2** (Robot Operating System 2) is the middleware backbone of modern robotics, serving as the "nervous system" that connects sensors, actuators, and intelligence. To understand ROS 2, it's essential to grasp the concepts of software development, as it is built on top of various programming languages and software frameworks.

## Why ROS 2?

### The Evolution from ROS 1

ROS 2 addresses critical limitations of its predecessor:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Architecture** | Master-based (single point of failure) | Peer-to-peer (distributed) |
| **Real-time** | Limited support | Full real-time capabilities |
| **Security** | Minimal | Built-in DDS security |
| **Platforms** | Linux-focused | Cross-platform (Linux, Windows, macOS) |
| **Multi-robot** | Complex | Native support |

Think of the architecture change from master-based to peer-to-peer as moving from a centralized hub to a distributed network. This change allows for more robust and scalable systems, similar to how a distributed power grid can provide more reliable electricity supply.

### Industry Adoption

Major robotics companies using ROS 2:
- **BMW**: Factory automation
- **Boston Dynamics**: Spot robot
- **Toyota**: Research platforms
- **NASA**: Space exploration

## Core Concepts

### 1. Nodes

**Nodes** are independent processes that perform specific tasks. Think of them as specialized workers in a factory:

- **Camera Node**: Captures images
- **Planning Node**: Decides robot actions
- **Motor Controller Node**: Controls wheel speeds

In software development, nodes can be thought of as separate threads or processes, each with its own responsibility. This concept is similar to the idea of modular design in electronics, where each module has a specific function.

```python
# Simple ROS 2 Node in Python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Topics

**Topics** are communication channels where nodes publish and subscribe to messages:

```mermaid
graph LR
    A[Camera Node] -->|Image Messages| B[/camera/image_raw Topic]
    B --> C[Image Processor]
    B --> D[Object Detector]
```

**Example: Publishing to a Topic**

```python
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
    def publish_status(self):
        msg = String()
        msg.data = 'Robot is active!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

Think of topics as a one-to-many communication channel, similar to a radio broadcast. Nodes can publish messages to a topic, and multiple nodes can subscribe to the same topic to receive those messages.

### 3. Services

**Services** provide request-response communication for short operations:

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
```

Services are like a phone call, where a node can request a specific action or information from another node, and receive a response.

### 4. Actions

**Actions** are for long-running tasks with feedback (e.g., navigating to a goal):

- **Goal**: Target position
- **Feedback**: Progress updates
- **Result**: Success/failure status

Actions are like a project management system, where a node can request a complex task to be performed, and receive updates on the progress and final result.

## ROS 2 Architecture

```
┌─────────────────────────────────────────────┐
│           Application Layer                 │
│  (Your Nodes: Navigation, Perception, etc.) │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│              ROS 2 Client Layer             │
│        (rclpy, rclcpp - APIs)               │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│           DDS Middleware Layer              │
│  (Fast-DDS, Cyclone DDS - Communication)    │
└─────────────────────────────────────────────┘
```

The ROS 2 architecture is like a layered cake, with the application layer at the top, the client layer in the middle, and the middleware layer at the bottom. Each layer has its own responsibility, and they work together to provide a robust and scalable system.

## Setting Up ROS 2

### Installation (Ubuntu 22.04)

```bash
# Add ROS 2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Create Your First Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a package
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Practical Example: Temperature Monitor

Let's build a complete system with publisher and subscriber:

### Publisher (Temperature Sensor)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TempSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(2.0, self.publish_temperature)
        
    def publish_temperature(self):
        temp = Float32()
        temp.data = 20.0 + random.uniform(-5, 5)  # Simulate reading
        self.publisher.publish(temp)
        self.get_logger().info(f'Temperature: {temp.data:.2f}°C')

def main():
    rclpy.init()
    sensor = TempSensor()
    rclpy.spin(sensor)
    rclpy.shutdown()
```

### Subscriber (Monitor)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TempMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32, 'temperature', self.temp_callback, 10
        )
        
    def temp_callback(self, msg):
        if msg.data > 25.0:
            self.get_logger().warn(f'HIGH TEMP: {msg.data:.2f}°C!')
        else:
            self.get_logger().info(f'Normal: {msg.data:.2f}°C')

def main():
    rclpy.init()
    monitor = TempMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()
```

### Running the System

```bash
# Terminal 1: Run sensor
ros2 run my_robot_pkg temp_sensor

# Terminal 2: Run monitor
ros2 run my_robot_pkg temp_monitor

# Terminal 3: Inspect topics
ros2 topic list
ros2 topic echo /temperature
```

## Key Takeaways

✅ **ROS 2 is the industry standard** for robot middleware  
✅ **Nodes** are independent processes that communicate via topics, services, and actions  
✅ **DDS middleware** provides reliable, real-time communication  
✅ **Python and C++** are the primary languages for ROS 2 development  

## Next Steps

In the next section, we'll dive deeper into **ROS 2 fundamentals**, including:
- Message types and custom messages
- Launch files for complex systems
- Parameters and configuration
- TF2 for coordinate transforms

**Continue:** [ROS 2 Fundamentals →](./ros2-fundamentals)

---

## Further Reading

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Principles](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)