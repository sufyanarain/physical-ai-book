---
sidebar_position: 2
---

# ROS 2 Fundamentals

Deep dive into advanced ROS 2 concepts for building production-ready robotic systems. As a software developer, you're likely familiar with programming concepts like object-oriented programming (OOP), algorithms, and data structures. However, working with robots requires understanding hardware and electronics concepts, such as circuits, sensors, actuators, motors, and mechanical systems. In this documentation, we'll provide explanations and analogies to help you bridge the gap between software and hardware.

## Custom Messages

Create custom message types for specialized data. Think of custom messages like creating a new data structure in your programming language. Just as you would define a class or struct to hold specific data, custom messages allow you to define a new message type that can be used to communicate between nodes in your ROS 2 system.

### Define a Custom Message

```msg
# RobotStatus.msg
string robot_id
float32 battery_level
float32 speed
geometry_msgs/Pose pose
bool is_moving
```

In this example, we're defining a custom message called `RobotStatus` that contains fields for the robot's ID, battery level, speed, pose, and a boolean indicating whether the robot is moving. This message can be used to publish the robot's status to other nodes in the system.

### Using Custom Messages

```python
from my_interfaces.msg import RobotStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.pub = self.create_publisher(RobotStatus, 'robot/status', 10)
        
    def publish_status(self):
        msg = RobotStatus()
        msg.robot_id = 'ROBOT_001'
        msg.battery_level = 85.5
        msg.speed = 1.2
        msg.is_moving = True
        self.pub.publish(msg)
```

Here, we're creating a node that publishes the `RobotStatus` message to a topic called `robot/status`. We create an instance of the `RobotStatus` message, fill in the fields, and then publish it to the topic.

## Launch Files

Launch files start multiple nodes with configuration. Think of launch files like a script that sets up and runs multiple programs at the same time. Just as you would write a script to automate tasks on your computer, launch files automate the process of starting and configuring nodes in your ROS 2 system.

```python
# robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='sensor_node',
            name='camera_sensor',
            parameters=[{'frame_rate': 30}]
        ),
        Node(
            package='my_robot_pkg',
            executable='controller_node',
            name='motor_controller',
            parameters=[{'max_speed': 2.0}]
        ),
    ])
```

In this example, we're defining a launch file that starts two nodes: `camera_sensor` and `motor_controller`. We specify the package and executable for each node, as well as any parameters that need to be set.

**Run:**
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## Parameters

Dynamic configuration without recompiling. Parameters are like variables that can be set and changed at runtime, without having to recompile your code. Just as you would use environment variables or command-line arguments to configure your program, parameters allow you to configure your ROS 2 nodes.

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('robot_name', 'default_robot')
        
        # Get parameter values
        self.rate = self.get_parameter('update_rate').value
        self.name = self.get_parameter('robot_name').value
        
        self.get_logger().info(f'Rate: {self.rate}, Name: {self.name}')
```

Here, we're creating a node that declares two parameters: `update_rate` and `robot_name`. We get the values of these parameters and log them to the console.

**Set parameters:**
```bash
ros2 run my_pkg node --ros-args -p update_rate:=20.0 -p robot_name:=RobotX
```

## TF2 - Coordinate Transforms

TF2 manages coordinate frames for sensors and robot parts. Think of TF2 like a system that helps you keep track of where everything is in 3D space. Just as you would use a mapping system to navigate a city, TF2 helps your robot navigate its environment by managing coordinate frames.

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
    
    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Position
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.3
        
        # Rotation (quaternion)
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)
```

In this example, we're creating a node that publishes a transform between two frames: `base_link` and `camera_link`. We set the position and rotation of the transform and send it to the TF2 system.

## URDF - Robot Description

Unified Robot Description Format defines robot structure. Think of URDF like a blueprint for your robot. Just as you would use a blueprint to design and build a house, URDF allows you to define the structure and components of your robot.

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Joint connecting base to camera -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

In this example, we're defining a simple robot with two links: `base_link` and `camera_link`. We define the visual properties of each link, as well as a joint that connects them.

**Visualize in RViz:**
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_robot.urdf
```

## Building a Mobile Robot Controller

Complete example integrating multiple concepts. This example shows how to create a mobile robot controller that uses many of the concepts we've covered so far.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 5.0
        self.target_y = 5.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def control_loop(self):
        # Calculate distance to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        cmd = Twist()
        
        if distance > 0.1:  # Not at target
            # Calculate angle to target
            angle_to_target = math.atan2(dy, dx)
            
            cmd.linear.x = self.get_parameter('linear_speed').value
            cmd.angular.z = angle_to_target
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Target reached!')
        
        self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    controller = MobileRobotController()
    rclpy.spin(controller)
    rclpy.shutdown()
```

## Quality of Service (QoS)

Configure communication reliability. QoS is like setting the priority of your emails. Just as you would set the priority of an email to ensure it gets delivered quickly, QoS allows you to set the priority of your messages to ensure they get delivered reliably.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable communication (important data)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Best effort (sensor data, can tolerate loss)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

self.pub = self.create_publisher(Image, 'camera/image', sensor_qos)
```

## Lifecycle Nodes

Manage node states for robust systems. Lifecycle nodes are like a state machine that manages the state of your node. Just as you would use a state machine to manage the state of a complex system, lifecycle nodes allow you to manage the state of your node and ensure it's always in a valid state.

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class ManagedNode(LifecycleNode):
    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        # Setup resources
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        self.get_logger().info('Activating...')
        # Start operations
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        # Pause operations
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
```

## Best Practices

1. **Use Namespaces**: Organize multi-robot systems. Namespaces are like folders that help you organize your files. Just as you would use folders to organize your files, namespaces help you organize your nodes and topics.
2. **Parameter Files**: Separate configuration from code. Parameter files are like configuration files that allow you to separate your configuration from your code. Just as you would use a configuration file to configure your program, parameter files allow you to configure your nodes.
3. **Error Handling**: Always validate inputs and handle exceptions. Error handling is like having a safety net that catches any errors that might occur. Just as you would use try-catch blocks to handle exceptions in your code, error handling ensures that your node can recover from any errors that might occur.
4. **Logging**: Use ROS 2 logging (DEBUG, INFO, WARN, ERROR, FATAL). Logging is like keeping a diary that records what's happening in your node. Just as you would use logging to debug your code, ROS 2 logging allows you to log messages at different levels of severity.
5. **Testing**: Write unit tests with `launch_testing`. Testing is like checking your work to make sure it's correct. Just as you would write unit tests to ensure your code is working correctly, `launch_testing` allows you to write tests for your nodes.

## Real-World Application: Warehouse Robot

Combining all concepts for a functional warehouse robot. This example shows how to create a warehouse robot that uses many of the concepts we've covered so far.

```python
class WarehouseRobot(Node):
    def __init__(self):
        super().__init__('warehouse_robot')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.task_sub = self.create_subscription(
            String, 'tasks', self.task_callback, 10
        )
        
        # Services
        self.emergency_stop_srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback
        )
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State
        self.state = 'IDLE'  # IDLE, MOVING, LOADING, ERROR
    
    def laser_callback(self, msg):
        # Obstacle avoidance logic
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            self.stop()
            self.get_logger().warn('Obstacle detected!')
    
    def task_callback(self, msg):
        # Process new task
        task = msg.data
        self.get_logger().info(f'New task: {task}')
        self.execute_task(task)
    
    def emergency_stop_callback(self, request, response):
        self.stop()
        self.state = 'ERROR'
        response.success = True
        response.message = 'Emergency stop activated'
        return response
```

## Key Takeaways

✅ **Custom messages** enable specialized data types  
✅ **Launch files** simplify multi-node systems  
✅ **Parameters** provide dynamic configuration  
✅ **TF2** manages coordinate transformations  
✅ **URDF** defines robot structure  
✅ **QoS** controls communication reliability  

**Next Module:** [The Digital Twin (Gazebo & Unity) →](../module-2/simulation-intro)

---

## Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Humble API Docs](https://docs.ros2.org/humble/api/)
- [TF2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)