---
sidebar_position: 2
---

# ROS 2 Fundamentals

Deep dive into advanced ROS 2 concepts for building production-ready robotic systems.

## Custom Messages

Create custom message types for specialized data:

### Define a Custom Message

```msg
# RobotStatus.msg
string robot_id
float32 battery_level
float32 speed
geometry_msgs/Pose pose
bool is_moving
```

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

## Launch Files

Launch files start multiple nodes with configuration:

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

**Run:**
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## Parameters

Dynamic configuration without recompiling:

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

**Set parameters:**
```bash
ros2 run my_pkg node --ros-args -p update_rate:=20.0 -p robot_name:=RobotX
```

## TF2 - Coordinate Transforms

TF2 manages coordinate frames for sensors and robot parts:

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

## URDF - Robot Description

Unified Robot Description Format defines robot structure:

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

**Visualize in RViz:**
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_robot.urdf
```

## Building a Mobile Robot Controller

Complete example integrating multiple concepts:

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

Configure communication reliability:

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

Manage node states for robust systems:

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

1. **Use Namespaces**: Organize multi-robot systems
2. **Parameter Files**: Separate configuration from code
3. **Error Handling**: Always validate inputs and handle exceptions
4. **Logging**: Use ROS 2 logging (DEBUG, INFO, WARN, ERROR, FATAL)
5. **Testing**: Write unit tests with `launch_testing`

## Real-World Application: Warehouse Robot

Combining all concepts for a functional warehouse robot:

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
