---
sidebar_position: 2
---

# ROS 2 بنیادیات

ایڈوانسڈ ROS 2 کے تصورات کا گہرا جائزہ لینے کے لیے تیار ہو جائیں تاکہ پروڈکشن ریڈی روبوٹک سسٹم تیار کیے جا سکیں۔

## کسٹم میسجز

ROS 2 میں، ہم **کسٹم میسجز** کا استعمال کرکے خصوصی ڈیٹا ٹائپس بنا سکتے ہیں۔ یہ ہمیں روبوٹک سسٹم کے لیے ضروری ڈیٹا کو منظم اور موثر طریقے سے منتقل کرنے کی سہولت دیتا ہے۔

### کسٹم میسج کا تعارف

ایک کسٹم میسج بنانے کے لیے، ہم ایک `.msg` فائل میں ڈیٹا فیلڈز کی تعریف کرتے ہیں۔ مثال کے طور پر، ہم ایک `RobotStatus` میسج بنا سکتے ہیں جو روبوٹ کی حالت کے بارے میں معلومات کو شامل کرتا ہے۔

```msg
# RobotStatus.msg
string robot_id
float32 battery_level
float32 speed
geometry_msgs/Pose pose
bool is_moving
```

### کسٹم میسجز کا استعمال

اب ہم اپنے کسٹم میسج کو پیٹھون میں استعمال کر سکتے ہیں۔ ہم ایک نود بناتے ہیں جو `RobotStatus` میسج کو پبلش کرتا ہے۔

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

## لانچ فائلیں

ROS 2 میں، **لانچ فائلیں** ہمیں ملٹی نود سسٹم کو آسان طریقے سے شروع کرنے کی سہولت دیتی ہیں۔ یہ ہمیں کنفیگریشن کے ساتھ کئی نودز کو ایک ساتھ شروع کرنے کی اجازت دیتا ہے۔

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

**چلائیں:**
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## پیرامیٹرز

ROS 2 میں، **پیرامیٹرز** ہمیں ڈائنامک کنفیگریشن کی سہولت دیتے ہیں۔ ہم نودز کو ری کامپائلنگ کے بغیر پیرامیٹرز کو ایڈجسٹ کر سکتے ہیں۔

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # پیرامیٹرز کا اعلان
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('robot_name', 'default_robot')
        
        # پیرامیٹر کی VALUES حاصل کریں
        self.rate = self.get_parameter('update_rate').value
        self.name = self.get_parameter('robot_name').value
        
        self.get_logger().info(f'ریٹ: {self.rate}, نام: {self.name}')
```

**پیرامیٹرز سیٹ کریں:**
```bash
ros2 run my_pkg node --ros-args -p update_rate:=20.0 -p robot_name:=RobotX
```

## TF2 - کوآرڈینیٹ ٹرانسفارمز

ROS 2 میں، **ٹی ایف 2** ہمیں کوآرڈینیٹ ٹرانسفارمز کو منیج کرنے کی سہولت دیتا ہے۔ یہ ہمیں روبوٹ کے مختلف حصوں کے درمیان کوآرڈینیٹ فریمز کو تبدیل کرنے کی اجازت دیتا ہے۔

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
        
        # پوزیشن
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.3
        
        # روٹیشن (کواٹرنیون)
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)
```

## URDF - روبوٹ کی وضاحت

ROS 2 میں، **یو آر ڈی ایف** ہمیں روبوٹ کی ساخت کو định義 کرنے کی سہولت دیتا ہے۔ یہ ہمیں روبوٹ کے لنکس، جوائنٹس، اور دیگر خصوصیات کو واضح کرنے کی اجازت دیتا ہے۔

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- بیس لنک -->
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
  
  <!-- کیمرہ لنک -->
  <link name="camera_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <!-- جوائنٹ جو بیس کو کیمرہ سے جوڑتا ہے -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

**RViz میں وضاحت کریں:**
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_robot.urdf
```

## موبائل روبوٹ کنٹرولر کی تعمیر

ROS 2 میں، ہم **موبائل روبوٹ کنٹرولر** کو بنانے کے لیے کئی تصورات کو یکجا کر سکتے ہیں۔ یہ ہمیں روبوٹ کو کنٹرول کرنے کی سہولت دیتا ہے۔

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # پبلشرز
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # سبسکرائبرز
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        
        # پیرامیٹرز
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
        # ٹارگٹ تک کی دوری کا حساب لگائیں
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        cmd = Twist()
        
        if distance > 0.1:  # ٹارگٹ پر نہیں
            # ٹارگٹ کی طرف کا زاویہ کا حساب لگائیں
            angle_to_target = math.atan2(dy, dx)
            
            cmd.linear.x = self.get_parameter('linear_speed').value
            cmd.angular.z = angle_to_target
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('ٹارگٹ حاصل کر لیا!')
        
        self.cmd_vel_pub.publish(cmd)
```

## کوالیٹی آف سروس (QoS)

ROS 2 میں، **کوالیٹی آف سروس (QoS)** ہمیں کمیونیکیشن کی قابل اعتمادیت کو کنٹرول کرنے کی سہولت دیتا ہے۔ یہ ہمیں ڈیٹا کی منتقلی کی کیفیت کو ایڈجسٹ کرنے کی اجازت دیتا ہے۔

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# قابل اعتماد کمیونیکیشن (اہم ڈیٹا)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# بہترین کوشش (سینسر ڈیٹا، نقصان کو सहن کر سکتا ہے)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

self.pub = self.create_publisher(Image, 'camera/image', sensor_qos)
```

## لائف سائیکل نودز

ROS 2 میں، **لائف سائیکل نودز** ہمیں روبوسٹ سسٹم کے لیے نودز کی حالتوں کو منیج کرنے کی سہولت دیتا ہے۔ یہ ہمیں نودز کی زندگی کے مختلف مراحل کو کنٹرول کرنے کی اجازت دیتا ہے۔

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class ManagedNode(LifecycleNode):
    def on_configure(self, state):
        self.get_logger().info('کنفیگریشن...')
        # وسائل سیٹ اپ کریں
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        self.get_logger().info('ایکٹیویٹنگ...')
        # آپریشنز شروع کریں
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        self.get_logger().info('ڈی ایکٹیویٹنگ...')
        # آپریشنز کو روکیں
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info('کلین اپ...')
        # وسائل کو ریلیز کریں
        return TransitionCallbackReturn.SUCCESS
```

## بہترین پریکٹسز

1. **نام سپیسز کا استعمال کریں**: ملٹی روبوٹ سسٹم کو منظم کریں
2. **پیرامیٹر فائلیں**: کنفیگریشن کو کوڈ سے الگ کریں
3. **ایrror ہینڈلنگ**: ہمیشہ انپٹس کی توثیق کریں اور اپوشن کو ہینڈل کریں
4. **لاگنگ**: ROS 2 لاگنگ (DEBUG, INFO, WARN, ERROR, FATAL) کا استعمال کریں
5. **ٹیسٹنگ**: `launch_testing` کے ساتھ یونٹ ٹیسٹ لکھیں

## حقیقی دنیا کی ایپلی کیشن: وہاؤس روبوٹ

ROS 2 کو حقیقی دنیا کی ایپلی کیشن میں استعمال کرنے کے لیے، ہم **وہاؤس روبوٹ** کو بنانے کے لیے کئی تصورات کو یکجا کر سکتے ہیں۔ یہ ہمیں وہاؤس میں سامان کو منتقل کرنے کے لیے روبوٹ کو کنٹرول کرنے کی سہولت دیتا ہے۔

```python
class WarehouseRobot(Node):
    def __init__(self):
        super().__init__('warehouse_robot')
        
        # پبلشرز
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        
        # سبسکرائبرز
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.task_sub = self.create_subscription(
            String, 'tasks', self.task_callback, 10
        )
        
        # سروسز
        self.emergency_stop_srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback
        )
        
        # ٹی ایف
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ریاست
        self.state = 'IDLE'  # IDLE, MOVING, LOADING, ERROR
    
    def laser_callback(self, msg):
        # رکاوٹ سے بچنے کی منطق
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            self.stop()
            self.get_logger().warn('رکاوٹ کا پتہ چلا!')
    
    def task_callback(self, msg):
        # نیا ٹاسک پروسس کریں
        task = msg.data
        self.get_logger().info(f'نیا ٹاسک: {task}')
        self.execute_task(task)
    
    def emergency_stop_callback(self, request, response):
        self.stop()
        self.state = 'ERROR'
        response.success = True
        response.message = 'ایمرجنسی اسٹاپ ایکٹیویٹڈ'
        return response
```

## کلیدی نکات

✅ **کسٹم میسجز** خصوصی ڈیٹا ٹائپس کو فعال کرتے ہیں  
✅ **لانچ فائلیں** ملٹی نود سسٹم کو آسان بناتی ہیں  
✅ **پیرامیٹرز** ڈائنامک کنفیگریشن فراہم کرتے ہیں  
✅ **ٹی ایف 2** کوآرڈینیٹ ٹرانسفارمز کو منیج کرتا ہے  
✅ **یو آر ڈی ایف** روبوٹ کی ساخت کو định義 کرتا ہے  
✅ **کیو او ایس** کمیونیکیشن کی قابل اعتمادیت کو کنٹرول کرتا ہے  

**اگلا ماڈیول:** [دی ڈیجیٹل ٹون (گیزبو اور یونٹی) →](../module-2/simulation-intro)

---

## وسائل

- [ROS 2 ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 ہمبل API ڈاکس](https://docs.ros2.org/humble/api/)
- [ٹی ایف 2 ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)