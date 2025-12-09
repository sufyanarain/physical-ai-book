---
sidebar_position: 1
---

# روبوٹ کے محاکات کا تعارف

محاکات روبوٹکس میں نظریہ اور عمل کے درمیان پل ہے۔ مہنگی ہارڈویئر کو تعینات کرنے سے پہلے، ہم **ڈیجیٹل جڑواں** کہلانے والے ورتوئل ماحول میں تجربات کرتے ہیں۔

## کیوں محاکات کریں؟

### فوائد

| پہلو | اصل روبوٹ | محاکات |
|--------|-----------|------------|
| **لاگت** | $10k-$100k+ | مفت |
| **خطرہ** | ہارڈویئر کی خرابی | کوئی خطرہ نہیں |
| **تکرار کی رفتار** | گھنٹے | سیکنڈ |
| **تجرباتی مناظر** | محدود | لامحدود |
| **ڈیبگنگ** | مشکل | مکمل بصیرت |

### صنعت کے معیار کے اوزار

- **گیزبو**: طبیعیات کا محاکات
- **یونٹی**: اعلی درجے کی پیشکش
- **آئزک سم**: این ویڈیا کی فوٹو ریالسٹک پلیٹ فارم
- **ویبوٹس**: मलٹی روبوٹ محاکات

## گیزبو کلاسک بمقابلہ گیزبو (آگنیشن)

**گیزبو کلاسک** ختم ہو رہا ہے۔ **نیا گیزبو (پہلے آگنیشن)** پیش کرتا ہے:

- بہتر کارکردگی
- بہتر طبیعیات کے انجن
- بہتر سینسر ماڈلز
- ویب پر مبنی بصری شکل

## طبیعیات کے انجن

گیزبو کئی طبیعیات کے انجن کی حمایت کرتا ہے:

### او ڈی ای (اوپن ڈائنامکس انجن)
- گیزبو کلاسک میں ڈیفالٹ
- بنیادی محاکات کے لیے اچھا
- معقول درستگی

### بلٹ
- تیز رفتار تصادم کی پہچان
- کھیلوں اور روبوٹکس میں استعمال ہوتا ہے

### ڈی اے آر ٹی (ڈائنامک اینیمیشن اور روبوٹکس ٹول کٹ)
- اعلی درستگی
- پیچیدہ رابطے کی حرکیات

### مثال: طبیعیات کی تشکیل

```xml
<world name="default">
  <physics type="ode">
    <real_time_update_rate>1000</real_time_update_rate>
    <max_step_size>0.001</max_step_size>
  </physics>
  
  <gravity>0 0 -9.81</gravity>
</world>
```

## اپنا پہلا گیزبو ورلڈ بنائیں

### سادہ ورلڈ فائل

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="robot_world">
    <!-- سورج کو روشنی کے لیے -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- زمینی ہوا -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- ایک باکس رکاوٹ شامل کریں -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
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
            <ambient>0.8 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**لانچ:**
```bash
gazebo robot_world.world
```

## سینسر محاکات

### لیڈار (لیزر اسکینر)

```xml
<sensor type="ray" name="lidar">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### کیمرہ

```xml
<sensor type="camera" name="camera1">
  <update_rate>30.0</update_rate>
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>1920</width>
      <height>1080</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.02</near>
      <far>300</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image</remapping>
    </ros>
  </plugin>
</sensor>
```

### آئی ایم یو (انرٹیل میزرمینٹ یونٹ)

```xml
<sensor name="imu_sensor" type="imu">
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu</remapping>
    </ros>
  </plugin>
</sensor>
```

## موبائل روبوٹ ماڈل کی تعمیر

کامل ایس ڈی ایف ایک ڈفرینشل ڈرائیو روبوٹ کے لیے:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="diff_drive_robot">
    <!-- بیس لنک -->
    <link name="base_link">
      <inertial>
        <mass>10</mass>
        <inertia>
          <ixx>0.2</ixx>
          <iyy>0.2</iyy>
          <izz>0.4</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
        </material>
      </visual>
    </link>
    
    <!-- بائیں پہیہ -->
    <link name="left_wheel">
      <pose>0 0.2 0 -1.5707 0 0</pose>
      <inertial>
        <mass>1</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
    
    <!-- بائیں پہیے کے لیے جوڑ -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <!-- ڈفرینشل ڈرائیو پلاگ ان -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </model>
</sdf>
```

## ROS 2 کے ساتھ انضمام

### ROS 2 کے ساتھ گیزبو لانچ کریں

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            'gazebo_ros/launch/gazebo.launch.py',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', 'robot.sdf'],
        ),
    ])
```

## اہم نکات

✅ محاکات وقت اور پیسہ بچاتا ہے  
✅ گیزبو حقیقی طبیعیات اور سینسروں کی پیشکش کرتا ہے  
✅ ایس ڈی ایف فارمیٹ روبوٹ ماڈلز کی تعریف کرتا ہے  
✅ ROS 2 انضمام سے مسلسل تجربات ممکن ہوتے ہیں  

**اگلا:** [گیزبو اور یونٹی ایڈوانسڈ →](./gazebo-unity)