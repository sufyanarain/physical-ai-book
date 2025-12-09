---
sidebar_position: 1
---

# Introduction to Robot Simulation

Simulation is the bridge between theory and practice in robotics. Before deploying expensive hardware, we test in virtual environments called **Digital Twins**. Think of a Digital Twin like a virtual prototype of your robot, where you can test and refine its behavior without the risks and costs associated with physical prototypes.

## Why Simulate?

### Benefits

| Aspect | Real Robot | Simulation |
|--------|-----------|------------|
| **Cost** | $10k-$100k+ | Free |
| **Risk** | Hardware damage | Zero risk |
| **Iteration Speed** | Hours | Seconds |
| **Testing Scenarios** | Limited | Unlimited |
| **Debugging** | Difficult | Full visibility |

Simulation offers numerous benefits, including reduced costs, minimized risk, faster iteration, and enhanced debugging capabilities. This is similar to how software developers use virtual environments and debuggers to test and refine their code.

### Industry Standard Tools

- **Gazebo**: Physics simulation
- **Unity**: High-fidelity rendering
- **Isaac Sim**: NVIDIA's photorealistic platform
- **Webots**: Multi-robot simulation

These tools provide a range of features, from basic physics simulations to advanced rendering and multi-robot simulations. Each tool has its strengths and is suited for different types of projects.

## Gazebo Classic vs. Gazebo (Ignition)

**Gazebo Classic** is being phased out. **New Gazebo (formerly Ignition)** offers:

- Better performance
- Improved physics engines
- Enhanced sensor models
- Web-based visualization

The new Gazebo offers significant improvements over the classic version, including better performance, more accurate physics engines, and enhanced sensor models. This is similar to how software frameworks evolve over time, with new versions offering improved performance and features.

## Physics Engines

Gazebo supports multiple physics engines:

### ODE (Open Dynamics Engine)
- Default in Gazebo Classic
- Good for basic simulations
- Moderate accuracy

### Bullet
- Fast collision detection
- Used in games and robotics

### DART (Dynamic Animation and Robotics Toolkit)
- High accuracy
- Complex contact dynamics

Physics engines are responsible for simulating the physical behavior of objects in the virtual environment. They take into account factors like gravity, friction, and collisions to create a realistic simulation. This is similar to how game engines use physics engines to simulate realistic gameplay.

### Example: Physics Configuration

```xml
<world name="default">
  <physics type="ode">
    <real_time_update_rate>1000</real_time_update_rate>
    <max_step_size>0.001</max_step_size>
  </physics>
  
  <gravity>0 0 -9.81</gravity>
</world>
```

In this example, we're configuring the physics engine to use the ODE engine, with a real-time update rate of 1000 and a maximum step size of 0.001. We're also setting the gravity to 9.81 m/s^2, which is the standard value for Earth's gravity.

## Creating Your First Gazebo World

### Simple World File

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="robot_world">
    <!-- Sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Add a box obstacle -->
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

In this example, we're creating a simple world with a sun, ground plane, and a box obstacle. The box is defined with a pose, collision geometry, and visual properties.

**Launch:**
```bash
gazebo robot_world.world
```

To launch the world, we simply run the `gazebo` command with the world file as an argument.

## Sensor Simulation

### LiDAR (Laser Scanner)

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

In this example, we're defining a LiDAR sensor with a pose, update rate, and ray properties. The sensor is configured to produce a 360-degree scan with a resolution of 1 degree and a range of 0.1 to 30 meters.

### Camera

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

In this example, we're defining a camera sensor with an update rate, horizontal field of view, image properties, and clip properties. The sensor is configured to produce a 1920x1080 image with a format of R8G8B8.

### IMU (Inertial Measurement Unit)

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

In this example, we're defining an IMU sensor with an update rate, angular velocity, and linear acceleration properties. The sensor is configured to produce a Gaussian noise model with a mean of 0 and a standard deviation of 2e-4 for the angular velocity and 1.7e-2 for the linear acceleration.

## Building a Mobile Robot Model

Complete SDF for a differential drive robot:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="diff_drive_robot">
    <!-- Base Link -->
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
    
    <!-- Left Wheel -->
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
    
    <!-- Joint for left wheel -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <!-- Differential Drive Plugin -->
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

In this example, we're defining a differential drive robot with a base link, left wheel, and right wheel. The robot has a mass of 10 kg and a inertia tensor. The left wheel has a mass of 1 kg and a collision geometry. The joint between the base link and the left wheel is a revolute joint with an axis of rotation along the y-axis.

## Integration with ROS 2

### Launch Gazebo with ROS 2

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

In this example, we're launching Gazebo with ROS 2 using the `launch` module. We're including the `gazebo.launch.py` file and spawning the robot entity using the `spawn_entity.py` executable.

## Key Takeaways

✅ Simulation saves time and money  
✅ Gazebo provides realistic physics and sensors  
✅ SDF format defines robot models  
✅ ROS 2 integration enables seamless testing  

**Next:** [Gazebo & Unity Advanced →](./gazebo-unity)