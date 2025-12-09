---
sidebar_position: 1
---

# Introduction to Robot Simulation

Simulation is the bridge between theory and practice in robotics. Before deploying expensive hardware, we test in virtual environments called **Digital Twins**. Think of a digital twin like a prototype in the hardware world, where you can test and refine your design before manufacturing the final product.

## Why Simulate?

### Benefits

| Aspect | Real Robot | Simulation |
|--------|-----------|------------|
| **Cost** | $10k-$100k+ | Free |
| **Risk** | Hardware damage | Zero risk |
| **Iteration Speed** | Hours | Seconds |
| **Testing Scenarios** | Limited | Unlimited |
| **Debugging** | Difficult | Full visibility |

Simulation offers numerous benefits, including reduced costs, minimized risk, and accelerated iteration. In the hardware world, you can think of simulation as a way to test your design without wasting materials or damaging equipment.

### Industry Standard Tools

- **Gazebo**: Physics simulation
- **Unity**: High-fidelity rendering
- **Isaac Sim**: NVIDIA's photorealistic platform
- **Webots**: Multi-robot simulation

These tools provide a range of features, from basic physics simulations to advanced rendering and multi-robot simulations. Gazebo, in particular, is a popular choice for robotics simulation due to its flexibility and customizability.

## Gazebo Classic vs. Gazebo (Ignition)

**Gazebo Classic** is being phased out. **New Gazebo (formerly Ignition)** offers:

- Better performance
- Improved physics engines
- Enhanced sensor models
- Web-based visualization

The new Gazebo (Ignition) provides significant improvements over the classic version, including better performance, more accurate physics simulations, and enhanced sensor models. This is similar to upgrading from an older hardware component to a newer, more efficient one.

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

These physics engines can be thought of as different algorithms for simulating real-world physics. Each engine has its strengths and weaknesses, and the choice of engine depends on the specific requirements of your simulation.

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

This code snippet shows how to configure the physics engine in Gazebo. The `physics` tag specifies the type of physics engine to use (in this case, ODE), and the `real_time_update_rate` and `max_step_size` tags control the simulation's time step and update rate.

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

This SDF file defines a simple world with a sun, ground plane, and a box obstacle. The `model` tag defines the box's pose, collision, and visual properties.

**Launch:**
```bash
gazebo robot_world.world
```

This command launches the Gazebo simulation with the specified world file.

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

This code snippet defines a LiDAR sensor with a horizontal scan range of 360 degrees and a resolution of 1 degree. The `plugin` tag specifies the ROS plugin for the LiDAR sensor.

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

This code snippet defines a camera sensor with a horizontal field of view (FOV) of 1.396 radians and an image resolution of 1920x1080 pixels.

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

This code snippet defines an IMU sensor with Gaussian noise models for angular velocity and linear acceleration.

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

This SDF file defines a differential drive robot with two wheels and a base link. The `plugin` tag specifies the ROS plugin for the differential drive controller.

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

This code snippet defines a ROS 2 launch file that launches Gazebo and spawns a robot entity.

## Key Takeaways

✅ Simulation saves time and money  
✅ Gazebo provides realistic physics and sensors  
✅ SDF format defines robot models  
✅ ROS 2 integration enables seamless testing  

**Next:** [Gazebo & Unity Advanced →](./gazebo-unity)

In this tutorial, we covered the basics of robot simulation using Gazebo and ROS 2. We learned how to create a simple world, add sensors and models, and integrate with ROS 2. We also discussed the benefits of simulation and the different physics engines available in Gazebo. With this knowledge, you can start building your own robot simulations and testing your algorithms in a virtual environment.