---
sidebar_position: 2
---

# Gazebo & Unity Integration

Advanced simulation techniques combining Gazebo's physics with Unity's visualization.

## Unity for Robotics

Unity offers photorealistic rendering and VR/AR capabilities:

- **Unity Robotics Hub**: ROS integration
- **Perception package**: Synthetic data generation
- **ML-Agents**: Reinforcement learning

### Setup Unity-ROS Bridge

```bash
# Install Unity-ROS packages
sudo apt install ros-humble-ros-tcp-endpoint
```

## Sim-to-Real Transfer

Key challenges in moving from simulation to real robots:

1. **Physics Gap**: Real-world friction, air resistance
2. **Sensor Noise**: Simulate realistic noise models
3. **Domain Randomization**: Vary lighting, textures, dynamics

### Domain Randomization Example

```xml
<world name="randomized">
  <scene>
    <ambient>
      <uniform min="0.3" max="1.0"/>
    </ambient>
  </scene>
  
  <physics>
    <gravity>
      <uniform min="9.7" max="9.9" axis="z"/>
    </gravity>
  </physics>
</world>
```

## Realistic Sensor Models

### Depth Camera with Noise

```xml
<sensor type="depth" name="depth_camera">
  <camera>
    <horizontal_fov>1.5708</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
</sensor>
```

## Multi-Robot Simulation

Simulate robot swarms and collaboration:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = []
    for i in range(5):
        robots.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'robot_{i}',
                    '-x', str(i * 2),
                    '-y', '0',
                    '-z', '0',
                ],
            )
        )
    return LaunchDescription(robots)
```

## Key Takeaways

✅ Unity provides high-fidelity visualization  
✅ Sim-to-real requires careful calibration  
✅ Domain randomization improves transfer  

**Next Module:** [NVIDIA Isaac →](../module-3/isaac-intro)
