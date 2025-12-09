---
sidebar_position: 2
---

# گیزبو اور یونٹی انٹیگریشن

گیزبو کی فزکس اور یونٹی کی وضاحت کے ساتھ اعلیٰ سمولیشن ٹیکنیکس۔

## یونٹی برائے روبوٹکس

یونٹی فوٹو ریالسٹک رینڈرنگ اور وی آر/اے آر کی صلاحیتوں کی پیشکش کرتی ہے:

- **یونٹی روبوٹکس ہب**: ROS انٹیگریشن
- **پرسپشن پیکیج**: سنٹھیٹک ڈیٹا جنریشن
- **ایم ایل ایجنٹس**: رینفورسمنٹ لیرننگ

### یونٹی-ROS برج سیٹ اپ

```bash
# یونٹی-ROS پیکیج انسٹال کریں
sudo apt install ros-humble-ros-tcp-endpoint
```

## سمولیشن سے حقیقی روبوٹس کی منتقلی

سمولیشن سے حقیقی روبوٹس کی طرف جانے میں اہم چیلنجز:

1. **فزکس گپ**: حقیقی دنیا کی摩擦، ہوا کا مزاحم
2. **سینسر نوائس**: حقیقی نوائس ماڈلز کی سمولیشن
3. **ڈومین رینڈمائزیشن**: روشنی، ٹیکسچرز، ڈائنامکس کی تبدیلی

### ڈومین رینڈمائزیشن کا مثال

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

## حقیقی سینسر ماڈلز

### ڈیپتھ کیمرہ کے ساتھ نوائس

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

## ملٹی روبوٹ سمولیشن

روبوٹ سوارم اور تعاون کی سمولیشن:

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

## اہم نکات

✅ یونٹی اعلیٰ معیار کی وضاحت فراہم کرتی ہے  
✅ سمولیشن سے حقیقی روبوٹس کی منتقلی کے لیے محتاط کیلیبریشن کی ضرورت ہے  
✅ ڈومین رینڈمائزیشن منتقلی میں بہتری لاتا ہے  

**اگلا ماڈیول:** [NVIDIA Isaac →](../module-3/isaac-intro)