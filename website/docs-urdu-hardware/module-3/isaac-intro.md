---
sidebar_position: 1
---

# NVIDIA Isaac کی تعارف

**NVIDIA Isaac** روبوٹکس کی ترقی کو تیز کرنے والا ایک مصنوعی ذہانت روبوٹ پلیٹ فارم ہے جو جی پی یو سے چلنے والی حسیات، محاکات اور ہیرا فیری کی سہولت فراہم کرتا ہے۔

## آئزک پلیٹ فارم کے اجزاء

### 1. آئزک سم
این ویڈیا اومنورس پر مبنی فوٹو ریالسٹک روبوٹ سمولیٹر:

- **ری ٹریسنگ**: جسمانی طور پر درست روشنی
- **فزکس**: فزکس ایکس 5.0 انجن
- **سنٹیٹک ڈیٹا**: مصنوعی ذہانت کے لئے بالکل درست ڈیٹا

### 2. آئزک ROS
ہارڈویئر ایکسیلریٹڈ ROS 2 پیکیجز:

- **VSLAM**: بصری همزمان مقام شناسی اور نقشہ سازی
- **آبجیکٹ ڈیٹیکشن**: ریئل ٹائم مصنوعی ذہانت کا اندازہ
- **نویگیشن**: نیویگیشن 2 انٹیگریشن

### 3. آئزک SDK
روبوٹکس ایپلی کیشنز کے لئے لائبریریاں:

- **ہیرا فیری**: گھیرنے کا منصوبہ بندی، تحریک کا کنٹرول
- **حسیات**: 3D باز سازی، علیحدگی
- **نویگیشن**: راستہ کی منصوبہ بندی، رکاوٹوں سے بچاؤ

## NVIDIA آئزک کیوں؟
| روایتی نقطہ نظر | آئزک پلیٹ فارم |
|---------------------|----------------|
| سی پی یو پر مبنی پروسیسنگ | جی پی یو ایکسیلریٹڈ (50x تیز تر) |
| دستی ڈیٹا تخلیق | سنٹیٹک ڈیٹا جنریشن |
| علیحدہ ٹرین/ڈپلوی | متحد ورک فل |
| محدود سکیلنگ | کلاؤڈ/ایج ڈپلویمنٹ |

## آئزک سم سیٹ اپ کرنا

### سسٹم کی ضروریات

- **جی پی یو**: این ویڈیا آر ٹی ایکس 2080 ٹی آئی یا زیادہ
- **وی آر اے ایم**: 8GB کم از کم، 24GB سفارش کی گئی
- **RAM**: 32GB کم از کم
- **او ایس**: ابونٹو 20.04/22.04 یا ونڈوز 10/11

### انسٹالیشن

```bash
# این ویڈیا اومنورس سے ڈاؤن لوڈ کریں
# https://www.nvidia.com/en-us/omniverse/apps/isaac-sim/

# یا ڈاکر کا استعمال کریں
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# آئزک سم چلائیں
./isaac-sim.sh
```

## آئزک سم میں روبوٹ بنانا

### یو آر ڈی ایف درآمد کریں

```python
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage

# روبوٹ یو آر ڈی ایف لوڈ کریں
robot_path = "/path/to/robot.urdf"
add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")
```

### سینسرز شامل کریں

```python
from omni.isaac.sensor import Camera

# کیمرہ بنائیں
camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,
    resolution=(1920, 1080)
)
```

## آئزک ROS حقیقی روبوٹس کے لئے

### بصری SLAM (VSLAM)

```bash
# آئزک ROS VSLAM انسٹال کریں
sudo apt install ros-humble-isaac-ros-visual-slam

# VSLAM لانچ کریں
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**مفت خصوصیات:**
- ریئل ٹائم 6DOF پوزیشن کا اندازہ
- لوپ بند ڈیٹیکشن
- جیٹسن پر ہارڈویئر ایکسیلریٹڈ

### آبجیکٹ ڈیٹیکشن

```bash
# آئزک ROS DNN انفرنس انسٹال کریں
sudo apt install ros-humble-isaac-ros-dnn-inference

# آبجیکٹ ڈیٹیکشن چلائیں
ros2 launch isaac_ros_detectnet detectnet.launch.py \
    model:=peoplenet
```

## سنٹیٹک ڈیٹا جنریشن

آئزک سم میں بالکل درست تربیتی ڈیٹا جنریٹ کریں:

```python
import omni.replicator.core as rep

# کیمرہ بنائیں
camera = rep.create.camera(position=(5, 5, 5))

# روشنی کو رینڈمائز کریں
with rep.trigger.on_frame():
    rep.randomizer.light_intensity(
        lights=rep.get.prims(semantics="light"),
        min_value=500,
        max_value=2000
    )

# ڈیٹا کیپچر کریں
rep.orchestrator.run()
```

## نیویگیشن آئزک کے ساتھ

نیویگیشن 2 + آئزک کا استعمال کرتے ہوئے خودکار طور پر نیویگیشن کریں:

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0
```

```bash
# نیویگیشن لانچ کریں
ros2 launch nav2_bringup navigation_launch.py
```

## آئزک کے ساتھ ہیرا فیری

### گھیرنے کی منصوبہ بندی

```python
from omni.isaac.manipulators import Gripper

gripper = Gripper(prim_path="/World/Robot/Gripper")

# گھیرنے کی منصوبہ بندی کریں
target_position = [0.5, 0.0, 0.3]
gripper.apply_action(target_position)
```

### موشن پلاننگ (RMPflow)

```python
from omni.isaac.motion_generation import ArticulationMotionPolicy

policy = ArticulationMotionPolicy(
    robot_articulation,
    physics_dt
)

# تصادم سے بچنے والی ٹریکٹری جنریٹ کریں
trajectory = policy.compute_joint_targets(target_pose)
```

## آئزک سم کے ساتھ تربیت

### رینفورسمنٹ لیرننگ

```python
from omni.isaac.gym.vec_env import VecEnvBase

class RobotEnv(VecEnvBase):
    def reset(self):
        # روبوٹ کو ابتدائی حالت میں ریسٹ کریں
        pass
    
    def step(self, actions):
        # ایکشنز کو لاگو کریں، مبصرین، انعامات واپس کریں
        pass
```

## کلید لینے والے نکات

✅ آئزک سم فوٹو ریالسٹک سمولیٹر فراہم کرتا ہے  
✅ آئزک ROS حسیات کو جی پی یو سے تیز کرتا ہے  
✅ سنٹیٹک ڈیٹا دستی لیبلنگ کو ختم کرتا ہے  
✅ متحد پلیٹ فارم کے لئے تربیت-سمولیٹ-ڈپلوی  

**اگلا:** [آئزک ایڈوانسڈ موضوعات →](./isaac-advanced)