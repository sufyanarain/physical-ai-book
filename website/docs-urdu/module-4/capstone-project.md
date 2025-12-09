---
sidebar_position: 2
---

# کپ اسٹون منصوبہ: خودکار ہیومینوئڈ

کورس کے تمام تصورات کو یکجا کرتے ہوئے ایک مکمل خودکار ہیومینوئڈ روبوٹ سسٹم تیار کریں۔

## منصوبہ کا جائزہ

ایک模拟ی ہیومینوئڈ روبوٹ بنائیں جو:

1. **آواز کے احکامات** (جیسے "کمرہ صاف کریں") وصول کرے
2. **LLM کا استعمال کرتے ہوئے کارروائیوں کی ترتیب منصوبہ بندی** کرے
3. **VSLAM اور Nav2 کا استعمال کرتے ہوئے رکاوٹوں سے بچے**
4. **کمپیوٹر وژن کا استعمال کرتے ہوئے اشیاء کی شناخت** کرے
5. **باجو کے کنٹرول کا استعمال کرتے ہوئے اشیاء کو ہاتھ سے پکڑے**

## سسٹم کی تعمیر

```
┌─────────────────────────────────────────────────┐
│              آواز کے احکامات کا داخلہ                │
│         (OpenAI Whisper API)                    │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│         شناختی منصوبہ بندی کی تہہ                       │
│         (GPT-4 ٹاسک تجزیہ)                      │
└──────────────────┬──────────────────────────────┘
                   │
         ┌─────────┴─────────┐
         │                   │
┌────────▼────────┐  ┌──────▼──────────┐
│   ادراک    │  │   Навигация    │
│  (YOLO/Isaac)   │  │   (Nav2/VSLAM)  │
└────────┬────────┘  └──────┬──────────┘
         │                   │
         └─────────┬─────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│           کارروائی کی تعمیل کی تہہ                │
│     (ROS 2 کنٹرولرز + Gazebo/Isaac)          │
└─────────────────────────────────────────────────┘
```

## نافذاتی مراحل

### مرحلہ 1: ماحول کی ترتیب

```bash
# ورکشاپ بنائیں
mkdir -p ~/humanoid_project/src
cd ~/humanoid_project

# ہیومینوئڈ ماڈل (جیسے Unitree G1) کلون کریں
git clone https://github.com/unitreerobotics/unitree_ros2.git src/

# بلڈ کریں
colcon build
source install/setup.bash
```

### مرحلہ 2: آواز کے احکامات کا ہینڈلر

```python
# voice_handler.py
import whisper
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceHandler(Node):
    def __init__(self):
        super().__init__('voice_handler')
        self.whisper = whisper.load_model("base")
        self.openai = OpenAI()
        
        # پبلشر برائے احکامات
        self.cmd_pub = self.create_publisher(String, 'voice_commands', 10)
        
        # ٹائمر برائے مائیک انپٹ
        self.create_timer(5.0, self.listen)
    
    def listen(self):
        # آڈیو ریکارڈ کریں (pyaudio/sounddevice کا استعمال کریں)
        audio_file = self.record_audio(duration=3)
        
        # ٹرانسکرائب کریں
        result = self.whisper.transcribe(audio_file)
        command = result["text"]
        
        self.get_logger().info(f"Command: {command}")
        
        # پبلش کریں
        self.cmd_pub.publish(String(data=command))

def main():
    rclpy.init()
    node = VoiceHandler()
    rclpy.spin(node)
```

### مرحلہ 3: ٹاسک پلانر

```python
# task_planner.py
from openai import OpenAI
import json

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.openai = OpenAI()
        
        # سبسکرائب برائے آواز کے احکامات
        self.cmd_sub = self.create_subscription(
            String, 'voice_commands', self.plan_callback, 10
        )
        
        # پبلشر برائے کارروائی کی ترتیب
        self.action_pub = self.create_publisher(String, 'action_sequence', 10)
    
    def plan_callback(self, msg):
        command = msg.data
        
        # کارروائی کی ترتیب تیار کریں
        plan = self.generate_plan(command)
        
        # JSON کے طور پر پبلش کریں
        self.action_pub.publish(String(data=json.dumps(plan)))
    
    def generate_plan(self, command):
        prompt = f"""
        اس احکام کو روبوٹ کی کارروائیوں کی ترتیب میں تبدیل کریں:
        Command: "{command}"
        
        دستیاب کارروائیاں:
        - navigate_to(x, y)
        - detect_objects(category)
        - pick_object(object_id)
        - place_object(x, y, z)
        - open_gripper()
        - close_gripper()
        
        JSON ایरے کے ساتھ کارروائیوں کو واپس کریں۔
        """
        
        response = self.openai.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "روبوٹ ٹاسک پلانر"},
                {"role": "user", "content": prompt}
            ]
        )
        
        plan_text = response.choices[0].message.content
        # JSON کو جواب سے اخذ کریں
        plan = json.loads(plan_text)
        
        return plan
```

### مرحلہ 4: Навигация ماڈیول

```python
# navigator.py
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.navigator = BasicNavigator()
        
        # سبسکرائب برائے Навигация احکامات
        self.nav_sub = self.create_subscription(
            PoseStamped, 'navigate_to', self.navigate_callback, 10
        )
    
    def navigate_callback(self, goal_pose):
        self.get_logger().info(f'ناوگیشن کر رہا ہے: {goal_pose.pose.position}')
        
        # ابتدائی پوزیشن سیٹ کریں
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        self.navigator.setInitialPose(initial_pose)
        
        # Nav2 کو सकریو ہونے کا انتظار کریں
        self.navigator.waitUntilNav2Active()
        
        # گول بھیجیں
        self.navigator.goToPose(goal_pose)
        
        # پیش رفت کی نگرانی کریں
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'باقی دوری: {feedback.distance_remaining}')
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('ناوگیشن کامیاب!')
        else:
            self.get_logger().error('ناوگیشن ناکام!')
```

### مرحلہ 5: اشیاء کی شناخت

```python
# object_detector.py
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
        # سبسکرائب برائے کیمرہ
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.detect_callback, 10
        )
        
        # پبلشر برائے شناخت شدہ اشیاء
        self.obj_pub = self.create_publisher(String, 'detected_objects', 10)
    
    def detect_callback(self, msg):
        # ROS ایمیج کو OpenCV میں تبدیل کریں
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # اشیاء کی شناخت کریں
        results = self.model(cv_image)
        
        # شناخت شدہ اشیاء کو اخذ کریں
        detections = []
        for result in results:
            for box in result.boxes:
                detection = {
                    'class': result.names[int(box.cls)],
                    'confidence': float(box.conf),
                    'bbox': box.xyxy[0].tolist()
                }
                detections.append(detection)
        
        # پبلش کریں
        self.obj_pub.publish(String(data=json.dumps(detections)))
```

### مرحلہ 6: ہاتھ سے پکڑنے والا کنٹرولر

```python
# manipulator.py
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')
        
        # MoveIt ایکشن کلائنٹ
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        
        # سبسکرائب برائے ہاتھ سے پکڑنے کے احکامات
        self.manip_sub = self.create_subscription(
            String, 'manipulation_command', self.manip_callback, 10
        )
    
    def manip_callback(self, msg):
        command = json.loads(msg.data)
        
        if command['action'] == 'pick_object':
            self.pick(command['object_id'])
        elif command['action'] == 'place_object':
            self.place(command['position'])
    
    def pick(self, object_id):
        # اشیاء کی پوزیشن حاصل کریں
        position = self.get_object_position(object_id)
        
        # گریپ پلان کریں
        goal = MoveGroup.Goal()
        goal.request.goal_constraints = self.create_grasp_constraints(position)
        
        # ایگزیکٹ کریں
        self.move_client.send_goal_async(goal)
```

### مرحلہ 7: میں کنٹرولر

```python
# main_controller.py
class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # سبسکرائب برائے کارروائی کی ترتیب
        self.action_sub = self.create_subscription(
            String, 'action_sequence', self.execute_sequence, 10
        )
        
        # کارروائی پبلشرز
        self.nav_pub = self.create_publisher(PoseStamped, 'navigate_to', 10)
        self.manip_pub = self.create_publisher(String, 'manipulation_command', 10)
    
    def execute_sequence(self, msg):
        actions = json.loads(msg.data)
        
        for action in actions:
            self.execute_action(action)
            
            # تکمیل کا انتظار کریں
            time.sleep(1)
    
    def execute_action(self, action):
        if action['type'] == 'navigate_to':
            pose = self.create_pose(action['params'])
            self.nav_pub.publish(pose)
        
        elif action['type'] == 'pick_object':
            cmd = json.dumps(action)
            self.manip_pub.publish(String(data=cmd))
```

## لانچ فائل

```python
# humanoid_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gazebo/Isaac Sim
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['humanoid_world.world']
        ),
        
        # آواز کے احکامات کا ہینڈلر
        Node(package='humanoid_project', executable='voice_handler'),
        
        # ٹاسک پلانر
        Node(package='humanoid_project', executable='task_planner'),
        
        # Навигация
        Node(package='humanoid_project', executable='navigator'),
        
        # اشیاء کی شناخت
        Node(package='humanoid_project', executable='object_detector'),
        
        # ہاتھ سے پکڑنے والا
        Node(package='humanoid_project', executable='manipulator'),
        
        # میں کنٹرولر
        Node(package='humanoid_project', executable='main_controller'),
    ])
```

## ڈیمو سیناریوز

### سیناریو 1: "کمرہ صاف کریں"

1. **آواز**: "کمرہ صاف کریں"
2. **منصوبہ**: [ناوگیشن کر رہا ہے(کمرہ), اشیاء کی شناخت(کچرا), اشیاء کو اٹھائیں(کچرا_1), ناوگیشن کر رہا ہے(ڈسٹ بن), اشیاء کو رکھیں(ڈسٹ بن)]
3. **ایگزیکٹ**: روبوٹ ناوگیشن کرتا ہے، اشیاء کی شناخت کرتا ہے، انہیں اٹھاتا ہے، ڈسٹ بن میں رکھتا ہے

### سیناریو 2: "مجھے پانی لے آؤ"

1. **آواز**: "مجھے پانی لے آؤ"
2. **منصوبہ**: [ناوگیشن کر رہا ہے(کچن), اشیاء کی شناخت(بوتل), اشیاء کو اٹھائیں(بوتل_1), ناوگیشن کر رہا ہے(صارف), اشیاء کو رکھیں(میز)]
3. **ایگزیکٹ**: روبوٹ پانی لے کر آتا ہے

## جائزہ معیار

- **آواز کی شناخت کی درستی** (90% سے زیادہ)
- **ٹاسک کی تکمیل کی شرح** (80% سے زیادہ)
- **ناوگیشن کی حفاظت** (0 تصادم)
- **اشیاء کی شناخت کی درستی** (85% سے زیادہ)
- **ایگزیکشن کا وقت** (ہر ٹاسک کے لیے 5 منٹ سے کم)

## اہم نکات

✅ تمام 4 ماڈیولز کو ایک سسٹم میں یکجا کیا  
✅ آواز سے کارروائی کی پوری پائپ لائن  
✅ حقیقی دنیا کی ٹاسک کی ایگزیکشن  
✅ ماڈیولر، قابل توسیع تعمیر  

## مبارک ہو! 🎉

آپ نے فزیکل AI اور ہیومینوئڈ روبوٹکس کی کتاب کا اختتام کر لیا ہے۔ اب آپ کے پاس یہ مہارت ہے:

- روبوٹس کو ROS 2 کے ساتھ بنائیں
- Gazebo اور Isaac کے ساتھ محاکات کریں
- ادراک اور منصوبہ بندی کے لیے AI کو یکجا کریں
- آواز کنٹرول والے خودکار سسٹم بنائیں

**سیکھنے کا سلسلہ جاری رکھیں** اور روبوٹکس کے مستقبل کی تعمیر کریں!

---

## اگلا قدم

- [ROS Discourse](https://discourse.ros.org/) میں شامل ہوں
- [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS) میں حصہ لیں
- اپنے منصوبے بنائیں
- کمیونٹی کے ساتھ شیئر کریں

**سوال؟** چیٹ بوٹ سے پوچھیں! →