---
sidebar_position: 1
---

# وژن-لینگویج-ایکشن (VLA)

بڑے لینگویج ماڈلز (LLMs) اور روبوٹکس کا انضمام قدرتی زبان روبوٹ کنٹرول کو ممکن بناتا ہے۔

## وژن-لینگویج-ایکشن (VLA) کیا ہے؟

**وژن-لینگویج-ایکشن** ماڈلز تینوں اجزاء کو ملا کر بنائے جاتے ہیں:

- **وژن**: ماحول کا مشاہدہ کرنا (کیمرے، ڈیپتھ سینسرز)
- **لینگویج**: قدرتی زبان کے احکامات کو سمجھنا
- **ایکشن**: جسمانی کاموں کو انجام دینا

### مثال کا فلو

```
یوزر: "لال کپ کو اٹھا کر میز پر رکھ دو"
    ↓
[LLM] → انٹینٹ کو پارس کرنا → "پک_اینڈ_پلیس(اوبجیکٹ='لال_کپ', لوکیشن='میز')"
    ↓
[وژن] → لال کپ کی پوزیشن کو ٹریک کرنا → پوزیشن: (x=0.5, y=0.2, z=0.1)
    ↓
[ایکشن] → ہاتھ کو موو کرنا → گریپ کرنا → ٹرانسپورٹ کرنا → رلیز کرنا
```

## وائس-ٹو-ایکشن پائپ لائن

### 1. اسپیکر ریگنیشن (وھسپر)

```python
import whisper

model = whisper.load_model("base")

# آڈیو کو ٹرانسکرائب کرنا
result = model.transcribe("command.wav")
text = result["text"]
print(f"کمانڈ: {text}")
```

### 2. انٹینٹ انڈراسٹینڈنگ (GPT-4)

```python
from openai import OpenAI

client = OpenAI()

response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "آپ ایک روبوٹ ٹاسک پلانر ہیں۔ قدرتی زبان کو روبوٹ ایکشنز میں تبدیل کریں۔"},
        {"role": "user", "content": text}
    ]
)

action_plan = response.choices[0].message.content
```

### 3. ایکشن ایگزیکوشن (ROS 2)

```python
class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        self.action_client = ActionClient(self, MoveArm, 'move_arm')
    
    def execute_plan(self, plan):
        # پلان کو پارس کرنا اور ایکشنز کو ایگزیکوٹ کرنا
        for action in parse_actions(plan):
            self.send_goal(action)
```

## کنورسیشنل روبوٹ بنانا

پورا سسٹم انٹیگریشن:

```python
import rclpy
from rclpy.node import Node
from openai import OpenAI
import whisper
from std_msgs.msg import String

class ConversationalRobot(Node):
    def __init__(self):
        super().__init__('conversational_robot')
        
        # ماڈلز کو انیشیلائز کرنا
        self.whisper_model = whisper.load_model("base")
        self.openai_client = OpenAI()
        
        # پبلشرز
        self.action_pub = self.create_publisher(String, 'robot/actions', 10)
        
        # سبسکرائبرز
        self.audio_sub = self.create_subscription(
            AudioData, 'microphone', self.audio_callback, 10
        )
    
    def audio_callback(self, msg):
        # سٹیپ 1: اسپیکر کو ٹیکسٹ میں تبدیل کرنا
        text = self.whisper_model.transcribe(msg.data)["text"]
        self.get_logger().info(f"سنی گئی بات: {text}")
        
        # سٹیپ 2: ایل ایل ایم پلاننگ
        action = self.plan_action(text)
        
        # سٹیپ 3: ایگزیکوٹ کرنا
        self.action_pub.publish(String(data=action))
    
    def plan_action(self, command):
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "روبوٹ ٹاسک پلانر"},
                {"role": "user", "content": command}
            ]
        )
        return response.choices[0].message.content
```

## ملٹی موڈل پرسیپشن

وژن اور لینگویج کو ملا کر:

```python
from transformers import pipeline

# وژن-لینگویج ماڈل
vl_model = pipeline("image-to-text", model="Salesforce/blip-image-captioning-large")

# امیج کو پروسس کرنا
image = capture_camera()
description = vl_model(image)[0]['generated_text']

# ایل ایل ایم کے لیے کنٹیکسٹ
context = f"موجودہ سین: {description}. یوزر کمانڈ: {user_command}"
```

## کوگنیٹیو پلاننگ

ایل ایل ایم کو کمپلیکس ٹاسکس کو سٹیپز میں توڑتا ہے:

```python
def cognitive_plan(task):
    prompt = f"""
    اس ٹاسک کو روبوٹ ایکشنز میں توڑیں:
    ٹاسک: {task}
    
    دستیاب ایکشنز:
    - move_to(x, y, z)
    - grasp_object(object_id)
    - release_object()
    - rotate(angle)
    
    ایک JSON لسٹ کو واپس کریں۔
    """
    
    response = llm.generate(prompt)
    actions = json.loads(response)
    return actions
```

### مثال

```
انپٹ: "ٹیبل کو صاف کریں"

آؤٹ پٹ:
[
  {"action": "move_to", "params": {"x": 0, "y": 0, "z": 0.5}},
  {"action": "detect_objects", "params": {"category": "تراش"}},
  {"action": "grasp_object", "params": {"object_id": "obj_001"}},
  {"action": "move_to", "params": {"x": 1, "y": 1, "z": 0.5}},
  {"action": "release_object", "params": {}}
]
```

## سیفٹی اور گراؤنڈنگ

سیف ایگزیکوشن کو یقینی بنائیں:

```python
def validate_action(action, safety_rules):
    # ورکسپیس بانڈز کو چیک کریں
    if not in_workspace(action.position):
        return False
    
    # کالیشن کو چیک کریں
    if will_collide(action):
        return False
    
    # فورس لمیٹس کو چیک کریں
    if action.force > MAX_FORCE:
        return False
    
    return True
```

## ریئل ورلڈ ایپلی کیشنز

### 1. ہوم اسسٹنٹ روبوٹ

```python
commands = [
    "مجھے کچن سے پانی لے کر دو",
    "لونگ روم میں لائٹس کو آن کریں",
    "میرا فون ڈھونڈیں"
]
```

### 2. وئیر ہاؤس روبوٹ

```python
commands = [
    "شیلف اے3 سے آئٹمز کو اٹھا کر اسٹیشن 5 پر ڈیلیور کریں",
    "شیلف بی2 کو اسٹیجنگ ایریا سے باکسز سے ری سٹاک کریں",
    "ایسل 7 میں انوینٹری کو اسکین کریں"
]
```

### 3. ہیلتھ کیئر روبوٹ

```python
commands = [
    "رووم 302 کو میڈیسن ڈیلیور کریں",
    "لابی میں وہیل چیئر لے کر آئیں",
    "ریڈیولوجی میں پیشنٹ کو اسکورت کریں"
]
```

## کی ٹیک آوے

✅ وژن-لینگویج-ایکشن وژن، لینگویج، اور ایکشن کو ملا کر بنایا جاتا ہے  
✅ ایل ایل ایمز قدرتی زبان روبوٹ کنٹرول کو ممکن بناتے ہیں  
✅ سیفٹی والیڈیشن بہت اہم ہے  
✅ ملٹی موڈل پرسیپشن سمجھ کو بہتر بناتی ہے  

**اگلا:** [کپسٹون پروجیکٹ →](./capstone-project)