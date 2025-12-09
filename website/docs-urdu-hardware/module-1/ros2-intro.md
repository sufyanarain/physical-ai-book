---
sidebar_position: 1
---

# ROS 2 کا تعارف

**ROS 2** (روبوٹ آپریٹنگ سسٹم 2) جدید روبوٹکس کے لیے درمیانی بنیادی ڈھانچہ ہے۔ یہ روبوٹ کے مختلف حصوں کو بے مثال طور پر بات چیت کرنے کی اجازت دیتا ہے، جس سے یہ "عصبی نظام" بن جاتا ہے جو سینسرز، ایکٹیوٹرز اور ذہانت کو جوڑتا ہے۔

## ROS 2 کیوں؟

### ROS 1 سے ارتقاء

ROS 2 اپنے پیشرو کی طرف سے اہم پابندیوں کا جواب دیتا ہے:

| خصوصیت | ROS 1 | ROS 2 |
|---------|-------|-------|
| **آرکیٹیکچر** | ماسٹر بیسڈ (ایک نقطہ کی ناکامی) | پیئر ٹو پیئر (分散) |
| **حقیقی وقت** | محدود حمایت | مکمل حقیقی وقت کی صلاحیت |
| **سیکیورٹی** | کم از کم | ڈی ڈی ایس سیکیورٹی کے ساتھ بنایا گیا |
| **پلیٹ فارم** | لینکس پر توجہ مرکوز | کراس پلیٹ فارم (لینکس، ونڈوز، مک او ایس) |
| **ملٹی روبوٹ** | پیچیدہ | مقامی حمایت |

### صنعت کی اپنائی

بڑی روبوٹکس کمپنیاں جو ROS 2 استعمال کر رہی ہیں:
- **BMW**: فیکٹری کی خودکاریت
- **بوسٹن ڈائنامکس**: اسپاٹ روبوٹ
- **ٹویوٹا**: تحقیقی پلیٹ فارم
- **ناسا**: خلا کی کھوج

## بنیادی تصورات

### 1. نوڈز

**نوڈز** آزاد پروسس ہیں جو مخصوص کام کرتے ہیں۔ انہیں ایک فیکٹری میں مہارت کے کارکنوں کے طور پر سوچیں:

- **کیمرہ نوڈ**: تصاویر کو پکڑتا ہے
- **منصوبہ بندی نوڈ**: روبوٹ کے اعمال کا فیصلہ کرتا ہے
- **موتر کنٹرولر نوڈ**: پہیوں کی رفتار کو کنٹرول کرتا ہے

```python
# ROS 2 نوڈ کو پیٹھون میں
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('ROS 2 سے ہیلو!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. موضوعات

**موضوعات** بات چیت کے چینلز ہیں جہاں نوڈز پیغامات کو شائع اور سبسکرائب کرتے ہیں:

```mermaid
graph LR
    A[کیمرہ نوڈ] -->|تصویر پیغامات| B[/camera/image_raw موضوع]
    B --> C[تصویر پروسسر]
    B --> D[آبجیکٹ ڈیٹیکٹر]
```

**مثال: موضوع کو شائع کرنا**

```python
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
    def publish_status(self):
        msg = String()
        msg.data = 'روبوٹ सकریا ہے!'
        self.publisher.publish(msg)
        self.get_logger().info(f'شائع کر رہا ہے: {msg.data}')
```

### 3. سروسز

**سروسز** مختصر آپریشن کے لیے درخواست-جواب بات چیت فراہم کرتے ہیں:

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
```

### 4. ایکشنز

**ایکشنز** طویل مدتی کاموں کے لیے فیڈ بیک کے ساتھ ہوتے ہیں (جیسے کہ ایک ہدف کی طرف جانا):

- **ہدف**: ٹارگٹ پوزیشن
- **فیڈ بیک**: پیشرفت کی اپ ڈیٹس
- **نتیجہ**: کامیابی/ناکامی کا درجہ

## ROS 2 آرکیٹیکچر

```
┌─────────────────────────────────────────────┐
│           ایپلی کیشن لیئر                 │
│  (آپ کے نوڈز: نیویگیشن، پریشانی، وغیرہ) │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│              ROS 2 کلائنٹ لیئر             │
│        (rclpy, rclcpp - APIs)               │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│           DDS مڈل ویئر لیئر              │
│  (Fast-DDS, Cyclone DDS - بات چیت)    │
└─────────────────────────────────────────────┘
```

## ROS 2 سیٹ اپ کرنا

### انسٹالیشن (Ubuntu 22.04)

```bash
# ROS 2 ریپوزٹری شامل کریں
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS 2 Humble انسٹال کریں
sudo apt update
sudo apt install ros-humble-desktop

# سیٹ اپ فائل کو سورس کریں
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### اپنا پہلا ورک اسپیس بنائیں

```bash
# ورک اسپیس بنائیں
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# ایک پیکیج بنائیں
ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy

# ورک اسپیس کو بلڈ کریں
cd ~/ros2_ws
colcon build

# ورک اسپیس کو سورس کریں
source install/setup.bash
```

## عملی مثال: ٹیمپریچر مانیٹر

آئیے ایک مکمل سسٹم بنائیں جس میں پبلشر اور سبسکرائبر ہو:

### پبلشر (ٹیمپریچر سینسر)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TempSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(2.0, self.publish_temperature)
        
    def publish_temperature(self):
        temp = Float32()
        temp.data = 20.0 + random.uniform(-5, 5)  # سمولیشن ریڈنگ
        self.publisher.publish(temp)
        self.get_logger().info(f'ٹیمپریچر: {temp.data:.2f}°C')

def main():
    rclpy.init()
    sensor = TempSensor()
    rclpy.spin(sensor)
    rclpy.shutdown()
```

### سبسکرائب (مونٹر)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TempMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32, 'temperature', self.temp_callback, 10
        )
        
    def temp_callback(self, msg):
        if msg.data > 25.0:
            self.get_logger().warn(f'ہائی ٹیمپ: {msg.data:.2f}°C!')
        else:
            self.get_logger().info(f'نارمل: {msg.data:.2f}°C')

def main():
    rclpy.init()
    monitor = TempMonitor()
    rclpy.spin(monitor)
    rclpy.shutdown()
```

### سسٹم کو چلانا

```bash
# ٹرمنل 1: سینسر چلائیں
ros2 run my_robot_pkg temp_sensor

# ٹرمنل 2: مونٹر چلائیں
ros2 run my_robot_pkg temp_monitor

# ٹرمنل 3: موضوعات کو انسپیکٹ کریں
ros2 topic list
ros2 topic echo /temperature
```

## کلیدی نکات

✅ **ROS 2 روبوٹکس کے لیے صنعت کا معیار ہے**  
✅ **نوڈز** آزاد پروسس ہیں جو موضوعات، سروسز، اور ایکشنز کے ذریعے بات چیت کرتے ہیں  
✅ **DDS مڈل ویئر** قابل اعتماد، حقیقی وقت کی بات چیت فراہم کرتا ہے  
✅ **پیٹھون اور سی++** ROS 2 ڈویلپمنٹ کے لیے بنیادی زبانوں ہیں  

## اگلا قدم

اگلی سیکشن میں، ہم **ROS 2 بنیادیات** میں گہرائی سے جائیں گے، بشمول:
- پیغام کی قسمیں اور کسٹم پیغامات
- لانچ فائلز کے لیے پیچیدہ سسٹم
- پیرامیٹرز اور کنفیگریشن
- TF2 کوآرڈینیٹ ٹرانسفارم کے لیے

**جاری رکھیں:** [ROS 2 بنیادیات →](./ros2-fundamentals)

---

## مزید پڑھیں

- [ROS 2 کے لیے سرکاری دستاویزات](https://docs.ros.org/en/humble/)
- [ROS 2 ڈیزائن کے اصول](https://design.ros2.org/)
- [DDS سپیسفیکیشن](https://www.omg.org/spec/DDS/)