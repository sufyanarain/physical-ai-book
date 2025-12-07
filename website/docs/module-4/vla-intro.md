---
sidebar_position: 1
---

# Vision-Language-Action (VLA)

The convergence of Large Language Models (LLMs) and robotics enables natural language robot control.

## What is VLA?

**Vision-Language-Action** models combine:

- **Vision**: Perceive the environment (cameras, depth sensors)
- **Language**: Understand natural language commands
- **Action**: Execute physical tasks

### Example Flow

```
User: "Pick up the red cup and place it on the table"
    ↓
[LLM] → Parse intent → "pick_and_place(object='red_cup', location='table')"
    ↓
[Vision] → Detect red cup → Position: (x=0.5, y=0.2, z=0.1)
    ↓
[Action] → Move arm → Grasp → Transport → Release
```

## Voice-to-Action Pipeline

### 1. Speech Recognition (Whisper)

```python
import whisper

model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("command.wav")
text = result["text"]
print(f"Command: {text}")
```

### 2. Intent Understanding (GPT-4)

```python
from openai import OpenAI

client = OpenAI()

response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "You are a robot task planner. Convert natural language to robot actions."},
        {"role": "user", "content": text}
    ]
)

action_plan = response.choices[0].message.content
```

### 3. Action Execution (ROS 2)

```python
class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        self.action_client = ActionClient(self, MoveArm, 'move_arm')
    
    def execute_plan(self, plan):
        # Parse plan and execute actions
        for action in parse_actions(plan):
            self.send_goal(action)
```

## Building a Conversational Robot

Complete system integration:

```python
import rclpy
from rclpy.node import Node
from openai import OpenAI
import whisper
from std_msgs.msg import String

class ConversationalRobot(Node):
    def __init__(self):
        super().__init__('conversational_robot')
        
        # Initialize models
        self.whisper_model = whisper.load_model("base")
        self.openai_client = OpenAI()
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'robot/actions', 10)
        
        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData, 'microphone', self.audio_callback, 10
        )
    
    def audio_callback(self, msg):
        # Step 1: Speech to text
        text = self.whisper_model.transcribe(msg.data)["text"]
        self.get_logger().info(f"Heard: {text}")
        
        # Step 2: LLM planning
        action = self.plan_action(text)
        
        # Step 3: Execute
        self.action_pub.publish(String(data=action))
    
    def plan_action(self, command):
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "Robot task planner"},
                {"role": "user", "content": command}
            ]
        )
        return response.choices[0].message.content
```

## Multimodal Perception

Combine vision and language:

```python
from transformers import pipeline

# Vision-language model
vl_model = pipeline("image-to-text", model="Salesforce/blip-image-captioning-large")

# Process image
image = capture_camera()
description = vl_model(image)[0]['generated_text']

# Context for LLM
context = f"Current scene: {description}. User command: {user_command}"
```

## Cognitive Planning

LLM breaks complex tasks into steps:

```python
def cognitive_plan(task):
    prompt = f"""
    Break down this task into robot actions:
    Task: {task}
    
    Available actions:
    - move_to(x, y, z)
    - grasp_object(object_id)
    - release_object()
    - rotate(angle)
    
    Return a JSON list of actions.
    """
    
    response = llm.generate(prompt)
    actions = json.loads(response)
    return actions
```

### Example

```
Input: "Clean the table"

Output:
[
  {"action": "move_to", "params": {"x": 0, "y": 0, "z": 0.5}},
  {"action": "detect_objects", "params": {"category": "trash"}},
  {"action": "grasp_object", "params": {"object_id": "obj_001"}},
  {"action": "move_to", "params": {"x": 1, "y": 1, "z": 0.5}},
  {"action": "release_object", "params": {}}
]
```

## Safety and Grounding

Ensure safe execution:

```python
def validate_action(action, safety_rules):
    # Check workspace bounds
    if not in_workspace(action.position):
        return False
    
    # Check collision
    if will_collide(action):
        return False
    
    # Check force limits
    if action.force > MAX_FORCE:
        return False
    
    return True
```

## Real-World Applications

### 1. Home Assistant Robot

```python
commands = [
    "Bring me water from the kitchen",
    "Turn on the lights in the living room",
    "Find my phone"
]
```

### 2. Warehouse Robot

```python
commands = [
    "Pick items from shelf A3 and deliver to station 5",
    "Restock shelf B2 with boxes from staging area",
    "Scan inventory in aisle 7"
]
```

### 3. Healthcare Robot

```python
commands = [
    "Deliver medication to room 302",
    "Bring wheelchair to the lobby",
    "Escort patient to radiology"
]
```

## Key Takeaways

✅ VLA combines vision, language, and action  
✅ LLMs enable natural language robot control  
✅ Safety validation is critical  
✅ Multimodal perception enhances understanding  

**Next:** [Capstone Project →](./capstone-project)
