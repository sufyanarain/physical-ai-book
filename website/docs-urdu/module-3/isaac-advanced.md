---
sidebar_position: 2
---

# آئزک کی پیشرفته موضوعات

آئزک کی پیشرفته صلاحیتوں کا جائزہ لینے کے لئے گہری نظر ڈالیں جو پیداواری روبوٹکس کے لئے ہیں۔

## ملٹی روبوٹ کوآرڈینیشن

آئزک سم میں روبوٹ کی فلیٹس کو محاکمت کریں:

```python
from omni.isaac.core import World

world = World()

# کئی روبوٹس کو جنم دیں
for i in range(10):
    robot = world.scene.add(
        Robot(prim_path=f"/World/Robot_{i}", position=[i*2, 0, 0])
    )
```

## کلاؤڈ ڈیپلویمنٹ

آئزک ایپلی کیشنز کو کلاؤڈ پر ڈیپلوی کریں:

```yaml
# کبیرنیٹس ڈیپلویمنٹ
apiVersion: apps/v1
kind: Deployment
metadata:
  name: isaac-inference
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: isaac-ros
        image: nvcr.io/nvidia/isaac-ros:latest
        resources:
          limits:
            nvidia.com/gpu: 1
```

## کارکردگی کو بہتر بنانا

### جی پی یو ایکسلریشن

```python
# جی پی یو پائپ لائن کو فعال کریں
pipeline = rs.create_rtx_lidar_scan_pipeline()
pipeline.set_cuda_device(0)
```

### بیچ پروسیسنگ

```python
# کئی کیمروں کی فیڈز کو پروسس کریں
cameras = [create_camera(i) for i in range(4)]
images = batch_capture(cameras)
results = model.infer_batch(images)
```

## کلید لینے والے نکات

✅ آئزک ایج سے لے کر کلاؤڈ تک سکیل کرتا ہے  
✅ جی پی یو ایکسلریشن ریئل ٹائم کے لئے اہم ہے  
✅ ملٹی روبوٹ سسٹم کو کوآرڈینیشن کی ضرورت ہے  

**اگلا ماڈیول:** [ویژن-لینگویج-ایکشن →](../module-4/vla-intro)