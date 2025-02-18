import os
import random
from ultralytics import YOLO
import matplotlib.pyplot as plt
import cv2

model = YOLO('runs/detect/train6/weights/best.pt')

test_images_path = './test/images'

all_images = [f for f in os.listdir(test_images_path) if f.endswith(('.jpg', '.jpeg', '.png'))]

sample_images = random.sample(all_images, min(10, len(all_images))) 
sample_image_paths = [os.path.join(test_images_path, img) for img in sample_images]

results = model.predict(source=sample_image_paths, save=True, save_txt=True)

plt.figure(figsize=(15, 6))

for i, result in enumerate(results):
    img = cv2.imread(result.path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if result.boxes is not None:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist() 
            conf = box.conf[0].item() 
            cls = box.cls[0].item()

            label = f"{model.names[int(cls)]}: {conf:.2f}"
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.putText(img, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    plt.subplot(2, 5, i + 1)
    plt.imshow(img)
    plt.axis('off')
    plt.title(f"Image {i + 1}")

plt.tight_layout()
plt.show()
