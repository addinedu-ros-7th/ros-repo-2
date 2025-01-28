import os
from ultralytics import YOLO

# 모델 로드 (사전 학습된 모델 사용)
model = YOLO('yolov8n.pt')

# 학습
model.train(data='./data.yaml', epochs=100, imgsz=640, batch=32)
