from ultralytics import YOLO

# 모델 로드
model = YOLO("yolo11n.pt")

# 훈련 실행
YAML_PATH = "/home/minsuje/ros2_ws/Tutorial/OD_Tutorial/YOLO/ciga_all_dataset/data.yaml"
model.train(
    data=YAML_PATH,
    epochs=150,
    batch=30,
    imgsz=640,
    patience=30,
    optimizer='AdamW',
    lr0=0.0001,
    augment=True,
)