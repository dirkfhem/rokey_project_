from ultralytics import YOLO
import os

# 1. 데이터셋 및 YAML 경로 설정 (로컬 환경)
DATASET_PATH = '/home/minsuje/smoke_fire_detection_yolo'
YAML_PATH = os.path.join(DATASET_PATH, 'data/data.yaml')  # 제공된 YAML 파일 경로
OUTPUT_PATH = os.path.join(DATASET_PATH, 'outputs')  # 예측 및 제출 파일 저장 경로

# 출력 디렉토리 생성
os.makedirs(OUTPUT_PATH, exist_ok=True)

# 2. 메인 실행
if __name__ == '__main__':
    # YOLOv8 모델 로드
    model = YOLO('yolov8s.pt')  # 사전 학습된 YOLOv8n 모델 사용
    
    # 모델 학습 (YOLOv8 내장 증강 활성화)
    model.train(
        data=YAML_PATH,
        epochs=50,
        batch=40,
        imgsz=160,
        patience=10,
        device=0,  # GPU 사용 (로컬 GPU가 없으면 'cpu'로 변경)
        optimizer='AdamW',
        lr0=0.001,
        augment=True,  # YOLOv8 내장 증강 활성화
        mosaic=1.0,    # 모자이크 증강
        hsv_h=0.015,   # 색조 증강
        hsv_s=0.7,     # 채도 증강
        hsv_v=0.4,     # 명도 증강
        flipud=0.5,    # 상하 플립
        fliplr=0.5,    # 좌우 플립
        scale=0.5,     # 스케일링
        translate=0.1   # 이동
    )
    
    # 검증 데이터로 평가
    metrics = model.val()
    print(f"mAP@50: {metrics.box.map50}")
    print(f"mAP@50:95: {metrics.box.map}")