import json
import csv
import time
import math
import os
import shutil
import sys
from ultralytics import YOLO
from pathlib import Path
import cv2

class YOLOWebcamProcessor:
    def __init__(self, model, output_dir):
        self.model = model
        self.output_dir = output_dir
        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names  # Use model-provided class names
        self.should_shutdown = False

    def run(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Failed to open webcam.")
            return

        while not self.should_shutdown:
            ret, img_crop = cap.read()
            if not ret:
                continue
            
            h, w = img_crop.shape[:2]
            crop_w, crop_h = 320, 320
            cx, cy = w // 2, h // 2
            x1 = cx - crop_w // 2
            x2 = cx + crop_w // 2
            y1 = cy - crop_h // 2
            y2 = cy + crop_h // 2
            img = img_crop[y1:y2, x1:x2]


            results = self.model(img, stream=True)
            print("Inference done on device:", self.model.device)

            object_count = 0
            fontScale = 1

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    confidence = math.ceil((box.conf[0] * 100)) / 100
                    cls = int(box.cls[0])
                    label = self.classNames.get(cls, f"class_{cls}")
                    self.confidences.append(confidence)

                    org = [x1, y1]
                    cv2.putText(img, f"{label}: {confidence}", org, cv2.FONT_HERSHEY_SIMPLEX, fontScale, (255, 0, 0), 2)

                    self.csv_output.append([x1, y1, x2, y2, confidence, label])
                    object_count += 1

            self.max_object_count = max(self.max_object_count, object_count)                #최대 객체수
            cv2.putText(img, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1) # 이미지에 최종 개수 넣기

            if object_count > 0:                                            # 객체가 하나이상 탐지되면
                filename = f'output_{int(time.time())}.jpg' 
                cv2.imwrite(os.path.join(self.output_dir, filename), img)   # 위에서 만든 사진 저장.

            display_img = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2)) # 보일 이미지
            cv2.imshow("Detection", display_img)                            # 보이기

            key = cv2.waitKey(10)
            if key == ord('q'):
                print("Shutting down...")
                self.should_shutdown = True

        cap.release()
        cv2.destroyAllWindowconfs()


def main():
    model_path = '/home/park/Downloads/min_fire.pt'
    # model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()

    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    processor = YOLOWebcamProcessor(model, output_dir)
    processor.run()
    print("Shutdown complete.")
    sys.exit(0)


if __name__ == '__main__':
    main()