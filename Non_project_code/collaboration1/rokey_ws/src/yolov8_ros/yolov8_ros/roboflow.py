import albumentations as A
import cv2
import os

# 증강 파이프라인
transform = A.Compose([
    A.Resize(640, 640),
    A.RandomRotate90(),
    A.HorizontalFlip(p=0.5),
    A.RGBShift(p=0.5),
])

# 원본 이미지 로드
image_dir = "/home/minsuje/rokey_ws/img_640"
output_dir = "/home/minsuje/rokey_ws/img"
os.makedirs(output_dir, exist_ok=True)

for i, filename in enumerate(os.listdir(image_dir)):
    if filename.endswith((".jpg", ".png")):
        img = cv2.imread(os.path.join(image_dir, filename))
        # 증강 적용
        augmented = transform(image=img)["image"]
        # 저장
        cv2.imwrite(os.path.join(output_dir, f"test_img_{str(i+1).zfill(2)}.jpg"), augmented)