import os
import random
import shutil
from pathlib import Path

# 디렉토리 설정
base_dir = "/home/minsuje/ros2_ws/Tutorial/OD_Tutorial/YOLO"
img_capture_dir = os.path.join(base_dir, "img_capture")
train_dir = os.path.join(base_dir, "train")
val_dir = os.path.join(base_dir, "val")
train_img_dir = os.path.join(train_dir, "images")
train_label_dir = os.path.join(train_dir, "labels")
val_img_dir = os.path.join(val_dir, "images")
val_label_dir = os.path.join(val_dir, "labels")

# 디렉토리 생성
for dir_path in [train_img_dir, train_label_dir, val_img_dir, val_label_dir]:
    Path(dir_path).mkdir(parents=True, exist_ok=True)

# 기존 브랜드와 클래스 ID 매핑
brand_to_class = {
    
}

# 새로운 0-based 클래스 매핑 생성
class_to_new_id = {v: i for i, v in enumerate(sorted(brand_to_class.values()))}
new_id_to_name = {i: name for name, old_id in brand_to_class.items() for i, v in enumerate(sorted(brand_to_class.values())) if v == old_id}

# 이미지 및 라벨 파일 목록
image_files = [f for f in os.listdir(img_capture_dir) if f.endswith(".jpg")]
label_files = [f for f in os.listdir(img_capture_dir) if f.endswith(".txt")]

# 이미지-라벨 쌍 확인
image_label_pairs = {}
for img_file in image_files:
    img_base = os.path.splitext(img_file)[0]
    label_file = img_base + ".txt"
    if label_file in label_files:
        image_label_pairs[img_file] = label_file
    else:
        print(f"No label found for {img_file}, skipping...")

# 새로운 라벨 파일 생성
temp_label_dir = os.path.join(base_dir, "temp_labels")
Path(temp_label_dir).mkdir(exist_ok=True)

for img_file, label_file in image_label_pairs.items():
    with open(os.path.join(img_capture_dir, label_file), "r") as f:
        lines = f.readlines()
    
    new_lines = []
    for line in lines:
        parts = line.strip().split()
        old_class_id = int(parts[0])
        if old_class_id in class_to_new_id:
            new_class_id = class_to_new_id[old_class_id]
            new_line = " ".join([str(new_class_id)] + parts[1:])
            new_lines.append(new_line)
    
    new_label_file = os.path.join(temp_label_dir, label_file)
    with open(new_label_file, "w") as f:
        f.write("\n".join(new_lines) + "\n")

# train/val 데이터셋 분할
image_list = list(image_label_pairs.keys())
random.shuffle(image_list)
split_idx = int(0.8 * len(image_list))  # 80% train, 20% val
train_files = image_list[:split_idx]
val_files = image_list[split_idx:]

# train/val 디렉토리로 파일 이동
for img_file in train_files:
    label_file = image_label_pairs[img_file]
    new_label_file = os.path.join(temp_label_dir, label_file)
    shutil.move(os.path.join(img_capture_dir, img_file), os.path.join(train_img_dir, img_file))
    shutil.move(new_label_file, os.path.join(train_label_dir, label_file))
    print(f"Moved to train: {img_file}, {label_file}")

for img_file in val_files:
    label_file = image_label_pairs[img_file]
    new_label_file = os.path.join(temp_label_dir, label_file)
    shutil.move(os.path.join(img_capture_dir, img_file), os.path.join(val_img_dir, img_file))
    shutil.move(new_label_file, os.path.join(val_label_dir, label_file))
    print(f"Moved to val: {img_file}, {label_file}")

# 임시 디렉토리 삭제
if os.path.exists(temp_label_dir):
    shutil.rmtree(temp_label_dir)

# data.yaml 파일 생성
yaml_content = f"""train: {train_img_dir}
val: {val_img_dir}
nc: {len(brand_to_class)}
names: {list(brand_to_class.keys())}
"""

yaml_path = os.path.join(base_dir, "data.yaml")
with open(yaml_path, "w") as f:
    f.write(yaml_content)

print("YOLO dataset preparation completed.")