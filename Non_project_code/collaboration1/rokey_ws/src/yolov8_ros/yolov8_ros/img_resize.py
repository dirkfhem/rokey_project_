from PIL import Image
import os

# 입력 및 출력 디렉토리 설정
input_dir = "/home/minsuje/rokey_ws/img"  # 원본 이미지 폴더
output_dir = "/home/minsuje/rokey_ws/img_640"  # 리사이징된 이미지 저장 폴더
target_size = (640, 640)  # 목표 크기

# 출력 디렉토리 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 이미지 리사이징
for filename in os.listdir(input_dir):
    if filename.endswith((".jpg", ".jpeg", ".png")):  # 이미지 확장자 필터링
        img_path = os.path.join(input_dir, filename)
        img = Image.open(img_path)
        
        # 리사이징 (비율 유지, 빈 공간 패딩)
        img.thumbnail(target_size, Image.LANCZOS)  # 비율 유지하며 축소
        new_img = Image.new("RGB", target_size, (0, 0, 0))  # 640x640 검은색 배경
        offset = ((target_size[0] - img.size[0]) // 2, 
                  (target_size[1] - img.size[1]) // 2)
        new_img.paste(img, offset)  # 이미지 중앙에 붙여넣기
        
        # 출력 경로에 저장
        output_path = os.path.join(output_dir, filename)
        new_img.save(output_path, "JPEG")
        print(f"Resized: {filename}")