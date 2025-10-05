import os

# 디렉토리 설정
input_dir = "/home/minsuje/rokey_ws/img"  # 이미지 폴더 경로
prefix = "test__img"  # 새 파일명 접두사
start_number = 142  # 시작 번호
padding = 2  # 번호 자릿수 (예: 01, 02)

# 지원하는 이미지 확장자
valid_extensions = (".jpg", ".jpeg", ".png")

# 디렉토리 내 파일 목록 가져오기
files = [f for f in os.listdir(input_dir) if f.lower().endswith(valid_extensions)]

# 파일 이름 변경
for i, filename in enumerate(sorted(files), start=start_number):
    # 원본 파일 경로
    old_path = os.path.join(input_dir, filename)
    
    # 확장자 추출
    ext = os.path.splitext(filename)[1]
    
    # 새 파일명 생성 (예: test_img_01.jpg)
    new_filename = f"{prefix}_{str(i).zfill(padding)}{ext}"
    new_path = os.path.join(input_dir, new_filename)
    
    # 파일 이름 변경
    os.rename(old_path, new_path)
    print(f"Renamed: {filename} -> {new_filename}")

print("Renaming complete!")