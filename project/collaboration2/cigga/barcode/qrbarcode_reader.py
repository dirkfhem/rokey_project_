import cv2
import pyzbar.pyzbar as pyzbar
from pygame import mixer
from collections import deque
import numpy as np
import subprocess
import time

# v4l2-ctl로 카메라 설정 최적화
def set_camera_controls(device="/dev/video0"):
    commands = [
        f"v4l2-ctl -d {device} --set-ctrl=brightness=128",
        f"v4l2-ctl -d {device} --set-ctrl=contrast=64",
        f"v4l2-ctl -d {device} --set-ctrl=saturation=64",
    ]
    for cmd in commands:
        try:
            subprocess.run(cmd, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"카메라 설정 오류: {cmd}, {e}")

# 초기 설정
used_codes = []
code_buffer = deque(maxlen=5)  # 다중 프레임 집계
try:
    with open("qrbarcode_data.txt", "r", encoding="utf8") as f:
        used_codes = [line.rstrip('\n') for line in f]
except FileNotFoundError:
    pass

# 카메라 초기화
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3, 640)  # 해상도 640x480
cap.set(4, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
set_camera_controls()  # 카메라 설정 적용

mixer.init()
sound = mixer.Sound('/home/minsuje/ros2_ws/barcode/qrbarcode_beep.mp3')

# FPS 계산용
start_time = time.time()
frame_count = 0

while True:
    success, frame = cap.read()
    if not success:
        print("프레임 캡처 실패")
        continue

    # FPS 계산
    frame_count += 1
    if time.time() - start_time > 1:
        print(f"FPS: {frame_count}")
        frame_count = 0
        start_time = time.time()

    # 이미지 전처리 (단순화)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 바코드 디코딩 (모든 유형 스캔)
    codes = pyzbar.decode(binary)  # symbols 파라미터 제거로 모든 유형 체크
    if not codes:
        # 인식 실패 시 프레임 저장
        cv2.imwrite(f'failed_frame_{time.time()}.png', frame)
        print("바코드 인식 실패, 프레임 저장됨")

    for code in codes:
        my_code = code.data.decode('utf-8')
        code_buffer.append(my_code)
        if code_buffer.count(my_code) >= 3:  # 3번 이상 동일 코드 감지
            if my_code not in used_codes:
                print(f"인식 성공: {my_code} (유형: {code.type})")
                used_codes.append(my_code)
                sound.play()
            else:
                print(f"이미 인식된 코드입니다: {my_code}")
                sound.play()
        # 바코드 영역 표시
        points = code.polygon
        if len(points) >= 4:  # 최소 4점 필요
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))

    cv2.imshow('QRcode Barcode Scan', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()