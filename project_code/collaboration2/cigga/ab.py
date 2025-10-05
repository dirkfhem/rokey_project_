import cv2
import numpy as np

# 이미지 로드
image = cv2.imread('image.jpg')

# BGR을 HSV 색상 공간으로 변환
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 녹색 범위 정의
lower_green = np.array([30, 40, 40])  # 녹색 하한
upper_green = np.array([50, 255, 255])  # 녹색 상한

# 녹색 마스크 생성
mask = cv2.inRange(hsv, lower_green, upper_green)

# 마스크 반전 (녹색 제외한 부분)
mask_inv = cv2.bitwise_not(mask)

# 원본 이미지에서 녹색 배경 제거
result = cv2.bitwise_and(image, image, mask=mask_inv)

# 알파 채널 추가 (투명 배경)
b, g, r = cv2.split(result)
alpha = mask_inv
rgba = cv2.merge([b, g, r, alpha])

# PNG로 저장 (투명 배경 유지)
cv2.imwrite('output.png', rgba)