#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Jun
#
# ============================================================================
# 🤖 자율주행 로봇용 교통 표지판 인식 시스템
# 
# 이 프로그램의 주요 기능:
# 1. 📷 실시간 카메라 영상에서 교통 표지판 자동 감지
# 2. 🔍 SIFT 알고리즘 기반 고정밀 이미지 매칭 기술 사용
# 3. 🚦 세 종류 표지판 동시 인식: 교차로, 좌회전, 우회전
# 4. 🎯 오감지 방지를 위한 다단계 품질 검증 시스템
# 5. 📡 ROS2 네트워크를 통한 실시간 감지 결과 전송
# 6. 🖼️ 디버깅용 매칭 과정 시각화 기능
# 7. ⚡ 성능 최적화를 위한 프레임 스킵 처리
# 
# 🔬 사용된 핵심 기술:
# - SIFT: 크기/회전에 강인한 특징점 검출 알고리즘
# - FLANN: 고속 특징점 매칭 라이브러리
# - Homography: 기하학적 변환을 통한 정확도 검증
# - MSE: 수치적 품질 평가를 통한 오감지 방지
# ============================================================================

from enum import Enum
import os

import cv2                          # OpenCV: 컴퓨터 비전 라이브러리
from cv_bridge import CvBridge      # ROS ↔ OpenCV 이미지 형식 변환기
import numpy as np                  # NumPy: 수치 계산 및 배열 처리
import rclpy                        # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # 압축 이미지 메시지 타입
from sensor_msgs.msg import Image           # 원본 이미지 메시지 타입
from std_msgs.msg import UInt8              # 부호없는 8비트 정수 메시지
from std_msgs.msg import Bool               # 참/거짓 불린 메시지

from ament_index_python.packages import get_package_share_directory

class DetectSign(Node):
    """
    🎯 교통 표지판 감지 및 분류 클래스
    
    이 클래스는 자율주행 차량이 교통 표지판을 인식하고 분류하는 핵심 기능을 담당합니다.
    
    주요 특징:
    - 실시간 이미지 처리로 즉각적인 반응
    - 높은 정확도의 SIFT 기반 특징점 매칭
    - 동시에 여러 표지판 감지 가능
    - 강력한 오감지 방지 시스템
    """

    def __init__(self):
        super().__init__('detect_sign')  # ROS2 노드 이름 설정

        # =============================================================
        # 📊 이미지 처리 형식 설정
        # =============================================================
        # 네트워크 대역폭과 처리 성능을 고려한 최적 설정
        self.sub_image_type = 'raw'         # 입력: 'compressed'(압축) 또는 'raw'(원본)
        self.pub_image_type = 'compressed'  # 출력: 'compressed'(압축) 또는 'raw'(원본)

        # =============================================================
        # 📥 구독자(Subscriber) 설정 - 카메라 데이터 수신
        # =============================================================
        
        if self.sub_image_type == 'compressed':
            # 압축된 이미지를 받는 경우 (네트워크 대역폭 절약)
            self.sub_image_original = self.create_subscription(
                CompressedImage,                    # 메시지 타입
                '/detect/image_input/compressed',   # 토픽 이름 (카메라 노드에서 발행)
                self.cbFindTrafficSign,            # 콜백 함수
                10                                 # 큐 크기 (최대 10개 메시지 버퍼링)
            )
        elif self.sub_image_type == 'raw':
            # 원본 이미지를 받는 경우 (현재 설정 - 고화질 처리)
            self.sub_image_original = self.create_subscription(
                Image,                      # 메시지 타입
                '/detect/image_input',      # 토픽 이름
                self.cbFindTrafficSign,     # 콜백 함수 (메인 처리 로직)
                10                         # 큐 크기
            )

        # =============================================================
        # 📤 발행자(Publisher) 설정 - 감지 결과 송신
        # =============================================================
        
        # 1️⃣ 범용 표지판 감지 결과 (숫자 코드로 전송)
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        
        # 2️⃣ 처리된 이미지 출력 (디버깅 및 모니터링용)
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        # =============================================================
        # 🎯 핵심 기능: 개별 표지판 감지 결과 발행자들
        # =============================================================
        # 각 표지판별로 독립적인 Bool 신호를 보내어
        # 제어 시스템이 정확히 어떤 표지판이 감지되었는지 알 수 있음
        
        self.pub_intersection_detected = self.create_publisher(Bool, '/detect/intersection_sign', 10)  # 교차로 표지판
        self.pub_left_detected = self.create_publisher(Bool, '/detect/left_sign', 10)              # 좌회전 표지판  
        self.pub_right_detected = self.create_publisher(Bool, '/detect/right_sign', 10)            # 우회전 표지판

        # =============================================================
        # 🔧 시스템 구성 요소 초기화
        # =============================================================
        self.cvBridge = CvBridge()  # ROS 이미지 ↔ OpenCV 이미지 변환 도구
        
        # 교통 표지판 종류 정의 (열거형으로 체계적 관리)
        self.TrafficSign = Enum('TrafficSign', 'intersection left right')
        
        # 성능 최적화용 프레임 카운터
        self.counter = 1

        # 컴퓨터 비전 알고리즘 초기화 및 템플릿 로드
        self.fnPreproc()

        self.get_logger().info('DetectSign Node Initialized')

    def fnPreproc(self):
        """
        🔬 컴퓨터 비전 시스템 전처리 및 초기화
        
        이 함수는 프로그램 시작 시 한 번만 실행되며, 다음 작업을 수행합니다:
        1. SIFT 특징점 검출기 생성
        2. 템플릿 표지판 이미지들 로드
        3. 각 템플릿의 특징점 미리 계산
        4. 고속 매칭을 위한 FLANN 매처 설정
        """
        
        # =============================================================
        # 🔍 SIFT (Scale-Invariant Feature Transform) 검출기 생성
        # =============================================================
        # SIFT는 이미지의 크기, 회전, 조명 변화에 강인한 특징점을 찾는 알고리즘
        # 표지판이 멀리 있거나 기울어져 있어도 정확히 인식할 수 있음
        self.sift = cv2.SIFT_create()
        
        # =============================================================
        # 📁 템플릿 이미지 로드 (미리 저장된 표준 표지판 이미지들)
        # =============================================================
        
        ####살릴거########################################
        # 주석 처리된 부분: 프로젝트 상대 경로를 사용한 이미지 로드 방식
        # 개발 환경에서는 이 방식이 더 편리할 수 있음
        # dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        # dir_path = os.path.join(dir_path, 'image')

        # self.img_intersection = cv2.imread(dir_path + '/intersection.png', 0)
        # self.img_left = cv2.imread(dir_path + '/left.png', 0)
        # self.img_right = cv2.imread(dir_path + '/right.png', 0)

        # self.kp_intersection, self.des_intersection = self.sift.detectAndCompute(
        #     self.img_intersection, None
        # )
        # self.kp_left, self.des_left = self.sift.detectAndCompute(self.img_left, None)
        # self.kp_right, self.des_right = self.sift.detectAndCompute(self.img_right, None)
        ####살릴거########################################
        
        ###살릴거#######################################################
        # 현재 사용 중인 방식: ROS2 패키지 시스템을 통한 이미지 로드
        # 배포 환경에서 안정적으로 파일을 찾을 수 있음
        package_share_dir = get_package_share_directory('turtlebot3_autorace_detect')

        # 그레이스케일로 템플릿 이미지들 로드 (컬러 정보 불필요, 처리 속도 향상)
        self.img_intersection = cv2.imread(os.path.join(package_share_dir, 'image', 'intersection.png'), cv2.IMREAD_GRAYSCALE)
        self.img_left = cv2.imread(os.path.join(package_share_dir, 'image', 'left.png'), cv2.IMREAD_GRAYSCALE)
        self.img_right = cv2.imread(os.path.join(package_share_dir, 'image', 'right.png'), cv2.IMREAD_GRAYSCALE)

        # 이미지 로드 실패 시 프로그램 안전 종료
        if self.img_intersection is None:
            self.get_logger().error("Failed to load intersection.png")
            rclpy.shutdown()
            return
        if self.img_left is None:
            self.get_logger().error("Failed to load left.png")
            rclpy.shutdown()
            return
        if self.img_right is None:
            self.get_logger().error("Failed to load right.png")
            rclpy.shutdown()
            return

        # =============================================================
        # 🎯 템플릿 이미지들의 특징점 미리 계산 (성능 최적화)
        # =============================================================
        # 실시간 처리에서는 카메라 이미지의 특징점만 계산하면 됨
        # 템플릿의 특징점은 미리 계산해두어 처리 속도 향상
        
        # kp: KeyPoint (특징점의 위치, 크기, 각도 정보)
        # des: Descriptor (특징점 주변의 특징을 숫자로 표현한 벡터)
        self.kp_intersection, self.des_intersection = self.sift.detectAndCompute(self.img_intersection, None)
        self.kp_left, self.des_left = self.sift.detectAndCompute(self.img_left, None)
        self.kp_right, self.des_right = self.sift.detectAndCompute(self.img_right, None)
        ###############################여기까지가지 지우기#############################
        
        # =============================================================
        # ⚡ FLANN 기반 고속 매칭 시스템 설정
        # =============================================================
        # FLANN: Fast Library for Approximate Nearest Neighbors
        # 수천 개의 특징점들 사이에서 유사한 것들을 빠르게 찾는 라이브러리
        
        FLANN_INDEX_KDTREE = 0  # KD-Tree 알고리즘 사용 (고차원 데이터에 효과적)
        index_params = {
            'algorithm': FLANN_INDEX_KDTREE,
            'trees': 5      # 트리 개수 (많을수록 정확하지만 메모리 사용량 증가)
        }

        search_params = {
            'checks': 50    # 검색 깊이 (클수록 정확하지만 처리 시간 증가)
        }

        # FLANN 매처 객체 생성
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        """
        📊 MSE (평균 제곱 오차) 계산 함수
        
        두 점 집합 간의 차이를 수치로 계산하여 매칭 품질을 평가합니다.
        값이 작을수록 더 정확한 매칭을 의미합니다.
        
        Args:
            arr1 (numpy.ndarray): 입력 이미지의 특징점들
            arr2 (numpy.ndarray): 템플릿 이미지의 특징점들
            
        Returns:
            float: 평균 제곱 오차 값 (낮을수록 더 정확한 매칭)
        """
        squared_diff = (arr1 - arr2) ** 2           # 각 점들 간의 거리 차이를 제곱
        total_sum = np.sum(squared_diff)            # 모든 차이값의 합
        num_all = arr1.shape[0] * arr1.shape[1]     # 전체 데이터 포인트 개수
        err = total_sum / num_all                   # 평균 계산
        return err

    def cbFindTrafficSign(self, image_msg):
        """
        🎯 교통 표지판 감지 메인 처리 함수
        
        이 함수는 카메라에서 새로운 이미지가 들어올 때마다 자동으로 호출됩니다.
        다음과 같은 단계로 표지판을 감지합니다:
        
        1. 프레임 스킵을 통한 성능 최적화
        2. 이미지 형식 변환 및 전처리  
        3. SIFT 특징점 추출
        4. 템플릿과의 특징점 매칭
        5. 매칭 품질 검증
        6. 감지 결과 발행 및 시각화
        
        Args:
            image_msg: ROS 이미지 메시지 (카메라에서 수신)
        """
        
        # =============================================================
        # ⚡ 성능 최적화: 프레임 스킵 처리
        # =============================================================
        # 매 3번째 프레임만 처리하여 CPU 부하를 1/3로 감소
        # 30fps → 10fps로 줄여도 표지판 인식에는 충분함
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        # =============================================================
        # 🖼️ ROS 이미지를 OpenCV 형식으로 변환
        # =============================================================
        if self.sub_image_type == 'compressed':
            # 압축된 이미지를 디코딩
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            # 원본 이미지를 BGR 형식으로 변환
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # =============================================================
        # 🎚️ 감지 임계값 설정 (정확도와 민감도의 균형)
        # =============================================================
        MIN_MATCH_COUNT = 5      # 최소 매칭 특징점 개수 (적으면 오감지, 많으면 감지 실패)
        MIN_MSE_DECISION = 70000 # MSE 임계값 (낮으면 엄격, 높으면 관대)

        # =============================================================
        # 🔍 입력 이미지에서 SIFT 특징점 추출
        # =============================================================
        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)

        # =============================================================
        # 🔗 각 템플릿과의 특징점 매칭 수행
        # =============================================================
        # k=2: 각 특징점에 대해 가장 유사한 2개의 매칭 후보를 찾음
        matches_intersection = self.flann.knnMatch(des1, self.des_intersection, k=2)
        matches_left = self.flann.knnMatch(des1, self.des_left, k=2)
        matches_right = self.flann.knnMatch(des1, self.des_right, k=2)

        image_out_num = 1  # 출력 이미지 타입 결정용 (1=원본, 2=교차로, 3=좌회전, 4=우회전)

        # =============================================================
        # 🚦 교차로 표지판 검출 및 검증
        # =============================================================
        good_intersection = []
        
        # Lowe's ratio test: 좋은 매칭과 나쁜 매칭을 구분하는 표준 방법
        for m, n in matches_intersection:
            if m.distance < 0.7*n.distance:  # 첫 번째 매칭이 두 번째보다 확실히 좋으면 채택
                good_intersection.append(m)
                
        # 충분한 개수의 좋은 매칭이 있는지 확인
        if len(good_intersection) > MIN_MATCH_COUNT:
            # 매칭된 특징점들의 좌표 추출
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_intersection]).reshape(-1, 1, 2)
            dst_pts = np.float32([
                self.kp_intersection[m.trainIdx].pt for m in good_intersection
            ]).reshape(-1, 1, 2)

            # Homography 변환 계산: 기하학적 일관성 검증
            # RANSAC 알고리즘으로 이상치(outlier) 제거
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_intersection = mask.ravel().tolist()

            # MSE 계산으로 최종 품질 검증
            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                # ✅ 교차로 표지판 감지 성공!
                
                # 범용 표지판 코드 발행
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.intersection.value
                self.pub_traffic_sign.publish(msg_sign)
                
                # 🎯 개별 교차로 표지판 감지 신호 발행
                msg_intersection = Bool()
                msg_intersection.data = True
                self.pub_intersection_detected.publish(msg_intersection)
                
                self.get_logger().info('Detect intersection sign')
                image_out_num = 2

        # =============================================================
        # 👈 좌회전 표지판 검출 및 검증 (동일한 로직)
        # =============================================================
        good_left = []
        for m, n in matches_left:
            if m.distance < 0.7*n.distance:
                good_left.append(m)
                
        if len(good_left) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_left]).reshape(-1, 1, 2)
            dst_pts = np.float32([
                self.kp_left[m.trainIdx].pt for m in good_left
            ]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_left = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                # ✅ 좌회전 표지판 감지 성공!
                
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.left.value
                self.pub_traffic_sign.publish(msg_sign)
                
                # 🎯 개별 좌회전 표지판 감지 신호 발행
                msg_left = Bool()
                msg_left.data = True
                self.pub_left_detected.publish(msg_left)
                
                self.get_logger().info('Detect left sign')
                image_out_num = 3
        else:
            matches_left = None

        # =============================================================
        # 👉 우회전 표지판 검출 및 검증 (동일한 로직)
        # =============================================================
        good_right = []
        for m, n in matches_right:
            if m.distance < 0.7*n.distance:
                good_right.append(m)
                
        if len(good_right) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_right]).reshape(-1, 1, 2)
            dst_pts = np.float32([
                self.kp_right[m.trainIdx].pt for m in good_right
            ]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_right = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                # ✅ 우회전 표지판 감지 성공!
                
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.right.value
                self.pub_traffic_sign.publish(msg_sign)
                
                # 🎯 개별 우회전 표지판 감지 신호 발행
                msg_right = Bool()
                msg_right.data = True
                self.pub_right_detected.publish(msg_right)
                
                self.get_logger().info('Detect right sign')
                image_out_num = 4
        else:
            matches_right = None

        # =============================================================
        # 🖼️ 결과 이미지 생성 및 발행 (디버깅 및 모니터링용)
        # =============================================================
        
        if image_out_num == 1:
            # 표지판이 감지되지 않은 경우: 원본 이미지 그대로 출력
            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        cv_image_input, 'jpg'
                    )
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        cv_image_input, 'bgr8'
                    )
                )
                
        elif image_out_num == 2:
            # 교차로 표지판 감지: 매칭 과정을 시각화한 이미지 출력
            draw_params_intersection = {
                'matchColor': (255, 0, 0),          # 매칭 선 색상 (빨간색)
                'singlePointColor': None,           # 단일 점 색상
                'matchesMask': matches_intersection, # 유효한 매칭만 표시
                'flags': 2                          # 그리기 옵션
            }
            
            # 입력 이미지와 템플릿을 나란히 놓고 매칭 선을 그림
            final_intersection = cv2.drawMatches(
                cv_image_input,           # 왼쪽: 실시간 카메라 이미지
                kp1,                      # 실시간 이미지의 특징점들
                self.img_intersection,    # 오른쪽: 교차로 템플릿 이미지
                self.kp_intersection,     # 템플릿의 특징점들
                good_intersection,        # 매칭 관계
                None,
                **draw_params_intersection
            )

            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        final_intersection, 'jpg'
                    )
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        final_intersection, 'bgr8'
                    )
                )
                
        elif image_out_num == 3:
            # 좌회전 표지판 감지: 매칭 시각화
            draw_params_left = {
                'matchColor': (255, 0, 0),
                'singlePointColor': None,
                'matchesMask': matches_left,
                'flags': 2
            }

            final_left = cv2.drawMatches(
                cv_image_input,
                kp1,
                self.img_left,
                self.kp_left,
                good_left,
                None,
                **draw_params_left
            )

            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        final_left, 'jpg'
                    )
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        final_left, 'bgr8'
                    )
                )
                
        elif image_out_num == 4:
            # 우회전 표지판 감지: 매칭 시각화
            draw_params_right = {
                'matchColor': (255, 0, 0),
                'singlePointColor': None,
                'matchesMask': matches_right,
                'flags': 2
            }
            
            final_right = cv2.drawMatches(
                cv_image_input,
                kp1,
                self.img_right,
                self.kp_right,
                good_right,
                None,
                **draw_params_right
            )

            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(
                        final_right, 'jpg'
                    )
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(
                        final_right, 'bgr8'
                    )
                )

def main(args=None):
    """
    🚀 프로그램 메인 진입점
    
    ROS2 시스템을 초기화하고 표지판 감지 노드를 실행합니다.
    Ctrl+C로 종료할 때까지 계속 실행되며, 안전한 종료 처리를 보장합니다.
    """
    rclpy.init(args=args)       # ROS2 시스템 초기화
    node = DetectSign()         # 표지판 감지 노드 인스턴스 생성
    rclpy.spin(node)            # 노드 실행 (이벤트 루프 시작)
    node.destroy_node()         # 노드 정리 및 리소스 해제
    rclpy.shutdown()            # ROS2 시스템 종료


if __name__ == '__main__':
    main()  # 프로그램이 직접 실행될 때만 main() 함수 호출