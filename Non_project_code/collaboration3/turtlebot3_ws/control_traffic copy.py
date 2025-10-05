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
# Author: Your Name
# Purpose: Traffic light control module for autonomous driving

import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import UInt8


class ControlTraffic(Node):

    def __init__(self):
        super().__init__('control_traffic')

        # 신호등 상태 구독자
        self.sub_traffic_light = self.create_subscription(
            UInt8,
            '/detect/traffic_light',
            self.callback_traffic_light,
            1
        )

        # 신뢰도 정보 구독자들
        self.sub_red_reliability = self.create_subscription(
            UInt8,
            '/detect/red_light_reliability',
            self.callback_red_reliability,
            1
        )
        self.sub_yellow_reliability = self.create_subscription(
            UInt8,
            '/detect/yellow_light_reliability',
            self.callback_yellow_reliability,
            1
        )
        self.sub_green_reliability = self.create_subscription(
            UInt8,
            '/detect/green_light_reliability',
            self.callback_green_reliability,
            1
        )

        # 기존 cmd_vel 구독 (다른 제어기에서 오는 명령)
        self.sub_cmd_vel_input = self.create_subscription(
            Twist,
            '/control/cmd_vel',
            self.callback_cmd_vel_input,
            1
        )

        # 최종 cmd_vel 발행자 (신호등 제어가 적용된)
        self.pub_cmd_vel_final = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # 신호등 상태 발행자 (다른 노드에서 참조용)
        self.pub_traffic_override = self.create_publisher(
            Bool,
            '/traffic_light_override',
            1
        )

        # 신호등 상태 정의
        self.TRAFFIC_LIGHT_NONE = 0
        self.TRAFFIC_LIGHT_RED = 1
        self.TRAFFIC_LIGHT_YELLOW = 2
        self.TRAFFIC_LIGHT_GREEN = 3
        
        # 현재 상태 변수들
        self.current_traffic_state = self.TRAFFIC_LIGHT_NONE
        self.traffic_override_active = False
        
        # 신뢰도 관련 변수들
        self.red_reliability = 100
        self.yellow_reliability = 100
        self.green_reliability = 100
        self.min_reliability_threshold = 70  # 최소 신뢰도 임계값
        self.reliability_override_count = 0
        
        # 상태 지속성을 위한 변수들
        self.none_detection_start = None
        self.NONE_TIMEOUT = 2.0
        self.RED_PERSISTENCE_TIME = 3.0
        
        # 로깅 및 안전 관련
        self.last_log_time = 0.0
        self.log_interval = 2.0  # 1.0 → 2.0 (로그 출력 빈도 감소)
        self.last_cmd_vel_time = time.time()
        self.cmd_vel_timeout = 1.0
        
        # 기본 제어 관련 변수들 추가
        self.MAX_VEL = 0.1
        self.basic_control_active = True
        
        # 주기적 안전 체크
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # 기본 제어 타이머 추가
        self.basic_control_timer = self.create_timer(0.1, self.basic_traffic_control)

        self.get_logger().info('Traffic Light Control Module with Reliability Check Initialized')

    def callback_red_reliability(self, msg):
        """빨간불 신뢰도 업데이트"""
        self.red_reliability = msg.data

    def callback_yellow_reliability(self, msg):
        """노란불 신뢰도 업데이트"""
        self.yellow_reliability = msg.data

    def callback_green_reliability(self, msg):
        """초록불 신뢰도 업데이트"""
        self.green_reliability = msg.data

    def callback_traffic_light(self, traffic_light_msg):
        """신뢰도를 고려한 신호등 상태 업데이트"""
        raw_state = traffic_light_msg.data
        
        # 신뢰도 체크
        is_reliable = self.check_state_reliability(raw_state)
        
        if is_reliable:
            # 신뢰도가 충분하면 정상 처리
            processed_state = self.process_traffic_state_with_persistence(raw_state)
            
            if processed_state != self.current_traffic_state:
                self.current_traffic_state = processed_state
                self.update_traffic_override()
                self.log_traffic_state_change()
                self.reliability_override_count = 0
        else:
            # 신뢰도가 낮으면 이전 상태 유지
            self.reliability_override_count += 1
            
            # 주기적으로 경고 로그 (빈도 감소)
            if self.reliability_override_count % 20 == 1:  # 10 → 20 (덜 자주 출력)
                self.get_logger().warn(f'Low reliability: {self.get_state_name(raw_state)} - maintaining current state')

    def process_traffic_state_with_persistence(self, raw_state):
        """상태 지속성을 적용한 신호등 상태 처리"""
        current_time = time.time()
        
        # 확실한 신호 감지
        if raw_state in [self.TRAFFIC_LIGHT_RED, self.TRAFFIC_LIGHT_YELLOW, self.TRAFFIC_LIGHT_GREEN]:
            self.none_detection_start = None
            return raw_state
        
        # NONE 상태 처리 - 안전을 위한 지속성 적용
        else:
            current_state = self.current_traffic_state
            
            # NONE 감지 시작 시간 기록
            if self.none_detection_start is None:
                self.none_detection_start = current_time
            
            time_since_none = current_time - self.none_detection_start
            
            # 빨간불에서 NONE이 된 경우 - 특히 주의
            if current_state == self.TRAFFIC_LIGHT_RED:
                if time_since_none < self.RED_PERSISTENCE_TIME:
                    return self.TRAFFIC_LIGHT_RED
                else:
                    return self.TRAFFIC_LIGHT_NONE
            
            # 다른 상태에서 NONE이 된 경우
            elif current_state in [self.TRAFFIC_LIGHT_YELLOW, self.TRAFFIC_LIGHT_GREEN]:
                if time_since_none < self.NONE_TIMEOUT:
                    return current_state
                else:
                    return self.TRAFFIC_LIGHT_NONE
            
            # 이미 NONE 상태인 경우
            else:
                return self.TRAFFIC_LIGHT_NONE

    def check_state_reliability(self, state):
        """현재 상태의 신뢰도 체크"""
        current_reliability = self.get_current_state_reliability(state)
        return current_reliability >= self.min_reliability_threshold

    def get_current_state_reliability(self, state):
        """현재 상태의 신뢰도 값 반환"""
        if state == self.TRAFFIC_LIGHT_RED:
            return self.red_reliability
        elif state == self.TRAFFIC_LIGHT_YELLOW:
            return self.yellow_reliability
        elif state == self.TRAFFIC_LIGHT_GREEN:
            return self.green_reliability
        else:  # NONE
            return 100

    def get_state_name(self, state):
        """상태 코드를 이름으로 변환"""
        state_names = {
            self.TRAFFIC_LIGHT_NONE: 'NONE',
            self.TRAFFIC_LIGHT_RED: 'RED',
            self.TRAFFIC_LIGHT_YELLOW: 'YELLOW',
            self.TRAFFIC_LIGHT_GREEN: 'GREEN'
        }
        return state_names.get(state, 'UNKNOWN')

    def update_traffic_override(self):
        """신호등 제어 오버라이드 상태 업데이트"""
        # 빨간불/노란불일 때 오버라이드 활성화
        if (self.current_traffic_state == self.TRAFFIC_LIGHT_RED or 
            self.current_traffic_state == self.TRAFFIC_LIGHT_YELLOW):
            self.traffic_override_active = True
        else:
            self.traffic_override_active = False
        
        # 오버라이드 상태 발행
        override_msg = Bool()
        override_msg.data = self.traffic_override_active
        self.pub_traffic_override.publish(override_msg)

    def callback_cmd_vel_input(self, cmd_vel_msg):
        """다른 제어기에서 오는 cmd_vel 처리"""
        self.last_cmd_vel_time = time.time()
        
        # cmd_vel 입력이 있으면 기본 제어 비활성화
        self.basic_control_active = False
        
        # 신호등 상태에 따라 최종 cmd_vel 결정
        final_cmd_vel = self.apply_traffic_control(cmd_vel_msg)
        self.pub_cmd_vel_final.publish(final_cmd_vel)

    def apply_traffic_control(self, input_cmd_vel):
        """신호등 상태에 따른 제어 적용"""
        final_cmd_vel = Twist()
        
        if (self.current_traffic_state == self.TRAFFIC_LIGHT_RED or 
            self.current_traffic_state == self.TRAFFIC_LIGHT_YELLOW):
            # 빨간불/노란불: 완전 정지
            final_cmd_vel.linear.x = 0.0
            final_cmd_vel.angular.z = 0.0
            
        elif self.current_traffic_state == self.TRAFFIC_LIGHT_GREEN:
            # 초록불: 입력 cmd_vel 그대로 통과
            final_cmd_vel = input_cmd_vel
            
        else:  # TRAFFIC_LIGHT_NONE
            # 신호등 없음: 입력 cmd_vel 그대로 통과
            final_cmd_vel = input_cmd_vel
        
        return final_cmd_vel

    def basic_traffic_control(self):
        """기본 신호등 제어 - cmd_vel 입력이 없을 때 사용"""
        current_time = time.time()
        time_since_last_cmd = current_time - self.last_cmd_vel_time
        
        # cmd_vel이 일정 시간 없으면 기본 제어 활성화
        if time_since_last_cmd > 0.5:  # 0.5초 이상 cmd_vel이 없으면
            self.basic_control_active = True
        
        # 기본 제어가 활성화된 경우에만 실행
        if not self.basic_control_active:
            return

        twist = Twist()
        
        # 신호등 상태에 따른 기본 제어
        if (self.current_traffic_state == self.TRAFFIC_LIGHT_RED or 
            self.current_traffic_state == self.TRAFFIC_LIGHT_YELLOW):
            # 빨간불/노란불: 완전 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.current_traffic_state == self.TRAFFIC_LIGHT_GREEN:
            # 초록불: 천천히 직진
            twist.linear.x = self.MAX_VEL * 0.5  # 절반 속도로 안전하게
            twist.angular.z = 0.0  # 직진
            
        else:  # TRAFFIC_LIGHT_NONE
            # 신호등 없음: 매우 천천히 직진 (안전)
            twist.linear.x = self.MAX_VEL * 0.3  # 더 느린 속도
            twist.angular.z = 0.0  # 직진
        
        # 기본 제어 cmd_vel 발행
        self.pub_cmd_vel_final.publish(twist)

    def safety_check(self):
        """주기적 안전 체크"""
        current_time = time.time()
        
        # cmd_vel 타임아웃 체크 (기본 제어가 활성화된 경우 제외)
        if current_time - self.last_cmd_vel_time > self.cmd_vel_timeout and not self.basic_control_active:
            # 타임아웃 시 안전 정지
            safety_stop = Twist()
            self.pub_cmd_vel_final.publish(safety_stop)
            
            # 로그 출력 (제한적)
            if self.should_log():
                self.get_logger().warn('cmd_vel timeout - Safety stop activated')

    def log_traffic_state_change(self):
        """신호등 상태 변경 로그 (핵심 정보만)"""
        state_name = self.get_state_name(self.current_traffic_state)
        override_status = "STOP" if self.traffic_override_active else "GO"
        
        self.get_logger().info(f'🚦 Traffic Light: {state_name} - {override_status}')

    def should_log(self):
        """로그 출력 제한"""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            return True
        return False

    def shut_down(self):
        """노드 종료 시 안전 정지"""
        self.get_logger().info('Traffic Control shutting down')
        stop_twist = Twist()
        self.pub_cmd_vel_final.publish(stop_twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlTraffic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()