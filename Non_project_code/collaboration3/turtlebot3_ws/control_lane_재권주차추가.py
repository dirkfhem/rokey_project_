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
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64

################## 재권 추가(주차 사인)##########################
from std_msgs.msg import UInt8
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion  # quaternion → roll, pitch, yaw 변환
from nav_msgs.msg import Odometry
# pip install transforms3d 해야함
import time
#################재권 추가 끝 ###########################

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        # 기존 차선 추종 관련 구독
        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        #####재권 추가 시작#############
        # 주차 사인 토픽 구독
        self.sub_sign = self.create_subscription(
            UInt8,
            '/detect/traffic_sign/parking',
            self.callback_detect_sign,
            10
        )

        self.sign_detected = False  # 중복 방지용

        #imu값 받아오기
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.callback_imu,
            10
        )

        self.current_yaw = 0.0
        self.yaw_triggered = False  # 중복 방지

        self.current_position = (0.0, 0.0)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_odom,
            10
        )
        
        self.parking_mode = False         # 파킹 시작 여부
        self.parking_completed = False    # 파킹 완료 여부
        self.target_position = (0.45, 1.75)
        self.tolerance_dist = 0.05
        self.target_yaw = 3.14
        self.tolerance_yaw = 0.1
        self.last_log_time = time.time()
        ########재권 추가 끝#########################

        # PD control related variables(기존 pd 제어 관련 변수)
        self.last_error = 0
        self.MAX_VEL = 0.1

        # Avoidance mode related variables(기존 회피 관련 변수)
        self.avoid_active = False
        self.avoid_twist = Twist()

        

    # 기존 최대 속도 설정 함수
    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.

        If avoidance mode is enabled, lane following control is ignored.
        """
        if self.avoid_active:
            return

        center = desired_center.data
        error = center - 500

        # Kp = 0.0025
        # Kd = 0.007

        Kp = 0.01
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()
        # Linear velocity: adjust speed based on error (maximum 0.05 limit)
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    # 기존 회피 제어 로직
    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    # 기존 회피 제어 로직
    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    #####재권 추가 시작(주차 사인 감지됐을 때 주차 동작)########################3
    def callback_odom(self, msg):
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)
    def sol_parking(self):
        if self.parking_mode and not self.parking_completed:
            tx, ty = self.target_position
            dist = ((self.current_position[0] - tx)**2 + (self.current_position[1] - ty)**2)**0.5
            current_time = time.time()
            if current_time - self.last_log_time >= 2.0:
                self.get_logger().info(f"[주차 접근중] 위치: x={self.current_position[0]:.2f}, y={self.current_position[1]:.2f}, 거리={dist:.3f}")
                self.last_log_time = current_time

            if dist > self.tolerance_dist:
                # 아직 목표 위치 도달 전 → 직진 명령
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
            else:
                # 목표 위치 도달 → yaw 확인
                yaw = self.current_yaw
                if abs(abs(yaw) - self.target_yaw) < self.tolerance_yaw:

                    park1_twist = Twist()
                    park1_twist.linear.x = 0.0
                    park1_twist.angular.z = 0.0
                    self.pub_cmd_vel.publish(park1_twist)
                    time.sleep(3.0)

                    self.get_logger().info(f"[주차] 목표 위치 및 방향 만족 → 회전 시작")
                    park2_twist = Twist()
                    park2_twist.linear.x = -0.02
                    park2_twist.angular.z = -0.93
                    self.pub_cmd_vel.publish(park2_twist)
                    time.sleep(2.0)

                    self.get_logger().info('주차... 후진 중')
                    park3_twist = Twist()
                    park3_twist.linear.x = -0.1
                    park3_twist.angular.z = 0.0
                    self.pub_cmd_vel.publish(park3_twist)
                    time.sleep(9.0)

                    stop_twist = Twist()
                    stop_twist.linear.x = 0.0
                    stop_twist.angular.z = 0.0
                    self.pub_cmd_vel.publish(stop_twist)  # 정지
                    time.sleep(3.0)
                    self.get_logger().info("주차 완료")

                    start_twist = Twist()
                    start_twist.linear.x = 0.1
                    start_twist.angular.z = 0.0
                    self.pub_cmd_vel.publish(start_twist)  # 앞으로 전진
                    time.sleep(7.0)
                    self.get_logger().info("주행 시작")

                    start2_twist = Twist()
                    start2_twist.linear.x = 0.1
                    start2_twist.angular.z = 0.5
                    self.pub_cmd_vel.publish(start2_twist)  # 앞으로 전진
                    time.sleep(3.0)
                    self.get_logger().info("좌회전 완료")

                    self.parking_completed = True
                    self.parking_mode = False
                else:
                    self.get_logger().info(f"[대기] yaw {round(yaw, 2)} → 3.14 도달 대기중")

    def callback_imu(self, msg):
        # quaternion → euler 변환
        q = msg.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        self.current_yaw = yaw  # rad 단위 (-π ~ π)

        # 로그
        # self.get_logger().info(f'현재 yaw(rad): {round(yaw, 2)}')

    def callback_detect_sign(self, msg):
        if msg.data == 1 and not self.parking_mode:
            self.get_logger().info("Parking sign detected. 주차 모드 진입")
            self.parking_mode = True
            self.parking_completed = False
            self.sign_detected = True
    ##############재권 추가 끝###########################

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
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
